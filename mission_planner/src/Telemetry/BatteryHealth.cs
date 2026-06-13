// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Battery Health - vehicle-parameter-driven thresholds
// ============================================================
// Single source of truth for battery state evaluation. Thresholds
// come from the vehicle's own ArduPilot configuration:
//   BATTn_LOW_VOLT / BATTn_CRT_VOLT  - voltage warn / critical
//   BATTn_ARM_VOLT                   - minimum arming voltage
//   BATTn_CAPACITY + BATTn_LOW_MAH / BATTn_CRT_MAH - capacity-based
// Nothing is hardcoded except a per-cell fallback used only when the
// vehicle has no voltage thresholds configured at all. The MP-reported
// battery_remaining percentage is intentionally NOT used.
// ============================================================

using System;
using System.Collections.Generic;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    /// <summary>Snapshot of one battery's readings, thresholds, and verdict.</summary>
    public class BatteryStatus
    {
        public int Index;
        public bool Configured;          // BATTn_MONITOR > 0
        public double Voltage;
        public double UsedMah;
        public double CapacityMah;       // BATTn_CAPACITY (0 = not set)
        public double RemainingMah;      // CapacityMah - UsedMah (0 when capacity unset)
        public double LowVoltage;        // resolved warn threshold (0 = none)
        public double CriticalVoltage;   // resolved critical threshold (0 = none)
        public double ArmVoltage;        // BATTn_ARM_VOLT (0 = disabled)
        public double LowMah;            // BATTn_LOW_MAH (0 = disabled)
        public double CriticalMah;       // BATTn_CRT_MAH (0 = disabled)
        public bool ThresholdsFromVehicle; // false = per-cell fallback in use
        public int Severity;             // 0 = ok, 1 = warning, 2 = critical
        public string Reason = "";       // which threshold tripped
        public bool BelowArmVoltage;     // disarmed and under BATTn_ARM_VOLT
    }

    public static class BatteryHealth
    {
        // Used ONLY when the vehicle has neither BATTn_LOW/CRT_VOLT nor a cell
        // count we can apply them to. 3.5/3.3 V per cell are conservative LiPo
        // under-load floors.
        private const double FALLBACK_LOW_PER_CELL = 3.5;
        private const double FALLBACK_CRT_PER_CELL = 3.3;

        /// <summary>
        /// Read battery n (1-based) from the connected vehicle and evaluate it
        /// against its own configured thresholds. Returns null when there is no
        /// MAV or the battery monitor is disabled.
        /// </summary>
        public static BatteryStatus Read(int idx)
        {
            var mav = MainV2.comPort?.MAV;
            if (mav?.cs == null) return null;

            if (GetBattParam(mav, idx, "MONITOR") <= 0) return null;

            var s = new BatteryStatus { Index = idx, Configured = true };

            // Live readings. MP exposes per-battery fields as battery_voltage /
            // battery_voltage2 / ..., same pattern for usedmah.
            s.Voltage = GetCsDouble(mav.cs, idx == 1 ? "battery_voltage" : $"battery_voltage{idx}");
            s.UsedMah = GetCsDouble(mav.cs, idx == 1 ? "battery_usedmah" : $"battery_usedmah{idx}");

            // Voltage thresholds straight from the vehicle (0 = disabled there).
            s.LowVoltage = GetBattParam(mav, idx, "LOW_VOLT");
            s.CriticalVoltage = GetBattParam(mav, idx, "CRT_VOLT");
            s.ArmVoltage = GetBattParam(mav, idx, "ARM_VOLT");
            s.ThresholdsFromVehicle = s.LowVoltage > 0 || s.CriticalVoltage > 0;
            if (!s.ThresholdsFromVehicle)
            {
                double cells = GetBattParam(mav, idx, "CELL_COUNT");
                if (cells <= 0 && s.Voltage > 0)
                {
                    cells = Math.Max(1, Math.Round(s.Voltage / 3.7)); // nominal LiPo cell
                }
                if (cells > 0)
                {
                    s.LowVoltage = cells * FALLBACK_LOW_PER_CELL;
                    s.CriticalVoltage = cells * FALLBACK_CRT_PER_CELL;
                }
            }

            // Capacity thresholds (remaining mAh vs BATTn_LOW/CRT_MAH).
            s.CapacityMah = GetBattParam(mav, idx, "CAPACITY");
            s.LowMah = GetBattParam(mav, idx, "LOW_MAH");
            s.CriticalMah = GetBattParam(mav, idx, "CRT_MAH");
            if (s.CapacityMah > 0 && s.UsedMah >= 0)
            {
                s.RemainingMah = Math.Max(0, s.CapacityMah - s.UsedMah);
            }

            Evaluate(s, armed: mav.cs.armed);
            return s;
        }

        private static void Evaluate(BatteryStatus s, bool armed)
        {
            bool haveMah = s.CapacityMah > 0 && s.UsedMah > 0;

            if (s.CriticalVoltage > 0 && s.Voltage > 0 && s.Voltage <= s.CriticalVoltage)
            {
                s.Severity = 2;
                s.Reason = $"{s.Voltage:F1}V <= critical voltage {s.CriticalVoltage:F1}V";
            }
            else if (haveMah && s.CriticalMah > 0 && s.RemainingMah <= s.CriticalMah)
            {
                s.Severity = 2;
                s.Reason = $"{s.RemainingMah:F0} mAh left <= critical {s.CriticalMah:F0} mAh";
            }
            else if (s.LowVoltage > 0 && s.Voltage > 0 && s.Voltage <= s.LowVoltage)
            {
                s.Severity = 1;
                s.Reason = $"{s.Voltage:F1}V <= low voltage {s.LowVoltage:F1}V";
            }
            else if (haveMah && s.LowMah > 0 && s.RemainingMah <= s.LowMah)
            {
                s.Severity = 1;
                s.Reason = $"{s.RemainingMah:F0} mAh left <= low {s.LowMah:F0} mAh";
            }

            // Pre-arm check: only meaningful while disarmed (ArduPilot blocks
            // arming below BATTn_ARM_VOLT; surface it before the pilot tries).
            if (!armed && s.ArmVoltage > 0 && s.Voltage > 0 && s.Voltage < s.ArmVoltage)
            {
                s.BelowArmVoltage = true;
                if (s.Severity < 1)
                {
                    s.Severity = 1;
                    s.Reason = $"{s.Voltage:F1}V < arm voltage {s.ArmVoltage:F1}V";
                }
            }
        }

        // BATTn_* params change rarely but Read() runs on every dashboard tick
        // and notification poll; cache reflection lookups for a few seconds.
        private static readonly object _paramCacheLock = new object();
        private static readonly Dictionary<string, (double Value, DateTime ReadUtc)> _paramCache
            = new Dictionary<string, (double, DateTime)>();
        private static readonly TimeSpan ParamCacheTtl = TimeSpan.FromSeconds(5);

        /// <summary>
        /// Read BATTn_XXX (or BATT_XXX when n==1) from MAVLink params, cached
        /// for a few seconds. Returns 0 if missing.
        /// </summary>
        public static double GetBattParam(dynamic mav, int idx, string suffix)
        {
            string cacheKey = (idx == 1 ? "BATT_" : $"BATT{idx}_") + suffix;
            lock (_paramCacheLock)
            {
                if (_paramCache.TryGetValue(cacheKey, out var hit)
                    && DateTime.UtcNow - hit.ReadUtc < ParamCacheTtl)
                {
                    return hit.Value;
                }
            }

            double value = ReadBattParamUncached(mav, idx, suffix);
            lock (_paramCacheLock)
            {
                _paramCache[cacheKey] = (value, DateTime.UtcNow);
            }
            return value;
        }

        private static double ReadBattParamUncached(dynamic mav, int idx, string suffix)
        {
            try
            {
                string name = (idx == 1 ? "BATT_" : $"BATT{idx}_") + suffix;
                var param = mav.param;
                if (param == null) return 0;
                // MP's MAVLinkParamList exposes ContainsKey / indexer with MAVLinkParam values.
                var containsKey = param.GetType().GetMethod("ContainsKey", new[] { typeof(string) });
                if (containsKey != null)
                {
                    bool has = (bool)containsKey.Invoke(param, new object[] { name });
                    if (!has) return 0;
                }
                var indexer = param.GetType().GetProperty("Item", new[] { typeof(string) });
                var entry = indexer?.GetValue(param, new object[] { name });
                if (entry == null) return 0;
                var valueProp = entry.GetType().GetProperty("Value");
                if (valueProp != null)
                {
                    var raw = valueProp.GetValue(entry);
                    return Convert.ToDouble(raw);
                }
                return Convert.ToDouble(entry);
            }
            catch
            {
                return 0;
            }
        }

        /// <summary>
        /// Read a numeric field/property by name from CurrentState. Returns 0 if missing.
        /// </summary>
        public static double GetCsDouble(object cs, string name)
        {
            try
            {
                var t = cs.GetType();
                var prop = t.GetProperty(name);
                if (prop != null) return Convert.ToDouble(prop.GetValue(cs));
                var field = t.GetField(name);
                if (field != null) return Convert.ToDouble(field.GetValue(cs));
            }
            catch { }
            return 0;
        }
    }
}
