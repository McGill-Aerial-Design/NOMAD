// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Flight Mode Controller
// ============================================================
// Thin reflection wrapper around MissionPlanner's MAVLinkInterface for
// the few mode/param operations the plugin needs to issue directly
// (without going through the Jetson API). Reflection so the plugin
// survives MP API drift across versions.
// ============================================================

using System;
using System.Linq;
using System.Reflection;
using MissionPlanner;
using NOMAD.MissionPlanner.Connectivity;

namespace NOMAD.MissionPlanner
{
    public static class FlightModeController
    {
        private static NOMADConfig _config;

        /// <summary>
        /// Called at plugin load so GuidedGoto can build the core client.
        /// </summary>
        public static void Initialize(NOMADConfig config)
        {
            _config = config;
        }
        /// <summary>
        /// Emergency-land the vehicle. Forces LAND_SPEED and WPNAV_SPEED_DN to
        /// at least <paramref name="minDescentCmS"/> (≥200 cm/s satisfies the
        /// CONOPS §4.5 termination requirement), then commands LAND mode.
        /// Returns true if the mode command was dispatched.
        /// </summary>
        public static bool EmergencyLand(int minDescentCmS = 250)
        {
            var comPort = MainV2.comPort;
            if (comPort == null)
            {
                Log.Warn("EmergencyLand: not connected.");
                return false;
            }
            try
            {
                // Push descent-speed params first so when LAND engages it uses
                // the fast rate. ArduCopter consults LAND_SPEED for the final
                // approach below LAND_ALT_LOW, and WPNAV_SPEED_DN for the
                // descent above it — both must be raised to hit ≥2 m/s the
                // whole way down.
                if (minDescentCmS > 0)
                {
                    TrySetParam(comPort, "LAND_SPEED", minDescentCmS);
                    TrySetParam(comPort, "WPNAV_SPEED_DN", minDescentCmS);
                }
                return TrySetMode(comPort, "LAND");
            }
            catch (Exception ex)
            {
                Log.Error($"EmergencyLand: {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Switch to GUIDED and fly to the given position at the given relative
        /// altitude (meters AGL). Used by the soft-boundary "return to boundary"
        /// action. Routes through the C++ core client boundary: the core sends
        /// MAV_CMD_DO_REPOSITION with the change-mode flag and verifies the
        /// arrival position, so a true return means the vehicle is at the
        /// target. Returns false (fail closed) when the core is not configured,
        /// refuses, or cannot reach the vehicle.
        /// </summary>
        public static bool GuidedGoto(double lat, double lng, double altRelM)
        {
            var client = CreateCoreClient();
            if (client == null)
            {
                Log.Warn("GuidedGoto: NOMAD core not configured.");
                return false;
            }
            var ok = client.Goto(lat, lng, altRelM);
            if (!ok)
            {
                Log.Warn("GuidedGoto: core rejected or could not reach the vehicle.");
            }
            return ok;
        }

        private static NomadCoreClient CreateCoreClient()
        {
            if (_config == null)
            {
                return null;
            }
            return new NomadCoreClient(_config.CoreExePath, _config.CoreMavlinkEndpoint, _config.CoreApiKey);
        }

        private static bool TrySetMode(object comPort, string modeName)
        {
            try
            {
                var methods = comPort.GetType()
                    .GetMethods(BindingFlags.Public | BindingFlags.Instance)
                    .Where(m => m.Name == "setMode")
                    .ToList();

                // Prefer the (string) / (sysid, compid, string) overloads —
                // both are stable across recent MP versions and don't require
                // building a mavlink_set_mode_t struct.
                foreach (var m in methods)
                {
                    var p = m.GetParameters();
                    if (p.Length == 1 && p[0].ParameterType == typeof(string))
                    {
                        m.Invoke(comPort, new object[] { modeName });
                        return true;
                    }
                }
                foreach (var m in methods)
                {
                    var p = m.GetParameters();
                    if (p.Length == 3
                        && (p[0].ParameterType == typeof(byte) || p[0].ParameterType == typeof(int))
                        && (p[1].ParameterType == typeof(byte) || p[1].ParameterType == typeof(int))
                        &&  p[2].ParameterType == typeof(string))
                    {
                        byte sysid  = TryGetByteProp(comPort, "sysidcurrent",  1);
                        byte compid = TryGetByteProp(comPort, "compidcurrent", 1);
                        m.Invoke(comPort, new object[]
                        {
                            Convert.ChangeType(sysid,  p[0].ParameterType),
                            Convert.ChangeType(compid, p[1].ParameterType),
                            modeName,
                        });
                        return true;
                    }
                }
                Log.Debug("FlightModeController: no compatible setMode overload found.");
                return false;
            }
            catch (TargetInvocationException tie)
            {
                Log.Error($"setMode({modeName}) threw: {tie.InnerException?.Message ?? tie.Message}");
                return false;
            }
            catch (Exception ex)
            {
                Log.Error($"setMode({modeName}) error: {ex.Message}");
                return false;
            }
        }

        private static byte TryGetByteProp(object obj, string name, byte fallback)
        {
            try
            {
                var p = obj.GetType().GetProperty(name, BindingFlags.Public | BindingFlags.Instance);
                if (p == null) return fallback;
                return Convert.ToByte(p.GetValue(obj));
            }
            catch { return fallback; }
        }

        private static bool TrySetParam(object comPort, string name, double value)
        {
            try
            {
                var methods = comPort.GetType().GetMethods(BindingFlags.Public | BindingFlags.Instance)
                    .Where(m => m.Name == "setParam").ToList();
                foreach (var m in methods)
                {
                    var p = m.GetParameters();
                    if (p.Length >= 2 && p[0].ParameterType == typeof(string))
                    {
                        var args = new object[p.Length];
                        args[0] = name;
                        args[1] = Convert.ChangeType(value, p[1].ParameterType);
                        for (int i = 2; i < p.Length; i++)
                        {
                            if (p[i].HasDefaultValue) args[i] = p[i].DefaultValue;
                            else if (p[i].ParameterType == typeof(bool)) args[i] = true;
                            else args[i] = p[i].ParameterType.IsValueType
                                ? Activator.CreateInstance(p[i].ParameterType) : null;
                        }
                        try { m.Invoke(comPort, args); return true; }
                        catch { /* try next overload */ }
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Error($"setParam({name}) error: {ex.Message}");
            }
            return false;
        }
    }
}
