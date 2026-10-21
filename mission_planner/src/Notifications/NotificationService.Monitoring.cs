// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public partial class NotificationService
    {
        private void MonitorTimer_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            if (System.Threading.Interlocked.CompareExchange(ref _pollGuard, 1, 0) != 0)
                return;

            try
            {
                var connection = MainV2.comPort;
                var mav = connection?.MAV;
                var state = mav?.cs;
                if (state?.connected != true)
                {
                    ObserveHealthContext(null, null, null);
                    return;
                }
                ObserveHealthContext(connection, mav, state);
                CheckAltitudeCallouts(state);
                CheckGPSHealth(state);
                CheckBatteryHealth(mav);
                CheckEKFPositionValidity(state.ekfflags);
                CheckOpticalFlowHealth(state);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NotificationService error: {ex.Message}");
            }
            finally
            {
                System.Threading.Interlocked.Exchange(ref _pollGuard, 0);
            }

        }

        private void ObserveHealthContext(MAVLinkInterface connection, MAVState mav, CurrentState state)
        {
            if (ReferenceEquals(connection, _observedConnection) && ReferenceEquals(mav, _observedMav) &&
                ReferenceEquals(state, _observedState)) { return; }
            _observedConnection = connection;
            _observedMav = mav;
            _observedState = state;
            _lastGpsFix = -1;
            _lastEkfPositionValidity = -1;
            _altBand = -1;
            _lastBatterySeverity.Clear();
            _lastBatterySpeechUtc.Clear();
            lock (_lock) { _lastNotificationTime.Clear(); }
        }

        private void CheckGPSHealth(CurrentState cs)
        {

            int satCount = (int)cs.satcount;
            int gpsFix = (int)cs.gpsstatus;
            double hdop = cs.gpshdop;

            if (satCount < GPS_MIN_SATS_CRITICAL && satCount > 0)
            {
                AddNotification(NotificationSeverity.Critical, NotificationCategory.GPS,
                    "GPS Critical", $"Only {satCount} satellites visible - position unreliable");
            }
            else if (satCount < GPS_MIN_SATS_WARNING && satCount > 0)
            {
                AddNotification(NotificationSeverity.Warning, NotificationCategory.GPS,
                    "GPS Low Sats", $"{satCount} satellites - consider better position");
            }

            if (_lastGpsFix != -1 && gpsFix != _lastGpsFix)
            {
                string fixName = GetGpsFixName(gpsFix);
                string lastFixName = GetGpsFixName(_lastGpsFix);
                if (gpsFix < _lastGpsFix)
                {
                    AddNotification(NotificationSeverity.Warning, NotificationCategory.GPS,
                        "GPS Fix Degraded", $"GPS changed from {lastFixName} to {fixName}");
                }
                else if (gpsFix > _lastGpsFix && gpsFix >= 3)
                {
                    AddNotification(NotificationSeverity.Info, NotificationCategory.GPS,
                        "GPS Fix Improved", $"GPS now has {fixName}");
                }
            }
            _lastGpsFix = gpsFix;

            if (hdop > GPS_HDOP_CRITICAL && hdop < 99)
            {
                AddNotification(NotificationSeverity.Critical, NotificationCategory.GPS,
                    "GPS HDOP Critical", $"HDOP {hdop:F1} - position accuracy degraded");
            }
            else if (hdop > GPS_HDOP_WARNING && hdop < 99)
            {
                AddNotification(NotificationSeverity.Warning, NotificationCategory.GPS,
                    "GPS HDOP High", $"HDOP {hdop:F1} - reduced accuracy");
            }
        }

        private void CheckBatteryHealth(MAVState mav)
        {
            for (int idx = 1; idx <= 2; idx++)
                CheckOneBattery(mav, idx);
        }

        private void CheckOneBattery(MAVState mav, int idx)
        {
            var state = BatteryHealth.Read(mav, idx);
            if (state == null) return;

            string label = $"BATT{idx}";
            string detail = $"{label}: {state.Voltage:F1}V";
            if (state.CapacityMah > 0)
                detail += $" · {state.RemainingMah:F0}/{state.CapacityMah:F0} mAh";

            if (state.Severity == 2)
            {
                AddNotification(NotificationSeverity.Critical, NotificationCategory.Battery,
                    $"{label} Critical", $"{detail} — {state.Reason} - LAND NOW");
            }
            else if (state.Severity == 1)
            {
                AddNotification(NotificationSeverity.Warning, NotificationCategory.Battery,
                    state.BelowArmVoltage ? $"{label} Below Arm Voltage" : $"{label} Low",
                    $"{detail} — {state.Reason}");
            }

            int previousSeverity = _lastBatterySeverity.TryGetValue(idx, out var value) ? value : 0;
            if (state.Severity == 2)
            {
                AudioAlerts.Play(AlertKind.BatteryCritical);
                if (CanSpeakBattery(idx))
                {
                    AudioAlerts.Speak($"Battery {idx} critical, {state.Voltage:F1} volts. Land now.",
                        component: $"battery.{idx}", ignoreRateLimit: true);
                }
            }
            else if (state.Severity == 1 && previousSeverity < 1)
            {
                AudioAlerts.Play(AlertKind.BatteryWarning);
                if (CanSpeakBattery(idx))
                {
                    string phrase = state.BelowArmVoltage
                        ? $"Battery {idx} below arming voltage, {state.Voltage:F1} volts."
                        : $"Battery {idx} low, {state.Voltage:F1} volts.";
                    AudioAlerts.Speak(phrase, component: $"battery.{idx}", ignoreRateLimit: true);
                }
            }
            _lastBatterySeverity[idx] = state.Severity;
        }

        private bool CanSpeakBattery(int idx)
        {
            var now = DateTime.UtcNow;
            if (_lastBatterySpeechUtc.TryGetValue(idx, out var last)
                && now - last < BatterySpeechInterval)
            {
                return false;
            }
            _lastBatterySpeechUtc[idx] = now;
            return true;
        }

        private void CheckEKFPositionValidity(int flags)
        {
            int validity = flags & 0x18;
            if (_lastEkfPositionValidity != -1 && validity != _lastEkfPositionValidity)
            {
                string description = validity switch
                {
                    0x18 => "Absolute and relative horizontal position valid",
                    0x10 => "Absolute horizontal position valid",
                    0x08 => "Relative horizontal position valid",
                    _ => "No horizontal position validity reported"
                };
                AddNotification(validity == 0 ? NotificationSeverity.Critical : NotificationSeverity.Warning,
                    NotificationCategory.EKF,
                    validity == 0 ? "EKF Position Unavailable" : "EKF Position Validity Changed", description);
            }
            _lastEkfPositionValidity = validity;
        }

        private int _altBand = -1;

        private void CheckAltitudeCallouts(CurrentState cs)
        {
            if (!AudioAlerts.AltitudeCalloutsEnabled || !cs.armed)
            {
                _altBand = -1;
                return;
            }

            var altitude = AltitudeUnits.ToMeters(cs.alt, CurrentState.multiplieralt);
            if (!altitude.HasValue) { _altBand = -1; return; }
            string phrase = AltitudeCallout.Next(altitude.Value, ref _altBand);
            if (phrase != null)
                AudioAlerts.Speak(phrase, component: "altitude", ignoreRateLimit: true);
        }

        private void CheckOpticalFlowHealth(CurrentState cs)
        {

            try
            {
                var rangeMeters = AltitudeUnits.ToMeters(cs.sonarrange, CurrentState.multiplieralt);
                if (rangeMeters.HasValue && rangeMeters.Value >= 100)
                {
                    AddNotification(NotificationSeverity.Warning, NotificationCategory.OpticalFlow,
                        "Rangefinder Issue", $"Rangefinder reading abnormal: {rangeMeters.Value:F1}m");
                }
            }
            catch
            {
            }
        }
    }
}
