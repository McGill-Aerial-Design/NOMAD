// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Threading.Tasks;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public partial class NotificationService
    {
        private async void MonitorTimer_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            // Guard against reentrant execution if a previous poll is still running
            if (System.Threading.Interlocked.CompareExchange(ref _pollGuard, 1, 0) != 0)
                return;

            try
            {
                CheckGPSHealth();
                CheckBatteryHealth();
                CheckEKFSource();
                await CheckVIOHealthAsync().ConfigureAwait(false);
                CheckOpticalFlowHealth();
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

        private void CheckGPSHealth()
        {
            var cs = MainV2.comPort?.MAV?.cs;
            if (cs == null) return;

            int satCount = (int)cs.satcount;
            int gpsFix = (int)cs.gpsstatus;
            double hdop = cs.gpshdop;

            // Check satellite count
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

            // Check GPS fix type changes
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

            // Check HDOP
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

        private void CheckBatteryHealth()
        {
            var mav = MainV2.comPort?.MAV;
            if (mav?.cs == null) return;

            // Walk BATT1 and BATT2. ArduPilot supports up to 9, but we only
            // care about the two configured on this airframe.
            for (int idx = 1; idx <= 2; idx++)
            {
                CheckOneBattery(mav, idx);
            }
        }

        private void CheckOneBattery(dynamic mav, int idx)
        {
            // All thresholds come from the vehicle's own BATTn_* configuration
            // (voltage + capacity mAh); see BatteryHealth. No percentage checks.
            var s = BatteryHealth.Read(idx);
            if (s == null) return;  // no MAV or BATTn_MONITOR disabled

            string label = $"BATT{idx}";
            string detail = $"{label}: {s.Voltage:F1}V";
            if (s.CapacityMah > 0) detail += $" · {s.RemainingMah:F0}/{s.CapacityMah:F0} mAh";

            if (s.Severity == 2)
            {
                AddNotification(NotificationSeverity.Critical, NotificationCategory.Battery,
                    $"{label} Critical", $"{detail} — {s.Reason} - LAND NOW");
            }
            else if (s.Severity == 1)
            {
                AddNotification(NotificationSeverity.Warning, NotificationCategory.Battery,
                    s.BelowArmVoltage ? $"{label} Below Arm Voltage" : $"{label} Low",
                    $"{detail} — {s.Reason}");
            }

            // Audio alerts (per battery, transition-driven for warning, repeating for critical).
            int last = _lastBatterySeverity.TryGetValue(idx, out var v) ? v : 0;
            if (s.Severity == 2)
            {
                AudioAlerts.Play(AlertKind.BatteryCritical);
                if (CanSpeakBattery(idx))
                    AudioAlerts.Speak($"Battery {idx} critical, {s.Voltage:F1} volts. Land now.",
                        component: $"battery.{idx}", ignoreRateLimit: true);
            }
            else if (s.Severity == 1 && last < 1)
            {
                AudioAlerts.Play(AlertKind.BatteryWarning);
                if (CanSpeakBattery(idx))
                    AudioAlerts.Speak(s.BelowArmVoltage
                            ? $"Battery {idx} below arming voltage, {s.Voltage:F1} volts."
                            : $"Battery {idx} low, {s.Voltage:F1} volts.",
                        component: $"battery.{idx}", ignoreRateLimit: true);
            }
            _lastBatterySeverity[idx] = s.Severity;
        }

        private bool CanSpeakBattery(int idx)
        {
            var now = DateTime.UtcNow;
            if (_lastBatterySpeechUtc.TryGetValue(idx, out var last) && now - last < BatterySpeechInterval)
                return false;
            _lastBatterySpeechUtc[idx] = now;
            return true;
        }

        // Param/CurrentState reflection helpers live in BatteryHealth.

        private void CheckEKFSource()
        {
            var cs = MainV2.comPort?.MAV?.cs;
            if (cs == null) return;

            // Try to read EK3_SRC1_POSZ or similar parameter indicating EKF source
            // ArduPilot uses EK3_SRC parameters for position/velocity/yaw sources
            try
            {
                // Check ekf_status_report or similar
                // EKF source changes are typically indicated through MAVLink EKF_STATUS_REPORT
                // For now, we track changes in position estimate sources

                // Check if using GPS vs VIO based on what's active
                var ekfFlags = (int)cs.ekfstatus;

                // Bit 0: attitude ok, Bit 1: velocity horiz ok, Bit 2: velocity vert ok
                // Bit 3: pos horiz rel ok, Bit 4: pos horiz abs ok, Bit 5: pos vert abs ok
                // Bit 6: pos vert agl ok, Bit 7: const pos mode

                bool posRelOk = (ekfFlags & 0x08) != 0;
                bool posAbsOk = (ekfFlags & 0x10) != 0;

                // Derive a simple "source" indicator
                int currentSource = posAbsOk ? 1 : (posRelOk ? 2 : 0);  // 1=GPS, 2=Relative, 0=None

                if (_lastEkfSource != -1 && currentSource != _lastEkfSource)
                {
                    string sourceName = currentSource switch
                    {
                        1 => "GPS (Absolute)",
                        2 => "Relative (VIO/OptFlow)",
                        _ => "None/Degraded"
                    };

                    var severity = currentSource == 0 ? NotificationSeverity.Critical : NotificationSeverity.Warning;
                    AddNotification(severity, NotificationCategory.EKF,
                        "EKF Source Changed", $"Position source: {sourceName}");
                }
                _lastEkfSource = currentSource;
            }
            catch
            {
                // Ignore EKF check errors
            }
        }

        private async Task CheckVIOHealthAsync()
        {
            if (_sender == null) return;

            var health = _sender.LastHealthStatus;

            // Query actual VIO status from the API endpoint instead of
            // inferring it from Jetson connectivity alone.
            bool vioActive = false;
            try
            {
                var vioResult = await _sender.GetVioStatusAsync().ConfigureAwait(false);
                if (vioResult.Success && !string.IsNullOrEmpty(vioResult.Data))
                {
                    var vioData = Newtonsoft.Json.Linq.JObject.Parse(vioResult.Data);
                    var vioHealth = (string)vioData["health"] ?? "unknown";
                    vioActive = vioHealth == "healthy";
                }
            }
            catch
            {
                // If the VIO status endpoint is unreachable, treat as inactive
                vioActive = false;
            }

            // Detect VIO activation/deactivation
            if (vioActive != _lastVioActive)
            {
                if (vioActive)
                {
                    AddNotification(NotificationSeverity.Info, NotificationCategory.VIO,
                        "VIO Active", "Visual-Inertial Odometry is now running");
                }
                else
                {
                    AddNotification(NotificationSeverity.Warning, NotificationCategory.VIO,
                        "VIO Offline", "Visual-Inertial Odometry lost - check Jetson");
                }
            }
            _lastVioActive = vioActive;

            // If VIO is active but Jetson temp is high, warn
            if (vioActive && health != null)
            {
                if (health.GpuTemp > 85 || health.CpuTemp > 85)
                {
                    AddNotification(NotificationSeverity.Warning, NotificationCategory.VIO,
                        "Jetson Overheating", $"Temperature: {Math.Max(health.GpuTemp, health.CpuTemp):F0}C - VIO may throttle");
                }
            }
        }

        private void CheckOpticalFlowHealth()
        {
            var cs = MainV2.comPort?.MAV?.cs;
            if (cs == null) return;

            try
            {
                // ArduPilot reports optical flow quality in cs.opt_m_x/y or via OPTICAL_FLOW message
                // Check if optical flow sensor is present and quality
                var optFlowQuality = cs.opt_m_x;  // This might be flow quality depending on setup

                // Most setups use rangefinder with optical flow
                var rangeFinderDist = cs.sonarrange;
                var rangeFinderHealthy = rangeFinderDist > 0 && rangeFinderDist < 100;  // Valid range

                // Only warn if we appear to have optical flow configured but it's degraded
                if (!rangeFinderHealthy && rangeFinderDist > 0)
                {
                    AddNotification(NotificationSeverity.Warning, NotificationCategory.OpticalFlow,
                        "Rangefinder Issue", $"Rangefinder reading abnormal: {rangeFinderDist:F1}m");
                }
            }
            catch
            {
                // Ignore optical flow errors
            }
        }
    }
}
