// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.IO;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADConfig
    {
        private static string ConfigPath => Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "Mission Planner",
            "plugins",
            "nomad_config.json"
        );

        /// <summary>
        /// Load configuration from file. If the primary file is missing or
        /// corrupt, fall back to the .bak written by the last successful Save().
        /// </summary>
        public static NOMADConfig Load()
        {
            var primary = ConfigPath;
            var backup = primary + ".bak";

            foreach (var path in new[] { primary, backup })
            {
                try
                {
                    if (!File.Exists(path)) continue;
                    var json = File.ReadAllText(path);
                    if (string.IsNullOrWhiteSpace(json)) continue;
                    var config = Deserialize(json);
                    if (path == backup)
                        Log.Warn("Loaded config from .bak (primary corrupt or missing).");
                    return config;
                }
                catch (Exception ex)
                {
                    Log.Error($"Failed to load config from {path} - {ex.Message}");
                }
            }

            return new NOMADConfig();
        }

        /// <summary>Load and validate a configuration profile from an arbitrary JSON file.</summary>
        public static NOMADConfig LoadFromFile(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
                throw new ArgumentException("A configuration file path is required.", nameof(path));

            var json = File.ReadAllText(path);
            if (string.IsNullOrWhiteSpace(json))
                throw new InvalidDataException("The configuration file is empty.");

            return Deserialize(json);
        }

        /// <summary>
        /// Save configuration atomically: write to .tmp first, then swap
        /// using File.Replace which keeps the previous version as .bak.
        /// </summary>
        public void Save()
        {
            try
            {
                var path = ConfigPath;
                var dir = Path.GetDirectoryName(path);
                if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                    Directory.CreateDirectory(dir);

                var json = JsonConvert.SerializeObject(this, Formatting.Indented);
                var tmp = path + ".tmp";
                var bak = path + ".bak";

                File.WriteAllText(tmp, json);

                if (File.Exists(path))
                {
                    // Atomic rename + backup. Replace() requires the destination
                    // to exist; otherwise fall through to a plain Move().
                    File.Replace(tmp, path, bak, ignoreMetadataErrors: true);
                }
                else
                {
                    File.Move(tmp, path);
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Failed to save config - {ex.Message}");
                try { File.Delete(ConfigPath + ".tmp"); } catch { }
            }
        }

        /// <summary>Export this configuration as a portable JSON profile.</summary>
        public void ExportToFile(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
                throw new ArgumentException("A configuration file path is required.", nameof(path));

            var directory = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
                Directory.CreateDirectory(directory);

            File.WriteAllText(path, JsonConvert.SerializeObject(this, Formatting.Indented));
        }

        private static NOMADConfig Deserialize(string json)
        {
            var config = JsonConvert.DeserializeObject<NOMADConfig>(json);
            if (config == null)
                throw new JsonSerializationException("The configuration file did not contain a NOMAD configuration.");

            config.MigrateDefaults();
            return config;
        }

        /// <summary>
        /// Migrate defaults for properties that may have been added in newer versions.
        /// </summary>
        private void MigrateDefaults()
        {
            // Migrate from old UDP format to RTSP (multiple viewers)
            if (VideoUrl == "udp://@:5600" || string.IsNullOrEmpty(VideoUrl))
            {
                // New default is RTSP stream (allows multiple viewers)
                var ip = EffectiveIP;
                if (string.IsNullOrWhiteSpace(ip))
                    ip = JetsonIP;
                VideoUrl = $"rtsp://{ip}:8554/stream";
            }

            // Migrate old Jetson IP to Tailscale if using Tailscale
            if (JetsonIP == "192.168.1.100" && UseTailscale)
            {
                JetsonIP = TailscaleIP;
            }

            // Migrate SSH username from 'nomad' to 'mad'
            if (SshUsername == "nomad")
            {
                SshUsername = "mad";
            }

            // Bump LTE MAVLink port off the RadioMaster default (14550) so the
            // two links don't fight for the same UDP port on the GCS. Users
            // who explicitly set a non-default value keep it.
            if (LteMavlinkPort == 14550)
            {
                LteMavlinkPort = 14560;
            }

            // Keep the old high-level dual-link toggle and the newer local
            // router toggle in lockstep unless a future UI exposes them separately.
            RouterEnabled = DualLinkEnabled;

            // Keep FOV within a practical range for 3D view usability.
            if (SlamCameraFovDeg < 30.0f || SlamCameraFovDeg > 140.0f)
            {
                SlamCameraFovDeg = 60.0f;
            }

            if (SlamMapRadiusM < 1.0f || SlamMapRadiusM > 20.0f)
            {
                SlamMapRadiusM = 3.0f;
            }

            LogVibrationWarning = ClampLog(LogVibrationWarning, 0, 200, 30);
            LogVibrationCritical = Math.Max(
                LogVibrationWarning,
                ClampLog(LogVibrationCritical, 0, 250, 60));
            LogHdopWarning = ClampLog(LogHdopWarning, 0, 20, 2);
            LogHdopCritical = Math.Max(
                LogHdopWarning,
                ClampLog(LogHdopCritical, 0, 30, 4));
            LogTuneRmsWarning = ClampLog(LogTuneRmsWarning, 0, 90, 5);
            LogTuneRmsCritical = Math.Max(
                LogTuneRmsWarning,
                ClampLog(LogTuneRmsCritical, 0, 180, 10));
            LogEkfVarianceWarning = ClampLog(LogEkfVarianceWarning, 0, 10, 0.8);
            LogEkfVarianceCritical = Math.Max(
                LogEkfVarianceWarning,
                ClampLog(LogEkfVarianceCritical, 0, 20, 1));
            if (LogMinimumSatellites < 0 || LogMinimumSatellites > 40) LogMinimumSatellites = 8;
            if (LogLiveBufferPoints < 60 || LogLiveBufferPoints > 10000) LogLiveBufferPoints = 600;
            if (string.IsNullOrWhiteSpace(JetsonLogDirectory)) JetsonLogDirectory = "~/NOMAD/logs";
            MotorMusicMotorCount = ClampInt(MotorMusicMotorCount, 1, 12, 4);
            MotorMusicMinOutputPwm = ClampInt(MotorMusicMinOutputPwm, 1000, 2000, 1100);
            MotorMusicMaxOutputPwm = ClampInt(MotorMusicMaxOutputPwm, MotorMusicMinOutputPwm, 2000, 1800);
            MotorMusicTranspose = ClampInt(MotorMusicTranspose, -48, 12, -24);
            if (MotorMusicTempoScale < 0.25 || MotorMusicTempoScale > 2.0)
                MotorMusicTempoScale = 1.0;

            if (Payloads == null)
            {
                Payloads = DefaultPayloads();
            }

            SprayTargetCameraRangeM = Clamp(SprayTargetCameraRangeM, 0.5f, 8.0f, 3.8f);
            SprayRangeToleranceM = Clamp(SprayRangeToleranceM, 0.05f, 1.0f, 0.25f);
            SprayTriggerMaxDistanceM = Clamp(SprayTriggerMaxDistanceM, 1.0f, 8.0f, 5.5f);
            if (SprayAimPixelX < 0 || SprayAimPixelX > 4000) SprayAimPixelX = 640;
            if (SprayAimPixelY < 0 || SprayAimPixelY > 3000) SprayAimPixelY = 390;
            if (SprayAimTolerancePx < 2 || SprayAimTolerancePx > 250) SprayAimTolerancePx = 25;
            SprayServoFireAngleDeg = Clamp(SprayServoFireAngleDeg, 0.0f, 180.0f, 82.0f);
            SprayForwardGain = Clamp(SprayForwardGain, 0.0f, 2.0f, 0.45f);
            SprayLateralGain = Clamp(SprayLateralGain, -0.02f, 0.02f, 0.0010f);
            SprayAltitudeGain = Clamp(SprayAltitudeGain, -0.02f, 0.02f, 0.0010f);
            SprayYawGain = Clamp(SprayYawGain, -0.02f, 0.02f, 0.0025f);
            SprayMaxForwardSpeedMps = Clamp(SprayMaxForwardSpeedMps, 0.05f, 2.0f, 0.45f);
            SprayMaxLateralSpeedMps = Clamp(SprayMaxLateralSpeedMps, 0.05f, 1.0f, 0.25f);
            SprayMaxAltitudeSpeedMps = Clamp(SprayMaxAltitudeSpeedMps, 0.05f, 1.0f, 0.20f);
            SprayMaxYawRateRadps = Clamp(SprayMaxYawRateRadps, 0.05f, 2.0f, 0.35f);
            if (SprayLockHoldMs < 100 || SprayLockHoldMs > 5000) SprayLockHoldMs = 700;
            SprayAlignTimeoutS = Clamp(SprayAlignTimeoutS, 2.0f, 60.0f, 20.0f);
        }

        private static float Clamp(float value, float min, float max, float fallback)
        {
            if (float.IsNaN(value) || float.IsInfinity(value)) return fallback;
            return Math.Max(min, Math.Min(max, value));
        }

        private static double ClampLog(double value, double min, double max, double fallback)
        {
            if (double.IsNaN(value) || double.IsInfinity(value) || value < min || value > max)
                return fallback;
            return value;
        }

        private static int ClampInt(int value, int min, int max, int fallback)
        {
            if (value < min || value > max) return fallback;
            return value;
        }

        /// <summary>
        /// Create a copy of the configuration.
        /// </summary>
        public NOMADConfig Clone()
        {
            var json = JsonConvert.SerializeObject(this);
            return Deserialize(json);
        }

        /// <summary>
        /// Reset to default values.
        /// </summary>
        public void ResetToDefaults()
        {
            var defaults = new NOMADConfig();

            JetsonIP = defaults.JetsonIP;
            JetsonPort = defaults.JetsonPort;
            JetsonApiKey = defaults.JetsonApiKey;
            JetsonSshUser = defaults.JetsonSshUser;
            TailscaleIP = defaults.TailscaleIP;
            UseTailscale = defaults.UseTailscale;
            VideoUrl = defaults.VideoUrl;
            VideoNetworkCaching = defaults.VideoNetworkCaching;
            PreferredVideoPlayer = defaults.PreferredVideoPlayer;
            VideoAutoStart = defaults.VideoAutoStart;
            HttpTimeoutSeconds = defaults.HttpTimeoutSeconds;
            AutoReconnect = defaults.AutoReconnect;
            HealthPollInterval = defaults.HealthPollInterval;
            VioConfidenceWarning = defaults.VioConfidenceWarning;
            VioConfidenceCritical = defaults.VioConfidenceCritical;
            VioAlertsEnabled = defaults.VioAlertsEnabled;
            SshUsername = defaults.SshUsername;
            TerminalTimeout = defaults.TerminalTimeout;
            SaveTerminalHistory = defaults.SaveTerminalHistory;
            DebugMode = defaults.DebugMode;
            ShowNotifications = defaults.ShowNotifications;
            DefaultTab = defaults.DefaultTab;
            DarkMode = defaults.DarkMode;
            TempWarningC = defaults.TempWarningC;
            TempCriticalC = defaults.TempCriticalC;
            AudioAlerts = defaults.AudioAlerts;
            AltitudeCallouts = defaults.AltitudeCallouts;
            MotorMusicMotorCount = defaults.MotorMusicMotorCount;
            MotorMusicMinOutputPwm = defaults.MotorMusicMinOutputPwm;
            MotorMusicMaxOutputPwm = defaults.MotorMusicMaxOutputPwm;
            MotorMusicTranspose = defaults.MotorMusicTranspose;
            MotorMusicTempoScale = defaults.MotorMusicTempoScale;
            DefaultLogDirectory = defaults.DefaultLogDirectory;
            JetsonLogDirectory = defaults.JetsonLogDirectory;
            LogVibrationWarning = defaults.LogVibrationWarning;
            LogVibrationCritical = defaults.LogVibrationCritical;
            LogHdopWarning = defaults.LogHdopWarning;
            LogHdopCritical = defaults.LogHdopCritical;
            LogMinimumSatellites = defaults.LogMinimumSatellites;
            LogTuneRmsWarning = defaults.LogTuneRmsWarning;
            LogTuneRmsCritical = defaults.LogTuneRmsCritical;
            LogEkfVarianceWarning = defaults.LogEkfVarianceWarning;
            LogEkfVarianceCritical = defaults.LogEkfVarianceCritical;
            LogLiveBufferPoints = defaults.LogLiveBufferPoints;
            LogInjectAlertsToHud = defaults.LogInjectAlertsToHud;
            DroneLengthCm = defaults.DroneLengthCm;
            DroneWidthCm = defaults.DroneWidthCm;
            DroneHeightCm = defaults.DroneHeightCm;
            CameraForwardOffsetCm = defaults.CameraForwardOffsetCm;
            CameraDownOffsetCm = defaults.CameraDownOffsetCm;
            SlamHeadingOffsetDeg = defaults.SlamHeadingOffsetDeg;
            SlamCameraFovDeg = defaults.SlamCameraFovDeg;
            SlamMapRadiusM = defaults.SlamMapRadiusM;
            Payloads = defaults.Payloads;
            SprayTargetCameraRangeM = defaults.SprayTargetCameraRangeM;
            SprayRangeToleranceM = defaults.SprayRangeToleranceM;
            SprayTriggerMaxDistanceM = defaults.SprayTriggerMaxDistanceM;
            SprayAimPixelX = defaults.SprayAimPixelX;
            SprayAimPixelY = defaults.SprayAimPixelY;
            SprayAimTolerancePx = defaults.SprayAimTolerancePx;
            SprayServoFireAngleDeg = defaults.SprayServoFireAngleDeg;
            SprayForwardGain = defaults.SprayForwardGain;
            SprayLateralGain = defaults.SprayLateralGain;
            SprayAltitudeGain = defaults.SprayAltitudeGain;
            SprayYawGain = defaults.SprayYawGain;
            SprayUseYawAlignment = defaults.SprayUseYawAlignment;
            SprayMaxForwardSpeedMps = defaults.SprayMaxForwardSpeedMps;
            SprayMaxLateralSpeedMps = defaults.SprayMaxLateralSpeedMps;
            SprayMaxAltitudeSpeedMps = defaults.SprayMaxAltitudeSpeedMps;
            SprayMaxYawRateRadps = defaults.SprayMaxYawRateRadps;
            SprayLockHoldMs = defaults.SprayLockHoldMs;
            SprayAlignTimeoutS = defaults.SprayAlignTimeoutS;
        }
    }
}
