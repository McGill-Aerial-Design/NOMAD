// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADConfig
    {
        private sealed class UnsupportedConfigurationMigrationException : JsonSerializationException
        {
            public UnsupportedConfigurationMigrationException(string message) : base(message) { }
        }

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
                catch (UnsupportedConfigurationMigrationException ex)
                {
                    Log.Error($"Configuration requires migration before Mission Planner can start - {ex.Message}");
                    throw new InvalidDataException(ex.Message, ex);
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
            var migratedJson = MigrateLegacyConfigKeys(json);
            var config = JsonConvert.DeserializeObject<NOMADConfig>(migratedJson);
            if (config == null)
                throw new JsonSerializationException("The configuration file did not contain a NOMAD configuration.");

            config.MigrateDefaults();
            return config;
        }

        private static string MigrateLegacyConfigKeys(string json)
        {
            var document = JObject.Parse(json);
            var mappings = new[]
            {
                ("JoystickZedEnabled", "JoystickCameraTiltEnabled"),
                ("JoystickZedDevice", "JoystickCameraTiltDevice"),
                ("JoystickZedTiltAxis", "JoystickCameraTiltAxis"),
                ("JoystickZedTiltInvert", "JoystickCameraTiltInvert"),
                ("JoystickZedDeadzone", "JoystickCameraTiltDeadzone"),
                ("JoystickZedMaxRateUsPerSec", "JoystickCameraTiltMaxRateUsPerSec"),
            };
            foreach (var (legacyKey, currentKey) in mappings)
            {
                if (document[currentKey] == null && document[legacyKey] != null)
                    document[currentKey] = document[legacyKey];
                document.Remove(legacyKey);
            }

            var legacyMode = document["RouterMode"]?.Value<string>();
            if (!string.IsNullOrWhiteSpace(legacyMode) &&
                !string.Equals(legacyMode, "Standalone", StringComparison.OrdinalIgnoreCase))
            {
                throw new UnsupportedConfigurationMigrationException(
                    $"RouterMode '{legacyMode}' is unsupported; run only the standalone ground router.");
            }

            ValidateLegacyLoopbackSetting(document, "RouterBindAddress");
            ValidateLegacyLoopbackSetting(document, "ManagementBindAddress");

            // DualLinkEnabled was the source of truth in the old model. If it
            // is present it wins; otherwise migrate the older RouterEnabled key.
            if (document["DualLinkEnabled"] == null && document["RouterEnabled"] != null)
            {
                document["DualLinkEnabled"] = document["RouterEnabled"];
            }

            var removed = new[]
            {
                "IntegratedFlightMode",
                "RouterLinks", "RouterConsumers", "RouterEnabled", "RouterMode",
                "RadioMasterConnectionType", "RadioMasterPort", "RadioMasterComPort",
                "RadioMasterTcpHost", "RadioMasterBaudRate", "LteMavlinkPort",
                "LteRemoteHost", "LteRemotePort", "AutoFailoverEnabled",
                "PreferredMavlinkLink", "AutoReconnectToPreferred",
                "PreferredLinkReconnectDelay", "MavlinkHeartbeatTimeout",
                "RouterBindAddress", "RouterDedupEnabled", "ManagementBindAddress",
                "CoreMavlinkEndpoint", "CoreClientMode", "CoreExePath",
                "JetsonApiKey", "JetsonIP", "JetsonPort",
            };
            var found = new System.Collections.Generic.List<string>();
            foreach (var key in removed)
            {
                if (document.Property(key) != null)
                {
                    found.Add(key);
                    document.Remove(key);
                }
            }
            if (found.Count > 0)
            {
                Log.Warn("Removed retired aircraft/router settings from Mission Planner config: " +
                    string.Join(", ", found) + ". Configure aircraft transport in nomad-runtime and ground links " +
                    "in the standalone router JSON.");
            }
            return document.ToString(Formatting.None);
        }

        private static void ValidateLegacyLoopbackSetting(JObject document, string key)
        {
            var address = document[key]?.Value<string>();
            if (!string.IsNullOrWhiteSpace(address) && address != "127.0.0.1")
            {
                throw new UnsupportedConfigurationMigrationException(
                    $"{key} must be 127.0.0.1; the standalone router is loopback-only.");
            }
        }

        /// <summary>
        /// Migrate defaults for properties that may have been added in newer versions.
        /// </summary>
        private void MigrateDefaults()
        {
            if (CoreRuntimePort < 1 || CoreRuntimePort > 65535)
            {
                CoreRuntimePort = Connectivity.NomadCoreClient.DefaultRuntimePort;
            }

            // Older profiles used an API-derived video URL. Keep them usable by
            // falling back to the standalone RTSP bridge's documented local URL.
            if (VideoUrl == "udp://@:5600" || string.IsNullOrWhiteSpace(VideoUrl))
            {
                VideoUrl = "rtsp://127.0.0.1:8554/stream";
            }

            if (ManagementPort < 1 || ManagementPort > 65535)
            {
                ManagementPort = 14610;
            }

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

            ActiveProfile = defaults.ActiveProfile;
            CoreRuntimePort = defaults.CoreRuntimePort;
            RouterLocalPort = defaults.RouterLocalPort;
            ManagementPort = defaults.ManagementPort;
            CoreApiKey = defaults.CoreApiKey;
            VideoUrl = defaults.VideoUrl;
            VideoNetworkCaching = defaults.VideoNetworkCaching;
            PreferredVideoPlayer = defaults.PreferredVideoPlayer;
            VideoAutoStart = defaults.VideoAutoStart;
            AutoStartHudVideo = defaults.AutoStartHudVideo;
            DebugMode = defaults.DebugMode;
            ShowNotifications = defaults.ShowNotifications;
            DefaultTab = defaults.DefaultTab;
            DarkMode = defaults.DarkMode;
            TempWarningC = defaults.TempWarningC;
            TempCriticalC = defaults.TempCriticalC;
            AudioAlerts = defaults.AudioAlerts;
            AltitudeCallouts = defaults.AltitudeCallouts;
            DefaultLogDirectory = defaults.DefaultLogDirectory;
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
