// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.IO;
using System.Globalization;
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
            return LoadFromPaths(ConfigPath, ConfigPath + ".bak");
        }

        internal static NOMADConfig LoadFromPaths(string primary, string backup)
        {
            foreach (var path in new[]
            {
                primary, backup
            }
            )
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
                ValidateInputBindings();
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
                try
                {
                    File.Delete(ConfigPath + ".tmp");
                }
                catch
                {

                }
                throw new IOException("Configuration was not saved.", ex);
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

            var profile = JObject.FromObject(this);
            profile.Remove(nameof(CoreClientCredential));
            File.WriteAllText(path, profile.ToString(Formatting.Indented));
        }

        private static NOMADConfig Deserialize(string json)
        {
            var document = JObject.Parse(json);
            try
            {
                var migratedJson = MigrateLegacyConfigKeys(json);
                var config = JsonConvert.DeserializeObject<NOMADConfig>(migratedJson);
                if (config == null)
                { throw new JsonSerializationException("The configuration file did not contain a NOMAD configuration."); }
                config.MigrateDefaults();
                return config;
            }
            catch (UnsupportedConfigurationMigrationException)
            {
                throw;
            }
            catch (Exception ex) when (document.Property("Payloads") != null || document.Property("Actuators") != null)
            {
                throw new UnsupportedConfigurationMigrationException("Invalid actuator configuration: " + ex.Message);
            }
        }

        private static string MigrateLegacyConfigKeys(string json)
        {
            var document = JObject.Parse(json);
            RejectRetiredActuatorOwnership(document);
            ValidateInputDocument(document);
            bool legacyAxisEnabled = document["JoystickCameraTiltEnabled"]?.Value<bool>() == true ||
                document["JoystickZedEnabled"]?.Value<bool>() == true;
            if (legacyAxisEnabled && document["JoystickPositionEnabled"] == null)
            { throw new UnsupportedConfigurationMigrationException(
                "Enabled legacy relative-rate position input requires explicit review before absolute position input is enabled. Preserve the original file."); }
            foreach (var suffix in new[]
            {
                "Enabled", "Device", "Axis", "Invert", "Deadzone"
            }
            )
            {
                string old = "JoystickCameraTilt" + suffix;
                string current = "JoystickPosition" + suffix;
                if (document[current] == null && document[old] != null)
                {
                    document[current] = document[old];
                }
                document.Remove(old);
            }
            foreach (var suffix in new[]
            {
                "Enabled", "Device", "TiltAxis", "TiltInvert", "Deadzone"
            }
            )
            {
                string old = "JoystickZed" + suffix;
                string current = "JoystickPosition" + suffix.Replace("Tilt", "");
                if (document[current] == null && document[old] != null)
                {
                    document[current] = document[old];
                }
                document.Remove(old);
            }
            foreach (var retired in new[]
            {
                "JoystickAutoSelectDevice", "JoystickCameraTiltMaxRateUsPerSec", "JoystickZedMaxRateUsPerSec"
            }
            )
            { document.Remove(retired); }

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
                "SprayTargetCameraRangeM",
                "SprayRangeToleranceM",
                "SprayTriggerMaxDistanceM",
                "SprayAimPixelX",
                "SprayAimPixelY",
                "SprayAimTolerancePx",
                "SprayServoFireAngleDeg",
                "SprayForwardGain",
                "SprayLateralGain",
                "SprayAltitudeGain",
                "SprayYawGain",
                "SprayUseYawAlignment",
                "SprayMaxForwardSpeedMps",
                "SprayMaxLateralSpeedMps",
                "SprayMaxAltitudeSpeedMps",
                "SprayMaxYawRateRadps",
                "SprayLockHoldMs",
                "SprayAlignTimeoutS",
                "RouterLinks", "RouterConsumers", "RouterEnabled", "RouterMode",
                "RadioMasterConnectionType", "RadioMasterPort", "RadioMasterComPort",
                "RadioMasterTcpHost", "RadioMasterBaudRate", "LteMavlinkPort",
                "LteRemoteHost", "LteRemotePort", "AutoFailoverEnabled",
                "PreferredMavlinkLink", "AutoReconnectToPreferred",
                "PreferredLinkReconnectDelay", "MavlinkHeartbeatTimeout",
                "RouterBindAddress", "RouterDedupEnabled", "ManagementBindAddress",
                "CoreMavlinkEndpoint", "CoreClientMode", "CoreExePath",
                "JetsonApiKey", "JetsonIP", "JetsonPort", "CoreApiKey",
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

        private static void RejectRetiredActuatorOwnership(JObject document)
        {
            foreach (var key in new[]
            {
                "Payloads", "Actuators"
            }
            )
            {
                if (document[key] == null)
                {
                    continue;
                }
                if (!(document[key] is JArray list) || list.Count > 0)
                { throw new UnsupportedConfigurationMigrationException("Persisted " + key +
                    " must be migrated to the runtime actuator configuration before loading. Preserve the original file."); }
                document.Remove(key);
            }
            if (document["SerialJoystickEnabled"]?.Value<bool>() == true ||
                !string.IsNullOrWhiteSpace(document["SerialJoystickScriptPath"]?.Value<string>()))
            { throw new UnsupportedConfigurationMigrationException(
                "Serial/virtual joystick bridging is retired. Select and review direct USB HID mappings; preserve the original file."); }
            foreach (var key in new[] { "SerialJoystickEnabled", "SerialJoystickPort", "SerialJoystickBaud",
                "SerialJoystickPython", "SerialJoystickScriptPath" }) { document.Remove(key); }
        }

        private static void ValidateInputDocument(JObject document)
        {
            var termination = document["JoystickTerminationButtonIndex"];
            var monitor = document["JoystickKillSwitchEnabled"];
            if ((termination != null && termination.Type != JTokenType.Integer) ||
                (monitor != null && monitor.Type != JTokenType.Boolean))
            {
                throw new UnsupportedConfigurationMigrationException(
                    "HID termination index must be an integer and its enabled flag must be boolean.");
            }
            if (termination != null) { ValidatePhysicalIndex(termination, "HID termination index"); }
            if (document["JoystickButtonIndices"] == null) { return; }
            if (!(document["JoystickButtonIndices"] is JArray indices))
            { throw new UnsupportedConfigurationMigrationException("HID button indices must be an integer array."); }
            foreach (var index in indices)
            {
                if (index.Type != JTokenType.Integer)
                { throw new UnsupportedConfigurationMigrationException("HID button indices must be integers."); }
                ValidatePhysicalIndex(index, "HID button indices");
            }
        }

        private static void ValidatePhysicalIndex(JToken token, string name)
        {
            if (!int.TryParse(token.ToString(Formatting.None), NumberStyles.Integer, CultureInfo.InvariantCulture,
                out int value) || value < 0 || value > 127)
            { throw new UnsupportedConfigurationMigrationException(name + " must be an integer between 0 and 127."); }
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
            ValidateInputBindings();
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

        }

        internal void ValidateInputBindings()
        {
            string error = GetInputMappingError();
            if (error != null) { throw new UnsupportedConfigurationMigrationException(error); }
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
            CoreClientCredential = defaults.CoreClientCredential;
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
            JoystickPositionActuatorId = defaults.JoystickPositionActuatorId;
        }
    }
}
