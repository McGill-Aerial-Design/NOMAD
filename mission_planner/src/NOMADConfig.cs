// ============================================================
// NOMAD Configuration
// ============================================================
// Handles plugin configuration persistence.
// Stored in Mission Planner's config directory.
// Supports all NOMAD features including video, terminal, and VIO.
// ============================================================

using System;
using System.IO;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Plugin configuration settings for NOMAD Mission Planner integration.
    /// </summary>
    public class NOMADConfig
    {
        // ============================================================
        // Connection Configuration
        // ============================================================

        /// <summary>
        /// Jetson IP address (local network or Tailscale).
        /// </summary>
        public string JetsonIP { get; set; } = "100.75.218.89";

        /// <summary>
        /// Jetson API port.
        /// </summary>
        public int JetsonPort { get; set; } = 8000;

        /// <summary>
        /// Full Jetson Base URL (computed property).
        /// </summary>
        [JsonIgnore]
        public string JetsonBaseUrl => $"http://{JetsonIP}:{JetsonPort}";

        /// <summary>
        /// Tailscale IP address (if using VPN).
        /// </summary>
        public string TailscaleIP { get; set; } = "100.75.218.89";

        /// <summary>
        /// Use Tailscale IP instead of local IP.
        /// </summary>
        public bool UseTailscale { get; set; } = true;

        /// <summary>
        /// Gets the effective IP based on UseTailscale setting.
        /// </summary>
        [JsonIgnore]
        public string EffectiveIP => UseTailscale ? TailscaleIP : JetsonIP;

        /// <summary>
        /// Gets the effective base URL.
        /// </summary>
        [JsonIgnore]
        public string EffectiveBaseUrl => $"http://{EffectiveIP}:{JetsonPort}";

        // ============================================================
        // Video Streaming Configuration
        // ============================================================

        /// <summary>
        /// Video stream URL for ZED camera.
        /// Default: RTSP stream supporting multiple simultaneous viewers.
        /// Format: rtsp://&lt;jetson-ip&gt;:8554/zed
        /// </summary>
        public string VideoUrl { get; set; } = "rtsp://100.75.218.89:8554/zed";

        /// <summary>
        /// Servo channel for ZED camera tilt control (0 = disabled).
        /// Typically AUX1-AUX6 on flight controller (channels 9-14).
        /// </summary>
        public int ZedServoChannel { get; set; } = 10;

        /// <summary>
        /// Minimum PWM for ZED camera tilt (looking down).
        /// </summary>
        public int ZedServoMin { get; set; } = 1000;

        /// <summary>
        /// Maximum PWM for ZED camera tilt (looking up).
        /// </summary>
        public int ZedServoMax { get; set; } = 2000;

        /// <summary>
        /// Center PWM for ZED camera tilt (level).
        /// </summary>
        public int ZedServoCenter { get; set; } = 1500;

        /// <summary>
        /// Network caching for video streams (ms).
        /// Lower = less latency, higher = more stable.
        /// </summary>
        public int VideoNetworkCaching { get; set; } = 100;

        /// <summary>
        /// Preferred video player: "Embedded", "VLC", "FFplay".
        /// </summary>
        public string PreferredVideoPlayer { get; set; } = "Embedded";

        /// <summary>
        /// Enable video stream auto-start when opening video tab.
        /// </summary>
        public bool VideoAutoStart { get; set; } = false;
        
        /// <summary>
        /// Auto-start video on Mission Planner's HUD when plugin loads.
        /// This displays the ZED camera feed as a background overlay on the HUD.
        /// </summary>
        public bool AutoStartHudVideo { get; set; } = true;

        // ============================================================
        // Communication Configuration
        // ============================================================

        /// <summary>
        /// Use ELRS/MAVLink mode instead of HTTP.
        /// </summary>
        public bool UseELRS { get; set; } = false;

        /// <summary>
        /// HTTP connection timeout in seconds.
        /// </summary>
        public int HttpTimeoutSeconds { get; set; } = 5;

        /// <summary>
        /// Enable auto-reconnect on connection loss.
        /// </summary>
        public bool AutoReconnect { get; set; } = true;

        /// <summary>
        /// Health polling interval (ms).
        /// </summary>
        public int HealthPollInterval { get; set; } = 2000;

        // ============================================================
        // MAVLink Dual Link Configuration
        // ============================================================

        /// <summary>
        /// Enable MAVLink dual link management (LTE + RadioMaster failover).
        /// </summary>
        public bool DualLinkEnabled { get; set; } = true;

        /// <summary>
        /// RadioMaster UDP port (typically 14550 for RC telemetry).
        /// </summary>
        public int RadioMasterPort { get; set; } = 14550;

        /// <summary>
        /// LTE/Tailscale MAVLink UDP port on Jetson (forwarded from Cube).
        /// </summary>
        public int LteMavlinkPort { get; set; } = 14550;

        /// <summary>
        /// Enable automatic failover between links.
        /// </summary>
        public bool AutoFailoverEnabled { get; set; } = true;

        /// <summary>
        /// Preferred MAVLink link when both are available.
        /// Options: "LTE", "RadioMaster", "None"
        /// </summary>
        public string PreferredMavlinkLink { get; set; } = "LTE";

        /// <summary>
        /// Auto-reconnect to preferred link when it becomes available.
        /// </summary>
        public bool AutoReconnectToPreferred { get; set; } = true;

        /// <summary>
        /// Delay in seconds before switching back to preferred link.
        /// </summary>
        public int PreferredLinkReconnectDelay { get; set; } = 10;

        /// <summary>
        /// MAVLink heartbeat timeout in seconds before considering link dead.
        /// </summary>
        public double MavlinkHeartbeatTimeout { get; set; } = 3.0;

        /// <summary>
        /// Link monitoring interval in milliseconds.
        /// </summary>
        public int LinkMonitorInterval { get; set; } = 500;

        // ============================================================
        // Task 1 Configuration (Outdoor Recon)
        // ============================================================

        /// <summary>
        /// Enable Task 1 features.
        /// </summary>
        public bool Task1Enabled { get; set; } = true;

        /// <summary>
        /// Auto-capture on waypoint arrival.
        /// </summary>
        public bool Task1AutoCapture { get; set; } = false;

        // ============================================================
        // Task 2 Configuration (Indoor Extinguish)
        // ============================================================

        /// <summary>
        /// Enable Task 2 features.
        /// </summary>
        public bool Task2Enabled { get; set; } = true;

        /// <summary>
        /// WASD nudge speed (m/s).
        /// </summary>
        public float WasdNudgeSpeed { get; set; } = 0.5f;

        /// <summary>
        /// WASD altitude change speed (m/s).
        /// </summary>
        public float WasdAltSpeed { get; set; } = 0.3f;

        /// <summary>
        /// Enable WASD by default on Task 2 tab.
        /// </summary>
        public bool WasdAutoEnable { get; set; } = false;

        // ============================================================
        // VIO Configuration
        // ============================================================

        /// <summary>
        /// VIO confidence warning threshold (0-100).
        /// </summary>
        public float VioConfidenceWarning { get; set; } = 50.0f;

        /// <summary>
        /// VIO confidence critical threshold (0-100).
        /// </summary>
        public float VioConfidenceCritical { get; set; } = 30.0f;

        /// <summary>
        /// Enable VIO status alerts.
        /// </summary>
        public bool VioAlertsEnabled { get; set; } = true;

        // ============================================================
        // Terminal Configuration
        // ============================================================

        /// <summary>
        /// SSH username for direct SSH connection.
        /// </summary>
        public string SshUsername { get; set; } = "mad";

        /// <summary>
        /// Terminal command timeout (seconds).
        /// </summary>
        public int TerminalTimeout { get; set; } = 30;

        /// <summary>
        /// Save terminal history between sessions.
        /// </summary>
        public bool SaveTerminalHistory { get; set; } = true;

        // ============================================================
        // UI Configuration
        // ============================================================

        /// <summary>
        /// Enable debug logging.
        /// </summary>
        public bool DebugMode { get; set; } = false;

        /// <summary>
        /// Show notifications for status changes.
        /// </summary>
        public bool ShowNotifications { get; set; } = true;

        /// <summary>
        /// Default tab to show on startup.
        /// </summary>
        public string DefaultTab { get; set; } = "Dashboard";

        /// <summary>
        /// Enable dark mode for NOMAD UI.
        /// </summary>
        public bool DarkMode { get; set; } = true;

        // ============================================================
        // Alert Configuration
        // ============================================================

        /// <summary>
        /// Temperature warning threshold (Celsius).
        /// </summary>
        public float TempWarningC { get; set; } = 75.0f;

        /// <summary>
        /// Temperature critical threshold (Celsius).
        /// </summary>
        public float TempCriticalC { get; set; } = 85.0f;

        /// <summary>
        /// Enable audio alerts for critical warnings.
        /// </summary>
        public bool AudioAlerts { get; set; } = true;

        // ============================================================
        // Persistence
        // ============================================================

        private static string ConfigPath => Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "Mission Planner",
            "plugins",
            "nomad_config.json"
        );

        /// <summary>
        /// Load configuration from file.
        /// </summary>
        public static NOMADConfig Load()
        {
            try
            {
                if (File.Exists(ConfigPath))
                {
                    var json = File.ReadAllText(ConfigPath);
                    var config = JsonConvert.DeserializeObject<NOMADConfig>(json);
                    
                    // Ensure non-null return
                    if (config != null)
                    {
                        // Migrate any missing properties with defaults
                        config.MigrateDefaults();
                        return config;
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to load config - {ex.Message}");
            }

            return new NOMADConfig();
        }

        /// <summary>
        /// Save configuration to file.
        /// </summary>
        public void Save()
        {
            try
            {
                var dir = Path.GetDirectoryName(ConfigPath);
                if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                {
                    Directory.CreateDirectory(dir);
                }

                var json = JsonConvert.SerializeObject(this, Formatting.Indented);
                File.WriteAllText(ConfigPath, json);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to save config - {ex.Message}");
            }
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
                VideoUrl = $"rtsp://{EffectiveIP}:8554/zed";
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
        }

        /// <summary>
        /// Create a copy of the configuration.
        /// </summary>
        public NOMADConfig Clone()
        {
            var json = JsonConvert.SerializeObject(this);
            return JsonConvert.DeserializeObject<NOMADConfig>(json) ?? new NOMADConfig();
        }

        /// <summary>
        /// Reset to default values.
        /// </summary>
        public void ResetToDefaults()
        {
            var defaults = new NOMADConfig();
            
            // Copy all properties from defaults
            JetsonIP = defaults.JetsonIP;
            JetsonPort = defaults.JetsonPort;
            TailscaleIP = defaults.TailscaleIP;
            UseTailscale = defaults.UseTailscale;
            VideoUrl = defaults.VideoUrl;
            ZedServoChannel = defaults.ZedServoChannel;
            ZedServoMin = defaults.ZedServoMin;
            ZedServoMax = defaults.ZedServoMax;
            ZedServoCenter = defaults.ZedServoCenter;
            VideoNetworkCaching = defaults.VideoNetworkCaching;
            PreferredVideoPlayer = defaults.PreferredVideoPlayer;
            VideoAutoStart = defaults.VideoAutoStart;
            UseELRS = defaults.UseELRS;
            HttpTimeoutSeconds = defaults.HttpTimeoutSeconds;
            AutoReconnect = defaults.AutoReconnect;
            HealthPollInterval = defaults.HealthPollInterval;
            Task1Enabled = defaults.Task1Enabled;
            Task1AutoCapture = defaults.Task1AutoCapture;
            Task2Enabled = defaults.Task2Enabled;
            WasdNudgeSpeed = defaults.WasdNudgeSpeed;
            WasdAltSpeed = defaults.WasdAltSpeed;
            WasdAutoEnable = defaults.WasdAutoEnable;
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
        }
    }
}
