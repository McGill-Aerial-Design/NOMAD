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
        public string JetsonIP { get; set; } = "100.85.121.98";

        /// <summary>
        /// Jetson API port.
        /// </summary>
        public int JetsonPort { get; set; } = 8000;

        /// <summary>
        /// Jetson API key (must match NOMAD_API_KEY on the Jetson).
        /// Leave empty for no authentication (development mode).
        /// </summary>
        public string JetsonApiKey { get; set; } = "";

        /// <summary>
        /// Full Jetson Base URL (computed property).
        /// </summary>
        [JsonIgnore]
        public string JetsonBaseUrl => $"http://{JetsonIP}:{JetsonPort}";

        /// <summary>
        /// Tailscale IP address (if using VPN).
        /// </summary>
        public string TailscaleIP { get; set; } = "100.85.121.98";

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
        /// Format: rtsp://&lt;jetson-ip&gt;:8554/primary
        /// </summary>
        public string VideoUrl { get; set; } = "rtsp://100.85.121.98:8554/primary";

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
        /// RadioMaster connection type: "UDP" or "COM"
        /// UDP uses network port, COM uses serial port (e.g., COM3)
        /// </summary>
        public string RadioMasterConnectionType { get; set; } = "UDP";

        /// <summary>
        /// RadioMaster UDP port (typically 14550 for RC telemetry).
        /// Used when RadioMasterConnectionType is "UDP"
        /// </summary>
        public int RadioMasterPort { get; set; } = 14550;

        /// <summary>
        /// RadioMaster COM port (e.g., "COM3", "COM4").
        /// Used when RadioMasterConnectionType is "COM"
        /// </summary>
        public string RadioMasterComPort { get; set; } = "COM3";

        /// <summary>
        /// RadioMaster COM port baud rate.
        /// ELRS typically uses 420000 or 115200
        /// </summary>
        public int RadioMasterBaudRate { get; set; } = 420000;

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
        // Drone Geometry & SLAM 3D Configuration
        // ============================================================

        /// <summary>Drone body length in cm (nose to tail).</summary>
        public float DroneLengthCm { get; set; } = 45.0f;

        /// <summary>Drone body width in cm (arm tip to arm tip).</summary>
        public float DroneWidthCm { get; set; } = 45.0f;

        /// <summary>Drone body height in cm (top to bottom).</summary>
        public float DroneHeightCm { get; set; } = 15.0f;

        /// <summary>Camera forward offset from drone center in cm.</summary>
        public float CameraForwardOffsetCm { get; set; } = 10.0f;

        /// <summary>Camera downward offset from drone center in cm.</summary>
        public float CameraDownOffsetCm { get; set; } = 5.0f;

        /// <summary>Drone frame type for 3D visualization: "Tricopter" or "Quadcopter".</summary>
        public string DroneFrameType { get; set; } = "Quadcopter";

        /// <summary>Heading offset in degrees to compensate for magnetometer calibration.</summary>
        public float SlamHeadingOffsetDeg { get; set; } = 0.0f;

        /// <summary>SLAM 3D camera field of view in degrees.</summary>
        public float SlamCameraFovDeg { get; set; } = 60.0f;

        /// <summary>SLAM 3D local map radius in meters.</summary>
        public float SlamMapRadiusM { get; set; } = 3.0f;

        // ============================================================
        // Servo Configuration (Cube Orange AUX Outputs via MAVLink)
        // All payloads, reels, water pump and camera tilt are wired to the
        // Cube and commanded via MAVLink DO_SET_SERVO. The Jetson API is
        // used only as a fallback when the MAVLink link is unavailable.
        // Channel numbers correspond to ArduPilot servo output numbers
        // (e.g. 9 = SERVO9 = AUX1 on most Cube builds).
        // ============================================================

        // --- Payload drop servos ---
        /// <summary>Cube servo output channel for payload 1 drop servo.</summary>
        public int Servo1Channel { get; set; } = 9;
        /// <summary>PWM (us) when payload 1 servo is locked/closed.</summary>
        public int Servo1PwmMin { get; set; } = 1000;
        /// <summary>PWM (us) when payload 1 servo is open/released (drop position).</summary>
        public int Servo1PwmMax { get; set; } = 2000;

        /// <summary>Cube servo output channel for payload 2 drop servo.</summary>
        public int Servo2Channel { get; set; } = 10;
        /// <summary>PWM (us) when payload 2 servo is locked/closed.</summary>
        public int Servo2PwmMin { get; set; } = 1000;
        /// <summary>PWM (us) when payload 2 servo is open/released (drop position).</summary>
        public int Servo2PwmMax { get; set; } = 2000;

        /// <summary>Cube servo output channel for payload 3 drop servo.</summary>
        public int Servo3Channel { get; set; } = 11;
        /// <summary>PWM (us) when payload 3 servo is locked/closed.</summary>
        public int Servo3PwmMin { get; set; } = 1000;
        /// <summary>PWM (us) when payload 3 servo is open/released (drop position).</summary>
        public int Servo3PwmMax { get; set; } = 2000;

        // --- Strap reel servo (payload 1) ---
        /// <summary>Cube servo output channel for payload 1 strap reel.</summary>
        public int ReelServoChannel { get; set; } = 12;
        /// <summary>PWM (us) to reel straps in (must be &gt;2000 us).</summary>
        public int ReelPwmIn { get; set; } = 2100;
        /// <summary>PWM (us) to reel straps out (must be &lt;1000 us).</summary>
        public int ReelPwmOut { get; set; } = 900;

        // --- Water pump ---
        /// <summary>Cube servo output channel for the water pump.</summary>
        public int WaterPumpChannel { get; set; } = 13;
        /// <summary>PWM (us) to activate the water pump.</summary>
        public int WaterPumpPwmOn { get; set; } = 2000;
        /// <summary>PWM (us) to stop the water pump.</summary>
        public int WaterPumpPwmOff { get; set; } = 1000;
        /// <summary>Duration (ms) the water pump fires per trigger.</summary>
        public int WaterPumpDurationMs { get; set; } = 500;

        // --- Camera tilt servo (MAVLink primary, Jetson API fallback) ---
        /// <summary>Cube servo output channel for camera tilt servo.</summary>
        public int CameraTiltChannel { get; set; } = 14;
        /// <summary>Camera tilt minimum PWM (us) — fully down.</summary>
        public int CameraTiltPwmMin { get; set; } = 700;
        /// <summary>Camera tilt maximum PWM (us) — fully up.</summary>
        public int CameraTiltPwmMax { get; set; } = 1450;

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
                VideoUrl = $"rtsp://{EffectiveIP}:8554/primary";
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

            // Keep FOV within a practical range for 3D view usability.
            if (SlamCameraFovDeg < 30.0f || SlamCameraFovDeg > 140.0f)
            {
                SlamCameraFovDeg = 60.0f;
            }

            if (SlamMapRadiusM < 1.0f || SlamMapRadiusM > 20.0f)
            {
                SlamMapRadiusM = 3.0f;
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
            JetsonApiKey = defaults.JetsonApiKey;
            TailscaleIP = defaults.TailscaleIP;
            UseTailscale = defaults.UseTailscale;
            VideoUrl = defaults.VideoUrl;
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
            DroneLengthCm = defaults.DroneLengthCm;
            DroneWidthCm = defaults.DroneWidthCm;
            DroneHeightCm = defaults.DroneHeightCm;
            CameraForwardOffsetCm = defaults.CameraForwardOffsetCm;
            CameraDownOffsetCm = defaults.CameraDownOffsetCm;
            SlamHeadingOffsetDeg = defaults.SlamHeadingOffsetDeg;
            SlamCameraFovDeg = defaults.SlamCameraFovDeg;
            SlamMapRadiusM = defaults.SlamMapRadiusM;
            Servo1Channel = defaults.Servo1Channel;
            Servo1PwmMin = defaults.Servo1PwmMin;
            Servo1PwmMax = defaults.Servo1PwmMax;
            Servo2Channel = defaults.Servo2Channel;
            Servo2PwmMin = defaults.Servo2PwmMin;
            Servo2PwmMax = defaults.Servo2PwmMax;
            Servo3Channel = defaults.Servo3Channel;
            Servo3PwmMin = defaults.Servo3PwmMin;
            Servo3PwmMax = defaults.Servo3PwmMax;
            ReelServoChannel = defaults.ReelServoChannel;
            ReelPwmIn = defaults.ReelPwmIn;
            ReelPwmOut = defaults.ReelPwmOut;
            WaterPumpChannel = defaults.WaterPumpChannel;
            WaterPumpPwmOn = defaults.WaterPumpPwmOn;
            WaterPumpPwmOff = defaults.WaterPumpPwmOff;
            WaterPumpDurationMs = defaults.WaterPumpDurationMs;
            CameraTiltChannel = defaults.CameraTiltChannel;
            CameraTiltPwmMin = defaults.CameraTiltPwmMin;
            CameraTiltPwmMax = defaults.CameraTiltPwmMax;
        }
    }
}
