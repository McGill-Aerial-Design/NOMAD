// ============================================================
// NOMAD Configuration
// ============================================================
// Handles plugin configuration persistence.
// Stored in Mission Planner's config directory.
// ============================================================

using System;
using System.IO;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Plugin configuration settings.
    /// </summary>
    public class NOMADConfig
    {
        // ============================================================
        // Configuration Properties
        // ============================================================

        /// <summary>
        /// Jetson IP address (local network).
        /// </summary>
        public string JetsonIP { get; set; } = "192.168.1.100";

        /// <summary>
        /// Jetson API port.
        /// </summary>
        public int JetsonPort { get; set; } = 8000;

        /// <summary>
        /// Full Jetson Base URL (computed property).
        /// </summary>
        public string JetsonBaseUrl => $"http://{JetsonIP}:{JetsonPort}";

        /// <summary>
        /// RTSP video stream URL (primary camera - ZED/Navigation).
        /// </summary>
        public string RtspUrlPrimary { get; set; } = "rtsp://192.168.1.100:8554/live";

        /// <summary>
        /// RTSP video stream URL (secondary camera - Gimbal/Targeting).
        /// </summary>
        public string RtspUrlSecondary { get; set; } = "rtsp://192.168.1.100:8554/gimbal";

        /// <summary>
        /// Use ELRS/MAVLink mode instead of HTTP.
        /// </summary>
        public bool UseELRS { get; set; } = false;

        /// <summary>
        /// HTTP connection timeout in seconds.
        /// </summary>
        public int HttpTimeoutSeconds { get; set; } = 5;

        /// <summary>
        /// Enable debug logging.
        /// </summary>
        public bool DebugMode { get; set; } = false;

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
                    return JsonConvert.DeserializeObject<NOMADConfig>(json) ?? new NOMADConfig();
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
                if (!Directory.Exists(dir))
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
    }
}
