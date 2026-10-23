// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Configuration
// ============================================================
// Handles plugin configuration persistence.
// Stored in Mission Planner's config directory.
// Configures saved aircraft settings, video, router status, observation, and operator input.
// ============================================================

using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Plugin configuration settings for NOMAD Mission Planner integration.
    /// </summary>
    public partial class NOMADConfig
    {
        // ============================================================
        // Video Streaming Configuration
        // ============================================================

        /// <summary>
        /// Named receiver inputs belonging to this aircraft. Empty means no configured source.
        /// </summary>
        public List<VideoInputProfile> VideoInputs { get; set; } = new List<VideoInputProfile>();
        public string SelectedVideoInputName { get; set; } = "";
        [JsonIgnore] public VideoInputProfile SelectedVideoInput => VideoInputs?.FirstOrDefault(input =>
            input != null && string.Equals(input.Name, SelectedVideoInputName, System.StringComparison.OrdinalIgnoreCase));

        /// <summary>
        /// Auto-start video on Mission Planner's HUD when plugin loads.
        /// This displays the configured video feed as a background overlay on the HUD.
        /// </summary>
        public bool AutoStartHudVideo { get; set; } = true;

        // ============================================================
        // Plugin Router Configuration
        // ============================================================

        /// <summary>
        /// Run the aircraft profile’s router inside Mission Planner.
        /// </summary>
        // Empty RouterLinks opens no physical connections.
        public bool RouterClientEnabled { get; set; } = false;

        /// <summary>
        /// Link monitoring interval in milliseconds.
        /// </summary>
        public int LinkMonitorInterval { get; set; } = 500;

        // ============================================================
        // Plugin router connection settings
        // ============================================================
        // The plugin owns physical links, failover,
        // duplicate suppression, and consumer permissions. Mission Planner
        // connects as the one bidirectional loopback consumer.

        /// <summary>Mission Planner's local UDP client endpoint for router telemetry.</summary>
        public int RouterLocalPort { get; set; } = 14600;

        public List<LinkConfig> RouterLinks { get; set; } = new List<LinkConfig>();
        public string RouterPreferredLink { get; set; } = "";

        // ============================================================
        // UI Configuration
        // ============================================================

        /// <summary>
        /// Show the startup diagnostics popup.
        /// </summary>
        public bool DebugMode { get; set; } = false;

        // ============================================================
        // Alert Configuration
        // ============================================================

        /// <summary>
        /// Enable audio alerts for critical warnings.
        /// </summary>
        public bool AudioAlerts { get; set; } = true;

        /// <summary>
        /// Speak Airbus-style altitude callouts in flight (gated by AudioAlerts).
        /// </summary>
        public bool AltitudeCallouts { get; set; } = true;

        public Newtonsoft.Json.Linq.JArray Inputs { get; set; } = new Newtonsoft.Json.Linq.JArray();

        /// <summary>
        /// Persisted max integrated angle rate (deg/s) at full stick deflection.
        /// At runtime, <see cref="GimbalController.MaxRateDegSec"/> is the
        /// authoritative value shared by the floating gimbal window, the
        /// settings dialog, and the physical joystick service. This field is
        /// only the on-disk snapshot — written when settings are saved,
        /// read once on plugin start to seed the controller.
        /// </summary>
        public float JoystickGimbalMaxRateDegSec { get; set; } = 60f;
        /// <summary>
        /// Capture unmodified arrow keys anywhere in Mission Planner and use them
        /// to nudge the gimbal pitch/roll target. The floating gimbal window always
        /// handles arrow keys while it is active.
        /// </summary>
        public bool GimbalArrowKeysEnabled { get; set; } = false;

    }
}
