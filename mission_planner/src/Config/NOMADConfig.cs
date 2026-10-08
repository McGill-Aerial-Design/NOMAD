// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Configuration
// ============================================================
// Handles plugin configuration persistence.
// Stored in Mission Planner's config directory.
// Configures runtime IPC, video, router status, observation, and operator input.
// ============================================================

using System;
using System.Collections.Generic;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Plugin configuration settings for NOMAD Mission Planner integration.
    /// </summary>
    public partial class NOMADConfig
    {
        // ============================================================
        // Connection Configuration
        // ============================================================


        /// <summary>Loopback TCP port used by the persistent C++ runtime.</summary>
        public int CoreRuntimePort { get; set; } = Connectivity.NomadCoreClient.DefaultRuntimePort;

        /// <summary>
        /// Shared secret used for HMAC proofs over loopback IPC, bound to the configured
        /// mission-planner identity. Deploy it independently of NOMAD_API_KEY.
        /// </summary>
        public string CoreClientCredential { get; set; } = "";

        // ============================================================
        // Video Streaming Configuration
        // ============================================================

        /// <summary>
        /// Video stream URL for the configured camera or video source.
        /// Default: RTSP stream supporting multiple simultaneous viewers.
        /// Format: rtsp://&lt;video-host&gt;:8554/stream
        /// </summary>
        public string VideoUrl { get; set; } = "";




        /// <summary>
        /// Auto-start video on Mission Planner's HUD when plugin loads.
        /// This displays the configured video feed as a background overlay on the HUD.
        /// </summary>
        public bool AutoStartHudVideo { get; set; } = true;

        // ============================================================
        // Standalone Ground Router Client Configuration
        // ============================================================

        /// <summary>
        /// Enable Mission Planner's standalone router status client.
        /// </summary>
        // Enables the standalone router status client only.
        public bool RouterClientEnabled { get; set; } = true;

        /// <summary>
        /// Link monitoring interval in milliseconds.
        /// </summary>
        public int LinkMonitorInterval { get; set; } = 500;

        // ============================================================
        // Standalone ground-router connection settings
        // ============================================================
        // The separately supervised host owns physical links, failover,
        // duplicate suppression, and consumer permissions. Mission Planner
        // only consumes telemetry and uses the loopback management client.

        /// <summary>Mission Planner's local UDP client endpoint for router telemetry.</summary>
        public int RouterLocalPort { get; set; } = 14600;

        /// <summary>Loopback TCP destination port for the standalone router management client.</summary>
        public int ManagementPort { get; set; } = 14610;

        // ============================================================
        // UI Configuration
        // ============================================================

        /// <summary>
        /// Enable debug logging.
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

        // ============================================================
        // Drone Geometry Configuration
        // ============================================================










        // ============================================================
        // Configured actuator commands use authenticated typed runtime IPC.
        public string JoystickPositionActuatorId { get; set; } = "";
        // ============================================================
        // Joystick Configuration (Mission Planner DirectInput-based)
        // ============================================================
        // Gimbal input produces angle targets; position input produces normalized
        // values for an explicitly configured runtime actuator ID.
        // DirectInput axes: X, Y, Z, Rx, Ry, Rz, Slider1, Slider2.

        /// <summary>Enable the gimbal joystick channel.</summary>
        public bool JoystickGimbalEnabled { get; set; } = false;
        /// <summary>DirectInput device name (must match one of MP's enumerated devices).</summary>
        public string JoystickGimbalDevice { get; set; } = "";
        /// <summary>Axis driving gimbal pitch (X / Y / Z / Rx / Ry / Rz / Slider1 / Slider2).</summary>
        public string JoystickGimbalPitchAxis { get; set; } = "Y";
        /// <summary>Invert pitch axis (stick forward = pitch up when invert=true on most flight sticks).</summary>
        public bool JoystickGimbalPitchInvert { get; set; } = true;
        /// <summary>Axis driving gimbal roll.</summary>
        public string JoystickGimbalRollAxis { get; set; } = "X";
        public bool JoystickGimbalRollInvert { get; set; } = false;
        /// <summary>Deadzone fraction [0..1] applied per axis.</summary>
        public float JoystickGimbalDeadzone { get; set; } = 0.08f;
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

        /// <summary>Enable the configured position input.</summary>
        public bool JoystickPositionEnabled { get; set; } = false;
        /// <summary>DirectInput device name. May be the same device as gimbal (different axes).</summary>
        public string JoystickPositionDevice { get; set; } = "";
        /// <summary>Axis producing a normalized position input.</summary>
        public string JoystickPositionAxis { get; set; } = "Y";
        public bool JoystickPositionInvert { get; set; } = true;
        public float JoystickPositionDeadzone { get; set; } = 0.08f;

        // Each physical switch uses configurable UP/DOWN DirectInput button indices.
        // Bindings carry backend actuator IDs and operation strings.
        public int[] JoystickButtonIndices { get; set; } = new[] { 0, 1, 2, 3, 4, 5 };
        public string JoystickSwitchDevice  { get; set; } = "";

        public string JoystickSw1UpAction   { get; set; } = "None";
        public string JoystickSw1DownAction { get; set; } = "None";
        public string JoystickSw2UpAction   { get; set; } = "None";
        public string JoystickSw2DownAction { get; set; } = "None";
        public string JoystickSw3UpAction   { get; set; } = "None";
        public string JoystickSw3DownAction { get; set; } = "None";

        /// <summary>Direct USB HID button index used only for the unavailable-termination monitor.</summary>
        public int JoystickTerminationButtonIndex { get; set; } = 6;
        /// <summary>Monitor the configured button and visibly report aircraft termination as unavailable.</summary>
        public bool JoystickKillSwitchEnabled { get; set; } = true;

    }
}
