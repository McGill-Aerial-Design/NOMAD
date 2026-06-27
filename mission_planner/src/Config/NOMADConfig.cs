// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Configuration
// ============================================================
// Handles plugin configuration persistence.
// Stored in Mission Planner's config directory.
// Supports all NOMAD features including video, terminal, and VIO.
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

        /// <summary>
        /// Jetson IP address (local network or Tailscale).
        /// </summary>
        /// <summary>
        /// Active NOMAD config profile name. Written by the profile loader
        /// (scripts/profile.py) so the plugin can show which profile is live.
        /// </summary>
        public string ActiveProfile { get; set; } = "dev";

        public string JetsonIP { get; set; } = "";

        /// <summary>
        /// Jetson API port.
        /// </summary>
        public int JetsonPort { get; set; } = 8000;

        /// <summary>
        /// Jetson API key (must match NOMAD_API_KEY on the Jetson).
        /// Defaults to the committed DEV key so the plugin works against the dev
        /// stack out of the box. Override locally (untracked) for a real drone.
        /// </summary>
        public string JetsonApiKey { get; set; } = "nomad-dev-key";

        /// <summary>
        /// SSH login user on the Jetson (used by terminal/service control over SSH).
        /// </summary>
        public string JetsonSshUser { get; set; } = "nomad";

        /// <summary>
        /// Full Jetson Base URL (computed property).
        /// </summary>
        [JsonIgnore]
        public string JetsonBaseUrl => $"http://{JetsonIP}:{JetsonPort}";

        /// <summary>
        /// Tailscale IP address (if using VPN).
        /// </summary>
        public string TailscaleIP { get; set; } = "";

        /// <summary>
        /// Use Tailscale IP instead of local IP.
        /// </summary>
        public bool UseTailscale { get; set; } = true;

        /// <summary>
        /// Gets the effective IP based on UseTailscale setting.
        /// </summary>
        [JsonIgnore]
        public string EffectiveIP => UseTailscale && !string.IsNullOrWhiteSpace(TailscaleIP) ? TailscaleIP : JetsonIP;

        /// <summary>
        /// Gets the effective base URL.
        /// </summary>
        [JsonIgnore]
        public string EffectiveBaseUrl
        {
            get
            {
                var ip = EffectiveIP;
                if (string.IsNullOrWhiteSpace(ip))
                    ip = "127.0.0.1";
                return $"http://{ip}:{JetsonPort}";
            }
        }

        // ============================================================
        // Video Streaming Configuration
        // ============================================================

        /// <summary>
        /// Video stream URL for ZED camera.
        /// Default: RTSP stream supporting multiple simultaneous viewers.
        /// Format: rtsp://&lt;jetson-ip&gt;:8554/stream
        /// </summary>
        public string VideoUrl { get; set; } = "";

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
        public int HealthPollInterval { get; set; } = 5000;

        // ============================================================
        // MAVLink Dual Link Configuration
        // ============================================================

        /// <summary>
        /// Enable MAVLink dual link management (LTE + RadioMaster failover).
        /// </summary>
        public bool DualLinkEnabled { get; set; } = true;

        /// <summary>
        /// RadioMaster connection type: "UDP", "COM", or "TCP"
        /// UDP uses network port, COM uses serial port (e.g., COM3), TCP uses TCP network port (e.g., SITL)
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
        /// RadioMaster TCP host to connect to (e.g. "127.0.0.1" for ArduPilot SITL).
        /// Used when RadioMasterConnectionType is "TCP" (port = RadioMasterPort).
        /// </summary>
        public string RadioMasterTcpHost { get; set; } = "127.0.0.1";

        /// <summary>
        /// RadioMaster COM port baud rate.
        /// ELRS typically uses 420000 or 115200
        /// </summary>
        public int RadioMasterBaudRate { get; set; } = 420000;

        /// <summary>
        /// LTE/Tailscale MAVLink UDP port the ground station listens on.
        /// Default 14560 to avoid colliding with the RadioMaster default (14550).
        /// </summary>
        public int LteMavlinkPort { get; set; } = 14560;

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
        // Ground-side MAVLink Router (MAVProxy-style multiplexer)
        // ============================================================
        // The router opens both source links itself (LTE UDP + RC UDP/COM),
        // tracks per-link health from real packet flow, dedupes duplicates,
        // and exposes a single merged UDP endpoint Mission Planner connects
        // to (UDPCl to 127.0.0.1:<RouterLocalPort>). Failover is zero-gap
        // because both source links are read in parallel at all times.

        /// <summary>
        /// Enable the local MAVLink router. When on, the plugin owns both
        /// source links and Mission Planner should connect to the local
        /// loopback endpoint instead of LTE/RC directly.
        /// </summary>
        public bool RouterEnabled { get; set; } = true;

        /// <summary>Local UDP port the router serves the merged stream on.</summary>
        public int RouterLocalPort { get; set; } = 14600;

        /// <summary>Address the router binds for the local merged stream.</summary>
        public string RouterBindAddress { get; set; } = "127.0.0.1";

        /// <summary>
        /// Deduplicate identical packets that arrive on both links (recommended).
        /// Disable only for diagnostics — costs ~1.5x bandwidth to MP.
        /// </summary>
        public bool RouterDedupEnabled { get; set; } = true;

        /// <summary>
        /// Optional outbound endpoint for LTE link. When non-empty, router
        /// sends GCS-originated traffic to this host:port over UDP. Leave
        /// empty to use the same endpoint packets were received from.
        /// </summary>
        public string LteRemoteHost { get; set; } = "";

        /// <summary>Outbound UDP port for LTE link (0 = use last-rx port).</summary>
        public int LteRemotePort { get; set; } = 0;

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

        /// <summary>
        /// Speak Airbus-style altitude callouts in flight (gated by AudioAlerts).
        /// </summary>
        public bool AltitudeCallouts { get; set; } = true;

        // ============================================================
        // Motor Music Configuration
        // ============================================================

        /// <summary>Number of motor outputs the music bridge should use.</summary>
        public int MotorMusicMotorCount { get; set; } = 4;

        /// <summary>Lowest active ArduPilot motor output, in PWM-equivalent microseconds.</summary>
        public int MotorMusicMinOutputPwm { get; set; } = 1100;

        /// <summary>Highest active ArduPilot motor output, in PWM-equivalent microseconds.</summary>
        public int MotorMusicMaxOutputPwm { get; set; } = 1800;

        /// <summary>Semitone offset applied before notes are sent to the motors.</summary>
        public int MotorMusicTranspose { get; set; } = -24;

        /// <summary>Playback speed multiplier for MIDI scheduling.</summary>
        public double MotorMusicTempoScale { get; set; } = 1.0;

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
        // Cube and commanded via MAVLink DO_SET_SERVO. Edge Core is used only
        // as a fallback path to send the same Cube MAVLink commands.
        // Channel numbers correspond to ArduPilot servo output numbers
        // (e.g. 9 = SERVO9 = AUX1 on most Cube builds).
        // ============================================================

        // --- Modular payloads (drop servos, slider servos, relay/GPIO outputs) ---
        /// <summary>Maximum number of configurable payloads (panel + Settings cap).</summary>
        public const int MaxPayloads = 8;

        /// <summary>
        /// Configurable payload outputs rendered on the payload panel and edited in
        /// Settings → Payloads. Each is a drop servo, a slider servo, or a relay/GPIO
        /// output. See <see cref="PayloadControl"/>.
        /// </summary>
        [JsonProperty(ObjectCreationHandling = ObjectCreationHandling.Replace)]
        public List<PayloadControl> Payloads { get; set; } = DefaultPayloads();

        /// <summary>Payload controls start empty and are configured per aircraft.</summary>
        public static List<PayloadControl> DefaultPayloads() => new List<PayloadControl>();

        // Strap reels and the camera tilt servo are regular payload entries now
        // (PayloadKind.Reel / PayloadKind.CamTilt) — add them in Settings →
        // Payloads. PayloadControl.NewReel / NewCamTilt carry the standard
        // NOMAD defaults (reel out <1000 us / in >2000 us / stop 1500 us;
        // tilt 700 down / 1250 level / 1450 up — the camera arm is mechanically
        // offset, so level is NOT the standard 1500 us).

        // ============================================================
        // Joystick Configuration (Mission Planner DirectInput-based)
        // ============================================================
        // Two independent joystick assignments routed by NomadJoystickService:
        //   * Gimbal: stick deflection → pitch/roll rate, integrated locally
        //     into MAV_CMD_DO_MOUNT_CONTROL angle commands.
        //   * ZED tilt: stick deflection → PWM rate, integrated locally into
        //     the camera tilt servo PWM target (DO_SET_SERVO).
        // Axes are referenced by DirectInput state property name: X, Y, Z,
        // Rx, Ry, Rz, Slider1, Slider2.

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

        /// <summary>Enable the ZED tilt joystick channel.</summary>
        public bool JoystickZedEnabled { get; set; } = false;
        /// <summary>DirectInput device name. May be the same device as gimbal (different axes).</summary>
        public string JoystickZedDevice { get; set; } = "";
        /// <summary>Axis driving ZED tilt rate.</summary>
        public string JoystickZedTiltAxis { get; set; } = "Y";
        public bool JoystickZedTiltInvert { get; set; } = true;
        public float JoystickZedDeadzone { get; set; } = 0.08f;
        /// <summary>Max integrated PWM rate (microseconds per second) at full stick deflection.</summary>
        public float JoystickZedMaxRateUsPerSec { get; set; } = 400f;

        // --- Three-position switch action mapping ---
        // joystick.py encodes each 3-position RadioMaster switch (sw1, sw2, sw3)
        // as a pair of virtual Xbox 360 buttons — UP and DOWN positions press a
        // dedicated button, middle releases both. NomadJoystickService dispatches
        // a configurable action per slot. Valid action IDs:
        //   None, DropToggleP1, DropToggleP2, DropToggleP3,
        //   ReelInP1, ReelOutP1, ReelInP2, ReelOutP2, FireWaterPump
        // Drop toggles and FireWaterPump are edge-triggered (fire on switch flip
        // toward the position); Reel actions run while the switch is held off-
        // centre and stop when it returns to middle.
        /// <summary>
        /// DirectInput device that publishes the switch buttons (from joystick.py
        /// or any other source). Independent of the gimbal/ZED axis devices so
        /// payload switches keep working even when both axis channels are off.
        /// Leave blank to fall back to the gimbal device, then the ZED device.
        /// </summary>
        public string JoystickSwitchDevice  { get; set; } = "";

        public string JoystickSw1UpAction   { get; set; } = "DropToggleP1";
        public string JoystickSw1DownAction { get; set; } = "DropToggleP2";
        public string JoystickSw2UpAction   { get; set; } = "DropToggleP3";
        public string JoystickSw2DownAction { get; set; } = "ReelInP1";
        public string JoystickSw3UpAction   { get; set; } = "ReelInP2";
        public string JoystickSw3DownAction { get; set; } = "FireWaterPump";

        /// <summary>
        /// Enable the dedicated kill-switch pushbutton (button index 6 on the
        /// virtual gamepad — joystick.py maps the radio kill switch to XInput
        /// BACK). When pressed, the plugin commands LAND mode and forces
        /// LAND_SPEED / WPNAV_SPEED_DN to <see cref="JoystickKillLandSpeedCmS"/>
        /// so the descent meets the CONOPS §4.5 ≥2 m/s requirement.
        /// </summary>
        public bool JoystickKillSwitchEnabled { get; set; } = true;

        /// <summary>
        /// Descent speed (cm/s) the kill switch forces before engaging LAND.
        /// Default 250 = 2.5 m/s, comfortably above the 2 m/s CONOPS floor.
        /// </summary>
        public int JoystickKillLandSpeedCmS { get; set; } = 250;

        /// <summary>
        /// When true, the joystick service auto-picks the first available
        /// DirectInput device for any role whose configured device name is
        /// blank or not currently enumerated, and re-checks periodically so
        /// hot-plugged controllers (e.g. the vgamepad created by joystick.py)
        /// get picked up without a settings round-trip. Default true.
        /// </summary>
        public bool JoystickAutoSelectDevice { get; set; } = true;

        // --- Serial → virtual gamepad bridge (jotystick.py) ---
        /// <summary>Auto-launch jotystick.py on plugin start so a serial-attached MCU appears as an Xbox 360 controller.</summary>
        public bool SerialJoystickEnabled { get; set; } = false;
        /// <summary>Serial port the MCU is on (e.g. COM10).</summary>
        public string SerialJoystickPort { get; set; } = "COM10";
        /// <summary>Baud rate.</summary>
        public int SerialJoystickBaud { get; set; } = 115200;
        /// <summary>Python executable to use. Leave blank to use "python" from PATH.</summary>
        public string SerialJoystickPython { get; set; } = "python";
        /// <summary>Absolute path to jotystick.py. Leave blank to auto-resolve relative to the plugin DLL.</summary>
        public string SerialJoystickScriptPath { get; set; } = "";

    }
}
