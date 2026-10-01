// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Configuration
// ============================================================
// Handles plugin configuration persistence.
// Stored in Mission Planner's config directory.
// Supports direct video, MAVLink, payload, geofence, and log-analysis features.
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
        /// Active NOMAD config profile name. Written by the profile loader
        /// (scripts/profile.py) so the plugin can show which profile is live.
        /// </summary>
        public string ActiveProfile { get; set; } = "dev";

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
        /// This displays the configured video feed as a background overlay on the HUD.
        /// </summary>
        public bool AutoStartHudVideo { get; set; } = true;

        // ============================================================
        // MAVLink Dual Link Configuration
        // ============================================================

        /// <summary>
        /// Enable Mission Planner's standalone router status client.
        /// </summary>
        public bool DualLinkEnabled { get; set; } = true;

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
        // Drone Geometry Configuration
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

        /// <summary>Camera field of view in degrees for future visualization.</summary>
        public float SlamCameraFovDeg { get; set; } = 60.0f;

        /// <summary>Local map radius in meters for future visualization.</summary>
        public float SlamMapRadiusM { get; set; } = 3.0f;

        // ============================================================
        // Servo Configuration (ArduPilot AUX outputs via MAVLink)
        // All payloads, reels, water pump and camera tilt are driven through
        // standard ArduPilot servo/relay outputs on any ArduPilot flight
        // controller via MAVLink DO_SET_SERVO / DO_SET_RELAY.
        // Commands are sent through the NOMAD C++ core, which owns the
        // MAVLink transport.
        // Channel numbers are ArduPilot servo output numbers (SERVOn_FUNCTION)
        // and are mapped per-board in the autopilot parameters, not hardcoded
        // to a specific flight controller.
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
        //     into typed NOMAD runtime angle-target requests.
        //   * Camera tilt: stick deflection → PWM rate, integrated locally into
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

        /// <summary>Enable the camera tilt joystick channel.</summary>
        public bool JoystickCameraTiltEnabled { get; set; } = false;
        /// <summary>DirectInput device name. May be the same device as gimbal (different axes).</summary>
        public string JoystickCameraTiltDevice { get; set; } = "";
        /// <summary>Axis driving camera tilt rate.</summary>
        public string JoystickCameraTiltAxis { get; set; } = "Y";
        public bool JoystickCameraTiltInvert { get; set; } = true;
        public float JoystickCameraTiltDeadzone { get; set; } = 0.08f;
        /// <summary>Max integrated PWM rate (microseconds per second) at full stick deflection.</summary>
        public float JoystickCameraTiltMaxRateUsPerSec { get; set; } = 400f;

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
        /// or any other source). Independent of the gimbal/camera-tilt axis devices so
        /// payload switches keep working even when both axis channels are off.
        /// Leave blank to fall back to the gimbal device, then the tilt device.
        /// </summary>
        public string JoystickSwitchDevice  { get; set; } = "";

        public string JoystickSw1UpAction   { get; set; } = "DropToggleP1";
        public string JoystickSw1DownAction { get; set; } = "DropToggleP2";
        public string JoystickSw2UpAction   { get; set; } = "DropToggleP3";
        public string JoystickSw2DownAction { get; set; } = "ReelInP1";
        public string JoystickSw3UpAction   { get; set; } = "ReelInP2";
        public string JoystickSw3DownAction { get; set; } = "FireWaterPump";

        /// <summary>
        /// Monitor button index 6 for termination requests. Aircraft-side
        /// termination is unavailable; a press reports that failure visibly.
        /// </summary>
        public bool JoystickKillSwitchEnabled { get; set; } = true;

        /// <summary>
        /// When true, the joystick service auto-picks the first available
        /// DirectInput device for any role whose configured device name is
        /// blank or not currently enumerated, and re-checks periodically so
        /// hot-plugged controllers (e.g. the vgamepad created by joystick.py)
        /// get picked up without a settings round-trip. Default true.
        /// </summary>
        public bool JoystickAutoSelectDevice { get; set; } = true;

        // --- Serial → virtual gamepad bridge (jotystick.py) ---
        /// <summary>Auto-launch jotystick.py on plugin start so a serial-attached MCU
        /// appears as an Xbox 360 controller.</summary>
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
