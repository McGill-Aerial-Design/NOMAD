// ============================================================
// NOMAD Joystick Service
// ============================================================
// Drives the Caddx gimbal and the ZED tilt servo from up to two
// physical DirectInput joysticks. Reuses Mission Planner's
// MissionPlanner.Joystick.JoystickBase device wrapper so we don't
// duplicate device enumeration / state polling, but DELIBERATELY
// never calls JoystickBase.start() — MP's start() spawns the RC
// override loop that fights the autopilot for control. We only
// acquire the device and poll GetCurrentState() ourselves.
//
// Routing:
//   * Gimbal: stick (X,Y) → integrated pitch/roll target via
//     GimbalController.ApplyStick → MAV_CMD_DO_MOUNT_CONTROL.
//   * ZED tilt: stick axis → integrated PWM target via
//     CubeOutputController.SendServoPwmAsync (tryOnly while drag).
// ============================================================

using System;
using System.Collections.Generic;
using System.Reflection;
using System.Windows.Forms;
using MissionPlanner;
using JoystickBase = MissionPlanner.Joystick.JoystickBase;
using IMyJoystickState = MissionPlanner.Joystick.IMyJoystickState;
using Timer = System.Windows.Forms.Timer;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Polls one or two DirectInput devices at 20 Hz and routes their stick
    /// values to the gimbal target integrator and the ZED tilt servo.
    /// </summary>
    public sealed class NomadJoystickService : IDisposable
    {
        private const int POLL_HZ = 20;

        private NOMADConfig _config;
        private Timer _timer;

        // Per-role device state — separate JoystickBase instances even if
        // both roles point at the same physical device, because each
        // AcquireJoystick call internally locks an opened device handle.
        // We dedupe at acquire-time by sharing one base when device names match.
        private JoystickBase _gimbalJoy;
        private JoystickBase _zedJoy;

        // Cached property accessors on IMyJoystickState (X, Y, …) so we read
        // by axis-name string from config without per-tick reflection cost.
        private static readonly Dictionary<string, PropertyInfo> AxisProps = BuildAxisMap();

        // Current ZED tilt PWM target (μs). Initialised from PayloadControlPanel
        // so a session that has already moved the slider doesn't snap on start.
        private float _zedTiltUs;

        public NomadJoystickService(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
        }

        // ============================================================
        // Lifecycle
        // ============================================================

        public void Start()
        {
            Stop(); // idempotent

            try { AcquireDevices(); }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD Joystick: device acquire failed — {ex.Message}");
            }

            // Seed tilt integrator from the live shared value so the first stick
            // motion is relative to where the slider/servo already is.
            _zedTiltUs = Math.Max(_config.CameraTiltPwmMin,
                          Math.Min(_config.CameraTiltPwmMax, PayloadControlPanel.LastTiltPulseUs));

            // The Caddx mount only honors DO_MOUNT_CONTROL absolute-angle commands
            // when it's in MAVLink targeting mode. Without this ping the mount may
            // be sitting in RC targeting from a previous session and silently drop
            // every angle command we send — which feels like the joystick is
            // controlling a rate instead of a position, since the gimbal won't
            // hold the angle we asked for.
            if (_config.JoystickGimbalEnabled)
            {
                GimbalController.SetMode(GimbalController.MountMode.MavlinkTargeting);
            }

            // Seed the centralized rate from persisted config so the floating
            // window's slider and our integrator start with the same value.
            GimbalController.MaxRateDegSec = _config.JoystickGimbalMaxRateDegSec;

            _timer = new Timer { Interval = 1000 / POLL_HZ };
            _timer.Tick += OnTick;
            _timer.Start();

            Console.WriteLine($"NOMAD Joystick: started (gimbal={_config.JoystickGimbalEnabled} dev='{_config.JoystickGimbalDevice}', " +
                              $"zed={_config.JoystickZedEnabled} dev='{_config.JoystickZedDevice}')");
        }

        public void Stop()
        {
            try { _timer?.Stop(); _timer?.Dispose(); } catch { }
            _timer = null;

            ReleaseJoy(ref _gimbalJoy);
            ReleaseJoy(ref _zedJoy);
        }

        public void RestartWithConfig()
        {
            Stop();
            if (_config.JoystickGimbalEnabled || _config.JoystickZedEnabled)
                Start();
        }

        public void UpdateConfig(NOMADConfig config)
        {
            if (config == null) return;
            _config = config;
            RestartWithConfig();
        }

        public void Dispose() => Stop();

        // ============================================================
        // Device acquisition
        // ============================================================

        public static IList<string> EnumerateDevices()
        {
            try { return JoystickBase.getDevices(); }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD Joystick: enumerate failed — {ex.Message}");
                return new List<string>();
            }
        }

        public static IReadOnlyList<string> AxisNames => new[]
        {
            "X", "Y", "Z", "Rx", "Ry", "Rz", "Slider1", "Slider2"
        };

        private void AcquireDevices()
        {
            // Share one JoystickBase across both roles if they target the
            // same physical device — DirectInput permits multiple readers
            // of one acquired device cheaper than two separate acquires.
            JoystickBase shared = null;

            if (_config.JoystickGimbalEnabled && !string.IsNullOrWhiteSpace(_config.JoystickGimbalDevice))
            {
                _gimbalJoy = CreateAndAcquire(_config.JoystickGimbalDevice);
                shared = _gimbalJoy;
            }

            if (_config.JoystickZedEnabled && !string.IsNullOrWhiteSpace(_config.JoystickZedDevice))
            {
                if (shared != null && _config.JoystickZedDevice == _config.JoystickGimbalDevice)
                    _zedJoy = shared;
                else
                    _zedJoy = CreateAndAcquire(_config.JoystickZedDevice);
            }
        }

        private static JoystickBase CreateAndAcquire(string deviceName)
        {
            // Pass a Func that returns the active MAVLink interface; MP uses
            // this in its own start() path. We never call start(), so the
            // delegate is effectively unused — but the API requires it.
            JoystickBase jb = JoystickBase.Create(() => MainV2.comPort);
            try
            {
                if (!jb.AcquireJoystick(deviceName))
                {
                    Console.WriteLine($"NOMAD Joystick: AcquireJoystick('{deviceName}') returned false");
                    jb.Dispose();
                    return null;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD Joystick: acquire '{deviceName}' failed — {ex.Message}");
                try { jb.Dispose(); } catch { }
                return null;
            }
            return jb;
        }

        private static void ReleaseJoy(ref JoystickBase joy)
        {
            if (joy == null) return;
            try { joy.UnAcquireJoyStick(); } catch { }
            try { joy.Dispose(); } catch { }
            joy = null;
        }

        // ============================================================
        // Polling loop (UI thread — fine since reads are non-blocking)
        // ============================================================

        private DateTime _lastTick = DateTime.UtcNow;

        private void OnTick(object sender, EventArgs e)
        {
            var now = DateTime.UtcNow;
            float dt = (float)(now - _lastTick).TotalSeconds;
            _lastTick = now;
            if (dt <= 0f || dt > 0.5f) dt = 1f / POLL_HZ;

            try { DriveGimbal(dt); }
            catch (Exception ex) { Console.WriteLine($"NOMAD Joystick gimbal: {ex.Message}"); }

            try { DriveZed(dt); }
            catch (Exception ex) { Console.WriteLine($"NOMAD Joystick zed: {ex.Message}"); }

            // Payload switch buttons emitted by joystick.py — read from whichever
            // device is acquired (gimbal preferred, then zed). Runs every tick so
            // edge detection doesn't depend on stick motion or DriveGimbal early-returning.
            try
            {
                var btnDev = _gimbalJoy ?? _zedJoy;
                if (btnDev != null)
                {
                    var st = SafeGetState(btnDev);
                    if (st != null) DrivePayloadButtons(st);
                }
            }
            catch (Exception ex) { Console.WriteLine($"NOMAD Joystick buttons: {ex.Message}"); }
        }

        private void DriveGimbal(float dt)
        {
            if (_gimbalJoy == null || !_config.JoystickGimbalEnabled) return;
            var st = SafeGetState(_gimbalJoy);
            if (st == null) return;

            float roll  = ReadAxisNorm(st, _config.JoystickGimbalRollAxis,  _config.JoystickGimbalRollInvert,  _config.JoystickGimbalDeadzone);
            float pitch = ReadAxisNorm(st, _config.JoystickGimbalPitchAxis, _config.JoystickGimbalPitchInvert, _config.JoystickGimbalDeadzone);

            if (roll == 0f && pitch == 0f) return; // no motion → don't spam mount

            // Use the centralized rate so the floating GimbalJoystickWindow's slider
            // and the settings dialog all agree on one value. No code duplication.
            GimbalController.ApplyStick(roll, pitch, dt, send: true);
        }

        private void DriveZed(float dt)
        {
            if (_zedJoy == null || !_config.JoystickZedEnabled) return;
            var st = SafeGetState(_zedJoy);
            if (st == null) return;

            float v = ReadAxisNorm(st, _config.JoystickZedTiltAxis, _config.JoystickZedTiltInvert, _config.JoystickZedDeadzone);
            if (v == 0f) return;

            int pwmMin = _config.CameraTiltPwmMin;
            int pwmMax = _config.CameraTiltPwmMax;
            float target = _zedTiltUs + v * _config.JoystickZedMaxRateUsPerSec * dt;
            if (target < pwmMin) target = pwmMin;
            if (target > pwmMax) target = pwmMax;

            // Skip the send entirely if the integrated change is sub-microsecond — DO_SET_SERVO is a 16-bit value.
            int newUs = (int)Math.Round(target);
            int oldUs = (int)Math.Round(_zedTiltUs);
            _zedTiltUs = target;
            if (newUs == oldUs) return;

            int channel = _config.CameraTiltChannel;
            if (channel <= 0) return;

            _ = CubeOutputController.SendServoPwmAsync(channel, newUs, tryOnly: true);
            PayloadControlPanel.SetExternalTiltPulse(newUs);
        }

        private static IMyJoystickState SafeGetState(JoystickBase joy)
        {
            try { return joy.GetCurrentState(); }
            catch { return null; }
        }

        // ============================================================
        // Axis decoding
        // ============================================================

        private static Dictionary<string, PropertyInfo> BuildAxisMap()
        {
            var dict = new Dictionary<string, PropertyInfo>(StringComparer.OrdinalIgnoreCase);
            var t = typeof(IMyJoystickState);
            foreach (var name in new[] { "X", "Y", "Z", "Rx", "Ry", "Rz" })
            {
                var p = t.GetProperty(name);
                if (p != null) dict[name] = p;
            }
            return dict;
        }

        /// <summary>
        /// Read an axis by config name and return it normalized to [-1, 1].
        /// DirectInput axes report [0, 65535] with 32767/8 as centre; sliders are
        /// the same range and read through GetSlider(). Returns 0 inside the deadzone.
        /// </summary>
        private static float ReadAxisNorm(IMyJoystickState st, string axisName, bool invert, float deadzone)
        {
            if (string.IsNullOrWhiteSpace(axisName) || st == null) return 0f;

            int raw;
            if (axisName.Equals("Slider1", StringComparison.OrdinalIgnoreCase))
            {
                var s = st.GetSlider(); if (s == null || s.Length < 1) return 0f; raw = s[0];
            }
            else if (axisName.Equals("Slider2", StringComparison.OrdinalIgnoreCase))
            {
                var s = st.GetSlider(); if (s == null || s.Length < 2) return 0f; raw = s[1];
            }
            else if (AxisProps.TryGetValue(axisName, out var prop))
            {
                raw = (int)prop.GetValue(st, null);
            }
            else
            {
                return 0f;
            }

            // 0..65535 → -1..1 with centre at 32767.5
            float norm = (raw - 32767.5f) / 32767.5f;
            if (norm > 1f) norm = 1f; else if (norm < -1f) norm = -1f;
            if (Math.Abs(norm) < deadzone) return 0f;

            // Re-scale post-deadzone so the working range covers the full [-1,1].
            float sign = norm < 0 ? -1f : 1f;
            float scaled = (Math.Abs(norm) - deadzone) / (1f - deadzone);
            return invert ? -sign * scaled : sign * scaled;
        }

        // ============================================================
        // Payload switch buttons
        // ============================================================
        // joystick.py maps the three 3-position switches on the RadioMaster onto
        // 6 virtual Xbox360 buttons (each switch position = one button). The
        // mapping is fixed in the python:
        //   button 0 (A)  = sw1 pos 1  → switch #1 → toggle drop/retract Payload 1
        //   button 1 (B)  = sw1 pos 2  → switch #2 → toggle drop/retract Payload 2
        //   button 2 (X)  = sw2 pos 1  → switch #3 → toggle drop/retract Payload 3
        //   button 3 (Y)  = sw2 pos 2  → switch #4 → reel-in Payload 1 (hold)
        //   button 4 (LB) = sw3 pos 1  → switch #5 → reel-in Payload 2 (hold)
        //   button 5 (RB) = sw3 pos 2  → switch #6 → fire water pump (edge)
        //
        // Drop/spray are edge-triggered (on press), reels are level-triggered
        // (run while held, neutral PWM on release).

        private const int BTN_DROP_P1   = 0;
        private const int BTN_DROP_P2   = 1;
        private const int BTN_DROP_P3   = 2;
        private const int BTN_REEL_P1   = 3;
        private const int BTN_REEL_P2   = 4;
        private const int BTN_SPRAY     = 5;

        private bool[] _prevButtons = new bool[8];
        private bool[] _payloadDropped = new bool[3]; // tracks toggle state for P1/P2/P3
        private readonly bool[] _reelHeld = new bool[2];

        private void DrivePayloadButtons(IMyJoystickState st)
        {
            bool[] buttons;
            try { buttons = st.GetButtons(); }
            catch { return; }
            if (buttons == null) return;

            bool B(int i) => i < buttons.Length && buttons[i];

            // Edge-triggered: drop / retract toggles for P1..P3
            for (int i = 0; i < 3; i++)
            {
                bool now = B(i);
                bool prev = i < _prevButtons.Length && _prevButtons[i];
                if (now && !prev)
                {
                    if (_payloadDropped[i]) { PayloadActions.Retract(_config, i + 1); _payloadDropped[i] = false; }
                    else                    { PayloadActions.Drop(_config, i + 1);    _payloadDropped[i] = true;  }
                }
            }

            // Level-triggered: reel P1 / P2 while held
            HandleReelHold(0, B(BTN_REEL_P1));
            HandleReelHold(1, B(BTN_REEL_P2));

            // Edge-triggered: water pump
            bool sprayNow = B(BTN_SPRAY);
            bool sprayPrev = BTN_SPRAY < _prevButtons.Length && _prevButtons[BTN_SPRAY];
            if (sprayNow && !sprayPrev) PayloadActions.FireWater(_config);

            // Save for edge detection next tick
            int n = Math.Min(buttons.Length, _prevButtons.Length);
            for (int i = 0; i < n; i++) _prevButtons[i] = buttons[i];
        }

        private void HandleReelHold(int reelIdx, bool held)
        {
            if (held && !_reelHeld[reelIdx])
            {
                _reelHeld[reelIdx] = true;
                PayloadActions.ReelStart(_config, reelIdx);
            }
            else if (!held && _reelHeld[reelIdx])
            {
                _reelHeld[reelIdx] = false;
                PayloadActions.ReelStop(_config, reelIdx);
            }
        }
    }
}
