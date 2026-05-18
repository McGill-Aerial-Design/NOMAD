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
        }

        private void DriveGimbal(float dt)
        {
            if (_gimbalJoy == null || !_config.JoystickGimbalEnabled) return;
            var st = SafeGetState(_gimbalJoy);
            if (st == null) return;

            float roll  = ReadAxisNorm(st, _config.JoystickGimbalRollAxis,  _config.JoystickGimbalRollInvert,  _config.JoystickGimbalDeadzone);
            float pitch = ReadAxisNorm(st, _config.JoystickGimbalPitchAxis, _config.JoystickGimbalPitchInvert, _config.JoystickGimbalDeadzone);

            if (roll == 0f && pitch == 0f) return; // no motion → don't spam mount

            GimbalController.ApplyStick(roll, pitch, _config.JoystickGimbalMaxRateDegSec, dt, send: true);
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
    }
}
