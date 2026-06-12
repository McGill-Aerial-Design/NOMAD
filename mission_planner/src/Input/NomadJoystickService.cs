// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
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
using System.Threading.Tasks;
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
    public sealed partial class NomadJoystickService : IDisposable
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
        // Dedicated button-source device. May alias _gimbalJoy / _zedJoy when
        // the configured switch device matches one of the axis devices, so we
        // only Acquire() once per physical handle.
        private JoystickBase _switchJoy;
        private bool _switchJoyOwned; // true if we created it (vs. aliased gimbal/zed)

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
                Log.Debug($"device acquire failed — {ex.Message}");
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

            Log.Debug($"started (gimbal={_config.JoystickGimbalEnabled} dev='{_config.JoystickGimbalDevice}', " +
                              $"zed={_config.JoystickZedEnabled} dev='{_config.JoystickZedDevice}')");
        }

        public void Stop()
        {
            try { _timer?.Stop(); _timer?.Dispose(); } catch { }
            _timer = null;

            // Release the dedicated switch device first if we own it; otherwise
            // just drop the alias so ReleaseJoy on gimbal/zed below frees it.
            if (_switchJoyOwned) ReleaseJoy(ref _switchJoy);
            else _switchJoy = null;
            _switchJoyOwned = false;

            ReleaseJoy(ref _gimbalJoy);
            ReleaseJoy(ref _zedJoy);
        }

        public void RestartWithConfig()
        {
            Stop();
            if (NeedsToRun()) Start();
        }

        /// <summary>
        /// True when the service has something to do — either an axis channel
        /// is enabled, or at least one switch slot is mapped to a real action
        /// (so payload switches keep working even with no gimbal/ZED routing).
        /// </summary>
        public bool NeedsToRun()
        {
            if (_config.JoystickGimbalEnabled || _config.JoystickZedEnabled) return true;
            if (_config.JoystickKillSwitchEnabled) return true;
            return AnySwitchMapped();
        }

        private bool AnySwitchMapped()
        {
            return GetSlotAction(0) != SwitchAction.None
                || GetSlotAction(1) != SwitchAction.None
                || GetSlotAction(2) != SwitchAction.None
                || GetSlotAction(3) != SwitchAction.None
                || GetSlotAction(4) != SwitchAction.None
                || GetSlotAction(5) != SwitchAction.None;
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
                Log.Error($"enumerate failed — {ex.Message}");
                return new List<string>();
            }
        }

        public static IReadOnlyList<string> AxisNames => new[]
        {
            "X", "Y", "Z", "Rx", "Ry", "Rz", "Slider1", "Slider2"
        };

        /// <summary>
        /// Resolve a configured device name to one actually present on the system.
        /// When auto-select is on and the configured value is blank or unmatched,
        /// returns the first available device so a freshly hot-plugged controller
        /// (typically the vgamepad spawned by joystick.py) gets picked up
        /// automatically.
        /// </summary>
        private string ResolveDeviceName(string configured, IList<string> available)
        {
            if (!string.IsNullOrWhiteSpace(configured))
            {
                foreach (var d in available)
                    if (string.Equals(d, configured, StringComparison.OrdinalIgnoreCase))
                        return d;
            }
            if (_config.JoystickAutoSelectDevice && available != null && available.Count > 0)
                return available[0];
            return null;
        }

        private void AcquireDevices()
        {
            var available = EnumerateDevices();

            // Share one JoystickBase across roles if they target the same
            // physical device — DirectInput permits multiple readers of one
            // acquired device cheaper than two separate acquires.
            // Skip roles whose handle is already live so re-acquire calls don't
            // leak handles when only one role needs catching up.
            if (_config.JoystickGimbalEnabled && _gimbalJoy == null)
            {
                var dev = ResolveDeviceName(_config.JoystickGimbalDevice, available);
                if (dev != null) _gimbalJoy = CreateAndAcquire(dev);
            }

            if (_config.JoystickZedEnabled && _zedJoy == null)
            {
                var dev = ResolveDeviceName(_config.JoystickZedDevice, available);
                if (dev != null)
                {
                    if (_gimbalJoy != null && string.Equals(dev, _config.JoystickGimbalDevice, StringComparison.OrdinalIgnoreCase))
                        _zedJoy = _gimbalJoy;
                    else
                        _zedJoy = CreateAndAcquire(dev);
                }
            }

            // Resolve the button-source device. Explicit config wins; otherwise
            // fall back to whichever axis device is already acquired so users
            // with a single virtual gamepad don't need to configure twice.
            if (_switchJoy != null) return;

            string switchDev = ResolveDeviceName(_config.JoystickSwitchDevice, available);
            if (switchDev == null)
            {
                _switchJoy = _gimbalJoy ?? _zedJoy;
                _switchJoyOwned = false;
            }
            else if (_gimbalJoy != null && string.Equals(switchDev, _config.JoystickGimbalDevice, StringComparison.OrdinalIgnoreCase))
            {
                _switchJoy = _gimbalJoy;
                _switchJoyOwned = false;
            }
            else if (_zedJoy != null && string.Equals(switchDev, _config.JoystickZedDevice, StringComparison.OrdinalIgnoreCase))
            {
                _switchJoy = _zedJoy;
                _switchJoyOwned = false;
            }
            else if (_gimbalJoy == null && _zedJoy == null && _config.JoystickAutoSelectDevice)
            {
                // Auto-select with no axis devices acquired — open switchDev directly.
                _switchJoy = CreateAndAcquire(switchDev);
                _switchJoyOwned = _switchJoy != null;
            }
            else
            {
                _switchJoy = CreateAndAcquire(switchDev);
                _switchJoyOwned = _switchJoy != null;
            }

            Log.Debug($"switch device='{switchDev}' acquired={(_switchJoy != null)} owned={_switchJoyOwned}");
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
                    Log.Debug($"AcquireJoystick('{deviceName}') returned false");
                    jb.Dispose();
                    return null;
                }
            }
            catch (Exception ex)
            {
                Log.Error($"acquire '{deviceName}' failed — {ex.Message}");
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
        private DateTime _lastReacquireAttempt = DateTime.MinValue;
        private const double REACQUIRE_INTERVAL_SEC = 3.0;

        private void OnTick(object sender, EventArgs e)
        {
            var now = DateTime.UtcNow;
            float dt = (float)(now - _lastTick).TotalSeconds;
            _lastTick = now;
            if (dt <= 0f || dt > 0.5f) dt = 1f / POLL_HZ;

            // Hot-plug recovery: if we're missing a device we expect, retry
            // device enumeration every few seconds so a controller (or the
            // joystick.py-spawned vgamepad) gets picked up without restarting
            // the plugin. Skip when nothing needs a device.
            bool needSwitches = AnySwitchMapped() || _config.JoystickKillSwitchEnabled;
            bool missingAxis = (_config.JoystickGimbalEnabled && _gimbalJoy == null)
                            || (_config.JoystickZedEnabled    && _zedJoy    == null);
            bool missingSwitch = needSwitches && _switchJoy == null;
            if ((missingAxis || missingSwitch) && (now - _lastReacquireAttempt).TotalSeconds >= REACQUIRE_INTERVAL_SEC)
            {
                _lastReacquireAttempt = now;
                try { AcquireDevices(); }
                catch (Exception ex) { Log.Debug($"re-acquire failed — {ex.Message}"); }
            }

            try { DriveGimbal(dt); }
            catch (Exception ex) { Log.Error($"gimbal: {ex.Message}"); }

            try { DriveZed(dt); }
            catch (Exception ex) { Log.Error($"zed: {ex.Message}"); }

            // Payload switch buttons emitted by joystick.py — read from whichever
            // device is acquired (gimbal preferred, then zed). Runs every tick so
            // edge detection doesn't depend on stick motion or DriveGimbal early-returning.
            try
            {
                var btnDev = _switchJoy ?? _gimbalJoy ?? _zedJoy;
                if (btnDev != null)
                {
                    var st = SafeGetState(btnDev);
                    if (st != null) DrivePayloadButtons(st);
                }
            }
            catch (Exception ex) { Log.Error($"buttons: {ex.Message}"); }
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

        // Payload switch-button handling lives in NomadJoystickService.Switches.cs.
    }
}
