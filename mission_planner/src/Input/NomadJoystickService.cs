// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Joystick Service
// ============================================================
// Drives the camera gimbal and configured position input from up to two
// physical DirectInput joysticks. Reuses Mission Planner's
// MissionPlanner.Joystick.JoystickBase device wrapper so we don't
// duplicate device enumeration / state polling, but DELIBERATELY
// never calls JoystickBase.start() — MP's start() spawns the RC
// override loop that fights the autopilot for control. We only
// acquire the device and poll GetCurrentState() ourselves.
//
// The gimbal receives input targets; configured actuator axes/buttons send semantic
// IDs and normalized values to nomad-runtime. This client owns no actuator policy.

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
    /// values to the gimbal target integrator and the configured position input.
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
        private JoystickBase _positionJoy;
        // Dedicated button-source device. May alias _gimbalJoy / _positionJoy when
        // the configured switch device matches one of the axis devices, so we
        // only Acquire() once per physical handle.
        private JoystickBase _switchJoy;
        private bool _switchJoyOwned; // true if we created it (vs. aliased gimbal/position)

        // Cached property accessors on IMyJoystickState (X, Y, …) so we read
        // by axis-name string from config without per-tick reflection cost.
        private static readonly Dictionary<string, PropertyInfo> AxisProps = BuildAxisMap();

        // Position input comes directly from a configured USB HID axis.



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
            _ = OutputController.GetActuatorsAsync();

            try
            {
                AcquireDevices();
            }
            catch (Exception ex)
            {
                Log.Debug($"device acquire failed — {ex.Message}");
            }

            // The mount only honors absolute-angle requests in MAVLink targeting
            // mode. Without this ping the mount may
            // be sitting in RC targeting from a previous session and silently drop
            // every angle command we send — which feels like the joystick is
            // controlling a rate instead of a position, since the gimbal won't
            // hold the angle we asked for.
            if (_config.JoystickGimbalEnabled)
            {
                GimbalController.SetMode(MountMode.MavlinkTargeting);
            }

            // Seed the centralized rate from persisted config so the floating
            // window's slider and our integrator start with the same value.
            GimbalController.MaxRateDegSec = _config.JoystickGimbalMaxRateDegSec;

            _timer = new Timer { Interval = 1000 / POLL_HZ };
            _timer.Tick += OnTick;
            _timer.Start();

            Log.Debug($"started (gimbal={_config.JoystickGimbalEnabled} dev='{_config.JoystickGimbalDevice}', " +
                              $"position={_config.JoystickPositionEnabled} dev='{_config.JoystickPositionDevice}')");
        }

        public void Stop()
        {
            ResetActuatorInput();
            if (_lastPositionInput.HasValue && !string.IsNullOrEmpty(_config.JoystickPositionActuatorId))
            { SendHid(_config.JoystickPositionActuatorId, "safe", 6); }
            _lastPositionInput = null;
            try
            {
                _timer?.Stop(); _timer?.Dispose();
            }
            catch
            {

            }
            _timer = null;

            // Release the dedicated switch device first if we own it; otherwise
            // just drop the alias so ReleaseJoy on gimbal/position below frees it.
            if (_switchJoyOwned) ReleaseJoy(ref _switchJoy);
            else _switchJoy = null;
            _switchJoyOwned = false;

            ReleaseJoy(ref _gimbalJoy);
            ReleaseJoy(ref _positionJoy);
        }

        public void RestartWithConfig()
        {
            Stop();
            if (NeedsToRun()) Start();
        }

        /// <summary>
        /// True when the service has something to do — either an axis channel
        /// is enabled, or at least one switch slot is mapped to a real action
        /// (so configured buttons keep working even with no gimbal/position routing).
        /// </summary>
        public bool NeedsToRun()
        {
            if (_config.JoystickGimbalEnabled || _config.JoystickPositionEnabled) return true;
            if (_config.JoystickKillSwitchEnabled) return true;
            return AnySwitchMapped();
        }

        private bool AnySwitchMapped()
        {
            return GetSlotAction(0) != "None"
                || GetSlotAction(1) != "None"
                || GetSlotAction(2) != "None"
                || GetSlotAction(3) != "None"
                || GetSlotAction(4) != "None"
                || GetSlotAction(5) != "None";
        }

        public void UpdateConfig(NOMADConfig config)
        {
            if (config == null) return;
            Stop();
            _config = config;
            if (NeedsToRun())
            {
                Start();
            }
        }

        public void Dispose() => Stop();

        // ============================================================
        // Device acquisition
        // ============================================================

        public static IList<string> EnumerateDevices()
        {
            try
            {
                return JoystickBase.getDevices();
            }
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

        // Only re-acquire the explicitly selected USB HID device.
        internal static string ResolveDeviceName(string configured, IList<string> available)
        {
            if (string.IsNullOrWhiteSpace(configured) || available == null)
            {
                return null;
            }
            foreach (var name in available)
            {
                if (string.Equals(name, configured, StringComparison.OrdinalIgnoreCase))
                {
                    return name;
                }
            }
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

            if (_config.JoystickPositionEnabled && _positionJoy == null)
            {
                var dev = ResolveDeviceName(_config.JoystickPositionDevice, available);
                if (dev != null)
                {
                    if (_gimbalJoy != null && string.Equals(dev, _config.JoystickGimbalDevice, StringComparison.OrdinalIgnoreCase))
                        _positionJoy = _gimbalJoy;
                    else
                        _positionJoy = CreateAndAcquire(dev);
                }
            }

            // A missing selected switch device stays unavailable; never substitute another input source.
            if (_switchJoy != null)
            {
                return;
            }
            string switchDev = ResolveDeviceName(_config.JoystickSwitchDevice, available);
            if (switchDev == null)
            {
                return;
            }
            if (_gimbalJoy != null && string.Equals(switchDev, _config.JoystickGimbalDevice, StringComparison.OrdinalIgnoreCase))
            {
                _switchJoy = _gimbalJoy;
                _switchJoyOwned = false;
            }
            else if (_positionJoy != null && string.Equals(switchDev, _config.JoystickPositionDevice, StringComparison.OrdinalIgnoreCase))
            {
                _switchJoy = _positionJoy;
                _switchJoyOwned = false;
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
                try
                {
                    jb.Dispose();
                }
                catch
                {

                }
                return null;
            }
            return jb;
        }

        private static void ReleaseJoy(ref JoystickBase joy)
        {
            if (joy == null) return;
            try
            {
                joy.UnAcquireJoyStick();
            }
            catch
            {

            }
            try
            {
                joy.Dispose();
            }
            catch
            {

            }
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
            // selected USB HID device) gets picked up without restarting
            // the plugin. Skip when nothing needs a device.
            bool needSwitches = AnySwitchMapped() || _config.JoystickKillSwitchEnabled;
            bool missingAxis = (_config.JoystickGimbalEnabled && _gimbalJoy == null)
                            || (_config.JoystickPositionEnabled    && _positionJoy    == null);
            bool missingSwitch = needSwitches && _switchJoy == null;
            if ((missingAxis || missingSwitch) && (now - _lastReacquireAttempt).TotalSeconds >= REACQUIRE_INTERVAL_SEC)
            {
                _lastReacquireAttempt = now;
                try
                {
                    AcquireDevices();
                }
                catch (Exception ex) { Log.Debug($"re-acquire failed — {ex.Message}"); }
            }

            try
            {
                DriveGimbal(dt);
            }
            catch (Exception ex) { Log.Error($"gimbal: {ex.Message}"); }

            try
            {
                DrivePositionInput(dt);
            }
            catch (Exception ex) { Log.Error($"position input: {ex.Message}"); }

            // Configured USB HID button inputs — read from whichever
            // device is acquired (explicitly selected device only). Runs every tick so
            // edge detection doesn't depend on stick motion or DriveGimbal early-returning.
            try
            {
                var btnDev = _switchJoy;
                if (btnDev != null)
                {
                    var st = SafeGetState(btnDev);
                    if (st != null)
                    {
                        _ = DriveActuatorButtons(st);
                    }
                    else
                    {
                        ResetActuatorInput();
                    }
                }
                else
                {
                    ResetActuatorInput();
                }
            }
            catch (Exception ex)
            {
                ResetActuatorInput();
                Log.Error($"buttons: {ex.Message}");
            }
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

        private void DrivePositionInput(float dt)
        {
            if (!_config.JoystickPositionEnabled)
            {
                return;
            }
            var state = _positionJoy == null ? null : SafeGetState(_positionJoy);
            bool valid = TryReadAxisNorm(state, _config.JoystickPositionAxis,
                _config.JoystickPositionInvert, _config.JoystickPositionDeadzone, out var value);
            TranslatePositionInput(valid, value);
        }

        private static IMyJoystickState SafeGetState(JoystickBase joy)
        {
            try
            {
                return joy.GetCurrentState();
            }
            catch
            {
                return null;
            }
        }

        // ============================================================
        // Axis decoding
        // ============================================================

    }
}
