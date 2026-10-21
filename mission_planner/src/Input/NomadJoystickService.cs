// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Joystick Service
// ============================================================
// Drives the camera gimbal from a selected
// physical DirectInput joysticks. Reuses Mission Planner's
// MissionPlanner.Joystick.JoystickBase device wrapper so we don't
// duplicate device enumeration / state polling, but DELIBERATELY
// never calls JoystickBase.start() — MP's start() spawns the RC
// override loop that fights the autopilot for control. We only
// acquire the device and poll GetCurrentState() ourselves.
//
// Payload input uses the profile's local interlocks and native Mission Planner commands.

using System;
using System.Collections.Generic;
using System.Linq;
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

        private JoystickBase _gimbalJoy;

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

            try
            {
                AcquireDevices();
                AcquirePayloadDevices();
            }
            catch (Exception ex)
            {
                Log.Debug($"device acquire failed — {ex.Message}");
            }

            _timer = new Timer { Interval = 1000 / POLL_HZ };
            _timer.Tick += OnTick;
            _timer.Start();


        }

        public void Stop()
        {
            ResetPayloadInput();
            StopPayloadDevices();
            _gimbalInput.Reset();
            try
            {
                _timer?.Stop(); _timer?.Dispose();
            }
            catch
            {

            }
            _timer = null;

            ReleaseJoy(ref _gimbalJoy);
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
        public bool NeedsToRun() => _config.JoystickGimbalEnabled || _config.JoystickPositionEnabled ||
            PayloadBindings().Any(binding => binding != "None" && !string.IsNullOrWhiteSpace(binding));

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
            bool missing = _config.JoystickGimbalEnabled && _gimbalJoy == null;
            if (missing && (now - _lastReacquireAttempt).TotalSeconds >= REACQUIRE_INTERVAL_SEC)
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
                DrivePayloadInputs();
            }
            catch (Exception ex) { Log.Error($"gimbal: {ex.Message}"); }

        }

        private readonly GimbalInputState _gimbalInput = new GimbalInputState();

        private void DriveGimbal(float dt)
        {
            if (_gimbalJoy == null || !_config.JoystickGimbalEnabled) return;
            var st = SafeGetState(_gimbalJoy);
            if (st == null) { RetirePayloadDevice(_gimbalJoy); }
            if (st == null) { _gimbalInput.Reset(); return; }

            bool rollValid = TryReadAxisNorm(st, _config.JoystickGimbalRollAxis,
                _config.JoystickGimbalRollInvert, _config.JoystickGimbalDeadzone, out float roll);
            bool pitchValid = TryReadAxisNorm(st, _config.JoystickGimbalPitchAxis,
                _config.JoystickGimbalPitchInvert, _config.JoystickGimbalDeadzone, out float pitch);
            if (!_gimbalInput.CanMove(rollValid && pitchValid, roll, pitch, OutputController.Generation))
            {
                return;
            }

            // Use the centralized rate so the floating GimbalJoystickWindow's slider
            // and the settings dialog all agree on one value. No code duplication.
            GimbalController.ApplyStick(roll, pitch, dt, send: true);
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
