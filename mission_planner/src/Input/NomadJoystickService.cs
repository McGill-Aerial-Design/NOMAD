// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Windows.Forms;
using MissionPlanner;
using JoystickBase = MissionPlanner.Joystick.JoystickBase;
using IMyJoystickState = MissionPlanner.Joystick.IMyJoystickState;
namespace NOMAD.MissionPlanner
{
    // Poll the acquired wrapper only. start() would start MP's flight RC override loop.
    public sealed partial class NomadJoystickService : IDisposable
    {
        private NOMADConfig _config;
        private Timer _timer;
        private readonly Dictionary<string, JoystickBase> _devices = new Dictionary<string, JoystickBase>(StringComparer.OrdinalIgnoreCase);
        private static readonly Dictionary<string, PropertyInfo> AxisProps = BuildAxisMap();
        private DateTime _lastAcquire = DateTime.MinValue, _lastTick;
        public NomadJoystickService(NOMADConfig config) { _config = config ?? throw new ArgumentNullException(nameof(config)); }
        public static IReadOnlyList<string> AxisNames => InputBinding.Axes;
        public bool NeedsToRun() => InputBinding.Parse(_config.Inputs).Any(binding => binding.Enabled);
        public void Start()
        {
            Stop(); _lastTick = DateTime.UtcNow; _lastAcquire = DateTime.MinValue;
            _timer = new Timer { Interval = 50 }; _timer.Tick += OnTick; _timer.Start();
        }
        public void Stop()
        {
            ResetInputs(true);
            _timer?.Stop(); _timer?.Dispose(); _timer = null;
            foreach (var handle in _devices.Values) { var device = handle; ReleaseJoy(ref device); }
            _devices.Clear();
        }
        public void RestartWithConfig() { Stop(); if (NeedsToRun()) { Start(); } }
        public void UpdateConfig(NOMADConfig config)
        { Stop(); _config = config ?? throw new ArgumentNullException(nameof(config)); if (NeedsToRun()) { Start(); } }
        public void Dispose() => Stop();
        public static IList<string> EnumerateDevices()
        {
            try { return JoystickBase.getDevices(); }
            catch (Exception error) { Log.Debug("HID enumeration failed: " + error.Message); return new List<string>(); }
        }
        internal static string ResolveDeviceName(string configured, IList<string> available)
        {
            if (string.IsNullOrWhiteSpace(configured) || available == null) { return null; }
            var matches = available.Where(name => string.Equals(name, configured, StringComparison.OrdinalIgnoreCase)).ToArray();
            return matches.Length == 1 ? matches[0] : null;
        }
        internal static JoystickBase CreateAndAcquire(string name)
        {
            var device = JoystickBase.Create(() => MainV2.comPort);
            try { if (device.AcquireJoystick(name)) { return device; } }
            catch (Exception error) { Log.Debug("HID acquire failed: " + error.Message); }
            ReleaseJoy(ref device); return null;
        }
        internal static void ReleaseJoy(ref JoystickBase device)
        {
            if (device == null) { return; }
            try { device.UnAcquireJoyStick(); } catch { }
            try { device.Dispose(); } catch { }
            device = null;
        }
        internal static IMyJoystickState SafeGetState(JoystickBase device)
        { try { return device?.GetCurrentState(); } catch { return null; } }
        private void OnTick(object sender, EventArgs args)
        {
            try
            {
                var now = DateTime.UtcNow;
                if (now - _lastAcquire > TimeSpan.FromSeconds(3))
                {
                    _lastAcquire = now;
                    var available = EnumerateDevices();
                    foreach (string name in _devices.Keys.ToArray())
                    {
                        if (ResolveDeviceName(name, available) != null) { continue; }
                        var retired = _devices[name]; _devices.Remove(name); ReleaseJoy(ref retired);
                    }
                    foreach (string name in InputBinding.Parse(_config.Inputs).Where(binding => binding.Enabled)
                        .Select(binding => binding.Device).Distinct(StringComparer.OrdinalIgnoreCase))
                    {
                        if (_devices.ContainsKey(name) || ResolveDeviceName(name, available) == null) { continue; }
                        var device = CreateAndAcquire(name);
                        if (device != null) { _devices.Add(name, device); }
                    }
                }
                var samples = new Dictionary<string, IMyJoystickState>(StringComparer.OrdinalIgnoreCase);
                foreach (var item in _devices.ToArray())
                {
                    var sample = SafeGetState(item.Value); samples[item.Key] = sample;
                    if (sample == null) { var device = item.Value; _devices.Remove(item.Key); ReleaseJoy(ref device); }
                }
                float dt = (float)(now - _lastTick).TotalSeconds; _lastTick = now;
                TranslateInputs(samples, dt > 0 && dt <= 0.5f ? dt : 0.05f);
            }
            catch (Exception error) { ResetInputs(true); Log.Error("HID input failed: " + error.Message); }
        }
    }
}
