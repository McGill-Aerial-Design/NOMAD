// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using JoystickBase = MissionPlanner.Joystick.JoystickBase;

namespace NOMAD.MissionPlanner
{
    public sealed partial class NomadJoystickService
    {
        private readonly Dictionary<string, JoystickBase> _payloadDevices = new Dictionary<string, JoystickBase>(StringComparer.OrdinalIgnoreCase);
        private JoystickBase _positionJoy, _switchJoy;
        private bool[] _previousButtons;
        private readonly HashSet<string> _conflicting = new HashSet<string>();
        private readonly HashSet<string> _touchedOutputs = new HashSet<string>();
        private long _payloadGeneration = -1, _inputRevision;
        private bool _positionCentered;
        private double? _lastPosition;

        private string[] PayloadBindings() => new[] { _config.JoystickSw1UpAction, _config.JoystickSw1DownAction,
            _config.JoystickSw2UpAction, _config.JoystickSw2DownAction, _config.JoystickSw3UpAction, _config.JoystickSw3DownAction };
        private static bool Binding(string binding, out string id, out string operation)
        {
            int separator = binding?.LastIndexOf(':') ?? -1;
            id = separator > 0 ? binding.Substring(0, separator) : "";
            operation = separator > 0 ? binding.Substring(separator + 1) : "";
            return id != "" && operation != "";
        }
        private void AcquirePayloadDevices()
        {
            var available = EnumerateDevices();
            if (_config.JoystickPositionEnabled) { _positionJoy = AcquirePayloadDevice(_config.JoystickPositionDevice, available); }
            if (PayloadBindings().Any(binding => Binding(binding, out _, out _)))
            { _switchJoy = AcquirePayloadDevice(_config.JoystickSwitchDevice, available); }
        }
        private JoystickBase AcquirePayloadDevice(string selected, IList<string> available)
        {
            string name = ResolveDeviceName(selected, available);
            if (name == null) { return null; }
            if (_gimbalJoy != null && string.Equals(name, _config.JoystickGimbalDevice, StringComparison.OrdinalIgnoreCase))
            { return _gimbalJoy; }
            if (!_payloadDevices.TryGetValue(name, out var device))
            {
                device = CreateAndAcquire(name);
                if (device != null) { _payloadDevices.Add(name, device); }
            }
            return device;
        }
        private void StopPayloadDevices()
        {
            foreach (var handle in _payloadDevices.Values) { var device = handle; ReleaseJoy(ref device); }
            _payloadDevices.Clear();
            _positionJoy = null;
            _switchJoy = null;
        }
        private void DrivePayloadInputs()
        {
            ObservePayloadGeneration();
            if ((_config.JoystickPositionEnabled && _positionJoy == null) ||
                (PayloadBindings().Any(binding => Binding(binding, out _, out _)) && _switchJoy == null))
            {
                if (DateTime.UtcNow - _lastPayloadAcquire > TimeSpan.FromSeconds(3))
                { _lastPayloadAcquire = DateTime.UtcNow; AcquirePayloadDevices(); }
            }
            if (_config.JoystickPositionEnabled)
            {
                var state = _positionJoy == null ? null : SafeGetState(_positionJoy);
                if (state == null) { RetirePayloadDevice(_positionJoy); }
                bool valid = TryReadAxisNorm(state, _config.JoystickPositionAxis, _config.JoystickPositionInvert,
                    _config.JoystickPositionDeadzone, out float normalized);
                TranslatePayloadPosition(valid, normalized);
            }
            bool[] buttons = null;
            try
            {
                var state = _switchJoy == null ? null : SafeGetState(_switchJoy);
                if (state == null) { RetirePayloadDevice(_switchJoy); }
                buttons = state?.GetButtons();
            }
            catch { }
            TranslatePayloadButtons(buttons);
        }
        private DateTime _lastPayloadAcquire = DateTime.MinValue;
        private void RetirePayloadDevice(JoystickBase device)
        {
            if (device == null) { return; }
            foreach (string name in _payloadDevices.Where(entry => entry.Value == device).Select(entry => entry.Key).ToArray())
            { _payloadDevices.Remove(name); }
            if (_positionJoy == device) { _positionJoy = null; }
            if (_switchJoy == device) { _switchJoy = null; }
            if (_gimbalJoy == device) { _gimbalJoy = null; }
            ReleaseJoy(ref device);
        }
        private void ObservePayloadGeneration()
        {
            long generation = OutputController.Generation;
            if (_payloadGeneration == generation) { return; }
            // Retire input without sending old releases to a newly selected aircraft.
            _payloadGeneration = generation;
            ClearPayloadInput();
        }

        internal void TranslatePayloadButtons(bool[] buttons)
        {
            ObservePayloadGeneration();
            var bindings = PayloadBindings();
            if (!bindings.Any(binding => Binding(binding, out _, out _))) { return; }
            if (_config.GetInputMappingError() != null || buttons == null || bindings.Where((binding, slot) =>
                Binding(binding, out _, out _) && _config.JoystickButtonIndices[slot] >= buttons.Length).Any())
            { ResetPayloadButtons(); return; }
            var pressed = bindings.Select((binding, slot) => Binding(binding, out _, out _) && buttons[_config.JoystickButtonIndices[slot]]).ToArray();
            var conflicts = bindings.Where((binding, slot) => pressed[slot] && Binding(binding, out _, out _))
                .Select(binding => binding.Substring(0, binding.LastIndexOf(':'))).GroupBy(id => id)
                .Where(group => group.Count() > 1).Select(group => group.Key).ToHashSet();
            for (int slot = 0; slot < bindings.Length; slot++)
            {
                int first = slot / 2 * 2;
                if (pressed[first] && pressed[first + 1] && Binding(bindings[slot], out var id, out _))
                { conflicts.Add(id); }
            }
            if (_previousButtons == null || !pressed.SequenceEqual(_previousButtons)) { Interlocked.Increment(ref _inputRevision); }
            long revision = Interlocked.Read(ref _inputRevision);
            var events = new List<(string Id, string Operation, int Slot)>();
            for (int slot = 0; slot < bindings.Length; slot++)
            {
                if (!Binding(bindings[slot], out var id, out var operation)) { continue; }
                if (conflicts.Contains(id))
                {
                    if (!_conflicting.Contains(id) && !events.Any(input => input.Id == id)) { events.Add((id, "safe", slot / 2)); }
                    continue;
                }
                if (_previousButtons != null && _previousButtons[slot] && !pressed[slot] &&
                    PayloadController.Current?.Find(id)?.Behavior == "ServoBidirectional")
                { events.Add((id, "release_input", slot / 2)); }
                int first = slot / 2 * 2;
                if (!pressed[first] && !pressed[first + 1] && (_previousButtons == null || _previousButtons[first] || _previousButtons[first + 1]))
                {
                    if (!events.Any(input => input.Id == id && input.Operation == "neutral")) { events.Add((id, "neutral", slot / 2)); }
                }
                else if (_previousButtons != null && pressed[slot] && !_previousButtons[slot])
                { events.Add((id, operation, slot / 2)); }
            }
            _previousButtons = pressed;
            _conflicting.Clear();
            _conflicting.UnionWith(conflicts);
            foreach (var input in events) { _touchedOutputs.Add(input.Id); }
            _ = SendPayloadEvents(events, revision);
        }
        private async Task SendPayloadEvents(List<(string Id, string Operation, int Slot)> events, long revision)
        {
            var owner = PayloadController.Current;
            var config = _config;
            long generation = _payloadGeneration;
            if (owner?.Owns(config) != true || generation != OutputController.Generation) { return; }
            bool Safety(string operation) => operation == "safe" || operation == "stop" || operation == "release_input";
            var safety = new List<Task<string>>();
            foreach (var input in events.Where(input => Safety(input.Operation)))
            {
                // Admit every original-target safety action before waiting for a native helper.
                safety.Add(owner.ActAsync(input.Id, input.Operation, source: "hid", slot: input.Slot,
                    inputStillCurrent: () => owner.Owns(config) && generation == OutputController.Generation));
            }
            await Task.WhenAll(safety).ConfigureAwait(false);
            foreach (var input in events.Where(input => !Safety(input.Operation)))
            {
                if (!owner.Owns(config) || generation != OutputController.Generation ||
                    revision != Interlocked.Read(ref _inputRevision)) { return; }
                await owner.ActAsync(input.Id, input.Operation, source: "hid", slot: input.Slot,
                    inputStillCurrent: () => owner.Owns(config) && generation == OutputController.Generation &&
                        revision == Interlocked.Read(ref _inputRevision)).ConfigureAwait(false);
            }
        }

        internal void TranslatePayloadPosition(bool valid, float normalized)
        {
            ObservePayloadGeneration();
            long generation = _payloadGeneration;
            var owner = PayloadController.Current;
            string id = _config.JoystickPositionActuatorId;
            if (!valid || float.IsNaN(normalized) || float.IsInfinity(normalized) || Math.Abs(normalized) > 1 ||
                owner?.Owns(_config) != true || owner.Find(id)?.Continuous != true)
            {
                if (_lastPosition.HasValue) { SendPayloadSafe(id); }
                _positionCentered = false;
                _lastPosition = null;
                return;
            }
            if (normalized == 0) { _positionCentered = true; }
            if (!_positionCentered) { return; }
            double position = (normalized + 1.0) / 2.0;
            if (_lastPosition.HasValue && Math.Abs(_lastPosition.Value - position) < 0.001) { return; }
            if (PayloadController.HasPendingWork) { return; }
            _lastPosition = position;
            _ = owner.ActAsync(id, "position", position, "hid", 6,
                () => owner.Owns(_config) && generation == OutputController.Generation && _positionCentered);
        }
        private void SendPayloadSafe(string id)
        {
            var owner = PayloadController.Current;
            if (owner?.Owns(_config) == true)
            { _ = owner.ActAsync(id, "safe", source: "hid", slot: 0); }
        }
        private void ResetPayloadInput()
        {
            if (_lastPosition.HasValue) { SendPayloadSafe(_config.JoystickPositionActuatorId); }
            ResetPayloadButtons();
            ClearPayloadInput();
        }
        private void ResetPayloadButtons()
        {
            foreach (string id in _touchedOutputs.ToArray()) { SendPayloadSafe(id); }
            Interlocked.Increment(ref _inputRevision);
            _previousButtons = null;
            _conflicting.Clear();
            _touchedOutputs.Clear();
        }
        private void ClearPayloadInput()
        {
            Interlocked.Increment(ref _inputRevision);
            _previousButtons = null;
            _conflicting.Clear();
            _touchedOutputs.Clear();
            _positionCentered = false;
            _lastPosition = null;
        }
    }
}
