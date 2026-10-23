// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using IMyJoystickState = MissionPlanner.Joystick.IMyJoystickState;
namespace NOMAD.MissionPlanner
{
    public sealed partial class NomadJoystickService
    {
        private readonly Dictionary<string, InputControlState> _states = new Dictionary<string, InputControlState>();
        // Availability survives conflict/neutral resets so absent devices cannot retire other controls.
        private readonly Dictionary<string, bool> _available = new Dictionary<string, bool>();
        private readonly Dictionary<string, double> _positions = new Dictionary<string, double>();
        private readonly HashSet<(string Id, string Input)> _touched = new HashSet<(string, string)>();
        private readonly HashSet<string> _conflicting = new HashSet<string>();
        private long _generation = -1, _inputRevision;
        private readonly GimbalInputState _gimbalInput = new GimbalInputState();
        internal void TranslateInputs(IDictionary<string, IMyJoystickState> samples, float dt = 0.05f)
        {
            if (!OutputController.IsCurrentProfile(_config)) { ResetInputs(false); return; }
            if (_generation != OutputController.Generation) { ResetInputs(false); _generation = OutputController.Generation; }
            var bindings = InputBinding.Parse(_config.Inputs).Where(binding => binding.Enabled).ToList();
            var events = new List<(string Id, string Operation, string Input)>();
            var active = new List<(InputBinding Binding, string Action)>();
            var positions = new List<(InputBinding Binding, InputControlState State)>();
            float pitch = 0, roll = 0;
            bool gimbalValid = true, hasGimbal = false;
            foreach (var binding in bindings)
            {
                if (!_states.TryGetValue(binding.Id, out var state)) { state = new InputControlState(); _states.Add(binding.Id, state); }
                int previous = state.Direction; bool ready = state.Ready;
                samples.TryGetValue(binding.Device, out var sample);
                bool valid = state.Sample(binding, sample);
                if (InputBinding.Gimbal(binding.Action))
                {
                    hasGimbal = true; gimbalValid &= valid;
                    if (binding.Action == "gimbal:pitch") { pitch = state.Value; } else { roll = state.Value; }
                    continue;
                }
                bool wasAvailable = _available.TryGetValue(binding.Id, out bool available) && available;
                _available[binding.Id] = valid;
                if (valid != wasAvailable || state.Changed) { Interlocked.Increment(ref _inputRevision); }
                if (!valid)
                {
                    foreach (var touched in _touched.Where(value => value.Input == binding.Id).ToArray())
                    { events.Add((touched.Id, "safe", binding.Id)); _touched.Remove(touched); }
                    _positions.Remove(binding.Id); continue;
                }
                if (binding.Continuous)
                {
                    active.Add((binding, binding.Action));
                    if (state.Ready) { positions.Add((binding, state)); }
                    continue;
                }
                AddDigitalEvents(binding, state, previous, ready, events, active);
            }
            var conflicts = active.Where(value => InputBinding.Split(value.Action, out _, out _))
                .GroupBy(value => value.Action.Substring(0, value.Action.LastIndexOf(':')))
                .Where(group => group.Count() > 1).Select(group => group.Key).ToHashSet();
            if (!_conflicting.SetEquals(conflicts)) { Interlocked.Increment(ref _inputRevision); }
            events.RemoveAll(value => conflicts.Contains(value.Id));
            foreach (string id in conflicts)
            {
                var inputs = active.Where(value => InputBinding.Split(value.Action, out var target, out _) && target == id).ToList();
                if (!_conflicting.Contains(id)) { events.Add((id, "safe", inputs[0].Binding.Id)); }
                foreach (var input in inputs)
                {
                    _states[input.Binding.Id].RequireNeutral();
                    _positions.Remove(input.Binding.Id);
                }
            }
            _conflicting.Clear(); _conflicting.UnionWith(conflicts);
            foreach (var input in events)
            {
                if (input.Operation == "safe" || input.Operation == "release_input") { _touched.Remove((input.Id, input.Input)); }
                else { _touched.Add((input.Id, input.Input)); }
            }
            _ = SendPayloadEvents(events.Distinct().ToList(), Interlocked.Read(ref _inputRevision));
            foreach (var position in positions)
            {
                if (InputBinding.Split(position.Binding.Action, out var id, out _) && !conflicts.Contains(id))
                { DrivePosition(position.Binding, position.State); }
            }
            if (!OutputController.IsCurrentProfile(_config) || _generation != OutputController.Generation)
            { _gimbalInput.Reset(); return; }
            if (hasGimbal && _gimbalInput.CanMove(gimbalValid, roll, pitch, _generation))
            { GimbalController.ApplyStick(roll, pitch, dt, true); }
        }
        private static void AddDigitalEvents(InputBinding binding, InputControlState state, int previous, bool ready,
            List<(string Id, string Operation, string Input)> events, List<(InputBinding Binding, string Action)> active)
        {
            if (previous != 0 && previous != state.Direction)
            {
                string released = previous < 0 ? binding.OtherAction : binding.Action;
                if (InputBinding.Split(released, out var id, out _) && PayloadController.Current?.Find(id)?.Behavior == "ServoBidirectional")
                { events.Add((id, "release_input", binding.Id)); }
            }
            if (state.Direction == 2)
            {
                if (!state.Changed) { return; }
                foreach (string action in binding.Actions)
                { if (InputBinding.Split(action, out var id, out _)) { events.Add((id, "safe", binding.Id)); } }
            }
            else if (state.Direction == 0 && state.Ready && (state.Changed || !ready))
            {
                foreach (string action in binding.Actions)
                { if (InputBinding.Split(action, out var id, out _)) { events.Add((id, "neutral", binding.Id)); } }
            }
            else if (state.Direction != 0)
            {
                string action = state.Direction < 0 ? binding.OtherAction : binding.Action;
                active.Add((binding, action));
                if (state.Ready && state.Changed && InputBinding.Split(action, out var id, out var operation))
                { events.Add((id, operation, binding.Id)); }
            }
        }
        private void DrivePosition(InputBinding binding, InputControlState state)
        {
            if (!InputBinding.Split(binding.Action, out var id, out _) || PayloadController.Current?.Find(id)?.Continuous != true) { return; }
            double position = (state.Value + 1.0) / 2;
            if ((_positions.TryGetValue(binding.Id, out var previous) && Math.Abs(previous - position) < 0.001) || PayloadController.HasPendingWork) { return; }
            var owner = PayloadController.Current;
            long generation = _generation, revision = Interlocked.Read(ref _inputRevision);
            _positions[binding.Id] = position; _touched.Add((id, binding.Id));
            _ = owner.ActAsync(id, "position", position, "hid", binding.Id,
                () => owner.Owns(_config) && generation == OutputController.Generation && state.Ready && revision == Interlocked.Read(ref _inputRevision));
        }
        private async Task SendPayloadEvents(List<(string Id, string Operation, string Input)> events, long revision)
        {
            var owner = PayloadController.Current; var config = _config; long generation = _generation;
            if (owner?.Owns(config) != true || generation != OutputController.Generation) { return; }
            bool Safety(string operation) => operation == "safe" || operation == "stop" || operation == "release_input";
            var safety = new List<Task<string>>();
            foreach (var input in events.Where(input => Safety(input.Operation)).GroupBy(input => input.Id)
                .Select(group => group.OrderBy(input => input.Operation == "safe" ? 0 : 1).First()))
            {
                // Admit every original-target safety action before waiting for native retries.
                safety.Add(owner.ActAsync(input.Id, input.Operation, source: "hid", inputId: input.Input,
                    inputStillCurrent: () => owner.Owns(config) && generation == OutputController.Generation));
            }
            await Task.WhenAll(safety).ConfigureAwait(false);
            foreach (var input in events.Where(input => !Safety(input.Operation)))
            {
                if (!owner.Owns(config) || generation != OutputController.Generation || revision != Interlocked.Read(ref _inputRevision)) { return; }
                await owner.ActAsync(input.Id, input.Operation, source: "hid", inputId: input.Input,
                    inputStillCurrent: () => owner.Owns(config) && generation == OutputController.Generation && revision == Interlocked.Read(ref _inputRevision)).ConfigureAwait(false);
            }
        }
        private void ResetInputs(bool safe)
        {
            if (safe && _generation == OutputController.Generation)
            { _ = SendPayloadEvents(_touched.Select(value => (value.Id, "safe", value.Input)).ToList(), Interlocked.Read(ref _inputRevision)); }
            Interlocked.Increment(ref _inputRevision);
            _states.Clear(); _available.Clear(); _positions.Clear(); _touched.Clear(); _conflicting.Clear(); _gimbalInput.Reset();
        }
    }
}
