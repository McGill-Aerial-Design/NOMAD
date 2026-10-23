// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    // Only local input interlocks live here; Mission Planner owns command ACKs and retries.
    internal sealed class PayloadController
    {
        private sealed class State
        {
            internal int Confirmations;
            internal long FirstConfirmation;
            internal string Intent;
            internal readonly HashSet<string> NeutralInputs = new HashSet<string>();
            internal bool Known, Active, Recovery;
            internal string Status = "";
        }
        internal static PayloadController Current { get; private set; }
        private readonly object _gate = new object();
        private NOMADConfig _config;
        internal IReadOnlyList<PayloadDefinition> Definitions { get; }
        private readonly Dictionary<string, State> _states = new Dictionary<string, State>();
        private Task<string> _pending = Task.FromResult("");
        private readonly HashSet<string> _safePending = new HashSet<string>();
        private bool _busy, _stopping, _stopRequested, _retired;
        private long _generation = -1;
        private static readonly object RepeatGate = new object();
        private static long _repeatUntil;
        private static bool _repeatPending;
        private static PayloadController _repeatOwner;
        private static string _repeatId;
        internal event Action<string> Changed;
        internal static bool HasPendingWork
        {
            get
            {
                var owner = Current;
                if (owner != null) { lock (owner._gate) { if (owner._busy || owner._stopping) { return true; } } }
                return RepeatBusy();
            }
        }

        internal PayloadController(NOMADConfig config)
        {
            _config = config;
            Definitions = PayloadDefinition.Parse(config.ActuatorDefinitions);
            foreach (var definition in Definitions) { _states.Add(definition.Id, new State()); }
        }

        internal static void ApplyProfile(NOMADConfig config)
        {
            if (Current != null && !Current._retired && Current._config.HasSameOutputSettings(config))
            {
                lock (Current._gate) { Current._config = config; Current.ResetInput(); }
                return;
            }
            Shutdown();
            Current = new PayloadController(config);
        }
        internal static void Shutdown()
        {
            if (Current != null)
            {
                lock (Current._gate) { Current._retired = true; Current._stopRequested = true; Current.ResetInput(); }
            }
            Current = null;
        }

        internal bool Owns(NOMADConfig config) => !_retired && ReferenceEquals(config, _config) && OutputController.IsCurrentProfile(config);
        internal PayloadDefinition Find(string id) => Definitions.FirstOrDefault(definition => definition.Id == id);
        internal string Status(string id)
        {
            lock (_gate) { return _states.TryGetValue(id, out var state) ? state.Status : "Output unavailable."; }
        }
        internal Task WaitForPendingAsync()
        { lock (_gate) { return _pending; } }
        private void ResetInput()
        {
            foreach (var state in _states.Values)
            { state.Confirmations = 0; state.Intent = null; state.NeutralInputs.Clear(); }
        }
        private void ObserveGeneration(long generation)
        {
            if (_generation == generation) { return; }
            _generation = generation;
            ResetInput();
            foreach (var state in _states.Values) { state.Known = false; }
        }

        internal Task<string> ActAsync(string id, string operation, double? value = null,
            string source = "ui", string inputId = null, Func<bool> inputStillCurrent = null)
        {
            var target = OutputController.CaptureTarget(_config);
            lock (_gate)
            {
                var definition = Find(id);
                if (_retired || target == null || inputStillCurrent?.Invoke() == false || definition == null)
                { return Task.FromResult("Aircraft or input is unavailable; no command sent."); }
                ObserveGeneration(target.Generation);
                var state = _states[id];
                if ((source != "ui" && source != "hid") || (source == "ui" ? inputId != null : !InputBinding.Parse(_config.Inputs).Any(binding =>
                    binding.Id == inputId && binding.Authorizes(id, operation))))
                { return Task.FromResult("Invalid input source; no command sent."); }
                if (operation == "neutral" && source == "hid" && !value.HasValue)
                { state.NeutralInputs.Add(inputId); return Task.FromResult("Neutral input observed; no command sent."); }
                if (operation == "release_input" && source == "hid" && definition.Behavior == "ServoBidirectional")
                {
                    state.NeutralInputs.Remove(inputId);
                    if (!_busy && !state.Active && !state.Recovery) { return Task.FromResult("Input released; no command sent."); }
                    operation = "stop";
                }
                if (operation == "toggle" && (definition.Behavior == "ServoToggle" || definition.Behavior == "RelayToggle"))
                {
                    if (!state.Known) { return SetStatus(id, "Toggle state is unknown. Send Safe before using toggle."); }
                    operation = state.Active ? "safe" : "activate";
                }
                bool safe = operation == "safe" || operation == "stop";
                if ((safe && value.HasValue) || (!safe && (!definition.Operations.Contains(operation) ||
                    (operation == "position" ? !ValidPosition(value) : value.HasValue))))
                { return SetStatus(id, "Invalid payload action or position; no command sent."); }
                if (safe)
                {
                    if (!_safePending.Add(id)) { return Task.FromResult("Safe command is already pending."); }
                    _stopping = true;
                    _stopRequested = true;
                    ResetInput();
                    _pending = SendSafeAsync(definition, state, target, _pending);
                    return _pending;
                }
                if (_busy || _stopping || state.Recovery || RepeatBusy())
                { return SetStatus(id, "Payload busy or needs an explicit accepted Safe."); }
                if (source == "hid" && operation == "position" && !definition.Continuous)
                { return SetStatus(id, "Position input requires a nonhazardous payload without confirmations."); }
                if (!Confirmed(definition, state, operation, value, source, inputId))
                { return SetStatus(id, "Confirmation incomplete or neutral needed."); }
                if (definition.Timed && !ReserveRepeat(definition.Id))
                { return SetStatus(id, "Another timed action is active."); }
                _busy = true;
                _stopRequested = false;
                _pending = ActivateAsync(definition, state, operation, value, target, inputStillCurrent);
                return _pending;
            }
        }

        private static bool ValidPosition(double? value) => value.HasValue && !double.IsNaN(value.Value) &&
            !double.IsInfinity(value.Value) && value >= 0 && value <= 1;
        private bool Confirmed(PayloadDefinition definition, State state, string operation, double? value, string source, string inputId)
        {
            if (definition.Confirmations == 0) { return true; }
            string intent = operation + ":" + value?.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ":" + source + ":" + inputId;
            long now = Stopwatch.GetTimestamp();
            if (state.Intent != intent || now - state.FirstConfirmation > definition.WindowMs * (Stopwatch.Frequency / 1000.0))
            { state.Confirmations = 0; state.Intent = intent; }
            if (source == "hid" && definition.RequireNeutral && !state.NeutralInputs.Contains(inputId)) { return false; }
            state.NeutralInputs.Remove(inputId);
            if (state.Confirmations == 0) { state.FirstConfirmation = now; }
            if (++state.Confirmations < definition.Confirmations) { return false; }
            state.Confirmations = 0;
            return true;
        }

        private static bool RepeatBusy()
        {
            lock (RepeatGate) { return _repeatPending || Stopwatch.GetTimestamp() < _repeatUntil; }
        }
        private bool ReserveRepeat(string id)
        {
            lock (RepeatGate)
            {
                if (_repeatPending || Stopwatch.GetTimestamp() < _repeatUntil) { return false; }
                _repeatOwner = this;
                _repeatId = id;
                _repeatPending = true;
                return true;
            }
        }
        private void CompleteRepeat(PayloadDefinition definition, bool repeatAttempted)
        {
            lock (RepeatGate)
            {
                if (_repeatOwner != this || _repeatId != definition.Id) { return; }
                _repeatPending = false;
                int duration = definition.Behavior == "RelayPulse" ? definition.PulseMs : definition.HoldMs;
                _repeatUntil = repeatAttempted ? Stopwatch.GetTimestamp() + (long)(duration * (Stopwatch.Frequency / 1000.0)) : 0;
            }
        }

        private async Task<string> ActivateAsync(PayloadDefinition definition, State state, string operation,
            double? value, OutputController.NativeTarget target, Func<bool> current)
        {
            bool attempted = false;
            bool repeatAttempted = false;
            bool StillCurrent() => !_retired && !_stopRequested && target.IsCurrent && current?.Invoke() != false;
            try
            {
                if (definition.Behavior == "ServoBidirectional")
                {
                    float trim = await target.ReadParameterAsync("SERVO" + definition.Channel + "_TRIM").ConfigureAwait(false);
                    if (trim != definition.Safe)
                    { return await SetStatus(definition.Id, "Live SERVO" + definition.Channel + "_TRIM must equal profile safe/neutral PWM; no activation sent."); }
                }
                if (!StillCurrent()) { return await SetStatus(definition.Id, "Input retired; no activation sent."); }
                if (definition.Behavior == "RelayPulse")
                {
                    attempted = true;
                    if (!await target.SendAsync(MAVLink.MAV_CMD.DO_SET_RELAY, definition.Channel, 0, current: StillCurrent).ConfigureAwait(false))
                    { return await SetStatus(definition.Id, "Relay initialization was rejected; pulse not started."); }
                }
                if (!StillCurrent()) { return await SetStatus(definition.Id, "Input retired; no activation sent."); }
                int pwm = operation == "position" ? definition.Minimum + (int)Math.Round(value.Value * (definition.Maximum - definition.Minimum), MidpointRounding.AwayFromZero) :
                    operation == "negative" || (definition.Behavior == "ServoToggle" && definition.Reversed) ? definition.Minimum : definition.Maximum;
                attempted = true;
                bool accepted;
                if (definition.Behavior == "RelayPulse")
                { repeatAttempted = true; accepted = await target.SendAsync(MAVLink.MAV_CMD.DO_REPEAT_RELAY, definition.Channel, 1, 2 * definition.PulseMs / 1000f, current: StillCurrent).ConfigureAwait(false); }
                else if (definition.Behavior == "ServoBidirectional")
                { repeatAttempted = true; accepted = await target.SendAsync(MAVLink.MAV_CMD.DO_REPEAT_SERVO, definition.Channel, pwm, 1, 2 * definition.HoldMs / 1000f, StillCurrent).ConfigureAwait(false); }
                else
                { accepted = await target.SendAsync(definition.Relay ? MAVLink.MAV_CMD.DO_SET_RELAY : MAVLink.MAV_CMD.DO_SET_SERVO,
                    definition.Channel, definition.Relay ? 1 : pwm, current: StillCurrent).ConfigureAwait(false); }
                lock (_gate)
                {
                    state.Known = accepted && target.IsCurrent;
                    state.Active = accepted;
                    if (!target.IsCurrent) { state.Recovery = true; }
                }
                if (!target.IsCurrent)
                { return await SetStatus(definition.Id, "Aircraft changed. Check payload state."); }
                return await SetStatus(definition.Id, accepted ? "Command accepted." :
                    "Command not confirmed. Check payload.");
            }
            catch (Exception error)
            {
                lock (_gate) { state.Recovery |= attempted; state.Known = false; }
                return await SetStatus(definition.Id, "Payload outcome uncertain. Check payload and use Safe. " + error.Message);
            }
            finally
            {
                if (definition.Timed) { CompleteRepeat(definition, repeatAttempted); }
                lock (_gate) { _busy = false; }
            }
        }

        private async Task<string> SendSafeAsync(PayloadDefinition definition, State state,
            OutputController.NativeTarget target, Task<string> previous)
        {
            try
            {
                // Finish the host's already-dispatched retries before safe, so they cannot reactivate afterwards.
                await previous.ConfigureAwait(false);
                bool accepted = await target.SendAsync(definition.Relay ? MAVLink.MAV_CMD.DO_SET_RELAY : MAVLink.MAV_CMD.DO_SET_SERVO,
                    definition.Channel, definition.Relay ? 0 : definition.Safe).ConfigureAwait(false);
                lock (_gate)
                {
                    state.Known = accepted && target.IsCurrent;
                    state.Active = false;
                    state.Recovery = !state.Known;
                    if (accepted)
                    {
                        lock (RepeatGate)
                        {
                            if (_repeatOwner == this && _repeatId == definition.Id)
                            { _repeatPending = false; _repeatUntil = 0; }
                        }
                    }
                }
                if (!target.IsCurrent)
                { return await SetStatus(definition.Id, "Aircraft changed during Safe. Check payload."); }
                return await SetStatus(definition.Id, accepted ? "Safe accepted." :
                    "Safe not confirmed. Check payload.");
            }
            catch (Exception error)
            {
                lock (_gate) { state.Known = false; state.Recovery = true; }
                return await SetStatus(definition.Id, "Safe outcome uncertain. Inspect before retrying. " + error.Message);
            }
            finally { lock (_gate) { _safePending.Remove(definition.Id); _stopping = _safePending.Count != 0; } }
        }

        private Task<string> SetStatus(string id, string message)
        {
            lock (_gate) { _states[id].Status = message; }
            Changed?.Invoke(id);
            return Task.FromResult(message);
        }
    }
}
