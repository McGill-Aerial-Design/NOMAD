// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Text;
using System.Threading.Tasks;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    // Native firmware owns entry checks, RC/fence responses, phase selection and the reboot latch.
    internal sealed class TerminationController : IDisposable
    {
        private readonly object _gate = new object();
        private readonly NOMADConfig _config;
        private OutputController.NativeTarget _target;
        private MAVState _mav;
        private int[] _subscriptions = Array.Empty<int>();
        private uint? _mode, _observedMode;
        private string _name, _outcome;
        private TaskCompletionSource<MAVLink.mavlink_available_modes_t?> _discovery;
        private int _requestedIndex, _modeCount;
        private long _heartbeat;
        private bool _armed, _supported, _configurationSupported, _selectable, _busy, _disposed, _ambiguous;
        private bool _seenTermination;

        internal TerminationController(NOMADConfig config)
        {
            _config = config;
            OutputController.ConnectionChanged += Invalidate;
            UpdateBinding();
        }

        internal void UpdateBinding()
        {
            var target = OutputController.CaptureTarget(_config);
            var mav = target?.Connection.MAV;
            lock (_gate)
            {
                if (_disposed || (_target?.Generation == target?.Generation && ReferenceEquals(_mav, mav))) { return; }
            }
            Invalidate();
            if (target == null || mav == null || !target.IsCurrent) { return; }
            lock (_gate)
            {
                if (_disposed) { return; }
                _target = target;
                _mav = mav;
            }
            var ids = new[]
            {
                target.Connection.SubscribeToPacketType(MAVLink.MAVLINK_MSG_ID.HEARTBEAT,
                    packet => Observe(target, mav, packet), target.System, target.Component, false),
                target.Connection.SubscribeToPacketType(MAVLink.MAVLINK_MSG_ID.AVAILABLE_MODES,
                    packet => Observe(target, mav, packet), target.System, target.Component, false),
            };
            bool keep;
            lock (_gate)
            {
                keep = ReferenceEquals(_target, target);
                if (keep) { _subscriptions = ids; }
            }
            if (!keep) { Unsubscribe(target, ids); }
        }

        private bool Observe(OutputController.NativeTarget target, MAVState mav, MAVLink.MAVLinkMessage packet)
        {
            // Never take OutputController's gate while holding our gate: native dispatch guards take the reverse order.
            if (!target.IsCurrent || !ReferenceEquals(target.Connection.MAV, mav) ||
                packet.sysid != target.System || packet.compid != target.Component) { return true; }
            lock (_gate)
            {
                if (_disposed || !ReferenceEquals(_target, target)) { return true; }
                if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.AVAILABLE_MODES)
                {
                    double age = (DateTime.UtcNow - packet.rxtime).TotalSeconds;
                    if (packet.rxtime.Kind != DateTimeKind.Utc || age < 0 || age > 3.5) { return true; }
                    var mode = packet.ToStructure<MAVLink.mavlink_available_modes_t>();
                    bool expected = _requestedIndex == 0 || mode.mode_index == _requestedIndex;
                    if (mode.number_modes == 0 || mode.mode_index == 0 || mode.mode_index > mode.number_modes ||
                        (_requestedIndex != 0 && _modeCount != 0 && mode.number_modes != _modeCount))
                    {
                        _supported = _configurationSupported = false;
                        if (expected) { _discovery?.TrySetResult(null); }
                        return true;
                    }
                    float encodedMode = mode.custom_mode;
                    int terminator = Array.IndexOf(mode.mode_name, (byte)0);
                    string name = Encoding.UTF8.GetString(mode.mode_name, 0,
                        terminator < 0 ? mode.mode_name.Length : terminator);
                    if (string.Equals(name, "TERMINATE", StringComparison.OrdinalIgnoreCase))
                    {
                        bool validMode = mode.custom_mode <= byte.MaxValue && (double)encodedMode == mode.custom_mode;
                        if (!validMode)
                        {
                            _selectable = _supported = _configurationSupported = false;
                            _discovery?.TrySetResult(null);
                            return true;
                        }
                        bool selectable = (mode.properties & (uint)MAVLink.MAV_MODE_PROPERTY.NOT_USER_SELECTABLE) == 0;
                        if (_requestedIndex != 0) { _selectable &= selectable; }
                        else { _selectable = selectable; }
                        if (!selectable) { _supported = false; }
                        _ambiguous |= _mode.HasValue && _mode.Value != mode.custom_mode;
                        // Remember out-of-order IDs for conflicts without establishing selectability.
                        if (expected || !_mode.HasValue) { _mode = mode.custom_mode; _name = name; }
                        if (expected && _requestedIndex != 0) { _seenTermination = true; }
                    }
                    if (_requestedIndex != 0 && expected)
                    {
                        _modeCount = mode.number_modes;
                        _discovery?.TrySetResult(mode);
                    }
                }
                else
                {
                    var heartbeat = packet.ToStructure<MAVLink.mavlink_heartbeat_t>();
                    double age = (DateTime.UtcNow - packet.rxtime).TotalSeconds;
                    if (heartbeat.autopilot != (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA ||
                        packet.rxtime.Kind != DateTimeKind.Utc || age < 0 || age > 3.5) { return true; }
                    _heartbeat = Stopwatch.GetTimestamp() - (long)(age * Stopwatch.Frequency);
                    _armed = (heartbeat.base_mode & (byte)MAVLink.MAV_MODE_FLAG.SAFETY_ARMED) != 0;
                    _observedMode = (heartbeat.base_mode & (byte)MAVLink.MAV_MODE_FLAG.CUSTOM_MODE_ENABLED) != 0
                        ? (uint?)heartbeat.custom_mode : null;
                    // MP 1.3.83 preserves the text for unknown modes, then records the numeric heartbeat itself.
                    if (!_ambiguous && _mode.HasValue && _observedMode == _mode) { mav.cs.mode = _name; }
                }
            }
            return true;
        }

        private bool Fresh => _heartbeat != 0 &&
            (Stopwatch.GetTimestamp() - _heartbeat) / (double)Stopwatch.Frequency <= 3.5;
        private bool ObservedNow => Fresh && !_ambiguous && _mode.HasValue && _observedMode == _mode;
        internal bool Observed { get { lock (_gate) { return ObservedNow; } } }
        internal bool Supported { get { lock (_gate) { return Fresh && _supported && _selectable && !_ambiguous; } } }
        internal bool Armed { get { lock (_gate) { return Fresh && _armed; } } }

        internal string Status
        {
            get
            {
                lock (_gate)
                {
                    if (_disposed || _target == null) { return "No live writable aircraft."; }
                    if (!Fresh) { return "No fresh aircraft heartbeat."; }
                    if (ObservedNow)
                    { return "TERMINATE observed. Reboot required."; }
                    if (_supported && !_armed) { return "TERMINATE supported; aircraft is not armed."; }
                    return _outcome ?? (_supported ? "TERMINATE supported."
                        : "Check TERMINATE support.");
                }
            }
        }

        internal bool CanRequest
        {
            get { lock (_gate) { return CanSend(_target, _mode); } }
        }

        private bool CanSend(OutputController.NativeTarget target, uint? mode) =>
            !_disposed && ReferenceEquals(_target, target) && target != null && _supported && _selectable && !_ambiguous &&
            mode.HasValue && _mode == mode && Fresh && _armed && _observedMode != mode;

        internal Task<bool> CheckAsync() => CheckNativeAsync(true);
        internal Task<bool> CheckConfigurationAsync() => CheckNativeAsync(false);

        internal bool CanConfigure(long generation)
        {
            lock (_gate)
            {
                return !_disposed && !_busy && _target?.Generation == generation && _configurationSupported &&
                    !_ambiguous && Fresh && !_armed && !ObservedNow;
            }
        }

        internal void RetireConfiguredSupport()
        {
            lock (_gate)
            {
                _supported = false;
                _outcome = "Termination settings changed; check support again before requesting TERMINATE.";
            }
        }

        private async Task<bool> CheckNativeAsync(bool requireConfigured)
        {
            UpdateBinding();
            OutputController.NativeTarget target;
            lock (_gate)
            {
                if (_disposed || _target == null || _busy) { return false; }
                target = _target;
                _busy = true;
                _supported = false;
                _configurationSupported = false;
                _selectable = true;
                _seenTermination = false;
                _requestedIndex = _modeCount = 0;
                _discovery = null;
                _outcome = "Checking support...";
            }
            try
            {
                bool advertised = await DiscoverModesAsync(target).ConfigureAwait(false);
                if (!advertised) { return SetOutcome(target, "Native capability discovery was incomplete or invalid."); }
                if (!target.IsCurrent) { return false; }
                float enabled = await target.ReadParameterAsync("AFS_ENABLE").ConfigureAwait(false);
                float action = await target.ReadParameterAsync("AFS_TERM_ACTION").ConfigureAwait(false);
                float rate = await target.ReadParameterAsync("AFS_TERM_RATE").ConfigureAwait(false);
                bool current = target.IsCurrent;
                lock (_gate)
                {
                    if (!current || !ReferenceEquals(_target, target)) { return false; }
                    _supported = advertised && _selectable && !_ambiguous && enabled == 1 && action == 44 &&
                        !float.IsNaN(rate) && !float.IsInfinity(rate) && rate >= 2;
                    _configurationSupported = advertised && !_ambiguous &&
                        target.Connection.MAVlist[target.System, target.Component].param.ContainsKey("AFS_TERM_RATE");
                    _outcome = _supported ? null : "Termination is not configured or selectable.";
                    return requireConfigured ? _supported : _configurationSupported;
                }
            }
            catch (Exception) { return SetOutcome(target, "Support could not be verified. No termination sent."); }
            finally { lock (_gate) { if (ReferenceEquals(_target, target)) { _busy = false; } } }
        }

        private async Task<bool> DiscoverModesAsync(OutputController.NativeTarget target)
        {
            var clock = Stopwatch.StartNew();
            int count = 1;
            try
            {
                // Request each index: pinned ArduPilot emits only the first record for index zero.
                // Scan the complete list so a later conflicting TERMINATE cannot be accepted.
                for (int index = 1; index <= count; index++)
                {
                    var reply = new TaskCompletionSource<MAVLink.mavlink_available_modes_t?>(
                        TaskCreationOptions.RunContinuationsAsynchronously);
                    lock (_gate)
                    {
                        if (!ReferenceEquals(_target, target) || _disposed) { return false; }
                        _requestedIndex = index;
                        _discovery = reply;
                    }
                    int remaining = 8000 - (int)clock.ElapsedMilliseconds;
                    if (remaining <= 0) { return false; }
                    var request = target.SendAsync(MAVLink.MAV_CMD.REQUEST_MESSAGE, 435, index,
                        current: () => { lock (_gate) { return ReferenceEquals(_target, target); } });
                    if (await Task.WhenAny(request, Task.Delay(remaining)).ConfigureAwait(false) != request ||
                        !await request.ConfigureAwait(false)) { return false; }
                    remaining = Math.Min(2500, 8000 - (int)clock.ElapsedMilliseconds);
                    if (remaining <= 0 ||
                        await Task.WhenAny(reply.Task, Task.Delay(remaining)).ConfigureAwait(false) != reply.Task)
                    { return false; }
                    var record = await reply.Task.ConfigureAwait(false);
                    if (!record.HasValue || !target.IsCurrent) { return false; }
                    count = record.Value.number_modes;
                }
                lock (_gate)
                { return ReferenceEquals(_target, target) && _seenTermination && _mode.HasValue && !_ambiguous; }
            }
            finally
            {
                lock (_gate)
                {
                    if (ReferenceEquals(_target, target))
                    { _requestedIndex = _modeCount = 0; _discovery = null; }
                }
            }
        }

        // Captured before the confirmation dialog; no future aircraft/profile may inherit that confirmation.
        internal bool TryCaptureRequest(out long generation, out uint mode)
        {
            lock (_gate)
            {
                generation = _target?.Generation ?? -1;
                mode = _mode ?? 0;
                return !_busy && CanSend(_target, _mode);
            }
        }

        internal string TargetDescription
        {
            get { lock (_gate) { return _target == null ? "unavailable aircraft" :
                $"{_config.AircraftName} (system {_target.System}, component {_target.Component})"; } }
        }

        internal async Task<string> RequestAsync(long generation, uint mode)
        {
            OutputController.NativeTarget target;
            lock (_gate) { target = _target; }
            if (target == null || !target.IsCurrent) { return "Aircraft changed; no termination sent."; }
            if (target.Connection.giveComport) { return "Mission Planner is busy; no termination sent."; }
            lock (_gate)
            {
                if (_busy || !ReferenceEquals(_target, target) || target.Generation != generation || !CanSend(target, mode))
                { return "Aircraft, support or input changed; no termination sent."; }
                _busy = true;
                _outcome = "Termination request pending.";
            }
            try
            {
                bool accepted = await target.SendAsync(MAVLink.MAV_CMD.DO_SET_MODE,
                    (float)MAVLink.MAV_MODE_FLAG.CUSTOM_MODE_ENABLED, mode,
                    current: () => { lock (_gate) { return CanSend(target, mode); } }).ConfigureAwait(false);
                bool current = target.IsCurrent;
                string outcome = !current ? "Aircraft changed; original request outcome is uncertain. Do not replay." :
                    accepted ? "Command accepted; waiting for TERMINATE." :
                    "Termination request was not accepted.";
                SetOutcome(target, outcome);
                return outcome;
            }
            catch (Exception)
            {
                const string outcome = "Termination outcome is uncertain. Inspect aircraft; do not replay.";
                SetOutcome(target, outcome);
                return outcome;
            }
            finally { lock (_gate) { if (ReferenceEquals(_target, target)) { _busy = false; } } }
        }

        private bool SetOutcome(OutputController.NativeTarget target, string outcome)
        {
            lock (_gate) { if (ReferenceEquals(_target, target)) { _outcome = outcome; } }
            return false;
        }

        private void Invalidate()
        {
            OutputController.NativeTarget target;
            int[] subscriptions;
            lock (_gate)
            {
                target = _target;
                subscriptions = _subscriptions;
                _target = null;
                _mav = null;
                _subscriptions = Array.Empty<int>();
                _mode = _observedMode = null;
                _name = _outcome = null;
                _heartbeat = 0;
                _discovery?.TrySetResult(null);
                _discovery = null;
                _requestedIndex = _modeCount = 0;
                _armed = _supported = _configurationSupported = _selectable = _busy = _ambiguous = false;
                _seenTermination = false;
            }
            Unsubscribe(target, subscriptions);
        }

        private static void Unsubscribe(OutputController.NativeTarget target, int[] subscriptions)
        {
            foreach (int id in subscriptions) { target.Connection.UnSubscribeToPacketType(id); }
        }

        public void Dispose()
        {
            lock (_gate) { _disposed = true; }
            OutputController.ConnectionChanged -= Invalidate;
            Invalidate();
        }
    }
}
