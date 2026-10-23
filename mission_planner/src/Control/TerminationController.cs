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
        private TaskCompletionSource<bool> _discovery;
        private long _heartbeat;
        private bool _armed, _supported, _selectable, _busy, _disposed, _ambiguous;

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
                    var mode = packet.ToStructure<MAVLink.mavlink_available_modes_t>();
                    float encodedMode = mode.custom_mode;
                    string name = Encoding.UTF8.GetString(mode.mode_name).TrimEnd('\0');
                    if (!string.Equals(name, "TERMINATE", StringComparison.OrdinalIgnoreCase)) { return true; }
                    _selectable = (mode.properties & 1U) == 0 && mode.custom_mode <= byte.MaxValue &&
                        (double)encodedMode == mode.custom_mode;
                    if (!_selectable) { _supported = false; }
                    if (mode.custom_mode > byte.MaxValue || (double)encodedMode != mode.custom_mode)
                    {
                        _discovery?.TrySetResult(false);
                        return true;
                    }
                    _ambiguous |= _mode.HasValue && _mode.Value != mode.custom_mode;
                    _mode = mode.custom_mode;
                    _name = name;
                    _discovery?.TrySetResult(_selectable && !_ambiguous);
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
                    { return "TERMINATE observed; physical outcome unverified. Reboot required to leave."; }
                    if (_supported && !_armed) { return "Native TERMINATE supported; aircraft is not armed."; }
                    return _outcome ?? (_supported ? "Native TERMINATE supported; entry checks remain onboard."
                        : "Check native TERMINATE support.");
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

        internal async Task<bool> CheckAsync()
        {
            UpdateBinding();
            OutputController.NativeTarget target;
            TaskCompletionSource<bool> discovery;
            lock (_gate)
            {
                if (_disposed || _target == null || _busy) { return false; }
                target = _target;
                _busy = true;
                _supported = false;
                _selectable = false;
                discovery = _discovery = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
                _outcome = "Checking native support...";
            }
            try
            {
                // Index zero requests all modes. Their records may arrive before or after the request ACK.
                if (!await target.SendAsync(MAVLink.MAV_CMD.REQUEST_MESSAGE, 435, 0).ConfigureAwait(false))
                { return SetOutcome(target, "Native mode discovery was not accepted."); }
                float enabled = await target.ReadParameterAsync("AFS_ENABLE").ConfigureAwait(false);
                float action = await target.ReadParameterAsync("AFS_TERM_ACTION").ConfigureAwait(false);
                float rate = await target.ReadParameterAsync("AFS_TERM_RATE").ConfigureAwait(false);
                var completed = await Task.WhenAny(discovery.Task, Task.Delay(2500)).ConfigureAwait(false);
                bool advertised = completed == discovery.Task && await discovery.Task.ConfigureAwait(false);
                bool current = target.IsCurrent;
                lock (_gate)
                {
                    if (!current || !ReferenceEquals(_target, target)) { return false; }
                    _supported = advertised && _selectable && !_ambiguous && enabled == 1 && action == 44 &&
                        !float.IsNaN(rate) && !float.IsInfinity(rate) && rate >= 2;
                    _outcome = _supported ? null : "Native controlled termination is not configured or selectable.";
                    return _supported;
                }
            }
            catch (Exception) { return SetOutcome(target, "Native support could not be verified. No termination sent."); }
            finally { lock (_gate) { if (ReferenceEquals(_target, target)) { _busy = false; } } }
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
                _outcome = "Termination request pending; physical outcome unverified.";
            }
            try
            {
                bool accepted = await target.SendAsync(MAVLink.MAV_CMD.DO_SET_MODE,
                    (float)MAVLink.MAV_MODE_FLAG.CUSTOM_MODE_ENABLED, mode,
                    current: () => { lock (_gate) { return CanSend(target, mode); } }).ConfigureAwait(false);
                bool current = target.IsCurrent;
                string outcome = !current ? "Aircraft changed; original request outcome is uncertain. Do not replay." :
                    accepted ? "Command ACK accepted; await a fresh TERMINATE heartbeat. Physical outcome unverified." :
                    "Native termination request was not accepted.";
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
                _discovery?.TrySetResult(false);
                _discovery = null;
                _armed = _supported = _selectable = _busy = _ambiguous = false;
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
