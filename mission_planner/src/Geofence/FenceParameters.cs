// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    internal sealed class FenceParameter
    {
        internal string Name, Units, DisplayName, Description;
        internal float Value;
        internal double Minimum, Maximum;
        internal bool HasRange;
        internal List<KeyValuePair<int, string>> Options, Bits;
        internal bool Allows(float value)
        {
            if (float.IsNaN(value) || float.IsInfinity(value)) { return false; }
            if (Options.Count > 0) { return Options.Any(option => option.Key == value); }
            if (Bits.Count > 0)
            {
                int mask = Bits.Aggregate(0, (all, bit) => all | (1 << bit.Key));
                return value >= 0 && value <= 2147483647d && value == Math.Truncate(value) && ((int)value & ~mask) == 0;
            }
            return HasRange && !double.IsNaN(Minimum) && !double.IsNaN(Maximum) &&
                !double.IsInfinity(Minimum) && !double.IsInfinity(Maximum) && value >= Minimum && value <= Maximum;
        }
    }

    // One read/confirmation session, bound to one native aircraft and a fresh disarmed heartbeat.
    internal sealed class FenceParameters : IDisposable
    {
        internal static readonly string[] Names = { "FENCE_ACTION", "FENCE_ALT_MAX", "FENCE_ALT_MIN",
            "FENCE_MARGIN", "FENCE_RADIUS", "FENCE_TYPE", "FENCE_ENABLE" };
        private readonly object _gate = new object();
        private readonly OutputController.NativeTarget _target;
        private readonly int _subscription;
        private long _heartbeat;
        private bool _armed = true, _disposed, _requiresRead;
        internal readonly List<FenceParameter> Parameters = new List<FenceParameter>();
        internal string TargetDescription => $"{_target.Profile.AircraftName} (system {_target.System}, component {_target.Component})";

        internal FenceParameters(NOMADConfig config)
        {
            _target = OutputController.CaptureTarget(config) ??
                throw new InvalidOperationException("Select a live writable aircraft before reading fence settings.");
            _subscription = _target.Connection.SubscribeToPacketType(MAVLink.MAVLINK_MSG_ID.HEARTBEAT,
                Observe, _target.System, _target.Component);
        }

        private bool Observe(MAVLink.MAVLinkMessage packet)
        {
            var heartbeat = packet.ToStructure<MAVLink.mavlink_heartbeat_t>();
            double age = (DateTime.UtcNow - packet.rxtime).TotalSeconds;
            if (heartbeat.autopilot != (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA ||
                packet.rxtime.Kind != DateTimeKind.Utc || age < 0 || age > 3.5) { return true; }
            lock (_gate)
            {
                if (_disposed) { return true; }
                _heartbeat = Stopwatch.GetTimestamp() - (long)(age * Stopwatch.Frequency);
                _armed = (heartbeat.base_mode & (byte)MAVLink.MAV_MODE_FLAG.SAFETY_ARMED) != 0;
            }
            return true;
        }

        internal bool CanApply
        {
            get
            {
                if (!_target.IsCurrent) { return false; }
                lock (_gate)
                {
                    return !_disposed && !_requiresRead && !_armed && _heartbeat != 0 &&
                        (Stopwatch.GetTimestamp() - _heartbeat) / (double)Stopwatch.Frequency <= 3.5;
                }
            }
        }

        internal async Task ReadAsync()
        {
            Parameters.Clear();
            var mav = _target.Connection.MAVlist[_target.System, _target.Component];
            string firmware = mav.cs.firmware.ToString();
            foreach (string name in Names)
            {
                if (!mav.param.ContainsKey(name)) { continue; }
                float value = await _target.ReadParameterAsync(name).ConfigureAwait(false);
                if (!_target.IsCurrent) { throw new InvalidOperationException("Aircraft changed; read again."); }
                var parameter = new FenceParameter
                {
                    Name = name, Value = value,
                    DisplayName = ParameterMetaDataRepository.GetParameterMetaData(name, "DisplayName", firmware),
                    Description = ParameterMetaDataRepository.GetParameterMetaData(name, "Description", firmware),
                    Units = ParameterMetaDataRepository.GetParameterMetaData(name, "Units", firmware),
                    Options = ParameterMetaDataRepository.GetParameterOptionsInt(name, firmware),
                    Bits = ParameterMetaDataRepository.GetParameterBitMaskInt(name, firmware)
                        .Where(bit => bit.Key >= 0 && bit.Key <= 30).ToList(),
                };
                parameter.HasRange = ParameterMetaDataRepository.GetParameterRange(name,
                    ref parameter.Minimum, ref parameter.Maximum, firmware);
                Parameters.Add(parameter);
            }
        }

        internal async Task<string> ApplyAsync(IDictionary<string, float> changes, Func<bool> active)
        {
            int verified = 0;
            string pending = "none";
            try
            {
                if (changes.Count == 0) { return "No fence settings changed."; }
                if (changes.Keys.Any(name => !Parameters.Any(p => p.Name == name && p.Allows(changes[name]))))
                { return "Invalid or unsupported fence setting; no parameters were sent."; }
                float? Candidate(string name) => changes.TryGetValue(name, out float value) ? value :
                    Parameters.Where(p => p.Name == name).Select(p => (float?)p.Value).FirstOrDefault();
                if ((changes.ContainsKey("FENCE_ALT_MIN") || changes.ContainsKey("FENCE_ALT_MAX")) &&
                    Candidate("FENCE_ALT_MIN") >= Candidate("FENCE_ALT_MAX"))
                { return "Minimum altitude must be below maximum altitude; no parameters were sent."; }
                if ((changes.ContainsKey("FENCE_MARGIN") || changes.ContainsKey("FENCE_RADIUS")) &&
                    Candidate("FENCE_MARGIN") >= Candidate("FENCE_RADIUS"))
                { return "Fence margin must be below fence radius; no parameters were sent."; }
                var ordered = Parameters.Where(p => changes.ContainsKey(p.Name))
                    .OrderBy(p => p.Name == "FENCE_ENABLE" && changes[p.Name] == 0 ? 0 : 1).ToList();
                foreach (var parameter in ordered)
                {
                    pending = parameter.Name;
                    if (!CanApply || active?.Invoke() != true) { throw new InvalidOperationException("Fresh disarmed aircraft required."); }
                    float before = await _target.ReadParameterAsync(parameter.Name).ConfigureAwait(false);
                    if (before != parameter.Value) { throw new InvalidOperationException("Aircraft setting changed; read again."); }
                    bool accepted = await _target.WriteParameterAsync(parameter.Name, changes[parameter.Name],
                        () => CanApply && active?.Invoke() == true).ConfigureAwait(false);
                    if (!accepted) { throw new InvalidOperationException("Mission Planner did not accept the setting."); }
                    float observed = await _target.ReadParameterAsync(parameter.Name).ConfigureAwait(false);
                    if (!CanApply || active?.Invoke() != true || observed != changes[parameter.Name])
                    { throw new InvalidOperationException("Requested value was not verified on the current disarmed aircraft."); }
                    parameter.Value = observed;
                    verified++;
                }
                return $"Verified {verified} fence setting(s) by native readback. Review aircraft behavior before flight.";
            }
            catch (Exception error)
            {
                lock (_gate) { _requiresRead = true; }
                return $"Stopped at {pending}; {verified} setting(s) verified. " + error.Message +
                    " The current setting may have changed; read again before retrying. No rollback was attempted.";
            }
        }

        public void Dispose()
        {
            lock (_gate) { if (_disposed) { return; } _disposed = true; }
            _target.Connection.UnSubscribeToPacketType(_subscription);
        }
    }
}
