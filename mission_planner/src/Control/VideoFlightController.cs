// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    internal sealed class VideoFlightController
    {
        private readonly OutputController.NativeTarget _target;
        private readonly MAVState _mav;
        private readonly Func<bool> _inputCurrent;
        private readonly VideoFlightInput _input;
        private Task _pending = Task.CompletedTask;
        private Task _stopTask;
        private readonly object _stopGate = new object();
        private int _sending, _stopped;
        private readonly float[] _parameters = new float[5];
        private static readonly string[] ParameterNames =
            { "GUID_TIMEOUT", "GUID_OPTIONS", "FRAME_CLASS", "MAV_GCS_SYSID", "MAV_GCS_SYSID_HI" };
        internal string Status { get; private set; } = "Checking flight support…";
        internal int Submitted { get; private set; }
        internal bool Active => Volatile.Read(ref _stopped) == 0 && _input.IsOwner;

        internal VideoFlightController(NOMADConfig config, VideoFlightInput input, Func<bool> inputCurrent)
        {
            _target = OutputController.CaptureTarget(config);
            _mav = _target?.Connection.MAV;
            _input = input;
            _inputCurrent = inputCurrent;
        }

        internal async Task<bool> PrepareAsync()
        {
            if (!AircraftCurrent())
            { Status = "Fly needs an airborne Copter in GUIDED with native joystick off."; return false; }
            try
            {
                for (int i = 0; i < ParameterNames.Length; i++)
                {
                    _parameters[i] = await _target.ReadParameterAsync(ParameterNames[i]).ConfigureAwait(false);
                    if (!AircraftCurrent()) { return false; }
                }
                if (!ParametersCurrent())
                { Status = "Check GUID_TIMEOUT (0.1–3 s), GUIDED options and GCS ID."; return false; }
                Status = "Click video to capture; Escape releases controls.";
                return true;
            }
            catch (Exception) { Status = "Flight support could not be verified; no flight input sent."; return false; }
        }

        internal bool AircraftCurrent(bool releasing = false)
        {
            if (_target == null || _mav == null || !_target.IsCurrent ||
                !ReferenceEquals(_target.Connection.MAV, _mav) || (!releasing && MainV2.joystick?.enabled == true) ||
                _mav.cs.firmware.ToString() != "ArduCopter2") { return false; }
            var packet = _mav.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.HEARTBEAT);
            if (packet == null || packet.sysid != _target.System || packet.compid != _target.Component ||
                packet.rxtime.Kind != DateTimeKind.Utc) { return false; }
            double age = (DateTime.UtcNow - packet.rxtime).TotalSeconds;
            var heartbeat = packet.ToStructure<MAVLink.mavlink_heartbeat_t>();
            return age >= 0 && age <= 3.5 &&
                heartbeat.autopilot == (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA && CopterType(heartbeat.type) &&
                heartbeat.system_status == (byte)MAVLink.MAV_STATE.ACTIVE &&
                (heartbeat.base_mode & (byte)MAVLink.MAV_MODE_FLAG.SAFETY_ARMED) != 0 &&
                (heartbeat.base_mode & (byte)MAVLink.MAV_MODE_FLAG.CUSTOM_MODE_ENABLED) != 0 && heartbeat.custom_mode == 4;
        }

        private static bool CopterType(byte type) => type == (byte)MAVLink.MAV_TYPE.QUADROTOR ||
            type == (byte)MAVLink.MAV_TYPE.HEXAROTOR || type == (byte)MAVLink.MAV_TYPE.OCTOROTOR ||
            type == (byte)MAVLink.MAV_TYPE.TRICOPTER || type == (byte)MAVLink.MAV_TYPE.DODECAROTOR ||
            type == (byte)MAVLink.MAV_TYPE.DECAROTOR;

        private bool ParametersCurrent()
        {
            for (int i = 0; i < ParameterNames.Length; i++)
            {
                if (!_mav.param.ContainsKey(ParameterNames[i]) ||
                    (float)_mav.param[ParameterNames[i]] != _parameters[i] || !Finite(_parameters[i])) { return false; }
            }
            if (_mav.param.ContainsKey("Q_ENABLE") && (float)_mav.param["Q_ENABLE"] != 0) { return false; }
            float options = _parameters[1], frame = _parameters[2], first = _parameters[3], last = _parameters[4];
            if (last < 0 || last > 255 || last != (int)last) { return false; }
            if (last <= first) { last = first; }
            return _parameters[0] >= .1f && _parameters[0] <= 3 && frame > 0 && frame == (int)frame &&
                frame != 6 && frame != 11 && frame != 13 &&
                options >= 0 && options <= 255 && options == (int)options && ((int)options & 180) == 0 &&
                first >= 1 && first <= 255 && first == (int)first && last >= first && last <= 255 && last == (int)last &&
                MAVLinkInterface.gcssysid >= first && MAVLinkInterface.gcssysid <= last;
        }

        internal void Sample(VideoFlightVelocity velocity)
        {
            if (!Active || Interlocked.CompareExchange(ref _sending, 1, 0) != 0) { return; }
            long sampled = Stopwatch.GetTimestamp();
            _pending = Task.Run(() =>
            {
                try
                {
                    bool Current() => Active && Fresh(sampled, .25) && _inputCurrent() &&
                        AircraftCurrent() && ParametersCurrent();
                    if (!OutputController.SendVideoVelocity(_target, velocity, Current))
                    { Interlocked.Exchange(ref _stopped, 1); Status = "Flight input retired; click again after checking aircraft."; }
                    else { Submitted++; Status = "Captured · Esc to release"; }
                }
                catch (Exception)
                { Interlocked.Exchange(ref _stopped, 1); Status = "Flight input uncertain; native GUIDED timeout applies."; }
                finally { Interlocked.Exchange(ref _sending, 0); }
            });
        }

        internal Task StopAsync()
        {
            Interlocked.Exchange(ref _stopped, 1);
            lock (_stopGate)
            { return _stopTask ?? (_stopTask = Task.Run(() => StopAfterPendingAsync(_pending))); }
        }

        private async Task StopAfterPendingAsync(Task pending)
        {
            await pending.ConfigureAwait(false);
            try
            {
                bool sent = OutputController.SendVideoVelocity(_target, default,
                    () => _input.IsOwner && AircraftCurrent(releasing: true));
                Status = sent ? "Zero input sent; stopping is not confirmed." : "Released; native GUIDED timeout applies.";
            }
            catch (Exception) { Status = "Released; zero input uncertain. Native GUIDED timeout applies."; }
            finally { _input.Release(); }
        }

        internal static bool Fresh(long ticks, double seconds) => ticks > 0 &&
            Stopwatch.GetTimestamp() >= ticks && (Stopwatch.GetTimestamp() - ticks) / (double)Stopwatch.Frequency < seconds;
        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
