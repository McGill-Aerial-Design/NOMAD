// SPDX-License-Identifier: Apache-2.0
using System;
using System.Threading.Tasks;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    internal static partial class OutputController
    {
        internal sealed class NativeTarget
        {
            internal MAVLinkInterface Connection;
            internal object Stream;
            internal byte System, Component;
            internal long Generation;
            internal NOMADConfig Profile;

            internal bool IsCurrent
            {
                get
                {
                    RefreshConnection();
                    lock (Gate)
                    {
                        return Generation == _generation && ReferenceEquals(Profile, _config) &&
                            Connection == _connection && Stream == _stream && _open && _writable;
                    }
                }
            }

            internal async Task<bool> SendAsync(MAVLink.MAV_CMD command, float p1, float p2,
                float p3 = 0, float p4 = 0, Func<bool> current = null)
            {
                Task<bool> task;
                lock (Gate)
                {
                    if (!IsCurrent || current?.Invoke() == false)
                    {
                        throw new InvalidOperationException("Aircraft or input changed; no new command was sent.");
                    }
                    if (_payloadBusy || Connection.giveComport) { throw new InvalidOperationException("Mission Planner is busy with a native request."); }
                    _payloadBusy = true;
                    try
                    {
                        task = Connection.doCommandAsync(System, Component, command, p1, p2, p3, p4, 0, 0, 0, true);
                    }
                    catch { _payloadBusy = false; throw; }
                }
                try { return await task.ConfigureAwait(false); }
                finally { lock (Gate) { _payloadBusy = false; } }
            }

            internal async Task<float> ReadTrimAsync(int channel)
            {
                Task<float> task;
                lock (Gate)
                {
                    if (!IsCurrent || _payloadBusy || Connection.giveComport)
                    { throw new InvalidOperationException("Aircraft changed or Mission Planner payload request is busy."); }
                    _payloadBusy = true;
                    try { task = Connection.GetParamAsync(System, Component, "SERVO" + channel + "_TRIM"); }
                    catch { _payloadBusy = false; throw; }
                }
                try { return await task.ConfigureAwait(false); }
                finally { lock (Gate) { _payloadBusy = false; } }
            }
        }

        internal static NativeTarget CaptureTarget(NOMADConfig config)
        {
            RefreshConnection();
            lock (Gate)
            {
                if (!ReferenceEquals(config, _config) || _config == null || !_open || !_writable ||
                    _system < 1 || _system > 255 || _component < 1 || _component > 255)
                { return null; }
                return new NativeTarget { Connection = _connection, Stream = _stream,
                    System = (byte)_system, Component = (byte)_component, Generation = _generation, Profile = config };
            }
        }
    }
}
