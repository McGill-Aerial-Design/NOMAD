// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using MissionPlanner;
using MissionPlanner.Comms;

namespace NOMAD.MissionPlanner
{
    internal interface ISimulationConnection
    {
        bool CanStart(out string reason);
        bool IsConnected { get; }
        Task ConnectAsync(SimulationEndpoint endpoint, string directory, CancellationToken cancellation);
        void CloseTransport();
        void CloseOwned();
    }

    internal sealed class NativeSimulationConnection : ISimulationConnection
    {
        private sealed class Owned
        {
            internal readonly MAVLinkInterface Link = new MAVLinkInterface();
            internal readonly TcpClient Client = new TcpClient(AddressFamily.InterNetwork);
            internal TcpSerial Stream;
            internal BufferedStream Log, RawLog;
            internal volatile bool Registered;
        }
        private readonly Func<MAVLinkInterface> _selected;
        private readonly Func<IList<MAVLinkInterface>> _connections;
        private readonly Action<MAVLinkInterface> _select;
        private readonly Func<Action, Task> _ui;
        private Owned _owned;

        internal NativeSimulationConnection() : this(() => MainV2.comPort, () => MainV2.Comports,
            link => { MainV2.comPort = link; MainV2._connectionControl?.UpdateSysIDS(); }, OnUi) { }

        internal NativeSimulationConnection(Func<MAVLinkInterface> selected, Func<IList<MAVLinkInterface>> connections,
            Action<MAVLinkInterface> select, Func<Action, Task> ui)
        { _selected = selected; _connections = connections; _select = select; _ui = ui; }

        private static bool Owns(Owned owner) => owner?.Stream != null && ReferenceEquals(owner.Link.BaseStream, owner.Stream) &&
            ReferenceEquals(owner.Stream.client, owner.Client);

        public bool IsConnected
        {
            get
            {
                var owner = Volatile.Read(ref _owned);
                var socket = owner?.Client.Client;
                return owner?.Registered == true && Owns(owner) && ReferenceEquals(_selected(), owner.Link) && socket?.Connected == true;
            }
        }

        public bool CanStart(out string reason)
        {
            var link = _selected();
            if (link == null || link.ReadOnly || link.logreadmode || link.giveComport ||
                _connections().Any(candidate => candidate.BaseStream?.IsOpen == true))
            { reason = "Disconnect aircraft/replay connections and leave read-only mode before starting a simulation."; return false; }
            reason = ""; return true;
        }

        public async Task ConnectAsync(SimulationEndpoint endpoint, string directory, CancellationToken cancellation)
        {
            if (IsConnected) { return; }
            CloseOwned();
            if (!CanStart(out string reason)) { throw new InvalidOperationException(reason); }
            var original = _selected();
            var originalStream = original.BaseStream;
            var owner = new Owned();
            Volatile.Write(ref _owned, owner);
            try
            {
                var connect = owner.Client.ConnectAsync(endpoint.Host, endpoint.MavlinkPort);
                if (await Task.WhenAny(connect, Task.Delay(5000, cancellation)).ConfigureAwait(false) != connect)
                { cancellation.ThrowIfCancellationRequested(); throw new TimeoutException("Simulator TCP connection timed out."); }
                await connect.ConfigureAwait(false);
                cancellation.ThrowIfCancellationRequested();
                owner.Stream = new TcpSerial { Host = endpoint.Host.ToString(), Port = endpoint.MavlinkPort.ToString(),
                    client = owner.Client, autoReconnect = false };
                string logName = "native-" + Guid.NewGuid().ToString("N");
                owner.Log = new BufferedStream(File.Open(Path.Combine(directory, logName + ".tlog"), FileMode.CreateNew));
                owner.RawLog = new BufferedStream(File.Open(Path.Combine(directory, logName + ".rlog"), FileMode.CreateNew));
                owner.Link.BaseStream = owner.Stream;
                owner.Link.logfile = owner.Log; owner.Link.rawlogfile = owner.RawLog;
                DateTime handoff = DateTime.UtcNow;
                // The native handshake is private until verified. Registration hands reads to MP's existing reader.
                await Task.Run(() => { cancellation.ThrowIfCancellationRequested(); owner.Link.Open(false, true, false); })
                    .ConfigureAwait(false);
                cancellation.ThrowIfCancellationRequested();
                var packet = owner.Link.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.HEARTBEAT);
                var age = packet == null ? TimeSpan.MaxValue : DateTime.UtcNow - packet.rxtime;
                if (!Owns(owner) || !owner.Stream.IsOpen || packet == null || packet.sysid != endpoint.SystemId ||
                    packet.compid != 1 || owner.Link.sysidcurrent != endpoint.SystemId || owner.Link.compidcurrent != 1 ||
                    packet.rxtime.Kind != DateTimeKind.Utc || packet.rxtime < handoff ||
                    age < TimeSpan.Zero || age > TimeSpan.FromSeconds(5))
                { throw new InvalidOperationException("A fresh heartbeat from the owned simulator was not observed."); }
                var heartbeat = packet.ToStructure<MAVLink.mavlink_heartbeat_t>();
                if (heartbeat.autopilot != (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA || (heartbeat.base_mode & 128) != 0)
                { throw new InvalidOperationException("Expected a fresh, disarmed ArduPilot simulator."); }
                await _ui(() =>
                {
                    cancellation.ThrowIfCancellationRequested();
                    if (!ReferenceEquals(Volatile.Read(ref _owned), owner) || !ReferenceEquals(_selected(), original) ||
                        !ReferenceEquals(original.BaseStream, originalStream) || !CanStart(out reason))
                    { throw new InvalidOperationException("Mission Planner's connection changed; it was left untouched."); }
                    _connections().Add(owner.Link); owner.Registered = true;
                    _select(owner.Link);
                }).ConfigureAwait(false);
            }
            catch { CloseOwned(); throw; }
        }

        public void CloseTransport() { Volatile.Read(ref _owned)?.Client.Dispose(); }

        public void CloseOwned()
        {
            var owner = Interlocked.Exchange(ref _owned, null);
            if (owner == null) { return; }
            try
            {
                if (owner.Registered)
                {
                    _ui(() =>
                    {
                        if (!Owns(owner) || owner.Link.logreadmode || !ReferenceEquals(owner.Link.logfile, owner.Log) ||
                            !ReferenceEquals(owner.Link.rawlogfile, owner.RawLog)) { return; }
                        bool readOnly = owner.Link.ReadOnly;
                        owner.Link.Close();
                        _connections().Remove(owner.Link);
                        if (ReferenceEquals(_selected(), owner.Link))
                        {
                            // MP may already have disposed the old placeholder. Never restore it.
                            var empty = new MAVLinkInterface { BaseStream = new TcpSerial { autoReconnect = false },
                                ReadOnly = readOnly };
                            _connections().Add(empty); _select(empty);
                        }
                    }).GetAwaiter().GetResult();
                }
                else { owner.Link.Close(); }
            }
            finally { owner.Client.Dispose(); owner.Log?.Dispose(); owner.RawLog?.Dispose(); }
        }

        private static async Task OnUi(Action action)
        {
            var form = MainV2.instance;
            if (form == null || form.IsDisposed || !form.IsHandleCreated)
            { throw new InvalidOperationException("Mission Planner is closing."); }
            if (!form.InvokeRequired) { action(); return; }
            var done = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
            form.BeginInvoke((Action)(() => { if (done.Task.IsCompleted) { return; } try { action(); done.TrySetResult(true); }
                catch (Exception error) { done.TrySetException(error); } }));
            if (await Task.WhenAny(done.Task, Task.Delay(5000)).ConfigureAwait(false) != done.Task)
            { done.TrySetException(new TimeoutException("Mission Planner's UI did not accept the simulation connection operation.")); }
            await done.Task.ConfigureAwait(false);
        }
    }
}
