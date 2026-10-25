// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Comms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private sealed class TrackingPeer : IDisposable
    {
        internal readonly NOMADConfig Config = new NOMADConfig { AircraftName = "Synthetic tracking peer" };
        internal readonly MAVLinkInterface Connection = new MAVLinkInterface();
        private readonly TcpListener _listener = new TcpListener(IPAddress.Loopback, 0);
        private readonly TcpClient _server;
        private readonly TcpSerial _serial = new TcpSerial { autoReconnect = false };
        private readonly Thread _worker;
        private readonly MAVLink.MavlinkParse _encoder = new MAVLink.MavlinkParse();
        private readonly object _gate = new object();
        private readonly Func<MAVLinkInterface> _previousConnection;
        private volatile bool _stopping;
        private Exception _failure;
        private int _mountTargets;
        internal bool StreamAttitude = true;
        private float _heading;
        private long _attitudeTicks;
        internal int MountTargets => Volatile.Read(ref _mountTargets);

        internal TrackingPeer()
        {
            string metadata = MissionPlanner.Utilities.Settings.GetDataDirectory();
            Check(Path.GetFileName(metadata.TrimEnd(Path.DirectorySeparatorChar)).StartsWith("NOMAD-VideoTests-"),
                "Synthetic tracking metadata must never use the operator's host cache");
            Directory.CreateDirectory(metadata);
            foreach (string vehicle in new[] { "SITL", "AP_Periph", "ArduSub", "Rover", "ArduCopter",
                "ArduPlane", "AntennaTracker", "Blimp", "Heli" })
            {
                File.WriteAllText(Path.Combine(metadata, vehicle + ".apm.pdef.xml"),
                    "<paramfile><vehicles><parameters name=\"" + vehicle + "\"><param name=\"" + vehicle +
                    ":FLTMODE1\"><values><value code=\"4\">Guided</value><value code=\"9\">Land</value>" +
                    "</values></param></parameters></vehicles></paramfile>");
            }
            _previousConnection = OutputController.CurrentConnection;
            _listener.Start();
            _serial.client = new TcpClient("127.0.0.1", ((IPEndPoint)_listener.LocalEndpoint).Port);
            _server = _listener.AcceptTcpClient();
            _server.ReceiveTimeout = 100;
            Connection.BaseStream = _serial;
            Connection.sysidcurrent = 201; Connection.compidcurrent = 42;
            _worker = new Thread(Respond) { IsBackground = true };
            _worker.Start();
            OutputController.CurrentConnection = () => Connection;
            OutputController.Initialize(Config);
            GimbalController.ApplyProfile(Config);
            Heartbeat(4); Pump(80);
            Until(() => Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.ATTITUDE) != null,
                "initial selected attitude");
        }

        internal void Heartbeat(uint mode, bool armed = true) => Send(MAVLink.MAVLINK_MSG_ID.HEARTBEAT,
            new MAVLink.mavlink_heartbeat_t
            {
                autopilot = (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA, type = (byte)MAVLink.MAV_TYPE.QUADROTOR,
                base_mode = (byte)(armed ? 129 : 1), custom_mode = mode, mavlink_version = 3,
                system_status = (byte)MAVLink.MAV_STATE.ACTIVE,
            });

        internal void Pump(int milliseconds)
        {
            var clock = Stopwatch.StartNew();
            while (clock.ElapsedMilliseconds < milliseconds)
            {
                if (StreamAttitude && (Stopwatch.GetTimestamp() - _attitudeTicks) / (double)Stopwatch.Frequency > .1)
                { Attitude(_heading); }
                if (!Connection.giveComport && _serial.BytesToRead > 0) { Connection.readPacket(); }
                else { Thread.Sleep(1); }
                Application.DoEvents();
                if (_failure != null) { throw new InvalidOperationException("Synthetic tracking peer failed", _failure); }
            }
        }

        internal void Attitude(float radians, byte system = 201)
        {
            if (system == 201) { _heading = radians; _attitudeTicks = Stopwatch.GetTimestamp(); }
            Send(MAVLink.MAVLINK_MSG_ID.ATTITUDE,
                new MAVLink.mavlink_attitude_t { time_boot_ms = 1, yaw = radians }, system);
        }

        internal void Until(Func<bool> condition, string diagnostic)
        {
            var clock = Stopwatch.StartNew();
            while (!condition() && clock.ElapsedMilliseconds < 8000) { Pump(5); }
            Check(condition(), "Timed out: " + diagnostic);
        }

        private void Respond()
        {
            var parser = new MAVLink.MavlinkParse();
            while (!_stopping)
            {
                try
                {
                    var packet = parser.ReadPacket(_server.GetStream());
                    if (packet == null) { continue; }
                    if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.MOUNT_CONTROL)
                    { Interlocked.Increment(ref _mountTargets); }
                    else if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG)
                    {
                        var command = packet.ToStructure<MAVLink.mavlink_command_long_t>();
                        if (command.command == (ushort)MAVLink.MAV_CMD.DO_MOUNT_CONTROL)
                        { Interlocked.Increment(ref _mountTargets); }
                    }
                }
                catch (IOException) { }
                catch (ObjectDisposedException) { }
                catch (SocketException) { }
                catch (Exception error) { _failure = error; return; }
            }
        }

        private void Send(MAVLink.MAVLINK_MSG_ID id, object payload, byte system = 201)
        {
            lock (_gate)
            {
                byte[] bytes = _encoder.GenerateMAVLinkPacket20(id, payload, false, system, 42, -1);
                _server.GetStream().Write(bytes, 0, bytes.Length);
            }
        }

        public void Dispose()
        {
            _stopping = true;
            _server.Close();
            Connection.Close();
            Check(_worker.Join(2000), "Synthetic tracking peer failed to stop");
            _listener.Stop();
            OutputController.Shutdown();
            OutputController.CurrentConnection = _previousConnection;
        }
    }
}
