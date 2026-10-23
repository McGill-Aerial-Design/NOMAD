// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Comms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private sealed class FlightPeer : IDisposable
    {
        internal readonly NOMADConfig Config = new NOMADConfig { AircraftName = "Synthetic flight peer" };
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
        private int _count;
        internal float Timeout = 1;
        internal int Mutations;
        internal int Count => Volatile.Read(ref _count);
        internal MAVLink.mavlink_set_position_target_local_ned_t Last;

        internal FlightPeer()
        {
            string metadata = MissionPlanner.Utilities.Settings.GetDataDirectory();
            Check(Path.GetFileName(metadata.TrimEnd(Path.DirectorySeparatorChar)).StartsWith("NOMAD-VideoTests-"),
                "Synthetic flight metadata must never use the operator's host cache");
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
                if (!Connection.giveComport && _serial.BytesToRead > 0) { Connection.readPacket(); }
                else { Thread.Sleep(1); }
                Application.DoEvents();
                if (_failure != null) { throw new InvalidOperationException("Synthetic flight peer failed", _failure); }
            }
        }

        internal void Until(Func<bool> condition, string diagnostic)
        {
            var clock = Stopwatch.StartNew();
            while (!condition() && clock.ElapsedMilliseconds < 8000) { Pump(5); }
            Check(condition(), "Timed out: " + diagnostic);
        }
        internal bool Finish(Task<bool> task)
        { Until(() => task.IsCompleted, "flight support task"); return task.GetAwaiter().GetResult(); }
        internal void Finish(Task task)
        { Until(() => task.IsCompleted, "flight release task"); task.GetAwaiter().GetResult(); }

        private void Respond()
        {
            var parser = new MAVLink.MavlinkParse();
            while (!_stopping)
            {
                try
                {
                    var packet = parser.ReadPacket(_server.GetStream());
                    if (packet == null) { continue; }
                    if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_READ)
                    {
                        var request = packet.ToStructure<MAVLink.mavlink_param_request_read_t>();
                        string name = Encoding.ASCII.GetString(request.param_id).TrimEnd('\0');
                        float value = name == "GUID_TIMEOUT" ? Timeout : name == "GUID_OPTIONS" ? 0 :
                            name == "FRAME_CLASS" ? 1 : name == "MAV_GCS_SYSID" ? MAVLinkInterface.gcssysid : 0;
                        Send(MAVLink.MAVLINK_MSG_ID.PARAM_VALUE, new MAVLink.mavlink_param_value_t
                        {
                            param_id = Encoding.ASCII.GetBytes(name.PadRight(16, '\0')),
                            param_value = value, param_count = 5, param_index = 0, param_type = 9,
                        });
                    }
                    else if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.SET_POSITION_TARGET_LOCAL_NED)
                    {
                        Last = packet.ToStructure<MAVLink.mavlink_set_position_target_local_ned_t>();
                        Interlocked.Increment(ref _count);
                    }
                    else if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.PARAM_SET ||
                        packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.SET_MODE)
                    { Interlocked.Increment(ref Mutations); }
                    else if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG)
                    {
                        var command = packet.ToStructure<MAVLink.mavlink_command_long_t>();
                        if (command.command == (ushort)MAVLink.MAV_CMD.COMPONENT_ARM_DISARM ||
                            command.command == (ushort)MAVLink.MAV_CMD.DO_SET_MODE)
                        { Interlocked.Increment(ref Mutations); }
                    }
                }
                catch (IOException) { }
                catch (ObjectDisposedException) { }
                catch (SocketException) { }
                catch (Exception error) { _failure = error; return; }
            }
        }

        private void Send(MAVLink.MAVLINK_MSG_ID id, object payload)
        {
            lock (_gate)
            {
                byte[] bytes = _encoder.GenerateMAVLinkPacket20(id, payload, false, 201, 42, -1);
                _server.GetStream().Write(bytes, 0, bytes.Length);
            }
        }

        public void Dispose()
        {
            _stopping = true;
            _server.Close();
            Connection.Close();
            Check(_worker.Join(2000), "Synthetic flight peer failed to stop");
            _listener.Stop();
            OutputController.Shutdown();
            OutputController.CurrentConnection = _previousConnection;
        }
    }
}
