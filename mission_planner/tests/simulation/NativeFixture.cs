// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using System.Web.Script.Serialization;
using MissionPlanner;
using MissionPlanner.Comms;
using NOMAD.MissionPlanner;

namespace NOMAD.Simulation
{
    // Controlled native-library integration. Does not construct MainV2 or run plugin startup.
    internal sealed partial class NativeFixture : IDisposable
    {
        private readonly MAVLinkInterface _connection = new MAVLinkInterface();
        private readonly TcpSerial _tcp = new TcpSerial { Host = "127.0.0.1", Port = "15760", autoReconnect = false };
        private readonly Stopwatch _clock = Stopwatch.StartNew();
        private readonly object _gate = new object();
        private readonly List<Pwm> _outputs = new List<Pwm>();
        private readonly Func<MAVLinkInterface> _previous = OutputController.CurrentConnection;
        private readonly List<object> _cases = new List<object>();
        private Thread _reader;
        private volatile bool _stopping;
        private Exception _readerError;
        private int _mountPackets;
        private NOMADConfig _config;
        private bool _disposed;

        private sealed class Pwm
        {
            internal double At;
            internal ushort Roll, Pitch, Yaw;
        }

        internal static void Run(string run, bool rendered, bool detect, bool occlusion)
        {
            var result = new Dictionary<string, object> { { "schema", 1 },
                { "cohort", "controlled-native-api" }, { "full_mp_lifecycle", "not_run" },
                { "passed", false }, { "stage", "connect" }, { "physical_outcome", "not_tested" } };
            NativeFixture fixture = null;
            try
            {
                fixture = new NativeFixture();
                fixture.Connect();
                result["stage"] = "mount_output";
                fixture.CheckMount();
                result["stage"] = "admission";
                fixture.CheckAdmission();
                if (rendered)
                {
                    result["stage"] = "video";
                    result["capture_latency"] = "unavailable: no capture clock is carried by the product pipeline";
                    fixture.CheckVideo(run, detect, occlusion);
                }
                result["passed"] = true;
            }
            catch (Exception error)
            {
                result["failure"] = error.Message;
                result["error_type"] = error.GetType().Name;
            }
            finally
            {
                if (fixture != null)
                {
                    result["cases"] = fixture._cases;
                    try { fixture.Dispose(); }
                    catch (Exception error)
                    { result["cleanup_error"] = error.GetType().Name; result["passed"] = false; }
                    result["reader_exited"] = fixture._reader == null || !fixture._reader.IsAlive;
                    result["passed"] = (bool)result["passed"] && (bool)result["reader_exited"];
                }
                File.WriteAllText(Path.Combine(run, "native.json"), new JavaScriptSerializer().Serialize(result));
            }
            if (!(bool)result["passed"]) { throw new InvalidOperationException("Controlled native API case failed; see native.json."); }
            Console.WriteLine("PASS actual MP APIs and SITL mount output; full MP lifecycle not run");
        }

        private void Connect()
        {
            _tcp.client = new TcpClient(AddressFamily.InterNetwork);
            var connect = _tcp.client.ConnectAsync(IPAddress.Loopback, 15760);
            Require(connect.Wait(5000), "loopback_connect_timeout");
            connect.GetAwaiter().GetResult();
            _connection.BaseStream = _tcp;
            _connection.CONNECT_TIMEOUT_SECONDS = 10;
            _connection.OnPacketReceived += Received;
            _connection.OnPacketSent += Sent;
            var open = Task.Run(() => _connection.Open(false, true, false));
            Require(open.Wait(15000), "native_open_timeout");
            open.GetAwaiter().GetResult();
            _cases.Add(new { name = "observed_identity", system = _connection.sysidcurrent,
                component = _connection.compidcurrent });
            Require(_connection.sysidcurrent == 201 && _connection.compidcurrent == 1,
                "unexpected_simulator_identity");
            Require(!_connection.MAV.cs.armed, "simulator_must_be_disarmed");
            _reader = new Thread(Read) { IsBackground = true, Name = "NOMAD simulation native MP reader" };
            _reader.Start();
            OutputController.CurrentConnection = () => _connection;
            _config = new NOMADConfig { AircraftName = "Owned simulation", AudioAlerts = false,
                AltitudeCallouts = false, AutoStartHudVideo = false };
            OutputController.Initialize(_config);
            GimbalController.ApplyProfile(_config);
            // This headless cohort has no MainV2 stream-rate loop; request fresh control inputs explicitly.
            foreach (var stream in new[] { new[] { 36, 100000 }, new[] { 30, 20000 } })
            {
                var request = _connection.doCommandAsync(201, 1, MAVLink.MAV_CMD.SET_MESSAGE_INTERVAL,
                    stream[0], stream[1], 0, 0, 0, 0, 0, true);
                Require(request.Wait(5000) && request.GetAwaiter().GetResult(), "control_stream_request_rejected: " + stream[0]);
            }
            WaitOutput(0, value => value.Roll > 0 && value.Pitch > 0 && value.Yaw > 0);
        }

        private void Read()
        {
            try
            {
                while (!_stopping)
                {
                    if (!_connection.giveComport && _tcp.BytesToRead > 0) { _connection.readPacket(); }
                    else { Thread.Sleep(2); }
                }
            }
            catch (Exception error) { if (!_stopping) { _readerError = error; } }
        }

        private void Received(object sender, MAVLink.MAVLinkMessage packet)
        {
            if (packet.sysid != 201 || packet.compid != 1 || packet.msgid != 36) { return; }
            var value = packet.ToStructure<MAVLink.mavlink_servo_output_raw_t>();
            lock (_gate)
            {
                if (_outputs.Count == 256) { _outputs.RemoveAt(0); }
                _outputs.Add(new Pwm { At = _clock.Elapsed.TotalSeconds,
                    Roll = value.servo9_raw, Pitch = value.servo10_raw, Yaw = value.servo11_raw });
            }
        }

        private void Sent(object sender, MAVLink.MAVLinkMessage packet)
        {
            if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.MOUNT_CONTROL)
            { Interlocked.Increment(ref _mountPackets); }
            if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG)
            {
                var value = packet.ToStructure<MAVLink.mavlink_command_long_t>();
                if (value.command == (ushort)MAVLink.MAV_CMD.DO_MOUNT_CONFIGURE ||
                    value.command == (ushort)MAVLink.MAV_CMD.DO_MOUNT_CONTROL)
                { Interlocked.Increment(ref _mountPackets); }
            }
        }

        private void CheckMount()
        {
            Require(Complete(GimbalController.RequestPitchRollTargetAsync(0, 0)).Submitted, "neutral_not_submitted");
            var neutral = WaitOutput(_clock.Elapsed.TotalSeconds, value =>
                Math.Abs(value.Roll - 1500) <= 3 && Math.Abs(value.Pitch - 1700) <= 3 &&
                Math.Abs(value.Yaw - 1500) <= 3);
            double start = _clock.Elapsed.TotalSeconds;
            Require(Complete(GimbalController.RequestPitchRollTargetAsync(-30, 15)).Submitted, "step_not_submitted");
            var moved = WaitOutput(start, value => value.Roll - neutral.Roll > 100 && neutral.Pitch - value.Pitch > 80);
            _cases.Add(new { name = "native_pitch_roll", elapsed_seconds = moved.At - start,
                roll_pwm = moved.Roll, pitch_pwm = moved.Pitch, submitted = true, measured_pose = false });
            start = _clock.Elapsed.TotalSeconds;
            Require(Complete(GimbalController.RequestTrackingTargetAsync(-30, 25, OutputController.Generation,
                GimbalController.ManualRevision, () => true)).Submitted, "yaw_not_submitted");
            var yaw = WaitOutput(start, value => value.Yaw - neutral.Yaw > 40 && Math.Abs(value.Roll - 1500) <= 3);
            _cases.Add(new { name = "native_tracking_yaw", elapsed_seconds = yaw.At - start,
                yaw_pwm = yaw.Yaw, roll_pwm = yaw.Roll, submitted = true, measured_pose = false });
        }

        private void CheckAdmission()
        {
            long retired = OutputController.Generation;
            OutputController.Initialize(_config);
            int before = Volatile.Read(ref _mountPackets);
            var stale = Complete(OutputController.SendGimbalTargetAsync(2, -20, 0, 0, retired, () => true));
            Require(stale.Outcome == GimbalRequestOutcome.NotSent, "retired_generation_was_admitted");
            _connection.ReadOnly = true;
            try
            {
                var readOnly = Complete(GimbalController.RequestPitchRollTargetAsync(-20, 10));
                Require(readOnly.Outcome == GimbalRequestOutcome.NotSent, "readonly_was_admitted");
                Thread.Sleep(200);
                Require(before == Volatile.Read(ref _mountPackets), "rejected_input_emitted_mount_packet");
            }
            finally { _connection.ReadOnly = false; OutputController.RefreshConnection(); }
            _cases.Add(new { name = "retired_readonly", new_mount_packets = 0 });
        }

        private Pwm WaitOutput(double after, Func<Pwm, bool> predicate)
        {
            double end = _clock.Elapsed.TotalSeconds + 5;
            while (_clock.Elapsed.TotalSeconds < end)
            {
                if (_readerError != null) { throw new IOException("native_reader_failed", _readerError); }
                lock (_gate) { foreach (var value in _outputs) { if (value.At > after && predicate(value)) { return value; } } }
                Thread.Sleep(5);
            }
            lock (_gate)
            {
                var last = _outputs.Count == 0 ? null : _outputs[_outputs.Count - 1];
                throw new TimeoutException("independent_servo_output_deadline; samples=" + _outputs.Count +
                    "; last=" + (last == null ? "none" : last.Roll + "," + last.Pitch + "," + last.Yaw) +
                    "; selected=" + _connection.sysidcurrent + "/" + _connection.compidcurrent);
            }
        }

        private static GimbalRequestResult Complete(Task<GimbalRequestResult> task)
        {
            Require(task.Wait(5000), "native_mount_helper_timeout");
            return task.GetAwaiter().GetResult();
        }

        private static void Require(bool condition, string diagnostic)
        { if (!condition) { throw new InvalidOperationException(diagnostic); } }

        public void Dispose()
        {
            if (_disposed) { return; }
            _disposed = true;
            OutputController.Shutdown();
            _stopping = true;
            _tcp.client?.Close();
            if (_reader != null) { _reader.Join(3000); }
            _connection.Close();
            OutputController.CurrentConnection = _previous;
        }
    }
}
