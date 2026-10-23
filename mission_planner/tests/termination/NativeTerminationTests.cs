// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Comms;
using MissionPlanner.Utilities;
using NOMAD.MissionPlanner;

internal static class NativeTerminationTests
{
    private static string HostRoot;
    private const BindingFlags HiddenStatic = BindingFlags.Static | BindingFlags.NonPublic;
    private const BindingFlags HiddenInstance = BindingFlags.Instance | BindingFlags.NonPublic;
    private static Type Output, Controller;
    private static string LastFault;

    [STAThread]
    private static int Main(string[] args)
    {
        HostRoot = args[0];
        AppDomain.CurrentDomain.FirstChanceException += (sender, args2) =>
        {
            if (args2.Exception is InvalidOperationException || args2.Exception is TimeoutException)
            { LastFault = args2.Exception.GetType().Name + ": " + args2.Exception.Message + " at " +
                new StackTrace(args2.Exception).GetFrame(0)?.GetMethod()?.Name; }
        };
        AppDomain.CurrentDomain.AssemblyResolve += (sender, request) =>
        {
            string name = new AssemblyName(request.Name).Name;
            string path = Path.Combine(HostRoot, name == "MissionPlanner" ? name + ".exe" : name + ".dll");
            return File.Exists(path) ? Assembly.LoadFrom(path) : null;
        };
        try { Run(args[1]); return 0; }
        catch (Exception error) { Console.Error.WriteLine(error); return 1; }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Run(string directory)
    {
        Output = typeof(NOMADConfig).Assembly.GetType("NOMAD.MissionPlanner.OutputController", true);
        Controller = typeof(NOMADConfig).Assembly.GetType("NOMAD.MissionPlanner.TerminationController", true);
        Settings.CustomUserDataDirectory = directory;
        Settings.AppConfigName = "NOMAD-Termination-Test-" + Guid.NewGuid().ToString("N");
        Settings.FileName = Path.Combine(directory, "host.xml");
        string metadata = Settings.GetDataDirectory();
        Directory.CreateDirectory(metadata);
        // Native mode decoding must use synthetic metadata, never download into the operator's shared cache.
        foreach (string vehicle in new[] { "SITL", "AP_Periph", "ArduSub", "Rover", "ArduCopter",
            "ArduPlane", "AntennaTracker", "Blimp", "Heli" })
        {
            File.WriteAllText(Path.Combine(metadata, vehicle + ".apm.pdef.xml"),
                "<paramfile><vehicles><parameters name=\"" + vehicle + "\"><param name=\"" + vehicle +
                ":FLTMODE1\"><values><value code=\"0\">Stabilize</value><value code=\"9\">Land</value>" +
                "</values></param></parameters></vehicles></paramfile>");
        }
        var log = typeof(NOMADConfig).Assembly.GetType("NOMAD.MissionPlanner.Log", true);
        log.GetField("_logPath", HiddenStatic).SetValue(null, Path.Combine(directory, "nomad.log"));
        try
        {
            using (var peer = new Peer())
            {
                peer.CheckSupport();
                Require(peer.CanRequest, "Valid advertised native contract was unavailable");
                peer.Announce = false;
                Require(!peer.CheckSupport() && !peer.CanRequest && !peer.Status.Contains("supported"),
                    "Missing fresh advertisement reused previous eligibility or claimed support");
                peer.Announce = true;
                peer.AfterAck = true;
                bool afterAck = peer.CheckSupport();
                Require(afterAck && peer.CanRequest, "Advertisement after ACK was lost: " + afterAck + "; " + peer.Status + "; " + LastFault +
                    "; hostbusy=" + peer.Connection.giveComport + "; payloadbusy=" + Output.GetField("_payloadBusy", HiddenStatic).GetValue(null));
                peer.Selectable = false;
                peer.SendMode(); peer.Pump(100);
                Require(!peer.CanRequest, "Later nonselectable advertisement retained activation eligibility");
                Require(!peer.CheckSupport(), "Nonselectable mode passed support check");
                peer.Selectable = true;
                peer.Mode = 16777217;
                Require(!peer.CheckSupport(), "Inexact float mode ID passed discovery");
                peer.Mode = 29;
                peer.Mode = 256;
                Require(!peer.CheckSupport(), "Mode ID overflowed native uint8 mode API");
                peer.Mode = 29;
                peer.Rate = float.NaN;
                Require(!peer.CheckSupport(), "NaN native rate passed support check");
                peer.Rate = 2;
                Require(peer.CheckSupport(), "Inclusive two m/s support boundary was rejected");
                peer.Accept = false;
                string denied = peer.Request();
                Require(denied.Contains("not accepted") && peer.ModeCommands == 1,
                    "Native rejection was retried by the application or claimed accepted");
                peer.Accept = true;
                string accepted = peer.Request();
                Require(accepted.Contains("Command accepted") && peer.ModeCommands == 2 &&
                    !peer.Status.Contains("TERMINATE observed"), "ACK alone became observed mode or completion");
                Require(peer.LastCommandMode == 29 && peer.LastCommandFlags == 1 && peer.FlightTerminationCommands == 0,
                    "Numeric discovery/DO_SET_MODE changed or stock flight-termination fallback was sent");
                peer.SendHeartbeat(29); peer.Pump(100);
                Require(peer.Status.Contains("TERMINATE observed") && peer.Connection.MAV.cs.mode == "TERMINATE",
                    "Real subscription/core heartbeat order lost the native mode name");
                peer.SendHeartbeat(29); peer.Pump(100);
                peer.SendHeartbeat(0); peer.Pump(100);
                Require(peer.Connection.MAV.cs.mode == "Stabilize", "Return to prior known mode kept a stale name");
                peer.SendHeartbeat(29); peer.Pump(100);
                peer.Rate = 1;
                Require(!peer.CheckSupport(), "Changed rate retained command eligibility");
                peer.SendHeartbeat(29); peer.Pump(100);
                Require(peer.Connection.MAV.cs.mode == "TERMINATE", "Parameter edit erased verified mode naming");
                peer.SendHeartbeat(9); peer.Pump(100);
                Require(peer.Connection.MAV.cs.mode == "Land", "Known native mode naming was not restored");
                peer.Rate = 2;
                Require(peer.CheckSupport(), "Support did not recover after corrected native readback");
                object[] capture = peer.Capture();
                Output.GetMethod("Initialize", HiddenStatic).Invoke(null, new object[] { peer.Config });
                Require(peer.Request(capture).Contains("no termination sent") && peer.ModeCommands == 2,
                    "Retired confirmation dispatched into a new profile generation");
                string knownMode = peer.Connection.MAV.cs.mode;
                peer.DisposeController();
                peer.SendHeartbeat(29); peer.Pump(100);
                Require(peer.Connection.MAV.cs.mode == knownMode, "Disposed observer still changed native mode naming");
            }
            NegativeScenarios();
            WidgetReviewScenarios();
            Console.WriteLine("PASS actual MP discovery/readback/ACK/observation/name/lifecycle loopback checks");
        }
        finally
        {
            Output.GetMethod("Shutdown", HiddenStatic).Invoke(null, null);
            string expectedParent = Environment.GetFolderPath(Environment.SpecialFolder.CommonApplicationData);
            if (string.Equals(Directory.GetParent(metadata.TrimEnd(Path.DirectorySeparatorChar)).FullName,
                expectedParent, StringComparison.OrdinalIgnoreCase) &&
                Path.GetFileName(metadata.TrimEnd(Path.DirectorySeparatorChar)).StartsWith("NOMAD-Termination-Test-"))
            { Directory.Delete(metadata, true); }
        }
    }

    private static void Require(bool condition, string diagnostic)
    { if (!condition) { throw new InvalidOperationException(diagnostic); } }

    private static void NegativeScenarios()
    {
        using (var peer = new Peer())
        {
            peer.Enabled = 0;
            Require(!peer.CheckSupport(), "Disabled AFS passed support check");
            peer.Enabled = 1; peer.Action = 42;
            Require(!peer.CheckSupport(), "Stock termination action passed support check");
            peer.Action = 44; peer.DiscoveryAccepted = false;
            Require(!peer.CheckSupport(), "Rejected discovery passed support check");
            peer.DiscoveryAccepted = true; peer.MissingRate = true;
            Require(!peer.CheckSupport(), "Absent extension parameter passed support check");
            peer.MissingRate = false;
            Require(peer.CheckSupport(), "Negative fixture did not establish support");
            var captured = peer.Capture();
            peer.SendHeartbeat(0, false); peer.Pump(100);
            Require(!peer.CanRequest && peer.Status.Contains("not armed") &&
                peer.Request(captured).Contains("no termination sent"), "Unarmed activation was admitted");
            peer.Pump(3600);
            peer.SendHeartbeat(0, true, 202); peer.Pump(100);
            Require(!peer.CanRequest && peer.Status.Contains("No fresh"), "Other aircraft renewed stale heartbeat");
            peer.SendHeartbeat(0); peer.Pump(100);
            Require(peer.CanRequest, "Fresh selected heartbeat did not restore eligibility");
            peer.Connection.giveComport = true;
            Require(peer.Request(captured).Contains("no termination sent"), "Native busy request was dispatched");
            peer.Connection.giveComport = false;
            peer.Connection.ReadOnly = true;
            Require(peer.Request(captured).Contains("no termination sent") && peer.ModeCommands == 0,
                "ReadOnly/invalid contract/unarmed/stale input emitted an activation");
        }
        using (var peer = new Peer())
        {
            Require(peer.CheckSupport(), "Pending fixture did not establish support");
            peer.HoldModeAck = true;
            var pending = peer.BeginRequest(peer.Capture());
            Require(peer.ModeSeen.Wait(2000), "Original request did not reach peer");
            Output.GetMethod("Initialize", HiddenStatic).Invoke(null, new object[] { peer.Config });
            peer.ReleaseModeAck.Set();
            Require(peer.Complete(pending).Contains("uncertain") && peer.ModeCommands == 1,
                "Late ACK became current success or caused application replay");
        }
        using (var peer = new Peer())
        {
            peer.HoldRate = true;
            var pending = peer.BeginCheck();
            Require(peer.RateSeen.Wait(2000), "Parameter readback did not reach peer");
            Output.GetMethod("Initialize", HiddenStatic).Invoke(null, new object[] { peer.Config });
            peer.ReleaseRate.Set();
            Require(!peer.Complete(pending) && !peer.CanRequest && peer.ModeCommands == 0,
                "Retired parameter readback published eligibility or activation");
        }
    }

    private static void WidgetReviewScenarios()
    {
        var widgetType = typeof(NOMADConfig).Assembly.GetType("NOMAD.MissionPlanner.TerminationWidget", true);
        foreach (string scenario in new[] { "Cancel", "OK", "FocusLoss", "ReleaseFocusLoss", "Retired", "SessionEnded" })
        {
            using var peer = new Peer();
            using var shutdown = new CancellationTokenSource();
            Require(peer.CheckSupport(), "Widget fixture did not establish native support");
            bool eligible = true;
            int confirmations = 0;
            Func<IWin32Window, string, DialogResult> confirm = (owner, target) =>
            {
                confirmations++;
                Require(target.Contains("Synthetic loopback") && target.Contains("201") && target.Contains("42"),
                    "Widget confirmation did not identify its captured aircraft");
                if (scenario == "Retired")
                { Output.GetMethod("Initialize", HiddenStatic).Invoke(null, new object[] { peer.Config }); }
                if (scenario == "SessionEnded") { shutdown.Cancel(); }
                return scenario == "Cancel" ? DialogResult.Cancel : DialogResult.OK;
            };
            using var widget = (Control)Activator.CreateInstance(widgetType, HiddenInstance, null,
                new object[] { peer.ControllerInstance, shutdown.Token, (Func<bool>)(() => eligible), confirm }, null);
            var sample = widgetType.GetMethod("SampleState", HiddenInstance);
            Action<bool, bool, bool, long> update = (active, left, right, now) =>
                sample.Invoke(widget, new object[] { active, left, right, now });
            var pending = widgetType.GetProperty("PendingReview", HiddenInstance);
            update(true, false, false, 0);
            update(true, true, false, 10);
            peer.HoldRate = scenario == "FocusLoss";
            update(true, true, true, 20);
            if (peer.HoldRate)
            {
                var deadline = Stopwatch.StartNew();
                while (!peer.RateSeen.IsSet && deadline.ElapsedMilliseconds < 5000) { peer.Pump(10); Application.DoEvents(); }
                Require(peer.RateSeen.IsSet, "Widget check did not reach held synthetic readback");
                eligible = false;
                update(false, true, true, 30);
                peer.ReleaseRate.Set();
            }
            FinishWidget(peer, (Task)pending.GetValue(widget));
            Require(peer.ModeCommands == 0 && confirmations == 0, "Widget dispatched or confirmed before key release");
            if (scenario == "ReleaseFocusLoss")
            {
                eligible = false;
                update(false, true, true, 35);
                eligible = true;
            }
            update(eligible, false, false, 40);
            FinishWidget(peer, (Task)pending.GetValue(widget));
            Require(confirmations == (scenario == "FocusLoss" || scenario == "ReleaseFocusLoss" ? 0 : 1), "Widget confirmation count was wrong: " + scenario);
            Require(peer.ModeCommands == (scenario == "OK" ? 1 : 0), "Widget command count was wrong: " + scenario);
            update(eligible, true, true, 50);
            FinishWidget(peer, (Task)pending.GetValue(widget));
            Require(peer.ModeCommands <= 1, "Held/repeated widget input replayed termination");
            if (scenario == "Cancel")
            {
                peer.SendHeartbeat(0, false); peer.Pump(100);
                widgetType.GetMethod("RefreshStatus", HiddenInstance).Invoke(widget, null);
                var status = (Label)widgetType.GetField("_status", HiddenInstance).GetValue(widget);
                Require(status.Text.Contains("Supported") && status.Text.Contains("disarmed"),
                    "Verified disarmed support still looked unavailable in the Dashboard widget");
            }
        }
        Console.WriteLine("PASS sampled Dashboard review Cancel/OK/focus-loss/retired-target/session gates");
    }

    private static void FinishWidget(Peer peer, Task task)
    {
        var deadline = Stopwatch.StartNew();
        while (!task.IsCompleted && deadline.ElapsedMilliseconds < 12000) { peer.Pump(10); Application.DoEvents(); }
        Require(task.IsCompleted, "Widget review deadline");
        task.GetAwaiter().GetResult();
    }

    private sealed class Peer : IDisposable
    {
        internal readonly NOMADConfig Config = new NOMADConfig { AircraftName = "Synthetic loopback" };
        internal readonly MAVLinkInterface Connection = new MAVLinkInterface();
        private readonly TcpListener _listener = new TcpListener(IPAddress.Loopback, 0);
        private readonly TcpClient _server;
        private readonly Thread _worker;
        private readonly object _sendGate = new object();
        private readonly MAVLink.MavlinkParse _encoder = new MAVLink.MavlinkParse();
        private readonly TcpSerial _serial = new TcpSerial { autoReconnect = false };
        private volatile bool _stopping;
        private Exception _failure;
        private object _controller;
        internal bool Announce = true, Selectable = true, AfterAck, Accept = true, DiscoveryAccepted = true;
        internal bool MissingRate, HoldModeAck, HoldRate;
        internal float Enabled = 1, Action = 44;
        internal readonly ManualResetEventSlim ModeSeen = new ManualResetEventSlim();
        internal readonly ManualResetEventSlim ReleaseModeAck = new ManualResetEventSlim();
        internal readonly ManualResetEventSlim RateSeen = new ManualResetEventSlim();
        internal readonly ManualResetEventSlim ReleaseRate = new ManualResetEventSlim();
        internal uint Mode = 29;
        internal float Rate = 2.2f;
        internal int ModeCommands, FlightTerminationCommands;
        internal float LastCommandMode, LastCommandFlags;

        internal Peer()
        {
            _listener.Start();
            _serial.client = new TcpClient("127.0.0.1", ((IPEndPoint)_listener.LocalEndpoint).Port);
            _server = _listener.AcceptTcpClient();
            Connection.BaseStream = _serial;
            Connection.sysidcurrent = 201;
            // Avoid MP's unrelated camera/gimbal auto-discovery for component 1 in this focused synthetic peer.
            Connection.compidcurrent = 42;
            _server.ReceiveTimeout = 100;
            _worker = new Thread(Respond) { IsBackground = true };
            _worker.Start();
            SendHeartbeat(0); Pump(100);
            Output.GetField("CurrentConnection", HiddenStatic).SetValue(null, new Func<MAVLinkInterface>(() => Connection));
            Output.GetMethod("Initialize", HiddenStatic).Invoke(null, new object[] { Config });
            _controller = Activator.CreateInstance(Controller, HiddenInstance, null, new object[] { Config }, null);
            SendHeartbeat(0); Pump(100);
        }

        internal object ControllerInstance => _controller;
        internal bool CanRequest => (bool)Controller.GetProperty("CanRequest", HiddenInstance).GetValue(_controller);
        internal string Status => (string)Controller.GetProperty("Status", HiddenInstance).GetValue(_controller);
        internal bool CheckSupport() => Finish((Task<bool>)Controller.GetMethod("CheckAsync", HiddenInstance)
            .Invoke(_controller, null));
        internal Task<bool> BeginCheck() => (Task<bool>)Controller.GetMethod("CheckAsync", HiddenInstance).Invoke(_controller, null);
        internal object[] Capture()
        {
            var args = new object[] { 0L, 0U };
            Require((bool)Controller.GetMethod("TryCaptureRequest", HiddenInstance).Invoke(_controller, args),
                "Fixture could not capture a request");
            return args;
        }
        internal string Request(object[] capture = null) => Finish((Task<string>)Controller.GetMethod("RequestAsync",
            HiddenInstance).Invoke(_controller, capture ?? Capture()));
        internal Task<string> BeginRequest(object[] capture) =>
            (Task<string>)Controller.GetMethod("RequestAsync", HiddenInstance).Invoke(_controller, capture);
        internal T Complete<T>(Task<T> task) => Finish(task);

        private T Finish<T>(Task<T> task)
        {
            var deadline = Stopwatch.StartNew();
            while (!task.IsCompleted && deadline.ElapsedMilliseconds < 12000) { Pump(10); }
            Require(task.IsCompleted, "Native helper fixture deadline exceeded");
            return task.GetAwaiter().GetResult();
        }

        internal void Pump(int milliseconds)
        {
            var clock = Stopwatch.StartNew();
            while (clock.ElapsedMilliseconds < milliseconds)
            {
                if (!Connection.giveComport && _serial.BytesToRead > 0) { Connection.readPacket(); }
                else { Thread.Sleep(1); }
                if (_failure != null) { throw new InvalidOperationException("Loopback peer failed", _failure); }
            }
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
                    if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_READ)
                    {
                        var request = packet.ToStructure<MAVLink.mavlink_param_request_read_t>();
                        string name = Encoding.ASCII.GetString(request.param_id).TrimEnd('\0');
                        if (name == "AFS_TERM_RATE" && MissingRate) { continue; }
                        if (name == "AFS_TERM_RATE" && HoldRate)
                        { RateSeen.Set(); Require(ReleaseRate.Wait(5000), "Held native parameter deadline"); }
                        Send(MAVLink.MAVLINK_MSG_ID.PARAM_VALUE, new MAVLink.mavlink_param_value_t
                        {
                            param_id = Encoding.ASCII.GetBytes(name.PadRight(16, '\0')),
                            param_value = name == "AFS_ENABLE" ? Enabled : name == "AFS_TERM_ACTION" ? Action : Rate,
                            param_count = 3, param_index = 0, param_type = 9,
                        });
                    }
                    else if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG)
                    {
                        var command = packet.ToStructure<MAVLink.mavlink_command_long_t>();
                        bool discovery = command.command == (ushort)MAVLink.MAV_CMD.REQUEST_MESSAGE && command.param1 == 435;
                        bool setMode = command.command == (ushort)MAVLink.MAV_CMD.DO_SET_MODE;
                        if (!discovery && !setMode && command.command != 185) { continue; }
                        Require(command.target_system == 201 && command.target_component == 42,
                            $"Wrong synthetic target: command {command.command}, system {command.target_system}, component {command.target_component}");
                        if (discovery) { Require(command.param2 == 0, "Discovery did not request all AVAILABLE_MODES records"); }
                        if (discovery && Announce && !AfterAck) { SendMode(); }
                        if (command.command == (ushort)MAVLink.MAV_CMD.DO_SET_MODE)
                        {
                            ModeCommands++;
                            LastCommandMode = command.param2;
                            LastCommandFlags = command.param1;
                            ModeSeen.Set();
                            if (HoldModeAck) { Require(ReleaseModeAck.Wait(5000), "Held ACK deadline"); }
                        }
                        if (command.command == 185) { FlightTerminationCommands++; }
                        Send(MAVLink.MAVLINK_MSG_ID.COMMAND_ACK, new MAVLink.mavlink_command_ack_t
                        { command = command.command, result = (byte)((discovery ? DiscoveryAccepted : Accept) ? 0 : 2) });
                        if (discovery && Announce && AfterAck) { SendMode(); }
                        SendHeartbeat(0);
                    }
                }
                catch (IOException) { }
                catch (ObjectDisposedException) { }
                catch (SocketException) { }
                catch (Exception error) { _failure = error; return; }
            }
        }

        internal void SendMode() => Send(MAVLink.MAVLINK_MSG_ID.AVAILABLE_MODES,
            new MAVLink.mavlink_available_modes_t { custom_mode = Mode, mode_index = 1, number_modes = 1,
                properties = Selectable ? 0U : 1U, mode_name = Encoding.ASCII.GetBytes("TERMINATE".PadRight(35, '\0')) });

        internal void SendHeartbeat(uint mode, bool armed = true, byte system = 201) => Send(MAVLink.MAVLINK_MSG_ID.HEARTBEAT,
            new MAVLink.mavlink_heartbeat_t { autopilot = (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA,
                type = (byte)MAVLink.MAV_TYPE.QUADROTOR, base_mode = (byte)(armed ? 129 : 1), custom_mode = mode,
                mavlink_version = 3 }, system);

        private void Send(MAVLink.MAVLINK_MSG_ID id, object payload, byte system = 201)
        {
            lock (_sendGate)
            {
                byte[] bytes = _encoder.GenerateMAVLinkPacket20(id, payload, false, system, 42, -1);
                _server.GetStream().Write(bytes, 0, bytes.Length);
            }
        }

        internal void DisposeController()
        {
            (_controller as IDisposable)?.Dispose();
            _controller = null;
        }

        public void Dispose()
        {
            DisposeController();
            _stopping = true;
            ReleaseModeAck.Set(); ReleaseRate.Set();
            _server.Close();
            Connection.Close();
            Require(_worker.Join(2000), "Loopback peer did not stop");
            _listener.Stop();
            ModeSeen.Dispose(); ReleaseModeAck.Dispose(); RateSeen.Dispose(); ReleaseRate.Dispose();
        }
    }
}
