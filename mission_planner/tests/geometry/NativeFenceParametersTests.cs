// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Comms;
using MissionPlanner.GCSViews;
using MissionPlanner.Utilities;
using NOMAD.MissionPlanner;

internal static class NativeFenceParametersTests
{
    private const BindingFlags Hidden = BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public;
    private const BindingFlags Static = BindingFlags.Static | BindingFlags.NonPublic;
    private static readonly Assembly Plugin = typeof(NOMADConfig).Assembly;
    private static Type Output => Plugin.GetType("NOMAD.MissionPlanner.OutputController", true);
    private static int _checks;
    private static void Check(bool ok, string message) { _checks++; if (!ok) { throw new Exception(message); } }
    private static object Call(object owner, string name, params object[] args) => owner.GetType().GetMethod(name, Hidden).Invoke(owner, args);
    private static T Field<T>(object owner, string name) => (T)owner.GetType().GetField(name, Hidden).GetValue(owner);
    private static object Create(string name, params object[] args) => Activator.CreateInstance(
        Plugin.GetType("NOMAD.MissionPlanner." + name, true), Hidden, null, args, null);

    [STAThread]
    private static int Main(string[] args)
    {
        AppDomain.CurrentDomain.AssemblyResolve += (sender, request) =>
        {
            string name = new AssemblyName(request.Name).Name;
            string file = Path.Combine(args[0], name + (name == "MissionPlanner" ? ".exe" : ".dll"));
            return File.Exists(file) ? Assembly.LoadFrom(file) : null;
        };
        try { Run(args[1]); Console.WriteLine($"PASS native Fence editor/parameters: {_checks} checks; synthetic peer only"); return 0; }
        catch (Exception error) { Console.Error.WriteLine(error); return 1; }
    }

    private static void Run(string directory)
    {
        Settings.CustomUserDataDirectory = directory;
        Settings.AppConfigName = "NOMAD-Fence-Test";
        Settings.FileName = Path.Combine(directory, "host.xml");
        string metadata = Settings.GetDataDirectory();
        Directory.CreateDirectory(metadata);
        foreach (string vehicle in new[] { "SITL", "AP_Periph", "ArduSub", "Rover", "ArduCopter", "ArduPlane", "AntennaTracker", "Blimp", "Heli" })
        {
            File.WriteAllText(Path.Combine(metadata, vehicle + ".apm.pdef.xml"),
                "<paramfile><vehicles><parameters name=\"" + vehicle + "\">" +
                "<param name=\"" + vehicle + ":FENCE_ENABLE\" humanName=\"Fence enabled\"><values><value code=\"0\">Disabled</value><value code=\"1\">Enabled</value></values></param>" +
                "<param name=\"" + vehicle + ":FENCE_MARGIN\" humanName=\"Fence margin\"><field name=\"Range\">0 100</field><field name=\"Units\">m</field></param>" +
                "<param name=\"" + vehicle + ":FENCE_TYPE\" humanName=\"Fence types\"><field name=\"Bitmask\">0:Altitude,2:Polygon</field></param>" +
                "</parameters></vehicles></paramfile>");
        }
        Plugin.GetType("NOMAD.MissionPlanner.Log").GetField("_logPath", Static).SetValue(null, Path.Combine(directory, "nomad.log"));
        using (var peer = new Peer())
        {
            TestEditor(peer);
            using (var controller = (IDisposable)Create("FenceParameters", peer.Config))
            {
                peer.Controller = controller;
                peer.Finish((Task)Call(controller, "ReadAsync"));
                var parameters = Field<IList>(controller, "Parameters");
                Check(parameters.Count == 3, "Only aircraft-supported fence parameters are exposed");
                object enabled = parameters.Cast<object>().Single(p => Field<string>(p, "Name") == "FENCE_ENABLE");
                Check(Field<IList>(enabled, "Options").Count == 2 && Field<string>(enabled, "DisplayName") == "Fence enabled",
                    "Native firmware metadata drives enum values and friendly labels");
                object types = parameters.Cast<object>().Single(p => Field<string>(p, "Name") == "FENCE_TYPE");
                Check((bool)Call(types, "Allows", 5f) && !(bool)Call(types, "Allows", 2f), "Metadata bit positions validate masks");
                Render(parameters, peer.Config, directory);
                int writes = peer.Writes;
                string invalid = peer.Apply(new Dictionary<string, float> { ["FENCE_MARGIN"] = float.NaN });
                Check(peer.Writes == writes && invalid.Contains("Invalid"), "Nonfinite settings rejected without writes");
                peer.Heartbeat(true); peer.Pump(40);
                string armed = peer.Apply(new Dictionary<string, float> { ["FENCE_MARGIN"] = 4 });
                Check(peer.Writes == writes && armed.Contains("0 setting(s) verified"), "Armed aircraft refuses writes");
                controller.Dispose();
                peer.Controller = (IDisposable)Create("FenceParameters", peer.Config);
                peer.Finish((Task)Call(peer.Controller, "ReadAsync"));
                peer.Heartbeat(false); peer.Pump(40);
                string applied = peer.Apply(new Dictionary<string, float> { ["FENCE_MARGIN"] = 4 });
                Check(applied.StartsWith("Verified 1") && peer.Values["FENCE_MARGIN"] == 4 && peer.Writes == writes + 1,
                    "Native PARAM_SET and independent readback verify one change on captured target");
                peer.IgnoreValue = true;
                string refused = peer.Apply(new Dictionary<string, float> { ["FENCE_MARGIN"] = 5, ["FENCE_ENABLE"] = 1 });
                Check(refused.Contains("Stopped at FENCE_MARGIN") && peer.Values["FENCE_ENABLE"] == 0 && peer.Writes == writes + 2,
                    "Different echoed/readback value stops the remaining settings");
                string retry = peer.Apply(new Dictionary<string, float> { ["FENCE_ENABLE"] = 1 });
                Check(peer.Writes == writes + 2 && retry.Contains("0 setting(s) verified"), "Uncertain session requires a new Read before retry");
                peer.IgnoreValue = false;
                peer.Controller.Dispose();
                peer.Controller = (IDisposable)Create("FenceParameters", peer.Config);
                peer.Finish((Task)Call(peer.Controller, "ReadAsync"));
                peer.Heartbeat(false); peer.Pump(40);
                peer.Connection.ReadOnly = true;
                string retired = peer.Apply(new Dictionary<string, float> { ["FENCE_ENABLE"] = 1 });
                Check(peer.Writes == writes + 2 && retired.Contains("0 setting(s) verified"), "Retired writable context refuses new writes");
                peer.Connection.ReadOnly = false;
                peer.Controller.Dispose();
                peer.Controller = (IDisposable)Create("FenceParameters", peer.Config);
                peer.Finish((Task)Call(peer.Controller, "ReadAsync"));
                peer.Heartbeat(false); peer.Pump(40);
                Thread.Sleep(3600);
                string stale = peer.Apply(new Dictionary<string, float> { ["FENCE_ENABLE"] = 1 });
                Check(peer.Writes == writes + 2 && stale.Contains("0 setting(s) verified"), "Expired disarmed heartbeat refuses writes");
                peer.Controller.Dispose();
                peer.Controller = (IDisposable)Create("FenceParameters", peer.Config);
                peer.Finish((Task)Call(peer.Controller, "ReadAsync"));
                peer.Heartbeat(false); peer.Pump(40);
                peer.RetireAfterWrite = true;
                string partial = peer.Apply(new Dictionary<string, float> { ["FENCE_MARGIN"] = 6, ["FENCE_ENABLE"] = 1 });
                Check(peer.Writes == writes + 3 && peer.Values["FENCE_ENABLE"] == 0 && partial.Contains("Stopped at FENCE_MARGIN"),
                    "Profile retirement during native write prevents readback claims and all remaining writes");
            }
        }
        Output.GetMethod("Shutdown", Static).Invoke(null, null);
    }

    private static void Render(IList parameters, NOMADConfig config, string directory)
    {
        foreach (float scale in new[] { 1f, 1.5f })
        using (var view = (UserControl)Create("NativeFenceSetupView", CancellationToken.None, Create("FenceSetupInput"), config))
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new System.Drawing.Point(-20000, -20000), ClientSize = new System.Drawing.Size((int)(800 * scale), (int)(850 * scale)) })
        {
            view.Dock = DockStyle.Fill; form.Controls.Add(view);
            foreach (object parameter in parameters) { Call(view, "AddParameter", parameter); }
            view.Scale(new System.Drawing.SizeF(scale, scale));
            form.Show(); form.PerformLayout(); Application.DoEvents();
            var margin = (NumericUpDown)view.Controls.Find("FENCE_MARGIN", true)[0];
            var type = (CheckedListBox)view.Controls.Find("FENCE_TYPE", true)[0];
            var enabled = (ComboBox)view.Controls.Find("FENCE_ENABLE", true)[0];
            Check(margin.Minimum == 0 && margin.Maximum == 100 && margin.Value == 2 &&
                type.CheckedItems.Count == 1 && (int)enabled.SelectedValue == 0,
                "Real populated controls show fresh numeric, bitmask and enum settings at " + scale);
            using (var bitmap = new System.Drawing.Bitmap(form.Width, form.Height))
            {
                form.DrawToBitmap(bitmap, new System.Drawing.Rectangle(0, 0, bitmap.Width, bitmap.Height));
                bitmap.Save(Path.Combine(directory, "populated-fence-" + scale + ".png"));
            }
        }
    }

    private static void TestEditor(Peer peer)
    {
        MainV2.comPort = peer.Connection;
        Settings.Instance["SHOWAGAIN_FlightPlan_Fence"] = "False";
        GMap.NET.GMaps.Instance.Mode = GMap.NET.AccessMode.CacheOnly;
        using (var planner = new FlightPlanner())
        {
            foreach (string name in new[] { "TXT_homealt", "TXT_homelat", "TXT_homelng" })
            { ((TextBox)typeof(FlightPlanner).GetField(name, Hidden).GetValue(planner)).Text = "1"; }
            var timer = (System.Windows.Forms.Timer)typeof(FlightPlanner).GetField("timer1", Hidden).GetValue(planner);
            planner.cmb_missiontype.SelectedItem = MAVLink.MAV_MISSION_TYPE.MISSION;
            planner.quickadd = true; planner.Commands.Rows.Clear(); planner.quickadd = false;
            planner.AddCommand(MAVLink.MAV_CMD.WAYPOINT, 7, 0, 0, 0, 2, 1, 30);
            var tag = new object(); planner.Commands.Rows[0].Tag = tag;
            int writes = peer.Writes;
            int packets = peer.Packets;
            var missions = peer.Connection.MAV.wps;
            var fences = peer.Connection.MAV.fencepoints;
            using (var drafts = (IDisposable)Create("NativePlanDrafts", planner))
            {
                Check((bool)Call(drafts, "SelectFence"), "Actual native combo/handler selects Fence");
                planner.AddCommand(MAVLink.MAV_CMD.FENCE_POLYGON_VERTEX_INCLUSION, 3, 0, 0, 0, 4, 3, 0);
                planner.cmb_missiontype.SelectedItem = MAVLink.MAV_MISSION_TYPE.MISSION;
                Check(planner.Commands.Rows.Cast<DataGridViewRow>().Count(row => !row.IsNewRow) == 1 &&
                    Convert.ToDouble(planner.Commands.Rows[0].Cells[1].Value) == 7 && ReferenceEquals(planner.Commands.Rows[0].Tag, tag),
                    "Actual native switch restores unsaved Mission row values/tags");
                planner.cmb_missiontype.SelectedItem = MAVLink.MAV_MISSION_TYPE.RALLY;
                planner.AddCommand(MAVLink.MAV_CMD.RALLY_POINT, 0, 0, 0, 0, 8, 7, 40);
                planner.cmb_missiontype.SelectedItem = MAVLink.MAV_MISSION_TYPE.FENCE;
                Check(planner.Commands.Rows.Cast<DataGridViewRow>().Count(row => !row.IsNewRow) == 1 &&
                    Convert.ToDouble(planner.Commands.Rows[0].Cells[5].Value) == 3, "Fence draft survives manual mode changes");
                planner.cmb_missiontype.SelectedItem = MAVLink.MAV_MISSION_TYPE.RALLY;
                Check(Convert.ToDouble(planner.Commands.Rows[0].Cells[5].Value) == 7, "Rally draft survives manual mode changes");
                peer.Pump(40);
                Check(peer.Packets == packets && peer.Writes == writes && ReferenceEquals(peer.Connection.MAV.wps, missions) && missions.Count == 0 &&
                    ReferenceEquals(peer.Connection.MAV.fencepoints, fences) && fences.Count == 0,
                    "Mode switching preserves aircraft caches and emits zero parameter writes");
                planner.quickadd = true;
                Check(!(bool)Call(drafts, "SelectFence") &&
                    (MAVLink.MAV_MISSION_TYPE)planner.cmb_missiontype.SelectedItem == MAVLink.MAV_MISSION_TYPE.RALLY,
                    "Native editor batch refuses switching and preserves the current mode");
                planner.quickadd = false;
                bool retire = true;
                DataGridViewRowsRemovedEventHandler onRemove = (sender, args) =>
                {
                    if (!retire) { return; }
                    retire = false;
                    Output.GetMethod("Initialize", Static).Invoke(null, new object[] { peer.Config });
                };
                planner.Commands.RowsRemoved += onRemove;
                Check(!(bool)Call(drafts, "SelectFence"), "Context retired during actual native switch rejects stale restoration");
                planner.Commands.RowsRemoved -= onRemove;
                Check(planner.Commands.Rows.Cast<DataGridViewRow>().Count(row => !row.IsNewRow) == 0,
                    "Reentrant context change does not restore saved Fence rows");
                Output.GetMethod("Initialize", Static).Invoke(null, new object[] { peer.Config });
                planner.cmb_missiontype.SelectedItem = MAVLink.MAV_MISSION_TYPE.MISSION;
                Check(planner.Commands.Rows.Cast<DataGridViewRow>().Count(row => !row.IsNewRow) == 0,
                    "Retired generation never restores a previous Mission draft");
            }
            timer.Stop();
        }
    }

    private sealed class Peer : IDisposable
    {
        internal readonly NOMADConfig Config = new NOMADConfig { AircraftName = "Synthetic Fence peer" };
        internal readonly MAVLinkInterface Connection = new MAVLinkInterface();
        internal readonly Dictionary<string, float> Values = new Dictionary<string, float>
            { ["FENCE_MARGIN"] = 2, ["FENCE_ENABLE"] = 0, ["FENCE_TYPE"] = 4 };
        private readonly TcpListener _listener = new TcpListener(IPAddress.Loopback, 0);
        private readonly TcpClient _server;
        private readonly TcpSerial _serial = new TcpSerial { autoReconnect = false };
        private readonly Thread _worker;
        private readonly MAVLink.MavlinkParse _encoder = new MAVLink.MavlinkParse();
        private readonly object _send = new object();
        private volatile bool _stop;
        private Exception _failure;
        internal IDisposable Controller;
        internal int Writes, Packets;
        internal bool IgnoreValue, RetireAfterWrite;

        internal Peer()
        {
            _listener.Start();
            _serial.client = new TcpClient("127.0.0.1", ((IPEndPoint)_listener.LocalEndpoint).Port);
            _server = _listener.AcceptTcpClient(); _server.ReceiveTimeout = 100;
            Connection.BaseStream = _serial; Connection.sysidcurrent = 201; Connection.compidcurrent = 42;
            _worker = new Thread(Respond) { IsBackground = true }; _worker.Start();
            Heartbeat(false); Pump(50);
            foreach (var pair in Values) { Parameter(pair.Key, pair.Value); }
            Pump(60);
            Output.GetField("CurrentConnection", Static).SetValue(null, new Func<MAVLinkInterface>(() => Connection));
            Output.GetMethod("Initialize", Static).Invoke(null, new object[] { Config });
        }

        internal void Finish(Task task)
        {
            var clock = Stopwatch.StartNew();
            while (!task.IsCompleted && clock.ElapsedMilliseconds < 12000) { Pump(10); }
            Check(task.IsCompleted, "Native Fence helper exceeded fixture deadline");
            task.GetAwaiter().GetResult();
        }
        internal string Apply(Dictionary<string, float> changes)
        {
            var task = (Task<string>)Call(Controller, "ApplyAsync", changes, new Func<bool>(() => true));
            Finish(task); return task.GetAwaiter().GetResult();
        }
        internal void Pump(int milliseconds)
        {
            var clock = Stopwatch.StartNew();
            while (clock.ElapsedMilliseconds < milliseconds)
            {
                if (!Connection.giveComport && _serial.BytesToRead > 0) { Connection.readPacket(); }
                else { Thread.Sleep(1); }
                if (_failure != null) { throw new Exception("Synthetic peer failed", _failure); }
            }
        }
        private void Respond()
        {
            var parser = new MAVLink.MavlinkParse();
            while (!_stop)
            {
                try
                {
                    var packet = parser.ReadPacket(_server.GetStream());
                    if (packet == null) { continue; }
                    Packets++;
                    if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_READ)
                    {
                        var request = packet.ToStructure<MAVLink.mavlink_param_request_read_t>();
                        Check(request.target_system == 201 && request.target_component == 42, "Read addressed captured target");
                        string name = Encoding.ASCII.GetString(request.param_id).TrimEnd('\0');
                        Parameter(name, Values[name]);
                    }
                    if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.PARAM_SET)
                    {
                        var request = packet.ToStructure<MAVLink.mavlink_param_set_t>();
                        Check(request.target_system == 201 && request.target_component == 42, "Write addressed captured target");
                        string name = Encoding.ASCII.GetString(request.param_id).TrimEnd('\0');
                        Writes++;
                        if (!IgnoreValue) { Values[name] = request.param_value; }
                        if (RetireAfterWrite)
                        {
                            RetireAfterWrite = false;
                            Output.GetMethod("Initialize", Static).Invoke(null, new object[] { Config });
                        }
                        Parameter(name, Values[name]);
                    }
                }
                catch (IOException) { }
                catch (SocketException) { }
                catch (ObjectDisposedException) { }
                catch (Exception error) { _failure = error; return; }
            }
        }
        internal void Heartbeat(bool armed) => Send(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, new MAVLink.mavlink_heartbeat_t
        { autopilot = (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA, type = (byte)MAVLink.MAV_TYPE.QUADROTOR,
            base_mode = (byte)(armed ? 129 : 1), mavlink_version = 3 });
        private void Parameter(string name, float value) => Send(MAVLink.MAVLINK_MSG_ID.PARAM_VALUE,
            new MAVLink.mavlink_param_value_t { param_id = Encoding.ASCII.GetBytes(name.PadRight(16, '\0')),
                param_value = value, param_count = 3, param_index = 0, param_type = 9 });
        private void Send(MAVLink.MAVLINK_MSG_ID id, object payload)
        {
            lock (_send)
            {
                byte[] bytes = _encoder.GenerateMAVLinkPacket20(id, payload, false, 201, 42, -1);
                _server.GetStream().Write(bytes, 0, bytes.Length);
            }
        }
        public void Dispose()
        {
            Controller?.Dispose(); _stop = true; _server.Close(); Connection.Close();
            Check(_worker.Join(2000), "Synthetic peer stopped"); _listener.Stop();
        }
    }
}
