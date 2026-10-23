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
    private static void Check(bool ok, string message) { Interlocked.Increment(ref _checks); if (!ok) { throw new Exception(message); } }
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
        TestTermination(directory);
        Output.GetMethod("Shutdown", Static).Invoke(null, null);
    }

    private static void TestTermination(string directory)
    {
        using (var peer = new Peer())
        using (var shared = (IDisposable)Create("TerminationController", peer.Config))
        {
            peer.Controller = (IDisposable)Create("FenceParameters", peer.Config);
            peer.Finish((Task)Call(peer.Controller, "ReadAsync"));
            peer.Finish((Task)Call(peer.Controller, "ReadTerminationAsync", shared));
            Check(Field<IList>(peer.Controller, "Parameters").Count == 3 && peer.ModeRequests == 0,
                "Stock aircraft without unique custom parameter retains ordinary Fence and sends no discovery/writes");
            peer.AddTermination(false, 1.9f, 0);
            peer.Finish((Task)Call(peer.Controller, "ReadTerminationAsync", shared));
            var parameters = Field<IList>(peer.Controller, "Parameters");
            Check(parameters.Count == 9 && peer.ModeRequests == 1 && peer.Writes == 0,
                "Advertised disabled extension exposes supported settings without enabling it");
            object action = parameters.Cast<object>().Single(p => Field<string>(p, "Name") == "AFS_TERM_ACTION");
            Check(Field<IList>(action, "Options").Count == 1 && (bool)Call(action, "Allows", 44f) &&
                !(bool)Call(action, "Allows", 42f), "Only controlled action44 is selectable, never a motor-cut action");
            var configured = EditTermination(parameters, peer.Config);
            foreach (float invalid in new[] { float.NaN, float.PositiveInfinity, 1.99f, 3.01f })
            {
                configured["AFS_TERM_RATE"] = invalid;
                string result = peer.Apply(configured);
                Check(peer.Writes == 0 && result.Contains("Invalid"), "Nonfinite/low/excess descent rate refuses all writes");
            }
            configured["AFS_TERM_RATE"] = 2.5f;
            configured["AFS_RC_FAIL_TIME"] = 0;
            Check(peer.Apply(configured).Contains("Invalid") && peer.Writes == 0, "Zero RC loss timeout refuses all writes");
            configured["AFS_RC_FAIL_TIME"] = 4.51f;
            Check(peer.Apply(configured).Contains("Invalid") && peer.Writes == 0, "RC loss timeout above4.5 refuses all writes");
            configured["AFS_RC_FAIL_TIME"] = 4.5f;
            foreach (string missing in new[] { "AFS_RC", "AFS_RC_FAIL_TIME", "AFS_GEOFENCE" })
            {
                object parameter = parameters.Cast<object>().Single(p => Field<string>(p, "Name") == missing);
                int index = parameters.IndexOf(parameter);
                parameters.Remove(parameter);
                var incomplete = new Dictionary<string, float>(configured); incomplete.Remove(missing);
                Check(peer.Apply(incomplete).Contains("invalid") && peer.Writes == 0,
                    "Missing commissioning field blocks enabling AFS: " + missing);
                parameters.Insert(index, parameter);
            }
            peer.Values["WP_SPD_DN"] = 2.3f;
            Check(peer.Apply(configured).Contains("invalid") && peer.Writes == 0, "Fresh lower native descent capability refuses all writes");
            foreach (float invalid in new[] { float.NaN, float.PositiveInfinity, 1.9f })
            {
                peer.Values["WP_SPD_DN"] = invalid;
                Check(peer.Apply(configured).Contains("invalid") && peer.Writes == 0,
                    "Nonfinite/insufficient fresh native descent capability refuses all writes");
            }
            peer.Values["WP_SPD_DN"] = 3;
            peer.Heartbeat(false); peer.Pump(30);
            string applied = peer.Apply(configured);
            Check(applied.StartsWith("Verified 4") && peer.Written.SequenceEqual(new[] { "AFS_TERM_ACTION", "AFS_TERM_RATE",
                "AFS_RC_FAIL_TIME", "AFS_ENABLE" }), "Native controlled action/config are readback-verified before enabling");
            Check(peer.Written.All(name => name.StartsWith("AFS_")) && peer.OtherCommands == 0,
                "Setup writes no ordinary landing/rate/RC-channel/GCS parameters and requests no flight mode");
            Check(!(bool)shared.GetType().GetProperty("CanRequest", Hidden).GetValue(shared),
                "Setup never grants armed menu request eligibility");
            Render(parameters, peer.Config, directory);
            peer.Controller.Dispose();
            peer.Controller = (IDisposable)Create("FenceParameters", peer.Config);
            peer.Finish((Task)Call(peer.Controller, "ReadAsync"));
            peer.Finish((Task)Call(peer.Controller, "ReadTerminationAsync", shared));
            peer.Heartbeat(false); peer.Pump(30);
            peer.IgnoreValue = true;
            int writes = peer.Writes;
            string refused = peer.Apply(new Dictionary<string, float> { ["AFS_TERM_RATE"] = 2.8f, ["AFS_RC_FAIL_TIME"] = 2 });
            Check(refused.Contains("Stopped at AFS_TERM_RATE") && peer.Writes == writes + 1 && peer.Values["AFS_RC_FAIL_TIME"] == 4.5f,
                "AFS echo/readback mismatch stops the remaining writes: " + refused + "; writes=" + (peer.Writes - writes));
        }
        using (var peer = new Peer(true))
        using (var shared = (IDisposable)Create("TerminationController", peer.Config))
        {
            peer.AddTermination(true);
            peer.Announce = false;
            peer.Controller = (IDisposable)Create("FenceParameters", peer.Config);
            peer.Finish((Task)Call(peer.Controller, "ReadAsync"));
            peer.Finish((Task)Call(peer.Controller, "ReadTerminationAsync", shared));
            Check(Field<IList>(peer.Controller, "Parameters").Count == 3 && peer.Writes == 0,
                "Custom parameter alone without native mode advertisement never exposes configuration");
            peer.Announce = true;
            peer.Selectable = false;
            peer.Finish((Task)Call(peer.Controller, "ReadTerminationAsync", shared));
            Check(Field<IList>(peer.Controller, "Parameters").Count == 3 && peer.Writes == 0,
                "Nonselectable native advertisement never exposes controlled configuration");
            peer.Selectable = true;
            peer.Finish((Task)Call(peer.Controller, "ReadTerminationAsync", shared));
            Check(Field<IList>(peer.Controller, "Parameters").Count == 9,
                "Advertised QuadPlane extension exposes configuration: " + peer.Controller.GetType().GetProperty("TerminationStatus", Hidden).GetValue(peer.Controller));
            var rate = Field<IList>(peer.Controller, "Parameters").Cast<object>().Single(p => Field<string>(p, "Name") == "AFS_TERM_RATE");
            Check(Field<double>(rate, "Maximum") == 3 && peer.ReadNames.Contains("Q_WP_SPD_DN") && !peer.ReadNames.Contains("WP_SPD_DN"),
                "QuadPlane reads its own native descent capability in metres per second");
            peer.Heartbeat(false); peer.Pump(30);
            peer.RetireAfterWrite = true;
            string retired = peer.Apply(new Dictionary<string, float> { ["AFS_TERM_ACTION"] = 44, ["AFS_ENABLE"] = 1 });
            Check(peer.Writes == 1 && retired.Contains("Stopped at AFS_TERM_ACTION") && peer.Values["AFS_ENABLE"] == 0,
                "Retirement during an AFS write prevents enabling and false readback claims");
        }
        using (var peer = new Peer())
        using (var shared = (IDisposable)Create("TerminationController", peer.Config))
        {
            peer.AddTermination(false);
            peer.Controller = (IDisposable)Create("FenceParameters", peer.Config);
            peer.Finish((Task)Call(peer.Controller, "ReadAsync"));
            peer.Finish((Task)Call(peer.Controller, "ReadTerminationAsync", shared));
            peer.Heartbeat(false); peer.Pump(30);
            peer.LowerLimitAfterWrite = true;
            string limited = peer.Apply(new Dictionary<string, float>
                { ["AFS_TERM_ACTION"] = 44, ["AFS_TERM_RATE"] = 2.8f, ["AFS_ENABLE"] = 1 });
            Check(peer.Writes == 1 && limited.Contains("Stopped at AFS_TERM_RATE") && peer.Values["AFS_ENABLE"] == 0 &&
                peer.Values["AFS_TERM_RATE"] == 2.2f, "Descent capability reduced during sequence blocks rate/enabling writes");
        }
    }

    private static Dictionary<string, float> EditTermination(IList parameters, NOMADConfig config)
    {
        using (var view = (UserControl)Create("NativeFenceSetupView", CancellationToken.None, Create("FenceSetupInput"), config))
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new System.Drawing.Point(-20000, -20000) })
        {
            form.Controls.Add(view); form.Show(); Application.DoEvents();
            foreach (object parameter in parameters) { Call(view, "AddParameter", parameter); }
            Check(((Dictionary<string, float>)Call(view, "ReadEdits")).Count == 0,
                "Unconfigured current action/rate/timeout are shown without manufacturing proposed changes");
            var action = (ComboBox)view.Controls.Find("AFS_TERM_ACTION", true)[0];
            var rate = (TextBox)view.Controls.Find("AFS_TERM_RATE", true)[0];
            var timeout = (TextBox)view.Controls.Find("AFS_RC_FAIL_TIME", true)[0];
            Check(action.Enabled && action.Items.Count == 1 && action.SelectedIndex == -1 && rate.Enabled &&
                rate.Text == "1.9" && timeout.Enabled && timeout.Text == "0",
                "Actual view allows explicit correction from disabled AFS/action0/rate1.9/timeout0");
            action.SelectedValue = 44; rate.Text = "2.5"; timeout.Text = "4.5";
            ((ComboBox)view.Controls.Find("AFS_ENABLE", true)[0]).SelectedValue = 1;
            var changes = (Dictionary<string, float>)Call(view, "ReadEdits");
            Check(changes.Count == 4 && changes["AFS_TERM_ACTION"] == 44 && changes["AFS_TERM_RATE"] == 2.5f &&
                changes["AFS_RC_FAIL_TIME"] == 4.5f && changes["AFS_ENABLE"] == 1,
                "Actual view edits produce explicit controlled changes for the native Apply helper");
            return changes;
        }
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
            foreach (object parameter in parameters)
            {
                if (Field<IList>(parameter, "Options").Count == 0) { continue; }
                var combo = (ComboBox)view.Controls.Find(Field<string>(parameter, "Name"), true)[0];
                Check(combo.SelectedValue is int selected && selected == Field<float>(parameter, "Value"),
                    "Populated enum retains the actual readback through parent/handle creation: " + combo.Name);
            }
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
        private bool _armedState;
        private Exception _failure;
        internal IDisposable Controller;
        internal int Writes, Packets, ModeRequests, OtherCommands;
        internal bool IgnoreValue, RetireAfterWrite, LowerLimitAfterWrite;
        internal bool Announce = true, Selectable = true;
        internal MAVLink.MAV_TYPE AircraftType = MAVLink.MAV_TYPE.QUADROTOR;
        internal readonly List<string> Written = new List<string>(), ReadNames = new List<string>();

        internal Peer(bool plane = false)
        {
            AircraftType = plane ? MAVLink.MAV_TYPE.FIXED_WING : MAVLink.MAV_TYPE.QUADROTOR;
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
        internal void AddTermination(bool plane, float rate = 2.2f, float timeout = 1)
        {
            AircraftType = plane ? MAVLink.MAV_TYPE.FIXED_WING : MAVLink.MAV_TYPE.QUADROTOR;
            foreach (var pair in new Dictionary<string, float> { ["AFS_ENABLE"] = 0, ["AFS_TERM_ACTION"] = 0,
                ["AFS_TERM_RATE"] = rate, ["AFS_RC"] = 1, ["AFS_RC_FAIL_TIME"] = timeout, ["AFS_GEOFENCE"] = 1,
                [plane ? "Q_WP_SPD_DN" : "WP_SPD_DN"] = 3 })
            { Values[pair.Key] = pair.Value; Parameter(pair.Key, pair.Value); }
            Heartbeat(false); Pump(80);
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
                        ReadNames.Add(name);
                        Parameter(name, Values[name]);
                        Heartbeat(_armedState);
                    }
                    if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.PARAM_SET)
                    {
                        var request = packet.ToStructure<MAVLink.mavlink_param_set_t>();
                        Check(request.target_system == 201 && request.target_component == 42, "Write addressed captured target");
                        string name = Encoding.ASCII.GetString(request.param_id).TrimEnd('\0');
                        Writes++;
                        Written.Add(name);
                        if (!IgnoreValue) { Values[name] = request.param_value; }
                        if (LowerLimitAfterWrite) { LowerLimitAfterWrite = false; Values["WP_SPD_DN"] = 2; }
                        if (RetireAfterWrite)
                        {
                            RetireAfterWrite = false;
                            Output.GetMethod("Initialize", Static).Invoke(null, new object[] { Config });
                        }
                        Parameter(name, Values[name]);
                        Heartbeat(_armedState);
                    }
                    if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG)
                    {
                        var request = packet.ToStructure<MAVLink.mavlink_command_long_t>();
                        if (request.command == (ushort)MAVLink.MAV_CMD.REQUEST_MESSAGE && request.param1 == 435)
                        {
                            ModeRequests++;
                            if (Announce) { Send(MAVLink.MAVLINK_MSG_ID.AVAILABLE_MODES, new MAVLink.mavlink_available_modes_t
                            { custom_mode = 29, mode_index = 1, number_modes = 1, properties = Selectable ? 0U : 1U,
                                mode_name = Encoding.ASCII.GetBytes("TERMINATE".PadRight(35, '\0')) }); }
                        }
                        else { OtherCommands++; }
                        Send(MAVLink.MAVLINK_MSG_ID.COMMAND_ACK, new MAVLink.mavlink_command_ack_t
                        { command = request.command, result = 0 });
                        Heartbeat(_armedState);
                    }
                }
                catch (IOException) { }
                catch (SocketException) { }
                catch (ObjectDisposedException) { }
                catch (Exception error) { _failure = error; return; }
            }
        }
        internal void Heartbeat(bool armed)
        {
            _armedState = armed;
            Send(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, new MAVLink.mavlink_heartbeat_t
            { autopilot = (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA, type = (byte)AircraftType,
                base_mode = (byte)(armed ? 129 : 1), mavlink_version = 3 });
        }
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
