// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.Serialization;
using System.Windows.Forms;
using Microsoft.Extensions.Caching.Memory;
using MissionPlanner;
using MissionPlanner.ArduPilot;
using MissionPlanner.Comms;
using MissionPlanner.Controls;
using MissionPlanner.Controls.BackstageView;
using MissionPlanner.GCSViews;
using MissionPlanner.GCSViews.ConfigurationView;
using MissionPlanner.Mavlink;
using MissionPlanner.Utilities;
using NOMAD.MissionPlanner;

// Only generation is isolated; the native editor, handlers, cache, host globals and transport are real MP types.
namespace NOMAD.MissionPlanner { internal static class OutputController { internal static long Generation; } }

internal static class NativeLandParameterDraftTests
{
    private const BindingFlags Fields = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
    private const BindingFlags Static = BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic;
    private static int _checks;
    private static void Check(bool condition, string message)
    { _checks++; if (!condition) { throw new Exception(message); } }

    [STAThread]
    private static int Main(string[] args)
    {
        AppDomain.CurrentDomain.AssemblyResolve += (sender, request) =>
        {
            string name = new AssemblyName(request.Name).Name;
            string path = Path.Combine(args[0], name == "MissionPlanner" ? name + ".exe" : name + ".dll");
            return File.Exists(path) ? Assembly.LoadFrom(path) : null;
        };
        try { Run(); Console.WriteLine($"PASS native LAND draft: {_checks} checks; zero outbound writes"); return 0; }
        catch (Exception error) { Console.Error.WriteLine(error); return 1; }
    }

    private static T Uninitialized<T>()
    {
        var value = (T)FormatterServices.GetUninitializedObject(typeof(T)); GC.SuppressFinalize(value); return value;
    }
    private static void Set(object owner, string name, object value) => owner.GetType().GetField(name, Fields).SetValue(owner, value);
    private static void Seed(string name, string key, string value)
    {
        var cache = (MemoryCache)typeof(ParameterMetaDataRepository).GetField("_cache", Static).GetValue(null);
        using (var entry = cache.CreateEntry(name + key + Firmwares.ArduCopter2)) { entry.Value = value; }
    }
    private static void SeedMetadata()
    {
        foreach (string name in new[] { "FENCE_ACTION", "LAND_SPEED", "LAND_SPEED_HIGH", "LAND_SPD_MS", "LAND_SPD_HIGH_MS", "UNRELATED" })
        {
            foreach (string key in new[] { "ReadOnly", "Range", "Values", "Bitmask", "Increment", "Units", "Description" })
            { Seed(name, key, ""); }
            Seed(name, "ReadOnly", "False"); Seed(name, "Range", "0 1000"); Seed(name, "Increment", "1");
        }
        Seed("FENCE_ACTION", "Values", "0:Report,1:RTL,2:Land");
    }

    private sealed class Fixture : IDisposable
    {
        internal readonly MAVLinkInterface Port;
        internal readonly MAVState Mav;
        internal readonly RecordingStream Stream = new RecordingStream();
        internal readonly ConfigRawParams Page;
        internal readonly DataGridView Grid;
        internal readonly string[] Rates;
        internal readonly Form Host;
        internal readonly BackstageView Backstage;
        internal Hashtable Changes => (Hashtable)typeof(ConfigRawParams).GetField("_changes", Fields).GetValue(Page);

        internal Fixture(bool modern = false)
        {
            SeedMetadata();
            Port = Uninitialized<MAVLinkInterface>(); Set(Port, "_baseStream", Stream); Set(Port, "writelock", new object());
            Set(Port, "readlock", new System.Threading.SemaphoreSlim(1, 1));
            Mav = Uninitialized<MAVState>(); Mav.sysid = 1; Mav.compid = 1;
            Mav.cs = Uninitialized<CurrentState>(); Mav.cs.firmware = Firmwares.ArduCopter2;
            Set(Mav, "<param>k__BackingField", new MAVLink.MAVLinkParamList());
            Set(Mav, "<packets>k__BackingField", new Dictionary<uint, Queue<MAVLink.MAVLinkMessage>>());
            Set(Mav, "<packetsLast>k__BackingField", new Dictionary<uint, MAVLink.MAVLinkMessage>());
            Set(Mav, "packetslock", new object());
            var list = Uninitialized<MAVList>();
            Set(list, "masterlist", new Dictionary<int, MAVState>()); Set(list, "hiddenlist", new Dictionary<int, MAVState>());
            Set(list, "locker", new object()); list[1, 1] = Mav;
            Port.MAVlist = list; Set(Port, "_sysidcurrent", 1); Set(Port, "_compidcurrent", 1);
            MainV2.comPort = Port;
            Heartbeat(DateTime.UtcNow);
            Rates = modern ? new[] { "LAND_SPD_MS", "LAND_SPD_HIGH_MS" } : new[] { "LAND_SPEED", "LAND_SPEED_HIGH" };
            AddParameter("FENCE_ACTION", 0); AddParameter(Rates[0], modern ? 0.5 : 50);
            AddParameter(Rates[1], 0); AddParameter("UNRELATED", 4);

            // Do not Activate: it clears drafts and starts metadata/default-list work.
            typeof(ConfigRawParams).GetField("startup", Static).SetValue(null, true);
            Page = new ConfigRawParams { Dock = DockStyle.Fill };
            Grid = Page.Controls.Find("Params", true).OfType<DataGridView>().Single();
            Grid.AllowUserToAddRows = false;
            foreach (string name in new[] { "FENCE_ACTION", Rates[0], Rates[1], "UNRELATED" })
            {
                int index = Grid.Rows.Add(); var row = Grid.Rows[index];
                row.Cells["Command"].Value = name;
                row.Cells["Value"].Value = Mav.param[name].Value.ToString(CultureInfo.InvariantCulture);
            }
            var software = Uninitialized<SoftwareConfig>();
            typeof(UserControl).GetConstructor(Type.EmptyTypes).Invoke(software, new object[0]);
            Backstage = Uninitialized<BackstageView>();
            typeof(UserControl).GetConstructor(Type.EmptyTypes).Invoke(Backstage, new object[0]);
            Set(Backstage, "_items", new BackstageViewCollection());
            var selected = new BackstageViewPage(typeof(ConfigRawParams), "Full Parameter List") { Page = Page };
            Backstage.Pages.Add(selected); Set(Backstage, "_activePage", selected);
            Backstage.Controls.Add(Page); software.Controls.Add(Backstage);
            Backstage.Dock = DockStyle.Fill; software.Dock = DockStyle.Fill;
            Host = new Form { ClientSize = new System.Drawing.Size(900, 650), ShowInTaskbar = false,
                StartPosition = FormStartPosition.Manual, Location = new System.Drawing.Point(-20000, -20000) };
            MainV2.View = new MainSwitcher(Host);
            var screen = new MainSwitcher.Screen("SWConfig", software);
            MainV2.View.screens.Add(screen); MainV2.View.current = screen;
            MainV2.View.Controls.Add(software);
            Host.Show(); Application.DoEvents();
            typeof(ConfigRawParams).GetField("startup", Static).SetValue(null, false);
        }

        internal void Heartbeat(DateTime time, bool armed = false, MAVLink.MAV_TYPE type = MAVLink.MAV_TYPE.QUADROTOR)
        {
            var heartbeat = new MAVLink.mavlink_heartbeat_t { autopilot = (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA,
                type = (byte)type, base_mode = armed ? (byte)MAVLink.MAV_MODE_FLAG.SAFETY_ARMED : (byte)0 };
            var bytes = new MAVLink.MavlinkParse().GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, heartbeat, 1, 1, 0);
            Mav.addPacket(new MAVLink.MAVLinkMessage(bytes, time));
        }
        internal void AddParameter(string name, double value)
        {
            Mav.param.Add(new MAVLink.MAVLinkParam(name, value, MAVLink.MAV_PARAM_TYPE.REAL32, null));
            Mav.param.TotalReported = Mav.param.TotalReceived;
        }
        internal DataGridViewCell Cell(string name) => Grid.Rows.Cast<DataGridViewRow>()
            .Single(row => Equals(row.Cells["Command"].Value, name)).Cells["Value"];
        internal void Edit(string name, string value)
        {
            Grid.CurrentCell = Cell(name); Check(Grid.BeginEdit(false), "native edit begins");
            ((DataGridViewTextBoxEditingControl)Grid.EditingControl).Text = value;
            Check(Grid.EndEdit(), "native edit commits to draft");
        }
        public void Dispose()
        {
            Check(Stream.Writes == 0, "native editor staging emits no transport writes");
            Host.Dispose(); MainV2.View = null; MainV2.comPort = null;
        }
    }

    private static void Reject(Fixture fixture, string reason)
    {
        int before = fixture.Changes.Count; bool rejected = false;
        try { new NativeLandParameterDraft(); } catch (InvalidOperationException) { rejected = true; }
        Check(rejected && fixture.Changes.Count == before, reason);
    }
    private static void Run()
    {
        var originalPort = MainV2.comPort; var originalView = MainV2.View;
        try
        {
            foreach (bool modern in new[] { false, true })
            using (var fixture = new Fixture(modern))
            {
                fixture.Edit("UNRELATED", "87"); fixture.Edit("FENCE_ACTION", "1");
                var handle = fixture.Page.Handle; var draft = new NativeLandParameterDraft();
                Check(draft.Preview.Contains("FENCE_ACTION: 1") && draft.Preview.Contains("ordinary landings") &&
                    draft.Preview.Contains("other edits"), "preview uses grid draft values and discloses global/native-write effects");
                Check(draft.TryStage(true, out _), "complete reviewed layout stages");
                double desired = modern ? 2 : 200;
                Check((double)fixture.Changes["FENCE_ACTION"] == 2 && (double)fixture.Changes[fixture.Rates[0]] == desired &&
                    (double)fixture.Changes[fixture.Rates[1]] == desired && (double)fixture.Changes["UNRELATED"] == 87,
                    "actual native callback queues exactly requested values and preserves unrelated edits");
                Check(fixture.Mav.param["FENCE_ACTION"].Value == 0 && fixture.Mav.param[fixture.Rates[1]].Value == 0 &&
                    fixture.Page.Handle == handle, "staging neither applies to cache nor reactivates/recreates the page");
                int edits = 0; fixture.Grid.CellBeginEdit += (sender, args) => edits++;
                Check(new NativeLandParameterDraft().TryStage(true, out string message) && message.Contains("queued") && edits == 3,
                    "already-matching grid values still traverse all three native edit callbacks");
            }
            using (var fixture = new Fixture())
            {
                fixture.Mav.param["FENCE_ACTION"].Value = 2;
                fixture.Edit("UNRELATED", "87"); fixture.Edit("FENCE_ACTION", "1");
                // Native Refresh replaces the visible cache values under startup without clearing its queued changes.
                typeof(ConfigRawParams).GetField("startup", Static).SetValue(null, true);
                fixture.Cell("FENCE_ACTION").Value = "2";
                typeof(ConfigRawParams).GetField("startup", Static).SetValue(null, false);
                Check((double)fixture.Changes["FENCE_ACTION"] == 1, "fixture retains the conflicting hidden native draft");
                Check(new NativeLandParameterDraft().TryStage(true, out _) && (double)fixture.Changes["FENCE_ACTION"] == 2 &&
                    (double)fixture.Changes["UNRELATED"] == 87, "matching displayed value replaces a conflicting queued value through the real handler");
            }
            using (var fixture = new Fixture())
            {
                var draft = new NativeLandParameterDraft();
                Check(!draft.TryStage(false, out _) && fixture.Changes.Count == 0, "decline preserves native draft");
                fixture.Port.ReadOnly = true;
                Check(new NativeLandParameterDraft().TryStage(true, out _), "read-only link permits staging without a write");
            }
            for (int fault = 0; fault < 15; fault++)
            using (var fixture = new Fixture())
            {
                var draft = new NativeLandParameterDraft();
                if (fault == 0) { OutputController.Generation++; }
                if (fault == 1) { fixture.Cell("FENCE_ACTION").Value = "1"; }
                if (fault == 2) { fixture.Heartbeat(DateTime.UtcNow.AddSeconds(-5)); }
                if (fault == 3) { fixture.Mav.cs.firmware = Firmwares.ArduPlane; }
                if (fault == 4) { fixture.Port.logreadmode = true; }
                if (fault == 5) { fixture.Mav.cs.armed = true; }
                if (fault == 6) { MainV2.View.current = null; }
                if (fault == 7) { Set(fixture.Port, "_baseStream", new RecordingStream()); }
                if (fault == 8) { fixture.Cell("FENCE_ACTION").ReadOnly = true; }
                if (fault == 9) { fixture.Mav.param["FENCE_ACTION"].Value = 1; }
                if (fault == 10) { fixture.Mav.sysid = 2; }
                if (fault == 11) { MainV2.comPort = null; }
                if (fault == 12) { fixture.Heartbeat(DateTime.UtcNow, armed: true); }
                if (fault == 13) { fixture.Port.giveComport = true; }
                if (fault == 14) { fixture.AddParameter("LAND_SPD_MS", 1); }
                int before = fixture.Changes.Count;
                Check(!draft.TryStage(true, out _) && fixture.Changes.Count == before,
                    "changed context/cache/draft rejects staging before any new edit");
            }
            foreach (var type in new[] { MAVLink.MAV_TYPE.FIXED_WING, MAVLink.MAV_TYPE.VTOL_QUADROTOR,
                MAVLink.MAV_TYPE.GROUND_ROVER, MAVLink.MAV_TYPE.GENERIC, (MAVLink.MAV_TYPE)255 })
            using (var fixture = new Fixture())
            {
                fixture.Heartbeat(DateTime.UtcNow, type: type);
                Reject(fixture, "fresh incompatible heartbeat cannot inherit cached Copter firmware/capabilities");
            }
            using (var fixture = new Fixture())
            {
                fixture.AddParameter("LAND_SPD_MS", 1); Reject(fixture, "mixed rate layout rejected");
            }
            using (var fixture = new Fixture())
            {
                fixture.Mav.param.Clear(); fixture.AddParameter("FENCE_ACTION", 0); fixture.AddParameter("LAND_SPEED", 50);
                Reject(fixture, "partial rate layout rejected");
            }
            using (var fixture = new Fixture())
            {
                fixture.Grid.Rows.Remove(fixture.Cell("FENCE_ACTION").OwningRow); Reject(fixture, "missing native row rejected");
            }
            using (var fixture = new Fixture())
            {
                fixture.Grid.CurrentCell = null; fixture.Cell(fixture.Rates[1]).OwningRow.Visible = false;
                Reject(fixture, "filtered native rate row requires clearing filters before preview");
            }
            using (var fixture = new Fixture())
            {
                var draft = new NativeLandParameterDraft(); Seed("FENCE_ACTION", "ReadOnly", "invalid-bool");
                Check(!draft.TryStage(true, out string message) && fixture.Changes.Count == 0 && message.Contains("confirmed: 0"),
                    "native callback rejection is not called a staged batch");
            }
            using (var fixture = new Fixture())
            {
                fixture.Edit("UNRELATED", "87"); var draft = new NativeLandParameterDraft();
                fixture.Grid.CellValueChanged += (sender, args) =>
                {
                    if (args.RowIndex == fixture.Cell("FENCE_ACTION").RowIndex && args.ColumnIndex == fixture.Cell("FENCE_ACTION").ColumnIndex)
                    { Seed(fixture.Rates[0], "ReadOnly", "invalid-bool"); }
                };
                Check(!draft.TryStage(true, out string message) && message.Contains("confirmed: 1") &&
                    fixture.Changes.Count == 2 && (double)fixture.Changes["FENCE_ACTION"] == 2 &&
                    (double)fixture.Changes["UNRELATED"] == 87, "partial native rejection preserves prior and unrelated queued edits truthfully");
            }
        }
        finally { MainV2.comPort = originalPort; MainV2.View = originalView; }
    }

    private sealed class RecordingStream : ICommsSerial
    {
        internal int Writes;
        public Stream BaseStream => Stream.Null;
        public int BaudRate { get; set; } public int BytesToRead => 0; public int BytesToWrite => 0;
        public int DataBits { get; set; } public bool DtrEnable { get; set; } public bool IsOpen => true;
        public string PortName { get; set; } public int ReadBufferSize { get; set; } public int ReadTimeout { get; set; }
        public bool RtsEnable { get; set; } public int WriteBufferSize { get; set; } public int WriteTimeout { get; set; }
        public void Close() { } public void DiscardInBuffer() { } public void Open() => throw new Exception("No connections");
        public int Read(byte[] buffer, int offset, int count) => throw new Exception("No reads");
        public int ReadByte() => throw new Exception("No reads"); public int ReadChar() => throw new Exception("No reads");
        public string ReadExisting() => throw new Exception("No reads"); public string ReadLine() => throw new Exception("No reads");
        public void Write(string value) { Writes++; throw new Exception("No writes"); }
        public void Write(byte[] buffer, int offset, int count) { Writes++; throw new Exception("No writes"); }
        public void WriteLine(string value) { Writes++; throw new Exception("No writes"); } public void toggleDTR() { }
        public void Dispose() { }
    }
}
