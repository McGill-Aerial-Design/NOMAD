// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.Serialization;
using System.Windows.Forms;
using GMap.NET;
using MissionPlanner;
using MissionPlanner.Comms;
using MissionPlanner.GCSViews;
using MissionPlanner.Mavlink;
using NOMAD.MissionPlanner;

// Only the selected-port global and generation are isolated. All native editor methods,
// row/column types, MAV state/cache and the recording transport are actual MP API types.
namespace MissionPlanner { public static class MainV2 { public static MAVLinkInterface comPort; } }
namespace NOMAD.MissionPlanner { internal static class OutputController { internal static long Generation; } }

internal static class NativeFenceDraftTests
{
    private const BindingFlags Fields = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
    private static int _checks;
    private static void Check(bool condition, string message)
    {
        _checks++;
        if (!condition) { throw new Exception(message); }
    }

    [STAThread]
    private static int Main(string[] args)
    {
        AppDomain.CurrentDomain.AssemblyResolve += (sender, request) =>
        {
            string name = new AssemblyName(request.Name).Name;
            string path = Path.Combine(args[0], name == "MissionPlanner" ? name + ".exe" : name + ".dll");
            return File.Exists(path) ? Assembly.LoadFrom(path) : null;
        };
        try { Run(); Console.WriteLine($"PASS native fence coordinates/draft: {_checks} checks, zero outbound writes"); return 0; }
        catch (Exception error) { Console.Error.WriteLine(error); return 1; }
    }

    private static T Uninitialized<T>()
    {
        var value = (T)FormatterServices.GetUninitializedObject(typeof(T));
        GC.SuppressFinalize(value);
        return value;
    }

    private static void Set(object owner, string field, object value) => owner.GetType().GetField(field, Fields).SetValue(owner, value);

    private static FlightPlanner Planner()
    {
        var planner = Uninitialized<FlightPlanner>();
        typeof(UserControl).GetConstructor(Type.EmptyTypes).Invoke(planner, new object[0]);
        planner.Commands = new MissionPlanner.Controls.MyDataGridView { AllowUserToAddRows = false };
        var names = new[] { "Command", "Param1", "Param2", "Param3", "Param4", "Lat", "Lon", "Alt", "TagData", "Frame" };
        foreach (string name in names)
        {
            DataGridViewColumn column;
            if (name == "Command")
            {
                var combo = new DataGridViewComboBoxColumn();
                combo.Items.AddRange("WAYPOINT", "FENCE_POLYGON_VERTEX_INCLUSION");
                column = combo;
            }
            else if (name == "Frame")
            {
                var combo = new DataGridViewComboBoxColumn(); combo.Items.Add(0); column = combo;
            }
            else { column = new DataGridViewTextBoxColumn(); }
            column.Name = name;
            planner.Commands.Columns.Add(column);
            Set(planner, name, column);
        }
        Set(planner, "cmdParamNames", new Dictionary<string, string[]>());
        planner.cmb_missiontype = new ComboBox();
        planner.cmb_missiontype.Items.AddRange(new object[] { MAVLink.MAV_MISSION_TYPE.MISSION, MAVLink.MAV_MISSION_TYPE.FENCE });
        planner.cmb_missiontype.SelectedItem = MAVLink.MAV_MISSION_TYPE.FENCE;
        typeof(FlightPlanner).GetProperty("instance").SetValue(null, planner, null);
        planner.Commands.Rows.Add("WAYPOINT", 7d, 0d, 0d, 0d, 1d, 2d, 3d, "original", 0);
        planner.Commands.Rows[0].Tag = new object();
        planner.Commands.Rows[0].Cells[8].Tag = new object();
        return planner;
    }

    private static string Values(FlightPlanner planner) => string.Join(";", planner.Commands.Rows.Cast<DataGridViewRow>()
        .Select(row => string.Join(",", row.Cells.Cast<DataGridViewCell>().Select(cell => Convert.ToString(cell.Value, CultureInfo.InvariantCulture)))));

    private static void Run()
    {
        var culture = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("fr-CA");
            Check(FenceCoordinates.TryParse(FenceCoordinates.PublishedC1, false, out var published, out _) && published.Count == 6,
                "published C1 is a six-vertex polygon independent of locale");
            Check(published[0] == new PointLatLng(50.0970884, -110.7328077) &&
                published[5] == new PointLatLng(50.0971194, -110.7382533), "published order and precision");
            Check(published[1] == new PointLatLng(50.1061216, -110.7327756) &&
                published[2] == new PointLatLng(50.1061482, -110.7437887) &&
                published[3] == new PointLatLng(50.1035232, -110.7437798) &&
                published[4] == new PointLatLng(50.0988785, -110.7382540), "all published middle vertices match source");
            Check(FenceCoordinates.TryParse("0 0\n0 1\n1 0\n0 0", true, out var reversed, out _) &&
                reversed[1] == new PointLatLng(1, 0), "explicit longitude-first and redundant closing vertex");
            foreach (string input in new[] { "", "1,2\n3,4", "1,2\n3,4\nnope", "NaN,0\n0,1\n1,0",
                "Infinity,0\n0,1\n1,0", "91,0\n0,1\n1,0", "0,181\n0,1\n1,0",
                "0,0\n1,1\n2,2", "0,0\n1,1\n0,1\n1,0", "0,0\n0,1\n0,0\n1,0",
                "0,179\n0,-179\n1,-179\n1,179", "1,2,3\n0,1\n1,0" })
            {
                Check(!FenceCoordinates.TryParse(input, false, out var invalid, out var error) && invalid == null &&
                    !string.IsNullOrEmpty(error), "whole invalid polygon rejected");
            }
            Check(FenceCoordinates.TryParse("-90,-180\n0,0\n90,0", false, out _, out _), "finite coordinate bounds accepted");
            Check(!FenceCoordinates.Validate(Enumerable.Range(0, 256).Select(i => new PointLatLng(0, i / 10d)).ToList(),
                out _), "vertex count above native supported encoding rejected");
        }
        finally { CultureInfo.CurrentCulture = culture; }
        Check(FenceCoordinates.TryParse(FenceCoordinates.PublishedC1, false, out var points, out _), "preset parse");
        var stream = new NoOutbound();
        var port = Uninitialized<MAVLinkInterface>();
        Set(port, "_baseStream", stream);
        var mav = Uninitialized<MAVState>(); mav.sysid = 1; mav.compid = 1;
        var list = Uninitialized<MAVList>();
        Set(list, "masterlist", new Dictionary<int, MAVState>()); Set(list, "hiddenlist", new Dictionary<int, MAVState>());
        Set(list, "locker", new object()); list[1, 1] = mav;
        port.MAVlist = list; Set(port, "_sysidcurrent", 1); Set(port, "_compidcurrent", 1);
        var cache = new ConcurrentDictionary<int, MAVLink.mavlink_mission_item_int_t>();
        cache[3] = new MAVLink.mavlink_mission_item_int_t { x = 123, y = 456, command = 5001 };
        mav.fencepoints = cache;
        var missions = new ConcurrentDictionary<int, MAVLink.mavlink_mission_item_int_t>(); mav.wps = missions;
        MainV2.comPort = port;
        using (var planner = Planner())
        {
            var polygon = new GMap.NET.WindowsForms.GMapPolygon(new List<PointLatLng>(), "sentinel");
            var overlay = new GMap.NET.WindowsForms.GMapOverlay("native-sentinel");
            planner.geofencepolygon = polygon; planner.geofenceoverlay = overlay;
            var draft = new NativeFenceDraft(planner, points);
            Check(draft.TryStage(out var message) && message.Contains("Draft staged"), "native staging success");
            Check(planner.Commands.Rows.Count == 6 && !planner.quickadd, "one native row per vertex and batch restored");
            for (int index = 0; index < 6; index++)
            {
                var cells = planner.Commands.Rows[index].Cells;
                Check((string)cells[0].Value == "FENCE_POLYGON_VERTEX_INCLUSION" && Convert.ToDouble(cells[1].Value) == 6 &&
                    Convert.ToDouble(cells[5].Value) == points[index].Lat &&
                    Convert.ToDouble(cells[6].Value) == points[index].Lng && Convert.ToDouble(cells[7].Value) == 0,
                    "actual native AddCommand assigns count/latitude/longitude/unused zero altitude");
            }
            Check(ReferenceEquals(planner.geofencepolygon, polygon) && ReferenceEquals(planner.geofenceoverlay, overlay),
                "native fence objects preserved");
            Check(ReferenceEquals(mav.fencepoints, cache) && cache.Count == 1 && cache[3].x == 123 &&
                ReferenceEquals(mav.wps, missions) && missions.Count == 0, "native aircraft caches preserved");
        }
        foreach (int fault in Enumerable.Range(0, 8))
        using (var planner = Planner())
        {
            var draft = new NativeFenceDraft(planner, points);
            var row = planner.Commands.Rows[0];
            string original = Values(planner);
            if (fault == 0) { /* A declined confirmation never calls TryStage. */ }
            if (fault == 1) { planner.cmb_missiontype.SelectedItem = MAVLink.MAV_MISSION_TYPE.MISSION; }
            if (fault == 2) { OutputController.Generation++; }
            if (fault == 3) { row.Cells[1].Value = 9d; original = Values(planner); }
            if (fault == 4)
            {
                bool once = true;
                planner.Commands.RowsAdded += (sender, args) => { if (once) { once = false; throw new Exception("injected row fault"); } };
            }
            if (fault == 5) { Set(port, "_baseStream", new NoOutbound()); }
            if (fault == 6) { planner.Commands.Rows[0].Cells[8].Tag = new object(); }
            if (fault == 7)
            {
                planner.Commands.Columns.Add(new DataGridViewTextBoxColumn { Name = "changed" });
                original = Values(planner);
            }
            if (fault != 0) { Check(!draft.TryStage(out _), "stale context or native row failure rejected"); }
            Check(Values(planner) == original && !planner.quickadd, "decline/stale/fault preserves original grid values");
            if (fault == 4)
            {
                Check(ReferenceEquals(planner.Commands.Rows[0].Tag, row.Tag) &&
                    ReferenceEquals(planner.Commands.Rows[0].Cells[8].Tag, row.Cells[8].Tag), "rollback preserves row and cell tags");
            }
            Set(port, "_baseStream", stream);
        }
        using (var planner = Planner())
        {
            planner.cmb_missiontype.SelectedItem = MAVLink.MAV_MISSION_TYPE.MISSION;
            string original = Values(planner);
            bool rejected = false;
            try { new NativeFenceDraft(planner, points); } catch (InvalidOperationException) { rejected = true; }
            Check(rejected && Values(planner) == original, "non-Fence draft is never silently switched or cleared");
        }
        Check(stream.Writes == 0, "every tested path emits zero transport writes");
    }

    private sealed class NoOutbound : ICommsSerial
    {
        internal int Writes;
        public Stream BaseStream => Stream.Null;
        public int BaudRate { get; set; } public int BytesToRead => 0; public int BytesToWrite => 0;
        public int DataBits { get; set; } public bool DtrEnable { get; set; } public bool IsOpen => false;
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
