// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using NOMAD.MissionPlanner;
using MissionPlanner;
using DialogHost = NOMAD.MissionPlanner.MessageBox;

namespace NOMAD.MissionPlanner
{
    // Test-only dialog host; the unchanged source partial binds to WinForms MessageBox in the plugin build.
    internal static class MessageBox
    {
        internal static DialogResult Choice;
        internal static Action DuringShow;
        internal static int Calls;
        internal static DialogResult Show(IWin32Window owner, string text, string title, MessageBoxButtons buttons,
            MessageBoxIcon icon, MessageBoxDefaultButton defaultButton)
        {
            if (buttons != MessageBoxButtons.YesNoCancel || defaultButton != MessageBoxDefaultButton.Button3)
            {
                throw new Exception("The native import choice must have explicit/default cancellation.");
            }
            Calls++; DuringShow?.Invoke(); return Choice;
        }
    }

    internal static class CustomMessageBox
    {
        internal enum MessageBoxButtons { YesNo }
        internal enum DialogResult { Yes, No }
        internal static string Last;
        internal static DialogResult Show(string text, string title, MessageBoxButtons buttons = MessageBoxButtons.YesNo)
        {
            Last = text; return DialogResult.No;
        }
    }

    internal static class MapOverlayManager
    {
        internal static int Draws;
        internal static void DrawBoundaries(GeofenceConfig config) { Draws++; }
    }

    public partial class NOMADBoundaryView : UserControl
    {
        private readonly GeofenceConfig _missionConfig;
        private readonly DataGridView _dgvSoftBoundary;
        private readonly DataGridView _dgvHardBoundary;
        internal NOMADBoundaryView(GeofenceConfig config)
        {
            _missionConfig = config;
            _dgvSoftBoundary = Grid(); _dgvHardBoundary = Grid();
            Controls.Add(_dgvSoftBoundary); Controls.Add(_dgvHardBoundary);
            Controls.Add(new CheckBox { Name = "chkAutoDraw", Checked = true });
            _dgvSoftBoundary.CellValueChanged += (s, e) => SaveBoundaryFromGrid(_dgvSoftBoundary, config.SoftBoundary);
            _dgvHardBoundary.CellValueChanged += (s, e) => SaveBoundaryFromGrid(_dgvHardBoundary, config.HardBoundary);
            LoadBoundaries(false);
        }
        private static DataGridView Grid()
        {
            var grid = new DataGridView { AllowUserToAddRows = false };
            grid.Columns.Add("Lat", "Lat"); grid.Columns.Add("Lon", "Lon"); return grid;
        }
        internal void Import() { BtnGetFromMP_Click(this, EventArgs.Empty); }
        internal void RecreateForTest() { RecreateHandle(); }
        internal void PendingEdit(bool inner)
        {
            var grid = inner ? _dgvSoftBoundary : _dgvHardBoundary;
            grid.CurrentCell = grid.Rows[0].Cells[0]; grid.CreateControl();
            if (!grid.BeginEdit(false)) { throw new Exception("Real framework grid did not begin editing."); }
            grid.EditingControl.Text = "7.25"; grid.NotifyCurrentCellDirty(true);
        }
        internal string GridValues()
        {
            return string.Join(";", new[] { _dgvSoftBoundary, _dgvHardBoundary }.SelectMany(grid =>
                grid.Rows.Cast<DataGridViewRow>().Select(row => row.Cells[0].Value + "," + row.Cells[1].Value)));
        }
    }
}

internal static class NativeFenceViewTests
{
    private static void Require(bool condition, string message)
    {
        if (!condition) { throw new Exception(message); }
    }

    private static GeofenceConfig Config(bool derive = false)
    {
        var config = new GeofenceConfig { SoftBoundaryFromHard = derive };
        GeofenceConfig.Writes = 0; GeofenceConfig.SaveResult = true; GeofenceConfig.BeforeSave = null;
        config.SoftBoundary.Vertices = new List<GpsPoint> { new GpsPoint(1, 1), new GpsPoint(1, 2), new GpsPoint(2, 1) };
        config.HardBoundary.Vertices = new List<GpsPoint> { new GpsPoint(3, 3), new GpsPoint(3, 4), new GpsPoint(4, 3) };
        MainV2.comPort = new MAVLinkInterface();
        foreach (var entry in NativeFenceImportTests.Polygon().Reverse())
        {
            MainV2.comPort.MAV.fencepoints.TryAdd(entry.Key, entry.Value);
        }
        DialogHost.Calls = 0; DialogHost.DuringShow = null; CustomMessageBox.Last = null; MapOverlayManager.Draws = 0;
        return config;
    }

    private static string Values(GeofenceConfig config)
    {
        return config.SoftBoundary.Vertices.Count + ":" + string.Join(";", config.SoftBoundary.Vertices
            .Select(point => point.Lat.ToString("R") + "," + point.Lon.ToString("R"))) + "|" +
            config.HardBoundary.Vertices.Count + ":" + string.Join(";", config.HardBoundary.Vertices
            .Select(point => point.Lat.ToString("R") + "," + point.Lon.ToString("R"))) + ";" +
            config.SoftBoundaryFromHard + ";" + config.SoftBoundaryInsetMeters + ";" + config.AdvisoryAudioAlertsEnabled;
    }

    internal static void Run()
    {
        foreach (bool inner in new[] { false, true })
        foreach (bool pending in new[] { false, true })
        {
            var config = Config();
            var untouched = inner ? config.HardBoundary.Vertices : config.SoftBoundary.Vertices;
            var cache = MainV2.comPort.MAV.fencepoints;
            var native = cache.ToArray().OrderBy(entry => entry.Key).ToArray();
            using (var view = new NOMADBoundaryView(config))
            {
                if (pending) { view.PendingEdit(inner); }
                DialogHost.Choice = inner ? DialogResult.Yes : DialogResult.No;
                view.Import();
                var imported = inner ? config.SoftBoundary.Vertices : config.HardBoundary.Vertices;
                NativeFenceImportTests.AssertConcave(imported,
                    "actual handler/grid callbacks preserve imported points after pending edit");
                Require(ReferenceEquals(untouched, inner ? config.HardBoundary.Vertices : config.SoftBoundary.Vertices),
                    "other outline list preserved");
                Require(config.Saves == 1 && MapOverlayManager.Draws == 1, "one save/draw with ordinary grid handlers");
                Require(ReferenceEquals(cache, MainV2.comPort.MAV.fencepoints) &&
                    native.SequenceEqual(cache.ToArray().OrderBy(entry => entry.Key)), "all native cache entries unchanged");
            }
        }
        foreach (var choice in new[] { DialogResult.Cancel, DialogResult.None, DialogResult.Abort })
        {
            var config = Config(); var inner = config.SoftBoundary.Vertices; var outer = config.HardBoundary.Vertices;
            using (var view = new NOMADBoundaryView(config))
            {
                var before = Values(config); var gridBefore = view.GridValues();
                DialogHost.Choice = choice; view.Import();
                Require(ReferenceEquals(inner, config.SoftBoundary.Vertices) && ReferenceEquals(outer, config.HardBoundary.Vertices)
                    && config.Saves == 0 && MapOverlayManager.Draws == 0, "cancel/close/nonchoice preserves geometry");
                Require(before == Values(config) && gridBefore == view.GridValues(), "cancel preserves all values/grid cells");
            }
        }
        foreach (int mutation in Enumerable.Range(0, 10))
        {
            var config = Config(); var inner = config.SoftBoundary.Vertices; var outer = config.HardBoundary.Vertices;
            using (var view = new NOMADBoundaryView(config))
            {
                var before = Values(config); var gridBefore = view.GridValues();
                DialogHost.Choice = DialogResult.No;
                DialogHost.DuringShow = () =>
                {
                    var port = MainV2.comPort;
                    switch (mutation)
                    {
                        case 0: MainV2.comPort = new MAVLinkInterface(); break;
                        case 1: port.MAV = new MAVState(); break;
                        case 2: port.MAV.sysid++; break;
                        case 3: port.MAV.fencepoints = new System.Collections.Concurrent.ConcurrentDictionary<int,
                            MAVLink.mavlink_mission_item_int_t>(port.MAV.fencepoints); break;
                        case 4: port.BaseStream = new TestStream(); break;
                        case 5: port.BaseStream.IsOpen = false; break;
                        case 6: var item = port.MAV.fencepoints[2]; item.x++; port.MAV.fencepoints[2] = item; break;
                        case 7: view.Dispose(); break;
                        case 8: port.MAV.compid++; break;
                        case 9: view.RecreateForTest(); break;
                    }
                };
                view.Import();
                Require(ReferenceEquals(inner, config.SoftBoundary.Vertices) && ReferenceEquals(outer, config.HardBoundary.Vertices)
                    && config.Saves == 0 && MapOverlayManager.Draws == 0, "modal context/cache mutation " + mutation);
                Require(before == Values(config) && (mutation == 7 || gridBefore == view.GridValues()) &&
                    CustomMessageBox.Last.Contains("changed"), "visible context rejection preserves values/grids");
            }
        }
        foreach (bool save in new[] { true, false })
        {
            var config = Config(true); GeofenceConfig.SaveResult = save;
            var inner = config.SoftBoundary.Vertices; var outer = config.HardBoundary.Vertices;
            using (var view = new NOMADBoundaryView(config))
            {
                var before = Values(config); var gridBefore = view.GridValues();
                view.PendingEdit(false); DialogHost.Choice = DialogResult.Yes; view.Import();
                Require(config.Saves == 0 && ReferenceEquals(inner, config.SoftBoundary.Vertices), "derived inner import rejected");
                Require(before == Values(config) && gridBefore == view.GridValues(), "derived target rejection preserves values");
                GeofenceConfig.BeforeSave = () => Require(ReferenceEquals(inner, config.SoftBoundary.Vertices) &&
                    ReferenceEquals(outer, config.HardBoundary.Vertices), "shared config stays original throughout staged save");
                DialogHost.Choice = DialogResult.No; view.Import();
                Require(config.Saves == 1, "derived outer import saves once without reload resave");
                if (save)
                {
                    Require(config.HardBoundary.Vertices.Count == 5 && config.SoftBoundary.Vertices.Count == 5 &&
                        MapOverlayManager.Draws == 1, "outer import preserves automatic inner derivation");
                }
                else
                {
                    Require(ReferenceEquals(inner, config.SoftBoundary.Vertices) && ReferenceEquals(outer, config.HardBoundary.Vertices)
                        && MapOverlayManager.Draws == 0, "save failure restores both original list identities");
                    Require(before == Values(config) && gridBefore == view.GridValues(), "failed save preserves all geometry/settings/grid cells");
                }
            }
        }
        var invalid = Config(); var source = MainV2.comPort.MAV.fencepoints[1]; source.command = 5002;
        MainV2.comPort.MAV.fencepoints[1] = source;
        using (var view = new NOMADBoundaryView(invalid))
        {
            var before = Values(invalid); var gridBefore = view.GridValues();
            view.Import(); Require(DialogHost.Calls == 0 && invalid.Saves == 0, "unsupported cache rejected before modal/mutation");
            Require(before == Values(invalid) && gridBefore == view.GridValues() && MapOverlayManager.Draws == 0 &&
                !string.IsNullOrEmpty(CustomMessageBox.Last), "unsupported cache visibly preserves local state");
        }
        var wrappedConfig = Config();
        MainV2.comPort.MAV.fencepoints.Clear();
        var seam = new[] { (10000000, 1790000000), (10000000, -1790000000),
            (20000000, -1790000000), (20000000, 1790000000) };
        for (int index = 0; index < seam.Length; index++)
        {
            MainV2.comPort.MAV.fencepoints.TryAdd(index, new MAVLink.mavlink_mission_item_int_t {
                seq = (ushort)index, command = 5001, param1 = 4, x = seam[index].Item1, y = seam[index].Item2 });
        }
        using (var view = new NOMADBoundaryView(wrappedConfig))
        {
            var before = Values(wrappedConfig); var gridBefore = view.GridValues();
            view.Import();
            Require(DialogHost.Calls == 0 && wrappedConfig.Saves == 0 && MapOverlayManager.Draws == 0 &&
                before == Values(wrappedConfig) && gridBefore == view.GridValues() &&
                CustomMessageBox.Last.Contains("longitude seam"), "wrapped polygon visibly rejected before mutation");
        }
        Console.WriteLine("PASS native fence view: actual partial/grid callbacks, pending edits, cancellation, context and save/derive isolation");
    }
}
