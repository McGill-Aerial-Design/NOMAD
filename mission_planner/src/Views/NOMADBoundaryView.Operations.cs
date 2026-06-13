// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Boundary View - Boundary Import/Export & Operations
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Reflection;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADBoundaryView
    {
        private void LoadBoundaries()
        {
            // Load soft boundary
            _dgvSoftBoundary.Rows.Clear();
            foreach (var point in _missionConfig.SoftBoundary.Vertices)
            {
                _dgvSoftBoundary.Rows.Add(point.Lat.ToString("F8"), point.Lon.ToString("F8"));
            }

            // Load hard boundary
            _dgvHardBoundary.Rows.Clear();
            foreach (var point in _missionConfig.HardBoundary.Vertices)
            {
                _dgvHardBoundary.Rows.Add(point.Lat.ToString("F8"), point.Lon.ToString("F8"));
            }

            UpdatePointCounts();
        }

        private bool _syncingSoftFromHard;

        /// <summary>
        /// Recompute the derived soft boundary when "auto from hard" is on and
        /// refresh its grid. Hooked into UpdatePointCounts so every hard-boundary
        /// mutation path (grid edit, paste, import, clear, add) picks it up.
        /// </summary>
        private void SyncSoftFromHard()
        {
            if (!_missionConfig.SoftBoundaryFromHard || _syncingSoftFromHard) return;
            try
            {
                _syncingSoftFromHard = true;
                _missionConfig.RegenerateSoftFromHard();
                _missionConfig.Save();
                _dgvSoftBoundary.Rows.Clear();
                foreach (var p in _missionConfig.SoftBoundary.Vertices)
                {
                    _dgvSoftBoundary.Rows.Add(p.Lat.ToString("F8"), p.Lon.ToString("F8"));
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Soft-from-hard sync failed - {ex.Message}");
            }
            finally
            {
                _syncingSoftFromHard = false;
            }
        }

        private void UpdatePointCounts()
        {
            SyncSoftFromHard();

            var softLabel = this.Controls.Find("lblSoftCount", true).FirstOrDefault() as Label;
            var hardLabel = this.Controls.Find("lblHardCount", true).FirstOrDefault() as Label;
            var softSaved = this.Controls.Find("lblSoftSaved", true).FirstOrDefault() as Label;
            var hardSaved = this.Controls.Find("lblHardSaved", true).FirstOrDefault() as Label;
            string stamp = $"Saved {DateTime.Now:HH:mm:ss} to plugin config";

            if (softLabel != null)
                softLabel.Text = $"{_missionConfig.SoftBoundary.Vertices.Count} pts";
            if (hardLabel != null)
                hardLabel.Text = $"{_missionConfig.HardBoundary.Vertices.Count} pts";
            if (softSaved != null)
                softSaved.Text = _missionConfig.SoftBoundary.Vertices.Count > 0 ? stamp : "No points";
            if (hardSaved != null)
                hardSaved.Text = _missionConfig.HardBoundary.Vertices.Count > 0 ? stamp : "No points";
        }

        private void DeleteSelectedPoint(DataGridView dgv, FlightBoundary boundary)
        {
            try
            {
                var rows = dgv.SelectedRows.Cast<DataGridViewRow>().OrderByDescending(r => r.Index).ToList();
                if (rows.Count == 0 && dgv.CurrentCell != null)
                {
                    var r = dgv.Rows[dgv.CurrentCell.RowIndex];
                    if (r != null) rows.Add(r);
                }
                if (rows.Count == 0)
                {
                    CustomMessageBox.Show("Select a row in the grid first.", "Delete Point");
                    return;
                }
                foreach (var r in rows)
                {
                    int idx = r.Index;
                    if (idx >= 0 && idx < boundary.Vertices.Count)
                        boundary.Vertices.RemoveAt(idx);
                    dgv.Rows.RemoveAt(idx);
                }
                _missionConfig.Save();
                UpdatePointCounts();
                AutoDrawBoundariesIfEnabled();
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Delete failed: {ex.Message}", "Error");
            }
        }

        private void BtnClearVehicleFence_Click(object sender, EventArgs e)
        {
            if (CustomMessageBox.Show("Disable fence and clear all fence points on the connected vehicle?", "Confirm",
                CustomMessageBox.MessageBoxButtons.YesNo) != CustomMessageBox.DialogResult.Yes)
                return;
            var r = MPFenceUploader.ClearFence();
            CustomMessageBox.Show(r.Message, r.Success ? "Cleared" : "Failed");
        }

        private void PasteCoordinates(DataGridView dgv, FlightBoundary boundary, string boundaryType)
        {
            using (var inputForm = new Form())
            {
                inputForm.Width = 550;
                inputForm.Height = 400;
                inputForm.Text = $"Paste {boundaryType.ToUpper()} Boundary Coordinates";
                inputForm.StartPosition = FormStartPosition.CenterParent;
                inputForm.BackColor = Color.FromArgb(40, 40, 45);
                inputForm.FormBorderStyle = FormBorderStyle.FixedDialog;
                inputForm.MaximizeBox = false;

                var instructions = new Label
                {
                    Text = "Paste coordinates (one per line). Supported formats:\n" +
                           "- lon, lat (e.g., -75.7554276757985, 45.32367641417768)\n" +
                           "- lat, lon (e.g., 45.32367641417768, -75.7554276757985)\n" +
                           "Auto-detects format based on value ranges.",
                    Location = new Point(20, 20),
                    Size = new Size(500, 60),
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(instructions);

                var textBox = new TextBox
                {
                    Location = new Point(20, 90),
                    Size = new Size(500, 200),
                    Multiline = true,
                    ScrollBars = ScrollBars.Vertical,
                    BackColor = Color.FromArgb(30, 30, 33),
                    ForeColor = Color.White,
                    Font = new Font("Consolas", 10),
                };
                inputForm.Controls.Add(textBox);

                var chkReplace = new CheckBox
                {
                    Text = "Replace existing points (unchecked = append)",
                    Location = new Point(20, 300),
                    ForeColor = Color.White,
                    AutoSize = true,
                    Checked = true,
                };
                inputForm.Controls.Add(chkReplace);

                var btnOk = new Button
                {
                    Text = "Import",
                    Location = new Point(330, 330),
                    Size = new Size(90, 30),
                    DialogResult = DialogResult.OK,
                    BackColor = NOMADTheme.ACCENT,
                    ForeColor = Color.White,
                    FlatStyle = FlatStyle.Flat,
                };
                inputForm.Controls.Add(btnOk);

                var btnCancel = new Button
                {
                    Text = "Cancel",
                    Location = new Point(430, 330),
                    Size = new Size(90, 30),
                    DialogResult = DialogResult.Cancel,
                    FlatStyle = FlatStyle.Flat,
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(btnCancel);

                inputForm.AcceptButton = btnOk;
                inputForm.CancelButton = btnCancel;

                if (inputForm.ShowDialog() == DialogResult.OK)
                {
                    var points = ParseCoordinates(textBox.Text);
                    if (points.Count > 0)
                    {
                        if (chkReplace.Checked)
                        {
                            boundary.Vertices.Clear();
                            dgv.Rows.Clear();
                        }

                        foreach (var point in points)
                        {
                            boundary.Vertices.Add(point);
                            dgv.Rows.Add(point.Lat.ToString("F8"), point.Lon.ToString("F8"));
                        }

                        _missionConfig.Save();
                        UpdatePointCounts();
                        AutoDrawBoundariesIfEnabled();
                        CustomMessageBox.Show($"Imported {points.Count} points to {boundaryType} boundary.", "Success");
                    }
                    else
                    {
                        CustomMessageBox.Show("No valid coordinates found.", "Warning");
                    }
                }
            }
        }

        private List<GpsPoint> ParseCoordinates(string input)
        {
            var points = new List<GpsPoint>();
            var lines = input.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);

            foreach (var line in lines)
            {
                var cleanLine = line.Trim();
                if (string.IsNullOrEmpty(cleanLine)) continue;

                // Try to parse various formats
                var parts = cleanLine.Split(new[] { ',', '\t', ' ' }, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length >= 2)
                {
                    if (double.TryParse(parts[0].Trim(), out double val1) &&
                        double.TryParse(parts[1].Trim(), out double val2))
                    {
                        double lat, lon;

                        // Auto-detect format:
                        // Longitude typically ranges -180 to 180 (but for Ottawa area ~-75)
                        // Latitude for North America is typically 24 to 70
                        // If abs(val1) > 90, it's likely longitude
                        if (Math.Abs(val1) > 90)
                        {
                            // lon, lat format
                            lon = val1;
                            lat = val2;
                        }
                        else if (Math.Abs(val2) > 90)
                        {
                            // lat, lon format
                            lat = val1;
                            lon = val2;
                        }
                        else
                        {
                            // Both could be valid lat or lon
                            // For Ottawa area (lat ~45, lon ~-75), check for negative values
                            if (val1 < 0)
                            {
                                lon = val1;
                                lat = val2;
                            }
                            else if (val2 < 0)
                            {
                                lat = val1;
                                lon = val2;
                            }
                            else
                            {
                                // Default to lat, lon
                                lat = val1;
                                lon = val2;
                            }
                        }

                        points.Add(new GpsPoint(lat, lon));
                    }
                }
            }

            return points;
        }

        private void ClearBoundary(DataGridView dgv, FlightBoundary boundary)
        {
            if (CustomMessageBox.Show("Clear all boundary points?", "Confirm",
                CustomMessageBox.MessageBoxButtons.YesNo) == CustomMessageBox.DialogResult.Yes)
            {
                boundary.Vertices.Clear();
                dgv.Rows.Clear();
                _missionConfig.Save();
                UpdatePointCounts();
            }
        }

        private void AddManualPoint(DataGridView dgv, FlightBoundary boundary)
        {
            // Use current position or last point
            double lat = MainV2.comPort?.MAV?.cs?.lat ?? 45.0;
            double lon = MainV2.comPort?.MAV?.cs?.lng ?? -75.0;

            var point = new GpsPoint(lat, lon);
            boundary.Vertices.Add(point);
            dgv.Rows.Add(lat.ToString("F8"), lon.ToString("F8"));
            _missionConfig.Save();
            UpdatePointCounts();
            AutoDrawBoundariesIfEnabled();
        }

        private void AutoDrawBoundariesIfEnabled()
        {
            try
            {
                var chkAutoDraw = this.Controls.Find("chkAutoDraw", true);
                if (chkAutoDraw.Length > 0 && chkAutoDraw[0] is CheckBox chk && chk.Checked)
                {
                    MapOverlayManager.DrawBoundaries(_missionConfig);
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Auto-draw error - {ex.Message}");
            }
        }

        private void BtnImportKml_Click(object sender, EventArgs e)
        {
            using (var ofd = new OpenFileDialog
            {
                Filter = "KML/KMZ Files|*.kml;*.kmz|All Files|*.*",
                Title = "Import Boundary from KML"
            })
            {
                if (ofd.ShowDialog() == DialogResult.OK)
                {
                    try
                    {
                        string content;

                        // KMZ files are zipped KML - extract the KML content
                        if (ofd.FileName.EndsWith(".kmz", StringComparison.OrdinalIgnoreCase))
                        {
                            using (var zip = ZipFile.OpenRead(ofd.FileName))
                            {
                                var kmlEntry = zip.Entries.FirstOrDefault(e =>
                                    e.Name.EndsWith(".kml", StringComparison.OrdinalIgnoreCase));
                                if (kmlEntry == null)
                                {
                                    CustomMessageBox.Show("No KML file found inside KMZ archive.", "Error");
                                    return;
                                }
                                using (var sr = new StreamReader(kmlEntry.Open()))
                                {
                                    content = sr.ReadToEnd();
                                }
                            }
                        }
                        else
                        {
                            content = File.ReadAllText(ofd.FileName);
                        }

                        var points = ParseKmlCoordinates(content);

                        if (points.Count > 0)
                        {
                            var result = CustomMessageBox.Show(
                                $"Import {points.Count} points as Soft (Yes) or Hard (No) boundary?",
                                "Select Boundary Type",
                                CustomMessageBox.MessageBoxButtons.YesNo);

                            if (result == CustomMessageBox.DialogResult.Yes)
                            {
                                _missionConfig.SoftBoundary.Vertices = points;
                            }
                            else
                            {
                                _missionConfig.HardBoundary.Vertices = points;
                            }
                            _missionConfig.Save();
                            LoadBoundaries();
                            AutoDrawBoundariesIfEnabled();
                            CustomMessageBox.Show($"Imported {points.Count} boundary points from KML.", "Success");
                        }
                        else
                        {
                            CustomMessageBox.Show("No valid coordinates found in KML file.", "Warning");
                        }
                    }
                    catch (Exception ex)
                    {
                        CustomMessageBox.Show($"Error importing KML: {ex.Message}", "Error");
                    }
                }
            }
        }

        private List<GpsPoint> ParseKmlCoordinates(string kmlContent)
        {
            var points = new List<GpsPoint>();

            var coordsMatch = System.Text.RegularExpressions.Regex.Match(
                kmlContent, @"<coordinates>\s*(.*?)\s*</coordinates>",
                System.Text.RegularExpressions.RegexOptions.Singleline);

            if (coordsMatch.Success)
            {
                var coordString = coordsMatch.Groups[1].Value;
                var coordPairs = coordString.Split(new[] { ' ', '\n', '\r', '\t' },
                    StringSplitOptions.RemoveEmptyEntries);

                foreach (var pair in coordPairs)
                {
                    var parts = pair.Split(',');
                    if (parts.Length >= 2)
                    {
                        if (double.TryParse(parts[0], out double lon) &&
                            double.TryParse(parts[1], out double lat))
                        {
                            points.Add(new GpsPoint(lat, lon));
                        }
                    }
                }
            }

            return points;
        }

        private void BtnImportGoogleMaps_Click(object sender, EventArgs e)
        {
            using (var inputForm = new Form())
            {
                inputForm.Width = 500;
                inputForm.Height = 300;
                inputForm.Text = "Import Coordinates";
                inputForm.StartPosition = FormStartPosition.CenterParent;
                inputForm.BackColor = Color.FromArgb(40, 40, 45);
                inputForm.FormBorderStyle = FormBorderStyle.FixedDialog;
                inputForm.MaximizeBox = false;
                inputForm.MinimizeBox = false;

                var label = new Label
                {
                    Left = 20, Top = 20, Width = 440,
                    Text = "Paste coordinates (one per line, format: lat,lon or lon,lat):",
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(label);

                var textBox = new TextBox
                {
                    Left = 20, Top = 50, Width = 440, Height = 120,
                    Multiline = true, ScrollBars = ScrollBars.Vertical,
                    BackColor = Color.FromArgb(30, 30, 33),
                    ForeColor = Color.White,
                    Font = new Font("Consolas", 10),
                };
                inputForm.Controls.Add(textBox);

                var chkReplace = new CheckBox
                {
                    Text = "Replace existing points (unchecked = append)",
                    Location = new Point(20, 180),
                    ForeColor = Color.White,
                    AutoSize = true,
                    Checked = true,
                };
                inputForm.Controls.Add(chkReplace);

                var cmbTarget = new ComboBox
                {
                    Location = new Point(20, 210),
                    Size = new Size(200, 25),
                    DropDownStyle = ComboBoxStyle.DropDownList,
                    BackColor = Color.FromArgb(50, 50, 53),
                    ForeColor = Color.White,
                };
                cmbTarget.Items.AddRange(new object[] { "Soft Boundary", "Hard Boundary" });
                cmbTarget.SelectedIndex = 0;
                inputForm.Controls.Add(cmbTarget);

                var btnOk = new Button
                {
                    Text = "Import", Left = 300, Width = 80, Top = 240,
                    DialogResult = DialogResult.OK,
                    BackColor = NOMADTheme.ACCENT,
                    ForeColor = Color.White,
                    FlatStyle = FlatStyle.Flat,
                };
                inputForm.Controls.Add(btnOk);

                var btnCancel = new Button
                {
                    Text = "Cancel", Left = 390, Width = 80, Top = 240,
                    DialogResult = DialogResult.Cancel,
                    FlatStyle = FlatStyle.Flat,
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(btnCancel);

                inputForm.AcceptButton = btnOk;
                inputForm.CancelButton = btnCancel;

                if (inputForm.ShowDialog() == DialogResult.OK && !string.IsNullOrWhiteSpace(textBox.Text))
                {
                    var points = ParseCoordinates(textBox.Text);
                    if (points.Count > 0)
                    {
                        var boundary = cmbTarget.SelectedIndex == 0
                            ? _missionConfig.SoftBoundary
                            : _missionConfig.HardBoundary;
                        var dgv = cmbTarget.SelectedIndex == 0
                            ? _dgvSoftBoundary
                            : _dgvHardBoundary;

                        if (chkReplace.Checked)
                        {
                            boundary.Vertices.Clear();
                            dgv.Rows.Clear();
                        }

                        foreach (var point in points)
                        {
                            boundary.Vertices.Add(point);
                            dgv.Rows.Add(point.Lat.ToString("F8"), point.Lon.ToString("F8"));
                        }

                        _missionConfig.Save();
                        UpdatePointCounts();
                        AutoDrawBoundariesIfEnabled();
                        CustomMessageBox.Show($"Imported {points.Count} points.", "Success");
                    }
                    else
                    {
                        CustomMessageBox.Show("No valid coordinates found.", "Warning");
                    }
                }
            }
        }

        private void BtnGetFromMP_Click(object sender, EventArgs e)
        {
            try
            {
                var mav = MainV2.comPort?.MAV;
                if (mav == null)
                {
                    CustomMessageBox.Show("Not connected to vehicle.", "Warning");
                    return;
                }

                var points = new List<GpsPoint>();

                // Try to access fencepoints via reflection (type varies by MP version)
                var fencepointsField = mav.GetType().GetProperty("fencepoints");
                if (fencepointsField != null)
                {
                    var fenceData = fencepointsField.GetValue(mav);
                    if (fenceData != null)
                    {
                        var valuesProperty = fenceData.GetType().GetProperty("Values");
                        if (valuesProperty != null)
                        {
                            var values = valuesProperty.GetValue(fenceData) as System.Collections.IEnumerable;
                            if (values != null)
                            {
                                foreach (var item in values)
                                {
                                    var latProp = item.GetType().GetField("lat");
                                    var lngProp = item.GetType().GetField("lng");
                                    if (latProp != null && lngProp != null)
                                    {
                                        var lat = Convert.ToDouble(latProp.GetValue(item));
                                        var lng = Convert.ToDouble(lngProp.GetValue(item));
                                        if (lat != 0 || lng != 0)
                                            points.Add(new GpsPoint(lat, lng));
                                    }
                                }
                            }
                        }
                    }
                }

                if (points.Count > 0)
                {
                    var result = CustomMessageBox.Show(
                        $"Import {points.Count} fence points as Soft (Yes) or Hard (No) boundary?",
                        "Select Boundary Type",
                        CustomMessageBox.MessageBoxButtons.YesNo);

                    if (result == CustomMessageBox.DialogResult.Yes)
                    {
                        _missionConfig.SoftBoundary.Vertices = points;
                    }
                    else
                    {
                        _missionConfig.HardBoundary.Vertices = points;
                    }
                    _missionConfig.Save();
                    LoadBoundaries();
                    AutoDrawBoundariesIfEnabled();
                    CustomMessageBox.Show($"Imported {points.Count} fence points.", "Success");
                }
                else
                {
                    CustomMessageBox.Show("No fence points found in Mission Planner.", "Warning");
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error getting fence: {ex.Message}", "Error");
            }
        }

        /// <summary>Read an editable boundary grid into a vertex list (skips blank/invalid rows).</summary>
        private static List<GpsPoint> ReadGridVertices(DataGridView dgv)
        {
            var points = new List<GpsPoint>();
            if (dgv == null) return points;
            foreach (DataGridViewRow row in dgv.Rows)
            {
                if (row.IsNewRow) continue;
                if (double.TryParse(Convert.ToString(row.Cells["Lat"].Value), out var lat) &&
                    double.TryParse(Convert.ToString(row.Cells["Lon"].Value), out var lon))
                {
                    points.Add(new GpsPoint(lat, lon));
                }
            }
            return points;
        }

        private List<GpsPoint> GetSelectedBoundaryVertices(out string boundaryName)
        {
            var soft = ReadGridVertices(_dgvSoftBoundary);
            var hard = ReadGridVertices(_dgvHardBoundary);
            bool hasSoft = soft != null && soft.Count > 0;
            bool hasHard = hard != null && hard.Count > 0;

            if (!hasSoft && !hasHard)
            {
                boundaryName = null;
                return null;
            }

            if (hasSoft && hasHard)
            {
                var result = CustomMessageBox.Show(
                    "Export Soft boundary (Yes) or Hard boundary (No)?",
                    "Select Boundary",
                    CustomMessageBox.MessageBoxButtons.YesNo);
                if (result == CustomMessageBox.DialogResult.Yes)
                {
                    boundaryName = "Soft";
                    return soft;
                }
                boundaryName = "Hard";
                return hard;
            }

            if (hasSoft) { boundaryName = "Soft"; return soft; }
            boundaryName = "Hard";
            return hard;
        }

        private void BtnExportToMPFence_Click(object sender, EventArgs e)
        {
            try
            {
                var hardVerts = _missionConfig.HardBoundary?.Vertices;
                if (hardVerts == null || hardVerts.Count < 3)
                {
                    CustomMessageBox.Show(
                        "Hard boundary needs at least 3 points before pushing to MP / drone.",
                        "Warning");
                    return;
                }
                var vertices = hardVerts;
                string boundaryName = "Hard";
                var strokeColor = Color.Red;
                var fillColor = Color.Transparent;
                string polyName = "NOMAD_Hard_Fence";

                // 1) Refresh the saved-config zone masks on both maps.
                try
                {
                    MapOverlayManager.DrawBoundaries(_missionConfig);
                }
                catch (Exception ex) { Log.Error($"Boundary zone draw failed - {ex.Message}"); }

                // 2) Keep Mission Planner's native Plan fence as an outline.
                bool planInjected = false;
                try
                {
                    planInjected = MapOverlayManager.ExportToMPGeoFence(
                        vertices,
                        polyName,
                        strokeColor,
                        fillColor,
                        3);
                }
                catch (Exception ex) { Log.Error($"Plan map inject failed - {ex.Message}"); }

                // 3) Upload to connected vehicle via MAVLink and set FENCE_* params.
                // For any termination action we also push LAND_SPEED at the
                // configured descent rate (CONOPS §4.5 requires >= 2 m/s);
                // warn-only flights leave LAND_SPEED untouched.
                string hardAction = _missionConfig.Failsafe.HardBoundaryAction;
                int fenceAction = MapFenceActionToParam(hardAction);
                int landSpeedCmS = (hardAction ?? "warn_and_kill").ToLower() == "warn_only"
                    ? 0
                    : (int)Math.Round(_missionConfig.TerminationDescentRateMps * 100);
                var upload = MPFenceUploader.UploadPolygon(
                    vertices,
                    _missionConfig.ReturnPoint,
                    _missionConfig.MaxAltitudeAglMeters,
                    fenceAction,
                    enableFence: true,
                    landSpeedCmS: landSpeedCmS);

                var parts = new List<string>();
                parts.Add($"Boundary: {boundaryName} ({vertices.Count} pts)");
                parts.Add(planInjected ? "Plan map: injected" : "Plan map: not available");
                parts.Add(upload.Success ? "Vehicle: " + upload.Message : "Vehicle: " + upload.Message);
                CustomMessageBox.Show(string.Join("\n", parts), upload.Success ? "Pushed to MP + Drone" : "Partial");
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error pushing fence: {ex.Message}", "Error");
            }
        }

        private static int MapFenceActionToParam(string action)
        {
            // ArduPilot FENCE_ACTION: 0=Report, 1=RTL or Land, 2=Always Land, 3=SmartRTL, 4=Brake, 5=SmartRTL-or-Land.
            // CONOPS §4.5 requires termination (vertical descent >=2 m/s) on
            // hard-boundary breach - RTL flies home horizontally first and
            // does NOT satisfy that, so both "kill" variants map to Land (2).
            switch ((action ?? "warn_and_kill").ToLower())
            {
                case "warn_only": return 0;
                case "auto_kill": return 2;
                case "warn_and_kill": return 2;
                default: return 2;
            }
        }

        private void SaveBoundaryFromGrid(DataGridView dgv, FlightBoundary boundary)
        {
            // The derived-soft sync repopulates the soft grid itself; its
            // CellValueChanged storm must not write partial rows back.
            if (_syncingSoftFromHard && dgv == _dgvSoftBoundary) return;
            try
            {
                boundary.Vertices.Clear();
                foreach (DataGridViewRow row in dgv.Rows)
                {
                    if (row.Cells["Lat"].Value != null && row.Cells["Lon"].Value != null)
                    {
                        if (double.TryParse(row.Cells["Lat"].Value.ToString(), out double lat) &&
                            double.TryParse(row.Cells["Lon"].Value.ToString(), out double lon))
                        {
                            boundary.Vertices.Add(new GpsPoint(lat, lon));
                        }
                    }
                }
                _missionConfig.Save();
                UpdatePointCounts();
            }
            catch (Exception ex)
            {
                Log.Error($"Save boundary from grid failed - {ex.Message}");
            }
        }
    }
}
