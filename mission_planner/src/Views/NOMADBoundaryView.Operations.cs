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
        }

        private void UpdatePointCounts()
        {
        }

        private void DeleteSelectedPoint(DataGridView dgv, FlightBoundary boundary)
        {
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
            CustomMessageBox.Show("Boundary editing is currently unavailable.", "Unavailable");
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
        }

        private void AddManualPoint(DataGridView dgv, FlightBoundary boundary)
        {
        }

        private void AutoDrawBoundariesIfEnabled()
        {
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
                    BackColor = Color.FromArgb(0, 122, 204),
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
                        }

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
                var hardVerts = null as List<GpsPoint>;
                if (hardVerts == null || hardVerts.Count < 3)
                {
                    CustomMessageBox.Show(
                        "Hard boundary needs at least 3 points before pushing to MP / drone.",
                        "Warning");
                    return;
                }
                var vertices = hardVerts;
                string boundaryName = "Hard";
                bool isSoft = false;
                var strokeColor = Color.Red;
                var fillColor = Color.FromArgb(60, Color.Red);
                string polyName = "NOMAD_Hard_Fence";

                // 1) Draw on Data map overlay
                try
                {
                    MapOverlayManager.DrawPolygon(vertices, polyName, strokeColor, fillColor, isSoft ? 2 : 3);
                    MapOverlayManager.RefreshMap();
                }
                catch (Exception ex) { Log.Error($"Data map draw failed - {ex.Message}"); }

                // 2) Inject into MP's Plan-view geofence overlay
                bool planInjected = false;
                try
                {
                    planInjected = MapOverlayManager.ExportToMPGeoFence(vertices, polyName, strokeColor, fillColor, isSoft ? 2 : 3);
                }
                catch (Exception ex) { Log.Error($"Plan map inject failed - {ex.Message}"); }

                // 3) Upload to connected vehicle via MAVLink and set FENCE_* params.
                // For any "kill" action we also force LAND_SPEED to 200 cm/s
                // (2 m/s) so the descent meets CONOPS §4.5; warn-only flights
                // leave LAND_SPEED untouched.
                string hardAction = "warn_and_kill";
                int fenceAction = MapFenceActionToParam(hardAction);
                int landSpeedCmS = (hardAction ?? "warn_and_kill").ToLower() == "warn_only" ? 0 : 200;
                var upload = MPFenceUploader.UploadPolygon(
                    vertices,
                    null,
                    122.0,
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
        }
    }
}
