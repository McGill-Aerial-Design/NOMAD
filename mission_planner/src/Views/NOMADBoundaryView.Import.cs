// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Boundary View - KML, clipboard and coordinate import
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
    }
}
