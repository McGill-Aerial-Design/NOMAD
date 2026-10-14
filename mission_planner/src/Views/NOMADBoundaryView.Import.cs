// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Boundary View - KML, clipboard and coordinate import
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Reflection;
using System.Windows.Forms;
using MissionPlanner;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADBoundaryView
    {
        private void PasteCoordinates(bool inner)
        {
            if (IsDisposed || Disposing)
            {
                return;
            }
            using (var dialog = CreateCoordinateDialog(inner))
            {
                dialog.ShowDialog(this);
            }
        }

        private Form CreateCoordinateDialog(bool inner, string savePath = null)
        {
            var config = _missionConfig;
            var original = JsonConvert.SerializeObject(config);
            var innerOutline = config.SoftBoundary;
            var outerOutline = config.HardBoundary;
            var innerPoints = innerOutline.Vertices;
            var outerPoints = outerOutline.Vertices;
            var ownerHandle = Handle;
            var dialog = new Form
            {
                Text = "Paste " + (inner ? "inner" : "outer") + " advisory coordinates",
                ClientSize = new Size(520, 420), MinimumSize = new Size(360, 380),
                StartPosition = FormStartPosition.CenterParent, MaximizeBox = false, MinimizeBox = false,
                BackColor = Color.FromArgb(40, 40, 45), ForeColor = Color.White,
            };
            var layout = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 6,
                Padding = new Padding(16) };
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            foreach (var size in new[] { SizeType.AutoSize, SizeType.AutoSize, SizeType.Percent,
                SizeType.AutoSize, SizeType.AutoSize, SizeType.AutoSize })
            {
                layout.RowStyles.Add(new RowStyle(size, size == SizeType.Percent ? 100 : 0));
            }
            var order = new ComboBox { Name = "coordinateOrder", DropDownStyle = ComboBoxStyle.DropDownList,
                Width = 230, Dock = DockStyle.Left };
            order.Items.AddRange(new object[] { "Latitude, longitude", "Longitude, latitude" });
            order.SelectedIndex = 0;
            var hint = new Label { Name = "coordinateHint", AutoSize = true, Dock = DockStyle.Fill };
            Action updateHint = () => hint.Text = "One pair per line; comma, space or tab. Use . for decimals.\nExample: " +
                (order.SelectedIndex == 1 ? "-75.5, 45.5" : "45.5, -75.5");
            order.SelectedIndexChanged += (s, e) => updateHint();
            updateHint();
            var text = new TextBox { Name = "coordinateText", Multiline = true, AcceptsReturn = true,
                ScrollBars = ScrollBars.Vertical,
                Dock = DockStyle.Fill, Font = new Font("Consolas", 10), BackColor = Color.FromArgb(30, 30, 33),
                ForeColor = Color.White };
            var replace = new CheckBox { Name = "replaceCoordinates", Text = "Replace points (uncheck to append)",
                AutoSize = true, Checked = true };
            var error = new Label { Name = "coordinateError", AutoSize = true,
                Dock = DockStyle.Fill, ForeColor = Color.Gold };
            var buttons = new FlowLayoutPanel { AutoSize = true, Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.RightToLeft, WrapContents = false };
            var cancel = new Button { Text = "Cancel", Name = "cancelCoordinates", Size = new Size(90, 30),
                DialogResult = DialogResult.Cancel, FlatStyle = FlatStyle.Flat };
            var import = new Button { Text = "Import", Name = "importCoordinates", Size = new Size(90, 30),
                FlatStyle = FlatStyle.Flat, BackColor = NOMADTheme.ACCENT };
            import.Click += (s, e) =>
            {
                if (IsDisposed || Disposing || !IsHandleCreated || Handle != ownerHandle ||
                    !ReferenceEquals(config, _missionConfig) || !ReferenceEquals(innerOutline, config.SoftBoundary) ||
                    !ReferenceEquals(outerOutline, config.HardBoundary) || !ReferenceEquals(innerPoints, innerOutline.Vertices) ||
                    !ReferenceEquals(outerPoints, outerOutline.Vertices) || JsonConvert.SerializeObject(config) != original)
                {
                    error.Text = "Outline editor changed; cancel and reopen Paste coordinates.";
                    return;
                }
                if (!TryParseCoordinates(text.Text, order.SelectedIndex == 1, out var points, out var message) ||
                    !ApplyOutlinePoints(points, inner ? DialogResult.Yes : DialogResult.No, out message,
                        savePath, append: !replace.Checked))
                {
                    error.Text = message;
                    return;
                }
                LoadBoundaries(regenerateSoft: false);
                AutoDrawBoundariesIfEnabled();
                dialog.DialogResult = DialogResult.OK;
            };
            buttons.Controls.Add(cancel); buttons.Controls.Add(import);
            layout.Controls.Add(order, 0, 0); layout.Controls.Add(hint, 0, 1); layout.Controls.Add(text, 0, 2);
            layout.Controls.Add(replace, 0, 3); layout.Controls.Add(error, 0, 4); layout.Controls.Add(buttons, 0, 5);
            dialog.Controls.Add(layout);
            dialog.AcceptButton = import; dialog.CancelButton = cancel;
            return dialog;
        }

        private static bool TryParseCoordinates(string input, bool longitudeFirst,
            out List<GpsPoint> points, out string error)
        {
            points = null;
            error = "Enter at least one coordinate pair.";
            var parsed = new List<GpsPoint>();
            var lines = (input ?? "").Replace("\r\n", "\n").Replace('\r', '\n').Split('\n');
            for (int index = 0; index < lines.Length; index++)
            {
                var line = lines[index].Trim();
                if (line.Length == 0)
                {
                    continue;
                }
                var parts = line.Contains(",") ? line.Split(',') :
                    line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                error = $"Line {index + 1}: enter exactly two numbers using . for decimals.";
                if (parts.Length != 2 ||
                    !double.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out var first) ||
                    !double.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out var second))
                {
                    return false;
                }
                double latitude = longitudeFirst ? second : first;
                double longitude = longitudeFirst ? first : second;
                error = $"Line {index + 1}: latitude must be -90 to 90 and longitude -180 to 180, both finite.";
                if (!GeoMath.IsValidCoordinate(latitude, longitude))
                {
                    return false;
                }
                parsed.Add(new GpsPoint(latitude, longitude));
            }
            if (parsed.Count == 0)
            {
                error = "Enter at least one coordinate pair.";
                return false;
            }
            points = parsed;
            error = null;
            return true;
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
                                $"Import {points.Count} points as inner advisory (Yes) or outer advisory (No) outline?",
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

    }
}
