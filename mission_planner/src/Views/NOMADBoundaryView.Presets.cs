// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Boundary View - Preset Management & Monitoring Events
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Windows.Forms;
using MissionPlanner;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADBoundaryView
    {
        private void LoadPresets()
        {
            _presets.Clear();
            try
            {
                if (Directory.Exists(PresetsDir))
                {
                    foreach (var file in Directory.GetFiles(PresetsDir, "*.json"))
                    {
                        try
                        {
                            var preset = ReadPreset(file);
                            if (preset != null)
                            {
                                _presets.Add(preset);
                            }
                        }
                        catch (Exception ex)
                        {
                            Log.Error($"Error loading boundary preset '{Path.GetFileName(file)}' - {ex.Message}");
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Error loading presets - {ex.Message}");
            }
        }

        private BoundaryPreset ReadPreset(string file)
        {
            var preset = JsonConvert.DeserializeObject<BoundaryPreset>(File.ReadAllText(file), new JsonSerializerSettings
            {
                MissingMemberHandling = MissingMemberHandling.Error,
            });
            if (preset == null || preset.SoftBoundary == null || preset.HardBoundary == null ||
                !IsValidAdvisoryAltitude(preset.AdvisoryAltitudeDisplayThresholdMeters))
            {
                throw new JsonSerializationException("Invalid advisory boundary preset; original file preserved.");
            }
            return preset;
        }

        private void RefreshPresetCombo()
        {
            _cmbPresets?.Items.Clear();
            _cmbPresets?.Items.Add("-- Select Preset --");
            foreach (var preset in _presets.OrderByDescending(p => p.CreatedAt))
            {
                _cmbPresets?.Items.Add(preset.Name);
            }
            if (_cmbPresets != null)
                _cmbPresets.SelectedIndex = 0;
        }

        private void LoadSelectedPreset(object sender, EventArgs e)
        {
            if (_cmbPresets.SelectedIndex <= 0) return;

            var preset = _presets[_cmbPresets.SelectedIndex - 1];

            if (CustomMessageBox.Show($"Load preset '{preset.Name}'?\nThis will replace current boundaries.",
                "Confirm", CustomMessageBox.MessageBoxButtons.YesNo) == CustomMessageBox.DialogResult.Yes)
            {
                _missionConfig.SoftBoundary.Vertices = preset.SoftBoundary.ToList();
                _missionConfig.HardBoundary.Vertices = preset.HardBoundary.ToList();
                _missionConfig.AdvisoryAltitudeDisplayThresholdMeters =
                    IsValidAdvisoryAltitude(preset.AdvisoryAltitudeDisplayThresholdMeters)
                        ? preset.AdvisoryAltitudeDisplayThresholdMeters
                        : 122.0;

                _missionConfig.Save();
                LoadBoundaries();
                _nudMaxAlt.Value = (decimal)_missionConfig.AdvisoryAltitudeDisplayThresholdMeters;

                CustomMessageBox.Show($"Preset '{preset.Name}' loaded as local advisory display data.", "Success");
            }
        }

        private void SaveCurrentAsPreset(object sender, EventArgs e)
        {
            using (var inputForm = new Form())
            {
                inputForm.Width = 400;
                inputForm.Height = 200;
                inputForm.Text = "Save Boundary Preset";
                inputForm.StartPosition = FormStartPosition.CenterParent;
                inputForm.BackColor = Color.FromArgb(40, 40, 45);
                inputForm.FormBorderStyle = FormBorderStyle.FixedDialog;

                var lblName = new Label { Text = "Preset Name:", Location = new Point(20, 20), ForeColor = Color.White, AutoSize = true };
                inputForm.Controls.Add(lblName);

                var txtName = new TextBox
                {
                    Location = new Point(20, 45),
                    Size = new Size(340, 25),
                    BackColor = Color.FromArgb(50, 50, 53),
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(txtName);

                var lblDesc = new Label { Text = "Description:", Location = new Point(20, 75), ForeColor = Color.White, AutoSize = true };
                inputForm.Controls.Add(lblDesc);

                var txtDesc = new TextBox
                {
                    Location = new Point(20, 100),
                    Size = new Size(340, 25),
                    BackColor = Color.FromArgb(50, 50, 53),
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(txtDesc);

                var btnOk = new Button
                {
                    Text = "Save",
                    Location = new Point(180, 135),
                    Size = new Size(80, 30),
                    DialogResult = DialogResult.OK,
                    BackColor = NOMADTheme.ACCENT,
                    ForeColor = Color.White,
                    FlatStyle = FlatStyle.Flat,
                };
                inputForm.Controls.Add(btnOk);

                var btnCancel = new Button
                {
                    Text = "Cancel",
                    Location = new Point(270, 135),
                    Size = new Size(80, 30),
                    DialogResult = DialogResult.Cancel,
                    FlatStyle = FlatStyle.Flat,
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(btnCancel);

                if (inputForm.ShowDialog() == DialogResult.OK && !string.IsNullOrWhiteSpace(txtName.Text))
                {

                    var preset = new BoundaryPreset
                    {
                        Name = txtName.Text,
                        Description = txtDesc.Text,
                        CreatedAt = DateTime.Now,
                        SoftBoundary = _missionConfig.SoftBoundary.Vertices.ToList(),
                        HardBoundary = _missionConfig.HardBoundary.Vertices.ToList(),
                        AdvisoryAltitudeDisplayThresholdMeters = _missionConfig.AdvisoryAltitudeDisplayThresholdMeters,
                    };

                    try
                    {
                        if (!Directory.Exists(PresetsDir))
                            Directory.CreateDirectory(PresetsDir);

                        var fileName = $"{txtName.Text.Replace(" ", "_")}_{DateTime.Now:yyyyMMdd_HHmmss}.json";
                        var filePath = Path.Combine(PresetsDir, fileName);
                        var json = JsonConvert.SerializeObject(preset, Formatting.Indented);
                        File.WriteAllText(filePath, json);

                        _presets.Add(preset);
                        RefreshPresetCombo();

                        CustomMessageBox.Show($"Preset '{preset.Name}' saved.", "Success");
                    }
                    catch (Exception ex)
                    {
                        CustomMessageBox.Show($"Error saving preset: {ex.Message}", "Error");
                    }
                }
            }
        }

        private void DeleteSelectedPreset(object sender, EventArgs e)
        {
            if (_cmbPresets.SelectedIndex <= 0) return;

            var preset = _presets[_cmbPresets.SelectedIndex - 1];

            if (CustomMessageBox.Show($"Delete preset '{preset.Name}'?", "Confirm",
                CustomMessageBox.MessageBoxButtons.YesNo) == CustomMessageBox.DialogResult.Yes)
            {
                try
                {
                    // Find and delete file
                    var files = Directory.GetFiles(PresetsDir, "*.json");
                    foreach (var file in files)
                    {
                        var json = File.ReadAllText(file);
                        var p = JsonConvert.DeserializeObject<BoundaryPreset>(json);
                        if (p?.Name == preset.Name && p?.CreatedAt == preset.CreatedAt)
                        {
                            File.Delete(file);
                            break;
                        }
                    }

                    _presets.Remove(preset);
                    RefreshPresetCombo();
                    CustomMessageBox.Show("Preset deleted.", "Success");
                }
                catch (Exception ex)
                {
                    CustomMessageBox.Show($"Error deleting preset: {ex.Message}", "Error");
                }
            }
        }

        // ============================================================
        // Monitor Events
        // ============================================================

        private void Monitor_AdvisoryStatusChanged(object sender, AdvisoryBoundaryStatusEventArgs e)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => Monitor_AdvisoryStatusChanged(sender, e)));
                return;
            }

            switch (e.Status)
            {
                case AdvisoryOutlineStatus.InsideOutlines:
                    _statusPanel.BackColor = Color.FromArgb(50, 70, 90);
                    _lblStatus.Text = "[visual advisory] Position is inside the saved outlines";
                    break;

                case AdvisoryOutlineStatus.OutsideInnerOutline:
                    _statusPanel.BackColor = Color.FromArgb(130, 105, 25);
                    _lblStatus.Text = "[visual advisory] Position is outside the inner outline";
                    break;

                case AdvisoryOutlineStatus.OutsideOuterOutline:
                    _statusPanel.BackColor = Color.FromArgb(115, 70, 45);
                    _lblStatus.Text = "[visual advisory] Position is outside the outer outline";
                    break;

                case AdvisoryOutlineStatus.NoOutline:
                    _statusPanel.BackColor = Color.FromArgb(80, 80, 90);
                    _lblStatus.Text = "[visual advisory] No valid outline configured";
                    break;

                case AdvisoryOutlineStatus.NoPosition:
                    _statusPanel.BackColor = Color.FromArgb(80, 80, 90);
                    _lblStatus.Text = "[visual advisory] Waiting for Mission Planner position";
                    break;
            }
        }

        private static bool IsValidAdvisoryAltitude(double altitude)
        {
            return !double.IsNaN(altitude) && !double.IsInfinity(altitude) && altitude >= 0 && altitude <= 10000;
        }

        public void UpdateData()
        {
            UiAsync.RunSync(this, UpdateDataCore, "UpdateData");
        }

        private void UpdateDataCore()
        {
            try
            {
                var cs = MainV2.comPort?.MAV?.cs;
                if (cs != null)
                {
                    _lblPosition.Text = $"Position: {cs.lat:F6}, {cs.lng:F6}";
                    UpdateAdvisoryAltitude(ReadCurrentAltitude());
                }
            }
            catch { }
        }
    }
}
