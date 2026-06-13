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
                        var json = File.ReadAllText(file);
                        var preset = JsonConvert.DeserializeObject<BoundaryPreset>(json);
                        if (preset != null)
                            _presets.Add(preset);
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Error loading presets - {ex.Message}");
            }
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
                _missionConfig.MaxAltitudeAglMeters = preset.MaxAltitudeMeters;

                _missionConfig.Save();
                LoadBoundaries();
                _nudMaxAlt.Value = (decimal)preset.MaxAltitudeMeters;

                CustomMessageBox.Show($"Preset '{preset.Name}' loaded.", "Success");
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
                        MaxAltitudeMeters = _missionConfig.MaxAltitudeAglMeters,
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

        private void Monitor_BoundaryStatusChanged(object sender, BoundaryStatusEventArgs e)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => Monitor_BoundaryStatusChanged(sender, e)));
                return;
            }

            switch (e.Status)
            {
                case "inside":
                    _statusPanel.BackColor = Color.FromArgb(40, 100, 40);
                    _lblStatus.Text = "[OK] Inside Boundaries";
                    _lblCountdown.Visible = false;
                    break;

                case "soft_violation":
                    _statusPanel.BackColor = Color.FromArgb(180, 150, 0);
                    _lblStatus.Text = "[!] SOFT BOUNDARY - Turn Around!";
                    _lblCountdown.Visible = false;
                    break;

                    case "hard_violation":
                    _statusPanel.BackColor = Color.FromArgb(180, 40, 40);
                    _lblStatus.Text = "[!!] HARD BOUNDARY VIOLATION!";
                    _lblCountdown.Visible = true;
                    break;

                case "no_position":
                    _statusPanel.BackColor = Color.FromArgb(80, 80, 90);
                    _lblStatus.Text = "[?] Waiting for GPS Position";
                    _lblCountdown.Visible = false;
                    break;
            }
        }

        private void Monitor_BoundaryViolation(object sender, BoundaryViolationEventArgs e)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => Monitor_BoundaryViolation(sender, e)));
                return;
            }

            if (e.BoundaryType == "hard" && _monitor?.KillCountdown != null)
            {
                _lblCountdown.Text = $"FORCED DESCENT IN {_monitor.KillCountdown} SECONDS!";
            }
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
                    _lblAltitude.Text = $"Alt: {cs.alt:F1}m / 122m";
                    if (cs.alt > 122)
                        _lblAltitude.ForeColor = Color.Red;
                    else
                        _lblAltitude.ForeColor = Color.White;
                }
            }
            catch { }
        }
    }
}
