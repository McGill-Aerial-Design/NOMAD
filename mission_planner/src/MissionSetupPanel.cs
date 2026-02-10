// ============================================================
// NOMAD Mission Setup Panel
// ============================================================
// Configuration panel for AEAC 2026 competition settings.
// Handles building coordinates, team info, equipment, and
// all mission-specific configuration.
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Mission setup panel for configuring competition parameters.
    /// </summary>
    public class MissionSetupPanel : UserControl
    {
        private readonly MissionConfig _missionConfig;
        
        // Team Info
        private TextBox _txtTeamName;
        private TextBox _txtCallsign;
        private ComboBox _cmbCurrentTask;
        
        // Task 1 Building
        private TextBox _txtTask1BuildingLat;
        private TextBox _txtTask1BuildingLon;
        private TextBox _txtTask1BuildingName;
        private ComboBox _cmbTask1DoorOrientation;
        private NumericUpDown _nudTask1DoorWidth;
        private NumericUpDown _nudTask1DoorHeight;
        
        // Task 2 Building
        private TextBox _txtTask2BuildingLat;
        private TextBox _txtTask2BuildingLon;
        private TextBox _txtTask2BuildingName;
        private ComboBox _cmbTask2DoorOrientation;
        private NumericUpDown _nudTask2DoorWidth;
        private NumericUpDown _nudTask2DoorHeight;
        
        // Search Volume
        private NumericUpDown _nudSearchRadius;
        private NumericUpDown _nudSearchMaxAlt;
        
        // UAM Corridor
        private NumericUpDown _nudUamMinAlt;
        private NumericUpDown _nudUamMaxAlt;
        private NumericUpDown _nudYieldAlt;
        
        // Weight
        private NumericUpDown _nudTakeoffWeight;
        private NumericUpDown _nudPayloadWeight;
        private Label _lblPayloadFraction;

        public MissionSetupPanel(MissionConfig missionConfig)
        {
            _missionConfig = missionConfig;

            BackColor = Color.FromArgb(30, 30, 30);
            Dock = DockStyle.Fill;

            InitializeComponents();
            LoadConfiguration();
        }

        private void InitializeComponents()
        {
            var mainPanel = new Panel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
                Padding = new Padding(10),
            };

            int yOffset = 10;
            int groupWidth = 550;

            // ============================================================
            // Team Information
            // ============================================================
            var teamGroup = CreateGroupBox("Team Information", Color.FromArgb(0, 150, 200), yOffset, groupWidth, 130);

            var lblTeamName = CreateLabel("Team Name:", 15, 25);
            teamGroup.Controls.Add(lblTeamName);
            _txtTeamName = CreateTextBox(120, 22, 200);
            teamGroup.Controls.Add(_txtTeamName);

            var lblCallsign = CreateLabel("Callsign:", 15, 55);
            teamGroup.Controls.Add(lblCallsign);
            _txtCallsign = CreateTextBox(120, 52, 200);
            // _txtCallsign.PlaceholderText = "e.g., Nomad 101A"; // PlaceholderText not available in .NET 4.8
            teamGroup.Controls.Add(_txtCallsign);

            var lblTask = CreateLabel("Current Task:", 15, 85);
            teamGroup.Controls.Add(lblTask);
            _cmbCurrentTask = new ComboBox
            {
                Location = new Point(120, 82),
                Size = new Size(150, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
            };
            _cmbCurrentTask.Items.AddRange(new object[] { "Task 1 - Reconnaissance", "Task 2 - Fire Extinguishing" });
            _cmbCurrentTask.SelectedIndexChanged += (s, e) =>
            {
                _missionConfig.CurrentTask = _cmbCurrentTask.SelectedIndex + 1;
                _missionConfig.Save();
            };
            teamGroup.Controls.Add(_cmbCurrentTask);

            mainPanel.Controls.Add(teamGroup);
            yOffset += 140;

            // ============================================================
            // Task 1 Building Configuration
            // ============================================================
            var task1BuildingGroup = CreateGroupBox("Task 1 Building (Outdoor Recon)", Color.FromArgb(0, 150, 200), yOffset, groupWidth, 180);

            var lblT1Name = CreateLabel("Building Name:", 15, 25);
            task1BuildingGroup.Controls.Add(lblT1Name);
            _txtTask1BuildingName = CreateTextBox(120, 22, 200);
            task1BuildingGroup.Controls.Add(_txtTask1BuildingName);

            var lblT1Lat = CreateLabel("Latitude:", 15, 55);
            task1BuildingGroup.Controls.Add(lblT1Lat);
            _txtTask1BuildingLat = CreateTextBox(120, 52, 150);
            // _txtTask1BuildingLat.PlaceholderText = "e.g., 45.123456"; // PlaceholderText not available in .NET 4.8
            task1BuildingGroup.Controls.Add(_txtTask1BuildingLat);

            var lblT1Lon = CreateLabel("Longitude:", 280, 55);
            task1BuildingGroup.Controls.Add(lblT1Lon);
            _txtTask1BuildingLon = CreateTextBox(360, 52, 150);
            // _txtTask1BuildingLon.PlaceholderText = "e.g., -73.123456"; // PlaceholderText not available in .NET 4.8
            task1BuildingGroup.Controls.Add(_txtTask1BuildingLon);

            var lblT1Door = CreateLabel("Door Faces:", 15, 85);
            task1BuildingGroup.Controls.Add(lblT1Door);
            _cmbTask1DoorOrientation = CreateOrientationCombo(120, 82);
            task1BuildingGroup.Controls.Add(_cmbTask1DoorOrientation);

            var lblT1DoorW = CreateLabel("Door Width:", 15, 115);
            task1BuildingGroup.Controls.Add(lblT1DoorW);
            _nudTask1DoorWidth = CreateNumericUpDown(120, 112, 4.0m, 1, 10, 1);
            task1BuildingGroup.Controls.Add(_nudTask1DoorWidth);
            var lblM1 = CreateLabel("m", 205, 117);
            task1BuildingGroup.Controls.Add(lblM1);

            var lblT1DoorH = CreateLabel("Door Height:", 240, 115);
            task1BuildingGroup.Controls.Add(lblT1DoorH);
            _nudTask1DoorHeight = CreateNumericUpDown(330, 112, 4.0m, 1, 10, 1);
            task1BuildingGroup.Controls.Add(_nudTask1DoorHeight);
            var lblM2 = CreateLabel("m", 415, 117);
            task1BuildingGroup.Controls.Add(lblM2);

            var btnT1GetPos = new Button
            {
                Text = "[GPS] Get Current Position",
                Location = new Point(15, 145),
                Size = new Size(170, 25),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 63),
                ForeColor = Color.White,
            };
            btnT1GetPos.Click += (s, e) =>
            {
                var lat = MainV2.comPort?.MAV?.cs?.lat ?? 0;
                var lon = MainV2.comPort?.MAV?.cs?.lng ?? 0;
                if (lat != 0 && lon != 0)
                {
                    _txtTask1BuildingLat.Text = lat.ToString("F6");
                    _txtTask1BuildingLon.Text = lon.ToString("F6");
                }
            };
            task1BuildingGroup.Controls.Add(btnT1GetPos);

            mainPanel.Controls.Add(task1BuildingGroup);
            yOffset += 190;

            // ============================================================
            // Task 2 Building Configuration
            // ============================================================
            var task2BuildingGroup = CreateGroupBox("Task 2 Building (Indoor Extinguish)", Color.FromArgb(255, 150, 50), yOffset, groupWidth, 180);

            var lblT2Name = CreateLabel("Building Name:", 15, 25);
            task2BuildingGroup.Controls.Add(lblT2Name);
            _txtTask2BuildingName = CreateTextBox(120, 22, 200);
            task2BuildingGroup.Controls.Add(_txtTask2BuildingName);

            var lblT2Lat = CreateLabel("Latitude:", 15, 55);
            task2BuildingGroup.Controls.Add(lblT2Lat);
            _txtTask2BuildingLat = CreateTextBox(120, 52, 150);
            task2BuildingGroup.Controls.Add(_txtTask2BuildingLat);

            var lblT2Lon = CreateLabel("Longitude:", 280, 55);
            task2BuildingGroup.Controls.Add(lblT2Lon);
            _txtTask2BuildingLon = CreateTextBox(360, 52, 150);
            task2BuildingGroup.Controls.Add(_txtTask2BuildingLon);

            var lblT2Door = CreateLabel("Door Faces:", 15, 85);
            task2BuildingGroup.Controls.Add(lblT2Door);
            _cmbTask2DoorOrientation = CreateOrientationCombo(120, 82);
            task2BuildingGroup.Controls.Add(_cmbTask2DoorOrientation);

            var lblT2DoorW = CreateLabel("Door Width:", 15, 115);
            task2BuildingGroup.Controls.Add(lblT2DoorW);
            _nudTask2DoorWidth = CreateNumericUpDown(120, 112, 4.0m, 1, 10, 1);
            task2BuildingGroup.Controls.Add(_nudTask2DoorWidth);
            var lblM3 = CreateLabel("m", 205, 117);
            task2BuildingGroup.Controls.Add(lblM3);

            var lblT2DoorH = CreateLabel("Door Height:", 240, 115);
            task2BuildingGroup.Controls.Add(lblT2DoorH);
            _nudTask2DoorHeight = CreateNumericUpDown(330, 112, 4.0m, 1, 10, 1);
            task2BuildingGroup.Controls.Add(_nudTask2DoorHeight);
            var lblM4 = CreateLabel("m", 415, 117);
            task2BuildingGroup.Controls.Add(lblM4);

            var btnT2GetPos = new Button
            {
                Text = "[GPS] Get Current Position",
                Location = new Point(15, 145),
                Size = new Size(170, 25),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 63),
                ForeColor = Color.White,
            };
            btnT2GetPos.Click += (s, e) =>
            {
                var lat = MainV2.comPort?.MAV?.cs?.lat ?? 0;
                var lon = MainV2.comPort?.MAV?.cs?.lng ?? 0;
                if (lat != 0 && lon != 0)
                {
                    _txtTask2BuildingLat.Text = lat.ToString("F6");
                    _txtTask2BuildingLon.Text = lon.ToString("F6");
                }
            };
            task2BuildingGroup.Controls.Add(btnT2GetPos);

            mainPanel.Controls.Add(task2BuildingGroup);
            yOffset += 190;

            // ============================================================
            // Search Volume Configuration
            // ============================================================
            var searchGroup = CreateGroupBox("Search Volume (Both Tasks)", Color.FromArgb(100, 200, 100), yOffset, groupWidth, 100);

            var lblRadius = CreateLabel("Horizontal Radius:", 15, 30);
            searchGroup.Controls.Add(lblRadius);
            _nudSearchRadius = CreateNumericUpDown(140, 27, 15, 5, 50, 0);
            searchGroup.Controls.Add(_nudSearchRadius);
            var lblM5 = CreateLabel("m from building", 225, 32);
            searchGroup.Controls.Add(lblM5);

            var lblMaxSearchAlt = CreateLabel("Max Search Alt:", 15, 60);
            searchGroup.Controls.Add(lblMaxSearchAlt);
            _nudSearchMaxAlt = CreateNumericUpDown(140, 57, 10, 1, 50, 0);
            searchGroup.Controls.Add(_nudSearchMaxAlt);
            var lblM6 = CreateLabel("m AGL", 225, 62);
            searchGroup.Controls.Add(lblM6);

            mainPanel.Controls.Add(searchGroup);
            yOffset += 110;

            // ============================================================
            // UAM Corridor Configuration
            // ============================================================
            var uamGroup = CreateGroupBox("UAM Corridor / ATC", Color.FromArgb(150, 100, 200), yOffset, groupWidth, 130);

            var lblUamMin = CreateLabel("Corridor Min Alt:", 15, 30);
            uamGroup.Controls.Add(lblUamMin);
            _nudUamMinAlt = CreateNumericUpDown(140, 27, 20, 0, 100, 0);
            uamGroup.Controls.Add(_nudUamMinAlt);
            var lblM7 = CreateLabel("m AGL", 225, 32);
            uamGroup.Controls.Add(lblM7);

            var lblUamMax = CreateLabel("Corridor Max Alt:", 15, 60);
            uamGroup.Controls.Add(lblUamMax);
            _nudUamMaxAlt = CreateNumericUpDown(140, 57, 35, 0, 100, 0);
            uamGroup.Controls.Add(_nudUamMaxAlt);
            var lblM8 = CreateLabel("m AGL", 225, 62);
            uamGroup.Controls.Add(lblM8);

            var lblYield = CreateLabel("Yield Alt (Medevac):", 15, 90);
            uamGroup.Controls.Add(lblYield);
            _nudYieldAlt = CreateNumericUpDown(140, 87, 50, 0, 120, 0);
            uamGroup.Controls.Add(_nudYieldAlt);
            var lblM9 = CreateLabel("m AGL (above corridor)", 225, 92);
            uamGroup.Controls.Add(lblM9);

            mainPanel.Controls.Add(uamGroup);
            yOffset += 140;

            // ============================================================
            // Weight / Payload
            // ============================================================
            var weightGroup = CreateGroupBox("Weight Configuration (Scoring)", Color.FromArgb(200, 150, 50), yOffset, groupWidth, 130);

            var lblTOW = CreateLabel("Takeoff Weight:", 15, 30);
            weightGroup.Controls.Add(lblTOW);
            _nudTakeoffWeight = CreateNumericUpDown(140, 27, 5.0m, 0.1m, 25, 2);
            weightGroup.Controls.Add(_nudTakeoffWeight);
            var lblKg1 = CreateLabel("kg (with payload)", 225, 32);
            weightGroup.Controls.Add(lblKg1);

            var lblPayload = CreateLabel("Payload Weight:", 15, 60);
            weightGroup.Controls.Add(lblPayload);
            _nudPayloadWeight = CreateNumericUpDown(140, 57, 1.0m, 0, 10, 2);
            weightGroup.Controls.Add(_nudPayloadWeight);
            var lblKg2 = CreateLabel("kg", 225, 62);
            weightGroup.Controls.Add(lblKg2);

            var lblFraction = CreateLabel("Payload Fraction:", 15, 90);
            weightGroup.Controls.Add(lblFraction);
            _lblPayloadFraction = new Label
            {
                Text = "0.0%",
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = Color.LimeGreen,
                Location = new Point(140, 87),
                AutoSize = true,
            };
            weightGroup.Controls.Add(_lblPayloadFraction);

            _nudTakeoffWeight.ValueChanged += UpdatePayloadFraction;
            _nudPayloadWeight.ValueChanged += UpdatePayloadFraction;

            mainPanel.Controls.Add(weightGroup);
            yOffset += 140;

            // ============================================================
            // Save Button
            // ============================================================
            var btnSave = new Button
            {
                Text = "[SAVE] Save Configuration",
                Location = new Point(10, yOffset),
                Size = new Size(200, 40),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
            };
            btnSave.Click += BtnSave_Click;
            mainPanel.Controls.Add(btnSave);

            var btnReset = new Button
            {
                Text = "[RESET] Reset to Defaults",
                Location = new Point(220, yOffset),
                Size = new Size(150, 40),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(100, 60, 60),
                ForeColor = Color.White,
            };
            btnReset.Click += BtnReset_Click;
            mainPanel.Controls.Add(btnReset);

            Controls.Add(mainPanel);
        }

        private GroupBox CreateGroupBox(string text, Color foreColor, int y, int width, int height)
        {
            return new GroupBox
            {
                Text = text,
                ForeColor = foreColor,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, y),
                Size = new Size(width, height),
                BackColor = Color.FromArgb(45, 45, 48),
            };
        }

        private Label CreateLabel(string text, int x, int y)
        {
            return new Label
            {
                Text = text,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
                Location = new Point(x, y),
                AutoSize = true,
            };
        }

        private TextBox CreateTextBox(int x, int y, int width)
        {
            return new TextBox
            {
                Location = new Point(x, y),
                Size = new Size(width, 25),
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
                BorderStyle = BorderStyle.FixedSingle,
            };
        }

        private NumericUpDown CreateNumericUpDown(int x, int y, decimal value, decimal min, decimal max, int decimals)
        {
            return new NumericUpDown
            {
                Location = new Point(x, y),
                Size = new Size(80, 25),
                Value = Math.Min(Math.Max(value, min), max),
                Minimum = min,
                Maximum = max,
                DecimalPlaces = decimals,
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
            };
        }

        private ComboBox CreateOrientationCombo(int x, int y)
        {
            var cmb = new ComboBox
            {
                Location = new Point(x, y),
                Size = new Size(100, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
            };
            cmb.Items.AddRange(new object[] { "North", "South", "East", "West" });
            return cmb;
        }

        private void LoadConfiguration()
        {
            // Team Info
            _txtTeamName.Text = _missionConfig.TeamName;
            _txtCallsign.Text = _missionConfig.Callsign;
            _cmbCurrentTask.SelectedIndex = Math.Max(0, _missionConfig.CurrentTask - 1);

            // Task 1 Building
            _txtTask1BuildingName.Text = _missionConfig.Task1Building?.Name ?? "";
            if (_missionConfig.Task1Building?.Coordinates != null)
            {
                _txtTask1BuildingLat.Text = _missionConfig.Task1Building.Coordinates.Lat.ToString("F6");
                _txtTask1BuildingLon.Text = _missionConfig.Task1Building.Coordinates.Lon.ToString("F6");
            }
            SetOrientationCombo(_cmbTask1DoorOrientation, _missionConfig.Task1Building?.DoorOrientation);
            _nudTask1DoorWidth.Value = (decimal)(_missionConfig.Task1Building?.DoorWidthMeters ?? 4.0);
            _nudTask1DoorHeight.Value = (decimal)(_missionConfig.Task1Building?.DoorHeightMeters ?? 4.0);

            // Task 2 Building
            _txtTask2BuildingName.Text = _missionConfig.Task2Building?.Name ?? "";
            if (_missionConfig.Task2Building?.Coordinates != null)
            {
                _txtTask2BuildingLat.Text = _missionConfig.Task2Building.Coordinates.Lat.ToString("F6");
                _txtTask2BuildingLon.Text = _missionConfig.Task2Building.Coordinates.Lon.ToString("F6");
            }
            SetOrientationCombo(_cmbTask2DoorOrientation, _missionConfig.Task2Building?.DoorOrientation);
            _nudTask2DoorWidth.Value = (decimal)(_missionConfig.Task2Building?.DoorWidthMeters ?? 4.0);
            _nudTask2DoorHeight.Value = (decimal)(_missionConfig.Task2Building?.DoorHeightMeters ?? 4.0);

            // Search Volume
            _nudSearchRadius.Value = (decimal)_missionConfig.Task1SearchVolume.HorizontalRadiusMeters;
            _nudSearchMaxAlt.Value = (decimal)_missionConfig.Task1SearchVolume.MaxAltitudeAgl;

            // UAM Corridor
            _nudUamMinAlt.Value = (decimal)_missionConfig.UamCorridorMinAlt;
            _nudUamMaxAlt.Value = (decimal)_missionConfig.UamCorridorMaxAlt;
            _nudYieldAlt.Value = (decimal)_missionConfig.YieldAltitude;

            // Weight
            _nudTakeoffWeight.Value = Math.Max(0.1m, (decimal)_missionConfig.TakeoffWeightKg);
            _nudPayloadWeight.Value = (decimal)_missionConfig.PayloadWeightKg;
            UpdatePayloadFraction(null, null);
        }

        private void SetOrientationCombo(ComboBox cmb, string orientation)
        {
            if (string.IsNullOrEmpty(orientation))
            {
                cmb.SelectedIndex = -1;
                return;
            }
            
            var index = cmb.FindStringExact(orientation);
            cmb.SelectedIndex = index >= 0 ? index : -1;
        }

        private void UpdatePayloadFraction(object sender, EventArgs e)
        {
            if (_nudTakeoffWeight.Value > 0)
            {
                var fraction = _nudPayloadWeight.Value / _nudTakeoffWeight.Value * 100;
                _lblPayloadFraction.Text = $"{fraction:F1}%";
                _lblPayloadFraction.ForeColor = fraction >= 20 ? Color.LimeGreen : Color.Yellow;
            }
            else
            {
                _lblPayloadFraction.Text = "N/A";
            }
        }

        private void BtnSave_Click(object sender, EventArgs e)
        {
            try
            {
                // Team Info
                _missionConfig.TeamName = _txtTeamName.Text;
                _missionConfig.Callsign = _txtCallsign.Text;
                _missionConfig.CurrentTask = _cmbCurrentTask.SelectedIndex + 1;

                // Task 1 Building
                _missionConfig.Task1Building = _missionConfig.Task1Building ?? new BuildingInfo();
                _missionConfig.Task1Building.Name = _txtTask1BuildingName.Text;
                if (double.TryParse(_txtTask1BuildingLat.Text, out double t1Lat) &&
                    double.TryParse(_txtTask1BuildingLon.Text, out double t1Lon))
                {
                    _missionConfig.Task1Building.Coordinates = new GpsPoint(t1Lat, t1Lon);
                }
                _missionConfig.Task1Building.DoorOrientation = _cmbTask1DoorOrientation.SelectedItem?.ToString();
                _missionConfig.Task1Building.DoorWidthMeters = (double)_nudTask1DoorWidth.Value;
                _missionConfig.Task1Building.DoorHeightMeters = (double)_nudTask1DoorHeight.Value;

                // Task 2 Building
                _missionConfig.Task2Building = _missionConfig.Task2Building ?? new BuildingInfo();
                _missionConfig.Task2Building.Name = _txtTask2BuildingName.Text;
                if (double.TryParse(_txtTask2BuildingLat.Text, out double t2Lat) &&
                    double.TryParse(_txtTask2BuildingLon.Text, out double t2Lon))
                {
                    _missionConfig.Task2Building.Coordinates = new GpsPoint(t2Lat, t2Lon);
                }
                _missionConfig.Task2Building.DoorOrientation = _cmbTask2DoorOrientation.SelectedItem?.ToString();
                _missionConfig.Task2Building.DoorWidthMeters = (double)_nudTask2DoorWidth.Value;
                _missionConfig.Task2Building.DoorHeightMeters = (double)_nudTask2DoorHeight.Value;

                // Search Volume
                _missionConfig.Task1SearchVolume.HorizontalRadiusMeters = (double)_nudSearchRadius.Value;
                _missionConfig.Task1SearchVolume.MaxAltitudeAgl = (double)_nudSearchMaxAlt.Value;
                _missionConfig.Task2SearchVolume.HorizontalRadiusMeters = (double)_nudSearchRadius.Value;
                _missionConfig.Task2SearchVolume.MaxAltitudeAgl = (double)_nudSearchMaxAlt.Value;

                // UAM Corridor
                _missionConfig.UamCorridorMinAlt = (double)_nudUamMinAlt.Value;
                _missionConfig.UamCorridorMaxAlt = (double)_nudUamMaxAlt.Value;
                _missionConfig.YieldAltitude = (double)_nudYieldAlt.Value;

                // Weight
                _missionConfig.TakeoffWeightKg = (double)_nudTakeoffWeight.Value;
                _missionConfig.PayloadWeightKg = (double)_nudPayloadWeight.Value;

                // Save
                _missionConfig.Save();
                CustomMessageBox.Show("Configuration saved successfully!", "Success");
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error saving configuration: {ex.Message}", "Error");
            }
        }

        private void BtnReset_Click(object sender, EventArgs e)
        {
            if (CustomMessageBox.Show("Reset all mission settings to defaults?", "Confirm",
                CustomMessageBox.MessageBoxButtons.YesNo) == CustomMessageBox.DialogResult.Yes)
            {
                // Reset to new config
                var newConfig = new MissionConfig();
                
                _missionConfig.TeamName = newConfig.TeamName;
                _missionConfig.Callsign = newConfig.Callsign;
                _missionConfig.CurrentTask = newConfig.CurrentTask;
                _missionConfig.Task1Building = newConfig.Task1Building;
                _missionConfig.Task2Building = newConfig.Task2Building;
                _missionConfig.Task1SearchVolume = newConfig.Task1SearchVolume;
                _missionConfig.Task2SearchVolume = newConfig.Task2SearchVolume;
                _missionConfig.UamCorridorMinAlt = newConfig.UamCorridorMinAlt;
                _missionConfig.UamCorridorMaxAlt = newConfig.UamCorridorMaxAlt;
                _missionConfig.YieldAltitude = newConfig.YieldAltitude;
                _missionConfig.TakeoffWeightKg = newConfig.TakeoffWeightKg;
                _missionConfig.PayloadWeightKg = newConfig.PayloadWeightKg;
                
                _missionConfig.Save();
                LoadConfiguration();
            }
        }
    }
}
