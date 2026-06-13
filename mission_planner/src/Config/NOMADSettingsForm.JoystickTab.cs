// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private TabPage CreateJoystickTab()
        {
            var tab = CreateTabPage("Joystick");
            int y = 10;

            AddSectionLabel(tab, "Physical Joystick Routing", ref y);
            AddLabel(tab,
                "Two DirectInput sticks: one drives the CADDx gimbal,",
                10, y, Color.FromArgb(180, 180, 180));
            y += 18;
            AddLabel(tab,
                "the other drives the ZED tilt servo. RC override is never sent.",
                10, y, Color.FromArgb(180, 180, 180));
            y += 24;

            _chkJoyAutoSelect = AddCheckBox(tab,
                "Auto-pick first available device (and hot-plug retry)", 20, y, Color.LimeGreen);
            y += 28;

            var devices = NomadJoystickService.EnumerateDevices();
            var axes = new System.Collections.Generic.List<string>(NomadJoystickService.AxisNames).ToArray();
            string[] deviceList = BuildDeviceComboList(devices);

            // Gimbal channel
            AddSectionLabel(tab, "Gimbal (CADDx)", ref y);

            _chkJoyGimbalEnabled = AddCheckBox(tab, "Enable", 20, y, Color.LimeGreen);
            y += 28;

            AddLabel(tab, "Device:", 20, y);
            _cmbJoyGimbalDevice = AddComboBox(tab, 90, y, 290, deviceList);
            y += 28;

            AddLabel(tab, "Pitch axis:", 20, y);
            _cmbJoyGimbalPitchAxis = AddComboBox(tab, 95, y, 80, axes);
            _chkJoyGimbalPitchInvert = AddCheckBox(tab, "invert", 185, y, Color.White);
            AddLabel(tab, "Roll axis:", 260, y);
            _cmbJoyGimbalRollAxis = AddComboBox(tab, 325, y, 60, axes);
            y += 28;

            _chkJoyGimbalRollInvert = AddCheckBox(tab, "invert roll", 325, y, Color.White);
            y += 28;

            AddLabel(tab, "Deadzone:", 20, y);
            _numJoyGimbalDeadzone = AddNumericUpDown(tab, 95, y, 60, 0.00m, 0.50m, 0.08m, 2);
            AddLabel(tab, "Max rate (deg/s):", 175, y);
            _numJoyGimbalMaxRate = AddNumericUpDown(tab, 270, y, 70, 5, 200, 60);
            _numJoyGimbalMaxRate.ValueChanged += (s, e) =>
                GimbalController.MaxRateDegSec = (float)_numJoyGimbalMaxRate.Value;
            y += 36;

            // ZED channel
            AddSectionLabel(tab, "ZED Tilt Servo", ref y);

            _chkJoyZedEnabled = AddCheckBox(tab, "Enable", 20, y, Color.LimeGreen);
            y += 28;

            AddLabel(tab, "Device:", 20, y);
            _cmbJoyZedDevice = AddComboBox(tab, 90, y, 290, deviceList);
            y += 28;

            AddLabel(tab, "Tilt axis:", 20, y);
            _cmbJoyZedTiltAxis = AddComboBox(tab, 95, y, 80, axes);
            _chkJoyZedTiltInvert = AddCheckBox(tab, "invert", 185, y, Color.White);
            y += 28;

            AddLabel(tab, "Deadzone:", 20, y);
            _numJoyZedDeadzone = AddNumericUpDown(tab, 95, y, 60, 0.00m, 0.50m, 0.08m, 2);
            AddLabel(tab, "Max rate (us/s):", 175, y);
            _numJoyZedMaxRate = AddNumericUpDown(tab, 280, y, 70, 50, 4000, 400);
            y += 36;

            _btnJoyRefreshDevices = new Button
            {
                Text = "Refresh device list",
                Location = new Point(20, y),
                Size = new Size(160, 26),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(70, 70, 75),
                ForeColor = Color.White,
            };
            _btnJoyRefreshDevices.Click += (s, e) =>
            {
                var fresh = NomadJoystickService.EnumerateDevices();
                var freshList = BuildDeviceComboList(fresh);
                string keepG = _cmbJoyGimbalDevice.SelectedItem?.ToString();
                string keepZ = _cmbJoyZedDevice.SelectedItem?.ToString();
                string keepS = _cmbSwitchDevice?.SelectedItem?.ToString();
                _cmbJoyGimbalDevice.Items.Clear();
                _cmbJoyZedDevice.Items.Clear();
                _cmbJoyGimbalDevice.Items.AddRange(freshList);
                _cmbJoyZedDevice.Items.AddRange(freshList);
                SetComboBoxValue(_cmbJoyGimbalDevice, keepG);
                SetComboBoxValue(_cmbJoyZedDevice, keepZ);
                if (_cmbSwitchDevice != null)
                {
                    _cmbSwitchDevice.Items.Clear();
                    _cmbSwitchDevice.Items.AddRange(freshList);
                    SetComboBoxValue(_cmbSwitchDevice, keepS);
                }
                _lblJoyStatus.Text = $"{fresh.Count} device(s) detected.";
            };
            tab.Controls.Add(_btnJoyRefreshDevices);

            _lblJoyStatus = new Label
            {
                Text = $"{devices.Count} device(s) detected.",
                Location = new Point(190, y + 5),
                AutoSize = true,
                ForeColor = Color.FromArgb(180, 180, 180),
            };
            tab.Controls.Add(_lblJoyStatus);
            y += 36;

            // 3-position switch action mapping
            AddSectionLabel(tab, "Transmitter Switches (3-position)", ref y);
            AddLabel(tab,
                "Each switch centers in the middle; UP and DOWN each fire an action.",
                10, y, Color.FromArgb(180, 180, 180));
            y += 22;

            AddLabel(tab, "Device:", 20, y);
            _cmbSwitchDevice = AddComboBox(tab, 90, y, 290, deviceList);
            y += 28;

            string[] actionLabels = SwitchActionLabels();
            AddLabel(tab, "SW1 up:", 20, y);
            _cmbSw1Up = AddComboBox(tab, 80, y, 220, actionLabels);
            AddLabel(tab, "SW1 down:", 310, y);
            _cmbSw1Down = AddComboBox(tab, 370, y, 130, actionLabels);
            y += 28;

            AddLabel(tab, "SW2 up:", 20, y);
            _cmbSw2Up = AddComboBox(tab, 80, y, 220, actionLabels);
            AddLabel(tab, "SW2 down:", 310, y);
            _cmbSw2Down = AddComboBox(tab, 370, y, 130, actionLabels);
            y += 28;

            AddLabel(tab, "SW3 up:", 20, y);
            _cmbSw3Up = AddComboBox(tab, 80, y, 220, actionLabels);
            AddLabel(tab, "SW3 down:", 310, y);
            _cmbSw3Down = AddComboBox(tab, 370, y, 130, actionLabels);
            y += 36;

            // Kill switch
            AddSectionLabel(tab, "Kill Switch (pushbutton)", ref y);
            AddLabel(tab,
                "Pressing the radio's kill button commands LAND at the descent",
                10, y, Color.FromArgb(180, 180, 180));
            y += 18;
            AddLabel(tab,
                "rate below. Minimum 200 cm/s (2 m/s) per CONOPS 4.5.",
                10, y, Color.FromArgb(180, 180, 180));
            y += 22;

            _chkKillSwitchEnabled = AddCheckBox(tab, "Enable kill switch", 20, y, Color.IndianRed);
            y += 28;

            AddLabel(tab, "Descent (cm/s):", 20, y);
            _numKillLandSpeed = AddNumericUpDown(tab, 130, y, 80, 200, 800, 250);
            AddLabel(tab, "= 2.5 m/s default", 220, y, Color.Gray);
            y += 36;

            // Serial bridge
            AddSectionLabel(tab, "Serial Bridge (joystick.py)", ref y);
            AddLabel(tab, "Auto-launches a Python serial to virtual Xbox 360 bridge.",
                10, y, Color.FromArgb(180, 180, 180));
            y += 18;
            AddLabel(tab, "Requires pyserial + vgamepad + ViGEmBus driver.",
                10, y, Color.FromArgb(180, 180, 180));
            y += 24;

            _chkSerialBridgeEnabled = AddCheckBox(tab, "Enable headless serial bridge", 20, y, Color.LimeGreen);
            y += 26;

            AddLabel(tab, "Serial port:", 20, y);
            _cmbSerialBridgePort = AddComboBox(
                tab,
                110,
                y,
                80,
                System.IO.Ports.SerialPort.GetPortNames().OrderBy(port => port).ToArray());
            _cmbSerialBridgePort.DropDownStyle = ComboBoxStyle.DropDown;
            var btnRefreshPorts = new Button
            {
                Text = "Refresh",
                Location = new Point(195, y),
                Size = new Size(58, 23),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(70, 70, 75),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 7),
            };
            btnRefreshPorts.Click += (s, e) => RefreshSerialPortList();
            tab.Controls.Add(btnRefreshPorts);
            AddLabel(tab, "Baud:", 265, y);
            _numSerialBridgeBaud = AddNumericUpDown(tab, 310, y, 80, 1200, 1000000, 115200);
            y += 26;

            AddLabel(tab, "Python:", 20, y);
            _txtSerialBridgePython = AddTextBox(tab, 110, y, 230);
            y += 26;

            AddLabel(tab, "Script path:", 20, y);
            _txtSerialBridgeScript = AddTextBox(tab, 110, y, 230);
            y += 28;

            AddLabel(tab, "Status:", 20, y);
            _lblSerialBridgeStatus = new Label
            {
                Text = "(unknown)",
                Location = new Point(110, y),
                AutoSize = true,
                ForeColor = Color.FromArgb(180, 180, 180),
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            tab.Controls.Add(_lblSerialBridgeStatus);

            _serialBridgeStatusTimer = new System.Windows.Forms.Timer { Interval = 500 };
            _serialBridgeStatusTimer.Tick += (s, e) => RefreshSerialBridgeStatus();
            this.HandleCreated += (s, e) => _serialBridgeStatusTimer.Start();
            this.FormClosed   += (s, e) => { try { _serialBridgeStatusTimer?.Stop(); _serialBridgeStatusTimer?.Dispose(); } catch { } };

            return tab;
        }

        private void RefreshSerialPortList()
        {
            string selected = _cmbSerialBridgePort?.Text?.Trim();
            var ports = System.IO.Ports.SerialPort.GetPortNames().OrderBy(port => port).ToArray();
            _cmbSerialBridgePort.Items.Clear();
            _cmbSerialBridgePort.Items.AddRange(ports);
            _cmbSerialBridgePort.Text = selected;
        }

        private static readonly (string Id, string Label)[] SwitchActionMap =
        {
            ("None",          "(unassigned)"),
            ("DropToggleP1",  "Drop / Retract Payload 1"),
            ("DropToggleP2",  "Drop / Retract Payload 2"),
            ("DropToggleP3",  "Drop / Retract Payload 3"),
            ("ReelInP1",      "Reel In - Payload 1 (hold)"),
            ("ReelOutP1",     "Reel Out - Payload 1 (hold)"),
            ("ReelInP2",      "Reel In - Payload 2 (hold)"),
            ("ReelOutP2",     "Reel Out - Payload 2 (hold)"),
            ("FireWaterPump", "Fire Water Pump"),
        };

        private static string[] SwitchActionLabels()
        {
            var labels = new string[SwitchActionMap.Length];
            for (int i = 0; i < SwitchActionMap.Length; i++) labels[i] = SwitchActionMap[i].Label;
            return labels;
        }

        private static string LabelForActionId(string id)
        {
            foreach (var (Id, Label) in SwitchActionMap)
                if (string.Equals(Id, id, StringComparison.OrdinalIgnoreCase)) return Label;
            return SwitchActionMap[0].Label;
        }

        private static string ActionIdForLabel(string label)
        {
            foreach (var (Id, Label) in SwitchActionMap)
                if (string.Equals(Label, label, StringComparison.OrdinalIgnoreCase)) return Id;
            return "None";
        }

        private static string[] BuildDeviceComboList(System.Collections.Generic.IList<string> devices)
        {
            var list = new System.Collections.Generic.List<string> { "(none)" };
            foreach (var d in devices) list.Add(d);
            return list.ToArray();
        }
    }
}
