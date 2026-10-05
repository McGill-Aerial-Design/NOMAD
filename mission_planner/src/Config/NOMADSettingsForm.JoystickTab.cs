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
                "Select explicit USB HID devices for gimbal and configured actuator input.",
                10, y, Color.FromArgb(180, 180, 180));
            y += 18;
            AddLabel(tab,
                "A missing selected device is never replaced by an unrelated device.",
                10, y, Color.FromArgb(180, 180, 180));
            y += 24;

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

            // Camera tilt channel
            AddSectionLabel(tab, "Configured Position Input", ref y);

            _chkJoyPositionEnabled = AddCheckBox(tab, "Enable", 20, y, Color.LimeGreen);
            y += 28;

            AddLabel(tab, "Device:", 20, y);
            _cmbJoyPositionDevice = AddComboBox(tab, 90, y, 290, deviceList);
            y += 28;

            AddLabel(tab, "Tilt axis:", 20, y);
            _cmbJoyPositionAxis = AddComboBox(tab, 95, y, 80, axes);
            _chkJoyPositionInvert = AddCheckBox(tab, "invert", 185, y, Color.White);
            y += 28;

            AddLabel(tab, "Deadzone:", 20, y);
            _numJoyPositionDeadzone = AddNumericUpDown(tab, 95, y, 60, 0.00m, 0.50m, 0.08m, 2);
            y += 36;

            AddLabel(tab, "Actuator ID:", 20, y);
            _txtJoyPositionActuatorId = AddTextBox(tab, 110, y, 240);
            y += 32;
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
                string keepCameraTilt = _cmbJoyPositionDevice.SelectedItem?.ToString();
                string keepS = _cmbSwitchDevice?.SelectedItem?.ToString();
                _cmbJoyGimbalDevice.Items.Clear();
                _cmbJoyPositionDevice.Items.Clear();
                _cmbJoyGimbalDevice.Items.AddRange(freshList);
                _cmbJoyPositionDevice.Items.AddRange(freshList);
                SetDeviceComboValue(_cmbJoyGimbalDevice, keepG);
                SetDeviceComboValue(_cmbJoyPositionDevice, keepCameraTilt);
                if (_cmbSwitchDevice != null)
                {
                    _cmbSwitchDevice.Items.Clear();
                    _cmbSwitchDevice.Items.AddRange(freshList);
                    SetDeviceComboValue(_cmbSwitchDevice, keepS);
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

            string[] actionLabels = new[] { "None" };
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

            AddSectionLabel(tab, "Termination Button", ref y);
            AddLabel(tab, "Aircraft termination is unavailable in this plugin.",
                10, y, Color.IndianRed);
            y += 24;
            _chkKillSwitchEnabled = AddCheckBox(tab, "Monitor button requests (reports unavailable)",
                20, y, Color.IndianRed);
            y += 36;

            AddSectionLabel(tab, "Direct USB HID button mapping", ref y);
            var combos = new[] { _cmbSw1Up, _cmbSw1Down, _cmbSw2Up, _cmbSw2Down, _cmbSw3Up, _cmbSw3Down };
            for (int index = 0; index < combos.Length; index++)
            {
                combos[index].DropDownStyle = ComboBoxStyle.DropDown;
                AddLabel(tab, "Slot " + index + " button index:", 20, y);
                _switchButtons[index] = AddNumericUpDown(tab, 170, y, 65, 0, 127, index);
                y += 27;
            }
            var discover = new Button { Text = "Load runtime action labels", Location = new Point(20, y), Size = new Size(210, 28) };
            discover.Click += async (s, e) => await LoadActuatorConfigurationAsync();
            tab.Controls.Add(discover);

            return tab;
        }

        private readonly System.Collections.Generic.Dictionary<string, string> _actionIds =
            new System.Collections.Generic.Dictionary<string, string>();
        private void ApplyActuatorActions(System.Collections.Generic.IReadOnlyList<Connectivity.NomadActuator> actuators)
        {
            var combos = new[] { _cmbSw1Up, _cmbSw1Down, _cmbSw2Up, _cmbSw2Down, _cmbSw3Up, _cmbSw3Down };
            var preservedIds = combos.Select(combo => ActionIdForLabel(combo.Text)).ToArray();
            _actionIds.Clear();
            _actionIds["None"] = "None";
            foreach (var actuator in actuators)
            {
                foreach (var action in actuator.Actions)
                {
                    if (action.Control != "button")
                    {
                        continue;
                    }
                    string id = actuator.Id + ":" + action.Operation;
                    _actionIds[actuator.Name + " / " + action.Label + " [" + id + "]"] = id;
                }
            }
            for (int index = 0; index < combos.Length; index++)
            {
                var combo = combos[index];
                string keep = preservedIds[index];
                combo.Items.Clear();
                combo.Items.AddRange(_actionIds.Keys.ToArray());
                combo.Text = _actionIds.FirstOrDefault(entry => entry.Value == keep).Key ?? keep;
            }
        }
        private string ActionIdForLabel(string label) => _actionIds.TryGetValue(label ?? "", out var id) ? id : label ?? "None";

        private static string[] BuildDeviceComboList(System.Collections.Generic.IList<string> devices)
        {
            var list = new System.Collections.Generic.List<string> { "(none)" };
            foreach (var d in devices) list.Add(d);
            return list.ToArray();
        }
    }
}
