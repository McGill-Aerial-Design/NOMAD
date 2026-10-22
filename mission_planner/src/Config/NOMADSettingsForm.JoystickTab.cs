// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private TabPage CreateJoystickTab()
        {
            var tab = CreateTabPage("Inputs");
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
            AddSectionLabel(tab, "Gimbal", ref y);

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
            y += 36;

            // Camera tilt channel
            AddSectionLabel(tab, "Configured Position Input", ref y);

            _chkJoyPositionEnabled = AddCheckBox(tab, "Enable", 20, y, Color.LimeGreen);
            y += 28;

            AddLabel(tab, "Device:", 20, y);
            _cmbJoyPositionDevice = AddComboBox(tab, 90, y, 290, deviceList);
            y += 28;

            AddLabel(tab, "Position axis:", 20, y);
            _cmbJoyPositionAxis = AddComboBox(tab, 95, y, 80, axes);
            _chkJoyPositionInvert = AddCheckBox(tab, "invert", 185, y, Color.White);
            y += 28;

            AddLabel(tab, "Deadzone:", 20, y);
            _numJoyPositionDeadzone = AddNumericUpDown(tab, 95, y, 60, 0.00m, 0.50m, 0.08m, 2);
            y += 36;

            AddLabel(tab, "Position payload:", 20, y);
            _cmbJoyPositionPayload = AddComboBox(tab, 150, y, 430, Array.Empty<string>());
            _cmbJoyPositionPayload.SelectedIndexChanged += (sender, args) => UpdatePositionEligibility();
            y += 28;
            _lblJoyPositionEligibility = new Label { Location = new Point(20, y), Size = new Size(680, 40), ForeColor = Color.Goldenrod };
            tab.Controls.Add(_lblJoyPositionEligibility);
            y += 48;
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

            var combos = new ComboBox[6];
            for (int index = 0; index < combos.Length; index++)
            {
                AddLabel(tab, "SW" + (index / 2 + 1) + (index % 2 == 0 ? " up" : " down"), 20, y);
                AddLabel(tab, "Button", 100, y);
                _switchButtons[index] = AddNumericUpDown(tab, 155, y, 65, 0, 127, index);
                combos[index] = AddComboBox(tab, 235, y, 450, Array.Empty<string>());
                y += 32;
            }
            _cmbSw1Up = combos[0]; _cmbSw1Down = combos[1]; _cmbSw2Up = combos[2];
            _cmbSw2Down = combos[3]; _cmbSw3Up = combos[4]; _cmbSw3Down = combos[5];
            AddLabel(tab, "Initial held buttons never activate. Each active action needs its own physical button index.", 20, y, NOMADTheme.WARNING);
            tab.AutoScrollMinSize = new Size(0, y + 40);
            return tab;
        }

        private ComboBox[] SwitchActions => new[] { _cmbSw1Up, _cmbSw1Down, _cmbSw2Up, _cmbSw2Down, _cmbSw3Up, _cmbSw3Down };
        private static string BindingValue(ComboBox combo) => combo.SelectedItem is KeyValuePair<string, string> choice ? choice.Key : "";

        private static void SetBindingChoices(ComboBox combo, IEnumerable<KeyValuePair<string, string>> choices, string selected)
        {
            var options = choices.ToList();
            selected = selected ?? "";
            if (!options.Any(option => option.Key == selected))
            { options.Add(new KeyValuePair<string, string>(selected, "Unavailable: " + selected)); }
            combo.DisplayMember = "Value"; combo.ValueMember = "Key";
            combo.DropDownWidth = 900;
            combo.Items.Clear();
            foreach (var option in options) { combo.Items.Add(option); }
            combo.SelectedItem = options.First(option => option.Key == selected);
        }
        private void RefreshPayloadBindings(bool keepSelection)
        {
            var definitions = PayloadDefinition.Parse(Config.ActuatorDefinitions);
            var selected = keepSelection ? SwitchActions.Select(BindingValue).ToArray() : new[]
            {
                Config.JoystickSw1UpAction, Config.JoystickSw1DownAction, Config.JoystickSw2UpAction,
                Config.JoystickSw2DownAction, Config.JoystickSw3UpAction, Config.JoystickSw3DownAction,
            };
            string position = keepSelection ? BindingValue(_cmbJoyPositionPayload) : Config.JoystickPositionActuatorId;
            var actions = new List<KeyValuePair<string, string>> { new KeyValuePair<string, string>("None", "None") };
            foreach (var definition in definitions)
            {
                foreach (string operation in definition.Operations.Where(value => value != "position").Concat(new[] { "safe" }))
                {
                    string label = operation == "negative" ? definition.NegativeLabel : operation == "positive" ? definition.PositiveLabel :
                        operation == "safe" ? (definition.Behavior == "ServoBidirectional" ? definition.NeutralLabel : definition.SafeLabel) : definition.PrimaryLabel;
                    string output = (definition.Relay ? "Relay " : "Servo ") + definition.Channel;
                    string action = char.ToUpperInvariant(operation[0]) + operation.Substring(1);
                    actions.Add(new KeyValuePair<string, string>(definition.Id + ":" + operation,
                        output + " · " + action + " (" + label + ") — " + definition.Name + " [" + definition.Id + "]"));
                }
            }
            for (int index = 0; index < SwitchActions.Length; index++)
            { SetBindingChoices(SwitchActions[index], actions, string.IsNullOrEmpty(selected[index]) ? "None" : selected[index]); }
            var continuous = new List<KeyValuePair<string, string>> { new KeyValuePair<string, string>("", "None") };
            continuous.AddRange(definitions.Where(value => value.Continuous).Select(value =>
                new KeyValuePair<string, string>(value.Id, "Servo " + value.Channel + " — " + value.Name + " [" + value.Id + "]")));
            SetBindingChoices(_cmbJoyPositionPayload, continuous, position);
            UpdatePositionEligibility();
        }
        private void UpdatePositionEligibility()
        {
            if (_lblJoyPositionEligibility == null) { return; }
            string id = BindingValue(_cmbJoyPositionPayload);
            var definition = PayloadDefinition.Parse(Config.ActuatorDefinitions).FirstOrDefault(value => value.Id == id);
            _lblJoyPositionEligibility.Text = id == "" ? "Choose a nonhazardous position payload with no confirmation requirement." :
                definition?.Continuous == true ? "Continuous position is available for " + definition.Name + ". Center the input before activation." :
                "Unavailable for continuous position. Review this saved binding; it has not been replaced.";
        }
        private static string[] BuildDeviceComboList(System.Collections.Generic.IList<string> devices)
        {
            var list = new System.Collections.Generic.List<string> { "(none)" };
            foreach (var d in devices) list.Add(d);
            return list.ToArray();
        }
    }
}
