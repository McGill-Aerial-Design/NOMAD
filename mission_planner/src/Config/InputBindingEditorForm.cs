// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using MissionPlanner;
using Newtonsoft.Json.Linq;
using JoystickBase = MissionPlanner.Joystick.JoystickBase;
namespace NOMAD.MissionPlanner
{
    [global::MissionPlanner.Utilities.PreventTheming]
    internal sealed class InputBindingEditorForm : Form
    {
        private readonly NOMADConfig _config;
        private readonly int _index;
        private readonly string _id;
        private readonly TextBox _name;
        private readonly CheckBox _enabled, _invert;
        private readonly ComboBox _device, _source, _axis, _button, _otherButton, _action, _otherAction;
        private readonly NumericUpDown _deadzone;
        private readonly Control _axisRow, _buttonRow, _otherButtonRow, _otherActionRow, _deadzoneRow, _invertRow;
        private readonly Label _status, _hint;
        private readonly Button _listenButton, _listenOther, _stopListen;
        private IList<string> _available = new List<string>();
        internal static readonly KeyValuePair<string, string>[] SourceChoices =
        {
            new KeyValuePair<string, string>("Button", "Button"),
            new KeyValuePair<string, string>("ButtonPair", "Two buttons"),
            new KeyValuePair<string, string>("Axis", "Continuous axis"),
            new KeyValuePair<string, string>("AxisSwitch2", "2-position switch"),
            new KeyValuePair<string, string>("AxisSwitch3", "3-position switch"),
        };
        internal static string SourceLabel(string source) => SourceChoices.First(choice => choice.Key == source).Value;
        private readonly Timer _timer;
        private JoystickBase _capture;
        private InputListenState _listen;
        private bool _second;
        internal JArray Inputs { get; private set; }
        private static bool Connected => MainV2.comPort?.BaseStream?.IsOpen == true;
        internal InputBindingEditorForm(NOMADConfig config, int index)
        {
            _config = config; _index = index;
            var original = index < 0 ? new InputBinding { Id = "input_" + Guid.NewGuid().ToString("N") } : InputBinding.Parse(config.Inputs)[index];
            _id = original.Id;
            Text = index < 0 ? "Add control" : "Edit control";
            Font = NOMADTheme.Font(); BackColor = NOMADTheme.BG_DARK; ForeColor = NOMADTheme.TEXT_PRIMARY;
            ClientSize = new Size(680, 640); MinimumSize = new Size(620, 560); StartPosition = FormStartPosition.CenterParent;
            var scroll = new Panel { Dock = DockStyle.Fill, AutoScroll = true, Padding = new Padding(12) };
            var body = new TableLayoutPanel { Dock = DockStyle.Top, AutoSize = true, ColumnCount = 1 };
            body.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100)); scroll.Controls.Add(body); Controls.Add(scroll);
            Control Add(string label, Control control) { var row = ControlFactory.LabeledRow(label, control); body.Controls.Add(row); return row; }
            body.Controls.Add(ControlFactory.Label("Control", foreColor: NOMADTheme.ACCENT));
            _name = new TextBox { Text = original.Name, BackColor = NOMADTheme.INPUT_BG, ForeColor = NOMADTheme.TEXT_PRIMARY }; Add("Control name", _name);
            _enabled = ControlFactory.CheckBox("Enabled", original.Enabled); Add("Use control", _enabled);
            body.Controls.Add(ControlFactory.Label("Input", foreColor: NOMADTheme.ACCENT));
            _device = Combo(); Add("USB HID device", _device);
            var refresh = ControlFactory.Button("Refresh devices", (sender, args) => RefreshDevices(), width: 160); body.Controls.Add(refresh);
            NOMADSettingsForm.SetBindingChoices(_device, Array.Empty<KeyValuePair<string, string>>(), original.Device); RefreshDevices();
            _source = Combo(); NOMADSettingsForm.SetBindingChoices(_source, SourceChoices, original.Source);
            Add("Input type", _source);
            _axis = Combo(); _axis.Items.AddRange(InputBinding.Axes); _axis.SelectedItem = original.Axis; _axisRow = Add("Axis", _axis);
            _button = Combo(); _otherButton = Combo();
            for (int i = 0; i < 128; i++) { _button.Items.Add(InputListenState.ButtonLabel(i)); _otherButton.Items.Add(InputListenState.ButtonLabel(i)); }
            _button.SelectedIndex = original.Button; _otherButton.SelectedIndex = original.OtherButton;
            _buttonRow = Add("Button / positive endpoint", _button); _otherButtonRow = Add("Negative endpoint button", _otherButton);
            _invert = ControlFactory.CheckBox("Invert axis (also swaps 2-position active endpoint)", original.Invert); _invertRow = Add("Axis direction", _invert);
            _deadzone = ControlFactory.Numeric(0, 0.5m, (decimal)original.Deadzone); _deadzone.DecimalPlaces = 2; _deadzone.Increment = 0.01m;
            _deadzoneRow = Add("Continuous axis deadzone", _deadzone);
            body.Controls.Add(ControlFactory.Label("Action", foreColor: NOMADTheme.ACCENT));
            _action = Combo(); _otherAction = Combo(); Add("Action / positive endpoint", _action); _otherActionRow = Add("Negative endpoint action", _otherAction);
            _hint = ControlFactory.Label("", foreColor: NOMADTheme.WARNING); _hint.MaximumSize = new Size(610, 0); body.Controls.Add(_hint);
            _listenButton = ControlFactory.Button("Listen for input", (sender, args) => BeginListen(false), width: 160);
            _listenOther = ControlFactory.Button("Listen negative button", (sender, args) => BeginListen(true), width: 190);
            _stopListen = ControlFactory.Button("Stop listening", (sender, args) => StopListen("Listen canceled."), width: 130);
            _stopListen.Enabled = false;
            body.Controls.Add(ControlFactory.ButtonRow(_listenButton, _listenOther, _stopListen));
            _status = ControlFactory.Label("", foreColor: NOMADTheme.WARNING); _status.MaximumSize = new Size(610, 0); body.Controls.Add(_status);
            var error = ControlFactory.Label("", foreColor: NOMADTheme.WARNING); error.MaximumSize = new Size(610, 0); body.Controls.Add(error);
            var use = ControlFactory.Button("Use control", width: 130);
            use.Click += (sender, args) =>
            {
                StopListen("");
                try { Inputs = BuildCandidate(); DialogResult = DialogResult.OK; }
                catch (Exception exception) { error.Text = exception.Message; }
            };
            var cancel = ControlFactory.Button("Cancel"); cancel.DialogResult = DialogResult.Cancel;
            var footer = ControlFactory.ButtonRow(use, cancel); footer.Dock = DockStyle.Bottom; Controls.Add(footer);
            AcceptButton = use; CancelButton = cancel;
            UpdateSource(original.Action, original.OtherAction);
            _source.SelectedIndexChanged += (sender, args) => { StopListen("Input type changed. " + ReadyStatus()); UpdateSource(Value(_action), Value(_otherAction)); };
            _device.SelectedIndexChanged += (sender, args) => StopListen("Device changed. " + ReadyStatus());
            _timer = new Timer { Interval = 50 }; _timer.Tick += PollListen; _timer.Start();
            UpdateListenAvailability(); _status.Text = ReadyStatus();
        }
        private static ComboBox Combo() => new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList,
            BackColor = NOMADTheme.INPUT_BG, ForeColor = NOMADTheme.TEXT_PRIMARY, DropDownWidth = 750 };
        private static string Value(ComboBox combo) => combo.SelectedItem is KeyValuePair<string, string> item ? item.Key : "";
        private void RefreshDevices()
        {
            StopListen(""); string selected = Value(_device);
            _available = NomadJoystickService.EnumerateDevices();
            var choices = _available.GroupBy(name => name, StringComparer.OrdinalIgnoreCase)
                .Select(group => new KeyValuePair<string, string>(group.Key,
                    group.Count() == 1 ? group.Key : "Unavailable (ambiguous): " + group.Key)).ToList();
            choices.Insert(0, new KeyValuePair<string, string>("", "Select a device"));
            NOMADSettingsForm.SetBindingChoices(_device, choices, selected);
            StopListen("");
        }
        private void UpdateSource(string action, string other)
        {
            string source = Value(_source); bool axis = source != "Button" && source != "ButtonPair";
            bool paired = source == "ButtonPair" || source == "AxisSwitch3";
            _axisRow.Visible = axis; _buttonRow.Visible = !axis; _otherButtonRow.Visible = source == "ButtonPair";
            _deadzoneRow.Visible = source == "Axis"; _invertRow.Visible = axis; _otherActionRow.Visible = paired;
            _listenOther.Visible = source == "ButtonPair";
            var choices = ActionChoices(_config, source == "Axis");
            NOMADSettingsForm.SetBindingChoices(_action, choices, action);
            NOMADSettingsForm.SetBindingChoices(_otherAction, choices, other);
            _hint.Text = source == "Axis" ? "Continuous axes support gimbal pitch/roll or nonhazardous position payloads without confirmations. Center before use." :
                source == "AxisSwitch2" ? "2-position axis: inactive endpoint establishes neutral; active endpoint fires once. Returning inactive does not switch a latched payload off. Bind Safe separately if needed." :
                "Release / center before activation. A held input never counts as repeated confirmation. Opposing buttons request Safe.";
        }
        internal static List<KeyValuePair<string, string>> ActionChoices(NOMADConfig config, bool continuous)
        {
            var result = new List<KeyValuePair<string, string>> { new KeyValuePair<string, string>("", "None") };
            if (continuous)
            {
                result.Add(new KeyValuePair<string, string>("gimbal:pitch", "Gimbal pitch movement"));
                result.Add(new KeyValuePair<string, string>("gimbal:roll", "Gimbal roll movement"));
            }
            foreach (var payload in PayloadDefinition.Parse(config.ActuatorDefinitions))
            {
                var operations = continuous ? (payload.Continuous ? new[] { "position" } : Array.Empty<string>()) :
                    payload.Operations.Where(operation => operation != "position").Concat(new[] { "safe" });
                foreach (string operation in operations)
                {
                    string label = operation == "negative" ? payload.NegativeLabel : operation == "positive" ? payload.PositiveLabel :
                        operation == "safe" ? payload.SafeLabel : payload.PrimaryLabel;
                    result.Add(new KeyValuePair<string, string>(payload.Id + ":" + operation,
                        payload.Name + " · " + operation + " (" + label + ") · " + (payload.Relay ? "Relay " : "Servo ") + payload.Channel));
                }
            }
            return result;
        }
        internal static string ActionLabel(NOMADConfig config, string action) => action == "" ? "None" :
            ActionChoices(config, true).Concat(ActionChoices(config, false)).FirstOrDefault(value => value.Key == action).Value ?? "Unavailable: " + action;
        private JArray BuildCandidate()
        {
            var binding = new InputBinding { Id = _id, Name = _name.Text.Trim(), Enabled = _enabled.Checked,
                Device = Value(_device), Source = Value(_source),
                Axis = _axis.SelectedItem?.ToString(), Button = _button.SelectedIndex, OtherButton = _otherButton.SelectedIndex,
                Invert = _invert.Checked, Deadzone = (float)_deadzone.Value, Action = Value(_action), OtherAction = Value(_otherAction) };
            var candidate = (JArray)_config.Inputs.DeepClone();
            if (_index < 0) { candidate.Add(JObject.FromObject(binding)); } else { candidate[_index] = JObject.FromObject(binding); }
            InputBinding.Parse(candidate); return candidate;
        }
        private void BeginListen(bool second)
        {
            StopListen("");
            if (Connected) { _status.Text = "Disconnect the aircraft before listening."; return; }
            string device = NomadJoystickService.ResolveDeviceName(Value(_device), NomadJoystickService.EnumerateDevices());
            if (device == null) { _status.Text = "Select one available, unambiguous device."; return; }
            _capture = NomadJoystickService.CreateAndAcquire(device);
            if (_capture == null) { _status.Text = "Input device could not be acquired."; return; }
            _second = second; _listen = new InputListenState(DateTime.UtcNow); PollListen(this, EventArgs.Empty);
        }
        private string ReadyStatus() => Connected ? "Disconnect the aircraft before listening." :
            NomadJoystickService.ResolveDeviceName(Value(_device), _available) == null ? "Select one available, unambiguous device." :
            "Ready. Listen, then move a button, toggle or axis.";
        private void UpdateListenAvailability()
        {
            bool canListen = !Connected && NomadJoystickService.ResolveDeviceName(Value(_device), _available) != null;
            _listenButton.Enabled = _listenOther.Enabled = canListen && _listen == null;
            _stopListen.Enabled = _listen != null;
            if (_listen == null && (Connected || _status.Text == "Disconnect the aircraft before listening."))
            { _status.Text = ReadyStatus(); }
        }
        private void PollListen(object sender, EventArgs args)
        {
            UpdateListenAvailability(); if (_listen == null) { return; }
            _listen.Sample(Connected ? null : NomadJoystickService.SafeGetState(_capture), Connected, DateTime.UtcNow);
            _status.Text = _listen.Status; if (_listen.Active) { return; }
            var detected = _listen;
            StopListen(detected.Status);
            ApplyDetection(detected, _second);
        }
        private void ApplyDetection(InputListenState detected, bool second)
        {
            if (detected.Button.HasValue)
            {
                if (second) { _otherButton.SelectedIndex = detected.Button.Value; }
                else
                {
                    if (Value(_source) != "ButtonPair") { SetSource("Button"); }
                    _button.SelectedIndex = detected.Button.Value;
                }
            }
            else if (detected.Axis != null)
            {
                if (second)
                { _status.Text = "An axis moved. Listen again and press the negative endpoint button."; return; }
                if (Value(_source) != "AxisSwitch2" && Value(_source) != "AxisSwitch3") { SetSource("Axis"); }
                _axis.SelectedItem = detected.Axis;
            }
            _status.Text = detected.Status;
        }
        private void SetSource(string source) => _source.SelectedItem = SourceChoices.First(choice => choice.Key == source);
        private void StopListen(string status)
        {
            _listen?.Stop(status); _listen = null; NomadJoystickService.ReleaseJoy(ref _capture);
            if (_stopListen != null) { _stopListen.Enabled = false; }
            if (_status != null) { _status.Text = status == "" ? ReadyStatus() : status; }
        }
        protected override void Dispose(bool disposing)
        {
            if (disposing) { _timer?.Stop(); _timer?.Dispose(); StopListen(""); }
            base.Dispose(disposing);
        }
    }
}
