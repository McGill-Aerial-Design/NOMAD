// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using MissionPlanner;
using Newtonsoft.Json.Linq;
using JoystickBase = MissionPlanner.Joystick.JoystickBase;
using Wpf = System.Windows.Controls;
using Thickness = System.Windows.Thickness;
using Visibility = System.Windows.Visibility;
namespace NOMAD.MissionPlanner
{
    [global::MissionPlanner.Utilities.PreventTheming]
    internal sealed class InputBindingEditorForm : Form
    {
        private readonly NOMADConfig _config;
        private readonly int _index;
        private readonly string _id;
        private readonly float _originalDeadzone;
        private readonly Wpf.TextBox _name, _deadzone;
        private readonly Wpf.CheckBox _enabled, _invert;
        private readonly Wpf.ComboBox _device, _source, _axis, _button, _otherButton, _action, _otherAction;
        private readonly Wpf.StackPanel _axisRow, _buttonRow, _otherButtonRow, _otherActionRow, _deadzoneRow, _invertRow;
        private readonly Wpf.TextBlock _status;
        private readonly Wpf.Button _listenButton, _listenOther, _stopListen;
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
            _id = original.Id; _originalDeadzone = original.Deadzone;
            Text = index < 0 ? "Add control" : "Edit control";
            Font = NOMADTheme.Font(); BackColor = NOMADTheme.BG_DARK; ForeColor = NOMADTheme.TEXT_PRIMARY;
            ClientSize = new Size(680, 640); MinimumSize = new Size(620, 560); StartPosition = FormStartPosition.CenterParent;
            var root = new Wpf.DockPanel { Margin = new Thickness(12) };
            var body = new Wpf.StackPanel();
            var scroll = new Wpf.ScrollViewer { Content = body, VerticalScrollBarVisibility = Wpf.ScrollBarVisibility.Auto };
            Wpf.StackPanel Add(string label, System.Windows.FrameworkElement control)
            {
                var row = new Wpf.StackPanel { Margin = new Thickness(0, 0, 0, 8) };
                row.Children.Add(WpfUi.Text(label)); row.Children.Add(control); body.Children.Add(row);
                System.Windows.Automation.AutomationProperties.SetName(control, label); return row;
            }
            body.Children.Add(WpfUi.Section("Control"));
            _name = new Wpf.TextBox { Text = original.Name }; Add("Control name", _name);
            _enabled = new Wpf.CheckBox { Content = "Enabled", IsChecked = original.Enabled }; Add("Use control", _enabled);
            body.Children.Add(WpfUi.Section("Input"));
            _device = Combo(); Add("Device", _device);
            var refresh = WpfUi.Action("Refresh devices", RefreshDevices); body.Children.Add(refresh);
            SetChoices(_device, Array.Empty<KeyValuePair<string, string>>(), original.Device); RefreshDevices();
            _source = Combo(); SetChoices(_source, SourceChoices, original.Source);
            Add("Input type", _source);
            _axis = Combo(); foreach (string axis in InputBinding.Axes) { _axis.Items.Add(axis); } _axis.SelectedItem = original.Axis; _axisRow = Add("Axis", _axis);
            _button = Combo(); _otherButton = Combo();
            for (int i = 0; i < 128; i++) { _button.Items.Add(InputListenState.ButtonLabel(i)); _otherButton.Items.Add(InputListenState.ButtonLabel(i)); }
            _button.SelectedIndex = original.Button; _otherButton.SelectedIndex = original.OtherButton;
            _buttonRow = Add("Button / positive endpoint", _button); _otherButtonRow = Add("Negative endpoint button", _otherButton);
            _invert = new Wpf.CheckBox { Content = "Invert", IsChecked = original.Invert }; _invertRow = Add("Axis direction", _invert);
            _deadzone = new Wpf.TextBox { Text = original.Deadzone.ToString(System.Globalization.CultureInfo.CurrentCulture) };
            _deadzoneRow = Add("Continuous axis deadzone", _deadzone);
            body.Children.Add(WpfUi.Section("Action"));
            _action = Combo(); _otherAction = Combo(); Add("Action / positive endpoint", _action); _otherActionRow = Add("Negative endpoint action", _otherAction);
            _listenButton = WpfUi.Action("Listen for input", () => BeginListen(false));
            _listenOther = WpfUi.Action("Listen negative button", () => BeginListen(true));
            _stopListen = WpfUi.Action("Stop listening", () => StopListen("Listen canceled."));
            _stopListen.IsEnabled = false;
            var listenActions = new Wpf.WrapPanel(); listenActions.Children.Add(_listenButton); listenActions.Children.Add(_listenOther); listenActions.Children.Add(_stopListen);
            body.Children.Add(listenActions);
            _status = WpfUi.Text(""); body.Children.Add(_status);
            var error = WpfUi.Text(""); body.Children.Add(error);
            var use = WpfUi.Action("Use control", () =>
            {
                StopListen("");
                try { Inputs = BuildCandidate(); DialogResult = DialogResult.OK; }
                catch (Exception exception) { error.Text = exception.Message; }
            });
            var cancel = WpfUi.Action("Cancel", () => { DialogResult = DialogResult.Cancel; Close(); });
            use.IsDefault = true; cancel.IsCancel = true;
            var footer = new Wpf.WrapPanel(); footer.Children.Add(use); footer.Children.Add(cancel);
            Wpf.DockPanel.SetDock(footer, Wpf.Dock.Bottom); root.Children.Add(footer); root.Children.Add(scroll);
            root.PreviewKeyDown += (sender, args) => { if (args.Key == System.Windows.Input.Key.Escape)
                { DialogResult = DialogResult.Cancel; Close(); args.Handled = true; } };
            Controls.Add(WpfUi.Host(root));
            UpdateSource(original.Action, original.OtherAction);
            _source.SelectionChanged += (sender, args) => { StopListen("Input type changed. " + ReadyStatus()); UpdateSource(Value(_action), Value(_otherAction)); };
            _device.SelectionChanged += (sender, args) => StopListen("Device changed. " + ReadyStatus());
            _timer = new Timer { Interval = 50 }; _timer.Tick += PollListen; _timer.Start();
            UpdateListenAvailability(); _status.Text = ReadyStatus();
        }
        private static Wpf.ComboBox Combo() => new Wpf.ComboBox();
        private static string Value(Wpf.ComboBox combo) => combo.SelectedItem is KeyValuePair<string, string> item ? item.Key : "";
        private static void SetChoices(Wpf.ComboBox combo, IEnumerable<KeyValuePair<string, string>> choices, string selected)
        {
            selected = selected ?? ""; combo.Items.Clear(); combo.DisplayMemberPath = "Value";
            foreach (var choice in choices) { combo.Items.Add(choice); if (choice.Key == selected) { combo.SelectedItem = choice; } }
            if (combo.SelectedItem == null)
            { var missing = new KeyValuePair<string, string>(selected, "Unavailable: " + selected); combo.Items.Add(missing); combo.SelectedItem = missing; }
        }
        private void RefreshDevices()
        {
            StopListen(""); string selected = Value(_device);
            _available = NomadJoystickService.EnumerateDevices();
            var choices = _available.GroupBy(name => name, StringComparer.OrdinalIgnoreCase)
                .Select(group => new KeyValuePair<string, string>(group.Key,
                    group.Count() == 1 ? group.Key : "Unavailable (ambiguous): " + group.Key)).ToList();
            choices.Insert(0, new KeyValuePair<string, string>("", "Select a device"));
            SetChoices(_device, choices, selected);
            StopListen("");
        }
        private void UpdateSource(string action, string other)
        {
            string source = Value(_source); bool axis = source != "Button" && source != "ButtonPair";
            bool paired = source == "ButtonPair" || source == "AxisSwitch3";
            _axisRow.Visibility = axis ? Visibility.Visible : Visibility.Collapsed; _buttonRow.Visibility = !axis ? Visibility.Visible : Visibility.Collapsed;
            _otherButtonRow.Visibility = source == "ButtonPair" ? Visibility.Visible : Visibility.Collapsed;
            _deadzoneRow.Visibility = source == "Axis" ? Visibility.Visible : Visibility.Collapsed; _invertRow.Visibility = axis ? Visibility.Visible : Visibility.Collapsed;
            _otherActionRow.Visibility = paired ? Visibility.Visible : Visibility.Collapsed;
            _listenOther.Visibility = source == "ButtonPair" ? Visibility.Visible : Visibility.Collapsed;
            var choices = ActionChoices(_config, source == "Axis");
            SetChoices(_action, choices, action);
            SetChoices(_otherAction, choices, other);
            _source.ToolTip = source == "Axis" ? "Continuous axes support gimbal pitch/roll or nonhazardous position payloads without confirmations. Center before use." :
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
            string source = Value(_source);
            float deadzone = _originalDeadzone;
            if (source == "Axis" &&
                (!float.TryParse(_deadzone.Text, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.CurrentCulture, out deadzone) ||
                float.IsNaN(deadzone) || float.IsInfinity(deadzone) || deadzone < 0 || deadzone > 0.5f))
            { throw new ArgumentException("Enter a deadzone from 0 to 0.5."); }
            var binding = new InputBinding { Id = _id, Name = _name.Text.Trim(), Enabled = _enabled.IsChecked == true,
                Device = Value(_device), Source = source,
                Axis = _axis.SelectedItem?.ToString(), Button = _button.SelectedIndex, OtherButton = _otherButton.SelectedIndex,
                Invert = _invert.IsChecked == true, Deadzone = deadzone, Action = Value(_action), OtherAction = Value(_otherAction) };
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
            _second = second; _listen = new InputListenState(System.Diagnostics.Stopwatch.GetTimestamp());
            PollListen(this, EventArgs.Empty);
        }
        private string ReadyStatus() => Connected ? "Disconnect the aircraft before listening." :
            NomadJoystickService.ResolveDeviceName(Value(_device), _available) == null ? "Select one available, unambiguous device." :
            "Ready. Listen, then move a button, toggle or axis.";
        private void UpdateListenAvailability()
        {
            bool canListen = !Connected && NomadJoystickService.ResolveDeviceName(Value(_device), _available) != null;
            _listenButton.IsEnabled = _listenOther.IsEnabled = canListen && _listen == null;
            _stopListen.IsEnabled = _listen != null;
            if (_listen == null && (Connected || _status.Text == "Disconnect the aircraft before listening."))
            { _status.Text = ReadyStatus(); }
        }
        private void PollListen(object sender, EventArgs args)
        {
            UpdateListenAvailability(); if (_listen == null) { return; }
            _listen.Sample(Connected ? null : NomadJoystickService.SafeGetState(_capture), Connected,
                System.Diagnostics.Stopwatch.GetTimestamp());
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
            if (_stopListen != null) { _stopListen.IsEnabled = false; }
            if (_status != null) { _status.Text = status == "" ? ReadyStatus() : status; }
        }
        protected override void Dispose(bool disposing)
        {
            if (disposing) { _timer?.Stop(); _timer?.Dispose(); StopListen(""); }
            base.Dispose(disposing);
        }
    }
}
