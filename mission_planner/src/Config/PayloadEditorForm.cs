// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using Newtonsoft.Json.Linq;
using Forms = System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    [global::MissionPlanner.Utilities.PreventTheming]
    internal sealed class PayloadEditorForm : Forms.Form
    {
        internal static readonly KeyValuePair<string, string>[] BehaviorChoices =
        {
            new KeyValuePair<string, string>("ServoToggle", "Servo · two endpoints"),
            new KeyValuePair<string, string>("ServoPosition", "Servo · proportional position"),
            new KeyValuePair<string, string>("ServoBidirectional", "Servo · timed bidirectional motion"),
            new KeyValuePair<string, string>("RelayToggle", "Relay · on / off"),
            new KeyValuePair<string, string>("RelayPulse", "Relay · bounded pulse"),
        };
        private readonly JArray _definitions;
        private readonly int _index;
        private readonly PayloadDefinition _original;
        private readonly StackPanel _body;
        private readonly TextBox _name, _primary, _safeLabel, _negative, _neutralLabel, _positive;
        private readonly ComboBox _behavior;
        private readonly TextBox _channel, _minimum, _maximum, _neutral, _safe, _hold, _pulse, _count, _window;
        private readonly CheckBox _reversed, _hazardous, _requireNeutral;
        private readonly FrameworkElement _minimumRow, _maximumRow, _neutralRow, _safeRow, _reversedRow, _holdRow, _pulseRow;
        private readonly FrameworkElement _primaryRow, _safeLabelRow, _negativeRow, _neutralLabelRow, _positiveRow;
        private readonly TextBlock _channelLabel, _error;
        internal JArray Definitions { get; private set; }

        internal PayloadEditorForm(JArray definitions, int index, string newId)
        {
            _definitions = (JArray)(definitions?.DeepClone() ?? new JArray()); _index = index;
            _original = index < 0 ? new PayloadDefinition { Id = newId, Name = "New payload", Behavior = "ServoToggle", Channel = 1 }
                : _definitions[index].ToObject<PayloadDefinition>();
            Text = index < 0 ? "Add payload" : "Edit payload";
            ClientSize = new System.Drawing.Size(650, 620); MinimumSize = new System.Drawing.Size(560, 540);
            StartPosition = Forms.FormStartPosition.CenterParent;
            var root = new DockPanel { Margin = new Thickness(12) };
            var footer = new StackPanel(); DockPanel.SetDock(footer, System.Windows.Controls.Dock.Bottom); root.Children.Add(footer);
            _error = WpfUi.Text(""); footer.Children.Add(_error);
            var buttons = new WrapPanel(); footer.Children.Add(buttons);
            var save = WpfUi.Action("Use payload", () =>
            {
                try { Definitions = BuildCandidate(); DialogResult = Forms.DialogResult.OK; }
                catch (Exception error) { _error.Text = error.Message; }
            });
            save.IsDefault = true; buttons.Children.Add(save);
            var cancel = WpfUi.Action("Cancel", () => DialogResult = Forms.DialogResult.Cancel);
            cancel.IsCancel = true; buttons.Children.Add(cancel);
            _body = new StackPanel(); root.Children.Add(new ScrollViewer { Content = _body, VerticalScrollBarVisibility = ScrollBarVisibility.Auto });
            Controls.Add(WpfUi.Host(root));
            _name = TextInput(_original.Name); Add("Name", _name);
            _behavior = new ComboBox { DisplayMemberPath = "Value" };
            foreach (var type in BehaviorChoices)
            { _behavior.Items.Add(type); if (type.Key == _original.Behavior) _behavior.SelectedItem = type; }
            Add("Behavior", _behavior);
            _channel = Number(_original.Channel); var channelRow = Add("Servo channel (1–16)", _channel);
            _channelLabel = (TextBlock)channelRow.Children[0];
            _minimum = Number(_original.Minimum); _minimumRow = Add("Minimum PWM", _minimum);
            _maximum = Number(_original.Maximum); _maximumRow = Add("Maximum PWM", _maximum);
            _neutral = Number(_original.Neutral); _neutralRow = Add("Neutral PWM", _neutral);
            _safe = Number(_original.Safe); _safeRow = Add("Safe PWM", _safe);
            _reversed = Check("Reversed (inactive at maximum)", _original.Reversed); _reversedRow = Add("Toggle direction", _reversed);
            _hold = Number(_original.HoldMs); _holdRow = Add("Motion hold (ms)", _hold);
            _pulse = Number(_original.PulseMs); _pulseRow = Add("Relay pulse (ms)", _pulse);
            _hazardous = Check("Hazardous output", _original.Hazardous); Add("Interlocks", _hazardous);
            _hazardous.ToolTip = "Requires at least two confirmations and a return to neutral.";
            _count = Number(_original.Confirmations); Add("Distinct confirmations (0–3)", _count);
            _window = Number(_original.WindowMs); Add("Confirmation window (ms)", _window);
            _requireNeutral = Check("Require neutral before activation", _original.RequireNeutral); Add("Neutral interlock", _requireNeutral);
            _primary = TextInput(_original.PrimaryLabel); _primaryRow = Add("Action label", _primary);
            _safeLabel = TextInput(_original.SafeLabel); _safeLabelRow = Add("Safe label", _safeLabel);
            _negative = TextInput(_original.NegativeLabel); _negativeRow = Add("Negative direction label", _negative);
            _neutralLabel = TextInput(_original.NeutralLabel); _neutralLabelRow = Add("Neutral / Stop label", _neutralLabel);
            _positive = TextInput(_original.PositiveLabel); _positiveRow = Add("Positive direction label", _positive);
            _behavior.SelectionChanged += (s, e) => ShowBehavior();
            _hazardous.Checked += (s, e) => ShowBehavior(); _hazardous.Unchecked += (s, e) => ShowBehavior();
            _count.TextChanged += (s, e) => ShowBehavior(); ShowBehavior();
        }
        private Grid Add(string label, FrameworkElement control)
        {
            var row = new Grid { Margin = new Thickness(0, 0, 0, 6) };
            row.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(220) }); row.ColumnDefinitions.Add(new ColumnDefinition());
            row.Children.Add(WpfUi.Text(label)); Grid.SetColumn(control, 1); row.Children.Add(control); _body.Children.Add(row); return row;
        }
        private static TextBox TextInput(string text) => new TextBox { Text = text ?? "" };
        private static TextBox Number(int value) => TextInput(value.ToString(CultureInfo.InvariantCulture));
        private static CheckBox Check(string text, bool value) => new CheckBox { Content = text, IsChecked = value };
        private static int Number(TextBox input, int min, int max)
        {
            if (!int.TryParse(input.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out int value) || value < min || value > max)
                throw new ArgumentException($"Enter a whole number from {min} to {max}.");
            return value;
        }
        private void ShowBehavior()
        {
            string behavior = ((KeyValuePair<string, string>)_behavior.SelectedItem).Key;
            bool servo = behavior.StartsWith("Servo", StringComparison.Ordinal), motion = behavior == "ServoBidirectional";
            _channelLabel.Text = servo ? "Servo channel (1–16)" : "Relay index (0–15)";
            _minimumRow.Visibility = _maximumRow.Visibility = _neutralRow.Visibility = servo ? Visibility.Visible : Visibility.Collapsed;
            _safeRow.Visibility = behavior == "ServoPosition" ? Visibility.Visible : Visibility.Collapsed;
            _reversedRow.Visibility = behavior == "ServoToggle" ? Visibility.Visible : Visibility.Collapsed;
            _holdRow.Visibility = motion ? Visibility.Visible : Visibility.Collapsed;
            _pulseRow.Visibility = behavior == "RelayPulse" ? Visibility.Visible : Visibility.Collapsed;
            bool continuous = behavior == "ServoPosition" && _hazardous.IsChecked != true &&
                int.TryParse(_count.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out int count) && count == 0;
            _primaryRow.Visibility = !motion && !continuous ? Visibility.Visible : Visibility.Collapsed;
            _safeLabelRow.Visibility = !motion ? Visibility.Visible : Visibility.Collapsed;
            _negativeRow.Visibility = _neutralLabelRow.Visibility = _positiveRow.Visibility = motion ? Visibility.Visible : Visibility.Collapsed;
            _behavior.ToolTip = motion ? "Safe equals neutral. Live SERVOx_TRIM must match before motion; hold is bounded." :
                behavior == "ServoToggle" ? "Safe is the inactive endpoint: minimum, or maximum when reversed." :
                behavior == "RelayPulse" ? "Relay pulse is bounded; explicit Safe turns the relay off." :
                continuous ? "The slider sends position changes directly. Safe remains available." : "";
        }
        internal JArray BuildCandidate()
        {
            var value = JObject.FromObject(_original).ToObject<PayloadDefinition>();
            value.Name = _name.Text.Trim(); value.Behavior = ((KeyValuePair<string, string>)_behavior.SelectedItem).Key;
            value.Channel = Number(_channel, 0, 16);
            if (value.Behavior.StartsWith("Servo", StringComparison.Ordinal))
            {
                value.Minimum = Number(_minimum, 500, 2500); value.Maximum = Number(_maximum, 500, 2500);
                value.Neutral = Number(_neutral, 500, 2500);
            }
            if (value.Behavior == "ServoToggle") value.Reversed = _reversed.IsChecked == true;
            if (value.Behavior == "ServoBidirectional") value.HoldMs = Number(_hold, 50, 1500);
            if (value.Behavior == "RelayPulse") value.PulseMs = Number(_pulse, 50, 1500);
            if (value.Behavior == "ServoToggle") value.Safe = value.Reversed ? value.Maximum : value.Minimum;
            else if (value.Behavior == "ServoBidirectional") value.Safe = value.Neutral;
            else if (value.Behavior == "ServoPosition") value.Safe = Number(_safe, 500, 2500);
            value.Hazardous = _hazardous.IsChecked == true; value.Confirmations = Number(_count, 0, 3);
            value.WindowMs = Number(_window, 500, 5000); value.RequireNeutral = _requireNeutral.IsChecked == true;
            value.PrimaryLabel = _primary.Text; value.SafeLabel = _safeLabel.Text; value.NegativeLabel = _negative.Text;
            value.NeutralLabel = _neutralLabel.Text; value.PositiveLabel = _positive.Text;
            var candidate = (JArray)_definitions.DeepClone();
            if (_index < 0) candidate.Add(JObject.FromObject(value)); else candidate[_index] = JObject.FromObject(value);
            PayloadDefinition.Parse(candidate); return candidate;
        }
    }
}
