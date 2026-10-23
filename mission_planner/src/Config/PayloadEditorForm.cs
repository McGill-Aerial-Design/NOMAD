// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    [global::MissionPlanner.Utilities.PreventTheming]
    internal sealed class PayloadEditorForm : Form
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
        private readonly TableLayoutPanel _body;
        private readonly TextBox _name, _primary, _safeLabel, _negative, _neutralLabel, _positive;
        private readonly ComboBox _behavior;
        private readonly NumericUpDown _channel, _minimum, _maximum, _neutral, _safe, _hold, _pulse, _count, _window;
        private readonly CheckBox _reversed, _hazardous, _requireNeutral;
        private readonly Control _minimumRow, _maximumRow, _neutralRow, _safeRow, _reversedRow, _holdRow, _pulseRow;
        private readonly Control _primaryRow, _safeLabelRow, _negativeRow, _neutralLabelRow, _positiveRow;
        private readonly Label _channelLabel, _timedHint, _error;
        internal JArray Definitions { get; private set; }

        internal PayloadEditorForm(JArray definitions, int index, string newId)
        {
            _definitions = (JArray)(definitions?.DeepClone() ?? new JArray());
            _index = index;
            _original = index < 0 ? new PayloadDefinition
                { Id = newId, Name = "New payload", Behavior = "ServoToggle", Channel = 1 } :
                _definitions[index].ToObject<PayloadDefinition>();
            Text = index < 0 ? "Add payload" : "Edit payload";
            Font = NOMADTheme.Font(); BackColor = NOMADTheme.BG_DARK; ForeColor = NOMADTheme.TEXT_PRIMARY;
            ClientSize = new Size(650, 620); MinimumSize = new Size(560, 540);
            StartPosition = FormStartPosition.CenterParent;
            var scroll = new Panel { Dock = DockStyle.Fill, AutoScroll = true, Padding = new Padding(12) };
            _body = new TableLayoutPanel { Dock = DockStyle.Top, AutoSize = true, ColumnCount = 1 };
            _body.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            scroll.Controls.Add(_body); Controls.Add(scroll);
            Add("Stable ID", new TextBox { Text = _original.Id, ReadOnly = true, BackColor = NOMADTheme.INPUT_BG,
                ForeColor = NOMADTheme.TEXT_SECONDARY });
            _name = TextInput(_original.Name); Add("Name", _name);
            _behavior = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, BackColor = NOMADTheme.INPUT_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY, DisplayMember = "Value" };
            foreach (var type in BehaviorChoices)
            { _behavior.Items.Add(type); if (type.Key == _original.Behavior) { _behavior.SelectedItem = type; } }
            Add("Behavior", _behavior);
            _channel = ControlFactory.Numeric(0, 16, _original.Channel);
            var channelRow = Add("Servo channel (1–16)", _channel);
            _channelLabel = (Label)channelRow.Controls[0];
            _minimum = ControlFactory.Numeric(500, 2500, _original.Minimum); _minimumRow = Add("Minimum PWM", _minimum);
            _maximum = ControlFactory.Numeric(500, 2500, _original.Maximum); _maximumRow = Add("Maximum PWM", _maximum);
            _neutral = ControlFactory.Numeric(500, 2500, _original.Neutral); _neutralRow = Add("Neutral PWM (saved calibration)", _neutral);
            _safe = ControlFactory.Numeric(500, 2500, _original.Safe); _safeRow = Add("Safe PWM", _safe);
            _reversed = ControlFactory.CheckBox("Reversed (inactive at maximum)", _original.Reversed);
            _reversedRow = Add("Toggle direction", _reversed);
            _hold = ControlFactory.Numeric(50, 1500, _original.HoldMs); _holdRow = Add("Motion hold (ms)", _hold);
            _pulse = ControlFactory.Numeric(50, 1500, _original.PulseMs); _pulseRow = Add("Relay pulse (ms)", _pulse);
            _timedHint = ControlFactory.Label("", foreColor: NOMADTheme.WARNING, fontSize: NOMADTheme.SIZE_SMALL);
            _timedHint.MaximumSize = new Size(590, 0); _body.Controls.Add(_timedHint);
            _hazardous = ControlFactory.CheckBox("Hazardous output", _original.Hazardous); Add("Interlocks", _hazardous);
            _count = ControlFactory.Numeric(0, 3, _original.Confirmations); Add("Distinct confirmations (0–3)", _count);
            _window = ControlFactory.Numeric(500, 5000, _original.WindowMs); Add("Confirmation window (ms)", _window);
            _requireNeutral = ControlFactory.CheckBox("Require neutral before activation", _original.RequireNeutral);
            Add("Neutral interlock", _requireNeutral);
            var interlockHint = ControlFactory.Label("Hazardous outputs require at least two confirmations and neutral. " +
                "Changing behavior never reduces these interlocks.", foreColor: NOMADTheme.WARNING, fontSize: NOMADTheme.SIZE_SMALL);
            interlockHint.MaximumSize = new Size(590, 0); _body.Controls.Add(interlockHint);
            _primary = TextInput(_original.PrimaryLabel); _primaryRow = Add("Action label", _primary);
            _safeLabel = TextInput(_original.SafeLabel); _safeLabelRow = Add("Safe label", _safeLabel);
            _negative = TextInput(_original.NegativeLabel); _negativeRow = Add("Negative direction label", _negative);
            _neutralLabel = TextInput(_original.NeutralLabel); _neutralLabelRow = Add("Neutral / Stop label", _neutralLabel);
            _positive = TextInput(_original.PositiveLabel); _positiveRow = Add("Positive direction label", _positive);
            var footer = new Panel { Dock = DockStyle.Bottom, Height = 100, Padding = new Padding(12) };
            _error = new Label { Dock = DockStyle.Fill, ForeColor = NOMADTheme.WARNING };
            var save = ControlFactory.Button("Use payload", width: 120, height: 32);
            save.Click += (sender, args) =>
            {
                try { Definitions = BuildCandidate(); DialogResult = DialogResult.OK; }
                catch (Exception error) { _error.Text = error.Message; }
            };
            var cancel = ControlFactory.Button("Cancel", width: 90, height: 32); cancel.DialogResult = DialogResult.Cancel;
            var buttons = ControlFactory.ButtonRow(save, cancel); buttons.Dock = DockStyle.Bottom;
            footer.Controls.Add(_error); footer.Controls.Add(buttons); Controls.Add(footer);
            AcceptButton = save; CancelButton = cancel;
            _behavior.SelectedIndexChanged += (sender, args) => ShowBehavior(); ShowBehavior();
        }

        private TableLayoutPanel Add(string label, Control control)
        {
            var row = ControlFactory.LabeledRow(label, control); _body.Controls.Add(row); return row;
        }
        private static TextBox TextInput(string text) => new TextBox
            { Text = text, BackColor = NOMADTheme.INPUT_BG, ForeColor = NOMADTheme.TEXT_PRIMARY };

        private void ShowBehavior()
        {
            string behavior = ((KeyValuePair<string, string>)_behavior.SelectedItem).Key;
            bool servo = behavior.StartsWith("Servo", StringComparison.Ordinal), motion = behavior == "ServoBidirectional";
            _channelLabel.Text = servo ? "Servo channel (1–16)" : "Relay index (0–15)";
            _minimumRow.Visible = _maximumRow.Visible = _neutralRow.Visible = servo;
            _safeRow.Visible = behavior == "ServoPosition";
            _reversedRow.Visible = behavior == "ServoToggle";
            _holdRow.Visible = motion; _pulseRow.Visible = behavior == "RelayPulse";
            _primaryRow.Visible = _safeLabelRow.Visible = !motion;
            _negativeRow.Visible = _neutralLabelRow.Visible = _positiveRow.Visible = motion;
            _timedHint.Text = motion ? "Safe equals neutral. Live SERVOx_TRIM must match before motion; hold is bounded." :
                behavior == "ServoToggle" ? "Safe is the inactive endpoint: minimum, or maximum when reversed." :
                behavior == "RelayPulse" ? "Relay pulse is bounded; explicit Safe turns the relay off." : "";
        }

        internal JArray BuildCandidate()
        {
            var value = JObject.FromObject(_original).ToObject<PayloadDefinition>();
            value.Name = _name.Text.Trim(); value.Behavior = ((KeyValuePair<string, string>)_behavior.SelectedItem).Key;
            value.Channel = (int)_channel.Value;
            value.Minimum = (int)_minimum.Value; value.Maximum = (int)_maximum.Value; value.Neutral = (int)_neutral.Value;
            value.Reversed = _reversed.Checked; value.HoldMs = (int)_hold.Value; value.PulseMs = (int)_pulse.Value;
            if (value.Behavior == "ServoToggle") { value.Safe = value.Reversed ? value.Maximum : value.Minimum; }
            else if (value.Behavior == "ServoBidirectional") { value.Safe = value.Neutral; }
            else if (value.Behavior == "ServoPosition") { value.Safe = (int)_safe.Value; }
            value.Hazardous = _hazardous.Checked; value.Confirmations = (int)_count.Value;
            value.WindowMs = (int)_window.Value; value.RequireNeutral = _requireNeutral.Checked;
            value.PrimaryLabel = _primary.Text; value.SafeLabel = _safeLabel.Text;
            value.NegativeLabel = _negative.Text; value.NeutralLabel = _neutralLabel.Text; value.PositiveLabel = _positive.Text;
            var candidate = (JArray)_definitions.DeepClone();
            if (_index < 0) { candidate.Add(JObject.FromObject(value)); } else { candidate[_index] = JObject.FromObject(value); }
            PayloadDefinition.Parse(candidate);
            return candidate;
        }
    }
}
