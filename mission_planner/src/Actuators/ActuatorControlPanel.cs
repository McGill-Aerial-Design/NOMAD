// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    // Suppress key auto-repeat: confirmations represent distinct operator presses.
    internal sealed class ActuatorButton : ControlFactory.ThemedButton
    {
        private static bool Repeated(Message message) => (message.Msg == 0x100 || message.Msg == 0x104) &&
            (message.WParam.ToInt64() == 13 || message.WParam.ToInt64() == 32) &&
            (message.LParam.ToInt64() & (1L << 30)) != 0;
        public override bool PreProcessMessage(ref Message message) => Repeated(message) || base.PreProcessMessage(ref message);
        protected override void WndProc(ref Message message)
        { if (!Repeated(message)) { base.WndProc(ref message); } }
    }

    public sealed class ActuatorControlPanel : UserControl
    {
        private readonly NOMADConfig _config;
        private readonly PayloadController _owner;
        private readonly Label _status = new Label { Visible = false, AutoSize = true, MaximumSize = new Size(650, 0), ForeColor = Color.Gold };
        private readonly Dictionary<string, PayloadRow> _payloadRows = new Dictionary<string, PayloadRow>();
        private readonly System.Windows.Forms.Timer _sliderTimer = new System.Windows.Forms.Timer { Interval = 50 };
        private bool _sliderSending;
        private sealed class PayloadRow
        {
            internal PayloadDefinition Definition;
            internal TableLayoutPanel Layout;
            internal TrackBar Slider;
            internal Label Pwm;
            internal bool Pending;
            internal int? ControllerPwm;
            internal long Revision, UpdatedTicks, Generation;
        }
        public ActuatorControlPanel(NOMADConfig config)
        {
            _config = config;
            _owner = PayloadController.Current;
            var rows = new FlowLayoutPanel { Dock = DockStyle.Fill, AutoScroll = true, WrapContents = false,
                FlowDirection = FlowDirection.TopDown };
            rows.SizeChanged += (sender, args) => _status.MaximumSize = new Size(
                Math.Max(1, rows.ClientSize.Width - _status.Margin.Horizontal - SystemInformation.VerticalScrollBarWidth), 0);
            rows.SizeChanged += (sender, args) =>
            {
                foreach (var row in _payloadRows.Values)
                { row.Layout.Width = Math.Max(420, rows.ClientSize.Width - 12 - SystemInformation.VerticalScrollBarWidth); }
            };
            Controls.Add(rows);
            if (_owner != null && _owner.Owns(config))
            {
                foreach (var definition in _owner.Definitions)
                {
                    var layout = new TableLayoutPanel { Height = 42, ColumnCount = 4, RowCount = 1,
                        Width = Math.Max(420, rows.ClientSize.Width - 12 - SystemInformation.VerticalScrollBarWidth) };
                    layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 40));
                    layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 60));
                    layout.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
                    layout.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
                    layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
                    layout.Controls.Add(new Label { Text = definition.Name, ForeColor = Color.White,
                        Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleLeft, AutoEllipsis = true }, 0, 0);
                    var buttons = new FlowLayoutPanel { AutoSize = true, WrapContents = false, Anchor = AnchorStyles.None };
                    var row = new PayloadRow { Definition = definition, Layout = layout,
                        ControllerPwm = _owner.TargetPwm(definition.Id),
                        Pwm = new Label { AutoSize = true, Anchor = AnchorStyles.None, ForeColor = Color.White,
                            Name = definition.Id + ":pwm", AccessibleName = definition.Name + " target PWM" } };
                    layout.Controls.Add(row.Pwm, 2, 0); layout.Controls.Add(buttons, 3, 0);
                    _payloadRows.Add(definition.Id, row);
                    if (definition.Behavior == "ServoPosition")
                    {
                        row.Slider = new TrackBar { Minimum = definition.Minimum, Maximum = definition.Maximum,
                            Value = _owner.TargetPwm(definition.Id) ?? definition.Neutral, TickStyle = TickStyle.None,
                            Anchor = AnchorStyles.Left | AnchorStyles.Right, AutoSize = false, Height = 26,
                            Name = definition.Id + ":slider", AccessibleName = definition.Name,
                            SmallChange = 1, LargeChange = Math.Max(1, (definition.Maximum - definition.Minimum) / 10) };
                        layout.Controls.Add(row.Slider, 1, 0);
                        row.Slider.ValueChanged += (sender, args) => SetPwm(row, row.Slider.Value);
                        if (definition.Continuous)
                        {
                            row.Slider.Scroll += (sender, args) =>
                            {
                                Interlocked.Increment(ref row.Revision);
                                row.UpdatedTicks = Stopwatch.GetTimestamp(); row.Generation = OutputController.Generation;
                                row.Pending = true;
                                GimbalController.NotifyServoInput(_config, definition.Channel, row.Slider.Value);
                            };
                        }
                    }
                    if (!definition.Continuous)
                    {
                        foreach (string operation in definition.Operations)
                        { AddButton(row, operation == "negative" ? definition.NegativeLabel :
                            operation == "positive" ? definition.PositiveLabel : definition.PrimaryLabel, operation, buttons); }
                    }
                    AddButton(row, definition.Behavior == "ServoBidirectional" ? definition.NeutralLabel : definition.SafeLabel,
                        "safe", buttons);
                    if (!definition.Relay)
                    {
                        if (row.ControllerPwm.HasValue || row.Slider != null)
                        { SetPwm(row, row.ControllerPwm ?? definition.Neutral); }
                        else { row.Pwm.Text = "Target —"; }
                    }
                    rows.Controls.Add(layout);
                }
                _owner.Changed += OnChanged;
            }
            rows.Controls.Add(_status);
            _sliderTimer.Tick += async (sender, args) => await SendSliderAsync();
            if (_payloadRows.Values.Any(row => row.Definition.Continuous)) { _sliderTimer.Start(); }
        }
        private void AddButton(PayloadRow row, string label, string operation, FlowLayoutPanel buttons)
        {
            var definition = row.Definition;
            var button = new ActuatorButton { Text = label, UseMnemonic = false, AutoSize = true,
                Name = definition.Id + ":" + operation, FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.BUTTON_BG, ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatAppearance = { BorderSize = 0 } };
            button.Click += async (sender, args) =>
            {
                long revision = Interlocked.Increment(ref row.Revision); row.Pending = false;
                double? value = operation == "position" ? Position(row) : (double?)null;
                if (!definition.Relay)
                {
                    int pwm = definition.ServoPwm(operation, value); SetPwm(row, pwm);
                    if (row.Slider != null) { row.Slider.Value = pwm; }
                    GimbalController.NotifyServoInput(_config, definition.Channel, pwm);
                }
                await SendAsync(definition.Id, operation, value, () => revision == Interlocked.Read(ref row.Revision));
            };
            buttons.Controls.Add(button);
        }
        private static double Position(PayloadRow row) => (row.Slider.Value - row.Definition.Minimum) /
            (double)(row.Definition.Maximum - row.Definition.Minimum);
        private static void SetPwm(PayloadRow row, int pwm) => row.Pwm.Text = "Target " + pwm + " µs";
        private async Task SendSliderAsync()
        {
            if (IsDisposed || _sliderSending) { return; }
            var row = _payloadRows.Values.FirstOrDefault(candidate => candidate.Pending);
            if (row == null) { return; }
            long updated = row.UpdatedTicks, revision = Interlocked.Read(ref row.Revision), generation = row.Generation;
            if (!Visible || !_owner.Owns(_config) || generation != OutputController.Generation ||
                (Stopwatch.GetTimestamp() - updated) / (double)Stopwatch.Frequency >= .5)
            {
                row.Pending = false;
                _status.Text = "Slider input expired or was retired; move it again when the output is available.";
                _status.Visible = true;
                return;
            }
            var target = OutputController.CaptureTarget(_config);
            if (PayloadController.HasPendingWork || target?.NativeRequestBusy == true) { return; }
            row.Pending = false; _sliderSending = true;
            try
            {
                await SendAsync(row.Definition.Id, "position", Position(row), () =>
                    revision == Interlocked.Read(ref row.Revision) && generation == OutputController.Generation &&
                    (Stopwatch.GetTimestamp() - updated) / (double)Stopwatch.Frequency < .5);
            }
            finally { _sliderSending = false; }
        }
        private async Task SendAsync(string id, string operation, double? value, Func<bool> current)
        {
            if (IsDisposed || _owner?.Owns(_config) != true) { return; }
            string status = await _owner.ActAsync(id, operation, value,
                inputStillCurrent: () => !IsDisposed && _owner.Owns(_config) && current());
            if (!IsDisposed && _owner.Owns(_config)) { _status.Text = status; _status.Visible = !string.IsNullOrWhiteSpace(status); }
        }
        private void OnChanged(string id)
        {
            if (!IsDisposed && _owner.Owns(_config))
            { UiAsync.RunSync(this, () =>
                {
                    if (IsDisposed) { return; }
                    _status.Text = _owner.Status(id); _status.Visible = !string.IsNullOrWhiteSpace(_status.Text);
                    if (_payloadRows.TryGetValue(id, out var row))
                    {
                        var pwm = _owner.TargetPwm(id);
                        if (pwm != row.ControllerPwm)
                        {
                            row.ControllerPwm = pwm;
                            if (!row.Pending && pwm.HasValue)
                            { SetPwm(row, pwm.Value); if (row.Slider != null) { row.Slider.Value = pwm.Value; } }
                        }
                    }
                }, "Payload state"); }
        }
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _sliderTimer.Stop(); _sliderTimer.Dispose();
                foreach (var row in _payloadRows.Values) { row.Pending = false; Interlocked.Increment(ref row.Revision); }
                if (_owner != null) { _owner.Changed -= OnChanged; }
            }
            base.Dispose(disposing);
        }
    }
}
