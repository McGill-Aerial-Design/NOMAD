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
    public sealed class ActuatorControlPanel : UserControl
    {
        private readonly NOMADConfig _config;
        private readonly PayloadController _owner;
        private readonly System.Windows.Controls.TextBlock _status = new System.Windows.Controls.TextBlock { Visibility = System.Windows.Visibility.Collapsed, Foreground = System.Windows.Media.Brushes.Gold, TextWrapping = System.Windows.TextWrapping.Wrap };
        internal System.Windows.FrameworkElement WpfRoot { get; }
        private readonly Dictionary<string, PayloadRow> _payloadRows = new Dictionary<string, PayloadRow>();
        private readonly System.Windows.Forms.Timer _sliderTimer = new System.Windows.Forms.Timer { Interval = 50 };
        private bool _sliderSending;
        private sealed class PayloadRow
        {
            internal PayloadDefinition Definition;
            internal System.Windows.Controls.Slider Slider;
            internal System.Windows.Controls.TextBlock Pwm;
            internal bool Updating;
            internal bool Pending;
            internal int? ControllerPwm;
            internal long Revision, UpdatedTicks, Generation;
        }
        public ActuatorControlPanel(NOMADConfig config)
        {
            _config = config;
            _owner = PayloadController.Current;
            var rows = new System.Windows.Controls.StackPanel { Margin = new System.Windows.Thickness(NOMADTheme.PAD, 0, NOMADTheme.PAD, NOMADTheme.PAD) };
            var root = new System.Windows.Controls.ScrollViewer
            {
                VerticalScrollBarVisibility = System.Windows.Controls.ScrollBarVisibility.Auto,
                HorizontalScrollBarVisibility = System.Windows.Controls.ScrollBarVisibility.Disabled,
                Background = WpfTheme.Card, Content = rows,
            };
            WpfRoot = root;
            var host = WpfUi.Host(root); host.BackColor = NOMADTheme.CARD_BG;
            Controls.Add(host);
            if (_owner != null && _owner.Owns(config))
            {
                foreach (var definition in _owner.Definitions)
                {
                    // One card per payload: name and target on top, slider below, actions right-aligned.
                    var layout = new System.Windows.Controls.StackPanel();
                    var title = new System.Windows.Controls.DockPanel { Margin = new System.Windows.Thickness(0, 0, 0, 4) };
                    var name = new System.Windows.Controls.TextBlock { Text = definition.Name, FontWeight = System.Windows.FontWeights.SemiBold,
                        TextTrimming = System.Windows.TextTrimming.CharacterEllipsis };
                    var buttons = new System.Windows.Controls.WrapPanel { HorizontalAlignment = System.Windows.HorizontalAlignment.Right,
                        Margin = new System.Windows.Thickness(0, 4, 0, 0) };
                    var row = new PayloadRow { Definition = definition,
                        ControllerPwm = _owner.TargetPwm(definition.Id),
                        Pwm = new System.Windows.Controls.TextBlock { Foreground = WpfTheme.SecondaryText,
                            Margin = new System.Windows.Thickness(8, 0, 0, 0) } };
                    System.Windows.Automation.AutomationProperties.SetName(row.Pwm, definition.Name + " target PWM");
                    System.Windows.Automation.AutomationProperties.SetAutomationId(row.Pwm, definition.Id + ":pwm");
                    System.Windows.Controls.DockPanel.SetDock(row.Pwm, System.Windows.Controls.Dock.Right);
                    title.Children.Add(row.Pwm); title.Children.Add(name);
                    layout.Children.Add(title);
                    _payloadRows.Add(definition.Id, row);
                    if (definition.Behavior == "ServoPosition")
                    {
                        row.Slider = new System.Windows.Controls.Slider { Minimum = definition.Minimum, Maximum = definition.Maximum,
                            Value = _owner.TargetPwm(definition.Id) ?? definition.Neutral, TickFrequency = 1,
                            IsSnapToTickEnabled = true, SmallChange = 1,
                            LargeChange = Math.Max(1, (definition.Maximum - definition.Minimum) / 10) };
                        System.Windows.Automation.AutomationProperties.SetName(row.Slider, definition.Name);
                        System.Windows.Automation.AutomationProperties.SetAutomationId(row.Slider, definition.Id + ":slider");
                        layout.Children.Add(row.Slider);
                        row.Slider.ValueChanged += (sender, args) =>
                        {
                            int pwm = (int)row.Slider.Value;
                            SetPwm(row, pwm);
                            if (definition.Continuous && !row.Updating)
                            {
                                Interlocked.Increment(ref row.Revision);
                                row.UpdatedTicks = Stopwatch.GetTimestamp(); row.Generation = OutputController.Generation;
                                row.Pending = true;
                                GimbalController.NotifyServoInput(_config, definition.Channel, pwm);
                            }
                        };
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
                    layout.Children.Add(buttons);
                    rows.Children.Add(new System.Windows.Controls.Border { Child = layout, Background = WpfTheme.Input,
                        BorderBrush = WpfTheme.Outline, BorderThickness = new System.Windows.Thickness(1),
                        CornerRadius = new System.Windows.CornerRadius(4), Padding = new System.Windows.Thickness(10, 8, 10, 8),
                        Margin = new System.Windows.Thickness(0, 0, 0, 8) });
                }
                _owner.Changed += OnChanged;
            }
            rows.Children.Add(_status);
            _sliderTimer.Tick += (sender, args) => UiAsync.Run(this, SendSliderAsync, "Payload slider");
            if (_payloadRows.Values.Any(row => row.Definition.Continuous)) { _sliderTimer.Start(); }
        }
        private void AddButton(PayloadRow row, string label, string operation, System.Windows.Controls.Panel buttons)
        {
            var definition = row.Definition;
            var button = WpfUi.Action(label, () => { });
            System.Windows.Automation.AutomationProperties.SetName(button, label);
            System.Windows.Automation.AutomationProperties.SetAutomationId(button, definition.Id + ":" + operation);
            button.PreviewKeyDown += (sender, args) =>
            {
                if (args.IsRepeat && (args.Key == System.Windows.Input.Key.Enter || args.Key == System.Windows.Input.Key.Space))
                { args.Handled = true; }
            };
            button.Click += (sender, args) => UiAsync.Run(this, async () =>
            {
                long revision = Interlocked.Increment(ref row.Revision); row.Pending = false;
                double? value = operation == "position" ? Position(row) : (double?)null;
                if (!definition.Relay)
                {
                    int pwm = definition.ServoPwm(operation, value); SetPwm(row, pwm);
                    if (row.Slider != null) { SetSlider(row, pwm); }
                    GimbalController.NotifyServoInput(_config, definition.Channel, pwm);
                }
                await SendAsync(definition.Id, operation, value, () => revision == Interlocked.Read(ref row.Revision));
            }, "Payload " + operation);
            buttons.Children.Add(button);
        }
        private static void SetSlider(PayloadRow row, int pwm)
        {
            row.Updating = true;
            try { row.Slider.Value = pwm; }
            finally { row.Updating = false; }
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
                _status.Visibility = System.Windows.Visibility.Visible;
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
            UiAsync.RunSync(this, () => { if (_owner.Owns(_config)) { ShowStatus(status); } }, "Payload status");
        }
        private void ShowStatus(string status)
        {
            _status.Text = status;
            _status.Visibility = string.IsNullOrWhiteSpace(status) ? System.Windows.Visibility.Collapsed : System.Windows.Visibility.Visible;
        }
        private void OnChanged(string id)
        {
            if (!IsDisposed && _owner.Owns(_config))
            { UiAsync.RunSync(this, () =>
                {
                    if (IsDisposed) { return; }
                    ShowStatus(_owner.Status(id));
                    if (_payloadRows.TryGetValue(id, out var row))
                    {
                        var pwm = _owner.TargetPwm(id);
                        if (pwm != row.ControllerPwm)
                        {
                            row.ControllerPwm = pwm;
                            if (!row.Pending && pwm.HasValue)
                            { SetPwm(row, pwm.Value); if (row.Slider != null) { SetSlider(row, pwm.Value); } }
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
