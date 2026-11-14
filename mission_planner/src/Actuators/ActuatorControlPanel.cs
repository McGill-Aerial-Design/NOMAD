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
            var rows = new System.Windows.Controls.StackPanel();
            var root = new System.Windows.Controls.ScrollViewer
            {
                VerticalScrollBarVisibility = System.Windows.Controls.ScrollBarVisibility.Auto,
                HorizontalScrollBarVisibility = System.Windows.Controls.ScrollBarVisibility.Auto,
                Content = rows,
            };
            WpfRoot = root;
            Controls.Add(WpfUi.Host(root));
            if (_owner != null && _owner.Owns(config))
            {
                foreach (var definition in _owner.Definitions)
                {
                    var layout = new System.Windows.Controls.Grid { MinWidth = 420, Margin = new System.Windows.Thickness(0, 3, 0, 3) };
                    layout.ColumnDefinitions.Add(new System.Windows.Controls.ColumnDefinition { Width = new System.Windows.GridLength(2, System.Windows.GridUnitType.Star) });
                    layout.ColumnDefinitions.Add(new System.Windows.Controls.ColumnDefinition { Width = new System.Windows.GridLength(3, System.Windows.GridUnitType.Star) });
                    layout.ColumnDefinitions.Add(new System.Windows.Controls.ColumnDefinition { Width = System.Windows.GridLength.Auto });
                    layout.ColumnDefinitions.Add(new System.Windows.Controls.ColumnDefinition { Width = System.Windows.GridLength.Auto });
                    var name = WpfUi.Text(definition.Name);
                    name.VerticalAlignment = System.Windows.VerticalAlignment.Center;
                    layout.Children.Add(name);
                    var buttons = new System.Windows.Controls.StackPanel { Orientation = System.Windows.Controls.Orientation.Horizontal };
                    System.Windows.Controls.Grid.SetColumn(buttons, 3);
                    var row = new PayloadRow { Definition = definition,
                        ControllerPwm = _owner.TargetPwm(definition.Id),
                        Pwm = WpfUi.Text("") };
                    System.Windows.Automation.AutomationProperties.SetName(row.Pwm, definition.Name + " target PWM");
                    System.Windows.Automation.AutomationProperties.SetAutomationId(row.Pwm, definition.Id + ":pwm");
                    System.Windows.Controls.Grid.SetColumn(row.Pwm, 2);
                    layout.Children.Add(row.Pwm); layout.Children.Add(buttons);
                    _payloadRows.Add(definition.Id, row);
                    if (definition.Behavior == "ServoPosition")
                    {
                        row.Slider = new System.Windows.Controls.Slider { Minimum = definition.Minimum, Maximum = definition.Maximum,
                            Value = _owner.TargetPwm(definition.Id) ?? definition.Neutral, TickFrequency = 1,
                            IsSnapToTickEnabled = true, SmallChange = 1,
                            LargeChange = Math.Max(1, (definition.Maximum - definition.Minimum) / 10) };
                        System.Windows.Automation.AutomationProperties.SetName(row.Slider, definition.Name);
                        System.Windows.Automation.AutomationProperties.SetAutomationId(row.Slider, definition.Id + ":slider");
                        System.Windows.Controls.Grid.SetColumn(row.Slider, 1);
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
                    rows.Children.Add(layout);
                }
                _owner.Changed += OnChanged;
            }
            rows.Children.Add(_status);
            _sliderTimer.Tick += async (sender, args) => await SendSliderAsync();
            if (_payloadRows.Values.Any(row => row.Definition.Continuous)) { _sliderTimer.Start(); }
        }
        private void AddButton(PayloadRow row, string label, string operation, System.Windows.Controls.StackPanel buttons)
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
            button.Click += async (sender, args) =>
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
            };
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
            if (!IsDisposed && _owner.Owns(_config)) { _status.Text = status; _status.Visibility = string.IsNullOrWhiteSpace(status) ? System.Windows.Visibility.Collapsed : System.Windows.Visibility.Visible; }
        }
        private void OnChanged(string id)
        {
            if (!IsDisposed && _owner.Owns(_config))
            { UiAsync.RunSync(this, () =>
                {
                    if (IsDisposed) { return; }
                    _status.Text = _owner.Status(id); _status.Visibility = string.IsNullOrWhiteSpace(_status.Text) ? System.Windows.Visibility.Collapsed : System.Windows.Visibility.Visible;
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
