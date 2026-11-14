// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using Forms = System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    [global::MissionPlanner.Utilities.PreventTheming]
    internal sealed class ServoGimbalEditorForm : Forms.Form
    {
        private sealed class AxisRow
        {
            internal string Name;
            internal CheckBox Use, Reverse;
            internal TextBox Channel, MinimumPwm, MaximumPwm, MinimumAngle, MaximumAngle;
        }
        private readonly CheckBox _enabled;
        private readonly Grid _axes;
        private readonly TextBlock _error;
        private readonly List<AxisRow> _rows = new List<AxisRow>();
        internal List<ServoGimbalAxis> Axes { get; private set; }

        internal ServoGimbalEditorForm(IReadOnlyList<ServoGimbalAxis> current)
        {
            Text = "Video gimbal outputs"; ClientSize = new System.Drawing.Size(850, 325); MinimumSize = new System.Drawing.Size(800, 320);
            StartPosition = Forms.FormStartPosition.CenterParent;
            var root = new DockPanel { Margin = new Thickness(8) };
            var header = new StackPanel(); DockPanel.SetDock(header, System.Windows.Controls.Dock.Top); root.Children.Add(header);
            _enabled = new CheckBox { Content = "Use flight-controller servo outputs", IsChecked = current?.Count > 0 };
            header.Children.Add(_enabled);
            header.Children.Add(WpfUi.Text("Select one to three axes for position servos. Configure outputs in Mission Planner.\n" +
                "Angle zero is the calibrated level/forward target. Tracking uses pitch and yaw with level roll."));
            var footer = new StackPanel(); DockPanel.SetDock(footer, System.Windows.Controls.Dock.Bottom); root.Children.Add(footer);
            _error = WpfUi.Text(""); footer.Children.Add(_error);
            var buttons = new WrapPanel(); footer.Children.Add(buttons);
            var use = WpfUi.Action("Use gimbal", () =>
            {
                try { Axes = BuildCandidate(); DialogResult = Forms.DialogResult.OK; }
                catch (Exception error) when (error is FormatException || error is OverflowException || error is Newtonsoft.Json.JsonException)
                { _error.Text = error.Message; }
            });
            use.IsDefault = true; buttons.Children.Add(use);
            var cancel = WpfUi.Action("Cancel", () => DialogResult = Forms.DialogResult.Cancel);
            cancel.IsCancel = true; buttons.Children.Add(cancel);
            _axes = new Grid(); root.Children.Add(_axes);
            string[] headers = { "Use", "Axis", "Channel", "Min PWM", "Max PWM", "Min deg", "Max deg", "Reverse" };
            foreach (string unused in headers) _axes.ColumnDefinitions.Add(new ColumnDefinition());
            _axes.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            for (int i = 0; i < headers.Length; i++) Add(new TextBlock { Text = headers[i] }, 0, i);
            foreach (string name in new[] { "Pitch", "Yaw", "Roll" })
            {
                var axis = current?.FirstOrDefault(item => item.Axis == name) ?? new ServoGimbalAxis
                    { Axis = name, MinimumAngleDeg = name == "Roll" ? -30 : -90, MaximumAngleDeg = name == "Roll" ? 30 : 90 };
                var row = new AxisRow { Name = name, Use = new CheckBox { IsChecked = current?.Any(item => item.Axis == name) == true },
                    Channel = Input(axis.Channel), MinimumPwm = Input(axis.MinimumPwm), MaximumPwm = Input(axis.MaximumPwm),
                    MinimumAngle = Input(axis.MinimumAngleDeg), MaximumAngle = Input(axis.MaximumAngleDeg),
                    Reverse = new CheckBox { IsChecked = axis.Reversed } };
                _rows.Add(row); int index = _rows.Count;
                _axes.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
                UIElement[] cells = { row.Use, new TextBlock { Text = name }, row.Channel, row.MinimumPwm,
                    row.MaximumPwm, row.MinimumAngle, row.MaximumAngle, row.Reverse };
                for (int i = 0; i < cells.Length; i++) Add(cells[i], index, i);
            }
            _enabled.Checked += (s, e) => _axes.IsEnabled = true;
            _enabled.Unchecked += (s, e) => _axes.IsEnabled = false;
            _axes.IsEnabled = _enabled.IsChecked == true;
            Controls.Add(WpfUi.Host(root));
        }
        private void Add(UIElement cell, int row, int column)
        { Grid.SetRow(cell, row); Grid.SetColumn(cell, column); _axes.Children.Add(cell); }
        private static TextBox Input(object value) => new TextBox { Text = Convert.ToString(value, CultureInfo.CurrentCulture) };
        internal List<ServoGimbalAxis> BuildCandidate()
        {
            var result = new List<ServoGimbalAxis>();
            if (_enabled.IsChecked != true) return result;
            foreach (var row in _rows.Where(item => item.Use.IsChecked == true))
                result.Add(new ServoGimbalAxis { Axis = row.Name, Channel = Integer(row.Channel.Text),
                    MinimumPwm = Integer(row.MinimumPwm.Text), MaximumPwm = Integer(row.MaximumPwm.Text),
                    MinimumAngleDeg = Angle(row.MinimumAngle.Text), MaximumAngleDeg = Angle(row.MaximumAngle.Text),
                    Reversed = row.Reverse.IsChecked == true });
            if (result.Count == 0) throw new FormatException("Select at least one servo gimbal axis.");
            ServoGimbalAxis.Validate(result); return result;
        }
        private static int Integer(string value)
        {
            if (!int.TryParse(value, NumberStyles.Integer, CultureInfo.CurrentCulture, out int result))
                throw new FormatException("Servo channels and PWM values must be whole numbers.");
            return result;
        }
        private static double Angle(string value)
        {
            if (!double.TryParse(value, NumberStyles.Float, CultureInfo.CurrentCulture, out double result))
                throw new FormatException("Enter both gimbal angle limits in degrees.");
            return result;
        }
    }
}
