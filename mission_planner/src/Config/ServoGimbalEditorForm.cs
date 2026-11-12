// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.Linq;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    [global::MissionPlanner.Utilities.PreventTheming]
    internal sealed class ServoGimbalEditorForm : Form
    {
        private readonly CheckBox _enabled;
        private readonly DataGridView _axes;
        private readonly Label _error;
        internal List<ServoGimbalAxis> Axes { get; private set; }

        internal ServoGimbalEditorForm(IReadOnlyList<ServoGimbalAxis> current)
        {
            Text = "Video gimbal outputs"; Font = NOMADTheme.Font();
            BackColor = NOMADTheme.BG_DARK; ForeColor = NOMADTheme.TEXT_PRIMARY;
            ClientSize = new Size(850, 325); MinimumSize = new Size(800, 320);
            StartPosition = FormStartPosition.CenterParent;
            _enabled = ControlFactory.CheckBox("Use flight-controller servo outputs", current?.Count > 0);
            _enabled.Dock = DockStyle.Top; _enabled.Height = 34;
            var help = new Label { Dock = DockStyle.Top, Height = 55, Padding = new Padding(8),
                Text = "Select one to three axes. Configure each output for servo commands in Mission Planner.\n" +
                    "Angle zero is the calibrated level/forward target. Tracking uses pitch and yaw with level roll." };
            _axes = new DataGridView { Dock = DockStyle.Fill, AllowUserToAddRows = false, AllowUserToDeleteRows = false,
                RowHeadersVisible = false, AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill,
                BackgroundColor = NOMADTheme.CARD_BG, BorderStyle = BorderStyle.None, EnableHeadersVisualStyles = false };
            _axes.DefaultCellStyle.BackColor = NOMADTheme.INPUT_BG;
            _axes.DefaultCellStyle.ForeColor = NOMADTheme.TEXT_PRIMARY;
            _axes.ColumnHeadersDefaultCellStyle.BackColor = NOMADTheme.CARD_BG;
            _axes.ColumnHeadersDefaultCellStyle.ForeColor = NOMADTheme.TEXT_PRIMARY;
            _axes.Columns.Add(new DataGridViewCheckBoxColumn { Name = "Use", HeaderText = "Use", FillWeight = 45 });
            _axes.Columns.Add(new DataGridViewTextBoxColumn { Name = "Axis", HeaderText = "Axis", ReadOnly = true, FillWeight = 70 });
            foreach (string name in new[] { "Channel", "Min PWM", "Max PWM", "Min deg", "Max deg" })
            { _axes.Columns.Add(new DataGridViewTextBoxColumn { Name = name, HeaderText = name }); }
            _axes.Columns.Add(new DataGridViewCheckBoxColumn { Name = "Reverse", HeaderText = "Reverse", FillWeight = 70 });
            foreach (string name in new[] { "Pitch", "Yaw", "Roll" })
            {
                var axis = current?.FirstOrDefault(item => item.Axis == name) ?? new ServoGimbalAxis
                    { Axis = name, MinimumAngleDeg = name == "Roll" ? -30 : -90, MaximumAngleDeg = name == "Roll" ? 30 : 90 };
                _axes.Rows.Add(current?.Any(item => item.Axis == name) == true, name, axis.Channel,
                    axis.MinimumPwm, axis.MaximumPwm, axis.MinimumAngleDeg, axis.MaximumAngleDeg, axis.Reversed);
            }
            _enabled.CheckedChanged += (sender, args) => _axes.Enabled = _enabled.Checked;
            _axes.Enabled = _enabled.Checked;
            _error = new Label { Dock = DockStyle.Fill, ForeColor = NOMADTheme.WARNING, Padding = new Padding(8) };
            var use = ControlFactory.Button("Use gimbal", width: 110, height: 30);
            use.Click += (sender, args) =>
            {
                try { Axes = BuildCandidate(); DialogResult = DialogResult.OK; }
                catch (Exception error) when (error is FormatException || error is OverflowException || error is Newtonsoft.Json.JsonException)
                { _error.Text = error.Message; }
            };
            var cancel = ControlFactory.Button("Cancel", width: 90, height: 30); cancel.DialogResult = DialogResult.Cancel;
            var footer = new Panel { Dock = DockStyle.Bottom, Height = 85 };
            var buttons = ControlFactory.ButtonRow(use, cancel); buttons.Dock = DockStyle.Bottom;
            footer.Controls.Add(_error); footer.Controls.Add(buttons);
            Controls.Add(_axes); Controls.Add(help); Controls.Add(_enabled); Controls.Add(footer);
            AcceptButton = use; CancelButton = cancel;
        }
        internal List<ServoGimbalAxis> BuildCandidate()
        {
            if (!_axes.EndEdit()) { throw new FormatException("Finish editing the gimbal calibration values."); }
            var result = new List<ServoGimbalAxis>();
            if (!_enabled.Checked) { return result; }
            foreach (DataGridViewRow row in _axes.Rows)
            {
                if (!Equals(row.Cells["Use"].Value, true)) { continue; }
                result.Add(new ServoGimbalAxis
                {
                    Axis = (string)row.Cells["Axis"].Value,
                    Channel = Integer(row.Cells["Channel"].Value),
                    MinimumPwm = Integer(row.Cells["Min PWM"].Value),
                    MaximumPwm = Integer(row.Cells["Max PWM"].Value),
                    MinimumAngleDeg = Angle(row.Cells["Min deg"].Value),
                    MaximumAngleDeg = Angle(row.Cells["Max deg"].Value),
                    Reversed = Equals(row.Cells["Reverse"].Value, true),
                });
            }
            if (result.Count == 0) { throw new FormatException("Select at least one servo gimbal axis."); }
            ServoGimbalAxis.Validate(result);
            return result;
        }
        private static int Integer(object value)
        {
            if (!int.TryParse(Convert.ToString(value, CultureInfo.CurrentCulture), NumberStyles.Integer,
                CultureInfo.CurrentCulture, out int result)) { throw new FormatException("Servo channels and PWM values must be whole numbers."); }
            return result;
        }
        private static double Angle(object value)
        {
            if (!double.TryParse(Convert.ToString(value, CultureInfo.CurrentCulture), NumberStyles.Float,
                CultureInfo.CurrentCulture, out double result)) { throw new FormatException("Enter both gimbal angle limits in degrees."); }
            return result;
        }
    }
}
