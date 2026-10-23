// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Threading;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public sealed partial class NativeFenceSetupView
    {
        private FenceParameters _parameters;
        private CancellationTokenRegistration _parameterShutdown;
        private TableLayoutPanel _parameterRows;
        private Label _parameterStatus;
        private Button _read, _apply;
        private readonly Dictionary<FenceParameter, Control> _editors = new Dictionary<FenceParameter, Control>();

        private void AddParameters(TableLayoutPanel layout, NOMADConfig config)
        {
            var group = Group(layout, "Aircraft fence response");
            Add(group, new Label { AutoSize = true, MaximumSize = new System.Drawing.Size(680, 0),
                Text = "Read the aircraft's supported fence settings. Apply while disarmed; each change is read back." });
            _parameterRows = new TableLayoutPanel { AutoSize = true, Dock = DockStyle.Fill, ColumnCount = 2 };
            _parameterRows.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            _parameterRows.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            Add(group, _parameterRows);
            var buttons = new FlowLayoutPanel { AutoSize = true, Dock = DockStyle.Fill };
            _read = Button("Read aircraft settings", async (s, e) => await ReadSettings(config));
            _apply = Button("Apply changed settings…", async (s, e) => await ApplySettings());
            _apply.Enabled = false;
            buttons.Controls.Add(_read); buttons.Controls.Add(_apply);
            Add(group, buttons);
            _parameterStatus = new Label { AutoSize = true, MaximumSize = new System.Drawing.Size(680, 0),
                Text = "No aircraft settings read. Reading never writes parameters." };
            Add(group, _parameterStatus);
            _parameterShutdown = _shutdown.Register(() => _parameters?.Dispose());
        }

        private async System.Threading.Tasks.Task ReadSettings(NOMADConfig config)
        {
            if (!Active || !_read.Enabled) { return; }
            SetParameterBusy(true);
            _parameters?.Dispose();
            _parameters = null;
            _tips.RemoveAll();
            foreach (Control control in _parameterRows.Controls.Cast<Control>().ToArray()) { control.Dispose(); }
            _parameterRows.Controls.Clear(); _parameterRows.RowStyles.Clear(); _parameterRows.RowCount = 0;
            _editors.Clear();
            try
            {
                _parameters = new FenceParameters(config);
                await _parameters.ReadAsync();
                if (!Active) { return; }
                foreach (var parameter in _parameters.Parameters) { AddParameter(parameter); }
                _parameterStatus.Text = _editors.Count == 0 ? "No supported fence parameters are cached on this aircraft."
                    : "Read from " + _parameters.TargetDescription + ". Review edits, then Apply while disarmed.";
            }
            catch (Exception error) { if (Active) { _parameterStatus.Text = "Read failed: " + error.Message; } }
            finally { if (Active) { SetParameterBusy(false); } }
        }

        private void AddParameter(FenceParameter parameter)
        {
            Control editor;
            if (parameter.Options.Count > 0)
            {
                var combo = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = 330,
                    DisplayMember = "Value", ValueMember = "Key", DataSource = parameter.Options };
                combo.SelectedValue = (int)parameter.Value;
                editor = combo;
            }
            else if (parameter.Bits.Count > 0)
            {
                var bits = new CheckedListBox { CheckOnClick = true, Width = 330,
                    Height = Math.Min(120, 22 * parameter.Bits.Count + 6), DisplayMember = "Value" };
                foreach (var bit in parameter.Bits)
                { bits.Items.Add(bit, ((int)parameter.Value & (1 << bit.Key)) != 0); }
                editor = bits;
            }
            else if (parameter.HasRange && parameter.Minimum <= parameter.Maximum &&
                parameter.Minimum >= (double)decimal.MinValue && parameter.Maximum <= (double)decimal.MaxValue &&
                parameter.Allows(parameter.Value))
            {
                editor = new NumericUpDown { DecimalPlaces = 2, Width = 170, Increment = 0.1m,
                    Minimum = (decimal)parameter.Minimum, Maximum = (decimal)parameter.Maximum,
                    Value = (decimal)parameter.Value };
            }
            else { editor = new TextBox { Text = parameter.Value.ToString(CultureInfo.InvariantCulture), ReadOnly = true }; }
            editor.Enabled = parameter.Allows(parameter.Value);
            editor.Name = parameter.Name;
            editor.BackColor = NOMADTheme.CONTROL_BG; editor.ForeColor = NOMADTheme.TEXT_PRIMARY;
            int row = _parameterRows.RowCount++;
            _parameterRows.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            var label = new Label { Text = (string.IsNullOrEmpty(parameter.DisplayName) ? parameter.Name : parameter.DisplayName) +
                (string.IsNullOrEmpty(parameter.Units) ? "" : " (" + parameter.Units + ")"),
                AutoSize = true, MaximumSize = new System.Drawing.Size(260, 0), Margin = new Padding(0, 6, 14, 8) };
            _tips.SetToolTip(label, parameter.Name + "\n" + parameter.Description);
            _tips.SetToolTip(editor, parameter.Name + "\n" + parameter.Description);
            _parameterRows.Controls.Add(label, 0, row);
            editor.Margin = new Padding(0, 0, 0, 8);
            _parameterRows.Controls.Add(editor, 1, row);
            _editors.Add(parameter, editor);
        }

        private async System.Threading.Tasks.Task ApplySettings()
        {
            if (!Active || !_apply.Enabled || _parameters == null) { return; }
            var changes = new Dictionary<string, float>();
            foreach (var pair in _editors.Where(pair => pair.Value.Enabled))
            {
                float value = pair.Key.Value;
                if (pair.Value is ComboBox combo && combo.SelectedValue is int selected) { value = selected; }
                else if (pair.Value is NumericUpDown numeric) { value = (float)numeric.Value; }
                else if (pair.Value is CheckedListBox bits)
                { value = bits.CheckedItems.Cast<KeyValuePair<int, string>>().Aggregate(0, (all, bit) => all | (1 << bit.Key)); }
                if (value != pair.Key.Value) { changes.Add(pair.Key.Name, value); }
            }
            if (changes.Count == 0) { _parameterStatus.Text = "No fence settings changed."; return; }
            if (!_parameters.CanApply)
            { _parameterStatus.Text = "Apply requires the same live aircraft and a fresh disarmed heartbeat. Read again if the target changed."; return; }
            SetParameterBusy(true);
            try
            {
                string summary = string.Join("\n", changes.Select(pair => pair.Key + " = " + pair.Value.ToString(CultureInfo.InvariantCulture)));
                var choice = MessageBox.Show(this, "Write these settings to " + _parameters.TargetDescription + "?\n\n" + summary +
                    "\n\nWrites are sequential. A failure stops the remaining changes; earlier writes remain applied.",
                    "Apply aircraft fence response", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning,
                    MessageBoxDefaultButton.Button2);
                if (choice != DialogResult.OK || !Active) { return; }
                string result = await _parameters.ApplyAsync(changes, () => Active);
                if (Active) { _parameterStatus.Text = result; }
            }
            finally { if (Active) { SetParameterBusy(false); } }
        }

        private void SetParameterBusy(bool busy)
        {
            _read.Enabled = !busy;
            _apply.Enabled = !busy && _parameters != null && _editors.Count > 0;
            _parameterRows.Enabled = !busy;
        }
    }
}
