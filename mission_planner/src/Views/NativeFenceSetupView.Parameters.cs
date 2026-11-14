// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Threading;
using System.Windows.Forms;
using System.Windows;
using Wpf = System.Windows.Controls;
using MessageBox = System.Windows.Forms.MessageBox;

namespace NOMAD.MissionPlanner
{
    public sealed partial class NativeFenceSetupView
    {
        private FenceParameters _parameters;
        private CancellationTokenRegistration _parameterShutdown;
        private Wpf.StackPanel _parameterRows;
        private Wpf.StackPanel _terminationRows;
        private Wpf.TextBlock _terminationStatus;
        internal TerminationController Termination { get; set; }
        private Wpf.TextBlock _parameterStatus;
        private Wpf.Button _read, _apply;
        private readonly Dictionary<FenceParameter, FrameworkElement> _editors = new Dictionary<FenceParameter, FrameworkElement>();

        private void AddParameters(Wpf.StackPanel layout, NOMADConfig config)
        {
            var group = Group(layout, "Aircraft fence response");
            _parameterRows = new Wpf.StackPanel();
            Add(group, _parameterRows);
            var termination = Group(group, "Controlled termination");
            _terminationStatus = WpfUi.Text("Read settings to check support.");
            Add(termination, _terminationStatus);
            _terminationRows = new Wpf.StackPanel();
            Add(termination, _terminationRows);
            var buttons = new Wpf.WrapPanel();
            _read = Button("Read aircraft settings", async (s, e) => await ReadSettings(config));
            _apply = Button("Apply changes…", async (s, e) => await ApplySettings());
            _apply.IsEnabled = false;
            buttons.Children.Add(_read); buttons.Children.Add(_apply);
            Add(group, buttons);
            _parameterStatus = WpfUi.Text("Settings not read.");
            Add(group, _parameterStatus);
            _parameterShutdown = _shutdown.Register(() => _parameters?.Dispose());
        }

        private async System.Threading.Tasks.Task ReadSettings(NOMADConfig config)
        {
            if (!Active || !_read.IsEnabled) { return; }
            SetParameterBusy(true);
            _parameters?.Dispose();
            _parameters = null;
            _parameterRows.Children.Clear(); _terminationRows.Children.Clear();
            _editors.Clear();
            try
            {
                _parameters = new FenceParameters(config);
                await _parameters.ReadAsync(Termination);
                if (Active) { await _parameters.ReadTerminationAsync(Termination); }
                if (!Active) { return; }
                foreach (var parameter in _parameters.Parameters) { AddParameter(parameter); }
                _terminationStatus.Text = _parameters.TerminationStatus;
                _parameterStatus.Text = _editors.Count == 0 ? "No supported fence parameters are cached on this aircraft."
                    : "Read from " + _parameters.TargetDescription + ". Review edits, then Apply while disarmed.";
            }
            catch (Exception error) { if (Active) { _parameterStatus.Text = "Read failed: " + error.Message; } }
            finally { if (Active) { SetParameterBusy(false); } }
        }

        private void AddParameter(FenceParameter parameter)
        {
            FrameworkElement editor;
            bool terminationNumber = parameter.Name == "AFS_TERM_RATE" || parameter.Name == "AFS_RC_FAIL_TIME";
            if (parameter.Options.Count > 0)
            {
                var combo = new Wpf.ComboBox { Width = 330, HorizontalAlignment = System.Windows.HorizontalAlignment.Left,
                    DisplayMemberPath = "Value", SelectedValuePath = "Key", ItemsSource = parameter.Options };
                editor = combo;
            }
            else if (parameter.Bits.Count > 0)
            {
                var bits = new Wpf.StackPanel();
                foreach (var bit in parameter.Bits)
                { bits.Children.Add(new Wpf.CheckBox { Content = bit.Value, Tag = bit.Key,
                    IsChecked = ((int)parameter.Value & (1 << bit.Key)) != 0 }); }
                editor = bits;
            }
            else if (parameter.HasRange && parameter.Minimum <= parameter.Maximum &&
                parameter.Minimum >= (double)decimal.MinValue && parameter.Maximum <= (double)decimal.MaxValue &&
                parameter.Allows(parameter.Value))
            {
                editor = new Wpf.TextBox { Text = parameter.Value.ToString(CultureInfo.InvariantCulture), Width = 170,
                    HorizontalAlignment = System.Windows.HorizontalAlignment.Left };
            }
            else { editor = new Wpf.TextBox { Text = parameter.Value.ToString(CultureInfo.InvariantCulture), IsReadOnly = !terminationNumber }; }
            editor.IsEnabled = parameter.Allows(parameter.Value) || parameter.Name == "AFS_TERM_ACTION" || terminationNumber;
            editor.Name = parameter.Name;
            var rows = parameter.Name.StartsWith("AFS_", StringComparison.Ordinal) ? _terminationRows : _parameterRows;
            var row = new Wpf.Grid();
            row.ColumnDefinitions.Add(new Wpf.ColumnDefinition { Width = new GridLength(240) });
            row.ColumnDefinitions.Add(new Wpf.ColumnDefinition());
            var label = new Wpf.TextBlock { Text = (string.IsNullOrEmpty(parameter.DisplayName) ? parameter.Name : parameter.DisplayName) +
                (string.IsNullOrEmpty(parameter.Units) ? "" : " (" + parameter.Units + ")"),
                TextWrapping = TextWrapping.Wrap, Margin = new Thickness(0, 6, 14, 8),
                ToolTip = parameter.Name + "\n" + parameter.Description };
            if (parameter.Name == "AFS_TERM_ACTION" && parameter.Value != 44)
            { label.Text += " (currently " + parameter.Value.ToString(CultureInfo.InvariantCulture) + ")"; }
            editor.ToolTip = parameter.Name + "\n" + parameter.Description;
            row.Children.Add(label); editor.Margin = new Thickness(0, 0, 0, 8);
            Wpf.Grid.SetColumn(editor, 1); row.Children.Add(editor); rows.Children.Add(row);
            if (editor is Wpf.ComboBox bound)
            {
                bound.SelectedValue = (int)parameter.Value;
                if (!parameter.Options.Any(option => option.Key == parameter.Value)) { bound.SelectedIndex = -1; }
            }
            _editors.Add(parameter, editor);
        }

        private Dictionary<string, float> ReadEdits()
        {
            var changes = new Dictionary<string, float>();
            foreach (var pair in _editors.Where(pair => pair.Value.IsEnabled))
            {
                float value = pair.Key.Value;
                if (pair.Value is Wpf.ComboBox combo && combo.SelectedValue is int selected) { value = selected; }
                else if (pair.Value is Wpf.StackPanel bits)
                { value = bits.Children.OfType<Wpf.CheckBox>().Where(bit => bit.IsChecked == true)
                    .Aggregate(0, (all, bit) => all | (1 << (int)bit.Tag)); }
                else if (pair.Value is Wpf.TextBox text && !text.IsReadOnly && !float.TryParse(text.Text,
                    NumberStyles.Float, CultureInfo.InvariantCulture, out value))
                { throw new ArgumentException("Enter a finite number using . for decimals: " + pair.Key.DisplayName); }
                if (pair.Value is Wpf.TextBox && value != pair.Key.Value && !pair.Key.Allows(value))
                { throw new ArgumentException("Value is outside the supported range: " + pair.Key.DisplayName); }
                if (value != pair.Key.Value) { changes.Add(pair.Key.Name, value); }
            }
            return changes;
        }

        private async System.Threading.Tasks.Task ApplySettings()
        {
            if (!Active || !_apply.IsEnabled || _parameters == null) { return; }
            Dictionary<string, float> changes;
            try { changes = ReadEdits(); }
            catch (ArgumentException error) { _parameterStatus.Text = error.Message; return; }
            if (changes.Count == 0) { _parameterStatus.Text = "No aircraft settings changed."; return; }
            if (!_parameters.CanApply)
            { _parameterStatus.Text = "Apply requires the same live aircraft and a fresh disarmed heartbeat. Read again if the target changed."; return; }
            SetParameterBusy(true);
            try
            {
                string summary = string.Join("\n", changes.Select(pair => pair.Key + " = " + pair.Value.ToString(CultureInfo.InvariantCulture)));
                var choice = MessageBox.Show(this, "Write these settings to " + _parameters.TargetDescription + "?\n\n" + summary +
                    "\n\nWrites are sequential. A failure stops the remaining changes; earlier writes remain applied.",
                    "Apply aircraft safety settings", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning,
                    MessageBoxDefaultButton.Button2);
                if (choice != DialogResult.OK || !Active) { return; }
                string result = await _parameters.ApplyAsync(changes, () => Active);
                if (Active) { _parameterStatus.Text = result; }
            }
            finally { if (Active) { SetParameterBusy(false); } }
        }

        private void SetParameterBusy(bool busy)
        {
            _read.IsEnabled = !busy;
            _apply.IsEnabled = !busy && _parameters != null && _editors.Count > 0;
            _parameterRows.IsEnabled = !busy;
            _terminationRows.IsEnabled = !busy;
        }
    }
}
