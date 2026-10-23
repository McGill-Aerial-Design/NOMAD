// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;
namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private DataGridView _inputList;
        private TabPage CreateJoystickTab()
        {
            var tab = CreateTabPage("Inputs");
            _inputList = CreateDefinitionList("Control", "Input", "Action", "Enabled");
            _inputList.Location = new Point(12, 15); _inputList.Size = new Size(700, 260); tab.Controls.Add(_inputList);
            var add = ControlFactory.Button("Add control", (sender, args) => EditInput(-1), width: 120);
            var edit = ControlFactory.Button("Edit", (sender, args) => EditInput(SelectedRow(_inputList)));
            var remove = ControlFactory.Button("Remove", (sender, args) => RemoveInput());
            var buttons = ControlFactory.ButtonRow(add, edit, remove); buttons.Dock = DockStyle.None;
            buttons.Location = new Point(12, 290); tab.Controls.Add(buttons);
            AddLabel(tab, "Gimbal maximum rate (deg/s):", 12, 340);
            _numJoyGimbalMaxRate = AddNumericUpDown(tab, 235, 337, 85, 5, 200, 60);
            _inputList.CellDoubleClick += (sender, args) => { if (args.RowIndex >= 0) { EditInput(args.RowIndex); } };
            return tab;
        }
        private void LoadInputList()
        {
            _inputList.Rows.Clear();
            var available = NomadJoystickService.EnumerateDevices();
            foreach (var binding in InputBinding.Parse(Config.Inputs))
            {
                string source = binding.Source == "Button" || binding.Source == "ButtonPair" ? InputListenState.ButtonLabel(binding.Button) +
                    (binding.Source == "ButtonPair" ? " / " + InputListenState.ButtonLabel(binding.OtherButton) : "") : binding.Axis + " · " + InputBindingEditorForm.SourceLabel(binding.Source);
                string device = NomadJoystickService.ResolveDeviceName(binding.Device, available) == null ?
                    "Unavailable: " + binding.Device : binding.Device;
                _inputList.Rows.Add(binding.Name, device + " · " + source,
                    InputBindingEditorForm.ActionLabel(Config, binding.Action) + (binding.Paired ? " / " + InputBindingEditorForm.ActionLabel(Config, binding.OtherAction) : ""),
                    binding.Enabled ? "Yes" : "No");
            }
        }
        private void EditInput(int index)
        {
            if (index < -1 || index >= Config.Inputs.Count) { return; }
            using (var editor = new InputBindingEditorForm(Config, index))
            { if (editor.ShowDialog(this) == DialogResult.OK) { Config.Inputs = editor.Inputs; LoadInputList(); } }
        }
        private void RemoveInput()
        {
            int index = SelectedRow(_inputList); if (index < 0) { return; }
            var candidate = (JArray)Config.Inputs.DeepClone(); candidate.RemoveAt(index);
            InputBinding.Parse(candidate); Config.Inputs = candidate; LoadInputList();
        }
        private static string BindingValue(ComboBox combo) => combo.SelectedItem is KeyValuePair<string, string> choice ? choice.Key : "";
        internal static void SetBindingChoices(ComboBox combo, IEnumerable<KeyValuePair<string, string>> choices, string selected)
        {
            var options = choices.ToList(); selected = selected ?? "";
            if (!options.Any(option => option.Key == selected)) { options.Add(new KeyValuePair<string, string>(selected, "Unavailable: " + selected)); }
            combo.DisplayMember = "Value"; combo.ValueMember = "Key"; combo.DropDownWidth = 900;
            combo.Items.Clear(); foreach (var option in options) { combo.Items.Add(option); }
            combo.SelectedItem = options.First(option => option.Key == selected);
        }
    }
}
