// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Controls;
using ComboBox = System.Windows.Controls.ComboBox;
using CheckBox = System.Windows.Controls.CheckBox;
using Label = System.Windows.Controls.TextBlock;
using TabPage = System.Windows.Controls.TabItem;
using DialogResult = System.Windows.Forms.DialogResult;
using Newtonsoft.Json.Linq;
namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private DataGrid _inputList;
        private ComboBox _cmbGimbalHorizontal, _cmbGimbalVertical;
        private CheckBox _chkGimbalHorizontalInverted, _chkGimbalVerticalInverted;
        private TabPage CreateJoystickTab()
        {
            var tab = CreateTabPage("Inputs");
            _inputList = CreateDefinitionList("Control", "Input", "Action", "Enabled");
            _inputList.CanUserSortColumns = false;
            AddAt(tab, _inputList, 12, 15, 700, 260);
            var add = WpfUi.Action("Add control", () => EditInput(-1));
            var edit = WpfUi.Action("Edit", () => EditInput(SelectedRow(_inputList)));
            var remove = WpfUi.Action("Remove", () => RemoveInput());
            edit.IsEnabled = remove.IsEnabled = false;
            _inputList.SelectionChanged += (sender, args) =>
                edit.IsEnabled = remove.IsEnabled = SelectedRow(_inputList) >= 0;
            var buttons = new StackPanel { Orientation = Orientation.Horizontal };
            buttons.Children.Add(add); buttons.Children.Add(edit); buttons.Children.Add(remove);
            AddAt(tab, buttons, 12, 290);
            AddLabel(tab, "Gimbal maximum rate (deg/s):", 12, 340);
            _numJoyGimbalMaxRate = AddNumericUpDown(tab, 235, 337, 85, 5, 200, 60);
            AddLabel(tab, "Gimbal screen horizontal axis:", 12, 380);
            _cmbGimbalHorizontal = AddComboBox(tab, 235, 377, 120, GimbalCommand.ScreenAxes);
            _chkGimbalHorizontalInverted = AddCheckBox(tab, "Reverse", 375, 379);
            AddLabel(tab, "Gimbal screen vertical axis:", 12, 415);
            _cmbGimbalVertical = AddComboBox(tab, 235, 412, 120, GimbalCommand.ScreenAxes);
            _chkGimbalVerticalInverted = AddCheckBox(tab, "Reverse", 375, 414);
            AddLabel(tab, "The screen stick and arrow keys use these axes. Physical controls keep their bindings.", 12, 455);
            _inputList.MouseDoubleClick += (sender, args) =>
            {
                if (ItemsControl.ContainerFromElement(_inputList, args.OriginalSource as System.Windows.DependencyObject) is DataGridRow row)
                { int index = _inputList.ItemContainerGenerator.IndexFromContainer(row); if (index >= 0) { EditInput(index); } }
            };
            return tab;
        }
        private void LoadInputList()
        {
            _inputList.Items.Clear();
            var available = NomadJoystickService.EnumerateDevices();
            foreach (var binding in InputBinding.Parse(Config.Inputs))
            {
                string source = binding.Source == "Button" || binding.Source == "ButtonPair" ? InputListenState.ButtonLabel(binding.Button) +
                    (binding.Source == "ButtonPair" ? " / " + InputListenState.ButtonLabel(binding.OtherButton) : "") : binding.Axis + " · " + InputBindingEditorForm.SourceLabel(binding.Source);
                string device = NomadJoystickService.ResolveDeviceName(binding.Device, available) == null ?
                    "Unavailable: " + binding.Device : binding.Device;
                AddDefinitionRow(_inputList, binding.Name, device + " · " + source,
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
            combo.DisplayMemberPath = "Value";
            combo.Items.Clear(); foreach (var option in options) { combo.Items.Add(option); }
            combo.SelectedItem = options.First(option => option.Key == selected);
        }
    }
}
