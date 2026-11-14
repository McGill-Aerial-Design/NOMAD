// SPDX-License-Identifier: Apache-2.0
using System;
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
        private DataGrid _payloadList;
        private TabPage CreateActuatorsTab()
        {
            var tab = CreateTabPage("Payloads");
            _payloadList = CreateDefinitionList("Name", "Type", "Output", "Interlocks");
            _payloadList.CanUserSortColumns = false;
            AddAt(tab, _payloadList, 12, 15, 700, 260);
            var add = WpfUi.Action("Add", () => EditPayload(-1));
            var edit = WpfUi.Action("Edit", () => EditPayload(SelectedRow(_payloadList)));
            var remove = WpfUi.Action("Remove", () => RemovePayload());
            edit.IsEnabled = remove.IsEnabled = false;
            _payloadList.SelectionChanged += (sender, args) =>
                edit.IsEnabled = remove.IsEnabled = SelectedRow(_payloadList) >= 0;
            var buttons = new StackPanel { Orientation = Orientation.Horizontal };
            buttons.Children.Add(add); buttons.Children.Add(edit); buttons.Children.Add(remove);
            AddAt(tab, buttons, 12, 290);
            _payloadList.MouseDoubleClick += (sender, args) =>
            {
                if (ItemsControl.ContainerFromElement(_payloadList, args.OriginalSource as System.Windows.DependencyObject) is DataGridRow row)
                { int index = _payloadList.ItemContainerGenerator.IndexFromContainer(row); if (index >= 0) { EditPayload(index); } }
            };
            return tab;
        }
        private void LoadPayloadList()
        {
            _payloadList.Items.Clear();
            foreach (var definition in PayloadDefinition.Parse(Config.ActuatorDefinitions))
            {
                AddDefinitionRow(_payloadList, definition.Name, PayloadEditorForm.BehaviorChoices.First(type => type.Key == definition.Behavior).Value,
                    (definition.Relay ? "Relay " : "Servo ") + definition.Channel,
                    (definition.Hazardous ? "Hazardous · " : "") + definition.Confirmations + " confirmations");
            }
        }
        private void EditPayload(int index)
        {
            if (index < -1 || index >= (Config.ActuatorDefinitions?.Count ?? 0)) { return; }
            using (var editor = CreatePayloadEditor(index))
            { if (editor.ShowDialog(this) == DialogResult.OK) { ApplyPayloadDefinitions(editor.Definitions); } }
        }
        private PayloadEditorForm CreatePayloadEditor(int index) =>
            new PayloadEditorForm(Config.ActuatorDefinitions, index, "payload_" + Guid.NewGuid().ToString("N"));
        private void ApplyPayloadDefinitions(JArray candidate)
        {
            PayloadDefinition.Parse(candidate);
            Config.ActuatorDefinitions = (JArray)candidate.DeepClone();
            LoadPayloadList(); LoadInputList();
        }
        private void RemovePayload()
        {
            int index = SelectedRow(_payloadList); if (index < 0) { return; }
            var candidate = (JArray)Config.ActuatorDefinitions.DeepClone(); candidate.RemoveAt(index);
            ApplyPayloadDefinitions(candidate);
        }
    }
}
