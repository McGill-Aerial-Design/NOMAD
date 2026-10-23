// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;
namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private DataGridView _payloadList;
        private TabPage CreateActuatorsTab()
        {
            var tab = CreateTabPage("Payloads");
            _payloadList = CreateDefinitionList("Name", "Type", "Output", "Interlocks");
            _payloadList.Location = new Point(12, 15); _payloadList.Size = new Size(700, 260);
            tab.Controls.Add(_payloadList);
            var add = ControlFactory.Button("Add", (sender, args) => EditPayload(-1));
            var edit = ControlFactory.Button("Edit", (sender, args) => EditPayload(SelectedRow(_payloadList)));
            var remove = ControlFactory.Button("Remove", (sender, args) => RemovePayload());
            var buttons = ControlFactory.ButtonRow(add, edit, remove); buttons.Dock = DockStyle.None;
            buttons.Location = new Point(12, 290); tab.Controls.Add(buttons);
            _payloadList.CellDoubleClick += (sender, args) => { if (args.RowIndex >= 0) { EditPayload(args.RowIndex); } };
            return tab;
        }
        private void LoadPayloadList()
        {
            _payloadList.Rows.Clear();
            foreach (var definition in PayloadDefinition.Parse(Config.ActuatorDefinitions))
            {
                _payloadList.Rows.Add(definition.Name, PayloadEditorForm.BehaviorChoices.First(type => type.Key == definition.Behavior).Value,
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
