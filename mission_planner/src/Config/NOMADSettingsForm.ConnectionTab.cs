// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private DataGridView _linkList;
        private ComboBox _cmbRouterPreferred;
        private Label _linkError;
        private TabPage CreateMultiLinkTab()
        {
            var tab = CreateTabPage("Links"); int y = 15;
            _chkRouterClientEnabled = AddCheckBox(tab, "Enable", 20, y); y += 38;
            AddLabel(tab, "Mission Planner UDPCl port:", 20, y);
            _numRouterLocalPort = AddNumericUpDown(tab, 235, y, 90, 1, 65535, 14600); y += 35;
            AddLabel(tab, "Preferred enabled link:", 20, y);
            _cmbRouterPreferred = AddComboBox(tab, 235, y, 400, Array.Empty<string>()); y += 40;
            _linkList = CreateDefinitionList("Name", "Transport", "Endpoint", "Priority", "Enabled");
            _linkList.Location = new Point(20, y); _linkList.Size = new Size(690, 210); tab.Controls.Add(_linkList); y += 225;
            var add = ControlFactory.Button("Add", (sender, args) => EditRouterLink(-1));
            var edit = ControlFactory.Button("Edit", (sender, args) => EditRouterLink(SelectedRow(_linkList)));
            var remove = ControlFactory.Button("Remove", (sender, args) => RemoveRouterLink());
            var buttons = ControlFactory.ButtonRow(add, edit, remove); buttons.Dock = DockStyle.None;
            buttons.Location = new Point(20, y); tab.Controls.Add(buttons); y += 42;
            _linkError = new Label { Location = new Point(20, y), Size = new Size(690, 45), ForeColor = NOMADTheme.WARNING };
            tab.Controls.Add(_linkError);
            _linkList.CellDoubleClick += (sender, args) => { if (args.RowIndex >= 0) { EditRouterLink(args.RowIndex); } };
            return tab;
        }
        private void LoadRouterLinks()
        {
            _linkError.Text = "";
            _linkList.Rows.Clear();
            foreach (var link in Config.RouterLinks)
            {
                string endpoint = link.Transport == "COM" ? link.Device + " @ " + link.BaudRate :
                    link.Transport == "TCP" ? link.RemoteHost + ":" + link.Port : link.BindAddress + ":" + link.Port;
                _linkList.Rows.Add(link.Name, link.Transport, endpoint, link.Priority, link.Enabled);
            }
            var options = new List<KeyValuePair<string, string>> { new KeyValuePair<string, string>("", "Automatic priority") };
            options.AddRange(Config.RouterLinks.Where(link => link.Enabled).Select(link =>
                new KeyValuePair<string, string>(link.Id, link.Name + " [" + link.Id + "]")));
            SetBindingChoices(_cmbRouterPreferred, options, Config.RouterPreferredLink);
        }
        private NOMADConfig RouterDraft()
        {
            var candidate = Config.Clone();
            candidate.RouterLocalPort = (int)_numRouterLocalPort.Value;
            candidate.RouterPreferredLink = BindingValue(_cmbRouterPreferred);
            return candidate;
        }
        private void EditRouterLink(int index)
        {
            if (index < -1 || index >= Config.RouterLinks.Count) { return; }
            try
            {
                using (var editor = new RouterLinkEditorForm(RouterDraft(), index, "link_" + Guid.NewGuid().ToString("N")))
                { if (editor.ShowDialog(this) == DialogResult.OK) { ApplyRouterDraft(editor.Candidate); } }
            }
            catch (Exception error) { _linkError.Text = error.Message; }
        }
        private void ApplyRouterDraft(NOMADConfig candidate)
        {
            candidate.Validate();
            Config.RouterLinks = candidate.RouterLinks.Select(link => link.Snapshot()).ToList();
            Config.RouterLocalPort = candidate.RouterLocalPort; Config.RouterPreferredLink = candidate.RouterPreferredLink;
            LoadRouterLinks();
        }
        private void RemoveRouterLink()
        {
            int index = SelectedRow(_linkList); if (index < 0) { return; }
            try
            {
                var candidate = RouterDraft(); candidate.RouterLinks.RemoveAt(index);
                ApplyRouterDraft(candidate);
            }
            catch (Exception error) { _linkError.Text = error.Message; }
        }
    }
}
