// SPDX-License-Identifier: Apache-2.0
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using TabPage = System.Windows.Controls.TabItem;
using Label = System.Windows.Controls.TextBlock;
using Button = System.Windows.Controls.Button;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private List<ServoGimbalAxis> _editingGimbalAxes = new List<ServoGimbalAxis>();
        private Label _gimbalOutputs;
        private Button _configureGimbal;
        private void AddServoGimbalSettings(TabPage tab, ref int y)
        {
            AddSectionLabel(tab, "Gimbal outputs", ref y);
            _gimbalOutputs = AddLabel(tab, "Mission Planner mount", 20, y);
            _gimbalOutputs.Width = 430;
            _configureGimbal = WpfUi.Action("Configure", () =>
            {
                using (var editor = new ServoGimbalEditorForm(_editingGimbalAxes))
                {
                    if (editor.ShowDialog(this) != DialogResult.OK) { return; }
                    _editingGimbalAxes = editor.Axes;
                    _chkTrackingCalibrated.IsChecked = false;
                    ShowGimbalOutputs();
                }
            });
            System.Windows.Automation.AutomationProperties.SetName(_configureGimbal, "Configure this video input's gimbal outputs");
            AddAt(tab, _configureGimbal, 470, y, 110); y += 40;
        }
        private void LoadServoGimbalSettings(VideoInputProfile input)
        {
            _editingGimbalAxes = input?.ServoGimbalAxes.Select(axis => axis.Copy()).ToList() ?? new List<ServoGimbalAxis>();
            _configureGimbal.IsEnabled = input != null;
            ShowGimbalOutputs();
        }
        private void ShowGimbalOutputs() => _gimbalOutputs.Text = _editingGimbalAxes.Count == 0 ? "Mission Planner mount" :
            string.Join(" · ", _editingGimbalAxes.Select(axis => axis.Axis + " → output " + axis.Channel));
    }
}
