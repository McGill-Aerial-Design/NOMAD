// SPDX-License-Identifier: Apache-2.0
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

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
            _gimbalOutputs.AutoSize = false; _gimbalOutputs.Size = new Size(430, 32);
            _configureGimbal = ControlFactory.Button("Configure", width: 110, height: 28);
            _configureGimbal.Location = new Point(470, y);
            _configureGimbal.AccessibleName = "Configure this video input's gimbal outputs";
            _configureGimbal.Click += (sender, args) =>
            {
                using (var editor = new ServoGimbalEditorForm(_editingGimbalAxes))
                {
                    if (editor.ShowDialog(this) != DialogResult.OK) { return; }
                    _editingGimbalAxes = editor.Axes;
                    _chkTrackingCalibrated.Checked = false;
                    ShowGimbalOutputs();
                }
            };
            tab.Controls.Add(_configureGimbal); y += 40;
        }
        private void LoadServoGimbalSettings(VideoInputProfile input)
        {
            _editingGimbalAxes = input?.ServoGimbalAxes.Select(axis => axis.Copy()).ToList() ?? new List<ServoGimbalAxis>();
            _configureGimbal.Enabled = input != null;
            ShowGimbalOutputs();
        }
        private void ShowGimbalOutputs() => _gimbalOutputs.Text = _editingGimbalAxes.Count == 0 ? "Mission Planner mount" :
            string.Join(" · ", _editingGimbalAxes.Select(axis => axis.Axis + " → output " + axis.Channel));
    }
}
