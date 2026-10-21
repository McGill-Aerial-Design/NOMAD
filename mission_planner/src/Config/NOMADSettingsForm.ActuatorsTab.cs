// SPDX-License-Identifier: Apache-2.0
using System.Drawing;
using System.Windows.Forms;
namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private TextBox _actuatorJson;
        private TabPage CreateActuatorsTab()
        {
            var tab = CreateTabPage("Payloads");
            AddLabel(tab, "Edit this aircraft's payload definitions, review calibration, then Save the profile.", 10, 10);
            AddLabel(tab, "Timed servos require live SERVOx_TRIM = safe/neutral PWM. One native repeat at a time.", 10, 40, NOMADTheme.WARNING);
            _actuatorJson = new TextBox { Location = new Point(10, 75), Size = new Size(710, 300),
                Multiline = true, ScrollBars = ScrollBars.Both, WordWrap = false,
                BackColor = NOMADTheme.INPUT_BG, ForeColor = NOMADTheme.TEXT_PRIMARY, Font = NOMADTheme.Mono() };
            tab.Controls.Add(_actuatorJson);
            AddLabel(tab, "Mission Planner owns ACKs/retries; stop waits for an in-flight native command. Physical state is unverified.", 10, 390, NOMADTheme.WARNING);
            return tab;
        }
    }
}
