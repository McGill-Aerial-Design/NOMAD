// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private TabPage CreateVideoTab()
        {
            var tab = CreateTabPage("Video");
            int y = 15;

            AddSectionLabel(tab, "Video Stream", ref y);

            AddLabel(tab, "Video URL:", 20, y);
            _txtVideoUrl = AddTextBox(tab, 130, y, 330);
            y += 30;

            _chkAutoStartHudVideo = AddCheckBox(tab, "Auto-start video on HUD", 20, y);
            y += 35;

            AddSectionLabel(tab, "Nozzle Servo", ref y);

            AddLabel(tab, "Controlled through ArduPilot servo outputs.", 20, y, Color.FromArgb(180, 180, 180));
            y += 20;
            AddLabel(tab, "Use the payload servo sliders (Video tab) to adjust live.", 20, y, Color.FromArgb(180, 180, 180));

            return tab;
        }

        private TabPage CreateUiTab()
        {
            var tab = CreateTabPage("UI");
            int y = 15;

            AddSectionLabel(tab, "User Interface", ref y);

            AddSectionLabel(tab, "Debug / Advanced", ref y);

            _chkDebugMode = AddCheckBox(tab, "Enable Debug Logging", 20, y);
            y += 40;

            return tab;
        }

        private TabPage CreateAlertsTab()
        {
            var tab = CreateTabPage("Alerts");
            int y = 15;

            AddSectionLabel(tab, "Notification Settings", ref y);

            _chkAudioAlerts = AddCheckBox(tab, "Enable audio alerts for critical warnings", 20, y);
            y += 30;
            _chkAltitudeCallouts = AddCheckBox(tab, "Speak altitude callouts in flight (Airbus-style)", 20, y);

            return tab;
        }
    }
}
