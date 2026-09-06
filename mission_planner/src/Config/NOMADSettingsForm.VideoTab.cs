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

            AddLabel(tab, "Network Cache (ms):", 20, y);
            _numVideoCaching = AddNumericUpDown(tab, 150, y, 80, 0, 5000, 100);
            y += 30;

            AddLabel(tab, "Video Player:", 20, y);
            _cmbVideoPlayer = AddComboBox(tab, 150, y, 120, new[] { "Embedded", "VLC", "FFplay" });
            y += 30;

            _chkVideoAutoStart = AddCheckBox(tab, "Auto-start video on tab open", 20, y);
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

            _chkDarkMode = AddCheckBox(tab, "Dark Mode (NOMAD panels)", 20, y);
            y += 30;

            _chkShowNotifications = AddCheckBox(tab, "Show status change notifications", 20, y);
            y += 30;

            AddLabel(tab, "Default Tab:", 20, y);
            _cmbDefaultTab = AddComboBox(tab, 130, y, 130, new[] { "Dashboard", "Video", "Terminal", "Health" });
            y += 40;

            AddSectionLabel(tab, "Debug / Advanced", ref y);

            _chkDebugMode = AddCheckBox(tab, "Enable Debug Logging", 20, y);
            y += 40;

            AddSectionLabel(tab, "SLAM 3D View", ref y);

            AddLabel(tab, "Camera FOV (deg):", 20, y);
            _numSlamFov = AddNumericUpDown(tab, 150, y, 70, 30, 140, 60);
            y += 24;
            AddLabel(tab, "Lower = zoom in, higher = wider view", 20, y, Color.Gray);
            y += 26;

            AddLabel(tab, "Local Map Radius (m):", 20, y);
            _numSlamMapRadius = AddNumericUpDown(tab, 150, y, 70, 1, 20, 3);
            y += 24;
            AddLabel(tab, "Controls how much SLAM data stays in the Mission Planner view", 20, y, Color.Gray);

            return tab;
        }

        private TabPage CreateAlertsTab()
        {
            var tab = CreateTabPage("Alerts");
            int y = 15;

            AddSectionLabel(tab, "Temperature Monitoring", ref y);

            AddLabel(tab, "Warning Threshold (C):", 20, y);
            _numTempWarning = AddNumericUpDown(tab, 180, y, 70, 40, 100, 75);
            y += 30;

            AddLabel(tab, "Critical Threshold (C):", 20, y);
            _numTempCritical = AddNumericUpDown(tab, 180, y, 70, 50, 110, 85);
            y += 40;

            AddSectionLabel(tab, "Notification Settings", ref y);

            _chkAudioAlerts = AddCheckBox(tab, "Enable audio alerts for critical warnings", 20, y);
            y += 30;
            _chkAltitudeCallouts = AddCheckBox(tab, "Speak altitude callouts in flight (Airbus-style)", 20, y);

            return tab;
        }
    }
}
