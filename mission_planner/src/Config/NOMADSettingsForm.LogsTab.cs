// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private TabPage CreateLogsTab()
        {
            var tab = CreateTabPage("Log Analysis");
            int y = 15;

            AddSectionLabel(tab, "Log Sources", ref y);
            AddLabel(tab, "Local log directory:", 20, y);
            _txtDefaultLogDirectory = AddTextBox(tab, 170, y, 410);
            y += 30;
            AddLabel(tab, "Jetson log directory:", 20, y);
            _txtJetsonLogDirectory = AddTextBox(tab, 170, y, 410);
            y += 40;

            AddSectionLabel(tab, "Health Thresholds", ref y);
            AddThresholdPair(tab, "Vibration (m/s2):", ref y,
                out _numLogVibrationWarning, out _numLogVibrationCritical, 0, 200, 1);
            AddThresholdPair(tab, "GPS HDOP:", ref y,
                out _numLogHdopWarning, out _numLogHdopCritical, 0, 20, 1);
            AddLabel(tab, "Minimum satellites:", 20, y);
            _numLogMinimumSatellites = AddNumericUpDown(tab, 210, y, 75, 0, 40, 8);
            y += 30;
            AddThresholdPair(tab, "Tune RMS error (deg):", ref y,
                out _numLogTuneWarning, out _numLogTuneCritical, 0, 90, 1);
            AddThresholdPair(tab, "EKF variance:", ref y,
                out _numLogEkfWarning, out _numLogEkfCritical, 0, 10, 2);

            AddSectionLabel(tab, "Live Tuning", ref y);
            AddLabel(tab, "Rolling buffer points:", 20, y);
            _numLogLiveBufferPoints = AddNumericUpDown(tab, 210, y, 90, 60, 10000, 600);
            y += 30;
            _chkLogInjectHud = AddCheckBox(tab, "Inject new tuning alerts into the Mission Planner HUD", 20, y);
            y += 25;
            AddLabel(tab, "Notifications always use the shared NOMAD notification service.", 20, y, Color.Gray);

            return tab;
        }

        private void AddThresholdPair(
            TabPage tab,
            string label,
            ref int y,
            out NumericUpDown warning,
            out NumericUpDown critical,
            decimal min,
            decimal max,
            int decimals)
        {
            AddLabel(tab, label, 20, y);
            AddLabel(tab, "Warn", 165, y);
            warning = AddNumericUpDown(tab, 210, y, 75, min, max, 0, decimals);
            AddLabel(tab, "Critical", 310, y);
            critical = AddNumericUpDown(tab, 370, y, 75, min, max, 0, decimals);
            y += 30;
        }
    }
}
