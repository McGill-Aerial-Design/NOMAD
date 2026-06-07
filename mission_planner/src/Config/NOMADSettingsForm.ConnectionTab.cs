// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.IO.Ports;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private TabPage CreateConnectionTab()
        {
            var tab = CreateTabPage("Connection");
            int y = 15;

            AddSectionLabel(tab, "Jetson Connection", ref y);

            AddLabel(tab, "Jetson IP:", 20, y);
            _txtJetsonIP = AddTextBox(tab, 150, y, 180);
            y += 30;

            AddLabel(tab, "API Port:", 20, y);
            _numPort = AddNumericUpDown(tab, 150, y, 80, 1, 65535, 8000);
            y += 30;

            AddLabel(tab, "API Key:", 20, y);
            _txtJetsonApiKey = AddTextBox(tab, 150, y, 180);
            _txtJetsonApiKey.UseSystemPasswordChar = true;
            y += 30;

            AddLabel(tab, "Tailscale IP:", 20, y);
            _txtTailscaleIP = AddTextBox(tab, 150, y, 180);
            y += 30;

            _chkUseTailscale = AddCheckBox(tab, "Use Tailscale IP", 20, y);
            y += 35;

            AddSectionLabel(tab, "SSH / HTTP Settings", ref y);

            AddLabel(tab, "SSH Username:", 20, y);
            _txtSshUsername = AddTextBox(tab, 150, y, 100);
            y += 30;

            AddLabel(tab, "HTTP Timeout (s):", 20, y);
            _numHttpTimeout = AddNumericUpDown(tab, 150, y, 60, 1, 60, 5);
            y += 30;

            _chkAutoReconnect = AddCheckBox(tab, "Auto-reconnect on connection loss", 20, y);
            y += 30;

            AddLabel(tab, "Health Poll (ms):", 20, y);
            _numHealthPollInterval = AddNumericUpDown(tab, 150, y, 80, 500, 30000, 5000);

            return tab;
        }

        private TabPage CreateDualLinkTab()
        {
            var tab = CreateTabPage("Dual Link");
            int y = 15;

            AddSectionLabel(tab, "MAVLink Dual Link (LTE + RadioMaster)", ref y);

            _chkDualLinkEnabled = AddCheckBox(tab, "Enable NOMAD dual-link router", 20, y, Color.LimeGreen);
            _chkDualLinkEnabled.CheckedChanged += (s, e) => UpdateDualLinkControlsState();
            y += 35;

            AddLabel(tab, "RadioMaster Type:", 40, y);
            _cmbRadioMasterConnType = AddComboBox(tab, 170, y, 80, new[] { "UDP", "COM", "TCP" });
            _cmbRadioMasterConnType.SelectedIndexChanged += (s, e) => UpdateRadioMasterConnTypeState();
            y += 30;

            _lblRadioMasterPort = AddLabel(tab, "UDP Port:", 40, y);
            _numRadioMasterPort = AddNumericUpDown(tab, 170, y, 80, 1, 65535, 14550);
            y += 30;

            _lblRadioTcpHost = AddLabel(tab, "TCP Host:", 40, y);
            _txtRadioTcpHost = AddTextBox(tab, 170, y, 110);
            y += 30;

            AddLabel(tab, "COM Port:", 40, y);
            _cmbRadioMasterComPort = AddComboBox(tab, 170, y, 80, SerialPort.GetPortNames());
            y += 30;

            AddLabel(tab, "Baud Rate:", 40, y);
            _cmbRadioMasterBaudRate = AddComboBox(tab, 170, y, 100, new[] { "115200", "420000", "460800", "921600" });
            y += 30;

            AddLabel(tab, "LTE MAVLink Port:", 40, y);
            _numLteMavlinkPort = AddNumericUpDown(tab, 170, y, 80, 1, 65535, 14560);
            y += 35;

            AddSectionLabel(tab, "Failover Settings", ref y);

            _chkAutoFailover = AddCheckBox(tab, "Enable Automatic Failover", 40, y);
            y += 30;

            AddLabel(tab, "Preferred Link:", 40, y);
            _cmbPreferredLink = AddComboBox(tab, 170, y, 130, new[] { "LTE (Tailscale)", "RadioMaster", "None" });
            y += 30;

            _chkAutoReconnectPreferred = AddCheckBox(tab, "Auto-reconnect to preferred link", 40, y);
            y += 30;

            AddLabel(tab, "Reconnect Delay (s):", 40, y);
            _numPreferredReconnectDelay = AddNumericUpDown(tab, 170, y, 60, 1, 120, 10);
            y += 30;

            AddLabel(tab, "Heartbeat Timeout (s):", 40, y);
            _numHeartbeatTimeout = AddNumericUpDown(tab, 170, y, 60, 1, 30, 3, 1);
            y += 30;

            AddLabel(tab, "Monitor Interval (ms):", 40, y);
            _numLinkMonitorInterval = AddNumericUpDown(tab, 170, y, 80, 100, 5000, 500);
            y += 35;

            AddSectionLabel(tab, "Local Router (MAVProxy-style)", ref y);

            AddLabel(tab, "Bind Address:", 40, y);
            _txtRouterBindAddress = AddTextBox(tab, 170, y, 130);
            y += 30;

            AddLabel(tab, "Local UDP Port:", 40, y);
            _numRouterLocalPort = AddNumericUpDown(tab, 170, y, 80, 1024, 65535, 14600);
            y += 30;

            _chkRouterDedup = AddCheckBox(tab, "Deduplicate cross-link packets", 40, y);
            y += 30;

            var hint = new Label
            {
                Text = "Enabled: connect Mission Planner as UDP Client / UDPCl to 127.0.0.1:14600 for merged LTE/RadioMaster failover.\n" +
                       "Disabled: connect Mission Planner directly to LTE UDP 14560 or RadioMaster UDP 14550.",
                Font = new Font("Segoe UI", 8, FontStyle.Italic),
                ForeColor = Color.FromArgb(150, 150, 150),
                Location = new Point(40, y),
                AutoSize = true,
                MaximumSize = new Size(440, 0),
            };
            tab.Controls.Add(hint);

            return tab;
        }

        private TabPage CreateVioTab()
        {
            var tab = CreateTabPage("VIO");
            int y = 15;

            AddSectionLabel(tab, "Visual-Inertial Odometry (Isaac ROS)", ref y);

            AddLabel(tab, "Confidence Warning (%):", 20, y);
            _numVioConfidenceWarning = AddNumericUpDown(tab, 180, y, 70, 0, 100, 50);
            y += 30;

            AddLabel(tab, "Confidence Critical (%):", 20, y);
            _numVioConfidenceCritical = AddNumericUpDown(tab, 180, y, 70, 0, 100, 30);
            y += 35;

            _chkVioAlertsEnabled = AddCheckBox(tab, "Enable VIO status alerts", 20, y);
            y += 40;

            AddSectionLabel(tab, "Terminal Settings", ref y);

            AddLabel(tab, "Command Timeout (s):", 20, y);
            _numTerminalTimeout = AddNumericUpDown(tab, 180, y, 70, 5, 300, 30);
            y += 30;

            _chkSaveTerminalHistory = AddCheckBox(tab, "Save terminal history between sessions", 20, y);

            return tab;
        }

        private void UpdateDualLinkControlsState()
        {
            bool enabled = _chkDualLinkEnabled.Checked;
            _cmbRadioMasterConnType.Enabled = enabled;
            _numRadioMasterPort.Enabled = enabled;
            _cmbRadioMasterComPort.Enabled = enabled;
            _cmbRadioMasterBaudRate.Enabled = enabled;
            _numLteMavlinkPort.Enabled = enabled;
            _chkAutoFailover.Enabled = enabled;
            _cmbPreferredLink.Enabled = enabled;
            _chkAutoReconnectPreferred.Enabled = enabled;
            _numPreferredReconnectDelay.Enabled = enabled;
            _numHeartbeatTimeout.Enabled = enabled;
            _numLinkMonitorInterval.Enabled = enabled;
            if (_txtRouterBindAddress != null) _txtRouterBindAddress.Enabled = enabled;
            if (_numRouterLocalPort != null) _numRouterLocalPort.Enabled = enabled;
            if (_chkRouterDedup != null) _chkRouterDedup.Enabled = enabled;

            if (enabled)
            {
                UpdateRadioMasterConnTypeState();
            }
        }

        private void UpdateRadioMasterConnTypeState()
        {
            int idx = _cmbRadioMasterConnType.SelectedIndex;
            bool isUDP = idx == 0;
            bool isCOM = idx == 1;
            bool isTCP = idx == 2;
            _numRadioMasterPort.Visible = isUDP || isTCP;
            if (_lblRadioTcpHost != null) _lblRadioTcpHost.Visible = isTCP;
            if (_txtRadioTcpHost != null) _txtRadioTcpHost.Visible = isTCP;
            _cmbRadioMasterComPort.Visible = isCOM;
            _cmbRadioMasterBaudRate.Visible = isCOM;
            if (_lblRadioMasterPort != null)
            {
                _lblRadioMasterPort.Text = isUDP ? "UDP Port:" : (isTCP ? "TCP Port:" : "COM Port:");
            }
        }
    }
}
