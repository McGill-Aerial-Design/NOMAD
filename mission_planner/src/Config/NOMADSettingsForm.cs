// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Settings Form
// ============================================================
// Comprehensive configuration dialog for NOMAD plugin settings.
// Includes all settings from NOMADConfig organized in tab pages.
// ============================================================

using System;
using System.Drawing;
using System.IO;
using System.Linq;
using System.IO.Ports;
using System.Net.Http;
using System.Text;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm : Form
    {
        // ============================================================
        // Fields
        // ============================================================

        private TabControl _tabControl;

        // Connection Tab
        private TextBox _txtJetsonIP;
        private NumericUpDown _numPort;
        private TextBox _txtJetsonApiKey;
        private TextBox _txtTailscaleIP;
        private CheckBox _chkUseTailscale;
        private TextBox _txtSshUsername;
        private NumericUpDown _numHttpTimeout;
        private CheckBox _chkAutoReconnect;
        private NumericUpDown _numHealthPollInterval;

        // Video Tab
        private TextBox _txtVideoUrl;
        private NumericUpDown _numVideoCaching;
        private ComboBox _cmbVideoPlayer;
        private CheckBox _chkVideoAutoStart;
        private CheckBox _chkAutoStartHudVideo;

        // Dual Link Tab
        private CheckBox _chkDualLinkEnabled;
        private ComboBox _cmbRadioMasterConnType;
        private Label _lblRadioMasterPort;
        private NumericUpDown _numRadioMasterPort;
        private Label _lblRadioTcpHost;
        private TextBox _txtRadioTcpHost;
        private ComboBox _cmbRadioMasterComPort;
        private ComboBox _cmbRadioMasterBaudRate;
        private NumericUpDown _numLteMavlinkPort;
        private CheckBox _chkAutoFailover;
        private ComboBox _cmbPreferredLink;
        private CheckBox _chkAutoReconnectPreferred;
        private NumericUpDown _numPreferredReconnectDelay;
        private NumericUpDown _numHeartbeatTimeout;
        private NumericUpDown _numLinkMonitorInterval;
        private TextBox _txtRouterBindAddress;
        private NumericUpDown _numRouterLocalPort;
        private CheckBox _chkRouterDedup;

        // VIO Tab
        private NumericUpDown _numVioConfidenceWarning;
        private NumericUpDown _numVioConfidenceCritical;
        private CheckBox _chkVioAlertsEnabled;

        // Terminal Tab
        private NumericUpDown _numTerminalTimeout;
        private CheckBox _chkSaveTerminalHistory;

        // UI Tab
        private CheckBox _chkDebugMode;
        private CheckBox _chkShowNotifications;
        private ComboBox _cmbDefaultTab;
        private CheckBox _chkDarkMode;
        private NumericUpDown _numSlamFov;
        private NumericUpDown _numSlamMapRadius;

        // Alerts Tab
        private NumericUpDown _numTempWarning;
        private NumericUpDown _numTempCritical;
        private CheckBox _chkAudioAlerts;

        // Servos Tab (strap reels + camera tilt; drop/slider/relay payloads have their own tab)
        private NumericUpDown _numReelCh, _numReelPwmIn, _numReelPwmOut;
        private NumericUpDown _numReel2Ch, _numReel2PwmIn, _numReel2PwmOut;
        private NumericUpDown _numTiltCh, _numTiltPwmMin, _numTiltPwmNeutral, _numTiltPwmMax, _numTiltAngleRange;
        private NumericUpDown _numSprayRange, _numSprayRangeTol, _numSprayTriggerMax, _numSprayAimX, _numSprayAimY, _numSprayAimTol;
        private NumericUpDown _numSprayServoAngle, _numSprayForwardGain, _numSprayLateralGain, _numSprayAltitudeGain, _numSprayYawGain;
        private NumericUpDown _numSprayMaxForward, _numSprayMaxLateral, _numSprayMaxAltitude, _numSprayMaxYaw, _numSprayLockMs, _numSprayTimeout;
        private CheckBox _chkSprayUseYaw;
        private Button _btnPushSprayCalibration;
        private Label _lblSprayCalibrationStatus;

        // Joystick Tab
        private CheckBox _chkJoyGimbalEnabled, _chkJoyZedEnabled;
        private CheckBox _chkJoyGimbalPitchInvert, _chkJoyGimbalRollInvert, _chkJoyZedTiltInvert;
        private ComboBox _cmbJoyGimbalDevice, _cmbJoyZedDevice;
        private ComboBox _cmbJoyGimbalPitchAxis, _cmbJoyGimbalRollAxis, _cmbJoyZedTiltAxis;
        private NumericUpDown _numJoyGimbalDeadzone, _numJoyZedDeadzone;
        private NumericUpDown _numJoyGimbalMaxRate, _numJoyZedMaxRate;
        private Button _btnJoyRefreshDevices;
        private Label _lblJoyStatus;
        // 3-position switch action mapping (6 slots: sw1/2/3 x up/down)
        private ComboBox _cmbSw1Up, _cmbSw1Down, _cmbSw2Up, _cmbSw2Down, _cmbSw3Up, _cmbSw3Down;
        private ComboBox _cmbSwitchDevice;
        private CheckBox _chkKillSwitchEnabled;
        private NumericUpDown _numKillLandSpeed;
        private CheckBox _chkJoyAutoSelect;
        // Serial bridge sub-section
        private CheckBox _chkSerialBridgeEnabled;
        private TextBox _txtSerialBridgePort, _txtSerialBridgePython, _txtSerialBridgeScript;
        private NumericUpDown _numSerialBridgeBaud;
        private Label _lblSerialBridgeStatus;
        private System.Windows.Forms.Timer _serialBridgeStatusTimer;

        // Externally-provided status source for the live bridge indicator.
        private Func<string> _serialBridgeStatusProvider;
        public void SetSerialBridgeStatusProvider(Func<string> provider)
        {
            _serialBridgeStatusProvider = provider;
            RefreshSerialBridgeStatus();
        }

        // Buttons
        private Button _btnOK;
        private Button _btnCancel;
        private Button _btnTest;
        private Button _btnReset;

        public NOMADConfig Config { get; private set; }

        // ============================================================
        // Constructor
        // ============================================================

        public NOMADSettingsForm(NOMADConfig config)
        {
            Config = config ?? new NOMADConfig();
            InitializeComponents();
            LoadSettings();
        }

        // ============================================================
        // UI Initialization
        // ============================================================

        private void InitializeComponents()
        {
            this.Text = "NOMAD Settings";
            this.Size = new Size(550, 650);
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.StartPosition = FormStartPosition.CenterParent;
            this.BackColor = Color.FromArgb(45, 45, 48);
            this.ForeColor = Color.White;

            // Tab Control
            _tabControl = new TabControl
            {
                Location = new Point(10, 10),
                Size = new Size(515, 540),
                BackColor = Color.FromArgb(45, 45, 48),
            };
            this.Controls.Add(_tabControl);

            // Add all tab pages
            _tabControl.TabPages.Add(CreateConnectionTab());
            _tabControl.TabPages.Add(CreateVideoTab());
            _tabControl.TabPages.Add(CreateDualLinkTab());
            _tabControl.TabPages.Add(CreateVioTab());
            _tabControl.TabPages.Add(CreateUiTab());
            _tabControl.TabPages.Add(CreateAlertsTab());
            _tabControl.TabPages.Add(CreatePayloadsTab());
            _tabControl.TabPages.Add(CreateSprayCalibrationTab());
            _tabControl.TabPages.Add(CreateServosTab());
            _tabControl.TabPages.Add(CreateJoystickTab());

            // Buttons at bottom
            int buttonY = 560;

            _btnTest = new Button
            {
                Text = "Test Connection",
                Location = new Point(10, buttonY),
                Size = new Size(120, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.ACCENT,
                ForeColor = Color.White
            };
            _btnTest.Click += BtnTest_Click;
            this.Controls.Add(_btnTest);

            _btnReset = new Button
            {
                Text = "Reset Defaults",
                Location = new Point(140, buttonY),
                Size = new Size(110, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(150, 80, 0),
                ForeColor = Color.White
            };
            _btnReset.Click += BtnReset_Click;
            this.Controls.Add(_btnReset);

            _btnOK = new Button
            {
                Text = "Save",
                Location = new Point(330, buttonY),
                Size = new Size(90, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 150, 100),
                ForeColor = Color.White,
                DialogResult = DialogResult.OK
            };
            _btnOK.Click += (s, e) => SaveSettings();
            this.Controls.Add(_btnOK);

            _btnCancel = new Button
            {
                Text = "Cancel",
                Location = new Point(430, buttonY),
                Size = new Size(90, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(100, 100, 100),
                ForeColor = Color.White,
                DialogResult = DialogResult.Cancel
            };
            this.Controls.Add(_btnCancel);

            this.AcceptButton = _btnOK;
            this.CancelButton = _btnCancel;
        }

        // ============================================================
        // Helper Methods
        // ============================================================

        private TabPage CreateTabPage(string text)
        {
            var tab = new TabPage(text)
            {
                BackColor = Color.FromArgb(45, 45, 48),
                ForeColor = Color.White,
                AutoScroll = true,
                Padding = new Padding(5)
            };
            return tab;
        }

        private void AddSectionLabel(TabPage tab, string text, ref int y)
        {
            var label = new Label
            {
                Text = "-- " + text + " --",
                Location = new Point(10, y),
                AutoSize = true,
                ForeColor = Color.FromArgb(100, 180, 255),
                Font = new Font("Segoe UI", 9, FontStyle.Bold)
            };
            tab.Controls.Add(label);
            y += 25;
        }

        private Label AddLabel(TabPage tab, string text, int x, int y, Color? color = null)
        {
            var label = new Label
            {
                Text = text,
                Location = new Point(x, y + 3),
                AutoSize = true,
                ForeColor = color ?? Color.LightGray
            };
            tab.Controls.Add(label);
            return label;
        }

        private TextBox AddTextBox(TabPage tab, int x, int y, int width)
        {
            var textBox = new TextBox
            {
                Location = new Point(x, y),
                Size = new Size(width, 23),
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            tab.Controls.Add(textBox);
            return textBox;
        }

        private NumericUpDown AddNumericUpDown(TabPage tab, int x, int y, int width, decimal min, decimal max, decimal value, int decimals = 0)
        {
            var num = new NumericUpDown
            {
                Location = new Point(x, y),
                Size = new Size(width, 23),
                Minimum = min,
                Maximum = max,
                Value = Math.Max(min, Math.Min(max, value)),
                DecimalPlaces = decimals,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            tab.Controls.Add(num);
            return num;
        }

        private CheckBox AddCheckBox(TabPage tab, string text, int x, int y, Color? color = null)
        {
            var checkBox = new CheckBox
            {
                Text = text,
                Location = new Point(x, y),
                AutoSize = true,
                ForeColor = color ?? Color.White
            };
            tab.Controls.Add(checkBox);
            return checkBox;
        }

        private ComboBox AddComboBox(TabPage tab, int x, int y, int width, string[] items)
        {
            var combo = new ComboBox
            {
                Location = new Point(x, y),
                Size = new Size(width, 23),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            combo.Items.AddRange(items);
            if (combo.Items.Count > 0)
                combo.SelectedIndex = 0;
            tab.Controls.Add(combo);
            return combo;
        }

        // LoadSettings/SaveSettings live in NOMADSettingsForm.Settings.cs.

        private void BtnReset_Click(object sender, EventArgs e)
        {
            var result = MessageBox.Show(
                "Reset all settings to defaults?\n\nThis cannot be undone.",
                "Reset Settings",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning
            );

            if (result == DialogResult.Yes)
            {
                Config = new NOMADConfig();
                LoadSettings();
            }
        }

        private async void BtnTest_Click(object sender, EventArgs e)
        {
            _btnTest.Enabled = false;
            _btnTest.Text = "Testing...";

            try
            {
                SaveSettings();

                JetsonApiService.Reconfigure(Config);

                var response = await JetsonApiService.GetAsync("/health");

                if (response.IsSuccessStatusCode)
                {
                    MessageBox.Show(
                        $"Connection successful!\n\nJetson at {Config.EffectiveIP}:{Config.JetsonPort} is reachable.",
                        "Success",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information
                    );
                }
                else
                {
                    MessageBox.Show(
                        $"Connection failed: HTTP {(int)response.StatusCode}",
                        "Error",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Warning
                    );
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(
                    $"Connection failed:\n{ex.Message}",
                    "Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
            finally
            {
                _btnTest.Enabled = true;
                _btnTest.Text = "Test Connection";
            }
        }

        private void RefreshSerialBridgeStatus()
        {
            if (_lblSerialBridgeStatus == null) return;
            string status;
            try { status = _serialBridgeStatusProvider?.Invoke() ?? "(no bridge instance)"; }
            catch (Exception ex) { status = "Error: " + ex.Message; }

            _lblSerialBridgeStatus.Text = status;
            if (status.StartsWith("Running", StringComparison.OrdinalIgnoreCase))
                _lblSerialBridgeStatus.ForeColor = Color.LimeGreen;
            else if (status.StartsWith("Disabled", StringComparison.OrdinalIgnoreCase))
                _lblSerialBridgeStatus.ForeColor = Color.Goldenrod;
            else
                _lblSerialBridgeStatus.ForeColor = Color.IndianRed;
        }

    }
}
