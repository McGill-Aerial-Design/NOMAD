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
        private CheckBox _chkUseELRS;
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

        // Google Drive
        private Button _btnUploadGDrive;
        private Label _lblGDriveStatus;

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
            _tabControl.TabPages.Add(CreateUploadsTab());
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
                BackColor = Color.FromArgb(0, 122, 204),
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

        // ============================================================
        // Settings Management
        // ============================================================

        private void LoadSettings()
        {
            // Connection
            _txtJetsonIP.Text = Config.JetsonIP;
            _numPort.Value = Config.JetsonPort;
            _txtJetsonApiKey.Text = Config.JetsonApiKey;
            _txtTailscaleIP.Text = Config.TailscaleIP;
            _chkUseTailscale.Checked = Config.UseTailscale;
            _txtSshUsername.Text = Config.SshUsername;
            _numHttpTimeout.Value = Config.HttpTimeoutSeconds;
            _chkAutoReconnect.Checked = Config.AutoReconnect;
            _numHealthPollInterval.Value = Config.HealthPollInterval;

            // Video
            _txtVideoUrl.Text = Config.VideoUrl;
            _numVideoCaching.Value = Config.VideoNetworkCaching;
            SetComboBoxValue(_cmbVideoPlayer, Config.PreferredVideoPlayer);
            _chkVideoAutoStart.Checked = Config.VideoAutoStart;
            _chkAutoStartHudVideo.Checked = Config.AutoStartHudVideo;

            // Dual Link
            _chkDualLinkEnabled.Checked = Config.DualLinkEnabled && Config.RouterEnabled;
            _cmbRadioMasterConnType.SelectedIndex = Config.RadioMasterConnectionType switch
            {
                "COM" => 1,
                "TCP" => 2,
                _ => 0 // default UDP
            };
            _numRadioMasterPort.Value = Config.RadioMasterPort;
            _txtRadioTcpHost.Text = Config.RadioMasterTcpHost;
            SetComboBoxValue(_cmbRadioMasterComPort, Config.RadioMasterComPort);
            SetComboBoxValue(_cmbRadioMasterBaudRate, Config.RadioMasterBaudRate.ToString());
            _numLteMavlinkPort.Value = Config.LteMavlinkPort;
            _chkAutoFailover.Checked = Config.AutoFailoverEnabled;
            _cmbPreferredLink.SelectedIndex = Config.PreferredMavlinkLink switch
            {
                "LTE" => 0,
                "RadioMaster" => 1,
                _ => 2
            };
            _chkAutoReconnectPreferred.Checked = Config.AutoReconnectToPreferred;
            _numPreferredReconnectDelay.Value = Config.PreferredLinkReconnectDelay;
            _numHeartbeatTimeout.Value = (decimal)Config.MavlinkHeartbeatTimeout;
            _numLinkMonitorInterval.Value = Config.LinkMonitorInterval;
            _txtRouterBindAddress.Text = Config.RouterBindAddress;
            _numRouterLocalPort.Value = Math.Max(_numRouterLocalPort.Minimum, Math.Min(_numRouterLocalPort.Maximum, Config.RouterLocalPort));
            _chkRouterDedup.Checked = Config.RouterDedupEnabled;

            // VIO / Terminal
            _numVioConfidenceWarning.Value = (decimal)Config.VioConfidenceWarning;
            _numVioConfidenceCritical.Value = (decimal)Config.VioConfidenceCritical;
            _chkVioAlertsEnabled.Checked = Config.VioAlertsEnabled;
            _numTerminalTimeout.Value = Config.TerminalTimeout;
            _chkSaveTerminalHistory.Checked = Config.SaveTerminalHistory;

            // UI
            _chkDarkMode.Checked = Config.DarkMode;
            _chkShowNotifications.Checked = Config.ShowNotifications;
            SetComboBoxValue(_cmbDefaultTab, Config.DefaultTab);
            _chkDebugMode.Checked = Config.DebugMode;
            _chkUseELRS.Checked = Config.UseELRS;
            _numSlamFov.Value = (decimal)Math.Max(30f, Math.Min(140f, Config.SlamCameraFovDeg));
            _numSlamMapRadius.Value = (decimal)Math.Max(1f, Math.Min(20f, Config.SlamMapRadiusM));

            // Alerts
            _numTempWarning.Value = (decimal)Config.TempWarningC;
            _numTempCritical.Value = (decimal)Config.TempCriticalC;
            _chkAudioAlerts.Checked = Config.AudioAlerts;

            // Payloads (drop / slider / relay) — own tab
            LoadPayloads();

            // Reels + camera tilt
            _numReelCh.Value      = Config.ReelServoChannel;
            _numReelPwmIn.Value   = Config.ReelPwmIn;
            _numReelPwmOut.Value  = Config.ReelPwmOut;
            _numReel2Ch.Value     = Config.Reel2ServoChannel;
            _numReel2PwmIn.Value  = Config.Reel2PwmIn;
            _numReel2PwmOut.Value = Config.Reel2PwmOut;
            _numTiltCh.Value          = Config.CameraTiltChannel;
            _numTiltPwmMin.Value      = Config.CameraTiltPwmMin;
            _numTiltPwmNeutral.Value  = Config.CameraTiltPwmNeutral;
            _numTiltPwmMax.Value      = Config.CameraTiltPwmMax;
            _numTiltAngleRange.Value  = Config.CameraTiltAngleRange;

            // Spray calibration
            _numSprayRange.Value       = (decimal)Config.SprayTargetCameraRangeM;
            _numSprayRangeTol.Value    = (decimal)Config.SprayRangeToleranceM;
            _numSprayTriggerMax.Value  = (decimal)Config.SprayTriggerMaxDistanceM;
            _numSprayAimX.Value        = Config.SprayAimPixelX;
            _numSprayAimY.Value        = Config.SprayAimPixelY;
            _numSprayAimTol.Value      = Config.SprayAimTolerancePx;
            _numSprayServoAngle.Value  = (decimal)Config.SprayServoFireAngleDeg;
            _numSprayForwardGain.Value = (decimal)Config.SprayForwardGain;
            _numSprayLateralGain.Value = (decimal)Config.SprayLateralGain;
            _numSprayAltitudeGain.Value= (decimal)Config.SprayAltitudeGain;
            _numSprayYawGain.Value     = (decimal)Config.SprayYawGain;
            _chkSprayUseYaw.Checked    = Config.SprayUseYawAlignment;
            _numSprayMaxForward.Value  = (decimal)Config.SprayMaxForwardSpeedMps;
            _numSprayMaxLateral.Value  = (decimal)Config.SprayMaxLateralSpeedMps;
            _numSprayMaxAltitude.Value = (decimal)Config.SprayMaxAltitudeSpeedMps;
            _numSprayMaxYaw.Value      = (decimal)Config.SprayMaxYawRateRadps;
            _numSprayLockMs.Value      = Config.SprayLockHoldMs;
            _numSprayTimeout.Value     = (decimal)Config.SprayAlignTimeoutS;

            // Joystick
            _chkJoyGimbalEnabled.Checked = Config.JoystickGimbalEnabled;
            SetComboBoxValue(_cmbJoyGimbalDevice,
                string.IsNullOrEmpty(Config.JoystickGimbalDevice) ? "(none)" : Config.JoystickGimbalDevice);
            SetComboBoxValue(_cmbJoyGimbalPitchAxis, Config.JoystickGimbalPitchAxis);
            _chkJoyGimbalPitchInvert.Checked = Config.JoystickGimbalPitchInvert;
            SetComboBoxValue(_cmbJoyGimbalRollAxis, Config.JoystickGimbalRollAxis);
            _chkJoyGimbalRollInvert.Checked = Config.JoystickGimbalRollInvert;
            _numJoyGimbalDeadzone.Value = (decimal)Math.Max(0f, Math.Min(0.5f, Config.JoystickGimbalDeadzone));
            _numJoyGimbalMaxRate.Value = (decimal)Math.Max(5f, Math.Min(200f, GimbalController.MaxRateDegSec));
            _numJoyGimbalMaxRate.ValueChanged += (s, e) =>
                GimbalController.MaxRateDegSec = (float)_numJoyGimbalMaxRate.Value;

            _chkJoyZedEnabled.Checked = Config.JoystickZedEnabled;
            SetComboBoxValue(_cmbJoyZedDevice,
                string.IsNullOrEmpty(Config.JoystickZedDevice) ? "(none)" : Config.JoystickZedDevice);
            SetComboBoxValue(_cmbJoyZedTiltAxis, Config.JoystickZedTiltAxis);
            _chkJoyZedTiltInvert.Checked = Config.JoystickZedTiltInvert;
            _numJoyZedDeadzone.Value = (decimal)Math.Max(0f, Math.Min(0.5f, Config.JoystickZedDeadzone));
            _numJoyZedMaxRate.Value = (decimal)Math.Max(50f, Math.Min(4000f, Config.JoystickZedMaxRateUsPerSec));

            SetComboBoxValue(_cmbSwitchDevice,
                string.IsNullOrEmpty(Config.JoystickSwitchDevice) ? "(none)" : Config.JoystickSwitchDevice);
            SetComboBoxValue(_cmbSw1Up,   LabelForActionId(Config.JoystickSw1UpAction));
            SetComboBoxValue(_cmbSw1Down, LabelForActionId(Config.JoystickSw1DownAction));
            SetComboBoxValue(_cmbSw2Up,   LabelForActionId(Config.JoystickSw2UpAction));
            SetComboBoxValue(_cmbSw2Down, LabelForActionId(Config.JoystickSw2DownAction));
            SetComboBoxValue(_cmbSw3Up,   LabelForActionId(Config.JoystickSw3UpAction));
            SetComboBoxValue(_cmbSw3Down, LabelForActionId(Config.JoystickSw3DownAction));

            _chkJoyAutoSelect.Checked = Config.JoystickAutoSelectDevice;
            _chkKillSwitchEnabled.Checked = Config.JoystickKillSwitchEnabled;
            _numKillLandSpeed.Value = Math.Max(_numKillLandSpeed.Minimum,
                                       Math.Min(_numKillLandSpeed.Maximum, Config.JoystickKillLandSpeedCmS));

            _chkSerialBridgeEnabled.Checked = Config.SerialJoystickEnabled;
            _txtSerialBridgePort.Text = Config.SerialJoystickPort ?? "";
            _numSerialBridgeBaud.Value = Math.Max(_numSerialBridgeBaud.Minimum,
                Math.Min(_numSerialBridgeBaud.Maximum, Config.SerialJoystickBaud));
            _txtSerialBridgePython.Text = Config.SerialJoystickPython ?? "python";
            _txtSerialBridgeScript.Text = Config.SerialJoystickScriptPath ?? "";

            UpdateDualLinkControlsState();
            UpdateRadioMasterConnTypeState();
        }

        private void SaveSettings()
        {
            // Connection
            Config.JetsonIP = _txtJetsonIP.Text.Trim();
            Config.JetsonPort = (int)_numPort.Value;
            Config.JetsonApiKey = _txtJetsonApiKey.Text.Trim();
            Config.TailscaleIP = _txtTailscaleIP.Text.Trim();
            Config.UseTailscale = _chkUseTailscale.Checked;
            Config.SshUsername = _txtSshUsername.Text.Trim();
            Config.HttpTimeoutSeconds = (int)_numHttpTimeout.Value;
            Config.AutoReconnect = _chkAutoReconnect.Checked;
            Config.HealthPollInterval = (int)_numHealthPollInterval.Value;

            // Video
            Config.VideoUrl = _txtVideoUrl.Text.Trim();
            Config.VideoNetworkCaching = (int)_numVideoCaching.Value;
            Config.PreferredVideoPlayer = _cmbVideoPlayer.SelectedItem?.ToString() ?? "Embedded";
            Config.VideoAutoStart = _chkVideoAutoStart.Checked;
            Config.AutoStartHudVideo = _chkAutoStartHudVideo.Checked;

            // Dual Link
            Config.DualLinkEnabled = _chkDualLinkEnabled.Checked;
            Config.RouterEnabled = _chkDualLinkEnabled.Checked;
            Config.RadioMasterConnectionType = _cmbRadioMasterConnType.SelectedIndex switch
            {
                1 => "COM",
                2 => "TCP",
                _ => "UDP"
            };
            Config.RadioMasterPort = (int)_numRadioMasterPort.Value;
            Config.RadioMasterTcpHost = string.IsNullOrWhiteSpace(_txtRadioTcpHost.Text) ? "127.0.0.1" : _txtRadioTcpHost.Text.Trim();
            Config.RadioMasterComPort = _cmbRadioMasterComPort.SelectedItem?.ToString() ?? "COM3";
            Config.RadioMasterBaudRate = int.TryParse(_cmbRadioMasterBaudRate.SelectedItem?.ToString(), out int baud) ? baud : 420000;
            Config.LteMavlinkPort = (int)_numLteMavlinkPort.Value;
            Config.AutoFailoverEnabled = _chkAutoFailover.Checked;
            Config.PreferredMavlinkLink = _cmbPreferredLink.SelectedIndex switch
            {
                0 => "LTE",
                1 => "RadioMaster",
                _ => "None"
            };
            Config.AutoReconnectToPreferred = _chkAutoReconnectPreferred.Checked;
            Config.PreferredLinkReconnectDelay = (int)_numPreferredReconnectDelay.Value;
            Config.MavlinkHeartbeatTimeout = (double)_numHeartbeatTimeout.Value;
            Config.LinkMonitorInterval = (int)_numLinkMonitorInterval.Value;
            Config.RouterBindAddress = string.IsNullOrWhiteSpace(_txtRouterBindAddress.Text)
                ? "127.0.0.1" : _txtRouterBindAddress.Text.Trim();
            Config.RouterLocalPort = (int)_numRouterLocalPort.Value;
            Config.RouterDedupEnabled = _chkRouterDedup.Checked;

            // VIO / Terminal
            Config.VioConfidenceWarning = (float)_numVioConfidenceWarning.Value;
            Config.VioConfidenceCritical = (float)_numVioConfidenceCritical.Value;
            Config.VioAlertsEnabled = _chkVioAlertsEnabled.Checked;
            Config.TerminalTimeout = (int)_numTerminalTimeout.Value;
            Config.SaveTerminalHistory = _chkSaveTerminalHistory.Checked;

            // UI
            Config.DarkMode = _chkDarkMode.Checked;
            Config.ShowNotifications = _chkShowNotifications.Checked;
            Config.DefaultTab = _cmbDefaultTab.SelectedItem?.ToString() ?? "Dashboard";
            Config.DebugMode = _chkDebugMode.Checked;
            Config.UseELRS = _chkUseELRS.Checked;
            Config.SlamCameraFovDeg = (float)_numSlamFov.Value;
            Config.SlamMapRadiusM = (float)_numSlamMapRadius.Value;

            // Alerts
            Config.TempWarningC = (float)_numTempWarning.Value;
            Config.TempCriticalC = (float)_numTempCritical.Value;
            Config.AudioAlerts = _chkAudioAlerts.Checked;

            // Payloads (drop / slider / relay) — own tab
            SavePayloads();

            // Reels + camera tilt
            Config.ReelServoChannel   = (int)_numReelCh.Value;
            Config.ReelPwmIn          = (int)_numReelPwmIn.Value;
            Config.ReelPwmOut         = (int)_numReelPwmOut.Value;
            Config.Reel2ServoChannel  = (int)_numReel2Ch.Value;
            Config.Reel2PwmIn         = (int)_numReel2PwmIn.Value;
            Config.Reel2PwmOut        = (int)_numReel2PwmOut.Value;
            Config.CameraTiltChannel   = (int)_numTiltCh.Value;
            Config.CameraTiltPwmMin    = (int)_numTiltPwmMin.Value;
            Config.CameraTiltPwmNeutral= (int)_numTiltPwmNeutral.Value;
            Config.CameraTiltPwmMax    = (int)_numTiltPwmMax.Value;
            Config.CameraTiltAngleRange= (int)_numTiltAngleRange.Value;

            // Spray calibration
            Config.SprayTargetCameraRangeM = (float)_numSprayRange.Value;
            Config.SprayRangeToleranceM = (float)_numSprayRangeTol.Value;
            Config.SprayTriggerMaxDistanceM = (float)_numSprayTriggerMax.Value;
            Config.SprayAimPixelX = (int)_numSprayAimX.Value;
            Config.SprayAimPixelY = (int)_numSprayAimY.Value;
            Config.SprayAimTolerancePx = (int)_numSprayAimTol.Value;
            Config.SprayServoFireAngleDeg = (float)_numSprayServoAngle.Value;
            Config.SprayForwardGain = (float)_numSprayForwardGain.Value;
            Config.SprayLateralGain = (float)_numSprayLateralGain.Value;
            Config.SprayAltitudeGain = (float)_numSprayAltitudeGain.Value;
            Config.SprayYawGain = (float)_numSprayYawGain.Value;
            Config.SprayUseYawAlignment = _chkSprayUseYaw.Checked;
            Config.SprayMaxForwardSpeedMps = (float)_numSprayMaxForward.Value;
            Config.SprayMaxLateralSpeedMps = (float)_numSprayMaxLateral.Value;
            Config.SprayMaxAltitudeSpeedMps = (float)_numSprayMaxAltitude.Value;
            Config.SprayMaxYawRateRadps = (float)_numSprayMaxYaw.Value;
            Config.SprayLockHoldMs = (int)_numSprayLockMs.Value;
            Config.SprayAlignTimeoutS = (float)_numSprayTimeout.Value;

            // Joystick
            Config.JoystickGimbalEnabled = _chkJoyGimbalEnabled.Checked;
            Config.JoystickGimbalDevice = NormalizeDevice(_cmbJoyGimbalDevice.SelectedItem?.ToString());
            Config.JoystickGimbalPitchAxis = _cmbJoyGimbalPitchAxis.SelectedItem?.ToString() ?? "Y";
            Config.JoystickGimbalPitchInvert = _chkJoyGimbalPitchInvert.Checked;
            Config.JoystickGimbalRollAxis = _cmbJoyGimbalRollAxis.SelectedItem?.ToString() ?? "X";
            Config.JoystickGimbalRollInvert = _chkJoyGimbalRollInvert.Checked;
            Config.JoystickGimbalDeadzone = (float)_numJoyGimbalDeadzone.Value;
            Config.JoystickGimbalMaxRateDegSec = (float)_numJoyGimbalMaxRate.Value;
            GimbalController.MaxRateDegSec = (float)_numJoyGimbalMaxRate.Value;

            Config.JoystickZedEnabled = _chkJoyZedEnabled.Checked;
            Config.JoystickZedDevice = NormalizeDevice(_cmbJoyZedDevice.SelectedItem?.ToString());
            Config.JoystickZedTiltAxis = _cmbJoyZedTiltAxis.SelectedItem?.ToString() ?? "Y";
            Config.JoystickZedTiltInvert = _chkJoyZedTiltInvert.Checked;
            Config.JoystickZedDeadzone = (float)_numJoyZedDeadzone.Value;
            Config.JoystickZedMaxRateUsPerSec = (float)_numJoyZedMaxRate.Value;

            Config.JoystickSwitchDevice  = NormalizeDevice(_cmbSwitchDevice?.SelectedItem?.ToString());
            Config.JoystickSw1UpAction   = ActionIdForLabel(_cmbSw1Up?.SelectedItem?.ToString());
            Config.JoystickSw1DownAction = ActionIdForLabel(_cmbSw1Down?.SelectedItem?.ToString());
            Config.JoystickSw2UpAction   = ActionIdForLabel(_cmbSw2Up?.SelectedItem?.ToString());
            Config.JoystickSw2DownAction = ActionIdForLabel(_cmbSw2Down?.SelectedItem?.ToString());
            Config.JoystickSw3UpAction   = ActionIdForLabel(_cmbSw3Up?.SelectedItem?.ToString());
            Config.JoystickSw3DownAction = ActionIdForLabel(_cmbSw3Down?.SelectedItem?.ToString());

            Config.JoystickAutoSelectDevice = _chkJoyAutoSelect.Checked;
            Config.JoystickKillSwitchEnabled = _chkKillSwitchEnabled.Checked;
            Config.JoystickKillLandSpeedCmS = (int)_numKillLandSpeed.Value;

            Config.SerialJoystickEnabled = _chkSerialBridgeEnabled.Checked;
            Config.SerialJoystickPort = _txtSerialBridgePort.Text.Trim();
            Config.SerialJoystickBaud = (int)_numSerialBridgeBaud.Value;
            Config.SerialJoystickPython = _txtSerialBridgePython.Text.Trim();
            Config.SerialJoystickScriptPath = _txtSerialBridgeScript.Text.Trim();
        }

        private static string NormalizeDevice(string s)
            => string.IsNullOrEmpty(s) || s == "(none)" ? "" : s;

        private static decimal ClampDec(NumericUpDown nud, decimal value)
            => Math.Max(nud.Minimum, Math.Min(nud.Maximum, value));

        private void SetComboBoxValue(ComboBox combo, string value)
        {
            var index = combo.Items.IndexOf(value);
            if (index >= 0)
                combo.SelectedIndex = index;
            else if (combo.Items.Count > 0)
                combo.SelectedIndex = 0;
        }

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
