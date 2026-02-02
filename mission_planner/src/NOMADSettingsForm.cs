// ============================================================
// NOMAD Settings Form
// ============================================================
// Comprehensive configuration dialog for NOMAD plugin settings.
// Includes all settings from NOMADConfig organized in tab pages.
// ============================================================

using System;
using System.Drawing;
using System.IO.Ports;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Settings dialog for NOMAD plugin configuration.
    /// Organized in tabs for better navigation of all settings.
    /// </summary>
    public class NOMADSettingsForm : Form
    {
        // ============================================================
        // Fields
        // ============================================================

        private TabControl _tabControl;
        
        // Connection Tab
        private TextBox _txtJetsonIP;
        private NumericUpDown _numPort;
        private TextBox _txtTailscaleIP;
        private CheckBox _chkUseTailscale;
        private TextBox _txtSshUsername;
        private NumericUpDown _numHttpTimeout;
        private CheckBox _chkAutoReconnect;
        private NumericUpDown _numHealthPollInterval;
        
        // Video Tab
        private TextBox _txtVideoUrl;
        private NumericUpDown _numServoChannel;
        private NumericUpDown _numServoMin;
        private NumericUpDown _numServoMax;
        private NumericUpDown _numServoCenter;
        private NumericUpDown _numVideoCaching;
        private ComboBox _cmbVideoPlayer;
        private CheckBox _chkVideoAutoStart;
        private CheckBox _chkAutoStartHudVideo;
        
        // Dual Link Tab
        private CheckBox _chkDualLinkEnabled;
        private ComboBox _cmbRadioMasterConnType;
        private NumericUpDown _numRadioMasterPort;
        private ComboBox _cmbRadioMasterComPort;
        private ComboBox _cmbRadioMasterBaudRate;
        private NumericUpDown _numLteMavlinkPort;
        private CheckBox _chkAutoFailover;
        private ComboBox _cmbPreferredLink;
        private CheckBox _chkAutoReconnectPreferred;
        private NumericUpDown _numPreferredReconnectDelay;
        private NumericUpDown _numHeartbeatTimeout;
        private NumericUpDown _numLinkMonitorInterval;
        
        // Tasks Tab
        private CheckBox _chkTask1Enabled;
        private CheckBox _chkTask1AutoCapture;
        private CheckBox _chkTask2Enabled;
        private NumericUpDown _numWasdNudgeSpeed;
        private NumericUpDown _numWasdAltSpeed;
        private CheckBox _chkWasdAutoEnable;
        
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
        
        // Alerts Tab
        private NumericUpDown _numTempWarning;
        private NumericUpDown _numTempCritical;
        private CheckBox _chkAudioAlerts;
        
        // GPIO Tab
        private NumericUpDown _numGpioPayload1;
        private NumericUpDown _numGpioPayload2;
        private NumericUpDown _numJetsonWaterGpio;
        private NumericUpDown _numJetsonServoPwm;
        
        // Buttons
        private Button _btnOK;
        private Button _btnCancel;
        private Button _btnTest;
        private Button _btnReset;

        /// <summary>
        /// Gets the configured settings.
        /// </summary>
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
            _tabControl.TabPages.Add(CreateTasksTab());
            _tabControl.TabPages.Add(CreateVioTab());
            _tabControl.TabPages.Add(CreateUiTab());
            _tabControl.TabPages.Add(CreateAlertsTab());
            _tabControl.TabPages.Add(CreateGpioTab());

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
        // Tab: Connection
        // ============================================================
        
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
            _numHealthPollInterval = AddNumericUpDown(tab, 150, y, 80, 500, 30000, 2000);
            
            return tab;
        }

        // ============================================================
        // Tab: Video
        // ============================================================
        
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

            AddSectionLabel(tab, "Camera Tilt Servo", ref y);

            AddLabel(tab, "Servo Channel:", 20, y);
            _numServoChannel = AddNumericUpDown(tab, 150, y, 60, 0, 16, 10);
            AddLabel(tab, "(0=off, 9-14=AUX1-6)", 220, y, Color.Gray);
            y += 30;

            AddLabel(tab, "Servo Min PWM:", 20, y);
            _numServoMin = AddNumericUpDown(tab, 150, y, 70, 500, 2500, 1000);
            y += 30;

            AddLabel(tab, "Servo Max PWM:", 20, y);
            _numServoMax = AddNumericUpDown(tab, 150, y, 70, 500, 2500, 2000);
            y += 30;

            AddLabel(tab, "Servo Center PWM:", 20, y);
            _numServoCenter = AddNumericUpDown(tab, 150, y, 70, 500, 2500, 1500);

            return tab;
        }

        // ============================================================
        // Tab: Dual Link
        // ============================================================
        
        private TabPage CreateDualLinkTab()
        {
            var tab = CreateTabPage("Dual Link");
            int y = 15;

            AddSectionLabel(tab, "MAVLink Dual Link (LTE + RadioMaster)", ref y);

            _chkDualLinkEnabled = AddCheckBox(tab, "Enable Dual Link Management", 20, y, Color.LimeGreen);
            _chkDualLinkEnabled.CheckedChanged += (s, e) => UpdateDualLinkControlsState();
            y += 35;

            AddLabel(tab, "RadioMaster Type:", 40, y);
            _cmbRadioMasterConnType = AddComboBox(tab, 170, y, 80, new[] { "UDP", "COM" });
            _cmbRadioMasterConnType.SelectedIndexChanged += (s, e) => UpdateRadioMasterConnTypeState();
            y += 30;

            AddLabel(tab, "UDP Port:", 40, y);
            _numRadioMasterPort = AddNumericUpDown(tab, 170, y, 80, 1, 65535, 14550);
            y += 30;

            AddLabel(tab, "COM Port:", 40, y);
            _cmbRadioMasterComPort = AddComboBox(tab, 170, y, 80, SerialPort.GetPortNames());
            y += 30;

            AddLabel(tab, "Baud Rate:", 40, y);
            _cmbRadioMasterBaudRate = AddComboBox(tab, 170, y, 100, new[] { "115200", "420000", "460800", "921600" });
            y += 30;

            AddLabel(tab, "LTE MAVLink Port:", 40, y);
            _numLteMavlinkPort = AddNumericUpDown(tab, 170, y, 80, 1, 65535, 14550);
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

            return tab;
        }

        // ============================================================
        // Tab: Tasks
        // ============================================================
        
        private TabPage CreateTasksTab()
        {
            var tab = CreateTabPage("Tasks");
            int y = 15;

            AddSectionLabel(tab, "Task 1 - Outdoor Recon (GPS)", ref y);

            _chkTask1Enabled = AddCheckBox(tab, "Enable Task 1 features", 20, y);
            y += 30;

            _chkTask1AutoCapture = AddCheckBox(tab, "Auto-capture on waypoint arrival", 40, y);
            y += 40;

            AddSectionLabel(tab, "Task 2 - Indoor (VIO)", ref y);

            _chkTask2Enabled = AddCheckBox(tab, "Enable Task 2 features", 20, y);
            y += 35;

            AddLabel(tab, "WASD Settings", 20, y, Color.FromArgb(200, 150, 0));
            y += 25;

            AddLabel(tab, "Nudge Speed (m/s):", 40, y);
            _numWasdNudgeSpeed = AddNumericUpDown(tab, 180, y, 70, 0.1m, 5.0m, 0.5m, 1);
            y += 30;

            AddLabel(tab, "Altitude Speed (m/s):", 40, y);
            _numWasdAltSpeed = AddNumericUpDown(tab, 180, y, 70, 0.1m, 3.0m, 0.3m, 1);
            y += 30;

            _chkWasdAutoEnable = AddCheckBox(tab, "Auto-enable WASD on Task 2 tab", 40, y);

            return tab;
        }

        // ============================================================
        // Tab: VIO
        // ============================================================
        
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

        // ============================================================
        // Tab: UI
        // ============================================================
        
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
            _cmbDefaultTab = AddComboBox(tab, 130, y, 130, new[] { "Dashboard", "Task 1", "Task 2", "Video", "Terminal", "Health" });
            y += 40;

            AddSectionLabel(tab, "Debug / Advanced", ref y);

            _chkDebugMode = AddCheckBox(tab, "Enable Debug Logging", 20, y);
            y += 30;

            _chkUseELRS = AddCheckBox(tab, "Use ELRS/MAVLink instead of HTTP", 20, y, Color.Orange);

            return tab;
        }

        // ============================================================
        // Tab: Alerts
        // ============================================================
        
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

            return tab;
        }

        // ============================================================
        // Tab: GPIO
        // ============================================================
        
        private TabPage CreateGpioTab()
        {
            var tab = CreateTabPage("GPIO");
            int y = 15;

            AddSectionLabel(tab, "Cube Orange Payload Pins (-1 = disabled)", ref y);

            AddLabel(tab, "Payload 1 AUX Pin:", 20, y);
            _numGpioPayload1 = AddNumericUpDown(tab, 180, y, 70, -1, 55, -1);
            AddLabel(tab, "(50-55 = AUX1-6)", 260, y, Color.Gray);
            y += 30;

            AddLabel(tab, "Payload 2 AUX Pin:", 20, y);
            _numGpioPayload2 = AddNumericUpDown(tab, 180, y, 70, -1, 55, -1);
            y += 40;

            AddSectionLabel(tab, "Jetson GPIO (via Edge Core API)", ref y);

            AddLabel(tab, "Water Pump GPIO:", 20, y);
            _numJetsonWaterGpio = AddNumericUpDown(tab, 180, y, 70, -1, 40, -1);
            y += 30;

            AddLabel(tab, "Servo PWM Channel:", 20, y);
            _numJetsonServoPwm = AddNumericUpDown(tab, 180, y, 70, -1, 4, -1);

            return tab;
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
            _numServoChannel.Value = Config.ZedServoChannel;
            _numServoMin.Value = Config.ZedServoMin;
            _numServoMax.Value = Config.ZedServoMax;
            _numServoCenter.Value = Config.ZedServoCenter;
            
            // Dual Link
            _chkDualLinkEnabled.Checked = Config.DualLinkEnabled;
            _cmbRadioMasterConnType.SelectedIndex = Config.RadioMasterConnectionType == "COM" ? 1 : 0;
            _numRadioMasterPort.Value = Config.RadioMasterPort;
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
            
            // Tasks
            _chkTask1Enabled.Checked = Config.Task1Enabled;
            _chkTask1AutoCapture.Checked = Config.Task1AutoCapture;
            _chkTask2Enabled.Checked = Config.Task2Enabled;
            _numWasdNudgeSpeed.Value = (decimal)Config.WasdNudgeSpeed;
            _numWasdAltSpeed.Value = (decimal)Config.WasdAltSpeed;
            _chkWasdAutoEnable.Checked = Config.WasdAutoEnable;
            
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
            
            // Alerts
            _numTempWarning.Value = (decimal)Config.TempWarningC;
            _numTempCritical.Value = (decimal)Config.TempCriticalC;
            _chkAudioAlerts.Checked = Config.AudioAlerts;
            
            // GPIO
            _numGpioPayload1.Value = Config.GpioPayload1Pin;
            _numGpioPayload2.Value = Config.GpioPayload2Pin;
            _numJetsonWaterGpio.Value = Config.JetsonWaterGpio;
            _numJetsonServoPwm.Value = Config.JetsonServoPwm;
            
            UpdateDualLinkControlsState();
            UpdateRadioMasterConnTypeState();
        }

        private void SaveSettings()
        {
            // Connection
            Config.JetsonIP = _txtJetsonIP.Text.Trim();
            Config.JetsonPort = (int)_numPort.Value;
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
            Config.ZedServoChannel = (int)_numServoChannel.Value;
            Config.ZedServoMin = (int)_numServoMin.Value;
            Config.ZedServoMax = (int)_numServoMax.Value;
            Config.ZedServoCenter = (int)_numServoCenter.Value;
            
            // Dual Link
            Config.DualLinkEnabled = _chkDualLinkEnabled.Checked;
            Config.RadioMasterConnectionType = _cmbRadioMasterConnType.SelectedIndex == 1 ? "COM" : "UDP";
            Config.RadioMasterPort = (int)_numRadioMasterPort.Value;
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
            
            // Tasks
            Config.Task1Enabled = _chkTask1Enabled.Checked;
            Config.Task1AutoCapture = _chkTask1AutoCapture.Checked;
            Config.Task2Enabled = _chkTask2Enabled.Checked;
            Config.WasdNudgeSpeed = (float)_numWasdNudgeSpeed.Value;
            Config.WasdAltSpeed = (float)_numWasdAltSpeed.Value;
            Config.WasdAutoEnable = _chkWasdAutoEnable.Checked;
            
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
            
            // Alerts
            Config.TempWarningC = (float)_numTempWarning.Value;
            Config.TempCriticalC = (float)_numTempCritical.Value;
            Config.AudioAlerts = _chkAudioAlerts.Checked;
            
            // GPIO
            Config.GpioPayload1Pin = (int)_numGpioPayload1.Value;
            Config.GpioPayload2Pin = (int)_numGpioPayload2.Value;
            Config.JetsonWaterGpio = (int)_numJetsonWaterGpio.Value;
            Config.JetsonServoPwm = (int)_numJetsonServoPwm.Value;
        }
        
        private void SetComboBoxValue(ComboBox combo, string value)
        {
            var index = combo.Items.IndexOf(value);
            if (index >= 0)
                combo.SelectedIndex = index;
            else if (combo.Items.Count > 0)
                combo.SelectedIndex = 0;
        }

        // ============================================================
        // State Updates
        // ============================================================
        
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
            
            if (enabled)
            {
                UpdateRadioMasterConnTypeState();
            }
        }
        
        private void UpdateRadioMasterConnTypeState()
        {
            bool isUDP = _cmbRadioMasterConnType.SelectedIndex == 0;
            _numRadioMasterPort.Visible = isUDP;
            _cmbRadioMasterComPort.Visible = !isUDP;
            _cmbRadioMasterBaudRate.Visible = !isUDP;
        }

        // ============================================================
        // Event Handlers
        // ============================================================
        
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

                using (var client = new System.Net.Http.HttpClient())
                {
                    client.Timeout = TimeSpan.FromSeconds(3);
                    var url = $"http://{Config.EffectiveIP}:{Config.JetsonPort}/health";
                    var response = await client.GetAsync(url);

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
    }
}
