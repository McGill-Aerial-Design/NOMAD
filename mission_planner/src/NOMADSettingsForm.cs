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
        private TextBox _txtRouterBindAddress;
        private NumericUpDown _numRouterLocalPort;
        private CheckBox _chkRouterDedup;

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
        private NumericUpDown _numSlamFov;
        private NumericUpDown _numSlamMapRadius;
        
        // Alerts Tab
        private NumericUpDown _numTempWarning;
        private NumericUpDown _numTempCritical;
        private CheckBox _chkAudioAlerts;
        
        // Servos Tab
        private NumericUpDown _numServo1Ch, _numServo1PwmMin, _numServo1PwmMax;
        private CheckBox _chkServo1Rev;
        private NumericUpDown _numServo1bCh, _numServo1bPwmMin, _numServo1bPwmMax;
        private CheckBox _chkServo1bRev;
        private NumericUpDown _numServo2Ch, _numServo2PwmMin, _numServo2PwmMax;
        private CheckBox _chkServo2Rev;
        private NumericUpDown _numServo3Ch, _numServo3PwmMin, _numServo3PwmMax;
        private CheckBox _chkServo3Rev;
        private NumericUpDown _numReelCh, _numReelPwmIn, _numReelPwmOut;
        private NumericUpDown _numReel2Ch, _numReel2PwmIn, _numReel2PwmOut;
        private NumericUpDown _numPumpRelay, _numPumpDuration, _numPumpRcChannel;
        private Button _btnApplyPumpRcMapping;
        private Label _lblPumpRcStatus;
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
        // 3-position switch action mapping (6 slots: sw1/2/3 × up/down)
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
        // Set by the plugin so the form doesn't need to know about
        // SerialJoystickBridge directly.
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
            _tabControl.TabPages.Add(CreateUploadsTab());
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

            AddSectionLabel(tab, "Nozzle Servo", ref y);

            AddLabel(tab, "Controlled through Cube Orange servo outputs.", 20, y, Color.FromArgb(180, 180, 180));
            y += 20;
            AddLabel(tab, "Use the nozzle slider in WASD Controls to adjust.", 20, y, Color.FromArgb(180, 180, 180));

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

            _chkDualLinkEnabled = AddCheckBox(tab, "Enable NOMAD dual-link router", 20, y, Color.LimeGreen);
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
        // Tab: Uploads
        // ============================================================

        private TabPage CreateUploadsTab()
        {
            var tab = CreateTabPage("Uploads");
            int y = 15;

            AddSectionLabel(tab, "Google Drive (Spray Photo Upload)", ref y);
            AddLabel(tab, "Upload OAuth2 token to Jetson (from gdrive_upload --setup):", 20, y, Color.LightGray);
            y += 25;

            _btnUploadGDrive = new Button
            {
                Text = "Upload GDrive Token...",
                Location = new Point(20, y),
                Size = new Size(200, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            _btnUploadGDrive.FlatAppearance.BorderSize = 0;
            _btnUploadGDrive.Click += async (s, e) => await UploadGDriveCredentials();
            tab.Controls.Add(_btnUploadGDrive);

            _lblGDriveStatus = new Label
            {
                Text = "",
                Location = new Point(230, y + 5),
                AutoSize = true,
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 9),
            };
            tab.Controls.Add(_lblGDriveStatus);

            return tab;
        }

        // ============================================================
        // Tab: Servos
        // ============================================================

        private TabPage CreateServosTab()
        {
            var tab = CreateTabPage("Servos");
            int y = 10;

            AddLabel(tab, "Channel = ArduPilot servo output number (e.g. 9 = AUX1).", 10, y, Color.Gray);
            y += 18;

            // Helper to add a servo row: label | ch | min | max
            void AddServoRow(string label, ref int row,
                out NumericUpDown ch, out NumericUpDown min, out NumericUpDown max,
                int defCh, int defMin, int defMax)
            {
                AddLabel(tab, label, 10, row);
                AddLabel(tab, "Ch", 155, row, Color.Gray);
                ch  = AddNumericUpDown(tab, 175, row, 45, 1, 99, defCh);
                AddLabel(tab, "Min", 228, row, Color.Gray);
                min = AddNumericUpDown(tab, 248, row, 55, 500, 2500, defMin);
                AddLabel(tab, "Max", 310, row, Color.Gray);
                max = AddNumericUpDown(tab, 330, row, 55, 500, 2500, defMax);
                row += 28;
            }

            AddSectionLabel(tab, "Drop Servos (PWM Max = drop position, Rev = servo mounted inverted)", ref y);
            int yRow;
            yRow = y; AddServoRow("Payload 1 (A):", ref y, out _numServo1Ch,  out _numServo1PwmMin,  out _numServo1PwmMax,  9,  1000, 2000); _chkServo1Rev  = AddCheckBox(tab, "Rev", 395, yRow);
            yRow = y; AddServoRow("Payload 1 (B):", ref y, out _numServo1bCh, out _numServo1bPwmMin, out _numServo1bPwmMax, 0,  1000, 2000); _chkServo1bRev = AddCheckBox(tab, "Rev", 395, yRow);
            _numServo1bCh.Minimum = 0; // 0 = disabled; AddServoRow hardcodes min=1 which would reject 0
            yRow = y; AddServoRow("Payload 2:", ref y, out _numServo2Ch, out _numServo2PwmMin, out _numServo2PwmMax, 10, 1000, 2000); _chkServo2Rev = AddCheckBox(tab, "Rev", 395, yRow);
            yRow = y; AddServoRow("Payload 3:", ref y, out _numServo3Ch, out _numServo3PwmMin, out _numServo3PwmMax, 11, 1000, 2000); _chkServo3Rev = AddCheckBox(tab, "Rev", 395, yRow);
            y += 4;

            AddSectionLabel(tab, "Strap Reel (Payload 1)", ref y);
            AddLabel(tab, "Channel:", 10, y);
            _numReelCh = AddNumericUpDown(tab, 100, y, 50, 1, 99, 12);
            y += 28;
            AddLabel(tab, "PWM In (>2000):", 10, y);
            _numReelPwmIn  = AddNumericUpDown(tab, 145, y, 60, 2001, 2500, 2100);
            y += 28;
            AddLabel(tab, "PWM Out (<1000):", 10, y);
            _numReelPwmOut = AddNumericUpDown(tab, 145, y, 60, 500, 999, 900);
            y += 32;

            AddSectionLabel(tab, "Strap Reel (Payload 2)", ref y);
            AddLabel(tab, "Channel:", 10, y);
            _numReel2Ch = AddNumericUpDown(tab, 100, y, 50, 1, 99, 13);
            y += 28;
            AddLabel(tab, "PWM In (>2000):", 10, y);
            _numReel2PwmIn  = AddNumericUpDown(tab, 145, y, 60, 2001, 2500, 2100);
            y += 28;
            AddLabel(tab, "PWM Out (<1000):", 10, y);
            _numReel2PwmOut = AddNumericUpDown(tab, 145, y, 60, 500, 999, 900);
            y += 32;

            AddSectionLabel(tab, "Water Pump (GPIO Relay)", ref y);
            AddLabel(tab, "Relay #:", 10, y);
            _numPumpRelay = AddNumericUpDown(tab, 100, y, 50, 0, 15, 0);
            AddLabel(tab, "(0 = RELAY1, 1 = RELAY2, …)", 160, y, Color.Gray);
            y += 28;
            AddLabel(tab, "Duration (ms):", 10, y);
            _numPumpDuration = AddNumericUpDown(tab, 130, y, 65, 50, 5000, 500);
            y += 28;
            AddLabel(tab, "RC Channel:", 10, y);
            _numPumpRcChannel = AddNumericUpDown(tab, 100, y, 50, 0, 16, 0);
            AddLabel(tab, "(0 = disabled, 5–16 = TX switch fires pump)", 160, y, Color.Gray);
            y += 28;
            _btnApplyPumpRcMapping = new Button
            {
                Text = "Apply RC Mapping",
                Location = new Point(10, y),
                Size = new Size(150, 26),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
            };
            _btnApplyPumpRcMapping.FlatAppearance.BorderSize = 0;
            _btnApplyPumpRcMapping.Click += BtnApplyPumpRcMapping_Click;
            tab.Controls.Add(_btnApplyPumpRcMapping);
            _lblPumpRcStatus = new Label
            {
                Text = "",
                Location = new Point(170, y + 5),
                AutoSize = true,
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
            };
            tab.Controls.Add(_lblPumpRcStatus);
            y += 36;

            AddSectionLabel(tab, "Camera Tilt (MAVLink primary, API fallback)", ref y);
            // Calibration: down=700us, straight=1250us, up=1450us, ±45° range
            AddServoRow("Camera Tilt:", ref y, out _numTiltCh, out _numTiltPwmMin, out _numTiltPwmMax, 14, 700, 1450);
            AddLabel(tab, "Neutral PWM:", 10, y);
            _numTiltPwmNeutral = AddNumericUpDown(tab, 120, y, 60, 500, 2500, 1250);
            AddLabel(tab, "Range (°):", 190, y, Color.Gray);
            _numTiltAngleRange = AddNumericUpDown(tab, 258, y, 50, 1, 90, 45);
            y += 28;

            return tab;
        }

        // ============================================================
        // Tab: Joystick — two DirectInput devices (gimbal + ZED tilt)
        // ============================================================
        private TabPage CreateJoystickTab()
        {
            var tab = CreateTabPage("Joystick");
            int y = 10;

            AddSectionLabel(tab, "Physical Joystick Routing", ref y);
            AddLabel(tab,
                "Two DirectInput sticks: one drives the CADDx gimbal,",
                10, y, Color.FromArgb(180, 180, 180));
            y += 18;
            AddLabel(tab,
                "the other drives the ZED tilt servo. RC override is never sent.",
                10, y, Color.FromArgb(180, 180, 180));
            y += 24;

            _chkJoyAutoSelect = AddCheckBox(tab,
                "Auto-pick first available device (and hot-plug retry)", 20, y, Color.LimeGreen);
            y += 28;

            var devices = NomadJoystickService.EnumerateDevices();
            var axes = new System.Collections.Generic.List<string>(NomadJoystickService.AxisNames).ToArray();
            string[] deviceList = BuildDeviceComboList(devices);

            // -------- Gimbal channel --------
            AddSectionLabel(tab, "Gimbal (CADDx)", ref y);

            _chkJoyGimbalEnabled = AddCheckBox(tab, "Enable", 20, y, Color.LimeGreen);
            y += 28;

            AddLabel(tab, "Device:", 20, y);
            _cmbJoyGimbalDevice = AddComboBox(tab, 90, y, 290, deviceList);
            y += 28;

            AddLabel(tab, "Pitch axis:", 20, y);
            _cmbJoyGimbalPitchAxis = AddComboBox(tab, 95, y, 80, axes);
            _chkJoyGimbalPitchInvert = AddCheckBox(tab, "invert", 185, y, Color.White);
            AddLabel(tab, "Roll axis:", 260, y);
            _cmbJoyGimbalRollAxis = AddComboBox(tab, 325, y, 60, axes);
            y += 28;

            _chkJoyGimbalRollInvert = AddCheckBox(tab, "invert roll", 325, y, Color.White);
            y += 28;

            AddLabel(tab, "Deadzone:", 20, y);
            _numJoyGimbalDeadzone = AddNumericUpDown(tab, 95, y, 60, 0.00m, 0.50m, 0.08m, 2);
            AddLabel(tab, "Max rate (°/s):", 175, y);
            _numJoyGimbalMaxRate = AddNumericUpDown(tab, 270, y, 70, 5, 200, 60);
            y += 36;

            // -------- ZED channel --------
            AddSectionLabel(tab, "ZED Tilt Servo", ref y);

            _chkJoyZedEnabled = AddCheckBox(tab, "Enable", 20, y, Color.LimeGreen);
            y += 28;

            AddLabel(tab, "Device:", 20, y);
            _cmbJoyZedDevice = AddComboBox(tab, 90, y, 290, deviceList);
            y += 28;

            AddLabel(tab, "Tilt axis:", 20, y);
            _cmbJoyZedTiltAxis = AddComboBox(tab, 95, y, 80, axes);
            _chkJoyZedTiltInvert = AddCheckBox(tab, "invert", 185, y, Color.White);
            y += 28;

            AddLabel(tab, "Deadzone:", 20, y);
            _numJoyZedDeadzone = AddNumericUpDown(tab, 95, y, 60, 0.00m, 0.50m, 0.08m, 2);
            AddLabel(tab, "Max rate (μs/s):", 175, y);
            _numJoyZedMaxRate = AddNumericUpDown(tab, 280, y, 70, 50, 4000, 400);
            y += 36;

            _btnJoyRefreshDevices = new Button
            {
                Text = "Refresh device list",
                Location = new Point(20, y),
                Size = new Size(160, 26),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(70, 70, 75),
                ForeColor = Color.White,
            };
            _btnJoyRefreshDevices.Click += (s, e) =>
            {
                var fresh = NomadJoystickService.EnumerateDevices();
                var freshList = BuildDeviceComboList(fresh);
                string keepG = _cmbJoyGimbalDevice.SelectedItem?.ToString();
                string keepZ = _cmbJoyZedDevice.SelectedItem?.ToString();
                string keepS = _cmbSwitchDevice?.SelectedItem?.ToString();
                _cmbJoyGimbalDevice.Items.Clear();
                _cmbJoyZedDevice.Items.Clear();
                _cmbJoyGimbalDevice.Items.AddRange(freshList);
                _cmbJoyZedDevice.Items.AddRange(freshList);
                SetComboBoxValue(_cmbJoyGimbalDevice, keepG);
                SetComboBoxValue(_cmbJoyZedDevice, keepZ);
                if (_cmbSwitchDevice != null)
                {
                    _cmbSwitchDevice.Items.Clear();
                    _cmbSwitchDevice.Items.AddRange(freshList);
                    SetComboBoxValue(_cmbSwitchDevice, keepS);
                }
                _lblJoyStatus.Text = $"{fresh.Count} device(s) detected.";
            };
            tab.Controls.Add(_btnJoyRefreshDevices);

            _lblJoyStatus = new Label
            {
                Text = $"{devices.Count} device(s) detected.",
                Location = new Point(190, y + 5),
                AutoSize = true,
                ForeColor = Color.FromArgb(180, 180, 180),
            };
            tab.Controls.Add(_lblJoyStatus);
            y += 36;

            // -------- 3-position switch action mapping --------
            // Each physical RadioMaster 3-position switch surfaces as a pair of
            // virtual buttons (UP / DOWN) via jotystick.py. Middle = both
            // released. Pick what each end-of-throw should fire.
            AddSectionLabel(tab, "Transmitter Switches (3-position)", ref y);
            AddLabel(tab,
                "Each switch centers in the middle; UP and DOWN each fire an action.",
                10, y, Color.FromArgb(180, 180, 180));
            y += 22;

            AddLabel(tab, "Device:", 20, y);
            _cmbSwitchDevice = AddComboBox(tab, 90, y, 290, deviceList);
            y += 28;

            string[] actionLabels = SwitchActionLabels();
            AddLabel(tab, "SW1 ↑:", 20, y);
            _cmbSw1Up = AddComboBox(tab, 80, y, 220, actionLabels);
            AddLabel(tab, "SW1 ↓:", 310, y);
            _cmbSw1Down = AddComboBox(tab, 370, y, 130, actionLabels);
            y += 28;

            AddLabel(tab, "SW2 ↑:", 20, y);
            _cmbSw2Up = AddComboBox(tab, 80, y, 220, actionLabels);
            AddLabel(tab, "SW2 ↓:", 310, y);
            _cmbSw2Down = AddComboBox(tab, 370, y, 130, actionLabels);
            y += 28;

            AddLabel(tab, "SW3 ↑:", 20, y);
            _cmbSw3Up = AddComboBox(tab, 80, y, 220, actionLabels);
            AddLabel(tab, "SW3 ↓:", 310, y);
            _cmbSw3Down = AddComboBox(tab, 370, y, 130, actionLabels);
            y += 36;

            // -------- Kill switch (XInput BACK button = radio kill pushbutton) --------
            AddSectionLabel(tab, "Kill Switch (pushbutton)", ref y);
            AddLabel(tab,
                "Pressing the radio's kill button commands LAND at the descent",
                10, y, Color.FromArgb(180, 180, 180));
            y += 18;
            AddLabel(tab,
                "rate below. Minimum 200 cm/s (2 m/s) per CONOPS §4.5.",
                10, y, Color.FromArgb(180, 180, 180));
            y += 22;

            _chkKillSwitchEnabled = AddCheckBox(tab, "Enable kill switch", 20, y, Color.IndianRed);
            y += 28;

            AddLabel(tab, "Descent (cm/s):", 20, y);
            _numKillLandSpeed = AddNumericUpDown(tab, 130, y, 80, 200, 800, 250);
            AddLabel(tab, "= " + "2.5 m/s default", 220, y, Color.Gray);
            y += 36;

            // -------- Serial → virtual gamepad bridge --------
            AddSectionLabel(tab, "Serial Bridge (jotystick.py)", ref y);
            AddLabel(tab, "Auto-launches a Python serial → virtual Xbox 360 bridge.",
                10, y, Color.FromArgb(180, 180, 180));
            y += 18;
            AddLabel(tab, "Requires pyserial + vgamepad + ViGEmBus driver.",
                10, y, Color.FromArgb(180, 180, 180));
            y += 24;

            _chkSerialBridgeEnabled = AddCheckBox(tab, "Enable bridge (spawn jotystick.py)", 20, y, Color.LimeGreen);
            y += 26;

            AddLabel(tab, "Serial port:", 20, y);
            _txtSerialBridgePort = AddTextBox(tab, 110, y, 80);
            AddLabel(tab, "Baud:", 210, y);
            _numSerialBridgeBaud = AddNumericUpDown(tab, 260, y, 80, 1200, 1000000, 115200);
            y += 26;

            AddLabel(tab, "Python:", 20, y);
            _txtSerialBridgePython = AddTextBox(tab, 110, y, 230);
            y += 26;

            AddLabel(tab, "Script path:", 20, y);
            _txtSerialBridgeScript = AddTextBox(tab, 110, y, 230);
            y += 28;

            AddLabel(tab, "Status:", 20, y);
            _lblSerialBridgeStatus = new Label
            {
                Text = "(unknown)",
                Location = new Point(110, y),
                AutoSize = true,
                ForeColor = Color.FromArgb(180, 180, 180),
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            tab.Controls.Add(_lblSerialBridgeStatus);

            // Poll the bridge status every 500 ms while the settings dialog is open.
            _serialBridgeStatusTimer = new System.Windows.Forms.Timer { Interval = 500 };
            _serialBridgeStatusTimer.Tick += (s, e) => RefreshSerialBridgeStatus();
            this.HandleCreated += (s, e) => _serialBridgeStatusTimer.Start();
            this.FormClosed   += (s, e) => { try { _serialBridgeStatusTimer?.Stop(); _serialBridgeStatusTimer?.Dispose(); } catch { } };

            return tab;
        }

        private void RefreshSerialBridgeStatus()
        {
            if (_lblSerialBridgeStatus == null) return;
            string status;
            try { status = _serialBridgeStatusProvider?.Invoke() ?? "(no bridge instance)"; }
            catch (Exception ex) { status = "Error: " + ex.Message; }

            _lblSerialBridgeStatus.Text = status;
            // Green when running, amber when disabled, red on failure / exit.
            if (status.StartsWith("Running", StringComparison.OrdinalIgnoreCase))
                _lblSerialBridgeStatus.ForeColor = Color.LimeGreen;
            else if (status.StartsWith("Disabled", StringComparison.OrdinalIgnoreCase))
                _lblSerialBridgeStatus.ForeColor = Color.Goldenrod;
            else
                _lblSerialBridgeStatus.ForeColor = Color.IndianRed;
        }

        // Switch action mapping helpers. Display labels in the ComboBoxes need
        // to round-trip to stable IDs persisted in NOMADConfig so renames in the
        // UI don't invalidate saved profiles.
        private static readonly (string Id, string Label)[] SwitchActionMap =
        {
            ("None",          "(unassigned)"),
            ("DropToggleP1",  "Drop / Retract Payload 1"),
            ("DropToggleP2",  "Drop / Retract Payload 2"),
            ("DropToggleP3",  "Drop / Retract Payload 3"),
            ("ReelInP1",      "Reel In — Payload 1 (hold)"),
            ("ReelOutP1",     "Reel Out — Payload 1 (hold)"),
            ("ReelInP2",      "Reel In — Payload 2 (hold)"),
            ("ReelOutP2",     "Reel Out — Payload 2 (hold)"),
            ("FireWaterPump", "Fire Water Pump"),
        };

        private static string[] SwitchActionLabels()
        {
            var labels = new string[SwitchActionMap.Length];
            for (int i = 0; i < SwitchActionMap.Length; i++) labels[i] = SwitchActionMap[i].Label;
            return labels;
        }

        private static string LabelForActionId(string id)
        {
            foreach (var (Id, Label) in SwitchActionMap)
                if (string.Equals(Id, id, StringComparison.OrdinalIgnoreCase)) return Label;
            return SwitchActionMap[0].Label;
        }

        private static string ActionIdForLabel(string label)
        {
            foreach (var (Id, Label) in SwitchActionMap)
                if (string.Equals(Label, label, StringComparison.OrdinalIgnoreCase)) return Id;
            return "None";
        }

        private static string[] BuildDeviceComboList(System.Collections.Generic.IList<string> devices)
        {
            var list = new System.Collections.Generic.List<string> { "(none)" };
            foreach (var d in devices) list.Add(d);
            return list.ToArray();
        }

        private TabPage CreateSprayCalibrationTab()
        {
            var tab = CreateTabPage("Spray");
            int y = 10;

            AddSectionLabel(tab, "Fixed Firing Geometry", ref y);
            AddLabel(tab, "Camera range (m):", 10, y);
            _numSprayRange = AddNumericUpDown(tab, 145, y, 70, 0.5m, 8.0m, 3.8m, 2);
            AddLabel(tab, "Tolerance (m):", 230, y, Color.Gray);
            _numSprayRangeTol = AddNumericUpDown(tab, 330, y, 60, 0.05m, 1.0m, 0.25m, 2);
            y += 28;

            AddLabel(tab, "Start max range (m):", 10, y);
            _numSprayTriggerMax = AddNumericUpDown(tab, 145, y, 70, 1.0m, 8.0m, 5.5m, 1);
            AddLabel(tab, "Operator can trigger before final approach.", 230, y, Color.Gray);
            y += 28;

            AddLabel(tab, "Aim pixel X:", 10, y);
            _numSprayAimX = AddNumericUpDown(tab, 145, y, 70, 0, 4000, 640);
            AddLabel(tab, "Y:", 230, y, Color.Gray);
            _numSprayAimY = AddNumericUpDown(tab, 260, y, 70, 0, 3000, 390);
            AddLabel(tab, "Tol px:", 345, y, Color.Gray);
            _numSprayAimTol = AddNumericUpDown(tab, 405, y, 55, 2, 250, 25);
            y += 28;

            AddLabel(tab, "Nozzle fire angle:", 10, y);
            _numSprayServoAngle = AddNumericUpDown(tab, 145, y, 70, 0, 180, 82, 1);
            AddLabel(tab, "Pump duration uses Servos tab value.", 230, y, Color.Gray);
            y += 36;

            AddSectionLabel(tab, "Alignment Gains", ref y);
            _chkSprayUseYaw = AddCheckBox(tab, "Use yaw for horizontal pixel alignment", 10, y);
            y += 28;
            AddLabel(tab, "Forward:", 10, y);
            _numSprayForwardGain = AddNumericUpDown(tab, 80, y, 70, 0, 2, 0.45m, 3);
            AddLabel(tab, "Yaw:", 170, y, Color.Gray);
            _numSprayYawGain = AddNumericUpDown(tab, 215, y, 75, -0.02m, 0.02m, 0.0025m, 4);
            y += 28;
            AddLabel(tab, "Lateral:", 10, y);
            _numSprayLateralGain = AddNumericUpDown(tab, 80, y, 75, -0.02m, 0.02m, 0.001m, 4);
            AddLabel(tab, "Altitude:", 170, y, Color.Gray);
            _numSprayAltitudeGain = AddNumericUpDown(tab, 240, y, 75, -0.02m, 0.02m, 0.001m, 4);
            y += 36;

            AddSectionLabel(tab, "Limits and Lock", ref y);
            AddLabel(tab, "Max fwd:", 10, y);
            _numSprayMaxForward = AddNumericUpDown(tab, 80, y, 60, 0.05m, 2, 0.45m, 2);
            AddLabel(tab, "lat:", 155, y, Color.Gray);
            _numSprayMaxLateral = AddNumericUpDown(tab, 190, y, 60, 0.05m, 1, 0.25m, 2);
            AddLabel(tab, "alt:", 265, y, Color.Gray);
            _numSprayMaxAltitude = AddNumericUpDown(tab, 300, y, 60, 0.05m, 1, 0.20m, 2);
            y += 28;
            AddLabel(tab, "Max yaw:", 10, y);
            _numSprayMaxYaw = AddNumericUpDown(tab, 80, y, 60, 0.05m, 2, 0.35m, 2);
            AddLabel(tab, "Lock ms:", 155, y, Color.Gray);
            _numSprayLockMs = AddNumericUpDown(tab, 220, y, 70, 100, 5000, 700);
            AddLabel(tab, "Timeout s:", 305, y, Color.Gray);
            _numSprayTimeout = AddNumericUpDown(tab, 385, y, 55, 2, 60, 20, 1);
            y += 40;

            _btnPushSprayCalibration = new Button
            {
                Text = "Push to Jetson",
                Location = new Point(10, y),
                Size = new Size(130, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
            };
            _btnPushSprayCalibration.Click += async (s, e) => await PushSprayCalibration();
            tab.Controls.Add(_btnPushSprayCalibration);

            _lblSprayCalibrationStatus = new Label
            {
                Text = "",
                Location = new Point(150, y + 6),
                AutoSize = true,
                ForeColor = Color.Gray,
            };
            tab.Controls.Add(_lblSprayCalibrationStatus);

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
            _txtRouterBindAddress.Text = Config.RouterBindAddress;
            _numRouterLocalPort.Value = Math.Max(_numRouterLocalPort.Minimum, Math.Min(_numRouterLocalPort.Maximum, Config.RouterLocalPort));
            _chkRouterDedup.Checked = Config.RouterDedupEnabled;

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
            _numSlamFov.Value = (decimal)Math.Max(30f, Math.Min(140f, Config.SlamCameraFovDeg));
            _numSlamMapRadius.Value = (decimal)Math.Max(1f, Math.Min(20f, Config.SlamMapRadiusM));
            
            // Alerts
            _numTempWarning.Value = (decimal)Config.TempWarningC;
            _numTempCritical.Value = (decimal)Config.TempCriticalC;
            _chkAudioAlerts.Checked = Config.AudioAlerts;
            
            // Servos
            _numServo1Ch.Value     = Config.Servo1Channel;
            _numServo1PwmMin.Value = Config.Servo1PwmMin;
            _numServo1PwmMax.Value = Config.Servo1PwmMax;
            _chkServo1Rev.Checked  = Config.Servo1Reversed;
            _numServo1bCh.Value    = Config.Servo1bChannel;
            _numServo1bPwmMin.Value= Config.Servo1bPwmMin;
            _numServo1bPwmMax.Value= Config.Servo1bPwmMax;
            _chkServo1bRev.Checked = Config.Servo1bReversed;
            _numServo2Ch.Value    = Config.Servo2Channel;
            _numServo2PwmMin.Value= Config.Servo2PwmMin;
            _numServo2PwmMax.Value= Config.Servo2PwmMax;
            _chkServo2Rev.Checked = Config.Servo2Reversed;
            _numServo3Ch.Value    = Config.Servo3Channel;
            _numServo3PwmMin.Value= Config.Servo3PwmMin;
            _numServo3PwmMax.Value= Config.Servo3PwmMax;
            _chkServo3Rev.Checked = Config.Servo3Reversed;
            _numReelCh.Value      = Config.ReelServoChannel;
            _numReelPwmIn.Value   = Config.ReelPwmIn;
            _numReelPwmOut.Value  = Config.ReelPwmOut;
            _numReel2Ch.Value     = Config.Reel2ServoChannel;
            _numReel2PwmIn.Value  = Config.Reel2PwmIn;
            _numReel2PwmOut.Value = Config.Reel2PwmOut;
            _numPumpRelay.Value    = Config.WaterPumpRelayNumber;
            _numPumpDuration.Value = Config.WaterPumpDurationMs;
            _numPumpRcChannel.Value = Math.Min(_numPumpRcChannel.Maximum, Math.Max(_numPumpRcChannel.Minimum, Config.WaterPumpRcChannel));
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
            // Single source of truth lives on GimbalController. Read the live
             // runtime value so this dialog matches what the floating gimbal
             // window's slider shows, then push edits straight back into the
             // controller as the user changes them.
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
            Config.RouterBindAddress = string.IsNullOrWhiteSpace(_txtRouterBindAddress.Text)
                ? "127.0.0.1" : _txtRouterBindAddress.Text.Trim();
            Config.RouterLocalPort = (int)_numRouterLocalPort.Value;
            Config.RouterDedupEnabled = _chkRouterDedup.Checked;

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
            Config.SlamCameraFovDeg = (float)_numSlamFov.Value;
            Config.SlamMapRadiusM = (float)_numSlamMapRadius.Value;
            
            // Alerts
            Config.TempWarningC = (float)_numTempWarning.Value;
            Config.TempCriticalC = (float)_numTempCritical.Value;
            Config.AudioAlerts = _chkAudioAlerts.Checked;
            
            // Servos
            Config.Servo1Channel      = (int)_numServo1Ch.Value;
            Config.Servo1PwmMin       = (int)_numServo1PwmMin.Value;
            Config.Servo1PwmMax       = (int)_numServo1PwmMax.Value;
            Config.Servo1Reversed     = _chkServo1Rev.Checked;
            Config.Servo1bChannel     = (int)_numServo1bCh.Value;
            Config.Servo1bPwmMin      = (int)_numServo1bPwmMin.Value;
            Config.Servo1bPwmMax      = (int)_numServo1bPwmMax.Value;
            Config.Servo1bReversed    = _chkServo1bRev.Checked;
            Config.Servo2Channel      = (int)_numServo2Ch.Value;
            Config.Servo2PwmMin       = (int)_numServo2PwmMin.Value;
            Config.Servo2PwmMax       = (int)_numServo2PwmMax.Value;
            Config.Servo2Reversed     = _chkServo2Rev.Checked;
            Config.Servo3Channel      = (int)_numServo3Ch.Value;
            Config.Servo3PwmMin       = (int)_numServo3PwmMin.Value;
            Config.Servo3PwmMax       = (int)_numServo3PwmMax.Value;
            Config.Servo3Reversed     = _chkServo3Rev.Checked;
            Config.ReelServoChannel   = (int)_numReelCh.Value;
            Config.ReelPwmIn          = (int)_numReelPwmIn.Value;
            Config.ReelPwmOut         = (int)_numReelPwmOut.Value;
            Config.Reel2ServoChannel  = (int)_numReel2Ch.Value;
            Config.Reel2PwmIn         = (int)_numReel2PwmIn.Value;
            Config.Reel2PwmOut        = (int)_numReel2PwmOut.Value;
            Config.WaterPumpRelayNumber = (int)_numPumpRelay.Value;
            Config.WaterPumpDurationMs  = (int)_numPumpDuration.Value;
            Config.WaterPumpRcChannel   = (int)_numPumpRcChannel.Value;
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
            bool isUDP = _cmbRadioMasterConnType.SelectedIndex == 0;
            _numRadioMasterPort.Visible = isUDP;
            _cmbRadioMasterComPort.Visible = !isUDP;
            _cmbRadioMasterBaudRate.Visible = !isUDP;
        }

        // ============================================================
        // Event Handlers
        // ============================================================

        private void BtnApplyPumpRcMapping_Click(object sender, EventArgs e)
        {
            int ch = (int)_numPumpRcChannel.Value;
            int relay = (int)_numPumpRelay.Value;

            // ArduPilot RCx_OPTION codes for relay toggle: RELAY1=28, RELAY2=34, RELAY3=35, RELAY4=36
            int optionCode;
            switch (relay)
            {
                case 0: optionCode = 28; break;
                case 1: optionCode = 34; break;
                case 2: optionCode = 35; break;
                case 3: optionCode = 36; break;
                default:
                    SetPumpRcStatus($"Relay {relay} has no RCx_OPTION code (use 0–3).", Color.OrangeRed);
                    return;
            }

            if (ch != 0 && (ch < 5 || ch > 16))
            {
                SetPumpRcStatus("RC channel must be 0 (disable) or 5–16.", Color.OrangeRed);
                return;
            }

            object comPort = null;
            try { comPort = global::MissionPlanner.MainV2.comPort; } catch { }
            if (comPort == null)
            {
                SetPumpRcStatus("Not connected to vehicle.", Color.OrangeRed);
                return;
            }

            // If disabling, clear the previously-saved channel's option (best-effort: clear all 5–16
            // would be invasive — instead clear the currently configured channel if known).
            if (ch == 0)
            {
                int prev = Config.WaterPumpRcChannel;
                if (prev >= 5 && prev <= 16)
                {
                    TrySetParamReflect(comPort, $"RC{prev}_OPTION", 0);
                    SetPumpRcStatus($"Cleared RC{prev}_OPTION.", Color.LightGreen);
                }
                else
                {
                    SetPumpRcStatus("Disabled (no prior channel to clear).", Color.Gray);
                }
                return;
            }

            bool ok = TrySetParamReflect(comPort, $"RC{ch}_OPTION", optionCode);
            SetPumpRcStatus(
                ok ? $"RC{ch}_OPTION = {optionCode} (relay {relay + 1})." : $"Failed to set RC{ch}_OPTION.",
                ok ? Color.LightGreen : Color.OrangeRed);
        }

        private void SetPumpRcStatus(string text, Color color)
        {
            if (_lblPumpRcStatus == null) return;
            _lblPumpRcStatus.Text = text;
            _lblPumpRcStatus.ForeColor = color;
        }

        private static bool TrySetParamReflect(object comPort, string name, double value)
        {
            try
            {
                var methods = comPort.GetType()
                    .GetMethods(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance)
                    .Where(m => m.Name == "setParam").ToList();
                foreach (var m in methods)
                {
                    var p = m.GetParameters();
                    if (p.Length >= 2 && p[0].ParameterType == typeof(string))
                    {
                        var args = new object[p.Length];
                        args[0] = name;
                        args[1] = Convert.ChangeType(value, p[1].ParameterType);
                        for (int i = 2; i < p.Length; i++)
                        {
                            if (p[i].HasDefaultValue) args[i] = p[i].DefaultValue;
                            else if (p[i].ParameterType == typeof(bool)) args[i] = true;
                            else args[i] = p[i].ParameterType.IsValueType
                                ? Activator.CreateInstance(p[i].ParameterType) : null;
                        }
                        try { m.Invoke(comPort, args); return true; }
                        catch { }
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: setParam({name}) error - {ex.Message}");
            }
            return false;
        }

        private async System.Threading.Tasks.Task UploadGDriveCredentials()
        {
            using (var dlg = new OpenFileDialog())
            {
                dlg.Title = "Select Google Drive OAuth2 Token JSON";
                dlg.Filter = "JSON files (*.json)|*.json";
                // Default to ~/.nomad/ where the token is saved
                var nomadDir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".nomad");
                if (Directory.Exists(nomadDir))
                    dlg.InitialDirectory = nomadDir;
                else
                    dlg.InitialDirectory = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);

                if (dlg.ShowDialog() != DialogResult.OK)
                    return;

                try
                {
                    _btnUploadGDrive.Enabled = false;
                    _lblGDriveStatus.Text = "Uploading...";
                    _lblGDriveStatus.ForeColor = Color.Yellow;

                    // Read the JSON file
                    string jsonContent = File.ReadAllText(dlg.FileName);

                    // Validate it looks like a token file
                    var parsed = JObject.Parse(jsonContent);
                    if (parsed["token"] == null && parsed["refresh_token"] == null)
                    {
                        _lblGDriveStatus.Text = "Invalid token JSON (no token/refresh_token)";
                        _lblGDriveStatus.ForeColor = Color.Red;
                        return;
                    }

                    // Upload to Jetson
                    var content = new StringContent(jsonContent, Encoding.UTF8, "application/json");
                    var response = await JetsonApiService.PostAsync("/api/admin/upload-gdrive-token", content);

                    if (response.IsSuccessStatusCode)
                    {
                        _lblGDriveStatus.Text = "Token uploaded to Jetson";
                        _lblGDriveStatus.ForeColor = Color.LimeGreen;
                    }
                    else
                    {
                        var body = await response.Content.ReadAsStringAsync();
                        string detail = body;
                        try
                        {
                            var err = JObject.Parse(body);
                            detail = err["detail"]?.ToString() ?? body;
                        }
                        catch { }
                        _lblGDriveStatus.Text = $"Failed: {detail}";
                        _lblGDriveStatus.ForeColor = Color.Red;
                    }
                }
                catch (Exception ex)
                {
                    _lblGDriveStatus.Text = $"Error: {ex.Message}";
                    _lblGDriveStatus.ForeColor = Color.Red;
                }
                finally
                {
                    _btnUploadGDrive.Enabled = true;
                }
            }
        }

        private JObject BuildSprayCalibrationPayload(bool persist)
        {
            return new JObject
            {
                ["target_camera_range_m"] = Config.SprayTargetCameraRangeM,
                ["range_tolerance_m"] = Config.SprayRangeToleranceM,
                ["trigger_max_distance_m"] = Config.SprayTriggerMaxDistanceM,
                ["aim_pixel_x"] = Config.SprayAimPixelX,
                ["aim_pixel_y"] = Config.SprayAimPixelY,
                ["aim_tolerance_px"] = Config.SprayAimTolerancePx,
                ["servo_fire_angle_deg"] = Config.SprayServoFireAngleDeg,
                ["spray_duration_ms"] = Config.WaterPumpDurationMs,
                ["water_pump_relay_number"] = Config.WaterPumpRelayNumber,
                ["forward_gain"] = Config.SprayForwardGain,
                ["lateral_gain"] = Config.SprayLateralGain,
                ["altitude_gain"] = Config.SprayAltitudeGain,
                ["yaw_gain"] = Config.SprayYawGain,
                ["use_yaw_alignment"] = Config.SprayUseYawAlignment,
                ["max_forward_speed_mps"] = Config.SprayMaxForwardSpeedMps,
                ["max_lateral_speed_mps"] = Config.SprayMaxLateralSpeedMps,
                ["max_altitude_speed_mps"] = Config.SprayMaxAltitudeSpeedMps,
                ["max_yaw_rate_radps"] = Config.SprayMaxYawRateRadps,
                ["lock_hold_ms"] = Config.SprayLockHoldMs,
                ["align_timeout_s"] = Config.SprayAlignTimeoutS,
                ["persist"] = persist,
            };
        }

        private async System.Threading.Tasks.Task PushSprayCalibration()
        {
            try
            {
                _btnPushSprayCalibration.Enabled = false;
                _lblSprayCalibrationStatus.Text = "Pushing...";
                _lblSprayCalibrationStatus.ForeColor = Color.Yellow;

                SaveSettings();
                Config.Save();
                JetsonApiService.Reconfigure(Config);

                var json = BuildSprayCalibrationPayload(true).ToString(Newtonsoft.Json.Formatting.None);
                var content = new StringContent(json, Encoding.UTF8, "application/json");
                var response = await JetsonApiService.PostAsync("/api/spray/calibration", content);

                if (response.IsSuccessStatusCode)
                {
                    _lblSprayCalibrationStatus.Text = "Spray calibration pushed";
                    _lblSprayCalibrationStatus.ForeColor = Color.LimeGreen;
                }
                else
                {
                    var body = await response.Content.ReadAsStringAsync();
                    string detail = body;
                    try
                    {
                        var err = JObject.Parse(body);
                        detail = err["detail"]?.ToString() ?? body;
                    }
                    catch { }
                    _lblSprayCalibrationStatus.Text = $"Failed: {detail}";
                    _lblSprayCalibrationStatus.ForeColor = Color.Red;
                }
            }
            catch (Exception ex)
            {
                _lblSprayCalibrationStatus.Text = $"Error: {ex.Message}";
                _lblSprayCalibrationStatus.ForeColor = Color.Red;
            }
            finally
            {
                _btnPushSprayCalibration.Enabled = true;
            }
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

                // Reconfigure centralized API service with updated settings
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
    }
}
