// ============================================================
// NOMAD Full Page Control - Mission Planner Integration
// ============================================================
// A complete tabbed interface for NOMAD operations in Mission Planner.
// Provides extended controls including:
// - Dashboard overview
// - Task 1 & Task 2 controls
// - Embedded video feeds
// - Jetson terminal access
// - Health monitoring
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Plugin;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Full-page NOMAD control interface with tabbed navigation.
    /// </summary>
    public class NOMADFullPage : UserControl
    {
        // ============================================================
        // Fields
        // ============================================================
        
        private readonly DualLinkSender _sender;
        private NOMADConfig _config;
        
        // Tab Control
        private TabControl _tabControl;
        
        // Tab Pages
        private TabPage _tabDashboard;
        private TabPage _tabTask1;
        private TabPage _tabTask2;
        private TabPage _tabVideo;
        private TabPage _tabTerminal;
        private TabPage _tabHealth;
        private TabPage _tabSettings;
        
        // Dashboard Tab Controls
        private Label _lblConnectionStatus;
        private Label _lblFlightMode;
        private Label _lblGpsStatus;
        private Label _lblBattery;
        private Label _lblPosition;
        private Label _lblVioStatus;
        private Label _lblJetsonStatus;
        private Panel _pnlQuickActions;
        
        // Video Tab Controls
        private EmbeddedVideoPlayer _zedVideoPlayer;
        private TrackBar _trkCameraTilt;
        private Label _lblTiltAngle;
        private Button _btnTiltCenter;
        
        // Terminal Tab Controls
        private JetsonTerminalControl _terminalControl;
        
        // Health Tab Controls
        private EnhancedHealthDashboard _healthDashboard;
        
        // Task Controls (from existing)
        private TelemetryInjector _telemetryInjector;
        private WASDNudgeControl _wasdControl;
        
        // Update Timer
        private System.Windows.Forms.Timer _updateTimer;
        
        // ============================================================
        // Constructor
        // ============================================================
        
        public NOMADFullPage(DualLinkSender sender, NOMADConfig config)
        {
            _sender = sender ?? throw new ArgumentNullException(nameof(sender));
            _config = config ?? throw new ArgumentNullException(nameof(config));
            
            // Initialize helper services
            try
            {
                _telemetryInjector = new TelemetryInjector(null);
                _wasdControl = new WASDNudgeControl(null);
                _wasdControl.NudgeSpeed = 0.5f;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD: Service init error - {ex.Message}");
            }
            
            InitializeUI();
            StartUpdateTimer();
        }
        
        // ============================================================
        // UI Initialization
        // ============================================================
        
        private void InitializeUI()
        {
            this.BackColor = Color.FromArgb(30, 30, 30);
            this.Dock = DockStyle.Fill;
            
            // Create main tab control
            _tabControl = new TabControl
            {
                Dock = DockStyle.Fill,
                Font = new Font("Segoe UI", 10),
                Padding = new Point(12, 6),
            };
            
            // Apply dark theme to tab control
            _tabControl.DrawMode = TabDrawMode.OwnerDrawFixed;
            _tabControl.DrawItem += TabControl_DrawItem;
            
            // Create tabs
            CreateDashboardTab();
            CreateTask1Tab();
            CreateTask2Tab();
            CreateVideoTab();
            CreateTerminalTab();
            CreateHealthTab();
            CreateSettingsTab();
            
            // Add tabs to control
            _tabControl.TabPages.Add(_tabDashboard);
            _tabControl.TabPages.Add(_tabTask1);
            _tabControl.TabPages.Add(_tabTask2);
            _tabControl.TabPages.Add(_tabVideo);
            _tabControl.TabPages.Add(_tabTerminal);
            _tabControl.TabPages.Add(_tabHealth);
            _tabControl.TabPages.Add(_tabSettings);
            
            this.Controls.Add(_tabControl);
        }
        
        private void TabControl_DrawItem(object sender, DrawItemEventArgs e)
        {
            // Dark theme tab rendering
            var tabControl = sender as TabControl;
            var tab = tabControl.TabPages[e.Index];
            var tabRect = tabControl.GetTabRect(e.Index);
            
            bool isSelected = (tabControl.SelectedIndex == e.Index);
            
            using (var brush = new SolidBrush(isSelected ? Color.FromArgb(0, 122, 204) : Color.FromArgb(45, 45, 48)))
            {
                e.Graphics.FillRectangle(brush, tabRect);
            }
            
            using (var brush = new SolidBrush(Color.White))
            {
                var sf = new StringFormat
                {
                    Alignment = StringAlignment.Center,
                    LineAlignment = StringAlignment.Center
                };
                e.Graphics.DrawString(tab.Text, e.Font, brush, tabRect, sf);
            }
        }
        
        // ============================================================
        // Dashboard Tab
        // ============================================================
        
        private void CreateDashboardTab()
        {
            _tabDashboard = new TabPage("Dashboard")
            {
                BackColor = Color.FromArgb(30, 30, 30),
                Padding = new Padding(10),
            };
            
            var panel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 3,
                BackColor = Color.Transparent,
            };
            
            panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 150));
            panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 150));
            panel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            
            // Connection Status Card
            var connCard = CreateDashboardCard("Connection Status", out _lblConnectionStatus);
            panel.Controls.Add(connCard, 0, 0);
            
            // Flight Mode Card
            var modeCard = CreateDashboardCard("Flight Mode", out _lblFlightMode);
            panel.Controls.Add(modeCard, 1, 0);
            
            // GPS Status Card
            var gpsCard = CreateDashboardCard("GPS Status", out _lblGpsStatus);
            panel.Controls.Add(gpsCard, 0, 1);
            
            // VIO Status Card
            var vioCard = CreateDashboardCard("VIO Status", out _lblVioStatus);
            panel.Controls.Add(vioCard, 1, 1);
            
            // Quick Actions Panel
            _pnlQuickActions = CreateQuickActionsPanel();
            panel.Controls.Add(_pnlQuickActions, 0, 2);
            panel.SetColumnSpan(_pnlQuickActions, 2);
            
            _tabDashboard.Controls.Add(panel);
        }
        
        private Panel CreateDashboardCard(string title, out Label valueLabel)
        {
            var card = new Panel
            {
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(5),
                Dock = DockStyle.Fill,
            };
            
            var titleLabel = new Label
            {
                Text = title,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = Color.Gray,
                Location = new Point(10, 10),
                AutoSize = true,
            };
            card.Controls.Add(titleLabel);
            
            valueLabel = new Label
            {
                Text = "--",
                Font = new Font("Segoe UI", 18, FontStyle.Bold),
                ForeColor = Color.White,
                Location = new Point(10, 40),
                AutoSize = true,
            };
            card.Controls.Add(valueLabel);
            
            return card;
        }
        
        private Panel CreateQuickActionsPanel()
        {
            var panel = new Panel
            {
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(5),
                Dock = DockStyle.Fill,
            };
            
            var titleLabel = new Label
            {
                Text = "Quick Actions",
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = Color.FromArgb(0, 122, 204),
                Location = new Point(10, 10),
                AutoSize = true,
            };
            panel.Controls.Add(titleLabel);
            
            var flowPanel = new FlowLayoutPanel
            {
                Location = new Point(10, 40),
                Size = new Size(600, 200),
                FlowDirection = FlowDirection.LeftToRight,
                BackColor = Color.Transparent,
            };
            
            // Quick action buttons
            var btnTask1 = CreateQuickActionButton("[CAP] Task 1 Capture", Color.FromArgb(0, 122, 204));
            btnTask1.Click += async (s, e) => await _sender.SendTask1Capture();
            flowPanel.Controls.Add(btnTask1);
            
            var btnTask2Reset = CreateQuickActionButton("[CLR] Reset Map", Color.FromArgb(200, 50, 50));
            btnTask2Reset.Click += async (s, e) => await _sender.SendTask2ResetMap();
            flowPanel.Controls.Add(btnTask2Reset);
            
            var btnVioReset = CreateQuickActionButton("[VIO] Reset VIO Origin", Color.FromArgb(100, 150, 50));
            btnVioReset.Click += (s, e) => ResetVioOrigin();
            flowPanel.Controls.Add(btnVioReset);
            
            // Removed manual refresh button - auto-refresh is enabled
            
            panel.Controls.Add(flowPanel);
            
            return panel;
        }
        
        private Button CreateQuickActionButton(string text, Color color)
        {
            return new Button
            {
                Text = text,
                Size = new Size(140, 50),
                Margin = new Padding(5),
                FlatStyle = FlatStyle.Flat,
                BackColor = color,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
        }
        
        // ============================================================
        // Task 1 Tab
        // ============================================================
        
        private void CreateTask1Tab()
        {
            _tabTask1 = new TabPage("Task 1: Recon")
            {
                BackColor = Color.FromArgb(30, 30, 30),
                Padding = new Padding(15),
            };
            
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
            };
            
            int yOffset = 10;
            
            // Title
            var title = new Label
            {
                Text = "Task 1: Outdoor Reconnaissance",
                Font = new Font("Segoe UI", 16, FontStyle.Bold),
                ForeColor = Color.FromArgb(0, 150, 200),
                Location = new Point(10, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(title);
            yOffset += 50;
            
            // Description
            var desc = new Label
            {
                Text = "GPS-based outdoor recon mission. Jetson is NOT mounted on drone.\n" +
                       "Uses ELRS telemetry to Mission Planner. RTK corrections via GPS_INJECT_DATA.",
                Font = new Font("Segoe UI", 10),
                ForeColor = Color.LightGray,
                Location = new Point(10, yOffset),
                Size = new Size(500, 60),
            };
            panel.Controls.Add(desc);
            yOffset += 70;
            
            // GPS Status Group
            var gpsGroup = new GroupBox
            {
                Text = "GPS Status",
                ForeColor = Color.FromArgb(0, 150, 200),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(500, 120),
                BackColor = Color.FromArgb(45, 45, 48),
            };
            
            _lblPosition = new Label
            {
                Text = "Position: Waiting for GPS...",
                Font = new Font("Consolas", 10),
                ForeColor = Color.White,
                Location = new Point(15, 30),
                AutoSize = true,
            };
            gpsGroup.Controls.Add(_lblPosition);
            
            var lblRtkStatus = new Label
            {
                Text = "RTK Status: Unknown",
                Font = new Font("Consolas", 10),
                ForeColor = Color.Yellow,
                Location = new Point(15, 55),
                AutoSize = true,
            };
            gpsGroup.Controls.Add(lblRtkStatus);
            
            var lblSats = new Label
            {
                Text = "Satellites: --",
                Font = new Font("Consolas", 10),
                ForeColor = Color.LightGray,
                Location = new Point(15, 80),
                AutoSize = true,
            };
            gpsGroup.Controls.Add(lblSats);
            
            panel.Controls.Add(gpsGroup);
            yOffset += 130;
            
            // Capture Controls
            var captureGroup = new GroupBox
            {
                Text = "Snapshot Capture",
                ForeColor = Color.FromArgb(0, 150, 200),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(500, 150),
                BackColor = Color.FromArgb(45, 45, 48),
            };
            
            var btnCapture = new Button
            {
                Text = "[CAP] CAPTURE SNAPSHOT",
                Location = new Point(15, 30),
                Size = new Size(470, 50),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            btnCapture.Click += async (s, e) =>
            {
                btnCapture.Enabled = false;
                btnCapture.Text = "Capturing...";
                try
                {
                    var result = await _sender.SendTask1Capture();
                    if (result.Success)
                    {
                        CustomMessageBox.Show("Snapshot captured successfully!", "Task 1");
                    }
                    else
                    {
                        CustomMessageBox.Show($"Capture failed: {result.Message}", "Error");
                    }
                }
                finally
                {
                    btnCapture.Enabled = true;
                    btnCapture.Text = "[CAP] CAPTURE SNAPSHOT";
                }
            };
            captureGroup.Controls.Add(btnCapture);
            
            var txtResult = new TextBox
            {
                Location = new Point(15, 90),
                Size = new Size(470, 45),
                Multiline = true,
                ReadOnly = true,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.LimeGreen,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.FixedSingle,
            };
            captureGroup.Controls.Add(txtResult);
            
            panel.Controls.Add(captureGroup);
            
            _tabTask1.Controls.Add(panel);
        }
        
        // ============================================================
        // Task 2 Tab
        // ============================================================
        
        private void CreateTask2Tab()
        {
            _tabTask2 = new TabPage("Task 2: Extinguish")
            {
                BackColor = Color.FromArgb(30, 30, 30),
                Padding = new Padding(15),
            };
            
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
            };
            
            int yOffset = 10;
            
            // Title
            var title = new Label
            {
                Text = "Task 2: Indoor Extinguish",
                Font = new Font("Segoe UI", 16, FontStyle.Bold),
                ForeColor = Color.FromArgb(255, 150, 50),
                Location = new Point(10, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(title);
            yOffset += 50;
            
            // VIO Status Group
            var vioGroup = new GroupBox
            {
                Text = "VIO Status",
                ForeColor = Color.FromArgb(100, 200, 100),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(500, 100),
                BackColor = Color.FromArgb(45, 45, 48),
            };
            
            _lblVioStatus = new Label
            {
                Text = "VIO: Waiting for connection...",
                Font = new Font("Consolas", 10),
                ForeColor = Color.Yellow,
                Location = new Point(15, 30),
                AutoSize = true,
            };
            vioGroup.Controls.Add(_lblVioStatus);
            
            var btnResetVio = new Button
            {
                Text = "Reset VIO Origin",
                Location = new Point(15, 60),
                Size = new Size(150, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(100, 150, 50),
                ForeColor = Color.White,
            };
            btnResetVio.Click += (s, e) => ResetVioOrigin();
            vioGroup.Controls.Add(btnResetVio);
            
            panel.Controls.Add(vioGroup);
            yOffset += 110;
            
            // Exclusion Map Group
            var mapGroup = new GroupBox
            {
                Text = "Target Exclusion Map",
                ForeColor = Color.FromArgb(255, 150, 50),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(500, 100),
                BackColor = Color.FromArgb(45, 45, 48),
            };
            
            var lblTargets = new Label
            {
                Text = "Targets: 0",
                Font = new Font("Segoe UI", 12),
                ForeColor = Color.White,
                Location = new Point(15, 30),
                AutoSize = true,
            };
            mapGroup.Controls.Add(lblTargets);
            
            var btnReset = new Button
            {
                Text = "RESET EXCLUSION MAP",
                Location = new Point(15, 55),
                Size = new Size(200, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(200, 50, 50),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            btnReset.Click += async (s, e) =>
            {
                var confirm = MessageBox.Show(
                    "Reset the exclusion map? All stored targets will be cleared.",
                    "Confirm Reset",
                    MessageBoxButtons.YesNo,
                    MessageBoxIcon.Warning
                );
                if (confirm == DialogResult.Yes)
                {
                    await _sender.SendTask2ResetMap();
                    lblTargets.Text = "Targets: 0";
                }
            };
            mapGroup.Controls.Add(btnReset);
            
            panel.Controls.Add(mapGroup);
            yOffset += 110;
            
            // WASD Nudge Group
            var nudgeGroup = new GroupBox
            {
                Text = "Indoor Nudge Control (WASD)",
                ForeColor = Color.FromArgb(100, 200, 100),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(500, 180),
                BackColor = Color.FromArgb(45, 45, 48),
            };
            
            var chkEnableWASD = new CheckBox
            {
                Text = "Enable WASD Indoor Control",
                Location = new Point(15, 30),
                Size = new Size(250, 25),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10),
            };
            chkEnableWASD.CheckedChanged += (s, e) =>
            {
                if (_wasdControl != null)
                {
                    _wasdControl.Enabled = chkEnableWASD.Checked;
                    if (chkEnableWASD.Checked)
                    {
                        CustomMessageBox.Show(
                            "WASD Control Enabled\n\n" +
                            "W/S = Forward/Back\n" +
                            "A/D = Left/Right\n" +
                            "Q/E = Up/Down\n\n" +
                            "CAUTION: Use only in Guided mode!",
                            "WASD Control"
                        );
                    }
                }
            };
            nudgeGroup.Controls.Add(chkEnableWASD);
            
            var lblNudgeHelp = new Label
            {
                Text = "W=Forward  S=Back  A=Left  D=Right  Q=Up  E=Down",
                Location = new Point(15, 60),
                ForeColor = Color.Gray,
                Font = new Font("Consolas", 9),
                AutoSize = true,
            };
            nudgeGroup.Controls.Add(lblNudgeHelp);
            
            var lblSpeed = new Label
            {
                Text = "Speed (m/s):",
                Location = new Point(15, 95),
                ForeColor = Color.LightGray,
                AutoSize = true,
            };
            nudgeGroup.Controls.Add(lblSpeed);
            
            var numSpeed = new NumericUpDown
            {
                Location = new Point(100, 92),
                Size = new Size(80, 25),
                DecimalPlaces = 2,
                Minimum = 0.1M,
                Maximum = 2.0M,
                Value = 0.5M,
                Increment = 0.1M,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
            };
            numSpeed.ValueChanged += (s, e) =>
            {
                if (_wasdControl != null)
                    _wasdControl.NudgeSpeed = (float)numSpeed.Value;
            };
            nudgeGroup.Controls.Add(numSpeed);
            
            // Visual WASD indicator
            var wasdPanel = CreateWASDVisual();
            wasdPanel.Location = new Point(250, 25);
            nudgeGroup.Controls.Add(wasdPanel);
            
            panel.Controls.Add(nudgeGroup);
            
            _tabTask2.Controls.Add(panel);
        }
        
        private Panel CreateWASDVisual()
        {
            var panel = new Panel
            {
                Size = new Size(150, 130),
                BackColor = Color.Transparent,
            };
            
            var keys = new[]
            {
                (text: "W", x: 55, y: 0),
                (text: "A", x: 0, y: 45),
                (text: "S", x: 55, y: 45),
                (text: "D", x: 110, y: 45),
                (text: "Q↑", x: 0, y: 90),
                (text: "E↓", x: 110, y: 90),
            };
            
            foreach (var key in keys)
            {
                var btn = new Button
                {
                    Text = key.text,
                    Location = new Point(key.x, key.y),
                    Size = new Size(40, 35),
                    FlatStyle = FlatStyle.Flat,
                    BackColor = Color.FromArgb(60, 60, 65),
                    ForeColor = Color.White,
                    Font = new Font("Segoe UI", 9, FontStyle.Bold),
                };
                btn.FlatAppearance.BorderColor = Color.FromArgb(80, 80, 85);
                panel.Controls.Add(btn);
            }
            
            return panel;
        }
        
        // ============================================================
        // Video Tab
        // ============================================================
        
        private void CreateVideoTab()
        {
            _tabVideo = new TabPage("ZED Camera")
            {
                BackColor = Color.FromArgb(30, 30, 30),
                Padding = new Padding(10),
            };
            
            var mainPanel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 2,
                BackColor = Color.Transparent,
            };
            
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 70));
            
            // ZED Camera Video Player (full width)
            _zedVideoPlayer = new EmbeddedVideoPlayer(
                "ZED Camera (Navigation/VIO)",
                _config.VideoUrl
            );
            _zedVideoPlayer.Dock = DockStyle.Fill;
            mainPanel.Controls.Add(_zedVideoPlayer, 0, 0);
            
            // Camera Tilt Control Panel
            var tiltPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                Padding = new Padding(10),
            };
            
            var lblTiltTitle = new Label
            {
                Text = "Camera Tilt (Servo Control)",
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, 5),
                AutoSize = true,
            };
            tiltPanel.Controls.Add(lblTiltTitle);
            
            // Tilt slider (maps to servo PWM)
            _trkCameraTilt = new TrackBar
            {
                Minimum = _config.ZedServoMin,
                Maximum = _config.ZedServoMax,
                Value = _config.ZedServoCenter,
                TickFrequency = 100,
                LargeChange = 100,
                SmallChange = 10,
                Location = new Point(10, 30),
                Size = new Size(400, 30),
            };
            _trkCameraTilt.ValueChanged += TrkCameraTilt_ValueChanged;
            tiltPanel.Controls.Add(_trkCameraTilt);
            
            _lblTiltAngle = new Label
            {
                Text = "PWM: 1500 (Level)",
                ForeColor = Color.LimeGreen,
                Font = new Font("Segoe UI", 9),
                Location = new Point(420, 35),
                AutoSize = true,
            };
            tiltPanel.Controls.Add(_lblTiltAngle);
            
            _btnTiltCenter = new Button
            {
                Text = "Center",
                Location = new Point(550, 28),
                Size = new Size(70, 28),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
            };
            _btnTiltCenter.Click += (s, e) => {
                _trkCameraTilt.Value = _config.ZedServoCenter;
            };
            tiltPanel.Controls.Add(_btnTiltCenter);
            
            var lblDown = new Label
            {
                Text = "▼ Down",
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                Location = new Point(10, 55),
                AutoSize = true,
            };
            tiltPanel.Controls.Add(lblDown);
            
            var lblUp = new Label
            {
                Text = "Up ▲",
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                Location = new Point(375, 55),
                AutoSize = true,
            };
            tiltPanel.Controls.Add(lblUp);
            
            mainPanel.Controls.Add(tiltPanel, 0, 1);
            _tabVideo.Controls.Add(mainPanel);
        }
        
        private void TrkCameraTilt_ValueChanged(object sender, EventArgs e)
        {
            var pwm = _trkCameraTilt.Value;
            var center = _config.ZedServoCenter;
            var range = _config.ZedServoMax - _config.ZedServoMin;
            var deviation = pwm - center;
            var angle = (int)((deviation / (range / 2.0)) * 45); // Approx -45 to +45 degrees
            
            string direction = angle == 0 ? "Level" : (angle > 0 ? "Up" : "Down");
            _lblTiltAngle.Text = $"PWM: {pwm} ({direction})";
            
            // Send servo command to flight controller
            if (_config.ZedServoChannel > 0)
            {
                SendCameraTiltCommand(pwm);
            }
        }
        
        private async void SendCameraTiltCommand(int pwm)
        {
            try
            {
                // Use MAVLink DO_SET_SERVO command via Mission Planner
                if (MainV2.comPort?.BaseStream?.IsOpen == true)
                {
                    MainV2.comPort.doCommand(
                        (byte)MainV2.comPort.sysidcurrent,
                        (byte)MainV2.comPort.compidcurrent,
                        MAVLink.MAV_CMD.DO_SET_SERVO,
                        _config.ZedServoChannel,  // Servo channel
                        pwm,                       // PWM value
                        0, 0, 0, 0, 0);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD: Camera tilt error - {ex.Message}");
            }
        }
        
        // ============================================================
        // Terminal Tab
        // ============================================================
        
        private void CreateTerminalTab()
        {
            _tabTerminal = new TabPage("Jetson Terminal")
            {
                BackColor = Color.FromArgb(30, 30, 30),
                Padding = new Padding(10),
            };
            
            _terminalControl = new JetsonTerminalControl(_config);
            _terminalControl.Dock = DockStyle.Fill;
            
            _tabTerminal.Controls.Add(_terminalControl);
        }
        
        // ============================================================
        // Health Tab
        // ============================================================
        
        private void CreateHealthTab()
        {
            _tabHealth = new TabPage("System Health")
            {
                BackColor = Color.FromArgb(30, 30, 30),
                Padding = new Padding(10),
            };
            
            _healthDashboard = new EnhancedHealthDashboard(_config);
            _healthDashboard.Dock = DockStyle.Fill;
            
            _tabHealth.Controls.Add(_healthDashboard);
        }
        
        // ============================================================
        // Settings Tab
        // ============================================================
        
        private void CreateSettingsTab()
        {
            _tabSettings = new TabPage("Settings")
            {
                BackColor = Color.FromArgb(30, 30, 30),
                Padding = new Padding(15),
            };
            
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
            };
            
            int yOffset = 10;
            
            // Jetson Connection Group
            var connGroup = new GroupBox
            {
                Text = "Jetson Connection",
                ForeColor = Color.FromArgb(0, 122, 204),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(400, 120),
                BackColor = Color.FromArgb(45, 45, 48),
            };
            
            var lblIp = new Label { Text = "IP Address:", Location = new Point(15, 30), ForeColor = Color.LightGray, AutoSize = true };
            connGroup.Controls.Add(lblIp);
            
            var txtIp = new TextBox
            {
                Text = _config.JetsonIP,
                Location = new Point(100, 27),
                Size = new Size(150, 25),
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
            };
            connGroup.Controls.Add(txtIp);
            
            var lblPort = new Label { Text = "Port:", Location = new Point(15, 60), ForeColor = Color.LightGray, AutoSize = true };
            connGroup.Controls.Add(lblPort);
            
            var txtPort = new TextBox
            {
                Text = _config.JetsonPort.ToString(),
                Location = new Point(100, 57),
                Size = new Size(80, 25),
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
            };
            connGroup.Controls.Add(txtPort);
            
            var btnTest = new Button
            {
                Text = "Test Connection",
                Location = new Point(260, 27),
                Size = new Size(120, 55),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
            };
            btnTest.Click += (s, e) => TestConnection(txtIp.Text, int.Parse(txtPort.Text));
            connGroup.Controls.Add(btnTest);
            
            panel.Controls.Add(connGroup);
            yOffset += 130;
            
            // Video Stream Group
            var videoGroup = new GroupBox
            {
                Text = "ZED Camera Settings",
                ForeColor = Color.FromArgb(200, 100, 200),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(400, 130),
                BackColor = Color.FromArgb(45, 45, 48),
            };
            
            var lblZedUrl = new Label { Text = "RTSP URL:", Location = new Point(15, 30), ForeColor = Color.LightGray, AutoSize = true };
            videoGroup.Controls.Add(lblZedUrl);
            
            var txtZedUrl = new TextBox
            {
                Text = _config.VideoUrl,
                Location = new Point(80, 27),
                Size = new Size(300, 25),
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
            };
            videoGroup.Controls.Add(txtZedUrl);
            
            var lblServoChannel = new Label { Text = "Tilt Servo:", Location = new Point(15, 60), ForeColor = Color.LightGray, AutoSize = true };
            videoGroup.Controls.Add(lblServoChannel);
            
            var txtServoChannel = new TextBox
            {
                Text = _config.ZedServoChannel.ToString(),
                Location = new Point(80, 57),
                Size = new Size(60, 25),
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
            };
            videoGroup.Controls.Add(txtServoChannel);
            
            var lblServoHelp = new Label
            {
                Text = "Channel (0=off, 9-14=AUX1-6)",
                Location = new Point(150, 60),
                ForeColor = Color.Gray,
                AutoSize = true,
                Font = new Font("Segoe UI", 8),
            };
            videoGroup.Controls.Add(lblServoHelp);
            
            var lblPwmRange = new Label { Text = "PWM Range:", Location = new Point(15, 90), ForeColor = Color.LightGray, AutoSize = true };
            videoGroup.Controls.Add(lblPwmRange);
            
            var txtPwmMin = new TextBox
            {
                Text = _config.ZedServoMin.ToString(),
                Location = new Point(90, 87),
                Size = new Size(50, 25),
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
            };
            videoGroup.Controls.Add(txtPwmMin);
            
            var lblTo = new Label { Text = "to", Location = new Point(145, 90), ForeColor = Color.Gray, AutoSize = true };
            videoGroup.Controls.Add(lblTo);
            
            var txtPwmMax = new TextBox
            {
                Text = _config.ZedServoMax.ToString(),
                Location = new Point(165, 87),
                Size = new Size(50, 25),
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
            };
            videoGroup.Controls.Add(txtPwmMax);
            
            panel.Controls.Add(videoGroup);
            yOffset += 140;
            
            // Save Button
            var btnSave = new Button
            {
                Text = "[SAVE] Save Settings",
                Location = new Point(10, yOffset),
                Size = new Size(150, 40),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(50, 150, 50),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
            };
            btnSave.Click += (s, e) =>
            {
                _config.JetsonIP = txtIp.Text;
                _config.JetsonPort = int.Parse(txtPort.Text);
                _config.VideoUrl = txtZedUrl.Text;
                if (int.TryParse(txtServoChannel.Text, out int servo)) _config.ZedServoChannel = servo;
                if (int.TryParse(txtPwmMin.Text, out int pwmMin)) _config.ZedServoMin = pwmMin;
                if (int.TryParse(txtPwmMax.Text, out int pwmMax)) _config.ZedServoMax = pwmMax;
                _config.Save();
                CustomMessageBox.Show("Settings saved!", "NOMAD");
            };
            panel.Controls.Add(btnSave);
            
            _tabSettings.Controls.Add(panel);
        }
        
        // ============================================================
        // Update Methods
        // ============================================================
        
        private void StartUpdateTimer()
        {
            _updateTimer = new System.Windows.Forms.Timer
            {
                Interval = 500, // 2Hz update
            };
            _updateTimer.Tick += UpdateTimer_Tick;
            _updateTimer.Start();
        }
        
        private void UpdateTimer_Tick(object sender, EventArgs e)
        {
            try
            {
                UpdateDashboard();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Update error: {ex.Message}");
            }
        }
        
        private void UpdateDashboard()
        {
            if (MainV2.comPort == null) return;
            
            var connected = MainV2.comPort.BaseStream?.IsOpen ?? false;
            var cs = MainV2.comPort.MAV?.cs;
            
            if (_lblConnectionStatus != null)
            {
                _lblConnectionStatus.Text = connected ? "CONNECTED" : "DISCONNECTED";
                _lblConnectionStatus.ForeColor = connected ? Color.LimeGreen : Color.Red;
            }
            
            if (_lblFlightMode != null && cs != null)
            {
                _lblFlightMode.Text = cs.mode ?? "UNKNOWN";
            }
            
            if (_lblGpsStatus != null && cs != null)
            {
                var gpsText = cs.gpsstatus >= 3 ? "3D FIX" : "NO FIX";
                _lblGpsStatus.Text = $"{gpsText} ({cs.satcount} sats)";
                _lblGpsStatus.ForeColor = cs.gpsstatus >= 3 ? Color.LimeGreen : Color.Yellow;
            }
            
            if (_lblPosition != null && cs != null)
            {
                _lblPosition.Text = $"Position: {cs.lat:F6}, {cs.lng:F6} @ {cs.alt:F1}m";
            }
        }
        
        // ============================================================
        // Action Methods
        // ============================================================
        
        private async void ResetVioOrigin()
        {
            try
            {
                var client = new System.Net.Http.HttpClient
                {
                    Timeout = TimeSpan.FromSeconds(5)
                };
                var response = await client.PostAsync(
                    $"{_config.EffectiveBaseUrl}/api/vio/reset_origin",
                    new System.Net.Http.StringContent("{}", System.Text.Encoding.UTF8, "application/json")
                );
                
                if (response.IsSuccessStatusCode)
                {
                    CustomMessageBox.Show("VIO origin reset successfully!", "NOMAD");
                }
                else
                {
                    CustomMessageBox.Show($"VIO reset failed: {response.StatusCode}", "Error");
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"VIO reset error: {ex.Message}", "Error");
            }
        }
        
        private void RefreshAllStatus()
        {
            UpdateDashboard();
            _healthDashboard?.RefreshHealth();
        }
        
        private async void TestConnection(string ip, int port)
        {
            try
            {
                var client = new System.Net.Http.HttpClient
                {
                    Timeout = TimeSpan.FromSeconds(3)
                };
                var response = await client.GetAsync($"http://{ip}:{port}/health");
                
                if (response.IsSuccessStatusCode)
                {
                    CustomMessageBox.Show("Connection successful!", "NOMAD");
                }
                else
                {
                    CustomMessageBox.Show($"Connection failed: {response.StatusCode}", "Error");
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Connection failed: {ex.Message}", "Error");
            }
        }
        
        // ============================================================
        // Public Methods
        // ============================================================
        
        public void UpdateConfig(NOMADConfig config)
        {
            _config = config;
            _zedVideoPlayer?.UpdateStreamUrl(_config.VideoUrl);
            _terminalControl?.UpdateConfig(_config);
            _healthDashboard?.UpdateConfig(_config);
            
            // Update tilt servo range if changed
            if (_trkCameraTilt != null)
            {
                _trkCameraTilt.Minimum = _config.ZedServoMin;
                _trkCameraTilt.Maximum = _config.ZedServoMax;
                _trkCameraTilt.Value = Math.Max(_config.ZedServoMin, Math.Min(_config.ZedServoMax, _trkCameraTilt.Value));
            }
        }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _updateTimer?.Stop();
                _updateTimer?.Dispose();
                _zedVideoPlayer?.Dispose();
                _terminalControl?.Dispose();
                _healthDashboard?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
