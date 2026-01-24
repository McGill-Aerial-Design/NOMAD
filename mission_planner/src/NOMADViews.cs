// ============================================================
// NOMAD Sidebar View Stubs - Placeholder Implementations
// ============================================================
// These are the individual view implementations for each sidebar section.
// Each view can be expanded with full functionality as needed.
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    // ============================================================
    // Base View Class
    // ============================================================
    
    /// <summary>
    /// Base class for NOMAD views with common styling
    /// </summary>
    public abstract class NOMADViewBase : UserControl
    {
        protected static readonly Color CARD_BG = Color.FromArgb(40, 40, 45);
        protected static readonly Color ACCENT_COLOR = Color.FromArgb(0, 122, 204);
        protected static readonly Color SUCCESS_COLOR = Color.FromArgb(76, 175, 80);
        protected static readonly Color WARNING_COLOR = Color.FromArgb(255, 152, 0);
        protected static readonly Color ERROR_COLOR = Color.FromArgb(244, 67, 54);
        protected static readonly Color TEXT_PRIMARY = Color.White;
        protected static readonly Color TEXT_SECONDARY = Color.FromArgb(180, 180, 180);
        
        protected NOMADViewBase()
        {
            this.BackColor = Color.FromArgb(30, 30, 33);
            this.Dock = DockStyle.Fill;
            this.Padding = new Padding(20);
            this.AutoScroll = true;
        }
        
        protected Panel CreateCard(string title, int width = -1, int height = -1)
        {
            var card = new Panel
            {
                BackColor = CARD_BG,
                Margin = new Padding(5),
                Padding = new Padding(15),
            };
            
            if (width > 0) card.Width = width;
            if (height > 0) card.Height = height;
            
            var titleLabel = new Label
            {
                Text = title.ToUpper(),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(15, 15),
                AutoSize = true,
            };
            card.Controls.Add(titleLabel);
            
            return card;
        }
        
        protected Button CreateButton(string text, Color bgColor, int width = 150, int height = 45)
        {
            var btn = new Button
            {
                Text = text,
                Size = new Size(width, height),
                Margin = new Padding(5),
                FlatStyle = FlatStyle.Flat,
                BackColor = bgColor,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            btn.FlatAppearance.BorderSize = 0;
            return btn;
        }
    }
    
    // ============================================================
    // Task 1 View - Outdoor Reconnaissance
    // ============================================================
    
    public class NOMADTask1View : NOMADViewBase, IUpdatableView
    {
        private readonly DualLinkSender _sender;
        private readonly NOMADConfig _config;
        private Label _lblPosition;
        private Label _lblGpsStatus;
        private Button _btnCapture;
        private TextBox _txtResult;
        
        public NOMADTask1View(DualLinkSender sender, NOMADConfig config)
        {
            _sender = sender;
            _config = config;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            var layout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = true,
            };
            
            // Description
            var descLabel = new Label
            {
                Text = "Task 1: Outdoor Reconnaissance\n\n" +
                       "GPS-based outdoor recon mission. Capture snapshots at waypoints.\n" +
                       "The Jetson processes images and logs coordinates automatically.",
                Font = new Font("Segoe UI", 11),
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                MaximumSize = new Size(600, 0),
                Margin = new Padding(0, 0, 0, 20),
            };
            layout.Controls.Add(descLabel);
            
            // GPS Status Card
            var gpsCard = CreateCard("GPS STATUS");
            gpsCard.Size = new Size(600, 120);
            
            _lblGpsStatus = new Label
            {
                Text = "Fix: Waiting...",
                Font = new Font("Consolas", 11),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 50),
                AutoSize = true,
            };
            gpsCard.Controls.Add(_lblGpsStatus);
            
            _lblPosition = new Label
            {
                Text = "Position: --",
                Font = new Font("Consolas", 11),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 80),
                AutoSize = true,
            };
            gpsCard.Controls.Add(_lblPosition);
            
            layout.Controls.Add(gpsCard);
            
            // Capture Card
            var captureCard = CreateCard("SNAPSHOT CAPTURE");
            captureCard.Size = new Size(600, 180);
            
            _btnCapture = CreateButton("CAPTURE SNAPSHOT", ACCENT_COLOR, 400, 55);
            _btnCapture.Location = new Point(15, 50);
            _btnCapture.Click += BtnCapture_Click;
            captureCard.Controls.Add(_btnCapture);
            
            _txtResult = new TextBox
            {
                Location = new Point(15, 120),
                Size = new Size(560, 45),
                Multiline = true,
                ReadOnly = true,
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = SUCCESS_COLOR,
                Font = new Font("Consolas", 10),
                BorderStyle = BorderStyle.FixedSingle,
                Text = "Ready to capture...",
            };
            captureCard.Controls.Add(_txtResult);
            
            layout.Controls.Add(captureCard);
            
            this.Controls.Add(layout);
        }
        
        private async void BtnCapture_Click(object sender, EventArgs e)
        {
            _btnCapture.Enabled = false;
            _btnCapture.Text = "Capturing...";
            _txtResult.Text = "Sending capture command...";
            _txtResult.ForeColor = WARNING_COLOR;
            
            try
            {
                var result = await _sender.SendTask1Capture();
                if (result.Success)
                {
                    _txtResult.Text = $"[OK] Capture successful: {result.Message}";
                    _txtResult.ForeColor = SUCCESS_COLOR;
                }
                else
                {
                    _txtResult.Text = $"[FAIL] Capture failed: {result.Message}";
                    _txtResult.ForeColor = ERROR_COLOR;
                }
            }
            catch (Exception ex)
            {
                _txtResult.Text = $"[ERROR] {ex.Message}";
                _txtResult.ForeColor = ERROR_COLOR;
            }
            finally
            {
                _btnCapture.Enabled = true;
                _btnCapture.Text = "CAPTURE SNAPSHOT";
            }
        }
        
        public void UpdateData()
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke((MethodInvoker)UpdateData);
                return;
            }
            
            try
            {
                var cs = MainV2.comPort?.MAV?.cs;
                if (cs == null) return;
                
                int gpsFix = (int)cs.gpsstatus;
                string fixText = gpsFix switch
                {
                    3 => "3D Fix",
                    4 => "DGPS",
                    5 => "RTK Float",
                    6 => "RTK Fixed",
                    _ => $"Fix Type {gpsFix}"
                };
                _lblGpsStatus.Text = $"Fix: {fixText} | Satellites: {cs.satcount}";
                _lblGpsStatus.ForeColor = gpsFix >= 3 ? SUCCESS_COLOR : WARNING_COLOR;
                
                _lblPosition.Text = $"Position: {cs.lat:F6}, {cs.lng:F6} | Alt: {cs.alt:F1}m";
            }
            catch { }
        }
    }
    
    // ============================================================
    // Task 2 View - Indoor Fire Extinguishing
    // ============================================================
    
    public class NOMADTask2View : NOMADViewBase, IUpdatableView
    {
        private readonly DualLinkSender _sender;
        private readonly NOMADConfig _config;
        private Label _lblVioStatus;
        private Label _lblTargetCount;
        private Button _btnResetMap;
        private Button _btnResetVio;
        
        public NOMADTask2View(DualLinkSender sender, NOMADConfig config)
        {
            _sender = sender;
            _config = config;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            var layout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = true,
            };
            
            // Description
            var descLabel = new Label
            {
                Text = "Task 2: Indoor Fire Extinguishing\n\n" +
                       "VIO-based indoor navigation. GPS is disabled.\n" +
                       "Use the exclusion map to track extinguished targets.",
                Font = new Font("Segoe UI", 11),
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                MaximumSize = new Size(600, 0),
                Margin = new Padding(0, 0, 0, 20),
            };
            layout.Controls.Add(descLabel);
            
            // VIO Status Card
            var vioCard = CreateCard("VIO STATUS");
            vioCard.Size = new Size(600, 100);
            
            _lblVioStatus = new Label
            {
                Text = "VIO: Inactive",
                Font = new Font("Consolas", 11),
                ForeColor = WARNING_COLOR,
                Location = new Point(15, 50),
                AutoSize = true,
            };
            vioCard.Controls.Add(_lblVioStatus);
            
            _btnResetVio = CreateButton("Reset VIO Origin", SUCCESS_COLOR, 180, 35);
            _btnResetVio.Location = new Point(400, 45);
            _btnResetVio.Click += async (s, e) => await _sender.ResetVioOriginAsync();
            vioCard.Controls.Add(_btnResetVio);
            
            layout.Controls.Add(vioCard);
            
            // Exclusion Map Card
            var mapCard = CreateCard("TARGET EXCLUSION MAP");
            mapCard.Size = new Size(600, 130);
            
            _lblTargetCount = new Label
            {
                Text = "Targets tracked: 0",
                Font = new Font("Segoe UI", 12),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 50),
                AutoSize = true,
            };
            mapCard.Controls.Add(_lblTargetCount);
            
            _btnResetMap = CreateButton("RESET EXCLUSION MAP", ERROR_COLOR, 250, 45);
            _btnResetMap.Location = new Point(15, 80);
            _btnResetMap.Click += async (s, e) =>
            {
                var confirm = MessageBox.Show(
                    "Reset the exclusion map? All tracked targets will be cleared.",
                    "Confirm Reset",
                    MessageBoxButtons.YesNo,
                    MessageBoxIcon.Warning
                );
                if (confirm == DialogResult.Yes)
                {
                    await _sender.SendTask2ResetMap();
                    _lblTargetCount.Text = "Targets tracked: 0";
                }
            };
            mapCard.Controls.Add(_btnResetMap);
            
            layout.Controls.Add(mapCard);
            
            // WASD Control hint
            var wasdLabel = new Label
            {
                Text = "Tip: For manual indoor control, use the dedicated WASD controller in the Quick Panel.",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                Margin = new Padding(0, 20, 0, 0),
            };
            layout.Controls.Add(wasdLabel);
            
            this.Controls.Add(layout);
        }
        
        public void UpdateData()
        {
            // VIO status updates would come from Jetson API
        }
    }
    
    // ============================================================
    // Video View with WASD Controls
    // ============================================================
    
    public class NOMADVideoView : NOMADViewBase, IUpdatableView
    {
        private readonly DualLinkSender _sender;
        private readonly NOMADConfig _config;
        private EmbeddedVideoPlayer _videoPlayer;
        private EnhancedWASDControl _wasdControl;
        private Label _lblStatus;
        
        public NOMADVideoView(DualLinkSender sender, NOMADConfig config)
        {
            _sender = sender;
            _config = config;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            // Main horizontal split: Video (left) + WASD Controls (right)
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
            };
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 60));  // Video
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 40));  // WASD
            
            // Left side: Video with controls
            var videoSection = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 2,
            };
            videoSection.RowStyles.Add(new RowStyle(SizeType.Percent, 85));
            videoSection.RowStyles.Add(new RowStyle(SizeType.Percent, 15));
            
            // Video player panel - left ZED camera
            var videoPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                Margin = new Padding(5),
            };
            
            try
            {
                // RTSP URL for ZED stream - left camera will be cropped
                string rtspUrl = $"rtsp://{_config.EffectiveIP}:8554/zed";
                _videoPlayer = new EmbeddedVideoPlayer("ZED Left Camera", rtspUrl);
                _videoPlayer.Dock = DockStyle.Fill;
                videoPanel.Controls.Add(_videoPlayer);
            }
            catch (Exception ex)
            {
                _lblStatus = new Label
                {
                    Text = $"Video player unavailable: {ex.Message}\n\n" +
                           $"Stream URL: rtsp://{_config.EffectiveIP}:8554/zed\n\n" +
                           "Use VLC or another player to view the stream.",
                    Font = new Font("Segoe UI", 12),
                    ForeColor = TEXT_SECONDARY,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                videoPanel.Controls.Add(_lblStatus);
            }
            
            videoSection.Controls.Add(videoPanel, 0, 0);
            
            // Video controls panel
            var controlsPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                Padding = new Padding(15),
            };
            
            var btnPlay = CreateButton("Play", SUCCESS_COLOR, 100, 40);
            btnPlay.Location = new Point(15, 15);
            btnPlay.Click += (s, e) => _videoPlayer?.StartStream();
            controlsPanel.Controls.Add(btnPlay);
            
            var btnStop = CreateButton("Stop", ERROR_COLOR, 100, 40);
            btnStop.Location = new Point(125, 15);
            btnStop.Click += (s, e) => _videoPlayer?.StopStream();
            controlsPanel.Controls.Add(btnStop);
            
            var lblUrl = new Label
            {
                Text = $"Stream: rtsp://{_config.EffectiveIP}:8554/zed",
                Font = new Font("Consolas", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(240, 25),
                AutoSize = true,
            };
            controlsPanel.Controls.Add(lblUrl);
            
            videoSection.Controls.Add(controlsPanel, 0, 1);
            mainLayout.Controls.Add(videoSection, 0, 0);
            
            // Right side: WASD Controls
            try
            {
                _wasdControl = new EnhancedWASDControl(
                    _config.WasdNudgeSpeed,
                    _config.WasdAltSpeed,
                    15.0f  // Default yaw rate
                );
                _wasdControl.Dock = DockStyle.Fill;
                mainLayout.Controls.Add(_wasdControl, 1, 0);
            }
            catch (Exception ex)
            {
                var errorPanel = new Panel
                {
                    Dock = DockStyle.Fill,
                    BackColor = CARD_BG,
                };
                var errorLabel = new Label
                {
                    Text = $"WASD controls unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 11),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                errorPanel.Controls.Add(errorLabel);
                mainLayout.Controls.Add(errorPanel, 1, 0);
            }
            
            this.Controls.Add(mainLayout);
        }
        
        public void UpdateData() { }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _videoPlayer?.Dispose();
                _wasdControl?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
    
    // ============================================================
    // Terminal View
    // ============================================================
    
    public class NOMADTerminalView : NOMADViewBase
    {
        private readonly NOMADConfig _config;
        private JetsonTerminalControl _terminal;
        
        public NOMADTerminalView(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            try
            {
                _terminal = new JetsonTerminalControl(_config);
                _terminal.Dock = DockStyle.Fill;
                this.Controls.Add(_terminal);
            }
            catch (Exception ex)
            {
                var errorLabel = new Label
                {
                    Text = $"Terminal unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 12),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                this.Controls.Add(errorLabel);
            }
        }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _terminal?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
    
    // ============================================================
    // Health View
    // ============================================================
    
    public class NOMADHealthView : NOMADViewBase, IUpdatableView
    {
        private readonly NOMADConfig _config;
        private EnhancedHealthDashboard _healthDashboard;
        
        public NOMADHealthView(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            try
            {
                _healthDashboard = new EnhancedHealthDashboard(_config);
                _healthDashboard.Dock = DockStyle.Fill;
                this.Controls.Add(_healthDashboard);
            }
            catch (Exception ex)
            {
                var errorLabel = new Label
                {
                    Text = $"Health dashboard unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 12),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                this.Controls.Add(errorLabel);
            }
        }
        
        public void UpdateData()
        {
            _healthDashboard?.RefreshHealth();
        }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _healthDashboard?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
    
    // ============================================================
    // Links View
    // ============================================================
    
    public class NOMADLinksView : NOMADViewBase, IUpdatableView
    {
        private readonly MAVLinkConnectionManager _connectionManager;
        private readonly NOMADConfig _config;
        private LinkHealthPanel _linkPanel;
        
        public NOMADLinksView(MAVLinkConnectionManager connectionManager, NOMADConfig config)
        {
            _connectionManager = connectionManager;
            _config = config;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            if (_connectionManager != null)
            {
                try
                {
                    _linkPanel = new LinkHealthPanel(_connectionManager, _config);
                    _linkPanel.Dock = DockStyle.Fill;
                    this.Controls.Add(_linkPanel);
                    return;
                }
                catch { }
            }
            
            // Fallback if no connection manager
            var layout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
            };
            
            var descLabel = new Label
            {
                Text = "Dual Link Failover Status\n\n" +
                       "Monitor the health of both communication links:\n" +
                       "• LTE/Tailscale: Primary long-range link via 4G\n" +
                       "• RadioMaster: Backup RC link via ELRS",
                Font = new Font("Segoe UI", 11),
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                MaximumSize = new Size(600, 0),
                Margin = new Padding(0, 0, 0, 20),
            };
            layout.Controls.Add(descLabel);
            
            var statusLabel = new Label
            {
                Text = _config.DualLinkEnabled 
                    ? "[OK] Dual link monitoring is enabled" 
                    : "[!] Dual link is disabled in settings",
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = _config.DualLinkEnabled ? SUCCESS_COLOR : WARNING_COLOR,
                AutoSize = true,
            };
            layout.Controls.Add(statusLabel);
            
            this.Controls.Add(layout);
        }
        
        public void UpdateData()
        {
            // Link panel updates itself
        }
    }
    
    // ============================================================
    // Settings View
    // ============================================================
    
    public class NOMADSettingsView : NOMADViewBase
    {
        private readonly NOMADConfig _config;
        
        public NOMADSettingsView(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            var layout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = true,
            };
            
            // Connection Settings
            var connCard = CreateCard("CONNECTION SETTINGS");
            connCard.Size = new Size(600, 180);
            
            AddSettingRow(connCard, "Jetson IP:", _config.JetsonIP, 50, out var txtJetsonIP);
            AddSettingRow(connCard, "Tailscale IP:", _config.TailscaleIP, 80, out var txtTailscaleIP);
            AddSettingRow(connCard, "Port:", _config.JetsonPort.ToString(), 110, out var txtPort);
            
            var chkUseTailscale = new CheckBox
            {
                Text = "Use Tailscale IP (remote access)",
                Checked = _config.UseTailscale,
                Location = new Point(15, 140),
                ForeColor = TEXT_PRIMARY,
                AutoSize = true,
            };
            connCard.Controls.Add(chkUseTailscale);
            
            layout.Controls.Add(connCard);
            
            // Feature Flags
            var featCard = CreateCard("FEATURES");
            featCard.Size = new Size(600, 150);
            
            var chkDualLink = new CheckBox
            {
                Text = "Enable Dual Link Failover",
                Checked = _config.DualLinkEnabled,
                Location = new Point(15, 50),
                ForeColor = TEXT_PRIMARY,
                AutoSize = true,
            };
            featCard.Controls.Add(chkDualLink);
            
            var chkAutoVideo = new CheckBox
            {
                Text = "Auto-start HUD video on connect",
                Checked = _config.AutoStartHudVideo,
                Location = new Point(15, 80),
                ForeColor = TEXT_PRIMARY,
                AutoSize = true,
            };
            featCard.Controls.Add(chkAutoVideo);
            
            var chkDebug = new CheckBox
            {
                Text = "Debug mode (verbose logging)",
                Checked = _config.DebugMode,
                Location = new Point(15, 110),
                ForeColor = TEXT_PRIMARY,
                AutoSize = true,
            };
            featCard.Controls.Add(chkDebug);
            
            layout.Controls.Add(featCard);
            
            // Save button
            var btnSave = CreateButton("Save Settings", ACCENT_COLOR, 200, 50);
            btnSave.Margin = new Padding(5, 20, 5, 5);
            btnSave.Click += (s, e) =>
            {
                try
                {
                    _config.JetsonIP = txtJetsonIP.Text;
                    _config.TailscaleIP = txtTailscaleIP.Text;
                    if (int.TryParse(txtPort.Text, out int port))
                        _config.JetsonPort = port;
                    _config.UseTailscale = chkUseTailscale.Checked;
                    _config.DualLinkEnabled = chkDualLink.Checked;
                    _config.AutoStartHudVideo = chkAutoVideo.Checked;
                    _config.DebugMode = chkDebug.Checked;
                    
                    _config.Save();
                    CustomMessageBox.Show("Settings saved successfully!", "NOMAD");
                }
                catch (Exception ex)
                {
                    CustomMessageBox.Show($"Failed to save settings: {ex.Message}", "Error");
                }
            };
            layout.Controls.Add(btnSave);
            
            this.Controls.Add(layout);
        }
        
        private void AddSettingRow(Panel parent, string label, string value, int yOffset, out TextBox textBox)
        {
            var lbl = new Label
            {
                Text = label,
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, yOffset),
                AutoSize = true,
            };
            parent.Controls.Add(lbl);
            
            textBox = new TextBox
            {
                Text = value,
                Location = new Point(150, yOffset - 3),
                Size = new Size(200, 25),
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = TEXT_PRIMARY,
                BorderStyle = BorderStyle.FixedSingle,
            };
            parent.Controls.Add(textBox);
        }
    }
}
