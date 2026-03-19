// ============================================================
// NOMAD Dashboard View - Main Overview Panel
// ============================================================
// A simplified, information-dense dashboard showing all key data at a glance.
// Features:
// - Connection status with visual indicators
// - Flight mode and telemetry summary  
// - Quick action buttons
// - Mini video preview
// - System health summary
// - Link status indicators
// ============================================================

using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Main dashboard view showing all critical information at a glance
    /// </summary>
    public class NOMADDashboardView : UserControl, IUpdatableView
    {
        // ============================================================
        // Constants
        // ============================================================
        
        private static readonly Color CARD_BG = Color.FromArgb(40, 40, 45);
        private static readonly Color CARD_BORDER = Color.FromArgb(60, 60, 65);
        private static readonly Color ACCENT_COLOR = Color.FromArgb(0, 122, 204);
        private static readonly Color SUCCESS_COLOR = Color.FromArgb(76, 175, 80);
        private static readonly Color WARNING_COLOR = Color.FromArgb(255, 152, 0);
        private static readonly Color ERROR_COLOR = Color.FromArgb(244, 67, 54);
        private static readonly Color TEXT_PRIMARY = Color.White;
        private static readonly Color TEXT_SECONDARY = Color.FromArgb(180, 180, 180);
        
        // ============================================================
        // Fields
        // ============================================================
        
        private readonly DualLinkSender _sender;
        private readonly MAVLinkConnectionManager _connectionManager;
        private readonly JetsonConnectionManager _jetsonConnectionManager;
        private NOMADConfig _config;
        private System.Threading.Timer _healthPollTimer;
        
        // Status cards
        private Panel _connectionCard;
        private Panel _flightModeCard;
        private Panel _gpsCard;
        private Panel _batteryCard;
        private Panel _vioCard;
        private Panel _jetsonCard;
        
        // Status labels
        private Label _lblConnectionStatus;
        private Label _lblConnectionValue;
        private Label _lblFlightMode;
        private Label _lblGpsStatus;
        private Label _lblGpsFix;
        private Label _lblBattery;
        private Label _lblBatteryVolts;
        private Label _lblVioStatus;
        private Label _lblVioConfidence;
        private Label _lblJetsonStatus;
        private Label _lblJetsonTemp;
        
        // Quick actions
        private Button _btnCapture;
        private Button _btnResetMap;
        private Button _btnResetVio;
        private Button _btnEmergency;
        
        // Link indicators
        private Panel _lteIndicator;
        private Panel _radioIndicator;
        private Label _lblLteStatus;
        private Label _lblRadioStatus;
        private Label _lblActiveLink;
        
        // Mini video panel
        private Panel _videoPreviewPanel;
        private Panel _videoPlaceholder;
        private Label _lblVideoStatus;
        private EmbeddedVideoPlayer _videoPlayer;
        private bool _jetsonOnline;
        private bool _videoInitialized;
        
        // Health summary labels (for real-time Jetson health updates)
        private Label _lblHealthCpu;
        private Label _lblHealthGpu;
        private Label _lblHealthMem;
        private Label _lblHealthDisk;
        private Label _lblHealthTemp;
        
        // Notification system
        private NotificationService _notificationService;
        private NotificationPanel _notificationPanel;
        
        // ============================================================
        // Constructor
        // ============================================================
        
        /// <summary>
        /// Gets the notification service for external components to add notifications.
        /// </summary>
        public NotificationService NotificationService => _notificationService;
        
        /// <summary>
        /// Sets the boundary monitor for boundary violation notifications.
        /// </summary>
        public void SetBoundaryMonitor(BoundaryMonitor monitor)
        {
            _notificationService?.SetBoundaryMonitor(monitor);
        }
        
        public NOMADDashboardView(DualLinkSender sender, NOMADConfig config, MAVLinkConnectionManager connectionManager = null, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _sender = sender;
            _config = config;
            _connectionManager = connectionManager;
            _jetsonConnectionManager = jetsonConnectionManager;
            
            // Initialize notification service
            _notificationService = new NotificationService(null, sender);
            
            InitializeUI();
            
            // Start health polling to keep Jetson status updated
            StartHealthPolling();
            
            // Start notification monitoring after UI is initialized
            _notificationService.StartMonitoring();
        }
        
        /// <summary>
        /// Starts periodic health polling to keep Jetson connection status updated.
        /// </summary>
        private void StartHealthPolling()
        {
            _healthPollTimer = new System.Threading.Timer(
                async _ => await PollJetsonHealth(),
                null,
                TimeSpan.FromMilliseconds(500),  // Initial delay
                TimeSpan.FromMilliseconds(_config.HealthPollInterval)  // Honor config polling interval
            );
        }
        
        /// <summary>
        /// Polls Jetson health status to update IsJetsonConnected.
        /// </summary>
        private async Task PollJetsonHealth()
        {
            try
            {
                if (_sender != null)
                {
                    await _sender.GetHealthAsync();
                }
            }
            catch
            {
                // Ignore polling errors
            }
        }
        
        // ============================================================
        // UI Initialization
        // ============================================================
        
        private void InitializeUI()
        {
            this.BackColor = Color.FromArgb(30, 30, 33);
            this.Dock = DockStyle.Fill;
            this.Padding = new Padding(20);
            this.AutoScroll = true;
            
            // Main layout using TableLayoutPanel for responsive design
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                ColumnCount = 3,
                RowCount = 4,
                BackColor = Color.Transparent,
                CellBorderStyle = TableLayoutPanelCellBorderStyle.None,
                Padding = new Padding(0),
                Margin = new Padding(0),
            };
            
            // Column widths (responsive)
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33f));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33f));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33f));
            
            // Row heights - adjusted for notification panel
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 110)); // Status cards row 1
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 110)); // Status cards row 2
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 200)); // Quick actions + Notifications + Link status
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 220)); // Video preview + Health summary
            
            // Row 1: Connection, Flight Mode, GPS Status
            _connectionCard = CreateStatusCard("Connection", "DISCONNECTED", out _lblConnectionStatus, out _lblConnectionValue, ERROR_COLOR);
            mainLayout.Controls.Add(_connectionCard, 0, 0);
            
            _flightModeCard = CreateStatusCard("Flight Mode", "UNKNOWN", out var lblModeTitle, out _lblFlightMode, TEXT_SECONDARY);
            mainLayout.Controls.Add(_flightModeCard, 1, 0);
            
            _gpsCard = CreateStatusCard("GPS Status", "No Fix", out _lblGpsStatus, out _lblGpsFix, WARNING_COLOR);
            mainLayout.Controls.Add(_gpsCard, 2, 0);
            
            // Row 2: Battery, VIO Status, Jetson Status
            _batteryCard = CreateStatusCard("Battery", "--.- V", out var lblBattTitle, out _lblBattery, TEXT_SECONDARY);
            _lblBatteryVolts = _lblBattery;
            mainLayout.Controls.Add(_batteryCard, 0, 1);
            
            _vioCard = CreateStatusCard("VIO Status", "Inactive", out _lblVioStatus, out _lblVioConfidence, TEXT_SECONDARY);
            mainLayout.Controls.Add(_vioCard, 1, 1);
            
            _jetsonCard = CreateStatusCard("Jetson", "Offline", out _lblJetsonStatus, out _lblJetsonTemp, ERROR_COLOR);
            mainLayout.Controls.Add(_jetsonCard, 2, 1);
            
            // Row 3: Quick Actions + Notifications + Link Status
            var quickActionsPanel = CreateQuickActionsPanel();
            mainLayout.Controls.Add(quickActionsPanel, 0, 2);
            
            // Notification panel next to Quick Actions
            _notificationPanel = new NotificationPanel(_notificationService);
            _notificationPanel.Margin = new Padding(5);
            mainLayout.Controls.Add(_notificationPanel, 1, 2);
            
            var linkStatusPanel = CreateLinkStatusPanel();
            mainLayout.Controls.Add(linkStatusPanel, 2, 2);
            
            // Row 4: Video Preview + Health Summary
            _videoPreviewPanel = CreateVideoPreviewPanel();
            mainLayout.Controls.Add(_videoPreviewPanel, 0, 3);
            mainLayout.SetColumnSpan(_videoPreviewPanel, 2);
            
            var healthSummaryPanel = CreateHealthSummaryPanel();
            mainLayout.Controls.Add(healthSummaryPanel, 2, 3);
            
            this.Controls.Add(mainLayout);
        }
        
        private Panel CreateStatusCard(string title, string initialValue, out Label titleLabel, out Label valueLabel, Color statusColor)
        {
            var card = new Panel
            {
                BackColor = CARD_BG,
                Margin = new Padding(5),
                Dock = DockStyle.Fill,
                Padding = new Padding(10),
            };
            
            // Add rounded corners effect
            card.Paint += (s, e) =>
            {
                using (var pen = new Pen(CARD_BORDER, 1))
                {
                    e.Graphics.DrawRectangle(pen, 0, 0, card.Width - 1, card.Height - 1);
                }
            };
            
            // Value label - added FIRST so it fills remaining space
            valueLabel = new Label
            {
                Text = initialValue,
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = statusColor,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft,
            };
            card.Controls.Add(valueLabel);
            
            // Title label at top - added SECOND so it docks at top
            titleLabel = new Label
            {
                Text = title.ToUpper(),
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                ForeColor = TEXT_SECONDARY,
                Dock = DockStyle.Top,
                Height = 20,
                TextAlign = ContentAlignment.BottomLeft,
            };
            card.Controls.Add(titleLabel);
            
            // Status indicator dot
            var indicator = new Panel
            {
                Size = new Size(12, 12),
                Location = new Point(card.Width - 27, 15),
                BackColor = statusColor,
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
            };
            indicator.Paint += (s, e) =>
            {
                e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
                using (var brush = new SolidBrush(indicator.BackColor))
                {
                    e.Graphics.FillEllipse(brush, 0, 0, 11, 11);
                }
            };
            card.Controls.Add(indicator);
            
            return card;
        }
        
        private Panel CreateQuickActionsPanel()
        {
            var panel = new Panel
            {
                BackColor = CARD_BG,
                Margin = new Padding(5),
                Dock = DockStyle.Fill,
                Padding = new Padding(15),
            };
            
            var titleLabel = new Label
            {
                Text = "QUICK ACTIONS",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(15, 15),
                AutoSize = true,
            };
            panel.Controls.Add(titleLabel);
            
            // Buttons flow panel - compact layout for single column
            var buttonsPanel = new FlowLayoutPanel
            {
                Location = new Point(15, 45),
                Size = new Size(200, 140),
                FlowDirection = FlowDirection.TopDown,
                WrapContents = true,
                BackColor = Color.Transparent,
            };
            
            _btnCapture = CreateActionButton("Capture", ACCENT_COLOR, 85, 50);
            _btnCapture.Click += async (s, e) =>
            {
                _btnCapture.Enabled = false;
                _btnCapture.Text = "Capturing...";
                try
                {
                    var result = await _sender.SendTask1Capture();
                    if (!result.Success)
                    {
                        CustomMessageBox.Show($"Capture failed: {result.Message}", "Error");
                    }
                }
                finally
                {
                    _btnCapture.Enabled = true;
                    _btnCapture.Text = "Capture";
                }
            };
            buttonsPanel.Controls.Add(_btnCapture);
            
            _btnResetMap = CreateActionButton("Reset Map", Color.FromArgb(200, 80, 80), 85, 50);
            _btnResetMap.Click += async (s, e) =>
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
                }
            };
            buttonsPanel.Controls.Add(_btnResetMap);
            
            _btnResetVio = CreateActionButton("Reset VIO", SUCCESS_COLOR, 85, 50);
            _btnResetVio.Click += async (s, e) =>
            {
                await _sender.ResetVioOriginAsync();
            };
            buttonsPanel.Controls.Add(_btnResetVio);
            
            _btnEmergency = CreateActionButton("EMRG", ERROR_COLOR, 85, 50);
            _btnEmergency.Font = new Font("Segoe UI", 9, FontStyle.Bold);
            _btnEmergency.Click += (s, e) =>
            {
                var confirm = MessageBox.Show(
                    "Trigger emergency stop? This will attempt to disarm the drone.",
                    "EMERGENCY STOP",
                    MessageBoxButtons.YesNo,
                    MessageBoxIcon.Warning
                );
                if (confirm == DialogResult.Yes)
                {
                    try
                    {
                        // Attempt to disarm via MAVLink
                        MainV2.comPort.doARMAsync(MainV2.comPort.MAV.sysid, MainV2.comPort.MAV.compid, false);
                    }
                    catch (Exception ex)
                    {
                        CustomMessageBox.Show($"Emergency action failed: {ex.Message}", "Error");
                    }
                }
            };
            buttonsPanel.Controls.Add(_btnEmergency);
            
            panel.Controls.Add(buttonsPanel);
            
            return panel;
        }
        
        private Button CreateActionButton(string text, Color bgColor, int width, int height)
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
        
        private Panel CreateLinkStatusPanel()
        {
            var panel = new Panel
            {
                BackColor = CARD_BG,
                Margin = new Padding(5),
                Dock = DockStyle.Fill,
                Padding = new Padding(15),
            };
            
            var titleLabel = new Label
            {
                Text = "LINK STATUS",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(15, 15),
                AutoSize = true,
            };
            panel.Controls.Add(titleLabel);
            
            // LTE Status
            _lteIndicator = new Panel
            {
                Size = new Size(16, 16),
                Location = new Point(15, 50),
                BackColor = TEXT_SECONDARY,
            };
            _lteIndicator.Paint += PaintCircle;
            panel.Controls.Add(_lteIndicator);
            
            _lblLteStatus = new Label
            {
                Text = "LTE/Tailscale: Unknown",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(40, 48),
                AutoSize = true,
            };
            panel.Controls.Add(_lblLteStatus);
            
            // Radio Status
            _radioIndicator = new Panel
            {
                Size = new Size(16, 16),
                Location = new Point(15, 80),
                BackColor = TEXT_SECONDARY,
            };
            _radioIndicator.Paint += PaintCircle;
            panel.Controls.Add(_radioIndicator);
            
            _lblRadioStatus = new Label
            {
                Text = "RadioMaster: Unknown",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(40, 78),
                AutoSize = true,
            };
            panel.Controls.Add(_lblRadioStatus);
            
            // Active link label
            _lblActiveLink = new Label
            {
                Text = "Active: --",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = SUCCESS_COLOR,
                Location = new Point(15, 115),
                AutoSize = true,
            };
            panel.Controls.Add(_lblActiveLink);
            
            return panel;
        }
        
        private void PaintCircle(object sender, PaintEventArgs e)
        {
            var panel = sender as Panel;
            e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
            using (var brush = new SolidBrush(panel.BackColor))
            {
                e.Graphics.FillEllipse(brush, 0, 0, 15, 15);
            }
        }
        
        private Panel CreateVideoPreviewPanel()
        {
            // Panel that fills entirely with video - no title, no padding, just video
            var panel = new Panel
            {
                BackColor = Color.FromArgb(15, 15, 18),
                Margin = new Padding(5),
                Dock = DockStyle.Fill,
                Padding = new Padding(0),
            };
            
            // Video display area fills the entire panel
            _videoPlaceholder = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(15, 15, 18),
            };
            
            // Initially show "Waiting for Jetson" message
            // Video will only be loaded when Jetson comes online
            _lblVideoStatus = new Label
            {
                Text = "Waiting for Jetson...",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_SECONDARY,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleCenter,
                BackColor = Color.FromArgb(15, 15, 18),
            };
            _videoPlaceholder.Controls.Add(_lblVideoStatus);
            
            panel.Controls.Add(_videoPlaceholder);
            
            return panel;
        }
        
        /// <summary>
        /// Called when Jetson connection status changes.
        /// Initializes video only when Jetson is online.
        /// </summary>
        private void InitializeVideoIfOnline()
        {
            if (_videoInitialized || !_jetsonOnline)
                return;
            
            try
            {
                // Clear placeholder
                _videoPlaceholder.Controls.Clear();
                
                // Build RTSP URL for left camera
                string rtspUrl = $"rtsp://{_config.EffectiveIP}:8554/primary";
                _videoPlayer = new EmbeddedVideoPlayer("ZED Left", rtspUrl, showControls: false, _jetsonConnectionManager);
                _videoPlayer.Dock = DockStyle.Fill;
                _videoPlaceholder.Controls.Add(_videoPlayer);
                
                _videoInitialized = true;
            }
            catch (Exception)
            {
                _lblVideoStatus = new Label
                {
                    Text = "Video unavailable",
                    Font = new Font("Segoe UI", 10),
                    ForeColor = TEXT_SECONDARY,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                    BackColor = Color.FromArgb(15, 15, 18),
                };
                _videoPlaceholder.Controls.Add(_lblVideoStatus);
            }
        }
        
        private Panel CreateHealthSummaryPanel()
        {
            var panel = new Panel
            {
                BackColor = CARD_BG,
                Margin = new Padding(5),
                Dock = DockStyle.Fill,
                Padding = new Padding(15),
            };
            
            var titleLabel = new Label
            {
                Text = "JETSON HEALTH",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(15, 15),
                AutoSize = true,
            };
            panel.Controls.Add(titleLabel);
            
            int yOffset = 50;
            
            // CPU - stored in class field for real-time updates
            _lblHealthCpu = new Label
            {
                Text = "CPU: --",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(_lblHealthCpu);
            yOffset += 25;
            
            // GPU
            _lblHealthGpu = new Label
            {
                Text = "GPU: --",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(_lblHealthGpu);
            yOffset += 25;
            
            // Memory
            _lblHealthMem = new Label
            {
                Text = "Memory: --",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(_lblHealthMem);
            yOffset += 25;
            
            // Disk
            _lblHealthDisk = new Label
            {
                Text = "Disk: --",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(_lblHealthDisk);
            yOffset += 25;
            
            // Temperature
            _lblHealthTemp = new Label
            {
                Text = "Temp: --",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(_lblHealthTemp);
            
            return panel;
        }
        
        // ============================================================
        // Data Updates
        // ============================================================
        
        public void UpdateData()
        {
            if (IsDisposed || !IsHandleCreated) return;

            if (this.InvokeRequired)
            {
                try { this.BeginInvoke((MethodInvoker)delegate { UpdateData(); }); }
                catch (ObjectDisposedException) { }
                catch (InvalidOperationException) { }
                return;
            }
            
            try
            {
                // Update from Mission Planner state
                var cs = MainV2.comPort?.MAV?.cs;
                if (cs == null) return;
                
                // Connection status
                bool connected = cs.connected;
                _lblConnectionValue.Text = connected ? "CONNECTED" : "DISCONNECTED";
                _lblConnectionValue.ForeColor = connected ? SUCCESS_COLOR : ERROR_COLOR;
                
                // Flight mode
                _lblFlightMode.Text = cs.mode ?? "UNKNOWN";
                _lblFlightMode.ForeColor = cs.armed ? WARNING_COLOR : TEXT_PRIMARY;
                
                // GPS status
                int gpsFix = (int)cs.gpsstatus;
                string gpsText = gpsFix switch
                {
                    0 => "No GPS",
                    1 => "No Fix",
                    2 => "2D Fix",
                    3 => "3D Fix",
                    4 => "DGPS",
                    5 => "RTK Float",
                    6 => "RTK Fixed",
                    _ => "Unknown"
                };
                _lblGpsFix.Text = $"{gpsText} ({cs.satcount} sats)";
                _lblGpsFix.ForeColor = gpsFix >= 3 ? SUCCESS_COLOR : (gpsFix >= 1 ? WARNING_COLOR : ERROR_COLOR);
                
                // Battery
                _lblBattery.Text = $"{cs.battery_voltage:F1}V ({cs.battery_remaining}%)";
                _lblBattery.ForeColor = cs.battery_remaining > 30 ? SUCCESS_COLOR : 
                                        (cs.battery_remaining > 15 ? WARNING_COLOR : ERROR_COLOR);
                
                // Check Jetson online status and get health data
                bool jetsonOnline = _sender?.IsJetsonConnected ?? false;
                if (jetsonOnline && !_jetsonOnline)
                {
                    // Jetson just came online - initialize video
                    _jetsonOnline = true;
                    _lblJetsonTemp.Text = "Online";
                    _lblJetsonTemp.ForeColor = SUCCESS_COLOR;
                    InitializeVideoIfOnline();
                }
                else if (!jetsonOnline && _jetsonOnline)
                {
                    // Jetson went offline
                    _jetsonOnline = false;
                    _lblJetsonTemp.Text = "Offline";
                    _lblJetsonTemp.ForeColor = ERROR_COLOR;
                    
                    // Reset health indicators to "--"
                    _lblHealthCpu.Text = "CPU: --";
                    _lblHealthCpu.ForeColor = TEXT_PRIMARY;
                    _lblHealthGpu.Text = "GPU: --";
                    _lblHealthGpu.ForeColor = TEXT_PRIMARY;
                    _lblHealthMem.Text = "Memory: --";
                    _lblHealthMem.ForeColor = TEXT_PRIMARY;
                    _lblHealthDisk.Text = "Disk: --";
                    _lblHealthDisk.ForeColor = TEXT_PRIMARY;
                    _lblHealthTemp.Text = "Temp: --";
                    _lblHealthTemp.ForeColor = TEXT_PRIMARY;
                }
                
                // Update health data from Jetson if online
                if (jetsonOnline && _sender != null)
                {
                    UpdateJetsonHealth();
                }
                
                // Update link status from connection manager
                UpdateLinkStatus();
            }
            catch
            {
                // Ignore update errors
            }
        }
        
        /// <summary>
        /// Updates Jetson health indicators from real API data.
        /// </summary>
        private void UpdateJetsonHealth()
        {
            try
            {
                // Get health data from the sender (which caches the last known values)
                var health = _sender?.LastHealthStatus;
                if (health == null) return;
                
                // CPU
                var cpuLoad = health.CpuUsage;
                _lblHealthCpu.Text = $"CPU: {cpuLoad:F0}%";
                _lblHealthCpu.ForeColor = cpuLoad > 90 ? ERROR_COLOR : (cpuLoad > 70 ? WARNING_COLOR : SUCCESS_COLOR);
                
                // GPU
                var gpuLoad = health.GpuUsage;
                _lblHealthGpu.Text = $"GPU: {gpuLoad:F0}%";
                _lblHealthGpu.ForeColor = gpuLoad > 90 ? ERROR_COLOR : (gpuLoad > 70 ? WARNING_COLOR : SUCCESS_COLOR);
                
                // Memory
                var memUsed = health.MemoryUsed;
                var memTotal = health.MemoryTotal;
                var memPercent = memTotal > 0 ? (memUsed / memTotal * 100) : 0;
                _lblHealthMem.Text = $"Memory: {memPercent:F0}%";
                _lblHealthMem.ForeColor = memPercent > 90 ? ERROR_COLOR : (memPercent > 75 ? WARNING_COLOR : SUCCESS_COLOR);
                
                // Disk
                var diskPercent = health.DiskUsed;
                _lblHealthDisk.Text = $"Disk: {diskPercent:F0}%";
                _lblHealthDisk.ForeColor = diskPercent > 90 ? ERROR_COLOR : (diskPercent > 75 ? WARNING_COLOR : SUCCESS_COLOR);
                
                // Temperature (use GPU temp as primary indicator)
                var temp = health.GpuTemp > 0 ? health.GpuTemp : health.CpuTemp;
                _lblHealthTemp.Text = $"Temp: {temp:F0}C";
                _lblHealthTemp.ForeColor = temp > 80 ? ERROR_COLOR : (temp > 65 ? WARNING_COLOR : SUCCESS_COLOR);
                
                // Also update the Jetson card temperature
                _lblJetsonTemp.Text = $"{temp:F0}C";
                _lblJetsonTemp.ForeColor = temp > 80 ? ERROR_COLOR : (temp > 65 ? WARNING_COLOR : SUCCESS_COLOR);
            }
            catch
            {
                // Ignore health update errors
            }
        }
        
        /// <summary>
        /// Updates link status indicators from connection manager and Jetson status.
        /// </summary>
        private void UpdateLinkStatus()
        {
            try
            {
                // First, check if Jetson is connected via HTTP (Tailscale)
                bool jetsonHttpConnected = _sender?.IsJetsonConnected ?? false;
                
                // Update LTE/Tailscale indicator based on Jetson HTTP connectivity
                _lteIndicator.BackColor = jetsonHttpConnected ? SUCCESS_COLOR : ERROR_COLOR;
                _lteIndicator.Invalidate();
                _lblLteStatus.Text = jetsonHttpConnected 
                    ? "LTE/Tailscale: Connected" 
                    : "LTE/Tailscale: Disconnected";
                
                // Get MAVLink status from connection manager if available
                if (_connectionManager != null)
                {
                    var status = _connectionManager.GetLinkStatus();
                    
                    // Radio status (MAVLink)
                    bool radioConnected = status.RadioConnected;
                    _radioIndicator.BackColor = radioConnected ? SUCCESS_COLOR : ERROR_COLOR;
                    _radioIndicator.Invalidate();
                    _lblRadioStatus.Text = radioConnected 
                        ? $"RadioMaster: {status.RadioLatencyMs}ms" 
                        : "RadioMaster: Disconnected";
                    
                    // Active link - prefer Tailscale if connected
                    if (_lblActiveLink != null)
                    {
                        if (jetsonHttpConnected)
                            _lblActiveLink.Text = "Active: Tailscale (HTTP)";
                        else if (radioConnected)
                            _lblActiveLink.Text = "Active: RadioMaster";
                        else
                            _lblActiveLink.Text = "Active: None";
                    }
                }
                else
                {
                    // No connection manager - show based on Jetson HTTP only
                    _radioIndicator.BackColor = ERROR_COLOR;
                    _radioIndicator.Invalidate();
                    _lblRadioStatus.Text = "RadioMaster: N/A";
                    
                    if (_lblActiveLink != null)
                    {
                        _lblActiveLink.Text = jetsonHttpConnected ? "Active: Tailscale (HTTP)" : "Active: None";
                    }
                }
            }
            catch
            {
                // Ignore link status errors
            }
        }
        
        // ============================================================
        // Dispose
        // ============================================================
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                // Stop and dispose the health polling timer
                if (_healthPollTimer != null)
                {
                    _healthPollTimer.Change(System.Threading.Timeout.Infinite, System.Threading.Timeout.Infinite);
                    _healthPollTimer.Dispose();
                    _healthPollTimer = null;
                }
                
                // Stop notification monitoring and dispose the service
                if (_notificationService != null)
                {
                    _notificationService.StopMonitoring();
                    _notificationService.Dispose();
                    _notificationService = null;
                }
                
                // Dispose the embedded video player
                if (_videoPlayer != null)
                {
                    _videoPlayer.Dispose();
                    _videoPlayer = null;
                }
            }
            
            base.Dispose(disposing);
        }
    }
}
