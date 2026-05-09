// ============================================================
// NOMAD Main Screen - Full-Page Mission Planner Integration
// ============================================================
// A complete sidebar-based interface for NOMAD operations in Mission Planner.
// Similar to HWConfig/SWConfig pages with BackstageView sidebar navigation.
//
// Features:
// - Sidebar navigation (left panel) with section buttons
// - Main content area on the right
// - Dashboard as the default/home view
// - Clean, modern dark theme design
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Controls;
using MissionPlanner.Plugin;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Full-page NOMAD screen with sidebar navigation.
    /// This is the main screen that appears when clicking NOMAD in the top navigation.
    /// Inherits from MyUserControl to integrate with Mission Planner's MainSwitcher system.
    /// </summary>
    public class NOMADMainScreen : MyUserControl, IActivate, IDeactivate
    {
        // ============================================================
        // Constants
        // ============================================================
        
        private const int SIDEBAR_WIDTH = 200;
        // Use NOMADTheme for consistent colors across the plugin
        private static readonly Color SIDEBAR_BG = Color.FromArgb(25, 25, 28);
        private static readonly Color CONTENT_BG = NOMADTheme.BG_DARK;
        private static readonly Color ACCENT_COLOR = NOMADTheme.ACCENT;
        private static readonly Color ACCENT_HOVER = Color.FromArgb(30, 144, 255);
        private static readonly Color TEXT_PRIMARY = NOMADTheme.TEXT_PRIMARY;
        private static readonly Color TEXT_SECONDARY = NOMADTheme.TEXT_SECONDARY;
        private static readonly Color CARD_BG = NOMADTheme.CARD_BG;
        private static readonly Color SUCCESS_COLOR = NOMADTheme.SUCCESS;
        private static readonly Color WARNING_COLOR = NOMADTheme.WARNING;
        private static readonly Color ERROR_COLOR = NOMADTheme.ERROR;
        
        // ============================================================
        // Fields
        // ============================================================
        
        private DualLinkSender _sender;
        private MAVLinkConnectionManager _connectionManager;
        private JetsonConnectionManager _jetsonConnectionManager;
        private NOMADConfig _config;
        private MissionConfig _missionConfig;
        private BoundaryMonitor _boundaryMonitor;
        
        // Layout panels
        private Panel _sidebarPanel;
        private Panel _contentPanel;
        private Panel _headerPanel;
        private Panel _viewContainer;
        
        // Sidebar buttons
        private Button _btnDashboard;
        private Button _btnTask1;
        private Button _btnTask2;
        private Button _btnBoundaries;
        private Button _btnVideo;
        private Button _btnTerminal;
        private Button _btnHealth;
        private Button _btnLinks;
        private Button _btnCalibration;
        private Button _btnRviz2;
        
        // Content views
        private UserControl _currentView;
        private NOMADDashboardView _dashboardView;
        private NOMADTask1View _task1View;
        private NOMADTask2View _task2View;
        private NOMADBoundaryView _boundaryView;
        private NOMADVideoView _videoView;
        private NOMADTerminalView _terminalView;
        private NOMADHealthView _healthView;
        private NOMADLinksView _linksView;
        private ZedCalibrationView _calibrationView;
        private Rviz2View _rviz2View;
        
        // Update timer
        private System.Windows.Forms.Timer _updateTimer;
        
        // Static configuration (set by the plugin before this screen is shown)
        private static DualLinkSender _staticSender;
        private static NOMADConfig _staticConfig;
        private static MAVLinkConnectionManager _staticConnectionManager;
        private static JetsonConnectionManager _staticJetsonConnectionManager;
        private static MissionConfig _staticMissionConfig;
        private static BoundaryMonitor _staticBoundaryMonitor;
        
        /// <summary>
        /// Sets the static configuration used by the MainSwitcher-created instance.
        /// Call this from the plugin before showing the NOMAD screen.
        /// </summary>
        public static void SetStaticConfig(DualLinkSender sender, NOMADConfig config, MAVLinkConnectionManager connectionManager = null, JetsonConnectionManager jetsonConnectionManager = null, MissionConfig missionConfig = null, BoundaryMonitor boundaryMonitor = null)
        {
            _staticSender = sender;
            _staticConfig = config;
            _staticConnectionManager = connectionManager;
            _staticJetsonConnectionManager = jetsonConnectionManager;
            _staticMissionConfig = missionConfig;
            _staticBoundaryMonitor = boundaryMonitor;
        }
        
        // ============================================================
        // Constructor
        // ============================================================
        
        /// <summary>
        /// Parameterless constructor required for MainSwitcher.
        /// Uses static configuration set via SetStaticConfig().
        /// </summary>
        public NOMADMainScreen() : this(_staticSender, _staticConfig, _staticConnectionManager, _staticJetsonConnectionManager, _staticMissionConfig, _staticBoundaryMonitor)
        {
        }
        
        /// <summary>
        /// Full constructor with explicit dependencies.
        /// </summary>
        public NOMADMainScreen(DualLinkSender sender, NOMADConfig config, MAVLinkConnectionManager connectionManager = null, JetsonConnectionManager jetsonConnectionManager = null, MissionConfig missionConfig = null, BoundaryMonitor boundaryMonitor = null)
        {
            _sender = sender;
            _config = config ?? NOMADConfig.Load(); // Fallback to loading config if null
            _connectionManager = connectionManager;
            _jetsonConnectionManager = jetsonConnectionManager;
            _missionConfig = missionConfig ?? MissionConfig.Load(); // Load mission config
            _boundaryMonitor = boundaryMonitor;
            
            // Create boundary monitor if not provided
            if (_boundaryMonitor == null && _missionConfig != null && _config != null)
            {
                _boundaryMonitor = new BoundaryMonitor(_missionConfig, _config);
            }
            
            // Create dummy sender if none provided
            if (_sender == null && _config != null)
            {
                _sender = new DualLinkSender(_config);
            }
            
            InitializeUI();
            InitializeViews();
            
            // Don't start timer yet - wait for Activate()
        }
        
        // ============================================================
        // IActivate / IDeactivate Implementation
        // ============================================================
        
        /// <summary>
        /// Called when this screen becomes active.
        /// </summary>
        public void Activate()
        {
            StartUpdateTimer();
            ShowView("Dashboard");
        }
        
        /// <summary>
        /// Called when this screen is deactivated.
        /// </summary>
        public void Deactivate()
        {
            StopUpdateTimer();
        }
        
        // ============================================================
        // UI Initialization
        // ============================================================
        
        private void InitializeUI()
        {
            this.BackColor = CONTENT_BG;
            this.Dock = DockStyle.Fill;
            
            // IMPORTANT: In Windows Forms, docking order matters!
            // Controls are docked in REVERSE order of addition.
            // So we add content area FIRST (fills remaining), then sidebar (docks left).
            CreateContentArea();
            CreateSidebar();
        }
        
        private void CreateSidebar()
        {
            _sidebarPanel = new Panel
            {
                Dock = DockStyle.Left,
                Width = SIDEBAR_WIDTH,
                BackColor = SIDEBAR_BG,
                Padding = new Padding(0),
            };
            
            // Logo/Title area at top
            var logoPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 45,
                BackColor = Color.FromArgb(20, 20, 23),
                Padding = new Padding(12, 8, 12, 5),
            };
            
            var logoLabel = new Label
            {
                Text = "NOMAD",
                Font = new Font("Segoe UI", 14, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(12, 10),
                AutoSize = true,
            };
            logoPanel.Controls.Add(logoLabel);
            
            var subtitleLabel = new Label
            {
                Text = "| AEAC 2026",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(88, 14),
                AutoSize = true,
            };
            logoPanel.Controls.Add(subtitleLabel);
            
            // Navigation buttons using FlowLayoutPanel for automatic positioning
            var navPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                Padding = new Padding(5),
                AutoScroll = true,
                BackColor = SIDEBAR_BG,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
            };
            
            // Dashboard button (primary)
            _btnDashboard = CreateSidebarButton("Dashboard");
            _btnDashboard.Click += (s, e) => ShowView("Dashboard");
            navPanel.Controls.Add(_btnDashboard);
            
            // Separator
            navPanel.Controls.Add(CreateSeparatorLabel("MISSIONS"));
            
            // Task 1 button
            _btnTask1 = CreateSidebarButton("Task 1: Recon");
            _btnTask1.Click += (s, e) => ShowView("Task1");
            navPanel.Controls.Add(_btnTask1);
            
            // Task 2 button  
            _btnTask2 = CreateSidebarButton("Task 2: Extinguish");
            _btnTask2.Click += (s, e) => ShowView("Task2");
            navPanel.Controls.Add(_btnTask2);
            
            // Separator
            navPanel.Controls.Add(CreateSeparatorLabel("SAFETY"));
            
            // Boundaries button (important for competition)
            _btnBoundaries = CreateSidebarButton("Flight Boundaries");
            _btnBoundaries.Click += (s, e) => ShowView("Boundaries");
            navPanel.Controls.Add(_btnBoundaries);
            
            // Separator
            navPanel.Controls.Add(CreateSeparatorLabel("TOOLS"));
            
            // Video button
            _btnVideo = CreateSidebarButton("Video Feed");
            _btnVideo.Click += (s, e) => ShowView("Video");
            navPanel.Controls.Add(_btnVideo);
            
            // Terminal button
            _btnTerminal = CreateSidebarButton("Terminal");
            _btnTerminal.Click += (s, e) => ShowView("Terminal");
            navPanel.Controls.Add(_btnTerminal);
            
            // Health button
            _btnHealth = CreateSidebarButton("System Health");
            _btnHealth.Click += (s, e) => ShowView("Health");
            navPanel.Controls.Add(_btnHealth);
            
            // Links button
            _btnLinks = CreateSidebarButton("Link Status");
            _btnLinks.Click += (s, e) => ShowView("Links");
            navPanel.Controls.Add(_btnLinks);
            
            // ZED Calibration button
            _btnCalibration = CreateSidebarButton("ZED Calibration");
            _btnCalibration.Click += (s, e) => ShowView("Calibration");
            navPanel.Controls.Add(_btnCalibration);

            // RViz2 button
            _btnRviz2 = CreateSidebarButton("RViz2 (Remote)");
            _btnRviz2.Click += (s, e) => ShowView("Rviz2");
            navPanel.Controls.Add(_btnRviz2);
            
            // IMPORTANT: In Windows Forms, docking order is reverse of Z-order
            // Add navPanel FIRST (will be at back, fills remaining space)
            // Add logoPanel SECOND (will be in front, docked at top)
            _sidebarPanel.Controls.Add(navPanel);
            _sidebarPanel.Controls.Add(logoPanel);
            
            this.Controls.Add(_sidebarPanel);
        }
        
        private Button CreateSidebarButton(string text)
        {
            var btn = new Button
            {
                Text = text,
                Size = new Size(SIDEBAR_WIDTH - 15, 36),
                Margin = new Padding(0, 1, 0, 1),
                FlatStyle = FlatStyle.Flat,
                BackColor = SIDEBAR_BG,  // Explicit background color to prevent default green
                ForeColor = TEXT_SECONDARY,
                Font = new Font("Segoe UI", 9),
                TextAlign = ContentAlignment.MiddleLeft,
                Padding = new Padding(10, 0, 0, 0),
                Cursor = Cursors.Hand,
            };
            btn.FlatAppearance.BorderSize = 0;
            btn.FlatAppearance.MouseOverBackColor = Color.FromArgb(45, 45, 50);
            btn.FlatAppearance.MouseDownBackColor = ACCENT_COLOR;
            
            return btn;
        }
        
        private Label CreateSeparatorLabel(string text)
        {
            if (string.IsNullOrEmpty(text))
            {
                return new Label
                {
                    Size = new Size(SIDEBAR_WIDTH - 15, 8),
                    Margin = new Padding(0, 4, 0, 4),
                };
            }
            
            return new Label
            {
                Text = text,
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                ForeColor = TEXT_SECONDARY,
                Size = new Size(SIDEBAR_WIDTH - 15, 22),
                Margin = new Padding(0, 8, 0, 4),
                TextAlign = ContentAlignment.BottomLeft,
                Padding = new Padding(5, 0, 0, 0),
            };
        }
        private void CreateContentArea()
        {
            _contentPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CONTENT_BG,
                Padding = new Padding(0),
            };
            
            // Header panel with breadcrumb/title - docked at top
            _headerPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 60,
                BackColor = Color.FromArgb(35, 35, 40),
                Padding = new Padding(25, 15, 25, 15),
            };
            
            var headerLabel = new Label
            {
                Name = "lblHeader",
                Text = "Dashboard",
                Font = new Font("Segoe UI", 16, FontStyle.Bold),
                ForeColor = TEXT_PRIMARY,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft,
            };
            _headerPanel.Controls.Add(headerLabel);
            
            // Create a view container panel that will hold the actual view content
            // This ensures the view doesn't overlap with the header
            _viewContainer = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CONTENT_BG,
                Padding = new Padding(20), // Padding around view content
            };
            
            // Add header first (will be at bottom of z-order)
            // Then add view container (will fill remaining space)
            // In WinForms, controls are docked in reverse order of addition
            _contentPanel.Controls.Add(_viewContainer);  // Added first, so it fills remaining space
            _contentPanel.Controls.Add(_headerPanel);    // Added last, so it docks on top
            
            this.Controls.Add(_contentPanel);
        }
        
        // ============================================================
        // View Management
        // ============================================================
        
        private void InitializeViews()
        {
            // Create all views lazily - they'll be created when first accessed
        }
        
        private void ShowView(string viewName)
        {
            // Update sidebar button states
            UpdateSidebarButtonState(viewName);
            
            // Update header
            var headerLabel = _headerPanel.Controls.Find("lblHeader", false);
            if (headerLabel.Length > 0)
            {
                string headerText = viewName;
                switch (viewName)
                {
                    case "Dashboard": headerText = "Dashboard"; break;
                    case "Task1": headerText = "Task 1: Outdoor Reconnaissance"; break;
                    case "Task2": headerText = "Task 2: Indoor Fire Extinguishing"; break;
                    case "Boundaries": headerText = "Flight Boundaries"; break;
                    case "Video": headerText = "Video Feed"; break;
                    case "Terminal": headerText = "Jetson Terminal"; break;
                    case "Health": headerText = "System Health"; break;
                    case "Links": headerText = "Dual Link Status"; break;
                    case "Calibration": headerText = "ZED Camera Calibration"; break;
                    case "Rviz2": headerText = "RViz2 Remote Viewer"; break;
                }
                ((Label)headerLabel[0]).Text = headerText;
            }
            
            // Remove current view
            if (_currentView != null)
            {
                _viewContainer.Controls.Remove(_currentView);
                // Don't dispose - keep cached for quick switching
            }
            
            // Get or create the requested view
            UserControl newView = null;
            switch (viewName)
            {
                case "Dashboard":
                    if (_dashboardView == null)
                    {
                        _dashboardView = new NOMADDashboardView(_sender, _config, _connectionManager, _jetsonConnectionManager);
                        // Connect boundary monitor to notification service
                        if (_boundaryMonitor != null)
                        {
                            _dashboardView.SetBoundaryMonitor(_boundaryMonitor);
                        }
                    }
                    newView = _dashboardView;
                    break;
                case "Task1":
                    if (_task1View == null) _task1View = new NOMADTask1View(_sender, _config, _missionConfig, _jetsonConnectionManager);
                    newView = _task1View;
                    break;
                case "Task2":
                    if (_task2View == null) _task2View = new NOMADTask2View(_sender, _config, _jetsonConnectionManager);
                    newView = _task2View;
                    break;
                case "Boundaries":
                    if (_boundaryView == null) _boundaryView = new NOMADBoundaryView(_missionConfig, _config, _boundaryMonitor);
                    newView = _boundaryView;
                    break;
                case "Video":
                    if (_videoView == null) _videoView = new NOMADVideoView(_sender, _config, _jetsonConnectionManager);
                    newView = _videoView;
                    break;
                case "Terminal":
                    if (_terminalView == null) _terminalView = new NOMADTerminalView(_config);
                    newView = _terminalView;
                    break;
                case "Health":
                    if (_healthView == null) _healthView = new NOMADHealthView(_config, _sender);
                    newView = _healthView;
                    break;
                case "Links":
                    if (_linksView == null) _linksView = new NOMADLinksView(_connectionManager, _config);
                    newView = _linksView;
                    break;
                case "Calibration":
                    if (_calibrationView == null) _calibrationView = new ZedCalibrationView(_config);
                    newView = _calibrationView;
                    break;
                case "Rviz2":
                    if (_rviz2View == null) _rviz2View = new Rviz2View(_config);
                    newView = _rviz2View;
                    break;
            }
            
            if (newView != null)
            {
                newView.Dock = DockStyle.Fill;
                _viewContainer.Controls.Add(newView);
                _currentView = newView;
            }
        }
        
        private void UpdateSidebarButtonState(string viewName)
        {
            // Reset all buttons to default state
            var buttons = new[] { _btnDashboard, _btnTask1, _btnTask2, _btnBoundaries, _btnVideo, _btnTerminal, _btnHealth, _btnLinks, _btnCalibration, _btnRviz2 };
            foreach (var btn in buttons)
            {
                if (btn != null)
                {
                    btn.BackColor = SIDEBAR_BG;  // Match sidebar background
                    btn.ForeColor = TEXT_SECONDARY;
                }
            }
            
            // Highlight active button
            Button activeBtn = null;
            switch (viewName)
            {
                case "Dashboard": activeBtn = _btnDashboard; break;
                case "Task1": activeBtn = _btnTask1; break;
                case "Task2": activeBtn = _btnTask2; break;
                case "Boundaries": activeBtn = _btnBoundaries; break;
                case "Video": activeBtn = _btnVideo; break;
                case "Terminal": activeBtn = _btnTerminal; break;
                case "Health": activeBtn = _btnHealth; break;
                case "Links": activeBtn = _btnLinks; break;
                case "Calibration": activeBtn = _btnCalibration; break;
                case "Rviz2": activeBtn = _btnRviz2; break;
            }
            
            if (activeBtn != null)
            {
                activeBtn.BackColor = ACCENT_COLOR;
                activeBtn.ForeColor = TEXT_PRIMARY;
            }
        }
        
        // ============================================================
        // Update Timer
        // ============================================================
        
        private void StartUpdateTimer()
        {
            if (_updateTimer == null)
            {
                _updateTimer = new System.Windows.Forms.Timer();
                _updateTimer.Interval = _config.HealthPollInterval; // Honor config polling interval
                _updateTimer.Tick += UpdateTimer_Tick;
            }
            _updateTimer.Start();
        }
        
        private void StopUpdateTimer()
        {
            _updateTimer?.Stop();
        }
        
        private void UpdateTimer_Tick(object sender, EventArgs e)
        {
            // Update the current view if it supports updates
            if (_currentView is IUpdatableView updatable)
            {
                updatable.UpdateData();
            }
        }
        
        // ============================================================
        // Cleanup
        // ============================================================
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _updateTimer?.Stop();
                _updateTimer?.Dispose();
                
                _dashboardView?.Dispose();
                _task1View?.Dispose();
                _task2View?.Dispose();
                _boundaryView?.Dispose();
                _videoView?.Dispose();
                _terminalView?.Dispose();
                _healthView?.Dispose();
                _linksView?.Dispose();
                _calibrationView?.Dispose();
                _rviz2View?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
    
    /// <summary>
    /// Interface for views that support periodic updates
    /// </summary>
    public interface IUpdatableView
    {
        void UpdateData();
    }
}
