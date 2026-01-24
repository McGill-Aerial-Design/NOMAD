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
        private static readonly Color SIDEBAR_BG = Color.FromArgb(25, 25, 28);
        private static readonly Color CONTENT_BG = Color.FromArgb(30, 30, 33);
        private static readonly Color ACCENT_COLOR = Color.FromArgb(0, 122, 204);
        private static readonly Color ACCENT_HOVER = Color.FromArgb(30, 144, 255);
        private static readonly Color TEXT_PRIMARY = Color.White;
        private static readonly Color TEXT_SECONDARY = Color.FromArgb(180, 180, 180);
        private static readonly Color CARD_BG = Color.FromArgb(40, 40, 45);
        private static readonly Color SUCCESS_COLOR = Color.FromArgb(76, 175, 80);
        private static readonly Color WARNING_COLOR = Color.FromArgb(255, 152, 0);
        private static readonly Color ERROR_COLOR = Color.FromArgb(244, 67, 54);
        
        // ============================================================
        // Fields
        // ============================================================
        
        private DualLinkSender _sender;
        private MAVLinkConnectionManager _connectionManager;
        private NOMADConfig _config;
        
        // Layout panels
        private Panel _sidebarPanel;
        private Panel _contentPanel;
        private Panel _headerPanel;
        private Panel _viewContainer;
        
        // Sidebar buttons
        private Button _btnDashboard;
        private Button _btnTask1;
        private Button _btnTask2;
        private Button _btnVideo;
        private Button _btnTerminal;
        private Button _btnHealth;
        private Button _btnLinks;
        private Button _btnSettings;
        private Button _currentActiveButton;
        
        // Content views
        private UserControl _currentView;
        private NOMADDashboardView _dashboardView;
        private NOMADTask1View _task1View;
        private NOMADTask2View _task2View;
        private NOMADVideoView _videoView;
        private NOMADTerminalView _terminalView;
        private NOMADHealthView _healthView;
        private NOMADLinksView _linksView;
        private NOMADSettingsView _settingsView;
        
        // Update timer
        private System.Windows.Forms.Timer _updateTimer;
        
        // Static configuration (set by the plugin before this screen is shown)
        private static DualLinkSender _staticSender;
        private static NOMADConfig _staticConfig;
        private static MAVLinkConnectionManager _staticConnectionManager;
        
        /// <summary>
        /// Sets the static configuration used by the MainSwitcher-created instance.
        /// Call this from the plugin before showing the NOMAD screen.
        /// </summary>
        public static void SetStaticConfig(DualLinkSender sender, NOMADConfig config, MAVLinkConnectionManager connectionManager = null)
        {
            _staticSender = sender;
            _staticConfig = config;
            _staticConnectionManager = connectionManager;
        }
        
        // ============================================================
        // Constructor
        // ============================================================
        
        /// <summary>
        /// Parameterless constructor required for MainSwitcher.
        /// Uses static configuration set via SetStaticConfig().
        /// </summary>
        public NOMADMainScreen() : this(_staticSender, _staticConfig, _staticConnectionManager)
        {
        }
        
        /// <summary>
        /// Full constructor with explicit dependencies.
        /// </summary>
        public NOMADMainScreen(DualLinkSender sender, NOMADConfig config, MAVLinkConnectionManager connectionManager = null)
        {
            _sender = sender;
            _config = config ?? NOMADConfig.Load(); // Fallback to loading config if null
            _connectionManager = connectionManager;
            
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
            
            _sidebarPanel.Controls.Add(logoPanel);
            
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
            
            // Separator
            navPanel.Controls.Add(CreateSeparatorLabel(""));
            
            // Settings button
            _btnSettings = CreateSidebarButton("Settings");
            _btnSettings.Click += (s, e) => ShowView("Settings");
            navPanel.Controls.Add(_btnSettings);
            
            _sidebarPanel.Controls.Add(navPanel);
            
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
                    case "Video": headerText = "Video Feed"; break;
                    case "Terminal": headerText = "Jetson Terminal"; break;
                    case "Health": headerText = "System Health"; break;
                    case "Links": headerText = "Dual Link Status"; break;
                    case "Settings": headerText = "Settings"; break;
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
                    if (_dashboardView == null) _dashboardView = new NOMADDashboardView(_sender, _config, _connectionManager);
                    newView = _dashboardView;
                    break;
                case "Task1":
                    if (_task1View == null) _task1View = new NOMADTask1View(_sender, _config);
                    newView = _task1View;
                    break;
                case "Task2":
                    if (_task2View == null) _task2View = new NOMADTask2View(_sender, _config);
                    newView = _task2View;
                    break;
                case "Video":
                    if (_videoView == null) _videoView = new NOMADVideoView(_sender, _config);
                    newView = _videoView;
                    break;
                case "Terminal":
                    if (_terminalView == null) _terminalView = new NOMADTerminalView(_config);
                    newView = _terminalView;
                    break;
                case "Health":
                    if (_healthView == null) _healthView = new NOMADHealthView(_config);
                    newView = _healthView;
                    break;
                case "Links":
                    if (_linksView == null) _linksView = new NOMADLinksView(_connectionManager, _config);
                    newView = _linksView;
                    break;
                case "Settings":
                    if (_settingsView == null) _settingsView = new NOMADSettingsView(_config);
                    newView = _settingsView;
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
            var buttons = new[] { _btnDashboard, _btnTask1, _btnTask2, _btnVideo, _btnTerminal, _btnHealth, _btnLinks, _btnSettings };
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
                case "Video": activeBtn = _btnVideo; break;
                case "Terminal": activeBtn = _btnTerminal; break;
                case "Health": activeBtn = _btnHealth; break;
                case "Links": activeBtn = _btnLinks; break;
                case "Settings": activeBtn = _btnSettings; break;
            }
            
            if (activeBtn != null)
            {
                activeBtn.BackColor = ACCENT_COLOR;
                activeBtn.ForeColor = TEXT_PRIMARY;
                _currentActiveButton = activeBtn;
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
                _updateTimer.Interval = 1000; // 1 second updates
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
                _videoView?.Dispose();
                _terminalView?.Dispose();
                _healthView?.Dispose();
                _linksView?.Dispose();
                _settingsView?.Dispose();
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
