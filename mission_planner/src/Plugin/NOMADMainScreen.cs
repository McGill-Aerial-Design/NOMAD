// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
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
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Controls;
using MissionPlanner.Plugin;
using NOMAD.MissionPlanner.Core;

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
        private Label _profileLabel;

        // Layout panels
        private Panel _sidebarPanel;
        private Panel _contentPanel;
        private Panel _headerPanel;
        private Panel _viewContainer;

        // Sidebar buttons
        private Button _btnDashboard;
        private Button _btnBoundaries;
        private Button _btnVideo;
        private Button _btnTerminal;
        private Button _btnHealth;
        private Button _btnLinks;
        private Button _btnCalibration;

        // Content views
        private UserControl _currentView;
        private NOMADDashboardView _dashboardView;
        private NOMADBoundaryView _boundaryView;
        private NOMADVideoView _videoView;
        private NOMADTerminalView _terminalView;
        private NOMADHealthView _healthView;
        private NOMADLinksView _linksView;
        private ZedCalibrationView _calibrationView;


        // Update timer
        private System.Windows.Forms.Timer _updateTimer;

        // Module-driven sidebar (optional; see src/Core module SDK). When a host
        // with descriptors is supplied, the sidebar is built from those instead of
        // the hardcoded buttons/views above. With no host the legacy path runs and
        // behavior is unchanged.
        private ModuleHost _moduleHost;
        private List<NomadViewDescriptor> _descriptors;
        private bool _isModuleMode;
        private readonly Dictionary<string, Button> _descriptorButtons = new Dictionary<string, Button>();
        private readonly Dictionary<string, Control> _descriptorViewCache = new Dictionary<string, Control>();
        private string _currentDescriptorId;

        // Static configuration (set by the plugin before this screen is shown)
        private static DualLinkSender _staticSender;
        private static NOMADConfig _staticConfig;
        private static MAVLinkConnectionManager _staticConnectionManager;
        private static JetsonConnectionManager _staticJetsonConnectionManager;
        private static ModuleHost _staticModuleHost;

        /// <summary>
        /// Sets the static configuration used by the MainSwitcher-created instance.
        /// Call this from the plugin before showing the NOMAD screen.
        /// </summary>
        public static void SetStaticConfig(DualLinkSender sender, NOMADConfig config, MAVLinkConnectionManager connectionManager = null, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _staticSender = sender;
            _staticConfig = config;
            _staticConnectionManager = connectionManager;
            _staticJetsonConnectionManager = jetsonConnectionManager;
        }

        /// <summary>
        /// Wire an optional module host before showing the NOMAD screen. The plugin
        /// builds this from discovered NOMAD modules; null means no extra views.
        /// </summary>
        public static void SetStaticModuleHost(ModuleHost host) => _staticModuleHost = host;

        // ============================================================
        // Constructor
        // ============================================================

        /// <summary>
        /// Parameterless constructor required for MainSwitcher.
        /// Uses static configuration set via SetStaticConfig().
        /// </summary>
        public NOMADMainScreen() : this(_staticSender, _staticConfig, _staticConnectionManager, _staticJetsonConnectionManager)
        {
        }

        /// <summary>
        /// Full constructor with explicit dependencies.
        /// </summary>
        public NOMADMainScreen(DualLinkSender sender, NOMADConfig config, MAVLinkConnectionManager connectionManager = null, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _sender = sender;
            _config = config ?? NOMADConfig.Load(); // Fallback to loading config if null
            _connectionManager = connectionManager;
            _jetsonConnectionManager = jetsonConnectionManager;

            // Create dummy sender if none provided
            if (_sender == null && _config != null)
            {
                _sender = new DualLinkSender(_config);
            }

            // Optional module host (set by the plugin). Inert unless it has modules.
            _moduleHost = _staticModuleHost;

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
            if (_isModuleMode)
            {
                var first = _descriptors.FirstOrDefault(d => d.Kind == NomadEntryKind.View);
                if (first != null) ShowEntry(first);
            }
            else
            {
                ShowView("Dashboard");
            }
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

            // If a module host with descriptors is present, build the sidebar from
            // those modules; otherwise fall back to the built-in hardcoded sidebar.
            _descriptors = _moduleHost != null ? _moduleHost.GetViewDescriptors().ToList() : null;
            _isModuleMode = _descriptors != null && _descriptors.Count > 0;
            if (_isModuleMode)
                CreateSidebarFromDescriptors();
            else
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

            // Active-profile indicator (set by scripts/profile.py on profile load).
            var profileText = string.IsNullOrWhiteSpace(_config?.ActiveProfile) ? "dev" : _config.ActiveProfile;
            _profileLabel = new Label
            {
                Text = "● " + profileText,
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                ForeColor = Color.FromArgb(120, 200, 120),
                Dock = DockStyle.Right,
                TextAlign = ContentAlignment.MiddleRight,
                AutoSize = false,
                Width = 110,
            };
            logoPanel.Controls.Add(_profileLabel);
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

            var btnGimbal = CreateSidebarButton("Caddx Gimbal");
            btnGimbal.Click += (s, e) => GimbalJoystickWindow.ShowSingleton(_config, this.FindForm());
            navPanel.Controls.Add(btnGimbal);

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
                    case "Boundaries": headerText = "Flight Boundaries"; break;
                    case "Video": headerText = "Video Feed"; break;
                    case "Terminal": headerText = "Jetson Terminal"; break;
                    case "Health": headerText = "System Health"; break;
                    case "Links": headerText = "Dual Link Status"; break;
                    case "Calibration": headerText = "ZED Camera Calibration"; break;
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
                    }
                    newView = _dashboardView;
                    break;
                case "Boundaries":
                    if (_boundaryView == null) _boundaryView = new NOMADBoundaryView(_config, null);
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
            var buttons = new[] { _btnDashboard, _btnBoundaries, _btnVideo, _btnTerminal, _btnHealth, _btnLinks, _btnCalibration };
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
                case "Boundaries": activeBtn = _btnBoundaries; break;
                case "Video": activeBtn = _btnVideo; break;
                case "Terminal": activeBtn = _btnTerminal; break;
                case "Health": activeBtn = _btnHealth; break;
                case "Links": activeBtn = _btnLinks; break;
                case "Calibration": activeBtn = _btnCalibration; break;
            }

            if (activeBtn != null)
            {
                activeBtn.BackColor = ACCENT_COLOR;
                activeBtn.ForeColor = TEXT_PRIMARY;
            }
        }

        // ============================================================
        // Module-driven Sidebar (NOMAD module SDK — see src/Core)
        // ============================================================
        // Built only when a ModuleHost with view descriptors is supplied. The
        // legacy CreateSidebar()/ShowView() path above is left untouched and runs
        // by default, so behavior is unchanged unless modules are wired in.

        private void CreateSidebarFromDescriptors()
        {
            _sidebarPanel = new Panel
            {
                Dock = DockStyle.Left,
                Width = SIDEBAR_WIDTH,
                BackColor = SIDEBAR_BG,
                Padding = new Padding(0),
            };

            var logoPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 45,
                BackColor = Color.FromArgb(20, 20, 23),
                Padding = new Padding(12, 8, 12, 5),
            };
            logoPanel.Controls.Add(new Label
            {
                Text = "NOMAD",
                Font = new Font("Segoe UI", 14, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(12, 10),
                AutoSize = true,
            });

            var navPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                Padding = new Padding(5),
                AutoScroll = true,
                BackColor = SIDEBAR_BG,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
            };

            string currentSection = null;
            foreach (var descriptor in _descriptors)
            {
                // Emit a section heading whenever the group changes.
                if (!string.IsNullOrEmpty(descriptor.Section) && descriptor.Section != currentSection)
                {
                    navPanel.Controls.Add(CreateSeparatorLabel(descriptor.Section));
                    currentSection = descriptor.Section;
                }

                var entry = descriptor; // capture for the click closure
                var btn = CreateSidebarButton(entry.ButtonText);
                btn.Click += (s, e) => ShowEntry(entry);
                navPanel.Controls.Add(btn);

                if (entry.Kind == NomadEntryKind.View)
                    _descriptorButtons[entry.Id] = btn;
            }

            // Same docking order rules as the legacy sidebar.
            _sidebarPanel.Controls.Add(navPanel);
            _sidebarPanel.Controls.Add(logoPanel);
            this.Controls.Add(_sidebarPanel);
        }

        private void ShowEntry(NomadViewDescriptor descriptor)
        {
            if (descriptor == null) return;

            // Action entries (e.g. floating windows) don't swap the content view.
            if (descriptor.Kind == NomadEntryKind.Action)
            {
                try { descriptor.Invoke(); }
                catch (Exception ex) { Log.Error($"module action '{descriptor.Id}' failed — {ex.Message}"); }
                return;
            }

            // Header text
            var headerLabel = _headerPanel.Controls.Find("lblHeader", false);
            if (headerLabel.Length > 0)
                ((Label)headerLabel[0]).Text = descriptor.Title;

            // Notify the outgoing view it is being hidden (kept cached, not disposed).
            if (_currentView is INomadView previousView)
                previousView.OnDeactivated();

            if (_currentView != null)
                _viewContainer.Controls.Remove(_currentView);

            // Lazily create + cache the view control.
            Control control;
            if (!_descriptorViewCache.TryGetValue(descriptor.Id, out control) || control == null)
            {
                control = descriptor.CreateView();
                _descriptorViewCache[descriptor.Id] = control;
            }

            if (control != null)
            {
                control.Dock = DockStyle.Fill;
                _viewContainer.Controls.Add(control);
                _currentView = control as UserControl; // keeps the update timer working
                _currentDescriptorId = descriptor.Id;
                if (control is INomadView activated)
                    activated.OnActivated();
            }

            UpdateDescriptorButtonState(descriptor.Id);
        }

        private void UpdateDescriptorButtonState(string activeId)
        {
            foreach (var pair in _descriptorButtons)
            {
                pair.Value.BackColor = SIDEBAR_BG;
                pair.Value.ForeColor = TEXT_SECONDARY;
            }

            Button activeBtn;
            if (!string.IsNullOrEmpty(activeId) &&
                _descriptorButtons.TryGetValue(activeId, out activeBtn) && activeBtn != null)
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
                _boundaryView?.Dispose();
                _videoView?.Dispose();
                _terminalView?.Dispose();
                _healthView?.Dispose();
                _linksView?.Dispose();
                _calibrationView?.Dispose();
                // Dispose any module-contributed views built in module mode.
                foreach (var cached in _descriptorViewCache.Values)
                    cached?.Dispose();
                _descriptorViewCache.Clear();
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
