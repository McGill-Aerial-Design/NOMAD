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
    public partial class NOMADMainScreen : MyUserControl, IActivate, IDeactivate
    {
        // ============================================================
        // Constants
        // ============================================================

        private const int SIDEBAR_WIDTH = 200;
        // Use NOMADTheme for consistent colors across the plugin
        private static readonly Color SIDEBAR_BG = Color.FromArgb(16, 16, 18);
        private static readonly Color CONTENT_BG = NOMADTheme.BG_DARK;
        private static readonly Color ACCENT_COLOR = NOMADTheme.ACCENT;
        private static readonly Color ACCENT_HOVER = Color.FromArgb(240, 60, 70);
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
        private GeofenceConfig _geofenceConfig;
        private BoundaryMonitor _boundaryMonitor;
        private bool _ownsBoundaryMonitor;   // true only when no plugin-owned monitor was provided
        private Label _profileLabel;

        // Layout panels
        private Panel _sidebarPanel;
        private Panel _contentPanel;
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
        private bool _themeReapplyPending;

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
        private static GeofenceConfig _staticGeofenceConfig;
        private static BoundaryMonitor _staticBoundaryMonitor;
        private static ModuleHost _staticModuleHost;

        /// <summary>
        /// Sets the static configuration used by the MainSwitcher-created instance.
        /// Call this from the plugin before showing the NOMAD screen.
        /// The geofence config + boundary monitor are plugin-owned so monitoring
        /// keeps running when MainSwitcher disposes/recreates this screen.
        /// </summary>
        public static void SetStaticConfig(DualLinkSender sender, NOMADConfig config, MAVLinkConnectionManager connectionManager = null, JetsonConnectionManager jetsonConnectionManager = null, GeofenceConfig geofenceConfig = null, BoundaryMonitor boundaryMonitor = null)
        {
            _staticSender = sender;
            _staticConfig = config;
            _staticConnectionManager = connectionManager;
            _staticJetsonConnectionManager = jetsonConnectionManager;
            _staticGeofenceConfig = geofenceConfig;
            _staticBoundaryMonitor = boundaryMonitor;
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

            // Geofence config + boundary monitor shared by the boundary view and
            // the dashboard's notification service. Prefer the plugin-owned
            // instances so monitoring/alerts survive screen disposal; fall back
            // to screen-owned ones (and dispose them) when none were provided.
            _geofenceConfig = _staticGeofenceConfig ?? GeofenceConfig.Load();
            _boundaryMonitor = _staticBoundaryMonitor;
            if (_boundaryMonitor == null)
            {
                _boundaryMonitor = new BoundaryMonitor(_geofenceConfig, _config);
                _ownsBoundaryMonitor = true;
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
            // Mission Planner's ThemeManager restyles this screen around
            // MainSwitcher.ShowScreen (recursively overwriting Back/ForeColor),
            // so the NOMAD palette only survived until MP's pass ran. Re-assert
            // now and once more after the current UI batch, whichever order MP
            // applies its theme in.
            ReapplyNomadTheme();
            QueueThemeReapply();

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
        /// Restore the NOMAD chrome colors (sidebar, logo strip, content area)
        /// after Mission Planner's ThemeManager repaints the control tree.
        /// View internals are built after MP's pass and keep their own colors.
        /// </summary>
        private void ReapplyNomadTheme()
        {
            BackColor = CONTENT_BG;
            if (_contentPanel != null) _contentPanel.BackColor = CONTENT_BG;
            if (_viewContainer != null) _viewContainer.BackColor = CONTENT_BG;
            if (_sidebarPanel == null) return;

            _sidebarPanel.BackColor = SIDEBAR_BG;
            foreach (Control child in _sidebarPanel.Controls)
            {
                if (child is FlowLayoutPanel nav)
                {
                    nav.BackColor = SIDEBAR_BG;
                    foreach (Control c in nav.Controls)
                    {
                        if (c is Button b)
                        {
                            b.BackColor = SIDEBAR_BG;
                            b.ForeColor = TEXT_SECONDARY;
                        }
                        else if (c is Label l)
                        {
                            l.BackColor = SIDEBAR_BG;
                            l.ForeColor = TEXT_SECONDARY;
                        }
                    }
                }
                else
                {
                    // Logo strip
                    child.BackColor = Color.FromArgb(8, 8, 10);
                    foreach (Control c in child.Controls)
                    {
                        if (c == _profileLabel) continue; // keeps its status color
                        c.BackColor = Color.FromArgb(8, 8, 10);
                        c.ForeColor = ACCENT_COLOR;
                    }
                }
            }
        }

        private void QueueThemeReapply()
        {
            if (IsDisposed)
                return;

            if (!IsHandleCreated)
            {
                _themeReapplyPending = true;
                return;
            }

            _themeReapplyPending = false;
            BeginInvoke((MethodInvoker)(() =>
            {
                if (!IsDisposed)
                    ReapplyNomadTheme();
            }));
        }

        protected override void OnHandleCreated(EventArgs e)
        {
            base.OnHandleCreated(e);
            if (_themeReapplyPending)
                QueueThemeReapply();
        }

        /// <summary>
        /// Called when this screen is deactivated.
        /// </summary>
        public void Deactivate()
        {
            StopUpdateTimer();
        }

        // UI construction lives in NOMADMainScreen.Layout.cs;
        // the module-driven sidebar path lives in NOMADMainScreen.Modules.cs.


        // ============================================================
        // View Management
        // ============================================================

        private void InitializeViews()
        {
            // Create all views lazily - they'll be created when first accessed
        }

        private void ShowView(string viewName)
        {
            // Keep the chrome on-theme even if MP's ThemeManager ran since.
            ReapplyNomadTheme();

            // Update sidebar button states
            UpdateSidebarButtonState(viewName);

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
                        if (_boundaryMonitor != null)
                        {
                            _dashboardView.SetBoundaryMonitor(_boundaryMonitor);
                        }
                    }
                    newView = _dashboardView;
                    break;
                case "Boundaries":
                    if (_boundaryView == null) _boundaryView = new NOMADBoundaryView(_geofenceConfig, _config, _boundaryMonitor);
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

                if (_ownsBoundaryMonitor) _boundaryMonitor?.Dispose();
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
