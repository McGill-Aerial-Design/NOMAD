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

        private MAVLinkConnectionManager _connectionManager;
        private NOMADConfig _config;
        private GeofenceConfig _geofenceConfig;
        private AdvisoryBoundaryMonitor _advisoryBoundaryMonitor;
        private bool _ownsAdvisoryBoundaryMonitor;

        // Layout panels
        private Panel _sidebarPanel;
        private Panel _contentPanel;
        private Panel _viewContainer;

        // Sidebar buttons
        private Button _btnDashboard;
        private Button _btnBoundaries;
        private Button _btnVideo;
        private Button _btnLinks;

        // Content views
        private UserControl _currentView;
        private NOMADDashboardView _dashboardView;
        private NOMADBoundaryView _boundaryView;
        private NOMADVideoView _videoView;
        private NOMADLinksView _linksView;


        // Update timer
        private System.Windows.Forms.Timer _updateTimer;
        private bool _themeReapplyPending;

        // Static configuration (set by the plugin before this screen is shown)
        private static System.Threading.CancellationToken _videoShutdown;
        internal static void SetVideoShutdown(System.Threading.CancellationToken token) => _videoShutdown = token;
        private static NOMADConfig _staticConfig;
        private static MAVLinkConnectionManager _staticConnectionManager;
        private static GeofenceConfig _staticGeofenceConfig;
        private static AdvisoryBoundaryMonitor _staticAdvisoryBoundaryMonitor;

        /// <summary>
        /// Sets the static configuration used by the MainSwitcher-created instance.
        /// Call this from the plugin before showing the NOMAD screen.
        /// Advisory outline config and preview are plugin-owned so they remain
        /// available when MainSwitcher recreates this screen.
        /// </summary>
        public static void SetStaticConfig(
            NOMADConfig config,
            MAVLinkConnectionManager connectionManager = null,
            GeofenceConfig geofenceConfig = null,
            AdvisoryBoundaryMonitor advisoryBoundaryMonitor = null)
        {
            _staticConfig = config;
            _staticConnectionManager = connectionManager;
            _staticGeofenceConfig = geofenceConfig;
            _staticAdvisoryBoundaryMonitor = advisoryBoundaryMonitor;
        }

        // ============================================================
        // Constructor
        // ============================================================

        /// <summary>
        /// Parameterless constructor required for MainSwitcher.
        /// Uses static configuration set via SetStaticConfig().
        /// </summary>
        public NOMADMainScreen() : this(_staticConfig, _staticConnectionManager)
        {
        }

        /// <summary>
        /// Full constructor with explicit dependencies.
        /// </summary>
        public NOMADMainScreen(NOMADConfig config, MAVLinkConnectionManager connectionManager = null)
        {
            _config = config ?? NOMADConfig.Load(); // Fallback to loading config if null
            _connectionManager = connectionManager;

            // Advisory outline config + preview shared by the boundary view and
            // the dashboard's notification service. Prefer the plugin-owned
            // instances so monitoring/alerts survive screen disposal; fall back
            // to screen-owned ones (and dispose them) when none were provided.
            _geofenceConfig = _staticGeofenceConfig ?? GeofenceConfig.Load();
            _advisoryBoundaryMonitor = _staticAdvisoryBoundaryMonitor;
            if (_advisoryBoundaryMonitor == null)
            {
                _advisoryBoundaryMonitor = new AdvisoryBoundaryMonitor(_geofenceConfig);
                _ownsAdvisoryBoundaryMonitor = true;
            }

            InitializeUI();

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
            ShowView("Dashboard");
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



        // ============================================================
        // View Management
        // ============================================================

        private void ShowView(string viewName)
        {
            // Keep the chrome on-theme even if MP's ThemeManager ran since.
            ReapplyNomadTheme();

            // Update sidebar button states (and clear any active module button).
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
                        _dashboardView = new NOMADDashboardView(_config, _connectionManager, _videoShutdown);
                        if (_advisoryBoundaryMonitor != null)
                        {
                            _dashboardView.SetAdvisoryBoundaryMonitor(_advisoryBoundaryMonitor);
                        }
                    }
                    newView = _dashboardView;
                    break;
                case "Boundaries":
                    if (_boundaryView == null)
                    {
                        _boundaryView = new NOMADBoundaryView(_geofenceConfig, _advisoryBoundaryMonitor);
                    }
                    newView = _boundaryView;
                    break;
                case "Video":
                    if (_videoView == null) _videoView = new NOMADVideoView(_config, _videoShutdown);
                    newView = _videoView;
                    break;
                case "Links":
                    if (_linksView == null) _linksView = new NOMADLinksView(_connectionManager, _config);
                    newView = _linksView;
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
            var buttons = new[]
            {
                _btnDashboard,
                _btnBoundaries,
                _btnVideo,
                _btnLinks,
            };
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
                case "Links": activeBtn = _btnLinks; break;
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
                _updateTimer.Interval = Math.Max(100, _config.LinkMonitorInterval);
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

                if (_ownsAdvisoryBoundaryMonitor) _advisoryBoundaryMonitor?.Dispose();
                _dashboardView?.Dispose();
                _boundaryView?.Dispose();
                _videoView?.Dispose();
                _linksView?.Dispose();
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
