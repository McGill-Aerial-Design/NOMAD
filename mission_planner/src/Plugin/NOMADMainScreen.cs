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
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Controls;
using MissionPlanner.Plugin;
using Button = System.Windows.Controls.Button;
using Label = System.Windows.Controls.TextBlock;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Full-page NOMAD screen with sidebar navigation.
    /// This is the main screen that appears when clicking NOMAD in the top navigation.
    /// Inherits from MyUserControl to integrate with Mission Planner's MainSwitcher system.
    /// </summary>
    [global::MissionPlanner.Utilities.PreventTheming]
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
        private NOMADConfig _observedStaticConfig;
        private MAVLinkConnectionManager _observedStaticConnectionManager;

        // Layout panels
        private Panel _sidebarPanel;
        private Panel _contentPanel;
        private Panel _viewContainer;

        // Sidebar buttons
        private Button _btnDashboard;
        private Button _btnBoundaries;
        private Button _btnVideo;
        private Button _btnLinks;
        private Button _btnSimulation;
        private Label _aircraftLabel;

        // Content views
        private UserControl _currentView;
        private string _currentViewName = "Dashboard";
        private NOMADDashboardView _dashboardView;
        private NativeFenceSetupView _boundaryView;
        private NOMADVideoView _videoView;
        private NOMADLinksView _linksView;
        private NOMADSimulationView _simulationView;
        private static Func<SimulationSession> _simulation;
        internal static void SetSimulationFactory(Func<SimulationSession> simulation) { _simulation = simulation; }


        // Update timer
        private System.Windows.Forms.Timer _updateTimer;

        // Static configuration (set by the plugin before this screen is shown)
        private static System.Threading.CancellationToken _videoShutdown;
        private static Action _showSettings;
        private static Func<NOMADConfig, string, bool> _selectVideoInput;
        private static Func<NOMADConfig, string, string, Task<bool>> _selectVideoModel;
        internal static Func<NOMADConfig, string, string, Task<bool>> SelectVideoModel => _selectVideoModel == null
            ? null : (Func<NOMADConfig, string, string, Task<bool>>)SelectCurrentVideoModel;
        private static Task<bool> SelectCurrentVideoModel(NOMADConfig config, string input, string path) =>
            _selectVideoModel?.Invoke(config, input, path) ?? Task.FromResult(false);
        internal static void SetVideoModelSelection(Func<NOMADConfig, string, string, Task<bool>> selectModel)
        { _selectVideoModel = selectModel; }
        internal static Func<NOMADConfig, string, bool> SelectVideoInput => _selectVideoInput == null
            ? null : (Func<NOMADConfig, string, bool>)SelectCurrentVideoInput;
        private static bool SelectCurrentVideoInput(NOMADConfig config, string name) =>
            _selectVideoInput?.Invoke(config, name) == true;
        internal static void SetVideoInputSelection(Func<NOMADConfig, string, bool> selectInput)
        { _selectVideoInput = selectInput; }
        internal static TerminationController Termination => _staticTermination;
        internal static void SetVideoShutdown(System.Threading.CancellationToken token, Action showSettings = null)
        {
            _videoShutdown = token;
            _showSettings = showSettings;
        }
        private static NOMADConfig _staticConfig;
        private static MAVLinkConnectionManager _staticConnectionManager;
        private static FenceSetupInput _staticFenceSetupInput;
        private static TerminationController _staticTermination;
        internal static void SetTerminationController(TerminationController controller) { _staticTermination = controller; }
        internal static void SetFenceSetupInput(FenceSetupInput input)
        {
            if (!ReferenceEquals(_staticFenceSetupInput, input)) { _staticFenceSetupInput?.Drafts?.Dispose(); }
            _staticFenceSetupInput = input;
        }

        /// <summary>
        /// Sets the static configuration used by the MainSwitcher-created instance.
        /// Call this from the plugin before showing the NOMAD screen.
        /// Configuration shared with the plugin-owned screen.
        /// </summary>
        public static void SetStaticConfig(
            NOMADConfig config,
            MAVLinkConnectionManager connectionManager = null)
        {
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
        public NOMADMainScreen() : this(_staticConfig, _staticConnectionManager)
        {
        }

        /// <summary>
        /// Full constructor with explicit dependencies.
        /// </summary>
        public NOMADMainScreen(NOMADConfig config, MAVLinkConnectionManager connectionManager = null)
        {
            _config = config ?? NOMADProfiles.Load().Selected.Config;
            _connectionManager = connectionManager;
            _observedStaticConfig = _staticConfig;
            _observedStaticConnectionManager = _staticConnectionManager;

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
            RefreshConfiguration();
            StartUpdateTimer();
            ShowView(_currentViewName);
            _dashboardView?.RefreshVideoInput();
            _videoView?.UpdateData();
        }

        /// <summary>
        /// Called when this screen is deactivated.
        /// </summary>
        public void Deactivate()
        {
            _videoView?.StopTracking();
            _simulationView?.PausePreview();
            StopUpdateTimer();
        }

        // UI construction lives in NOMADMainScreen.Layout.cs;



        // ============================================================
        // View Management
        // ============================================================

        private void ShowView(string viewName)
        {
            if (_currentView != null && _currentViewName == viewName) { return; }
            _currentViewName = viewName;
            UpdateSidebarButtonState(viewName);

            // Remove current view
            if (_currentView != null)
            {
                if (ReferenceEquals(_currentView, _videoView)) { _videoView.StopTracking(); }
                if (ReferenceEquals(_currentView, _simulationView)) { _simulationView.PausePreview(); }
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
                    }
                    newView = _dashboardView;
                    break;
                case "Boundaries":
                    if (_boundaryView == null)
                    {
                        _boundaryView = new NativeFenceSetupView(_videoShutdown,
                            _staticFenceSetupInput ?? new FenceSetupInput(), _config);
                        _boundaryView.Termination = _staticTermination;
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
                case "Simulation":
                    if (_simulationView == null) { _simulationView = new NOMADSimulationView(_simulation?.Invoke(), _videoShutdown); }
                    newView = _simulationView;
                    break;
            }

            if (newView != null)
            {
                newView.Dock = DockStyle.Fill;
                _viewContainer.Controls.Add(newView);
                _currentView = newView;
                if (newView is NOMADSimulationView simulation) { simulation.UpdateData(); }
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
                _btnSimulation,
            };
            foreach (var btn in buttons)
            {
                if (btn != null)
                {
                    btn.Background = WpfTheme.Background;
                    btn.Foreground = WpfTheme.SecondaryText;
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
                case "Simulation": activeBtn = _btnSimulation; break;
            }

            if (activeBtn != null)
            {
                activeBtn.Background = WpfTheme.Accent;
                activeBtn.Foreground = WpfTheme.Text;
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
            RefreshConfiguration();
            _dashboardView?.RefreshVideoInput();
            _videoView?.UpdateData();
            // Update the current view if it supports updates
            if (_currentView is IUpdatableView updatable)
            {
                updatable.UpdateData();
            }
        }

        private void RefreshConfiguration()
        {
            if (_staticConfig == null || (ReferenceEquals(_observedStaticConfig, _staticConfig) &&
                ReferenceEquals(_observedStaticConnectionManager, _staticConnectionManager)))
            {
                return;
            }
            _observedStaticConfig = _staticConfig;
            _observedStaticConnectionManager = _staticConnectionManager;
            _config = _staticConfig;
            _aircraftLabel.Text = "Aircraft: " + _config.AircraftName;
            _connectionManager = _staticConnectionManager;
            _viewContainer.Controls.Clear();
            _dashboardView?.Dispose();
            _boundaryView?.Dispose();
            _videoView?.Dispose();
            _linksView?.Dispose();
            _simulationView?.Dispose();
            _dashboardView = null;
            _boundaryView = null;
            _videoView = null;
            _linksView = null;
            _simulationView = null;
            _currentView = null;
            if (_updateTimer != null)
            {
                _updateTimer.Interval = Math.Max(100, _config.LinkMonitorInterval);
            }
            ShowView("Dashboard");
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
                _linksView?.Dispose();
                _simulationView?.Dispose();
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
