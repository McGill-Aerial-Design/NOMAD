// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Dashboard View - Main Overview Panel
// ============================================================
// A simplified, information-dense dashboard showing all key data at a glance.
// Features:
// - Connection status with visual indicators
// - Flight mode and telemetry summary
// - Enlarged video preview
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
    public partial class NOMADDashboardView : UserControl, IUpdatableView
    {
        // ============================================================
        // Constants
        // ============================================================

        // Kept locally because NOMADTheme.TEXT_SECONDARY (150,150,150) differs from this value
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

        // EKF source control
        private EkfSourceControlPanel _ekfControlPanel;

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

            // Prefer the plugin-owned NotificationService so monitoring continues
            // even when this view is not focused. Fall back to a local one only if
            // the plugin didn't create one (e.g. running standalone in tests).
            _notificationService = NotificationService.Shared
                ?? new NotificationService(null, sender);

            InitializeUI();

            // Start health polling to keep Jetson status updated
            StartHealthPolling();

            // Start monitoring only if this view actually owns the service (no Shared yet).
            if (NotificationService.Shared == null)
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

        // UI construction lives in NOMADDashboardView.Layout.cs.

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


        // ============================================================
        // Data Updates
        // ============================================================

        public void UpdateData()
        {
            if (IsDisposed || !IsHandleCreated) return;
            UiAsync.RunSync(this, UpdateDataCore, "UpdateData");
        }

        private void UpdateDataCore()
        {
            try
            {
                // Update from Mission Planner state
                var cs = MainV2.comPort?.MAV?.cs;
                if (cs == null) return;

                // Connection status
                bool connected = cs.connected;
                _lblConnectionValue.Text = connected ? "CONNECTED" : "DISCONNECTED";
                _lblConnectionValue.ForeColor = connected ? NOMADTheme.SUCCESS : NOMADTheme.ERROR;

                // Flight mode
                _lblFlightMode.Text = cs.mode ?? "UNKNOWN";
                _lblFlightMode.ForeColor = cs.armed ? NOMADTheme.WARNING : NOMADTheme.TEXT_PRIMARY;

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
                _lblGpsFix.ForeColor = gpsFix >= 3 ? NOMADTheme.SUCCESS : (gpsFix >= 1 ? NOMADTheme.WARNING : NOMADTheme.ERROR);

                // Battery
                _lblBattery.Text = $"{cs.battery_voltage:F1}V ({cs.battery_remaining}%)";
                _lblBattery.ForeColor = cs.battery_remaining > 30 ? NOMADTheme.SUCCESS :
                                        (cs.battery_remaining > 15 ? NOMADTheme.WARNING : NOMADTheme.ERROR);

                // Check Jetson online status and get health data
                bool jetsonOnline = _sender?.IsJetsonConnected ?? false;
                if (jetsonOnline && !_jetsonOnline)
                {
                    // Jetson just came online - initialize video
                    _jetsonOnline = true;
                    _lblJetsonTemp.Text = "Online";
                    _lblJetsonTemp.ForeColor = NOMADTheme.SUCCESS;
                    InitializeVideoIfOnline();
                }
                else if (!jetsonOnline && _jetsonOnline)
                {
                    // Jetson went offline
                    _jetsonOnline = false;
                    _lblJetsonTemp.Text = "Offline";
                    _lblJetsonTemp.ForeColor = NOMADTheme.ERROR;

                    // Reset health indicators to "--"
                    _lblHealthCpu.Text = "CPU: --";
                    _lblHealthCpu.ForeColor = NOMADTheme.TEXT_PRIMARY;
                    _lblHealthGpu.Text = "GPU: --";
                    _lblHealthGpu.ForeColor = NOMADTheme.TEXT_PRIMARY;
                    _lblHealthMem.Text = "Memory: --";
                    _lblHealthMem.ForeColor = NOMADTheme.TEXT_PRIMARY;
                    _lblHealthDisk.Text = "Disk: --";
                    _lblHealthDisk.ForeColor = NOMADTheme.TEXT_PRIMARY;
                    _lblHealthTemp.Text = "Temp: --";
                    _lblHealthTemp.ForeColor = NOMADTheme.TEXT_PRIMARY;
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
                _lblHealthCpu.ForeColor = cpuLoad > 90 ? NOMADTheme.ERROR : (cpuLoad > 70 ? NOMADTheme.WARNING : NOMADTheme.SUCCESS);

                // GPU
                var gpuLoad = health.GpuUsage;
                _lblHealthGpu.Text = $"GPU: {gpuLoad:F0}%";
                _lblHealthGpu.ForeColor = gpuLoad > 90 ? NOMADTheme.ERROR : (gpuLoad > 70 ? NOMADTheme.WARNING : NOMADTheme.SUCCESS);

                // Memory
                var memPercent = health.MemoryUsed;
                _lblHealthMem.Text = $"Memory: {memPercent:F0}%";
                _lblHealthMem.ForeColor = memPercent > 90 ? NOMADTheme.ERROR : (memPercent > 75 ? NOMADTheme.WARNING : NOMADTheme.SUCCESS);

                // Disk
                var diskFreeGb = health.DiskFreeGb;
                _lblHealthDisk.Text = $"Disk: {diskFreeGb:F0} GB free";
                _lblHealthDisk.ForeColor = diskFreeGb < 10 ? NOMADTheme.ERROR : (diskFreeGb < 25 ? NOMADTheme.WARNING : NOMADTheme.SUCCESS);

                // Temperature (use GPU temp as primary indicator)
                var temp = health.GpuTemp > 0 ? health.GpuTemp : health.CpuTemp;
                _lblHealthTemp.Text = $"Temp: {temp:F0}C";
                _lblHealthTemp.ForeColor = temp > 80 ? NOMADTheme.ERROR : (temp > 65 ? NOMADTheme.WARNING : NOMADTheme.SUCCESS);

                // Also update the Jetson card temperature
                _lblJetsonTemp.Text = $"{temp:F0}C";
                _lblJetsonTemp.ForeColor = temp > 80 ? NOMADTheme.ERROR : (temp > 65 ? NOMADTheme.WARNING : NOMADTheme.SUCCESS);
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
                _lteIndicator.BackColor = jetsonHttpConnected ? NOMADTheme.SUCCESS : NOMADTheme.ERROR;
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
                    _radioIndicator.BackColor = radioConnected ? NOMADTheme.SUCCESS : NOMADTheme.ERROR;
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
                    _radioIndicator.BackColor = NOMADTheme.ERROR;
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

                // Only dispose the notification service if WE own it. When it's the
                // plugin-wide Shared instance, the plugin's Exit() handles cleanup so
                // closing this view doesn't kill alerts for the rest of the session.
                if (_notificationService != null && _notificationService != NotificationService.Shared)
                {
                    _notificationService.StopMonitoring();
                    _notificationService.Dispose();
                }
                _notificationService = null;

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
