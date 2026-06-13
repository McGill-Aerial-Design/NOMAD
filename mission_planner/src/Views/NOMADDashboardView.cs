// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Dashboard View - Main Overview Panel
// ============================================================
// Compact, information-dense dashboard for the operator:
// - Flight mode / GPS / battery status cards
// - Geofence, dual-link and Jetson health summaries
// - Notification feed beside a small auto-playing video preview
// ============================================================

using System;
using System.Drawing;
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
        // Fields
        // ============================================================

        private readonly DualLinkSender _sender;
        private readonly MAVLinkConnectionManager _connectionManager;
        private readonly JetsonConnectionManager _jetsonConnectionManager;
        private NOMADConfig _config;
        private System.Threading.Timer _healthPollTimer;

        // Status card value labels
        private Label _lblFlightMode;
        private Label _lblGpsFix;
        private Label _lblBattery;
        private Label _lblGeofence;
        private Label _lblLinks;
        private Label _lblJetson;

        // Mini video panel
        private Panel _videoPreviewPanel;
        private Panel _videoPlaceholder;
        private Label _lblVideoStatus;
        private EmbeddedVideoPlayer _videoPlayer;
        private bool _jetsonOnline;
        private bool _videoInitialized;

        // Geofence status source (plugin-owned monitor)
        private BoundaryMonitor _boundaryMonitor;

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
        /// Sets the boundary monitor for boundary violation notifications and
        /// the geofence status card.
        /// </summary>
        public void SetBoundaryMonitor(BoundaryMonitor monitor)
        {
            _boundaryMonitor = monitor;
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
        /// Initializes the chrome-less auto-playing preview when online.
        /// </summary>
        private void InitializeVideoIfOnline()
        {
            if (_videoInitialized || !_jetsonOnline)
                return;

            try
            {
                _videoPlaceholder.Controls.Clear();

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
                    Font = new Font("Segoe UI", 9),
                    ForeColor = NOMADTheme.TEXT_MUTED,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                    BackColor = Color.Black,
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
                var cs = MainV2.comPort?.MAV?.cs;

                // Flight mode (DISCONNECTED stands in for the old connection card)
                bool connected = cs?.connected ?? false;
                if (!connected)
                {
                    _lblFlightMode.Text = "DISCONNECTED";
                    _lblFlightMode.ForeColor = NOMADTheme.ERROR;
                }
                else
                {
                    _lblFlightMode.Text = cs.armed ? $"{cs.mode} · ARMED" : (cs.mode ?? "UNKNOWN");
                    _lblFlightMode.ForeColor = cs.armed ? NOMADTheme.WARNING : NOMADTheme.TEXT_PRIMARY;
                }

                if (cs != null)
                {
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

                    // Battery: voltage + remaining mAh, judged against the
                    // vehicle's own BATTn_* thresholds (no percentage).
                    var batt = BatteryHealth.Read(1);
                    if (batt != null)
                    {
                        _lblBattery.Text = batt.CapacityMah > 0
                            ? $"{batt.Voltage:F1}V · {batt.RemainingMah:F0} mAh"
                            : $"{batt.Voltage:F1}V";
                        _lblBattery.ForeColor = batt.Severity == 2 ? NOMADTheme.ERROR
                            : (batt.Severity == 1 ? NOMADTheme.WARNING : NOMADTheme.SUCCESS);
                    }
                    else
                    {
                        _lblBattery.Text = $"{cs.battery_voltage:F1}V";
                        _lblBattery.ForeColor = NOMADTheme.TEXT_SECONDARY;
                    }
                }

                UpdateGeofenceCard();

                // Jetson online tracking + video init
                bool jetsonOnline = _sender?.IsJetsonConnected ?? false;
                if (jetsonOnline && !_jetsonOnline)
                {
                    _jetsonOnline = true;
                    InitializeVideoIfOnline();
                }
                else if (!jetsonOnline && _jetsonOnline)
                {
                    _jetsonOnline = false;
                }
                UpdateJetsonCard(jetsonOnline);

                UpdateLinksCard(jetsonOnline);
            }
            catch
            {
                // Ignore update errors
            }
        }

        /// <summary>Geofence card: monitoring state + containment status.</summary>
        private void UpdateGeofenceCard()
        {
            if (_boundaryMonitor == null)
            {
                _lblGeofence.Text = "No monitor";
                _lblGeofence.ForeColor = NOMADTheme.TEXT_MUTED;
                return;
            }

            if (!_boundaryMonitor.IsMonitoring)
            {
                _lblGeofence.Text = "Monitor OFF";
                _lblGeofence.ForeColor = NOMADTheme.TEXT_SECONDARY;
                return;
            }

            switch (_boundaryMonitor.CurrentStatus)
            {
                case "inside":
                    _lblGeofence.Text = "INSIDE";
                    _lblGeofence.ForeColor = NOMADTheme.SUCCESS;
                    break;
                case "soft_violation":
                    _lblGeofence.Text = "SOFT VIOLATION";
                    _lblGeofence.ForeColor = NOMADTheme.WARNING;
                    break;
                case "hard_violation":
                    _lblGeofence.Text = _boundaryMonitor.KillCountdown.HasValue
                        ? $"HARD — {_boundaryMonitor.KillCountdown}s"
                        : "HARD VIOLATION";
                    _lblGeofence.ForeColor = NOMADTheme.ERROR;
                    break;
                default:
                    _lblGeofence.Text = "Waiting for GPS";
                    _lblGeofence.ForeColor = NOMADTheme.WARNING;
                    break;
            }
        }

        /// <summary>Jetson card: online/offline + temperature and load in one line.</summary>
        private void UpdateJetsonCard(bool online)
        {
            if (!online)
            {
                _lblJetson.Text = "Offline";
                _lblJetson.ForeColor = NOMADTheme.ERROR;
                return;
            }

            try
            {
                var health = _sender?.LastHealthStatus;
                if (health == null)
                {
                    _lblJetson.Text = "Online";
                    _lblJetson.ForeColor = NOMADTheme.SUCCESS;
                    return;
                }

                var temp = health.GpuTemp > 0 ? health.GpuTemp : health.CpuTemp;
                _lblJetson.Text = $"{temp:F0}°C · CPU {health.CpuUsage:F0}% · GPU {health.GpuUsage:F0}%\nMem {health.MemoryUsed:F0}% · Disk {health.DiskFreeGb:F0} GB free";

                bool bad = temp > 80 || health.CpuUsage > 90 || health.GpuUsage > 90 || health.MemoryUsed > 90 || health.DiskFreeGb < 10;
                bool warn = temp > 65 || health.CpuUsage > 70 || health.GpuUsage > 70 || health.MemoryUsed > 75 || health.DiskFreeGb < 25;
                _lblJetson.ForeColor = bad ? NOMADTheme.ERROR : (warn ? NOMADTheme.WARNING : NOMADTheme.SUCCESS);
            }
            catch
            {
                // Ignore health update errors
            }
        }

        /// <summary>Links card: LTE/Tailscale + RadioMaster + active link.</summary>
        private void UpdateLinksCard(bool jetsonHttpConnected)
        {
            try
            {
                string lte = jetsonHttpConnected ? "LTE ✓" : "LTE ✗";
                string radio;
                string active;

                if (_connectionManager != null)
                {
                    var status = _connectionManager.GetLinkStatus();
                    radio = status.RadioConnected ? $"Radio ✓ {status.RadioLatencyMs}ms" : "Radio ✗";
                    active = jetsonHttpConnected ? "Tailscale" : (status.RadioConnected ? "RadioMaster" : "none");
                }
                else
                {
                    radio = "Radio —";
                    active = jetsonHttpConnected ? "Tailscale" : "none";
                }

                _lblLinks.Text = $"{lte} · {radio}\nActive: {active}";
                _lblLinks.ForeColor = jetsonHttpConnected ? NOMADTheme.SUCCESS
                    : (radio.Contains("✓") ? NOMADTheme.WARNING : NOMADTheme.ERROR);
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
