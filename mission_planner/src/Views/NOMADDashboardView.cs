// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Dashboard View - Main Overview Panel
// ============================================================
// Compact operator dashboard for flight state, safety, links, notifications,
// and the configured direct RTSP preview.
// ============================================================

using System;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADDashboardView : UserControl, IUpdatableView
    {
        private readonly MAVLinkConnectionManager _connectionManager;
        private readonly NOMADConfig _config;
        private readonly System.Threading.CancellationToken _videoShutdown;
        private readonly Func<IVideoPipeline> _createPipeline;
        private readonly bool _ownsNotificationService;

        private Label _lblFlightMode;
        private Label _lblGpsFix;
        private Label _lblBattery;
        private Label _lblGeofence;
        private Label _lblLinks;
        private Label _lblCore;

        private Panel _videoPreviewPanel;
        private Panel _videoPlaceholder;
        private Label _lblVideoStatus;
        private EmbeddedVideoPlayer _videoPlayer;

        private BoundaryMonitor _boundaryMonitor;
        private NotificationService _notificationService;
        private NotificationPanel _notificationPanel;

        public NotificationService NotificationService => _notificationService;

        public void SetBoundaryMonitor(BoundaryMonitor monitor)
        {
            _boundaryMonitor = monitor;
            _notificationService?.SetBoundaryMonitor(monitor);
        }

        public NOMADDashboardView(NOMADConfig config, MAVLinkConnectionManager connectionManager = null,
            System.Threading.CancellationToken videoShutdown = default)
            : this(config, connectionManager, videoShutdown, () => new GStreamerVideoPipeline()) { }

        internal NOMADDashboardView(NOMADConfig config, MAVLinkConnectionManager connectionManager,
            System.Threading.CancellationToken videoShutdown, Func<IVideoPipeline> createPipeline)
        {
            _videoShutdown = videoShutdown;
            _createPipeline = createPipeline;
            _config = config ?? new NOMADConfig();
            _connectionManager = connectionManager;
            _notificationService = NotificationService.Shared;
            _ownsNotificationService = _notificationService == null;
            if (_notificationService == null)
            {
                _notificationService = new NotificationService();
                _notificationService.StartMonitoring();
            }

            InitializeUI();
            InitializeVideoIfConfigured();
        }

        private void InitializeVideoIfConfigured()
        {
            if (string.IsNullOrWhiteSpace(_config.VideoUrl))
            {
                return;
            }

            try
            {
                _videoPlayer = new EmbeddedVideoPlayer("Video Feed", _config.VideoUrl, false,
                    _videoShutdown, _createPipeline)
                {
                    Dock = DockStyle.Fill,
                };
                _videoPlaceholder.Controls.Add(_videoPlayer);
                _lblVideoStatus.Visible = false;
                _videoPlayer.BringToFront();
                _lblVideoStatus.Text = "Video: connecting";
            }
            catch (Exception ex)
            {
                _videoPlayer?.Dispose();
                _videoPlayer = null;
                _lblVideoStatus.Visible = true;
                _lblVideoStatus.Text = $"Video unavailable: {ex.Message}";
                _lblVideoStatus.ForeColor = NOMADTheme.ERROR;
            }
        }

        public void UpdateData()
        {
            if (IsDisposed || !IsHandleCreated)
                return;
            UiAsync.RunSync(this, UpdateDataCore, "UpdateData");
        }

        private void UpdateDataCore()
        {
            try
            {
                var cs = MainV2.comPort?.MAV?.cs;
                UpdateFlightCards(cs);
                UpdateGeofenceCard();
                UpdateLinksCard();
                UpdateCoreCard();
            }
            catch
            {
            }
        }

        private void UpdateFlightCards(dynamic cs)
        {
            bool connected = cs?.connected ?? false;
            if (!connected)
            {
                _lblFlightMode.Text = "DISCONNECTED";
                _lblFlightMode.ForeColor = NOMADTheme.ERROR;
                _lblGpsFix.Text = "No telemetry";
                _lblGpsFix.ForeColor = NOMADTheme.TEXT_SECONDARY;
                _lblBattery.Text = "--.- V";
                _lblBattery.ForeColor = NOMADTheme.TEXT_SECONDARY;
                return;
            }

            _lblFlightMode.Text = cs.armed ? $"{cs.mode} · ARMED" : (cs.mode ?? "UNKNOWN");
            _lblFlightMode.ForeColor = cs.armed ? NOMADTheme.WARNING : NOMADTheme.TEXT_PRIMARY;

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
                _ => "Unknown",
            };
            _lblGpsFix.Text = $"{gpsText} ({cs.satcount} sats)";
            _lblGpsFix.ForeColor = gpsFix >= 3
                ? NOMADTheme.SUCCESS
                : (gpsFix >= 1 ? NOMADTheme.WARNING : NOMADTheme.ERROR);

            var battery = BatteryHealth.Read(1);
            if (battery == null)
            {
                _lblBattery.Text = $"{cs.battery_voltage:F1}V";
                _lblBattery.ForeColor = NOMADTheme.TEXT_SECONDARY;
                return;
            }

            _lblBattery.Text = battery.CapacityMah > 0
                ? $"{battery.Voltage:F1}V · {battery.RemainingMah:F0} mAh"
                : $"{battery.Voltage:F1}V";
            _lblBattery.ForeColor = battery.Severity == 2
                ? NOMADTheme.ERROR
                : (battery.Severity == 1 ? NOMADTheme.WARNING : NOMADTheme.SUCCESS);
        }

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
                    _lblGeofence.Text = "HARD — TERMINATION UNAVAILABLE";
                    _lblGeofence.ForeColor = NOMADTheme.ERROR;
                    break;
                default:
                    _lblGeofence.Text = "Waiting for GPS";
                    _lblGeofence.ForeColor = NOMADTheme.WARNING;
                    break;
            }
        }

        private void UpdateLinksCard()
        {
            if (_connectionManager == null)
            {
                _lblLinks.Text = "Direct MAVLink";
                _lblLinks.ForeColor = NOMADTheme.TEXT_SECONDARY;
                return;
            }

            var status = _connectionManager.GetLinkStatus();
            _lblLinks.Text = _connectionManager.GetStatusSummary();
            _lblLinks.ForeColor = status.ActiveLink == LinkType.None.ToString()
                ? NOMADTheme.ERROR
                : NOMADTheme.SUCCESS;
        }

        private void UpdateCoreCard()
        {
            bool configured = _config.CoreRuntimePort >= 1 && _config.CoreRuntimePort <= 65535
                && !string.IsNullOrWhiteSpace(_config.CoreClientCredential);
            _lblCore.Text = configured ? "IPC configured" : "Not configured";
            _lblCore.ForeColor = configured ? NOMADTheme.SUCCESS : NOMADTheme.WARNING;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                if (_ownsNotificationService && _notificationService != null)
                {
                    _notificationService.StopMonitoring();
                    _notificationService.Dispose();
                }
                _notificationService = null;
                _videoPlayer?.Dispose();
                _videoPlayer = null;
            }
            base.Dispose(disposing);
        }
    }
}
