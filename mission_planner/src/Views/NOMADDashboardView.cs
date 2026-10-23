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
        private Label _lblLinks;
        private Label _lblGimbalCommand;

        private Panel _videoPreviewPanel;
        private VideoInputPlayer _videoHost;
        private TerminationWidget _terminationWidget;

        private NotificationService _notificationService;
        private NotificationPanel _notificationPanel;

        public NotificationService NotificationService => _notificationService;

        public NOMADDashboardView(NOMADConfig config, MAVLinkConnectionManager connectionManager,
            System.Threading.CancellationToken videoShutdown)
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
        }

        internal void RefreshVideoInput() { _videoHost?.RefreshInput(); }

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
                var mav = MainV2.comPort?.MAV;
                UpdateFlightCards(mav);
                UpdateLinksCard();
                UpdateGimbalCard();
                RefreshVideoInput();
                _terminationWidget?.RefreshStatus();
            }
            catch
            {
            }
        }

        private void UpdateFlightCards(MAVState mav)
        {
            var cs = mav?.cs;
            bool connected = cs?.connected == true;
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

            UpdateBatteryCard(BatteryHealth.Read(mav, 1));
        }

        private void UpdateBatteryCard(BatteryStatus battery)
        {
            if (battery == null)
            {
                _lblBattery.Text = "Battery unavailable";
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

        private void UpdateLinksCard()
        {
            if (_connectionManager == null)
            {
                _lblLinks.Text = "Router unavailable";
                _lblLinks.ForeColor = NOMADTheme.TEXT_SECONDARY;
                return;
            }

            _lblLinks.Text = _connectionManager.GetStatusSummary();
            // Retained active IDs can outlive fresh management data; keep this summary neutral.
            _lblLinks.ForeColor = NOMADTheme.TEXT_PRIMARY;
        }

        private void UpdateGimbalCard()
        {
            _lblGimbalCommand.Text = GimbalController.LastCommandStatus;
            _lblGimbalCommand.ForeColor = NOMADTheme.TEXT_SECONDARY;
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
                _videoHost?.Dispose();
                _terminationWidget?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
