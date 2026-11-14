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
using System.Windows.Media;
using TextBlock = System.Windows.Controls.TextBlock;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADDashboardView : UserControl, IUpdatableView
    {
        private readonly MAVLinkConnectionManager _connectionManager;
        private readonly NOMADConfig _config;
        private readonly System.Threading.CancellationToken _videoShutdown;
        private readonly Func<IVideoPipeline> _createPipeline;
        private readonly bool _ownsNotificationService;

        private TextBlock _lblFlightMode;
        private TextBlock _lblGpsFix;
        private TextBlock _lblBattery;
        private TextBlock _lblLinks;
        private TextBlock _lblGimbalCommand;

        private Panel _videoPreviewPanel;
        private VideoInputPlayer _videoHost;
        private TerminationWidget _terminationWidget;

        private NotificationService _notificationService;
        private NotificationPanel _notificationPanel;

        public NotificationService NotificationService => _notificationService;

        public NOMADDashboardView(NOMADConfig config, MAVLinkConnectionManager connectionManager,
            System.Threading.CancellationToken videoShutdown)
            : this(config, connectionManager, videoShutdown, MediaWorkerPipeline.Create) { }

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
                _lblFlightMode.Foreground = Brushes.OrangeRed;
                _lblGpsFix.Text = "No telemetry";
                _lblGpsFix.Foreground = WpfTheme.SecondaryText;
                _lblBattery.Text = "--.- V";
                _lblBattery.Foreground = WpfTheme.SecondaryText;
                return;
            }

            _lblFlightMode.Text = cs.armed ? $"{cs.mode} · ARMED" : (cs.mode ?? "UNKNOWN");
            _lblFlightMode.Foreground = cs.armed ? Brushes.Orange : WpfTheme.Text;

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
            _lblGpsFix.Foreground = gpsFix >= 3
                ? Brushes.LimeGreen
                : (gpsFix >= 1 ? Brushes.Orange : Brushes.OrangeRed);

            UpdateBatteryCard(BatteryHealth.Read(mav, 1));
        }

        private void UpdateBatteryCard(BatteryStatus battery)
        {
            if (battery == null)
            {
                _lblBattery.Text = "Battery unavailable";
                _lblBattery.Foreground = WpfTheme.SecondaryText;
                return;
            }

            _lblBattery.Text = battery.HasRemainingCapacity
                ? $"{battery.Voltage:F1}V · {battery.RemainingMah:F0} mAh"
                : $"{battery.Voltage:F1}V";
            _lblBattery.Foreground = battery.Severity == 2
                ? Brushes.OrangeRed
                : (battery.Severity == 1 ? Brushes.Orange :
                    (battery.ThresholdsFromVehicle ? Brushes.LimeGreen : WpfTheme.SecondaryText));
        }

        private void UpdateLinksCard()
        {
            if (_connectionManager == null)
            {
                _lblLinks.Text = "Router unavailable";
                _lblLinks.Foreground = WpfTheme.SecondaryText;
                return;
            }

            _lblLinks.Text = _connectionManager.GetStatusSummary();
            // Retained active IDs can outlive fresh management data; keep this summary neutral.
            _lblLinks.Foreground = WpfTheme.Text;
        }

        private void UpdateGimbalCard()
        {
            _lblGimbalCommand.Text = GimbalController.LastCommandStatus;
            _lblGimbalCommand.Foreground = WpfTheme.SecondaryText;
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
