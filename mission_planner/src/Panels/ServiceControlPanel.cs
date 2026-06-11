// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Service Control Panel for NOMAD - manages Jetson services including:
    /// - MAVLink Router
    /// - MediaMTX (RTSP Server)
    /// - noVNC
    /// - Edge Core
    /// - Isaac ROS Container
    /// - VIO Pipeline Status
    /// </summary>
    public partial class ServiceControlPanel : UserControl
    {
        private readonly DualLinkSender _sender;
        private readonly System.Threading.Timer _pollTimer;
        private readonly int _pollIntervalMs;
        private int _isPolling = 0;
        private int _pollCycle = 0;

        // Track transient failures so we can avoid UI flapping and log spam.
        private int _servicesFailStreak = 0;
        private int _isaacFailStreak = 0;
        private int _vioFailStreak = 0;
        private int _videoFailStreak = 0;
        private int _slamFailStreak = 0;

        // Service status indicators
        private Label _lblMavlinkStatus;
        private Label _lblMediamtxStatus;
        private Label _lblNoVncStatus;
        private Label _lblEdgeCoreStatus;
        private Label _lblIsaacRosStatus;
        private Label _lblVioStatus;

        // Service control buttons
        private Button _btnMavlinkRestart;
        private Button _btnMediamtxRestart;
        private Button _btnEdgeCoreRestart;
        private Button _btnNoVncStart;
        private Button _btnNoVncStop;
        private Button _btnIsaacRosStart;
        private Button _btnIsaacRosStop;

        // VIO trajectory info
        private Label _lblVioTrajectoryPoints;
        private Button _btnClearTrajectory;

        // ROS HTTP Bridge
        private Label _lblRosBridgeStatus;
        private Button _btnStartRosBridge;
        private Button _btnStopRosBridge;

        // Nvblox
        private Label _lblNvbloxStatus;
        private Button _btnNvbloxLaunch;
        private Button _btnNvbloxStop;

        // Video bridges
        private Label _lblVideoBridgesStatus;
        private Button _btnStartBridges;

        // SLAM
        private Label _lblSlamStatus;
        private Button _btnStopSlam;

        // Status text
        private Label _lblLastUpdate;
        private TextBox _txtLog;
        private Panel _servicesPanel;
        private Panel _logPanel;

        // Left service pane layout constants
        private const int ServiceLeftCol = 15;
        private const int ServiceStatusCol = 140;
        private const int ServiceStatusWidth = 280;
        private const int ServiceActionCol = 430;
        private const int ServiceStartCol = 385;
        private const int ServiceStopCol = 460;

        public ServiceControlPanel(DualLinkSender sender, int pollIntervalMs = 3000)
        {
            _sender = sender ?? throw new ArgumentNullException(nameof(sender));
            // Service Control performs multiple network calls per cycle.
            // Clamp to a sane minimum to avoid timeout churn under load.
            _pollIntervalMs = Math.Max(5000, pollIntervalMs);

            InitializeUI();

            // Poll using configured interval
            _pollTimer = new System.Threading.Timer(
                _ => PollServicesAsync(),
                null,
                TimeSpan.FromSeconds(2),
                TimeSpan.FromMilliseconds(_pollIntervalMs)
            );

            LogMessage("Service poller v2 active (stability patch loaded)");
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _pollTimer?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
