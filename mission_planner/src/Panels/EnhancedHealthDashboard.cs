// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Enhanced Health Dashboard
// ============================================================
// Provides comprehensive system health monitoring for the Jetson
// Orin Nano with real-time graphs and alerts.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Net.Http;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Enhanced health dashboard with graphs and detailed monitoring.
    /// </summary>
    public partial class EnhancedHealthDashboard : UserControl
    {
        // ============================================================
        // Fields
        // ============================================================

        private NOMADConfig _config;
        private System.Threading.Timer _pollTimer;
        private int _isPollInFlight = 0; // Interlocked guard — prevents overlapping polls
        private DateTime _lastHealthSuccessUtc = DateTime.MinValue;
        private DateTime _lastIsaacPollUtc = DateTime.MinValue;
        private static readonly TimeSpan OfflineGrace = TimeSpan.FromSeconds(15);
        private static readonly TimeSpan OptionalEndpointBudget = TimeSpan.FromMilliseconds(1200);
        private static readonly TimeSpan IsaacPollInterval = TimeSpan.FromSeconds(15);

        // Data history for graphs
        private readonly Queue<float> _cpuTempHistory = new Queue<float>();
        private readonly Queue<float> _gpuTempHistory = new Queue<float>();
        private readonly Queue<float> _cpuLoadHistory = new Queue<float>();
        private readonly Queue<float> _gpuLoadHistory = new Queue<float>();
        private readonly Queue<float> _memoryHistory = new Queue<float>();
        private const int HISTORY_LENGTH = 60; // 2 minutes at 2s interval

        // UI Controls
        private Panel _statusPanel;
        private Panel _metricsPanel;
        private Panel _graphPanel;
        private Panel _networkPanel;
        private Panel _alertsPanel;

        // Status indicators
        private Label _lblOverallStatus;
        private Label _lblLastUpdate;

        // Git update controls
        private Button _btnGitUpdate;
        private Label _lblGitStatus;

        // Metric labels
        private Label _lblCpuTemp, _lblCpuLoad;
        private Label _lblGpuTemp, _lblGpuLoad;
        private Label _lblMemory, _lblDisk;
        private Label _lblPower, _lblFan;

        // Progress bars
        private ProgressBar _prgCpuTemp, _prgCpuLoad;
        private ProgressBar _prgGpuTemp, _prgGpuLoad;
        private ProgressBar _prgMemory, _prgDisk;

        // Network labels
        private Label _lblTailscaleIP;
        private Label _lblVioStatus;

        // Extended network labels (from /network/status)
        private Label _lblInternetStatus;
        private Label _lblGcsReachable;
        private Label _lblModemStatus;
        private Label _lblModemSignal;
        private Label _lblModemConnection;   // "NOMAD-LTE: activated"
        private Label _lblModemInterface;    // "wwan0  10.50.10.2"
        private Label _lblPeerCount;

        // Graph panel for drawing
        private PictureBox _graphBox;
        private ComboBox _cmbGraphType;

        // Alerts
        private ListBox _lstAlerts;
        private readonly List<string> _alerts = new List<string>();

        // VIO Drift Stats (VO-006)
        private Label _lblDriftCycles;
        private Label _lblDriftAvg;
        private Label _lblDriftMax;
        private Label _lblDriftWarning;

        // ============================================================
        // Constructor
        // ============================================================

        public EnhancedHealthDashboard(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));

            InitializeUI();
            StartPolling();
        }

        // ============================================================
        // Public Methods
        // ============================================================

        public void UpdateConfig(NOMADConfig config)
        {
            _config = config;
        }

        // ============================================================
        // Cleanup
        // ============================================================

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
