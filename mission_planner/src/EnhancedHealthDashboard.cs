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
    public class EnhancedHealthDashboard : UserControl
    {
        // ============================================================
        // Fields
        // ============================================================
        
        private NOMADConfig _config;
        private System.Threading.Timer _pollTimer;
        private int _isPollInFlight = 0; // Interlocked guard — prevents overlapping polls
        
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
        private Button _btnReconnectTailscale;
        
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
        // UI Initialization
        // ============================================================
        
        private void InitializeUI()
        {
            this.BackColor = Color.FromArgb(30, 30, 30);
            this.Dock = DockStyle.Fill;
            this.AutoScroll = true;
            
            var mainPanel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 3,
                BackColor = Color.Transparent,
                Padding = new Padding(10),
            };
            
            mainPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 80));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 280));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            
            // Row 0: Overall Status
            _statusPanel = CreateStatusPanel();
            mainPanel.Controls.Add(_statusPanel, 0, 0);
            mainPanel.SetColumnSpan(_statusPanel, 2);
            
            // Row 1, Col 0: Metrics
            _metricsPanel = CreateMetricsPanel();
            mainPanel.Controls.Add(_metricsPanel, 0, 1);
            
            // Row 1, Col 1: Network/VIO
            _networkPanel = CreateNetworkPanel();
            mainPanel.Controls.Add(_networkPanel, 1, 1);
            
            // Row 2, Col 0: Graph
            _graphPanel = CreateGraphPanel();
            mainPanel.Controls.Add(_graphPanel, 0, 2);
            
            // Row 2, Col 1: Alerts
            _alertsPanel = CreateAlertsPanel();
            mainPanel.Controls.Add(_alertsPanel, 1, 2);
            
            this.Controls.Add(mainPanel);
        }
        
        private Panel CreateStatusPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(5),
                Padding = new Padding(15, 10, 15, 10),
            };
            
            _lblOverallStatus = new Label
            {
                Text = "● CONNECTING...",
                Font = new Font("Segoe UI", 18, FontStyle.Bold),
                ForeColor = Color.Yellow,
                Location = new Point(15, 15),
                AutoSize = true,
            };
            panel.Controls.Add(_lblOverallStatus);
            
            _lblLastUpdate = new Label
            {
                Text = "Last update: Never",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.Gray,
                Location = new Point(15, 50),
                AutoSize = true,
            };
            panel.Controls.Add(_lblLastUpdate);
            
            // Auto-refresh indicator (no manual button needed)
            var lblAutoRefresh = new Label
            {
                Text = $"[AUTO] Refreshing every {_config.HealthPollInterval / 1000.0:0.#}s",
                Location = new Point(400, 28),
                ForeColor = Color.LimeGreen,
                Font = new Font("Segoe UI", 8),
                AutoSize = true,
            };
            panel.Controls.Add(lblAutoRefresh);
            
            // Git status label
            _lblGitStatus = new Label
            {
                Text = "",
                Location = new Point(400, 50),
                ForeColor = Color.Gray,
                Font = new Font("Consolas", 8),
                AutoSize = true,
            };
            panel.Controls.Add(_lblGitStatus);
            
            // Git update button
            _btnGitUpdate = new Button
            {
                Text = "Git Update",
                Location = new Point(250, 20),
                Size = new Size(85, 28),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(70, 100, 150),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            _btnGitUpdate.FlatAppearance.BorderSize = 0;
            _btnGitUpdate.Click += async (s, e) => await TriggerGitUpdate();
            panel.Controls.Add(_btnGitUpdate);
            
            return panel;
        }
        
        private Panel CreateMetricsPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(5),
                Padding = new Padding(10),
            };
            
            var title = new Label
            {
                Text = "System Metrics",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = Color.FromArgb(0, 150, 200),
                Location = new Point(10, 10),
                AutoSize = true,
            };
            panel.Controls.Add(title);
            
            int yOffset = 40;
            
            // CPU Temperature
            CreateMetricRow(panel, "CPU Temp:", ref _lblCpuTemp, ref _prgCpuTemp, ref yOffset, Color.Orange);
            
            // GPU Temperature
            CreateMetricRow(panel, "GPU Temp:", ref _lblGpuTemp, ref _prgGpuTemp, ref yOffset, Color.OrangeRed);
            
            // CPU Load
            CreateMetricRow(panel, "CPU Load:", ref _lblCpuLoad, ref _prgCpuLoad, ref yOffset, Color.DodgerBlue);
            
            // GPU Load
            CreateMetricRow(panel, "GPU Load:", ref _lblGpuLoad, ref _prgGpuLoad, ref yOffset, Color.LimeGreen);
            
            // Memory
            CreateMetricRow(panel, "Memory:", ref _lblMemory, ref _prgMemory, ref yOffset, Color.MediumPurple);
            
            // Disk
            CreateMetricRow(panel, "Disk:", ref _lblDisk, ref _prgDisk, ref yOffset, Color.Goldenrod);
            
            // Power and Fan
            _lblPower = new Label
            {
                Text = "Power: --W",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.LightGray,
                Location = new Point(10, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(_lblPower);
            
            _lblFan = new Label
            {
                Text = "Fan: --%",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.LightGray,
                Location = new Point(150, yOffset),
                AutoSize = true,
            };
            panel.Controls.Add(_lblFan);
            
            return panel;
        }
        
        private void CreateMetricRow(Panel panel, string label, ref Label valueLabel, ref ProgressBar progressBar, ref int yOffset, Color color)
        {
            var lblTitle = new Label
            {
                Text = label,
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.LightGray,
                Location = new Point(10, yOffset),
                Width = 70,
            };
            panel.Controls.Add(lblTitle);
            
            progressBar = new ProgressBar
            {
                Location = new Point(85, yOffset),
                Size = new Size(150, 18),
                Maximum = 100,
                Style = ProgressBarStyle.Continuous,
            };
            panel.Controls.Add(progressBar);
            
            valueLabel = new Label
            {
                Text = "--",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.White,
                Location = new Point(245, yOffset),
                Width = 80,
                TextAlign = ContentAlignment.MiddleRight,
            };
            panel.Controls.Add(valueLabel);
            
            yOffset += 30;
        }
        
        private Panel CreateNetworkPanel()
        {
            // Outer panel is a vertical docked layout that gracefully resizes:
            //   row 0: title strip
            //   row 1: Edge Network group (AutoSize, holds Tailscale + reachability + modem)
            //   row 2: VIO Pipeline group (AutoSize)
            //   row 3: VIO Tilt Drift group (AutoSize)
            //   row 4: percent-100 spacer to absorb slack
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(5),
                Padding = new Padding(10),
                AutoScroll = true,
            };

            var grid = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                ColumnCount = 1,
                BackColor = Color.Transparent,
            };
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));  // title
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));  // Edge Network group
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));  // VIO Pipeline group
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));  // VIO Tilt Drift group

            grid.Controls.Add(BuildTitleStrip(), 0, 0);
            grid.Controls.Add(BuildEdgeNetworkGroup(), 0, 1);
            grid.Controls.Add(BuildVioPipelineGroup(), 0, 2);
            grid.Controls.Add(BuildVioDriftGroup(), 0, 3);

            panel.Controls.Add(grid);
            return panel;
        }

        // --- Network panel: title strip -------------------------------------

        private Control BuildTitleStrip()
        {
            var strip = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                Height = 32,
                ColumnCount = 2,
                RowCount = 1,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 0, 0, 6),
            };
            strip.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            strip.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));

            strip.Controls.Add(new Label
            {
                Text = "Edge Diagnostics",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = Color.FromArgb(100, 200, 100),
                AutoSize = true,
                Margin = new Padding(0, 6, 0, 0),
            }, 0, 0);

            _btnReconnectTailscale = new Button
            {
                Text = "Reconnect",
                Size = new Size(80, 24),
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 8),
                Anchor = AnchorStyles.Right,
                Margin = new Padding(0, 3, 0, 0),
                Cursor = Cursors.Hand,
            };
            _btnReconnectTailscale.FlatAppearance.BorderSize = 0;
            _btnReconnectTailscale.Click += async (s, e) => await TriggerTailscaleReconnect();
            strip.Controls.Add(_btnReconnectTailscale, 1, 0);

            return strip;
        }

        // --- Edge Network group ---------------------------------------------
        // Owns everything related to the link from this Jetson to the GCS:
        // Tailscale (IP + peers, no redundant "connected" line — that's on
        // the Link Status page), reachability, and the cellular modem.

        private GroupBox BuildEdgeNetworkGroup()
        {
            var group = new GroupBox
            {
                Text = "Edge Network",
                ForeColor = Color.FromArgb(120, 200, 255),
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                BackColor = Color.FromArgb(40, 40, 43),
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Padding = new Padding(8, 4, 8, 8),
                Margin = new Padding(0, 0, 0, 6),
                Dock = DockStyle.Top,
            };

            var rows = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                ColumnCount = 2,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 14, 0, 0),
            };
            rows.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 96));
            rows.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));

            _lblTailscaleIP = AddKvRow(rows, "Tailscale:", "--", Color.White, new Font("Consolas", 9));
            _lblPeerCount = AddKvRow(rows, "Peers:", "--", Color.LightGray, new Font("Segoe UI", 9));
            _lblInternetStatus = AddKvRow(rows, "Internet:", "--", Color.Gray, new Font("Segoe UI", 9, FontStyle.Bold));
            _lblGcsReachable = AddKvRow(rows, "GCS reach:", "--", Color.Gray, new Font("Segoe UI", 9, FontStyle.Bold));

            // Modem sub-section header (just a coloured separator label)
            var modemHeader = new Label
            {
                Text = "LTE modem",
                Font = new Font("Segoe UI", 9, FontStyle.Bold | FontStyle.Italic),
                ForeColor = Color.FromArgb(255, 180, 100),
                AutoSize = true,
                Margin = new Padding(0, 8, 0, 2),
            };
            rows.SetColumnSpan(modemHeader, 2);
            rows.Controls.Add(modemHeader, 0, rows.RowCount);
            rows.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            rows.RowCount++;

            _lblModemConnection = AddKvRow(rows, "Profile:", "--", Color.LightGray, new Font("Segoe UI", 9));
            _lblModemStatus = AddKvRow(rows, "Carrier:", "--", Color.Gray, new Font("Segoe UI", 9));
            _lblModemSignal = AddKvRow(rows, "Signal:", "--", Color.Gray, new Font("Consolas", 9));
            _lblModemInterface = AddKvRow(rows, "Bind:", "--", Color.LightGray, new Font("Consolas", 9));

            group.Controls.Add(rows);
            return group;
        }

        /// <summary>
        /// Helper: append a "label : value" row to a 2-column TableLayoutPanel.
        /// Returns the value Label so the caller can mutate it later.
        /// </summary>
        private static Label AddKvRow(TableLayoutPanel host, string key, string value, Color valueColor, Font valueFont)
        {
            host.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            int row = host.RowCount;
            host.RowCount = row + 1;

            var k = new Label
            {
                Text = key,
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.LightGray,
                AutoSize = true,
                Margin = new Padding(0, 2, 6, 2),
            };
            host.Controls.Add(k, 0, row);

            var v = new Label
            {
                Text = value,
                Font = valueFont,
                ForeColor = valueColor,
                AutoSize = true,
                Margin = new Padding(0, 2, 0, 2),
            };
            host.Controls.Add(v, 1, row);
            return v;
        }

        // --- VIO Pipeline group ---------------------------------------------

        private GroupBox BuildVioPipelineGroup()
        {
            var group = new GroupBox
            {
                Text = "VIO Pipeline",
                ForeColor = Color.FromArgb(255, 150, 50),
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                BackColor = Color.FromArgb(40, 40, 43),
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Padding = new Padding(8, 4, 8, 8),
                Margin = new Padding(0, 0, 0, 6),
                Dock = DockStyle.Top,
            };
            _lblVioStatus = new Label
            {
                Text = "Status: Unknown\nConfidence: --\nRate: -- Hz",
                Font = new Font("Consolas", 9),
                ForeColor = Color.White,
                AutoSize = true,
                Margin = new Padding(0, 14, 0, 0),
                Dock = DockStyle.Top,
            };
            group.Controls.Add(_lblVioStatus);
            return group;
        }

        // --- VIO Tilt Drift group -------------------------------------------

        private GroupBox BuildVioDriftGroup()
        {
            var group = new GroupBox
            {
                Text = "VIO Tilt Drift",
                ForeColor = Color.FromArgb(100, 200, 255),
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                BackColor = Color.FromArgb(40, 40, 43),
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Padding = new Padding(8, 4, 8, 8),
                Margin = new Padding(0, 0, 0, 6),
                Dock = DockStyle.Top,
            };

            var grid = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                ColumnCount = 2,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 14, 0, 0),
            };
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));

            _lblDriftCycles = new Label { Text = "Cycles: --", Font = new Font("Consolas", 9), ForeColor = Color.White, AutoSize = true, Margin = new Padding(0, 2, 8, 2) };
            _lblDriftAvg = new Label { Text = "Avg: --", Font = new Font("Consolas", 9), ForeColor = Color.White, AutoSize = true, Margin = new Padding(0, 2, 8, 2) };
            _lblDriftMax = new Label { Text = "Max: --", Font = new Font("Consolas", 9), ForeColor = Color.White, AutoSize = true, Margin = new Padding(0, 2, 8, 2) };
            _lblDriftWarning = new Label { Text = "", Font = new Font("Segoe UI", 9, FontStyle.Bold), ForeColor = Color.Orange, AutoSize = true, Visible = false, Margin = new Padding(0, 2, 0, 2) };

            grid.Controls.Add(_lblDriftCycles, 0, 0);
            grid.Controls.Add(_lblDriftAvg, 1, 0);
            grid.Controls.Add(_lblDriftMax, 0, 1);
            grid.Controls.Add(_lblDriftWarning, 1, 1);
            group.Controls.Add(grid);
            return group;
        }
        
        private Panel CreateGraphPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(5),
                Padding = new Padding(10),
            };
            
            var title = new Label
            {
                Text = "History Graph",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = Color.FromArgb(200, 100, 200),
                Location = new Point(10, 10),
                AutoSize = true,
            };
            panel.Controls.Add(title);
            
            _cmbGraphType = new ComboBox
            {
                Location = new Point(120, 7),
                Size = new Size(130, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
            };
            _cmbGraphType.Items.AddRange(new object[] { "Temperature", "Load", "Memory" });
            _cmbGraphType.SelectedIndex = 0;
            _cmbGraphType.SelectedIndexChanged += (s, e) => DrawGraph();
            panel.Controls.Add(_cmbGraphType);
            
            _graphBox = new PictureBox
            {
                Location = new Point(10, 40),
                Size = new Size(320, 180),
                BackColor = Color.FromArgb(20, 20, 20),
                BorderStyle = BorderStyle.FixedSingle,
            };
            _graphBox.Paint += GraphBox_Paint;
            panel.Controls.Add(_graphBox);
            
            // Resize handling
            panel.Resize += (s, e) =>
            {
                _graphBox.Width = panel.Width - 25;
                _graphBox.Height = panel.Height - 55;
                DrawGraph();
            };
            
            return panel;
        }
        
        private Panel CreateAlertsPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(5),
                Padding = new Padding(10),
            };
            
            var title = new Label
            {
                Text = "[!] Alerts",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = Color.FromArgb(255, 100, 100),
                Location = new Point(10, 10),
                AutoSize = true,
            };
            panel.Controls.Add(title);
            
            var btnClear = new Button
            {
                Text = "Clear",
                Location = new Point(220, 7),
                Size = new Size(60, 25),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
            btnClear.Click += (s, e) =>
            {
                _alerts.Clear();
                _lstAlerts.Items.Clear();
            };
            panel.Controls.Add(btnClear);
            
            _lstAlerts = new ListBox
            {
                Location = new Point(10, 40),
                Size = new Size(300, 180),
                BackColor = Color.FromArgb(20, 20, 20),
                ForeColor = Color.Orange,
                Font = new Font("Consolas", 8),
                BorderStyle = BorderStyle.FixedSingle,
            };
            panel.Controls.Add(_lstAlerts);
            
            // Resize handling
            panel.Resize += (s, e) =>
            {
                _lstAlerts.Width = panel.Width - 25;
                _lstAlerts.Height = panel.Height - 55;
                btnClear.Left = panel.Width - 80;
            };
            
            return panel;
        }
        
        // ============================================================
        // Data Polling
        // ============================================================
        
        private void StartPolling()
        {
            _pollTimer = new System.Threading.Timer(
                _ => PollHealth(),
                null,
                TimeSpan.FromMilliseconds(500),
                TimeSpan.FromMilliseconds(_config.HealthPollInterval)
            );
        }
        
        private async void PollHealth()
        {
            // Prevent overlapping polls — if the previous one is still in-flight, skip.
            if (System.Threading.Interlocked.Exchange(ref _isPollInFlight, 1) == 1)
                return;

            try
            {
                if (IsDisposed || !IsHandleCreated) return;

                // Fire health + network in parallel — both are fast cached reads.
                var healthTask  = JetsonApiService.GetAsync("/health/detailed");
                var networkTask = JetsonApiService.GetAsync("/network/status");

                // Isaac status can be slow (docker introspection inside container).
                // Start it in parallel but don't let it block the health/network update.
                var isaacTask = JetsonApiService.GetAsync("/api/isaac/status");

                // Wait for the fast pair first.
                await Task.WhenAll(healthTask, networkTask);

                if (IsDisposed || !IsHandleCreated) return;

                var healthResponse  = healthTask.Result;
                var networkResponse = networkTask.Result;

                if (!healthResponse.IsSuccessStatusCode)
                {
                    if (!IsDisposed && IsHandleCreated)
                        this.BeginInvoke((Action)(() => UpdateStatusError($"HTTP {healthResponse.StatusCode}")));
                    return;
                }

                var healthJson = await healthResponse.Content.ReadAsStringAsync();
                var healthData = JObject.Parse(healthJson);

                JObject networkData = null;
                if (networkResponse.IsSuccessStatusCode)
                {
                    var networkJson = await networkResponse.Content.ReadAsStringAsync();
                    networkData = JObject.Parse(networkJson);
                }

                // Update health + network immediately — don't wait for isaac.
                if (!IsDisposed && IsHandleCreated)
                    this.BeginInvoke((Action)(() => UpdateUI(healthData, networkData, null)));

                // Now collect isaac result (it may already be done, or we wait a bit more).
                var isaacCompleted = isaacTask.IsCompleted ||
                    await Task.WhenAny(isaacTask, Task.Delay(2000)) == isaacTask;

                if (isaacCompleted && isaacTask.Status == System.Threading.Tasks.TaskStatus.RanToCompletion && isaacTask.Result.IsSuccessStatusCode)
                {
                    var isaacJson = await isaacTask.Result.Content.ReadAsStringAsync();
                    var isaacData = JObject.Parse(isaacJson);
                    if (!IsDisposed && IsHandleCreated)
                        this.BeginInvoke((Action)(() => UpdateDriftStats(isaacData)));
                }
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
            catch (Exception ex)
            {
                try
                {
                    if (!IsDisposed && IsHandleCreated)
                        this.BeginInvoke((Action)(() => UpdateStatusError(ex.Message)));
                }
                catch (ObjectDisposedException) { }
                catch (InvalidOperationException) { }
            }
            finally
            {
                System.Threading.Interlocked.Exchange(ref _isPollInFlight, 0);
            }
        }
        
        public void RefreshHealth()
        {
            PollHealth();
        }
        
        // ============================================================
        // UI Updates
        // ============================================================
        
        private void UpdateUI(JObject data, JObject networkData = null, JObject isaacData = null)
        {
            try
            {
                // Overall Status
                var status = data["status"]?.ToString() ?? "unknown";
                UpdateOverallStatus(status);
                
                // CPU
                var cpuTemp = data["cpu_temp"]?.Value<float>() ?? 0;
                var cpuLoad = data["cpu_load"]?.Value<float>() ?? 0;
                UpdateMetric(_lblCpuTemp, _prgCpuTemp, cpuTemp, "C", 85, 95);
                UpdateMetric(_lblCpuLoad, _prgCpuLoad, cpuLoad, "%", 80, 95);
                
                // GPU
                var gpuTemp = data["gpu_temp"]?.Value<float>() ?? 0;
                var gpuLoad = data["gpu_load"]?.Value<float>() ?? 0;
                UpdateMetric(_lblGpuTemp, _prgGpuTemp, gpuTemp, "C", 85, 95);
                UpdateMetric(_lblGpuLoad, _prgGpuLoad, gpuLoad, "%", 80, 95);
                
                // Memory
                var memUsed = data["memory_used_pct"]?.Value<float>() ?? 0;
                UpdateMetric(_lblMemory, _prgMemory, memUsed, "%", 80, 95);
                
                // Disk
                var diskUsed = data["disk_used_pct"]?.Value<float>() ?? 0;
                UpdateMetric(_lblDisk, _prgDisk, diskUsed, "%", 80, 95);
                
                // Power and Fan
                var power = data["power_draw_w"]?.Value<float>() ?? 0;
                var fan = data["fan_speed_pct"]?.Value<float>() ?? 0;
                _lblPower.Text = $"Power: {power:F1}W";
                _lblFan.Text = $"Fan: {fan:F0}%";
                
                // Network status from /network/status endpoint
                if (networkData != null)
                {
                    UpdateNetworkStatus(networkData);
                }
                else
                {
                    // Fallback to basic tailscale info from /health/detailed.
                    var tsConnected = data["tailscale_connected"]?.Value<bool>() ?? false;
                    var tsIp = data["tailscale_ip"]?.ToString() ?? "--";
                    _lblTailscaleIP.Text = tsConnected ? tsIp : "(daemon down)";
                    _lblTailscaleIP.ForeColor = tsConnected ? Color.White : Color.Orange;
                    _lblPeerCount.Text = "--";
                    _lblInternetStatus.Text = "--";
                    _lblInternetStatus.ForeColor = Color.Gray;
                    _lblGcsReachable.Text = "--";
                    _lblGcsReachable.ForeColor = Color.Gray;
                    _lblModemConnection.Text = "--";
                    _lblModemStatus.Text = "--";
                    _lblModemSignal.Text = "--";
                    _lblModemInterface.Text = "--";
                }
                
                // VIO Status (from /health/detailed or /health)
                var vio = data["vio"];
                if (vio != null && vio.Type != JTokenType.Null)
                {
                    var vioHealth = vio["health"]?.ToString() ?? "unknown";
                    var vioConf = vio["tracking_confidence"]?.Value<float>() ?? 0;
                    var vioRate = vio["message_rate_hz"]?.Value<float>() ?? 0;
                    _lblVioStatus.Text = $"Status: {vioHealth}\nConfidence: {vioConf:F1}\nRate: {vioRate:F0} Hz";
                    _lblVioStatus.ForeColor = vioHealth == "healthy" ? Color.LimeGreen :
                        (vioHealth == "degraded" ? Color.Orange : Color.Red);
                }
                else
                {
                    _lblVioStatus.Text = "Status: No Data\nConfidence: --\nRate: -- Hz";
                    _lblVioStatus.ForeColor = Color.Gray;
                }

                // Update history
                UpdateHistory(cpuTemp, gpuTemp, cpuLoad, gpuLoad, memUsed);
                
                // Timestamp
                _lblLastUpdate.Text = $"Last update: {DateTime.Now:HH:mm:ss}";
                
                // Draw graph
                DrawGraph();

                // VIO Tilt Drift Stats — only update when isaacData provided
                // (poll loop fires it separately after the fast health update)
                if (isaacData != null)
                    UpdateDriftStats(isaacData);

                // Check alerts
                CheckAlerts(cpuTemp, gpuTemp, memUsed, diskUsed);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Health UI error: {ex.Message}");
            }
        }
        
        private void UpdateNetworkStatus(JObject networkData)
        {
            try
            {
                // Tailscale status
                var tailscale = networkData["tailscale"];
                if (tailscale != null && tailscale.Type != JTokenType.Null)
                {
                    var status = tailscale["status"]?.ToString() ?? "unknown";
                    var ip = tailscale["ip"]?.ToString() ?? "--";
                    // peer_count and latency_ms may be JSON null -- use nullable types
                    var peerCount = tailscale["peer_count"]?.Type == JTokenType.Null
                        ? 0 : (tailscale["peer_count"]?.Value<int>() ?? 0);
                    var latency = tailscale["latency_ms"];
                    
                    bool isConnected = status.Equals("connected", StringComparison.OrdinalIgnoreCase);
                    // Tailscale "connected" status is now implied by the Link Status
                    // page showing live LTE traffic, so this panel just shows the IP.
                    // When the daemon is down (or in a weird state) we colour the IP
                    // line so the diagnostic is still surfaced here.
                    _lblTailscaleIP.Text = string.IsNullOrEmpty(ip) || ip == "--"
                        ? (isConnected ? "(no IP)" : status.Replace("_", " "))
                        : ip;
                    _lblTailscaleIP.ForeColor = isConnected
                        ? Color.White
                        : (status == "connecting" ? Color.Yellow : Color.Orange);
                    _lblPeerCount.Text = peerCount > 0 ? peerCount.ToString() : "0";
                    _lblPeerCount.ForeColor = peerCount > 0 ? Color.LightGray : Color.Gray;
                }
                else
                {
                    _lblTailscaleIP.Text = "(daemon down)";
                    _lblTailscaleIP.ForeColor = Color.Orange;
                    _lblPeerCount.Text = "--";
                }

                // Internet reachability
                var internetReachable = networkData["internet_reachable"]?.Value<bool>() ?? false;
                _lblInternetStatus.Text = internetReachable ? "Reachable" : "Unreachable";
                _lblInternetStatus.ForeColor = internetReachable ? Color.LimeGreen : Color.Red;

                // GCS reachability (Jetson → GCS via Tailscale)
                var gcsReachable = networkData["gcs_reachable"]?.Value<bool>() ?? false;
                _lblGcsReachable.Text = gcsReachable ? "Reachable" : "Unreachable";
                _lblGcsReachable.ForeColor = gcsReachable ? Color.LimeGreen : Color.Red;

                // Modem
                var modem = networkData["modem"];
                if (modem != null && modem.Type != JTokenType.Null)
                {
                    UpdateModemFields(modem);
                }
                else
                {
                    _lblModemConnection.Text = "Not detected";
                    _lblModemConnection.ForeColor = Color.Gray;
                    _lblModemStatus.Text = "--";
                    _lblModemStatus.ForeColor = Color.Gray;
                    _lblModemSignal.Text = "--";
                    _lblModemSignal.ForeColor = Color.Gray;
                    _lblModemInterface.Text = "--";
                    _lblModemInterface.ForeColor = Color.Gray;
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Network status error: {ex.Message}");
            }
        }

        /// <summary>
        /// Populate the four LTE-modem rows from the /network/status modem
        /// object. Treats the NM "Profile" row as authoritative for the
        /// data-session state (NOMAD-LTE: activated vs activating vs not
        /// configured), since mmcli sometimes shows "connected" before the
        /// NM connection has finished bringing the interface up.
        /// </summary>
        private void UpdateModemFields(JToken modem)
        {
            var connected = modem["connected"]?.Value<bool>() ?? false;
            var carrier = modem["carrier"]?.ToString() ?? "";
            var technology = modem["technology"]?.ToString() ?? "";
            var signalQuality = modem["signal_quality"]?.ToString() ?? "";
            var signalDbm = modem["signal_strength_dbm"];
            var signalPercent = modem["signal_percent"];
            var nmName = modem["nm_connection_name"]?.ToString() ?? "";
            var nmState = modem["nm_connection_state"]?.ToString() ?? "";
            var iface = modem["interface"]?.ToString() ?? "";
            var ip4 = modem["ip_address"]?.ToString() ?? "";

            // Profile row: "NOMAD-LTE: activated"
            if (!string.IsNullOrEmpty(nmName))
            {
                _lblModemConnection.Text = string.IsNullOrEmpty(nmState)
                    ? nmName
                    : $"{nmName} ({nmState})";
                _lblModemConnection.ForeColor = nmState == "activated"
                    ? Color.LimeGreen
                    : nmState == "activating" ? Color.Yellow : Color.Orange;
            }
            else
            {
                _lblModemConnection.Text = "(no NM profile)";
                _lblModemConnection.ForeColor = Color.Gray;
            }

            // Carrier row
            if (!string.IsNullOrEmpty(carrier))
            {
                _lblModemStatus.Text = string.IsNullOrEmpty(technology)
                    ? carrier
                    : $"{carrier} · {technology}";
                _lblModemStatus.ForeColor = connected ? Color.LimeGreen : Color.Orange;
            }
            else
            {
                _lblModemStatus.Text = connected ? "Connected (no carrier name)" : "Disconnected";
                _lblModemStatus.ForeColor = connected ? Color.LightGreen : Color.Red;
            }

            // Signal row: "-92 dBm · Good · 50%"
            if (signalDbm != null && signalDbm.Type != JTokenType.Null)
            {
                var parts = new System.Collections.Generic.List<string>
                {
                    $"{signalDbm} dBm"
                };
                if (!string.IsNullOrEmpty(signalQuality)) parts.Add(Capitalize(signalQuality));
                if (signalPercent != null && signalPercent.Type != JTokenType.Null)
                    parts.Add($"{signalPercent}%");
                _lblModemSignal.Text = string.Join("  ·  ", parts);
                _lblModemSignal.ForeColor = GetSignalColor(signalQuality);
            }
            else if (!string.IsNullOrEmpty(signalQuality))
            {
                _lblModemSignal.Text = Capitalize(signalQuality);
                _lblModemSignal.ForeColor = GetSignalColor(signalQuality);
            }
            else
            {
                _lblModemSignal.Text = "--";
                _lblModemSignal.ForeColor = Color.Gray;
            }

            // Bind row: "wwan0  10.50.10.2"
            string ifPart = string.IsNullOrEmpty(iface) ? "--" : iface;
            string ipPart = string.IsNullOrEmpty(ip4) ? "(no IP)" : ip4;
            _lblModemInterface.Text = $"{ifPart}  {ipPart}";
            _lblModemInterface.ForeColor = !string.IsNullOrEmpty(ip4) ? Color.White : Color.Orange;
        }

        private static string Capitalize(string s) =>
            string.IsNullOrEmpty(s) ? s : char.ToUpper(s[0]) + s.Substring(1).Replace('_', ' ');

        private Color GetSignalColor(string quality)
        {
            return quality?.ToLower() switch
            {
                "excellent" => Color.LimeGreen,
                "good" => Color.LightGreen,
                "fair" => Color.Yellow,
                "poor" => Color.Orange,
                _ => Color.Gray
            };
        }
        
        private async Task TriggerTailscaleReconnect()
        {
            // Reconnect runs `tailscale down && tailscale up` on the Jetson.
            // It can take 5-10s; keep the user oriented with a clear progress
            // label on the button and a final result message.
            try
            {
                _btnReconnectTailscale.Enabled = false;
                _btnReconnectTailscale.Text = "Restarting…";

                var response = await JetsonApiService.PostAsync("/network/reconnect");
                string detail;
                bool success;

                try
                {
                    var json = await response.Content.ReadAsStringAsync();
                    var data = JObject.Parse(json);
                    success = data["success"]?.Value<bool>() ?? false;
                    detail = data["message"]?.ToString() ?? "(no detail)";
                }
                catch
                {
                    success = false;
                    detail = $"HTTP {(int)response.StatusCode}";
                }

                if (success)
                {
                    AddAlert($"[{DateTime.Now:HH:mm:ss}] Tailscale {detail}");
                    _btnReconnectTailscale.Text = "✓ Reconnected";
                    _btnReconnectTailscale.ForeColor = Color.LimeGreen;
                }
                else
                {
                    AddAlert($"[{DateTime.Now:HH:mm:ss}] Tailscale reconnect failed: {detail}");
                    _btnReconnectTailscale.Text = "✗ Failed";
                    _btnReconnectTailscale.ForeColor = Color.OrangeRed;
                }

                // Revert button text after 3s so it stays useful.
                _ = Task.Delay(3000).ContinueWith(_ =>
                {
                    if (IsDisposed) return;
                    try
                    {
                        BeginInvoke((Action)(() =>
                        {
                            _btnReconnectTailscale.Text = "Reconnect";
                            _btnReconnectTailscale.ForeColor = Color.White;
                        }));
                    }
                    catch { }
                });
            }
            catch (Exception ex)
            {
                AddAlert($"[{DateTime.Now:HH:mm:ss}] Reconnect error: {ex.Message}");
                _btnReconnectTailscale.Text = "Reconnect";
                _btnReconnectTailscale.ForeColor = Color.White;
            }
            finally
            {
                _btnReconnectTailscale.Enabled = true;
            }
        }
        
        private async Task TriggerGitUpdate()
        {
            try
            {
                _btnGitUpdate.Enabled = false;
                _btnGitUpdate.Text = "Updating...";
                _lblGitStatus.Text = "Stashing & pulling...";
                _lblGitStatus.ForeColor = Color.Yellow;
                
                var response = await JetsonApiService.PostLongRunAsync("/api/admin/git-update");
                
                if (response.IsSuccessStatusCode)
                {
                    var json = await response.Content.ReadAsStringAsync();
                    var data = JObject.Parse(json);
                    var success = data["success"]?.Value<bool>() ?? false;
                    
                    if (success)
                    {
                        // Show pull result - find the git pull step manually to avoid LINQ
                        var steps = data["steps"] as JArray;
                        string pullOutput = "Updated";
                        if (steps != null)
                        {
                            foreach (var step in steps)
                            {
                                if (step["step"]?.ToString() == "git pull origin main")
                                {
                                    pullOutput = step["output"]?.ToString() ?? "Updated";
                                    break;
                                }
                            }
                        }
                        
                        _lblGitStatus.Text = TruncateString(pullOutput, 50);
                        _lblGitStatus.ForeColor = Color.LimeGreen;
                        AddAlert($"[{DateTime.Now:HH:mm:ss}] Git update successful");
                    }
                    else
                    {
                        var error = data["error"]?.ToString() ?? "Unknown error";
                        _lblGitStatus.Text = $"Failed: {error}";
                        _lblGitStatus.ForeColor = Color.Red;
                        AddAlert($"[{DateTime.Now:HH:mm:ss}] Git update failed: {error}");
                    }
                }
                else
                {
                    string detail = null;
                    try
                    {
                        var errorJson = await response.Content.ReadAsStringAsync();
                        if (!string.IsNullOrWhiteSpace(errorJson))
                        {
                            var errorData = JObject.Parse(errorJson);
                            detail = errorData["detail"]?.ToString() ?? errorData["error"]?.ToString();
                        }
                    }
                    catch
                    {
                        // Keep generic status when response body is not JSON.
                    }

                    var statusText = $"HTTP {response.StatusCode}";
                    if (!string.IsNullOrWhiteSpace(detail))
                    {
                        statusText += $": {detail}";
                    }

                    _lblGitStatus.Text = TruncateString(statusText, 80);
                    _lblGitStatus.ForeColor = Color.Red;
                    AddAlert($"[{DateTime.Now:HH:mm:ss}] Git update request failed: {statusText}");
                }
            }
            catch (Exception ex)
            {
                _lblGitStatus.Text = $"Error: {ex.Message}";
                _lblGitStatus.ForeColor = Color.Red;
                AddAlert($"[{DateTime.Now:HH:mm:ss}] Git update error: {ex.Message}");
            }
            finally
            {
                _btnGitUpdate.Enabled = true;
                _btnGitUpdate.Text = "Git Update";
            }
        }
        
        private string TruncateString(string value, int maxLength)
        {
            if (string.IsNullOrEmpty(value)) return value;
            return value.Length <= maxLength ? value : value.Substring(0, maxLength - 3) + "...";
        }
        
        private void UpdateDriftStats(JObject isaacData)
        {
            try
            {
                if (isaacData == null)
                {
                    _lblDriftCycles.Text = "Cycles: --";
                    _lblDriftAvg.Text = "Avg: --";
                    _lblDriftMax.Text = "Max: --";
                    _lblDriftWarning.Visible = false;
                    return;
                }

                // Look for tilt_drift in bridge_stats or stats
                var bridgeStats = isaacData["bridge_stats"] ?? isaacData["stats"];
                var tiltDrift = bridgeStats?["tilt_drift"];

                if (tiltDrift == null || tiltDrift.Type == JTokenType.Null)
                {
                    _lblDriftCycles.Text = "Cycles: 0";
                    _lblDriftAvg.Text = "Avg: --";
                    _lblDriftMax.Text = "Max: --";
                    _lblDriftWarning.Visible = false;
                    return;
                }

                var cycles = tiltDrift["cycles"]?.Value<int>() ?? 0;
                var avgDrift = tiltDrift["avg_drift_m"]?.Value<float>() ?? 0f;
                var maxDrift = tiltDrift["max_drift_m"]?.Value<float>() ?? 0f;

                _lblDriftCycles.Text = $"Cycles: {cycles}";
                _lblDriftAvg.Text = $"Avg: {avgDrift * 100f:F1}cm";
                _lblDriftMax.Text = $"Max: {maxDrift * 100f:F1}cm";

                // VO-006: Warn if max drift > 5cm
                if (maxDrift > 0.05f)
                {
                    _lblDriftWarning.Text = "DRIFT HIGH";
                    _lblDriftWarning.ForeColor = Color.Red;
                    _lblDriftWarning.Visible = true;
                }
                else if (cycles > 0)
                {
                    _lblDriftWarning.Text = "OK";
                    _lblDriftWarning.ForeColor = Color.LimeGreen;
                    _lblDriftWarning.Visible = true;
                }
                else
                {
                    _lblDriftWarning.Visible = false;
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Drift stats error: {ex.Message}");
            }
        }

        private void UpdateOverallStatus(string status)
        {
            switch (status.ToLower())
            {
                case "ok":
                    _lblOverallStatus.Text = "● HEALTHY";
                    _lblOverallStatus.ForeColor = Color.LimeGreen;
                    break;
                case "warning":
                    _lblOverallStatus.Text = "● WARNING";
                    _lblOverallStatus.ForeColor = Color.Yellow;
                    break;
                case "critical":
                    _lblOverallStatus.Text = "● CRITICAL";
                    _lblOverallStatus.ForeColor = Color.Red;
                    break;
                default:
                    _lblOverallStatus.Text = "● UNKNOWN";
                    _lblOverallStatus.ForeColor = Color.Gray;
                    break;
            }
        }
        
        private void UpdateMetric(Label label, ProgressBar progress, float value, string unit, float warnThreshold, float critThreshold)
        {
            label.Text = $"{value:F1}{unit}";
            progress.Value = Math.Min(100, Math.Max(0, (int)value));
            
            if (value >= critThreshold)
            {
                label.ForeColor = Color.Red;
            }
            else if (value >= warnThreshold)
            {
                label.ForeColor = Color.Yellow;
            }
            else
            {
                label.ForeColor = Color.LimeGreen;
            }
        }
        
        private void UpdateStatusError(string error)
        {
            _lblOverallStatus.Text = "● OFFLINE";
            _lblOverallStatus.ForeColor = Color.Red;
            _lblLastUpdate.Text = $"Error: {error}";
        }
        
        private void UpdateHistory(float cpuTemp, float gpuTemp, float cpuLoad, float gpuLoad, float memory)
        {
            AddToHistory(_cpuTempHistory, cpuTemp);
            AddToHistory(_gpuTempHistory, gpuTemp);
            AddToHistory(_cpuLoadHistory, cpuLoad);
            AddToHistory(_gpuLoadHistory, gpuLoad);
            AddToHistory(_memoryHistory, memory);
        }
        
        private void AddToHistory(Queue<float> queue, float value)
        {
            queue.Enqueue(value);
            while (queue.Count > HISTORY_LENGTH)
            {
                queue.Dequeue();
            }
        }
        
        // ============================================================
        // Graph Drawing
        // ============================================================
        
        private void DrawGraph()
        {
            _graphBox?.Invalidate();
        }
        
        private void GraphBox_Paint(object sender, PaintEventArgs e)
        {
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;

            var fullRect = _graphBox.ClientRectangle;

            // Background
            g.Clear(Color.FromArgb(20, 20, 20));

            // Margins for axis labels
            int leftMargin = 40;
            int bottomMargin = 25;
            int topMargin = 5;
            int rightMargin = 5;

            // Plot area inside margins
            var rect = new Rectangle(
                leftMargin, topMargin,
                fullRect.Width - leftMargin - rightMargin,
                fullRect.Height - topMargin - bottomMargin);

            // Get data based on selection
            Queue<float> data1, data2;
            Color color1, color2;
            string label1, label2;
            string yAxisLabel;

            switch (_cmbGraphType?.SelectedIndex ?? 0)
            {
                case 0: // Temperature
                    data1 = _cpuTempHistory;
                    data2 = _gpuTempHistory;
                    color1 = Color.Orange;
                    color2 = Color.OrangeRed;
                    label1 = "CPU";
                    label2 = "GPU";
                    yAxisLabel = "°C";
                    break;
                case 1: // Load
                    data1 = _cpuLoadHistory;
                    data2 = _gpuLoadHistory;
                    color1 = Color.DodgerBlue;
                    color2 = Color.LimeGreen;
                    label1 = "CPU";
                    label2 = "GPU";
                    yAxisLabel = "%";
                    break;
                default: // Memory
                    data1 = _memoryHistory;
                    data2 = _memoryHistory;
                    color1 = Color.MediumPurple;
                    color2 = Color.MediumPurple;
                    label1 = "Memory";
                    label2 = "";
                    yAxisLabel = "%";
                    break;
            }

            // Grid lines and Y-axis labels
            using (var gridPen = new Pen(Color.FromArgb(40, 40, 40)))
            using (var axisPen = new Pen(Color.FromArgb(80, 80, 80)))
            using (var axisFont = new Font("Segoe UI", 7))
            {
                for (int i = 0; i <= 4; i++)
                {
                    int y = rect.Top + rect.Height * i / 4;
                    g.DrawLine(gridPen, rect.Left, y, rect.Right, y);

                    int value = 100 - (i * 25);
                    string yText = $"{value}{yAxisLabel}";
                    var textSize = g.MeasureString(yText, axisFont);
                    g.DrawString(yText, axisFont, Brushes.Gray,
                        rect.Left - textSize.Width - 3, y - textSize.Height / 2);
                }

                // X-axis labels (time ago)
                int totalSeconds = HISTORY_LENGTH * (_config.HealthPollInterval / 1000);
                for (int i = 0; i <= 4; i++)
                {
                    int x = rect.Left + rect.Width * i / 4;
                    g.DrawLine(gridPen, x, rect.Top, x, rect.Bottom);

                    int secsAgo = totalSeconds - (totalSeconds * i / 4);
                    string xText = secsAgo == 0 ? "now" : $"-{secsAgo}s";
                    var textSize = g.MeasureString(xText, axisFont);
                    g.DrawString(xText, axisFont, Brushes.Gray,
                        x - textSize.Width / 2, rect.Bottom + 3);
                }

                // Axis border lines
                g.DrawLine(axisPen, rect.Left, rect.Top, rect.Left, rect.Bottom);
                g.DrawLine(axisPen, rect.Left, rect.Bottom, rect.Right, rect.Bottom);
            }

            // Draw lines
            DrawGraphLine(g, rect, data1, color1, 100);
            if (_cmbGraphType?.SelectedIndex != 2)
            {
                DrawGraphLine(g, rect, data2, color2, 100);
            }

            // Legend
            using (var brush1 = new SolidBrush(color1))
            using (var brush2 = new SolidBrush(color2))
            using (var font = new Font("Segoe UI", 8))
            {
                g.FillRectangle(brush1, fullRect.Width - 80, 5, 10, 10);
                g.DrawString(label1, font, Brushes.White, fullRect.Width - 65, 3);

                if (!string.IsNullOrEmpty(label2))
                {
                    g.FillRectangle(brush2, fullRect.Width - 80, 20, 10, 10);
                    g.DrawString(label2, font, Brushes.White, fullRect.Width - 65, 18);
                }
            }
        }

        private void DrawGraphLine(Graphics g, Rectangle rect, Queue<float> data, Color color, float maxValue)
        {
            if (data.Count < 2) return;

            var values = data.ToArray();
            var points = new PointF[values.Length];

            for (int i = 0; i < values.Length; i++)
            {
                float x = rect.Left + rect.Width * i / (float)(HISTORY_LENGTH - 1);
                float y = rect.Top + rect.Height - (rect.Height * values[i] / maxValue);
                points[i] = new PointF(x, Math.Max(rect.Top, Math.Min(rect.Bottom, y)));
            }

            using (var pen = new Pen(color, 2))
            {
                g.DrawLines(pen, points);
            }
        }
        
        // ============================================================
        // Alerts
        // ============================================================
        
        private void CheckAlerts(float cpuTemp, float gpuTemp, float memory, float disk)
        {
            var timestamp = DateTime.Now.ToString("HH:mm:ss");
            
            if (cpuTemp > _config.TempCriticalC)
            {
                AddAlert($"[{timestamp}] CRITICAL: CPU Temp {cpuTemp:F1}°C");
            }
            else if (cpuTemp > _config.TempWarningC)
            {
                AddAlert($"[{timestamp}] WARNING: CPU Temp {cpuTemp:F1}°C");
            }
            
            if (gpuTemp > _config.TempCriticalC)
            {
                AddAlert($"[{timestamp}] CRITICAL: GPU Temp {gpuTemp:F1}°C");
            }
            else if (gpuTemp > _config.TempWarningC)
            {
                AddAlert($"[{timestamp}] WARNING: GPU Temp {gpuTemp:F1}°C");
            }
            
            if (memory > 95)
            {
                AddAlert($"[{timestamp}] CRITICAL: Memory at {memory:F0}%");
            }
            
            if (disk > 95)
            {
                AddAlert($"[{timestamp}] WARNING: Disk at {disk:F0}%");
            }
        }
        
        private void AddAlert(string alert)
        {
            // Avoid duplicate consecutive alerts
            if (_alerts.Count > 0 && _alerts[_alerts.Count - 1].Contains(alert.Substring(alert.IndexOf(']') + 1)))
                return;
            
            _alerts.Add(alert);
            _lstAlerts.Items.Add(alert);
            
            // Keep only last 100 alerts
            while (_alerts.Count > 100)
            {
                _alerts.RemoveAt(0);
                _lstAlerts.Items.RemoveAt(0);
            }
            
            // Scroll to bottom
            _lstAlerts.TopIndex = _lstAlerts.Items.Count - 1;
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
