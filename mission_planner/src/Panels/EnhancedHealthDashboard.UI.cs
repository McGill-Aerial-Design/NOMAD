// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Enhanced Health Dashboard — UI
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class EnhancedHealthDashboard
    {
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
                Padding = new Padding(15, 8, 15, 8),
            };

            var table = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 2,
                BackColor = Color.Transparent,
                Margin = new Padding(0),
                Padding = new Padding(0),
            };
            table.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            table.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            table.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            table.RowStyles.Add(new RowStyle(SizeType.AutoSize));

            _lblOverallStatus = new Label
            {
                Text = "\u25cf CONNECTING...",
                Font = NOMADTheme.Font(18, FontStyle.Bold),
                ForeColor = Color.Yellow,
                AutoSize = true,
                Anchor = AnchorStyles.Left,
            };
            table.Controls.Add(_lblOverallStatus, 0, 0);

            // Auto-refresh indicator (no manual button needed)
            table.Controls.Add(new Label
            {
                Text = $"[AUTO] Refreshing every {_config.HealthPollInterval / 1000.0:0.#}s",
                ForeColor = Color.LimeGreen,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
                AutoSize = true,
                Anchor = AnchorStyles.Right,
                Margin = new Padding(NOMADTheme.PAD, 8, 0, 0),
            }, 1, 0);

            _lblLastUpdate = new Label
            {
                Text = "Last update: Never",
                Font = NOMADTheme.Font(),
                ForeColor = Color.Gray,
                AutoSize = true,
                Margin = new Padding(0, 2, 0, 0),
            };
            table.Controls.Add(_lblLastUpdate, 0, 1);
            table.SetColumnSpan(_lblLastUpdate, 2);

            panel.Controls.Add(table);
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

            // 3-col table: label | progress bar (stretches) | value (right). Rows
            // AutoSize so the value never clips at narrow widths.
            var table = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                ColumnCount = 3,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                BackColor = Color.Transparent,
            };
            table.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 78));
            table.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            table.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 86));

            var title = new Label
            {
                Text = "System Metrics",
                Font = NOMADTheme.Font(NOMADTheme.SIZE_HEADING, FontStyle.Bold),
                ForeColor = Color.FromArgb(0, 150, 200),
                AutoSize = true,
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
            };
            table.Controls.Add(title, 0, 0);
            table.SetColumnSpan(title, 3);

            MetricRow(table, "CPU Temp:", ref _lblCpuTemp, ref _prgCpuTemp);
            MetricRow(table, "GPU Temp:", ref _lblGpuTemp, ref _prgGpuTemp);
            MetricRow(table, "CPU Load:", ref _lblCpuLoad, ref _prgCpuLoad);
            MetricRow(table, "GPU Load:", ref _lblGpuLoad, ref _prgGpuLoad);
            MetricRow(table, "Memory:", ref _lblMemory, ref _prgMemory);
            MetricRow(table, "Disk:", ref _lblDisk, ref _prgDisk);

            // Power and Fan share one row.
            _lblPower = new Label { Text = "Power: --W", Font = NOMADTheme.Font(), ForeColor = Color.LightGray, AutoSize = true, Margin = new Padding(0, 6, NOMADTheme.PAD, 0) };
            _lblFan = new Label { Text = "Fan: --%", Font = NOMADTheme.Font(), ForeColor = Color.LightGray, AutoSize = true, Margin = new Padding(0, 6, 0, 0) };
            var powerFan = new FlowLayoutPanel { AutoSize = true, AutoSizeMode = AutoSizeMode.GrowAndShrink, WrapContents = true, BackColor = Color.Transparent, Margin = new Padding(0), Padding = new Padding(0) };
            powerFan.Controls.Add(_lblPower);
            powerFan.Controls.Add(_lblFan);
            int r = table.RowCount;
            table.RowCount = r + 1;
            table.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            table.Controls.Add(powerFan, 0, r);
            table.SetColumnSpan(powerFan, 3);

            panel.Controls.Add(table);
            return panel;
        }

        private void MetricRow(TableLayoutPanel table, string label, ref Label valueLabel, ref ProgressBar progressBar)
        {
            int r = table.RowCount;
            table.RowCount = r + 1;
            table.RowStyles.Add(new RowStyle(SizeType.AutoSize));

            table.Controls.Add(new Label
            {
                Text = label,
                Font = NOMADTheme.Font(),
                ForeColor = Color.LightGray,
                AutoSize = true,
                Anchor = AnchorStyles.Left,
                Margin = new Padding(0, 4, NOMADTheme.GAP, 0),
            }, 0, r);

            progressBar = new ProgressBar
            {
                Dock = DockStyle.Fill,
                Height = 18,
                Maximum = 100,
                Style = ProgressBarStyle.Continuous,
                Margin = new Padding(0, 2, 0, 2),
            };
            table.Controls.Add(progressBar, 1, r);

            valueLabel = new Label
            {
                Text = "--",
                Font = NOMADTheme.Font(),
                ForeColor = Color.White,
                AutoSize = false,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleRight,
                Margin = new Padding(NOMADTheme.GAP, 2, 0, 2),
            };
            table.Controls.Add(valueLabel, 2, r);
        }

        private Panel CreateNetworkPanel()
        {
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
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));

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

            return strip;
        }

        // --- Edge Network group ---------------------------------------------

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

            panel.Resize += (s, e) =>
            {
                _lstAlerts.Width = panel.Width - 25;
                _lstAlerts.Height = panel.Height - 55;
                btnClear.Left = panel.Width - 80;
            };

            return panel;
        }
    }
}
