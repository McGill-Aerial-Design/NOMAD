// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// MAVLink Link Status — per-link card
// ============================================================
// Internal UserControl rendering one link's live metrics (latency, loss,
// throughput, RSSI, heartbeat age) with a throughput sparkline. Used by
// LinkHealthPanel.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    // ================================================================
    // Per-link card
    // ================================================================

    internal class LinkCard : UserControl
    {
        private readonly LinkType _type;
        private readonly string _title;

        private Label _lblTitle;
        private Label _lblHealth;
        private Label _lblEndpoint;
        private Label _lblLatency;
        private Label _lblLoss;
        private Label _lblRate;
        private Label _lblFrames;
        private Label _lblRssi;
        private Label _lblHb;
        private SparklinePanel _spark;
        private Button _btnSetActive;
        private Label _lblActiveBadge;

        public event EventHandler SetActiveRequested;

        public LinkCard(string title, LinkType type)
        {
            _title = title;
            _type = type;
            BackColor = NOMADTheme.CARD_BG;
            DoubleBuffered = true;
            Padding = new Padding(14, 12, 14, 12);
            BuildUi();
        }

        private void BuildUi()
        {
            // One docked TableLayout that owns every cell, so everything
            // reflows when the card resizes. Rows top-to-bottom:
            //   0 title + active-badge
            //   1 big health text
            //   2 endpoint
            //   3 sparkline (fills available space)
            //   4 metrics grid (2 columns × 3 rows of labels)
            //   5 button row
            var grid = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 7,
                BackColor = Color.Transparent,
            };
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));        // title
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));        // health
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));        // endpoint
            grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 72));    // sparkline (fixed)
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));        // metrics
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));        // button
            grid.RowStyles.Add(new RowStyle(SizeType.Percent, 100));    // spacer absorbs slack

            // Row 0: title + active badge (mini 2-column flex row)
            var titleRow = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                ColumnCount = 2,
                RowCount = 1,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                BackColor = Color.Transparent,
                Margin = new Padding(0),
            };
            titleRow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            titleRow.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));

            _lblTitle = new Label
            {
                Text = _title,
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                AutoSize = true,
                Margin = new Padding(0, 2, 0, 4),
            };
            titleRow.Controls.Add(_lblTitle, 0, 0);

            _lblActiveBadge = new Label
            {
                Text = "ACTIVE",
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                ForeColor = Color.White,
                BackColor = NOMADTheme.SUCCESS,
                Padding = new Padding(6, 2, 6, 2),
                AutoSize = true,
                Visible = false,
                Margin = new Padding(0, 2, 0, 4),
                Anchor = AnchorStyles.Right,
            };
            titleRow.Controls.Add(_lblActiveBadge, 1, 0);
            grid.Controls.Add(titleRow, 0, 0);

            // Row 1: big health status
            _lblHealth = new Label
            {
                Text = "DISCONNECTED",
                Font = new Font("Segoe UI", 16, FontStyle.Bold),
                ForeColor = NOMADTheme.ERROR,
                AutoSize = true,
                Margin = new Padding(0, 2, 0, 2),
            };
            grid.Controls.Add(_lblHealth, 0, 1);

            // Row 2: endpoint
            _lblEndpoint = new Label
            {
                Text = "—",
                Font = new Font("Consolas", 9),
                ForeColor = NOMADTheme.TEXT_MUTED,
                AutoSize = true,
                Margin = new Padding(0, 0, 0, 6),
            };
            grid.Controls.Add(_lblEndpoint, 0, 2);

            // Row 3: sparkline (fixed 72px row, fills width)
            _spark = new SparklinePanel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(28, 28, 30),
                Margin = new Padding(0, 2, 0, 10),
            };
            grid.Controls.Add(_spark, 0, 3);

            // Row 4: metrics — 2-column AutoSize sub-grid
            var metrics = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                ColumnCount = 2,
                RowCount = 3,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 0, 0, 6),
            };
            metrics.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            metrics.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            for (int r = 0; r < 3; r++) metrics.RowStyles.Add(new RowStyle(SizeType.AutoSize));

            _lblLatency = MakeMetric();
            _lblLoss = MakeMetric();
            _lblRate = MakeMetric();
            _lblFrames = MakeMetric();
            _lblRssi = MakeMetric();
            _lblHb = MakeMetric();

            metrics.Controls.Add(_lblLatency, 0, 0);
            metrics.Controls.Add(_lblFrames, 1, 0);
            metrics.Controls.Add(_lblLoss, 0, 1);
            metrics.Controls.Add(_lblRssi, 1, 1);
            metrics.Controls.Add(_lblRate, 0, 2);
            metrics.Controls.Add(_lblHb, 1, 2);
            grid.Controls.Add(metrics, 0, 4);

            // Row 5: action button
            _btnSetActive = new Button
            {
                Text = "Set active",
                Size = new Size(110, 28),
                BackColor = NOMADTheme.BTN_PRIMARY,
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Cursor = Cursors.Hand,
                Anchor = AnchorStyles.Left,
                Margin = new Padding(0, 4, 0, 0),
            };
            _btnSetActive.FlatAppearance.BorderSize = 0;
            _btnSetActive.Click += (s, e) => SetActiveRequested?.Invoke(this, EventArgs.Empty);
            grid.Controls.Add(_btnSetActive, 0, 5);

            Controls.Add(grid);
        }

        private static Label MakeMetric()
        {
            return new Label
            {
                Font = new Font("Segoe UI", 9),
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                AutoSize = true,
                Text = "—",
                Margin = new Padding(0, 2, 8, 2),
            };
        }

        public void Update(LinkStatistics s, bool isActive, bool isOverride)
        {
            _lblEndpoint.Text = string.IsNullOrEmpty(s.Endpoint) ? "—" : s.Endpoint;

            _lblHealth.Text = s.IsConnected ? s.Health.ToString().ToUpperInvariant() : "DISCONNECTED";
            _lblHealth.ForeColor = HealthColor(s.Health, s.IsConnected);

            _lblLatency.Text = s.IsConnected ? $"HB jitter: {s.LatencyMs,5:F0} ms" : "HB jitter:    — ms";
            _lblLoss.Text = s.IsConnected ? $"Loss:    {s.PacketLossPercent,5:F1} %" : "Loss:       — %";

            string rate = FormatRate(s.DataRateBps);
            _lblRate.Text = $"Rate:  {rate}";

            _lblFrames.Text = $"Frames: {s.PacketsReceived:N0}";
            if (s.PacketsDuplicate > 0) _lblFrames.Text += $"  (+{s.PacketsDuplicate:N0} dup)";

            _lblHb.Text = s.HeartbeatCount > 0
                ? $"HB:    {s.HeartbeatCount:N0}   age {Math.Max(0, (DateTime.UtcNow - s.LastHeartbeat).TotalSeconds):F1}s"
                : "HB:    —";

            _lblRssi.Text = s.Rssi.HasValue
                ? $"RSSI:  {s.Rssi.Value,3}  rem {s.RemRssi.GetValueOrDefault(0),3}"
                : "RSSI:    —";

            _spark.Push(s.DataRateBps, s.IsConnected ? HealthColor(s.Health, true) : NOMADTheme.TEXT_MUTED);

            _lblActiveBadge.Visible = isActive;
            _lblActiveBadge.BackColor = isOverride ? NOMADTheme.WARNING : NOMADTheme.SUCCESS;
            _lblActiveBadge.Text = isOverride ? "ACTIVE (manual)" : "ACTIVE";

            _btnSetActive.Enabled = !isActive;
            _btnSetActive.BackColor = isActive ? NOMADTheme.BUTTON_BG : NOMADTheme.BTN_PRIMARY;
        }

        private static Color HealthColor(LinkHealth h, bool connected)
        {
            if (!connected) return NOMADTheme.ERROR;
            return h switch
            {
                LinkHealth.Excellent => NOMADTheme.SUCCESS,
                LinkHealth.Good => Color.LightGreen,
                LinkHealth.Fair => Color.Gold,
                LinkHealth.Poor => NOMADTheme.WARNING,
                LinkHealth.Critical => Color.OrangeRed,
                _ => NOMADTheme.ERROR,
            };
        }

        private static string FormatRate(double bytesPerSec)
        {
            if (bytesPerSec < 1) return "  0 B/s";
            if (bytesPerSec < 1024) return $"{bytesPerSec,5:F0} B/s";
            if (bytesPerSec < 1024 * 1024) return $"{bytesPerSec / 1024.0,5:F1} KB/s";
            return $"{bytesPerSec / (1024.0 * 1024.0),5:F1} MB/s";
        }
    }
}
