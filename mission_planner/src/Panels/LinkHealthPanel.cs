// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// MAVLink Link Status panel
// ============================================================
// Real-time UI for the dual-link router. Shows live per-link
// metrics (latency, loss, throughput, RSSI, heartbeat age) for
// both LTE and RadioMaster, plus router controls, a throughput
// sparkline per link, failover settings and event log. All data
// comes from MAVLinkConnectionManager / GroundLinkRouter.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public class LinkHealthPanel : UserControl
    {
        // ============================================================
        // Fields
        // ============================================================

        private readonly MAVLinkConnectionManager _cm;
        private readonly NOMADConfig _config;
        private readonly Timer _refresh;

        // Header
        private Label _lblActive;
        private Label _lblRouterStatus;
        private Label _lblLocalEndpoint;
        private Button _btnCopyEndpoint;
        private Label _lblCopied;

        // Link cards
        private LinkCard _lteCard;
        private LinkCard _radioCard;

        // Settings row
        private CheckBox _chkAuto;
        private ComboBox _cmbPreferred;
        private CheckBox _chkDedup;
        private CheckBox _chkAutoReconnect;
        private Label _lblManualOverride;
        private Button _btnReleaseOverride;
        private Button _btnReset;

        // Log
        private ListBox _lstLog;

        // ============================================================
        // Ctor
        // ============================================================

        public LinkHealthPanel(MAVLinkConnectionManager connectionManager, NOMADConfig config)
        {
            _cm = connectionManager ?? throw new ArgumentNullException(nameof(connectionManager));
            _config = config ?? throw new ArgumentNullException(nameof(config));

            BackColor = NOMADTheme.BG_DARK;
            Dock = DockStyle.Fill;
            DoubleBuffered = true;

            BuildUi();
            HookEvents();

            _refresh = new Timer { Interval = Math.Max(200, _config.LinkMonitorInterval) };
            _refresh.Tick += (s, e) => RefreshAll();
            _refresh.Start();
            RefreshAll();
        }

        // ============================================================
        // UI
        // ============================================================

        private void BuildUi()
        {
            var root = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 4,
                BackColor = Color.Transparent,
                Padding = new Padding(12),
            };
            // Fixed header/settings rows keep WinForms from collapsing the
            // panel during the first layout pass; the live rows split the rest.
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 100));
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 60));
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 88));
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 40));
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));

            var header = BuildHeader();
            header.Dock = DockStyle.Fill;
            root.Controls.Add(header, 0, 0);

            var linkRow = BuildLinkRow();
            linkRow.Dock = DockStyle.Fill;
            root.Controls.Add(linkRow, 0, 1);

            var settingsRow = BuildSettingsRow();
            settingsRow.Dock = DockStyle.Fill;
            root.Controls.Add(settingsRow, 0, 2);

            var logPanel = BuildLogPanel();
            logPanel.Dock = DockStyle.Fill;
            root.Controls.Add(logPanel, 0, 3);

            Controls.Add(root);
        }

        private Panel BuildHeader()
        {
            var panel = MakeCard();
            panel.Padding = new Padding(16, 10, 16, 10);

            var title = new Label
            {
                Text = "MAVLink Link Status",
                Font = new Font("Segoe UI", 14, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                Location = new Point(0, 4),
                AutoSize = true,
            };
            panel.Controls.Add(title);

            _lblActive = new Label
            {
                Text = "Active: —",
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                AutoSize = true,
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
            };
            panel.Controls.Add(_lblActive);
            panel.Resize += (s, e) =>
            {
                _lblActive.Location = new Point(panel.ClientSize.Width - _lblActive.Width - 16, 6);
                LayoutEndpointRow(panel);
            };

            _lblRouterStatus = new Label
            {
                Text = "Router: …",
                Font = new Font("Segoe UI", 9),
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Location = new Point(0, 36),
                AutoSize = true,
            };
            panel.Controls.Add(_lblRouterStatus);

            _lblLocalEndpoint = new Label
            {
                Text = "",
                Font = new Font("Consolas", 9),
                ForeColor = NOMADTheme.INFO,
                Location = new Point(0, 56),
                AutoSize = true,
            };
            panel.Controls.Add(_lblLocalEndpoint);

            _btnCopyEndpoint = new Button
            {
                Text = "Copy",
                Size = new Size(60, 22),
                BackColor = NOMADTheme.BUTTON_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 8),
                Cursor = Cursors.Hand,
            };
            _btnCopyEndpoint.FlatAppearance.BorderSize = 0;
            _btnCopyEndpoint.Click += (s, e) => CopyEndpoint();
            panel.Controls.Add(_btnCopyEndpoint);

            _lblCopied = new Label
            {
                Text = "",
                Font = new Font("Segoe UI", 8),
                ForeColor = NOMADTheme.SUCCESS,
                AutoSize = true,
            };
            panel.Controls.Add(_lblCopied);

            panel.HandleCreated += (s, e) => LayoutEndpointRow(panel);
            return panel;
        }

        private void LayoutEndpointRow(Panel panel)
        {
            if (_lblLocalEndpoint == null || _btnCopyEndpoint == null || _lblCopied == null) return;
            int x = _lblLocalEndpoint.Right + 8;
            _btnCopyEndpoint.Location = new Point(x, 54);
            _lblCopied.Location = new Point(_btnCopyEndpoint.Right + 8, 56);
        }

        private Panel BuildLinkRow()
        {
            var row = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 6, 0, 0),
            };
            row.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            row.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            row.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            _lteCard = new LinkCard("LTE / Tailscale", LinkType.LTE)
            {
                Dock = DockStyle.Fill,
                Margin = new Padding(0, 0, 6, 0),
            };
            _lteCard.SetActiveRequested += (s, e) => _cm.SwitchToLink(LinkType.LTE);

            _radioCard = new LinkCard("RadioMaster", LinkType.RadioMaster)
            {
                Dock = DockStyle.Fill,
                Margin = new Padding(6, 0, 0, 0),
            };
            _radioCard.SetActiveRequested += (s, e) => _cm.SwitchToLink(LinkType.RadioMaster);

            row.Controls.Add(_lteCard, 0, 0);
            row.Controls.Add(_radioCard, 1, 0);
            return row;
        }

        private Panel BuildSettingsRow()
        {
            var panel = MakeCard();
            panel.Margin = new Padding(0, 6, 0, 0);

            _chkAuto = new CheckBox
            {
                Text = "Auto-failover",
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = new Font("Segoe UI", 9),
                Location = new Point(14, 12),
                AutoSize = true,
                Checked = _cm.Config.AutoFailoverEnabled,
            };
            _chkAuto.CheckedChanged += (s, e) =>
            {
                _cm.SetAutoFailoverEnabled(_chkAuto.Checked);
                _config.AutoFailoverEnabled = _chkAuto.Checked;
                PersistConfig();
            };
            panel.Controls.Add(_chkAuto);

            _chkAutoReconnect = new CheckBox
            {
                Text = "Return to preferred when healthy",
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = new Font("Segoe UI", 9),
                Location = new Point(14, 38),
                AutoSize = true,
                Checked = _cm.Config.AutoReconnectPreferred,
            };
            _chkAutoReconnect.CheckedChanged += (s, e) =>
            {
                _cm.SetAutoReconnectPreferred(_chkAutoReconnect.Checked);
                _config.AutoReconnectToPreferred = _chkAutoReconnect.Checked;
                PersistConfig();
            };
            panel.Controls.Add(_chkAutoReconnect);

            var lblPref = new Label { Text = "Preferred:", ForeColor = NOMADTheme.TEXT_SECONDARY, Font = new Font("Segoe UI", 9), Location = new Point(240, 13), AutoSize = true };
            panel.Controls.Add(lblPref);

            _cmbPreferred = new ComboBox
            {
                Location = new Point(310, 10),
                Size = new Size(110, 22),
                DropDownStyle = ComboBoxStyle.DropDownList,
                Font = new Font("Segoe UI", 9),
            };
            _cmbPreferred.Items.AddRange(new object[] { "LTE", "RadioMaster", "None" });
            _cmbPreferred.SelectedIndex = _cm.Config.PreferredLink switch
            {
                LinkType.LTE => 0,
                LinkType.RadioMaster => 1,
                _ => 2,
            };
            _cmbPreferred.SelectedIndexChanged += (s, e) =>
            {
                var pref = _cmbPreferred.SelectedIndex switch
                {
                    0 => LinkType.LTE,
                    1 => LinkType.RadioMaster,
                    _ => LinkType.None,
                };
                _cm.SetPreferredLink(pref);
                _config.PreferredMavlinkLink = pref switch
                {
                    LinkType.LTE => "LTE",
                    LinkType.RadioMaster => "RadioMaster",
                    _ => "None",
                };
                PersistConfig();
            };
            panel.Controls.Add(_cmbPreferred);

            _chkDedup = new CheckBox
            {
                Text = "Deduplicate cross-link packets",
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = new Font("Segoe UI", 9),
                Location = new Point(450, 12),
                AutoSize = true,
                Checked = _cm.Config.RouterDedupEnabled,
            };
            _chkDedup.CheckedChanged += (s, e) =>
            {
                _cm.SetDedupEnabled(_chkDedup.Checked);
                _config.RouterDedupEnabled = _chkDedup.Checked;
                PersistConfig();
            };
            panel.Controls.Add(_chkDedup);

            _lblManualOverride = new Label
            {
                Text = "",
                ForeColor = NOMADTheme.WARNING,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Location = new Point(450, 38),
                AutoSize = true,
            };
            panel.Controls.Add(_lblManualOverride);

            _btnReleaseOverride = new Button
            {
                Text = "Release override",
                Size = new Size(120, 24),
                BackColor = NOMADTheme.BUTTON_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 8),
                Visible = false,
                Cursor = Cursors.Hand,
            };
            _btnReleaseOverride.FlatAppearance.BorderSize = 0;
            _btnReleaseOverride.Click += (s, e) => _cm.SwitchToLink(LinkType.None);
            panel.Controls.Add(_btnReleaseOverride);

            _btnReset = new Button
            {
                Text = "Reset counters",
                Size = new Size(120, 24),
                BackColor = NOMADTheme.BUTTON_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 8),
                Cursor = Cursors.Hand,
            };
            _btnReset.FlatAppearance.BorderSize = 0;
            _btnReset.Click += (s, e) => _cm.ResetCounters();
            panel.Controls.Add(_btnReset);
            panel.Resize += (s, e) => LayoutSettingsButtons(panel);
            panel.HandleCreated += (s, e) => LayoutSettingsButtons(panel);

            return panel;
        }

        private void LayoutSettingsButtons(Panel panel)
        {
            if (_btnReleaseOverride == null || _btnReset == null) return;
            int x = Math.Max(14, panel.ClientSize.Width - 140);
            _btnReleaseOverride.Location = new Point(x, 42);
            _btnReset.Location = new Point(x, 14);
        }

        private Panel BuildLogPanel()
        {
            var panel = MakeCard();
            panel.Margin = new Padding(0, 6, 0, 0);
            panel.Padding = new Padding(14, 8, 14, 12);

            var grid = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 2,
                BackColor = Color.Transparent,
            };
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            grid.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            var title = new Label
            {
                Text = "Failover log",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                AutoSize = true,
                Margin = new Padding(0, 0, 0, 6),
            };
            grid.Controls.Add(title, 0, 0);

            var btnClear = new Button
            {
                Text = "Clear",
                Size = new Size(60, 22),
                BackColor = NOMADTheme.BUTTON_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 8),
                Cursor = Cursors.Hand,
                Margin = new Padding(0, 0, 0, 6),
                Anchor = AnchorStyles.Right,
            };
            btnClear.FlatAppearance.BorderSize = 0;
            btnClear.Click += (s, e) => { _lstLog.Items.Clear(); };
            grid.Controls.Add(btnClear, 1, 0);

            _lstLog = new ListBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(25, 25, 25),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.None,
                IntegralHeight = false,
            };
            grid.SetColumnSpan(_lstLog, 2);
            grid.Controls.Add(_lstLog, 0, 1);

            panel.Controls.Add(grid);
            return panel;
        }

        private static Panel MakeCard()
        {
            return new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(14),
            };
        }

        // ============================================================
        // Event wiring
        // ============================================================

        private void HookEvents()
        {
            _cm.LinkStatusChanged += (s, e) =>
            {
                UiAsync.RunSync(this, () => RefreshAll(), "LinkStatusChanged");
            };
            _cm.FailoverOccurred += (s, e) =>
            {
                UiAsync.RunSync(this, () => AppendLog(e), "FailoverOccurred");
            };
            _cm.ActiveLinkChanged += (s, t) =>
            {
                UiAsync.RunSync(this, () => RefreshAll(), "ActiveLinkChanged");
            };
            _cm.LogMessage += (s, msg) =>
            {
                UiAsync.RunSync(this, () => _lstLog.Items.Add($"[{DateTime.Now:HH:mm:ss}] {msg}"), "LogMessage");
            };
        }

        // ============================================================
        // Refresh
        // ============================================================

        private void RefreshAll()
        {
            try
            {
                var lte = _cm.LteStatistics;
                var radio = _cm.RadioMasterStatistics;
                var active = _cm.ActiveLink;
                var ovr = _cm.ManualOverride;

                _lblActive.Text = $"Active: {(active == LinkType.None ? "—" : active.ToString())}";
                _lblActive.ForeColor = active switch
                {
                    LinkType.LTE => NOMADTheme.SUCCESS,
                    LinkType.RadioMaster => Color.MediumTurquoise,
                    _ => NOMADTheme.WARNING,
                };

                bool running = _cm.IsMonitoring;
                _lblRouterStatus.Text = running ? "Router: running — both links open" : "Router: stopped";
                _lblRouterStatus.ForeColor = running ? NOMADTheme.TEXT_SECONDARY : NOMADTheme.ERROR;
                _lblLocalEndpoint.Text = $"Local: {_cm.LocalMergedEndpoint}   (set Mission Planner to UDP Client / UDPCl to this port)";

                _lteCard.Update(lte, isActive: active == LinkType.LTE, isOverride: ovr == LinkType.LTE);
                _radioCard.Update(radio, isActive: active == LinkType.RadioMaster, isOverride: ovr == LinkType.RadioMaster);

                if (ovr == LinkType.None)
                {
                    _lblManualOverride.Text = "";
                    _btnReleaseOverride.Visible = false;
                }
                else
                {
                    _lblManualOverride.Text = $"Manual override → {ovr}";
                    _btnReleaseOverride.Visible = true;
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD: link panel refresh failed - {ex}");
            }
        }

        private void AppendLog(FailoverEventArgs e)
        {
            string line = $"[{e.Timestamp:HH:mm:ss}] {e.FromLink} → {e.ToLink}   {e.Reason}";
            _lstLog.Items.Add(line);
            while (_lstLog.Items.Count > 200) _lstLog.Items.RemoveAt(0);
            _lstLog.TopIndex = Math.Max(0, _lstLog.Items.Count - 1);
        }

        private void PersistConfig()
        {
            try { _config.Save(); }
            catch (Exception ex) { System.Diagnostics.Debug.WriteLine($"NOMAD: persist failed - {ex.Message}"); }
        }

        private void CopyEndpoint()
        {
            try
            {
                Clipboard.SetText(_cm.LocalMergedEndpoint);
                _lblCopied.Text = "copied";
                var t = new Timer { Interval = 1500 };
                t.Tick += (s, e) => { _lblCopied.Text = ""; t.Stop(); t.Dispose(); };
                t.Start();
            }
            catch { }
        }

        // ============================================================
        // Cleanup
        // ============================================================

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _refresh?.Stop();
                _refresh?.Dispose();
            }
            base.Dispose(disposing);
        }
    }

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

    // ================================================================
    // Throughput sparkline
    // ================================================================

    internal class SparklinePanel : Panel
    {
        private readonly LinkedList<double> _values = new LinkedList<double>();
        private Color _lineColor = NOMADTheme.ACCENT;
        private const int MAX_POINTS = 80;

        public SparklinePanel() { DoubleBuffered = true; }

        public void Push(double value, Color color)
        {
            _values.AddLast(value);
            while (_values.Count > MAX_POINTS) _values.RemoveFirst();
            _lineColor = color;
            Invalidate();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.Clear(BackColor);

            // baseline
            using (var basePen = new Pen(Color.FromArgb(60, 60, 60)))
            {
                g.DrawLine(basePen, 0, ClientSize.Height - 1, ClientSize.Width, ClientSize.Height - 1);
            }

            if (_values.Count < 2) return;

            double max = Math.Max(1, _values.Max());
            int w = ClientSize.Width;
            int h = ClientSize.Height;
            float dx = (float)w / Math.Max(1, MAX_POINTS - 1);

            var pts = new List<PointF>();
            int i = 0;
            int offset = MAX_POINTS - _values.Count;
            foreach (var v in _values)
            {
                float x = (offset + i) * dx;
                float y = (float)(h - 2 - (v / max) * (h - 6));
                pts.Add(new PointF(x, y));
                i++;
            }

            // soft fill under the line
            using (var fillPath = new GraphicsPath())
            using (var fillBrush = new SolidBrush(Color.FromArgb(50, _lineColor)))
            {
                fillPath.AddLine(pts[0].X, h, pts[0].X, pts[0].Y);
                for (int k = 1; k < pts.Count; k++) fillPath.AddLine(pts[k - 1], pts[k]);
                fillPath.AddLine(pts[pts.Count - 1].X, pts[pts.Count - 1].Y, pts[pts.Count - 1].X, h);
                g.FillPath(fillBrush, fillPath);
            }

            using (var pen = new Pen(_lineColor, 1.5f))
            {
                for (int k = 1; k < pts.Count; k++)
                    g.DrawLine(pen, pts[k - 1], pts[k]);
            }
        }
    }
}
