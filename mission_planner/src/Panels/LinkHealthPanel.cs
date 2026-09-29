// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// MAVLink Link Status panel
// ============================================================
// Real-time UI for the multi-link router. Shows live per-link
// metrics and the active route controls reported by the standalone host.
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

        private readonly IRouterStatusProvider _provider;
        private readonly NOMADConfig _config;
        private readonly Timer _refresh;

        // Header
        private Label _lblActive;
        private Label _lblRouterStatus;
        private Label _lblLocalEndpoint;
        private Button _btnCopyEndpoint;
        private Label _lblCopied;

        // Link cards
        private FlowLayoutPanel _linkRow;
        private readonly Dictionary<string, LinkCard> _cards = new Dictionary<string, LinkCard>();

        // Route selection
        private Label _lblManualOverride;
        private Button _btnReleaseOverride;

        // Log
        private ListBox _lstLog;

        // ============================================================
        // Ctor
        // ============================================================

        public LinkHealthPanel(IRouterStatusProvider connectionManager, NOMADConfig config)
        {
            _provider = connectionManager ?? throw new ArgumentNullException(nameof(connectionManager));
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
                RowCount = 3,
                BackColor = Color.Transparent,
                Padding = new Padding(12),
            };
            // The header AutoSizes; the live link cards (with the throughput graphs
            // + full per-link stats) take the lion's share of the height; the
            // settings + failover log share one compact bottom row, so the graphs
            // get the vertical space and the stats under them are never clipped.
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 190));
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));

            var header = BuildHeader();
            header.Dock = DockStyle.Fill;
            root.Controls.Add(header, 0, 0);

            var linkRow = BuildLinkRow();
            linkRow.Dock = DockStyle.Fill;
            root.Controls.Add(linkRow, 0, 1);

            var bottomRow = BuildBottomRow();
            bottomRow.Dock = DockStyle.Fill;
            root.Controls.Add(bottomRow, 0, 2);

            Controls.Add(root);
        }

        // Bottom strip: route controls on the left and the failover log on the right.
        private TableLayoutPanel BuildBottomRow()
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

            var settings = BuildSettingsRow();
            settings.Dock = DockStyle.Fill;
            settings.Margin = new Padding(0, 0, 6, 0);
            row.Controls.Add(settings, 0, 0);

            var log = BuildLogPanel();
            log.Dock = DockStyle.Fill;
            log.Margin = new Padding(6, 0, 0, 0);
            row.Controls.Add(log, 1, 0);

            return row;
        }

        private Panel BuildHeader()
        {
            // Docked AutoSize card: title | active on top, then router status, then a
            // wrapping endpoint row — all reflow instead of being absolutely placed.
            var panel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(16, 10, 16, 10),
            };
            panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            panel.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));

            panel.Controls.Add(new Label
            {
                Text = "MAVLink Link Status",
                Font = NOMADTheme.Font(NOMADTheme.SIZE_TITLE, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                AutoSize = true,
                Anchor = AnchorStyles.Left,
            }, 0, 0);

            _lblActive = new Label
            {
                Text = "Active: —",
                Font = NOMADTheme.Font(NOMADTheme.SIZE_LARGE, FontStyle.Bold),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                AutoSize = true,
                Anchor = AnchorStyles.Right,
                Margin = new Padding(NOMADTheme.PAD, 4, 0, 0),
            };
            panel.Controls.Add(_lblActive, 1, 0);

            _lblRouterStatus = new Label
            {
                Text = "Router: …",
                Font = NOMADTheme.Font(),
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                AutoSize = true,
                Margin = new Padding(0, 6, 0, 0),
            };
            panel.Controls.Add(_lblRouterStatus, 0, 1);
            panel.SetColumnSpan(_lblRouterStatus, 2);

            _lblLocalEndpoint = new Label
            {
                Text = "",
                Font = NOMADTheme.Mono(),
                ForeColor = NOMADTheme.INFO,
                AutoSize = true,
                Anchor = AnchorStyles.Left,
                Margin = new Padding(0, 4, NOMADTheme.GAP, 0),
            };
            _btnCopyEndpoint = new Button
            {
                Text = "Copy",
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Padding = new Padding(8, 1, 8, 1),
                Margin = new Padding(0, 2, NOMADTheme.GAP, 0),
                BackColor = NOMADTheme.BUTTON_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
                Cursor = Cursors.Hand,
            };
            _btnCopyEndpoint.FlatAppearance.BorderSize = 0;
            _btnCopyEndpoint.Click += (s, e) => CopyEndpoint();

            _lblCopied = new Label
            {
                Text = "",
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
                ForeColor = NOMADTheme.SUCCESS,
                AutoSize = true,
                Anchor = AnchorStyles.Left,
                Margin = new Padding(0, 4, 0, 0),
            };

            var endpointRow = new FlowLayoutPanel
            {
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                WrapContents = true,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 2, 0, 0),
                Padding = new Padding(0),
            };
            endpointRow.Controls.Add(_lblLocalEndpoint);
            endpointRow.Controls.Add(_btnCopyEndpoint);
            endpointRow.Controls.Add(_lblCopied);
            panel.Controls.Add(endpointRow, 0, 2);
            panel.SetColumnSpan(endpointRow, 2);

            return panel;
        }

        private Panel BuildLinkRow()
        {
            var row = new FlowLayoutPanel
            { Dock = DockStyle.Fill, AutoScroll = true, WrapContents = true, BackColor = Color.Transparent };
            _linkRow = row;
            RebuildLinkCards(_provider.LinkStatistics);
            return row;
        }

        private void RebuildLinkCards(IReadOnlyList<LinkStatistics> links)
        {
            foreach (var card in _cards.Values) { card.Dispose(); }
            _cards.Clear();
            _linkRow.Controls.Clear();
            foreach (var stats in links)
            {
                var id = stats.Type;
                var card = new LinkCard(stats.Name, id) { Width = 330, Height = 270 };
                card.SetActiveRequested += (sender, args) => _provider.SwitchToLink(id);
                _cards.Add(id, card);
                _linkRow.Controls.Add(card);
            }
        }

        private Panel BuildSettingsRow()
        {
            var panel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                AutoScroll = true,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(14),
                Margin = new Padding(0, 6, 0, 0),
            };
            panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));

            var flow = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                WrapContents = true,
                FlowDirection = FlowDirection.LeftToRight,
                BackColor = Color.Transparent,
                Margin = new Padding(0),
                Padding = new Padding(0),
            };

            _btnReleaseOverride = SettingButton("Release override");
            _btnReleaseOverride.Visible = false;
            _btnReleaseOverride.Click += (s, e) => _provider.SwitchToLink(LinkType.None);

            _lblManualOverride = new Label
            {
                Text = "",
                ForeColor = NOMADTheme.WARNING,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_BODY, FontStyle.Bold),
                AutoSize = true,
                Margin = new Padding(NOMADTheme.GAP, 5, 0, 0),
            };

            flow.Controls.Add(_btnReleaseOverride);
            flow.Controls.Add(_lblManualOverride);

            panel.Controls.Add(flow);
            return panel;
        }

        private static Button SettingButton(string text)
        {
            var b = new Button
            {
                Text = text,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Padding = new Padding(8, 2, 8, 2),
                Margin = new Padding(0, 2, NOMADTheme.GAP, 0),
                BackColor = NOMADTheme.BUTTON_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
                Cursor = Cursors.Hand,
            };
            b.FlatAppearance.BorderSize = 0;
            return b;
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
                Font = NOMADTheme.Font(NOMADTheme.SIZE_HEADING, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                AutoSize = true,
                Margin = new Padding(0, 0, 0, 6),
            };
            grid.Controls.Add(title, 0, 0);

            var btnClear = SettingButton("Clear");
            btnClear.Anchor = AnchorStyles.Right;
            btnClear.Click += (s, e) => { _lstLog.Items.Clear(); };
            grid.Controls.Add(btnClear, 1, 0);

            _lstLog = new ListBox
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = NOMADTheme.Mono(NOMADTheme.SIZE_BODY),
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
            _provider.LinkStatusChanged += (s, e) =>
            {
                UiAsync.RunSync(this, () => RefreshAll(), "LinkStatusChanged");
            };
            _provider.FailoverOccurred += (s, e) =>
            {
                UiAsync.RunSync(this, () => AppendLog(e), "FailoverOccurred");
            };
            _provider.ActiveLinkChanged += (s, t) =>
            {
                UiAsync.RunSync(this, () => RefreshAll(), "ActiveLinkChanged");
            };
            _provider.LogMessage += (s, msg) =>
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
                var links = _provider.LinkStatistics;
                var active = _provider.ActiveLink;
                var ovr = _provider.ManualOverride;

                _lblActive.Text = !_provider.IsRouterAvailable
                    ? "Active: unavailable"
                    : $"Active: {(active == LinkType.None ? "—" : active.ToString())}";
                _lblActive.ForeColor = !_provider.IsRouterAvailable || string.IsNullOrEmpty(active)
                    ? NOMADTheme.WARNING : NOMADTheme.TEXT_PRIMARY;

                bool running = _provider.IsMonitoring;
                _lblRouterStatus.Text = LinkStatusDisplay.FormatRouterStatus(
                    running, _provider.IsRouterAvailable, links);
                _lblRouterStatus.ForeColor = _provider.IsRouterAvailable
                    ? NOMADTheme.TEXT_SECONDARY : NOMADTheme.ERROR;
                _lblLocalEndpoint.Text = $"Mode: {_provider.RouterMode}   Local: {_provider.LocalMergedEndpoint}   " +
                    "(set Mission Planner to UDP Client / UDPCl to this port)";

                if (LinkStatusDisplay.HasMembershipChanged(links, _cards.Keys))
                {
                    RebuildLinkCards(links);
                }
                foreach (var stats in links)
                {
                    if (_cards.TryGetValue(stats.Type, out var card))
                    { card.Update(stats, active == stats.Type, ovr == stats.Type); }
                }

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

        private void CopyEndpoint()
        {
            try
            {
                Clipboard.SetText(_provider.LocalMergedEndpoint);
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
}
