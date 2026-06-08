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
}
