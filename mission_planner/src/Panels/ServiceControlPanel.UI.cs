// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class ServiceControlPanel
    {
        private void InitializeUI()
        {
            this.BackColor = NOMADTheme.CARD_BG;
            this.Dock = DockStyle.Fill;
            this.MinimumSize = new Size(640, 420);

            // Two panes: services (left, scrolls) + activity log (right). Percentage
            // columns + a MinimumSize keep both usable at any window size.
            var rootLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                BackColor = NOMADTheme.CARD_BG,
                Margin = new Padding(0),
                Padding = new Padding(0),
            };
            rootLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 58));
            rootLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 42));

            _servicesPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.CARD_BG,
                AutoScroll = true,
                Padding = new Padding(NOMADTheme.PAD),
            };
            _logPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.PANEL_ALT,
                Padding = new Padding(NOMADTheme.PAD),
            };
            rootLayout.Controls.Add(_servicesPanel, 0, 0);
            rootLayout.Controls.Add(_logPanel, 1, 0);
            this.Controls.Add(rootLayout);

            // Services: a single-column stack of AutoSize rows that reflow.
            var services = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                ColumnCount = 1,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                BackColor = Color.Transparent,
            };
            services.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));

            Stack(services, new Label
            {
                Text = "NOMAD Service Control",
                Font = NOMADTheme.Font(NOMADTheme.SIZE_LARGE, FontStyle.Bold),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                AutoSize = true,
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
            });

            Stack(services, ServiceRow("MAVLink Router", out _lblMavlinkStatus,
                _btnMavlinkRestart = ActionBtn("Restart", NOMADTheme.BUTTON_BG)));
            _btnMavlinkRestart.Click += async (s, e) => await RestartServiceAsync("mavlink-router", _lblMavlinkStatus);

            Stack(services, ServiceRow("MediaMTX (RTSP)", out _lblMediamtxStatus,
                _btnMediamtxRestart = ActionBtn("Restart", NOMADTheme.BUTTON_BG)));
            _btnMediamtxRestart.Click += async (s, e) => await RestartServiceAsync("mediamtx", _lblMediamtxStatus);

            Stack(services, ServiceRow("noVNC", out _lblNoVncStatus,
                _btnNoVncStart = ActionBtn("Start", NOMADTheme.BTN_START),
                _btnNoVncStop = ActionBtn("Stop", NOMADTheme.BTN_STOP)));
            _btnNoVncStart.Click += async (s, e) => await StartNoVncAsync();
            _btnNoVncStop.Click += async (s, e) => await StopNoVncAsync();

            Stack(services, ServiceRow("NOMAD Services", out _lblEdgeCoreStatus,
                _btnEdgeCoreRestart = ActionBtn("Restart All", NOMADTheme.BUTTON_BG)));
            _btnEdgeCoreRestart.Click += async (s, e) => await RestartAllServicesAsync();

            Stack(services, ServiceRow("Isaac ROS", out _lblIsaacRosStatus,
                _btnIsaacRosStart = ActionBtn("Start", NOMADTheme.BTN_START),
                _btnIsaacRosStop = ActionBtn("Stop", NOMADTheme.BTN_STOP)));
            _btnIsaacRosStart.Click += async (s, e) => await StartIsaacRosAsync();
            _btnIsaacRosStop.Click += async (s, e) => await StopIsaacRosAsync();

            Stack(services, ServiceRow("ROS Adapter Node", out _lblRosBridgeStatus,
                _btnStartRosBridge = ActionBtn("Start", NOMADTheme.BTN_START),
                _btnStopRosBridge = ActionBtn("Stop", NOMADTheme.BTN_STOP)));
            _lblRosBridgeStatus.Text = "Unknown";
            _btnStartRosBridge.Click += async (s, e) => await StartRosBridgeAsync();
            _btnStopRosBridge.Click += async (s, e) => await StopRosBridgeAsync();

            // Nvblox: status-only (launch/stop routes were gutted from this baseline).
            Stack(services, ServiceRow("Nvblox", out _lblNvbloxStatus));

            Stack(services, ServiceRow("Video Bridges", out _lblVideoBridgesStatus,
                _btnStartBridges = ActionBtn("Start", NOMADTheme.BUTTON_BG)));
            _btnStartBridges.Click += async (s, e) => await StartVideoBridgesAsync();

            // Separator
            Stack(services, new Panel { Height = 1, BackColor = NOMADTheme.CARD_BORDER, Margin = new Padding(0, NOMADTheme.GAP, 0, NOMADTheme.GAP) });

            // VIO / VSLAM status section
            Stack(services, new Label
            {
                Text = "VIO / VSLAM Status",
                Font = NOMADTheme.Font(NOMADTheme.SIZE_HEADING, FontStyle.Bold),
                ForeColor = NOMADTheme.INFO,
                AutoSize = true,
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
            });

            Stack(services, ServiceRow("Status", out _lblVioStatus));

            Stack(services, ServiceRow("Trajectory", out _lblVioTrajectoryPoints,
                _btnClearTrajectory = ActionBtn("Clear", NOMADTheme.BUTTON_BG)));
            _lblVioTrajectoryPoints.Text = "0 points";
            _lblVioTrajectoryPoints.ForeColor = NOMADTheme.TEXT_PRIMARY;
            _btnClearTrajectory.Click += async (s, e) => await ClearTrajectoryAsync();

            _lblLastUpdate = new Label
            {
                Text = "Last update: Never",
                ForeColor = NOMADTheme.TEXT_MUTED,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
                AutoSize = true,
                Margin = new Padding(0, NOMADTheme.GAP, 0, 0),
            };
            Stack(services, _lblLastUpdate);

            _servicesPanel.Controls.Add(services);

            // Activity log
            var lblLogTitle = new Label
            {
                Text = "Activity Log:",
                Dock = DockStyle.Top,
                Height = 22,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_BODY, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleLeft,
            };
            _txtLog = new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ScrollBars = ScrollBars.Vertical,
                ReadOnly = true,
                BackColor = NOMADTheme.BG_DARK,
                ForeColor = NOMADTheme.SUCCESS,
                Font = NOMADTheme.Mono(NOMADTheme.SIZE_SMALL),
            };
            _logPanel.Controls.Add(_txtLog);
            _logPanel.Controls.Add(lblLogTitle);
        }

        // Append a full-width row to a single-column table with an AutoSize height.
        private static void Stack(TableLayoutPanel host, Control row)
        {
            int r = host.RowCount;
            host.RowCount = r + 1;
            host.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            row.Dock = DockStyle.Fill;
            host.Controls.Add(row, 0, r);
        }

        // One service line: name | status (stretches) | action buttons (wrap). The
        // status label is handed back via `status` so the poller can update it.
        private static TableLayoutPanel ServiceRow(string name, out Label status, params Control[] actions)
        {
            var row = new TableLayoutPanel
            {
                ColumnCount = 3,
                RowCount = 1,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 0, 0, NOMADTheme.GAP),
            };
            row.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 130));
            row.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            row.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));

            row.Controls.Add(new Label
            {
                Text = name + ":",
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Font = NOMADTheme.Font(),
                AutoSize = true,
                Anchor = AnchorStyles.Left,
                Margin = new Padding(0, 5, NOMADTheme.GAP, 0),
            }, 0, 0);

            status = new Label
            {
                Text = "Checking...",
                ForeColor = NOMADTheme.WARNING,
                Font = NOMADTheme.Font(),
                AutoSize = true,
                Anchor = AnchorStyles.Left,
                Margin = new Padding(0, 5, NOMADTheme.GAP, 0),
            };
            row.Controls.Add(status, 1, 0);

            var flow = new FlowLayoutPanel
            {
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                WrapContents = false,
                Anchor = AnchorStyles.Right,
                Margin = new Padding(0),
                Padding = new Padding(0),
            };
            foreach (var a in actions)
                flow.Controls.Add(a);
            row.Controls.Add(flow, 2, 0);
            return row;
        }

        private static Button ActionBtn(string text, Color color)
        {
            var b = new Button
            {
                Text = text,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Padding = new Padding(8, 3, 8, 3),
                Margin = new Padding(0, 0, NOMADTheme.GAP, 0),
                FlatStyle = FlatStyle.Flat,
                BackColor = color,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            b.FlatAppearance.BorderSize = 0;
            return b;
        }
    }
}
