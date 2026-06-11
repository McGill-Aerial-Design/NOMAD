// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
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
            this.Size = new Size(920, 650);
            this.MinimumSize = new Size(860, 550);
            this.AutoScroll = false;

            var rootLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                BackColor = NOMADTheme.CARD_BG,
                Margin = new Padding(0),
                Padding = new Padding(0),
            };
            rootLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 560));
            rootLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));

            _servicesPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.CARD_BG,
                AutoScroll = true,
                Padding = new Padding(0),
            };

            _logPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(38, 38, 42),
                Padding = new Padding(10),
            };

            rootLayout.Controls.Add(_servicesPanel, 0, 0);
            rootLayout.Controls.Add(_logPanel, 1, 0);
            this.Controls.Add(rootLayout);

            int yOffset = 10;
            int leftCol = ServiceLeftCol;
            int rightCol = ServiceActionCol;

            // Title
            var lblTitle = new Label
            {
                Text = "NOMAD Service Control",
                Location = new Point(leftCol, yOffset),
                Size = new Size(520, 25),
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(lblTitle);
            yOffset += 35;

            // === MAVLink Router ===
            AddServiceRow("MAVLink Router", ref _lblMavlinkStatus, ref _btnMavlinkRestart, ref yOffset);
            _btnMavlinkRestart.Click += async (s, e) => await RestartServiceAsync("mavlink-router", _lblMavlinkStatus);

            // === MediaMTX ===
            AddServiceRow("MediaMTX (RTSP)", ref _lblMediamtxStatus, ref _btnMediamtxRestart, ref yOffset);
            _btnMediamtxRestart.Click += async (s, e) => await RestartServiceAsync("mediamtx", _lblMediamtxStatus);

            // === noVNC (with Start/Stop) ===
            AddNoVncRow(ref yOffset);

            // === NOMAD Services (Full Restart) ===
            AddServiceRow("NOMAD Services", ref _lblEdgeCoreStatus, ref _btnEdgeCoreRestart, ref yOffset, "Restart All");
            _btnEdgeCoreRestart.Click += async (s, e) => await RestartAllServicesAsync();

            // === Isaac ROS (with Start/Stop) ===
            AddIsaacRosRow(ref yOffset);

            // === ROS HTTP Bridge (with Start/Stop) ===
            AddRosBridgeRow(ref yOffset);

            // === Nvblox (with Launch/Stop) ===
            AddNvbloxRow(ref yOffset);

            // === Video Bridges ===
            AddServiceRow("Video Bridges", ref _lblVideoBridgesStatus, ref _btnStartBridges, ref yOffset, "Start");
            _btnStartBridges.Click += async (s, e) => await StartVideoBridgesAsync();

            // === SLAM Service ===
            AddServiceRow("SLAM / Mesh", ref _lblSlamStatus, ref _btnStopSlam, ref yOffset, "Stop SLAM");
            _btnStopSlam.Click += async (s, e) => await StopSlamAsync();



            // Separator
            yOffset += 10;
            var separator = new Label
            {
                BorderStyle = BorderStyle.Fixed3D,
                Location = new Point(leftCol, yOffset),
                Size = new Size(520, 2),
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(separator);
            yOffset += 15;

            // === VIO Status Section ===
            var lblVioTitle = new Label
            {
                Text = "VIO / VSLAM Status",
                Location = new Point(leftCol, yOffset),
                Size = new Size(200, 20),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = NOMADTheme.INFO
            };
            _servicesPanel.Controls.Add(lblVioTitle);
            yOffset += 25;

            // VIO Status
            var lblVioLabel = new Label
            {
                Text = "Status:",
                Location = new Point(leftCol, yOffset),
                Size = new Size(80, 20),
                ForeColor = NOMADTheme.TEXT_SECONDARY
            };
            _servicesPanel.Controls.Add(lblVioLabel);

            _lblVioStatus = new Label
            {
                Text = "Unknown",
                Location = new Point(ServiceStatusCol, yOffset),
                Size = new Size(ServiceStatusWidth, 20),
                ForeColor = NOMADTheme.WARNING,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblVioStatus);

            yOffset += 30;

            // VIO Trajectory Points
            var lblTrajLabel = new Label
            {
                Text = "Trajectory:",
                Location = new Point(leftCol, yOffset),
                Size = new Size(80, 20),
                ForeColor = NOMADTheme.TEXT_SECONDARY
            };
            _servicesPanel.Controls.Add(lblTrajLabel);

            _lblVioTrajectoryPoints = new Label
            {
                Text = "0 points",
                Location = new Point(ServiceStatusCol, yOffset),
                Size = new Size(ServiceStatusWidth, 20),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblVioTrajectoryPoints);

            _btnClearTrajectory = new Button
            {
                Text = "Clear",
                Location = new Point(rightCol, yOffset - 3),
                Size = new Size(100, 25),
                BackColor = NOMADTheme.BUTTON_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
            };
            _btnClearTrajectory.Click += async (s, e) => await ClearTrajectoryAsync();
            _servicesPanel.Controls.Add(_btnClearTrajectory);
            yOffset += 40;

            // Last update
            _lblLastUpdate = new Label
            {
                Text = "Last update: Never",
                Location = new Point(leftCol, yOffset),
                Size = new Size(520, 20),
                ForeColor = NOMADTheme.TEXT_MUTED,
                Font = new Font("Segoe UI", 8),
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblLastUpdate);

            var lblLogTitle = new Label
            {
                Text = "Activity Log:",
                Dock = DockStyle.Top,
                Height = 22,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
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
                Font = new Font("Consolas", 8),
            };

            _logPanel.Controls.Add(_txtLog);
            _logPanel.Controls.Add(lblLogTitle);
        }

        private void AddServiceRow(string serviceName, ref Label statusLabel, ref Button actionButton, ref int yOffset, string buttonText = "Restart")
        {
            int leftCol = ServiceLeftCol;
            int rightCol = ServiceActionCol;

            var lblName = new Label
            {
                Text = serviceName + ":",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(120, 20),
                ForeColor = NOMADTheme.TEXT_SECONDARY
            };
            _servicesPanel.Controls.Add(lblName);

            statusLabel = new Label
            {
                Text = "Checking...",
                Location = new Point(ServiceStatusCol, yOffset + 3),
                Size = new Size(ServiceStatusWidth, 20),
                ForeColor = NOMADTheme.WARNING,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(statusLabel);

            actionButton = new Button
            {
                Text = buttonText,
                Location = new Point(rightCol, yOffset),
                Size = new Size(100, 25),
                BackColor = NOMADTheme.BUTTON_BG,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(actionButton);

            yOffset += 35;
        }

        private void AddIsaacRosRow(ref int yOffset)
        {
            int leftCol = ServiceLeftCol;
            int startCol = ServiceStartCol;
            int stopCol = ServiceStopCol;

            var lblName = new Label
            {
                Text = "Isaac ROS:",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(120, 20),
                ForeColor = NOMADTheme.TEXT_SECONDARY
            };
            _servicesPanel.Controls.Add(lblName);

            _lblIsaacRosStatus = new Label
            {
                Text = "Checking...",
                Location = new Point(ServiceStatusCol, yOffset + 3),
                Size = new Size(240, 20),
                ForeColor = NOMADTheme.WARNING,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblIsaacRosStatus);

            _btnIsaacRosStart = new Button
            {
                Text = "Start",
                Location = new Point(startCol, yOffset),
                Size = new Size(70, 25),
                BackColor = NOMADTheme.BTN_START,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnIsaacRosStart.Click += async (s, e) => await StartIsaacRosAsync();
            _servicesPanel.Controls.Add(_btnIsaacRosStart);

            _btnIsaacRosStop = new Button
            {
                Text = "Stop",
                Location = new Point(stopCol, yOffset),
                Size = new Size(70, 25),
                BackColor = NOMADTheme.BTN_STOP,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnIsaacRosStop.Click += async (s, e) => await StopIsaacRosAsync();
            _servicesPanel.Controls.Add(_btnIsaacRosStop);

            yOffset += 35;
        }

        private void AddRosBridgeRow(ref int yOffset)
        {
            int leftCol = ServiceLeftCol;
            int startCol = ServiceStartCol;
            int stopCol = ServiceStopCol;

            var lblName = new Label
            {
                Text = "ROS HTTP Bridge:",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(130, 20),
                ForeColor = NOMADTheme.TEXT_SECONDARY
            };
            _servicesPanel.Controls.Add(lblName);

            _lblRosBridgeStatus = new Label
            {
                Text = "Unknown",
                Location = new Point(ServiceStatusCol, yOffset + 3),
                Size = new Size(240, 20),
                ForeColor = NOMADTheme.WARNING,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblRosBridgeStatus);

            _btnStartRosBridge = new Button
            {
                Text = "Start",
                Location = new Point(startCol, yOffset),
                Size = new Size(70, 25),
                BackColor = NOMADTheme.BTN_START,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnStartRosBridge.Click += async (s, e) => await StartRosBridgeAsync();
            _servicesPanel.Controls.Add(_btnStartRosBridge);

            _btnStopRosBridge = new Button
            {
                Text = "Stop",
                Location = new Point(stopCol, yOffset),
                Size = new Size(70, 25),
                BackColor = NOMADTheme.BTN_STOP,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnStopRosBridge.Click += async (s, e) => await StopRosBridgeAsync();
            _servicesPanel.Controls.Add(_btnStopRosBridge);

            yOffset += 35;
        }

        private void AddNoVncRow(ref int yOffset)
        {
            int leftCol = ServiceLeftCol;
            int startCol = ServiceStartCol;
            int stopCol = ServiceStopCol;

            var lblName = new Label
            {
                Text = "noVNC:",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(120, 20),
                ForeColor = NOMADTheme.TEXT_SECONDARY
            };
            _servicesPanel.Controls.Add(lblName);

            _lblNoVncStatus = new Label
            {
                Text = "Checking...",
                Location = new Point(ServiceStatusCol, yOffset + 3),
                Size = new Size(240, 20),
                ForeColor = NOMADTheme.WARNING,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblNoVncStatus);

            _btnNoVncStart = new Button
            {
                Text = "Start",
                Location = new Point(startCol, yOffset),
                Size = new Size(70, 25),
                BackColor = NOMADTheme.BTN_START,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnNoVncStart.Click += async (s, e) => await StartNoVncAsync();
            _servicesPanel.Controls.Add(_btnNoVncStart);

            _btnNoVncStop = new Button
            {
                Text = "Stop",
                Location = new Point(stopCol, yOffset),
                Size = new Size(70, 25),
                BackColor = NOMADTheme.BTN_STOP,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnNoVncStop.Click += async (s, e) => await StopNoVncAsync();
            _servicesPanel.Controls.Add(_btnNoVncStop);

            yOffset += 35;
        }

        private void AddNvbloxRow(ref int yOffset)
        {
            int leftCol = ServiceLeftCol;
            int launchCol = ServiceStartCol;
            int stopCol = ServiceStopCol;

            var lblName = new Label
            {
                Text = "Nvblox:",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(120, 20),
                ForeColor = NOMADTheme.TEXT_SECONDARY
            };
            _servicesPanel.Controls.Add(lblName);

            _lblNvbloxStatus = new Label
            {
                Text = "Checking...",
                Location = new Point(ServiceStatusCol, yOffset + 3),
                Size = new Size(240, 20),
                ForeColor = NOMADTheme.WARNING,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblNvbloxStatus);

            _btnNvbloxLaunch = new Button
            {
                Text = "Launch",
                Location = new Point(launchCol, yOffset),
                Size = new Size(70, 25),
                BackColor = NOMADTheme.BTN_START,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnNvbloxLaunch.Click += async (s, e) => await LaunchNvbloxAsync();
            _servicesPanel.Controls.Add(_btnNvbloxLaunch);

            _btnNvbloxStop = new Button
            {
                Text = "Stop",
                Location = new Point(stopCol, yOffset),
                Size = new Size(70, 25),
                BackColor = NOMADTheme.BTN_STOP,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnNvbloxStop.Click += async (s, e) => await StopNvbloxAsync();
            _servicesPanel.Controls.Add(_btnNvbloxStop);

            yOffset += 35;
        }
    }
}
