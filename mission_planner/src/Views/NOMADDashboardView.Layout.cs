// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADDashboardView.Layout.cs - Dashboard layout construction
// ============================================================
// Builds the status cards, link-status panel, health summary,
// and video preview panels. Status polling and data updates
// live in NOMADDashboardView.cs.
// ============================================================

using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADDashboardView
    {
        // ============================================================
        // UI Initialization
        // ============================================================

        private void InitializeUI()
        {
            this.BackColor = NOMADTheme.BG_DARK;
            this.Dock = DockStyle.Fill;
            this.Padding = new Padding(20);
            this.AutoScroll = true;

            // Main layout using TableLayoutPanel for responsive design
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                ColumnCount = 3,
                RowCount = 4,
                BackColor = Color.Transparent,
                CellBorderStyle = TableLayoutPanelCellBorderStyle.None,
                Padding = new Padding(0),
                Margin = new Padding(0),
            };

            // Column widths (responsive)
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33f));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33f));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33f));

            // Row heights
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 110)); // Status cards row 1
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 110)); // Status cards row 2
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 200)); // Notifications + Link Status + Health Summary
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 420)); // Enlarged video preview (spans all 3 columns)
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 190)); // EKF Source Control

            // Row 1: Connection, Flight Mode, GPS Status
            _connectionCard = CreateStatusCard("Connection", "DISCONNECTED", out _lblConnectionStatus, out _lblConnectionValue, NOMADTheme.ERROR);
            mainLayout.Controls.Add(_connectionCard, 0, 0);

            _flightModeCard = CreateStatusCard("Flight Mode", "UNKNOWN", out var lblModeTitle, out _lblFlightMode, TEXT_SECONDARY);
            mainLayout.Controls.Add(_flightModeCard, 1, 0);

            _gpsCard = CreateStatusCard("GPS Status", "No Fix", out _lblGpsStatus, out _lblGpsFix, NOMADTheme.WARNING);
            mainLayout.Controls.Add(_gpsCard, 2, 0);

            // Row 2: Battery, VIO Status, Jetson Status
            _batteryCard = CreateStatusCard("Battery", "--.- V", out var lblBattTitle, out _lblBattery, TEXT_SECONDARY);
            _lblBatteryVolts = _lblBattery;
            mainLayout.Controls.Add(_batteryCard, 0, 1);

            _vioCard = CreateStatusCard("VIO Status", "Inactive", out _lblVioStatus, out _lblVioConfidence, TEXT_SECONDARY);
            mainLayout.Controls.Add(_vioCard, 1, 1);

            _jetsonCard = CreateStatusCard("Jetson", "Offline", out _lblJetsonStatus, out _lblJetsonTemp, NOMADTheme.ERROR);
            mainLayout.Controls.Add(_jetsonCard, 2, 1);

            // Row 3: Notifications + Link Status + Health Summary
            _notificationPanel = new NotificationPanel(_notificationService);
            _notificationPanel.Margin = new Padding(5);
            mainLayout.Controls.Add(_notificationPanel, 0, 2);

            var linkStatusPanel = CreateLinkStatusPanel();
            mainLayout.Controls.Add(linkStatusPanel, 1, 2);

            var healthSummaryPanel = CreateHealthSummaryPanel();
            mainLayout.Controls.Add(healthSummaryPanel, 2, 2);

            // Row 4: Enlarged video preview spans all 3 columns
            _videoPreviewPanel = CreateVideoPreviewPanel();
            mainLayout.Controls.Add(_videoPreviewPanel, 0, 3);
            mainLayout.SetColumnSpan(_videoPreviewPanel, 3);

            // Row 5: EKF Source Control (spans all columns)
            if (_sender != null)
            {
                _ekfControlPanel = new EkfSourceControlPanel(_sender);
                _ekfControlPanel.Dock = DockStyle.Fill;
                _ekfControlPanel.Margin = new Padding(5);
                mainLayout.Controls.Add(_ekfControlPanel, 0, 4);
                mainLayout.SetColumnSpan(_ekfControlPanel, 3);
            }

            this.Controls.Add(mainLayout);
        }

        private Panel CreateStatusCard(string title, string initialValue, out Label titleLabel, out Label valueLabel, Color statusColor)
        {
            var card = ControlFactory.CardPanelWithBorder();

            // Value label - added FIRST so it fills remaining space
            valueLabel = new Label
            {
                Text = initialValue,
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = statusColor,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft,
            };
            card.Controls.Add(valueLabel);

            // Title label at top - added SECOND so it docks at top
            titleLabel = new Label
            {
                Text = title.ToUpper(),
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                ForeColor = TEXT_SECONDARY,
                Dock = DockStyle.Top,
                Height = 20,
                TextAlign = ContentAlignment.BottomLeft,
            };
            card.Controls.Add(titleLabel);

            // Status indicator dot
            var indicator = new Panel
            {
                Size = new Size(12, 12),
                Location = new Point(card.Width - 27, 15),
                BackColor = statusColor,
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
            };
            indicator.Paint += (s, e) =>
            {
                e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
                using (var brush = new SolidBrush(indicator.BackColor))
                {
                    e.Graphics.FillEllipse(brush, 0, 0, 11, 11);
                }
            };
            card.Controls.Add(indicator);

            return card;
        }

        private Panel CreateLinkStatusPanel()
        {
            var panel = ControlFactory.CardPanel(padding: 15);

            var titleLabel = ControlFactory.Label("LINK STATUS", bold: true, foreColor: NOMADTheme.ACCENT, fontSize: 10);
            titleLabel.Location = new Point(15, 15);
            panel.Controls.Add(titleLabel);

            // LTE Status
            _lteIndicator = new Panel
            {
                Size = new Size(16, 16),
                Location = new Point(15, 50),
                BackColor = TEXT_SECONDARY,
            };
            _lteIndicator.Paint += PaintCircle;
            panel.Controls.Add(_lteIndicator);

            _lblLteStatus = ControlFactory.Label("LTE/Tailscale: Unknown", fontSize: 10);
            _lblLteStatus.Location = new Point(40, 48);
            panel.Controls.Add(_lblLteStatus);

            // Radio Status
            _radioIndicator = new Panel
            {
                Size = new Size(16, 16),
                Location = new Point(15, 80),
                BackColor = TEXT_SECONDARY,
            };
            _radioIndicator.Paint += PaintCircle;
            panel.Controls.Add(_radioIndicator);

            _lblRadioStatus = ControlFactory.Label("RadioMaster: Unknown", fontSize: 10);
            _lblRadioStatus.Location = new Point(40, 78);
            panel.Controls.Add(_lblRadioStatus);

            // Active link label
            _lblActiveLink = ControlFactory.Label("Active: --", bold: true, foreColor: NOMADTheme.SUCCESS, fontSize: 11);
            _lblActiveLink.Location = new Point(15, 115);
            panel.Controls.Add(_lblActiveLink);

            return panel;
        }

        private void PaintCircle(object sender, PaintEventArgs e)
        {
            var panel = sender as Panel;
            e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
            using (var brush = new SolidBrush(panel.BackColor))
            {
                e.Graphics.FillEllipse(brush, 0, 0, 15, 15);
            }
        }

        private Panel CreateVideoPreviewPanel()
        {
            // Panel that fills entirely with video - no title, no padding, just video
            var panel = new Panel
            {
                BackColor = Color.FromArgb(15, 15, 18),
                Margin = new Padding(5),
                Dock = DockStyle.Fill,
                Padding = new Padding(0),
            };

            // Video display area fills the entire panel
            _videoPlaceholder = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(15, 15, 18),
            };

            // Initially show "Waiting for Jetson" message
            // Video will only be loaded when Jetson comes online
            _lblVideoStatus = new Label
            {
                Text = "Waiting for Jetson...",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_SECONDARY,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleCenter,
                BackColor = Color.FromArgb(15, 15, 18),
            };
            _videoPlaceholder.Controls.Add(_lblVideoStatus);

            panel.Controls.Add(_videoPlaceholder);

            return panel;
        }
        private Panel CreateHealthSummaryPanel()
        {
            var panel = ControlFactory.CardPanel(padding: 15);

            var titleLabel = ControlFactory.Label("JETSON HEALTH", bold: true, foreColor: NOMADTheme.ACCENT, fontSize: 10);
            titleLabel.Location = new Point(15, 15);
            panel.Controls.Add(titleLabel);

            int yOffset = 50;

            // CPU - stored in class field for real-time updates
            _lblHealthCpu = ControlFactory.Label("CPU: --", fontSize: 10);
            _lblHealthCpu.Location = new Point(15, yOffset);
            panel.Controls.Add(_lblHealthCpu);
            yOffset += 25;

            // GPU
            _lblHealthGpu = ControlFactory.Label("GPU: --", fontSize: 10);
            _lblHealthGpu.Location = new Point(15, yOffset);
            panel.Controls.Add(_lblHealthGpu);
            yOffset += 25;

            // Memory
            _lblHealthMem = ControlFactory.Label("Memory: --", fontSize: 10);
            _lblHealthMem.Location = new Point(15, yOffset);
            panel.Controls.Add(_lblHealthMem);
            yOffset += 25;

            // Disk
            _lblHealthDisk = ControlFactory.Label("Disk: --", fontSize: 10);
            _lblHealthDisk.Location = new Point(15, yOffset);
            panel.Controls.Add(_lblHealthDisk);
            yOffset += 25;

            // Temperature
            _lblHealthTemp = ControlFactory.Label("Temp: --", fontSize: 10);
            _lblHealthTemp.Location = new Point(15, yOffset);
            panel.Controls.Add(_lblHealthTemp);

            return panel;
        }
    }
}
