// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADDashboardView.Layout.cs - Dashboard layout construction
// ============================================================

using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADDashboardView
    {
        private void InitializeUI()
        {
            BackColor = NOMADTheme.BG_DARK;
            Dock = DockStyle.Fill;
            Padding = new Padding(8);
            AutoScroll = true;

            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 3,
                RowCount = 3,
                BackColor = Color.Transparent,
                CellBorderStyle = TableLayoutPanelCellBorderStyle.None,
                Padding = new Padding(0),
                Margin = new Padding(0),
            };

            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33f));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33f));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33f));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 92));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 92));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            mainLayout.Controls.Add(CreateStatusCard("Flight Mode", "UNKNOWN", out _lblFlightMode, NOMADTheme.TEXT_SECONDARY), 0, 0);
            mainLayout.Controls.Add(CreateStatusCard("GPS", "No Fix", out _lblGpsFix, NOMADTheme.WARNING), 1, 0);
            mainLayout.Controls.Add(CreateStatusCard("Battery", "--.- V", out _lblBattery, NOMADTheme.TEXT_SECONDARY), 2, 0);

            mainLayout.Controls.Add(CreateStatusCard("Geofence", "--", out _lblGeofence, NOMADTheme.TEXT_SECONDARY), 0, 1);
            mainLayout.Controls.Add(CreateStatusCard("Links", "--", out _lblLinks, NOMADTheme.TEXT_SECONDARY, fontSize: 10), 1, 1);
            mainLayout.Controls.Add(CreateStatusCard("Core", "--", out _lblCore, NOMADTheme.TEXT_SECONDARY, fontSize: 10), 2, 1);

            _notificationPanel = new NotificationPanel(_notificationService)
            {
                Dock = DockStyle.Fill,
                Margin = new Padding(4),
            };
            mainLayout.Controls.Add(_notificationPanel, 0, 2);
            mainLayout.SetColumnSpan(_notificationPanel, 2);

            _videoPreviewPanel = CreateVideoPreviewPanel();
            mainLayout.Controls.Add(_videoPreviewPanel, 2, 2);
            Controls.Add(mainLayout);
        }

        private Panel CreateStatusCard(string title, string initialValue, out Label valueLabel, Color statusColor, float fontSize = 12f)
        {
            var card = ControlFactory.CardPanelWithBorder();
            card.Margin = new Padding(4);

            valueLabel = new Label
            {
                Text = initialValue,
                Font = new Font("Segoe UI", fontSize, FontStyle.Bold),
                ForeColor = statusColor,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft,
            };
            card.Controls.Add(valueLabel);

            var titleLabel = new Label
            {
                Text = title.ToUpper(),
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Dock = DockStyle.Top,
                Height = 20,
                TextAlign = ContentAlignment.BottomLeft,
            };
            card.Controls.Add(titleLabel);
            return card;
        }

        private Panel CreateVideoPreviewPanel()
        {
            var panel = new Panel
            {
                BackColor = Color.Black,
                Margin = new Padding(4),
                Dock = DockStyle.Fill,
                Padding = new Padding(1),
            };
            panel.Paint += (s, e) =>
            {
                using (var pen = new Pen(NOMADTheme.CARD_BORDER))
                    e.Graphics.DrawRectangle(pen, 0, 0, panel.Width - 1, panel.Height - 1);
            };

            _videoPlaceholder = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
            };
            _lblVideoStatus = new Label
            {
                Text = string.IsNullOrWhiteSpace(_config.VideoUrl)
                    ? "Video: configure a direct RTSP URL"
                    : "Video: ready",
                Font = new Font("Segoe UI", 9),
                ForeColor = NOMADTheme.TEXT_MUTED,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleCenter,
                BackColor = Color.Black,
            };
            _videoPlaceholder.Controls.Add(_lblVideoStatus);
            panel.Controls.Add(_videoPlaceholder);
            return panel;
        }
    }
}
