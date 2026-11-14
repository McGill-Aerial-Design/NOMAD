// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADDashboardView.Layout.cs - Dashboard layout construction
// ============================================================

using System.Drawing;
using System.Windows.Forms;
using System.Windows;
using Wpf = System.Windows.Controls;
using Media = System.Windows.Media;

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

            mainLayout.Controls.Add(CreateStatusCard("Links", "--", out _lblLinks, NOMADTheme.TEXT_SECONDARY, fontSize: 10), 0, 1);
            mainLayout.Controls.Add(CreateStatusCard("Gimbal", "Idle", out _lblGimbalCommand,
                NOMADTheme.TEXT_SECONDARY, fontSize: 10), 1, 1);

            _terminationWidget = new TerminationWidget(NOMADMainScreen.Termination, _videoShutdown)
            { Dock = DockStyle.Fill, Margin = new Padding(4) };
            mainLayout.Controls.Add(_terminationWidget, 2, 1);

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

        private Control CreateStatusCard(string title, string initialValue, out Wpf.TextBlock valueLabel, Color statusColor, float fontSize = 12f)
        {
            var content = new Wpf.StackPanel { Margin = new Thickness(10) };
            content.Children.Add(new Wpf.TextBlock { Text = title.ToUpperInvariant(), FontSize = 11,
                FontWeight = FontWeights.Bold, Foreground = WpfTheme.SecondaryText });
            valueLabel = new Wpf.TextBlock { Text = initialValue, FontSize = fontSize * 4 / 3,
                FontWeight = FontWeights.Bold, TextWrapping = TextWrapping.Wrap,
                Foreground = new Media.SolidColorBrush(Media.Color.FromArgb(statusColor.A, statusColor.R, statusColor.G, statusColor.B)) };
            content.Children.Add(valueLabel);
            var card = WpfUi.Host(new Wpf.Border { Background = WpfTheme.Card, BorderBrush = WpfTheme.Outline,
                BorderThickness = new Thickness(1), Child = content });
            card.Margin = new Padding(4);
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
            _videoHost = new VideoInputPlayer(_config, false, _videoShutdown, _createPipeline,
                NOMADMainScreen.SelectVideoInput);
            panel.Controls.Add(_videoHost);
            return panel;
        }
    }
}
