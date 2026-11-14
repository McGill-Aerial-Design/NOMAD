// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using Forms = System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADMainScreen
    {
        private void InitializeUI()
        {
            BackColor = CONTENT_BG;
            Dock = Forms.DockStyle.Fill;
            _contentPanel = new Forms.Panel { Dock = Forms.DockStyle.Fill, BackColor = CONTENT_BG };
            _viewContainer = new Forms.Panel
            { Dock = Forms.DockStyle.Fill, BackColor = CONTENT_BG, Padding = new Forms.Padding(12) };
            _contentPanel.Controls.Add(_viewContainer);
            Controls.Add(_contentPanel);

            var navigation = new StackPanel { Margin = new Thickness(8), Background = WpfTheme.Background };
            navigation.Children.Add(new TextBlock
            {
                Text = "NOMAD", FontSize = 20, FontWeight = FontWeights.Bold,
                Foreground = WpfTheme.Accent, Margin = new Thickness(6, 6, 6, 14),
            });
            _aircraftLabel = WpfUi.Text("Aircraft: " + _config.AircraftName);
            _aircraftLabel.Name = "ActiveAircraft";
            _aircraftLabel.Margin = new Thickness(6, 0, 6, 12);
            navigation.Children.Add(_aircraftLabel);
            _btnDashboard = SidebarButton("Dashboard", () => ShowView("Dashboard"));
            navigation.Children.Add(_btnDashboard);
            navigation.Children.Add(WpfUi.Section("SETUP"));
            _btnBoundaries = SidebarButton("Fence setup", () => ShowView("Boundaries"));
            navigation.Children.Add(_btnBoundaries);
            navigation.Children.Add(WpfUi.Section("TOOLS"));
            _btnVideo = SidebarButton("Video & Payloads", () => ShowView("Video"));
            navigation.Children.Add(_btnVideo);
            _btnLinks = SidebarButton("Link Status", () => ShowView("Links"));
            navigation.Children.Add(_btnLinks);
            navigation.Children.Add(SidebarButton("Gimbal", () => GimbalJoystickWindow.ShowSingleton(_config, FindForm())));
            _btnSimulation = SidebarButton("Testing & Simulation", () => ShowView("Simulation"));
            navigation.Children.Add(_btnSimulation);
            navigation.Children.Add(WpfUi.Section("PREFERENCES"));
            var settings = SidebarButton("Settings…", () =>
            { if (!_videoShutdown.IsCancellationRequested) { _showSettings?.Invoke(); } });
            settings.Name = "OpenSettings";
            navigation.Children.Add(settings);
            var scroll = new ScrollViewer
            {
                Content = navigation, Background = WpfTheme.Background,
                VerticalScrollBarVisibility = ScrollBarVisibility.Auto,
                HorizontalScrollBarVisibility = ScrollBarVisibility.Disabled,
            };
            _sidebarPanel = new Forms.Panel
            { Dock = Forms.DockStyle.Left, Width = SIDEBAR_WIDTH, BackColor = SIDEBAR_BG };
            _sidebarPanel.Controls.Add(WpfUi.Host(scroll));
            Controls.Add(_sidebarPanel);
        }

        private static Button SidebarButton(string caption, System.Action action)
        {
            var button = WpfUi.Action(caption, action);
            button.HorizontalContentAlignment = HorizontalAlignment.Left;
            button.HorizontalAlignment = HorizontalAlignment.Stretch;
            button.MinHeight = 36;
            button.Background = WpfTheme.Background;
            button.Foreground = WpfTheme.SecondaryText;
            button.Margin = new Thickness(0, 1, 0, 1);
            return button;
        }
    }
}
