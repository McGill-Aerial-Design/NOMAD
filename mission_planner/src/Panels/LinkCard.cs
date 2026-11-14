// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace NOMAD.MissionPlanner
{
    internal sealed class LinkCard : Border
    {
        private readonly TextBlock _health = new TextBlock { FontSize = 21, FontWeight = FontWeights.Bold };
        private readonly TextBlock _endpoint = new TextBlock();
        private readonly TextBlock _latency = new TextBlock(), _loss = new TextBlock(), _rate = new TextBlock(),
            _frames = new TextBlock(), _rssi = new TextBlock(), _heartbeat = new TextBlock();
        private readonly TextBlock _badge = new TextBlock();
        private readonly SparklinePanel _spark = new SparklinePanel { Height = 60, Margin = new Thickness(0, 4, 0, 6) };
        private readonly Button _active;
        public event EventHandler SetActiveRequested;

        public LinkCard(string title, string type)
        {
            Width = 330; Margin = new Thickness(0, 0, 8, 8); Padding = new Thickness(14, 12, 14, 12);
            Background = WpfTheme.Card;
            var stack = new StackPanel(); Child = stack;
            var heading = new DockPanel();
            DockPanel.SetDock(_badge, Dock.Right); heading.Children.Add(_badge);
            heading.Children.Add(new TextBlock { Text = title, FontSize = 15, FontWeight = FontWeights.Bold });
            stack.Children.Add(heading); stack.Children.Add(_health); stack.Children.Add(_endpoint); stack.Children.Add(_spark);
            var metrics = new Grid();
            metrics.ColumnDefinitions.Add(new ColumnDefinition()); metrics.ColumnDefinitions.Add(new ColumnDefinition());
            for (int i = 0; i < 3; i++) metrics.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            TextBlock[] labels = { _latency, _frames, _loss, _rssi, _rate, _heartbeat };
            for (int i = 0; i < labels.Length; i++)
            { Grid.SetRow(labels[i], i / 2); Grid.SetColumn(labels[i], i % 2); metrics.Children.Add(labels[i]); }
            stack.Children.Add(metrics);
            _active = WpfUi.Action("Set active", () => SetActiveRequested?.Invoke(this, EventArgs.Empty));
            _active.HorizontalAlignment = HorizontalAlignment.Left; _active.Margin = new Thickness(0, 4, 0, 0);
            stack.Children.Add(_active);
        }

        public void Update(LinkStatistics s, bool isActive, bool isOverride)
        {
            _endpoint.Text = string.IsNullOrEmpty(s.Endpoint) ? "—" : s.Endpoint;
            _health.Text = s.IsStale ? "STALE" : (s.IsConnected ? s.Health.ToString().ToUpperInvariant() : "DISCONNECTED");
            _health.Foreground = s.IsStale ? WpfTheme.SecondaryText : HealthColor(s.Health, s.IsConnected);
            _latency.Text = !s.IsStale && s.IsConnected ? $"HB jitter: {s.LatencyMs,5:F0} ms" : "HB jitter:    — ms";
            _loss.Text = !s.IsStale && s.IsConnected ? $"Loss:    {s.PacketLossPercent,5:F1} %" : "Loss:       — %";
            _rate.Text = $"Rate:  {FormatRate(s.DataRateBps)}";
            _frames.Text = $"Frames: {s.PacketsReceived:N0}";
            if (s.PacketsDuplicate > 0) _frames.Text += $"  (+{s.PacketsDuplicate:N0} dup)";
            _heartbeat.Text = !s.IsStale && s.HeartbeatCount > 0
                ? $"HB:    {s.HeartbeatCount:N0}   age {Math.Max(0, (DateTime.UtcNow - s.LastHeartbeat).TotalSeconds):F1}s" : "HB:    —";
            _rssi.Text = s.Rssi.HasValue ? $"RSSI:  {s.Rssi.Value,3}  rem {s.RemRssi.GetValueOrDefault(0),3}" : "RSSI:    —";
            _spark.Push(s.DataRateBps, !s.IsStale && s.IsConnected ? HealthColor(s.Health, true) : WpfTheme.SecondaryText);
            _badge.Visibility = isActive ? Visibility.Visible : Visibility.Collapsed;
            _badge.Text = isOverride ? "ACTIVE (manual)" : "ACTIVE";
            _badge.Foreground = isOverride ? Brushes.Gold : Brushes.LightGreen;
            _active.IsEnabled = !isActive && !s.IsStale && s.IsEnabled;
        }

        private static Brush HealthColor(LinkHealth health, bool connected)
        {
            if (!connected) return Brushes.OrangeRed;
            return health switch
            {
                LinkHealth.Excellent => Brushes.LimeGreen, LinkHealth.Good => Brushes.LightGreen,
                LinkHealth.Fair => Brushes.Gold, LinkHealth.Poor => Brushes.Orange,
                LinkHealth.Critical => Brushes.OrangeRed, _ => Brushes.OrangeRed,
            };
        }
        private static string FormatRate(double value)
        {
            if (value < 1) return "  0 B/s";
            if (value < 1024) return $"{value,5:F0} B/s";
            if (value < 1024 * 1024) return $"{value / 1024.0,5:F1} KB/s";
            return $"{value / (1024.0 * 1024.0),5:F1} MB/s";
        }
    }
}
