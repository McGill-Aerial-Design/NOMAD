// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using Forms = System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public class LinkHealthPanel : Forms.UserControl
    {
        private readonly IRouterStatusProvider _provider;
        private readonly Forms.Timer _refresh;
        private readonly Dictionary<string, LinkCard> _cards = new Dictionary<string, LinkCard>();
        private readonly TextBlock _active = WpfUi.Text("Active: —"), _status = WpfUi.Text("Router: …"),
            _endpoint = WpfUi.Text(""), _copied = WpfUi.Text(""), _override = WpfUi.Text("");
        private readonly WrapPanel _links = new WrapPanel();
        private readonly ListBox _log = new ListBox();
        private readonly Button _release;

        public LinkHealthPanel(IRouterStatusProvider connectionManager, NOMADConfig config)
        {
            _provider = connectionManager ?? throw new ArgumentNullException(nameof(connectionManager));
            if (config == null) throw new ArgumentNullException(nameof(config));
            Dock = Forms.DockStyle.Fill;
            var root = new Grid { Margin = new Thickness(12) };
            root.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });
            root.RowDefinitions.Add(new RowDefinition());
            root.RowDefinitions.Add(new RowDefinition { Height = new GridLength(190) });
            var header = new StackPanel { Background = WpfTheme.Card, Margin = new Thickness(0, 0, 0, 8) };
            header.Children.Add(WpfUi.Section("MAVLink Link Status")); header.Children.Add(_active); header.Children.Add(_status);
            var endpointRow = new WrapPanel(); endpointRow.Children.Add(_endpoint);
            endpointRow.Children.Add(WpfUi.Action("Copy", CopyEndpoint)); endpointRow.Children.Add(_copied);
            header.Children.Add(endpointRow); root.Children.Add(header);
            var scroll = new ScrollViewer { Content = _links, VerticalScrollBarVisibility = ScrollBarVisibility.Auto };
            Grid.SetRow(scroll, 1); root.Children.Add(scroll);
            var bottom = new Grid(); bottom.ColumnDefinitions.Add(new ColumnDefinition()); bottom.ColumnDefinitions.Add(new ColumnDefinition());
            var settings = new WrapPanel { Background = WpfTheme.Card, Margin = new Thickness(0, 4, 4, 0) };
            _release = WpfUi.Action("Release override", () => _provider.SwitchToLink(LinkType.None));
            settings.Children.Add(_release); settings.Children.Add(_override); bottom.Children.Add(settings);
            var logPanel = new DockPanel { Background = WpfTheme.Card, Margin = new Thickness(4, 4, 0, 0) };
            var logHeader = new DockPanel();
            var clear = WpfUi.Action("Clear", () => _log.Items.Clear()); DockPanel.SetDock(clear, System.Windows.Controls.Dock.Right);
            logHeader.Children.Add(clear); logHeader.Children.Add(WpfUi.Section("Failover log"));
            DockPanel.SetDock(logHeader, System.Windows.Controls.Dock.Top); logPanel.Children.Add(logHeader); logPanel.Children.Add(_log);
            Grid.SetColumn(logPanel, 1); bottom.Children.Add(logPanel); Grid.SetRow(bottom, 2); root.Children.Add(bottom);
            Controls.Add(WpfUi.Host(root));
            _provider.LinkStatusChanged += OnLinkStatusChanged; _provider.FailoverOccurred += OnFailoverOccurred;
            _provider.ActiveLinkChanged += OnActiveLinkChanged; _provider.LogMessage += OnLogMessage;
            _refresh = new Forms.Timer { Interval = Math.Max(200, config.LinkMonitorInterval) };
            _refresh.Tick += (s, e) => RefreshAll(); _refresh.Start(); RefreshAll();
        }

        private void OnLinkStatusChanged(object sender, LinkStatusChangedEventArgs e) => UiAsync.RunSync(this, RefreshAll, "LinkStatusChanged");
        private void OnFailoverOccurred(object sender, FailoverEventArgs e) => UiAsync.RunSync(this,
            () => AppendLog($"[{e.Timestamp:HH:mm:ss}] {e.FromLink} → {e.ToLink}   {e.Reason}"), "FailoverOccurred");
        private void OnActiveLinkChanged(object sender, string link) => UiAsync.RunSync(this, RefreshAll, "ActiveLinkChanged");
        private void OnLogMessage(object sender, string message) => UiAsync.RunSync(this,
            () => AppendLog($"[{DateTime.Now:HH:mm:ss}] {message}"), "LogMessage");

        private void RefreshAll()
        {
            try
            {
                var links = _provider.LinkStatistics; var active = _provider.ActiveLink; var ovr = _provider.ManualOverride;
                _active.Text = !_provider.IsRouterAvailable ? "Active: unavailable" : $"Active: {(active == LinkType.None ? "—" : active.ToString())}";
                _active.Foreground = !_provider.IsRouterAvailable || string.IsNullOrEmpty(active) ? Brushes.Orange : WpfTheme.Text;
                _status.Text = LinkStatusDisplay.FormatRouterStatus(_provider.IsMonitoring, links);
                _status.Foreground = _provider.IsRouterAvailable ? WpfTheme.SecondaryText : Brushes.OrangeRed;
                _endpoint.Text = $"Mode: {_provider.RouterMode}   Local: {_provider.LocalMergedEndpoint}   " +
                    "(set Mission Planner to UDP Client / UDPCl to this port)";
                if (LinkStatusDisplay.HasMembershipChanged(links, _cards.Keys))
                {
                    _cards.Clear(); _links.Children.Clear();
                    foreach (var stats in links)
                    {
                        var id = stats.Type; var card = new LinkCard(stats.Name, id);
                        card.SetActiveRequested += (s, e) => _provider.SwitchToLink(id);
                        _cards.Add(id, card); _links.Children.Add(card);
                    }
                }
                foreach (var stats in links)
                    if (_cards.TryGetValue(stats.Type, out var card)) card.Update(stats, active == stats.Type, ovr == stats.Type);
                _override.Text = ovr == LinkType.None ? "" : $"Manual override → {ovr}";
                _release.Visibility = ovr == LinkType.None ? Visibility.Collapsed : Visibility.Visible;
            }
            catch (Exception ex) { System.Diagnostics.Debug.WriteLine($"NOMAD: link panel refresh failed - {ex}"); }
        }
        private void AppendLog(string line)
        {
            _log.Items.Add(line); while (_log.Items.Count > 200) _log.Items.RemoveAt(0);
            _log.ScrollIntoView(_log.Items[_log.Items.Count - 1]);
        }
        private void CopyEndpoint()
        {
            try
            {
                Forms.Clipboard.SetText(_provider.LocalMergedEndpoint); _copied.Text = "copied";
                var timer = new Forms.Timer { Interval = 1500 };
                timer.Tick += (s, e) => { _copied.Text = ""; timer.Stop(); timer.Dispose(); }; timer.Start();
            }
            catch { }
        }
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _provider.LinkStatusChanged -= OnLinkStatusChanged; _provider.FailoverOccurred -= OnFailoverOccurred;
                _provider.ActiveLinkChanged -= OnActiveLinkChanged; _provider.LogMessage -= OnLogMessage;
                _refresh?.Stop(); _refresh?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
