// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using Forms = System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public class NotificationPanel : Forms.UserControl
    {
        private readonly NotificationService _notificationService;
        private readonly StackPanel _notificationListPanel = new StackPanel();
        private readonly TextBlock _lblUnreadCount = new TextBlock();
        internal FrameworkElement WpfRoot { get; }

        public NotificationPanel(NotificationService notificationService)
        {
            _notificationService = notificationService ?? throw new ArgumentNullException(nameof(notificationService));
            var root = new DockPanel { Background = WpfTheme.Card, Margin = new Thickness(10, 8, 10, 4) };
            WpfRoot = root;
            var header = new DockPanel { Margin = new Thickness(0, 0, 0, 5) };
            DockPanel.SetDock(header, System.Windows.Controls.Dock.Top);
            var clear = WpfUi.Action("Clear", () => _notificationService.ClearAll());
            DockPanel.SetDock(clear, System.Windows.Controls.Dock.Right);
            header.Children.Add(clear);
            var title = new StackPanel { Orientation = Orientation.Horizontal };
            title.Children.Add(WpfUi.Section("NOTIFICATIONS"));
            _lblUnreadCount.Margin = new Thickness(8, 0, 0, 0);
            _lblUnreadCount.Foreground = Brushes.White;
            _lblUnreadCount.Background = Brushes.IndianRed;
            title.Children.Add(_lblUnreadCount);
            header.Children.Add(title);
            root.Children.Add(header);
            root.Children.Add(new ScrollViewer
            {
                VerticalScrollBarVisibility = ScrollBarVisibility.Auto,
                HorizontalScrollBarVisibility = ScrollBarVisibility.Disabled,
                Content = _notificationListPanel,
            });
            Controls.Add(WpfUi.Host(root));
            _notificationService.NotificationAdded += OnNotificationAdded;
            _notificationService.NotificationsCleared += OnNotificationsCleared;
            RefreshNotificationList();
        }

        protected override void OnHandleCreated(EventArgs e)
        {
            base.OnHandleCreated(e);
            RefreshNotificationList();
        }

        private void OnNotificationAdded(object sender, NotificationEventArgs e) => RefreshNotificationList();
        private void OnNotificationsCleared(object sender, EventArgs e) => RefreshNotificationList();

        public void RefreshNotificationList()
        {
            UiAsync.RunSync(this, () =>
            {
                _notificationListPanel.Children.Clear();
                foreach (var notification in _notificationService.Notifications.Take(50))
                {
                    _notificationListPanel.Children.Add(CreateItem(notification));
                }
                UpdateUnreadCount();
            }, "RefreshNotificationList");
        }

        public void MarkAllAsRead()
        {
            _notificationService.MarkAllRead();
            UpdateUnreadCount();
        }

        private FrameworkElement CreateItem(Notification notification)
        {
            var severity = notification.Severity == NotificationSeverity.Critical ? Brushes.IndianRed :
                notification.Severity == NotificationSeverity.Warning ? Brushes.Orange : Brushes.DodgerBlue;
            var row = new DockPanel { Background = WpfTheme.Input, Margin = new Thickness(0, 0, 0, 2),
                Tag = notification, Cursor = System.Windows.Input.Cursors.Hand };
            var bar = new Border { Width = 4, Background = severity, Margin = new Thickness(0, 0, 8, 0) };
            DockPanel.SetDock(bar, System.Windows.Controls.Dock.Left);
            row.Children.Add(bar);
            var meta = new StackPanel { Width = 58, Margin = new Thickness(0, 6, 6, 6) };
            DockPanel.SetDock(meta, System.Windows.Controls.Dock.Left);
            meta.Children.Add(new TextBlock { Text = notification.Category.ToString(), Foreground = WpfTheme.Text,
                FontWeight = FontWeights.Bold, TextTrimming = TextTrimming.CharacterEllipsis });
            meta.Children.Add(new TextBlock { Text = notification.TimestampFormatted, Foreground = WpfTheme.SecondaryText,
                FontSize = 10 });
            row.Children.Add(meta);
            var content = new StackPanel { Margin = new Thickness(0, 6, 8, 6) };
            content.Children.Add(new TextBlock { Text = notification.Title ?? "", Foreground = severity,
                FontWeight = FontWeights.Bold, TextWrapping = TextWrapping.Wrap });
            content.Children.Add(new TextBlock { Text = notification.Message ?? "", Foreground = WpfTheme.SecondaryText,
                TextWrapping = TextWrapping.Wrap });
            row.Children.Add(content);
            row.MouseLeftButtonUp += (sender, args) => { notification.IsRead = true; UpdateUnreadCount(); };
            return row;
        }

        private void UpdateUnreadCount()
        {
            int count = _notificationService.UnreadCount;
            _lblUnreadCount.Text = count > 99 ? "99+" : count.ToString();
            _lblUnreadCount.Visibility = count > 0 ? Visibility.Visible : Visibility.Collapsed;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _notificationService.NotificationAdded -= OnNotificationAdded;
                _notificationService.NotificationsCleared -= OnNotificationsCleared;
            }
            base.Dispose(disposing);
        }
    }
}
