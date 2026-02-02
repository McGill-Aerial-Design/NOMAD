// ============================================================
// NOMAD Notification Panel
// ============================================================
// Scrollable, timestamped notification list for the dashboard.
// Shows critical flight warnings: GPS, VIO, EKF, battery, boundaries.
// Non-intrusive design that fits alongside Quick Actions.
// ============================================================

using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Notification panel UI component showing timestamped flight alerts.
    /// </summary>
    public class NotificationPanel : UserControl
    {
        // ============================================================
        // Constants - delegated to NOMADTheme for consistency
        // ============================================================

        private static readonly Color CARD_BG = NOMADTheme.CARD_BG;
        private static readonly Color CARD_BORDER = NOMADTheme.CARD_BORDER;
        private static readonly Color ACCENT_COLOR = NOMADTheme.ACCENT;
        private static readonly Color SUCCESS_COLOR = NOMADTheme.SUCCESS;
        private static readonly Color WARNING_COLOR = NOMADTheme.WARNING;
        private static readonly Color ERROR_COLOR = NOMADTheme.ERROR;
        private static readonly Color INFO_COLOR = NOMADTheme.INFO;
        private static readonly Color TEXT_PRIMARY = NOMADTheme.TEXT_PRIMARY;
        private static readonly Color TEXT_SECONDARY = NOMADTheme.TEXT_SECONDARY;
        private static readonly Color TEXT_MUTED = NOMADTheme.TEXT_MUTED;

        // ============================================================
        // Fields
        // ============================================================

        private readonly NotificationService _notificationService;
        private Panel _notificationListPanel;
        private Label _lblTitle;
        private Label _lblUnreadCount;
        private Button _btnClear;
        private Panel _headerPanel;

        // ============================================================
        // Constructor
        // ============================================================

        public NotificationPanel(NotificationService notificationService)
        {
            _notificationService = notificationService ?? throw new ArgumentNullException(nameof(notificationService));

            InitializeUI();

            // Subscribe to notification events
            _notificationService.NotificationAdded += OnNotificationAdded;
            _notificationService.NotificationsCleared += OnNotificationsCleared;

            // Load existing notifications
            RefreshNotificationList();
        }

        // ============================================================
        // UI Initialization
        // ============================================================

        private void InitializeUI()
        {
            this.BackColor = CARD_BG;
            this.Dock = DockStyle.Fill;
            this.Padding = new Padding(0);

            // Header panel
            _headerPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 40,
                BackColor = Color.Transparent,
                Padding = new Padding(15, 10, 10, 5),
            };

            _lblTitle = new Label
            {
                Text = "NOTIFICATIONS",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(15, 10),
                AutoSize = true,
            };
            _headerPanel.Controls.Add(_lblTitle);

            _lblUnreadCount = new Label
            {
                Text = "",
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                ForeColor = TEXT_PRIMARY,
                BackColor = ERROR_COLOR,
                AutoSize = true,
                Padding = new Padding(4, 2, 4, 2),
                Visible = false,
            };
            _lblUnreadCount.Location = new Point(_lblTitle.Right + 8, 10);
            _headerPanel.Controls.Add(_lblUnreadCount);

            _btnClear = new Button
            {
                Text = "Clear",
                Size = new Size(50, 22),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = TEXT_SECONDARY,
                Font = new Font("Segoe UI", 8),
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
                Cursor = Cursors.Hand,
            };
            _btnClear.FlatAppearance.BorderSize = 0;
            _btnClear.Location = new Point(this.Width - 65, 8);
            _btnClear.Click += (s, e) =>
            {
                _notificationService.ClearAll();
            };
            _headerPanel.Controls.Add(_btnClear);

            // Scrollable notification list
            _notificationListPanel = new Panel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
                BackColor = Color.Transparent,
                Padding = new Padding(5, 5, 5, 5),
            };
            
            // IMPORTANT: In WinForms, docking order is reverse of add order
            // Add list FIRST (fills remaining space), then header (docks top)
            this.Controls.Add(_notificationListPanel);
            this.Controls.Add(_headerPanel);

            // Handle resize to reposition clear button
            this.Resize += (s, e) =>
            {
                _btnClear.Location = new Point(this.Width - 65, 8);
            };
        }

        // ============================================================
        // Event Handlers
        // ============================================================

        private void OnNotificationAdded(object sender, NotificationEventArgs e)
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke((MethodInvoker)(() => OnNotificationAdded(sender, e)));
                return;
            }

            AddNotificationItem(e.Notification, insertAtTop: true);
            UpdateUnreadCount();
        }

        private void OnNotificationsCleared(object sender, EventArgs e)
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke((MethodInvoker)(() => OnNotificationsCleared(sender, e)));
                return;
            }

            _notificationListPanel.Controls.Clear();
            UpdateUnreadCount();
        }

        // ============================================================
        // Public Methods
        // ============================================================

        /// <summary>
        /// Refresh the notification list from the service.
        /// </summary>
        public void RefreshNotificationList()
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke((MethodInvoker)RefreshNotificationList);
                return;
            }

            _notificationListPanel.Controls.Clear();

            foreach (var notification in _notificationService.Notifications)
            {
                AddNotificationItem(notification, insertAtTop: false);
            }

            UpdateUnreadCount();
        }

        /// <summary>
        /// Mark all notifications as read and update UI.
        /// </summary>
        public void MarkAllAsRead()
        {
            _notificationService.MarkAllRead();
            UpdateUnreadCount();
        }

        // ============================================================
        // Private Methods
        // ============================================================

        private void AddNotificationItem(Notification notification, bool insertAtTop)
        {
            var panel = CreateNotificationItemPanel(notification);

            if (insertAtTop)
            {
                _notificationListPanel.Controls.Add(panel);
                _notificationListPanel.Controls.SetChildIndex(panel, 0);
            }
            else
            {
                _notificationListPanel.Controls.Add(panel);
            }

            // Limit visible items
            while (_notificationListPanel.Controls.Count > 50)
            {
                _notificationListPanel.Controls.RemoveAt(_notificationListPanel.Controls.Count - 1);
            }
        }

        private Panel CreateNotificationItemPanel(Notification notification)
        {
            var panel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 52,
                BackColor = Color.FromArgb(35, 35, 38),
                Margin = new Padding(0, 0, 0, 2),
                Padding = new Padding(8, 6, 8, 6),
                Tag = notification,
            };

            // Severity indicator bar
            var severityBar = new Panel
            {
                Width = 4,
                Dock = DockStyle.Left,
                BackColor = GetSeverityColor(notification.Severity),
            };
            panel.Controls.Add(severityBar);

            // Category icon/badge
            var categoryBadge = new Label
            {
                Text = GetCategoryAbbreviation(notification.Category),
                Font = new Font("Segoe UI", 7, FontStyle.Bold),
                ForeColor = TEXT_PRIMARY,
                BackColor = GetCategoryColor(notification.Category),
                AutoSize = false,
                Size = new Size(28, 16),
                TextAlign = ContentAlignment.MiddleCenter,
                Location = new Point(12, 6),
            };
            panel.Controls.Add(categoryBadge);

            // Timestamp
            var lblTime = new Label
            {
                Text = notification.TimestampFormatted,
                Font = new Font("Segoe UI", 7),
                ForeColor = TEXT_MUTED,
                Location = new Point(45, 6),
                AutoSize = true,
            };
            panel.Controls.Add(lblTime);

            // Title
            var lblTitle = new Label
            {
                Text = notification.Title,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = GetSeverityColor(notification.Severity),
                Location = new Point(12, 22),
                AutoSize = true,
                MaximumSize = new Size(this.Width - 30, 0),
            };
            panel.Controls.Add(lblTitle);

            // Message (truncated if needed)
            var message = notification.Message;
            if (message.Length > 60)
            {
                message = message.Substring(0, 57) + "...";
            }
            var lblMessage = new Label
            {
                Text = message,
                Font = new Font("Segoe UI", 8),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(12, 36),
                AutoSize = true,
                MaximumSize = new Size(this.Width - 30, 0),
            };
            panel.Controls.Add(lblMessage);

            // Hover effect
            panel.MouseEnter += (s, e) => panel.BackColor = Color.FromArgb(45, 45, 50);
            panel.MouseLeave += (s, e) => panel.BackColor = Color.FromArgb(35, 35, 38);

            // Click to mark as read
            panel.Click += (s, e) =>
            {
                notification.IsRead = true;
                UpdateUnreadCount();
            };

            // Apply click handler to all children
            foreach (Control child in panel.Controls)
            {
                child.Click += (s, e) =>
                {
                    notification.IsRead = true;
                    UpdateUnreadCount();
                };
            }

            return panel;
        }

        private void UpdateUnreadCount()
        {
            var count = _notificationService.UnreadCount;
            if (count > 0)
            {
                _lblUnreadCount.Text = count > 99 ? "99+" : count.ToString();
                _lblUnreadCount.Visible = true;
                _lblUnreadCount.Location = new Point(_lblTitle.Right + 8, 10);
            }
            else
            {
                _lblUnreadCount.Visible = false;
            }
        }

        private Color GetSeverityColor(NotificationSeverity severity)
        {
            return severity switch
            {
                NotificationSeverity.Critical => ERROR_COLOR,
                NotificationSeverity.Warning => WARNING_COLOR,
                NotificationSeverity.Info => INFO_COLOR,
                _ => TEXT_SECONDARY
            };
        }

        private Color GetCategoryColor(NotificationCategory category)
        {
            return category switch
            {
                NotificationCategory.GPS => Color.FromArgb(33, 150, 243),
                NotificationCategory.VIO => Color.FromArgb(156, 39, 176),
                NotificationCategory.OpticalFlow => Color.FromArgb(0, 188, 212),
                NotificationCategory.EKF => Color.FromArgb(255, 193, 7),
                NotificationCategory.Battery => Color.FromArgb(244, 67, 54),
                NotificationCategory.Boundary => Color.FromArgb(255, 87, 34),
                NotificationCategory.Link => Color.FromArgb(76, 175, 80),
                NotificationCategory.System => Color.FromArgb(96, 125, 139),
                _ => Color.FromArgb(100, 100, 100)
            };
        }

        private string GetCategoryAbbreviation(NotificationCategory category)
        {
            return category switch
            {
                NotificationCategory.GPS => "GPS",
                NotificationCategory.VIO => "VIO",
                NotificationCategory.OpticalFlow => "OPT",
                NotificationCategory.EKF => "EKF",
                NotificationCategory.Battery => "BAT",
                NotificationCategory.Boundary => "BND",
                NotificationCategory.Link => "LNK",
                NotificationCategory.System => "SYS",
                _ => "???"
            };
        }

        // ============================================================
        // Cleanup
        // ============================================================

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                if (_notificationService != null)
                {
                    _notificationService.NotificationAdded -= OnNotificationAdded;
                    _notificationService.NotificationsCleared -= OnNotificationsCleared;
                }
            }
            base.Dispose(disposing);
        }
    }
}
