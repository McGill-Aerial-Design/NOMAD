// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Notification Service
// ============================================================
// Centralized notification system for flight-critical warnings.
// Monitors GPS health, EKF source changes, battery, optical flow,
// and local advisory outline position. Non-intrusive timestamped alerts.
// ============================================================

using System;
using System.Collections.Generic;
using System.Linq;
using System.Timers;

namespace NOMAD.MissionPlanner
{
    public enum NotificationSeverity
    {
        Info,
        Warning,
        Critical
    }

    public enum NotificationCategory
    {
        GPS,
        VIO,
        OpticalFlow,
        EKF,
        Battery,
        Boundary,
        Link,
        System
    }

    public class Notification
    {
        public DateTime Timestamp { get; set; }
        public NotificationSeverity Severity { get; set; }
        public NotificationCategory Category { get; set; }
        public string Title { get; set; }
        public string Message { get; set; }
        public bool IsRead { get; set; }
        public string Id { get; set; }

        public Notification()
        {
            Timestamp = DateTime.Now;
            Id = Guid.NewGuid().ToString("N").Substring(0, 8);
        }

        public string TimestampFormatted => Timestamp.ToString("HH:mm:ss");
    }

    public class NotificationEventArgs : EventArgs
    {
        public Notification Notification { get; set; }
    }

    public partial class NotificationService : IDisposable
    {
        /// <summary>
        /// Process-wide instance set by NOMADPlugin at load time. Views should prefer
        /// this over constructing their own so monitoring runs regardless of the tab.
        /// </summary>
        public static NotificationService Shared { get; set; }

        private const int GPS_MIN_SATS_WARNING = 8;
        private const int GPS_MIN_SATS_CRITICAL = 5;
        private const double GPS_HDOP_WARNING = 2.0;
        private const double GPS_HDOP_CRITICAL = 4.0;
        private const int NOTIFICATION_COOLDOWN_SECONDS = 30;
        private const int MAX_NOTIFICATIONS = 100;

        private readonly List<Notification> _notifications = new List<Notification>();
        private readonly object _lock = new object();
        private readonly Dictionary<string, DateTime> _lastNotificationTime = new Dictionary<string, DateTime>();
        private Timer _monitorTimer;
        private bool _disposed;
        private int _pollGuard;

        private AdvisoryBoundaryMonitor _advisoryBoundaryMonitor;
        private int _lastEkfSource = -1;
        private int _lastGpsFix = -1;
        private readonly Dictionary<int, int> _lastBatterySeverity = new Dictionary<int, int>();
        private readonly Dictionary<int, DateTime> _lastBatterySpeechUtc = new Dictionary<int, DateTime>();
        private static readonly TimeSpan BatterySpeechInterval = TimeSpan.FromSeconds(10);
        private AdvisoryOutlineStatus _lastBoundaryStatus = AdvisoryOutlineStatus.NoPosition;

        public event EventHandler<NotificationEventArgs> NotificationAdded;
        public event EventHandler NotificationsCleared;

        public IReadOnlyList<Notification> Notifications
        {
            get
            {
                lock (_lock)
                {
                    return _notifications.OrderByDescending(n => n.Timestamp).ToList();
                }
            }
        }

        public int UnreadCount
        {
            get
            {
                lock (_lock)
                {
                    return _notifications.Count(n => !n.IsRead);
                }
            }
        }

        public bool IsMonitoring { get; private set; }

        public NotificationService(AdvisoryBoundaryMonitor boundaryMonitor = null)
        {
            SetAdvisoryBoundaryMonitor(boundaryMonitor);
        }

        public void StartMonitoring(int intervalMs = 1000)
        {
            if (IsMonitoring) return;

            _monitorTimer = new Timer(intervalMs);
            _monitorTimer.Elapsed += MonitorTimer_Elapsed;
            _monitorTimer.AutoReset = true;
            _monitorTimer.Start();
            IsMonitoring = true;

            AddNotification(NotificationSeverity.Info, NotificationCategory.System,
                "Monitoring Started", "Notification service is now active");
        }

        public void StopMonitoring()
        {
            if (!IsMonitoring) return;

            _monitorTimer?.Stop();
            _monitorTimer?.Dispose();
            _monitorTimer = null;
            IsMonitoring = false;
        }

        public void AddNotification(NotificationSeverity severity, NotificationCategory category, string title, string message)
        {
            var key = $"{category}_{title}";
            if (IsOnCooldown(key)) return;

            var notification = new Notification
            {
                Severity = severity,
                Category = category,
                Title = title,
                Message = message
            };

            lock (_lock)
            {
                _notifications.Add(notification);
                while (_notifications.Count > MAX_NOTIFICATIONS)
                    _notifications.RemoveAt(0);
                _lastNotificationTime[key] = DateTime.Now;
            }

            NotificationAdded?.Invoke(this, new NotificationEventArgs { Notification = notification });
        }

        public void MarkAllRead()
        {
            lock (_lock)
            {
                foreach (var notification in _notifications)
                    notification.IsRead = true;
            }
        }

        public void ClearAll()
        {
            lock (_lock)
            {
                _notifications.Clear();
            }
            NotificationsCleared?.Invoke(this, EventArgs.Empty);
        }

        public void SetAdvisoryBoundaryMonitor(AdvisoryBoundaryMonitor monitor)
        {
            if (_advisoryBoundaryMonitor != null)
            {
                _advisoryBoundaryMonitor.StatusChanged -= OnAdvisoryBoundaryStatusChanged;
            }

            _advisoryBoundaryMonitor = monitor;
            if (_advisoryBoundaryMonitor != null)
            {
                _advisoryBoundaryMonitor.StatusChanged += OnAdvisoryBoundaryStatusChanged;
            }
        }

        private void OnAdvisoryBoundaryStatusChanged(object sender, AdvisoryBoundaryStatusEventArgs e)
        {
            if (e.Status == _lastBoundaryStatus)
            {
                return;
            }

            if (IsOutsideOutline(e.Status))
            {
                AddNotification(NotificationSeverity.Warning, NotificationCategory.Boundary,
                    "Outside local advisory outline",
                    $"Mission Planner telemetry is outside {e.OutlineName}. " +
                    "This outline is not enforced by NOMAD runtime or the aircraft.");
            }
            else if (e.Status == AdvisoryOutlineStatus.InsideOutlines && IsOutsideOutline(_lastBoundaryStatus))
            {
                AddNotification(NotificationSeverity.Info, NotificationCategory.Boundary,
                    "Position inside local advisory outlines",
                    "The saved outlines are visual references and do not report vehicle safety state.");
            }

            _lastBoundaryStatus = e.Status;
        }

        private static bool IsOutsideOutline(AdvisoryOutlineStatus status)
        {
            return status == AdvisoryOutlineStatus.OutsideInnerOutline ||
                status == AdvisoryOutlineStatus.OutsideOuterOutline;
        }

        private bool IsOnCooldown(string key)
        {
            lock (_lock)
            {
                return _lastNotificationTime.TryGetValue(key, out var lastTime)
                    && (DateTime.Now - lastTime).TotalSeconds < NOTIFICATION_COOLDOWN_SECONDS;
            }
        }

        private string GetGpsFixName(int fix)
        {
            return fix switch
            {
                0 => "No GPS",
                1 => "No Fix",
                2 => "2D Fix",
                3 => "3D Fix",
                4 => "DGPS",
                5 => "RTK Float",
                6 => "RTK Fixed",
                _ => "Unknown"
            };
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            StopMonitoring();
            SetAdvisoryBoundaryMonitor(null);
        }
    }
}
