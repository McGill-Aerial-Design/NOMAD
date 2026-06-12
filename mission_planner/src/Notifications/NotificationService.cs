// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Notification Service
// ============================================================
// Centralized notification system for flight-critical warnings.
// Monitors GPS health, VIO status, EKF source changes, battery,
// and flight boundary proximity. Non-intrusive timestamped alerts.
// ============================================================

using System;
using System.Collections.Generic;
using System.Linq;
using System.Timers;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Notification severity levels.
    /// </summary>
    public enum NotificationSeverity
    {
        Info,
        Warning,
        Critical
    }

    /// <summary>
    /// Notification category for filtering.
    /// </summary>
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

    /// <summary>
    /// A single notification item.
    /// </summary>
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

    /// <summary>
    /// Notification added event args.
    /// </summary>
    public class NotificationEventArgs : EventArgs
    {
        public Notification Notification { get; set; }
    }

    /// <summary>
    /// Central notification service that monitors telemetry and raises alerts.
    /// </summary>
    public partial class NotificationService : IDisposable
    {
        /// <summary>
        /// Process-wide instance set by NOMADPlugin at load time. Views should prefer
        /// this over constructing their own so monitoring (and audio/TTS alerts) runs
        /// regardless of which tab is open.
        /// </summary>
        public static NotificationService Shared { get; set; }

        // ============================================================
        // Constants - Thresholds
        // ============================================================

        // GPS thresholds
        private const int GPS_MIN_SATS_WARNING = 8;
        private const int GPS_MIN_SATS_CRITICAL = 5;
        private const double GPS_HDOP_WARNING = 2.0;
        private const double GPS_HDOP_CRITICAL = 4.0;

        // Battery % thresholds (used when ArduPilot has no capacity-based failsafe set;
        // voltage thresholds are pulled live from BATTn_LOW_VOLT / BATTn_CRT_VOLT params).
        private const double BATTERY_WARNING_PERCENT = 30.0;
        private const double BATTERY_CRITICAL_PERCENT = 15.0;
        // Fallback voltage thresholds if the BATTn_LOW_VOLT param hasn't been received yet.
        // ArduPilot convention: 0 means "disabled"; we treat those as "no voltage check".
        private const double BATTERY_VOLTAGE_FALLBACK_LOW_PER_CELL = 3.5;
        private const double BATTERY_VOLTAGE_FALLBACK_CRT_PER_CELL = 3.3;

        // VIO thresholds
        private const double VIO_CONFIDENCE_WARNING = 0.5;
        private const double VIO_CONFIDENCE_CRITICAL = 0.2;

        // Boundary proximity (meters)
        private const double BOUNDARY_PROXIMITY_WARNING = 20.0;
        private const double BOUNDARY_PROXIMITY_CRITICAL = 10.0;

        // Cooldown to prevent notification spam (seconds)
        private const int NOTIFICATION_COOLDOWN_SECONDS = 30;

        // ============================================================
        // Fields
        // ============================================================

        private readonly List<Notification> _notifications = new List<Notification>();
        private readonly object _lock = new object();
        private readonly Dictionary<string, DateTime> _lastNotificationTime = new Dictionary<string, DateTime>();
        private Timer _monitorTimer;
        private bool _disposed;
        private int _pollGuard;

        // Reference to boundary monitor for events
        private BoundaryMonitor _boundaryMonitor;
        private DualLinkSender _sender;

        // State tracking for change detection
        private int _lastEkfSource = -1;
        private int _lastGpsFix = -1;
        // Per-battery severity 0=ok, 1=warning, 2=critical. Indexed by battery number (1, 2, ...).
        private readonly Dictionary<int, int> _lastBatterySeverity = new Dictionary<int, int>();
        // Per-battery last-spoken time. The voltage in each phrase changes every poll,
        // so the AudioAlerts text-dedup never matches — rate-limit per battery here.
        private readonly Dictionary<int, DateTime> _lastBatterySpeechUtc = new Dictionary<int, DateTime>();
        private static readonly TimeSpan BatterySpeechInterval = TimeSpan.FromSeconds(10);
        private bool _lastVioActive = false;
        private string _lastBoundaryStatus = "inside";

        // Max notifications to keep
        private const int MAX_NOTIFICATIONS = 100;

        // ============================================================
        // Events
        // ============================================================

        /// <summary>
        /// Fired when a new notification is added.
        /// </summary>
        public event EventHandler<NotificationEventArgs> NotificationAdded;

        /// <summary>
        /// Fired when notifications are cleared.
        /// </summary>
        public event EventHandler NotificationsCleared;

        // ============================================================
        // Properties
        // ============================================================

        /// <summary>
        /// Gets all notifications, newest first.
        /// </summary>
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

        /// <summary>
        /// Gets the count of unread notifications.
        /// </summary>
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

        /// <summary>
        /// Is monitoring active.
        /// </summary>
        public bool IsMonitoring { get; private set; }

        // ============================================================
        // Constructor
        // ============================================================

        public NotificationService(BoundaryMonitor boundaryMonitor = null, DualLinkSender sender = null)
        {
            _boundaryMonitor = boundaryMonitor;
            _sender = sender;

            // Subscribe to boundary events if available
            if (_boundaryMonitor != null)
            {
                _boundaryMonitor.BoundaryViolation += OnBoundaryViolation;
                _boundaryMonitor.BoundaryStatusChanged += OnBoundaryStatusChanged;
            }
        }

        // ============================================================
        // Public Methods
        // ============================================================

        /// <summary>
        /// Start monitoring telemetry for notification triggers.
        /// </summary>
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

        /// <summary>
        /// Stop monitoring.
        /// </summary>
        public void StopMonitoring()
        {
            if (!IsMonitoring) return;

            _monitorTimer?.Stop();
            _monitorTimer?.Dispose();
            _monitorTimer = null;
            IsMonitoring = false;
        }

        /// <summary>
        /// Add a notification manually.
        /// </summary>
        public void AddNotification(NotificationSeverity severity, NotificationCategory category, string title, string message)
        {
            // Check cooldown to prevent spam
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

                // Trim old notifications
                while (_notifications.Count > MAX_NOTIFICATIONS)
                {
                    _notifications.RemoveAt(0);
                }

                _lastNotificationTime[key] = DateTime.Now;
            }

            NotificationAdded?.Invoke(this, new NotificationEventArgs { Notification = notification });
        }

        /// <summary>
        /// Mark all notifications as read.
        /// </summary>
        public void MarkAllRead()
        {
            lock (_lock)
            {
                foreach (var n in _notifications)
                {
                    n.IsRead = true;
                }
            }
        }

        /// <summary>
        /// Clear all notifications.
        /// </summary>
        public void ClearAll()
        {
            lock (_lock)
            {
                _notifications.Clear();
            }
            NotificationsCleared?.Invoke(this, EventArgs.Empty);
        }

        /// <summary>
        /// Update sender reference for VIO/Jetson checks.
        /// </summary>
        public void SetSender(DualLinkSender sender)
        {
            _sender = sender;
        }

        /// <summary>
        /// Update boundary monitor reference, re-subscribing event handlers.
        /// </summary>
        public void SetBoundaryMonitor(BoundaryMonitor monitor)
        {
            if (_boundaryMonitor != null)
            {
                _boundaryMonitor.BoundaryViolation -= OnBoundaryViolation;
                _boundaryMonitor.BoundaryStatusChanged -= OnBoundaryStatusChanged;
            }

            _boundaryMonitor = monitor;

            if (_boundaryMonitor != null)
            {
                _boundaryMonitor.BoundaryViolation += OnBoundaryViolation;
                _boundaryMonitor.BoundaryStatusChanged += OnBoundaryStatusChanged;
            }
        }

        // ============================================================
        // Boundary Event Handlers
        // ============================================================

        private void OnBoundaryViolation(object sender, BoundaryViolationEventArgs e)
        {
            var severity = e.BoundaryType == "hard"
                ? NotificationSeverity.Critical
                : NotificationSeverity.Warning;

            AddNotification(severity, NotificationCategory.Boundary,
                $"{e.BoundaryType.ToUpper()} Boundary Violation",
                e.RequiredAction);
        }

        private void OnBoundaryStatusChanged(object sender, BoundaryStatusEventArgs e)
        {
            if (e.Status == _lastBoundaryStatus) return;

            if (e.Status == "inside" && _lastBoundaryStatus != "inside")
            {
                AddNotification(NotificationSeverity.Info, NotificationCategory.Boundary,
                    "Back Inside Boundary", "Drone has returned to safe area");
            }
            else if (e.Status == "soft_violation")
            {
                AddNotification(NotificationSeverity.Warning, NotificationCategory.Boundary,
                    "Approaching Boundary", "Turn around - soft boundary crossed");
            }
            else if (e.Status == "hard_violation")
            {
                AddNotification(NotificationSeverity.Critical, NotificationCategory.Boundary,
                    "HARD BOUNDARY CROSSED", "Forced descent required per competition rules");
            }

            _lastBoundaryStatus = e.Status;
        }

        // ============================================================
        // Helpers
        // ============================================================

        private bool IsOnCooldown(string key)
        {
            lock (_lock)
            {
                if (_lastNotificationTime.TryGetValue(key, out var lastTime))
                {
                    return (DateTime.Now - lastTime).TotalSeconds < NOTIFICATION_COOLDOWN_SECONDS;
                }
                return false;
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

        // ============================================================
        // IDisposable
        // ============================================================

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            StopMonitoring();

            if (_boundaryMonitor != null)
            {
                _boundaryMonitor.BoundaryViolation -= OnBoundaryViolation;
                _boundaryMonitor.BoundaryStatusChanged -= OnBoundaryStatusChanged;
            }
        }
    }
}
