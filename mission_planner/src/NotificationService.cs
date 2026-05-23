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
using System.Threading.Tasks;
using System.Timers;
using MissionPlanner;

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
    public class NotificationService : IDisposable
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
        /// Update boundary monitor reference.
        /// </summary>
        public void SetBoundaryMonitor(BoundaryMonitor monitor)
        {
            // Unsubscribe from old
            if (_boundaryMonitor != null)
            {
                _boundaryMonitor.BoundaryViolation -= OnBoundaryViolation;
                _boundaryMonitor.BoundaryStatusChanged -= OnBoundaryStatusChanged;
            }

            _boundaryMonitor = monitor;

            // Subscribe to new
            if (_boundaryMonitor != null)
            {
                _boundaryMonitor.BoundaryViolation += OnBoundaryViolation;
                _boundaryMonitor.BoundaryStatusChanged += OnBoundaryStatusChanged;
            }
        }

        /// <summary>
        /// Update sender reference for VIO/Jetson checks.
        /// </summary>
        public void SetSender(DualLinkSender sender)
        {
            _sender = sender;
        }

        // ============================================================
        // Monitoring Logic
        // ============================================================

        private async void MonitorTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            // Guard against reentrant execution if a previous poll is still running
            if (System.Threading.Interlocked.CompareExchange(ref _pollGuard, 1, 0) != 0)
                return;
            
            try
            {
                CheckGPSHealth();
                CheckBatteryHealth();
                CheckEKFSource();
                await CheckVIOHealthAsync().ConfigureAwait(false);
                CheckOpticalFlowHealth();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NotificationService error: {ex.Message}");
            }
            finally
            {
                System.Threading.Interlocked.Exchange(ref _pollGuard, 0);
            }
        }

        private void CheckGPSHealth()
        {
            var cs = MainV2.comPort?.MAV?.cs;
            if (cs == null) return;

            int satCount = (int)cs.satcount;
            int gpsFix = (int)cs.gpsstatus;
            double hdop = cs.gpshdop;

            // Check satellite count
            if (satCount < GPS_MIN_SATS_CRITICAL && satCount > 0)
            {
                AddNotification(NotificationSeverity.Critical, NotificationCategory.GPS,
                    "GPS Critical", $"Only {satCount} satellites visible - position unreliable");
            }
            else if (satCount < GPS_MIN_SATS_WARNING && satCount > 0)
            {
                AddNotification(NotificationSeverity.Warning, NotificationCategory.GPS,
                    "GPS Low Sats", $"{satCount} satellites - consider better position");
            }

            // Check GPS fix type changes
            if (_lastGpsFix != -1 && gpsFix != _lastGpsFix)
            {
                string fixName = GetGpsFixName(gpsFix);
                string lastFixName = GetGpsFixName(_lastGpsFix);

                if (gpsFix < _lastGpsFix)
                {
                    AddNotification(NotificationSeverity.Warning, NotificationCategory.GPS,
                        "GPS Fix Degraded", $"GPS changed from {lastFixName} to {fixName}");
                }
                else if (gpsFix > _lastGpsFix && gpsFix >= 3)
                {
                    AddNotification(NotificationSeverity.Info, NotificationCategory.GPS,
                        "GPS Fix Improved", $"GPS now has {fixName}");
                }
            }
            _lastGpsFix = gpsFix;

            // Check HDOP
            if (hdop > GPS_HDOP_CRITICAL && hdop < 99)
            {
                AddNotification(NotificationSeverity.Critical, NotificationCategory.GPS,
                    "GPS HDOP Critical", $"HDOP {hdop:F1} - position accuracy degraded");
            }
            else if (hdop > GPS_HDOP_WARNING && hdop < 99)
            {
                AddNotification(NotificationSeverity.Warning, NotificationCategory.GPS,
                    "GPS HDOP High", $"HDOP {hdop:F1} - reduced accuracy");
            }
        }

        private void CheckBatteryHealth()
        {
            var mav = MainV2.comPort?.MAV;
            if (mav?.cs == null) return;

            // Walk BATT1 and BATT2. ArduPilot supports up to 9, but we only
            // care about the two configured on this airframe.
            for (int idx = 1; idx <= 2; idx++)
            {
                CheckOneBattery(mav, idx);
            }
        }

        private void CheckOneBattery(dynamic mav, int idx)
        {
            // Skip if not configured on the vehicle (BATTn_MONITOR == 0 = Disabled).
            double monitor = GetBattParam(mav, idx, "MONITOR");
            if (monitor <= 0) return;

            // Live readings from CurrentState. MP exposes per-battery fields as
            // battery_voltage / battery_voltage2 / ..., same for _remaining.
            double voltage = GetCsDouble(mav.cs, idx == 1 ? "battery_voltage" : $"battery_voltage{idx}");
            double percent = GetCsDouble(mav.cs, idx == 1 ? "battery_remaining" : $"battery_remaining{idx}");

            // Thresholds: prefer the vehicle's own params; fall back to per-cell
            // estimate from BATTn_CELL_COUNT if voltage params aren't set.
            double lowV = GetBattParam(mav, idx, "LOW_VOLT");
            double crtV = GetBattParam(mav, idx, "CRT_VOLT");
            if (lowV <= 0 || crtV <= 0)
            {
                double cells = GetBattParam(mav, idx, "CELL_COUNT");
                if (cells <= 0)
                {
                    // Best-effort guess from voltage: nominal 3.7 V/cell
                    if (voltage > 0) cells = Math.Max(1, Math.Round(voltage / 3.7));
                }
                if (cells > 0)
                {
                    if (lowV <= 0) lowV = cells * BATTERY_VOLTAGE_FALLBACK_LOW_PER_CELL;
                    if (crtV <= 0) crtV = cells * BATTERY_VOLTAGE_FALLBACK_CRT_PER_CELL;
                }
            }

            string label = $"BATT{idx}";
            int severity = 0;
            string detail = $"{label}: {voltage:F1}V";
            if (percent > 0) detail += $" ({percent:F0}%)";

            // Voltage check (ArduPilot semantics: 0 = disabled)
            if (crtV > 0 && voltage > 0 && voltage <= crtV)
            {
                severity = 2;
                AddNotification(NotificationSeverity.Critical, NotificationCategory.Battery,
                    $"{label} Voltage Critical",
                    $"{detail} ≤ CRT_VOLT {crtV:F1}V — LAND NOW");
            }
            else if (lowV > 0 && voltage > 0 && voltage <= lowV)
            {
                severity = 1;
                AddNotification(NotificationSeverity.Warning, NotificationCategory.Battery,
                    $"{label} Voltage Low",
                    $"{detail} ≤ LOW_VOLT {lowV:F1}V");
            }

            // Percent check (only escalates, never downgrades severity from voltage)
            if (percent > 0 && percent <= BATTERY_CRITICAL_PERCENT && severity < 2)
            {
                severity = 2;
                AddNotification(NotificationSeverity.Critical, NotificationCategory.Battery,
                    $"{label} Critical", $"{detail} — LAND IMMEDIATELY");
            }
            else if (percent > 0 && percent <= BATTERY_WARNING_PERCENT && severity < 1)
            {
                severity = 1;
                AddNotification(NotificationSeverity.Warning, NotificationCategory.Battery,
                    $"{label} Low", $"{detail}");
            }

            // Audio alerts (per battery, transition-driven for warning, repeating for critical).
            int last = _lastBatterySeverity.TryGetValue(idx, out var v) ? v : 0;
            if (severity == 2)
            {
                AudioAlerts.Play(AlertKind.BatteryCritical);
                if (CanSpeakBattery(idx))
                    AudioAlerts.Speak($"Battery {idx} critical, {voltage:F1} volts. Land now.",
                        component: $"battery.{idx}", ignoreRateLimit: true);
            }
            else if (severity == 1 && last < 1)
            {
                AudioAlerts.Play(AlertKind.BatteryWarning);
                if (CanSpeakBattery(idx))
                    AudioAlerts.Speak($"Battery {idx} low, {voltage:F1} volts.",
                        component: $"battery.{idx}", ignoreRateLimit: true);
            }
            _lastBatterySeverity[idx] = severity;
        }

        private bool CanSpeakBattery(int idx)
        {
            var now = DateTime.UtcNow;
            if (_lastBatterySpeechUtc.TryGetValue(idx, out var last) && now - last < BatterySpeechInterval)
                return false;
            _lastBatterySpeechUtc[idx] = now;
            return true;
        }

        /// <summary>
        /// Read BATTn_XXX (or BATT_XXX when n==1) from MAVLink params. Returns 0 if missing.
        /// </summary>
        private static double GetBattParam(dynamic mav, int idx, string suffix)
        {
            try
            {
                string name = (idx == 1 ? "BATT_" : $"BATT{idx}_") + suffix;
                var param = mav.param;
                if (param == null) return 0;
                // MP's MAVLinkParamList exposes ContainsKey / indexer with MAVLinkParam values.
                var containsKey = param.GetType().GetMethod("ContainsKey", new[] { typeof(string) });
                if (containsKey != null)
                {
                    bool has = (bool)containsKey.Invoke(param, new object[] { name });
                    if (!has) return 0;
                }
                var indexer = param.GetType().GetProperty("Item", new[] { typeof(string) });
                var entry = indexer?.GetValue(param, new object[] { name });
                if (entry == null) return 0;
                // entry is MAVLinkParam — has Value (double or float) or implicit conversion.
                var valueProp = entry.GetType().GetProperty("Value");
                if (valueProp != null)
                {
                    var raw = valueProp.GetValue(entry);
                    return Convert.ToDouble(raw);
                }
                return Convert.ToDouble(entry);
            }
            catch
            {
                return 0;
            }
        }

        /// <summary>
        /// Read a numeric field/property by name from CurrentState. Returns 0 if missing.
        /// </summary>
        private static double GetCsDouble(object cs, string name)
        {
            try
            {
                var t = cs.GetType();
                var prop = t.GetProperty(name);
                if (prop != null) return Convert.ToDouble(prop.GetValue(cs));
                var field = t.GetField(name);
                if (field != null) return Convert.ToDouble(field.GetValue(cs));
            }
            catch { }
            return 0;
        }

        private void CheckEKFSource()
        {
            var cs = MainV2.comPort?.MAV?.cs;
            if (cs == null) return;

            // Try to read EK3_SRC1_POSZ or similar parameter indicating EKF source
            // ArduPilot uses EK3_SRC parameters for position/velocity/yaw sources
            try
            {
                // Check ekf_status_report or similar
                // EKF source changes are typically indicated through MAVLink EKF_STATUS_REPORT
                // For now, we track changes in position estimate sources

                // Check if using GPS vs VIO based on what's active
                var ekfFlags = (int)cs.ekfstatus;
                
                // Bit 0: attitude ok, Bit 1: velocity horiz ok, Bit 2: velocity vert ok
                // Bit 3: pos horiz rel ok, Bit 4: pos horiz abs ok, Bit 5: pos vert abs ok
                // Bit 6: pos vert agl ok, Bit 7: const pos mode

                bool posRelOk = (ekfFlags & 0x08) != 0;
                bool posAbsOk = (ekfFlags & 0x10) != 0;

                // Derive a simple "source" indicator
                int currentSource = posAbsOk ? 1 : (posRelOk ? 2 : 0);  // 1=GPS, 2=Relative, 0=None

                if (_lastEkfSource != -1 && currentSource != _lastEkfSource)
                {
                    string sourceName = currentSource switch
                    {
                        1 => "GPS (Absolute)",
                        2 => "Relative (VIO/OptFlow)",
                        _ => "None/Degraded"
                    };

                    var severity = currentSource == 0 ? NotificationSeverity.Critical : NotificationSeverity.Warning;
                    AddNotification(severity, NotificationCategory.EKF,
                        "EKF Source Changed", $"Position source: {sourceName}");
                }
                _lastEkfSource = currentSource;
            }
            catch
            {
                // Ignore EKF check errors
            }
        }

        private async Task CheckVIOHealthAsync()
        {
            if (_sender == null) return;

            var health = _sender.LastHealthStatus;

            // Query actual VIO status from the API endpoint instead of
            // inferring it from Jetson connectivity alone.
            bool vioActive = false;
            try
            {
                var vioResult = await _sender.GetVioStatusAsync().ConfigureAwait(false);
                if (vioResult.Success && !string.IsNullOrEmpty(vioResult.Data))
                {
                    var vioData = Newtonsoft.Json.Linq.JObject.Parse(vioResult.Data);
                    var vioHealth = (string)vioData["health"] ?? "unknown";
                    vioActive = vioHealth == "healthy";
                }
            }
            catch
            {
                // If the VIO status endpoint is unreachable, treat as inactive
                vioActive = false;
            }

            // Detect VIO activation/deactivation
            if (vioActive != _lastVioActive)
            {
                if (vioActive)
                {
                    AddNotification(NotificationSeverity.Info, NotificationCategory.VIO,
                        "VIO Active", "Visual-Inertial Odometry is now running");
                }
                else
                {
                    AddNotification(NotificationSeverity.Warning, NotificationCategory.VIO,
                        "VIO Offline", "Visual-Inertial Odometry lost - check Jetson");
                }
            }
            _lastVioActive = vioActive;

            // If VIO is active but Jetson temp is high, warn
            if (vioActive && health != null)
            {
                if (health.GpuTemp > 85 || health.CpuTemp > 85)
                {
                    AddNotification(NotificationSeverity.Warning, NotificationCategory.VIO,
                        "Jetson Overheating", $"Temperature: {Math.Max(health.GpuTemp, health.CpuTemp):F0}C - VIO may throttle");
                }
            }
        }
        
        /// <summary>
        /// Synchronous wrapper kept for callers outside the async timer path.
        /// </summary>
        private void CheckVIOHealth()
        {
            CheckVIOHealthAsync().GetAwaiter().GetResult();
        }

        private void CheckOpticalFlowHealth()
        {
            var cs = MainV2.comPort?.MAV?.cs;
            if (cs == null) return;

            try
            {
                // ArduPilot reports optical flow quality in cs.opt_m_x/y or via OPTICAL_FLOW message
                // Check if optical flow sensor is present and quality
                var optFlowQuality = cs.opt_m_x;  // This might be flow quality depending on setup

                // Most setups use rangefinder with optical flow
                var rangeFinderDist = cs.sonarrange;
                var rangeFinderHealthy = rangeFinderDist > 0 && rangeFinderDist < 100;  // Valid range

                // Only warn if we appear to have optical flow configured but it's degraded
                if (!rangeFinderHealthy && rangeFinderDist > 0)
                {
                    AddNotification(NotificationSeverity.Warning, NotificationCategory.OpticalFlow,
                        "Rangefinder Issue", $"Rangefinder reading abnormal: {rangeFinderDist:F1}m");
                }
            }
            catch
            {
                // Ignore optical flow errors
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
                    "HARD BOUNDARY CROSSED", "Kill switch required per competition rules");
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
