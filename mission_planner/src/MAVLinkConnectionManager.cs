// ============================================================
// MAVLink Connection Manager - Dual Link Failover
// ============================================================
// Manages dual MAVLink connections with automatic failover:
// 1. Primary: LTE link via Tailscale (Jetson UDP forward)
// 2. Secondary: RadioMaster transmitter on UDP port 14550
//
// Features:
// - Real-time link quality monitoring
// - Automatic failover on connection loss
// - Manual link switching
// - Link health statistics
// ============================================================

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using MissionPlanner;
using MissionPlanner.Comms;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Link type enumeration for MAVLink connections.
    /// </summary>
    public enum LinkType
    {
        /// <summary>Primary LTE/Tailscale link via Jetson</summary>
        LTE,
        /// <summary>Secondary RadioMaster transmitter link</summary>
        RadioMaster,
        /// <summary>No active link</summary>
        None
    }

    /// <summary>
    /// Link health status for monitoring.
    /// </summary>
    public enum LinkHealth
    {
        Excellent,  // < 50ms latency, 0% packet loss
        Good,       // < 150ms latency, < 2% packet loss  
        Fair,       // < 300ms latency, < 10% packet loss
        Poor,       // < 500ms latency, < 25% packet loss
        Critical,   // > 500ms latency or > 25% packet loss
        Disconnected
    }

    /// <summary>
    /// Statistics for a single MAVLink link.
    /// </summary>
    public class LinkStatistics
    {
        public LinkType Type { get; set; }
        public string Name { get; set; }
        public string Endpoint { get; set; }
        public bool IsConnected { get; set; }
        public LinkHealth Health { get; set; }
        public double LatencyMs { get; set; }
        public double PacketLossPercent { get; set; }
        public long PacketsReceived { get; set; }
        public long PacketsSent { get; set; }
        public long BytesReceived { get; set; }
        public long BytesSent { get; set; }
        public DateTime LastHeartbeat { get; set; }
        public DateTime LastPacketTime { get; set; }
        public int HeartbeatCount { get; set; }
        public double DataRateBps { get; set; }

        public string HealthColor => Health switch
        {
            LinkHealth.Excellent => "#00FF00",
            LinkHealth.Good => "#90EE90",
            LinkHealth.Fair => "#FFD700",
            LinkHealth.Poor => "#FFA500",
            LinkHealth.Critical => "#FF4500",
            LinkHealth.Disconnected => "#FF0000",
            _ => "#808080"
        };

        public string StatusText => IsConnected
            ? $"{Health} ({LatencyMs:F0}ms, {PacketLossPercent:F1}% loss)"
            : "Disconnected";
    }

    /// <summary>
    /// Event arguments for link status changes.
    /// </summary>
    public class LinkStatusChangedEventArgs : EventArgs
    {
        public LinkType Link { get; set; }
        public LinkStatistics Statistics { get; set; }
        public bool IsActive { get; set; }
    }

    /// <summary>
    /// Event arguments for failover events.
    /// </summary>
    public class FailoverEventArgs : EventArgs
    {
        public LinkType FromLink { get; set; }
        public LinkType ToLink { get; set; }
        public string Reason { get; set; }
        public DateTime Timestamp { get; set; }
    }

    /// <summary>
    /// Manages dual MAVLink connections with automatic failover support.
    /// </summary>
    public class MAVLinkConnectionManager : IDisposable
    {
        // ============================================================
        // Constants
        // ============================================================

        /// <summary>Default RadioMaster UDP port</summary>
        public const int RADIOMASTER_PORT = 14550;

        /// <summary>Default LTE/Tailscale UDP port on Jetson</summary>
        public const int LTE_PORT = 14550;

        /// <summary>Heartbeat timeout before considering link dead (seconds)</summary>
        public const double HEARTBEAT_TIMEOUT_SEC = 3.0;

        /// <summary>Minimum time between automatic failovers (seconds)</summary>
        public const double FAILOVER_COOLDOWN_SEC = 5.0;

        /// <summary>Number of missed heartbeats before failover</summary>
        public const int MISSED_HEARTBEATS_THRESHOLD = 3;

        // ============================================================
        // Configuration
        // ============================================================

        /// <summary>Configuration for the connection manager</summary>
        public class ConnectionConfig
        {
            /// <summary>Jetson Tailscale IP for LTE link</summary>
            public string JetsonTailscaleIP { get; set; } = "100.75.218.89";

            /// <summary>UDP port for LTE link on Jetson</summary>
            public int LtePort { get; set; } = LTE_PORT;

            /// <summary>Local port for RadioMaster (typically 14550)</summary>
            public int RadioMasterPort { get; set; } = RADIOMASTER_PORT;

            /// <summary>Enable automatic failover</summary>
            public bool AutoFailoverEnabled { get; set; } = true;

            /// <summary>Preferred link when both available</summary>
            public LinkType PreferredLink { get; set; } = LinkType.LTE;

            /// <summary>Link monitoring interval (ms)</summary>
            public int MonitorIntervalMs { get; set; } = 500;

            /// <summary>Auto-reconnect to preferred link when available</summary>
            public bool AutoReconnectPreferred { get; set; } = true;

            /// <summary>Seconds to wait before switching back to preferred link</summary>
            public int PreferredLinkReconnectDelaySec { get; set; } = 10;
        }

        // ============================================================
        // Fields
        // ============================================================

        private ConnectionConfig _config;
        private LinkType _activeLink = LinkType.None;
        private readonly object _lock = new object();
        private bool _disposed;

        // Link statistics
        private readonly LinkStatistics _lteStats;
        private readonly LinkStatistics _radioMasterStats;

        // Monitoring
        private CancellationTokenSource _monitorCts;
        private Task _monitorTask;
        private DateTime _lastFailoverTime = DateTime.MinValue;
        private DateTime _preferredLinkAvailableSince = DateTime.MinValue;

        // Heartbeat tracking
        private DateTime _lteLastHeartbeat = DateTime.MinValue;
        private DateTime _radioMasterLastHeartbeat = DateTime.MinValue;
        private int _lteMissedHeartbeats = 0;
        private int _radioMasterMissedHeartbeats = 0;

        // Packet tracking for statistics
        private long _ltePacketsTotal = 0;
        private long _ltePacketsLost = 0;
        private long _radioMasterPacketsTotal = 0;
        private long _radioMasterPacketsLost = 0;
        private readonly Queue<double> _lteLatencyHistory = new Queue<double>();
        private readonly Queue<double> _radioMasterLatencyHistory = new Queue<double>();
        private const int LATENCY_HISTORY_SIZE = 20;

        // UDP listeners for monitoring
        private UdpClient _lteMonitor;
        private UdpClient _radioMasterMonitor;

        // ============================================================
        // Events
        // ============================================================

        /// <summary>Fired when link status changes</summary>
        public event EventHandler<LinkStatusChangedEventArgs> LinkStatusChanged;

        /// <summary>Fired when automatic failover occurs</summary>
        public event EventHandler<FailoverEventArgs> FailoverOccurred;

        /// <summary>Fired when active link changes (manual or automatic)</summary>
        public event EventHandler<LinkType> ActiveLinkChanged;

        // ============================================================
        // Properties
        // ============================================================

        /// <summary>Current active link type</summary>
        public LinkType ActiveLink
        {
            get { lock (_lock) return _activeLink; }
        }

        /// <summary>Current configuration</summary>
        public ConnectionConfig Config => _config;

        /// <summary>LTE link statistics</summary>
        public LinkStatistics LteStatistics => _lteStats;

        /// <summary>RadioMaster link statistics</summary>
        public LinkStatistics RadioMasterStatistics => _radioMasterStats;

        /// <summary>Whether monitoring is active</summary>
        public bool IsMonitoring => _monitorTask != null && !_monitorTask.IsCompleted;

        /// <summary>Returns true if LTE link is healthy enough to use</summary>
        public bool IsLteHealthy => _lteStats.IsConnected && 
            _lteStats.Health != LinkHealth.Disconnected && 
            _lteStats.Health != LinkHealth.Critical;

        /// <summary>Returns true if RadioMaster link is healthy enough to use</summary>
        public bool IsRadioMasterHealthy => _radioMasterStats.IsConnected && 
            _radioMasterStats.Health != LinkHealth.Disconnected && 
            _radioMasterStats.Health != LinkHealth.Critical;

        // ============================================================
        // Constructor
        // ============================================================

        public MAVLinkConnectionManager(ConnectionConfig config = null)
        {
            _config = config ?? new ConnectionConfig();

            _lteStats = new LinkStatistics
            {
                Type = LinkType.LTE,
                Name = "LTE/Tailscale",
                Endpoint = $"{_config.JetsonTailscaleIP}:{_config.LtePort}",
                Health = LinkHealth.Disconnected
            };

            _radioMasterStats = new LinkStatistics
            {
                Type = LinkType.RadioMaster,
                Name = "RadioMaster",
                Endpoint = $"localhost:{_config.RadioMasterPort}",
                Health = LinkHealth.Disconnected
            };
        }

        // ============================================================
        // Public Methods
        // ============================================================

        /// <summary>
        /// Update configuration at runtime.
        /// </summary>
        public void UpdateConfig(ConnectionConfig config)
        {
            lock (_lock)
            {
                _config = config ?? throw new ArgumentNullException(nameof(config));
                _lteStats.Endpoint = $"{_config.JetsonTailscaleIP}:{_config.LtePort}";
            }
        }

        /// <summary>
        /// Start monitoring both links.
        /// </summary>
        public void StartMonitoring()
        {
            if (_monitorTask != null && !_monitorTask.IsCompleted)
                return;

            _monitorCts = new CancellationTokenSource();
            _monitorTask = Task.Run(() => MonitorLoop(_monitorCts.Token));
        }

        /// <summary>
        /// Stop monitoring.
        /// </summary>
        public void StopMonitoring()
        {
            _monitorCts?.Cancel();
            try
            {
                _monitorTask?.Wait(2000);
            }
            catch { }

            _lteMonitor?.Close();
            _radioMasterMonitor?.Close();
        }

        /// <summary>
        /// Manually switch to a specific link.
        /// </summary>
        /// <param name="targetLink">Link to switch to</param>
        /// <param name="force">Force switch even if link is unhealthy</param>
        /// <returns>True if switch was successful</returns>
        public bool SwitchToLink(LinkType targetLink, bool force = false)
        {
            lock (_lock)
            {
                if (targetLink == _activeLink)
                    return true;

                var stats = targetLink == LinkType.LTE ? _lteStats : _radioMasterStats;
                
                if (!force && !stats.IsConnected)
                {
                    Console.WriteLine($"NOMAD: Cannot switch to {targetLink} - link not connected");
                    return false;
                }

                var oldLink = _activeLink;
                _activeLink = targetLink;

                Console.WriteLine($"NOMAD: Manual switch from {oldLink} to {targetLink}");
                
                // Attempt to switch Mission Planner's connection
                SwitchMissionPlannerConnection(targetLink);

                ActiveLinkChanged?.Invoke(this, targetLink);
                return true;
            }
        }

        /// <summary>
        /// Get the best available link based on current health.
        /// </summary>
        public LinkType GetBestAvailableLink()
        {
            lock (_lock)
            {
                bool lteOk = IsLteHealthy;
                bool rmOk = IsRadioMasterHealthy;

                // If neither available, return None
                if (!lteOk && !rmOk)
                    return LinkType.None;

                // If only one available, use it
                if (lteOk && !rmOk)
                    return LinkType.LTE;
                if (!lteOk && rmOk)
                    return LinkType.RadioMaster;

                // Both available - use preferred or better health
                if (_config.PreferredLink != LinkType.None)
                    return _config.PreferredLink;

                // Compare health
                return CompareHealth(_lteStats, _radioMasterStats) >= 0 
                    ? LinkType.LTE 
                    : LinkType.RadioMaster;
            }
        }

        /// <summary>
        /// Get combined status summary for display.
        /// </summary>
        public string GetStatusSummary()
        {
            lock (_lock)
            {
                var active = _activeLink == LinkType.None ? "NONE" : _activeLink.ToString();
                var lteStatus = _lteStats.IsConnected ? $"✓ {_lteStats.LatencyMs:F0}ms" : "✗ Offline";
                var rmStatus = _radioMasterStats.IsConnected ? $"✓ {_radioMasterStats.LatencyMs:F0}ms" : "✗ Offline";
                
                return $"Active: {active} | LTE: {lteStatus} | Radio: {rmStatus}";
            }
        }

        /// <summary>
        /// Process incoming MAVLink heartbeat for link monitoring.
        /// Call this from the MAVLink message handler.
        /// </summary>
        public void ProcessHeartbeat(LinkType fromLink)
        {
            lock (_lock)
            {
                var now = DateTime.UtcNow;
                
                if (fromLink == LinkType.LTE)
                {
                    var timeSinceLast = _lteLastHeartbeat == DateTime.MinValue 
                        ? 0 
                        : (now - _lteLastHeartbeat).TotalMilliseconds;
                    
                    _lteLastHeartbeat = now;
                    _lteStats.LastHeartbeat = now;
                    _lteStats.HeartbeatCount++;
                    _lteStats.IsConnected = true;
                    _lteMissedHeartbeats = 0;
                    
                    // Update latency estimate (heartbeat interval)
                    if (timeSinceLast > 0 && timeSinceLast < 5000)
                    {
                        UpdateLatency(_lteLatencyHistory, timeSinceLast / 2); // Approximate RTT
                        _lteStats.LatencyMs = GetAverageLatency(_lteLatencyHistory);
                    }
                    
                    UpdateLinkHealth(_lteStats);
                }
                else if (fromLink == LinkType.RadioMaster)
                {
                    var timeSinceLast = _radioMasterLastHeartbeat == DateTime.MinValue 
                        ? 0 
                        : (now - _radioMasterLastHeartbeat).TotalMilliseconds;
                    
                    _radioMasterLastHeartbeat = now;
                    _radioMasterStats.LastHeartbeat = now;
                    _radioMasterStats.HeartbeatCount++;
                    _radioMasterStats.IsConnected = true;
                    _radioMasterMissedHeartbeats = 0;
                    
                    if (timeSinceLast > 0 && timeSinceLast < 5000)
                    {
                        UpdateLatency(_radioMasterLatencyHistory, timeSinceLast / 2);
                        _radioMasterStats.LatencyMs = GetAverageLatency(_radioMasterLatencyHistory);
                    }
                    
                    UpdateLinkHealth(_radioMasterStats);
                }
            }
        }

        /// <summary>
        /// Track packet statistics for a link.
        /// </summary>
        public void TrackPacket(LinkType link, int bytesReceived, bool wasLost = false)
        {
            lock (_lock)
            {
                var stats = link == LinkType.LTE ? _lteStats : _radioMasterStats;
                
                stats.PacketsReceived++;
                stats.BytesReceived += bytesReceived;
                stats.LastPacketTime = DateTime.UtcNow;
                
                if (link == LinkType.LTE)
                {
                    _ltePacketsTotal++;
                    if (wasLost) _ltePacketsLost++;
                    _lteStats.PacketLossPercent = _ltePacketsTotal > 0 
                        ? (_ltePacketsLost * 100.0 / _ltePacketsTotal) 
                        : 0;
                }
                else
                {
                    _radioMasterPacketsTotal++;
                    if (wasLost) _radioMasterPacketsLost++;
                    _radioMasterStats.PacketLossPercent = _radioMasterPacketsTotal > 0 
                        ? (_radioMasterPacketsLost * 100.0 / _radioMasterPacketsTotal) 
                        : 0;
                }
            }
        }

        // ============================================================
        // Private Methods
        // ============================================================

        private async Task MonitorLoop(CancellationToken ct)
        {
            Console.WriteLine("NOMAD: Link monitoring started");

            while (!ct.IsCancellationRequested)
            {
                try
                {
                    // Check link health
                    CheckLinkHealth();

                    // Auto-failover logic
                    if (_config.AutoFailoverEnabled)
                    {
                        CheckAndPerformFailover();
                    }

                    // Check for return to preferred link
                    if (_config.AutoReconnectPreferred)
                    {
                        CheckPreferredLinkReturn();
                    }

                    // Raise status events
                    RaiseLinkStatusEvents();

                    await Task.Delay(_config.MonitorIntervalMs, ct);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"NOMAD: Monitor error: {ex.Message}");
                    await Task.Delay(1000, ct);
                }
            }

            Console.WriteLine("NOMAD: Link monitoring stopped");
        }

        private void CheckLinkHealth()
        {
            var now = DateTime.UtcNow;

            lock (_lock)
            {
                // Check LTE heartbeat timeout
                if (_lteStats.IsConnected)
                {
                    var elapsed = (now - _lteLastHeartbeat).TotalSeconds;
                    if (elapsed > HEARTBEAT_TIMEOUT_SEC)
                    {
                        _lteMissedHeartbeats++;
                        if (_lteMissedHeartbeats >= MISSED_HEARTBEATS_THRESHOLD)
                        {
                            _lteStats.IsConnected = false;
                            _lteStats.Health = LinkHealth.Disconnected;
                            Console.WriteLine($"NOMAD: LTE link disconnected (no heartbeat for {elapsed:F1}s)");
                        }
                    }
                }

                // Check RadioMaster heartbeat timeout
                if (_radioMasterStats.IsConnected)
                {
                    var elapsed = (now - _radioMasterLastHeartbeat).TotalSeconds;
                    if (elapsed > HEARTBEAT_TIMEOUT_SEC)
                    {
                        _radioMasterMissedHeartbeats++;
                        if (_radioMasterMissedHeartbeats >= MISSED_HEARTBEATS_THRESHOLD)
                        {
                            _radioMasterStats.IsConnected = false;
                            _radioMasterStats.Health = LinkHealth.Disconnected;
                            Console.WriteLine($"NOMAD: RadioMaster link disconnected (no heartbeat for {elapsed:F1}s)");
                        }
                    }
                }

                // Update health based on metrics
                if (_lteStats.IsConnected)
                    UpdateLinkHealth(_lteStats);
                if (_radioMasterStats.IsConnected)
                    UpdateLinkHealth(_radioMasterStats);
            }
        }

        private void CheckAndPerformFailover()
        {
            lock (_lock)
            {
                // Check cooldown
                if ((DateTime.UtcNow - _lastFailoverTime).TotalSeconds < FAILOVER_COOLDOWN_SEC)
                    return;

                var currentStats = _activeLink == LinkType.LTE ? _lteStats : _radioMasterStats;
                var alternateStats = _activeLink == LinkType.LTE ? _radioMasterStats : _lteStats;
                var alternateLink = _activeLink == LinkType.LTE ? LinkType.RadioMaster : LinkType.LTE;

                // Check if current link is failed
                bool currentFailed = !currentStats.IsConnected || 
                                     currentStats.Health == LinkHealth.Disconnected ||
                                     currentStats.Health == LinkHealth.Critical;

                bool alternateBetter = alternateStats.IsConnected && 
                                       alternateStats.Health != LinkHealth.Disconnected &&
                                       alternateStats.Health != LinkHealth.Critical;

                if (currentFailed && alternateBetter)
                {
                    PerformFailover(_activeLink, alternateLink, "Primary link failed");
                }
                // Also failover if alternate is significantly better
                else if (alternateStats.IsConnected && 
                         currentStats.Health == LinkHealth.Poor &&
                         (alternateStats.Health == LinkHealth.Excellent || alternateStats.Health == LinkHealth.Good))
                {
                    PerformFailover(_activeLink, alternateLink, "Better link available");
                }
            }
        }

        private void CheckPreferredLinkReturn()
        {
            lock (_lock)
            {
                if (_activeLink == _config.PreferredLink)
                {
                    _preferredLinkAvailableSince = DateTime.MinValue;
                    return;
                }

                var preferredStats = _config.PreferredLink == LinkType.LTE ? _lteStats : _radioMasterStats;
                
                // Check if preferred link is healthy
                bool preferredHealthy = preferredStats.IsConnected &&
                                        preferredStats.Health != LinkHealth.Disconnected &&
                                        preferredStats.Health != LinkHealth.Critical &&
                                        preferredStats.Health != LinkHealth.Poor;

                if (preferredHealthy)
                {
                    if (_preferredLinkAvailableSince == DateTime.MinValue)
                    {
                        _preferredLinkAvailableSince = DateTime.UtcNow;
                    }
                    else if ((DateTime.UtcNow - _preferredLinkAvailableSince).TotalSeconds >= _config.PreferredLinkReconnectDelaySec)
                    {
                        PerformFailover(_activeLink, _config.PreferredLink, "Preferred link recovered");
                        _preferredLinkAvailableSince = DateTime.MinValue;
                    }
                }
                else
                {
                    _preferredLinkAvailableSince = DateTime.MinValue;
                }
            }
        }

        private void PerformFailover(LinkType from, LinkType to, string reason)
        {
            Console.WriteLine($"NOMAD: FAILOVER {from} -> {to}: {reason}");
            
            _activeLink = to;
            _lastFailoverTime = DateTime.UtcNow;

            // Switch Mission Planner's active connection
            SwitchMissionPlannerConnection(to);

            // Fire events
            FailoverOccurred?.Invoke(this, new FailoverEventArgs
            {
                FromLink = from,
                ToLink = to,
                Reason = reason,
                Timestamp = DateTime.UtcNow
            });

            ActiveLinkChanged?.Invoke(this, to);

            // Show notification
            try
            {
                MainV2.instance?.BeginInvoke((Action)(() =>
                {
                    CustomMessageBox.Show(
                        $"MAVLink Failover: {from} → {to}\nReason: {reason}",
                        "NOMAD Link Failover",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Warning
                    );
                }));
            }
            catch { }
        }

        private void SwitchMissionPlannerConnection(LinkType targetLink)
        {
            // This method would ideally switch Mission Planner's active connection
            // Mission Planner supports multiple MAVLink connections via MainV2.Comports
            // 
            // For now, we provide guidance - the user configures MP with both connections
            // and this manager tracks which should be "active"
            //
            // Future enhancement: Programmatically switch MP's primary connection
            
            try
            {
                // Log the switch request
                Console.WriteLine($"NOMAD: Requesting MP connection switch to {targetLink}");
                
                // Access Mission Planner's connection list
                if (MainV2.Comports != null && MainV2.Comports.Count > 1)
                {
                    // Find the connection matching target link
                    // Convention: First connection is typically the primary (LTE)
                    //             Second connection is secondary (RadioMaster)
                    int targetIndex = targetLink == LinkType.LTE ? 0 : 1;
                    
                    if (targetIndex < MainV2.Comports.Count)
                    {
                        var targetConnection = MainV2.Comports[targetIndex];
                        if (targetConnection?.BaseStream?.IsOpen == true)
                        {
                            // Set as primary connection
                            MainV2.comPort = targetConnection;
                            Console.WriteLine($"NOMAD: Switched to connection {targetIndex} ({targetLink})");
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Error switching MP connection: {ex.Message}");
            }
        }

        private void UpdateLinkHealth(LinkStatistics stats)
        {
            double latency = stats.LatencyMs;
            double loss = stats.PacketLossPercent;

            if (!stats.IsConnected)
            {
                stats.Health = LinkHealth.Disconnected;
            }
            else if (latency < 50 && loss < 0.5)
            {
                stats.Health = LinkHealth.Excellent;
            }
            else if (latency < 150 && loss < 2)
            {
                stats.Health = LinkHealth.Good;
            }
            else if (latency < 300 && loss < 10)
            {
                stats.Health = LinkHealth.Fair;
            }
            else if (latency < 500 && loss < 25)
            {
                stats.Health = LinkHealth.Poor;
            }
            else
            {
                stats.Health = LinkHealth.Critical;
            }
        }

        private void UpdateLatency(Queue<double> history, double latency)
        {
            history.Enqueue(latency);
            while (history.Count > LATENCY_HISTORY_SIZE)
                history.Dequeue();
        }

        private double GetAverageLatency(Queue<double> history)
        {
            if (history.Count == 0) return 0;
            double sum = 0;
            foreach (var l in history) sum += l;
            return sum / history.Count;
        }

        private int CompareHealth(LinkStatistics a, LinkStatistics b)
        {
            // Returns positive if a is better, negative if b is better
            int healthCompare = (int)b.Health - (int)a.Health; // Lower enum = better
            if (healthCompare != 0) return healthCompare;
            
            // Same health level - compare latency
            return (int)(b.LatencyMs - a.LatencyMs); // Lower latency = better
        }

        private void RaiseLinkStatusEvents()
        {
            LinkStatusChanged?.Invoke(this, new LinkStatusChangedEventArgs
            {
                Link = LinkType.LTE,
                Statistics = _lteStats,
                IsActive = _activeLink == LinkType.LTE
            });

            LinkStatusChanged?.Invoke(this, new LinkStatusChangedEventArgs
            {
                Link = LinkType.RadioMaster,
                Statistics = _radioMasterStats,
                IsActive = _activeLink == LinkType.RadioMaster
            });
        }

        // ============================================================
        // IDisposable
        // ============================================================

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            StopMonitoring();
            _monitorCts?.Dispose();
            _lteMonitor?.Dispose();
            _radioMasterMonitor?.Dispose();
        }
    }
}
