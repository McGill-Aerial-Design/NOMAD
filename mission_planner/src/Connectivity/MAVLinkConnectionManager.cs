// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// MAVLink Connection Manager — standalone router management client
// ============================================================
// Observes the separately supervised ground router and projects its
// status into the LinkStatistics / event surface used by the plugin.
// ============================================================

using System;
using System.Collections.Generic;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Snapshot of one link's health for UI consumers.
    /// </summary>
    public class LinkStatistics
    {
        public string Type { get; set; }
        public string Name { get; set; }
        public string Endpoint { get; set; }
        public string TransportType { get; set; }
        public bool IsEnabled { get; set; }
        public bool IsConnected { get; set; }
        public bool IsStale { get; set; }
        public LinkHealth Health { get; set; }
        public double LatencyMs { get; set; }
        public double PacketLossPercent { get; set; }
        public long PacketsReceived { get; set; }   // forwarded (unique) frames
        public long PacketsDuplicate { get; set; }
        public long PacketsSent { get; set; }       // outbound bytes count, just informational
        public long BytesReceived { get; set; }
        public long BytesSent { get; set; }
        public DateTime LastHeartbeat { get; set; }
        public DateTime LastPacketTime { get; set; }
        public int HeartbeatCount { get; set; }
        public double DataRateBps { get; set; }
        public int? Rssi { get; set; }
        public int? RemRssi { get; set; }

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
            ? $"{Health} (HB jitter {LatencyMs:F0}ms, {PacketLossPercent:F1}% loss)"
            : "Disconnected";
    }

    public class LinkStatusChangedEventArgs : EventArgs
    {
        public string Link { get; set; }
        public LinkStatistics Statistics { get; set; }
        public bool IsActive { get; set; }
    }

    public class LinkStatusSnapshot
    {
        public bool LTEConnected { get; set; }
        public double LTELatencyMs { get; set; }
        public double LTEPacketLoss { get; set; }
        public bool RadioConnected { get; set; }
        public double RadioLatencyMs { get; set; }
        public double RadioPacketLoss { get; set; }
        public string ActiveLink { get; set; }
    }

    /// <summary>
    /// Status and safe link-selection surface for the standalone router.
    /// </summary>
    public interface IRouterStatusProvider : IDisposable
    {
        string RouterMode { get; }
        bool IsMonitoring { get; }
        bool IsRouterAvailable { get; }
        bool IsStatusStale { get; }
        string ActiveLink { get; }
        string ManualOverride { get; }
        string LocalMergedEndpoint { get; }
        IReadOnlyList<LinkStatistics> LinkStatistics { get; }
        IReadOnlyCollection<FailoverEventArgs> FailoverLog { get; }

        bool SwitchToLink(string target);

        event EventHandler<LinkStatusChangedEventArgs> LinkStatusChanged;
        event EventHandler<FailoverEventArgs> FailoverOccurred;
        event EventHandler<string> ActiveLinkChanged;
        event EventHandler<string> LogMessage;
    }

    public partial class MAVLinkConnectionManager : IRouterStatusProvider
    {
        // ============================================================
        // Configuration
        // ============================================================

        /// <summary>Local Mission Planner client endpoints for the standalone router.</summary>
        public class ConnectionConfig
        {
            public int RouterLocalPort { get; set; } = 14600;
            public int ManagementPort { get; set; } = 14610;
        }

        // ============================================================
        // State
        // ============================================================

        private ConnectionConfig _config;
        private StandaloneRouterClient _standalone;
        private readonly object _lock = new object();
        private bool _disposed;

        private readonly LinkStatistics _lteStats = new LinkStatistics
        {
            Type = LinkType.LTE,
            Name = "LTE / Tailscale",
            Health = LinkHealth.Disconnected,
        };
        private readonly LinkStatistics _radioStats = new LinkStatistics
        {
            Type = LinkType.RadioMaster,
            Name = "RadioMaster",
            Health = LinkHealth.Disconnected,
        };

        // ============================================================
        // Events
        // ============================================================

        public event EventHandler<LinkStatusChangedEventArgs> LinkStatusChanged;
        public event EventHandler<FailoverEventArgs> FailoverOccurred;
        public event EventHandler<string> ActiveLinkChanged;
        public event EventHandler<string> LogMessage;

        // ============================================================
        // Properties
        // ============================================================

        public string RouterMode => "Standalone";
        public string ActiveLink => _standalone?.ActiveLink ?? LinkType.None;
        public string ManualOverride => _standalone?.ManualOverride ?? LinkType.None;
        public IReadOnlyList<LinkStatistics> LinkStatistics =>
            _standalone?.LinkStatistics ?? Array.Empty<LinkStatistics>();
        public LinkStatistics LteStatistics => FindLink(LinkType.LTE) ?? _lteStats;
        public LinkStatistics RadioMasterStatistics => FindLink(LinkType.RadioMaster) ?? _radioStats;
        public bool IsMonitoring => _standalone?.IsMonitoring == true;
        public bool IsRouterAvailable => _standalone?.IsRouterAvailable == true;
        public bool IsStatusStale => _standalone?.IsStatusStale == true;
        public string LocalMergedEndpoint => $"udp://127.0.0.1:{_config.RouterLocalPort}";

        public IReadOnlyCollection<FailoverEventArgs> FailoverLog =>
            _standalone?.FailoverLog ??
            (IReadOnlyCollection<FailoverEventArgs>)Array.Empty<FailoverEventArgs>();

        public bool IsLteHealthy => IsHealthy(LteStatistics);

        public bool IsRadioMasterHealthy => IsHealthy(RadioMasterStatistics);

        private static bool IsHealthy(LinkStatistics stats) => stats.IsConnected &&
            stats.Health != LinkHealth.Disconnected && stats.Health != LinkHealth.Critical;

        public LinkStatusSnapshot GetLinkStatus()
        {
            lock (_lock)
            {
                var links = LinkStatistics;
                var lte = FindLink(links, LinkType.LTE);
                var radio = FindLink(links, LinkType.RadioMaster);

                return new LinkStatusSnapshot
                {
                    LTEConnected = lte?.IsConnected == true && !lte.IsStale,
                    LTELatencyMs = lte?.LatencyMs ?? 0,
                    LTEPacketLoss = lte?.PacketLossPercent ?? 0,
                    RadioConnected = radio?.IsConnected == true && !radio.IsStale,
                    RadioLatencyMs = radio?.LatencyMs ?? 0,
                    RadioPacketLoss = radio?.PacketLossPercent ?? 0,
                    ActiveLink = ActiveLink
                };
            }
        }

        // ============================================================
        // Construction
        // ============================================================

        public MAVLinkConnectionManager(ConnectionConfig config = null)
        {
            _config = config ?? new ConnectionConfig();
        }

        public void UpdateConfig(ConnectionConfig config)
        {
            lock (_lock)
            {
                _config = config ?? throw new ArgumentNullException(nameof(config));
            }
        }

        // ============================================================
        // Lifecycle
        // ============================================================

        public void StartMonitoring()
        {
            if (_standalone == null)
            {
                StartManagementClient();
                return;
            }
            _standalone.Start();
        }

        public void StopMonitoring()
        {
            try { _standalone?.Stop(); } catch { }
        }

        /// <summary>
        /// Reconnect the management client after its local endpoint changes.
        /// This does not restart the separately supervised router process.
        /// </summary>
        public void RestartManagementClient()
        {
            StopMonitoring();
            _standalone?.Dispose();
            _standalone = null;
            StartManagementClient();
        }

        /// <summary>
        /// Manual override of the active outbound link. Pass LinkType.None to
        /// release the override and resume auto-failover.
        /// </summary>
        public bool SwitchToLink(string target)
        {
            return _standalone?.SwitchToLink(target) == true;
        }

        public string GetStatusSummary() => _standalone?.GetStatusSummary()
            ?? "Standalone router management unavailable";

        private LinkStatistics FindLink(string id) => FindLink(LinkStatistics, id);

        private static LinkStatistics FindLink(IEnumerable<LinkStatistics> links, string id) =>
            links.FirstOrDefault(link => string.Equals(link.Type, id, StringComparison.OrdinalIgnoreCase));

        // ============================================================
        // IDisposable
        // ============================================================

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            try { _standalone?.Dispose(); } catch { }
            _standalone = null;
        }
    }
}
