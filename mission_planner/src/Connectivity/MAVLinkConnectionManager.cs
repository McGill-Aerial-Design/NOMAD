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
        MAVLinkConnectionManager.ConnectionConfig Config { get; }
        string RouterMode { get; }
        bool IsMonitoring { get; }
        bool IsRouterAvailable { get; }
        bool IsStatusStale { get; }
        bool SupportsLiveConfiguration { get; }
        string ActiveLink { get; }
        string ManualOverride { get; }
        string LocalMergedEndpoint { get; }
        IReadOnlyList<LinkStatistics> LinkStatistics { get; }
        IReadOnlyCollection<FailoverEventArgs> FailoverLog { get; }

        bool SwitchToLink(string target);
        void SetAutoFailoverEnabled(bool enabled);
        void SetAutoReconnectPreferred(bool enabled);
        void SetDedupEnabled(bool enabled);
        void SetPreferredLink(string link);
        void ResetCounters();

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

        /// <summary>
        /// Local Mission Planner settings for the external router client.
        /// Link definitions seed status while the management endpoint is unavailable.
        /// </summary>
        public class ConnectionConfig
        {
            public List<LinkConfig> Links { get; set; }
            public int LtePort { get; set; } = 14560;
            public string LteRemoteHost { get; set; } = "";
            public int LteRemotePort { get; set; } = 0;

            public string RadioMasterConnectionType { get; set; } = "UDP";
            public int RadioMasterPort { get; set; } = 14550;
            public string RadioMasterComPort { get; set; } =
                Environment.OSVersion.Platform == PlatformID.Win32NT ? "COM3" : "/dev/ttyUSB0";
            public int RadioMasterBaudRate { get; set; } = 420000;
            public string RadioMasterTcpHost { get; set; } = "127.0.0.1";

            public bool AutoFailoverEnabled { get; set; } = true;
            public string PreferredLink { get; set; } = LinkType.LTE;
            public bool AutoReconnectPreferred { get; set; } = true;
            public int PreferredLinkReconnectDelaySec { get; set; } = 10;
            public int MonitorIntervalMs { get; set; } = 250;
            public double HeartbeatTimeoutSec { get; set; } = 3.0;

            // Router endpoint (where MP connects to as UDPCl)
            public string RouterMode { get; set; } = "Standalone";
            public string RouterBindAddress { get; set; } = "127.0.0.1";
            public int RouterLocalPort { get; set; } = 14600;
            public bool RouterDedupEnabled { get; set; } = true;
            public string ManagementBindAddress { get; set; } = "127.0.0.1";
            public int ManagementPort { get; set; } = 14610;
        }

        // ============================================================
        // State
        // ============================================================

        private ConnectionConfig _config;
        private StandaloneRouterClient _standalone;
        private readonly object _lock = new object();
        private bool _disposed;

        private readonly LinkStatistics _lteStats;
        private readonly LinkStatistics _radioStats;

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
        public ConnectionConfig Config => _config;
        public IReadOnlyList<LinkStatistics> LinkStatistics => _standalone != null
            ? _standalone.LinkStatistics
            : (_config.Links == null ? new[] { _lteStats, _radioStats } :
                _config.Links.Select(l => new LinkStatistics { Type = l.Id, Name = l.Name ?? l.Id,
                    Endpoint = l.Transport == "COM" ? l.Device : l.Transport + ":" + l.Port,
                    TransportType = l.Transport, IsEnabled = l.Enabled, Health = LinkHealth.Disconnected }).ToArray());
        public LinkStatistics LteStatistics => FindLink(LinkType.LTE) ?? _lteStats;
        public LinkStatistics RadioMasterStatistics => FindLink(LinkType.RadioMaster) ?? _radioStats;
        public bool IsMonitoring => _standalone?.IsMonitoring == true;
        public bool IsRouterAvailable => _standalone?.IsRouterAvailable == true;
        public bool IsStatusStale => _standalone?.IsStatusStale == true;
        public bool SupportsLiveConfiguration => false;
        public string LocalMergedEndpoint => $"udp://{_config.RouterBindAddress}:{_config.RouterLocalPort}";

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

            _lteStats = new LinkStatistics
            {
                Type = LinkType.LTE,
                Name = "LTE / Tailscale",
                Endpoint = $"udp://0.0.0.0:{_config.LtePort}",
                TransportType = "UDP",
                IsEnabled = true,
                Health = LinkHealth.Disconnected
            };
            _radioStats = new LinkStatistics
            {
                Type = LinkType.RadioMaster,
                Name = "RadioMaster",
                Endpoint = _config.RadioMasterConnectionType.Equals("COM", StringComparison.OrdinalIgnoreCase)
                    ? $"{_config.RadioMasterComPort} @ {_config.RadioMasterBaudRate}"
                    : (_config.RadioMasterConnectionType.Equals("TCP", StringComparison.OrdinalIgnoreCase)
                        ? $"tcp://0.0.0.0:{_config.RadioMasterPort}"
                        : $"udp://0.0.0.0:{_config.RadioMasterPort}"),
                TransportType = _config.RadioMasterConnectionType,
                IsEnabled = true,
                Health = LinkHealth.Disconnected
            };
        }

        public void UpdateConfig(ConnectionConfig config)
        {
            lock (_lock)
            {
                _config = config ?? throw new ArgumentNullException(nameof(config));
                _lteStats.Endpoint = $"udp://0.0.0.0:{_config.LtePort}";
                _lteStats.TransportType = "UDP";
                _radioStats.Endpoint = _config.RadioMasterConnectionType.Equals("COM", StringComparison.OrdinalIgnoreCase)
                    ? $"{_config.RadioMasterComPort} @ {_config.RadioMasterBaudRate}"
                    : (_config.RadioMasterConnectionType.Equals("TCP", StringComparison.OrdinalIgnoreCase)
                        ? $"tcp://0.0.0.0:{_config.RadioMasterPort}"
                        : $"udp://0.0.0.0:{_config.RadioMasterPort}");
                _radioStats.TransportType = _config.RadioMasterConnectionType;
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

        /// <summary>Record the legacy setting; host JSON remains authoritative.</summary>
        public void SetAutoFailoverEnabled(bool enabled)
        {
            _config.AutoFailoverEnabled = enabled;
            if (_standalone != null) _standalone.LogConfigurationIsRestartRequired("automatic failover");
        }

        /// <summary>Record the legacy setting; host JSON remains authoritative.</summary>
        public void SetAutoReconnectPreferred(bool enabled)
        {
            _config.AutoReconnectPreferred = enabled;
            if (_standalone != null) _standalone.LogConfigurationIsRestartRequired("preferred-link recovery");
        }

        /// <summary>Record the legacy setting; host JSON remains authoritative.</summary>
        public void SetDedupEnabled(bool enabled)
        {
            _config.RouterDedupEnabled = enabled;
            if (_standalone != null) _standalone.LogConfigurationIsRestartRequired("deduplication");
        }

        /// <summary>Record the legacy setting; host JSON remains authoritative.</summary>
        public void SetPreferredLink(string link)
        {
            _config.PreferredLink = link;
            if (_standalone != null) _standalone.LogConfigurationIsRestartRequired("preferred link");
        }

        public string GetStatusSummary() => _standalone?.GetStatusSummary()
            ?? "Standalone router management unavailable";

        public void ResetCounters()
        {
            if (_standalone != null) _standalone.LogConfigurationIsRestartRequired("counter reset");
        }

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
