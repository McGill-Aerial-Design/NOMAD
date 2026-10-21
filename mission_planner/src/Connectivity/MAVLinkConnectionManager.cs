// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// MAVLink Connection Manager — plugin-owned router
// ============================================================
// Owns the ground router and projects its
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

    /// <summary>
    /// Status and safe link-selection surface for the plugin-owned router.
    /// </summary>
    public interface IRouterStatusProvider : IDisposable
    {
        string RouterMode { get; }
        bool IsMonitoring { get; }
        bool IsRouterAvailable { get; }
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

    public sealed class MAVLinkConnectionManager : IRouterStatusProvider
    {
        public class ConnectionConfig
        {
            public int RouterLocalPort { get; set; } = 14600;
            public List<LinkConfig> Links { get; set; } = new List<LinkConfig>();
            public string PreferredLink { get; set; } = "";
        }

        private ConnectionConfig _config;
        private volatile GroundLinkRouter _router;
        private bool _disposed;
        public event EventHandler<LinkStatusChangedEventArgs> LinkStatusChanged;
        public event EventHandler<FailoverEventArgs> FailoverOccurred;
        public event EventHandler<string> ActiveLinkChanged;
        public event EventHandler<string> LogMessage;
        public string RouterMode => "Plugin";
        public bool IsMonitoring => _router?.IsRunning == true;
        public bool IsRouterAvailable => IsMonitoring;
        public string ActiveLink => _router?.ActiveLink ?? LinkType.None;
        public string ManualOverride => _router?.ManualOverride ?? LinkType.None;
        public string LocalMergedEndpoint => $"UDPCl 127.0.0.1:{_config.RouterLocalPort}";
        public IReadOnlyList<LinkStatistics> LinkStatistics => _router == null
            ? Array.Empty<LinkStatistics>() : _router.Links.Select(ToStatistics).ToArray();
        public IReadOnlyCollection<FailoverEventArgs> FailoverLog => _router?.FailoverLog ??
            (IReadOnlyCollection<FailoverEventArgs>)Array.Empty<FailoverEventArgs>();

        public MAVLinkConnectionManager(ConnectionConfig config = null)
        {
            _config = config ?? new ConnectionConfig();
        }

        public void UpdateConfig(ConnectionConfig config)
        {
            StopMonitoring();
            _config = config ?? throw new ArgumentNullException(nameof(config));
        }

        internal static GroundLinkRouter.RouterConfig BuildRouterConfig(ConnectionConfig config)
        {
            return new GroundLinkRouter.RouterConfig
            {
                Links = config.Links?.Select(link => link?.Snapshot()).ToList(),
                PreferredLink = config.PreferredLink,
                Consumers = new List<ConsumerConfig>
                {
                    new ConsumerConfig { Id = "mission_planner", RouterPort = config.RouterLocalPort,
                        ClientPort = 0, AllowOutbound = true }
                }
            };
        }

        public void StartMonitoring()
        {
            if (_disposed) { throw new ObjectDisposedException(nameof(MAVLinkConnectionManager)); }
            if (IsMonitoring || _config.Links == null || _config.Links.Count == 0) { return; }
            var router = new GroundLinkRouter(BuildRouterConfig(_config));
            router.StatsUpdated += OnStatsUpdated;
            router.FailoverOccurred += OnFailover;
            router.ActiveLinkChanged += OnActiveLinkChanged;
            router.LogMessage += OnLogMessage;
            _router = router;
            try { router.Start(); }
            catch { StopMonitoring(); throw; }
        }

        public void StopMonitoring()
        {
            var router = _router;
            _router = null;
            if (router == null) { return; }
            router.StatsUpdated -= OnStatsUpdated;
            router.FailoverOccurred -= OnFailover;
            router.ActiveLinkChanged -= OnActiveLinkChanged;
            router.LogMessage -= OnLogMessage;
            router.Dispose();
        }

        public bool SwitchToLink(string target) => _router?.SetManualOverride(target) == true;
        public string GetStatusSummary() => _router?.GetStatusSummary() ?? "Router stopped; configure physical links in this profile";
        private void OnFailover(object sender, FailoverEventArgs args)
        {
            if (sender == _router) { FailoverOccurred?.Invoke(this, args); }
        }
        private void OnActiveLinkChanged(object sender, string link)
        {
            if (sender == _router) { ActiveLinkChanged?.Invoke(this, link); }
        }
        private void OnLogMessage(object sender, string message)
        {
            if (sender == _router) { LogMessage?.Invoke(this, message); }
        }
        private void OnStatsUpdated(object sender, EventArgs args)
        {
            if (sender != _router) { return; }
            foreach (var stats in LinkStatistics)
            {
                if (sender != _router) { return; }
                LinkStatusChanged?.Invoke(this, new LinkStatusChangedEventArgs
                { Link = stats.Type, Statistics = stats, IsActive = stats.Type == ActiveLink });
            }
        }

        private static LinkStatistics ToStatistics(LinkSourceStats source) => new LinkStatistics
        {
            Type = source.Type, Name = source.Name, Endpoint = source.Endpoint, TransportType = source.Transport,
            IsEnabled = source.Enabled, IsConnected = source.IsConnected, Health = source.Health,
            LatencyMs = source.LatencyMs, PacketLossPercent = source.PacketLossPercent,
            PacketsReceived = source.FramesForwarded, PacketsDuplicate = source.FramesDuplicate,
            BytesReceived = source.BytesReceived, BytesSent = source.BytesSentOutbound,
            PacketsSent = source.BytesSentOutbound, LastHeartbeat = source.LastHeartbeatTime,
            LastPacketTime = source.LastPacketTime, HeartbeatCount = source.HeartbeatCount,
            DataRateBps = source.DataRateBps, Rssi = source.Rssi, RemRssi = source.RemRssi
        };

        public void Dispose()
        {
            if (_disposed) { return; }
            _disposed = true;
            StopMonitoring();
        }
    }
}
