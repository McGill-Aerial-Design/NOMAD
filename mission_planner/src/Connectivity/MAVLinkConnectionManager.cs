// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// MAVLink Connection Manager — façade over GroundLinkRouter
// ============================================================
// Owns a GroundLinkRouter and projects its live state into the
// LinkStatistics / event surface the rest of the plugin already
// consumes (LinkHealthPanel, NOMADDashboardView, NOMADLinksView).
// All real I/O and health tracking happens in the router; this
// class is the public API + glue.
// ============================================================

using System;
using System.Collections.Generic;

namespace NOMAD.MissionPlanner
{
    public enum LinkType
    {
        LTE,
        RadioMaster,
        None
    }

    public enum LinkHealth
    {
        Excellent,
        Good,
        Fair,
        Poor,
        Critical,
        Disconnected
    }

    /// <summary>
    /// Snapshot of one link's health for UI consumers.
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
            ? $"{Health} ({LatencyMs:F0}ms, {PacketLossPercent:F1}% loss)"
            : "Disconnected";
    }

    public class LinkStatusChangedEventArgs : EventArgs
    {
        public LinkType Link { get; set; }
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

    public class FailoverEventArgs : EventArgs
    {
        public LinkType FromLink { get; set; }
        public LinkType ToLink { get; set; }
        public string Reason { get; set; }
        public DateTime Timestamp { get; set; }
    }

    public class MAVLinkConnectionManager : IDisposable
    {
        // ============================================================
        // Configuration
        // ============================================================

        /// <summary>
        /// Connection configuration. Mostly mirrors NOMADConfig fields and
        /// is translated into a GroundLinkRouter.RouterConfig at Start time.
        /// </summary>
        public class ConnectionConfig
        {
            public string JetsonTailscaleIP { get; set; } = "";
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
            public LinkType PreferredLink { get; set; } = LinkType.LTE;
            public bool AutoReconnectPreferred { get; set; } = true;
            public int PreferredLinkReconnectDelaySec { get; set; } = 10;
            public int MonitorIntervalMs { get; set; } = 250;
            public double HeartbeatTimeoutSec { get; set; } = 3.0;

            // Router endpoint (where MP connects to as UDPCl)
            public string RouterBindAddress { get; set; } = "127.0.0.1";
            public int RouterLocalPort { get; set; } = 14600;
            public bool RouterDedupEnabled { get; set; } = true;
        }

        // ============================================================
        // State
        // ============================================================

        private ConnectionConfig _config;
        private GroundLinkRouter _router;
        private readonly object _lock = new object();
        private bool _disposed;

        private readonly LinkStatistics _lteStats;
        private readonly LinkStatistics _radioStats;

        // ============================================================
        // Events
        // ============================================================

        public event EventHandler<LinkStatusChangedEventArgs> LinkStatusChanged;
        public event EventHandler<FailoverEventArgs> FailoverOccurred;
        public event EventHandler<LinkType> ActiveLinkChanged;
        public event EventHandler<string> LogMessage;

        // ============================================================
        // Properties
        // ============================================================

        public LinkType ActiveLink => _router?.ActiveLink ?? LinkType.None;
        public LinkType ManualOverride => _router?.ManualOverride ?? LinkType.None;
        public ConnectionConfig Config => _config;
        public LinkStatistics LteStatistics => _lteStats;
        public LinkStatistics RadioMasterStatistics => _radioStats;
        public bool IsMonitoring => _router?.IsRunning == true;
        public string LocalMergedEndpoint => _router?.LocalEndpoint != null
            ? $"udp://{_router.LocalEndpoint.Address}:{_router.LocalEndpoint.Port}"
            : $"udp://{_config.RouterBindAddress}:{_config.RouterLocalPort}";

        public IReadOnlyCollection<FailoverEventArgs> FailoverLog =>
            _router?.FailoverLog ?? (IReadOnlyCollection<FailoverEventArgs>)Array.Empty<FailoverEventArgs>();

        public bool IsLteHealthy => _lteStats.IsConnected &&
            _lteStats.Health != LinkHealth.Disconnected &&
            _lteStats.Health != LinkHealth.Critical;

        public bool IsRadioMasterHealthy => _radioStats.IsConnected &&
            _radioStats.Health != LinkHealth.Disconnected &&
            _radioStats.Health != LinkHealth.Critical;

        public LinkStatusSnapshot GetLinkStatus()
        {
            lock (_lock)
            {
                return new LinkStatusSnapshot
                {
                    LTEConnected = _lteStats.IsConnected,
                    LTELatencyMs = _lteStats.LatencyMs,
                    LTEPacketLoss = _lteStats.PacketLossPercent,
                    RadioConnected = _radioStats.IsConnected,
                    RadioLatencyMs = _radioStats.LatencyMs,
                    RadioPacketLoss = _radioStats.PacketLossPercent,
                    ActiveLink = ActiveLink.ToString()
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
                Health = LinkHealth.Disconnected
            };
        }

        public void UpdateConfig(ConnectionConfig config)
        {
            lock (_lock)
            {
                _config = config ?? throw new ArgumentNullException(nameof(config));
                _lteStats.Endpoint = $"udp://0.0.0.0:{_config.LtePort}";
                _radioStats.Endpoint = _config.RadioMasterConnectionType.Equals("COM", StringComparison.OrdinalIgnoreCase)
                    ? $"{_config.RadioMasterComPort} @ {_config.RadioMasterBaudRate}"
                    : (_config.RadioMasterConnectionType.Equals("TCP", StringComparison.OrdinalIgnoreCase)
                        ? $"tcp://0.0.0.0:{_config.RadioMasterPort}"
                        : $"udp://0.0.0.0:{_config.RadioMasterPort}");
            }
        }

        // ============================================================
        // Lifecycle
        // ============================================================

        public void StartMonitoring()
        {
            if (_router?.IsRunning == true) return;

            // Clean up any previous (stopped) router instance to release sockets.
            if (_router != null)
            {
                try { _router.Dispose(); } catch { }
                _router = null;
            }

            var rc = new GroundLinkRouter.RouterConfig
            {
                BindAddress = _config.RouterBindAddress,
                LocalPort = _config.RouterLocalPort,
                DedupEnabled = _config.RouterDedupEnabled,
                LteBindPort = _config.LtePort,
                LteRemoteHost = _config.LteRemoteHost,
                LteRemotePort = _config.LteRemotePort,
                RadioMasterConnectionType = _config.RadioMasterConnectionType,
                RadioBindPort = _config.RadioMasterPort,
                RadioComPort = _config.RadioMasterComPort,
                RadioBaudRate = _config.RadioMasterBaudRate,
                RadioTcpHost = _config.RadioMasterTcpHost,
                AutoFailoverEnabled = _config.AutoFailoverEnabled,
                PreferredLink = _config.PreferredLink,
                AutoReconnectPreferred = _config.AutoReconnectPreferred,
                PreferredLinkReconnectDelaySec = _config.PreferredLinkReconnectDelaySec,
                StatsTickMs = Math.Max(100, _config.MonitorIntervalMs),
                HeartbeatTimeoutSec = Math.Max(0.5, _config.HeartbeatTimeoutSec),
            };

            _router = new GroundLinkRouter(rc);
            _router.StatsUpdated += OnRouterStatsUpdated;
            _router.FailoverOccurred += (s, e) => FailoverOccurred?.Invoke(this, e);
            _router.ActiveLinkChanged += (s, t) => ActiveLinkChanged?.Invoke(this, t);
            _router.LogMessage += (s, msg) => LogMessage?.Invoke(this, msg);

            try
            {
                _router.Start();
            }
            catch (Exception ex)
            {
                Log.Error($"router failed to start — {ex.Message}");
            }
        }

        public void StopMonitoring()
        {
            try { _router?.Stop(); } catch { }
        }

        /// <summary>
        /// Tear down the running router and start a fresh one using the current
        /// <see cref="Config"/>. Use after <see cref="UpdateConfig"/> when network
        /// settings (ports, bindings, link type) have changed and the router
        /// needs to rebind its sockets.
        /// </summary>
        public void RestartRouter()
        {
            StopMonitoring();
            StartMonitoring();
        }

        /// <summary>
        /// Manual override of the active outbound link. Pass LinkType.None to
        /// release the override and resume auto-failover.
        /// </summary>
        public bool SwitchToLink(LinkType target)
        {
            if (_router == null) return false;
            _router.SetManualOverride(target);
            return true;
        }

        /// <summary>
        /// Live setter for auto-failover. Updates the running router so the
        /// change takes effect immediately (no restart required).
        /// </summary>
        public void SetAutoFailoverEnabled(bool enabled)
        {
            _config.AutoFailoverEnabled = enabled;
            if (_router != null) _router.Config.AutoFailoverEnabled = enabled;
        }

        /// <summary>Live setter for auto-reconnect-to-preferred.</summary>
        public void SetAutoReconnectPreferred(bool enabled)
        {
            _config.AutoReconnectPreferred = enabled;
            if (_router != null) _router.Config.AutoReconnectPreferred = enabled;
        }

        /// <summary>Live setter for cross-link dedup.</summary>
        public void SetDedupEnabled(bool enabled)
        {
            _config.RouterDedupEnabled = enabled;
            if (_router != null) _router.Config.DedupEnabled = enabled;
        }

        /// <summary>Live setter for the preferred link.</summary>
        public void SetPreferredLink(LinkType link)
        {
            _config.PreferredLink = link;
            if (_router != null) _router.Config.PreferredLink = link;
        }

        public string GetStatusSummary() => _router?.GetStatusSummary()
            ?? $"Active: {ActiveLink} | LTE: offline | Radio: offline";

        public void ResetCounters() => _router?.ResetCounters();

        // ============================================================
        // Bridging router → LinkStatistics
        // ============================================================

        private void OnRouterStatsUpdated(object sender, EventArgs e)
        {
            if (_router == null) return;
            ProjectStats(_router.Lte, _lteStats);
            ProjectStats(_router.Radio, _radioStats);

            LinkStatusChanged?.Invoke(this, new LinkStatusChangedEventArgs
            {
                Link = LinkType.LTE,
                Statistics = _lteStats,
                IsActive = ActiveLink == LinkType.LTE,
            });
            LinkStatusChanged?.Invoke(this, new LinkStatusChangedEventArgs
            {
                Link = LinkType.RadioMaster,
                Statistics = _radioStats,
                IsActive = ActiveLink == LinkType.RadioMaster,
            });
        }

        private static void ProjectStats(LinkSourceStats src, LinkStatistics dst)
        {
            dst.IsConnected = src.IsConnected;
            dst.Health = src.Health;
            dst.LatencyMs = src.LatencyMs;
            dst.PacketLossPercent = src.PacketLossPercent;
            dst.PacketsReceived = src.FramesReceived;
            dst.PacketsDuplicate = src.FramesDuplicate;
            dst.BytesReceived = src.BytesReceived;
            dst.BytesSent = src.BytesSentOutbound;
            dst.LastHeartbeat = src.LastHeartbeatTime;
            dst.LastPacketTime = src.LastPacketTime;
            dst.HeartbeatCount = src.HeartbeatCount;
            dst.DataRateBps = src.DataRateBps;
            dst.Rssi = src.Rssi;
            dst.RemRssi = src.RemRssi;
        }

        // ============================================================
        // IDisposable
        // ============================================================

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            try { _router?.Dispose(); } catch { }
            _router = null;
        }
    }
}
