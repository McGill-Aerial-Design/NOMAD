// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Net;
namespace NOMAD.MissionPlanner
{
    // Compatibility names only; router identity is an arbitrary, case-sensitive string.
    public static class LinkType
    {
        public const string LTE = "LTE";
        public const string RadioMaster = "RadioMaster";
        public const string None = "";
    }
    public enum LinkHealth { Excellent, Good, Fair, Poor, Critical, Disconnected }
    public class FailoverEventArgs : EventArgs
    {
        public string FromLink { get; set; }
        public string ToLink { get; set; }
        public string Reason { get; set; }
        public DateTime Timestamp { get; set; }
    }
    public class LinkSourceStats
    {
        public LinkSourceStats Snapshot()
        {
            var copy = (LinkSourceStats)MemberwiseClone();
            copy.LastRemote = LastRemote == null ? null : new IPEndPoint(LastRemote.Address, LastRemote.Port);
            return copy;
        }
        public string Type;
        public string Name;
        public string Endpoint = "";
        public string Transport = "";
        public bool Enabled;
        public bool IsOpen;         // socket bound / serial open
        public bool IsConnected;    // received traffic in the last few seconds
        public LinkHealth Health = LinkHealth.Disconnected;

        public double LatencyMs;          // smoothed heartbeat-interval deviation, not RF round-trip latency
        public double PacketLossPercent;  // from sequence-number gaps
        public double DataRateBps;        // EMA of received bytes/sec

        public long FramesReceived;
        public long FramesForwarded;      // after dedup
        public long FramesDuplicate;
        public long BytesReceived;
        public long BytesSentOutbound;
        public long FrameErrors;
        public int HeartbeatCount;

        public DateTime LastPacketTime;
        public DateTime LastHeartbeatTime;

        public int? Rssi;     // last RADIO_STATUS rssi (0-254, higher = better)
        public int? RemRssi;  // remote RSSI from RADIO_STATUS

        public IPEndPoint LastRemote; // last UDP sender (for outbound replies)
    }

    public class LinkConfig
    {
        internal LinkConfig Snapshot() => (LinkConfig)MemberwiseClone();
        public string Id { get; set; }
        public string Name { get; set; }
        public bool Enabled { get; set; } = true;
        public string Transport { get; set; } = "UDP";
        public string BindAddress { get; set; } = "0.0.0.0";
        public int Port { get; set; }
        public string RemoteHost { get; set; } = "";
        public int RemotePort { get; set; }
        public string Device { get; set; } = "COM3";
        public int BaudRate { get; set; } = 420000;
        public int Priority { get; set; }
        public double ReconnectSeconds { get; set; } = 3;
    }
    public class ConsumerConfig
    {
        internal ConsumerConfig Snapshot() => (ConsumerConfig)MemberwiseClone();
        public string Id { get; set; }
        public int RouterPort { get; set; }
        // Zero learns one loopback client's source port; nonzero is a fixed client-owned listener.
        public int ClientPort { get; set; }
        // Existing configurations remain bidirectional unless they opt into observation-only mode.
        public bool AllowOutbound { get; set; } = true;
    }

    public static class RouterConsumerPolicy
    {
        public static List<ConsumerConfig> ForIntegratedFlight(IEnumerable<ConsumerConfig> configured)
        {
            var consumers = new List<ConsumerConfig>();
            if (configured == null)
            {
                return null;
            }

            foreach (var consumer in configured)
            {
                if (consumer == null)
                {
                    consumers.Add(null);
                    continue;
                }

                var isMissionPlanner = string.Equals(
                    consumer.Id, "mission_planner", StringComparison.OrdinalIgnoreCase);
                consumers.Add(new ConsumerConfig
                {
                    Id = consumer.Id,
                    RouterPort = consumer.RouterPort,
                    ClientPort = consumer.ClientPort,
                    AllowOutbound = isMissionPlanner ? false : consumer.AllowOutbound
                });
            }
            return consumers;
        }
    }

    public partial class GroundLinkRouter
    {
        public class RouterConfig
        {
            public List<LinkConfig> Links;
            public List<ConsumerConfig> Consumers;
            public string BindAddress = "127.0.0.1";
            public int LocalPort = 14600;
            public bool DedupEnabled = true;

            // LTE
            public int LteBindPort = 14560;          // LTE-side uplink (14560 avoids the RC default 14550)
            public string LteRemoteHost = "";        // outbound LTE host (empty = reply to LastRemote)
            public int LteRemotePort = 0;

            // RadioMaster
            public string RadioMasterConnectionType = "UDP"; // "COM", "UDP", or "TCP"
            // UDP listen port / TCP connect port; differs from LteBindPort.
            public int RadioBindPort = 14550;
            public string RadioComPort = "COM3";     // serial path (used when RadioMasterConnectionType == "COM")
            public int RadioBaudRate = 420000;       // baud rate for serial
            public string RadioTcpHost = "127.0.0.1"; // TCP server host to connect to (e.g. SITL) when type == "TCP"

            // Connection-type convenience flags.
            public bool RadioIsSerial => "COM".Equals(RadioMasterConnectionType, StringComparison.OrdinalIgnoreCase);
            public bool RadioIsTcp => "TCP".Equals(RadioMasterConnectionType, StringComparison.OrdinalIgnoreCase);

            public bool AutoFailoverEnabled = true;
            public string PreferredLink = LinkType.LTE;
            public bool AutoReconnectPreferred = true;
            public int PreferredLinkReconnectDelaySec = 10;

            public int StatsTickMs = 250;
            public double HeartbeatTimeoutSec = 3.0;
            public double FailoverCooldownSec = 2.0;

            // The management endpoint is deliberately separate from raw MAVLink
            // consumer sockets and is always loopback-only.
            public string ManagementBindAddress = "127.0.0.1";
            public int ManagementPort = 14610;
        }

    }
}
