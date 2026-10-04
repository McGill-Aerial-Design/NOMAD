// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Globalization;

namespace NOMAD.MissionPlanner
{
    public partial class GroundLinkRouter
    {
        public RouterStatusSnapshot GetStatusSnapshot()
        {
            lock (_gate)
            {
                var now = _clock.Seconds();
                var snapshot = new RouterStatusSnapshot
                {
                    Running = _running,
                    ConfiguredLinkCount = _links.Count,
                    ConnectedLinkCount = 0,
                    ActiveLinkId = ActiveLink ?? "",
                    ManualOverrideId = ManualOverride ?? "",
                    AutomaticFailoverEnabled = _cfg.AutoFailoverEnabled,
                    PreferredLinkId = _cfg.PreferredLink ?? "",
                    TimestampUtc = _clock.UtcNow().ToString("O", CultureInfo.InvariantCulture),
                };

                foreach (var link in _links)
                {
                    var status = BuildLinkStatus(link, now);
                    snapshot.Links.Add(status);
                    if (status.IsConnected)
                    {
                        snapshot.ConnectedLinkCount++;
                    }
                }

                return snapshot;
            }
        }

        private static RouterLinkStatusSnapshot BuildLinkStatus(PhysicalLink link, double now)
        {
            var stats = link.Stats;
            return new RouterLinkStatusSnapshot
            {
                StableId = link.Config.Id,
                DisplayName = stats.Name ?? link.Config.Id,
                TransportType = link.Config.Transport,
                Endpoint = stats.Endpoint ?? "",
                Enabled = link.Config.Enabled,
                IsOpen = stats.IsOpen,
                IsConnected = stats.IsConnected,
                Health = stats.Health.ToString(),
                LastPacketAgeMs = AgeMilliseconds(link.LastPacket, now),
                LastHeartbeatAgeMs = AgeMilliseconds(link.LastHeartbeat, now),
                PacketLossEstimate = stats.PacketLossPercent,
                DataRateBytesPerSecond = stats.DataRateBps,
                HeartbeatJitterMs = stats.LatencyMs,
                Rssi = stats.Rssi,
                RemoteRssi = stats.RemRssi,
                HeartbeatCount = stats.HeartbeatCount,
                ReceivedFrameCount = stats.FramesReceived,
                ForwardedFrameCount = stats.FramesForwarded,
                DuplicateFrameCount = stats.FramesDuplicate,
            };
        }

        private static double? AgeMilliseconds(double timestamp, double now)
        {
            if (timestamp == RouterClock.Unset)
            {
                return null;
            }

            return Math.Max(0, (now - timestamp) * 1000);
        }
    }
}
