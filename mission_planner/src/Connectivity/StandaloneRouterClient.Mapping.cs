// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    public sealed partial class StandaloneRouterClient
    {
        public string GetStatusSummary()
        {
            lock (_gate)
            {
                if (!_connected)
                {
                    return "Router management unavailable";
                }
                if (_stale || !IsFreshStatusLocked())
                {
                    return "Router management status stale";
                }

                var connected = _links.Count(link => link.IsConnected && !link.IsStale);
                return $"Router: connected — Physical links: {connected}/{_links.Count}";
            }
        }

        private void ApplyStatus(RouterStatusSnapshot status)
        {
            var previousActive = "";
            var next = status.Links.Select(ToStatistics).ToList();
            var statusRunning = status.Running;
            if (!statusRunning)
            {
                foreach (var link in next)
                {
                    link.IsStale = true;
                }
            }

            lock (_gate)
            {
                previousActive = _activeLink;
                _activeLink = statusRunning ? status.ActiveLinkId ?? "" : "";
                _manualOverride = statusRunning ? status.ManualOverrideId ?? "" : "";
                _links.Clear();
                _links.AddRange(next);
                _lastStatus = _clock.Seconds();
                _stale = !statusRunning;
                _connected = true;
                _lastUnavailableMessage = null;
            }

            foreach (var stats in next)
            {
                LinkStatusChanged?.Invoke(this, new LinkStatusChangedEventArgs
                {
                    Link = stats.Type,
                    Statistics = Clone(stats),
                    IsActive = statusRunning && stats.Type == status.ActiveLinkId,
                });
            }
            var nextActive = statusRunning ? status.ActiveLinkId ?? "" : "";
            if (!string.Equals(previousActive, nextActive, StringComparison.Ordinal))
            {
                ActiveLinkChanged?.Invoke(this, nextActive);
            }
        }

        private LinkStatistics ToStatistics(RouterLinkStatusSnapshot link)
        {
            return new LinkStatistics
            {
                Type = link.StableId,
                Name = link.DisplayName,
                Endpoint = link.Endpoint,
                TransportType = link.TransportType,
                IsEnabled = link.Enabled,
                IsConnected = link.IsConnected,
                IsStale = false,
                Health = ParseHealth(link.Health),
                LatencyMs = link.HeartbeatJitterMs,
                PacketLossPercent = link.PacketLossEstimate,
                PacketsReceived = link.ForwardedFrameCount,
                PacketsDuplicate = link.DuplicateFrameCount,
                LastHeartbeat = TimestampFromAge(link.LastHeartbeatAgeMs),
                LastPacketTime = TimestampFromAge(link.LastPacketAgeMs),
                DataRateBps = link.DataRateBytesPerSecond,
                Rssi = link.Rssi,
                RemRssi = link.RemoteRssi,
                HeartbeatCount = link.HeartbeatCount > int.MaxValue ? int.MaxValue : (int)link.HeartbeatCount,
            };
        }

        private static LinkHealth ParseHealth(string value)
        {
            return Enum.TryParse(value, true, out LinkHealth health) ? health : LinkHealth.Disconnected;
        }

        private DateTime TimestampFromAge(double? age)
        {
            return age.HasValue ? _clock.UtcNow().AddMilliseconds(-Math.Max(0, age.Value)) : DateTime.MinValue;
        }

        private static LinkStatistics Clone(LinkStatistics source)
        {
            return new LinkStatistics
            {
                Type = source.Type,
                Name = source.Name,
                Endpoint = source.Endpoint,
                TransportType = source.TransportType,
                IsEnabled = source.IsEnabled,
                IsConnected = source.IsConnected,
                IsStale = source.IsStale,
                Health = source.Health,
                LatencyMs = source.LatencyMs,
                PacketLossPercent = source.PacketLossPercent,
                PacketsReceived = source.PacketsReceived,
                PacketsDuplicate = source.PacketsDuplicate,
                PacketsSent = source.PacketsSent,
                BytesReceived = source.BytesReceived,
                BytesSent = source.BytesSent,
                LastHeartbeat = source.LastHeartbeat,
                LastPacketTime = source.LastPacketTime,
                HeartbeatCount = source.HeartbeatCount,
                DataRateBps = source.DataRateBps,
                Rssi = source.Rssi,
                RemRssi = source.RemRssi,
            };
        }

        private static FailoverEventArgs CopyFailover(FailoverEventArgs source)
        {
            return new FailoverEventArgs
            {
                FromLink = source.FromLink,
                ToLink = source.ToLink,
                Reason = source.Reason,
                Timestamp = source.Timestamp,
            };
        }

        private DateTime ParseTimestamp(string value)
        {
            return DateTime.TryParse(value, null, DateTimeStyles.RoundtripKind, out var parsed)
                ? parsed.ToUniversalTime()
                : _clock.UtcNow();
        }

        private static bool IsOk(IDictionary<string, object> message)
        {
            var value = RouterManagementProtocol.GetValue(message, "ok");
            return value is bool boolean
                ? boolean
                : string.Equals(RouterManagementProtocol.GetString(message, "ok"), "true",
                    StringComparison.OrdinalIgnoreCase);
        }
    }
}
