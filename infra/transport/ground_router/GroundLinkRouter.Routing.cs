// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    public partial class GroundLinkRouter
    {
        // Only delivered frames enter this bounded window: a standby copy cannot suppress the active copy.
        private readonly Dictionary<string, Tuple<string, double>> _forwarded =
            new Dictionary<string, Tuple<string, double>>();
        private double _nextSweep;
        private string _paramLink;
        private double _paramActivity = RouterClock.Unset;
        private const double ParamTimeout = 4;

        private void ProcessIncoming(PhysicalLink link, byte[] bytes, int count)
        {
            link.Stats.BytesReceived += count;
            link.Parser.Push(bytes, count, frame => ProcessFrame(link, frame));
            link.Stats.FrameErrors = link.Parser.ResyncCount;
        }

        private void ProcessFrame(PhysicalLink link, MavlinkFrame frame)
        {
            var now = _clock.Seconds();
            var s = link.Stats;
            if (!Usable(link, now))
            {
                link.HealthySince = RouterClock.Unset;
            }
            s.LastPacketTime = _clock.UtcNow();
            link.LastPacket = now;
            s.IsConnected = true;
            s.FramesReceived++;
            UpdateSequence(link, frame);
            UpdateFrameMetrics(link, frame, now);
            ForwardInbound(link, frame, now);
        }

        private void UpdateFrameMetrics(PhysicalLink link, MavlinkFrame frame, double now)
        {
            var s = link.Stats;
            if (frame.IsHeartbeat)
            {
                if (link.LastHeartbeat != RouterClock.Unset)
                {
                    double jitter = Math.Abs((now - link.LastHeartbeat) * 1000 - 1000);
                    s.LatencyMs = s.LatencyMs * 0.7 + jitter * 0.3;
                }
                s.LastHeartbeatTime = s.LastPacketTime;
                link.LastHeartbeat = now;
                s.HeartbeatCount++;
            }
            if (frame.IsRadioStatus && frame.PayloadLength >= 6)
            {
                // MAVLink uses size-sorted fields for both v1 and v2 RADIO_STATUS.
                s.Rssi = frame.Raw[frame.PayloadOffset + 4];
                s.RemRssi = frame.Raw[frame.PayloadOffset + 5];
            }
        }

        private void ForwardInbound(PhysicalLink link, MavlinkFrame frame, double now)
        {
            var s = link.Stats;
            SelectLink(now);
            string source = link.Config.Id;
            string selected = ActiveLink;
            if (now - _paramActivity > ParamTimeout)
            {
                _paramLink = null;
            }
            if (IsParameter(frame) && _paramLink != null && now - _paramActivity <= ParamTimeout)
            {
                selected = _paramLink;
            }
            if (source != selected)
            {
                CountDuplicate(link, frame, now); return;
            }
            if (CountDuplicate(link, frame, now))
            {
                return;
            }
            if (IsParameter(frame) && _paramLink != null)
            {
                _paramActivity = now;
            }
            Sweep(now);
            if (_forwarded.Count >= 16384)
            {
                return;
            }
            _forwarded[Convert.ToBase64String(frame.Raw)] = Tuple.Create(source, now);
            s.FramesForwarded++;
            foreach (var consumer in _consumers)
            {
                consumer.Send(frame.Raw);
            }
            Sweep(now);
        }

        private bool CountDuplicate(PhysicalLink link, MavlinkFrame frame, double now)
        {
            if (!_cfg.DedupEnabled)
            {
                return false;
            }
            if (_forwarded.TryGetValue(Convert.ToBase64String(frame.Raw), out var seen) &&
                seen.Item1 != link.Config.Id && (now - seen.Item2) * 1000 < 750)
            {
                link.Stats.FramesDuplicate++; return true;
            }
            return false;
        }

        private void Sweep(double now)
        {
            if (now < _nextSweep && _forwarded.Count < 16384)
            {
                return;
            }
            foreach (var key in _forwarded.Where(p => (now - p.Value.Item2) * 1000 >= 750)
                .Select(p => p.Key).ToList()) { _forwarded.Remove(key); }
            // Fail closed on overflow: drop new unseen telemetry until entries age out.
            _nextSweep = now + .250;
        }

        private static void UpdateSequence(PhysicalLink link, MavlinkFrame frame)
        {
            int key = (frame.Sysid << 8) | frame.Compid;
            if (link.Sequences.TryGetValue(key, out var previous))
            {
                int delta = (frame.Seq - previous + 256) & 255;
                if (delta == 0 || delta >= 128)
                {
                    return;
                }
                link.Seen++;
                link.Lost += delta - 1;
            }
            link.Sequences[key] = frame.Seq;
            long total = link.Seen + link.Lost;
            link.Stats.PacketLossPercent = total == 0 ? 0 : link.Lost * 100.0 / total;
        }

        private static bool IsParameter(MavlinkFrame frame) =>
            (frame.Msgid >= 20 && frame.Msgid <= 23) || (frame.Msgid >= 320 && frame.Msgid <= 324);

        private void ForwardOutbound(ConsumerConfig consumer, MavlinkFrame frame)
        {
            if (!consumer.AllowOutbound)
            {
                return;
            }
            var now = _clock.Seconds();
            if (_forwarded.TryGetValue(Convert.ToBase64String(frame.Raw), out var echo) &&
                (now - echo.Item2) * 1000 < 750) { return; }
            SelectLink(now);
            string selected = SelectOutboundLink();
            if (IsParameter(frame))
            {
                if (_paramLink == null || now - _paramActivity > ParamTimeout)
                {
                    _paramLink = selected; _paramActivity = now;
                }
                selected = _paramLink;
            }
            var link = _links.FirstOrDefault(l => l.Config.Id == selected);
            if (link == null || (!Usable(link, now) && !link.CanAnnounce))
            {
                return;
            }
            try
            {
                link.Send(frame.Raw);
                if (IsParameter(frame))
                {
                    _paramActivity = now;
                }
            }
            catch (Exception ex)
            {
                link.Dispose();
                EmitLog(selected + " send failed; frame not retried: " + ex.Message);
            }
        }

        private string SelectOutboundLink()
        {
            if (ManualOverride != LinkType.None)
            {
                return ManualOverride;
            }
            if (ActiveLink != LinkType.None)
            {
                return ActiveLink;
            }
            var initial = _links.Where(l => l.Config.Enabled && l.CanAnnounce)
                .OrderByDescending(l => l.Config.Id == _cfg.PreferredLink)
                .ThenByDescending(l => l.Config.Priority)
                .ThenBy(l => l.Config.Id, StringComparer.Ordinal).FirstOrDefault();
            if (initial != null)
            {
                SetActiveLink(initial.Config.Id, "initial GCS announcement");
            }
            return ActiveLink;
        }

        private bool Usable(PhysicalLink link, double now) => link != null && link.Config.Enabled &&
            link.Stats.IsOpen && (now - link.LastPacket) < _cfg.HeartbeatTimeoutSec;

        private void Tick(double now)
        {
            foreach (var link in _links)
            {
                var s = link.Stats;
                double rate = (s.BytesReceived - link.PreviousBytes) / Math.Max(.001, (now - _lastTick));
                s.DataRateBps = s.DataRateBps * .6 + rate * .4;
                link.PreviousBytes = s.BytesReceived;
                s.IsConnected = Usable(link, now);
                ClassifyHealth(s, now - link.LastPacket, now - link.LastHeartbeat);
                bool healthy = s.IsConnected && s.Health <= LinkHealth.Fair;
                if (!healthy)
                {
                    link.HealthySince = RouterClock.Unset;
                }
                else if (link.HealthySince == RouterClock.Unset)
                {
                    link.HealthySince = now;
                }
            }
            _lastTick = now;
            SelectLink(now);
            EnqueueNotification(() => StatsUpdated?.Invoke(this, EventArgs.Empty));
        }

        private void SelectLink(double now)
        {
            if (ManualOverride != LinkType.None || !_cfg.AutoFailoverEnabled)
            {
                return;
            }
            var ranked = _links.Where(l => Usable(l, now)).OrderBy(l => l.Stats.Health == LinkHealth.Critical)
                .ThenByDescending(l => l.Config.Id == _cfg.PreferredLink)
                .ThenByDescending(l => l.Config.Priority).ThenBy(l => l.Config.Id, StringComparer.Ordinal).ToList();
            var best = ranked.FirstOrDefault();
            var current = _links.FirstOrDefault(l => l.Config.Id == ActiveLink);
            if (best == null && current != null && current.CanAnnounce)
            {
                return;
            }
            if (!Usable(current, now))
            {
                SetActiveLink(best?.Config.Id ?? LinkType.None, "active link unavailable");
                return;
            }
            if (best == null || best == current || !_cfg.AutoReconnectPreferred)
            {
                return;
            }
            if ((now - _lastSwitch) < _cfg.FailoverCooldownSec)
            {
                return;
            }
            if (best.HealthySince != RouterClock.Unset &&
                (now - best.HealthySince) >= _cfg.PreferredLinkReconnectDelaySec)
            {
                SetActiveLink(best.Config.Id, "preferred priority link recovered");
            }
        }
    }
}
