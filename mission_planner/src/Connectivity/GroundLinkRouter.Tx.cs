// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Transmit-side: outbound forwarding, link send, watchdog, failover
// ============================================================

using System;
using System.IO.Ports;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    public partial class GroundLinkRouter
    {
        // ============================================================
        // Outbound forwarding (GCS → aircraft)
        // ============================================================

        private void ForwardOutbound(byte[] data, int length)
        {
            // Only forward fully-parsed frames. Partial chunks stay buffered in
            // _localParser until the rest arrives — forwarding the raw partial
            // here would re-send those bytes again inside the completed frame.
            _localParser.Push(data, length, (frame) =>
            {
                if (frame.IsParamRequest)
                {
                    var source = SelectParamTransactionSource();
                    ForwardToLink(source, frame.Raw, frame.RawLength, allowFallback: ManualOverride == LinkType.None);
                }
                else
                    ForwardOutboundFrame(frame.Raw, frame.RawLength);
            });
        }

        private void ForwardOutboundFrame(byte[] data, int length)
        {
            var target = ManualOverride != LinkType.None ? ManualOverride : ActiveLink;
            ForwardToLink(target, data, length, allowFallback: ManualOverride == LinkType.None);
        }

        private void ForwardToLink(LinkType target, byte[] data, int length, bool allowFallback = true)
        {
            if (target == LinkType.LTE && Lte.IsOpen)
                SendLte(data, length);
            else if (target == LinkType.RadioMaster && Radio.IsOpen)
                SendRadio(data, length);
            else if (allowFallback && Lte.IsOpen)
                SendLte(data, length);
            else if (allowFallback && Radio.IsOpen)
                SendRadio(data, length);
        }

        private void SendLte(byte[] data, int length)
        {
            try
            {
                IPEndPoint ep = null;
                if (!string.IsNullOrEmpty(_cfg.LteRemoteHost) && _cfg.LteRemotePort > 0)
                {
                    if (IPAddress.TryParse(_cfg.LteRemoteHost, out var ip))
                        ep = new IPEndPoint(ip, _cfg.LteRemotePort);
                    else
                    {
                        var hostIp = Dns.GetHostAddresses(_cfg.LteRemoteHost);
                        if (hostIp.Length > 0) ep = new IPEndPoint(hostIp[0], _cfg.LteRemotePort);
                    }
                }
                if (ep == null) ep = Lte.LastRemote;
                if (ep == null) return; // no destination yet
                _lteSock?.Send(data, length, ep);
                Lte.BytesSentOutbound += length;
            }
            catch { }
        }

        private void SendRadio(byte[] data, int length)
        {
            try
            {
                if (_cfg.RadioIsSerial)
                {
                    if (_radioSerial?.IsOpen == true)
                    {
                        _radioSerial.Write(data, 0, length);
                        Radio.BytesSentOutbound += length;
                    }
                }
                else if (_cfg.RadioIsTcp)
                {
                    if (_radioTcpClient?.Connected == true)
                    {
                        _radioTcpClient.GetStream().Write(data, 0, length);
                        Radio.BytesSentOutbound += length;
                    }
                }
                else
                {
                    var ep = Radio.LastRemote;
                    if (ep == null || _radioSock == null) return;
                    _radioSock.Send(data, length, ep);
                    Radio.BytesSentOutbound += length;
                }
            }
            catch { }
        }

        // ============================================================
        // Stats / health / failover
        // ============================================================

        private async Task StatsLoop(CancellationToken ct)
        {
            while (!ct.IsCancellationRequested)
            {
                try
                {
                    Tick();
                    StatsUpdated?.Invoke(this, EventArgs.Empty);
                    await Task.Delay(_cfg.StatsTickMs, ct).ConfigureAwait(false);
                }
                catch (OperationCanceledException) { break; }
                catch (Exception ex) { EmitLog($"Stats loop error: {ex.Message}"); }
            }
        }

        private void Tick()
        {
            var now = DateTime.UtcNow;

            // Throughput EMA
            if (_lastStatsTick != DateTime.MinValue)
            {
                double secs = Math.Max(0.001, (now - _lastStatsTick).TotalSeconds);
                double lteRate = (Lte.BytesReceived - _lteBytesAtLastTick) / secs;
                double radioRate = (Radio.BytesReceived - _radioBytesAtLastTick) / secs;
                Lte.DataRateBps = Lte.DataRateBps * 0.6 + lteRate * 0.4;
                Radio.DataRateBps = Radio.DataRateBps * 0.6 + radioRate * 0.4;
            }
            _lastStatsTick = now;
            _lteBytesAtLastTick = Lte.BytesReceived;
            _radioBytesAtLastTick = Radio.BytesReceived;

            // Heartbeat liveness
            EvaluateLiveness(Lte, now);
            EvaluateLiveness(Radio, now);

            // Health classification
            ClassifyHealth(Lte, now);
            ClassifyHealth(Radio, now);

            // Failover
            if (_cfg.AutoFailoverEnabled && ManualOverride == LinkType.None)
                EvaluateFailover(now);

            // Watchdog: re-open any source link whose rx loop died or whose
            // initial bind failed. Without this, a single transient socket
            // failure permanently kills one leg of the dual link.
            WatchdogReopen(now);
        }

        private void WatchdogReopen(DateTime now)
        {
            if (_cts == null || _cts.IsCancellationRequested) return;

            if (!Lte.IsOpen && (now - _lastLteReopenAttempt) >= REOPEN_BACKOFF)
            {
                EmitLog("Watchdog: LTE link is down, re-opening socket");
                try { _lteSock?.Close(); } catch { } _lteSock = null;
                OpenLte();
            }

            if (!Radio.IsOpen && (now - _lastRadioReopenAttempt) >= REOPEN_BACKOFF)
            {
                EmitLog("Watchdog: RadioMaster link is down, re-opening");
                try { _radioSock?.Close(); } catch { } _radioSock = null;
                try { if (_radioSerial?.IsOpen == true) _radioSerial.Close(); } catch { } _radioSerial = null;
                OpenRadio();
            }
        }

        private void EvaluateLiveness(LinkSourceStats s, DateTime now)
        {
            if (!s.IsOpen) { s.IsConnected = false; return; }
            if (s.LastPacketTime == DateTime.MinValue) { s.IsConnected = false; return; }
            var elapsed = (now - s.LastPacketTime).TotalSeconds;
            // Treat link as connected if any packets within timeout window.
            s.IsConnected = elapsed < _cfg.HeartbeatTimeoutSec;
        }

        private static void ClassifyHealth(LinkSourceStats s, DateTime now)
        {
            if (!s.IsOpen || !s.IsConnected) { s.Health = LinkHealth.Disconnected; return; }

            double packetAge = s.LastPacketTime == DateTime.MinValue
                ? double.PositiveInfinity
                : (now - s.LastPacketTime).TotalSeconds;
            double hbAge = s.LastHeartbeatTime == DateTime.MinValue
                ? double.PositiveInfinity
                : (now - s.LastHeartbeatTime).TotalSeconds;
            double loss = s.PacketLossPercent;
            double jitter = s.LatencyMs;
            double rate = s.DataRateBps;

            // Grade the MAVLink telemetry stream, not physical RF distance.
            // Heartbeat freshness and received byte rate are the failure signals.
            // Sequence-gap loss is useful for quality scoring, but routers and
            // redundant paths can make it misleading, so it should not alone
            // declare a live link critical.
            if (packetAge > 5.0 || hbAge > 8.0 ||
                (s.FramesReceived > 50 && rate < 5.0))
            {
                s.Health = LinkHealth.Critical;
            }
            else if (hbAge > 4.0 || loss >= 60.0 ||
                     (s.FramesReceived > 100 && rate < 20.0))
            {
                s.Health = LinkHealth.Poor;
            }
            else if (hbAge > 2.5 || loss >= 25.0 || jitter >= 1500.0 ||
                     (s.FramesReceived > 100 && rate < 80.0))
            {
                s.Health = LinkHealth.Fair;
            }
            else if (hbAge > 1.5 || loss >= 5.0 || jitter >= 500.0 ||
                     (s.FramesReceived > 100 && rate < 300.0))
            {
                s.Health = LinkHealth.Good;
            }
            else
            {
                s.Health = LinkHealth.Excellent;
            }
        }

        private void EvaluateFailover(DateTime now)
        {
            if ((now - _lastFailover).TotalSeconds < _cfg.FailoverCooldownSec) return;

            var current = ActiveLink == LinkType.LTE ? Lte : ActiveLink == LinkType.RadioMaster ? Radio : null;
            var alt = ActiveLink == LinkType.LTE ? Radio : Lte;
            var altType = ActiveLink == LinkType.LTE ? LinkType.RadioMaster : LinkType.LTE;

            bool currentFailed = current == null || !current.IsConnected ||
                                 current.Health == LinkHealth.Disconnected ||
                                 current.Health == LinkHealth.Critical;

            bool altUsable = alt.IsConnected &&
                             alt.Health != LinkHealth.Disconnected &&
                             alt.Health != LinkHealth.Critical;

            if (currentFailed && altUsable)
            {
                SetActiveLink(altType, current == null ? "no active link, falling back" : $"{ActiveLink} unhealthy ({current.Health})");
                return;
            }

            // Return to preferred once it's been healthy for the configured delay
            if (_cfg.AutoReconnectPreferred && _cfg.PreferredLink != LinkType.None && ActiveLink != _cfg.PreferredLink)
            {
                var pref = _cfg.PreferredLink == LinkType.LTE ? Lte : Radio;
                bool prefGood = pref.IsConnected &&
                                pref.Health != LinkHealth.Disconnected &&
                                pref.Health != LinkHealth.Critical &&
                                pref.Health != LinkHealth.Poor;
                if (prefGood)
                {
                    if (_preferredHealthySince == DateTime.MinValue) _preferredHealthySince = now;
                    else if ((now - _preferredHealthySince).TotalSeconds >= _cfg.PreferredLinkReconnectDelaySec)
                    {
                        SetActiveLink(_cfg.PreferredLink, "preferred link recovered");
                        _preferredHealthySince = DateTime.MinValue;
                    }
                }
                else _preferredHealthySince = DateTime.MinValue;
            }
            else _preferredHealthySince = DateTime.MinValue;
        }

        private void SetActiveLink(LinkType newActive, string reason)
        {
            if (newActive == ActiveLink) return;
            var from = ActiveLink;
            ActiveLink = newActive;
            _lastFailover = DateTime.UtcNow;

            var ev = new FailoverEventArgs
            {
                FromLink = from,
                ToLink = newActive,
                Reason = reason,
                Timestamp = DateTime.UtcNow,
            };
            lock (_failoverLog)
            {
                _failoverLog.AddLast(ev);
                while (_failoverLog.Count > FAILOVER_LOG_MAX) _failoverLog.RemoveFirst();
            }

            FailoverOccurred?.Invoke(this, ev);
            ActiveLinkChanged?.Invoke(this, newActive);
            EmitLog($"Failover {from} → {newActive}: {reason}");
        }
    }
}
