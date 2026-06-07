// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Receive-side: source-link setup, RX loops, frame processing
// ============================================================

using System;
using System.Collections.Generic;
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
        // Source link setup
        // ============================================================

        private void OpenLte()
        {
            _lastLteReopenAttempt = DateTime.UtcNow;
            try
            {
                var sock = new UdpClient(new IPEndPoint(IPAddress.Any, _cfg.LteBindPort));
                DisableUdpConnReset(sock);
                _lteSock = sock;
                Lte.IsOpen = true;
                Task.Run(() => UdpRxLoop(_lteSock, LinkType.LTE, _cts.Token));
                EmitLog($"LTE listening on udp://0.0.0.0:{_cfg.LteBindPort}");
            }
            catch (Exception ex)
            {
                Lte.IsOpen = false;
                try { _lteSock?.Close(); } catch { }
                _lteSock = null;
                EmitLog($"LTE bind failed on port {_cfg.LteBindPort}: {ex.Message}");
            }
        }

        private void OpenRadio()
        {
            _lastRadioReopenAttempt = DateTime.UtcNow;
            if (_cfg.RadioIsSerial)
            {
                try
                {
                    _radioSerial = new SerialPort(_cfg.RadioComPort, _cfg.RadioBaudRate, Parity.None, 8, StopBits.One)
                    {
                        ReadBufferSize = 65536,
                        WriteBufferSize = 65536,
                        ReadTimeout = SerialPort.InfiniteTimeout,
                    };
                    _radioSerial.Open();
                    Radio.IsOpen = true;
                    Task.Run(() => SerialRxLoop(_radioSerial, _cts.Token));
                    EmitLog($"RadioMaster opened serial {_cfg.RadioComPort} @ {_cfg.RadioBaudRate}");
                }
                catch (Exception ex)
                {
                    Radio.IsOpen = false;
                    EmitLog($"RadioMaster serial open failed ({_cfg.RadioComPort}): {ex.Message}");
                }
            }
            else if (_cfg.RadioIsTcp)
            {
                try
                {
                    var client = new TcpClient { NoDelay = true };
                    if (!client.ConnectAsync(_cfg.RadioTcpHost, _cfg.RadioBindPort).Wait(2000))
                        throw new TimeoutException("connect timed out");
                    _radioTcpClient = client;
                    Radio.IsOpen = true;
                    Radio.LastRemote = client.Client.RemoteEndPoint as IPEndPoint;
                    Task.Run(() => TcpRxLoop(_radioTcpClient, _cts.Token));
                    EmitLog($"RadioMaster connected tcp://{_cfg.RadioTcpHost}:{_cfg.RadioBindPort}");
                }
                catch (Exception ex)
                {
                    Radio.IsOpen = false;
                    try { _radioTcpClient?.Close(); } catch { }
                    _radioTcpClient = null;
                    EmitLog($"RadioMaster TCP connect failed ({_cfg.RadioTcpHost}:{_cfg.RadioBindPort}): {ex.Message}");
                }
            }
            else
            {
                if (_cfg.RadioBindPort == _cfg.LteBindPort)
                {
                    EmitLog($"RadioMaster UDP port {_cfg.RadioBindPort} collides with LTE port — skipping RC bind");
                    Radio.IsOpen = false;
                    return;
                }
                try
                {
                    var sock = new UdpClient(new IPEndPoint(IPAddress.Any, _cfg.RadioBindPort));
                    DisableUdpConnReset(sock);
                    _radioSock = sock;
                    Radio.IsOpen = true;
                    Task.Run(() => UdpRxLoop(_radioSock, LinkType.RadioMaster, _cts.Token));
                    EmitLog($"RadioMaster listening on udp://0.0.0.0:{_cfg.RadioBindPort}");
                }
                catch (Exception ex)
                {
                    Radio.IsOpen = false;
                    try { _radioSock?.Close(); } catch { }
                    _radioSock = null;
                    EmitLog($"RadioMaster UDP bind failed on port {_cfg.RadioBindPort}: {ex.Message}");
                }
            }
        }

        // ============================================================
        // Receive loops
        // ============================================================

        // TCP client stream (e.g. SITL). MAVLink is a byte stream here; the parser
        // reassembles frames, so partial reads are fine.
        private async Task TcpRxLoop(TcpClient client, CancellationToken ct)
        {
            var buffer = new byte[8192];
            try
            {
                var stream = client.GetStream();
                while (!ct.IsCancellationRequested)
                {
                    int n;
                    try { n = await stream.ReadAsync(buffer, 0, buffer.Length, ct).ConfigureAwait(false); }
                    catch (ObjectDisposedException) { break; }
                    catch (System.IO.IOException) { break; }
                    if (n <= 0) break; // peer closed the connection
                    ProcessIncoming(LinkType.RadioMaster, _radioParser, buffer, n);
                }
            }
            catch (Exception ex)
            {
                EmitLog($"RadioMaster TCP rx error: {ex.Message}");
            }
            finally
            {
                Radio.IsOpen = false;
                Radio.IsConnected = false;
                Radio.Health = LinkHealth.Disconnected;
                if (!ct.IsCancellationRequested) EmitLog("RadioMaster TCP rx loop ended — watchdog will reconnect");
            }
        }

        private async Task UdpRxLoop(UdpClient sock, LinkType type, CancellationToken ct)
        {
            try
            {
                while (!ct.IsCancellationRequested)
                {
                    try
                    {
                        var result = await sock.ReceiveAsync().ConfigureAwait(false);
                        var effectiveType = ClassifyUdpSource(type, result.RemoteEndPoint);
                        var stats = effectiveType == LinkType.LTE ? Lte : Radio;
                        var effectiveParser = effectiveType == LinkType.LTE ? _lteParser : _radioParser;
                        stats.LastRemote = result.RemoteEndPoint;
                        ProcessIncoming(effectiveType, effectiveParser, result.Buffer, result.Buffer.Length);
                    }
                    catch (ObjectDisposedException) { break; }
                    catch (SocketException sx)
                    {
                        // Transient errors we should *not* die on:
                        //   ConnectionReset (10054) — ICMP port-unreachable from a prior Send.
                        //                              SIO_UDP_CONNRESET should suppress these
                        //                              on Windows, but belt-and-suspenders.
                        //   MessageSize    (10040) — oversized datagram; drop and continue.
                        //   Interrupted    (10004) — async cancel; loop will re-check ct.
                        //   NetworkReset   (10052), HostUnreachable (10065), etc.
                        if (sx.SocketErrorCode == SocketError.ConnectionReset ||
                            sx.SocketErrorCode == SocketError.MessageSize ||
                            sx.SocketErrorCode == SocketError.Interrupted ||
                            sx.SocketErrorCode == SocketError.NetworkReset ||
                            sx.SocketErrorCode == SocketError.HostUnreachable ||
                            sx.SocketErrorCode == SocketError.ConnectionRefused)
                        {
                            continue;
                        }
                        EmitLog($"{type} rx socket error ({sx.SocketErrorCode}): {sx.Message} — closing rx loop, watchdog will reopen");
                        break;
                    }
                    catch (Exception ex)
                    {
                        EmitLog($"{type} rx error: {ex.Message}");
                        try { await Task.Delay(100, ct).ConfigureAwait(false); } catch { break; }
                    }
                }
            }
            finally
            {
                // Mark the link down so the watchdog reopens the socket.
                if (type == LinkType.LTE)
                {
                    Lte.IsOpen = false;
                    Lte.IsConnected = false;
                    Lte.Health = LinkHealth.Disconnected;
                    if (!ct.IsCancellationRequested) EmitLog("LTE rx loop ended unexpectedly");
                }
                else
                {
                    Radio.IsOpen = false;
                    Radio.IsConnected = false;
                    Radio.Health = LinkHealth.Disconnected;
                    if (!ct.IsCancellationRequested) EmitLog("RadioMaster rx loop ended unexpectedly");
                }
            }
        }

        private LinkType ClassifyUdpSource(LinkType socketType, IPEndPoint remote)
        {
            if (socketType == LinkType.RadioMaster && IsTailscaleAddress(remote?.Address))
            {
                EmitLog($"LTE telemetry arrived on RadioMaster UDP port {_cfg.RadioBindPort} from {remote}; treating it as LTE. Check Jetson GCS_PORT_LTE should be {_cfg.LteBindPort}.");
                return LinkType.LTE;
            }

            return socketType;
        }

        private static bool IsTailscaleAddress(IPAddress address)
        {
            // Tailscale uses the 100.64.0.0/10 CGNAT range. A bare bytes[0]==100
            // check would also match public 100.0.0.0/8 IPs (e.g. 100.0.0.1
            // through 100.63.255.255), so verify the top two bits of the second
            // octet are 01 (64-127).
            if (address == null) return false;
            var bytes = address.GetAddressBytes();
            if (bytes.Length == 4) return bytes[0] == 100 && (bytes[1] & 0xC0) == 64;
            if (address.IsIPv4MappedToIPv6)
            {
                bytes = address.MapToIPv4().GetAddressBytes();
                return bytes.Length == 4 && bytes[0] == 100 && (bytes[1] & 0xC0) == 64;
            }
            return false;
        }

        private void SerialRxLoop(SerialPort port, CancellationToken ct)
        {
            var buf = new byte[4096];
            try
            {
                while (!ct.IsCancellationRequested && port.IsOpen)
                {
                    try
                    {
                        int n = port.Read(buf, 0, buf.Length);
                        if (n > 0) ProcessIncoming(LinkType.RadioMaster, _radioParser, buf, n);
                    }
                    catch (TimeoutException) { }
                    catch (System.IO.IOException ex)
                    {
                        // USB disconnect or port yanked. Bail so watchdog can reopen.
                        EmitLog($"Radio serial I/O error: {ex.Message} — closing rx loop, watchdog will reopen");
                        break;
                    }
                    catch (InvalidOperationException ex)
                    {
                        EmitLog($"Radio serial closed: {ex.Message} — watchdog will reopen");
                        break;
                    }
                    catch (Exception ex) { EmitLog($"Radio serial rx error: {ex.Message}"); Thread.Sleep(100); }
                }
            }
            finally
            {
                Radio.IsOpen = false;
                Radio.IsConnected = false;
                Radio.Health = LinkHealth.Disconnected;
                if (!ct.IsCancellationRequested) EmitLog("RadioMaster serial rx loop ended unexpectedly");
            }
        }

        private async Task LocalRxLoop(CancellationToken ct)
        {
            // MP is a UDP client — its source endpoint is ephemeral, so latch it
            // on the first inbound packet and use it as the downlink destination.
            while (!ct.IsCancellationRequested)
            {
                try
                {
                    var result = await _localSock.ReceiveAsync().ConfigureAwait(false);
                    _mpEndpoint = result.RemoteEndPoint;
                    ForwardOutbound(result.Buffer, result.Buffer.Length);
                }
                catch (ObjectDisposedException) { break; }
                catch (SocketException) { break; }
                catch (Exception ex) { EmitLog($"Local rx error: {ex.Message}"); }
            }
        }

        // ============================================================
        // Frame processing
        // ============================================================

        private void ProcessIncoming(LinkType type, MavlinkFrameParser parser, byte[] buffer, int length)
        {
            var stats = type == LinkType.LTE ? Lte : Radio;
            stats.BytesReceived += length;
            stats.LastPacketTime = DateTime.UtcNow;
            stats.IsConnected = true;

            parser.Push(buffer, length, (frame) =>
            {
                stats.FramesReceived++;
                UpdateSeqLoss(type, frame.Sysid, frame.Compid, frame.Seq);

                if (frame.IsHeartbeat)
                {
                    var now = DateTime.UtcNow;
                    if (stats.LastHeartbeatTime != DateTime.MinValue)
                    {
                        var dt = (now - stats.LastHeartbeatTime).TotalMilliseconds;
                        // Heartbeats nominally arrive at 1Hz. Latency proxy = |dt - 1000|.
                        // For more realistic ms numbers, smooth via EMA.
                        double dev = Math.Max(0, Math.Abs(dt - 1000.0));
                        stats.LatencyMs = stats.LatencyMs <= 0 ? dev : stats.LatencyMs * 0.7 + dev * 0.3;
                    }
                    stats.LastHeartbeatTime = now;
                    stats.HeartbeatCount++;
                }

                if (frame.IsRadioStatus)
                {
                    // RADIO_STATUS XML field order (v1 wire order):
                    //   rssi u8, remrssi u8, txbuf u8, noise u8, remnoise u8, rxerrors u16, fixed u16
                    // MAVLink v2 sorts by descending field size on the wire, so it becomes:
                    //   rxerrors u16, fixed u16, rssi u8, remrssi u8, txbuf u8, noise u8, remnoise u8
                    if (frame.IsV2 && frame.PayloadLength >= 6)
                    {
                        stats.Rssi = frame.Payload[frame.PayloadOffset + 4];
                        stats.RemRssi = frame.Payload[frame.PayloadOffset + 5];
                    }
                    else if (!frame.IsV2 && frame.PayloadLength >= 2)
                    {
                        stats.Rssi = frame.Payload[frame.PayloadOffset + 0];
                        stats.RemRssi = frame.Payload[frame.PayloadOffset + 1];
                    }
                }

                stats.FrameErrors = parser.CrcErrors;

                MaybePromoteReceivingLink(type);

                bool isDuplicate = IsCrossLinkDuplicate(type, frame);
                bool shouldForward = ShouldForwardInbound(type, frame);
                if (isDuplicate && !shouldForward)
                {
                    stats.FramesDuplicate++;
                }

                if (shouldForward)
                {
                    stats.FramesForwarded++;
                    ForwardToMp(frame.Raw, frame.RawLength);
                }
            });
        }

        private bool IsCrossLinkDuplicate(LinkType source, MavlinkFrame frame)
        {
            if (!_cfg.DedupEnabled || frame.IsParamValue)
                return false;

            var key = new DedupKey(frame.Sysid, frame.Compid, frame.Msgid, frame.Seq, frame.RawHash);
            bool duplicate = false;
            lock (_dedupLock)
            {
                if (_seenFrames.TryGetValue(key, out var seen) &&
                    seen.Source != source &&
                    (DateTime.UtcNow - seen.Timestamp) < DEDUP_WINDOW)
                {
                    duplicate = true;
                }
                _seenFrames[key] = new DedupSeen(source, DateTime.UtcNow);

                if (DateTime.UtcNow >= _nextDedupSweep)
                {
                    SweepDedup();
                    _nextDedupSweep = DateTime.UtcNow + TimeSpan.FromSeconds(2);
                }
            }
            return duplicate;
        }

        private bool ShouldForwardInbound(LinkType source, MavlinkFrame frame)
        {
            if (frame.IsParamValue)
                return ShouldForwardParamValue(source);

            if (ManualOverride != LinkType.None)
                return source == ManualOverride;

            var target = ManualOverride != LinkType.None ? ManualOverride : ActiveLink;
            if (target == LinkType.None)
                target = _cfg.PreferredLink == LinkType.None ? LinkType.LTE : _cfg.PreferredLink;

            if (source == target)
                return true;

            return !IsLinkUsable(target) && IsLinkUsable(source);
        }

        private void MaybePromoteReceivingLink(LinkType source)
        {
            if (ManualOverride != LinkType.None || source == ActiveLink)
                return;

            if (!IsLinkUsable(ActiveLink) && IsLinkUsable(source))
                SetActiveLink(source, "active link idle, alternate link receiving traffic");
        }

        private void SweepDedup()
        {
            var cutoff = DateTime.UtcNow - DEDUP_WINDOW;
            var stale = new List<DedupKey>();
            foreach (var kvp in _seenFrames)
                if (kvp.Value.Timestamp < cutoff) stale.Add(kvp.Key);
            foreach (var k in stale) _seenFrames.Remove(k);
        }

        private void UpdateSeqLoss(LinkType type, byte sysid, byte compid, byte seq)
        {
            int key = (sysid << 8) | compid;
            var dict = type == LinkType.LTE ? _lteLastSeqByComp : _radioLastSeqByComp;

            if (dict.TryGetValue(key, out byte last))
            {
                int delta = (seq - last + 256) & 0xFF;
                if (delta > 0)
                {
                    if (type == LinkType.LTE)
                    {
                        _lteSeenSeqCount++;
                        if (delta > 1) _lteLostSeqCount += (delta - 1);
                    }
                    else
                    {
                        _radioSeenSeqCount++;
                        if (delta > 1) _radioLostSeqCount += (delta - 1);
                    }
                }
                // delta == 0 means duplicate seq from the same component — ignore.
            }
            dict[key] = seq;

            if (type == LinkType.LTE)
            {
                long total = _lteSeenSeqCount + _lteLostSeqCount;
                Lte.PacketLossPercent = total > 0 ? Math.Min(100, _lteLostSeqCount * 100.0 / total) : 0;
            }
            else
            {
                long total = _radioSeenSeqCount + _radioLostSeqCount;
                Radio.PacketLossPercent = total > 0 ? Math.Min(100, _radioLostSeqCount * 100.0 / total) : 0;
            }
        }

        // ============================================================
        // Forwarding to Mission Planner
        // ============================================================

        private void ForwardToMp(byte[] data, int length)
        {
            var ep = _mpEndpoint;
            if (ep == null) return;
            try { _localSock?.Send(data, length, ep); }
            catch { /* socket closed or MP not listening yet */ }
        }

        // ============================================================
        // Param transaction pinning
        // ============================================================

        private bool ShouldForwardParamValue(LinkType source)
        {
            lock (_paramLock)
            {
                if (ManualOverride != LinkType.None)
                    return source == ManualOverride;

                var now = DateTime.UtcNow;
                if (_paramTransactionSource == LinkType.None || (now - _lastParamActivityTime) > PARAM_TRANSACTION_TIMEOUT)
                    _paramTransactionSource = source;

                if (source != _paramTransactionSource)
                    return false;

                _lastParamActivityTime = now;
                return true;
            }
        }

        private LinkType SelectParamTransactionSource()
        {
            lock (_paramLock)
            {
                var now = DateTime.UtcNow;
                if (_paramTransactionSource != LinkType.None && (now - _lastParamActivityTime) <= PARAM_TRANSACTION_TIMEOUT)
                {
                    _lastParamActivityTime = now;
                    return _paramTransactionSource;
                }

                var preferred = ManualOverride != LinkType.None ? ManualOverride : ActiveLink;
                if (ManualOverride != LinkType.None)
                    _paramTransactionSource = ManualOverride;
                else if (IsLinkUsable(preferred))
                    _paramTransactionSource = preferred;
                else if (IsLinkUsable(LinkType.LTE))
                    _paramTransactionSource = LinkType.LTE;
                else if (IsLinkUsable(LinkType.RadioMaster))
                    _paramTransactionSource = LinkType.RadioMaster;
                else
                    _paramTransactionSource = preferred == LinkType.None ? LinkType.LTE : preferred;

                _lastParamActivityTime = now;
                return _paramTransactionSource;
            }
        }

        private bool IsLinkUsable(LinkType link)
        {
            if (link == LinkType.LTE) return Lte.IsOpen && Lte.IsConnected;
            if (link == LinkType.RadioMaster) return Radio.IsOpen && Radio.IsConnected;
            return false;
        }
    }
}
