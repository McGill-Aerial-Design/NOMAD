// ============================================================
// Ground-side MAVLink Router (MAVProxy-style multiplexer)
// ============================================================
// Owns both source links (LTE + RadioMaster) directly, parses
// MAVLink v1/v2 frame boundaries, tracks real per-link health
// from packet flow, and exposes a single local UDP endpoint
// Mission Planner connects to as a UDP client.
//
// Both source sockets stay open and reading at all times, so
// failover is fast because both source links are read and scored
// at all times, but Mission Planner only receives one coherent
// MAVLink stream at a time. This avoids duplicate/interleaved
// parameter streams during ArduPilot parameter downloads.
// Outbound (GCS-originated) traffic is forwarded to whichever
// source link is currently "active" (the healthiest one).
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
    /// <summary>
    /// Live statistics for a single source link, owned by the router.
    /// </summary>
    public class LinkSourceStats
    {
        public LinkType Type;
        public string Endpoint = "";
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

    /// <summary>
    /// Local UDP/serial multiplexer that mirrors two MAVLink links into
    /// a single loopback endpoint for Mission Planner.
    /// </summary>
    public class GroundLinkRouter : IDisposable
    {
        // ============================================================
        // Configuration
        // ============================================================

        public class RouterConfig
        {
            public string BindAddress = "127.0.0.1";
            public int LocalPort = 14600;
            public bool DedupEnabled = true;

            // LTE
            public int LteBindPort = 14560;          // we listen here for Jetson uplink (14560 to avoid RC default 14550)
            public string LteRemoteHost = "";        // outbound to Jetson (empty = reply to LastRemote)
            public int LteRemotePort = 0;

            // RadioMaster
            public bool RadioIsSerial;               // true = use serial port, false = UDP
            public int RadioBindPort = 14550;        // UDP listen port (must differ from LteBindPort)
            public string RadioComPort = "COM3";     // serial path
            public int RadioBaudRate = 420000;

            public bool AutoFailoverEnabled = true;
            public LinkType PreferredLink = LinkType.LTE;
            public bool AutoReconnectPreferred = true;
            public int PreferredLinkReconnectDelaySec = 10;

            public int StatsTickMs = 250;
            public double HeartbeatTimeoutSec = 3.0;
            public double FailoverCooldownSec = 2.0;
        }

        // ============================================================
        // Public state
        // ============================================================

        public LinkSourceStats Lte { get; } = new LinkSourceStats { Type = LinkType.LTE, Health = LinkHealth.Disconnected };
        public LinkSourceStats Radio { get; } = new LinkSourceStats { Type = LinkType.RadioMaster, Health = LinkHealth.Disconnected };

        public LinkType ActiveLink { get; private set; } = LinkType.None;
        public LinkType ManualOverride { get; private set; } = LinkType.None; // None = follow auto-failover
        public IPEndPoint LocalEndpoint { get; private set; }
        public bool IsRunning => _cts != null && !_cts.IsCancellationRequested;
        public RouterConfig Config => _cfg;

        public event EventHandler<LinkType> ActiveLinkChanged;
        public event EventHandler<FailoverEventArgs> FailoverOccurred;
        public event EventHandler StatsUpdated;
        public event EventHandler<string> LogMessage;

        // ============================================================
        // Private state
        // ============================================================

        private readonly RouterConfig _cfg;
        private CancellationTokenSource _cts;

        // Sockets
        private UdpClient _lteSock;
        private UdpClient _radioSock;
        private SerialPort _radioSerial;
        private UdpClient _localSock;             // bound to LocalPort; MP connects here as a UDP client
        private volatile IPEndPoint _mpEndpoint;  // ephemeral endpoint MP sends from (captured on first received packet)

        // Frame parsers (one per source, stateful — handles split datagrams/serial chunks)
        private readonly MavlinkFrameParser _lteParser = new MavlinkFrameParser();
        private readonly MavlinkFrameParser _radioParser = new MavlinkFrameParser();
        private readonly MavlinkFrameParser _localParser = new MavlinkFrameParser();

        // Dedup
        private readonly object _dedupLock = new object();
        private readonly Dictionary<DedupKey, DedupSeen> _seenFrames = new Dictionary<DedupKey, DedupSeen>();
        private static readonly TimeSpan DEDUP_WINDOW = TimeSpan.FromMilliseconds(750);
        private DateTime _nextDedupSweep = DateTime.MinValue;

        // Health bookkeeping
        private DateTime _lastFailover = DateTime.MinValue;
        private DateTime _preferredHealthySince = DateTime.MinValue;
        private DateTime _lastStatsTick = DateTime.MinValue;
        private long _lteBytesAtLastTick;
        private long _radioBytesAtLastTick;
        // MAVLink seq is per-(sysid, compid); a single global last-seq mixes
        // counters from multiple components and reports phantom loss. Track
        // last-seen per (sysid << 8 | compid) tuple per link instead.
        private readonly Dictionary<int, byte> _lteLastSeqByComp = new Dictionary<int, byte>();
        private readonly Dictionary<int, byte> _radioLastSeqByComp = new Dictionary<int, byte>();
        private long _lteSeenSeqCount;
        private long _lteLostSeqCount;
        private long _radioSeenSeqCount;
        private long _radioLostSeqCount;

        // Parameter downloads are request/response transactions. Keep each
        // transaction pinned to one link so the flight controller does not get
        // duplicate PARAM_REQUEST_* bursts through LTE and RadioMaster at once.
        private readonly object _paramLock = new object();
        private LinkType _paramTransactionSource = LinkType.None;
        private DateTime _lastParamActivityTime = DateTime.MinValue;
        private static readonly TimeSpan PARAM_TRANSACTION_TIMEOUT = TimeSpan.FromSeconds(4);

        // Failover log (ring buffer)
        private readonly LinkedList<FailoverEventArgs> _failoverLog = new LinkedList<FailoverEventArgs>();
        private const int FAILOVER_LOG_MAX = 50;

        // ============================================================
        // Construction
        // ============================================================

        public GroundLinkRouter(RouterConfig cfg)
        {
            _cfg = cfg ?? throw new ArgumentNullException(nameof(cfg));
            UpdateEndpointStrings();
        }

        public IReadOnlyCollection<FailoverEventArgs> FailoverLog
        {
            get { lock (_failoverLog) return new List<FailoverEventArgs>(_failoverLog); }
        }

        // ============================================================
        // Lifecycle
        // ============================================================

        public void Start()
        {
            if (IsRunning) return;
            _cts = new CancellationTokenSource();

            // MP connects as a UDP *client* to (BindAddress:LocalPort), so the
            // router has to bind the local socket on LocalPort and act as the
            // server. MP's ephemeral source endpoint is captured on the first
            // received packet, then used as the return path for downlink frames.
            try
            {
                if (!IPAddress.TryParse(_cfg.BindAddress, out var ip)) ip = IPAddress.Loopback;
                _localSock = new UdpClient(new IPEndPoint(ip, _cfg.LocalPort));
                _mpEndpoint = null; // set when MP first sends to us
                LocalEndpoint = (IPEndPoint)_localSock.Client.LocalEndPoint;
            }
            catch (Exception ex)
            {
                Log($"FATAL: could not allocate local socket → {_cfg.BindAddress}:{_cfg.LocalPort} — {ex.Message}");
                Cleanup();
                throw;
            }

            OpenLte();
            OpenRadio();

            // Set initial active link to preferred, even if neither is connected yet —
            // it lets outbound packets flow through once a link comes up.
            ActiveLink = _cfg.PreferredLink == LinkType.None ? LinkType.LTE : _cfg.PreferredLink;
            ActiveLinkChanged?.Invoke(this, ActiveLink);

            Task.Run(() => LocalRxLoop(_cts.Token));
            Task.Run(() => StatsLoop(_cts.Token));
            Log($"Router started: listening for Mission Planner on udp://{_cfg.BindAddress}:{_cfg.LocalPort}");
        }

        public void Stop()
        {
            if (!IsRunning) return;
            try { _cts.Cancel(); } catch { }
            Cleanup();
            Log("Router stopped");
        }

        private void Cleanup()
        {
            try { _lteSock?.Close(); } catch { } _lteSock = null;
            try { _radioSock?.Close(); } catch { } _radioSock = null;
            try { if (_radioSerial?.IsOpen == true) _radioSerial.Close(); } catch { } _radioSerial = null;
            try { _localSock?.Close(); } catch { } _localSock = null;
            Lte.IsOpen = false; Lte.IsConnected = false; Lte.Health = LinkHealth.Disconnected;
            Radio.IsOpen = false; Radio.IsConnected = false; Radio.Health = LinkHealth.Disconnected;
        }

        public void Dispose()
        {
            Stop();
            try { _cts?.Dispose(); } catch { }
        }

        // ============================================================
        // Source link setup
        // ============================================================

        private void OpenLte()
        {
            try
            {
                _lteSock = new UdpClient(new IPEndPoint(IPAddress.Any, _cfg.LteBindPort));
                Lte.IsOpen = true;
                Task.Run(() => UdpRxLoop(_lteSock, LinkType.LTE, _cts.Token));
                Log($"LTE listening on udp://0.0.0.0:{_cfg.LteBindPort}");
            }
            catch (Exception ex)
            {
                Lte.IsOpen = false;
                Log($"LTE bind failed on port {_cfg.LteBindPort}: {ex.Message}");
            }
        }

        private void OpenRadio()
        {
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
                    Log($"RadioMaster opened serial {_cfg.RadioComPort} @ {_cfg.RadioBaudRate}");
                }
                catch (Exception ex)
                {
                    Radio.IsOpen = false;
                    Log($"RadioMaster serial open failed ({_cfg.RadioComPort}): {ex.Message}");
                }
            }
            else
            {
                if (_cfg.RadioBindPort == _cfg.LteBindPort)
                {
                    Log($"RadioMaster UDP port {_cfg.RadioBindPort} collides with LTE port — skipping RC bind");
                    Radio.IsOpen = false;
                    return;
                }
                try
                {
                    _radioSock = new UdpClient(new IPEndPoint(IPAddress.Any, _cfg.RadioBindPort));
                    Radio.IsOpen = true;
                    Task.Run(() => UdpRxLoop(_radioSock, LinkType.RadioMaster, _cts.Token));
                    Log($"RadioMaster listening on udp://0.0.0.0:{_cfg.RadioBindPort}");
                }
                catch (Exception ex)
                {
                    Radio.IsOpen = false;
                    Log($"RadioMaster UDP bind failed on port {_cfg.RadioBindPort}: {ex.Message}");
                }
            }
        }

        // ============================================================
        // Receive loops
        // ============================================================

        private async Task UdpRxLoop(UdpClient sock, LinkType type, CancellationToken ct)
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
                catch (SocketException) { break; }
                catch (Exception ex) { Log($"{type} rx error: {ex.Message}"); await Task.Delay(100, ct).ContinueWith(_ => { }); }
            }
        }

        private LinkType ClassifyUdpSource(LinkType socketType, IPEndPoint remote)
        {
            if (socketType == LinkType.RadioMaster && IsTailscaleAddress(remote?.Address))
            {
                Log($"LTE telemetry arrived on RadioMaster UDP port {_cfg.RadioBindPort} from {remote}; treating it as LTE. Check Jetson GCS_PORT_LTE should be {_cfg.LteBindPort}.");
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
            while (!ct.IsCancellationRequested && port.IsOpen)
            {
                try
                {
                    int n = port.Read(buf, 0, buf.Length);
                    if (n > 0) ProcessIncoming(LinkType.RadioMaster, _radioParser, buf, n);
                }
                catch (TimeoutException) { }
                catch (Exception ex) { Log($"Radio serial rx error: {ex.Message}"); Thread.Sleep(100); }
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
                catch (Exception ex) { Log($"Local rx error: {ex.Message}"); }
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
        // Forwarding
        // ============================================================

        private void ForwardToMp(byte[] data, int length)
        {
            var ep = _mpEndpoint;
            if (ep == null) return;
            try { _localSock?.Send(data, length, ep); }
            catch { /* socket closed or MP not listening yet */ }
        }

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
                catch (Exception ex) { Log($"Stats loop error: {ex.Message}"); }
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
            // Heartbeat jitter is useful as a secondary signal, but ELRS and
            // serial/USB buffering can make it noisy even when packets are live.
            if (packetAge > 5.0 || hbAge > 8.0 || loss >= 40.0 ||
                (s.FramesReceived > 50 && rate < 5.0))
            {
                s.Health = LinkHealth.Critical;
            }
            else if (hbAge > 4.0 || loss >= 25.0 ||
                     (s.FramesReceived > 100 && rate < 20.0))
            {
                s.Health = LinkHealth.Poor;
            }
            else if (hbAge > 2.5 || loss >= 10.0 || jitter >= 1500.0 ||
                     (s.FramesReceived > 100 && rate < 80.0))
            {
                s.Health = LinkHealth.Fair;
            }
            else if (loss >= 3.0 || jitter >= 800.0 ||
                     (s.FramesReceived > 100 && rate < 250.0))
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
            Log($"Failover {from} → {newActive}: {reason}");
        }

        // ============================================================
        // Public controls
        // ============================================================

        /// <summary>
        /// Manually pin outbound to a link. Pass LinkType.None to release.
        /// </summary>
        public bool SetManualOverride(LinkType target)
        {
            ManualOverride = target;
            if (target != LinkType.None) SetActiveLink(target, "manual override");
            else Log("Manual override released — auto-failover resumed");
            return true;
        }

        /// <summary>Reset cumulative counters and history.</summary>
        public void ResetCounters()
        {
            Lte.FramesReceived = Lte.FramesForwarded = Lte.FramesDuplicate = 0;
            Lte.BytesReceived = Lte.BytesSentOutbound = 0;
            Lte.PacketLossPercent = 0;
            Radio.FramesReceived = Radio.FramesForwarded = Radio.FramesDuplicate = 0;
            Radio.BytesReceived = Radio.BytesSentOutbound = 0;
            Radio.PacketLossPercent = 0;
            _lteSeenSeqCount = _lteLostSeqCount = _radioSeenSeqCount = _radioLostSeqCount = 0;
            _lteLastSeqByComp.Clear();
            _radioLastSeqByComp.Clear();
            lock (_dedupLock) _seenFrames.Clear();
            lock (_paramLock)
            {
                _paramTransactionSource = LinkType.None;
                _lastParamActivityTime = DateTime.MinValue;
            }
        }

        /// <summary>Live single-line status for tooltips / dashboards.</summary>
        public string GetStatusSummary()
        {
            string l(LinkSourceStats s) => s.IsConnected
                ? $"{s.Health} {s.LatencyMs:F0}ms {s.PacketLossPercent:F1}%"
                : "offline";
            return $"Active: {ActiveLink}  |  LTE: {l(Lte)}  |  Radio: {l(Radio)}";
        }

        private void UpdateEndpointStrings()
        {
            Lte.Endpoint = $"udp://0.0.0.0:{_cfg.LteBindPort}";
            Radio.Endpoint = _cfg.RadioIsSerial
                ? $"{_cfg.RadioComPort} @ {_cfg.RadioBaudRate}"
                : $"udp://0.0.0.0:{_cfg.RadioBindPort}";
        }

        private void Log(string msg)
        {
            try { Console.WriteLine($"NOMAD Router: {msg}"); } catch { }
            try { LogMessage?.Invoke(this, msg); } catch { }
        }

        // ============================================================
        // Dedup key
        // ============================================================

        private struct DedupKey : IEquatable<DedupKey>
        {
            public readonly byte Sysid;
            public readonly byte Compid;
            public readonly uint Msgid;
            public readonly byte Seq;
            public readonly uint RawHash;

            public DedupKey(byte sysid, byte compid, uint msgid, byte seq, uint rawHash)
            { Sysid = sysid; Compid = compid; Msgid = msgid; Seq = seq; RawHash = rawHash; }

            public bool Equals(DedupKey o) => Sysid == o.Sysid && Compid == o.Compid && Msgid == o.Msgid && Seq == o.Seq && RawHash == o.RawHash;
            public override bool Equals(object o) => o is DedupKey k && Equals(k);
            public override int GetHashCode()
            {
                unchecked
                {
                    int h = 17;
                    h = h * 31 + Sysid;
                    h = h * 31 + Compid;
                    h = h * 31 + (int)Msgid;
                    h = h * 31 + Seq;
                    h = h * 31 + (int)RawHash;
                    return h;
                }
            }
        }

        private struct DedupSeen
        {
            public readonly LinkType Source;
            public readonly DateTime Timestamp;

            public DedupSeen(LinkType source, DateTime timestamp)
            {
                Source = source;
                Timestamp = timestamp;
            }
        }
    }

    // ================================================================
    // MAVLink frame parser
    // ================================================================
    // Lightweight byte-stream parser for v1 (STX 0xFE) and v2 (STX 0xFD)
    // frames. Does not validate CRC — we trust the upstream and just need
    // accurate frame boundaries for dedup/forwarding. Frame errors
    // (resync events) are counted.
    // ================================================================

    internal class MavlinkFrameParser
    {
        public long CrcErrors;
        public long FramesEmitted;

        private const byte STX_V1 = 0xFE;
        private const byte STX_V2 = 0xFD;
        private const int MAX_FRAME = 280; // v2 + signature

        private readonly byte[] _buf = new byte[MAX_FRAME];
        private int _len;
        private int _expected; // expected total frame length, 0 = unknown yet

        public void Push(byte[] data, int length, Action<MavlinkFrame> onFrame)
        {
            for (int i = 0; i < length; i++)
            {
                byte b = data[i];

                if (_len == 0)
                {
                    if (b == STX_V1 || b == STX_V2)
                    {
                        _buf[_len++] = b;
                        _expected = 0;
                    }
                    continue;
                }

                if (_len < MAX_FRAME) _buf[_len++] = b;
                else { Resync(); continue; }

                if (_expected == 0 && _len >= 2)
                {
                    byte payloadLen = _buf[1];
                    if (_buf[0] == STX_V1) _expected = 6 + payloadLen + 2; // header(6)+payload+crc(2)
                    else
                    {
                        // v2: header(10)+payload+crc(2)+signature(13 if MAVLINK_IFLAG_SIGNED bit 0 of incompat_flags)
                        bool signed = _len >= 3 && (_buf[2] & 0x01) != 0;
                        _expected = 10 + payloadLen + 2 + (signed ? 13 : 0);
                    }
                    if (_expected > MAX_FRAME) { Resync(); continue; }
                }

                if (_expected > 0 && _len >= _expected)
                {
                    EmitFrame(onFrame);
                    _len = 0; _expected = 0;
                }
            }
        }

        private void Resync()
        {
            CrcErrors++;
            // Find next STX in current buffer to recover (cheap heuristic)
            for (int j = 1; j < _len; j++)
            {
                if (_buf[j] == STX_V1 || _buf[j] == STX_V2)
                {
                    int rem = _len - j;
                    Buffer.BlockCopy(_buf, j, _buf, 0, rem);
                    _len = rem;
                    _expected = 0;
                    return;
                }
            }
            _len = 0; _expected = 0;
        }

        private void EmitFrame(Action<MavlinkFrame> onFrame)
        {
            FramesEmitted++;
            bool v2 = _buf[0] == STX_V2;
            byte payloadLen = _buf[1];

            byte seq, sysid, compid;
            uint msgid;
            int payloadOffset;

            if (v2)
            {
                seq = _buf[4];
                sysid = _buf[5];
                compid = _buf[6];
                msgid = (uint)(_buf[7] | (_buf[8] << 8) | (_buf[9] << 16));
                payloadOffset = 10;
            }
            else
            {
                seq = _buf[2];
                sysid = _buf[3];
                compid = _buf[4];
                msgid = _buf[5];
                payloadOffset = 6;
            }

            // We have to copy the raw frame because the parser reuses _buf.
            var raw = new byte[_expected];
            Buffer.BlockCopy(_buf, 0, raw, 0, _expected);
            uint rawHash = ComputeFnv1a(raw, _expected);

            onFrame(new MavlinkFrame
            {
                Raw = raw,
                RawLength = _expected,
                IsV2 = v2,
                Sysid = sysid,
                Compid = compid,
                Msgid = msgid,
                Seq = seq,
                RawHash = rawHash,
                Payload = raw,
                PayloadOffset = payloadOffset,
                PayloadLength = payloadLen,
            });
        }

        private static uint ComputeFnv1a(byte[] data, int length)
        {
            unchecked
            {
                uint hash = 2166136261u;
                for (int i = 0; i < length; i++)
                {
                    hash ^= data[i];
                    hash *= 16777619u;
                }
                return hash;
            }
        }
    }

    internal struct MavlinkFrame
    {
        public byte[] Raw;
        public int RawLength;
        public bool IsV2;
        public byte Sysid;
        public byte Compid;
        public uint Msgid;
        public byte Seq;
        public uint RawHash;
        public byte[] Payload;       // alias of Raw — payload begins at PayloadOffset
        public int PayloadOffset;
        public int PayloadLength;

        public bool IsHeartbeat => Msgid == 0; // MAVLINK_MSG_ID_HEARTBEAT
        public bool IsRadioStatus => Msgid == 109; // RADIO_STATUS
        public bool IsParamRequest => Msgid == 20 || Msgid == 21; // PARAM_REQUEST_READ / PARAM_REQUEST_LIST
        public bool IsParamValue => Msgid == 22; // PARAM_VALUE
    }
}
