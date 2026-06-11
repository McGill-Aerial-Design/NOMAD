// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
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
    public partial class GroundLinkRouter : IDisposable
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
            public string RadioMasterConnectionType = "UDP"; // "COM", "UDP", or "TCP"
            public int RadioBindPort = 14550;        // UDP listen port / TCP connect port (must differ from LteBindPort)
            public string RadioComPort = "COM3";     // serial path (used when RadioMasterConnectionType == "COM")
            public int RadioBaudRate = 420000;       // baud rate for serial
            public string RadioTcpHost = "127.0.0.1"; // TCP server host to connect to (e.g. SITL) when type == "TCP"

            // Connection-type convenience flags.
            public bool RadioIsSerial => "COM".Equals(RadioMasterConnectionType, StringComparison.OrdinalIgnoreCase);
            public bool RadioIsTcp => "TCP".Equals(RadioMasterConnectionType, StringComparison.OrdinalIgnoreCase);

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
        private TcpListener _radioTcpListener;
        private TcpClient _radioTcpClient;
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
        private DateTime _lastLocalReopenAttempt = DateTime.MinValue;
        private DateTime _lastLteReopenAttempt = DateTime.MinValue;
        private DateTime _lastRadioReopenAttempt = DateTime.MinValue;
        private static readonly TimeSpan REOPEN_BACKOFF = TimeSpan.FromSeconds(3);
        private volatile bool _localRxRunning;
        private readonly object _activeLinkLock = new object();

        // Windows-only ioctl: tells the UDP socket to *not* surface ICMP
        // "Port Unreachable" errors as SocketExceptions on the next Receive.
        // Without this, a single outbound Send to an endpoint that briefly
        // stops listening will kill the rx loop on the very next ReceiveAsync.
        private const int SIO_UDP_CONNRESET = -1744830452; // 0x9800000C
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
            if (!OpenLocalSocket())
            {
                Cleanup();
                throw new SocketException((int)SocketError.AddressAlreadyInUse);
            }

            OpenLte();
            OpenRadio();

            // Set initial active link to preferred, even if neither is connected yet —
            // it lets outbound packets flow through once a link comes up.
            ActiveLink = _cfg.PreferredLink == LinkType.None ? LinkType.LTE : _cfg.PreferredLink;
            ActiveLinkChanged?.Invoke(this, ActiveLink);

            Task.Run(() => StatsLoop(_cts.Token));
            EmitLog($"Router started: listening for Mission Planner on udp://{_cfg.BindAddress}:{_cfg.LocalPort}");
        }

        private bool OpenLocalSocket()
        {
            _lastLocalReopenAttempt = DateTime.UtcNow;
            try
            {
                if (!IPAddress.TryParse(_cfg.BindAddress, out var ip)) ip = IPAddress.Loopback;
                var sock = new UdpClient(new IPEndPoint(ip, _cfg.LocalPort));
                DisableUdpConnReset(sock);
                _localSock = sock;
                _mpEndpoint = null; // set when MP first sends to us
                LocalEndpoint = (IPEndPoint)sock.Client.LocalEndPoint;
                _localRxRunning = true;
                Task.Run(() => LocalRxLoop(sock, _cts.Token));
                return true;
            }
            catch (Exception ex)
            {
                _localRxRunning = false;
                try { _localSock?.Close(); } catch { }
                _localSock = null;
                EmitLog($"FATAL: could not allocate local socket → {_cfg.BindAddress}:{_cfg.LocalPort} — {ex.Message}");
                return false;
            }
        }

        public void Stop()
        {
            if (!IsRunning) return;
            try { _cts.Cancel(); } catch { }
            Cleanup();
            EmitLog("Router stopped");
        }

        private void Cleanup()
        {
            try { _lteSock?.Close(); } catch { } _lteSock = null;
            try { _radioSock?.Close(); } catch { } _radioSock = null;
            try { _radioTcpListener?.Stop(); } catch { } _radioTcpListener = null;
            try { _radioTcpClient?.Close(); } catch { } _radioTcpClient = null;
            try { if (_radioSerial?.IsOpen == true) _radioSerial.Close(); } catch { } _radioSerial = null;
            try { _localSock?.Close(); } catch { } _localSock = null;
            _localRxRunning = false;
            Lte.IsOpen = false; Lte.IsConnected = false; Lte.Health = LinkHealth.Disconnected;
            Radio.IsOpen = false; Radio.IsConnected = false; Radio.Health = LinkHealth.Disconnected;
        }

        public void Dispose()
        {
            Stop();
            try { _cts?.Dispose(); } catch { }
        }

        private static void DisableUdpConnReset(UdpClient sock)
        {
            // Windows-only: stops ICMP "Port Unreachable" responses from
            // outbound Sends from surfacing as SocketExceptions on the very
            // next Receive. Without this, ReceiveAsync throws WSAECONNRESET
            // (10054) any time we Send to a peer that briefly stops listening
            // — which silently killed the rx loop and froze the link's
            // heartbeat counter, leaving stale "connected" status in the UI.
            if (Environment.OSVersion.Platform != PlatformID.Win32NT) return;
            try
            {
                sock.Client.IOControl(SIO_UDP_CONNRESET, new byte[] { 0, 0, 0, 0 }, null);
            }
            catch { /* non-Windows or unsupported transport — best-effort */ }
        }

        // ============================================================
        // Public controls
        // ============================================================

        /// <summary>
        /// Manually pin outbound to a link. Pass LinkType.None to release.
        /// </summary>
        public bool SetManualOverride(LinkType target)
        {
            lock (_activeLinkLock) ManualOverride = target;
            if (target != LinkType.None) SetActiveLink(target, "manual override");
            else EmitLog("Manual override released — auto-failover resumed");
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
            if (_cfg.RadioMasterConnectionType.Equals("COM", StringComparison.OrdinalIgnoreCase))
            {
                Radio.Endpoint = $"{_cfg.RadioComPort} @ {_cfg.RadioBaudRate}";
            }
            else if (_cfg.RadioIsTcp)
            {
                Radio.Endpoint = $"tcp://{_cfg.RadioTcpHost}:{_cfg.RadioBindPort}";
            }
            else // UDP default
            {
                Radio.Endpoint = $"udp://0.0.0.0:{_cfg.RadioBindPort}";
            }
        }

        private void EmitLog(string msg)
        {
            try { Log.Debug(msg); } catch { }
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
        public long ResyncCount;
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
            ResyncCount++;
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
        public int PayloadOffset;
        public int PayloadLength;

        public bool IsHeartbeat => Msgid == 0; // MAVLINK_MSG_ID_HEARTBEAT
        public bool IsRadioStatus => Msgid == 109; // RADIO_STATUS
        public bool IsParamRequest => Msgid == 20 || Msgid == 21; // PARAM_REQUEST_READ / PARAM_REQUEST_LIST
        public bool IsParamValue => Msgid == 22; // PARAM_VALUE
    }
}
