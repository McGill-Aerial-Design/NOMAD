// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Dual-link router stress & behaviour tests
// ============================================================
// Compiled together with src/Connectivity/GroundLinkRouter*.cs,
// src/Connectivity/MAVLinkConnectionManager.cs and src/UI/Log.cs
// by scripts/build/test_plugin_duallink.ps1 (plain csc, no test
// framework — exits non-zero on failure).
// Run via `pixi run test-plugin-duallink`.
//
// Exercises the router over real loopback UDP sockets:
//   - MAVLink v1/v2 frame parsing (splits, batching, garbage)
//   - health grading thresholds
//   - cross-link dedup: mirrored streams reach MP exactly once
//   - automatic failover and preferred-link recovery
//   - manual link override (inbound + outbound pinning)
//   - outbound routing to the active link only (a GCS command
//     must never be duplicated onto both links)
//   - parameter transaction pinning: reads stay on one link and
//     re-pin to the surviving link after the pinned link dies
//   - bind-failure fallback + watchdog socket reopen
//   - stop/restart rebinding
//   - high-rate mirrored stress with strict no-loss/no-dup checks
// ============================================================

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using NOMAD.MissionPlanner;

internal static partial class DualLinkStressTests
{
    private static int _failures;
    private static int _failuresAtTestStart;
    private static bool TestFailedSoFar => _failures > _failuresAtTestStart;

    private static int Main()
    {
        RunAll().GetAwaiter().GetResult();
        Console.WriteLine(_failures == 0
            ? "All dual-link tests passed."
            : $"{_failures} dual-link assertion(s) FAILED.");
        return _failures == 0 ? 0 : 1;
    }

    private static async Task RunAll()
    {
        Run("parser: single v2 frame", ParserSingleV2);
        Run("parser: single v1 frame", ParserSingleV1);
        Run("parser: byte-by-byte split delivery", ParserByteSplit);
        Run("parser: back-to-back frames in one buffer", ParserBackToBack);
        Run("parser: garbage before frame is skipped", ParserGarbageSkipped);
        Run("parser: signed v2 frame", ParserSignedV2);
        Run("parser: frame split across chunk boundary", ParserChunkBoundary);
        Run("health: classification thresholds", HealthClassification);

        await RunAsync("router: single-link forwarding, no loss / no dup", SingleLinkNoLossNoDup);
        await RunAsync("router: mirrored traffic deduped to exactly one copy", DedupMirrored);
        await RunAsync("router: per-component sequence loss tracking", SeqLossPerComponent);
        await RunAsync("router: failover to healthy link + preferred recovery", FailoverAndRecovery);
        await RunAsync("router: manual override pins inbound and outbound", ManualOverride);
        await RunAsync("router: outbound goes to active link only", OutboundActiveOnly);
        await RunAsync("router: outbound split frame reassembled once", OutboundSplitFrame);
        await RunAsync("router: bind failure falls back, watchdog reopens", BindFailureWatchdog);
        await RunAsync("router: param transaction pinning and re-pin", ParamPinningAndRepin);
        await RunAsync("router: stop/restart rebinds cleanly", StopRestart);
        await RunAsync("router: stress — mirrored high-rate, strict no dup/loss", StressMirrored);
        await RunAsync("router: stress — bidirectional concurrent traffic", StressBidirectional);
        await RunAsync("manager: facade lifecycle and projections", ManagerFacade);
    }

    // ============================================================
    // Harness
    // ============================================================

    private static void Run(string name, Action test)
    {
        _failuresAtTestStart = _failures;
        var sw = Stopwatch.StartNew();
        try { test(); }
        catch (Exception ex) { _failures++; Console.WriteLine($"    FAIL: unhandled exception — {ex}"); }
        Console.WriteLine($"  [{(TestFailedSoFar ? "FAIL" : " ok ")}] {name} ({sw.ElapsedMilliseconds} ms)");
    }

    private static async Task RunAsync(string name, Func<Task> test)
    {
        _failuresAtTestStart = _failures;
        var sw = Stopwatch.StartNew();
        try { await test(); }
        catch (Exception ex) { _failures++; Console.WriteLine($"    FAIL: unhandled exception — {ex}"); }
        Console.WriteLine($"  [{(TestFailedSoFar ? "FAIL" : " ok ")}] {name} ({sw.ElapsedMilliseconds} ms)");
    }

    private static void Check(bool cond, string what)
    {
        if (!cond) { _failures++; Console.WriteLine($"    FAIL: {what}"); }
    }

    private static void CheckEq(long actual, long expected, string what)
        => Check(actual == expected, $"{what}: expected {expected}, got {actual}");

    private static async Task<bool> WaitUntil(Func<bool> cond, int timeoutMs, int pollMs = 15)
    {
        var sw = Stopwatch.StartNew();
        while (sw.ElapsedMilliseconds < timeoutMs)
        {
            if (cond()) return true;
            await Task.Delay(pollMs);
        }
        return cond();
    }

    // ============================================================
    // MAVLink frame builders
    // ============================================================

    private static class Frames
    {
        public const uint MARKER_MSGID = 30; // arbitrary telemetry id used for tracer frames

        public static byte[] V2(byte sysid, byte compid, uint msgid, byte seq, byte[] payload, bool signedFrame = false)
        {
            int total = 10 + payload.Length + 2 + (signedFrame ? 13 : 0);
            var f = new byte[total];
            f[0] = 0xFD;
            f[1] = (byte)payload.Length;
            f[2] = (byte)(signedFrame ? 0x01 : 0x00); // incompat_flags bit0 = signed
            f[4] = seq; f[5] = sysid; f[6] = compid;
            f[7] = (byte)(msgid & 0xFF);
            f[8] = (byte)((msgid >> 8) & 0xFF);
            f[9] = (byte)((msgid >> 16) & 0xFF);
            Buffer.BlockCopy(payload, 0, f, 10, payload.Length);
            // CRC/signature left zero — the router's parser frames only, no CRC check.
            return f;
        }

        public static byte[] V1(byte sysid, byte compid, byte msgid, byte seq, byte[] payload)
        {
            var f = new byte[6 + payload.Length + 2];
            f[0] = 0xFE; f[1] = (byte)payload.Length;
            f[2] = seq; f[3] = sysid; f[4] = compid; f[5] = msgid;
            Buffer.BlockCopy(payload, 0, f, 6, payload.Length);
            return f;
        }

        public static byte[] Heartbeat(byte sysid, byte compid, byte seq) => V2(sysid, compid, 0, seq, new byte[9]);

        public static byte[] Marker(uint id, byte seq, byte sysid = 1, byte compid = 1)
        {
            var p = new byte[8];
            p[0] = (byte)id; p[1] = (byte)(id >> 8); p[2] = (byte)(id >> 16); p[3] = (byte)(id >> 24);
            p[4] = 0xA5; p[5] = 0x5A; p[6] = 0xA5; p[7] = 0x5A;
            return V2(sysid, compid, MARKER_MSGID, seq, p);
        }

        public static byte[] ParamValue(ushort index, byte seq)
        {
            var p = new byte[25];
            p[0] = (byte)index; p[1] = (byte)(index >> 8);
            return V2(1, 1, 22, seq, p);
        }

        public static byte[] ParamRequestList(byte seq) => V2(255, 190, 21, seq, new byte[2]);

        public static uint MsgIdOf(byte[] d)
        {
            if (d.Length >= 10 && d[0] == 0xFD) return (uint)(d[7] | (d[8] << 8) | (d[9] << 16));
            if (d.Length >= 6 && d[0] == 0xFE) return d[5];
            return uint.MaxValue;
        }

        public static uint? MarkerIdOf(byte[] d)
        {
            if (d.Length < 18 || d[0] != 0xFD || MsgIdOf(d) != MARKER_MSGID) return null;
            return (uint)d[10] | ((uint)d[11] << 8) | ((uint)d[12] << 16) | ((uint)d[13] << 24);
        }

        public static ushort? ParamIndexOf(byte[] d)
        {
            if (d.Length < 12 || d[0] != 0xFD || MsgIdOf(d) != 22) return null;
            return (ushort)(d[10] | (d[11] << 8));
        }
    }

    // ============================================================
    // Loopback UDP test endpoints
    // ============================================================

    private sealed class UdpSink : IDisposable
    {
        public readonly UdpClient Client;
        public readonly ConcurrentQueue<byte[]> Rx = new ConcurrentQueue<byte[]>();
        private volatile bool _running = true;
        private const int SIO_UDP_CONNRESET = -1744830452;

        public UdpSink()
        {
            Client = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0));
            try { Client.Client.IOControl(SIO_UDP_CONNRESET, new byte[4], null); } catch { }
            Client.Client.ReceiveBufferSize = 1 << 20;
            Task.Run(RxLoop);
        }

        public static UdpSink ConnectedTo(int port)
        {
            var s = new UdpSink();
            s.Client.Connect(IPAddress.Loopback, port);
            return s;
        }

        private async Task RxLoop()
        {
            while (_running)
            {
                try
                {
                    var r = await Client.ReceiveAsync().ConfigureAwait(false);
                    Rx.Enqueue(r.Buffer);
                }
                catch (ObjectDisposedException) { break; }
                catch (SocketException) { if (!_running) break; }
                catch { if (!_running) break; }
            }
        }

        public void Send(byte[] d) => Client.Send(d, d.Length);
        public void SendTo(int port, byte[] d) => Client.Send(d, d.Length, new IPEndPoint(IPAddress.Loopback, port));

        public List<byte[]> Drain()
        {
            var l = new List<byte[]>();
            while (Rx.TryDequeue(out var d)) l.Add(d);
            return l;
        }

        public void Dispose()
        {
            _running = false;
            try { Client.Close(); } catch { }
        }
    }

    /// <summary>Background heartbeat sender simulating a vehicle on one link.</summary>
    private sealed class Pump : IDisposable
    {
        private readonly CancellationTokenSource _cts = new CancellationTokenSource();

        public Pump(Action<byte[]> send, int hz, byte sysid = 1, byte compid = 1)
        {
            int period = Math.Max(1, 1000 / hz);
            Task.Run(async () =>
            {
                byte seq = 0;
                while (!_cts.IsCancellationRequested)
                {
                    try { send(Frames.Heartbeat(sysid, compid, seq++)); } catch { }
                    try { await Task.Delay(period, _cts.Token); } catch { break; }
                }
            });
        }

        public void Dispose() => _cts.Cancel();
    }

    /// <summary>
    /// One router instance plus three loopback peers: Mission Planner (UDP
    /// client of the router's local port), the LTE uplink source, and the
    /// RadioMaster UDP source. Timings are shortened so failover paths run
    /// in test time instead of flight time.
    /// </summary>
    private sealed class Bed : IDisposable
    {
        private static int _nextBase = 28600;
        public static int NextBase() => Interlocked.Add(ref _nextBase, 4);

        public readonly GroundLinkRouter Router;
        public readonly UdpSink Mp, LteSrc, RadioSrc;
        public readonly int LocalPort, LtePort, RadioPort;
        public readonly ConcurrentQueue<string> Logs = new ConcurrentQueue<string>();
        public readonly List<FailoverEventArgs> Failovers = new List<FailoverEventArgs>();
        private byte _gcsSeq;

        public Bed(Action<GroundLinkRouter.RouterConfig> tweak = null, int basePort = 0)
        {
            int b = basePort != 0 ? basePort : NextBase();
            LocalPort = b; LtePort = b + 1; RadioPort = b + 2;
            var cfg = new GroundLinkRouter.RouterConfig
            {
                BindAddress = "127.0.0.1",
                LocalPort = LocalPort,
                LteBindPort = LtePort,
                RadioMasterConnectionType = "UDP",
                RadioBindPort = RadioPort,
                StatsTickMs = 50,
                HeartbeatTimeoutSec = 0.6,
                FailoverCooldownSec = 0.2,
                PreferredLink = LinkType.LTE,
                PreferredLinkReconnectDelaySec = 1,
            };
            tweak?.Invoke(cfg);
            Router = new GroundLinkRouter(cfg);
            Router.LogMessage += (s, m) => Logs.Enqueue($"{DateTime.UtcNow:HH:mm:ss.fff} {m}");
            Router.FailoverOccurred += (s, e) => { lock (Failovers) Failovers.Add(e); };
            Router.Start();
            Mp = UdpSink.ConnectedTo(LocalPort);
            LteSrc = new UdpSink();
            RadioSrc = new UdpSink();
        }

        /// <summary>The router learns MP's ephemeral endpoint from MP's first send.</summary>
        public async Task LatchMp()
        {
            Mp.Send(Frames.Heartbeat(255, 190, _gcsSeq++));
            await Task.Delay(80);
        }

        public void SendLte(byte[] d) => LteSrc.SendTo(LtePort, d);
        public void SendRadio(byte[] d) => RadioSrc.SendTo(RadioPort, d);

        public void DumpLogs(string why)
        {
            Console.WriteLine($"    -- router log ({why}) --");
            foreach (var l in Logs) Console.WriteLine($"    {l}");
        }

        public void Dispose()
        {
            try { Router.Dispose(); } catch { }
            Mp.Dispose(); LteSrc.Dispose(); RadioSrc.Dispose();
        }
    }

    private static List<uint> DrainMarkers(UdpSink s, List<uint> into = null)
    {
        into = into ?? new List<uint>();
        foreach (var d in s.Drain())
        {
            var m = Frames.MarkerIdOf(d);
            if (m.HasValue) into.Add(m.Value);
        }
        return into;
    }

    // ============================================================
    // Parser tests
    // ============================================================

    private static void ParserSingleV2()
    {
        var p = new MavlinkFrameParser();
        var f = Frames.V2(7, 8, 0x12345, 42, new byte[] { 1, 2, 3, 4, 5 });
        var got = new List<MavlinkFrame>();
        p.Push(f, f.Length, got.Add);
        CheckEq(got.Count, 1, "v2 emits one frame");
        if (got.Count == 1)
        {
            var m = got[0];
            Check(m.IsV2, "v2 flag set");
            CheckEq(m.Sysid, 7, "sysid");
            CheckEq(m.Compid, 8, "compid");
            CheckEq(m.Msgid, 0x12345, "24-bit msgid");
            CheckEq(m.Seq, 42, "seq");
            CheckEq(m.RawLength, f.Length, "raw length");
            Check(m.Raw.Take(f.Length).SequenceEqual(f), "raw bytes preserved");
            CheckEq(m.PayloadOffset, 10, "payload offset");
            CheckEq(m.PayloadLength, 5, "payload length");
        }
        CheckEq(p.FramesEmitted, 1, "FramesEmitted counter");
        CheckEq(p.ResyncCount, 0, "no resyncs");
    }

    private static void ParserSingleV1()
    {
        var p = new MavlinkFrameParser();
        var f = Frames.V1(3, 4, 33, 11, new byte[] { 9, 8, 7 });
        var got = new List<MavlinkFrame>();
        p.Push(f, f.Length, got.Add);
        CheckEq(got.Count, 1, "v1 emits one frame");
        if (got.Count == 1)
        {
            var m = got[0];
            Check(!m.IsV2, "v1 flag");
            CheckEq(m.Sysid, 3, "sysid");
            CheckEq(m.Compid, 4, "compid");
            CheckEq(m.Msgid, 33, "msgid");
            CheckEq(m.Seq, 11, "seq");
            CheckEq(m.RawLength, f.Length, "raw length");
            CheckEq(m.PayloadOffset, 6, "payload offset");
        }
    }

    private static void ParserByteSplit()
    {
        var p = new MavlinkFrameParser();
        var f = Frames.V2(1, 1, 33, 5, new byte[7]);
        int emitted = 0;
        bool premature = false;
        for (int i = 0; i < f.Length; i++)
        {
            int idx = i;
            p.Push(new[] { f[i] }, 1, _ =>
            {
                if (idx < f.Length - 1) premature = true;
                emitted++;
            });
        }
        CheckEq(emitted, 1, "byte-by-byte push emits exactly one frame");
        Check(!premature, "frame only emitted once complete");
    }

    private static void ParserBackToBack()
    {
        var p = new MavlinkFrameParser();
        var buf = Frames.V2(1, 1, 1, 0, new byte[3])
            .Concat(Frames.V1(1, 1, 2, 1, new byte[4]))
            .Concat(Frames.V2(1, 1, 3, 2, new byte[5]))
            .ToArray();
        var msgids = new List<uint>();
        p.Push(buf, buf.Length, m => msgids.Add(m.Msgid));
        Check(msgids.SequenceEqual(new uint[] { 1, 2, 3 }), $"three frames parsed in order (got [{string.Join(",", msgids)}])");
    }

    private static void ParserGarbageSkipped()
    {
        var p = new MavlinkFrameParser();
        var garbage = new byte[] { 0x00, 0x12, 0x34, 0x56, 0x99, 0x10 }; // no STX bytes
        var f = Frames.V2(1, 1, 77, 9, new byte[2]);
        var got = new List<MavlinkFrame>();
        p.Push(garbage, garbage.Length, got.Add);
        CheckEq(got.Count, 0, "garbage alone emits nothing");
        p.Push(f, f.Length, got.Add);
        CheckEq(got.Count, 1, "frame after garbage parses");
        if (got.Count == 1) CheckEq(got[0].Msgid, 77, "msgid after garbage");
    }

    private static void ParserSignedV2()
    {
        var p = new MavlinkFrameParser();
        var f = Frames.V2(1, 1, 5, 3, new byte[5], signedFrame: true);
        CheckEq(f.Length, 30, "signed v2 frame is header+payload+crc+13B signature");
        var got = new List<MavlinkFrame>();
        p.Push(f, f.Length - 1, got.Add);
        CheckEq(got.Count, 0, "signed frame not emitted before signature complete");
        p.Push(new[] { f[f.Length - 1] }, 1, got.Add);
        CheckEq(got.Count, 1, "signed frame emitted after final byte");
        if (got.Count == 1) CheckEq(got[0].RawLength, 30, "signed frame raw length includes signature");
    }

    private static void ParserChunkBoundary()
    {
        var p = new MavlinkFrameParser();
        var f1 = Frames.V2(1, 1, 10, 0, new byte[6]);
        var f2 = Frames.V2(1, 1, 11, 1, new byte[8]);
        var chunk1 = f1.Concat(f2.Take(7)).ToArray();
        var chunk2 = f2.Skip(7).ToArray();
        var msgids = new List<uint>();
        p.Push(chunk1, chunk1.Length, m => msgids.Add(m.Msgid));
        CheckEq(msgids.Count, 1, "only complete frame emitted from first chunk");
        p.Push(chunk2, chunk2.Length, m => msgids.Add(m.Msgid));
        Check(msgids.SequenceEqual(new uint[] { 10, 11 }), "both frames recovered across chunk boundary");
    }

    // ============================================================
    // Health classification (private static — invoked via reflection)
    // ============================================================

    private static void HealthClassification()
    {
        var mi = typeof(GroundLinkRouter).GetMethod("ClassifyHealth", BindingFlags.NonPublic | BindingFlags.Static);
        Check(mi != null, "ClassifyHealth method found");
        if (mi == null) return;
        var now = DateTime.UtcNow;

        LinkHealth Grade(Action<LinkSourceStats> set)
        {
            var s = new LinkSourceStats
            {
                IsOpen = true,
                IsConnected = true,
                LastPacketTime = now,
                LastHeartbeatTime = now,
                FramesReceived = 1000,
                DataRateBps = 5000,
                PacketLossPercent = 0,
                LatencyMs = 0,
            };
            set(s);
            mi.Invoke(null, new object[] { s, now });
            return s.Health;
        }

        Check(Grade(s => s.IsOpen = false) == LinkHealth.Disconnected, "closed link → Disconnected");
        Check(Grade(s => s.IsConnected = false) == LinkHealth.Disconnected, "silent link → Disconnected");
        Check(Grade(s => { }) == LinkHealth.Excellent, "fresh fast link → Excellent");
        Check(Grade(s => s.LastHeartbeatTime = now.AddSeconds(-2)) == LinkHealth.Good, "heartbeat 2s stale → Good");
        Check(Grade(s => s.LastHeartbeatTime = now.AddSeconds(-3)) == LinkHealth.Fair, "heartbeat 3s stale → Fair");
        Check(Grade(s => s.LastHeartbeatTime = now.AddSeconds(-5)) == LinkHealth.Poor, "heartbeat 5s stale → Poor");
        Check(Grade(s => s.LastHeartbeatTime = now.AddSeconds(-9)) == LinkHealth.Critical, "heartbeat 9s stale → Critical");
        Check(Grade(s => s.LastPacketTime = now.AddSeconds(-6)) == LinkHealth.Critical, "packets 6s stale → Critical");
        Check(Grade(s => s.PacketLossPercent = 10) == LinkHealth.Good, "10% loss → Good");
        Check(Grade(s => s.PacketLossPercent = 30) == LinkHealth.Fair, "30% loss → Fair");
        Check(Grade(s => s.PacketLossPercent = 70) == LinkHealth.Poor, "70% loss → Poor");
        Check(Grade(s => s.LatencyMs = 600) == LinkHealth.Good, "600ms jitter → Good");
        Check(Grade(s => s.LatencyMs = 1600) == LinkHealth.Fair, "1600ms jitter → Fair");
        Check(Grade(s => s.DataRateBps = 10) == LinkHealth.Poor, "10 B/s on a busy link → Poor");
        Check(Grade(s => s.DataRateBps = 50) == LinkHealth.Fair, "50 B/s → Fair");
        Check(Grade(s => s.DataRateBps = 100) == LinkHealth.Good, "100 B/s → Good");
    }
}
