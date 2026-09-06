// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Dual-link stress tests — shared harness
// ============================================================
// Companion partial: failure counters, assertion helpers, MAVLink
// frame builders, the UDP sink/pump/bed fixtures, and the marker
// drain helper. Split from DualLinkStressTests.cs to keep files
// under the source-size policy.
// ============================================================

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using NOMAD.MissionPlanner;

internal static partial class DualLinkStressTests
{
    private static int _failures;
    private static int _failuresAtTestStart;
    private static bool TestFailedSoFar => _failures > _failuresAtTestStart;

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
}
