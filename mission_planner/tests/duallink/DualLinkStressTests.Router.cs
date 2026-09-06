// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Dual-link router stress tests — router/manager integration half
// ============================================================
// Companion to DualLinkStressTests.cs (harness, frame builders,
// parser and health tests live there). These tests drive a real
// GroundLinkRouter over loopback UDP sockets.
// ============================================================

using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using NOMAD.MissionPlanner;

internal static partial class DualLinkStressTests
{

    // ============================================================
    // Router integration tests
    // ============================================================

    private static async Task SingleLinkNoLossNoDup()
    {
        using (var bed = new Bed())
        {
            await bed.LatchMp();
            const int N = 500;
            for (int i = 0; i < N; i++)
            {
                bed.SendLte(Frames.Marker((uint)i, (byte)i));
                if (i % 50 == 49) await Task.Delay(2);
            }
            var got = new List<uint>();
            await WaitUntil(() => { DrainMarkers(bed.Mp, got); return got.Count >= N; }, 5000);
            await Task.Delay(150);
            DrainMarkers(bed.Mp, got);
            CheckEq(got.Count, N, "MP received frame count");
            CheckEq(got.Distinct().Count(), N, "unique marker count");
            CheckEq(bed.Router.Lte.FramesReceived, N, "router LTE FramesReceived");
            CheckEq(bed.Router.Lte.FramesForwarded, N, "router LTE FramesForwarded");
            CheckEq(bed.Router.Lte.FramesDuplicate, 0, "no duplicates recorded");
            Check(bed.Router.ActiveLink == LinkType.LTE, "active link stays LTE");
            if (TestFailedSoFar) bed.DumpLogs("single link");
        }
    }

    private static async Task DedupMirrored()
    {
        using (var bed = new Bed())
        {
            await bed.LatchMp();
            // Warm up LTE so the active link never flips mid-test (a flip can
            // legitimately double-deliver in-flight twins; that path is
            // exercised separately in the failover test).
            bed.SendLte(Frames.Heartbeat(1, 9, 0));
            await WaitUntil(() => bed.Router.Lte.IsConnected, 2000);

            const int N = 400;
            var rng = new Random(1234);
            for (int i = 0; i < N; i++)
            {
                var f = Frames.Marker((uint)(1000 + i), (byte)i);
                if (rng.Next(4) == 0) { bed.SendRadio(f); bed.SendLte(f); } // radio-first ordering race
                else { bed.SendLte(f); bed.SendRadio(f); }
                if (i % 40 == 39) await Task.Delay(2);
            }
            var got = new List<uint>();
            await WaitUntil(() => { DrainMarkers(bed.Mp, got); return got.Count >= N; }, 5000);
            await Task.Delay(800); // a full dedup window — late twins would surface as dups here
            DrainMarkers(bed.Mp, got);
            CheckEq(got.Count, N, "MP received exactly one copy of each mirrored frame");
            CheckEq(got.Distinct().Count(), N, "all unique markers present (no loss)");
            CheckEq(bed.Router.Lte.FramesReceived + bed.Router.Radio.FramesReceived, 2L * N + 1, "both links ingested the full stream");
            Check(bed.Router.Lte.FramesDuplicate + bed.Router.Radio.FramesDuplicate > 0, "cross-link duplicates were detected and counted");
            if (TestFailedSoFar) bed.DumpLogs("dedup mirrored");
        }
    }

    private static async Task SeqLossPerComponent()
    {
        using (var bed = new Bed())
        {
            await bed.LatchMp();
            // Two components interleaved on one link must not report phantom loss.
            for (byte i = 0; i < 50; i++)
            {
                bed.SendLte(Frames.Heartbeat(1, 1, i));
                bed.SendLte(Frames.Heartbeat(1, 191, i));
            }
            await WaitUntil(() => bed.Router.Lte.FramesReceived >= 100, 3000);
            Check(bed.Router.Lte.PacketLossPercent < 0.5,
                $"interleaved components report ~0% loss (got {bed.Router.Lte.PacketLossPercent:F1}%)");

            // Real gaps on one component must register as loss.
            for (int i = 0; i < 40; i++) bed.SendLte(Frames.Heartbeat(1, 1, (byte)(50 + i * 2)));
            await WaitUntil(() => bed.Router.Lte.PacketLossPercent > 5, 3000);
            Check(bed.Router.Lte.PacketLossPercent > 5,
                $"gappy stream reports loss (got {bed.Router.Lte.PacketLossPercent:F1}%)");
            if (TestFailedSoFar) bed.DumpLogs("seq loss");
        }
    }

    private static async Task FailoverAndRecovery()
    {
        using (var bed = new Bed())
        {
            await bed.LatchMp();
            // LTE comes up first so the router deterministically starts on the
            // preferred link (if radio traffic lands first while LTE is still
            // silent, the router rightly promotes radio until LTE recovers).
            var ltePump = new Pump(f => bed.SendLte(f), 10, 1, 1);
            Pump radioPump = null;
            try
            {
                Check(await WaitUntil(() => bed.Router.Lte.IsConnected, 2000), "LTE comes up");
                radioPump = new Pump(f => bed.SendRadio(f), 10, 1, 2);
                Check(await WaitUntil(() => bed.Router.Radio.IsConnected, 3000), "radio comes up");
                Check(bed.Router.ActiveLink == LinkType.LTE, "starts on preferred LTE");

                ltePump.Dispose(); // LTE goes silent mid-flight
                Check(await WaitUntil(() => bed.Router.ActiveLink == LinkType.RadioMaster, 5000), "fails over to RadioMaster");
                lock (bed.Failovers)
                    Check(bed.Failovers.Any(e => e.FromLink == LinkType.LTE && e.ToLink == LinkType.RadioMaster),
                        "failover event recorded LTE→Radio");

                // Telemetry must keep flowing on the surviving link. Markers use
                // their own compid so their seq counter does not interleave with
                // the heartbeat pump's and fake packet loss.
                bed.Mp.Drain();
                for (int i = 0; i < 20; i++) bed.SendRadio(Frames.Marker((uint)(2000 + i), (byte)i, 1, 7));
                var got = new List<uint>();
                Check(await WaitUntil(() => { DrainMarkers(bed.Mp, got); return got.Count >= 20; }, 3000),
                    "radio traffic reaches MP after failover");
                CheckEq(got.Distinct().Count(), got.Count, "no duplicates after failover");

                // Preferred link recovers → router returns to it after the hold-down.
                ltePump = new Pump(f => bed.SendLte(f), 10, 1, 1);
                Check(await WaitUntil(() => bed.Router.ActiveLink == LinkType.LTE, 8000), "returns to preferred LTE after recovery");
                lock (bed.Failovers)
                    Check(bed.Failovers.Any(e => e.ToLink == LinkType.LTE && e.Reason.Contains("preferred")),
                        "preferred-recovery event recorded");
                Check(bed.Router.FailoverLog.Count >= 2, "failover log retains events");
            }
            finally { ltePump.Dispose(); radioPump?.Dispose(); }
            if (TestFailedSoFar) bed.DumpLogs("failover");
        }
    }

    private static async Task ManualOverride()
    {
        using (var bed = new Bed())
        {
            await bed.LatchMp();
            using (var ltePump = new Pump(f => bed.SendLte(f), 10, 1, 1))
            {
                Check(await WaitUntil(() => bed.Router.Lte.IsConnected, 2000), "LTE up first");
            using (var radioPump = new Pump(f => bed.SendRadio(f), 10, 1, 2))
            {
                Check(await WaitUntil(() => bed.Router.Radio.IsConnected && bed.Router.ActiveLink == LinkType.LTE, 5000),
                    "both links up, active on preferred LTE");

                bed.Router.SetManualOverride(LinkType.RadioMaster);
                Check(bed.Router.ActiveLink == LinkType.RadioMaster, "override switches active link immediately");
                Check(bed.Router.ManualOverride == LinkType.RadioMaster, "override recorded");

                bed.Mp.Drain();
                for (int i = 0; i < 10; i++)
                {
                    // Marker compids are distinct from the pumps' so the marker seq
                    // counters do not interleave with heartbeats and fake loss.
                    bed.SendLte(Frames.Marker((uint)(3000 + i), (byte)i, 1, 7));
                    bed.SendRadio(Frames.Marker((uint)(3100 + i), (byte)i, 1, 8));
                }
                var got = new List<uint>();
                Check(await WaitUntil(() => { DrainMarkers(bed.Mp, got); return got.Count(m => m >= 3100 && m < 3200) >= 10; }, 3000),
                    "override-link traffic delivered");
                await Task.Delay(250);
                DrainMarkers(bed.Mp, got);
                CheckEq(got.Count(m => m >= 3000 && m < 3100), 0, "non-override link traffic suppressed");

                // Outbound pinned to the override link, never duplicated.
                bed.LteSrc.Drain(); bed.RadioSrc.Drain();
                bed.Mp.Send(Frames.Marker(3200, 99, 255, 190));
                var radioGot = new List<uint>();
                Check(await WaitUntil(() => { DrainMarkers(bed.RadioSrc, radioGot); return radioGot.Contains(3200u); }, 2000),
                    "outbound goes to override link");
                await Task.Delay(200);
                CheckEq(DrainMarkers(bed.LteSrc).Count(m => m == 3200), 0, "outbound not sent to the other link");

                bed.Router.SetManualOverride(LinkType.None);
                Check(bed.Router.ManualOverride == LinkType.None, "override released");
                Check(await WaitUntil(() => bed.Router.ActiveLink == LinkType.LTE, 8000),
                    "auto-failover resumes and returns to preferred link");
            }
            }
            if (TestFailedSoFar) bed.DumpLogs("manual override");
        }
    }

    private static async Task OutboundActiveOnly()
    {
        using (var bed = new Bed())
        {
            await bed.LatchMp();
            using (var ltePump = new Pump(f => bed.SendLte(f), 10, 1, 1))
            {
                Check(await WaitUntil(() => bed.Router.Lte.IsConnected, 2000), "LTE up first");
                using (var radioPump = new Pump(f => bed.SendRadio(f), 10, 1, 2))
                {
                    Check(await WaitUntil(() => bed.Router.Radio.IsConnected && bed.Router.ActiveLink == LinkType.LTE, 5000),
                        "both links up, active on LTE");

                    bed.LteSrc.Drain(); bed.RadioSrc.Drain();
                    for (int i = 0; i < 50; i++)
                    {
                        bed.Mp.Send(Frames.Marker((uint)(4000 + i), (byte)i, 255, 190));
                        if (i % 10 == 9) await Task.Delay(2);
                    }
                    var lteGot = new List<uint>();
                    Check(await WaitUntil(() => { DrainMarkers(bed.LteSrc, lteGot); return lteGot.Count >= 50; }, 3000),
                        "all outbound frames reach the active link");
                    CheckEq(lteGot.Distinct().Count(), 50, "no outbound duplication on the active link");
                    await Task.Delay(250);
                    CheckEq(DrainMarkers(bed.RadioSrc).Count, 0,
                        "no outbound frames leak to the standby link (a command must never go out twice)");
                }
            }
            if (TestFailedSoFar) bed.DumpLogs("outbound active only");
        }
    }

    private static async Task OutboundSplitFrame()
    {
        using (var bed = new Bed())
        {
            await bed.LatchMp();
            using (var ltePump = new Pump(f => bed.SendLte(f), 10))
            {
                Check(await WaitUntil(() => bed.Router.Lte.IsConnected, 3000), "LTE up");
                bed.LteSrc.Drain();

                var f = Frames.Marker(4500, 7, 255, 190);
                var head = new byte[12];
                var tail = new byte[f.Length - 12];
                Array.Copy(f, 0, head, 0, head.Length);
                Array.Copy(f, 12, tail, 0, tail.Length);
                bed.Mp.Send(head);
                await Task.Delay(50);
                bed.Mp.Send(tail);

                var got = new List<uint>();
                Check(await WaitUntil(() => { DrainMarkers(bed.LteSrc, got); return got.Contains(4500u); }, 2000),
                    "split outbound frame reassembled and forwarded");
                await Task.Delay(200);
                DrainMarkers(bed.LteSrc, got);
                CheckEq(got.Count(m => m == 4500), 1, "split frame forwarded exactly once");
            }
            if (TestFailedSoFar) bed.DumpLogs("outbound split frame");
        }
    }

    private static async Task BindFailureWatchdog()
    {
        int basePort = Bed.NextBase();
        var blocker = new UdpClient(new IPEndPoint(IPAddress.Any, basePort + 1)); // occupy the LTE port
        try
        {
            using (var bed = new Bed(null, basePort))
            {
                Check(!bed.Router.Lte.IsOpen, "LTE bind fails while port is occupied");
                Check(bed.Router.Radio.IsOpen, "radio still binds");
                await bed.LatchMp();
                using (var radioPump = new Pump(f => bed.SendRadio(f), 10, 1, 2))
                {
                    Check(await WaitUntil(() => bed.Router.ActiveLink == LinkType.RadioMaster, 4000),
                        "router promotes the only receiving link");

                    bed.Mp.Drain();
                    for (int i = 0; i < 10; i++) bed.SendRadio(Frames.Marker((uint)(5000 + i), (byte)i, 1, 2));
                    var got = new List<uint>();
                    Check(await WaitUntil(() => { DrainMarkers(bed.Mp, got); return got.Count >= 10; }, 3000),
                        "telemetry flows on the surviving link");

                    bed.RadioSrc.Drain();
                    bed.Mp.Send(Frames.Marker(5100, 1, 255, 190));
                    var rGot = new List<uint>();
                    Check(await WaitUntil(() => { DrainMarkers(bed.RadioSrc, rGot); return rGot.Contains(5100u); }, 2000),
                        "outbound uses the open link");

                    // Free the port: the watchdog must reopen LTE within its backoff.
                    blocker.Close();
                    Check(await WaitUntil(() => bed.Router.Lte.IsOpen, 8000), "watchdog reopens LTE after the port is freed");
                    using (var ltePump = new Pump(f => bed.SendLte(f), 10, 1, 1))
                    {
                        Check(await WaitUntil(() => bed.Router.Lte.IsConnected, 3000), "LTE carries traffic after reopen");
                        Check(await WaitUntil(() => bed.Router.ActiveLink == LinkType.LTE, 8000), "preferred link resumes after recovery");
                    }
                }
                if (TestFailedSoFar) bed.DumpLogs("bind failure watchdog");
            }
        }
        finally { try { blocker.Close(); } catch { } }
    }

    private static async Task ParamPinningAndRepin()
    {
        using (var bed = new Bed())
        {
            await bed.LatchMp();
            var ltePump = new Pump(f => bed.SendLte(f), 10, 1, 3);
            using (var radioPump = new Pump(f => bed.SendRadio(f), 10, 1, 2))
            {
                try
                {
                    Check(await WaitUntil(() => bed.Router.Lte.IsConnected && bed.Router.Radio.IsConnected
                        && bed.Router.ActiveLink == LinkType.LTE, 5000), "both links up, active on LTE");
                    bed.LteSrc.Drain(); bed.RadioSrc.Drain();

                    // A parameter download request must go out on exactly one link —
                    // duplicate PARAM_REQUEST bursts confuse ArduPilot's param stream.
                    bed.Mp.Send(Frames.ParamRequestList(1));
                    var lteFrames = new List<byte[]>();
                    Check(await WaitUntil(() => { lteFrames.AddRange(bed.LteSrc.Drain()); return lteFrames.Any(d => Frames.MsgIdOf(d) == 21); }, 2000),
                        "param request forwarded to the pinned (active) link");
                    await Task.Delay(200);
                    CheckEq(bed.RadioSrc.Drain().Count(d => Frames.MsgIdOf(d) == 21), 0,
                        "param request not duplicated to the second link");

                    // Mirrored PARAM_VALUE storm: MP must see each exactly once.
                    bed.Mp.Drain();
                    const int P = 200;
                    for (int i = 0; i < P; i++)
                    {
                        var pv = Frames.ParamValue((ushort)i, (byte)i);
                        bed.SendLte(pv);
                        bed.SendRadio(pv);
                        if (i % 25 == 24) await Task.Delay(2);
                    }
                    var seen = new List<ushort>();
                    Func<int, bool> drainParams = target =>
                    {
                        foreach (var d in bed.Mp.Drain())
                        {
                            var ix = Frames.ParamIndexOf(d);
                            if (ix.HasValue) seen.Add(ix.Value);
                        }
                        return seen.Count >= target;
                    };
                    Check(await WaitUntil(() => drainParams(P), 4000), "all param values delivered");
                    await Task.Delay(300);
                    drainParams(int.MaxValue);
                    CheckEq(seen.Count, P, "param values delivered exactly once (no cross-link duplicates)");
                    CheckEq(seen.Distinct().Count(), P, "every param index present (no loss)");

                    // Pinned link dies mid-download: after the 4 s transaction
                    // timeout, parameter traffic must re-pin to the survivor.
                    ltePump.Dispose();
                    Check(await WaitUntil(() => bed.Router.ActiveLink == LinkType.RadioMaster, 5000),
                        "failover after pinned link death");
                    await Task.Delay(4300); // PARAM_TRANSACTION_TIMEOUT must expire

                    seen.Clear(); bed.Mp.Drain();
                    for (int i = 0; i < 20; i++) bed.SendRadio(Frames.ParamValue((ushort)(1000 + i), (byte)i));
                    Check(await WaitUntil(() => drainParams(20), 3000), "param stream re-pins to the surviving link");

                    bed.RadioSrc.Drain();
                    bed.Mp.Send(Frames.ParamRequestList(2));
                    var radioFrames = new List<byte[]>();
                    Check(await WaitUntil(() => { radioFrames.AddRange(bed.RadioSrc.Drain()); return radioFrames.Any(d => Frames.MsgIdOf(d) == 21); }, 2000),
                        "subsequent param requests routed to the surviving link");
                }
                finally { ltePump.Dispose(); }
            }
            if (TestFailedSoFar) bed.DumpLogs("param pinning");
        }
    }

    private static async Task StopRestart()
    {
        using (var bed = new Bed())
        {
            await bed.LatchMp();
            using (var ltePump = new Pump(f => bed.SendLte(f), 10))
            {
                Check(await WaitUntil(() => bed.Router.Lte.IsConnected, 3000), "link up before restart");
                Check(bed.Router.IsRunning, "router running");

                bed.Router.Stop();
                Check(!bed.Router.IsRunning, "router stopped");
                Check(bed.Router.Lte.Health == LinkHealth.Disconnected, "links marked disconnected on stop");
                await Task.Delay(200);

                bed.Router.Start();
                Check(bed.Router.IsRunning, "router restarted");
                await bed.LatchMp(); // MP endpoint must be re-learned after restart
                bed.Mp.Drain();
                var got = new List<uint>();
                for (int i = 0; i < 10; i++) bed.SendLte(Frames.Marker((uint)(6000 + i), (byte)i));
                Check(await WaitUntil(() => { DrainMarkers(bed.Mp, got); return got.Count >= 10; }, 4000),
                    "traffic flows again after restart");
            }
            if (TestFailedSoFar) bed.DumpLogs("stop/restart");
        }
    }


    // ============================================================
    // Manager façade
    // ============================================================

    private static async Task ManagerFacade()
    {
        int b = Bed.NextBase();
        var cfg = new MAVLinkConnectionManager.ConnectionConfig
        {
            RouterBindAddress = "127.0.0.1",
            RouterLocalPort = b,
            LtePort = b + 1,
            RadioMasterPort = b + 2,
            MonitorIntervalMs = 100,
            HeartbeatTimeoutSec = 0.6,
            PreferredLinkReconnectDelaySec = 1,
        };
        using (var mgr = new MAVLinkConnectionManager(cfg))
        using (var lteSrc = new UdpSink())
        using (var mp = UdpSink.ConnectedTo(b))
        {
            mgr.StartMonitoring();
            Check(mgr.IsMonitoring, "manager starts monitoring");
            Check(mgr.LocalMergedEndpoint.Contains(b.ToString()), "merged endpoint advertises the router port");

            mp.Send(Frames.Heartbeat(255, 190, 0));
            using (var pump = new Pump(f => lteSrc.SendTo(b + 1, f), 10))
            {
                Check(await WaitUntil(() => mgr.LteStatistics.IsConnected && mgr.LteStatistics.HeartbeatCount > 0, 4000),
                    "stats projected from the router");
                Check(mgr.IsLteHealthy, "LTE reported healthy");
                Check(mgr.ActiveLink == LinkType.LTE, "active link is LTE");
                Check(mgr.GetLinkStatus().ActiveLink == "LTE", "snapshot reports active link");

                Check(mgr.SwitchToLink(LinkType.RadioMaster), "manual switch accepted");
                Check(mgr.ManualOverride == LinkType.RadioMaster, "manual override projected");
                mgr.SwitchToLink(LinkType.None);
                Check(mgr.ManualOverride == LinkType.None, "override released");

                Check(!string.IsNullOrEmpty(mgr.GetStatusSummary()), "status summary available");
                mgr.ResetCounters();
            }
            mgr.StopMonitoring();
            Check(!mgr.IsMonitoring, "manager stops monitoring");
        }
    }
}
