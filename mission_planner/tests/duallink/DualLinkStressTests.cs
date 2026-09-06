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
