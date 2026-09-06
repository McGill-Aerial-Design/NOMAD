// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Dual-link stress tests — high-rate stress half
// ============================================================
// Companion partial: mirrored and bidirectional high-rate stress
// with strict no-loss / no-duplicate checks. Split from
// DualLinkStressTests.Router.cs to keep files under the
// source-size policy.
// ============================================================

using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using NOMAD.MissionPlanner;

internal static partial class DualLinkStressTests
{
    private static async Task StressMirrored()
    {
        using (var bed = new Bed())
        {
            await bed.LatchMp();
            bed.SendLte(Frames.Heartbeat(1, 9, 0)); // pin active=LTE before the storm
            await WaitUntil(() => bed.Router.Lte.IsConnected, 2000);

            const int N = 3000;
            var frames = new byte[N][];
            for (int i = 0; i < N; i++) frames[i] = Frames.Marker((uint)(100000 + i), (byte)i);

            Func<Action<byte[]>, int, Task> blast = async (send, seed) =>
            {
                var rng = new Random(seed);
                int i = 0;
                while (i < N)
                {
                    // Batch 1–3 frames per datagram to exercise the stream parser.
                    int k = Math.Min(1 + rng.Next(3), N - i);
                    int len = 0;
                    for (int j = 0; j < k; j++) len += frames[i + j].Length;
                    var dgram = new byte[len];
                    int off = 0;
                    for (int j = 0; j < k; j++)
                    {
                        Buffer.BlockCopy(frames[i + j], 0, dgram, off, frames[i + j].Length);
                        off += frames[i + j].Length;
                    }
                    send(dgram);
                    i += k;
                    if ((i & 31) < k) await Task.Delay(1); // pace the burst so OS buffers never overflow
                }
            };

            var t1 = Task.Run(() => blast(d => bed.SendLte(d), 42));
            var t2 = Task.Run(() => blast(d => bed.SendRadio(d), 43));
            await Task.WhenAll(t1, t2);

            var got = new List<uint>();
            Check(await WaitUntil(() => { DrainMarkers(bed.Mp, got); return got.Count >= N; }, 15000),
                $"all {N} mirrored frames delivered");
            await Task.Delay(900); // > dedup window: late cross-link twins would surface here
            DrainMarkers(bed.Mp, got);
            CheckEq(got.Count, N, "zero duplicates under mirrored stress");
            CheckEq(got.Distinct().Count(), N, "zero loss under mirrored stress");
            Check(bed.Router.IsRunning, "router still running after stress");
            Check(bed.Router.Lte.FramesReceived >= N, $"LTE ingested full stream ({bed.Router.Lte.FramesReceived}/{N})");
            Check(bed.Router.Radio.FramesReceived >= N, $"Radio ingested full stream ({bed.Router.Radio.FramesReceived}/{N})");
            if (TestFailedSoFar) bed.DumpLogs("mirrored stress");
        }
    }

    private static async Task StressBidirectional()
    {
        using (var bed = new Bed())
        {
            await bed.LatchMp();
            bed.SendLte(Frames.Heartbeat(1, 9, 0)); // pin active=LTE before the storm
            await WaitUntil(() => bed.Router.Lte.IsConnected, 2000);

            const int N = 1500; // inbound mirrored frames per link
            const int M = 300;  // GCS-originated outbound frames
            var inbound = new byte[N][];
            for (int i = 0; i < N; i++) inbound[i] = Frames.Marker((uint)(200000 + i), (byte)i);

            Func<Action<byte[]>, Task> blastIn = async send =>
            {
                for (int i = 0; i < N; i++)
                {
                    send(inbound[i]);
                    if ((i & 31) == 31) await Task.Delay(1);
                }
            };
            Func<Task> blastOut = async () =>
            {
                for (int i = 0; i < M; i++)
                {
                    bed.Mp.Send(Frames.Marker((uint)(300000 + i), (byte)i, 255, 190));
                    if ((i & 15) == 15) await Task.Delay(1);
                }
            };

            bed.LteSrc.Drain(); bed.RadioSrc.Drain();
            await Task.WhenAll(
                Task.Run(() => blastIn(d => bed.SendLte(d))),
                Task.Run(() => blastIn(d => bed.SendRadio(d))),
                Task.Run(blastOut));

            var got = new List<uint>();
            Check(await WaitUntil(() => { DrainMarkers(bed.Mp, got); return got.Count(m => m >= 200000 && m < 300000) >= N; }, 15000),
                $"all {N} inbound frames delivered during bidirectional stress");
            await Task.Delay(900);
            DrainMarkers(bed.Mp, got);
            CheckEq(got.Count(m => m >= 200000 && m < 300000), N, "zero inbound duplicates under bidirectional stress");

            var lteOut = new List<uint>();
            Check(await WaitUntil(() => { DrainMarkers(bed.LteSrc, lteOut); return lteOut.Count(m => m >= 300000) >= M; }, 5000),
                $"all {M} outbound frames reached the active link");
            CheckEq(lteOut.Count(m => m >= 300000), M, "no outbound duplication");
            await Task.Delay(250);
            CheckEq(DrainMarkers(bed.RadioSrc).Count(m => m >= 300000), 0, "no outbound frames leaked to the standby link");

            Check(bed.Router.IsRunning, "router still running");
            Check(bed.Router.Lte.BytesSentOutbound > 0, "outbound byte counter advanced");
            Check(!string.IsNullOrEmpty(bed.Router.GetStatusSummary()), "status summary available");

            bed.Router.ResetCounters();
            CheckEq(bed.Router.Lte.FramesReceived, 0, "counters reset");
            if (TestFailedSoFar) bed.DumpLogs("bidirectional stress");
        }
    }
}
