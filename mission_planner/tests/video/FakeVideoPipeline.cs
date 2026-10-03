// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Concurrent;
using System.Drawing;
using System.Threading;
using NOMAD.MissionPlanner;

internal sealed class FakeVideoPipeline : IVideoPipeline
{
    public readonly ManualResetEventSlim StartEntered = new ManualResetEventSlim();
    public readonly ManualResetEventSlim ReleaseStart = new ManualResetEventSlim(true);
    public readonly ManualResetEventSlim ReadEntered = new ManualResetEventSlim();
    public readonly ManualResetEventSlim Cancelled = new ManualResetEventSlim();
    public readonly ManualResetEventSlim ReleaseRead = new ManualResetEventSlim();
    public readonly BlockingCollection<Bitmap> Frames = new BlockingCollection<Bitmap>();
    public bool FailStartup;
    public bool HoldRead;
    public Bitmap LateFrame;
    public int Starts;
    public int Disposals;
    public int Resources;
    public string Pipeline;
    private CancellationTokenRegistration _cancellation;

    public void Start(string pipeline, CancellationToken cancellation)
    {
        Interlocked.Increment(ref Starts);
        Pipeline = pipeline;
        _cancellation = cancellation.Register(() => Cancelled.Set());
        Resources = 2;
        StartEntered.Set();
        VideoLifetimeTests.Wait(ReleaseStart, "startup release");
        if (FailStartup)
        {
            throw new InvalidOperationException("injected partial startup failure");
        }
        Resources = 3;
    }

    public Bitmap ReadFrame(CancellationToken cancellation)
    {
        ReadEntered.Set();
        if (HoldRead)
        {
            VideoLifetimeTests.Wait(ReleaseRead, "late frame release");
            return LateFrame;
        }
        return Frames.Take(cancellation);
    }

    public void Dispose()
    {
        Resources = 0;
        Interlocked.Increment(ref Disposals);
        _cancellation.Dispose();
        while (Frames.TryTake(out var frame))
        {
            frame.Dispose();
        }
    }
}
