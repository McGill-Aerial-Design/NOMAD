// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Threading;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private sealed class FakeVisualTracker : IVisualTracker
    {
        internal volatile bool Found = true;
        internal bool HoldUpdates;
        internal readonly ManualResetEventSlim Entered = new ManualResetEventSlim();
        internal readonly ManualResetEventSlim Release = new ManualResetEventSlim();
        internal Bitmap InitializedImage;
        internal Color InitialColor;
        internal int Initializations, Updates, Disposals;
        private RectangleF _bounds;
        private readonly int _thread = Thread.CurrentThread.ManagedThreadId;
        public void Initialize(Bitmap frame, RectangleF bounds, CancellationToken token)
        {
            Check(Thread.CurrentThread.ManagedThreadId == _thread, "Tracker initialized outside its owner worker");
            token.ThrowIfCancellationRequested();
            Initializations++; InitializedImage = frame; InitialColor = frame.GetPixel(0, 0); _bounds = bounds;
        }
        public bool TryUpdate(Bitmap frame, CancellationToken token, out RectangleF bounds)
        {
            Check(Thread.CurrentThread.ManagedThreadId == _thread, "Tracker updated outside its owner worker");
            Updates++; Entered.Set();
            if (HoldUpdates) { Wait(Release, "native visual update release"); }
            token.ThrowIfCancellationRequested();
            bounds = _bounds;
            return Found;
        }
        public void Dispose()
        {
            Check(Thread.CurrentThread.ManagedThreadId == _thread, "Tracker disposed outside its owner worker");
            Disposals++;
        }
    }

    private static Bitmap SolidVideoImage(Color color)
    {
        var image = new Bitmap(100, 80);
        using (var graphics = Graphics.FromImage(image)) { graphics.Clear(color); }
        return image;
    }

    private static void PlayerVisionFrame(EmbeddedVideoPlayer player, FakeVideoPipeline pipeline, Color color)
    {
        long previous = player.DisplayedVideoFrame?.Sequence ?? -1;
        pipeline.Frames.Add(SolidVideoImage(color));
        var deadline = Stopwatch.StartNew();
        while (deadline.Elapsed < TimeSpan.FromSeconds(5))
        {
            Tick(player);
            if (player.DisplayedVideoFrame?.Sequence > previous) { return; }
            Thread.Sleep(5);
        }
        throw new InvalidOperationException("Timed out waiting for player's processed visual frame: " + player.TrackingStatus);
    }

    private static void GenericVisualWorker()
    {
        var pipeline = new FakeVideoPipeline();
        var trackers = new List<FakeVisualTracker>();
        using (var session = new VideoSession(() => pipeline, createTracker: token =>
            { var tracker = new FakeVisualTracker(); trackers.Add(tracker); return tracker; }))
        {
            session.Start("fake"); Wait(pipeline.ReadEntered, "visual warmup");
            Check(session.TrackerReady && trackers.Count == 1, "Visual runtime was not warmed before selectable frames");
            pipeline.Frames.Add(SolidVideoImage(Color.Red));
            using (var source = NextVideoFrame(session))
            {
                long first = session.RequestRegion(source, new RectangleF(10, 10, 20, 20));
                var retiredClone = GetField<Bitmap>(GetField<object>(session, "_pendingSelection"), "Image");
                long selected = session.RequestRegion(source, new RectangleF(30, 20, 20, 20));
                Check(first != 0 && selected > first, "Selection did not replace its single pending request");
                CheckDisposed(retiredClone);
                pipeline.Frames.Add(SolidVideoImage(Color.Blue));
                using (var current = NextVideoFrame(session))
                {
                    var tracker = trackers[trackers.Count - 1];
                    Check(current.Region.Found && current.Region.Revision == selected && current.Region.Bounds.Value.X == 30 &&
                        tracker.InitialColor.ToArgb() == Color.Red.ToArgb() && !ReferenceEquals(tracker.InitializedImage, source.Image),
                        "Selection initialized against a newer image or published mismatching target results");
                    CheckDisposed(tracker.InitializedImage);
                    tracker.Found = false;
                }
                pipeline.Frames.Add(SolidVideoImage(Color.Blue));
                using (var lost = NextVideoFrame(session))
                { Check(lost.Region != null && !lost.Region.Found && !session.IsRegionCurrent(selected), "Loss retained visual admission"); }
                pipeline.Frames.Add(SolidVideoImage(Color.Red));
                using (var later = NextVideoFrame(session))
                { Check(later.Region != null && !later.Region.Found, "Lost marker was dropped or target silently reacquired"); }
            }
        }
        CheckClean(pipeline);
        pipeline = new FakeVideoPipeline();
        FakeVisualTracker active = null;
        using (var session = new VideoSession(() => pipeline,
            createTracker: token => active = new FakeVisualTracker { HoldUpdates = true }))
        {
            session.Start("fake"); Wait(pipeline.ReadEntered, "held visual source");
            pipeline.Frames.Add(SolidVideoImage(Color.Red));
            using (var source = NextVideoFrame(session))
            {
                long selected = session.RequestRegion(source, new RectangleF(20, 20, 20, 20));
                pipeline.Frames.Add(SolidVideoImage(Color.Blue));
                Check(SpinWait.SpinUntil(() => active.Updates > 0, TimeSpan.FromSeconds(5)),
                    "Selected native update did not begin");
                Wait(active.Entered, "held native visual update");
                session.RetireRegion();
                Check(!session.IsRegionCurrent(selected), "Stop retained visual admission during native update");
                active.Release.Set();
                using (var late = NextVideoFrame(session))
                { Check(late.Region == null, "Late native visual result escaped its retired selection revision"); }
                Check(active.Disposals == 1, "Retired native visual tracker was not disposed by its worker");
                var pending = GetField<object>(session, "_pendingSelection");
                Check(pending == null, "Stop retained its pending image clone");
            }
        }
        CheckClean(pipeline);
        Check(VideoGeometry.TryImageRegion(new Point(10, 20), new Point(90, 90), new Size(100, 80),
            new Size(100, 100), out var drag) && drag.X == 10 && drag.Y == 10 && drag.Width == 80 && drag.Height == 70,
            "Drag region did not use the same zoom/letterbox mapping as clicks");
        var scaled = VideoGeometry.DefaultRegion(new PointF(1000, 1000), new Size(4000, 3000));
        Check(scaled.Width == 400 && scaled.Height == 400, "Click region became too small after tracker downscale");
    }
}
