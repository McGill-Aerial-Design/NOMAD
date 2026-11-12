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
    private static void TrackingSampleDisplayIndependence()
    {
        using (var peer = new TrackingPeer())
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), Size = new Size(800, 600) })
        {
            var pipeline = new FakeVideoPipeline();
            var profile = new VideoInputProfile { Name = "Independent samples", Url = "udp://@:5600",
                TrackingCalibrated = true };
            using (var player = new EmbeddedVideoPlayer(profile, true, CancellationToken.None, () => pipeline,
                peer.Config, createTracker: (token, mode) => new FakeVisualTracker()))
            {
                form.Controls.Add(player); form.Show(); player.StartStream();
                Wait(pipeline.ReadEntered, "independent sample source");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.SelectRegion(new RectangleF(40, 30, 20, 20)), "Independent sample selection failed");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                var selected = player.DisplayedVideoFrame;
                CheckTrackingOutlineVisibility(player, true);
                var sample = new VideoTrackingSample(selected.Image.Size, selected.Sequence + 1, selected.Generation,
                    Stopwatch.GetTimestamp(), DateTime.UtcNow, selected.Region, selected.Motion);
                var session = GetField<VideoSession>(player, "_session");
                session.PublishVideoFrame(selected.Generation, new VideoFrame(SolidVideoImage(Color.Green),
                    Array.Empty<VideoDetection>(), selected.Sequence + 100, selected.Generation,
                    Stopwatch.GetTimestamp(), DateTime.UtcNow, ""));
                Tick(player);
                Check(player.RegionReady && player.DisplayedVideoFrame.Region == null && player.StartTracking(),
                    "New plain display pixels invalidated a verified tracking sample");
                CheckTrackingOutlineVisibility(player, true);
                typeof(EmbeddedVideoPlayer).GetMethod("UpdateVisionSample",
                    System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic)
                    .Invoke(player, new object[] { sample });
                Check(player.RegionReady && player.IsTracking &&
                    GetField<long>(player, "_latestSequence") == sample.Sequence,
                    "A newer tracking sample was compared with the newer display sequence");
                player.StopTracking();
                var wrongRevision = new VideoRegionResult(sample.Region.Revision + 1, VisualTrackingMode.Object,
                    new VisualTrackingResult(VisualTrackingState.Verified, "Wrong selection", sample.Region.Bounds,
                        sample.Region.AimPoint));
                foreach (var invalid in new[]
                {
                    new VideoTrackingSample(sample.ImageSize, sample.Sequence, sample.Generation,
                        Stopwatch.GetTimestamp() - Stopwatch.Frequency, DateTime.UtcNow, sample.Region),
                    new VideoTrackingSample(sample.ImageSize, sample.Sequence, sample.Generation - 1,
                        Stopwatch.GetTimestamp(), DateTime.UtcNow, sample.Region),
                    new VideoTrackingSample(new Size(200, 160), sample.Sequence, sample.Generation,
                        Stopwatch.GetTimestamp(), DateTime.UtcNow, sample.Region),
                    new VideoTrackingSample(sample.ImageSize, sample.Sequence, sample.Generation,
                        Stopwatch.GetTimestamp(), DateTime.UtcNow, wrongRevision),
                })
                {
                    SetField(player, "_latestTrackingSample", invalid);
                    CheckTrackingOutlineVisibility(player, false);
                }
                session.PublishVideoFrame(sample.Generation, new VideoFrame(SolidVideoImage(Color.Green),
                    Array.Empty<VideoDetection>(), sample.Sequence + 200, sample.Generation,
                    Stopwatch.GetTimestamp(), DateTime.UtcNow, "", sample.Region));
                Tick(player);
                CheckTrackingOutlineVisibility(player, true);
                var searching = new VideoRegionResult(sample.Region.Revision, VisualTrackingMode.Object,
                    new VisualTrackingResult(VisualTrackingState.Searching, "Searching"));
                session.PublishTrackingSample(sample.Generation, new VideoTrackingSample(sample.ImageSize,
                    sample.Sequence + 201, sample.Generation, Stopwatch.GetTimestamp(), DateTime.UtcNow, searching));
                Tick(player);
                Check(player.DisplayedVideoFrame.Region.Found && !player.RegionReady,
                    "A late Searching sample did not retire the displayed frame's older verified result");
                CheckTrackingOutlineVisibility(player, false);
                player.StopStream();
                CheckTrackingOutlineVisibility(player, false);
            }
        }
    }

    private static void CheckTrackingOutlineVisibility(EmbeddedVideoPlayer player, bool expected)
    {
        foreach (var size in new[] { new Size(100, 80), new Size(320, 240) })
        using (var box = new PictureBox { Size = size, Image = player.DisplayedVideoFrame?.Image })
        using (var bitmap = new Bitmap(size.Width, size.Height))
        using (var graphics = Graphics.FromImage(bitmap))
        {
            typeof(EmbeddedVideoPlayer).GetMethod("OnVideoPaint",
                System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic)
                .Invoke(player, new object[] { box, new PaintEventArgs(graphics, box.ClientRectangle) });
            bool yellow = false;
            for (int y = 0; y < bitmap.Height && !yellow; y++)
            for (int x = 0; x < bitmap.Width && !yellow; x++)
            {
                var color = bitmap.GetPixel(x, y);
                yellow = color.R > 200 && color.G > 200 && color.B < 80;
            }
            Check(yellow == expected, "Tracking outline visibility did not match the latest verified sample: " + size +
                "; expected=" + expected + "; displayed region=" + player.DisplayedVideoFrame?.Region?.State);
        }
    }

    private sealed class FakeVisualTracker : IVisualTracker
    {
        internal volatile bool Found = true;
        internal volatile VisualTrackingState State = VisualTrackingState.Verified;
        internal PointF? AimPoint;
        internal VisualFrameContext LastContext;
        internal bool HoldUpdates;
        internal readonly ManualResetEventSlim Entered = new ManualResetEventSlim();
        internal readonly ManualResetEventSlim Release = new ManualResetEventSlim();
        internal Bitmap InitializedImage;
        internal Color InitialColor;
        internal int Initializations, Updates, Disposals;
        private RectangleF _bounds;
        private readonly int _thread = Thread.CurrentThread.ManagedThreadId;
        public void Initialize(Bitmap frame, RectangleF bounds, CancellationToken token, VisualFrameContext context = default)
        {
            Check(Thread.CurrentThread.ManagedThreadId == _thread, "Tracker initialized outside its owner worker");
            token.ThrowIfCancellationRequested();
            Initializations++; InitializedImage = frame; InitialColor = frame.GetPixel(0, 0); _bounds = bounds;
        }
        public VisualTrackingResult Update(Bitmap frame, CancellationToken token, VisualFrameContext context = default)
        {
            Check(Thread.CurrentThread.ManagedThreadId == _thread, "Tracker updated outside its owner worker");
            Updates++; Entered.Set();
            if (HoldUpdates) { Wait(Release, "native visual update release"); }
            token.ThrowIfCancellationRequested();
            LastContext = context;
            var state = Found ? State : VisualTrackingState.Lost;
            return new VisualTrackingResult(state, state.ToString(),
                state == VisualTrackingState.Verified ? _bounds : (RectangleF?)null,
                state == VisualTrackingState.Verified ? AimPoint ?? new PointF(_bounds.X + _bounds.Width / 2,
                    _bounds.Y + _bounds.Height / 2) : (PointF?)null);
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
        bool selected = player.SelectedRegion.HasValue;
        pipeline.Frames.Add(SolidVideoImage(color));
        var deadline = Stopwatch.StartNew();
        while (deadline.Elapsed < TimeSpan.FromSeconds(5))
        {
            Tick(player);
            if (player.DisplayedVideoFrame?.Sequence > previous && (!selected || !player.SelectedRegion.HasValue ||
                GetField<long>(player, "_latestSequence") >= player.DisplayedVideoFrame.Sequence)) { return; }
            Thread.Sleep(5);
        }
        throw new InvalidOperationException("Timed out waiting for player's processed visual frame: " + player.TrackingStatus);
    }

    private static void GenericVisualWorker()
    {
        var pipeline = new FakeVideoPipeline();
        var trackers = new List<FakeVisualTracker>();
        using (var session = new VideoSession(() => pipeline, createTracker: (token, mode) =>
            { var tracker = new FakeVisualTracker(); trackers.Add(tracker); return tracker; }))
        {
            session.Start("fake"); Wait(pipeline.ReadEntered, "visual warmup");
            Check(SpinWait.SpinUntil(() => session.TrackerReady, TimeSpan.FromSeconds(5)), "Tracker warmup did not complete");
            Check(session.TrackerReady && trackers.Count == 1, "Visual runtime was not warmed before selectable frames");
            pipeline.Frames.Add(SolidVideoImage(Color.Red));
            using (var source = NextVideoFrame(session))
            {
                long first = session.RequestRegion(source, new RectangleF(10, 10, 20, 20));
                var retired = GetField<VideoFrame>(GetField<object>(session, "_pendingSelection"), "Frame");
                long selected = session.RequestRegion(source, new RectangleF(30, 20, 20, 20));
                Check(first != 0 && selected > first, "Selection did not replace its single pending request");
                bool released = false;
                try { retired.Retain(); } catch (ObjectDisposedException) { released = true; }
                Check(released && source.Image.GetPixel(0, 0).ToArgb() == Color.Red.ToArgb(),
                    "Replacing selection retained its lease or disposed the displayed source");
                pipeline.Frames.Add(SolidVideoImage(Color.Blue));
                using (var current = NextVideoFrame(session))
                {
                    var tracker = trackers[trackers.Count - 1];
                    Check(current.Region.Found && current.Region.Revision == selected && current.Region.Bounds.Value.X == 30 &&
                        tracker.InitialColor.ToArgb() == Color.Red.ToArgb() && !ReferenceEquals(tracker.InitializedImage, source.Image),
                        "Selection initialized against a newer image or published mismatching target results");
                    CheckDisposed(tracker.InitializedImage);
                    Check(source.Image.GetPixel(0, 0).ToArgb() == Color.Red.ToArgb(),
                        "Initializing selection disposed pixels still owned by the source envelope");
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
        // A tracker that cannot load reports its own cause (e.g. GPU unavailable), not a generic message.
        pipeline = new FakeVideoPipeline();
        using (var session = new VideoSession(() => pipeline, createTracker: (token, mode) =>
            throw new InvalidOperationException("GPU Object tracker could not load: No NVIDIA DirectX 12 adapter found.")))
        {
            session.Start("fake"); Wait(pipeline.ReadEntered, "failed tracker warmup");
            Check(SpinWait.SpinUntil(() => session.TrackerError != null, TimeSpan.FromSeconds(5)),
                "Failed tracker warmup did not publish an error");
            Check(!session.TrackerReady && session.TrackerError != null &&
                session.TrackerError.Contains("No NVIDIA DirectX 12 adapter found.") &&
                session.TrackerError.Contains("Video continues."),
                "A tracker load failure hid its cause from the operator: " + session.TrackerError);
        }
        CheckClean(pipeline);
        pipeline = new FakeVideoPipeline();
        FakeVisualTracker active = null;
        using (var session = new VideoSession(() => pipeline,
            createTracker: (token, mode) => active = new FakeVisualTracker { HoldUpdates = true }))
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
                AwaitCapture(() => active.Disposals == 1, "retired native visual update cleanup");
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

    private static void PendingRegionInitialization()
    {
        using var form = new Form { ShowInTaskbar = false, Location = new Point(-2000, -2000),
            StartPosition = FormStartPosition.Manual, Size = new Size(800, 600) };
        var pipeline = new FakeVideoPipeline();
        FakeVisualTracker tracker = null;
        using var player = new EmbeddedVideoPlayer(new VideoInputProfile { Name = "Slow selection", Url = "udp://@:5600" },
            true, CancellationToken.None, () => pipeline,
            createTracker: (token, mode) => tracker = new FakeVisualTracker { HoldUpdates = true });
        form.Controls.Add(player); form.Show();
        player.StartStream(); Wait(pipeline.ReadEntered, "slow selection source");
        PlayerVisionFrame(player, pipeline, Color.Blue);
        Check(player.SelectRegion(new RectangleF(10, 10, 20, 20)), "Slow selection rejected fresh pixels");
        long previous = player.DisplayedVideoFrame.Sequence;
        pipeline.Frames.Add(SolidVideoImage(Color.Blue));
        Check(SpinWait.SpinUntil(() => tracker.Updates > 0, TimeSpan.FromSeconds(5)), "Slow update did not begin");
        Wait(tracker.Entered, "slow selection initialization/update");
        try
        {
            for (int i = 0; i < 7; i++) { Thread.Sleep(90); Tick(player); }
            Check(player.SelectedRegion.HasValue && !player.RegionReady && player.SubmittedTrackingCommands == 0,
                "Pending initialization expired with the old displayed image or admitted gimbal commands");
        }
        finally { tracker.HoldUpdates = false; tracker.Release.Set(); }
        var wait = Stopwatch.StartNew();
        while (player.DisplayedVideoFrame.Sequence == previous && wait.ElapsedMilliseconds < 5000)
        { Tick(player); Thread.Sleep(5); }
        Check(player.SelectedRegion.HasValue && !player.RegionReady && !player.StartTracking(),
            "Stale first result established readiness or destroyed pending initialization");
        PlayerVisionFrame(player, pipeline, Color.Blue);
        Check(player.RegionReady && player.SelectedRegion.HasValue && player.SubmittedTrackingCommands == 0,
            "Fresh post-initialization result did not establish selection without moving the gimbal");
        for (int i = 0; i < 7; i++) { Thread.Sleep(90); Tick(player); }
        Check(player.SelectedRegion.HasValue && !player.RegionReady && !player.IsTracking,
            "Stale video must retain identity without visual/control admission");

        PlayerVisionFrame(player, pipeline, Color.Blue);
        var previousTracker = tracker;
        Check(player.SelectRegion(new RectangleF(10, 10, 20, 20)), "Timeout selection rejected fresh pixels");
        pipeline.Frames.Add(SolidVideoImage(Color.Blue));
        Check(SpinWait.SpinUntil(() => tracker != previousTracker && tracker.Updates > 0, TimeSpan.FromSeconds(5)),
            "Timeout initialization did not begin");
        try
        {
            for (int i = 0; i < 24; i++) { Thread.Sleep(90); Tick(player); }
            Check(!player.SelectedRegion.HasValue && !player.RegionReady && player.SubmittedTrackingCommands == 0,
                "Pending initialization exceeded its bounded wait or admitted commands");
        }
        finally { tracker.HoldUpdates = false; tracker.Release.Set(); }
        PlayerVisionFrame(player, pipeline, Color.Blue);
        Check(!player.SelectedRegion.HasValue && !player.RegionReady, "Late result revived an expired selection");
    }
}
