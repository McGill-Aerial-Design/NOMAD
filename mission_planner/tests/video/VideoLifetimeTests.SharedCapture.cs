// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Drawing;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private sealed class ExclusiveCameraPipeline : IVideoPipeline
    {
        internal readonly BlockingCollection<Tuple<Bitmap, long, DateTime, Exception>> Items =
            new BlockingCollection<Tuple<Bitmap, long, DateTime, Exception>>();
        internal readonly ManualResetEventSlim StartEntered = new ManualResetEventSlim();
        internal readonly ManualResetEventSlim ReleaseStart = new ManualResetEventSlim(true);
        internal readonly ManualResetEventSlim DisposeEntered = new ManualResetEventSlim();
        internal readonly ManualResetEventSlim ReleaseDispose = new ManualResetEventSlim(true);
        internal readonly ManualResetEventSlim ReadTaken = new ManualResetEventSlim();
        internal readonly ManualResetEventSlim ReleaseRead = new ManualResetEventSlim(true);
        internal int Starts, Disposals, Reads, OwnerThread;
        internal long StartedTicks, DisposedTicks;
        internal bool FailStart, HoldRead, ThrowOnDispose;
        internal CancellationToken SourceToken;

        public void Start(string pipeline, CancellationToken token)
        {
            Check(Interlocked.Increment(ref Starts) == 1, "Exclusive camera was opened twice");
            OwnerThread = Thread.CurrentThread.ManagedThreadId;
            SourceToken = token;
            StartedTicks = Stopwatch.GetTimestamp();
            StartEntered.Set(); Wait(ReleaseStart, "shared camera startup release");
            token.ThrowIfCancellationRequested();
            if (FailStart) { throw new InvalidOperationException("Injected camera startup failure"); }
        }

        public Bitmap ReadFrame(CancellationToken token, out long ticks, out DateTime utc)
        {
            Check(Thread.CurrentThread.ManagedThreadId == OwnerThread, "Camera read left its native owner worker");
            Interlocked.Increment(ref Reads);
            var item = Items.Take(token);
            ReadTaken.Set();
            if (HoldRead) { Wait(ReleaseRead, "held shared camera read"); }
            ticks = item.Item2; utc = item.Item3;
            if (item.Item4 != null) { throw item.Item4; }
            return item.Item1;
        }

        internal Bitmap Emit(Color color, long ticks = 0, DateTime utc = default)
        {
            var image = SolidVideoImage(color);
            Items.Add(Tuple.Create(image, ticks == 0 ? Stopwatch.GetTimestamp() : ticks,
                utc == default ? DateTime.UtcNow : utc, (Exception)null));
            return image;
        }

        public void Dispose()
        {
            Check(Thread.CurrentThread.ManagedThreadId == OwnerThread, "Camera disposed outside its native owner worker");
            DisposeEntered.Set(); Wait(ReleaseDispose, "shared camera disposal release");
            DisposedTicks = Stopwatch.GetTimestamp();
            Interlocked.Increment(ref Disposals);
            while (Items.TryTake(out var item)) { item.Item1?.Dispose(); }
            if (ThrowOnDispose) { throw new InvalidOperationException("Injected camera close failure"); }
        }
    }

    private static void AwaitCapture(Func<bool> ready, string message)
    {
        var clock = Stopwatch.StartNew();
        while (!ready() && clock.Elapsed.TotalSeconds < 5) { Application.DoEvents(); Thread.Sleep(2); }
        Check(ready(), "Timed out: " + message);
    }

    private static bool CameraColor(Image image, Color color) => image is Bitmap bitmap &&
        bitmap.GetPixel(0, 0).ToArgb() == color.ToArgb();

    private static int CaptureSubscribers(string key)
    {
        var flags = BindingFlags.Static | BindingFlags.NonPublic;
        var type = typeof(SharedVideoCapture);
        var gate = type.GetField("Gate", flags).GetValue(null);
        lock (gate)
        {
            var captures = (IDictionary)type.GetField("Captures", flags).GetValue(null);
            if (!captures.Contains(key)) { return 0; }
            var capture = captures[key];
            return ((ICollection)capture.GetType().GetField("Viewers",
                BindingFlags.Instance | BindingFlags.NonPublic).GetValue(capture)).Count;
        }
    }

    private static void SharedCameraViews()
    {
        const string key = @"\\?\usb#shared-three-view-camera#video";
        var profile = CaptureInput(key);
        var native = new ExclusiveCameraPipeline();
        int factories = 0;
        Func<IVideoPipeline> factory = () => { Interlocked.Increment(ref factories); return native; };
        using (var dashboard = new EmbeddedVideoPlayer(profile, false, CancellationToken.None, factory))
        using (var video = new EmbeddedVideoPlayer(profile, true, CancellationToken.None, factory))
        using (var hud = new PictureBox())
        using (var overlay = new HudVideoPlayer(hud, frame => hud.Image = frame, () => hud.Image, factory))
        {
            var handle = dashboard.Handle;
            video.StartStream(); overlay.Start(VideoPipeline.Build(profile), VideoPipeline.ShareKey(profile));
            AwaitCapture(() => CaptureSubscribers(VideoPipeline.ShareKey(profile)) == 3 &&
                GetField<VideoSession>(dashboard, "_session").State == VideoState.Streaming &&
                GetField<VideoSession>(video, "_session").State == VideoState.Streaming &&
                GetField<VideoSession>(overlay, "_session").State == VideoState.Streaming, "three camera viewers");
            var original = native.Emit(Color.Red);
            AwaitCapture(() =>
            {
                Tick(dashboard); Tick(video); Tick(overlay);
                return dashboard.DisplayedVideoFrame != null && video.DisplayedVideoFrame != null && hud.Image != null;
            }, "all camera surfaces display one capture");
            Check(factories == 1 && native.Starts == 1 && handle != IntPtr.Zero,
                "Dashboard, Video and HUD opened separate capture pipelines");
            var a = dashboard.DisplayedVideoFrame.Image; var b = video.DisplayedVideoFrame.Image;
            var c = (Bitmap)hud.Image;
            Check(!ReferenceEquals(a, b) && !ReferenceEquals(b, c) && !ReferenceEquals(a, c) &&
                CameraColor(a, Color.Red) && CameraColor(b, Color.Red) && CameraColor(c, Color.Red),
                "Viewers borrowed one bitmap or did not receive the same camera pixels");
            dashboard.StopStream(); CheckDisposed(a);
            native.Emit(Color.Blue);
            AwaitCapture(() =>
            {
                Tick(video); Tick(overlay);
                return CameraColor(video.DisplayedVideoFrame.Image, Color.Blue) && CameraColor(hud.Image, Color.Blue);
            }, "remaining camera viewers continue");
            Check(native.Disposals == 0, "Stopping one viewer closed the shared device");
            video.StopStream(); native.Emit(Color.Lime);
            AwaitCapture(() => { Tick(overlay); return CameraColor(hud.Image, Color.Lime); },
                "HUD keeps camera after other views stop");
            Check(native.Disposals == 0, "Stopping Video closed a camera still used by HUD");
            overlay.Dispose();
            AwaitCapture(() => native.Disposals == 1 && CaptureSubscribers(VideoPipeline.ShareKey(profile)) == 0,
                "last camera viewer native cleanup");
            Check(native.Disposals == 1 && CaptureSubscribers(VideoPipeline.ShareKey(profile)) == 0,
                "Last camera viewer retained the native capture");
            CheckDisposed(original);
        }
    }

    // A UDP port delivers each datagram to one socket, so a second view opening its own receiver showed nothing.
    private static void SharedNetworkViews()
    {
        var profile = new VideoInputProfile { Name = "Simulator", Url = "udp://127.0.0.1:64552" };
        string key = VideoPipeline.ShareKey(profile);
        var native = new ExclusiveCameraPipeline();
        int factories = 0;
        Func<IVideoPipeline> factory = () => { Interlocked.Increment(ref factories); return native; };
        using (var dashboard = new EmbeddedVideoPlayer(profile, false, CancellationToken.None, factory))
        using (var video = new EmbeddedVideoPlayer(profile, true, CancellationToken.None, factory))
        {
            var handle = dashboard.Handle;
            video.StartStream();
            AwaitCapture(() => CaptureSubscribers(key) == 2 &&
                GetField<VideoSession>(dashboard, "_session").State == VideoState.Streaming &&
                GetField<VideoSession>(video, "_session").State == VideoState.Streaming, "two UDP viewers");
            native.Emit(Color.Red);
            AwaitCapture(() =>
            {
                Tick(dashboard); Tick(video);
                return dashboard.DisplayedVideoFrame != null && video.DisplayedVideoFrame != null;
            }, "Dashboard and Video both display the UDP input");
            Check(factories == 1 && native.Starts == 1 && handle != IntPtr.Zero,
                "Dashboard and Video opened separate receivers on one UDP port");
            dashboard.StopStream(); native.Emit(Color.Blue);
            AwaitCapture(() => { Tick(video); return CameraColor(video.DisplayedVideoFrame.Image, Color.Blue); },
                "Video keeps the UDP input after Dashboard stops");
            Check(native.Disposals == 0, "Stopping one view closed a UDP receiver still in use");
        }
        AwaitCapture(() => native.Disposals == 1 && CaptureSubscribers(key) == 0, "last UDP viewer cleanup");
        Check(native.Disposals == 1 && CaptureSubscribers(key) == 0, "The last UDP viewer retained the receiver");
        Check(key != VideoPipeline.ShareKey(new VideoInputProfile { Name = "Other", Url = "udp://127.0.0.1:64553" }) &&
            VideoPipeline.ShareKey(new VideoInputProfile { Name = "A", Url = "rtsp://cam.invalid/Live" }) !=
            VideoPipeline.ShareKey(new VideoInputProfile { Name = "B", Url = "rtsp://cam.invalid/live" }),
            "Different UDP ports or case-distinct RTSP paths would share one receiver");
    }

    private static void SharedCameraClocksAndLatestFrame()
    {
        const string key = @"\\?\usb#shared-receipt-clock#video";
        var native = new ExclusiveCameraPipeline();
        using (var fast = SharedVideoCapture.Create(key, () => native))
        using (var slow = SharedVideoCapture.Create(key, () => native))
        {
            fast.Start("owned clock capture", CancellationToken.None);
            slow.Start("owned clock capture", CancellationToken.None);
            long old = Stopwatch.GetTimestamp() - Stopwatch.Frequency * 2;
            DateTime oldUtc = DateTime.UtcNow.AddSeconds(-2);
            native.Emit(Color.Red, old, oldUtc);
            using (var frame = fast.ReadFrame(CancellationToken.None, out long ticks, out DateTime utc))
            {
                Check(ticks == old && utc == oldUtc && (Stopwatch.GetTimestamp() - ticks) / (double)Stopwatch.Frequency > 1,
                    "A delayed camera reader freshened the decoder receipt clock");
            }
            long current = Stopwatch.GetTimestamp();
            native.Emit(Color.Blue, current);
            using (var blue = fast.ReadFrame(CancellationToken.None, out _, out _))
            {
                AwaitCapture(() => GetField<long>(slow, "_receivedTicks") == current, "slow viewer latest slot");
                using (var latest = slow.ReadFrame(CancellationToken.None, out long ticks, out _))
                {
                    Check(ticks == current && CameraColor(latest, Color.Blue) &&
                        CameraColor(blue, Color.Blue) && !ReferenceEquals(blue, latest),
                        "A slow viewer blocked delivery, kept a backlog or shared another viewer's bitmap");
                }
            }
        }
        Check(native.Disposals == 1, "Clock fixture retained its shared capture");
        var throughSession = new ExclusiveCameraPipeline();
        using (var session = new VideoSession(() => throughSession))
        {
            session.Start("owned stale capture", key + "-session");
            long old = Stopwatch.GetTimestamp() - Stopwatch.Frequency * 2;
            DateTime oldUtc = DateTime.UtcNow.AddSeconds(-2);
            throughSession.Emit(Color.Red, old, oldUtc);
            using (var frame = NextVideoFrame(session))
            { Check(!frame.Fresh && frame.ReceivedTicks == old && frame.ReceivedUtc == oldUtc,
                "VideoSession replaced a shared frame's original decoder receipt time"); }
        }
    }

    private static void SharedCameraEndAndFailure()
    {
        foreach (bool error in new[] { false, true })
        {
            string key = @"\\?\usb#shared-end-" + error + "#video";
            var native = new ExclusiveCameraPipeline();
            using (var first = new VideoSession(() => native))
            using (var second = new VideoSession(() => native))
            {
                first.Start("owned end capture", key); second.Start("owned end capture", key);
                AwaitCapture(() => CaptureSubscribers(key) == 2 && first.State == VideoState.Streaming &&
                    second.State == VideoState.Streaming, "camera end subscribers");
                native.Items.Add(Tuple.Create((Bitmap)null, 0L, default(DateTime), error ?
                    (Exception)new InvalidOperationException("Injected camera unplug") : null));
                Wait(first.Completion, "first camera end"); Wait(second.Completion, "second camera end");
                Check(first.State == VideoState.Stopped && second.State == VideoState.Stopped &&
                    first.Error == (error ? "Injected camera unplug" : null) &&
                    second.Error == (error ? "Injected camera unplug" : null) && native.Disposals == 1,
                    "Camera error/EOF did not fan out and close the native owner once");
            }
            var reopened = new ExclusiveCameraPipeline();
            using (var session = new VideoSession(() => reopened))
            {
                session.Start("owned end capture", key);
                AwaitCapture(() => session.State == VideoState.Streaming, "explicit camera restart after end");
            }
            AwaitCapture(() => reopened.Disposals == 1, "reopened camera asynchronous cleanup");
            Check(reopened.Starts == 1 && reopened.Disposals == 1, "Ended camera could not explicitly restart");
        }
        const string failedKey = @"\\?\usb#shared-start-failure#video";
        var failed = new ExclusiveCameraPipeline { FailStart = true }; failed.ReleaseStart.Reset();
        using (var first = new VideoSession(() => failed))
        using (var second = new VideoSession(() => failed))
        {
            try
            {
                first.Start("owned startup failure", failedKey); Wait(failed.StartEntered, "held camera start");
                second.Start("owned startup failure", failedKey);
                AwaitCapture(() => CaptureSubscribers(failedKey) == 2, "both camera startup waiters");
            }
            finally { failed.ReleaseStart.Set(); }
            Wait(first.Completion, "first camera startup failure"); Wait(second.Completion, "second camera startup failure");
            Check(failed.Starts == 1 && failed.Disposals == 1 && first.Error != null && second.Error != null,
                "Shared camera startup failure retried the native open or stranded a waiter");
        }
        var recovered = new ExclusiveCameraPipeline();
        using (var session = new VideoSession(() => recovered))
        {
            session.Start("owned startup failure", failedKey);
            AwaitCapture(() => session.State == VideoState.Streaming, "explicit restart after failed camera startup");
        }
        AwaitCapture(() => recovered.Disposals == 1, "recovered camera asynchronous cleanup");
        Check(recovered.Starts == 1 && recovered.Disposals == 1, "Failed camera startup left an unusable pool entry");
        var closeFailure = new ExclusiveCameraPipeline { ThrowOnDispose = true };
        using (var session = new VideoSession(() => closeFailure))
        {
            session.Start("owned camera close fault", failedKey);
            AwaitCapture(() => session.State == VideoState.Streaming, "camera before close fault");
            session.Stop();
            Wait(session.Completion, "native camera close failure completion");
            Check(closeFailure.Disposals == 1 && session.Error == "Injected camera close failure" &&
                CaptureSubscribers(failedKey) == 0, "The last viewer hid a native camera close failure");
        }
    }

    private static void SharedCameraClosingAndSourceChange()
    {
        const string key = @"\\?\usb#shared-close-before-reopen#video";
        var old = new ExclusiveCameraPipeline(); old.ReleaseDispose.Reset();
        var replacement = new ExclusiveCameraPipeline();
        using (var first = new VideoSession(() => old))
        using (var next = new VideoSession(() => replacement))
        {
            first.Start("owned camera source", key);
            AwaitCapture(() => first.State == VideoState.Streaming, "camera before held close");
            var stopped = Task.Run(() => first.Stop());
            Wait(stopped, "nonblocking camera retirement");
            try
            {
                Wait(old.DisposeEntered, "native camera close entered");
                next.Start("owned camera source", key);
                Check(!replacement.StartEntered.Wait(100), "A second open overlapped native camera disposal");
            }
            finally { old.ReleaseDispose.Set(); }
            Wait(first.Completion, "camera close completion");
            AwaitCapture(() => next.State == VideoState.Streaming, "camera reopen");
            Check(old.Disposals == 1 && replacement.Starts == 1 && replacement.StartedTicks >= old.DisposedTicks,
                "Camera restart lost exclusive ownership or overlapped native disposal");
        }
        var sourceA = new ExclusiveCameraPipeline { HoldRead = true }; sourceA.ReleaseRead.Reset();
        var sourceB = new ExclusiveCameraPipeline();
        using (var a = new VideoSession(() => sourceA))
        using (var switched = new VideoSession(() => sourceB))
        {
            const string aKey = @"\\?\usb#shared-late-a#video", bKey = @"\\?\usb#shared-late-b#video";
            try
            {
                a.Start("owned source A", aKey); sourceA.Emit(Color.Red);
                Wait(sourceA.ReadTaken, "held camera A read");
                switched.Start("owned source A", aKey);
                AwaitCapture(() => CaptureSubscribers(aKey) == 2, "source A shared before change");
                long retired = switched.Generation;
                switched.Stop();
                Wait(switched.Completion, "old shared viewer retirement");
                Check(!sourceA.SourceToken.IsCancellationRequested, "Changing one view canceled another view's camera A");
                switched.Start("owned source B", bKey);
                AwaitCapture(() => switched.State == VideoState.Streaming, "replacement camera B");
                sourceA.ReleaseRead.Set(); sourceB.Emit(Color.Blue);
                using (var frame = NextVideoFrame(switched))
                { Check(CameraColor(frame.Image, Color.Blue) && frame.Generation != retired,
                    "A late camera A image reached the replacement camera B generation"); }
                using (var frame = NextVideoFrame(a))
                { Check(CameraColor(frame.Image, Color.Red), "Changing another view discarded camera A's own late frame"); }
            }
            finally { sourceA.ReleaseRead.Set(); }
        }
        AwaitCapture(() => sourceA.Disposals == 1 && sourceB.Disposals == 1, "source handoff cleanup");
        Check(sourceA.Disposals == 1 && sourceB.Disposals == 1, "A source handoff retained a camera owner");
        var networkA = new FakeVideoPipeline(); var networkB = new FakeVideoPipeline();
        using (var a = new VideoSession(() => networkA))
        using (var b = new VideoSession(() => networkB))
        {
            a.Start("same network pipeline"); b.Start("same network pipeline");
            Wait(networkA.ReadEntered, "first independent network reader");
            Wait(networkB.ReadEntered, "second independent network reader");
            Check(networkA.Starts == 1 && networkB.Starts == 1, "Camera sharing changed network pipeline ownership");
        }
    }
}
