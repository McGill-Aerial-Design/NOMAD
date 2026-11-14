// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void VideoThreadPoolAvailability()
    {
        ThreadPool.GetMinThreads(out int minimum, out int minimumIo);
        ThreadPool.GetMaxThreads(out int maximum, out int maximumIo);
        var pipeline = new FakeVideoPipeline();
        VideoSession first = null, second = null;
        try
        {
            Check(ThreadPool.SetMinThreads(2, minimumIo) && ThreadPool.SetMaxThreads(2, maximumIo),
                "Could not establish the two-worker thread-pool fixture");
            first = new VideoSession(() => pipeline); second = new VideoSession(() => pipeline);
            first.Start("fixture", "thread-pool-fixture"); second.Start("fixture", "thread-pool-fixture");
            Check(SpinWait.SpinUntil(() => first.State == VideoState.Streaming &&
                second.State == VideoState.Streaming, TimeSpan.FromSeconds(3)),
                "Shared video startup exhausted the two-worker thread pool");
            pipeline.Frames.Add(SolidVideoImage(Color.Blue));
            using (var a = NextVideoFrame(first))
            using (var b = NextVideoFrame(second))
            { Check(a.Fresh && b.Fresh, "Shared video did not deliver fresh frames with a bounded thread pool"); }
            var work = Task.Run(() => 1);
            Check(work.Wait(1000) && work.Result == 1, "Live video prevented unrelated thread-pool work");
        }
        finally
        {
            ThreadPool.SetMaxThreads(maximum, maximumIo);
            ThreadPool.SetMinThreads(minimum, minimumIo);
            first?.Dispose(); second?.Dispose();
            if (first != null) { Wait(first.Completion, "first bounded-pool video cleanup"); }
            if (second != null) { Wait(second.Completion, "second bounded-pool video cleanup"); }
        }
        CheckClean(pipeline);
    }

    private static void MediaWorkerPoolStarvation()
    {
        ThreadPool.GetMinThreads(out int minimum, out int minimumIo);
        ThreadPool.GetMaxThreads(out int maximum, out int maximumIo);
        string directory = Path.Combine(Path.GetTempPath(), "NOMAD-media-pool-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(directory);
        MediaWorkerHost.FixtureMode = true; MediaWorkerHost.FixtureDirectory = directory;
        using (var occupied = new CountdownEvent(2))
        using (var release = new ManualResetEventSlim())
        {
            Task first = null, second = null;
            Task<MediaWorkerDetector> startup = null;
            Task<MediaWorkerDetector> handshake = null;
            try
            {
                Check(ThreadPool.SetMinThreads(2, minimumIo) && ThreadPool.SetMaxThreads(2, maximumIo),
                    "Could not establish the two-worker heartbeat fixture");
                first = Task.Run(() => { occupied.Signal(); release.Wait(); });
                second = Task.Run(() => { occupied.Signal(); release.Wait(); });
                Check(occupied.Wait(5000), "Both fixture pool workers were not occupied");
                MediaWorkerHost.RequestStart();
                startup = Task.Factory.StartNew(() => new MediaWorkerDetector("fixture:ok", CancellationToken.None),
                    CancellationToken.None, TaskCreationOptions.LongRunning, TaskScheduler.Default);
                WaitUi(() => startup.IsCompleted, "native startup without an available pool worker");
                var detector = startup.GetAwaiter().GetResult();
                long epoch = MediaWorkerHost.WorkerEpoch;
                var worker = CurrentMediaWorker();
                Thread.Sleep(2500);
                Check(!worker.Process.HasExited && MediaWorkerHost.WorkerEpoch == epoch,
                    "An exhausted parent pool killed the native worker heartbeat");
                using (var image = new Bitmap(64, 48))
                { Check(detector.Detect(image, CancellationToken.None).Count == 0,
                    "Native operations failed while the parent pool remained occupied"); }
                MediaWorkerHost.FixtureHoldResourceHello = true;
                handshake = Task.Factory.StartNew(() => new MediaWorkerDetector("fixture:ok", CancellationToken.None),
                    CancellationToken.None, TaskCreationOptions.LongRunning, TaskScheduler.Default);
                WaitUi(() => handshake.IsCompleted, "stalled resource handshake retirement without a pool worker");
                Check(handshake.IsFaulted && handshake.Exception.GetBaseException() is MediaWorkerException &&
                    MediaWorkerHost.CanRecover, "A stalled handshake did not enter the one-recovery failure path");
                MediaWorkerHost.FixtureHoldResourceHello = false;
                var recovery = Task.Factory.StartNew(() => new MediaWorkerDetector("fixture:ok", CancellationToken.None),
                    CancellationToken.None, TaskCreationOptions.LongRunning, TaskScheduler.Default);
                WaitUi(() => recovery.IsCompleted, "native handshake recovery without a pool worker");
                using (var recovered = recovery.GetAwaiter().GetResult())
                { Check(MediaWorkerHost.WorkerEpoch > epoch, "Handshake recovery reused the failed child"); }
            }
            finally
            {
                release.Set();
                MediaWorkerHost.FixtureHoldResourceHello = false;
                ThreadPool.SetMaxThreads(maximum, maximumIo); ThreadPool.SetMinThreads(minimum, minimumIo);
                if (first != null) { Wait(first, "first occupied pool worker release"); }
                if (second != null) { Wait(second, "second occupied pool worker release"); }
                if (startup != null)
                {
                    WaitUi(() => startup.IsCompleted, "starved native startup retirement");
                    if (startup.Status == TaskStatus.RanToCompletion) { startup.Result.Dispose(); }
                }
                if (handshake != null)
                {
                    WaitUi(() => handshake.IsCompleted, "stalled resource handshake cleanup");
                    if (handshake.Status == TaskStatus.RanToCompletion) { handshake.Result.Dispose(); }
                }
                try { MediaWorkerHost.Shutdown(); Wait(MediaWorkerHost.ShutdownCompletion, "pool fixture worker exit"); }
                finally { MediaWorkerHost.FixtureMode = false; MediaWorkerHost.FixtureDirectory = null; }
                MediaWorkerHost.RequestStart();
                Directory.Delete(directory, true);
            }
        }
    }

    private static MediaWorkerHost.Worker CurrentMediaWorker() => (MediaWorkerHost.Worker)
        typeof(MediaWorkerHost).GetField("_current", System.Reflection.BindingFlags.Static |
            System.Reflection.BindingFlags.NonPublic).GetValue(null);

    private static void MediaWorkerShutdownHandles()
    {
        string directory = Path.Combine(Path.GetTempPath(), "NOMAD-media-handles-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(directory);
        MediaWorkerHost.FixtureMode = true; MediaWorkerHost.FixtureDirectory = directory;
        MediaWorkerHost.Worker worker = null;
        try
        {
            MediaWorkerHost.RequestStart();
            using (var detector = new MediaWorkerDetector("fixture:ok", CancellationToken.None)) { }
            worker = CurrentMediaWorker();
            var handle = worker.Process.SafeHandle;
            using (var observer = Process.GetProcessById(worker.Id))
            {
                MediaWorkerHost.Shutdown();
                Wait(MediaWorkerHost.ShutdownCompletion, "shutdown with no subsequent Open");
                Check(observer.WaitForExit(2000), "Shutdown completed before the owned child exited");
                Check(handle.IsClosed && MediaWorkerHost.WorkerEpoch == 0,
                    "Shutdown retained the retired process handle or current-worker reference");
            }
        }
        finally
        {
            try { MediaWorkerHost.Shutdown(); Wait(MediaWorkerHost.ShutdownCompletion, "handle fixture worker exit"); }
            finally
            {
                worker?.Dispose();
                MediaWorkerHost.FixtureMode = false; MediaWorkerHost.FixtureDirectory = null;
            }
            MediaWorkerHost.RequestStart();
            Directory.Delete(directory, true);
        }
    }

    private static void MediaWorkerIsolation()
    {
        VideoTrackingSample lastTracking = null;
        ThreadPool.GetMinThreads(out int minimum, out int minimumIo);
        ThreadPool.GetMaxThreads(out int maximum, out int maximumIo);
        string directory = Path.Combine(Path.GetTempPath(), "NOMAD-media-fixture-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(directory);
        var modules = Process.GetCurrentProcess().Modules.Cast<ProcessModule>()
            .Select(module => module.ModuleName).ToArray();
        MediaWorkerHost.FixtureMode = true; MediaWorkerHost.FixtureDirectory = directory;
        try
        {
            Check(ThreadPool.SetMinThreads(2, minimumIo) && ThreadPool.SetMaxThreads(2, maximumIo),
                "Could not establish the two-worker native media fixture");
            MediaWorkerHost.RequestStart();
            foreach (var prerequisite in new[]
            {
                ("fixture:no-nvidia", MediaWire.NvidiaUnavailable),
                ("fixture:vc-runtime", MediaWire.VisualCppRequired),
                ("fixture:private-error", MediaWire.OperationUnavailable),
            })
            {
                string diagnostic = null;
                try { using (var unavailable = new MediaWorkerDetector(prerequisite.Item1, CancellationToken.None)) { } }
                catch (InvalidOperationException error) { diagnostic = error.Message; }
                Check(diagnostic == prerequisite.Item2,
                    "Real media worker lost a safe prerequisite or exposed a private error: " + diagnostic);
            }
            bool rejected = false;
            try { MediaWire.PixelBytes(16385, 1); } catch (InvalidDataException) { rejected = true; }
            Check(rejected, "Transport accepted an oversized frame");
            using (var session = new VideoSession(MediaWorkerPipeline.Create,
                createTracker: (token, mode) => VisualTrackerFactory.Create(token, mode)))
            using (var second = new VideoSession(MediaWorkerPipeline.Create))
            {
                session.Start("fixture", "owned-media-fixture");
                second.Start("fixture", "owned-media-fixture");
                WaitMedia(() => session.TrackerReady && session.State == VideoState.Streaming &&
                    second.State == VideoState.Streaming, "shared worker startup", 120000);
                long original = MediaWorkerHost.WorkerEpoch;
                using (var frame = NextVideoFrame(session))
                { Check(session.RequestRegion(frame, new RectangleF(8, 8, 16, 16)) != 0, "Worker selection refused"); }
                WaitMedia(() =>
                {
                    var sample = session.TakeTrackingSample();
                    if (sample != null) { lastTracking = sample; }
                    return sample?.Region?.Found == true;
                }, "independent native tracking sample");
                // Throughput and fault recovery retain their normal-pool budget after constrained startup is proven.
                ThreadPool.SetMaxThreads(maximum, maximumIo);
                ThreadPool.SetMinThreads(minimum, minimumIo);
                var stats = MediaWorkerHost.FixtureStats();
                Check(stats.Width == 1, "Shared viewers opened more than one child capture");
                session.ConfigureDetection(true, "fixture:hang");
                var elapsed = Stopwatch.StartNew();
                long last = elapsed.ElapsedMilliseconds, maximumGap = 0;
                int frames = 0, uiTicks = 0, trackingSamples = 0;
                using (var timer = new System.Windows.Forms.Timer { Interval = 15 })
                {
                    timer.Tick += (sender, args) => uiTicks++;
                    timer.Start();
                    while (elapsed.ElapsedMilliseconds < 1400)
                    {
                        Application.DoEvents();
                        if (session.TakeTrackingSample()?.Region?.Found == true) { trackingSamples++; }
                        using (var frame = session.TakeFrame())
                        {
                            if (frame != null)
                            { frames++; maximumGap = Math.Max(maximumGap, elapsed.ElapsedMilliseconds - last);
                              last = elapsed.ElapsedMilliseconds; }
                        }
                        Thread.Sleep(2);
                    }
                    timer.Stop();
                }
                Check(frames >= 20 && maximumGap <= 250 && uiTicks >= 20 && trackingSamples >= 10,
                    "Held inference blocked live video or UI: frames=" + frames + ", gap=" + maximumGap + ", ticks=" + uiTicks + ", tracking samples=" + trackingSamples);
                Console.WriteLine("  Held inference: frames=" + frames + ", maximum gap=" + maximumGap +
                    " ms, UI ticks=" + uiTicks + ", independent tracking samples=" + trackingSamples);
                WaitMedia(() => MediaWorkerHost.WorkerEpoch > original && session.State == VideoState.Streaming &&
                    second.State == VideoState.Streaming, "one plain-video recovery", 10000);
                Check(!session.DetectionEnabled && !session.HasRegion(2),
                    "Worker restart restored detection or selection authority");
                stats = MediaWorkerHost.FixtureStats();
                Check(stats.Width == 1 && stats.Height == 0 && stats.Mode == 0,
                    "Plain recovery reconstructed a detector/tracker or duplicate capture");
                long replacement = MediaWorkerHost.WorkerEpoch;
                MediaWorkerHost.CrashFixture();
                WaitMedia(() => session.State == VideoState.Stopped && second.State == VideoState.Stopped,
                    "second fault reaches stopped state");
                Thread.Sleep(400);
                Check(MediaWorkerHost.WorkerEpoch == replacement && !MediaWorkerHost.CanRecover,
                    "Second fault restarted the worker without operator retry");
                var stop = Stopwatch.StartNew(); session.Stop(); second.Stop(); stop.Stop();
                Check(stop.ElapsedMilliseconds < 100, "Stop waited for a media process");
                Wait(session.Completion, "faulted session cleanup"); Wait(second.Completion, "faulted shared cleanup");
            }
            MediaWorkerHost.RequestStart();
            using (var detector = new MediaWorkerDetector("fixture:ok", CancellationToken.None))
            using (var image = new Bitmap(64, 48))
            {
                for (int index = 0; index < 150; index++) { detector.Detect(image, CancellationToken.None); }
                Check(MediaWorkerHost.FixtureStats().Ticks <= 128 * 1024,
                    "Native image transfers leaked an allocation on each inference");
            }
            MediaWorkerHost.Shutdown(); MediaWorkerHost.Shutdown(); MediaWorkerHost.RequestStart();
            using (var restarted = new VideoSession(MediaWorkerPipeline.Create))
            {
                restarted.Start("fixture");
                WaitMedia(() => restarted.State == VideoState.Streaming, "start fenced behind chained unload cleanup");
                using (var frame = NextVideoFrame(restarted)) { Check(frame.Fresh, "Reload produced stale pixels"); }
                Thread.Sleep(300);
                Check(restarted.State == VideoState.Streaming, "An old shutdown killed the replacement media worker");
                restarted.Stop(); Wait(restarted.Completion, "reloaded source cleanup");
            }
            using (var detector = new MediaWorkerDetector("", CancellationToken.None))
            using (var image = new Bitmap(Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory,
                "..", "..", "vision", "vehicle.png"))))
            { Check(detector.Detect(image, CancellationToken.None).Count > 0,
                "Embedded worker CPU model/private dependencies did not detect the vehicle fixture"); }
            var workerAssembly = System.Reflection.Assembly.ReflectionOnlyLoadFrom(
                Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "..", "..", "..", "media_worker",
                    "bin", "Release", "NOMADMediaWorker.exe")));
            Check(workerAssembly.GetReferencedAssemblies().All(reference =>
                !reference.Name.StartsWith("MissionPlanner", StringComparison.OrdinalIgnoreCase) &&
                reference.Name != "MAVLink" && reference.Name != "NOMADPlugin"),
                "Media worker acquired a Mission Planner/aircraft dependency");
            Check(Process.GetCurrentProcess().Modules.Cast<ProcessModule>().All(module =>
                modules.Contains(module.ModuleName) || (!module.ModuleName.StartsWith("onnxruntime", StringComparison.OrdinalIgnoreCase) &&
                !module.ModuleName.StartsWith("OpenCvSharpExtern", StringComparison.OrdinalIgnoreCase) &&
                !module.ModuleName.StartsWith("libgstreamer", StringComparison.OrdinalIgnoreCase))),
                "NOMAD loaded native media libraries in the parent");
        }
        catch (Exception error)
        {
            Console.Error.WriteLine("Media fixture failure: " + error + "; last tracking=" +
                lastTracking?.Region?.Status + "; worker=" + MediaWorkerHost.DiagnosticStatus);
            throw;
        }
        finally
        {
            ThreadPool.SetMaxThreads(maximum, maximumIo);
            ThreadPool.SetMinThreads(minimum, minimumIo);
            MediaWorkerHost.Shutdown(); Wait(MediaWorkerHost.ShutdownCompletion, "owned media process exit");
            MediaWorkerHost.FixtureMode = false; MediaWorkerHost.FixtureDirectory = null;
            MediaWorkerHost.RequestStart();
            Directory.Delete(directory, true);
        }
    }


    private static void MediaWorkerHeldOperations()
    {
        string directory = Path.Combine(Path.GetTempPath(), "NOMAD-media-held-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(directory);
        MediaWorkerHost.FixtureMode = true; MediaWorkerHost.FixtureDirectory = directory;
        try
        {
            foreach (string stage in new[] { "model load", "tracker load", "initialize", "track", "start", "read", "dispose" })
            {
                Func<CancellationToken, VisualTrackingMode, IVisualTracker> tracker = null;
                if (stage == "tracker load" || stage == "initialize" || stage == "track")
                { tracker = (token, mode) => VisualTrackerFactory.Create(token, mode, stage == "tracker load"); }
                using (var session = new VideoSession(MediaWorkerPipeline.Create, createTracker: tracker))
                {
                    if (stage == "model load") { session.ConfigureDetection(true, "fixture:hang-load"); }
                    string pipeline = stage == "start" ? "fixture:hang-start" :
                        stage == "read" ? "fixture:hang-read" : stage == "dispose" ? "fixture:hang-dispose" : "fixture";
                    long priorWorker = MediaWorkerHost.WorkerEpoch;
                    session.Start(pipeline);
                    WaitMedia(() => MediaWorkerHost.WorkerEpoch > priorWorker, "fresh held-operation worker for " + stage, 120000);
                    if (stage == "initialize" || stage == "track")
                    {
                        WaitMedia(() => session.TrackerReady, "tracker before held " + stage);
                        using (var frame = NextVideoFrame(session))
                        { Check(session.RequestRegion(frame, new RectangleF(stage == "initialize" ? 13 : 14, 8, 16, 16)) > 0,
                            "Held tracking selection was refused"); }
                    }
                    string operation = stage == "model load" ? "detector" : stage == "tracker load" ? "tracker" : stage;
                    if (stage != "dispose")
                    { WaitMedia(() => (MediaWorkerHost.FixtureStats().Text ?? "").Split(',').Contains("held " + operation),
                        "native entry for held " + stage); }
                    if (stage == "dispose") { WaitMedia(() => session.State == VideoState.Streaming, "capture before disposal"); }
                    var timer = Stopwatch.StartNew();
                    session.Stop();
                    Check(timer.ElapsedMilliseconds < 100, "Stop waited for held " + stage);
                    if (stage == "dispose")
                    { WaitMedia(() => (MediaWorkerHost.FixtureStats().Text ?? "").Split(',').Contains("held dispose"),
                        "held capture disposal"); }
                    Check(!session.Completion.IsCompleted, "Held " + stage + " falsely reported completed cleanup");
                    Wait(session.Completion, "held " + stage + " process cleanup");
                    Check(session.State == VideoState.Stopped, "Held " + stage + " did not finish retirement");
                }
                Console.WriteLine("  PASS: held " + stage + " retired without a UI wait");
            }
            foreach (int rate in new[] { 30, 60, 120 })
            {
                using (var session = new VideoSession(MediaWorkerPipeline.Create))
                {
                    session.ConfigureDetection(true, "fixture:hang");
                    session.Start(rate == 30 ? "fixture" : "fixture:" + rate);
                    WaitMedia(() => session.State == VideoState.Streaming, "alignment source at " + rate);
                    var clock = Stopwatch.StartNew();
                    long previous = 0, last = 0, maximumGap = 0;
                    int frames = 0;
                    while (clock.ElapsedMilliseconds < 650)
                    {
                        using (var frame = session.TakeFrame())
                        {
                            if (frame != null)
                            {
                                Check(frame.Sequence > previous && frame.Detections.Count == 0,
                                    "Late analysis rewound or annotated an already emitted frame");
                                previous = frame.Sequence; frames++;
                                maximumGap = Math.Max(maximumGap, clock.ElapsedMilliseconds - last);
                                last = clock.ElapsedMilliseconds;
                            }
                        }
                        Application.DoEvents(); Thread.Sleep(2);
                    }
                    Check(frames >= 8 && maximumGap <= 250,
                        "Alignment window starved plain output at " + rate + ": frames=" + frames + ", gap=" + maximumGap);
                    session.Stop(); Wait(session.Completion, "alignment source cleanup");
                    Console.WriteLine("  PASS: " + rate + " FPS input, frames=" + frames + ", maximum gap=" + maximumGap + " ms");
                }
            }
        }
        finally
        {
            MediaWorkerHost.Shutdown(); Wait(MediaWorkerHost.ShutdownCompletion, "held fixture worker exit");
            MediaWorkerHost.FixtureMode = false; MediaWorkerHost.FixtureDirectory = null; MediaWorkerHost.RequestStart();
            Directory.Delete(directory, true);
        }
    }


    private static void WaitMedia(Func<bool> ready, string diagnostic, int milliseconds = 7000)
    {
        var deadline = Stopwatch.StartNew();
        while (deadline.ElapsedMilliseconds < milliseconds)
        {
            if (ready()) { return; }
            Application.DoEvents(); Thread.Sleep(5);
        }
        throw new InvalidOperationException("Timed out waiting for " + diagnostic);
    }
}
