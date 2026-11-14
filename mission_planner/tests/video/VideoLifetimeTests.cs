// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    [STAThread]
    private static int Main(string[] args)
    {
        var previousContext = SynchronizationContext.Current;
        bool autoInstall = WindowsFormsSynchronizationContext.AutoInstall;
        WindowsFormsSynchronizationContext uiContext = null;
        try
        {
            WindowsFormsSynchronizationContext.AutoInstall = false;
            uiContext = new WindowsFormsSynchronizationContext();
            SynchronizationContext.SetSynchronizationContext(uiContext);
            string references = args[0];
            AppDomain.CurrentDomain.AssemblyResolve += (sender, request) =>
            {
                string name = new AssemblyName(request.Name).Name;
                string path = Path.Combine(references, name == "MissionPlanner" ? name + ".exe" : name + ".dll");
                return File.Exists(path) ? Assembly.LoadFrom(path) : null;
            };
            ConfigureVideoTestStorage();
            if (args.Length == 3 && args[1] == "--filter") { _filter = args[2]; }
            if (args.Length == 3 && args[1] == "--capture-probe")
            {
                Run("native capture pipeline stays NULL and preserves exact UTF-8 device identity", () => CaptureNativeNull(args[2]));
                return 0;
            }
            Run("shared video leaves a bounded thread pool available", VideoThreadPoolAvailability);
            Run("native worker startup and heartbeat survive an exhausted parent pool", MediaWorkerPoolStarvation);
            Run("native worker shutdown retires process handles without a subsequent open", MediaWorkerShutdownHandles);
            Run("owned media worker faults preserve parent UI and bounded plain recovery", MediaWorkerIsolation);
            Run("owned media worker held operations and high-rate display alignment", MediaWorkerHeldOperations);
            Run("native RTSP discovery preserves normal OPTIONS", () => NativeRtspDiscovery(false));
            Run("native RTSP discovery tolerates receivers that reset OPTIONS", () => NativeRtspDiscovery(true));
            Run("native media transport and opt-in isolated GPU pipeline", NativeMediaTransport);
            Run("native CPU tracking latency includes the media worker handoff", NativeCpuTrackingLatency);
            Run("local capture inputs share playback and exact device ownership", LocalCaptureInputs);
            Run("local capture profile validation and configuration round trip", LocalCaptureProfiles);
            Run("passive capture picker, missing device and draft isolation", LocalCaptureSettings);
            Run("History menu click/close/reopen retains ownership until player disposal", TrackingHistoryMenuLifetime);
            Run("one exclusive camera feeds Dashboard, Video and HUD until its last viewer closes", SharedCameraViews);
            Run("one UDP network input feeds Dashboard and Video; distinct sources stay separate", SharedNetworkViews);
            Run("shared camera clones preserve receipt clocks and bound slow viewers", SharedCameraClocksAndLatestFrame);
            Run("camera EOF, unplug and failed startup reach every viewer and permit explicit restart", SharedCameraEndAndFailure);
            Run("camera close joins before reopen, source generations fence late pixels and networks stay separate",
                SharedCameraClosingAndSourceChange);
            Run("playback controls attached", PlaybackControlsAttached);
            Run("WPF video actions preserve native playback, host DPI and fullscreen keyboard input", WpfVideoActions);
            Run("video status footer keeps playback and tracking readable at every size", VideoStatusFooterLayout);
            Run("tracking refusal stays visible after fresh video", TrackingFailureFeedback);
            Run("silent video shows stale status and releases HUD pixels until fresh recovery", StaleVideoDisplay);
            Run("snapshot files preserve captures and handle write failure", SnapshotFilesAndFailure);
            Run("video toggles fit enabled labels at enlarged fonts", VideoToggleButtonSizing);
            Run("disabled buttons remain readable and inactive in the dark theme", DisabledButtonsKeepContrast);
            Run("tracking ray geometry and calibrated image directions", TrackingRayGeometry);
            Run("servo visual feedback centers a delayed PWM plant through motion and turns", ServoTrackingResponse);
            Run("servo position feedback settles faster than the previous angular-rate integrator", ServoTrackingStepResponse);
            Run("servo feedback bounds acceleration, travel, windup and reset", ServoTrackingLimitsAndReset);
            Run("ground anchor geometry, dateline, mirrored rays and invalidation", GroundProjectionGeometry);
            Run("heading ring cardinal directions and native yaw frames", HeadingReportedFrames);
            Run("native motion history, actual delayed samples and submitted pose steps", NativeMotionHistory);
            Run("native primary mount, held target, cached HOME and reboot references", NativeMotionReferences);
            Run("delayed tracking stays centered through aircraft yaw and reversal", DelayedTrackingResponse);
            Run("busy native requests retain elapsed visual steering time", TrackingBusyVisualTime);
            Run("fresh 30 Hz tracking samples reach bounded native requests", TrackingRequestCadence);
            Run("Object orbit bounds sustained centering and delayed transients", ObjectOrbitResponse);
            Run("tracking heading wrap, busy recovery and stale/wrong/invalid attitude", TrackingHeadingFences);
            Run("processing delay pauses steering while fresh inputs preserve tracking intent", TrackingProcessingDelay);
            Run("Area memory, canonical point, paused yaw and explicit recovery fencing", AreaRuntimeRecovery);
            Run("Object memory, paused steering, Stop/manual/stale disarm and guarded return", ObjectRuntimeRecovery);
            Run("persistent wrong coarse prior preserves native Area verification and new commands", PersistentWrongPriorStillCommands);
            Run("frame-consistent estimated ground hint and heading in both viewports", GroundHintAndHeadingFrame);
            Run("start-stop and repeated stop", StartStop);
            Run("duplicate start", DuplicateStart);
            Run("partial startup failure", PartialStartupFailure);
            Run("stop during startup", () => StopDuringStartup(false));
            Run("dispose during startup", () => StopDuringStartup(true));
            Run("dispose while streaming", DisposeStreaming);
            Run("restart and stale callback", RestartAndStaleFrame);
            Run("concurrent repeated stop", ConcurrentStop);
            Run("startup factory failure", FactoryFailure);
            Run("pending frame replacement and cleanup", PendingFrameCleanup);
            Run("late read result during stop", LateReadDuringStop);
            Run("view reopen and shutdown", ViewReopenAndShutdown);
            Run("handle recreation", HandleRecreation);
            Run("HUD disposal and plugin exit", HudAndPluginShutdown);
            Run("disposed view and HUD subscriptions", DisposedSubscriptions);
            Run("background plugin cancellation", BackgroundShutdown);
            Run("already cancelled plugin token", AlreadyCancelledShutdown);
            Run("borrowed native frame lifetime", BorrowedFrameLifetime);
            Run("retained video envelopes release pixels after the last owner", RetainedFrameLifetime);
            Run("fresh receipts of frozen pixels expose capture-time observability limit", FreshReceiptsDoNotEstablishFreshCapture);
            Run("dashboard preview reopen and shutdown", DashboardShutdown);
            Run("UDP input pipeline", VideoUdpInput);
            Run("video URL port and property regressions", VideoAddressRegressions);
            Run("UDP URL short forms, hosts and boundaries", UdpVideoAddresses);
            Run("RTSP property containment and full HUD frame", RtspVideoAddresses);
            Run("invalid video URL refuses native startup", InvalidVideoDoesNotStart);
            Run("named receiver profile options reach both owners", VideoProfileOptions);
            Run("empty video profiles start no source", EmptyVideoDoesNotStart);
            Run("video drafts, rename, selection and aircraft isolation", VideoSettingsDrafts);
            Run("active input replacement, cached views and stopped state", ActiveVideoInput);
            Run("pending video restart uses the latest input and Stop retires it", PendingPlaybackRestart);
            Run("active input save ownership, rollback and HUD replacement", ActiveVideoInputSave);
            Run("definition editing preserves active selection", VideoDefinitionSelection);
            Run("vision envelope, detector toggles and inference cancellation", VisionWorkerLifetime);
            Run("detector failure preserves plain video", VisionFailureKeepsVideo);
            Run("model construction hot swap, plain playback and explicit recovery", DetectionModelSwap);
            Run("shared model catalog, editor cancellation and stable files", ModelCatalogAndEditor);
            Run("native built-in model duplication reuses verified worker assets", NativeBuiltinModelDuplication);
            Run("oversized optional model catalog preserves plain video and exact selection", OversizedCatalogKeepsPlainVideo);
            Run("observation retains selection-time model provenance after a swap", ObservationSelectionModelProvenance);
            Run("model-only selection preserves source and generic region ownership", ModelSwitchKeepsRegionAndSource);
            Run("model catalog publication follows a shared path across cameras", ModelCatalogSharedPath);
            Run("model construction cancellation joins and disposes during shutdown", ModelLoadShutdown);
            Run("bounded gimbal tracking angles", BoundedTrackingAngles);
            Run("generic visual worker selection, replacement, loss and retirement", GenericVisualWorker);
            Run("pending tracker initialization waits for a fresh result without control admission", PendingRegionInitialization);
            Run("tracking samples stay independent of newer plain display frames", TrackingSampleDisplayIndependence);
            Run("continuous payload sliders send calibrated PWM and preserve Safe", ContinuousPayloadSlider);
            Run("servo gimbal calibration and video profiles stay isolated", ServoGimbalConfiguration);
            Run("video servo gimbal tracking yields to a shared payload slider", ServoGimbalTracking);
            Run("servo tracking streams fresh native positions without ACK stalls and resets after loss", ServoGimbalStreaming);
            Run("zoom selection, manual override and observation ownership", VisionSelectionAndObservation);
            Run("natural EOF retires selection and calibration validates", VisionEndAndCalibration);
            return 0;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine(ex);
            return 1;
        }
        finally
        {
            MediaWorkerHost.Shutdown();
            Wait(MediaWorkerHost.ShutdownCompletion, "final media fixture exit");
            CleanupVideoTestStorage();
            SynchronizationContext.SetSynchronizationContext(previousContext);
            uiContext?.Dispose();
            WindowsFormsSynchronizationContext.AutoInstall = autoInstall;
        }
    }

    private static string _videoUserData;
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
    private static void ConfigureVideoTestStorage()
    {
        var directory = Path.Combine(Path.GetTempPath(), "NOMAD-Video-" + Guid.NewGuid().ToString("N"));
        _videoUserData = directory;
        Directory.CreateDirectory(directory);
        MissionPlanner.Utilities.Settings.CustomUserDataDirectory = directory;
        MissionPlanner.Utilities.Settings.AppConfigName = "NOMAD-VideoTests-" + Guid.NewGuid().ToString("N");
        MissionPlanner.Utilities.Settings.FileName = Path.Combine(directory, "host.xml");
    }

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
    private static void CleanupVideoTestStorage()
    {
        if (_videoUserData != null && Directory.Exists(_videoUserData) &&
            Directory.GetParent(_videoUserData).FullName.TrimEnd(Path.DirectorySeparatorChar) ==
                Path.GetTempPath().TrimEnd(Path.DirectorySeparatorChar) &&
            Path.GetFileName(_videoUserData).StartsWith("NOMAD-Video-") &&
            Guid.TryParseExact(Path.GetFileName(_videoUserData).Substring("NOMAD-Video-".Length), "N", out _))
        { Directory.Delete(_videoUserData, true); }
        string metadata = MissionPlanner.Utilities.Settings.GetDataDirectory().TrimEnd(Path.DirectorySeparatorChar);
        if (Path.GetFileName(metadata).StartsWith("NOMAD-VideoTests-") && Directory.Exists(metadata) &&
            Directory.GetParent(metadata).FullName == Environment.GetFolderPath(Environment.SpecialFolder.CommonApplicationData))
        { Directory.Delete(metadata, true); }
    }

    private static string _filter;

    private static void Run(string name, Action test)
    {
        if (_filter != null && name.IndexOf(_filter, StringComparison.OrdinalIgnoreCase) < 0) { return; }
        test();
        Console.WriteLine("PASS: " + name);
    }

    internal static void Check(bool condition, string message)
    {
        if (!condition)
        {
            throw new InvalidOperationException(message);
        }
    }

    internal static void Wait(ManualResetEventSlim signal, string name)
    {
        Check(signal.Wait(TimeSpan.FromSeconds(10)), "Timed out waiting for " + name);
    }

    private static void Wait(Task task, string name)
    {
        Check(task.Wait(TimeSpan.FromSeconds(10)), "Timed out waiting for " + name);
    }

    private static T WaitUi<T>(Task<T> task, string name)
    {
        WaitUi((Task)task, name);
        return task.GetAwaiter().GetResult();
    }

    private static void WaitUi(Task task, string name)
    {
        WaitUi(() => task.IsCompleted, name);
        task.GetAwaiter().GetResult();
    }

    private static void WaitUi(Func<bool> ready, string name)
    {
        var clock = System.Diagnostics.Stopwatch.StartNew();
        while (!ready() && clock.Elapsed < TimeSpan.FromSeconds(10))
        { Application.DoEvents(); Thread.Sleep(1); }
        Check(ready(), "Timed out waiting for " + name);
    }

    private static void CheckClean(FakeVideoPipeline fake)
    {
        Check(SpinWait.SpinUntil(() => fake.Resources == 0 && fake.Disposals == 1, TimeSpan.FromSeconds(10)),
            "Timed out waiting for asynchronous pipeline cleanup");
        Check(fake.Resources == 0, "Pipeline resources remain: " + fake.Resources);
        Check(fake.Disposals == 1, "Pipeline disposal count: " + fake.Disposals);
    }

    private static void StartStop()
    {
        var fake = new FakeVideoPipeline();
        using (var session = new VideoSession(() => fake))
        {
            Check(session.Start("fake"), "Initial start was rejected");
            Wait(fake.ReadEntered, "streaming");
            session.Stop();
            session.Stop();
            Wait(session.Completion, "repeated stop completion");
            Check(session.State == VideoState.Stopped, "Repeated stop did not remain stopped");
            Check(session.Completion.IsCompleted, "Stop returned with work running");
            CheckClean(fake);
        }
    }

    private static void DuplicateStart()
    {
        var fake = new FakeVideoPipeline();
        fake.ReleaseStart.Reset();
        using (var session = new VideoSession(() => fake))
        {
            session.Start("fake");
            Wait(fake.StartEntered, "startup");
            Check(!session.Start("duplicate"), "Duplicate startup was accepted");
            fake.ReleaseStart.Set();
            Wait(fake.ReadEntered, "streaming");
            Check(!session.Start("duplicate"), "Duplicate streaming start was accepted");
            session.Stop();
            Wait(session.Completion, "duplicate start cleanup");
            Check(fake.Starts == 1, "Duplicate pipelines started: " + fake.Starts);
            CheckClean(fake);
        }
    }

    private static void PartialStartupFailure()
    {
        var fake = new FakeVideoPipeline { FailStartup = true };
        fake.ReleaseStart.Reset();
        using (var session = new VideoSession(() => fake))
        {
            session.Start("fake");
            Wait(fake.StartEntered, "partial startup");
            var source = GetField<CancellationTokenSource>(session, "_cancellation");
            var handle = source.Token.WaitHandle.SafeWaitHandle;
            fake.ReleaseStart.Set();
            Wait(session.Completion, "failed startup cleanup");
            Check(session.State == VideoState.Stopped, "Failed startup retained active state");
            Check(session.Error == "injected partial startup failure", "Startup failure was not reported");
            CheckClean(fake);
            Check(handle.IsClosed, "Failed startup retained its cancellation wait handle");
            Check(GetField<CancellationTokenSource>(session, "_cancellation") == null,
                "Failed startup retained its cancellation source");
            session.Stop();
            CheckClean(fake);
        }
    }

    private static void StopDuringStartup(bool dispose)
    {
        var fake = new FakeVideoPipeline();
        fake.ReleaseStart.Reset();
        using (var session = new VideoSession(() => fake))
        {
            session.Start("fake");
            Wait(fake.StartEntered, "held startup");
            var clock = System.Diagnostics.Stopwatch.StartNew();
            if (dispose) { session.Dispose(); } else { session.Stop(); }
            Check(clock.ElapsedMilliseconds < 100, "Retirement waited for held native startup");
            Wait(fake.Cancelled, "startup cancellation");
            Check(!session.Completion.IsCompleted && fake.Disposals == 0,
                "Held startup was reported complete or disposed before its native owner returned");
            Check(!session.Start("duplicate"), "Start was accepted during shutdown");
            fake.ReleaseStart.Set();
            Wait(session.Completion, "shutdown completion");
            Check(!fake.ReadEntered.IsSet, "Cancelled startup resurrected streaming");
            CheckClean(fake);
            Check(session.State == (dispose ? VideoState.Disposed : VideoState.Stopped), "Incorrect shutdown state");
            if (dispose)
            {
                Check(!session.Start("after dispose"), "Disposed session restarted");
            }
        }
    }

    private static void DisposeStreaming()
    {
        var fake = new FakeVideoPipeline();
        var session = new VideoSession(() => fake);
        session.Start("fake");
        Wait(fake.ReadEntered, "streaming");
        session.Dispose();
        session.Dispose();
        Check(session.State == VideoState.Disposed, "Dispose was not terminal");
        Wait(session.Completion, "streaming disposal completion");
        Check(session.Completion.IsCompleted, "Dispose retained work");
        CheckClean(fake);
    }

    private static void RestartAndStaleFrame()
    {
        var first = new FakeVideoPipeline();
        var second = new FakeVideoPipeline();
        int created = 0;
        using (var session = new VideoSession(() => ++created == 1 ? first : second))
        {
            session.Start("first");
            Wait(first.ReadEntered, "first stream");
            long oldGeneration = session.Generation;
            session.Stop();
            Wait(session.Completion, "first stream retirement");
            Check(session.Start("second"), "Restart was rejected");
            Wait(second.ReadEntered, "second stream");
            var current = new Bitmap(2, 2);
            session.PublishFrame(session.Generation, current);
            var stale = new Bitmap(1, 1);
            session.PublishFrame(oldGeneration, stale);
            using (var displayed = session.TakeFrame())
            {
                Check(ReferenceEquals(displayed.Image, current), "Stale callback replaced the new frame");
            }
            CheckDisposed(stale);
            session.Stop();
            CheckClean(first);
            CheckClean(second);
        }
    }

    private static void CheckDisposed(Bitmap bitmap)
    {
        try
        {
            int width = bitmap.Width;
            throw new InvalidOperationException("Frame was not disposed; width: " + width);
        }
        catch (ArgumentException) { }
    }

    private static void ConcurrentStop()
    {
        var fake = new FakeVideoPipeline();
        using (var session = new VideoSession(() => fake))
        {
            session.Start("fake");
            Wait(fake.ReadEntered, "streaming");
            Wait(Task.WhenAll(Task.Run(() => session.Stop()), Task.Run(() => session.Stop())), "concurrent stops");
            Wait(session.Completion, "concurrent stop completion");
            CheckClean(fake);
            Check(session.State == VideoState.Stopped, "Concurrent stop left stopping state");
        }
    }

    private static void FactoryFailure()
    {
        using (var session = new VideoSession(() => { throw new InvalidOperationException("factory failure"); }))
        {
            session.Start("fake");
            Wait(session.Completion, "factory failure");
            Check(session.State == VideoState.Stopped, "Factory failure left active state");
            Check(session.Error == "factory failure", "Factory failure was not reported");
            session.Stop();
        }
    }

    private static void PendingFrameCleanup()
    {
        var fake = new FakeVideoPipeline();
        using (var session = new VideoSession(() => fake))
        {
            session.Start("fake");
            Wait(fake.ReadEntered, "streaming");
            var first = new Bitmap(1, 1);
            var second = new Bitmap(2, 2);
            session.PublishFrame(session.Generation, first);
            session.PublishFrame(session.Generation, second);
            CheckDisposed(first);
            session.Stop();
            CheckDisposed(second);
            Check(session.TakeFrame() == null, "Stop retained pending frame");
            CheckClean(fake);
        }
    }

    private static void LateReadDuringStop()
    {
        var fake = new FakeVideoPipeline { HoldRead = true, LateFrame = new Bitmap(1, 1) };
        using (var session = new VideoSession(() => fake))
        {
            session.Start("fake");
            Wait(fake.ReadEntered, "held read");
            var clock = System.Diagnostics.Stopwatch.StartNew();
            session.Stop();
            Check(clock.ElapsedMilliseconds < 100, "Stop blocked on a held native read");
            Wait(fake.Cancelled, "read cancellation");
            Check(!session.Completion.IsCompleted && fake.Disposals == 0,
                "Held read was reported complete or disposed before its native owner returned");
            var stale = new Bitmap(1, 1);
            session.PublishFrame(session.Generation - 1, stale);
            CheckDisposed(stale);
            fake.ReleaseRead.Set();
            Wait(session.Completion, "late read drainage");
            CheckDisposed(fake.LateFrame);
            Check(session.TakeFrame() == null, "Late result was retained after stop");
            CheckClean(fake);
        }
    }
}
