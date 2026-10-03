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
        if (args.Length == 1 && args[0] == "--child")
        {
            new ManualResetEvent(false).WaitOne();
            return 0;
        }
        try
        {
            string references = args[0];
            AppDomain.CurrentDomain.AssemblyResolve += (sender, request) =>
            {
                string name = new AssemblyName(request.Name).Name;
                string path = Path.Combine(references, name == "MissionPlanner" ? name + ".exe" : name + ".dll");
                return File.Exists(path) ? Assembly.LoadFrom(path) : null;
            };
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
            Run("external process cleanup", ExternalProcessCleanup);
            Run("external startup failure", ExternalStartupFailure);
            Run("disposed view and HUD subscriptions", DisposedSubscriptions);
            Run("background plugin cancellation", BackgroundShutdown);
            Run("already cancelled plugin token", AlreadyCancelledShutdown);
            Run("borrowed native frame lifetime", BorrowedFrameLifetime);
            Run("dashboard preview reopen and shutdown", DashboardShutdown);
            return 0;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine(ex);
            return 1;
        }
    }

    private static void Run(string name, Action test)
    {
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

    private static void CheckClean(FakeVideoPipeline fake)
    {
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
            Check(fake.Starts == 1, "Duplicate pipelines started: " + fake.Starts);
            CheckClean(fake);
        }
    }

    private static void PartialStartupFailure()
    {
        var fake = new FakeVideoPipeline { FailStartup = true };
        using (var session = new VideoSession(() => fake))
        {
            session.Start("fake");
            Wait(session.Completion, "failed startup cleanup");
            Check(session.State == VideoState.Stopped, "Failed startup retained active state");
            Check(session.Error == "injected partial startup failure", "Startup failure was not reported");
            CheckClean(fake);
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
            var stopping = Task.Run(() => { if (dispose) { session.Dispose(); } else { session.Stop(); } });
            Wait(fake.Cancelled, "startup cancellation");
            Check(!stopping.IsCompleted, "Stop did not wait for owned startup");
            Check(!session.Start("duplicate"), "Start was accepted during shutdown");
            fake.ReleaseStart.Set();
            Wait(stopping, "shutdown join");
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
            Check(session.Start("second"), "Restart was rejected");
            Wait(second.ReadEntered, "second stream");
            var current = new Bitmap(2, 2);
            session.PublishFrame(session.Generation, current);
            var stale = new Bitmap(1, 1);
            session.PublishFrame(oldGeneration, stale);
            using (var displayed = session.TakeFrame())
            {
                Check(ReferenceEquals(displayed, current), "Stale callback replaced the new frame");
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
            var stopping = Task.Run(() => session.Stop());
            Wait(fake.Cancelled, "read cancellation");
            Check(!stopping.IsCompleted, "Stop returned before read drained");
            fake.ReleaseRead.Set();
            Wait(stopping, "late read drainage");
            CheckDisposed(fake.LateFrame);
            Check(session.TakeFrame() == null, "Late result was retained after stop");
            CheckClean(fake);
        }
    }
}
