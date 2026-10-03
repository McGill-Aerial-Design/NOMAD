// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static T GetField<T>(object owner, string name)
    {
        return (T)owner.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic).GetValue(owner);
    }

    private static void SetField(object owner, string name, object value)
    {
        owner.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic).SetValue(owner, value);
    }

    private static void Tick(object owner)
    {
        owner.GetType().GetMethod("OnFrameTick", BindingFlags.Instance | BindingFlags.NonPublic)
            .Invoke(owner, new object[] { null, EventArgs.Empty });
    }

    private static void ViewReopenAndShutdown()
    {
        using (var shutdown = new CancellationTokenSource())
        {
            var first = new FakeVideoPipeline();
            var oldView = new NOMADVideoView(new NOMADConfig(), shutdown.Token, () => first);
            var oldPlayer = GetField<EmbeddedVideoPlayer>(oldView, "_videoPlayer");
            oldPlayer.StartStream();
            Wait(first.ReadEntered, "first view stream");
            var oldSession = GetField<VideoSession>(oldPlayer, "_session");
            long oldGeneration = oldSession.Generation;
            oldView.Dispose();
            CheckClean(first);
            Check(!GetField<System.Windows.Forms.Timer>(oldPlayer, "_frameTimer").Enabled, "Old timer remains enabled");
            var second = new FakeVideoPipeline();
            using (var view = new NOMADVideoView(new NOMADConfig(), shutdown.Token, () => second))
            {
                var player = GetField<EmbeddedVideoPlayer>(view, "_videoPlayer");
                player.StartStream();
                Wait(second.ReadEntered, "reopened stream");
                var stale = new Bitmap(1, 1);
                oldSession.PublishFrame(oldGeneration, stale);
                CheckDisposed(stale);
                shutdown.Cancel();
                CheckClean(first);
                CheckClean(second);
                Check(!GetField<System.Windows.Forms.Timer>(player, "_frameTimer").Enabled, "Shutdown retained timer");
                player.StartStream();
                Check(second.Starts == 1, "Shutdown view restarted");
            }
        }
    }

    private static void HandleRecreation()
    {
        var fake = new FakeVideoPipeline();
        using (var player = new EmbeddedVideoPlayer("fake", "fake", false, CancellationToken.None, () => fake))
        {
            var handle = player.Handle;
            Wait(fake.ReadEntered, "automatic start");
            player.GetType().GetMethod("RecreateHandle", BindingFlags.Instance | BindingFlags.NonPublic)
                .Invoke(player, null);
            Check(fake.Starts == 1, "Handle recreation started a duplicate pipeline");
            var session = GetField<VideoSession>(player, "_session");
            var frame = new Bitmap(2, 2);
            session.PublishFrame(session.Generation, frame);
            Tick(player);
            Check(ReferenceEquals(GetField<Bitmap>(player, "_displayFrame"), frame), "Frame was not displayed");
            player.Dispose();
            CheckDisposed(frame);
            CheckClean(fake);
            Check(handle != IntPtr.Zero, "Test did not create a control handle");
        }
    }

    private static void HudAndPluginShutdown()
    {
        var first = new FakeVideoPipeline();
        using (var hud = new PictureBox())
        using (var video = new HudVideoPlayer(hud, frame => hud.Image = frame, () => hud.Image, () => first))
        {
            video.Start("fake");
            video.Start("duplicate");
            Wait(first.ReadEntered, "HUD stream");
            Check(first.Starts == 1, "HUD duplicated startup");
            hud.Dispose();
            CheckClean(first);
            Check(!GetField<System.Windows.Forms.Timer>(video, "_timer").Enabled, "Disposed HUD retained timer");
        }
        CheckPluginExit();
    }

    private static void CheckPluginExit()
    {
        var fake = new FakeVideoPipeline();
        using (var shutdown = new CancellationTokenSource())
        using (var hud = new PictureBox())
        using (var video = new HudVideoPlayer(hud, frame => hud.Image = frame, () => hud.Image, () => fake))
        {
            var plugin = new NOMADPlugin();
            SetField(plugin, "_videoShutdown", shutdown);
            SetField(plugin, "_hudVideo", video);
            var menu = new ToolStripMenuItem("video");
            bool menuDisposed = false;
            menu.Disposed += (sender, e) => menuDisposed = true;
            var handler = (EventHandler)Delegate.CreateDelegate(typeof(EventHandler), plugin,
                "OnHudVideoMenuClicked");
            menu.Click += handler;
            SetField(plugin, "_hudVideoMenuItem", menu);
            video.Start("fake");
            Wait(fake.ReadEntered, "plugin HUD stream");
            var session = GetField<VideoSession>(video, "_session");
            var frame = new Bitmap(2, 2);
            session.PublishFrame(session.Generation, frame);
            Tick(video);
            Check(ReferenceEquals(hud.Image, frame), "HUD did not display owned frame");
            plugin.Exit();
            plugin.Exit();
            Check(shutdown.IsCancellationRequested, "Plugin exit did not propagate shutdown");
            CheckClean(fake);
            Check(hud.Image == null, "Plugin exit retained HUD image");
            CheckDisposed(frame);
            Check(!video.IsActive, "Plugin exit left HUD running");
            Check(menuDisposed, "Plugin exit retained its video menu subscription");
        }
    }

    private static void DisposedSubscriptions()
    {
        using (var shutdown = new CancellationTokenSource())
        using (var hud = new PictureBox())
        {
            var view = CreateDisposedView(shutdown.Token);
            var video = CreateDisposedHudVideo(hud);
            GC.Collect();
            GC.WaitForPendingFinalizers();
            GC.Collect();
            Check(!view.IsAlive, "Shutdown subscription retained disposed video view");
            Check(!video.IsAlive, "HUD subscription retained disposed video owner");
            GC.KeepAlive(hud);
            GC.KeepAlive(shutdown);
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static WeakReference CreateDisposedView(CancellationToken token)
    {
        var view = new NOMADVideoView(new NOMADConfig(), token, () => new FakeVideoPipeline());
        var weak = new WeakReference(GetField<EmbeddedVideoPlayer>(view, "_videoPlayer"));
        view.Dispose();
        return weak;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static WeakReference CreateDisposedHudVideo(PictureBox hud)
    {
        var video = new HudVideoPlayer(hud, frame => hud.Image = frame, () => hud.Image, () => new FakeVideoPipeline());
        var weak = new WeakReference(video);
        video.Dispose();
        return weak;
    }

    private static void BackgroundShutdown()
    {
        var fake = new FakeVideoPipeline();
        using (var shutdown = new CancellationTokenSource())
        using (var player = new EmbeddedVideoPlayer("fake", "fake", true, shutdown.Token, () => fake))
        {
            var handle = player.Handle;
            player.StartStream();
            Wait(fake.ReadEntered, "background shutdown stream");
            Wait(Task.Run(() => shutdown.Cancel()), "background cancellation without UI pumping");
            CheckClean(fake);
            Application.DoEvents();
            Check(player.IsDisposed, "UI did not release the shutdown view");
            Check(handle != IntPtr.Zero, "Background shutdown test had no UI handle");
        }
    }

    private static void AlreadyCancelledShutdown()
    {
        int creates = 0;
        using (var shutdown = new CancellationTokenSource())
        {
            shutdown.Cancel();
            using (var player = new EmbeddedVideoPlayer("fake", "fake", true, shutdown.Token, () =>
            {
                ++creates;
                return new FakeVideoPipeline();
            }))
            {
                player.StartStream();
                Check(creates == 0, "Cancelled plugin token allowed pipeline creation");
                Check(player.IsDisposed, "Cancelled plugin token retained view resources");
            }
        }
    }
}
