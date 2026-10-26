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
    private static void PlaybackControlsAttached()
    {
        using (var player = new EmbeddedVideoPlayer(
            new VideoInputProfile { Name = "Test input", Url = "rtsp://video.invalid/stream" }, true,
            CancellationToken.None, () => new FakeVideoPipeline()))
        {
            Check(FindVideoButton(player, "Fly with keyboard and mouse") == null &&
                typeof(EmbeddedVideoPlayer).GetField("_flyButton", BindingFlags.Instance | BindingFlags.NonPublic) == null,
                "Removed flight control is present in the player");
            foreach (var label in new[] { "Play", "Stop", "Full screen", "Take snapshot" })
            {
                var button = FindVideoButton(player, label);
                Check(button != null && button.Parent != null && button.Visible,
                    $"Playback control {label} is not visible in the player");
                Check(!string.IsNullOrWhiteSpace(GetField<ToolTip>(player, "_videoToolTips").GetToolTip(button)),
                    $"Playback control {label} has no explanatory tooltip");
            }
            player.Size = new Size(100, 450);
            player.PerformLayout();
            var toolbar = player.Controls.Find("VideoToolbar", true)[0];
            Check(toolbar.Height > 45 && toolbar.Height < player.Height,
                "Narrow video toolbar failed to wrap its controls into available width");
        }
    }

    private static void VideoToggleButtonSizing()
    {
        using (var player = new EmbeddedVideoPlayer(new VideoInputProfile { Name = "Sizing", Url = "udp://@:5600" },
            true, CancellationToken.None, () => new FakeVideoPipeline()))
        using (var enlarged = new Font("Segoe UI", 18))
        {
            player.Size = new Size(900, 600);
            var button = GetField<Button>(player, "_detectButton");
            button.Font = enlarged;
            button.Text = "Detect on";
            button.Parent.PerformLayout(); player.PerformLayout();
            Size preferred = button.GetPreferredSize(Size.Empty);
            Size text = TextRenderer.MeasureText(button.Text, enlarged, Size.Empty, TextFormatFlags.SingleLine);
            Check(button.Width >= preferred.Width && button.Height >= preferred.Height &&
                button.ClientSize.Width >= text.Width && button.ClientSize.Height >= text.Height,
                "Detect on clipped or wrapped at enlarged font size");
            var tracking = GetField<Button>(player, "_trackButton");
            tracking.Font = enlarged;
            var fixedSize = tracking.Size;
            foreach (string caption in new[] { "Start tracking", "Stop tracking", "Start tracking" })
            {
                tracking.Text = caption; tracking.Parent.PerformLayout();
                text = TextRenderer.MeasureText(caption, enlarged);
                Check(tracking.Size == fixedSize && !tracking.AutoSize && tracking.ClientSize.Width >= text.Width &&
                    tracking.ClientSize.Height >= text.Height, "Tracking toggle resized or clipped when its state changed");
            }
        }
    }

    private static void DisabledButtonsKeepContrast()
    {
        int clicks = 0;
        foreach (var alignment in new[] { ContentAlignment.MiddleLeft, ContentAlignment.MiddleCenter,
            ContentAlignment.MiddleRight })
        using (var button = ControlFactory.Button("Track", (sender, args) => clicks++, width: 180, height: 32))
        using (var image = new Bitmap(180, 32))
        {
            button.TextAlign = alignment; button.Enabled = false; button.CreateControl();
            button.DrawToBitmap(image, new Rectangle(0, 0, 180, 32));
            int count = 0, totalX = 0;
            for (int y = 0; y < image.Height; y++)
            for (int x = 0; x < image.Width; x++)
            {
                var color = image.GetPixel(x, y);
                if (color.R >= 140 && color.G >= 140 && color.B >= 140)
                { count++; totalX += x; }
            }
            Check(count > 10, "Disabled button text disappeared into the dark theme");
            double center = totalX / (double)count;
            Check(alignment == ContentAlignment.MiddleLeft ? center < 60 :
                alignment == ContentAlignment.MiddleRight ? center > 120 : center > 65 && center < 115,
                "Disabled themed button ignored text alignment");
            button.PerformClick();
        }
        Check(clicks == 0, "Readable disabled button became actionable");
    }

    private static Button FindVideoButton(Control parent, string label)
    {
        foreach (Control child in parent.Controls)
        {
            if (child is Button button && button.AccessibleName == label)
            {
                return button;
            }
            var match = FindVideoButton(child, label);
            if (match != null)
            {
                return match;
            }
        }
        return null;
    }

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
            var config = VideoConfig();
            config.VideoInputs.Add(new VideoInputProfile { Name = "Replacement", Url = "rtsp://replacement.invalid/live" });
            var oldView = new NOMADVideoView(config, shutdown.Token, () => first);
            var oldPlayer = GetField<VideoInputPlayer>(oldView, "_videoHost").Player;
            oldPlayer.StartStream();
            Wait(first.ReadEntered, "first view stream");
            var oldSession = GetField<VideoSession>(oldPlayer, "_session");
            long oldGeneration = oldSession.Generation;
            oldView.Dispose();
            CheckClean(first);
            Check(!GetField<System.Windows.Forms.Timer>(oldPlayer, "_frameTimer").Enabled, "Old timer remains enabled");
            var second = new FakeVideoPipeline();
            config.SelectedVideoInputName = "Replacement";
            using (var view = new NOMADVideoView(config, shutdown.Token, () => second))
            {
                var player = GetField<VideoInputPlayer>(view, "_videoHost").Player;
                player.StartStream();
                Wait(second.ReadEntered, "reopened stream");
                Check(second.Pipeline.Contains("replacement.invalid/live"), "Reopened video retained the previous selected source.");
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
        using (var player = new EmbeddedVideoPlayer(
            new VideoInputProfile { Name = "Test input", Url = "rtsp://video.invalid/stream" }, false, CancellationToken.None, () => fake))
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
        var view = new NOMADVideoView(VideoConfig(), token, () => new FakeVideoPipeline());
        var weak = new WeakReference(GetField<VideoInputPlayer>(view, "_videoHost").Player);
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
        using (var player = new EmbeddedVideoPlayer(
            new VideoInputProfile { Name = "Test input", Url = "rtsp://video.invalid/stream" }, true, shutdown.Token, () => fake))
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
        foreach (bool disposeSource in new[] { false, true })
        {
            using (var shutdown = new CancellationTokenSource())
            {
                var token = shutdown.Token;
                shutdown.Cancel();
                if (disposeSource)
                {
                    shutdown.Dispose();
                }
                CheckCancelledView(token);
            }
        }
    }

    private static void CheckCancelledView(CancellationToken token)
    {
        int creates = 0;
        using (var player = new EmbeddedVideoPlayer(
            new VideoInputProfile { Name = "Test input", Url = "rtsp://video.invalid/stream" }, true, token, () =>
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
