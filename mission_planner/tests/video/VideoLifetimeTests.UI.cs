// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

using NOMAD.MissionPlanner;
using Label = System.Windows.Controls.TextBlock;
using Button = System.Windows.Controls.Button;
using Brushes = System.Windows.Media.Brushes;
using ElementHost = System.Windows.Forms.Integration.ElementHost;

internal static partial class VideoLifetimeTests
{
    [System.Runtime.InteropServices.DllImport("user32.dll")]
    private static extern IntPtr GetThreadDpiAwarenessContext();
    [System.Runtime.InteropServices.DllImport("user32.dll")]
    private static extern bool AreDpiAwarenessContextsEqual(IntPtr first, IntPtr second);

    private static void WpfVideoActions()
    {
        IntPtr hostDpi = GetThreadDpiAwarenessContext();
        var pipeline = new FakeVideoPipeline();
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), ClientSize = new Size(900, 600) })
        using (var player = new EmbeddedVideoPlayer(new VideoInputProfile { Name = "WPF actions", Url = "udp://@:5600" },
            true, CancellationToken.None, () => pipeline, createTracker: (token, mode) => new FakeVisualTracker()))
        {
            form.Controls.Add(player); form.Show(); Application.DoEvents();
            Check(AreDpiAwarenessContextsEqual(hostDpi, GetThreadDpiAwarenessContext()),
                "Loading the WPF player changed the Mission Planner host's DPI mode");
            var play = FindVideoButton(player, "Play");
            Check(play.Focusable && System.Windows.Input.KeyboardNavigation.GetIsTabStop(play),
                "The WPF playback action is absent from keyboard navigation");
            ClickVideoButton(play);
            Wait(pipeline.ReadEntered, "WPF Play reaches the existing video owner");
            pipeline.Frames.Add(new Bitmap(12, 8));
            WaitUi(() => { Tick(player); return player.DisplayedVideoFrame?.Image.Width == 12; }, "WPF playback frame");
            Check(pipeline.Starts == 1 && player.SubmittedTrackingCommands == 0,
                "WPF Play started duplicate capture or moved the gimbal");
            ClickVideoButton(FindVideoButton(player, "Stop"));
            WaitUi(() => { Tick(player); return player.DisplayedVideoFrame == null; }, "WPF Stop releases displayed pixels");
            CheckClean(pipeline);
            ClickVideoButton(FindVideoButton(player, "Full screen"));
            Check(player.Toolbar.FindForm() == GetField<Form>(player, "_fullscreenForm"),
                "The WPF controls remained in the hidden original viewport");
            var source = System.Windows.PresentationSource.FromVisual(player.VideoControls);
            player.VideoControls.RaiseEvent(new System.Windows.Input.KeyEventArgs(
                System.Windows.Input.Keyboard.PrimaryDevice, source, Environment.TickCount, System.Windows.Input.Key.Escape)
                { RoutedEvent = System.Windows.Input.Keyboard.PreviewKeyDownEvent });
            Check(GetField<Form>(player, "_fullscreenForm") == null && player.Toolbar.Parent == player,
                "Escape from a WPF control did not restore the embedded video page");
        }
    }

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
                Check(button != null && button.Parent != null && button.Visibility == System.Windows.Visibility.Visible,
                    $"Playback control {label} is not visible in the player");
                Check(!string.IsNullOrWhiteSpace(button.ToolTip as string),
                    $"Playback control {label} has no explanatory tooltip");
            }
            Check(player.VisionOptions.Visibility == System.Windows.Visibility.Collapsed, "Occasional video tools clutter the default toolbar");
            var more = FindVideoButton(player, "Video options");
            ClickVideoButton(more);
            Check(player.VisionOptions.Visibility == System.Windows.Visibility.Visible, "More did not reveal the video options");
            ClickVideoButton(more);
            Check(player.VisionOptions.Visibility == System.Windows.Visibility.Collapsed, "More did not collapse the video options");
            var footer = GetField<ElementHost>(player, "_statusPanel");
            for (int index = 0; index < 2; index++)
            {
                player.ToggleFullscreen();
                var fullscreen = GetField<Form>(player, "_fullscreenForm");
                Check(player.Toolbar.Parent == fullscreen && footer.Parent == fullscreen &&
                    System.Windows.PresentationSource.FromVisual(GetField<Label>(player, "_lblStatus")) != null &&
                    System.Windows.PresentationSource.FromVisual(GetField<Label>(player, "_lblVisionStatus")) != null &&
                    GetField<Button>(player, "_trackButton").IsVisible,
                    "Fullscreen hid playback, tracking controls or live warnings");
                if (index == 0) { player.ToggleFullscreen(); }
                else { fullscreen.Close(); }
                Check(player.Toolbar.Parent == player && !player.Toolbar.IsDisposed &&
                    footer.Parent == player && !footer.IsDisposed &&
                    GetField<Form>(player, "_fullscreenForm") == null,
                    "Leaving fullscreen lost or disposed the player controls");
            }
            player.Size = new Size(380, 600);
            player.PerformLayout();
            var toolbar = player.Controls.Find("VideoToolbar", true)[0];
            Check(toolbar.Height > 45 && toolbar.Height < player.Height,
                "Narrow video toolbar failed to wrap its controls into available width");
            player.ToggleFullscreen();
            var closing = GetField<Form>(player, "_fullscreenForm");
            var ownedToolbar = player.Toolbar;
            player.Dispose();
            Check(closing.IsDisposed && ownedToolbar.IsDisposed && footer.IsDisposed,
                "Disposing an actively fullscreen player leaked its window, toolbar or footer");
            Check(System.Windows.Application.Current == null,
                "The plugin created a WPF Application instead of using the host message loop");
        }
    }

    private static void VideoStatusFooterLayout()
    {
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), ClientSize = new Size(1600, 700) })
        using (var player = new EmbeddedVideoPlayer(new VideoInputProfile { Name = "Footer", Url = "udp://@:5600" },
            true, CancellationToken.None, () => throw new InvalidOperationException("A layout test must not start video.")))
        {
            form.Controls.Add(player); form.Show(); Application.DoEvents();
            var playback = GetField<Label>(player, "_lblStatus");
            var tracking = GetField<Label>(player, "_lblVisionStatus");
            var footer = GetField<ElementHost>(player, "_statusPanel");
            var view = (VideoStatusView)footer.Child;
            Check(playback.Parent == tracking.Parent && footer.Parent == player && footer.Dock == DockStyle.Bottom,
                "Playback and tracking do not share one bottom footer");
            Check(playback.Text == "Ready - Click Play" && tracking.Text.Contains("calibration"),
                "The idle footer lost playback readiness or tracking calibration guidance");
            double normalFont = playback.FontSize;
            string trackingGuidance = tracking.Text;
            foreach (int width in new[] { 1600, 800, 380 })
            {
                form.ClientSize = new Size(width, 700);
                foreach (bool warning in new[] { false, true })
                {
                    playback.Text = warning ? "Snapshot could not be saved; check the output folder" : "Ready - Click Play";
                    tracking.Text = warning ? MediaWire.VisualCppRequired : trackingGuidance;
                    playback.FontSize = tracking.FontSize = warning ? 24 : normalFont;
                    player.PerformLayout(); footer.PerformLayout(); Application.DoEvents();
                    view.UpdateLayout();
                    var video = GetField<PictureBox>(player, "_videoBox");
                    Check(footer.Bottom == player.ClientSize.Height && video.Bottom <= footer.Top &&
                        footer.Top >= player.Toolbar.Bottom,
                        "The status footer overlaps video or controls at width " + width);
                    var playbackBounds = playback.TransformToAncestor(view).TransformBounds(new System.Windows.Rect(playback.RenderSize));
                    var trackingBounds = tracking.TransformToAncestor(view).TransformBounds(new System.Windows.Rect(tracking.RenderSize));
                    Check(playback.Visibility == System.Windows.Visibility.Visible && tracking.Visibility == System.Windows.Visibility.Visible &&
                        !playbackBounds.IntersectsWith(trackingBounds),
                        "Status messages overlap or disappear at width " + width);
                    foreach (var label in new[] { playback, tracking })
                    {
                        var bounds = label.TransformToAncestor(view).TransformBounds(new System.Windows.Rect(label.RenderSize));
                        Check(new System.Windows.Rect(view.RenderSize).Contains(bounds) && label.ActualHeight >= label.DesiredSize.Height,
                            System.Windows.Automation.AutomationProperties.GetName(label) + " is clipped at width " + width);
                    }
                }
            }
        }
    }

    private static void VideoToggleButtonSizing()
    {
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), ClientSize = new Size(900, 600) })
        using (var player = new EmbeddedVideoPlayer(new VideoInputProfile { Name = "Sizing", Url = "udp://@:5600" },
            true, CancellationToken.None, () => new FakeVideoPipeline()))
        {
            form.Controls.Add(player); form.Show(); Application.DoEvents();
            var button = GetField<Button>(player, "_detectButton");
            button.FontSize = 24;
            button.Content = "Detect on";
            player.PerformLayout(); player.VideoControls.UpdateLayout();
            Check(button.ActualWidth >= button.DesiredSize.Width - button.Margin.Left - button.Margin.Right &&
                button.ActualHeight >= button.DesiredSize.Height - button.Margin.Top - button.Margin.Bottom,
                "Detect on clipped or wrapped at enlarged font size");
            var tracking = GetField<Button>(player, "_trackButton");
            tracking.FontSize = 24;
            player.PerformLayout(); player.VideoControls.UpdateLayout();
            var fixedSize = tracking.RenderSize;
            foreach (string caption in new[] { "Start tracking", "Stop tracking", "Start tracking" })
            {
                player.VideoControls.TrackCaption.Text = caption;
                player.VideoControls.UpdateLayout();
                Check(tracking.RenderSize == fixedSize && player.VideoControls.TrackCaption.ActualWidth <= tracking.ActualWidth &&
                    player.VideoControls.TrackCaption.ActualHeight <= tracking.ActualHeight,
                    "Tracking toggle resized or clipped when its state changed");
            }
        }
    }

    private static void TrackingFailureFeedback()
    {
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), Size = new Size(800, 600) })
        {
            var pipeline = new FakeVideoPipeline();
            using (var player = new EmbeddedVideoPlayer(new VideoInputProfile { Name = "Feedback", Url = "udp://@:5600",
                TrackingCalibrated = true }, true, CancellationToken.None, () => pipeline,
                createTracker: (token, mode) => new FakeVisualTracker()))
            {
                form.Controls.Add(player); form.Show(); player.StartStream();
                Wait(pipeline.ReadEntered, "tracking refusal video");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.SelectRegion(new RectangleF(20, 20, 20, 20)), "Feedback target selection failed");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(!player.StartTracking(), "Tracking started without aircraft telemetry");
                var status = GetField<Label>(player, "_lblVisionStatus");
                Check(status.Text.Contains("fresh aircraft attitude and heartbeat"),
                    "Tracking refusal was hidden behind a generic target status");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(status.Text.Contains("fresh aircraft attitude and heartbeat"),
                    "The next visual sample erased the actionable tracking refusal");
            }
            CheckClean(pipeline);
        }
    }

    private static void StaleVideoDisplay()
    {
        var fake = new FakeVideoPipeline();
        using (var player = new EmbeddedVideoPlayer(new VideoInputProfile { Name = "Stall", Url = "udp://@:5600" },
            true, CancellationToken.None, () => fake, createTracker: (token, mode) => new FakeVisualTracker()))
        {
            player.StartStream(); Wait(fake.ReadEntered, "stalled player startup");
            fake.Frames.Add(new Bitmap(2, 2));
            WaitUi(() => { Tick(player); return player.DisplayedVideoFrame != null; }, "initial player image");
            var image = player.DisplayedVideoFrame.Image;
            var status = GetField<Label>(player, "_lblStatus");
            Check(status.Foreground == Brushes.LimeGreen, "Fresh player frame was not shown as streaming");
            WaitUi(() => { Tick(player); return status.Text.Contains("stale"); }, "stale player warning");
            Tick(player);
            Check(status.Foreground == Brushes.Yellow && ReferenceEquals(player.DisplayedVideoFrame.Image, image),
                "Silent player stayed healthy or discarded its last frame context");
            fake.Frames.Add(new Bitmap(3, 3));
            WaitUi(() => { Tick(player); return player.DisplayedVideoFrame.Image.Width == 3; }, "fresh player recovery");
            Check(status.Foreground == Brushes.LimeGreen && fake.Starts == 1,
                "Fresh player frame did not restore streaming without a restart");
            CheckDisposed(image);
        }
        CheckClean(fake);

        fake = new FakeVideoPipeline();
        using (var hud = new PictureBox())
        using (var foreign = new Bitmap(4, 4))
        {
            int clears = 0;
            using (var video = new HudVideoPlayer(hud, frame =>
                { if (frame == null) { clears++; } hud.Image = frame; }, () => hud.Image, () => fake))
            {
                video.Start("fake"); Wait(fake.ReadEntered, "stalled HUD startup");
                fake.Frames.Add(new Bitmap(2, 2));
                WaitUi(() => { Tick(video); return hud.Image != null; }, "initial HUD image");
                var image = hud.Image;
                WaitUi(() => { Tick(video); return hud.Image == null; }, "stale HUD clearing");
                Tick(video);
                Check(clears == 1 && video.IsActive, "Silent HUD repeatedly cleared or stopped capture");
                CheckDisposed((Bitmap)image);
                fake.Frames.Add(new Bitmap(3, 3));
                WaitUi(() => { Tick(video); return hud.Image != null; }, "fresh HUD recovery");
                Check(hud.Image.Width == 3 && fake.Starts == 1, "HUD did not recover without restarting capture");
                image = hud.Image;
                hud.Image = foreign;
                WaitUi(() =>
                {
                    Tick(video);
                    return GetField<VideoFrame>(video, "_displayedVideoFrame") == null;
                }, "foreign HUD ownership after silence");
                Check(ReferenceEquals(hud.Image, foreign) && clears == 1, "Stale HUD cleared another owner's image");
                CheckDisposed((Bitmap)image);
            }
        }
        CheckClean(fake);
    }

    private static void SnapshotFilesAndFailure()
    {
        string directory = Path.Combine(Path.GetTempPath(), "NOMAD-Snapshot-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(directory);
        try
        {
            using (var player = new EmbeddedVideoPlayer(
                new VideoInputProfile { Name = "Snapshot", Url = "udp://@:5600" }, true,
                CancellationToken.None, () => throw new InvalidOperationException("Snapshot test must not start video.")))
            {
                var source = new Bitmap(2, 2);
                SetField(player, "_displayFrame", source);
                source.SetPixel(0, 0, Color.Red);
                string first = player.TakeSnapshot(directory);
                source.SetPixel(0, 0, Color.Blue);
                string second = player.TakeSnapshot(directory);
                Check(first != null && second != null && first != second && Directory.GetFiles(directory).Length == 2,
                    "Repeated snapshots reused a filename or did not create two files");
                using (var red = new Bitmap(first))
                using (var blue = new Bitmap(second))
                {
                    Check(red.RawFormat.Guid == System.Drawing.Imaging.ImageFormat.Png.Guid &&
                        blue.RawFormat.Guid == System.Drawing.Imaging.ImageFormat.Png.Guid &&
                        red.GetPixel(0, 0).ToArgb() == Color.Red.ToArgb() &&
                        blue.GetPixel(0, 0).ToArgb() == Color.Blue.ToArgb(),
                        "Snapshot files did not preserve each capture as a PNG");
                }
                bool rejected = false;
                try { EmbeddedVideoPlayer.WriteSnapshot(source, first); }
                catch (IOException) { rejected = true; }
                using (var unchanged = new Bitmap(first))
                {
                    Check(rejected && unchanged.GetPixel(0, 0).ToArgb() == Color.Red.ToArgb(),
                        "A snapshot collision overwrote or removed an existing capture");
                }
                Check(player.TakeSnapshot(first) == null && GetField<Label>(player, "_lblStatus").Foreground == Brushes.Red &&
                    GetField<Label>(player, "_lblStatus").Text.Contains("could not be saved") &&
                    source.GetPixel(0, 0).ToArgb() == Color.Blue.ToArgb() && Directory.GetFiles(directory).Length == 2,
                    "A failed snapshot escaped, claimed success, changed files or disposed the displayed source");
                string previousDirectory = Environment.CurrentDirectory;
                try
                {
                    Environment.CurrentDirectory = directory;
                    Check(player.TakeSnapshot("") == null && player.TakeSnapshot(".") == null &&
                        GetField<Label>(player, "_lblStatus").Foreground == Brushes.Red &&
                        Directory.GetFiles(directory).Length == 2,
                        "An unavailable Desktop or relative destination wrote to the host working directory");
                }
                finally { Environment.CurrentDirectory = previousDirectory; }
            }
        }
        finally { Directory.Delete(directory, true); }
    }

    private static void DisabledButtonsKeepContrast()
    {
        int clicks = 0;
        foreach (var alignment in new[] { System.Windows.HorizontalAlignment.Left,
            System.Windows.HorizontalAlignment.Center, System.Windows.HorizontalAlignment.Right })
        {
            var button = WpfUi.Action("Track", () => clicks++);
            button.Resources.MergedDictionaries.Add(WpfTheme.Resources());
            button.HorizontalContentAlignment = alignment; button.IsEnabled = false;
            button.Margin = new System.Windows.Thickness(); button.Width = 180; button.Height = 32;
            button.Measure(new System.Windows.Size(180, 32));
            button.Arrange(new System.Windows.Rect(0, 0, 180, 32)); button.UpdateLayout();
            var image = new System.Windows.Media.Imaging.RenderTargetBitmap(180, 32, 96, 96,
                System.Windows.Media.PixelFormats.Pbgra32);
            image.Render(button);
            var pixels = new byte[180 * 32 * 4]; image.CopyPixels(pixels, 180 * 4, 0);
            int count = 0, totalX = 0;
            for (int y = 0; y < 32; y++)
            for (int x = 0; x < 180; x++)
            {
                int offset = (y * 180 + x) * 4;
                if (pixels[offset] >= 140 && pixels[offset + 1] >= 140 && pixels[offset + 2] >= 140)
                { count++; totalX += x; }
            }
            Check(count > 10, "Disabled button text disappeared into the dark theme");
            double center = totalX / (double)count;
            Check(alignment == System.Windows.HorizontalAlignment.Left ? center < 60 :
                alignment == System.Windows.HorizontalAlignment.Right ? center > 120 : center > 65 && center < 115,
                "Disabled themed button ignored text alignment");
            bool refused = false;
            try
            {
                var peer = new System.Windows.Automation.Peers.ButtonAutomationPeer(button);
                ((System.Windows.Automation.Provider.IInvokeProvider)peer.GetPattern(
                    System.Windows.Automation.Peers.PatternInterface.Invoke)).Invoke();
            }
            catch (System.Windows.Automation.ElementNotEnabledException) { refused = true; }
            Check(refused, "Disabled WPF action remained available to accessibility input");
        }
        Check(clicks == 0, "Readable disabled button became actionable");
    }

    private static Button FindVideoButton(Control parent, string label)
    {
        if (parent is EmbeddedVideoPlayer player)
        {
            foreach (var item in WpfTree(player.VideoControls))
            {
                if (item is Button button && System.Windows.Automation.AutomationProperties.GetName(button) == label)
                { return button; }
            }
        }
        foreach (Control child in parent.Controls)
        {
            var match = FindVideoButton(child, label);
            if (match != null)
            {
                return match;
            }
        }
        return null;
    }

    private static System.Collections.Generic.IEnumerable<System.Windows.DependencyObject> WpfTree(System.Windows.DependencyObject root)
    {
        yield return root;
        for (int index = 0; index < System.Windows.Media.VisualTreeHelper.GetChildrenCount(root); index++)
        {
            foreach (var child in WpfTree(System.Windows.Media.VisualTreeHelper.GetChild(root, index)))
            { yield return child; }
        }
    }

    private static void ClickVideoButton(Button button)
    {
        Check(button != null && button.IsEnabled, "The requested WPF action is unavailable");
        var peer = new System.Windows.Automation.Peers.ButtonAutomationPeer(button);
        var invoke = (System.Windows.Automation.Provider.IInvokeProvider)
            peer.GetPattern(System.Windows.Automation.Peers.PatternInterface.Invoke);
        invoke.Invoke();
        button.Dispatcher.Invoke(System.Windows.Threading.DispatcherPriority.ApplicationIdle, new Action(() => { }));
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
            Check(!AudioAlerts.Enabled && !AudioAlerts.SpeechEnabled && !AudioAlerts.AltitudeCalloutsEnabled,
                "Plugin exit retained audio admission");
            Check(shutdown.IsCancellationRequested, "Plugin exit did not propagate shutdown");
            CheckClean(fake);
            Check(hud.Image == null, "Plugin exit retained HUD image");
            CheckDisposed(frame);
            Check(!video.IsActive, "Plugin exit left HUD running");
            Check(menuDisposed, "Plugin exit retained its video menu subscription");
        }
        MediaWorkerHost.RequestStart();
    }

    private static void DisposedSubscriptions()
    {
        using (var shutdown = new CancellationTokenSource())
        using (var hud = new PictureBox())
        {
            var view = CreateDisposedView(shutdown.Token);
            var video = CreateDisposedHudVideo(hud);
            WaitUi(() =>
            {
                GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
                return !view.IsAlive && !video.IsAlive;
            }, "disposed view continuations and subscriptions to release");
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
