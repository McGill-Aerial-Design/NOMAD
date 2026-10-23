// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void FlightInputVectors()
    {
        var input = new VideoFlightInput();
        var other = new VideoFlightInput();
        Check(input.Claim() && !other.Claim(), "Two video controls acquired the aircraft");
        try
        {
            Check(!input.TrySample(VideoFlightKeys.Down, out _), "Held Shift crossed capture boundary");
            Check(!input.TrySample(VideoFlightKeys.None, out _), "Release sample dispatched movement");
            Check(input.TrySample(VideoFlightKeys.Forward | VideoFlightKeys.Right | VideoFlightKeys.Up |
                VideoFlightKeys.YawRight, out var vector), "Fresh keys did not produce movement");
            Check(Math.Abs(vector.Forward * vector.Forward + vector.Right * vector.Right - 4) < .001 &&
                vector.Down == -1 && Math.Abs(vector.YawRate - Math.PI / 6) < .001 && vector.Valid,
                "Diagonal speed, NED vertical sign or yaw radians are wrong");
            input.TrySample((VideoFlightKeys)255, out vector);
            Check(vector.Forward == 0 && vector.Right == 0 && vector.Down == 0 && vector.YawRate == 0,
                "Opposite controls did not cancel");
            Check(!new VideoFlightVelocity { Forward = float.NaN }.Valid &&
                !new VideoFlightVelocity { Forward = 2, Right = 2 }.Valid, "Invalid vector passed boundary validation");
        }
        finally { input.Release(); }
        Check(other.Claim(), "Released capture remained occupied");
        other.Release();
    }

    private static void FlightNativeFences()
    {
        using (var peer = new FlightPeer())
        {
            foreach (float invalid in new[] { 0, -1, float.NaN, 3.1f })
            {
                peer.Timeout = invalid;
                var refused = new VideoFlightController(peer.Config, new VideoFlightInput(), () => true);
                Check(!peer.Finish(refused.PrepareAsync()), "Invalid native GUID_TIMEOUT admitted capture");
            }
            peer.Timeout = 1;
            var input = new VideoFlightInput();
            var controller = new VideoFlightController(peer.Config, input, () => true);
            Check(peer.Finish(controller.PrepareAsync()) && input.Claim(), "Valid native fixture could not prepare");
            controller.Sample(new VideoFlightVelocity { Forward = 2, Down = -1, YawRate = .5f });
            peer.Until(() => peer.Count == 1, "velocity packet");
            var packet = peer.Last;
            Check(packet.target_system == 201 && packet.target_component == 42 && packet.coordinate_frame == 8 &&
                packet.type_mask == 1479 && packet.vx == 2 && packet.vy == 0 && packet.vz == -1 && packet.yaw_rate == .5f,
                "Native packet changed frame, mask, target or units");
            var gate = typeof(OutputController).GetField("Gate", BindingFlags.NonPublic | BindingFlags.Static).GetValue(null);
            Monitor.Enter(gate);
            try { controller.Sample(new VideoFlightVelocity { Right = 2 }); Thread.Sleep(300); }
            finally { Monitor.Exit(gate); }
            peer.Until(() => !controller.Active, "stale dispatch refusal");
            Check(peer.Count == 1, "Input older than 250 ms renewed native motion");
            peer.Finish(controller.StopAsync());
            peer.Until(() => peer.Count == 2, "release zero");
            Check(peer.Last.vx == 0 && peer.Last.vy == 0 && peer.Last.vz == 0 && peer.Last.yaw_rate == 0,
                "Release did not clear all commanded axes");

            input = new VideoFlightInput();
            controller = new VideoFlightController(peer.Config, input, () => true);
            Check(peer.Finish(controller.PrepareAsync()) && input.Claim(), "Second capture could not prepare");
            peer.Heartbeat(9); peer.Pump(50);
            controller.Sample(new VideoFlightVelocity { Forward = 2 });
            peer.Until(() => !controller.Active, "pilot mode takeover");
            peer.Finish(controller.StopAsync()); peer.Pump(50);
            Check(peer.Count == 2 && peer.Mutations == 0, "Takeover emitted setpoint, mode, arm or parameter write");
        }
    }

    private static void FlightCaptureLifecycle()
    {
        using (var peer = new FlightPeer())
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), Size = new Size(800, 600) })
        {
            var pipeline = new FakeVideoPipeline();
            var profile = new VideoInputProfile { Name = "Synthetic", Url = "udp://@:5600", TrackingCalibrated = true };
            using (var player = new EmbeddedVideoPlayer(profile, true, CancellationToken.None, () => pipeline, peer.Config))
            {
                form.Controls.Add(player);
                form.Show();
                int captures = 0, releases = 0;
                bool foreground = true;
                VideoFlightKeys keys = VideoFlightKeys.Forward;
                player.FlightForeground = () => foreground;
                player.FlightCursor = (captured, box) => { if (captured) { captures++; } else { releases++; } };
                player.FlightKeys = () => keys;
                var box = GetField<PictureBox>(player, "_videoBox");
                player.FlightPointer = () => box.PointToScreen(new Point(box.Width / 2, box.Height / 2));
                player.StartStream(); Wait(pipeline.ReadEntered, "flight video");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                player.SetFlightEnabled(true);
                Check(peer.Finish(player.BeginFlightCaptureAsync()) && captures == 1, "Actual player capture failed");
                Thread.Sleep(60); player.SampleFlight(); peer.Pump(20);
                Check(peer.Count == 0, "Held key on entry moved aircraft");
                keys = VideoFlightKeys.None;
                Thread.Sleep(60); player.SampleFlight();
                keys = VideoFlightKeys.Forward;
                Thread.Sleep(60); player.SampleFlight();
                peer.Until(() => peer.Count > 0, "captured movement");
                var escape = Message.Create(IntPtr.Zero, 0x100, (IntPtr)Keys.Escape, IntPtr.Zero);
                Check(player.PreFilterMessage(ref escape) && !player.FlightCaptured && releases == 1,
                    "Escape did not consume the key and restore cursor");
                peer.Finish(player.FlightRelease);
                PlayerVisionFrame(player, pipeline, Color.Red);
                Check(peer.Finish(player.BeginFlightCaptureAsync()), "Capture after release failed");
                Thread.Sleep(280); player.SampleFlight();
                Check(!player.FlightCaptured && releases == 2, "UI stall retained capture");
                peer.Finish(player.FlightRelease);
                PlayerVisionFrame(player, pipeline, Color.Green);
                Check(peer.Finish(player.BeginFlightCaptureAsync()), "Capture before focus test failed");
                foreground = false; player.SampleFlight();
                Check(!player.FlightCaptured && releases == 3, "Focus loss retained cursor or control");
                peer.Finish(player.FlightRelease);
            }
        }
    }

    private static void FlightStopDrain()
    {
        using (var peer = new FlightPeer())
        {
            var input = new VideoFlightInput();
            var controller = new VideoFlightController(peer.Config, input, () => true);
            Check(peer.Finish(controller.PrepareAsync()) && input.Claim(), "Stop fixture could not capture");
            var other = new VideoFlightInput();
            var gate = typeof(OutputController).GetField("Gate", BindingFlags.NonPublic | BindingFlags.Static).GetValue(null);
            Task stop;
            Monitor.Enter(gate);
            try
            {
                controller.Sample(new VideoFlightVelocity { Forward = 2 });
                stop = controller.StopAsync();
                Check(ReferenceEquals(stop, controller.StopAsync()), "Repeated stop created another release");
                Check(!other.Claim(), "New owner bypassed pending stop and cursor cleanup");
            }
            finally { Monitor.Exit(gate); }
            peer.Finish(stop);
            peer.Until(() => peer.Count == 1, "one stop zero after retired queued motion");
            Check(peer.Last.vx == 0 && other.Claim(), "Stop did not clear motion and release ownership");
            other.Release();
            Check(input.Claim(), "Original input could not recapture after drained stop");
            input.Release();

            controller = new VideoFlightController(peer.Config, input, () => true);
            Check(peer.Finish(controller.PrepareAsync()) && input.Claim(), "Retirement fixture could not capture");
            Monitor.Enter(gate);
            try
            {
                controller.Sample(new VideoFlightVelocity { Forward = 2 });
                OutputController.Initialize(new NOMADConfig());
                stop = controller.StopAsync();
            }
            finally { Monitor.Exit(gate); }
            peer.Finish(stop); peer.Pump(50);
            Check(peer.Count == 1 && !VideoFlightInput.Captured, "Retired target received movement or release zero");
        }
    }

    private static void FlightVideoAndShutdown()
    {
        using (var peer = new FlightPeer())
        using (var form = new Form { ShowInTaskbar = false, Location = new Point(-2000, -2000),
            StartPosition = FormStartPosition.Manual, Size = new Size(800, 600) })
        using (var shutdown = new CancellationTokenSource())
        {
            var pipeline = new FakeVideoPipeline();
            var profile = new VideoInputProfile { Name = "Synthetic", Url = "udp://@:5600", TrackingCalibrated = true };
            int uiThread = Thread.CurrentThread.ManagedThreadId;
            using (var player = new EmbeddedVideoPlayer(profile, true, shutdown.Token, () => pipeline, peer.Config))
            {
                form.Controls.Add(player); form.Show();
                int releases = 0;
                player.FlightForeground = () => true;
                player.FlightKeys = () => VideoFlightKeys.None;
                player.FlightCursor = (captured, box) =>
                {
                    Check(Thread.CurrentThread.ManagedThreadId == uiThread, "Cursor cleanup ran on worker thread");
                    if (!captured) { releases++; }
                };
                player.StartStream(); Wait(pipeline.ReadEntered, "flight loss video");
                PlayerVisionFrame(player, pipeline, Color.Blue); player.SetFlightEnabled(true);
                Check(peer.Finish(player.BeginFlightCaptureAsync()), "Loss fixture capture failed");
                for (int i = 0; i < 6; i++) { Thread.Sleep(90); player.SampleFlight(); }
                Check(!player.FlightCaptured && releases == 1, "Stale video retained control or cursor");
                peer.Finish(player.FlightRelease);
                PlayerVisionFrame(player, pipeline, Color.Red);
                Check(peer.Finish(player.BeginFlightCaptureAsync()), "Source fixture capture failed");
                player.StopStream(); peer.Finish(player.FlightRelease);
                Check(!player.FlightCaptured && releases == 2, "Video stop retained capture");
            }
            pipeline = new FakeVideoPipeline();
            using (var player = new EmbeddedVideoPlayer(profile, true, shutdown.Token, () => pipeline, peer.Config))
            {
                form.Controls.Add(player);
                bool cleanedOnUi = false;
                player.FlightForeground = () => true;
                player.FlightKeys = () => VideoFlightKeys.None;
                player.FlightCursor = (captured, box) =>
                {
                    if (!captured) { cleanedOnUi = Thread.CurrentThread.ManagedThreadId == uiThread; }
                };
                player.StartStream(); Wait(pipeline.ReadEntered, "shutdown flight video");
                PlayerVisionFrame(player, pipeline, Color.Blue); player.SetFlightEnabled(true);
                Check(peer.Finish(player.BeginFlightCaptureAsync()), "Shutdown fixture capture failed");
                var cancel = Task.Run(() => shutdown.Cancel());
                Thread.Sleep(60);
                var other = new VideoFlightInput();
                Check(!other.Claim(), "Worker released ownership before UI cursor cleanup");
                peer.Finish(cancel);
                peer.Until(() => player.IsDisposed, "captured shutdown UI cleanup");
                peer.Finish(player.FlightRelease);
                Check(cleanedOnUi && !VideoFlightInput.Captured && player.IsDisposed,
                    "Captured worker shutdown retained ownership or skipped UI cleanup");
            }
        }
    }
}
