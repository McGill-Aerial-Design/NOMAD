// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Reflection;
using System.Threading;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void TrackingRequestCadence()
    {
        using (var peer = new TrackingPeer())
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), Size = new Size(800, 600) })
        {
            var pipeline = new FakeVideoPipeline();
            var profile = new VideoInputProfile { Name = "Tracking cadence", Url = "udp://@:5600",
                TrackingCalibrated = true, TrackingMaxRateDegSec = 60 };
            using (var player = new EmbeddedVideoPlayer(profile, true, CancellationToken.None, () => pipeline,
                peer.Config, createTracker: (token, mode) => new FakeVisualTracker()))
            {
                form.Controls.Add(player); form.Show(); player.StartStream();
                Wait(pipeline.ReadEntered, "cadence video");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.SelectRegion(new RectangleF(65, 30, 20, 20)), "Cadence selection failed");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.StartTracking(), "Cadence tracking did not start");
                var frame = player.DisplayedVideoFrame;
                long received = System.Diagnostics.Stopwatch.GetTimestamp();
                var sample = new VideoTrackingSample(frame.Image.Size, frame.Sequence + 1, frame.Generation,
                    received, DateTime.UtcNow, frame.Region, frame.Motion);
                SetField(player, "_lastRequestTicks", received - System.Diagnostics.Stopwatch.Frequency * 35 / 1000);
                SetField(player, "_lastStepTicks", received - System.Diagnostics.Stopwatch.Frequency / 30);
                typeof(EmbeddedVideoPlayer).GetMethod("UpdateVisionSample", BindingFlags.Instance | BindingFlags.NonPublic)
                    .Invoke(player, new object[] { sample });
                peer.Until(() => player.SubmittedTrackingCommands == 1, "native target from a fresh 30 Hz sample");
                Check(peer.MountTargets > 0 && player.CommandedYawDeg > 0 && player.CommandedYawDeg <= 2,
                    "30 Hz visual steering failed to deliver a bounded native mount target");
                player.StopTracking();
            }
        }
    }

    private static void TrackingBusyVisualTime()
    {
        using (var peer = new TrackingPeer())
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), Size = new Size(800, 600) })
        {
            var pipeline = new FakeVideoPipeline();
            var profile = new VideoInputProfile { Name = "Busy visual correction", Url = "udp://@:5600",
                TrackingCalibrated = true, TrackingMaxRateDegSec = 60 };
            using (var player = new EmbeddedVideoPlayer(profile, true, CancellationToken.None, () => pipeline,
                peer.Config, createTracker: (token, mode) => new FakeVisualTracker()))
            {
                form.Controls.Add(player); form.Show();
                player.StartStream(); Wait(pipeline.ReadEntered, "busy visual video");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.SelectRegion(new RectangleF(65, 30, 20, 20)), "Busy visual selection failed");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.StartTracking(), "Busy visual tracking did not start");
                var busy = typeof(OutputController).GetField("_payloadBusy", BindingFlags.NonPublic | BindingFlags.Static);
                try
                {
                    busy.SetValue(null, true);
                    for (int frame = 0; frame < 5; frame++)
                    { Thread.Sleep(60); PlayerVisionFrame(player, pipeline, Color.Blue); peer.Pump(15); }
                    Check(player.SubmittedTrackingCommands == 0, "Busy native path accepted a visual target");
                }
                finally { busy.SetValue(null, false); }
                Thread.Sleep(60); PlayerVisionFrame(player, pipeline, Color.Blue);
                peer.Until(() => player.SubmittedTrackingCommands > 0, "fresh correction after native busy interval");
                // Independent pinhole error is atan(25/50)=26.565 degrees; the .2s budget permits5.313 degrees.
                Check(player.CommandedYawDeg >= 4.8f && player.CommandedYawDeg <= 5.4f,
                    "Rejected busy samples consumed elapsed steering time: yaw=" + player.CommandedYawDeg);
                player.StopTracking();
            }
        }
    }

    private static void TrackingHeadingFences()
    {
        using (var peer = new TrackingPeer())
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), Size = new Size(800, 600) })
        {
            var pipeline = new FakeVideoPipeline();
            var profile = new VideoInputProfile { Name = "Heading", Url = "udp://@:5600",
                TrackingCalibrated = true, TrackingMaxRateDegSec = 60 };
            using (var player = new EmbeddedVideoPlayer(profile, true, CancellationToken.None, () => pipeline,
                peer.Config, createTracker: (token, mode) => new FakeVisualTracker()))
            {
                form.Controls.Add(player); form.Show();
                player.StartStream(); Wait(pipeline.ReadEntered, "heading video");
                foreach (float endpoint in new[] { (float)Math.PI, -(float)Math.PI })
                {
                    peer.Attitude(endpoint); peer.Pump(30);
                    PlayerVisionFrame(player, pipeline, Color.Blue);
                    Check(player.SelectRegion(new RectangleF(40, 30, 20, 20)), "Endpoint selection failed");
                    PlayerVisionFrame(player, pipeline, Color.Blue);
                    Check(player.StartTracking(), "Valid float-pi heading endpoint was rejected");
                    player.StopTracking();
                }
                peer.Attitude((float)(179 * Math.PI / 180)); peer.Pump(30);
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.SelectRegion(new RectangleF(40, 30, 20, 20)), "Heading selection failed");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.StartTracking(), "Fresh selected attitude did not admit tracking");
                peer.Attitude((float)(-179 * Math.PI / 180)); peer.Pump(30);
                Thread.Sleep(110); PlayerVisionFrame(player, pipeline, Color.Blue);
                peer.Until(() => player.CommandedYawDeg < -1.9f, "body yaw compensation across heading wrap");
                Check(Math.Abs(player.CommandedYawDeg + 2) < .01 && peer.MountTargets > 0,
                    "Heading wrap commanded a long turn or wrong native yaw direction");
                TrackingBusyHeading(peer, player, pipeline);
                peer.Attitude((float)(-163 * Math.PI / 180)); peer.Pump(20);
                peer.Pump(70);
                int beforeClipped = player.SubmittedTrackingCommands;
                typeof(EmbeddedVideoPlayer).GetField("_lastStepTicks", BindingFlags.NonPublic | BindingFlags.Instance)
                    .SetValue(player, System.Diagnostics.Stopwatch.GetTimestamp() - System.Diagnostics.Stopwatch.Frequency / 20);
                PlayerVisionFrame(player, pipeline, Color.Blue);
                peer.Until(() => player.SubmittedTrackingCommands > beforeClipped, "first deliberately rate-clipped sample");
                Check(player.CommandedYawDeg > -17.9, "Heading fixture did not exercise rate clipping");
                for (int frame = 0; frame < 3 && Math.Abs(player.CommandedYawDeg + 18) >= .01; frame++)
                { Thread.Sleep(110); PlayerVisionFrame(player, pipeline, Color.Blue); peer.Pump(20); }
                peer.Until(() => Math.Abs(player.CommandedYawDeg + 18) < .01, "rate-clipped heading remainder");
                foreach (string fault in new[] { "Stale", "WrongTarget", "Invalid" })
                {
                    if (!player.IsTracking)
                    {
                        peer.StreamAttitude = true; peer.Attitude(0); peer.Heartbeat(4); peer.Pump(30);
                        PlayerVisionFrame(player, pipeline, Color.Blue);
                        Check(player.SelectRegion(new RectangleF(40, 30, 20, 20)), "Fault selection failed");
                        PlayerVisionFrame(player, pipeline, Color.Blue);
                        Check(player.StartTracking(), "Fault fixture could not restart explicitly");
                    }
                    int commands = player.SubmittedTrackingCommands;
                    peer.StreamAttitude = false;
                    if (fault == "Invalid") { peer.Attitude(float.NaN); peer.Pump(20); }
                    else
                    {
                        Thread.Sleep(550);
                        if (fault == "WrongTarget") { peer.Attitude(1, 202); peer.Pump(20); }
                    }
                    PlayerVisionFrame(player, pipeline, Color.Blue);
                    Check(!player.IsTracking && player.SubmittedTrackingCommands == commands,
                        fault + " aircraft attitude retained tracking or emitted another mount target");
                }
            }
        }
    }

    private static void TrackingBusyHeading(TrackingPeer peer, EmbeddedVideoPlayer player, FakeVideoPipeline pipeline)
    {
        var busy = typeof(OutputController).GetField("_payloadBusy", BindingFlags.NonPublic | BindingFlags.Static);
        int commands = player.SubmittedTrackingCommands;
        try
        {
            busy.SetValue(null, true);
            peer.Attitude((float)(-176 * Math.PI / 180)); peer.Pump(20);
            Thread.Sleep(110); PlayerVisionFrame(player, pipeline, Color.Blue); peer.Pump(20);
            Check(player.IsTracking && player.SubmittedTrackingCommands == commands, "Busy heading sent or retired input");
        }
        finally { busy.SetValue(null, false); }
        peer.Attitude((float)(-173 * Math.PI / 180)); peer.Pump(20);
        Thread.Sleep(110); PlayerVisionFrame(player, pipeline, Color.Blue);
        peer.Until(() => player.SubmittedTrackingCommands > commands, "new heading after busy");
        Check(Math.Abs(player.CommandedYawDeg + 8) < .01,
            "Busy sample consumed the heading reference or replayed its old command");
    }
}
