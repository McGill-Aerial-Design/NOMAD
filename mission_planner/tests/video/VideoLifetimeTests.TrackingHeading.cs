// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Reflection;
using System.Threading;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
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
                Thread.Sleep(110); PlayerVisionFrame(player, pipeline, Color.Blue); peer.Pump(20);
                Thread.Sleep(110); PlayerVisionFrame(player, pipeline, Color.Blue);
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
