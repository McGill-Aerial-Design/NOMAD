// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.Threading;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void TrackingProcessingDelay()
    {
        foreach (var mode in new[] { VisualTrackingMode.Object, VisualTrackingMode.Area })
        using (var peer = new TrackingPeer())
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), Size = new Size(800, 600) })
        {
            var pipeline = new FakeVideoPipeline();
            FakeVisualTracker tracker = null;
            var profile = new VideoInputProfile { Name = "Processing delay", Url = "udp://@:5600",
                TrackingMode = mode.ToString(), TrackingCalibrated = true, TrackingMaxRateDegSec = 60 };
            using (var player = new EmbeddedVideoPlayer(profile, true, CancellationToken.None, () => pipeline,
                peer.Config, createTracker: (token, selected) => tracker = new FakeVisualTracker()))
            {
                form.Controls.Add(player); form.Show(); player.StartStream(); Wait(pipeline.ReadEntered, "delay video");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.SelectRegion(new RectangleF(65, 30, 20, 20)), "Delay selection failed");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.StartTracking(), "Delay tracking did not start");
                tracker.Entered.Reset(); tracker.HoldUpdates = true;
                pipeline.Frames.Add(SolidVideoImage(Color.Blue));
                Wait(tracker.Entered, "held tracking update");
                try
                {
                    long sequence = player.DisplayedVideoFrame.Sequence;
                    var elapsed = Stopwatch.StartNew();
                    while (elapsed.ElapsedMilliseconds < 750)
                    {
                        pipeline.Frames.Add(SolidVideoImage(Color.Blue));
                        peer.Pump(35); Tick(player);
                    }
                    Check(player.DisplayedVideoFrame.Sequence > sequence && player.DisplayedVideoFrame.Fresh,
                        "The delay fixture did not preserve fresh progressing video");
                    Check(player.IsTracking && !player.RegionReady && GetField<bool>(player, "_trackingPaused"),
                        "Processing delay discarded intent instead of pausing current inputs: " + player.TrackingStatus);
                    int commands = player.SubmittedTrackingCommands;
                    peer.Pump(80); Tick(player);
                    Check(player.SubmittedTrackingCommands == commands, "Stale tracking emitted a native request");
                    tracker.State = VisualTrackingState.Searching;
                }
                finally { tracker.HoldUpdates = false; tracker.Release.Set(); }
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.IsTracking && !player.RegionReady, "An unverified delayed return resumed steering");
                tracker.State = VisualTrackingState.Verified;
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.IsTracking && player.RegionReady && !GetField<bool>(player, "_trackingPaused"),
                    "A fresh verified return did not resume the same tracking intent");
                int resumedCommands = player.SubmittedTrackingCommands;
                Thread.Sleep(60); PlayerVisionFrame(player, pipeline, Color.Blue);
                peer.Until(() => player.SubmittedTrackingCommands > resumedCommands,
                    "bounded native command after processing recovery");
                player.StopTracking();
            }
        }
    }
}
