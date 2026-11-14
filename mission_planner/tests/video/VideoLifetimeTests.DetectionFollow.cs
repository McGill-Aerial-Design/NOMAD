// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Threading;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static VideoDetection Box(string label, float confidence, float x, float y, float size) =>
        new VideoDetection(0, label, confidence, new RectangleF(x, y, size, size));

    private static void DetectionTargetingRules()
    {
        var large = Box("car", .9f, 0, 0, 100);
        var small = Box("car", .6f, 40, 40, 20);
        var tiny = Box("car", .99f, 45, 45, 4);
        var picked = DetectionTargeting.At(new[] { large, small, tiny }, new PointF(47, 47), 8);
        Check(ReferenceEquals(picked, small), "Click did not choose the smallest trackable box under the point");
        Check(DetectionTargeting.At(new[] { large }, new PointF(150, 150), 8) == null, "Click outside every box chose one");

        var tracked = new RectangleF(42, 42, 20, 20);
        var person = Box("person", .9f, 40, 40, 20);
        Check(ReferenceEquals(DetectionTargeting.Confirming(new[] { person, small }, tracked, "car"), small) &&
            DetectionTargeting.Confirming(new[] { person }, tracked, "car") == null &&
            ReferenceEquals(DetectionTargeting.Confirming(new[] { person }, tracked, null), person) &&
            DetectionTargeting.Confirming(new[] { Box("car", .9f, 80, 80, 20) }, tracked, "car") == null,
            "Detector confirmation ignored class or overlap");

        var near = Box("car", .5f, 60, 40, 20);
        var far = Box("car", .99f, 200, 200, 20);
        Check(ReferenceEquals(DetectionTargeting.Reacquire(new[] { far, near, person }, "car", tracked, 8), near) &&
            DetectionTargeting.Reacquire(new[] { far, person }, "car", tracked, 8) == null,
            "Re-acquisition jumped class or beyond its reach");

        var centered = Box("dog", .7f, 90, 90, 20);
        var corner = Box("dog", .75f, 0, 0, 20);
        Check(ReferenceEquals(DetectionTargeting.Auto(new[] { corner, centered }, new Size(200, 200), 8), centered) &&
            DetectionTargeting.Auto(new[] { Box("dog", .9f, 90, 90, 4) }, new Size(200, 200), 8) == null,
            "Auto follow did not prefer a confident, centered, trackable detection");
        Check(DetectionTargeting.Overlap(tracked, tracked) == 1 && DetectionTargeting.Overlap(tracked, far.Bounds) == 0,
            "Overlap boundaries are wrong");
    }

    private static void BuiltinClassSelection()
    {
        string directory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "builtin-classes-" + Guid.NewGuid().ToString("N"));
        var catalog = new VisionModelCatalog(directory);
        try
        {
            Check(catalog.DetectorPath("") == "" && catalog.LoadBuiltin().AllowedClasses.SequenceEqual(new[] { 1, 2, 3, 5, 7 }),
                "Built-in defaults changed before the operator chose classes");
            var people = VisionModelProfile.Builtin();
            people.Model = Path.Combine(directory, "yolox_nano.onnx"); people.AllowedClasses = new[] { 0 };
            Check(catalog.Save(people, catalog.BuiltinPath) == catalog.BuiltinPath, "Built-in classes saved elsewhere");
            string other = Path.Combine(directory, "other.json");
            Check(catalog.DetectorPath("") == catalog.BuiltinPath && catalog.DetectorPath(other) == other &&
                catalog.LoadBuiltin().AllowedClasses.SequenceEqual(new[] { 0 }) &&
                catalog.Choices().Count == 1 && catalog.Choices()[0].Path == "",
                "Saved built-in classes were not used for the built-in choice or appeared as a separate model");
        }
        finally { if (Directory.Exists(directory)) { Directory.Delete(directory, true); } }
    }

    private static void DetectionFollowsGimbal()
    {
        using (var peer = new TrackingPeer())
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), Size = new Size(800, 600) })
        {
            var pipeline = new FakeVideoPipeline();
            var profile = new VideoInputProfile { Name = "Detection follow", Url = "udp://@:5600", TrackingCalibrated = true };
            using (var player = new EmbeddedVideoPlayer(profile, true, CancellationToken.None, () => pipeline, peer.Config,
                createDetector: (path, token) => new FakeDetector(), createTracker: (token, mode) => new FakeVisualTracker()))
            {
                form.Controls.Add(player); form.Show(); player.StartStream();
                Wait(pipeline.ReadEntered, "detection follow source");
                player.EnableDetection(true);
                FrameUntil(player, pipeline, () => player.DisplayedVideoFrame?.Detections.Count == 1, "first detection");
                var frame = player.DisplayedVideoFrame;
                var viewport = new Size(frame.Image.Width, frame.Image.Height);
                Check(player.SelectTargetAt(new Point(viewport.Width / 2, viewport.Height / 2), viewport),
                    "Clicking a detection box did not select it");
                FrameUntil(player, pipeline, () => player.IsTracking, "click-to-follow tracking");
                player.StopTracking();
                Check(!player.IsTracking && !player.AutoFollow, "Stop left the gimbal or auto follow active");

                ClickVideoButton(player.VideoControls.ClearButton);
                player.SetAutoFollow(true);
                FrameUntil(player, pipeline, () => player.IsTracking, "automatic follow of the detection");
                Check(player.AutoFollow && player.VideoControls.AutoFollowCaption.Text == "Auto follow on",
                    "Auto follow state is not shown");
                player.StopTracking();
                Check(!player.AutoFollow, "Stop did not end auto follow");
                ClickVideoButton(player.VideoControls.ClearButton);
                for (int index = 0; index < 3; index++) { PlayerVisionFrame(player, pipeline, Color.Red); }
                Check(!player.IsTracking, "Auto follow restarted after the operator stopped it");
            }
        }
    }

    private static void FrameUntil(EmbeddedVideoPlayer player, FakeVideoPipeline pipeline, Func<bool> done, string what)
    {
        for (int index = 0; index < 60 && !done(); index++) { PlayerVisionFrame(player, pipeline, Color.Red); }
        Check(done(), "Timed out waiting for " + what + ": " + player.TrackingStatus);
    }
}
