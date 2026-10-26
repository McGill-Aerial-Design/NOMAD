// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.Reflection;
using System.IO;
using System.Threading;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private sealed class WrongHintVisualTracker : IVisualTracker
    {
        private readonly IVisualTracker _tracker;
        internal WrongHintVisualTracker(CancellationToken token, VisualTrackingMode mode)
        { _tracker = VisualTrackerFactory.Create(token, mode); }
        public void Initialize(Bitmap frame, RectangleF bounds, CancellationToken token, VisualFrameContext context = default)
        { _tracker.Initialize(frame, bounds, token, context); }
        public VisualTrackingResult Update(Bitmap frame, CancellationToken token, VisualFrameContext context = default) =>
            _tracker.Update(frame, token, new VisualFrameContext(new PointF(-1000, -1000)));
        public void Dispose() => _tracker.Dispose();
    }

    private static void PersistentWrongPriorStillCommands()
    {
        using (var peer = new TrackingPeer())
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), Size = new Size(800, 600) })
        using (var surface = AreaSurfaceImage())
        {
            var pipeline = new FakeVideoPipeline();
            var profile = new VideoInputProfile { Name = "Wrong area hint", Url = "udp://@:5600", TrackingMode = "Area",
                TrackingCalibrated = true, TrackingMaxRateDegSec = 60 };
            using (var player = new EmbeddedVideoPlayer(profile, true, CancellationToken.None, () => pipeline,
                peer.Config, createTracker: (token, mode) => new WrongHintVisualTracker(token, mode)))
            {
                form.Controls.Add(player); form.Show(); player.StartStream(); Wait(pipeline.ReadEntered, "real Area source");
                PlayerSurfaceFrame(player, pipeline, surface);
                Check(player.SelectRegion(new RectangleF(280, 170, 180, 140)), "Native Area selection failed");
                for (int index = 0; index < 3; index++)
                {
                    PlayerSurfaceFrame(player, pipeline, surface);
                    Console.WriteLine("  Native Area initialization " + index + ": " +
                        player.DisplayedVideoFrame.Region?.State + ", " + player.TrackingStatus);
                }
                peer.Heartbeat(4); peer.Attitude(0); peer.Pump(30);
                Check(player.RegionReady && player.SubmittedTrackingCommands == 0 && player.StartTracking(),
                    "Native Area fixture failed to verify and explicitly arm: " + player.TrackingStatus);
                int commands = player.SubmittedTrackingCommands;
                for (int index = 0; index < 8; index++)
                {
                    Thread.Sleep(80); peer.Pump(20); PlayerSurfaceFrame(player, pipeline, surface); peer.Pump(20);
                    Check(player.DisplayedVideoFrame.Region.Found && player.IsTracking && player.RegionReady,
                        "A persistent wrong prior caused Media to pause an always-unique native Area");
                }
                peer.Until(() => player.SubmittedTrackingCommands > commands, "native command despite persistent wrong prior");
                Check(peer.MountTargets > 0 && player.CommandedYawDeg > 0 && player.CommandedYawDeg <= 90,
                    "Soft-prior tracking never delivered a fresh bounded native target");
            }
        }
    }

    private static Bitmap AreaSurfaceImage()
    {
        var bitmap = new Bitmap(640, 480);
        var random = new Random(173);
        using (var graphics = Graphics.FromImage(bitmap))
        {
            graphics.Clear(Color.FromArgb(90, 105, 80));
            for (int index = 0; index < 2200; index++)
            {
                int x = random.Next(640), y = random.Next(480), size = random.Next(3, 13);
                using (var brush = new SolidBrush(Color.FromArgb(random.Next(256), random.Next(256), random.Next(256))))
                { graphics.FillRectangle(brush, x, y, size, random.Next(3, 13)); }
            }
        }
        return bitmap;
    }

    private static void PlayerSurfaceFrame(EmbeddedVideoPlayer player, FakeVideoPipeline pipeline, Bitmap surface)
    {
        long previous = player.DisplayedVideoFrame?.Sequence ?? -1;
        pipeline.Frames.Add((Bitmap)surface.Clone());
        var deadline = Stopwatch.StartNew();
        while (deadline.Elapsed < TimeSpan.FromSeconds(5))
        {
            Tick(player);
            if (player.DisplayedVideoFrame?.Sequence > previous) { return; }
            Thread.Sleep(5);
        }
        throw new InvalidOperationException("Timed out waiting for real Area frame: " + player.TrackingStatus);
    }

    private static void AreaRuntimeRecovery()
    {
        using (var peer = new TrackingPeer())
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), Size = new Size(800, 600) })
        {
            var pipeline = new FakeVideoPipeline();
            FakeVisualTracker visual = null;
            var profile = new VideoInputProfile { Name = "Area runtime", Url = "udp://@:5600", TrackingMode = "Area",
                TrackingCalibrated = true, TrackingMaxRateDegSec = 60 };
            using (var player = new EmbeddedVideoPlayer(profile, true, CancellationToken.None, () => pipeline,
                peer.Config, createTracker: (token, mode) => visual = new FakeVisualTracker { AimPoint = new PointF(50, 40) }))
            {
                form.Controls.Add(player); form.Show(); player.StartStream(); Wait(pipeline.ReadEntered, "area source");
                var session = GetField<VideoSession>(player, "_session");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.TrackingMode == VisualTrackingMode.Area &&
                    GetField<Button>(player, "_clearAreaButton").Visible, "Area default or explicit Clear was missing");
                Check(player.SelectRegion(new RectangleF(70, 50, 20, 20)), "Area fixture could not select");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                var toggle = GetField<Button>(player, "_trackButton");
                int toggleWidth = toggle.Width;
                Check(player.Controls.Find("TrackingToggle", true).Length == 1,
                    "Tracking controls did not collapse to one toggle");
                toggle.PerformClick();
                Check(player.IsTracking && toggle.Text == "Stop tracking" && toggle.Width == toggleWidth,
                    "Verified Area toggle did not start without resizing");
                int commands = player.SubmittedTrackingCommands;
                Thread.Sleep(110); PlayerVisionFrame(player, pipeline, Color.Blue); peer.Pump(20);
                Check(player.SubmittedTrackingCommands == commands,
                    "Bounding-box center overrode the independently supplied canonical point");
                SetField(player, "_trackingUnappliedYawChange", 20f);
                visual.State = VisualTrackingState.Searching;
                pipeline.Frames.Add(SolidVideoImage(Color.Blue));
                using (var dropped = NextVideoFrame(session))
                { Check(dropped.Region.State == VisualTrackingState.Searching, "No authoritative dropped Searching sample"); }
                Tick(player);
                Check(player.IsTracking && !player.RegionReady && player.SelectedRegion.HasValue &&
                    GetField<float>(player, "_trackingUnappliedYawChange") == 0 && visual.Disposals == 0,
                    "Searching dropped the reference/intent or retained heading debt");
                peer.Attitude((float)(60 * Math.PI / 180)); peer.Pump(30);
                foreach (var state in new[] { VisualTrackingState.Outside, VisualTrackingState.Searching })
                { visual.State = state; PlayerVisionFrame(player, pipeline, Color.Blue); }
                Check(player.SubmittedTrackingCommands == commands && !player.RegionReady,
                    "Unverified area emitted a native request");
                var visionGate = GetField<object>(player, "_visionGate");
                var previousConnection = OutputController.CurrentConnection;
                int nativeBoundaryReads = 0;
                OutputController.CurrentConnection = () =>
                {
                    Check(!Monitor.IsEntered(visionGate), "Native output boundary entered while the vision lock was held");
                    Interlocked.Increment(ref nativeBoundaryReads);
                    return previousConnection();
                };
                try
                {
                    visual.State = VisualTrackingState.Verified;
                    PlayerVisionFrame(player, pipeline, Color.Blue);
                    Check(player.IsTracking && player.RegionReady && player.SubmittedTrackingCommands == commands,
                        "Resume replayed yaw or moved on its baseline sample");
                    Thread.Sleep(110); PlayerVisionFrame(player, pipeline, Color.Blue); peer.Pump(20);
                    Check(player.SubmittedTrackingCommands == commands,
                        "Aircraft yaw accumulated during pause was replayed after recovery");
                    visual.AimPoint = new PointF(70, 40);
                    Thread.Sleep(110); PlayerVisionFrame(player, pipeline, Color.Blue);
                    peer.Until(() => player.SubmittedTrackingCommands > commands, "fresh canonical Area command");
                    Check(nativeBoundaryReads > 0 && player.CommandedYawDeg > 0 && player.CommandedYawDeg <= 12,
                        "Recovery did not cross the native boundary with a new bounded command");
                }
                finally { OutputController.CurrentConnection = previousConnection; }
                toggle.PerformClick(); commands = player.SubmittedTrackingCommands;
                Check(toggle.Text == "Start tracking" && toggle.Width == toggleWidth,
                    "Tracking toggle did not stop without resizing");
                Check(player.SelectedRegion.HasValue && !player.IsTracking && visual.Disposals == 0,
                    "Stop erased Area memory or left tracking intent armed");
                visual.State = VisualTrackingState.Searching; PlayerVisionFrame(player, pipeline, Color.Blue);
                visual.State = VisualTrackingState.Verified; PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(!player.IsTracking && player.SubmittedTrackingCommands == commands,
                    "An explicit Stop automatically regained steering");
                peer.Heartbeat(4); peer.Pump(20);
                Check(player.StartTracking(), "Explicit re-arm failed");
                GimbalController.SetTargetAngles(0, 0);
                visual.State = VisualTrackingState.Searching; PlayerVisionFrame(player, pipeline, Color.Blue);
                visual.State = VisualTrackingState.Verified; PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(!player.IsTracking && player.SelectedRegion.HasValue && player.SubmittedTrackingCommands == commands,
                    "Manual input erased memory or automatically regained steering");
                Check(player.StartTracking(), "Claim fixture could not re-arm");
                GimbalController.ClaimTracking(); Tick(player);
                Check(!player.IsTracking && player.SelectedRegion.HasValue,
                    "A newer tracking owner did not disarm the old reference's command intent");
                Check(player.StartTracking(), "Stale-input fixture could not re-arm");
                Thread.Sleep(550); Tick(player);
                Check(!player.IsTracking && player.SelectedRegion.HasValue && !player.RegionReady,
                    "Stale video left automatic recovery armed or erased independent Area memory");
                peer.Heartbeat(4); peer.Attitude(0); peer.Pump(20);
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(!player.IsTracking && player.RegionReady && player.SubmittedTrackingCommands == commands,
                    "Fresh pixels after stale input automatically regained control");
                player.SetTrackingMode(VisualTrackingMode.Object);
                Check(!player.SelectedRegion.HasValue && !GetField<Button>(player, "_clearAreaButton").Visible,
                    "Mode replacement retained the previous Area reference");
                player.SetTrackingMode(VisualTrackingMode.Area);
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.SelectRegion(new RectangleF(30, 20, 20, 20)), "Clear fixture selection failed");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                GetField<Button>(player, "_clearAreaButton").PerformClick();
                Check(!player.SelectedRegion.HasValue && !session.HasRegion(GetField<long>(player, "_regionRevision")),
                    "Explicit Clear kept an Area reference alive");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.SelectRegion(new RectangleF(30, 20, 20, 20)), "Source fixture selection failed");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                OutputController.Initialize(new NOMADConfig { AircraftName = "Retired area binding" }); Tick(player);
                Check(!player.SelectedRegion.HasValue && !player.IsTracking,
                    "An aircraft/profile generation change retained visual Area memory");
                player.UpdateInput(profile.Copy());
                Check(!player.SelectedRegion.HasValue && !player.IsTracking && player.DisplayedVideoFrame == null,
                    "Source/profile replacement retained Area memory or commands");
            }
        }
    }

    private static void GroundHintAndHeadingFrame()
    {
        using (var peer = new TrackingPeer())
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), Size = new Size(800, 600) })
        {
            GimbalController.SetTargetAngles(-45, 0);
            var command = GimbalController.RequestTrackingTargetAsync(-45, 0, OutputController.Generation,
                GimbalController.ManualRevision, () => true);
            peer.Until(() => command.IsCompleted, "ground frame pose"); Check(command.Result.Submitted, "No ground pose");
            peer.Pump(600); peer.MotionPosition(45, 0, 200, 100); peer.Pump(20);
            var pipeline = new FakeVideoPipeline(); FakeVisualTracker visual = null;
            var profile = new VideoInputProfile { Name = "Ground estimate", Url = "udp://@:5600", TrackingMode = "Area",
                TrackingCalibrated = true, GroundPredictionEnabled = true };
            using (var player = new EmbeddedVideoPlayer(profile, true, CancellationToken.None, () => pipeline,
                peer.Config, createTracker: (token, mode) => visual = new FakeVisualTracker { AimPoint = new PointF(50, 40) }))
            {
                form.Controls.Add(player); form.Show();
                player.StartStream(); Wait(pipeline.ReadEntered, "ground-frame source");
                PlayerVisionFrame(player, pipeline, Color.Black);
                Check(player.SelectRegion(new RectangleF(40, 30, 20, 20)), "Ground-frame selection failed");
                PlayerVisionFrame(player, pipeline, Color.Black);
                var verified = player.DisplayedVideoFrame;
                Check(verified.Motion?.IsCurrent == true && verified.Motion.Ground != null &&
                    GroundProjection.TryCreateAnchor(profile, verified.Image.Size, verified.Region.AimPoint.Value,
                        verified.Motion.Ground, 0, out var expectedAnchor), "Verified frame lacked its native ground snapshot");
                // Build the expectation from the same captured sample before advancing native position.
                GroundProjection.TryCreateAnchor(profile, verified.Image.Size, verified.Region.AimPoint.Value,
                    verified.Motion.Ground, 0, out expectedAnchor);
                peer.MotionPosition(45.00001, 0, 200, 100); peer.Pump(20);
                visual.State = VisualTrackingState.Searching;
                PlayerVisionFrame(player, pipeline, Color.Black);
                var searching = player.DisplayedVideoFrame;
                Check(searching.Region.GroundSearchHint.HasValue && searching.Motion.IsCurrent &&
                    GroundProjection.TryProjectAnchor(profile, searching.Image.Size, expectedAnchor,
                        searching.Motion.Ground, out var expected), "A retained area lacked its optional current search hint");
                GroundProjection.TryProjectAnchor(profile, searching.Image.Size, expectedAnchor, searching.Motion.Ground, out expected);
                Check(Distance(searching.Region.GroundSearchHint.Value, expected) < .01 &&
                    visual.LastContext.SearchHint == searching.Region.GroundSearchHint &&
                    !player.RegionReady && player.SubmittedTrackingCommands == 0,
                    "Ground hint used another frame's pose or became visual/control authority");
                CheckSearchingObservation(player);
                CheckGroundHintVisibility(player, true);
                long revision = GetField<long>(player, "_regionRevision");
                try
                {
                    SetField(player, "_regionRevision", revision + 1);
                    CheckGroundHintVisibility(player, false);
                }
                finally { SetField(player, "_regionRevision", revision); }
                CheckHeadingViewportRendering(player);
                Thread.Sleep(550);
                CheckHeadingViewportRendering(player, false);
                peer.Heartbeat(4);
                peer.MotionPosition(45.00001, 0, 200, 101); peer.Pump(20);
                PlayerVisionFrame(player, pipeline, Color.Black);
                Check(!player.DisplayedVideoFrame.Region.GroundSearchHint.HasValue && player.SelectedRegion.HasValue,
                    "An epoch change reused its old ground anchor or erased independent visual Area memory");
                visual.State = VisualTrackingState.Verified; PlayerVisionFrame(player, pipeline, Color.Black);
                visual.State = VisualTrackingState.Searching; PlayerVisionFrame(player, pipeline, Color.Black);
                CheckGroundHintVisibility(player, true);
                RejectInconsistentGroundPrior(player, pipeline, peer, visual);
                Check(player.SelectRegion(new RectangleF(40, 30, 20, 20)), "New ground-prior selection failed");
                PlayerVisionFrame(player, pipeline, Color.Black); PlayerVisionFrame(player, pipeline, Color.Black);
                CheckGroundHintVisibility(player, true);
                GetField<Button>(player, "_clearAreaButton").PerformClick();
                CheckGroundHintVisibility(player, false);
                player.SetTrackingMode(VisualTrackingMode.Object);
                CheckGroundHintVisibility(player, false);
            }
        }
    }

    private static void RejectInconsistentGroundPrior(EmbeddedVideoPlayer player, FakeVideoPipeline pipeline,
        TrackingPeer peer, FakeVisualTracker visual)
    {
        peer.MotionPosition(45.00001, .01, 200, 101); peer.Pump(20);
        peer.Until(() => peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT)
            .ToStructure<MAVLink.mavlink_global_position_int_t>().lon == 100000, "inconsistent native position arrival");
        visual.State = VisualTrackingState.Verified;
        PlayerVisionFrame(player, pipeline, Color.Black);
        var frame = player.DisplayedVideoFrame;
        float radius = VisualFrameContext.PriorSearchRadius(frame.Image.Size, new RectangleF(40, 30, 20, 20));
        Check(visual.LastContext.SearchHint.HasValue &&
            Math.Abs(visual.LastContext.SearchHint.Value.X - frame.Region.AimPoint.Value.X) > radius,
            "Ground fixture did not disagree beyond its qualified image-scaled neighborhood");
        Check(frame.Region.Found && player.RegionReady && !frame.Region.GroundSearchHint.HasValue,
            "A contradictory ground estimate replaced visual identity or remained available");
        for (int index = 0; index < 3; index++)
        {
            PlayerVisionFrame(player, pipeline, Color.Black);
            Check(player.DisplayedVideoFrame.Region.Found && !visual.LastContext.SearchHint.HasValue &&
                !player.DisplayedVideoFrame.Region.GroundSearchHint.HasValue,
                "A disabled ground anchor was reseeded to fake agreement within the same selection");
        }
    }

    private static void CheckGroundHintVisibility(EmbeddedVideoPlayer player, bool expected)
    {
        var box = GetField<PictureBox>(player, "_videoBox");
        using (var bitmap = new Bitmap(box.Width, box.Height))
        using (var graphics = Graphics.FromImage(bitmap))
        {
            typeof(EmbeddedVideoPlayer).GetMethod("PaintMotionEstimates", BindingFlags.NonPublic | BindingFlags.Instance)
                .Invoke(player, new object[] { box, graphics });
            bool amber = false;
            for (int y = 0; y < bitmap.Height && !amber; y++)
            for (int x = 0; x < bitmap.Width && !amber; x++)
            {
                var color = bitmap.GetPixel(x, y);
                amber = color.R > 170 && color.G > 90 && color.G < 200 && color.B < 90;
            }
            Check(amber == expected, "Estimated hint paint ignored its current selection/mode/revision binding");
        }
    }

    private static void CheckSearchingObservation(EmbeddedVideoPlayer player)
    {
        string directory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "owned-area-observation-" + Guid.NewGuid().ToString("N"));
        player.ObservationDirectory = directory;
        try
        {
            string path = player.SaveObservation();
            Check(path != null, "Current Searching frame could not be saved");
            var metadata = Newtonsoft.Json.Linq.JObject.Parse(File.ReadAllText(Path.ChangeExtension(path, ".json")));
            var selected = metadata["SelectedTarget"];
            Check((string)selected["Mode"] == "Area" && (string)selected["State"] == "Searching" &&
                !(bool)selected["Found"] && (bool)selected["ReferenceRetained"] &&
                selected["Box"].Type == Newtonsoft.Json.Linq.JTokenType.Null &&
                selected["CanonicalAimPoint"].Type == Newtonsoft.Json.Linq.JTokenType.Null &&
                (string)selected["Tracker"] != "OpenCV CSRT" && !(bool)metadata["TargetPosition"]["Available"],
                "Searching memory/estimated prior was exported as a current verified target or metric position");
        }
        finally { if (Directory.Exists(directory)) { Directory.Delete(directory, true); } }
    }

    private static void CheckHeadingViewportRendering(EmbeddedVideoPlayer player, bool expectedMarker = true)
    {
        var paint = typeof(EmbeddedVideoPlayer).GetMethod("OnVideoPaint", BindingFlags.NonPublic | BindingFlags.Instance);
        var primary = GetField<PictureBox>(player, "_videoBox");
        using (var fullscreen = new PictureBox { Size = new Size(320, 240), Image = player.DisplayedVideoFrame.Image })
        {
            foreach (var box in new[] { primary, fullscreen })
            {
                using (var bitmap = new Bitmap(box.Width, box.Height))
                using (var graphics = Graphics.FromImage(bitmap))
                {
                    paint.Invoke(player, new object[] { box, new PaintEventArgs(graphics, box.ClientRectangle) });
                    bool marker = false;
                    for (int y = box.Height / 2 - 38; y < box.Height / 2 - 31; y++)
                    for (int x = box.Width / 2 - 3; x <= box.Width / 2 + 3; x++)
                    { var color = bitmap.GetPixel(x, y); marker |= color.B > 180 && color.G > 90 && color.R < 90; }
                    Check(marker == expectedMarker, "Current/stale heading marker did not match its viewport binding: " +
                        box.Size + ", heading=" + player.DisplayedVideoFrame.Motion.Heading?.RelativeDegrees +
                        ", current=" + player.DisplayedVideoFrame.Motion.IsCurrent);
                }
            }
        }
    }
}
