// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;
using Newtonsoft.Json;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private sealed class FakeDetector : IVideoDetector
    {
        internal readonly ManualResetEventSlim Entered = new ManualResetEventSlim();
        internal bool Fail, Hold, NativeCancellationError;
        internal int Disposals;
        public string Identity => "owned fake detector";
        public IReadOnlyList<VideoDetection> Detect(Bitmap frame, CancellationToken token)
        {
            Entered.Set();
            if (Hold)
            {
                Check(token.WaitHandle.WaitOne(TimeSpan.FromSeconds(5)), "Inference cancellation did not arrive");
                if (NativeCancellationError) { throw new InvalidOperationException("native cancellation fault"); }
                token.ThrowIfCancellationRequested();
            }
            if (Fail) { throw new InvalidOperationException("private-model-path must not reach the status"); }
            return new[] { new VideoDetection(2, "car", .8f, new RectangleF(0, 0, frame.Width, frame.Height)) };
        }
        public void Dispose() { Disposals++; }
    }

    private static VideoFrame NextVideoFrame(VideoSession session)
    {
        var deadline = Stopwatch.StartNew();
        while (deadline.Elapsed < TimeSpan.FromSeconds(5))
        {
            var frame = session.TakeFrame();
            if (frame != null) { return frame; }
            Thread.Sleep(5);
        }
        throw new InvalidOperationException("Timed out waiting for an owned video envelope: " + session.Error);
    }

    private static void VisionWorkerLifetime()
    {
        var pipeline = new FakeVideoPipeline();
        var detectors = new List<FakeDetector>();
        using (var session = new VideoSession(() => pipeline, (path, token) =>
            { var detector = new FakeDetector(); detectors.Add(detector); return detector; }))
        {
            session.ConfigureDetection(true, ""); session.Start("fake");
            pipeline.Frames.Add(new Bitmap(16, 8));
            long firstSequence;
            using (var frame = NextVideoFrame(session))
            {
                firstSequence = frame.Sequence;
                Check(frame.Image.Width == 16 && frame.Detections.Count == 1 && frame.ReceivedUtc.Kind == DateTimeKind.Utc &&
                    frame.Fresh && frame.DetectorIdentity == "owned fake detector", "Inference and image envelope separated");
            }
            session.ConfigureDetection(false, ""); pipeline.Frames.Add(new Bitmap(8, 4));
            using (var frame = NextVideoFrame(session))
            { Check(frame.Detections.Count == 0 && frame.Sequence > firstSequence, "Detect off retained old results"); }
            Check(detectors[0].Disposals == 1, "Disabled detector retained native resources");
            session.ConfigureDetection(true, ""); pipeline.Frames.Add(new Bitmap(10, 6));
            using (var frame = NextVideoFrame(session))
            { Check(frame.Detections.Count == 1 && detectors.Count == 2, "Detect on/off/on reused retired state"); }
            session.Stop();
            Check(detectors[1].Disposals == 1, "Stop retained its detector");
        }
        CheckClean(pipeline);
        foreach (bool nativeError in new[] { false, true })
        {
            pipeline = new FakeVideoPipeline();
            var detector = new FakeDetector { Hold = true, NativeCancellationError = nativeError };
            var image = new Bitmap(16, 8);
            using (var session = new VideoSession(() => pipeline, (path, token) => detector))
            {
                session.ConfigureDetection(true, ""); session.Start("fake"); pipeline.Frames.Add(image);
                Wait(detector.Entered, "held inference");
                Wait(Task.Run(() => session.Stop()), "inference stop and join");
                Check(session.TakeFrame() == null && session.Completion.IsCompleted && detector.Disposals == 1,
                    "Cancelled inference published or retained work");
                CheckDisposed(image);
            }
            CheckClean(pipeline);
        }
    }

    private static void VisionFailureKeepsVideo()
    {
        var pipeline = new FakeVideoPipeline();
        var detector = new FakeDetector { Fail = true };
        using (var session = new VideoSession(() => pipeline, (path, token) => detector))
        {
            session.ConfigureDetection(true, ""); session.Start("fake"); pipeline.Frames.Add(new Bitmap(16, 8));
            using (var frame = NextVideoFrame(session))
            {
                Check(frame.Detections.Count == 0 && session.State == VideoState.Streaming && !session.DetectionEnabled &&
                    session.VisionError != null && !session.VisionError.Contains("private-model-path"),
                    "Detector failure stopped video or leaked private diagnostics");
            }
            pipeline.Frames.Add(new Bitmap(10, 5));
            using (var frame = NextVideoFrame(session))
            { Check(frame.Image.Width == 10 && frame.Detections.Count == 0, "Plain video did not continue after model failure"); }
        }
        CheckClean(pipeline);
    }

    private static VideoFrame TestVisionFrame(long sequence, Color color, VideoDetection[] detections,
        long generation = 1, double ageSeconds = 0)
    {
        var image = new Bitmap(100, 80);
        using (var graphics = Graphics.FromImage(image)) { graphics.Clear(color); }
        return new VideoFrame(image, detections, sequence, generation,
            Stopwatch.GetTimestamp() - (long)(ageSeconds * Stopwatch.Frequency), DateTime.UtcNow, "owned fake detector");
    }
    private static VideoDetection Car(float x = 20) => new VideoDetection(2, "car", .9f, new RectangleF(x, 20, 20, 10));

    private static void BoundedTrackingAngles()
    {
        var input = new VideoInputProfile { TrackingCalibrated = true };
        var imageSize = new Size(100, 80);
        var target = new RectangleF(80, 60, 10, 10);
        Check(VideoTrackingAngles.TryStep(input, imageSize, target, 0, 0, .1, out float pitch, out float yaw) &&
            pitch == -1 && yaw == 1, "Tracking rate/sign or pitch/yaw mapping changed");
        Check(VideoTrackingAngles.TryStep(input, imageSize, target, 0, 0, 100, out pitch, out yaw) &&
            pitch == -2 && yaw == 2, "Long frame interval bypassed dt ceiling");
        input.PitchSign = -1; input.YawSign = -1;
        Check(VideoTrackingAngles.TryStep(input, imageSize, target, 0, 0, .1, out pitch, out yaw) &&
            pitch == 1 && yaw == -1, "Reverse calibration signs were ignored");
        input.PitchSign = 1; input.YawSign = 1;
        input.PitchMinDeg = 0; input.YawMaxDeg = 0;
        Check(!VideoTrackingAngles.TryStep(input, imageSize, target, 0, 0, .1, out pitch, out yaw) &&
            pitch == 0 && yaw == 0, "Tracking crossed absolute angle limits");
        input.PitchMinDeg = -90; input.YawMaxDeg = 90;
        Check(!VideoTrackingAngles.TryStep(input, imageSize, new RectangleF(45, 35, 10, 10), 0, 0, .1,
            out pitch, out yaw), "Centered target bypassed deadband");
        foreach (double invalid in new[] { double.NaN, double.PositiveInfinity, -1, 0 })
        { Check(!VideoTrackingAngles.TryStep(input, imageSize, target, 0, 0, invalid, out pitch, out yaw), "Invalid dt admitted tracking"); }
        input.TrackingCalibrated = false;
        Check(!VideoTrackingAngles.TryStep(input, imageSize, target, 0, 0, .1, out pitch, out yaw), "Uncalibrated input moved");
        input.TrackingCalibrated = true; input.PitchMaxDeg = -45;
        Check(!VideoTrackingAngles.TryStep(input, imageSize, target, 0, 0, .1, out pitch, out yaw) && pitch == 0,
            "Out-of-range manual target snapped into narrower calibration limits");
        input.PitchMaxDeg = 90; input.TrackingMaxRateDegSec = 30;
        var smallError = new RectangleF(51, 38, 4, 4);
        float slower = 0, faster = 0;
        for (int index = 0; index < 10; index++)
        { VideoTrackingAngles.TryStep(input, imageSize, smallError, 0, slower, .1, out pitch, out slower); }
        for (int index = 0; index < 20; index++)
        { VideoTrackingAngles.TryStep(input, imageSize, smallError, 0, faster, .05, out pitch, out faster); }
        Check(Math.Abs(slower - faster) < .001 && slower > 3 && slower < 4,
            "The same one-second image error produced frame-rate-dependent control gain");
        Check(VideoTrackingAngles.TryStep(input, imageSize, new RectangleF(45, 35, 10, 10), 0, 0, .1,
            out pitch, out yaw, 90) && pitch == 0 && yaw == -3,
            "Aircraft yaw compensation bypassed the calibrated total rate");
        foreach (float invalid in new[] { float.NaN, float.PositiveInfinity, 181 })
        { Check(!VideoTrackingAngles.TryStep(input, imageSize, target, 0, 0, .1, out pitch, out yaw, invalid),
            "Invalid aircraft rotation admitted a tracking target"); }
    }

    private static void VisionSelectionAndObservation()
    {
        var pipeline = new FakeVideoPipeline();
        var input = new VideoInputProfile { Name = "Owned input", Url = "rtsp://secret.invalid/live" };
        string directory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "observation-" + Guid.NewGuid().ToString("N"));
        var visual = new FakeVisualTracker();
        var detector = new FakeDetector();
        using (var player = new EmbeddedVideoPlayer(input, true, CancellationToken.None, () => pipeline,
            createDetector: (path, token) => detector, createTracker: token => visual = new FakeVisualTracker()))
        {
            player.ObservationDirectory = directory;
            player.StartStream(); Wait(pipeline.ReadEntered, "selection source");
            var session = GetField<VideoSession>(player, "_session");
            PlayerVisionFrame(player, pipeline, Color.Red);
            Check(!player.SelectTargetAt(new Point(30, 5), new Size(100, 100)), "Letterbox margin selected a target");
            Check(player.SelectTargetAt(new Point(30, 35), new Size(100, 100)) && player.SelectedRegion.HasValue &&
                !player.RegionReady, "Generic click did not select a pending region without detection");
            PlayerVisionFrame(player, pipeline, Color.Red);
            Check(player.RegionReady && player.DisplayedVideoFrame.Region.Found && player.SubmittedTrackingCommands == 0,
                "Visual selection failed to lock or moved before explicit Track");
            Check(!player.StartTracking() && !player.IsTracking, "Uncalibrated UI started tracking");
            GimbalController.SetTargetAngles(0, 0);
            Check(player.SelectedRegion == null && player.TrackingStatus.Contains("Manual"), "Manual input retained selection");
            Check(player.SelectRegion(new RectangleF(20, 20, 20, 15)), "Fresh explicit ROI was rejected");
            PlayerVisionFrame(player, pipeline, Color.Red);
            var selected = player.SelectedRegion;
            player.EnableDetection(true); PlayerVisionFrame(player, pipeline, Color.Red);
            player.EnableDetection(false);
            Check(player.RegionReady && player.SelectedRegion == selected, "Detector toggle retired an independent visual target");
            detector.Fail = true; player.EnableDetection(true); PlayerVisionFrame(player, pipeline, Color.Red);
            Check(player.RegionReady && session.VisionError != null && player.SelectedRegion == selected,
                "Optional detector failure retired visual tracking");
            long savedSequence = player.DisplayedVideoFrame.Sequence;
            string path = player.SaveObservation();
            Check(path != null && File.Exists(path) && File.Exists(Path.ChangeExtension(path, ".json")), "Observation pair was not saved");
            var metadata = JObject.Parse(File.ReadAllText(Path.ChangeExtension(path, ".json")));
            Check((long)metadata["Sequence"] == savedSequence && (int)metadata["Width"] == 100 &&
                (string)metadata["SelectedTarget"]["Tracker"] == "OpenCV CSRT" &&
                (float)metadata["SelectedTarget"]["Box"]["X"] == 20 &&
                !(bool)metadata["TargetPosition"]["Available"] && !(bool)metadata["AircraftPosition"]["Available"] &&
                !metadata.ToString().Contains("secret.invalid"), "Observation mismatched frame, target location or secret URL");
            using (var image = new Bitmap(path))
            { Check(image.Width == 100 && image.GetPixel(0, 0).R == 255, "Observation PNG did not match displayed image"); }
            visual.Found = false; pipeline.Frames.Add(SolidVideoImage(Color.Red));
            using (var lost = NextVideoFrame(session))
            { Check(lost.Region != null && !lost.Region.Found, "Visual fixture did not establish authoritative loss"); }
            Tick(player);
            Check(player.SelectedRegion == null && !player.RegionReady,
                "Dropped loss envelope retained a selected ROI or enabled Track");
            player.UpdateInput(input.Copy());
            Check(player.SelectedRegion == null && !player.IsTracking && player.DisplayedVideoFrame == null,
                "Source replacement retained selected identity or display ownership");
            var stale = TestVisionFrame(11, Color.Red, new[] { Car() }, session.Generation, 1);
            session.PublishVideoFrame(session.Generation, stale); Tick(player);
            Check(player.SaveObservation() == null, "Stale observation was saved");
            player.StopStream(); CheckDisposed(stale.Image);
        }
        Check(Path.GetFullPath(directory).StartsWith(Path.GetFullPath(AppDomain.CurrentDomain.BaseDirectory),
            StringComparison.OrdinalIgnoreCase), "Observation test output left its owned directory");
        if (Directory.Exists(directory)) { Directory.Delete(directory, true); }
        Check(VideoGeometry.TryImagePoint(new Point(960, 540), new Size(640, 360), new Size(1920, 1080), out var pixel) &&
            pixel.X == 320 && pixel.Y == 180, "Fullscreen zoom used another coordinate mapping");
    }

    private static void VisionEndAndCalibration()
    {
        var pipeline = new FakeVideoPipeline();
        using (var player = new EmbeddedVideoPlayer(new VideoInputProfile
            { Name = "Owned source", Url = "rtsp://owned.invalid/live" }, true, CancellationToken.None, () => pipeline,
                createTracker: token => new FakeVisualTracker()))
        {
            player.StartStream(); Wait(pipeline.ReadEntered, "EOF source");
            var session = GetField<VideoSession>(player, "_session");
            PlayerVisionFrame(player, pipeline, Color.Red);
            Check(player.SelectTargetAt(new Point(30, 25), new Size(100, 80)), "EOF fixture did not establish selection");
            PlayerVisionFrame(player, pipeline, Color.Red);
            pipeline.Frames.Add(null); Wait(session.Completion, "natural EOF"); Tick(player);
            Check(player.SelectedRegion == null && !player.IsTracking && player.DisplayedVideoFrame == null &&
                !GetField<System.Windows.Forms.Timer>(player, "_frameTimer").Enabled,
                "Natural EOF retained tracking ownership or its timer");
        }
        CheckClean(pipeline);
        foreach (var change in new Action<VideoInputProfile>[]
        {
            value => value.HorizontalFovDeg = 0, value => value.PitchMinDeg = 91,
            value => value.YawMaxDeg = 91, value => value.PitchSign = 0,
            value => value.TrackingMaxRateDegSec = 0, value => value.TrackingMaxRateDegSec = 61,
            value => value.DetectorModelProfilePath = "relative.json"
        })
        {
            var input = new VideoInputProfile { Name = "Owned", Url = "rtsp://owned.invalid/live" };
            change(input);
            ExpectInvalidVision(() => input.Validate());
        }
        foreach (var field in new[] { "TrackingCalibrated", "HorizontalFovDeg", "DetectorModelProfilePath" })
        {
            var document = new JObject { ["VideoInputs"] = new JArray(new JObject { [field] = new JArray() }) };
            ExpectInvalidVision(() => NOMADConfig.ValidateInputDocument(document));
        }
        var config = VideoConfig();
        config.SelectedVideoInput.TrackingCalibrated = true;
        config.SelectedVideoInput.HorizontalFovDeg = 110;
        config.SelectedVideoInput.PitchSign = -1;
        var copy = config.Clone();
        Check(copy.SelectedVideoInput.TrackingCalibrated && copy.SelectedVideoInput.HorizontalFovDeg == 110 &&
            copy.SelectedVideoInput.PitchSign == -1, "Vision calibration was lost in configuration round trip");
        copy.SelectedVideoInput.HorizontalFovDeg = 80;
        Check(config.SelectedVideoInput.HorizontalFovDeg == 110, "Video calibration copy shared operator state");
    }

    private static void ExpectInvalidVision(Action action)
    {
        try { action(); }
        catch (JsonSerializationException) { return; }
        throw new InvalidOperationException("Malformed vision configuration was accepted");
    }
}
