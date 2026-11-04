// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Concurrent;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Net;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Threading;
using System.Web.Script.Serialization;
using System.Windows.Forms;
using MissionPlanner.Utilities;
using NOMAD.MissionPlanner;

namespace NOMAD.Simulation
{
    internal sealed partial class NativeFixture
    {
        [DllImport("user32.dll")]
        private static extern uint GetGuiResources(IntPtr process, uint flags);
        private sealed class OwnedForm : Form
        {
            protected override bool ShowWithoutActivation => Environment.GetEnvironmentVariable("NOMAD_SIMULATION_VISIBLE") != "1";
        }

        private sealed class PipelineProbe : IVideoPipeline
        {
            private readonly GStreamerVideoPipeline _pipeline = new GStreamerVideoPipeline();
            internal int Entered, Returned;
            internal long ReadStarted, ReadReturned;
            public void Start(string value, CancellationToken token) => _pipeline.Start(value, token);
            public Bitmap ReadFrame(CancellationToken token, out long receivedTicks, out DateTime receivedUtc)
            {
                Interlocked.Increment(ref Entered); Interlocked.Exchange(ref ReadStarted, System.Diagnostics.Stopwatch.GetTimestamp());
                var image = _pipeline.ReadFrame(token, out receivedTicks, out receivedUtc);
                Interlocked.Increment(ref Returned); Interlocked.Exchange(ref ReadReturned, System.Diagnostics.Stopwatch.GetTimestamp());
                return image;
            }
            public void Dispose() => _pipeline.Dispose();
        }

        private sealed class ImageRecorder : IDisposable
        {
            private readonly BlockingCollection<Tuple<long, Bitmap>> _images = new BlockingCollection<Tuple<long, Bitmap>>(4);
            private readonly Thread _writer;
            private Exception _error;
            internal ImageRecorder(string run)
            {
                string directory = Path.Combine(run, "decoded"); Directory.CreateDirectory(directory);
                _writer = new Thread(() =>
                {
                    foreach (var item in _images.GetConsumingEnumerable())
                    using (item.Item2)
                    {
                        try { item.Item2.Save(Path.Combine(directory, "decoded_" + item.Item1.ToString("D8") + ".png"), ImageFormat.Png); }
                        catch (Exception error) { _error = error; }
                    }
                }) { IsBackground = true, Name = "NOMAD diagnostic PNG recorder" };
                _writer.Start();
            }
            internal void Add(long sequence, Bitmap image)
            {
                var clone = (Bitmap)image.Clone();
                if (!_images.TryAdd(Tuple.Create(sequence, clone))) { clone.Dispose(); }
            }
            public void Dispose()
            {
                _images.CompleteAdding();
                if (!_writer.Join(10000)) { throw new TimeoutException("Diagnostic image recorder deadline"); }
                _images.Dispose();
                if (_error != null) { throw new IOException("Diagnostic image recording failed", _error); }
            }
        }

        private void CheckVideo(string run, bool detect, bool occlusion)
        {
            var scene = new JavaScriptSerializer().Deserialize<System.Collections.Generic.Dictionary<string, object>>(
                File.ReadAllText(Path.Combine(run, "world-manifest.json")));
            var motion = scene.ContainsKey("object_motion")
                ? scene["object_motion"] as System.Collections.Generic.Dictionary<string, object> : null;
            var camera = (System.Collections.Generic.Dictionary<string, object>)scene["camera"];
            int imageWidth = Convert.ToInt32(camera["width"]), imageHeight = Convert.ToInt32(camera["height"]);
            int hfov = (int)Math.Round(Convert.ToDouble(camera["hfov_radians"]) * 180 / Math.PI);
            float[] preset = motion == null ? null : ((System.Collections.IEnumerable)motion["selection_roi"])
                .Cast<object>().Select(value => Convert.ToSingle(value)).ToArray();
            int duration = int.Parse(Environment.GetEnvironmentVariable("NOMAD_SIMULATION_DURATION") ?? "0");
            Require(duration == 0 || (!detect && !occlusion && duration >= 600 && duration <= 1800),
                "invalid_endurance_duration");
            string prefix = Path.Combine(run, "host/media/gstreamer/1.0/x86_64");
            GStreamer.BundledPath = prefix;
            string library = GStreamer.LookForGstreamer();
            Require(string.Equals(Path.GetFullPath(library), Path.GetFullPath(Path.Combine(prefix,
                "bin/libgstreamer-1.0-0.dll")), StringComparison.OrdinalIgnoreCase),
                "media_runtime_provenance_mismatch");
            var stream = _connection.doCommandAsync(201, 1, MAVLink.MAV_CMD.SET_MESSAGE_INTERVAL,
                30, 50000, 0, 0, 0, 0, 0, true);
            Require(stream.Wait(5000) && stream.GetAwaiter().GetResult(), "attitude_stream_request_rejected");
            double initialPitch = motion == null ? -45 : Convert.ToDouble(motion["initial_pitch_deg"]);
            double initialYaw = motion == null ? 0 : Convert.ToDouble(motion["initial_yaw_deg"]);
            Require(Complete(motion == null ? GimbalController.RequestPitchRollTargetAsync(-45, 0) :
                GimbalController.RequestTrackingTargetAsync((float)initialPitch, (float)initialYaw,
                    OutputController.Generation, GimbalController.ManualRevision, () => true)).Submitted,
                "video_initial_pose_not_submitted");
            WaitActualVideoPose(run, -initialPitch * Math.PI / 180, -initialYaw * Math.PI / 180,
                motion == null ? .0035 : .008, motion == null ? 5 : 15); // A large pan must settle at the bounded rate.
            var profile = new VideoInputProfile { Name = "Owned rendered camera", Url = "udp://@:15600",
                TrackingMode = "Area", TrackingCalibrated = true, HorizontalFovDeg = hfov,
                PitchMinDeg = -85, PitchMaxDeg = 30, YawMinDeg = -120, YawMaxDeg = 150,
                TrackingMaxRateDegSec = 20, DropOldFrames = true, QueueMaxBuffers = 1 };
            if (motion != null)
            {
                profile.TrackingMode = "Object"; profile.TrackingMaxRateDegSec = 30;
                // Same profile setting an operator chooses; the embedded model is used, not a folder path.
                if (Environment.GetEnvironmentVariable("NOMAD_SIMULATION_OBJECT_TRACKER") == "GPU")
                { profile.ObjectTracker = "GPU"; }
                profile.YawMinDeg = -160; profile.YawMaxDeg = 160;
            }
            string[] modelPaths = null, modelIdentities = null;
            float[] modelScores = null;
            if (detect)
            {
                VisionRuntime.EnsureLoaded(CancellationToken.None);
                var catalog = new VisionModelCatalog();
                modelPaths = new string[4]; modelIdentities = new string[4];
                modelScores = new float[4];
                for (int index = 0; index < 4; index++)
                {
                    var model = VisionModelProfile.Builtin(); model.Model = VisionRuntime.BuiltinModelPath;
                    model.Name = new[] { "Owned A vehicles", "Owned B car", "Owned C strict car", "Owned D person filter" }[index];
                    if (index > 0) { model.AllowedClasses = new[] { index == 3 ? 0 : 2 }; }
                    if (index == 2) { model.ScoreThreshold = .5f; }
                    modelPaths[index] = catalog.Save(model); modelIdentities[index] = model.Identity;
                    modelScores[index] = model.ScoreThreshold;
                }
                profile.DetectorModelProfilePath = modelPaths[0];
            }
            bool visible = Environment.GetEnvironmentVariable("NOMAD_SIMULATION_VISIBLE") == "1";
            var pipeline = new PipelineProbe();
            AreaVisualTracker area = null;
            CsrtVisualTracker objectTracker = null;
            EfficientTamVisualTracker gpuTracker = null;
            using (var form = new OwnedForm { ShowInTaskbar = visible,
                StartPosition = visible ? FormStartPosition.CenterScreen : FormStartPosition.Manual,
                Location = new Point(-2500, -2500), ClientSize = new Size(900, 700),
                Text = detect ? "NOMAD detector calibration - Gazebo" : "NOMAD tracking test - Gazebo" })
            using (var player = new EmbeddedVideoPlayer(profile, true, CancellationToken.None,
                () => pipeline, _config, createTracker: (token, mode) =>
                {
                    var tracker = VisualTrackerFactory.Create(token, mode, profile.ObjectTracker == "GPU");
                    Volatile.Write(ref area, tracker as AreaVisualTracker);
                    Volatile.Write(ref objectTracker, tracker as CsrtVisualTracker);
                    Volatile.Write(ref gpuTracker, tracker as EfficientTamVisualTracker);
                    return tracker;
                }))
            using (var timer = new System.Windows.Forms.Timer { Interval = 33 })
            using (var preview = new System.Windows.Forms.Timer { Interval = 120000 })
            using (var bridge = new RtpBridge(IPAddress.Loopback, 15761, 15600))
            using (var records = new StreamWriter(Path.Combine(run, "video-frames.jsonl")))
            using (var health = new StreamWriter(Path.Combine(run, "video-health.jsonl")))
            using (var images = new ImageRecorder(run))
            using (var process = System.Diagnostics.Process.GetCurrentProcess())
            {
                player.Dock = DockStyle.Fill; form.Controls.Add(player);
                int stage = 0, verified = 0, eligible = 0, initialCommands = 0, stoppedCommands = 0, width = 0;
                int currentModel = 0, modelFrames = 0, modelFailures = 0, warmFrames = 0;
                int searchFrames = 0, recoveryFrames = 0;
                long pauseTicks = 0, selectedRevision = 0;
                double firstSearch = 0;
                double recoveredAt = 0;
                float maximumConfidence = 0;
                long videoGeneration = -1;
                long sequence = 0;
                double started = _clock.Elapsed.TotalSeconds, action = started;
                double sampled = -1;
                double trackingStarted = 0, windowStarted = 0;
                int windowFrames = 0, windowVerified = 0, windows = 0, savedImages = 0, movingImages = 0;
                int motionDisplayedFrames = 0;
                Exception failure = null;
                var horizontal = new System.Collections.Generic.List<double>();
                var vertical = new System.Collections.Generic.List<double>();
                var json = new JavaScriptSerializer();
                var session = (VideoSession)typeof(EmbeddedVideoPlayer).GetField("_session",
                    BindingFlags.Instance | BindingFlags.NonPublic).GetValue(player);
                preview.Tick += (sender, args) => { preview.Stop(); form.Close(); };
                Action finish = () =>
                {
                    timer.Stop();
                    if (visible && Environment.GetEnvironmentVariable("NOMAD_SIMULATION_PREVIEW") == "1")
                    {
                        player.StopTracking();
                        form.Text = "NOMAD simulation preview - close this window to end";
                        preview.Start();
                    }
                    else { form.Close(); }
                };
                Action stopTracking = () =>
                {
                    var field = typeof(EmbeddedVideoPlayer).GetField("_trackingInFlight",
                        BindingFlags.Instance | BindingFlags.NonPublic);
                    if ((int)field.GetValue(player) != 0) { return; }
                    var toggle = player.Controls.Find("TrackingToggle", true).Cast<Button>().Single();
                    toggle.PerformClick(); stoppedCommands = player.SubmittedTrackingCommands;
                    Require(!player.IsTracking && toggle.Width == width && toggle.Text == "Start tracking",
                        "stop_toggle_state_or_width_failed");
                    stage = 3; action = _clock.Elapsed.TotalSeconds;
                };
                timer.Tick += (sender, args) =>
                {
                    try
                    {
                        process.Refresh();
                        Require(_clock.Elapsed.TotalSeconds - started <
                            (duration > 0 ? duration + 90 : motion != null ? 120 : occlusion ? 40 : 25),
                            "video_deadline_stage_" + stage + ": " + session.State + "/" + session.Error +
                            "; RTP packets=" + bridge.Packets);
                        Require(bridge.Error == null, "rtp_bridge_failed: " + bridge.Error?.Message);
                        var frame = player.DisplayedVideoFrame;
                        if (_clock.Elapsed.TotalSeconds - sampled >= 1)
                        {
                            sampled = _clock.Elapsed.TotalSeconds;
                            TrackingTransition[] transitions;
                            lock (typeof(EmbeddedVideoPlayer).GetField("_visionGate", BindingFlags.Instance |
                                BindingFlags.NonPublic).GetValue(player))
                            {
                                transitions = ((System.Collections.Generic.Queue<TrackingTransition>)
                                    typeof(EmbeddedVideoPlayer).GetField("_trackingEvents", BindingFlags.Instance |
                                        BindingFlags.NonPublic).GetValue(player)).ToArray();
                            }
                            health.WriteLine(json.Serialize(new { elapsed = sampled, stage, bridge.Packets,
                                tracking_seconds = trackingStarted == 0 ? (double?)null : sampled - trackingStarted,
                                process_cpu_ms = process.TotalProcessorTime.TotalMilliseconds,
                                private_bytes = process.PrivateMemorySize64, working_set_bytes = process.WorkingSet64,
                                handle_count = process.HandleCount, gdi_objects = GetGuiResources(process.Handle, 0),
                                user_objects = GetGuiResources(process.Handle, 1),
                                pipeline.Entered, pipeline.Returned, pipeline.ReadStarted, pipeline.ReadReturned,
                                worker_completed = session.Completion.IsCompleted,
                                decoded_sequence = typeof(VideoSession).GetField("_sequence", BindingFlags.Instance |
                                    BindingFlags.NonPublic).GetValue(session), displayed_sequence = frame?.Sequence,
                                receipt_age = frame?.AgeSeconds, tracking = player.IsTracking,
                                events = transitions }));
                            health.Flush();
                        }
                        if (stage == 2 || stage == 3 || stage == 20 || stage == 30 || stage == 40)
                        { Require(frame?.Fresh == true, "active_video_receipt_stale"); }
                        if (frame == null || frame.Sequence == sequence) { return; }
                        sequence = frame.Sequence;
                        var aim = frame.Region?.AimPoint;
                        var bounds = frame.Region?.Bounds;
                        var objectState = Volatile.Read(ref objectTracker);
                        var objectSize = objectState?.ProcessingImageSize ?? Size.Empty;
                        Pwm nativeOutput;
                        lock (_gate) { nativeOutput = _outputs.LastOrDefault(); }
                        records.WriteLine(json.Serialize(new { sequence, decoded_received_ticks = frame.ReceivedTicks,
                            displayed_ticks = System.Diagnostics.Stopwatch.GetTimestamp(),
                            frequency = System.Diagnostics.Stopwatch.Frequency, frame.Generation, frame.DetectionRevision,
                            process_cpu_ms = process.TotalProcessorTime.TotalMilliseconds,
                            private_bytes = process.PrivateMemorySize64, working_set_bytes = process.WorkingSet64,
                            handle_count = process.HandleCount,
                            frame.DetectorIdentity, stage, state = frame.Region?.State.ToString(),
                            status = frame.Region?.Status,
                            tracking_status = player.TrackingStatus,
                            commanded_pitch = player.CommandedPitchDeg, commanded_yaw = player.CommandedYawDeg,
                            motion_current = frame.Motion?.IsCurrent,
                            heading_relative = frame.Motion?.Heading?.RelativeDegrees,
                            latest_worker_timing = Volatile.Read(ref area)?.LastTiming,
                            latest_object_diagnostic = objectState?.LastObjectDiagnostic ??
                                Volatile.Read(ref gpuTracker)?.LastDiagnostic,
                            object_tracker_provider = Volatile.Read(ref gpuTracker)?.Provider ?? "CPU",
                            latest_worker_diagnostics_frame_bound = false,
                            latest_object_processing_width = objectSize.Width,
                            latest_object_processing_height = objectSize.Height,
                            region_revision = frame.Region?.Revision, tracking = player.IsTracking,
                            request_ticks = typeof(EmbeddedVideoPlayer).GetField("_lastRequestTicks",
                                BindingFlags.Instance | BindingFlags.NonPublic).GetValue(player),
                            detections = frame.Detections.Select(value => new { value.ClassId, value.Confidence,
                                x = value.Bounds.X, y = value.Bounds.Y, width = value.Bounds.Width, height = value.Bounds.Height }),
                            verified = frame.Region?.Found == true, aim_x = aim?.X, aim_y = aim?.Y,
                            region_bounds = bounds.HasValue ? new[] { bounds.Value.X, bounds.Value.Y,
                                bounds.Value.Width, bounds.Value.Height } : null,
                            native_pitch_pwm = nativeOutput?.Pitch ?? 0, native_yaw_pwm = nativeOutput?.Yaw ?? 0,
                            submitted = player.SubmittedTrackingCommands, fingerprint = Fingerprint(frame.Image) }));
                        records.Flush();
                        if (stage == 40) { motionDisplayedFrames++; }
                        bool saveMotion = stage == 40 && movingImages < 120;
                        bool save = motion != null ? (stage < 40 && savedImages < 6) || saveMotion :
                            duration == 0 ? sequence <= 120 : savedImages < 120 && (sequence <= 60 || sequence % 600 == 0);
                        if (save)
                        { images.Add(sequence, frame.Image); savedImages++; if (stage == 40) { movingImages++; } }
                        if (stage == 1 && (!frame.Fresh || !player.RegionReady)) { warmFrames = 0; }
                        if (stage <= 1 && !frame.Fresh) { return; }
                        Require(frame.Fresh, "displayed_frame_is_receipt_stale");
                        if (stage == 0)
                        {
                            Require(frame.Image.Width == imageWidth && frame.Image.Height == imageHeight,
                                "wrong_rendered_resolution: expected " + imageWidth + "x" + imageHeight);
                            if (detect)
                            {
                                videoGeneration = frame.Generation;
                                player.EnableDetection(true); stage = 10; action = _clock.Elapsed.TotalSeconds; return;
                            }
                            Require(player.SelectRegion(motion == null ? new RectangleF(360, 180, 144, 144) :
                                new RectangleF(preset[0], preset[1], preset[2], preset[3])), "initial_region_selection_failed");
                            stage = 1; action = _clock.Elapsed.TotalSeconds;
                        }
                        else if (stage == 10)
                        {
                            Require(frame.Generation == videoGeneration, "model_switch_restarted_video");
                            Require(frame.DetectorIdentity == "" || frame.DetectorIdentity == modelIdentities[currentModel],
                                "old_model_result_reached_current_frame");
                            if (frame.DetectorIdentity != modelIdentities[currentModel]) { return; }
                            maximumConfidence = Math.Max(maximumConfidence, frame.Detections.Where(value => value.ClassId == 2)
                                .Select(value => value.Confidence).DefaultIfEmpty(0).Max());
                            if (currentModel < 3)
                            {
                                if (frame.Detections.Any(value => value.ClassId == 2 && value.Confidence >= modelScores[currentModel]))
                                { modelFrames++; }
                            }
                            else
                            {
                                Require(frame.Detections.Count == 0, "excluded_rendered_class_was_accepted");
                                modelFrames++;
                            }
                            if (modelFrames < 3 && _clock.Elapsed.TotalSeconds - action < 5) { return; }
                            bool modelPassed = modelFrames >= 3;
                            if (!modelPassed) { modelFailures++; }
                            _cases.Add(new { name = "rendered_named_model", identity = frame.DetectorIdentity,
                                matching_frames = modelFrames, detections = frame.Detections.Count, passed = modelPassed,
                                score_threshold = modelScores[currentModel], maximum_car_confidence = maximumConfidence });
                            if (++currentModel == 4)
                            {
                                Require(modelFailures == 0, "rendered_named_model_positive_failures_" + modelFailures);
                                stage = 4; finish(); return;
                            }
                            modelFrames = 0; player.UpdateDetectionModel(modelPaths[currentModel]);
                            maximumConfidence = 0;
                            action = _clock.Elapsed.TotalSeconds;
                        }
                        else if (stage == 1 && player.RegionReady)
                        {
                            // One explicit Start after cold initialization; never auto-rearm a stale/retired intent.
                            warmFrames = frame.AgeSeconds <= .2 ? warmFrames + 1 : 0;
                            if (warmFrames < 3) { return; }
                            var toggle = player.Controls.Find("TrackingToggle", true).Cast<Button>().Single();
                            width = toggle.Width; initialCommands = player.SubmittedTrackingCommands;
                            toggle.PerformClick();
                            Require(player.IsTracking && toggle.Text == "Stop tracking", "tracking_toggle_failed");
                            selectedRevision = frame.Region.Revision;
                            videoGeneration = frame.Generation;
                            stage = 2; action = _clock.Elapsed.TotalSeconds;
                            trackingStarted = action;
                        }
                        else if (stage == 2)
                        {
                            Require(player.IsTracking, "tracking_intent_disarmed: " + player.TrackingStatus);
                            eligible++;
                            if (frame.Region?.Found == true)
                            {
                                verified++;
                                if (_clock.Elapsed.TotalSeconds - action >= 3 && aim.HasValue)
                                {
                                    horizontal.Add(Math.Abs(aim.Value.X - imageWidth / 2.0));
                                    vertical.Add(Math.Abs(aim.Value.Y - imageHeight / 2.0));
                                }
                            }
                            if (_clock.Elapsed.TotalSeconds - action < 5) { return; }
                            Require(eligible >= 40, "displayed_rate_below_8_fps: frames=" + eligible +
                                "; seconds=" + (_clock.Elapsed.TotalSeconds - action));
                            Require(verified >= eligible * .95, "positive_rendered_coverage_failed: " + verified + "/" + eligible);
                            // An initially centered Object can hold still; its later body disturbance must drive new output.
                            Require(motion != null || player.SubmittedTrackingCommands > initialCommands,
                                "video_never_submitted_steering");
                            Require(horizontal.Count > 0 && Percentile(horizontal, .95) <= imageWidth * .035 &&
                                Percentile(vertical, .95) <= imageHeight * .035, "image_centering_failed");
                            if (motion != null)
                            {
                                File.WriteAllText(Path.Combine(run, "object-motion.start.tmp"), Path.GetFileName(run));
                                File.Move(Path.Combine(run, "object-motion.start.tmp"), Path.Combine(run, "object-motion.start"));
                                initialCommands = player.SubmittedTrackingCommands;
                                stage = 40; action = _clock.Elapsed.TotalSeconds;
                            }
                            else if (duration > 0)
                            {
                                long last = Directory.GetFiles(Path.Combine(run, "frames"), "capture_*.json")
                                    .Select(path => long.Parse(Path.GetFileNameWithoutExtension(path).Substring(8)))
                                    .DefaultIfEmpty(-1).Max();
                                Require(last >= 0, "endurance_prefix_capture_missing");
                                File.WriteAllText(Path.Combine(run, "oracle-prefix.json"), json.Serialize(new {
                                    last_sequence = last + 30, scope = "initial five-second steering slice; three-second capture drain" }));
                                stage = 30; windowStarted = trackingStarted;
                                windowFrames = eligible; windowVerified = verified;
                            }
                            else if (occlusion)
                            {
                                File.WriteAllText(Path.Combine(run, "occluder.start.tmp"), Path.GetFileName(run));
                                File.Move(Path.Combine(run, "occluder.start.tmp"), Path.Combine(run, "occluder.start"));
                                stage = 20; action = _clock.Elapsed.TotalSeconds;
                            }
                            else { stopTracking(); }
                        }
                        else if (stage == 40)
                        {
                            Require(player.IsTracking && frame.Generation == videoGeneration &&
                                frame.Region?.Revision == selectedRevision, "object_motion_retired_selection_or_intent");
                            long request = (long)typeof(EmbeddedVideoPlayer).GetField("_lastRequestTicks",
                                BindingFlags.Instance | BindingFlags.NonPublic).GetValue(player);
                            if (frame.Region?.Found != true)
                            {
                                if (pauseTicks == 0) { pauseTicks = System.Diagnostics.Stopwatch.GetTimestamp(); }
                                Require(request <= pauseTicks, "unverified_object_admitted_new_request");
                            }
                            else { pauseTicks = 0; }
                            if (!File.Exists(Path.Combine(run, "object-motion-complete.json"))) { return; }
                            if (recoveredAt == 0) { recoveredAt = _clock.Elapsed.TotalSeconds; }
                            if (_clock.Elapsed.TotalSeconds - recoveredAt < 1) { return; }
                            Require(player.SubmittedTrackingCommands > initialCommands + 5,
                                "object_motion_controller_did_not_submit_updates");
                            _cases.Add(new { name = "rendered_object_body_motion", motion,
                                qualification = "requires independent semantic mask and actual body/joint oracle" });
                            stopTracking();
                        }
                        else if (stage == 30)
                        {
                            Require(player.IsTracking && frame.Generation == videoGeneration &&
                                frame.Region?.Revision == selectedRevision, "endurance_intent_or_generation_changed");
                            windowFrames++; if (frame.Region?.Found == true) { windowVerified++; }
                            double span = _clock.Elapsed.TotalSeconds - windowStarted;
                            if (span >= 60)
                            {
                                Require(windowFrames / span >= 8 && windowVerified >= .95 * windowFrames,
                                    "endurance_window_failed: fps=" + windowFrames / span + "; verified=" +
                                    windowVerified + "/" + windowFrames);
                                windows++; windowFrames = 0; windowVerified = 0; windowStarted = _clock.Elapsed.TotalSeconds;
                            }
                            if (_clock.Elapsed.TotalSeconds - trackingStarted < duration) { return; }
                            span = _clock.Elapsed.TotalSeconds - windowStarted;
                            if (span >= 5)
                            { Require(windowFrames / span >= 8 && windowVerified >= .95 * windowFrames,
                                "endurance_final_window_failed"); }
                            _cases.Add(new { name = "sustained_area_tracking", duration_seconds = duration,
                                completed_windows = windows, no_reselection = true,
                                accuracy_scope = "independent pixel oracle covers initial slice; long segment is stability/resource evidence" });
                            stopTracking();
                        }
                        else if (stage == 20)
                        {
                            Require(player.IsTracking && player.SelectedRegion.HasValue &&
                                frame.Region?.Revision == selectedRevision, "occlusion_retired_identity_or_intent");
                            long request = (long)typeof(EmbeddedVideoPlayer).GetField("_lastRequestTicks",
                                BindingFlags.Instance | BindingFlags.NonPublic).GetValue(player);
                            if (frame.Region?.Found != true)
                            {
                                if (firstSearch == 0) { firstSearch = _clock.Elapsed.TotalSeconds; }
                                if (pauseTicks == 0) { pauseTicks = System.Diagnostics.Stopwatch.GetTimestamp(); }
                                Require(request <= pauseTicks, "unverified_area_admitted_new_request");
                                searchFrames++; recoveryFrames = 0; recoveredAt = 0;
                            }
                            else
                            {
                                pauseTicks = 0;
                                if (searchFrames > 0 && ++recoveryFrames == 3) { recoveredAt = _clock.Elapsed.TotalSeconds; }
                            }
                            Require(_clock.Elapsed.TotalSeconds - action < 15, "occlusion_recovery_deadline");
                            if (searchFrames < 5 || recoveryFrames < 3 || _clock.Elapsed.TotalSeconds - action < 6)
                            { return; }
                            Require(File.Exists(Path.Combine(run, "occluder-ack.json")), "occluder_never_observed_trigger");
                            Require(recoveredAt - firstSearch <= 5, "displayed_visual_recovery_exceeded_5_seconds");
                            _cases.Add(new { name = "occlusion_visual_recovery", searchFrames, recoveryFrames,
                                unverified_host_seconds = recoveredAt - firstSearch,
                                identity_retained = true, no_new_unverified_requests = true,
                                visibility_truth = "required independent capture/box oracle" });
                            stopTracking();
                        }
                        else if (stage == 3)
                        {
                            Require(!player.IsTracking && player.SubmittedTrackingCommands == stoppedCommands,
                                "stop_automatically_rearmed_or_admitted_input");
                            if (_clock.Elapsed.TotalSeconds - action < .7) { return; }
                            using (var image = (Bitmap)frame.Image.Clone())
                            { image.Save(Path.Combine(run, "rendered-final.png"), ImageFormat.Png); }
                            _cases.Add(new { name = motion == null ? "rendered_area_ui" : "rendered_object_ui", eligible, verified,
                                horizontal_p95 = Percentile(horizontal, .95), vertical_p95 = Percentile(vertical, .95),
                                source_clock_alignment = "requires independent offline oracle", tracking_width = width });
                            stage = 4; finish();
                        }
                    }
                    catch (Exception error)
                    {
                        if (failure == null) { failure = error; }
                        finish();
                    }
                };
                form.Shown += (sender, args) => { player.StartStream(); timer.Start(); };
                Application.Run(form);
                timer.Stop();
                if (failure != null) { throw failure; }
                Require(stage == 4, "rendered_form_closed_before_case_completed");
            }
        }

        private static double Percentile(System.Collections.Generic.List<double> values, double percentile)
        {
            var sorted = values.OrderBy(value => value).ToArray();
            return sorted[(int)Math.Ceiling((sorted.Length - 1) * percentile)];
        }

        private void WaitActualVideoPose(string run, double expectedPitch = Math.PI / 4, double expectedYaw = 0,
            double tolerance = .0035, double maximumHostSeconds = 5)
        {
            double deadline = _clock.Elapsed.TotalSeconds + maximumHostSeconds;
            long firstStable = -1;
            while (_clock.Elapsed.TotalSeconds < deadline)
            {
                string path = Path.Combine(run, "joints.csv");
                using (var input = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite))
                using (var reader = new StreamReader(input))
                {
                    string last = null, line;
                    while ((line = reader.ReadLine()) != null) { if (line.Split(',').Length == 7) { last = line; } }
                    if (last != null && !last.StartsWith("simulation"))
                    {
                        var values = last.Split(',').Select(value => double.Parse(value,
                            System.Globalization.CultureInfo.InvariantCulture)).ToArray();
                        bool stable = Math.Abs(values[1]) < tolerance && Math.Abs(values[2] - expectedPitch) < tolerance &&
                            Math.Abs(values[3] - expectedYaw) < tolerance;
                        if (!stable) { firstStable = -1; }
                        else if (firstStable < 0) { firstStable = (long)values[0]; }
                        else if (values[0] - firstStable >= 300000000) { return; }
                    }
                }
                Thread.Sleep(20);
            }
            throw new TimeoutException("Actual simulated mount did not settle before selection.");
        }

        private static string Fingerprint(Bitmap image)
        {
            var bits = image.LockBits(new Rectangle(0, 0, image.Width, image.Height), ImageLockMode.ReadOnly,
                PixelFormat.Format32bppArgb);
            try
            {
                var pixels = new byte[bits.Stride * bits.Height]; Marshal.Copy(bits.Scan0, pixels, 0, pixels.Length);
                var text = new System.Text.StringBuilder(6144);
                for (int y = 0; y < 48; y++) for (int x = 0; x < 64; x++)
                {
                    int offset = ((y * 10 + 5) * image.Height / 480) * bits.Stride +
                        ((x * 10 + 5) * image.Width / 640) * 4;
                    int gray = (77 * pixels[offset + 2] + 150 * pixels[offset + 1] + 29 * pixels[offset]) >> 8;
                    text.Append(gray.ToString("x2"));
                }
                return text.ToString();
            }
            finally { image.UnlockBits(bits); }
        }
    }
}
