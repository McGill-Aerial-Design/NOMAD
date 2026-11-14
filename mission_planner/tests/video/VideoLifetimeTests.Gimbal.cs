// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Threading;
using System.Windows.Forms;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static ServoGimbalAxis PitchServo() => new ServoGimbalAxis { Axis = "Pitch", Channel = 14 };

    private static void ServoGimbalConfiguration()
    {
        var config = VideoConfig();
        config.SelectedVideoInput.ServoGimbalAxes.Add(PitchServo());
        var clone = config.Clone();
        config.SelectedVideoInput.ServoGimbalAxes[0].MaximumPwm = 1900;
        Check(clone.SelectedVideoInput.ServoGimbalAxes[0].MaximumPwm == 2000,
            "Cloning profiles shared mutable gimbal calibration");
        var inputCopy = config.SelectedVideoInput.Copy();
        inputCopy.ServoGimbalAxes[0].Channel = 15;
        Check(config.SelectedVideoInput.ServoGimbalAxes[0].Channel == 14, "Video view copied a live gimbal binding by reference");
        var axis = PitchServo();
        Check(axis.TryPwm(-90, out int low) && low == 1000 && axis.TryPwm(90, out int high) && high == 2000 &&
            !axis.TryPwm(double.NaN, out _) && !axis.TryPwm(91, out _), "Servo gimbal lost its calibration bounds");
        axis.Reversed = true;
        Check(axis.TryPwm(-90, out low) && low == 2000 && axis.TryAngle(2000, out float angle) && angle == -90,
            "Gimbal direction or PWM-to-target conversion did not follow reversal");
        foreach (Action<ServoGimbalAxis> invalid in new Action<ServoGimbalAxis>[]
        {
            value => value.Channel = 0, value => value.Channel = 17,
            value => value.MinimumPwm = 499, value => value.MaximumPwm = 2501,
            value => value.MaximumPwm = value.MinimumPwm, value => value.MinimumAngleDeg = double.NaN,
            value => value.MinimumAngleDeg = 10, value => value.MaximumAngleDeg = 181,
        })
        {
            var bad = PitchServo(); invalid(bad); bool rejected = false;
            try { ServoGimbalAxis.Validate(new[] { bad }); } catch (JsonException) { rejected = true; }
            Check(rejected, "An invalid servo calibration was accepted");
        }
        foreach (string json in new[] { "null", "[{}]", "[{\"Axis\":true}]" })
        {
            bool rejected = false;
            try { ServoGimbalAxis.Parse(JToken.Parse(json)); } catch (JsonException) { rejected = true; }
            Check(rejected, "Malformed or incomplete servo gimbal JSON was accepted");
        }
        using (var editor = new ServoGimbalEditorForm(clone.SelectedGimbalAxes))
        {
            var grid = GetField<System.Windows.Controls.Grid>(editor, "_axes");
            T Cell<T>(int row, int column) where T : System.Windows.UIElement => grid.Children.OfType<T>().Single(
                item => System.Windows.Controls.Grid.GetRow(item) == row && System.Windows.Controls.Grid.GetColumn(item) == column);
            Cell<System.Windows.Controls.CheckBox>(2, 0).IsChecked = true;
            Cell<System.Windows.Controls.TextBox>(2, 2).Text = "15";
            Cell<System.Windows.Controls.CheckBox>(3, 0).IsChecked = true;
            Cell<System.Windows.Controls.TextBox>(3, 2).Text = "16";
            Cell<System.Windows.Controls.CheckBox>(3, 7).IsChecked = true;
            var edited = editor.BuildCandidate();
            Check(edited.Count == 3 && edited[2].Axis == "Roll" && edited[2].Reversed &&
                clone.SelectedGimbalAxes.Count == 1, "Gimbal editor did not keep its draft independent");
            Cell<System.Windows.Controls.TextBox>(2, 2).Text = "14"; bool rejected = false;
            try { editor.BuildCandidate(); } catch (JsonException) { rejected = true; }
            Check(rejected, "Two gimbal axes shared one servo output");
            Cell<System.Windows.Controls.TextBox>(2, 2).Text = "15";
            editor.ShowInTaskbar = false; editor.Location = new Point(-2000, -2000); editor.Show();
            using (var image = new Bitmap(editor.Width, editor.Height))
            { editor.DrawToBitmap(image, new Rectangle(Point.Empty, image.Size));
              image.Save(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "servo-gimbal-editor.png")); }
            editor.Close();
        }
        var yawOnly = new VideoInputProfile { TrackingCalibrated = true, ServoGimbalAxes = new List<ServoGimbalAxis>
            { new ServoGimbalAxis { Axis = "Yaw", Channel = 15 } } };
        Check(VideoTrackingAngles.TryStep(yawOnly, new Size(100, 80), new PointF(80, 20), 0, 0, .1,
            out float pitch, out float yaw) && pitch == 0 && yaw > 0,
            "Single-axis tracking moved an unconfigured axis");
    }

    private static void ServoGimbalStreaming()
    {
        using (var peer = new TrackingPeer { ServoAckDelayMs = 80, ServoAckLimit = 2 })
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), ClientSize = new Size(900, 600) })
        {
            var input = new VideoInputProfile { Name = "Streaming servo", Url = "udp://@:5600",
                TrackingCalibrated = true, TrackingMaxRateDegSec = 60, ServoGimbalAxes = new List<ServoGimbalAxis>
                { PitchServo(), new ServoGimbalAxis { Axis = "Yaw", Channel = 15 } } };
            peer.Config.VideoInputs.Add(input); peer.Config.SelectedVideoInputName = input.Name;
            GimbalController.ApplyProfile(peer.Config);
            var pipeline = new FakeVideoPipeline();
            FakeVisualTracker tracker = null;
            using (var player = new EmbeddedVideoPlayer(input, true, CancellationToken.None, () => pipeline, peer.Config,
                createTracker: (token, mode) => tracker = new FakeVisualTracker { AimPoint = new PointF(80, 20) }))
            {
                form.Controls.Add(player); form.Show(); player.StartStream();
                Wait(pipeline.ReadEntered, "streaming servo source");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.SelectRegion(new RectangleF(60, 10, 20, 20)), "Streaming servo selection failed");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.StartTracking(), "Streaming servo refused tracking");
                peer.Pump(40); PlayerVisionFrame(player, pipeline, Color.Blue);
                peer.Until(() => player.SubmittedTrackingCommands > 0, "initial acknowledged servo pose");
                int commands = peer.ServoTargets.Length;
                int frames = 0;
                var clock = Stopwatch.StartNew();
                while (clock.ElapsedMilliseconds < 1200)
                {
                    PlayerVisionFrame(player, pipeline, Color.Blue);
                    frames++;
                    peer.Pump(1, precise: true);
                    while (clock.Elapsed.TotalSeconds < frames / 30.0) { peer.Pump(1, precise: true); }
                }
                double hz = (peer.ServoTargets.Length - commands) / 2.0 / clock.Elapsed.TotalSeconds;
                double inputHz = frames / clock.Elapsed.TotalSeconds;
                Check(player.IsTracking && hz >= 25 && hz <= 34,
                    $"Native servo stream stalled: {hz:F1} Hz from {inputHz:F1} FPS; {player.TrackingStatus}");
                Console.WriteLine($"  Native MP servo stream without update ACKs: {hz:F1} Hz from {inputHz:F1} FPS (two axes)");
                var busy = typeof(OutputController).GetField("_payloadBusy", BindingFlags.NonPublic | BindingFlags.Static);
                var before = GetField<ServoTrackingController>(player, "_servoTrackingController");
                commands = peer.ServoTargets.Length;
                try
                {
                    busy.SetValue(null, true);
                    for (int frame = 0; frame < 5; frame++)
                    { peer.Pump(34); PlayerVisionFrame(player, pipeline, Color.Blue); }
                    Check(peer.ServoTargets.Length == commands &&
                        before.Equals(GetField<ServoTrackingController>(player, "_servoTrackingController")),
                        "Busy servo requests advanced PID state or dispatched a queued target");
                }
                finally { busy.SetValue(null, false); }
                tracker.State = VisualTrackingState.Searching;
                peer.Pump(34); PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.IsTracking && !player.RegionReady &&
                    GetField<ServoTrackingController>(player, "_servoTrackingController").Equals(default(ServoTrackingController)),
                    "Target loss did not pause servo steering and discard its accumulated controller state");
                tracker.State = VisualTrackingState.Verified; tracker.AimPoint = new PointF(50, 40);
                PlayerVisionFrame(player, pipeline, Color.Blue);
                float heldPitch = player.CommandedPitchDeg, heldYaw = player.CommandedYawDeg;
                peer.Pump(34); PlayerVisionFrame(player, pipeline, Color.Blue);
                peer.Until(() => peer.ServoTargets.Length > commands, "fresh centered pose after servo recovery");
                Check(player.CommandedPitchDeg == heldPitch && player.CommandedYawDeg == heldYaw,
                    "Servo recovery released a stored PID correction on a centered target");
                player.StopTracking(); commands = peer.ServoTargets.Length;
                peer.Pump(34); PlayerVisionFrame(player, pipeline, Color.Blue); peer.Pump(80);
                Check(!player.IsTracking && peer.ServoTargets.Length == commands,
                    "Stopped servo tracking replayed an old position request");
                player.StopStream();
            }
        }
    }

    private static void ServoGimbalTracking()
    {
        using (var peer = new TrackingPeer())
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), ClientSize = new Size(900, 600) })
        {
            var input = new VideoInputProfile { Name = "Servo camera", Url = "udp://@:5600", TrackingCalibrated = true,
                ServoGimbalAxes = new List<ServoGimbalAxis> { PitchServo(), new ServoGimbalAxis { Axis = "Yaw", Channel = 15 } } };
            peer.Config.VideoInputs.Add(input); peer.Config.SelectedVideoInputName = input.Name;
            GimbalController.ApplyProfile(peer.Config);
            peer.Config.ActuatorDefinitions = new JArray(new JObject
            {
                ["id"] = "camera", ["name"] = "Camera pitch", ["behavior"] = "ServoPosition", ["channel"] = 14,
                ["safe_pwm"] = 1000, ["hazardous"] = false, ["confirmation_count"] = 0,
            });
            PayloadController.ApplyProfile(peer.Config);
            var pipeline = new FakeVideoPipeline();
            using (var player = new EmbeddedVideoPlayer(input, true, CancellationToken.None, () => pipeline, peer.Config,
                createTracker: (token, mode) => new FakeVisualTracker { AimPoint = new PointF(80, 20) }))
            using (var panel = new ActuatorControlPanel(peer.Config) { Dock = DockStyle.Bottom, Height = 100 })
            {
                form.Controls.Add(player); form.Controls.Add(panel); form.Show(); player.StartStream();
                Wait(pipeline.ReadEntered, "servo gimbal video source");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                Check(player.SelectRegion(new RectangleF(60, 10, 20, 20)), "Servo camera selection failed");
                PlayerVisionFrame(player, pipeline, Color.Blue);
                var previousHeartbeat = peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.HEARTBEAT);
                var previousAttitude = peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.ATTITUDE);
                peer.Heartbeat(4); peer.Attitude(0);
                peer.Until(() => !ReferenceEquals(previousHeartbeat,
                    peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.HEARTBEAT)) &&
                    !ReferenceEquals(previousAttitude,
                    peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.ATTITUDE)),
                    "fresh selected heartbeat and attitude before servo tracking");
                Check(player.StartTracking(), "Calibrated servo gimbal refused tracking: " + player.TrackingStatus);
                Thread.Sleep(60); PlayerVisionFrame(player, pipeline, Color.Blue);
                peer.Until(() => peer.ServoTargets.Length >= 2, "pitch/yaw tracking servo commands");
                Check(peer.ServoTargets.Any(value => value.Channel == 14) && peer.ServoTargets.Any(value => value.Channel == 15),
                    "Tracking did not use both mapped servo axes");
                var slider = FindPayloadControl<System.Windows.Controls.Slider>(panel, "camera:slider");
                slider.Value = 1700;
                Check(!player.IsTracking, "Shared payload slider did not retire automatic gimbal tracking immediately");
                peer.Until(() => PayloadController.Current.TargetPwm("camera") == 1700, "manual shared-servo target");
                int commands = peer.ServoTargets.Length;
                Thread.Sleep(60); PlayerVisionFrame(player, pipeline, Color.Blue); peer.Pump(100);
                Check(peer.ServoTargets.Length == commands && !player.IsTracking,
                    "Automatic tracking resumed after the payload slider override");
                player.StopStream();
            }
            PayloadController.Shutdown();
        }
    }
}
