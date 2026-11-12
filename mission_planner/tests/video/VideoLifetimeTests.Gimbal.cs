// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
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
            var grid = GetField<DataGridView>(editor, "_axes");
            grid.Rows[1].Cells["Use"].Value = true; grid.Rows[1].Cells["Channel"].Value = 15;
            grid.Rows[2].Cells["Use"].Value = true; grid.Rows[2].Cells["Channel"].Value = 16;
            grid.Rows[2].Cells["Reverse"].Value = true;
            var edited = editor.BuildCandidate();
            Check(edited.Count == 3 && edited[2].Axis == "Roll" && edited[2].Reversed &&
                clone.SelectedGimbalAxes.Count == 1, "Gimbal editor did not keep its draft independent");
            grid.Rows[1].Cells["Channel"].Value = 14; bool rejected = false;
            try { editor.BuildCandidate(); } catch (JsonException) { rejected = true; }
            Check(rejected, "Two gimbal axes shared one servo output");
            grid.Rows[1].Cells["Channel"].Value = 15;
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
                Check(player.StartTracking(), "Calibrated servo gimbal refused tracking: " + player.TrackingStatus);
                Thread.Sleep(60); PlayerVisionFrame(player, pipeline, Color.Blue);
                peer.Until(() => peer.ServoTargets.Length >= 2, "pitch/yaw tracking servo commands");
                Check(peer.ServoTargets.Any(value => value.Channel == 14) && peer.ServoTargets.Any(value => value.Channel == 15),
                    "Tracking did not use both mapped servo axes");
                var slider = panel.Controls.Find("camera:slider", true).OfType<TrackBar>().Single();
                slider.Value = 1700;
                typeof(TrackBar).GetMethod("OnScroll", BindingFlags.Instance | BindingFlags.NonPublic)
                    .Invoke(slider, new object[] { EventArgs.Empty });
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
