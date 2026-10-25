// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static readonly Size TrackingImageSize = new Size(1280, 720);

    // Project a fixed world ray into a moving camera, independently of the controller's inverse geometry.
    private static PointF ProjectTrackingRay(double pitch, double yaw, double bearing, double targetPitch)
    {
        double p = pitch * Math.PI / 180, y = yaw * Math.PI / 180;
        double b = bearing * Math.PI / 180, t = targetPitch * Math.PI / 180;
        double x = Math.Cos(t) * Math.Cos(b), right = Math.Cos(t) * Math.Sin(b), down = -Math.Sin(t);
        double forward = x * Math.Cos(p) * Math.Cos(y) + right * Math.Cos(p) * Math.Sin(y) - down * Math.Sin(p);
        Check(forward > 0, "Tracked world ray left the camera's forward hemisphere");
        return new PointF((float)((-x * Math.Sin(y) + right * Math.Cos(y)) / forward / 2),
            (float)((x * Math.Sin(p) * Math.Cos(y) + right * Math.Sin(p) * Math.Sin(y) + down * Math.Cos(p)) /
                forward / (2.0 * TrackingImageSize.Height / TrackingImageSize.Width)));
    }

    private static RectangleF TrackingRegion(PointF error, int pitchSign = 1, int yawSign = 1) =>
        new RectangleF((.5f + error.X * yawSign) * TrackingImageSize.Width - 10,
            (.5f + error.Y * pitchSign) * TrackingImageSize.Height - 10, 20, 20);

    private static void TrackingRayGeometry()
    {
        Check(VideoTrackingAngles.TryPredictHeading((float)(170 * Math.PI / 180), 0, 0, 0,
            (float)(40 * Math.PI / 180), .4, out float prediction) &&
            Math.Abs(prediction * 180 / Math.PI + 174) < .001,
            "Fresh angular-rate prediction did not wrap through the heading boundary");
        Check(VideoTrackingAngles.TryPredictHeading(0, (float)(Math.PI / 2), 0, .2f, .8f, .1, out prediction) &&
            Math.Abs(prediction - .02) < .0001, "Body pitch rate at ninety-degree roll did not predict world yaw");
        Check(!VideoTrackingAngles.TryPredictHeading(0, 0, 0, 0, float.NaN, .1, out prediction) &&
            !VideoTrackingAngles.TryPredictHeading(0, 0, 0, 0, 0, .5, out prediction) &&
            !VideoTrackingAngles.TryPredictHeading(0, 0, 2, 0, 0, .1, out prediction),
            "Invalid rates or stale attitude admitted heading prediction");
        foreach (int pitchSign in new[] { -1, 1 })
        foreach (int yawSign in new[] { -1, 1 })
        {
            var profile = new VideoInputProfile { TrackingCalibrated = true, TrackingMaxRateDegSec = 30,
                PitchSign = pitchSign, YawSign = yawSign };
            var error = ProjectTrackingRay(-60, 25, 40, -55);
            VideoTrackingAngles.TryStep(profile, TrackingImageSize, TrackingRegion(error, pitchSign, yawSign),
                -60, 25, .1, out float pitch, out float yaw);
            Check(pitch > -60 && yaw > 25 && Math.Abs(pitch + 60) <= 3 && Math.Abs(yaw - 25) <= 3,
                "Downward-camera correction has wrong direction or exceeds its calibrated rate");
            var after = ProjectTrackingRay(pitch, yaw, 40, -55);
            Check(Math.Abs(after.X) < Math.Abs(error.X) && Math.Abs(after.Y) < Math.Abs(error.Y),
                "Camera-ray correction did not reduce both projected errors");
        }
        var normal = new VideoInputProfile { TrackingCalibrated = true, TrackingMaxRateDegSec = 30 };
        var horizontal = ProjectTrackingRay(-60, 20, 30, -60);
        VideoTrackingAngles.TryStep(normal, TrackingImageSize, TrackingRegion(horizontal), -60, 20, .05,
            out _, out float correctedYaw);
        Check(correctedYaw - 20 > .49f && correctedYaw - 20 <= .51f,
            "Downward-camera azimuth correction still uses the level-camera approximation");
        VideoTrackingAngles.TryStep(normal, TrackingImageSize, new RectangleF(1000, 600, 20, 20), -89, 0, .1,
            out float nearPolePitch, out float nearPoleYaw);
        Check(!float.IsNaN(nearPolePitch) && !float.IsNaN(nearPoleYaw) &&
            Math.Abs(nearPolePitch + 89) <= 3 && Math.Abs(nearPoleYaw) <= 3 && nearPolePitch >= -90,
            "Near-nadir geometry produced nonfinite or unbounded targets");
    }

    private static void DelayedTrackingResponse()
    {
        foreach (float startPitch in new[] { -32f, -60f })
        foreach (int direction in new[] { -1, 1 })
        foreach (int period in new[] { 5, 10 })
        foreach (int translationDirection in new[] { -1, 1 })
        foreach (int delayTicks in new[] { 14, 80 })
        foreach (int attitudePeriod in new[] { 10, 25 })
        { CheckDelayedTracking(startPitch, direction, period, translationDirection, delayTicks, attitudePeriod); }
    }

    private static void CheckDelayedTracking(float startPitch, int direction, int period, int translationDirection,
        int delayTicks, int attitudePeriod)
    {
        var profile = new VideoInputProfile { TrackingCalibrated = true, TrackingMaxRateDegSec = 60 };
        var history = new List<PointF>();
        float commandedPitch = startPitch, commandedYaw = direction * 10;
        double actualPitch = startPitch, actualYaw = commandedYaw, heading = 0, bearing = commandedYaw, previousHeading = 0;
        double reportedHeading = 0;
        double reportedRate = 0;
        int attitudeTick = 0;
        float unapplied = 0;
        double maximum = 0, steady = 0, settled = 0;
        for (int tick = 0; tick < 1000; tick++)
        {
            double time = tick * .01;
            int motion = time >= 1 && time < 4 ? direction : time >= 4 && time < 7 ? -direction : 0;
            heading += motion * 30 * .01;
            if (tick % attitudePeriod == 0)
            { reportedHeading = heading; reportedRate = motion * 30; attitudeTick = tick; }
            bearing += motion * translationDirection * 2 * .01;
            actualPitch += (commandedPitch - actualPitch) * (.01 / .08);
            actualYaw += (commandedYaw - actualYaw) * (.01 / .08);
            PointF image = ProjectTrackingRay(actualPitch, actualYaw + heading, bearing, startPitch);
            history.Add(image);
            double error = Math.Max(Math.Abs(image.X), Math.Abs(image.Y));
            if (time >= 1 && time < 7) { maximum = Math.Max(maximum, error); }
            if ((time >= 3 && time < 4) || (time >= 6 && time < 7)) { steady = Math.Max(steady, error); }
            if (time >= 9) { settled = Math.Max(settled, error); }
            if (tick % period != 0) { continue; }
            // Source/renderer delay precedes receipt; the live decoder still supplies fresh frames.
            PointF delayed = history[Math.Max(0, history.Count - 1 - delayTicks)];
            float previousPitch = commandedPitch, previousYaw = commandedYaw;
            Check(VideoTrackingAngles.TryPredictHeading((float)(reportedHeading * Math.PI / 180), 0, 0, 0,
                (float)(reportedRate * Math.PI / 180), (tick - attitudeTick) * .01, out float currentHeading),
                "Fresh model attitude was rejected");
            float change = VideoTrackingAngles.HeadingChangeDegrees((float)(previousHeading * Math.PI / 180),
                currentHeading) + unapplied;
            float step = (float)(60 * period * .01);
            float compensation = GimbalCommand.Clamp(change, -step, step);
            if (VideoTrackingAngles.TryStep(profile, TrackingImageSize, TrackingRegion(delayed),
                commandedPitch, commandedYaw, period * .01, out commandedPitch, out commandedYaw,
                compensation))
            {
                previousHeading = currentHeading * 180 / Math.PI;
                unapplied = Math.Abs(commandedYaw) == 90 ? 0 : change - compensation;
            }
            Check(Math.Abs(commandedPitch - previousPitch) <= 60 * period * .01 + .0001 &&
                Math.Abs(commandedYaw - previousYaw) <= 60 * period * .01 + .0001,
                "Delayed control bypassed the calibrated target rate");
        }
        string diagnostic = $"pitch={startPitch}, yaw={direction}, period={period}, translation={translationDirection}, " +
            $"delay={delayTicks}, attitude={attitudePeriod}: " +
            $"peak={maximum:F3}, steady={steady:F3}, settled={settled:F3}";
        Check(maximum < .12 && steady < .08 && settled < .035,
            "Delayed moving-aircraft tracking failed its centering/settling bounds: " + diagnostic);
        Console.WriteLine("  " + diagnostic);
    }
}
