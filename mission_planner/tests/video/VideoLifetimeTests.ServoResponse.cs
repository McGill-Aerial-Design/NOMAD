// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void ServoTrackingStepResponse()
    {
        var settling = new double[2];
        for (int run = 0; run < 2; run++)
        {
            var profile = ServoResponseProfile();
            var controller = new ServoTrackingController();
            var history = new List<PointF>();
            float yaw = 0;
            double actual = 0;
            int nextFrame = 0, previousFrame = 0, frame = 0;
            for (int tick = 0; tick < 600; tick++)
            {
                int pwm = (int)Math.Round(1500 + yaw / .18);
                double physical = (pwm - 1500) * .18;
                actual += Math.Max(-90, Math.Min(90, (physical - actual) / .12)) * .01;
                history.Add(ProjectTrackingRay(0, actual, 20, 0));
                if (Math.Abs(20 - actual) >= 2) { settling[run] = (tick + 1) * .01; }
                if (tick != nextFrame) { continue; }
                double dt = tick == 0 ? 1.0 / 30 : (tick - previousFrame) * .01;
                previousFrame = tick; nextFrame += ++frame % 3 == 0 ? 4 : 3;
                PointF target = TrackingRegion(history[Math.Max(0, tick - 10)]);
                if (run == 0)
                { controller.TryStep(profile, TrackingImageSize, target, 0, yaw, dt, out _, out yaw); }
                else
                { VideoTrackingAngles.TryStep(profile, TrackingImageSize, target, 0, yaw, dt, out _, out yaw); }
            }
        }
        Check(settling[0] < 1.2 && settling[0] < settling[1] * .65,
            $"Servo feedback did not improve step settling: PID={settling[0]:F2}s, previous={settling[1]:F2}s");
        Console.WriteLine($"  Twenty-degree PWM step, stays within two degrees: PID={settling[0]:F2}s, " +
            $"previous={settling[1]:F2}s (100ms image delay, 120ms shaft lag)");
    }

    private static void ServoTrackingResponse()
    {
        double worstSettled = 0;
        int cases = 0;
        foreach (int delayMs in new[] { 80, 250, 800 })
        foreach (double scale in new[] { .8, 1.2 })
        foreach (double lag in new[] { .08, .25 })
        foreach (bool reversed in new[] { false, true })
        {
            var profile = ServoResponseProfile();
            profile.CameraDelayEstimateMs = delayMs;
            profile.ServoGimbalAxes.ForEach(axis => axis.Reversed = reversed);
            var controller = new ServoTrackingController();
            float pitch = -35, yaw = 0;
            double actualPitch = pitch, actualYaw = yaw, previousHeading = 0;
            int pitchPwm = 1500, yawPwm = 1500, frame = 0, previousFrame = 0, nextFrame = 0;
            var images = new List<PointF>();
            double settled = 0, peak = 0;
            for (int tick = 0; tick < 2400; tick++)
            {
                double time = tick * .01;
                double heading = time < 4 ? 0 : time < 6 ? (time - 4) * 12 : time < 8 ? (8 - time) * 12 : 0;
                double bearing = time < 1 ? 0 : 10 + Math.Max(0, Math.Min(4, time - 3)) * 3;
                double targetPitch = time < 1 ? -35 : -30 + Math.Max(0, Math.Min(4, time - 3));
                // Independent PWM plant: deadband, calibration error, finite shaft speed and mechanical lag.
                profile.ServoGimbalAxes[0].TryPwm(pitch, out int wantedPitch);
                profile.ServoGimbalAxes[1].TryPwm(yaw, out int wantedYaw);
                if (Math.Abs(wantedPitch - pitchPwm) > 3) { pitchPwm = wantedPitch; }
                if (Math.Abs(wantedYaw - yawPwm) > 3) { yawPwm = wantedYaw; }
                double sign = reversed ? -1 : 1;
                double physicalPitch = sign * (pitchPwm - 1500) * .18 * scale;
                double physicalYaw = sign * (yawPwm - 1500) * .18 * scale;
                actualPitch += Math.Max(-90, Math.Min(90, (physicalPitch - actualPitch) / lag)) * .01;
                actualYaw += Math.Max(-90, Math.Min(90, (physicalYaw - actualYaw) / lag)) * .01;
                PointF error = ProjectTrackingRay(actualPitch, actualYaw + heading, bearing, targetPitch);
                images.Add(error);
                double magnitude = Math.Max(Math.Abs(error.X), Math.Abs(error.Y));
                peak = Math.Max(peak, magnitude);
                if (time >= 20) { settled = Math.Max(settled, magnitude); }
                if (tick != nextFrame) { continue; }
                double dt = tick == 0 ? .033 : (tick - previousFrame) * .01;
                previousFrame = tick;
                nextFrame += ++frame % 3 == 0 ? 4 : 3;
                var delayed = images[Math.Max(0, tick - delayMs / 10)];
                // One-pixel measurement noise, below the continuous angular deadband.
                delayed.X += (frame % 2 == 0 ? 1 : -1) / (float)TrackingImageSize.Width;
                var target = TrackingRegion(delayed);
                float beforePitch = pitch, beforeYaw = yaw;
                Check(controller.TryStep(profile, TrackingImageSize, target, pitch, yaw, dt, out pitch, out yaw,
                    (float)(heading - previousHeading)), "Servo feedback rejected a valid delayed image");
                previousHeading = heading;
                Check(Math.Abs(pitch - beforePitch) <= 60 * dt + .0001 &&
                    Math.Abs(yaw - beforeYaw) <= 60 * dt + .0001,
                    "Servo feedback exceeded the calibrated setpoint speed");
            }
            string diagnostic = $"delay={delayMs}ms, scale={scale}, lag={lag}, reversed={reversed}, " +
                $"peak={peak:F3}, settled={settled:F3}";
            Check(peak < .3 && settled < .015, "Delayed PWM servo failed to center: " + diagnostic);
            worstSettled = Math.Max(worstSettled, settled); cases++;
        }
        Console.WriteLine($"  Servo PWM plant: {cases} moving/turning cases; worst settled image error={worstSettled:P2}");
    }

    private static VideoInputProfile ServoResponseProfile() => new VideoInputProfile
    {
        TrackingCalibrated = true, TrackingMaxRateDegSec = 60,
        ServoGimbalAxes = new List<ServoGimbalAxis>
        {
            new ServoGimbalAxis { Axis = "Pitch", Channel = 14 },
            new ServoGimbalAxis { Axis = "Yaw", Channel = 15 },
        },
    };

    private static void ServoTrackingLimitsAndReset()
    {
        var profile = ServoResponseProfile();
        profile.YawMinDeg = -10; profile.YawMaxDeg = 10;
        var controller = new ServoTrackingController();
        float pitch = 0, yaw = 0;
        double previousRate = 0;
        for (int frame = 0; frame < 180; frame++)
        {
            float previous = yaw;
            Check(controller.TryStep(profile, TrackingImageSize, new PointF(1100, 360), pitch, yaw, 1.0 / 30,
                out pitch, out yaw), "Servo feedback rejected a calibrated bounded target");
            double rate = (yaw - previous) * 30;
            Check(yaw <= 10 && yaw >= -10 && Math.Abs(rate) <= 60.001,
                "Servo feedback escaped travel or speed limits");
            if (yaw < 10) { Check(Math.Abs(rate - previousRate) <= 12.001, "Servo velocity jumped on a fresh image"); }
            previousRate = rate;
        }
        Check(yaw == 10, "Servo travel-limit fixture did not actually saturate");
        controller.TryStep(profile, TrackingImageSize, new PointF(400, 360), pitch, yaw, 1.0 / 30,
            out pitch, out yaw);
        Check(yaw < 10, "Saturated servo accumulated integral windup instead of reversing immediately");
        var resumed = new ServoTrackingController();
        resumed.TryStep(profile, TrackingImageSize, new PointF(640, 360), pitch, yaw, .5, out _, out float held);
        Check(held == yaw, "A reset controller released stored velocity or integral on a centered target");
        resumed.TryStep(profile, TrackingImageSize, new PointF(1100, 360), pitch, yaw, 20, out _, out float bounded);
        Check(Math.Abs(bounded - yaw) <= 6.001, "A delayed servo sample released an unbounded catch-up step");
        foreach (double invalid in new[] { double.NaN, double.PositiveInfinity, 0, -.01 })
        {
            Check(!resumed.TryStep(profile, TrackingImageSize, new PointF(640, 360), pitch, yaw, invalid, out _, out _),
                "Servo feedback accepted an invalid sample interval");
        }
        Check(!resumed.TryStep(profile, TrackingImageSize, new PointF(float.NaN, 360), pitch, yaw, .033, out _, out _),
            "Servo feedback accepted a nonfinite image error");
        resumed = default; profile.HorizontalFovDeg = 5;
        Check(resumed.TryStep(profile, TrackingImageSize, new PointF(653, 360), 0, 0, .033, out _, out float zoomYaw) &&
            zoomYaw > 0, "Servo deadband suppressed a one-percent image error at narrow FOV");
        profile.ServoGimbalAxes.RemoveAt(0);
        Check(resumed.TryStep(profile, TrackingImageSize, new PointF(800, 100), pitch, yaw, .033,
            out float unconfiguredPitch, out _) && unconfiguredPitch == pitch,
            "Single-axis servo feedback moved the unconfigured tilt axis");
    }
}
