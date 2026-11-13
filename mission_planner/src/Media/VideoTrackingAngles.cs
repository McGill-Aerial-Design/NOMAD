// SPDX-License-Identifier: Apache-2.0
using System;
using System.Linq;
using System.Drawing;

namespace NOMAD.MissionPlanner
{
    internal static class VideoTrackingAngles
    {
        internal static bool TryPredictHeading(float yaw, float roll, float pitch, float pitchRate, float yawRate,
            double age, out float heading)
        {
            heading = yaw;
            if (age < 0 || age >= .5 || double.IsNaN(age) || !VideoDetection.Finite(yaw) ||
                Math.Abs(yaw) > (float)Math.PI || !VideoDetection.Finite(roll) || !VideoDetection.Finite(pitch) ||
                Math.Abs(roll) > (float)Math.PI || Math.Abs(pitch) > (float)(Math.PI / 2) ||
                !VideoDetection.Finite(pitchRate) || !VideoDetection.Finite(yawRate)) { return false; }
            double cosine = Math.Cos(pitch);
            // Body gyro rates predict Euler yaw between messages; avoid extrapolating at its pitch singularity.
            if (Math.Abs(cosine) > .1)
            {
                double rate = (Math.Sin(roll) * pitchRate + Math.Cos(roll) * yawRate) / cosine;
                double predicted = yaw + rate * age;
                heading = (float)Math.Atan2(Math.Sin(predicted), Math.Cos(predicted));
            }
            return true;
        }

        internal static float HeadingChangeDegrees(float previousRadians, float currentRadians) =>
            (float)(Math.Atan2(Math.Sin(currentRadians - previousRadians), Math.Cos(currentRadians - previousRadians)) *
                180 / Math.PI);
        internal static bool TryStep(VideoInputProfile calibration, Size image, PointF target,
            float currentPitch, float currentYaw, double dt, out float pitch, out float yaw,
            float aircraftYawChange = 0, bool movingObject = false)
        {
            pitch = currentPitch; yaw = currentYaw;
            if (!TryErrors(calibration, image, target, currentPitch, .02, out double pitchError, out double yawError) ||
                double.IsNaN(dt) || double.IsInfinity(dt) || dt <= 0 || !VideoDetection.Finite(currentYaw) ||
                !VideoDetection.Finite(aircraftYawChange) || Math.Abs(aircraftYawChange) > 180 ||
                currentYaw < calibration.YawMinDeg || currentYaw > calibration.YawMaxDeg) { return false; }
            double elapsed = Math.Min(dt, .2);
            double step = calibration.TrackingMaxRateDegSec * elapsed;
            double pitchGain = movingObject ? 2 : 1;
            // An Object's bearing can move with the aircraft; do not counter yaw against verified image motion.
            if (movingObject && yawError * aircraftYawChange > 0)
            { aircraftYawChange *= (float)Math.Max(0, 1 - Math.Abs(yawError) / 5); }
            // Native mount yaw is body-relative; counter aircraft rotation without amplifying delayed image error.
            pitch = GimbalCommand.Clamp(currentPitch + (float)Math.Max(-step, Math.Min(step,
                pitchError * pitchGain * elapsed)), calibration.PitchMinDeg, calibration.PitchMaxDeg);
            yaw = GimbalCommand.Clamp(currentYaw + (float)Math.Max(-step, Math.Min(step,
                yawError * elapsed - aircraftYawChange)), calibration.YawMinDeg, calibration.YawMaxDeg);
            if (calibration.ServoGimbalAxes.Count > 0)
            {
                if (!calibration.ServoGimbalAxes.Any(axis => axis.Axis == "Pitch")) { pitch = currentPitch; }
                if (!calibration.ServoGimbalAxes.Any(axis => axis.Axis == "Yaw")) { yaw = currentYaw; }
            }
            return Math.Abs(pitch - currentPitch) >= .01 || Math.Abs(yaw - currentYaw) >= .01;
        }

        internal static bool TryErrors(VideoInputProfile calibration, Size image, PointF target,
            float currentPitch, double deadband, out double pitchError, out double yawError)
        {
            pitchError = yawError = 0;
            if (calibration == null || !calibration.TrackingCalibrated || image.Width <= 0 || image.Height <= 0 ||
                !VideoDetection.Finite(target.X) || !VideoDetection.Finite(target.Y) ||
                !VideoDetection.Finite(currentPitch) || currentPitch < calibration.PitchMinDeg ||
                currentPitch > calibration.PitchMaxDeg || calibration.HorizontalFovDeg <= 0 ||
                calibration.HorizontalFovDeg >= 180) { return false; }
            double x = (target.X - image.Width / 2.0) / image.Width;
            double y = (target.Y - image.Height / 2.0) / image.Height;
            double focal = image.Width / (2 * Math.Tan(calibration.HorizontalFovDeg * Math.PI / 360));
            double right = Math.Abs(x) < deadband ? 0 : x * image.Width / focal * calibration.YawSign;
            double down = Math.Abs(y) < deadband ? 0 : y * image.Height / focal * calibration.PitchSign;
            double cameraPitch = currentPitch * Math.PI / 180;
            // Rotate the camera ray into mount axes; image X is not azimuth at a downward pitch.
            double forward = Math.Cos(cameraPitch) + Math.Sin(cameraPitch) * down;
            double vertical = -Math.Sin(cameraPitch) + Math.Cos(cameraPitch) * down;
            yawError = Math.Atan2(right, forward) * 180 / Math.PI;
            pitchError = Math.Atan2(-vertical, Math.Sqrt(forward * forward + right * right)) *
                180 / Math.PI - currentPitch;
            return true;
        }
    }
}
