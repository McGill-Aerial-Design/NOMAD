// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;

namespace NOMAD.MissionPlanner
{
    internal static class VideoTrackingAngles
    {
        internal static bool TryStep(VideoInputProfile calibration, Size image, RectangleF target,
            float currentPitch, float currentYaw, double dt, out float pitch, out float yaw)
        {
            pitch = currentPitch; yaw = currentYaw;
            if (!calibration.TrackingCalibrated || image.Width <= 0 || image.Height <= 0 ||
                double.IsNaN(dt) || double.IsInfinity(dt) || dt <= 0 ||
                !VideoDetection.Finite(currentPitch) || !VideoDetection.Finite(currentYaw) ||
                currentPitch < calibration.PitchMinDeg || currentPitch > calibration.PitchMaxDeg ||
                currentYaw < calibration.YawMinDeg || currentYaw > calibration.YawMaxDeg) { return false; }
            double x = (target.X + target.Width / 2 - image.Width / 2.0) / image.Width;
            double y = (target.Y + target.Height / 2 - image.Height / 2.0) / image.Height;
            double focal = image.Width / (2 * Math.Tan(calibration.HorizontalFovDeg * Math.PI / 360));
            double yawError = Math.Abs(x) < .02 ? 0 : Math.Atan(x * image.Width / focal) * 180 / Math.PI;
            double pitchError = Math.Abs(y) < .02 ? 0 : -Math.Atan(y * image.Height / focal) * 180 / Math.PI;
            double elapsed = Math.Min(dt, .2);
            double step = calibration.TrackingMaxRateDegSec * elapsed;
            // A one-per-second error response keeps the control gain independent of frame rate.
            pitch = GimbalCommand.Clamp(currentPitch + (float)Math.Max(-step, Math.Min(step,
                pitchError * elapsed * calibration.PitchSign)), calibration.PitchMinDeg, calibration.PitchMaxDeg);
            yaw = GimbalCommand.Clamp(currentYaw + (float)Math.Max(-step, Math.Min(step,
                yawError * elapsed * calibration.YawSign)), calibration.YawMinDeg, calibration.YawMaxDeg);
            return Math.Abs(pitch - currentPitch) >= .01 || Math.Abs(yaw - currentYaw) >= .01;
        }
    }
}
