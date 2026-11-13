// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    // Visual feedback controls position setpoints; neither PWM nor submitted angles measure the servo shaft.
    internal struct ServoTrackingController
    {
        private Axis _pitch, _yaw;

        internal bool TryStep(VideoInputProfile calibration, Size image, PointF target,
            float currentPitch, float currentYaw, double dt, out float pitch, out float yaw,
            float aircraftYawChange = 0, bool movingObject = false)
        {
            pitch = currentPitch; yaw = currentYaw;
            if (!VideoTrackingAngles.TryErrors(calibration, image, target, currentPitch, 0,
                    out double pitchError, out double yawError) ||
                double.IsNaN(dt) || double.IsInfinity(dt) || dt <= 0 || !VideoDetection.Finite(currentYaw) ||
                !VideoDetection.Finite(aircraftYawChange) || Math.Abs(aircraftYawChange) > 180 ||
                currentYaw < calibration.YawMinDeg || currentYaw > calibration.YawMaxDeg ||
                calibration.TrackingMaxRateDegSec < 1 || calibration.TrackingMaxRateDegSec > 60) { return false; }
            var pitchAxis = calibration.ServoGimbalAxes.FirstOrDefault(axis => axis.Axis == "Pitch");
            var yawAxis = calibration.ServoGimbalAxes.FirstOrDefault(axis => axis.Axis == "Yaw");
            if (pitchAxis == null && yawAxis == null) { return false; }
            // A late frame cannot release a large catch-up position step.
            double elapsed = Math.Min(dt, .1);
            double delay = (calibration.LatencyMs + calibration.CameraDelayEstimateMs) / 1000.0;
            double gain = Math.Min(3, .6 / (Math.Max(0, delay) + .15));
            double deadband = Math.Min(.2, Math.Atan(2 * Math.Tan(calibration.HorizontalFovDeg * Math.PI / 360) *
                .003 * Math.Min(1, (double)image.Height / image.Width)) * 180 / Math.PI);
            if (movingObject && yawError * aircraftYawChange > 0)
            { aircraftYawChange *= (float)Math.Max(0, 1 - Math.Abs(yawError) / 5); }
            if (pitchAxis != null)
            {
                pitch = _pitch.Step(pitchError, currentPitch, elapsed, gain, deadband, calibration.TrackingMaxRateDegSec, 0,
                    Math.Max(calibration.PitchMinDeg, pitchAxis.MinimumAngleDeg),
                    Math.Min(calibration.PitchMaxDeg, pitchAxis.MaximumAngleDeg));
            }
            if (yawAxis != null)
            {
                yaw = _yaw.Step(yawError, currentYaw, elapsed, gain, deadband, calibration.TrackingMaxRateDegSec,
                    -aircraftYawChange / dt, Math.Max(calibration.YawMinDeg, yawAxis.MinimumAngleDeg),
                    Math.Min(calibration.YawMaxDeg, yawAxis.MaximumAngleDeg));
            }
            return true;
        }

        private struct Axis
        {
            private bool _initialized;
            private double _previousError, _derivative, _integral, _velocity;

            internal float Step(double error, float position, double dt, double gain, double deadband, double maxRate,
                double feedForward, double minimum, double maximum)
            {
                // A continuous angular deadband avoids a PWM jump at the old two-percent image threshold.
                error = Math.Sign(error) * Math.Max(0, Math.Abs(error) - deadband);
                if (!_initialized) { _previousError = error; _initialized = true; }
                _derivative += dt / (.12 + dt) * ((error - _previousError) / dt - _derivative);
                if (error * _previousError <= 0 || Math.Abs(error) > 8) { _integral = 0; }
                _previousError = error;
                double integralGain = gain * .2;
                double proposedIntegral = Clamp(_integral + error * dt, -maxRate / (4 * integralGain),
                    maxRate / (4 * integralGain));
                double demand = gain * error + integralGain * proposedIntegral + .1 * _derivative + feedForward;
                if (Math.Abs(demand) <= maxRate || error * demand < 0) { _integral = proposedIntegral; }
                double velocity = Clamp(demand, -maxRate, maxRate);
                _velocity += Clamp(velocity - _velocity, -maxRate * 6 * dt, maxRate * 6 * dt);
                double target = Clamp(position + _velocity * dt, minimum, maximum);
                if ((target <= minimum && _velocity < 0) || (target >= maximum && _velocity > 0))
                { _integral = 0; _velocity = 0; }
                return (float)target;
            }

            private static double Clamp(double value, double minimum, double maximum) =>
                Math.Max(minimum, Math.Min(maximum, value));
        }
    }
}
