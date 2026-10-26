// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;

namespace NOMAD.MissionPlanner
{
    // These are camera estimates and search hints, never evidence that a target was visually found.
    internal sealed class GroundCameraEstimate
    {
        internal object Epoch { get; }
        internal double Latitude { get; }
        internal double Longitude { get; }
        internal double AltitudeMslMeters { get; }
        internal double HomeAltitudeMslMeters { get; }
        internal double PitchDegrees { get; }
        internal double RollDegrees { get; }
        internal double HeadingDegrees { get; }
        internal long CaptureTicks { get; }
        internal string Source { get; }

        internal GroundCameraEstimate(object epoch, double latitude, double longitude, double altitudeMsl,
            double homeAltitudeMsl, double pitch, double roll, double heading, long captureTicks, string source)
        {
            Epoch = epoch; Latitude = latitude; Longitude = longitude; AltitudeMslMeters = altitudeMsl;
            HomeAltitudeMslMeters = homeAltitudeMsl; PitchDegrees = pitch; RollDegrees = roll;
            HeadingDegrees = heading; CaptureTicks = captureTicks; Source = source;
        }
    }

    internal sealed class GroundAreaAnchor
    {
        internal object Epoch { get; }
        internal double Latitude { get; }
        internal double Longitude { get; }
        internal double AltitudeMslMeters { get; }
        internal GroundAreaAnchor(object epoch, double latitude, double longitude, double altitudeMsl)
        { Epoch = epoch; Latitude = latitude; Longitude = longitude; AltitudeMslMeters = altitudeMsl; }
    }

    internal static class GroundProjection
    {
        private const double EarthRadiusMeters = 6378137, Radians = Math.PI / 180, MaximumRangeMeters = 5000;

        internal static bool TryCreateAnchor(VideoInputProfile calibration, Size image, PointF pixel,
            GroundCameraEstimate camera, double planeHeightAboveHomeMeters, out GroundAreaAnchor anchor)
        {
            anchor = null;
            if (!Valid(calibration, image, camera) || !Finite(pixel.X) || !Finite(pixel.Y) ||
                pixel.X < 0 || pixel.Y < 0 || pixel.X > image.Width || pixel.Y > image.Height ||
                !Finite(planeHeightAboveHomeMeters) || Math.Abs(planeHeightAboveHomeMeters) > 10000) { return false; }
            double height = camera.AltitudeMslMeters - camera.HomeAltitudeMslMeters - planeHeightAboveHomeMeters;
            if (height < 1 || height > MaximumRangeMeters) { return false; }
            double focal = image.Width / (2 * Math.Tan(calibration.HorizontalFovDeg * Radians / 2));
            double right = (pixel.X - image.Width / 2.0) / focal * calibration.YawSign;
            double down = (pixel.Y - image.Height / 2.0) / focal * calibration.PitchSign;
            Rotate(camera, 1, right, down, out double north, out double east, out double vertical);
            double length = Math.Sqrt(north * north + east * east + vertical * vertical);
            if (vertical / length < Math.Sin(5 * Radians)) { return false; }
            double distance = height / vertical;
            north *= distance; east *= distance;
            if (Math.Sqrt(north * north + east * east) > MaximumRangeMeters) { return false; }
            double latitude = camera.Latitude + north / EarthRadiusMeters / Radians;
            double longitude = Wrap(camera.Longitude + east /
                (EarthRadiusMeters * Math.Cos(camera.Latitude * Radians)) / Radians);
            if (Math.Abs(latitude) > 85) { return false; }
            anchor = new GroundAreaAnchor(camera.Epoch, latitude, longitude,
                camera.HomeAltitudeMslMeters + planeHeightAboveHomeMeters);
            return true;
        }

        internal static bool TryProjectAnchor(VideoInputProfile calibration, Size image, GroundAreaAnchor anchor,
            GroundCameraEstimate camera, out PointF pixel)
        {
            pixel = PointF.Empty;
            if (!Valid(calibration, image, camera) || anchor == null || !ReferenceEquals(anchor.Epoch, camera.Epoch) ||
                !Finite(anchor.Latitude) || Math.Abs(anchor.Latitude) > 85 || !Finite(anchor.Longitude) ||
                Math.Abs(anchor.Longitude) > 180 || !Finite(anchor.AltitudeMslMeters)) { return false; }
            double north = (anchor.Latitude - camera.Latitude) * Radians * EarthRadiusMeters;
            double east = Wrap(anchor.Longitude - camera.Longitude) * Radians * EarthRadiusMeters *
                Math.Cos(camera.Latitude * Radians);
            double down = camera.AltitudeMslMeters - anchor.AltitudeMslMeters;
            if (Math.Sqrt(north * north + east * east) > MaximumRangeMeters || Math.Abs(down) > MaximumRangeMeters)
            { return false; }
            Rotate(camera, 1, 0, 0, out double fn, out double fe, out double fd);
            Rotate(camera, 0, 1, 0, out double rn, out double re, out double rd);
            Rotate(camera, 0, 0, 1, out double dn, out double de, out double dd);
            double forward = north * fn + east * fe + down * fd;
            if (forward <= .1) { return false; }
            double focal = image.Width / (2 * Math.Tan(calibration.HorizontalFovDeg * Radians / 2));
            double x = image.Width / 2.0 + focal * (north * rn + east * re + down * rd) /
                forward * calibration.YawSign;
            double y = image.Height / 2.0 + focal * (north * dn + east * de + down * dd) /
                forward * calibration.PitchSign;
            if (!Finite(x) || !Finite(y) || Math.Abs(x) > 1e7 || Math.Abs(y) > 1e7) { return false; }
            pixel = new PointF((float)x, (float)y);
            return true;
        }

        private static bool Valid(VideoInputProfile calibration, Size image, GroundCameraEstimate camera) =>
            calibration?.TrackingCalibrated == true && image.Width > 0 && image.Height > 0 &&
            calibration.HorizontalFovDeg >= 5 && calibration.HorizontalFovDeg <= 170 &&
            Math.Abs(calibration.PitchSign) == 1 && Math.Abs(calibration.YawSign) == 1 &&
            camera?.Epoch != null && Finite(camera.Latitude) && Math.Abs(camera.Latitude) <= 85 &&
            Finite(camera.Longitude) && Math.Abs(camera.Longitude) <= 180 &&
            Finite(camera.AltitudeMslMeters) && Finite(camera.HomeAltitudeMslMeters) &&
            Finite(camera.PitchDegrees) && Math.Abs(camera.PitchDegrees) <= 90 &&
            Finite(camera.RollDegrees) && Math.Abs(camera.RollDegrees) <= 180 && Finite(camera.HeadingDegrees);

        private static void Rotate(GroundCameraEstimate camera, double forward, double right, double down,
            out double north, out double east, out double vertical)
        {
            double roll = camera.RollDegrees * Radians, pitch = camera.PitchDegrees * Radians;
            double yaw = camera.HeadingDegrees * Radians;
            double rolledRight = Math.Cos(roll) * right - Math.Sin(roll) * down;
            double rolledDown = Math.Sin(roll) * right + Math.Cos(roll) * down;
            double horizontal = Math.Cos(pitch) * forward + Math.Sin(pitch) * rolledDown;
            vertical = -Math.Sin(pitch) * forward + Math.Cos(pitch) * rolledDown;
            north = Math.Cos(yaw) * horizontal - Math.Sin(yaw) * rolledRight;
            east = Math.Sin(yaw) * horizontal + Math.Cos(yaw) * rolledRight;
        }

        internal static double Wrap(double degrees) => Math.Atan2(Math.Sin(degrees * Radians),
            Math.Cos(degrees * Radians)) / Radians;
        internal static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    internal sealed class VideoHeadingEstimate
    {
        internal float RelativeDegrees { get; }
        internal string Source { get; }
        internal bool Estimated => true;
        internal long ReceivedTicks { get; }
        internal VideoHeadingEstimate(float relativeDegrees, string source, long receivedTicks)
        { RelativeDegrees = relativeDegrees; Source = source; ReceivedTicks = receivedTicks; }
        internal PointF RingOffset => RingPoint(RelativeDegrees);
        internal static PointF RingPoint(double degrees) => new PointF((float)Math.Sin(degrees * Math.PI / 180),
            (float)-Math.Cos(degrees * Math.PI / 180));
    }
}
