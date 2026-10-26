// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.Threading;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void GroundProjectionGeometry()
    {
        var calibration = new VideoInputProfile { TrackingCalibrated = true, HorizontalFovDeg = 90 };
        var image = new Size(640, 480);
        var epoch = new object();
        var camera = new GroundCameraEstimate(epoch, 45, 179.9999, 200, 100, -45, 0, 0, 1, "test");
        Check(GroundProjection.TryCreateAnchor(calibration, image, new PointF(320, 240), camera, 0, out var anchor),
            "A downward calibrated center ray failed to intersect the HOME-height plane");
        double expectedLatitude = 45 + 100 / 6378137.0 * 180 / Math.PI;
        Check(Math.Abs(anchor.Latitude - expectedLatitude) < 1e-9 &&
            Math.Abs(anchor.Longitude - camera.Longitude) < 1e-9 && anchor.AltitudeMslMeters == 100,
            "Center ray did not hit independently calculated 100 m north / HOME altitude");
        Check(GroundProjection.TryProjectAnchor(calibration, image, anchor, camera, out var pixel) &&
            Distance(pixel, new PointF(320, 240)) < .001, "Stationary anchor projection failed");
        double longitudeShift = 100 / (6378137.0 * Math.Cos(45 * Math.PI / 180)) * 180 / Math.PI;
        var eastCamera = new GroundCameraEstimate(epoch, 45, GroundProjection.Wrap(camera.Longitude + longitudeShift),
            200, 100, -45, 0, 0, 2, "test");
        Check(GroundProjection.TryProjectAnchor(calibration, image, anchor, eastCamera, out pixel) &&
            Math.Abs(pixel.X - (320 - 320 / Math.Sqrt(2))) < .01 && Math.Abs(pixel.Y - 240) < .01,
            "Eastward translation across the dateline did not move the anchor left by its independent projection");
        foreach (int sign in new[] { -1, 1 })
        {
            calibration.PitchSign = calibration.YawSign = sign;
            var tilted = new GroundCameraEstimate(epoch, 45, 0, 200, 100, -60, 20, 135, 3, "test");
            var selected = new PointF(430, 290);
            Check(GroundProjection.TryCreateAnchor(calibration, image, selected, tilted, 20, out var tiltedAnchor) &&
                GroundProjection.TryProjectAnchor(calibration, image, tiltedAnchor, tilted, out pixel) &&
                Distance(selected, pixel) < .01, "Rolled, mirrored, offset-plane projection failed");
        }
        calibration.PitchSign = calibration.YawSign = 1;
        var nadir = new GroundCameraEstimate(epoch, 45, 0, 200, 100, -90, 0, 0, 4, "test");
        Check(GroundProjection.TryCreateAnchor(calibration, image, new PointF(320, 240), nadir, 0, out var nadirAnchor) &&
            Math.Abs(nadirAnchor.Latitude - 45) < 1e-9, "Nadir added aircraft pitch or displaced the center ray");
        foreach (double pitch in new[] { -1.0, 0, 45, double.NaN })
        {
            var invalid = new GroundCameraEstimate(epoch, 45, 0, 200, 100, pitch, 0, 0, 5, "test");
            Check(!GroundProjection.TryCreateAnchor(calibration, image, new PointF(320, 240), invalid, 0, out _),
                "Shallow/upward/nonfinite ray was accepted: " + pitch);
        }
        var changed = new GroundCameraEstimate(new object(), 45, 0, 200, 100, -45, 0, 0, 6, "test");
        Check(!GroundProjection.TryProjectAnchor(calibration, image, anchor, changed, out _),
            "A changed home/source epoch reused an old ground anchor");
        var behind = new GroundCameraEstimate(epoch, 45, 179.9999, 200, 100, 45, 0, 180, 7, "test");
        Check(!GroundProjection.TryProjectAnchor(calibration, image, anchor, behind, out _),
            "An anchor behind the camera projected into the image");
        var offscreen = new GroundAreaAnchor(epoch, expectedLatitude, GroundProjection.Wrap(camera.Longitude +
            3 * longitudeShift), 100);
        Check(GroundProjection.TryProjectAnchor(calibration, image, offscreen, camera, out pixel) && pixel.X > image.Width,
            "Finite offscreen prediction was discarded");
        var pole = new GroundCameraEstimate(epoch, 89, 0, 200, 100, -45, 0, 0, 8, "test");
        Check(!GroundProjection.TryCreateAnchor(calibration, image, new PointF(320, 240), pole, 0, out _),
            "Near-pole tangent estimate was accepted");
    }

    private static double Distance(PointF first, PointF second) =>
        Math.Sqrt(Math.Pow(first.X - second.X, 2) + Math.Pow(first.Y - second.Y, 2));

    private static void HeadingReportedFrames()
    {
        var position = new MAVLink.mavlink_global_position_int_t
        { lat = 450000000, lon = 100000000, alt = 100000, vx = 2000, vy = 1000, vz = -500 };
        Check(VideoMotionHistory.TryPredictPosition(position, .25, out double latitude, out double longitude,
            out double altitude) && Math.Abs((latitude - 45) * Math.PI / 180 * 6378137 - 5) < 1e-6 &&
            Math.Abs((longitude - 10) * Math.PI / 180 * 6378137 * Math.Cos(Math.PI / 4) - 2.5) < 1e-6 &&
            Math.Abs(altitude - 101.25) < 1e-9, "Zero-delay constant velocity prediction has incorrect units or signs");
        Check(!VideoMotionHistory.TryPredictPosition(position, .5, out _, out _, out _),
            "Position prediction extrapolated stale velocity");
        var expected = new[] { new PointF(0, -1), new PointF(1, 0), new PointF(0, 1), new PointF(-1, 0) };
        for (int index = 0; index < expected.Length; ++index)
        { Check(Distance(VideoHeadingEstimate.RingPoint(index * 90), expected[index]) < .001, "Heading ring sign error"); }
        foreach (ushort flags in new ushort[] { 0, 16, 32, 48, 64, 80 })
        {
            var report = new MAVLink.mavlink_gimbal_device_attitude_status_t
            { flags = flags, q = new[] { (float)Math.Sqrt(.5), 0, 0, (float)Math.Sqrt(.5) } };
            Check(VideoMotionHistory.TryReportedPose(report, out double pitch, out double roll, out double yaw,
                out bool earth) && Math.Abs(yaw - 90) < .001 && Math.Abs(pitch) < .001 && Math.Abs(roll) < .001 &&
                earth == (flags == 16 || (flags & 64) != 0), "Explicit/legacy reported yaw frame mismatch: " + flags);
        }
        foreach (var invalid in new[]
        {
            new MAVLink.mavlink_gimbal_device_attitude_status_t { flags = 96, q = new float[] { 1, 0, 0, 0 } },
            new MAVLink.mavlink_gimbal_device_attitude_status_t { flags = 1, q = new float[] { 1, 0, 0, 0 } },
            new MAVLink.mavlink_gimbal_device_attitude_status_t { failure_flags = 1, q = new float[] { 1, 0, 0, 0 } },
            new MAVLink.mavlink_gimbal_device_attitude_status_t { q = new float[] { 0, 0, 0, 0 } },
            new MAVLink.mavlink_gimbal_device_attitude_status_t { q = new float[] { float.NaN, 0, 0, 0 } },
        })
        { Check(!VideoMotionHistory.TryReportedPose(invalid, out _, out _, out _, out _), "Invalid mount report accepted"); }
    }

    private static void NativeMotionHistory()
    {
        using (var peer = new TrackingPeer())
        using (var history = new VideoMotionHistory(peer.Config))
        {
            GimbalController.SetTargetAngles(-45, 0);
            peer.MotionPosition(45, 0, 200, 100); peer.Pump(30);
            history.Capture(Stopwatch.GetTimestamp());
            Check(history.Heading == null, "An unsent UI target became a camera pose");
            var command = GimbalController.RequestTrackingTargetAsync(-45, 90, OutputController.Generation,
                GimbalController.ManualRevision, () => true);
            peer.Until(() => command.IsCompleted, "submitted motion estimate");
            Check(command.Result.Submitted && GimbalController.LastSubmittedPose != null,
                "Successful target did not publish submission metadata");
            long firstTicks = Stopwatch.GetTimestamp(); history.Capture(firstTicks);
            Check(history.Heading != null && Math.Abs(history.Heading.RelativeDegrees + 90) < .01,
                "Camera-right target did not place aircraft marker left");
            Check(!history.TryEstimate(firstTicks, 0, out _), "A newly submitted target was called settled");
            peer.Pump(600); peer.MotionPosition(45, 0, 200, 100); peer.Pump(20);
            long before = Stopwatch.GetTimestamp(); history.Capture(before);
            Check(history.TryEstimate(before, 0, out var estimate), "Fresh settled native estimate was unavailable");
            var previousSnapshot = history.Snapshot(before, 0);
            Check(previousSnapshot.IsCurrent && previousSnapshot.Ground != null, "A current native snapshot was invalid");
            Check(!history.TryEstimate(before, 100, out _), "A large telemetry gap manufactured bracketing history");
            var epoch = estimate.Epoch;
            DelayedNativePosition(peer, history);
            peer.StreamAttitude = true;
            peer.MotionPosition(45.0001, 0, 200, 101); peer.Pump(20);
            history.Capture(Stopwatch.GetTimestamp());
            Check(!ReferenceEquals(epoch, history.Epoch) && !previousSnapshot.IsCurrent,
                "HOME height change did not invalidate anchors and published snapshots");
            command = GimbalController.RequestTrackingTargetAsync(-30, -90, OutputController.Generation,
                GimbalController.ManualRevision, () => true);
            peer.Until(() => command.IsCompleted, "changed pose step");
            long after = Stopwatch.GetTimestamp(); history.Capture(after);
            Check(history.Heading != null && Math.Abs(history.Heading.RelativeDegrees - 90) < .01 &&
                !history.TryEstimate(after, 0, out _), "A submitted pose step was blended or called settled");
            peer.MotionPosition(45, 0, 200, 101, fix: 2); peer.Pump(20);
            after = Stopwatch.GetTimestamp(); history.Capture(after);
            Check(history.Heading != null && !history.TryEstimate(after, 0, out _),
                "GPS loss removed the independent heading or retained a ground estimate");
            previousSnapshot = history.Snapshot(after, 0);
            peer.StreamAttitude = false; Thread.Sleep(550);
            history.Capture(Stopwatch.GetTimestamp());
            Check(history.Heading == null && !history.TryEstimate(Stopwatch.GetTimestamp(), 0, out _),
                "Stale attitude retained heading or a ground hint");
            GimbalController.InvalidateInput();
            Check(GimbalController.LastSubmittedPose == null, "Input invalidation retained submitted metadata");
            OutputController.Initialize(new NOMADConfig { AircraftName = "Changed motion binding" });
            Check(!previousSnapshot.IsCurrent, "A different output profile retained an old heading snapshot");
        }
    }

    private static void DelayedNativePosition(TrackingPeer peer, VideoMotionHistory history)
    {
        peer.StreamAttitude = false;
        var first = CaptureNativeMotionSample(peer, history, 45);
        peer.Until(() => (DateTime.UtcNow - first.PositionUtc).TotalMilliseconds >= 100 &&
            (DateTime.UtcNow - first.AttitudeUtc).TotalMilliseconds >= 100,
            "separated native motion samples");
        var second = CaptureNativeMotionSample(peer, history, 45.0001);
        DateTime overlapStart = first.PositionUtc > first.AttitudeUtc ? first.PositionUtc : first.AttitudeUtc;
        DateTime overlapEnd = second.PositionUtc < second.AttitudeUtc ? second.PositionUtc : second.AttitudeUtc;
        double positionInterval = (second.PositionUtc - first.PositionUtc).TotalMilliseconds;
        double attitudeInterval = (second.AttitudeUtc - first.AttitudeUtc).TotalMilliseconds;
        double clockUncertainty = first.CaptureMilliseconds + second.CaptureMilliseconds + 2;
        string diagnostic = $"position={positionInterval:F2}ms attitude={attitudeInterval:F2}ms " +
            $"overlap={(overlapEnd - overlapStart).TotalMilliseconds:F2}ms clock-bound={clockUncertainty:F2}ms";
        Check(positionInterval < 500 && attitudeInterval < 500 &&
            (overlapEnd - overlapStart).TotalMilliseconds > 40 + 2 * clockUncertainty,
            "Native interpolation fixture did not establish bounded overlapping samples: " + diagnostic);
        DateTime requestedUtc = overlapStart.AddTicks((overlapEnd - overlapStart).Ticks / 2);
        int delay = (int)Math.Round((second.CaptureUtc - requestedUtc).TotalMilliseconds);
        DateTime actualQueryUtc = second.CaptureUtc.AddMilliseconds(-delay);
        double fraction = (actualQueryUtc - first.PositionUtc).TotalMilliseconds / positionInterval;
        double expectedLatitude = 45 + .0001 * fraction;
        double tolerance = .0001 * clockUncertainty / positionInterval + 1e-9;
        bool found = history.TryEstimate(second.CaptureTicks, delay, out var estimate);
        Check(found && Math.Abs(estimate.Latitude - expectedLatitude) <= tolerance &&
            estimate.Latitude > 45 && estimate.Latitude < 45.0001,
            "Delayed ground estimate did not interpolate observed packets: " + diagnostic +
            $" delay={delay}ms found={found} expected={expectedLatitude:F9} actual={estimate?.Latitude:F9}");
    }

    private static MotionSampleTimes CaptureNativeMotionSample(TrackingPeer peer, VideoMotionHistory history,
        double latitude)
    {
        var previousAttitude = peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.ATTITUDE);
        var previousPosition = peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT);
        peer.Attitude(0);
        peer.MotionPosition(latitude, 0, 200, 100);
        peer.Until(() => !ReferenceEquals(previousAttitude,
            peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.ATTITUDE)) &&
            !ReferenceEquals(previousPosition,
                peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT)),
            "matching new native position and attitude samples");
        var result = new MotionSampleTimes
        {
            PositionUtc = peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT).rxtime,
            AttitudeUtc = peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.ATTITUDE).rxtime,
            CaptureTicks = Stopwatch.GetTimestamp(), CaptureUtc = DateTime.UtcNow,
        };
        history.Capture(result.CaptureTicks);
        result.CaptureMilliseconds = (Stopwatch.GetTimestamp() - result.CaptureTicks) * 1000.0 / Stopwatch.Frequency;
        return result;
    }

    private sealed class MotionSampleTimes
    {
        internal DateTime PositionUtc, AttitudeUtc, CaptureUtc;
        internal long CaptureTicks;
        internal double CaptureMilliseconds;
    }

    private static void NativeMotionReferences()
    {
        using (var peer = new TrackingPeer())
        using (var history = new VideoMotionHistory(peer.Config))
        {
            peer.Connection.MAV.param["MNT1_TYPE"] = new MAVLink.MAVLinkParam("MNT1_TYPE", 0,
                MAVLink.MAV_PARAM_TYPE.INT8, null);
            peer.Connection.MAV.param["MNT2_TYPE"] = new MAVLink.MAVLinkParam("MNT2_TYPE", 1,
                MAVLink.MAV_PARAM_TYPE.INT8, null);
            peer.MotionPosition(45, 0, 200, 100); peer.MotionMount(1); peer.Pump(20);
            history.Capture(Stopwatch.GetTimestamp());
            Check(history.Heading == null, "MNT2-only aircraft accepted a secondary device1 report");
            peer.MotionMount(2); peer.Pump(20);
            long ticks = Stopwatch.GetTimestamp(); history.Capture(ticks);
            Check(history.Heading != null && Math.Abs(history.Heading.RelativeDegrees + 30) < .01 &&
                !history.TryEstimate(ticks, 0, out _), "Primary2 report did not supply heading alone");
            object epoch = history.Epoch;
            peer.MotionMount(1); peer.Pump(20); history.Capture(Stopwatch.GetTimestamp());
            Check(history.Heading == null && ReferenceEquals(epoch, history.Epoch),
                "Secondary report retained heading or changed ground reference");
            peer.MotionMount(2, 202); peer.Pump(20); history.Capture(Stopwatch.GetTimestamp());
            Check(history.Heading == null, "Wrong aircraft supplied a primary mount heading");
            var command = GimbalController.RequestTrackingTargetAsync(-45, 0, OutputController.Generation,
                GimbalController.ManualRevision, () => true);
            peer.Until(() => command.IsCompleted, "held ground target");
            Check(command.Result.Submitted, "Held ground target was not submitted");
            peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.HOME_POSITION).rxtime =
                DateTime.UtcNow.AddMinutes(-10);
            peer.Pump(2100); peer.Heartbeat(4);
            peer.MotionPosition(45, 0, 200, 100, sendHome: false); peer.MotionMount(2); peer.Pump(20);
            ticks = Stopwatch.GetTimestamp(); history.Capture(ticks);
            Check(history.TryEstimate(ticks, 0, out var estimate) && Math.Abs(estimate.HeadingDegrees) < .01 &&
                estimate.Source.Contains("held") && history.Heading != null &&
                Math.Abs(history.Heading.RelativeDegrees + 30) < .01 && ReferenceEquals(epoch, history.Epoch),
                "Cached HOME/held target expired, or reported pitch/yaw contaminated ground pose");
            peer.Connection.MAV.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.GIMBAL_DEVICE_ATTITUDE_STATUS).rxtime =
                DateTime.UtcNow.AddSeconds(-1);
            history.Capture(Stopwatch.GetTimestamp());
            Check(history.Heading != null && Math.Abs(history.Heading.RelativeDegrees) < .01 &&
                ReferenceEquals(epoch, history.Epoch), "Heading report fallback changed the ground reference");
            peer.MotionPosition(45, 0, 200, 150, sendHome: false); peer.Pump(20);
            ticks = Stopwatch.GetTimestamp(); history.Capture(ticks);
            Check(!history.TryEstimate(ticks, 0, out _), "Inconsistent HOME/MSL/relative altitude was accepted");
            peer.MotionPosition(45, 0, 200, 101); peer.Pump(20);
            ticks = Stopwatch.GetTimestamp(); history.Capture(ticks);
            Check(history.TryEstimate(ticks, 0, out _) && !ReferenceEquals(epoch, history.Epoch),
                "Changed valid HOME did not replace the ground reference");
            peer.StreamAttitude = false;
            peer.Attitude(0, boot: 100); peer.MotionPosition(45, 0, 200, 101, boot: 100); peer.Pump(20);
            history.Capture(Stopwatch.GetTimestamp()); epoch = history.Epoch;
            peer.Attitude(0, boot: 1); peer.MotionPosition(45, 0, 200, 101, sendHome: false); peer.Pump(20);
            ticks = Stopwatch.GetTimestamp(); history.Capture(ticks);
            Check(!history.TryEstimate(ticks, 0, out _) && !ReferenceEquals(epoch, history.Epoch),
                "Reboot reused old HOME and pre-reboot submitted pose");
        }
    }
}
