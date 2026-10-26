// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    internal sealed class VideoMotionSnapshot
    {
        private readonly Func<bool> _isCurrent;
        internal GroundCameraEstimate Ground { get; }
        internal VideoHeadingEstimate Heading { get; }
        internal object Epoch { get; }
        internal bool IsCurrent => _isCurrent?.Invoke() == true;
        internal VideoMotionSnapshot(object epoch, GroundCameraEstimate ground, VideoHeadingEstimate heading,
            Func<bool> isCurrent)
        { Epoch = epoch; Ground = ground; Heading = heading; _isCurrent = isCurrent; }
    }

    // Only the video worker owns this bounded history; it reads the selected MP connection without requesting data.
    internal sealed class VideoMotionHistory : IDisposable
    {
        private readonly NOMADConfig _config;
        private readonly List<AttitudeSample> _attitudes = new List<AttitudeSample>();
        private readonly List<PositionSample> _positions = new List<PositionSample>();
        private readonly List<PoseSample> _poses = new List<PoseSample>();
        private readonly List<PoseSample> _submittedPoses = new List<PoseSample>();
        private OutputController.NativeTarget _target;
        private MAVState _mav;
        private volatile object _epoch = new object();
        private MAVLink.MAVLinkMessage _lastAttitude, _lastPosition, _lastPose;
        private MAVLink.mavlink_home_position_t? _home;
        private GimbalController.SubmittedPose _lastSubmission;
        private bool _freshGround, _freshAircraft;
        private long _now;
        private long _minimumTicks;
        internal object Epoch => _epoch;
        internal VideoHeadingEstimate Heading { get; private set; }

        internal VideoMotionHistory(NOMADConfig config) { _config = config; }

        internal void Capture(long frameReceivedTicks)
        {
            _now = Stopwatch.GetTimestamp();
            _freshAircraft = false; _freshGround = false; Heading = null;
            if (frameReceivedTicks > _now || Seconds(_now - frameReceivedTicks) >= .5) { return; }
            var target = OutputController.CaptureTarget(_config);
            if (target == null) { Reset(); return; }
            var mav = target.Connection.MAV;
            if (_target == null || !_target.IsCurrent || !ReferenceEquals(_mav, mav))
            { Reset(); _target = target; _mav = mav; }
            DateTime utc = DateTime.UtcNow;
            var heartbeat = Packet(mav, MAVLink.MAVLINK_MSG_ID.HEARTBEAT, target.Component, utc, 3.5);
            var attitudePacket = Packet(mav, MAVLink.MAVLINK_MSG_ID.ATTITUDE, target.Component, utc, .5);
            if (heartbeat == null || attitudePacket == null || heartbeat.ToStructure<MAVLink.mavlink_heartbeat_t>()
                .autopilot != (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA) { return; }
            var attitude = attitudePacket.ToStructure<MAVLink.mavlink_attitude_t>();
            if (!VideoTrackingAngles.TryPredictHeading(attitude.yaw, attitude.roll, attitude.pitch,
                attitude.pitchspeed, attitude.yawspeed, 0, out _)) { return; }
            if (_lastAttitude != null && Rebooted(_lastAttitude.ToStructure<MAVLink.mavlink_attitude_t>()
                .time_boot_ms, attitude.time_boot_ms))
            { ResetSamples(); _minimumTicks = Ticks(attitudePacket, utc); }
            if (!ReferenceEquals(_lastAttitude, attitudePacket))
            {
                Add(_attitudes, new AttitudeSample(Ticks(attitudePacket, utc), attitude));
                _lastAttitude = attitudePacket;
            }
            _freshAircraft = true;
            CapturePose(utc);
            CapturePosition(utc);
            if (!target.IsCurrent || !ReferenceEquals(target.Connection.MAV, mav)) { Reset(); return; }
            Heading = GetHeading(frameReceivedTicks, 0);
        }

        internal VideoMotionSnapshot Snapshot(long frameReceivedTicks, int delayMilliseconds)
        {
            TryEstimate(frameReceivedTicks, delayMilliseconds, out var ground);
            var epoch = _epoch; var target = _target; var mav = _mav;
            return new VideoMotionSnapshot(epoch, ground, GetHeading(frameReceivedTicks, delayMilliseconds),
                () => ReferenceEquals(epoch, _epoch) && target?.IsCurrent == true &&
                    ReferenceEquals(target.Connection.MAV, mav) &&
                    Seconds(Stopwatch.GetTimestamp() - frameReceivedTicks) >= 0 &&
                    Seconds(Stopwatch.GetTimestamp() - frameReceivedTicks) < .5);
        }

        internal bool TryEstimate(long frameReceivedTicks, int delayMilliseconds, out GroundCameraEstimate estimate)
        {
            estimate = null;
            if (!_freshGround || !_home.HasValue || !Current() ||
                !QueryTicks(frameReceivedTicks, delayMilliseconds, out long ticks) ||
                !TryHeading(ticks, delayMilliseconds == 0, out double aircraftHeading) ||
                !TrySubmittedPose(ticks, out var pose) || Seconds(ticks - pose.ChangedTicks) < .5 ||
                !TryPosition(ticks, delayMilliseconds == 0, out double latitude, out double longitude,
                    out double altitude)) { return false; }
            estimate = new GroundCameraEstimate(_epoch, latitude, longitude, altitude, _home.Value.altitude / 1000.0,
                pose.Pitch, pose.Roll, GroundProjection.Wrap(pose.Yaw + aircraftHeading),
                ticks, pose.Source);
            return true;
        }

        internal VideoHeadingEstimate GetHeading(long frameReceivedTicks, int delayMilliseconds)
        {
            if (!_freshAircraft || !Current() || !QueryTicks(frameReceivedTicks, delayMilliseconds, out long ticks) ||
                !TryHeading(_now, true, out double currentHeading) ||
                !TryHeading(ticks, delayMilliseconds == 0, out double frameHeading) || !TryPose(ticks, out var pose))
            { return null; }
            double cameraHeading = pose.Yaw + (pose.EarthYaw ? 0 : frameHeading);
            return new VideoHeadingEstimate((float)GroundProjection.Wrap(currentHeading - cameraHeading),
                pose.Source, frameReceivedTicks);
        }

        private void CapturePosition(DateTime utc)
        {
            var homePacket = Packet(_mav, MAVLink.MAVLINK_MSG_ID.HOME_POSITION, _target.Component, utc, double.MaxValue);
            var positionPacket = Packet(_mav, MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT, _target.Component, utc, .5);
            var gpsPacket = Packet(_mav, MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT, _target.Component, utc, 1.5);
            if (homePacket == null || positionPacket == null || gpsPacket == null ||
                (_minimumTicks > 0 && Ticks(homePacket, utc) < _minimumTicks) ||
                Ticks(positionPacket, utc) < _minimumTicks ||
                Ticks(gpsPacket, utc) < _minimumTicks) { return; }
            var home = homePacket.ToStructure<MAVLink.mavlink_home_position_t>();
            var position = positionPacket.ToStructure<MAVLink.mavlink_global_position_int_t>();
            var gps = gpsPacket.ToStructure<MAVLink.mavlink_gps_raw_int_t>();
            if (gps.fix_type < 3 || gps.fix_type > 8 || Math.Abs((double)home.latitude) > 850000000 ||
                Math.Abs((double)home.longitude) > 1800000000 || Math.Abs((double)position.lat) > 850000000 ||
                Math.Abs((double)position.lon) > 1800000000 ||
                Math.Abs((double)position.alt - position.relative_alt - home.altitude) > 3000) { return; }
            if (_home.HasValue && (_home.Value.latitude != home.latitude || _home.Value.longitude != home.longitude ||
                _home.Value.altitude != home.altitude))
            { _epoch = new object(); _positions.Clear(); _lastPosition = null; }
            _home = home;
            if (_lastPosition != null && Rebooted(_lastPosition.ToStructure<MAVLink.mavlink_global_position_int_t>()
                .time_boot_ms, position.time_boot_ms))
            { ResetSamples(); _minimumTicks = Ticks(positionPacket, utc); return; }
            if (!ReferenceEquals(_lastPosition, positionPacket))
            {
                Add(_positions, new PositionSample(Ticks(positionPacket, utc), position));
                _lastPosition = positionPacket;
            }
            _freshGround = true;
        }

        private void CapturePose(DateTime utc)
        {
            CaptureSubmittedPose();
            var packet = Packet(_mav, MAVLink.MAVLINK_MSG_ID.GIMBAL_DEVICE_ATTITUDE_STATUS, _target.Component, utc, .5);
            byte primary = PrimaryMount();
            if (packet != null && packet.ToStructure<MAVLink.mavlink_gimbal_device_attitude_status_t>()
                .gimbal_device_id != primary) { packet = null; }
            if (primary != 0 && packet != null && Ticks(packet, utc) >= _minimumTicks)
            {
                var reported = packet.ToStructure<MAVLink.mavlink_gimbal_device_attitude_status_t>();
                if (!TryReportedPose(reported, out double pitch, out double roll, out double yaw, out bool earth))
                { _poses.Clear(); _lastPose = null; return; }
                if (_lastPose != null && Rebooted(_lastPose
                    .ToStructure<MAVLink.mavlink_gimbal_device_attitude_status_t>().time_boot_ms, reported.time_boot_ms))
                { ResetSamples(); _minimumTicks = Ticks(packet, utc); return; }
                string source = "Estimated autopilot primary mount " + primary + " report";
                source += earth ? " / earth yaw" : " / vehicle yaw";
                if (!ReferenceEquals(_lastPose, packet))
                {
                    long ticks = Ticks(packet, utc);
                    Add(_poses, new PoseSample(ticks, ticks, pitch, roll, yaw, earth, source));
                    _lastPose = packet;
                }
                return;
            }
            _poses.Clear(); _lastPose = null;
        }

        private byte PrimaryMount()
        {
            if (!_mav.param.ContainsKey("MNT1_TYPE")) { return 0; }
            double first = _mav.param["MNT1_TYPE"].Value;
            if (!GroundProjection.Finite(first) || first < 0 || first != Math.Truncate(first)) { return 0; }
            if (first > 0) { return 1; }
            if (!_mav.param.ContainsKey("MNT2_TYPE")) { return 0; }
            double second = _mav.param["MNT2_TYPE"].Value;
            return GroundProjection.Finite(second) && second > 0 && second == Math.Truncate(second) ? (byte)2 : (byte)0;
        }

        private void CaptureSubmittedPose()
        {
            var submitted = GimbalController.LastSubmittedPose;
            if (submitted == null || submitted.Generation != _target.Generation ||
                submitted.SubmittedTicks < _minimumTicks || submitted.SubmittedTicks > _now)
            { _submittedPoses.Clear(); _lastSubmission = null; return; }
            if (!ReferenceEquals(submitted, _lastSubmission))
            {
                Add(_submittedPoses, new PoseSample(submitted.SubmittedTicks, submitted.ChangedTicks, submitted.Pitch,
                    submitted.Roll, submitted.Yaw, false, "Estimated NOMAD held mount target"));
                _lastSubmission = submitted;
            }
        }

        internal static bool TryReportedPose(MAVLink.mavlink_gimbal_device_attitude_status_t reported,
            out double pitch, out double roll, out double yaw, out bool earth)
        {
            pitch = roll = yaw = 0; earth = false;
            int frameFlags = reported.flags & 96;
            if (reported.failure_flags != 0 || (reported.flags & 3) != 0 || frameFlags == 96 ||
                reported.q == null || reported.q.Length != 4) { return false; }
            double norm = 0;
            foreach (float value in reported.q)
            { if (!GroundProjection.Finite(value)) { return false; } norm += value * value; }
            if (norm < .95 || norm > 1.05) { return false; }
            double scale = Math.Sqrt(norm), w = reported.q[0] / scale, x = reported.q[1] / scale;
            double y = reported.q[2] / scale, z = reported.q[3] / scale;
            roll = Math.Atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180 / Math.PI;
            pitch = Math.Asin(Math.Max(-1, Math.Min(1, 2 * (w * y - z * x)))) * 180 / Math.PI;
            yaw = Math.Atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180 / Math.PI;
            earth = frameFlags == 64 || (frameFlags == 0 && (reported.flags & 16) != 0);
            return true;
        }

        private bool TryHeading(long ticks, bool latest, out double heading)
        {
            heading = 0;
            if (!Bracket(_attitudes, ticks, latest, out int before, out int after, out double fraction)) { return false; }
            var first = _attitudes[before]; var second = _attitudes[after];
            if (before != after)
            {
                heading = GroundProjection.Wrap(first.Value.yaw * 180 / Math.PI + fraction *
                    GroundProjection.Wrap((second.Value.yaw - first.Value.yaw) * 180 / Math.PI));
                return true;
            }
            if (!VideoTrackingAngles.TryPredictHeading(first.Value.yaw, first.Value.roll, first.Value.pitch,
                first.Value.pitchspeed, first.Value.yawspeed, Seconds(ticks - first.Ticks), out float radians))
            { return false; }
            heading = radians * 180 / Math.PI; return true;
        }

        private bool TryPosition(long ticks, bool latest, out double latitude, out double longitude, out double altitude)
        {
            latitude = longitude = altitude = 0;
            if (!Bracket(_positions, ticks, latest, out int before, out int after, out double fraction)) { return false; }
            var first = _positions[before].Value; var second = _positions[after].Value;
            if (before == after)
            { return TryPredictPosition(first, Seconds(ticks - _positions[before].Ticks), out latitude, out longitude,
                out altitude); }
            latitude = (first.lat + fraction * ((double)second.lat - first.lat)) / 1e7;
            longitude = GroundProjection.Wrap(first.lon / 1e7 + fraction *
                GroundProjection.Wrap((second.lon - (double)first.lon) / 1e7));
            altitude = (first.alt + fraction * ((double)second.alt - first.alt)) / 1000;
            return true;
        }

        internal static bool TryPredictPosition(MAVLink.mavlink_global_position_int_t position, double age,
            out double latitude, out double longitude, out double altitude)
        {
            latitude = longitude = altitude = 0;
            double northSpeed = position.vx / 100.0, eastSpeed = position.vy / 100.0, downSpeed = position.vz / 100.0;
            if (!GroundProjection.Finite(age) || age < 0 || age >= .5 ||
                Math.Abs((double)position.lat) > 850000000 || Math.Abs((double)position.lon) > 1800000000 ||
                Math.Sqrt(northSpeed * northSpeed + eastSpeed * eastSpeed) > 100 || Math.Abs(downSpeed) > 50)
            { return false; }
            latitude = position.lat / 1e7 + northSpeed * age / 6378137 * 180 / Math.PI;
            longitude = GroundProjection.Wrap(position.lon / 1e7 + eastSpeed * age /
                (6378137 * Math.Cos(position.lat / 1e7 * Math.PI / 180)) * 180 / Math.PI);
            altitude = position.alt / 1000.0 - downSpeed * age;
            return Math.Abs(latitude) <= 85;
        }

        private bool TryPose(long ticks, out PoseSample pose)
        {
            pose = null;
            for (int index = _poses.Count - 1; index >= 0; --index)
            {
                if (_poses[index].Ticks > ticks) { continue; }
                double age = Seconds(ticks - _poses[index].Ticks);
                if (age <= .5) { pose = _poses[index]; return true; }
                break;
            }
            return TrySubmittedPose(ticks, out pose);
        }

        private bool TrySubmittedPose(long ticks, out PoseSample pose)
        {
            pose = null;
            var current = GimbalController.LastSubmittedPose;
            if (current == null || current.Generation != _target.Generation ||
                !ReferenceEquals(current, _lastSubmission)) { return false; }
            for (int index = _submittedPoses.Count - 1; index >= 0; --index)
            {
                if (_submittedPoses[index].Ticks > ticks) { continue; }
                pose = _submittedPoses[index]; return true;
            }
            return false;
        }

        private static bool Bracket<T>(List<T> samples, long ticks, bool latest, out int before, out int after,
            out double fraction) where T : TimedSample
        {
            before = after = 0; fraction = 0;
            for (int index = samples.Count - 1; index >= 0; --index)
            {
                if (samples[index].Ticks > ticks) { continue; }
                before = index; after = index;
                if (samples[index].Ticks == ticks) { return true; }
                if (index == samples.Count - 1) { return latest && Seconds(ticks - samples[index].Ticks) < .5; }
                after = index + 1;
                long interval = samples[after].Ticks - samples[before].Ticks;
                if (interval <= 0 || Seconds(interval) > .5) { return false; }
                fraction = (ticks - samples[before].Ticks) / (double)interval; return true;
            }
            return false;
        }

        private MAVLink.MAVLinkMessage Packet(MAVState mav, MAVLink.MAVLINK_MSG_ID id, byte component,
            DateTime utc, double maximumAge)
        {
            var packet = mav?.getPacketLast((uint)id);
            if (packet == null || packet.sysid != _target.System || packet.compid != component ||
                packet.rxtime.Kind != DateTimeKind.Utc) { return null; }
            double age = (utc - packet.rxtime).TotalSeconds;
            return age >= 0 && age <= maximumAge ? packet : null;
        }

        private void Add<T>(List<T> samples, T sample) where T : TimedSample
        {
            if (samples.Count > 0 && sample.Ticks <= samples[samples.Count - 1].Ticks) { return; }
            while (samples.Count > 0 && (samples.Count >= 128 || Seconds(_now - samples[0].Ticks) > 3))
            { samples.RemoveAt(0); }
            samples.Add(sample);
        }

        private bool QueryTicks(long received, int delayMilliseconds, out long ticks)
        {
            ticks = received - (long)(delayMilliseconds / 1000.0 * Stopwatch.Frequency);
            return delayMilliseconds >= 0 && delayMilliseconds <= 2000 && received <= _now &&
                Seconds(_now - received) < .5 && ticks > 0;
        }
        private bool Current() => _target?.IsCurrent == true && ReferenceEquals(_target.Connection.MAV, _mav);
        private long Ticks(MAVLink.MAVLinkMessage packet, DateTime utc) =>
            _now - (long)((utc - packet.rxtime).TotalSeconds * Stopwatch.Frequency);
        private static double Seconds(long ticks) => ticks / (double)Stopwatch.Frequency;
        private static bool Rebooted(uint previous, uint current) => current != previous &&
            unchecked(current - previous) > 0x80000000;
        private void ResetSamples()
        {
            _epoch = new object(); _attitudes.Clear(); _positions.Clear(); _poses.Clear(); _submittedPoses.Clear();
            _lastAttitude = _lastPosition = _lastPose = null; _lastSubmission = null;
            _home = null; _freshGround = false; Heading = null;
        }
        internal void Reset()
        { ResetSamples(); _target = null; _mav = null; _freshAircraft = false; _minimumTicks = 0; }
        public void Dispose() => Reset();

        private abstract class TimedSample
        {
            internal long Ticks { get; }
            protected TimedSample(long ticks) { Ticks = ticks; }
        }
        private sealed class AttitudeSample : TimedSample
        {
            internal MAVLink.mavlink_attitude_t Value { get; }
            internal AttitudeSample(long ticks, MAVLink.mavlink_attitude_t value) : base(ticks) { Value = value; }
        }
        private sealed class PositionSample : TimedSample
        {
            internal MAVLink.mavlink_global_position_int_t Value { get; }
            internal PositionSample(long ticks, MAVLink.mavlink_global_position_int_t value) : base(ticks) { Value = value; }
        }
        private sealed class PoseSample : TimedSample
        {
            internal long ChangedTicks { get; }
            internal double Pitch { get; }
            internal double Roll { get; }
            internal double Yaw { get; }
            internal bool EarthYaw { get; }
            internal string Source { get; }
            internal PoseSample(long ticks, long changed, double pitch, double roll, double yaw, bool earth,
                string source) : base(ticks)
            { ChangedTicks = changed; Pitch = pitch; Roll = roll; Yaw = yaw; EarthYaw = earth; Source = source; }
        }
    }
}
