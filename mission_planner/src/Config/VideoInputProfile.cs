// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Linq;
using System.IO;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    public sealed class VideoInputProfile
    {
        public string Name { get; set; } = "";
        public string Url { get; set; } = "";
        public string RtspTransport { get; set; } = "TCP";
        public int LatencyMs { get; set; } = 100;
        public int QueueMaxBuffers { get; set; } = 1;
        public bool DropOldFrames { get; set; } = true;
        public string DetectorModelProfilePath { get; set; } = "";
        public bool TrackingCalibrated { get; set; }
        public string TrackingMode { get; set; } = "Object";
        public bool GroundPredictionEnabled { get; set; }
        public double GroundPlaneHeightMeters { get; set; }
        public int CameraDelayEstimateMs { get; set; }
        public int HorizontalFovDeg { get; set; } = 90;
        public int PitchMinDeg { get; set; } = -90;
        public int PitchMaxDeg { get; set; } = 90;
        public int YawMinDeg { get; set; } = -90;
        public int YawMaxDeg { get; set; } = 90;
        public int PitchSign { get; set; } = 1;
        public int YawSign { get; set; } = 1;
        public int TrackingMaxRateDegSec { get; set; } = 10;

        internal VideoInputProfile Copy() => (VideoInputProfile)MemberwiseClone();

        internal void Validate()
        {
            if (string.IsNullOrWhiteSpace(Name) || Name != Name.Trim() || Name.Length > 80 ||
                Name.Any(char.IsControl))
            { throw new JsonSerializationException("Video input names must be trimmed text from 1 to 80 characters."); }
            if (RtspTransport != "TCP" && RtspTransport != "UDP")
            { throw new JsonSerializationException("Video RTSP transport must be TCP or UDP."); }
            if (LatencyMs < 0 || LatencyMs > 5000 || QueueMaxBuffers < 1 || QueueMaxBuffers > 120)
            { throw new JsonSerializationException("Video latency must be 0..5000 ms and queue size 1..120 frames."); }
            if ((TrackingMode != "Object" && TrackingMode != "Area") || CameraDelayEstimateMs < 0 ||
                CameraDelayEstimateMs > 2000 || double.IsNaN(GroundPlaneHeightMeters) ||
                double.IsInfinity(GroundPlaneHeightMeters) || Math.Abs(GroundPlaneHeightMeters) > 10000)
            { throw new JsonSerializationException("Review tracking mode, plane height and camera delay (0..2000 ms)."); }
            if (DetectorModelProfilePath == null || DetectorModelProfilePath.Length > 1024 ||
                DetectorModelProfilePath.Any(char.IsControl) || HorizontalFovDeg < 5 || HorizontalFovDeg > 170 ||
                (DetectorModelProfilePath != "" && (!Path.IsPathRooted(DetectorModelProfilePath) ||
                    Path.GetPathRoot(DetectorModelProfilePath).Length < 3)) ||
                PitchMinDeg < -90 || PitchMaxDeg > 90 || PitchMinDeg >= PitchMaxDeg ||
                YawMinDeg < GimbalCommand.YAW_MIN_DEG || YawMaxDeg > GimbalCommand.YAW_MAX_DEG ||
                YawMinDeg >= YawMaxDeg ||
                (PitchSign != -1 && PitchSign != 1) || (YawSign != -1 && YawSign != 1) ||
                TrackingMaxRateDegSec < 1 || TrackingMaxRateDegSec > 60)
            { throw new JsonSerializationException("Review vision FOV, pitch/yaw limits, signs and rate (1..60 deg/s)."); }
            try
            {
                if (Url?.StartsWith("udp://", StringComparison.OrdinalIgnoreCase) == true)
                { VideoStreamAddress.UdpPort(Url); }
                else { VideoStreamAddress.QuotedRtspUrl(Url); }
            }
            catch (FormatException)
            { throw new JsonSerializationException("Video input requires an absolute RTSP URL or UDP listening port."); }
        }
    }
}
