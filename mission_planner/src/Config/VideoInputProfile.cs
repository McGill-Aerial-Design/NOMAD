// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Linq;
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

        internal void Validate()
        {
            if (string.IsNullOrWhiteSpace(Name) || Name != Name.Trim() || Name.Length > 80 ||
                Name.Any(char.IsControl))
            { throw new JsonSerializationException("Video input names must be trimmed text from 1 to 80 characters."); }
            if (RtspTransport != "TCP" && RtspTransport != "UDP")
            { throw new JsonSerializationException("Video RTSP transport must be TCP or UDP."); }
            if (LatencyMs < 0 || LatencyMs > 5000 || QueueMaxBuffers < 1 || QueueMaxBuffers > 120)
            { throw new JsonSerializationException("Video latency must be 0..5000 ms and queue size 1..120 frames."); }
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
