// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Globalization;

namespace NOMAD.MissionPlanner
{
    internal static class VideoPipeline
    {
        internal static string Build(VideoInputProfile input)
        {
            input.Validate();
            string source;
            if (input.Url.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
            {
                source = $"udpsrc port={VideoStreamAddress.UdpPort(input.Url)} buffer-size=90000 ! " +
                    "application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264 ! ";
            }
            else
            {
                source = $"rtspsrc location={VideoStreamAddress.QuotedRtspUrl(input.Url)} " +
                    $"protocols={input.RtspTransport.ToLowerInvariant()} " +
                    $"latency={input.LatencyMs.ToString(CultureInfo.InvariantCulture)} " +
                    $"drop-on-latency={input.DropOldFrames.ToString().ToLowerInvariant()} do-retransmission=false ! " +
                    "application/x-rtp,media=video ! ";
            }
            string leaky = input.DropOldFrames ? "downstream" : "no";
            return source + "decodebin3 ! queue " +
                $"max-size-buffers={input.QueueMaxBuffers.ToString(CultureInfo.InvariantCulture)} " +
                $"max-size-bytes=0 max-size-time=0 leaky={leaky} ! videoconvert ! " +
                "video/x-raw,format=BGRA ! appsink name=outsink sync=false";
        }
    }
}
