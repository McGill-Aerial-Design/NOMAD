// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Globalization;

namespace NOMAD.MissionPlanner
{
    internal static class VideoPipeline
    {
        // Viewers of one source share a receiver. A device path is case-insensitive; an RTSP path is not.
        internal static string ShareKey(VideoInputProfile input)
        {
            if (input == null) { return null; }
            if (input.SourceType == "Camera") { return "camera:" + input.CaptureDevicePath.ToUpperInvariant(); }
            return input.Url.StartsWith("udp://", StringComparison.OrdinalIgnoreCase)
                ? "udp:" + VideoStreamAddress.UdpPort(input.Url) : "rtsp:" + input.Url;
        }

        internal static string Build(VideoInputProfile input)
        {
            input.Validate();
            string source;
            if (input.SourceType == "Camera")
            {
                source = "ksvideosrc device-path=" + VideoCaptureDevice.QuotedPath(input.CaptureDevicePath) + " ! " +
                    "video/x-raw,width=(int)[1,1920],height=(int)[1,1080],framerate=(fraction)[1/1,30/1];" +
                    "image/jpeg,width=(int)[1,1920],height=(int)[1,1080],framerate=(fraction)[1/1,30/1] ! ";
            }
            else if (input.Url.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
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
            bool camera = input.SourceType == "Camera";
            string leaky = camera || input.DropOldFrames ? "downstream" : "no";
            int queue = camera ? 1 : input.QueueMaxBuffers;
            return source + "decodebin3 ! queue " +
                $"max-size-buffers={queue.ToString(CultureInfo.InvariantCulture)} " +
                $"max-size-bytes=0 max-size-time=0 leaky={leaky} ! videoconvert ! " +
                "video/x-raw,format=BGRA ! appsink name=outsink sync=false";
        }
    }
}
