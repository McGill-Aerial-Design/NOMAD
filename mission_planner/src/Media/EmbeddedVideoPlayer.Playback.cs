// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private string BuildGStreamerPipeline()
        {
            int queueBuffers = _latencyMs <= 100 ? 1 : Math.Min(_latencyMs / 50, 10);
            string leaky = _latencyMs <= 100 ? "leaky=2" : "leaky=0";
            string syncValue = _latencyMs <= 100 ? "false" : "true";

            if (_streamUrl.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
            {
                var port = ExtractUdpPort(_streamUrl);
                return $"udpsrc port={port} buffer-size=90000 ! " +
                       "application/x-rtp,media=(string)video,clock-rate=(int)90000," +
                       "encoding-name=(string)H264 ! decodebin3 ! " +
                       $"queue max-size-buffers={queueBuffers} {leaky} ! " +
                       "videoconvert ! video/x-raw,format=BGRA ! " +
                       $"appsink name=outsink sync={syncValue}";
            }

            return $"rtspsrc location={_streamUrl} protocols=tcp latency={_latencyMs} " +
                   "do-retransmission=false ! " +
                   "application/x-rtp,media=video,encoding-name=H264 ! " +
                   "rtph264depay ! h264parse disable-passthrough=true ! avdec_h264 ! " +
                   $"queue max-size-buffers={queueBuffers} {leaky} ! " +
                   "videoconvert ! video/x-raw,format=BGRA ! " +
                   $"appsink name=outsink sync={syncValue}";
        }

        public void StartStream()
        {
            ValidateUiThread();
            if (_session.Start(BuildGStreamerPipeline()))
            {
                _frameTimer.Start();
                _lblStatus.Text = "Connecting...";
                _lblStatus.ForeColor = Color.Yellow;
            }
        }

        public void StopStream()
        {
            ValidateUiThread();
            _session.Stop();
            try
            {
            }
            finally
            {
                _frameTimer.Stop();
                ClearVideoDisplayAndDisposeBuffers();
            }
            if (!IsDisposed)
            {
                _lblStatus.Text = _session.Error == null ? "Stopped" : $"Error: {_session.Error}";
                _lblStatus.ForeColor = _session.Error == null ? Color.Gray : Color.Red;
            }
        }
    }
}
