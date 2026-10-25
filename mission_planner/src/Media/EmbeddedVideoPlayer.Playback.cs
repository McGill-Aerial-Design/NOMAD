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
            return VideoPipeline.Build(_input);
        }

        public void StartStream()
        {
            ValidateUiThread();
            try
            {
                if (_session.Start(BuildGStreamerPipeline()))
                {
                    _frameTimer.Start();
                    _lblStatus.Text = "Connecting...";
                    _lblStatus.ForeColor = Color.Yellow;
                }
            }
            catch (Exception error) when (error is FormatException || error is Newtonsoft.Json.JsonSerializationException)
            {
                _lblStatus.Text = error.Message;
                _lblStatus.ForeColor = Color.Red;
            }
        }

        public void StopStream()
        {
            ValidateUiThread();
            RetireTracking("Video stopped");
            try
            {
                _session.Stop();
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
