// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private void OnFrameTick(object sender, EventArgs e)
        {
            if (IsDisposed)
            {
                return;
            }
            if (_session.State == VideoState.Disposed)
            {
                Dispose();
                return;
            }
            var frame = _session.TakeFrame();
            if (frame != null)
            {
                var old = _displayedVideoFrame;
                _displayedVideoFrame = frame;
                _displayFrame = frame.Image;
                _videoBox.Image = frame.Image;
                if (_fullscreenBox != null && !_fullscreenBox.IsDisposed)
                {
                    _fullscreenBox.Image = frame.Image;
                }
                old?.Dispose();
                _lblStatus.Text = _flyEnabled ? FlightStatus : $"Streaming {frame.Image.Width}x{frame.Image.Height}";
                _lblStatus.ForeColor = Color.LimeGreen;
                UpdateVisionFrame(frame);
                _videoBox.Invalidate();
                _fullscreenBox?.Invalidate();
            }
            UpdateVisionStatus();
            SampleFlight();
            if (_session.State == VideoState.Stopped)
            {
                RetireTracking("Video ended; select again after restarting");
                ClearVideoDisplayAndDisposeBuffers();
                _frameTimer.Stop();
                _lblStatus.Text = _session.Error == null ? "Stopped" : $"Error: {_session.Error}";
                _lblStatus.ForeColor = _session.Error == null ? Color.Gray : Color.Red;
            }
        }

        private void ClearVideoDisplayAndDisposeBuffers()
        {
            if (_fullscreenBox != null && !_fullscreenBox.IsDisposed)
            {
                _fullscreenBox.Image = null;
            }
            if (_videoBox != null && !_videoBox.IsDisposed)
            {
                _videoBox.Image = null;
            }
            if (_displayedVideoFrame != null) { _displayedVideoFrame.Dispose(); }
            else { _displayFrame?.Dispose(); }
            _displayedVideoFrame = null;
            _displayFrame = null;
        }
    }
}
