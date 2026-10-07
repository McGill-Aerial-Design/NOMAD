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
                var old = _displayFrame;
                _displayFrame = frame;
                _videoBox.Image = frame;
                if (_fullscreenBox != null && !_fullscreenBox.IsDisposed)
                {
                    _fullscreenBox.Image = frame;
                }
                old?.Dispose();
                _lblStatus.Text = $"Streaming {frame.Width}x{frame.Height}";
                _lblStatus.ForeColor = Color.LimeGreen;
            }
            if (_session.State == VideoState.Stopped)
            {
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
            _displayFrame?.Dispose();
            _displayFrame = null;
        }
    }
}
