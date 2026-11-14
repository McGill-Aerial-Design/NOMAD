// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Diagnostics;
using System.Drawing;
using Brushes = System.Windows.Media.Brushes;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private long _lastFrameTickTicks;
        private void OnFrameTick(object sender, EventArgs e)
        {
            try { RefreshFrame(); }
            catch (Exception error)
            {
                _frameTimer.Stop();
                Log.Error("Video view disabled after a callback failure: " + error);
                try
                {
                    RetireTracking("Video view failed; restart video before selecting again");
                    _session.Stop();
                    ClearVideoDisplayAndDisposeBuffers();
                    _lblStatus.Text = "Video unavailable; restart video";
                    _lblStatus.Foreground = Brushes.Red;
                }
                catch (Exception cleanup) { Log.Error("Video view cleanup failed: " + cleanup); }
            }
        }

        private void RefreshFrame()
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
            long now = Stopwatch.GetTimestamp();
            if (IsTracking && _lastFrameTickTicks != 0 &&
                (now - _lastFrameTickTicks) / (double)Stopwatch.Frequency >= .5)
            { DisarmTracking("Video UI update gap"); }
            _lastFrameTickTicks = now;
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
                _lblStatus.Text = $"Streaming {frame.Image.Width}x{frame.Image.Height}";
                _lblStatus.Foreground = Brushes.LimeGreen;
            }
            if (_session.IsActive && _displayedVideoFrame?.Fresh == false)
            {
                _lblStatus.Text = "Video stale; waiting for frames";
                _lblStatus.Foreground = Brushes.Yellow;
            }
            var sample = _session.TakeTrackingSample();
            if (sample != null) { UpdateVisionSample(sample); }
            UpdateVisionStatus();
            _videoBox.Invalidate();
            _fullscreenBox?.Invalidate();
            if (_session.State == VideoState.Stopped)
            {
                _playbackRequested = false;
                RetireTracking("Video ended; select again after restarting");
                ClearVideoDisplayAndDisposeBuffers();
                _frameTimer.Stop();
                _lblStatus.Text = _session.Error == null ? "Stopped" : $"Error: {_session.Error}";
                _lblStatus.Foreground = _session.Error == null ? Brushes.Gray : Brushes.Red;
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
            else if (_displayFrame != null) { MediaPixels.Dispose(_displayFrame); }
            _displayedVideoFrame = null;
            _displayFrame = null;
        }
    }
}
