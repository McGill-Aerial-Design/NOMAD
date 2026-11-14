// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using Brushes = System.Windows.Media.Brushes;
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private long _playbackRevision;
        private volatile bool _playbackRequested;
        private string BuildGStreamerPipeline()
        {
            return VideoPipeline.Build(_input);
        }

        public void StartStream()
        {
            ValidateUiThread();
            if (IsDisposed || _session.State == VideoState.Disposed) { return; }
            _playbackRequested = true;
            long revision = Interlocked.Increment(ref _playbackRevision);
            _ = StartWhenReadyAsync(revision);
        }

        private async Task StartWhenReadyAsync(long revision)
        {
            try
            {
                if (_session.State == VideoState.Stopping ||
                    (_session.State == VideoState.Stopped && !_session.Completion.IsCompleted))
                { await _session.Completion; }
                if (IsDisposed || !_playbackRequested || revision != Volatile.Read(ref _playbackRevision) ||
                    _session.State == VideoState.Disposed) { return; }
                if (_session.Start(BuildGStreamerPipeline(), VideoPipeline.ShareKey(_input)))
                {
                    _frameTimer.Start();
                    _lblStatus.Text = "Connecting...";
                    _lblStatus.Foreground = Brushes.Yellow;
                }
            }
            catch (Exception error)
            {
                Log.Error("Video could not start: " + error);
                if (!IsDisposed && revision == Volatile.Read(ref _playbackRevision))
                {
                    _playbackRequested = false;
                    _lblStatus.Text = error is FormatException || error is Newtonsoft.Json.JsonSerializationException
                        ? error.Message : "Video could not start; review the input and try again";
                    _lblStatus.Foreground = Brushes.Red;
                }
            }
        }

        public void StopStream()
        {
            ValidateUiThread();
            _playbackRequested = false;
            long revision = Interlocked.Increment(ref _playbackRevision);
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
                _lblStatus.Text = "Stopping…";
                _lblStatus.Foreground = _session.Error == null ? Brushes.Gray : Brushes.Red;
                _ = UpdateStoppedStatusAsync(revision);
            }
        }

        private async Task UpdateStoppedStatusAsync(long revision)
        {
            try { await _session.Completion; }
            catch (Exception error) { Log.Error("Video cleanup failed: " + error); }
            if (!IsDisposed && !_playbackRequested && revision == Volatile.Read(ref _playbackRevision))
            { _lblStatus.Text = _session.Error == null ? "Stopped" : $"Error: {_session.Error}"; }
        }
    }
}
