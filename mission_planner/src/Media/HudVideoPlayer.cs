// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;

using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    internal sealed class HudVideoPlayer : IDisposable
    {
        private readonly Control _hud;
        private readonly Action<Bitmap> _setFrame;
        private readonly Func<Image> _getFrame;
        private readonly VideoSession _session;
        private readonly System.Windows.Forms.Timer _timer = new System.Windows.Forms.Timer { Interval = 33 };
        private Bitmap _display;
        private VideoFrame _displayedVideoFrame;
        private volatile bool _disposed;
        private volatile bool _startRequested;
        private long _playbackRevision;

        internal HudVideoPlayer(Control hud, Action<Bitmap> setFrame, Func<Image> getFrame,
            Func<IVideoPipeline> createPipeline)
        {
            _setFrame = setFrame;
            _getFrame = getFrame;
            _session = new VideoSession(createPipeline);
            _hud = hud ?? throw new InvalidOperationException("HUD is not available");
            if (_hud.IsDisposed)
            {
                throw new InvalidOperationException("HUD is disposed");
            }
            _hud.Disposed += OnHudDisposed;
            _timer.Tick += OnFrameTick;
        }

        public bool IsActive => !_disposed && (_startRequested || _session.IsActive);

        public void Start(string pipeline, string shareKey = null)
        {
            if (_disposed) { return; }
            _startRequested = true;
            _ = StartWhenReadyAsync(pipeline, shareKey, Interlocked.Increment(ref _playbackRevision));
        }

        private async Task StartWhenReadyAsync(string pipeline, string shareKey, long revision)
        {
            try
            {
                if (_session.State == VideoState.Stopping ||
                    (_session.State == VideoState.Stopped && !_session.Completion.IsCompleted))
                { await _session.Completion; }
                if (_disposed || !_startRequested || revision != Volatile.Read(ref _playbackRevision)) { return; }
                if (_session.Start(pipeline, shareKey)) { _timer.Start(); }
            }
            catch (Exception error)
            {
                Log.Error("HUD video could not start: " + error);
                if (!_disposed && revision == Volatile.Read(ref _playbackRevision)) { Dispose(); }
            }
        }

        internal void UpdateInput(string pipeline, string shareKey = null)
        {
            if (_disposed) { return; }
            bool wasPlaying = IsActive;
            _startRequested = false; Interlocked.Increment(ref _playbackRevision);
            _session.Stop();
            _timer.Stop();
            ClearDisplay();
            if (wasPlaying) { Start(pipeline, shareKey); }
        }

        private void ClearDisplay()
        {
            if (!_hud.IsDisposed && ReferenceEquals(_getFrame(), _display)) { _setFrame(null); }
            _displayedVideoFrame?.Dispose();
            _displayedVideoFrame = null;
            _display = null;
        }

        private void OnFrameTick(object sender, EventArgs e)
        {
            try { RefreshFrame(); }
            catch (Exception error)
            {
                _timer.Stop(); Log.Error("HUD video disabled after a callback failure: " + error);
                try { Dispose(); } catch (Exception cleanup) { Log.Error("HUD video cleanup failed: " + cleanup); }
            }
        }

        private void RefreshFrame()
        {
            if (_disposed)
            {
                return;
            }
            var frame = _session.TakeFrame();
            if (frame != null)
            {
                var old = _displayedVideoFrame;
                _displayedVideoFrame = frame;
                _display = frame.Image;
                _setFrame(frame.Image);
                old?.Dispose();
            }
            if (_displayedVideoFrame?.Fresh == false) { ClearDisplay(); }
            if (_session.State == VideoState.Stopped)
            {
                if (_session.Error != null)
                {
                    Log.Error("HUD video: " + _session.Error);
                }
                Dispose();
            }
        }

        private void OnHudDisposed(object sender, EventArgs e) => Dispose();

        internal void Retire()
        {
            _startRequested = false; Interlocked.Increment(ref _playbackRevision);
            _session.Dispose();
        }

        public void Dispose()
        {
            if (_disposed)
            {
                return;
            }
            _disposed = true;
            Retire();
            _timer.Stop();
            _timer.Tick -= OnFrameTick;
            _timer.Dispose();
            _hud.Disposed -= OnHudDisposed;
            ClearDisplay();
        }
    }
}
