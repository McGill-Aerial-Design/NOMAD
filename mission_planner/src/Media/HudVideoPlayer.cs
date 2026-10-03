// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;

using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    internal sealed class HudVideoPlayer : IDisposable
    {
        private readonly Control _hud;
        private readonly Action<Bitmap> _setFrame;
        private readonly Func<Image> _getFrame;
        private readonly VideoSession _session;
        private readonly Timer _timer = new Timer { Interval = 33 };
        private Bitmap _display;
        private bool _disposed;

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

        public bool IsActive => _session.IsActive;

        public void Start(string pipeline)
        {
            if (_session.Start(pipeline))
            {
                _timer.Start();
            }
        }

        private void OnFrameTick(object sender, EventArgs e)
        {
            if (_disposed)
            {
                return;
            }
            var frame = _session.TakeFrame();
            if (frame != null)
            {
                var old = _display;
                _display = frame;
                _setFrame(frame);
                old?.Dispose();
            }
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

        public void Dispose()
        {
            if (_disposed)
            {
                return;
            }
            _disposed = true;
            _session.Dispose();
            _timer.Stop();
            _timer.Tick -= OnFrameTick;
            _timer.Dispose();
            _hud.Disposed -= OnHudDisposed;
            if (!_hud.IsDisposed && ReferenceEquals(_getFrame(), _display))
            {
                _setFrame(null);
            }
            _display?.Dispose();
            _display = null;
        }
    }
}
