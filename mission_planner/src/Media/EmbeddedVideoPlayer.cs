// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.Threading;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer : UserControl
    {
        private VideoInputProfile _input;
        private readonly VideoSession _session;
        private readonly ToolTip _videoToolTips = new ToolTip();
        private readonly System.Windows.Forms.Timer _frameTimer;
        private CancellationTokenRegistration _shutdown;
        private Bitmap _displayFrame;
        private VideoFrame _displayedVideoFrame;
        private readonly NOMADConfig _aircraft;
        private PictureBox _videoBox;
        private Label _lblStatus;
        private Form _fullscreenForm;
        private PictureBox _fullscreenBox;
        private readonly bool _showControls;
        private readonly int _uiThread = Thread.CurrentThread.ManagedThreadId;
        private bool _isPlaying => _session.IsActive;

        internal EmbeddedVideoPlayer(VideoInputProfile input, bool showControls,
            CancellationToken shutdown, Func<IVideoPipeline> createPipeline, NOMADConfig aircraft = null,
            Func<string, CancellationToken, IVideoDetector> createDetector = null,
            Func<CancellationToken, IVisualTracker> createTracker = null)
        {
            _input = input.Copy();
            _aircraft = aircraft;
            _showControls = showControls;
            _session = new VideoSession(createPipeline, createDetector,
                showControls ? createTracker ?? VisualTrackerFactory.Create : null);
            _frameTimer = new System.Windows.Forms.Timer { Interval = 33 };
            _frameTimer.Tick += OnFrameTick;
            try
            {
                InitializeUI();
                InitializeVision();
                if (shutdown.IsCancellationRequested)
                {
                    Dispose();
                    return;
                }
                _shutdown = shutdown.Register(OnPluginShutdown);
            }
            catch (ObjectDisposedException) when (shutdown.IsCancellationRequested)
            {
                Dispose();
            }
            catch
            {
                Dispose();
                throw;
            }
        }

        protected override void OnHandleCreated(EventArgs e)
        {
            base.OnHandleCreated(e);
            if (!_showControls)
            {
                StartStream();
            }
        }

        private void ShutdownVideo()
        {
            RetireTracking("Video closed");
            GimbalController.ManualInput -= OnManualGimbalInput;
            OutputController.ConnectionChanged -= OnAircraftChanged;
            try
            {
                _session.Dispose();
            }
            finally
            {
                _frameTimer.Stop();
                _frameTimer.Tick -= OnFrameTick;
                _frameTimer.Dispose();
                ClearVideoDisplayAndDisposeBuffers();
            }
        }

        private void OnPluginShutdown()
        {
            RetireTracking("Plugin stopped");
            _session.Dispose();
            if (Thread.CurrentThread.ManagedThreadId == _uiThread)
            {
                Dispose();
                return;
            }
            // Worker cleanup is already complete; the UI releases its controls on its own thread.
            if (IsHandleCreated && !IsDisposed)
            {
                UiAsync.RunSync(this, () =>
                {
                    if (!IsDisposed)
                    {
                        Dispose();
                    }
                }, "video shutdown");
            }
        }

        private void ValidateUiThread()
        {
            if (Thread.CurrentThread.ManagedThreadId != _uiThread)
            {
                throw new InvalidOperationException("Video controls must be used on their UI thread");
            }
        }

        protected override void Dispose(bool disposing)
        {
            try
            {
                if (disposing)
                {
                    _shutdown.Dispose();
                    try
                    {
                        ShutdownVideo();
                    }
                    finally
                    {
                        _videoToolTips.Dispose();
                        _fullscreenForm?.Dispose();
                    }
                }
            }
            finally
            {
                base.Dispose(disposing);
            }
        }
    }
}
