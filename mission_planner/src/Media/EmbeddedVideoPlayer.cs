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
        private string _streamUrl;
        private int _latencyMs = 100;
        private readonly VideoSession _session;
        private readonly System.Windows.Forms.Timer _frameTimer;
        private CancellationTokenRegistration _shutdown;
        private Bitmap _displayFrame;
        private PictureBox _videoBox;
        private Label _lblStatus;
        private TrackBar _trkLatency;
        private Label _lblLatencyValue;
        private Form _fullscreenForm;
        private PictureBox _fullscreenBox;
        private readonly bool _showControls;
        private readonly int _uiThread = Thread.CurrentThread.ManagedThreadId;
        private bool _isPlaying => _session.IsActive;

        public EmbeddedVideoPlayer(string title, string streamUrl, bool showControls = true)
            : this(title, streamUrl, showControls, CancellationToken.None, () => new GStreamerVideoPipeline()) { }

        internal EmbeddedVideoPlayer(string title, string streamUrl, bool showControls,
            CancellationToken shutdown, Func<IVideoPipeline> createPipeline)
        {
            _streamUrl = streamUrl ?? "";
            _showControls = showControls;
            _session = new VideoSession(createPipeline);
            _frameTimer = new System.Windows.Forms.Timer { Interval = 33 };
            _frameTimer.Tick += OnFrameTick;
            try
            {
                InitializeUI();
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

        private int ExtractUdpPort(string url)
        {
            if (string.IsNullOrEmpty(url))
            {
                return 5600;
            }
            var cleaned = url;
            if (cleaned.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
            {
                cleaned = cleaned.Substring(6);
            }
            cleaned = cleaned.Replace("@", "").TrimStart(':');
            return int.TryParse(cleaned, out int port) ? port : 5600;
        }

        private void ShutdownVideo()
        {
            _session.Dispose();
            try
            {
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
