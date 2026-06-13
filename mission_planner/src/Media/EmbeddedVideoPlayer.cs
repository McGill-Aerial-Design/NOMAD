// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
extern alias MPDrawing;

// ============================================================
// NOMAD Embedded Video Player - GStreamer Implementation
// ============================================================
// Uses Mission Planner's built-in GStreamer for RTSP/UDP video streaming.
// Features: Topic switching via API, fullscreen mode, latency control.
// Falls back to external VLC/FFplay if GStreamer is unavailable.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Net.Http;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner.Utilities;
using Newtonsoft.Json.Linq;
using MPBitmap = MPDrawing::System.Drawing.Bitmap;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Embedded video player using Mission Planner's GStreamer wrapper.
    /// </summary>
    public partial class EmbeddedVideoPlayer : UserControl
    {
        private string _streamUrl;
        private string _apiBaseUrl;
        private int _latencyMs = 100;
        private bool _isPlaying;

        private GStreamer _gst;
        private PictureBox _videoBox;
        private Label _lblStatus;
        private TrackBar _trkLatency;
        private Label _lblLatencyValue;
        private ComboBox _cmbTopic;
        private Form _fullscreenForm;
        private PictureBox _fullscreenBox;
        private List<(string Name, string Display)> _topics = new List<(string, string)>();

        private CheckBox _chkDetections;
        private bool _overlayEnabled;
        private bool _syncingOverlayState;

        private System.Windows.Forms.Timer _depthPollTimer;
        private double? _centerDepthM;
        private DateTime _centerDepthStamp;

        private readonly SemaphoreSlim _lifecycleLock = new SemaphoreSlim(1, 1);
        private int _streamGeneration;
        private volatile bool _stopping;
        private bool _suppressTopicChange;

        private const int FrameBufferCount = 3;
        private readonly object _frameBufferLock = new object();
        private Bitmap[] _frameBuffers;
        private int _frameBufferWidth;
        private int _frameBufferHeight;
        private int _displayBufferIndex = -1;
        private int _pendingBufferIndex = -1;
        private int _nextBufferIndex;
        private int _frameCount;

        private readonly bool _showControls;

        /// <summary>
        /// Creates an embedded video player. With showControls=false the player
        /// is chrome-less (no buttons/topic/latency bar) and auto-plays the
        /// default topic — used for the dashboard mini preview.
        /// </summary>
        public EmbeddedVideoPlayer(string title, string streamUrl, bool showControls = true, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _streamUrl = streamUrl;
            _showControls = showControls;
            ParseApiUrl(streamUrl);
            InitializeUI();

            this.HandleCreated += (s, e) => UiAsync.Run(this, async () =>
            {
                if (_topics.Count == 0)
                {
                    await RefreshTopicsAsync(autoSelectRgb: true);
                }

                await SyncOverlayStatusAsync();

                if (!_showControls && !_isPlaying && !IsDisposed)
                {
                    StartStream();
                }
            }, "EmbeddedVideoHandleCreated");
        }


        private void ParseApiUrl(string rtspUrl)
        {
            try
            {
                var uri = new Uri(rtspUrl);
                _apiBaseUrl = $"http://{uri.Host}:8000";
            }
            catch
            {
                _apiBaseUrl = NOMADConfig.Load().EffectiveBaseUrl;
            }
        }

        private int ExtractUdpPort(string url)
        {
            var cleaned = url.Replace("udp://", "").Replace("@", "").TrimStart(':');
            return int.TryParse(cleaned, out int port) ? port : 5600;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                if (_overlayEnabled)
                {
                    try
                    {
                        JetsonApiService.ApiClient.PostAsync(
                            $"{_apiBaseUrl}/api/video/overlay/disable", null)
                            .ConfigureAwait(false);
                    }
                    catch { }
                }
                try { _depthPollTimer?.Stop(); _depthPollTimer?.Dispose(); } catch { }
                StopStream();
                _lifecycleLock.Dispose();
                if (_fullscreenForm != null && !_fullscreenForm.IsDisposed)
                    _fullscreenForm.Close();
            }
            base.Dispose(disposing);
        }
    }
}
