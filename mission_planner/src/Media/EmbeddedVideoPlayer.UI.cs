// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
extern alias MPDrawing;

// ============================================================
// NOMAD Embedded Video Player - UI partial
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
    public partial class EmbeddedVideoPlayer
    {
        private void InitializeUI()
        {
            BackColor = Color.Black;
            Dock = DockStyle.Fill;

            _videoBox = new PictureBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                SizeMode = PictureBoxSizeMode.Zoom,
            };
            _videoBox.DoubleClick += (s, e) => ToggleFullscreen();
            _videoBox.Paint += OnVideoPaint;

            _depthPollTimer = new System.Windows.Forms.Timer { Interval = 200 };
            _depthPollTimer.Tick += async (s, e) => await PollCenterDepthAsync();
            _depthPollTimer.Start();

            _lblStatus = new Label
            {
                Text = TryInitializeGStreamer() ? "Ready - Click Play" : "GStreamer not found",
                Dock = DockStyle.Bottom,
                Height = 22,
                ForeColor = Color.Gray,
                BackColor = Color.FromArgb(30, 30, 30),
                TextAlign = ContentAlignment.MiddleCenter,
                Font = new Font("Segoe UI", 8),
            };

            var ctrlPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 60,
                BackColor = Color.FromArgb(35, 35, 38),
                Padding = new Padding(5),
            };

            var btnPlay = CreateButton("Play", 10, 5, 55, Color.FromArgb(60, 120, 60));
            var btnStop = CreateButton("Stop", 70, 5, 55, Color.FromArgb(120, 60, 60));
            var btnFull = CreateButton("Full", 130, 5, 50, Color.FromArgb(70, 70, 75));
            var btnVLC = CreateButton("VLC", 185, 5, 45, Color.FromArgb(70, 70, 75));
            var btnSnap = CreateButton("Snap", 235, 5, 50, Color.FromArgb(80, 80, 120));

            btnPlay.Click += (s, e) => StartStream();
            btnStop.Click += (s, e) => StopStream();
            btnFull.Click += (s, e) => ToggleFullscreen();
            btnVLC.Click += (s, e) => OpenExternal();
            btnSnap.Click += (s, e) => TakeSnapshot();

            var lblTopic = new Label { Text = "Topic:", Location = new Point(295, 8), ForeColor = Color.Cyan, AutoSize = true, Font = new Font("Segoe UI", 8) };
            _cmbTopic = new ComboBox
            {
                Location = new Point(340, 5),
                Size = new Size(160, 22),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(45, 45, 48),
                ForeColor = Color.White,
            };
            _cmbTopic.SelectedIndexChanged += (s, e) => UiAsync.Run(this, SwitchTopicAsync, nameof(SwitchTopicAsync));

            var btnRefresh = CreateButton("...", 505, 5, 30, Color.FromArgb(60, 60, 65));
            btnRefresh.Click += (s, e) => UiAsync.Run(this, () => RefreshTopicsAsync(autoSelectRgb: false), nameof(RefreshTopicsAsync));

            var lblLat = new Label { Text = "Latency:", Location = new Point(10, 33), ForeColor = Color.Gray, AutoSize = true, Font = new Font("Segoe UI", 8) };
            _trkLatency = new TrackBar
            {
                Location = new Point(65, 30),
                Size = new Size(160, 25),
                Minimum = 20,
                Maximum = 500,
                Value = _latencyMs,
                TickFrequency = 50,
            };
            _trkLatency.ValueChanged += (s, e) =>
            {
                _lblLatencyValue.Text = $"{_trkLatency.Value}ms";
            };
            _lblLatencyValue = new Label { Text = $"{_latencyMs}ms", Location = new Point(230, 33), ForeColor = Color.LightGray, AutoSize = true, Font = new Font("Segoe UI", 8) };

            var btnApplyLatency = CreateButton("Apply", 275, 30, 55, Color.FromArgb(0, 100, 140));
            btnApplyLatency.Font = new Font("Segoe UI", 7.5f);
            btnApplyLatency.Click += (s, e) => UiAsync.Run(this, async () =>
            {
                int newLatency = _trkLatency.Value;
                if (newLatency == _latencyMs && _isPlaying) return;

                _latencyMs = newLatency;
                btnApplyLatency.Enabled = false;
                btnApplyLatency.Text = "...";

                try
                {
                    if (_isPlaying)
                    {
                        await RestartStreamAsync();
                        _lblStatus.Text = $"Latency: {_latencyMs}ms ({(_latencyMs <= 100 ? "low-latency" : "smooth")})";
                        _lblStatus.ForeColor = Color.Cyan;
                    }
                    else
                    {
                        _lblStatus.Text = $"Latency set to {_latencyMs}ms (applied on Play)";
                        _lblStatus.ForeColor = Color.DarkCyan;
                    }
                }
                finally
                {
                    btnApplyLatency.Enabled = true;
                    btnApplyLatency.Text = "Apply";
                }
            }, "ApplyVideoLatency");

            _chkDetections = new CheckBox
            {
                Text = "HSV (ROS2)",
                Location = new Point(340, 33),
                AutoSize = true,
                ForeColor = Color.FromArgb(255, 180, 60),
                BackColor = Color.Transparent,
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                Checked = false,
            };
            _chkDetections.CheckedChanged += (s, e) => UiAsync.Run(this, async () =>
            {
                if (_syncingOverlayState) return;

                bool requestedEnabled = _chkDetections.Checked;

                _lblStatus.Text = requestedEnabled
                    ? "Enabling HSV overlay..."
                    : "Disabling HSV overlay...";
                _lblStatus.ForeColor = Color.Yellow;

                try
                {
                    await SetHsvModeAsync(requestedEnabled);
                    _overlayEnabled = requestedEnabled;
                    _lblStatus.Text = requestedEnabled
                        ? "HSV overlay enabled"
                        : "HSV overlay disabled";
                    _lblStatus.ForeColor = requestedEnabled ? Color.LimeGreen : Color.Gray;
                }
                catch (Exception ex)
                {
                    _lblStatus.Text = $"HSV toggle failed: {ex.Message}";
                    _syncingOverlayState = true;
                    _chkDetections.Checked = !requestedEnabled;
                    _syncingOverlayState = false;
                    _overlayEnabled = _chkDetections.Checked;
                }
            }, "ToggleHsvOverlay");

            ctrlPanel.Controls.AddRange(new Control[] { btnPlay, btnStop, btnFull, btnVLC, btnSnap, lblTopic, _cmbTopic, btnRefresh, lblLat, _trkLatency, _lblLatencyValue, btnApplyLatency, _chkDetections });

            Controls.Add(_videoBox);
            // Chrome-less mode: only the video surface (double-click still
            // toggles fullscreen). The status label and control strip exist but
            // stay un-parented so status writes elsewhere in the class are safe.
            if (_showControls)
            {
                Controls.Add(_lblStatus);
                Controls.Add(ctrlPanel);
            }

            _topics.Add(("/zed/zed_node/rgb/color/rect/image", "Left Rect"));
            PopulateTopics();
        }

        private Button CreateButton(string text, int x, int y, int width, Color color)
        {
            return new Button
            {
                Text = text,
                Location = new Point(x, y),
                Size = new Size(width, 24),
                FlatStyle = FlatStyle.Flat,
                BackColor = color,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
        }

        private void PopulateTopics()
        {
            _suppressTopicChange = true;
            try
            {
                _cmbTopic.Items.Clear();
                foreach (var (_, display) in _topics)
                    _cmbTopic.Items.Add(display);
                if (_cmbTopic.Items.Count > 0)
                    _cmbTopic.SelectedIndex = 0;
            }
            finally
            {
                _suppressTopicChange = false;
            }
        }

        private void OnVideoPaint(object sender, PaintEventArgs e)
        {
            if (_videoBox.Image == null) return;

            int cx = _videoBox.Width / 2;
            int cy = _videoBox.Height / 2;
            const int arm = 18;
            const int gap = 5;

            e.Graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

            using (var outline = new Pen(Color.FromArgb(140, Color.Black), 4f))
            using (var fore = new Pen(Color.FromArgb(220, Color.Cyan), 1.5f))
            {
                foreach (var pen in new[] { outline, fore })
                {
                    e.Graphics.DrawLine(pen, cx - arm, cy, cx - gap, cy);
                    e.Graphics.DrawLine(pen, cx + gap, cy, cx + arm, cy);
                    e.Graphics.DrawLine(pen, cx, cy - arm, cx, cy - gap);
                    e.Graphics.DrawLine(pen, cx, cy + gap, cx, cy + arm);
                }
            }

            DrawCenterDepthReadout(e.Graphics);
        }

        private void DrawCenterDepthReadout(Graphics g)
        {
            string text;
            bool stale = _centerDepthM == null
                         || (DateTime.UtcNow - _centerDepthStamp).TotalSeconds > 2.0;
            if (_centerDepthM is double d && !stale)
                text = $"Range: {d:0.00} m";
            else
                text = "Range: --";

            using (var font = new Font("Consolas", 10f, FontStyle.Bold))
            {
                var size = g.MeasureString(text, font);
                int pad = 6;
                int margin = 8;
                var rect = new RectangleF(
                    _videoBox.Width - size.Width - pad * 2 - margin,
                    _videoBox.Height - size.Height - pad * 2 - margin,
                    size.Width + pad * 2,
                    size.Height + pad * 2);

                using (var bg = new SolidBrush(Color.FromArgb(160, 0, 0, 0)))
                    g.FillRectangle(bg, rect);

                using (var fg = new SolidBrush(stale ? Color.LightGray : Color.LightGreen))
                    g.DrawString(text, font, fg, rect.X + pad, rect.Y + pad);
            }
        }

        public void ToggleFullscreen()
        {
            if (_fullscreenForm != null && !_fullscreenForm.IsDisposed)
            {
                if (_fullscreenBox != null && !_fullscreenBox.IsDisposed)
                {
                    _fullscreenBox.Image = null;
                }
                _fullscreenForm.Close();
                _fullscreenForm = null;
                _fullscreenBox = null;
                return;
            }

            _fullscreenForm = new Form
            {
                FormBorderStyle = FormBorderStyle.None,
                WindowState = FormWindowState.Maximized,
                BackColor = Color.Black,
                KeyPreview = true,
            };
            _fullscreenForm.KeyDown += (s, e) => { if (e.KeyCode == Keys.Escape) ToggleFullscreen(); };

            _fullscreenBox = new PictureBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                SizeMode = PictureBoxSizeMode.Zoom,
            };
            _fullscreenBox.DoubleClick += (s, e) => ToggleFullscreen();

            lock (_frameBufferLock)
            {
                if (_frameBuffers != null &&
                    _displayBufferIndex >= 0 &&
                    _displayBufferIndex < _frameBuffers.Length)
                {
                    _fullscreenBox.Image = _frameBuffers[_displayBufferIndex];
                }
            }

            _fullscreenForm.Controls.Add(_fullscreenBox);
            _fullscreenForm.Show();
        }

        public void OpenExternal()
        {
            string vlcArgs;

            if (_streamUrl.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
            {
                var port = ExtractUdpPort(_streamUrl);
                var sdp = $"v=0\no=- 0 0 IN IP4 127.0.0.1\ns=Stream\nc=IN IP4 127.0.0.1\nt=0 0\n" +
                          $"m=video {port} RTP/AVP 96\na=rtpmap:96 H264/90000";
                var sdpPath = Path.Combine(Path.GetTempPath(), "nomad_stream.sdp");
                File.WriteAllText(sdpPath, sdp);
                vlcArgs = $"--network-caching={_latencyMs} \"{sdpPath}\"";
            }
            else
            {
                vlcArgs = $"--network-caching={_latencyMs} --rtsp-tcp \"{_streamUrl}\"";
            }

            var vlcPaths = new[] { "vlc", @"C:\Program Files\VideoLAN\VLC\vlc.exe", @"C:\Program Files (x86)\VideoLAN\VLC\vlc.exe" };

            foreach (var path in vlcPaths)
            {
                try
                {
                    System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo { FileName = path, Arguments = vlcArgs, UseShellExecute = true });
                    _lblStatus.Text = "Opened in VLC";
                    return;
                }
                catch { }
            }

            MessageBox.Show($"VLC not found.\n\nStream URL: {_streamUrl}", "VLC Not Found", MessageBoxButtons.OK);
        }

        public void TakeSnapshot()
        {
            Bitmap source = null;
            lock (_frameBufferLock)
            {
                if (_frameBuffers != null &&
                    _displayBufferIndex >= 0 &&
                    _displayBufferIndex < _frameBuffers.Length)
                {
                    source = _frameBuffers[_displayBufferIndex];
                }
            }
            if (source == null) { _lblStatus.Text = "No frame available"; return; }
            var path = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.Desktop), $"NOMAD_{DateTime.Now:yyyyMMdd_HHmmss}.png");
            using (var snap = (Bitmap)source.Clone())
            {
                snap.Save(path);
            }
            _lblStatus.Text = $"Saved: {Path.GetFileName(path)}";
            _lblStatus.ForeColor = Color.LimeGreen;
        }

        public void UpdateStreamUrl(string newUrl)
        {
            var wasPlaying = _isPlaying;
            StopStream();
            _streamUrl = newUrl;
            ParseApiUrl(newUrl);
            if (wasPlaying) StartStream();
        }
    }
}
