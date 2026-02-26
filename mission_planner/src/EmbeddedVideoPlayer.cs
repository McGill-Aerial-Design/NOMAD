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
using System.Drawing.Drawing2D;
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
    public class EmbeddedVideoPlayer : UserControl
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
        private MPBitmap _lastFrame;
        private List<(string Name, string Display)> _topics = new List<(string, string)>();
        
        // Detection overlay
        private CheckBox _chkDetections;
        private bool _overlayEnabled;
        private volatile List<DetectionBox> _currentDetections = new List<DetectionBox>();
        private CancellationTokenSource _detectionPollCts;
        private volatile int _sourceFrameWidth;
        private volatile int _sourceFrameHeight;
        // Cached GDI objects for overlay rendering (avoid per-frame allocation)
        private Font _overlayFont;
        private Font _overlayBadgeFont;
        
        /// <summary>
        /// Creates an embedded video player.
        /// </summary>
        public EmbeddedVideoPlayer(string title, string streamUrl, bool showControls = true, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _streamUrl = streamUrl;
            ParseApiUrl(streamUrl);
            InitializeUI();
            
            // Auto-fetch topics when control is loaded
            this.HandleCreated += async (s, e) =>
            {
                // Only auto-fetch if no topics exist
                if (_topics.Count == 0)
                {
                    await RefreshTopicsAsync(autoSelectRgb: true);
                }
            };
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
                _apiBaseUrl = "http://100.85.121.98:8000";
            }
        }
        
        private void InitializeUI()
        {
            BackColor = Color.Black;
            Dock = DockStyle.Fill;
            
            // Video display
            _videoBox = new PictureBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                SizeMode = PictureBoxSizeMode.Zoom,
            };
            _videoBox.DoubleClick += (s, e) => ToggleFullscreen();
            
            // Status bar
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
            
            // Control panel
            var ctrlPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 60,
                BackColor = Color.FromArgb(35, 35, 38),
                Padding = new Padding(5),
            };
            
            // Row 1: Buttons
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
            
            // Topic selector
            var lblTopic = new Label { Text = "Topic:", Location = new Point(295, 8), ForeColor = Color.Cyan, AutoSize = true, Font = new Font("Segoe UI", 8) };
            _cmbTopic = new ComboBox
            {
                Location = new Point(340, 5),
                Size = new Size(160, 22),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(45, 45, 48),
                ForeColor = Color.White,
            };
            _cmbTopic.SelectedIndexChanged += async (s, e) => await SwitchTopicAsync();
            
            var btnRefresh = CreateButton("...", 505, 5, 30, Color.FromArgb(60, 60, 65));
            btnRefresh.Click += async (s, e) => await RefreshTopicsAsync(autoSelectRgb: false);
            
            // Row 2: Latency slider + Apply button
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
            btnApplyLatency.Click += async (s, e) =>
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
                        StopStream();
                        // Wait for GStreamer to fully release native resources
                        await System.Threading.Tasks.Task.Delay(500);
                        StartStream();
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
            };

            // Detection overlay checkbox
            _chkDetections = new CheckBox
            {
                Text = "YOLO",
                Location = new Point(340, 33),
                AutoSize = true,
                ForeColor = Color.FromArgb(255, 180, 60),
                BackColor = Color.Transparent,
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                Checked = false,
            };
            _chkDetections.CheckedChanged += (s, e) =>
            {
                _overlayEnabled = _chkDetections.Checked;
                if (_overlayEnabled)
                    StartDetectionPolling();
                else
                    StopDetectionPolling();
            };

            ctrlPanel.Controls.AddRange(new Control[] { btnPlay, btnStop, btnFull, btnVLC, btnSnap, lblTopic, _cmbTopic, btnRefresh, lblLat, _trkLatency, _lblLatencyValue, btnApplyLatency, _chkDetections });
            
            Controls.Add(_videoBox);
            Controls.Add(_lblStatus);
            Controls.Add(ctrlPanel);
            
            // Initialize with default topic
            _topics.Add(("/zed/zed_node/rgb/image_rect_color", "RGB Color"));
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
            _cmbTopic.Items.Clear();
            foreach (var (_, display) in _topics)
                _cmbTopic.Items.Add(display);
            if (_cmbTopic.Items.Count > 0)
                _cmbTopic.SelectedIndex = 0;
        }
        
        private async System.Threading.Tasks.Task RefreshTopicsAsync(bool autoSelectRgb = false)
        {
            try
            {
                _lblStatus.Text = "Refreshing topics...";
                _lblStatus.ForeColor = Color.Yellow;
                
                var json = await JetsonApiService.ApiClient.GetStringAsync($"{_apiBaseUrl}/api/video/topics");
                var data = JObject.Parse(json);
                var arr = data["topics"] as JArray;
                
                _topics.Clear();
                if (arr != null)
                {
                    foreach (var t in arr)
                    {
                        var name = t["name"]?.ToString();
                        var disp = t["display_name"]?.ToString() ?? name;
                        if (!string.IsNullOrEmpty(name))
                            _topics.Add((name, disp));
                    }
                }
                
                if (_topics.Count == 0)
                    _topics.Add(("/zed/zed_node/rgb/image_rect_color", "RGB Color"));
                
                PopulateTopics();
                _lblStatus.Text = $"Found {_topics.Count} topics";
                _lblStatus.ForeColor = Color.LimeGreen;
                
                // Auto-select rgb/image_rect_color topic if requested
                if (autoSelectRgb && _topics.Count > 0)
                {
                    int rgbIndex = _topics.FindIndex(t => 
                        t.Name.IndexOf("rgb/image_rect_color", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        t.Name.IndexOf("rgb_image_rect_color", StringComparison.OrdinalIgnoreCase) >= 0);
                    
                    if (rgbIndex >= 0)
                    {
                        _cmbTopic.SelectedIndex = rgbIndex;
                        // Switch to the RGB topic on the server
                        await SwitchTopicAsync();
                    }
                }
            }
            catch (Exception ex)
            {
                _lblStatus.Text = $"Refresh failed: {ex.Message}";
                _lblStatus.ForeColor = Color.Red;
            }
        }
        
        private async System.Threading.Tasks.Task SwitchTopicAsync()
        {
            if (_cmbTopic.SelectedIndex < 0 || _cmbTopic.SelectedIndex >= _topics.Count) return;
            
            var (name, display) = _topics[_cmbTopic.SelectedIndex];
            _lblStatus.Text = $"Switching to {display}...";
            _lblStatus.ForeColor = Color.Yellow;
            
            try
            {
                var resp = await JetsonApiService.ApiClient.PostAsync($"{_apiBaseUrl}/api/video/source?topic={Uri.EscapeDataString(name)}", null);
                if (!resp.IsSuccessStatusCode)
                {
                    _lblStatus.Text = $"Switch failed: {resp.StatusCode}";
                    _lblStatus.ForeColor = Color.Red;
                    return;
                }
                
                // Brief delay then restart stream to pick up new topic
                await System.Threading.Tasks.Task.Delay(300);
                if (_isPlaying)
                {
                    StopStream();
                    await System.Threading.Tasks.Task.Delay(200);
                    StartStream();
                }
                
                _lblStatus.Text = $"Streaming: {display}";
                _lblStatus.ForeColor = Color.LimeGreen;
            }
            catch (Exception ex)
            {
                _lblStatus.Text = $"Switch error: {ex.Message}";
                _lblStatus.ForeColor = Color.Red;
            }
        }
        
        private bool TryInitializeGStreamer()
        {
            try
            {
                var gstPath = GStreamer.LookForGstreamer();
                return !string.IsNullOrWhiteSpace(gstPath) && GStreamer.GstLaunchExists;
            }
            catch { return false; }
        }
        
        private string BuildGStreamerPipeline()
        {
            // FROM WORKING COMMIT 98fea60 - Uses decodebin3 for automatic codec detection
            // Requirements:
            // 1. appsink MUST be named "outsink" (Mission Planner looks for this)
            // 2. format=BGRA (32-bit matches Windows Forms bitmap creation)
            // 3. decodebin3 handles H264 decoding automatically
            //
            // Latency control:
            //   - rtspsrc latency: jitter buffer in ms (how much RTP data to buffer)
            //   - queue: buffer depth controls smoothness vs latency tradeoff
            //   - sync: when true, frames display at their PTS; when false, push ASAP
            //   Low latency (<100ms): minimal buffer, sync=false, drop old frames
            //   High latency (>100ms): larger buffer for smoother playback
            
            // Queue depth scales with latency to allow smoother playback at higher settings
            int queueBuffers = _latencyMs <= 100 ? 1 : Math.Min(_latencyMs / 50, 10);
            string leaky = _latencyMs <= 100 ? "leaky=2" : "leaky=0";
            // Enable clock sync for higher latency (smooth, even playback)
            string syncVal = _latencyMs <= 100 ? "false" : "true";
            
            if (_streamUrl.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
            {
                var port = ExtractUdpPort(_streamUrl);
                return $"udpsrc port={port} buffer-size=90000 ! " +
                       $"application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264 ! " +
                       $"decodebin3 ! queue max-size-buffers={queueBuffers} {leaky} ! " +
                       $"videoconvert ! video/x-raw,format=BGRA ! " +
                       $"appsink name=outsink sync={syncVal}";
            }
            
            // RTSP pipeline - latency controls the rtspsrc jitter buffer
            return $"rtspsrc location={_streamUrl} latency={_latencyMs} udp-reconnect=1 timeout=0 do-retransmission=false ! " +
                   $"application/x-rtp ! decodebin3 ! queue max-size-buffers={queueBuffers} {leaky} ! " +
                   $"videoconvert ! video/x-raw,format=BGRA ! " +
                   $"appsink name=outsink sync={syncVal}";
        }
        
        public void StartStream()
        {
            if (_isPlaying) return;
            
            if (!TryInitializeGStreamer())
            {
                _lblStatus.Text = "GStreamer not available - use VLC";
                _lblStatus.ForeColor = Color.Orange;
                return;
            }
            
            try
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Starting stream to {_streamUrl}");
                _gst = new GStreamer();
                _gst.OnNewImage += OnGstNewImage;
                
                var pipeline = BuildGStreamerPipeline();
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Pipeline: {pipeline}");
                
                _gst.Start(pipeline);
                _isPlaying = true;
                _lblStatus.Text = "Connecting...";
                _lblStatus.ForeColor = Color.Yellow;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Start error - {ex}");
                _lblStatus.Text = $"Error: {ex.Message}";
                _lblStatus.ForeColor = Color.Red;
            }
        }
        
        public void StopStream()
        {
            if (!_isPlaying && _gst == null) return;
            
            try
            {
                if (_gst != null)
                {
                    // Unhook event first to prevent callbacks during shutdown
                    try { _gst.OnNewImage -= OnGstNewImage; } catch { }
                    
                    // Stop GStreamer pipeline
                    try { _gst.Stop(); } catch { }
                    
                    // Give GStreamer time to release resources
                    System.Threading.Thread.Sleep(100);
                    
                    _gst = null;
                }
            }
            catch { }
            
            _isPlaying = false;

            // Clear video display to prevent painting stale/freed frame data
            try
            {
                var oldImage = _videoBox?.Image;
                if (_videoBox != null) _videoBox.Image = null;
                oldImage?.Dispose();
                if (_fullscreenBox != null && !_fullscreenBox.IsDisposed)
                {
                    var oldFull = _fullscreenBox.Image;
                    _fullscreenBox.Image = null;
                    oldFull?.Dispose();
                }
            }
            catch { }

            _lblStatus.Text = "Stopped";
            _lblStatus.ForeColor = Color.Gray;
        }
        
        private int _frameCount = 0;
        
        /// <summary>
        /// GStreamer frame callback - invoked on background thread.
        /// Uses MPBitmap.LockBits() to access raw pixel data directly (PROVEN WORKING from commit 39f9f48).
        /// </summary>
        private void OnGstNewImage(object sender, MPBitmap frame)
        {
            if (frame == null)
            {
                System.Diagnostics.Debug.WriteLine("NOMAD Video: Received null frame");
                return;
            }
            
            if (frame.Width <= 0 || frame.Height <= 0)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Invalid frame: {frame.Width}x{frame.Height}");
                return;
            }
            
            _frameCount++;
            if (_frameCount % 30 == 1)
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Frame #{_frameCount} - {frame.Width}x{frame.Height}");
            
            // Marshal to UI thread
            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => OnGstNewImage(sender, frame)));
                return;
            }
            
            if (IsDisposed || _videoBox == null) return;
            
            try
            {
                // Copy pixel data into a managed Bitmap.
                // We MUST NOT wrap frame.LockBits().Scan0 directly because
                // GStreamer can free that buffer when the pipeline stops,
                // causing an AccessViolationException in PictureBox.OnPaint.
                var lockData = frame.LockBits(Rectangle.Empty, null, SkiaSharp.SKColorType.Bgra8888);
                var displayBitmap = new Bitmap(
                    frame.Width,
                    frame.Height,
                    PixelFormat.Format32bppPArgb);
                
                var bmpData = displayBitmap.LockBits(
                    new Rectangle(0, 0, frame.Width, frame.Height),
                    System.Drawing.Imaging.ImageLockMode.WriteOnly,
                    PixelFormat.Format32bppPArgb);
                
                try
                {
                    // Source stride: BGRA8888 = 4 bytes per pixel, no padding
                    // (MPBitmap.LockBits with SKColorType.Bgra8888 gives tightly packed rows)
                    var srcStride = 4 * frame.Width;
                    var dstStride = bmpData.Stride;
                    var rowBytes = Math.Min(srcStride, dstStride);
                    unsafe
                    {
                        byte* src = (byte*)lockData.Scan0;
                        byte* dst = (byte*)bmpData.Scan0;
                        for (int y = 0; y < frame.Height; y++)
                        {
                            Buffer.MemoryCopy(src + y * srcStride, dst + y * dstStride, dstStride, rowBytes);
                        }
                    }
                }
                finally
                {
                    displayBitmap.UnlockBits(bmpData);
                }
                
                // Update video display
                var oldImage = _videoBox.Image;
                
                // Track source frame dimensions for bbox normalization
                _sourceFrameWidth = frame.Width;
                _sourceFrameHeight = frame.Height;
                
                // Draw detection bounding boxes if overlay is enabled
                if (_overlayEnabled)
                    DrawDetectionOverlay(displayBitmap);
                
                _videoBox.Image = displayBitmap;
                oldImage?.Dispose();
                
                // Update fullscreen if active
                if (_fullscreenBox != null && !_fullscreenBox.IsDisposed)
                {
                    var oldFull = _fullscreenBox.Image;
                    _fullscreenBox.Image = (Bitmap)displayBitmap.Clone();
                    oldFull?.Dispose();
                }
                
                // Store for snapshots
                var oldLastFrame = _lastFrame;
                _lastFrame = (MPBitmap)frame.Clone();
                oldLastFrame?.Dispose();
                
                // Update status periodically
                if (_frameCount % 30 == 1)
                {
                    _lblStatus.Text = $"Streaming {frame.Width}x{frame.Height}";
                    _lblStatus.ForeColor = Color.LimeGreen;
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Frame error - {ex.Message}");
            }
        }
        
        public void ToggleFullscreen()
        {
            if (_fullscreenForm != null && !_fullscreenForm.IsDisposed)
            {
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
            if (_lastFrame == null) { _lblStatus.Text = "No frame available"; return; }
            var path = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.Desktop), $"NOMAD_{DateTime.Now:yyyyMMdd_HHmmss}.png");
            _lastFrame.Save(path);
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
        
        private int ExtractUdpPort(string url)
        {
            var cleaned = url.Replace("udp://", "").Replace("@", "").TrimStart(':');
            return int.TryParse(cleaned, out int port) ? port : 5600;
        }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                StopDetectionPolling();
                StopStream();
                _lastFrame?.Dispose();
                _overlayFont?.Dispose();
                _overlayBadgeFont?.Dispose();
                if (_fullscreenForm != null && !_fullscreenForm.IsDisposed)
                    _fullscreenForm.Close();
            }
            base.Dispose(disposing);
        }
        
        // ==================== Detection Overlay ====================
        
        /// <summary>
        /// Represents a single detected object bounding box for overlay rendering.
        /// </summary>
        private class DetectionBox
        {
            public string Label;
            public float Confidence;
            /// <summary>Normalized bounding box [0-1] relative to source frame size.</summary>
            public float NormX, NormY, NormW, NormH;
        }
        
        private void StartDetectionPolling()
        {
            StopDetectionPolling();
            _detectionPollCts = new CancellationTokenSource();
            var ct = _detectionPollCts.Token;
            Task.Run(async () =>
            {
                while (!ct.IsCancellationRequested)
                {
                    try
                    {
                        var json = await JetsonApiService.ApiClient.GetStringAsync(
                            $"{_apiBaseUrl}/api/detections");
                        var data = JObject.Parse(json);
                        var current = data["current"]?["detections"] as JArray;
                        
                        if (current != null && current.Count > 0)
                        {
                            var boxes = new List<DetectionBox>();
                            int fw = _sourceFrameWidth;
                            int fh = _sourceFrameHeight;
                            foreach (var d in current)
                            {
                                // Use 2D bbox fields exclusively (pixel coords from ZED)
                                float bx = d["bbox_x"]?.Value<float>() ?? 0;
                                float by = d["bbox_y"]?.Value<float>() ?? 0;
                                float bw = d["bbox_w"]?.Value<float>() ?? 0;
                                float bh = d["bbox_h"]?.Value<float>() ?? 0;
                                
                                // Skip detections with no valid 2D bbox
                                if (bw <= 0 || bh <= 0) continue;
                                
                                // Normalize pixel coords to [0-1] using source frame dimensions
                                if (fw > 0 && fh > 0)
                                {
                                    bx /= fw;
                                    by /= fh;
                                    bw /= fw;
                                    bh /= fh;
                                }
                                
                                boxes.Add(new DetectionBox
                                {
                                    Label = d["label"]?.ToString() ?? "",
                                    Confidence = d["confidence"]?.Value<float>() ?? 0,
                                    NormX = bx,
                                    NormY = by,
                                    NormW = bw,
                                    NormH = bh,
                                });
                            }
                            _currentDetections = boxes;
                        }
                        else
                        {
                            _currentDetections = new List<DetectionBox>();
                        }
                    }
                    catch
                    {
                        // Silently ignore poll failures
                    }
                    
                    // Poll at ~5Hz
                    try { await Task.Delay(200, ct); } catch (OperationCanceledException) { break; }
                }
            }, ct);
        }
        
        private void StopDetectionPolling()
        {
            try
            {
                _detectionPollCts?.Cancel();
                _detectionPollCts?.Dispose();
            }
            catch { }
            _detectionPollCts = null;
            _currentDetections = new List<DetectionBox>();
        }
        
        /// <summary>
        /// Get the overlay color for a detection class label.
        /// </summary>
        private static Color GetDetectionOverlayColor(string label)
        {
            if (string.IsNullOrEmpty(label)) return Color.White;
            string lower = label.ToLowerInvariant();
            if (lower.Contains("red")) return Color.FromArgb(255, 60, 60);
            if (lower.Contains("blue")) return Color.FromArgb(60, 120, 255);
            if (lower.Contains("green")) return Color.FromArgb(60, 220, 60);
            if (lower.Contains("yellow")) return Color.FromArgb(255, 230, 50);
            if (lower.Contains("black")) return Color.FromArgb(160, 160, 160);
            if (lower.Contains("white")) return Color.FromArgb(255, 255, 255);
            return Color.FromArgb(220, 120, 255);
        }
        
        /// <summary>
        /// Draw detection bounding boxes and labels onto a bitmap.
        /// </summary>
        private void DrawDetectionOverlay(Bitmap bmp)
        {
            // Take a snapshot reference (volatile read is safe for reference types)
            var detections = _currentDetections;
            if (detections == null || detections.Count == 0) return;
            
            int w = bmp.Width;
            int h = bmp.Height;
            
            // Lazy-init cached fonts
            if (_overlayFont == null)
                _overlayFont = new Font("Segoe UI", 9, FontStyle.Bold);
            if (_overlayBadgeFont == null)
                _overlayBadgeFont = new Font("Segoe UI", 8, FontStyle.Bold);
            
            using (var g = Graphics.FromImage(bmp))
            {
                g.SmoothingMode = SmoothingMode.AntiAlias;
                g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.ClearTypeGridFit;
                
                foreach (var det in detections)
                {
                    // Convert normalized coords to pixel coords
                    float px = det.NormX * w;
                    float py = det.NormY * h;
                    float pw = det.NormW * w;
                    float ph = det.NormH * h;
                    
                    // Skip invalid boxes
                    if (pw < 2 || ph < 2) continue;
                    
                    var color = GetDetectionOverlayColor(det.Label);
                    
                    // Draw bounding box
                    using (var pen = new Pen(color, 2.0f))
                    {
                        g.DrawRectangle(pen, px, py, pw, ph);
                    }
                    
                    // Draw label background + text
                    string labelText = $"{det.Label} {det.Confidence:P0}";
                    var textSize = g.MeasureString(labelText, _overlayFont);
                    float labelY = py - textSize.Height - 2;
                    if (labelY < 0) labelY = py + ph + 2;
                    
                    // Semi-transparent background
                    using (var bgBrush = new SolidBrush(Color.FromArgb(180, 0, 0, 0)))
                    {
                        g.FillRectangle(bgBrush, px, labelY, textSize.Width + 4, textSize.Height);
                    }
                    
                    using (var textBrush = new SolidBrush(color))
                    {
                        g.DrawString(labelText, _overlayFont, textBrush, px + 2, labelY);
                    }
                }
                
                // Draw detection count badge in top-right corner
                string badge = $"{detections.Count} detections";
                using (var bgBrush = new SolidBrush(Color.FromArgb(160, 0, 0, 0)))
                using (var textBrush = new SolidBrush(Color.FromArgb(255, 180, 60)))
                {
                    var textSize = g.MeasureString(badge, _overlayBadgeFont);
                    float bx = w - textSize.Width - 8;
                    float by = 4;
                    g.FillRectangle(bgBrush, bx - 2, by, textSize.Width + 6, textSize.Height + 2);
                    g.DrawString(badge, _overlayBadgeFont, textBrush, bx, by + 1);
                }
            }
        }
    }
}
