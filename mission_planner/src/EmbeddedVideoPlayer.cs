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
        private Bitmap _lastFrame;
        private List<(string Name, string Display)> _topics = new List<(string, string)>();
        
        // Unified HSV toggle (ROS2 detection lifecycle + video overlay)
        private CheckBox _chkDetections;
        private bool _overlayEnabled;
        private bool _syncingOverlayState;
        
        // Stream lifecycle serialization - prevents overlapping native GStreamer teardown/startup
        private readonly SemaphoreSlim _lifecycleLock = new SemaphoreSlim(1, 1);
        private int _streamGeneration;
        private volatile bool _stopping;
        private bool _suppressTopicChange;
        
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

                await SyncOverlayStatusAsync();
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
                _apiBaseUrl = NOMADConfig.Load().EffectiveBaseUrl;
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
            _videoBox.Paint += OnVideoPaint;
            
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
            };

            // Detection overlay checkbox
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
            _chkDetections.CheckedChanged += async (s, e) =>
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
            };

            ctrlPanel.Controls.AddRange(new Control[] { btnPlay, btnStop, btnFull, btnVLC, btnSnap, lblTopic, _cmbTopic, btnRefresh, lblLat, _trkLatency, _lblLatencyValue, btnApplyLatency, _chkDetections });
            
            Controls.Add(_videoBox);
            Controls.Add(_lblStatus);
            Controls.Add(ctrlPanel);
            
            // Initialize with default topic
            _topics.Add(("/zed/zed_node/rgb/color/rect/image", "Left Rect"));
            PopulateTopics();
        }

        private async System.Threading.Tasks.Task SyncOverlayStatusAsync()
        {
            try
            {
                var overlayJson = await JetsonApiService.ApiClient.GetStringAsync($"{_apiBaseUrl}/api/video/overlay/status");
                var overlayData = JObject.Parse(overlayJson);
                var overlayEnabled = overlayData["enabled"]?.Value<bool>() ?? false;

                _syncingOverlayState = true;
                _chkDetections.Checked = overlayEnabled;
                _syncingOverlayState = false;
                _overlayEnabled = overlayEnabled;
            }
            catch
            {
                // Keep local default if status probe fails.
                _syncingOverlayState = false;
            }
        }

        private async System.Threading.Tasks.Task SetHsvModeAsync(bool enabled)
        {
            // Use LongRunClient (60s) — the video bridge relays the toggle through a
            // secondary HTTP call, so the chained round-trip can exceed the 5s ApiClient timeout.
            if (enabled)
            {
                var overlayResp = await JetsonApiService.LongRunClient.PostAsync(
                    $"{_apiBaseUrl}/api/video/overlay/enable", null);
                await EnsureRequestSucceededAsync(overlayResp, "overlay enable", requireSuccessField: true);
                return;
            }

            var overlayDisableResp = await JetsonApiService.LongRunClient.PostAsync(
                $"{_apiBaseUrl}/api/video/overlay/disable", null);
            await EnsureRequestSucceededAsync(overlayDisableResp, "overlay disable", requireSuccessField: true);
        }

        private async System.Threading.Tasks.Task EnsureRequestSucceededAsync(
            HttpResponseMessage response,
            string operation,
            bool requireSuccessField = false)
        {
            if (!response.IsSuccessStatusCode)
            {
                throw new InvalidOperationException(
                    $"{operation} returned {response.StatusCode}");
            }

            var payload = await response.Content.ReadAsStringAsync();
            if (string.IsNullOrWhiteSpace(payload))
            {
                if (requireSuccessField)
                    throw new InvalidOperationException($"{operation} returned empty response");
                return;
            }

            try
            {
                var json = JObject.Parse(payload);
                var successToken = json["success"];
                if (successToken != null && !successToken.Value<bool>())
                {
                    var detail = json["error"]?.ToString()
                        ?? json["message"]?.ToString()
                        ?? payload;
                    throw new InvalidOperationException($"{operation} failed: {detail}");
                }

                if (requireSuccessField && successToken == null)
                    throw new InvalidOperationException($"{operation} returned no success field");
            }
            catch (Newtonsoft.Json.JsonException) when (!requireSuccessField)
            {
                // Some endpoints return plain text; HTTP status already validated.
            }
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
        
        private async System.Threading.Tasks.Task RefreshTopicsAsync(bool autoSelectRgb = false)
        {
            try
            {
                _lblStatus.Text = "Refreshing topics...";
                _lblStatus.ForeColor = Color.Yellow;
                
                var json = await JetsonApiService.LongRunClient.GetStringAsync($"{_apiBaseUrl}/api/video/topics");
                var data = JObject.Parse(json);
                var arr = data["topics"] as JArray;
                
                _topics.Clear();
                if (arr != null)
                {
                    // Allow the four useful RGB combinations from the ZED:
                    // {left, right} × {raw, rect}. The ZED publishes them under
                    // /zed/zed_node/{rgb,left,right,rgb_raw,left_raw,right_raw}/...
                    // — "rgb" is an alias for "left", which we keep as the
                    // default "Left" label.
                    foreach (var t in arr)
                    {
                        var name = t["name"]?.ToString();
                        if (string.IsNullOrEmpty(name)) continue;

                        var lower = name.ToLowerInvariant();
                        if (lower.IndexOf("gray") >= 0 || lower.IndexOf("depth") >= 0)
                            continue;

                        bool isLeft  = lower.IndexOf("/left")  >= 0
                                     || lower.IndexOf("/rgb")  >= 0;  // ZED rgb == left
                        bool isRight = lower.IndexOf("/right") >= 0;
                        if (!isLeft && !isRight) continue;

                        // ZED naming: "_raw" suffix marks raw; "rect" in path marks rectified.
                        bool isRaw  = lower.IndexOf("_raw") >= 0
                                    || lower.IndexOf("/raw") >= 0
                                    || lower.IndexOf("image_raw") >= 0;
                        bool isRect = lower.IndexOf("rect") >= 0;
                        if (!isRaw && !isRect) continue;

                        string side = isRight ? "Right" : "Left";
                        string kind = isRect ? "Rect" : "Raw";
                        _topics.Add((name, $"{side} {kind}"));
                    }

                    // Stable display order: Left Rect, Left Raw, Right Rect, Right Raw.
                    var order = new System.Collections.Generic.Dictionary<string, int>
                    {
                        { "Left Rect", 0 }, { "Left Raw", 1 },
                        { "Right Rect", 2 }, { "Right Raw", 3 },
                    };
                    _topics.Sort((a, b) =>
                    {
                        int ai, bi;
                        if (!order.TryGetValue(a.Display, out ai)) ai = 99;
                        if (!order.TryGetValue(b.Display, out bi)) bi = 99;
                        return ai.CompareTo(bi);
                    });

                    // De-duplicate display labels: if the server reports both
                    // /zed/zed_node/rgb/... and /zed/zed_node/left/... they
                    // collapse to the same "Left Rect" label — keep the first.
                    var seen = new System.Collections.Generic.HashSet<string>();
                    _topics.RemoveAll(t => !seen.Add(t.Display));
                }

                if (_topics.Count == 0)
                    _topics.Add(("/zed/zed_node/rgb/color/rect/image", "Left Rect"));
                
                PopulateTopics();
                _lblStatus.Text = $"Found {_topics.Count} topics";
                _lblStatus.ForeColor = Color.LimeGreen;
                
                // Auto-select RGB color topic if requested (prefer SDK 5.2 path)
                if (autoSelectRgb && _topics.Count > 0)
                {
                    int rgbIndex = _topics.FindIndex(t => 
                        t.Name.IndexOf("rgb/color/rect/image", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        t.Name.IndexOf("rgb_color_rect_image", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        t.Name.IndexOf("rgb/image_rect_color", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        t.Name.IndexOf("rgb_image_rect_color", StringComparison.OrdinalIgnoreCase) >= 0);
                    
                    if (rgbIndex >= 0)
                    {
                        _suppressTopicChange = true;
                        _cmbTopic.SelectedIndex = rgbIndex;
                        _suppressTopicChange = false;
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
            if (_suppressTopicChange) return;
            if (_cmbTopic.SelectedIndex < 0 || _cmbTopic.SelectedIndex >= _topics.Count) return;
            
            var (name, display) = _topics[_cmbTopic.SelectedIndex];
            _lblStatus.Text = $"Switching to {display}...";
            _lblStatus.ForeColor = Color.Yellow;
            
            try
            {
                var wasPlaying = _isPlaying;
                
                // Stop client stream FIRST to prevent GStreamer from crashing
                // when the server restarts its RTSP pipeline during the topic switch.
                // Without this, GStreamer's ThreadStart tries to read caps from a
                // dying RTSP connection and hits AccessViolationException.
                if (wasPlaying)
                    StopStream();
                
                var resp = await JetsonApiService.ApiClient.PostAsync($"{_apiBaseUrl}/api/video/source?topic={Uri.EscapeDataString(name)}", null);
                if (!resp.IsSuccessStatusCode)
                {
                    _lblStatus.Text = $"Switch failed: {resp.StatusCode}";
                    _lblStatus.ForeColor = Color.Red;
                    return;
                }
                
                // Wait for server pipeline to fully restart before reconnecting
                await System.Threading.Tasks.Task.Delay(500);
                if (wasPlaying && !IsDisposed)
                {
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
            // Use explicit H264 depay/parse/decode and force RTSP over TCP.
            // This avoids UDP packet-loss corruption loops where avdec_h264 repeatedly
            // reinitializes and shows heavy artifacts.
            // Requirements:
            // 1. appsink MUST be named "outsink" (Mission Planner looks for this)
            // 2. format=BGRA (32-bit matches Windows Forms bitmap creation)
            // 3. Keep a deterministic decode path for stability on lossy links
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
            
                 // RTSP pipeline - force TCP transport to eliminate UDP loss artifacts.
                 return $"rtspsrc location={_streamUrl} protocols=tcp latency={_latencyMs} do-retransmission=false ! " +
                     $"application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse disable-passthrough=true ! avdec_h264 ! " +
                     $"queue max-size-buffers={queueBuffers} {leaky} ! " +
                   $"videoconvert ! video/x-raw,format=BGRA ! " +
                   $"appsink name=outsink sync={syncVal}";
        }
        
        /// <summary>
        /// Serialized stop-then-start that prevents overlapping native GStreamer teardown/startup.
        /// All restart paths (latency change, topic switch) must use this method.
        /// </summary>
        private async System.Threading.Tasks.Task RestartStreamAsync()
        {
            await _lifecycleLock.WaitAsync();
            try
            {
                StopStream();
                // Wait for GStreamer native thread to fully exit before restarting
                await System.Threading.Tasks.Task.Delay(500);
                if (!IsDisposed)
                    StartStream();
            }
            finally
            {
                _lifecycleLock.Release();
            }
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
            
            GStreamer gst = null;
            try
            {
                _stopping = false;
                _streamGeneration++;
                
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Starting stream to {_streamUrl}");
                gst = new GStreamer();
                gst.OnNewImage += OnGstNewImage;
                
                var pipeline = BuildGStreamerPipeline();
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Pipeline: {pipeline}");
                
                gst.Start(pipeline);
                _gst = gst;
                _isPlaying = true;
                gst = null; // Ownership transferred to _gst
                _lblStatus.Text = "Connecting...";
                _lblStatus.ForeColor = Color.Yellow;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Start error - {ex}");
                _lblStatus.Text = $"Error: {ex.Message}";
                _lblStatus.ForeColor = Color.Red;
            }
            finally
            {
                // Clean up partially-started instance on failure
                if (gst != null)
                {
                    try { gst.OnNewImage -= OnGstNewImage; } catch { }
                    try { gst.Stop(); } catch { }
                    try { (gst as IDisposable)?.Dispose(); } catch { }
                }
            }
        }
        
        public void StopStream()
        {
            if (!_isPlaying && _gst == null) return;
            
            _stopping = true;
            _streamGeneration++; // Invalidate all pending frame callbacks
            
            // Capture and clear reference to prevent double-stop on the same instance
            var gst = _gst;
            _gst = null;
            _isPlaying = false;
            
            try
            {
                if (gst != null)
                {
                    // Unhook event first to prevent new callbacks during shutdown
                    try { gst.OnNewImage -= OnGstNewImage; } catch { }
                    
                    // Stop GStreamer pipeline
                    try { gst.Stop(); } catch { }
                    
                    // Give GStreamer native thread time to fully exit
                    System.Threading.Thread.Sleep(300);
                    
                    // Dispose if supported to release native resources deterministically
                    try { (gst as IDisposable)?.Dispose(); } catch { }
                }
            }
            catch { }

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
        /// GStreamer frame callback - invoked on GStreamer's background thread.
        /// Copies pixel data into a managed Bitmap on THIS thread (while native buffer is valid),
        /// then marshals only the managed copy to UI thread via BeginInvoke.
        /// This prevents use-after-free when the native frame buffer is recycled or freed.
        /// </summary>
        private void OnGstNewImage(object sender, MPBitmap frame)
        {
            if (frame == null || frame.Width <= 0 || frame.Height <= 0) return;
            if (_stopping) return;
            
            var generation = _streamGeneration;
            _frameCount++;
            if (_frameCount % 30 == 1)
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Frame #{_frameCount} - {frame.Width}x{frame.Height}");
            
            // Copy pixel data on the GStreamer callback thread while the native buffer is valid.
            // We MUST NOT defer this to BeginInvoke because GStreamer may free the buffer
            // after this callback returns, causing AccessViolationException.
            Bitmap displayBitmap;
            try
            {
                var lockData = frame.LockBits(Rectangle.Empty, null, SkiaSharp.SKColorType.Bgra8888);
                displayBitmap = new Bitmap(frame.Width, frame.Height, PixelFormat.Format32bppPArgb);
                var bmpData = displayBitmap.LockBits(
                    new Rectangle(0, 0, frame.Width, frame.Height),
                    System.Drawing.Imaging.ImageLockMode.WriteOnly,
                    PixelFormat.Format32bppPArgb);
                try
                {
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
            }
            catch
            {
                return; // Frame became invalid during copy - pipeline likely tearing down
            }
            
            var width = frame.Width;
            var height = frame.Height;
            
            // Marshal only the fully-managed bitmap to the UI thread
            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => UpdateVideoDisplay(displayBitmap, width, height, generation)));
                return;
            }
            
            UpdateVideoDisplay(displayBitmap, width, height, generation);
        }
        
        /// <summary>
        /// Draws a center crosshair over the live video so the pilot can aim when
        /// no HSV circles are detected. Only shown while a frame is live.
        /// </summary>
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
        }

        /// <summary>
        /// Updates PictureBox and fullscreen with a managed bitmap. Called on UI thread only.
        /// Rejects stale frames from previous stream sessions via generation check.
        /// </summary>
        private void UpdateVideoDisplay(Bitmap displayBitmap, int width, int height, int generation)
        {
            if (IsDisposed || _videoBox == null || generation != _streamGeneration || _stopping)
            {
                displayBitmap.Dispose();
                return;
            }
            
            try
            {
                var oldImage = _videoBox.Image;
                _videoBox.Image = displayBitmap;
                oldImage?.Dispose();
                
                if (_fullscreenBox != null && !_fullscreenBox.IsDisposed)
                {
                    var oldFull = _fullscreenBox.Image;
                    _fullscreenBox.Image = (Bitmap)displayBitmap.Clone();
                    oldFull?.Dispose();
                }
                
                // Store managed copy for snapshots (not native frame)
                var oldLastFrame = _lastFrame;
                _lastFrame = (Bitmap)displayBitmap.Clone();
                oldLastFrame?.Dispose();
                
                if (_frameCount % 30 == 1)
                {
                    _lblStatus.Text = $"Streaming {width}x{height}";
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
                // Disable server-side overlay on dispose
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
                StopStream();
                _lastFrame?.Dispose();
                _lifecycleLock.Dispose();
                if (_fullscreenForm != null && !_fullscreenForm.IsDisposed)
                    _fullscreenForm.Close();
            }
            base.Dispose(disposing);
        }
    }
}
