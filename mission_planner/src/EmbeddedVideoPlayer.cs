extern alias MPDrawing;

// ============================================================
// NOMAD Embedded Video Player Control
// ============================================================
// Provides in-app RTSP video streaming for Mission Planner
// using the built-in GStreamer pipeline (same core as OSD video).
// ============================================================

using System;
using System.Drawing;
using System.IO;
using System.Windows.Forms;
using MissionPlanner.Utilities;
using MPBitmap = MPDrawing::System.Drawing.Bitmap;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Embedded video player for RTSP streams within Mission Planner.
    /// Uses Mission Planner's GStreamer wrapper for native video playback.
    /// Falls back to external player only if GStreamer is not available.
    /// </summary>
    public class EmbeddedVideoPlayer : UserControl
    {
        // ============================================================
        // Fields
        // ============================================================
        
        private string _streamTitle;
        private string _streamUrl;       // RTSP URL for stereo stream
        private bool _isPlaying;
        private bool _useGStreamer;
        
        // UI Controls
        private Panel _videoPanel;
        private Panel _controlPanel;
        private Label _lblTitle;
        private Label _lblStatus;
        private Button _btnPlayStop;
        private Button _btnFullscreen;
        private Button _btnExternal;
        private Button _btnSnapshot;
        private ComboBox _cmbCameraView;  // Camera view selector
        private TrackBar _trkLatency;
        private Label _lblLatency;
        
        // GStreamer (Mission Planner built-in)
        private GStreamer _gst;
        private PictureBox _videoBox;
        private Form _fullscreenForm;
        private PictureBox _fullscreenBox;
        private MPBitmap _lastFrame;
        private string _embeddedInitErrorMessage = null;
        
        // Playback settings - ultra-low latency defaults
        private int _networkCaching = 50; // ms - minimum for stable playback
        private string _quality = "auto";
        private string _selectedStream = "zed";  // Current selected stream name
        private bool _showControls = true;    // Whether to show control bar and title
        
        // Stream configuration
        private string _baseRtspUrl;  // Base URL without stream path (e.g., rtsp://ip:8554)
        private System.Collections.Generic.List<(string Name, string DisplayName)> _availableStreams;
        private Button _btnRefreshStreams;
        private ComboBox _cmbStreamSelect;  // Stream selector (replaces camera view)
        
        // ============================================================
        // Constructor
        // ============================================================
        
        /// <summary>
        /// Creates an embedded video player.
        /// </summary>
        /// <param name="title">Title to display</param>
        /// <param name="rtspUrl">RTSP stream URL (e.g., rtsp://ip:8554/zed)</param>
        /// <param name="showControls">Whether to show control bar and title (false for minimal view)</param>
        public EmbeddedVideoPlayer(string title, string rtspUrl, bool showControls = true)
        {
            _streamTitle = title;
            _streamUrl = rtspUrl;  // Full stream URL (e.g., rtsp://ip:8554/zed)
            _isPlaying = false;
            _showControls = showControls;
            _useGStreamer = CheckGStreamerAvailable();
            
            // Parse base URL and stream name from rtspUrl
            ParseStreamUrl(rtspUrl);
            
            // Initialize with default streams (will be updated when refreshed)
            _availableStreams = new System.Collections.Generic.List<(string, string)>
            {
                ("zed", "ZED Camera (Main)"),
                ("live", "Live Stream"),
            };
            
            InitializeUI();
            
            // Auto-start streaming when control is loaded
            this.Load += (s, e) => {
                if (_useGStreamer)
                {
                    StartStream();
                }
            };
        }
        
        /// <summary>
        /// Parses the RTSP URL to extract base URL and stream name.
        /// </summary>
        private void ParseStreamUrl(string rtspUrl)
        {
            try
            {
                var uri = new Uri(rtspUrl);
                _baseRtspUrl = $"{uri.Scheme}://{uri.Host}:{uri.Port}";
                _selectedStream = uri.AbsolutePath.TrimStart('/');
                if (string.IsNullOrEmpty(_selectedStream))
                    _selectedStream = "zed";
            }
            catch
            {
                _baseRtspUrl = "rtsp://100.75.218.89:8554";
                _selectedStream = "zed";
            }
        }
        
        // ============================================================
        // GStreamer Availability Check
        // ============================================================
        
        private bool CheckGStreamerAvailable()
        {
            try
            {
                var gstPath = GStreamer.LookForGstreamer();
                if (string.IsNullOrWhiteSpace(gstPath) || !GStreamer.GstLaunchExists)
                {
                    _embeddedInitErrorMessage = "GStreamer runtime not found. Install Mission Planner GStreamer support or run its GStreamer downloader.";
                    System.Diagnostics.Debug.WriteLine("NOMAD: GStreamer not available");
                    return false;
                }

                System.Diagnostics.Debug.WriteLine("NOMAD: GStreamer is available");
                return true;
            }
            catch (Exception ex)
            {
                _embeddedInitErrorMessage = ex.Message;
                System.Diagnostics.Debug.WriteLine($"NOMAD: GStreamer check failed - {ex}");
            }
            
            return false;
        }
        
        // ============================================================
        // UI Initialization
        // ============================================================
        
        private void InitializeUI()
        {
            this.BackColor = Color.FromArgb(30, 30, 30);
            this.Dock = DockStyle.Fill;
            this.Padding = new Padding(_showControls ? 5 : 0);
            
            // If showControls is false, create minimal UI (just the video)
            if (!_showControls)
            {
                InitializeMinimalUI();
                return;
            }
            
            var mainPanel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
                BackColor = Color.Transparent,
            };
            
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 30));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 80));
            
            // Title Bar
            var titlePanel = CreateTitleBar();
            mainPanel.Controls.Add(titlePanel, 0, 0);
            
            // Video Display Area
            _videoPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                Margin = new Padding(0),
            };
            
            _videoBox = new PictureBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                SizeMode = PictureBoxSizeMode.Zoom,
            };

            if (_useGStreamer)
            {
                _videoPanel.Controls.Add(_videoBox);
            }
            else
            {
                var placeholder = new Label
                {
                    Text = "[VIDEO] ZED Camera Feed\n\nClick Play to start stream\n\n" +
                           "NOTE: Close OSD video first (right-click HUD > Video > Stop)",
                    Dock = DockStyle.Fill,
                    ForeColor = Color.Gray,
                    Font = new Font("Segoe UI", 11),
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                _videoPanel.Controls.Add(placeholder);

                if (!string.IsNullOrEmpty(_embeddedInitErrorMessage))
                {
                    var info = new Label
                    {
                        Text = "Embedded player unavailable: " + _embeddedInitErrorMessage,
                        Dock = DockStyle.Bottom,
                        ForeColor = Color.Orange,
                        Font = new Font("Segoe UI", 8),
                        Height = 32,
                        TextAlign = ContentAlignment.MiddleLeft,
                    };
                    mainPanel.Controls.Add(info, 0, 2);
                }
            }
            
            mainPanel.Controls.Add(_videoPanel, 0, 1);
            
            // Control Bar
            _controlPanel = CreateControlBar();
            mainPanel.Controls.Add(_controlPanel, 0, 2);
            
            this.Controls.Add(mainPanel);
        }
        
        /// <summary>
        /// Creates minimal UI with just the video display - no title, no controls.
        /// </summary>
        private void InitializeMinimalUI()
        {
            this.BackColor = Color.Black;
            this.Padding = new Padding(0);
            
            _videoPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                Margin = new Padding(0),
            };
            
            _videoBox = new PictureBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                SizeMode = PictureBoxSizeMode.Zoom,
            };

            if (_useGStreamer)
            {
                _videoPanel.Controls.Add(_videoBox);
            }
            else
            {
                var placeholder = new Label
                {
                    Text = "Video",
                    Dock = DockStyle.Fill,
                    ForeColor = Color.Gray,
                    Font = new Font("Segoe UI", 10),
                    TextAlign = ContentAlignment.MiddleCenter,
                    BackColor = Color.Black,
                };
                _videoPanel.Controls.Add(placeholder);
            }
            
            this.Controls.Add(_videoPanel);
        }
        
        private Panel CreateTitleBar()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
            };
            
            _lblTitle = new Label
            {
                Text = $"[VID] {_streamTitle}",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = Color.White,
                Location = new Point(10, 5),
                AutoSize = true,
            };
            panel.Controls.Add(_lblTitle);
            
            _lblStatus = new Label
            {
                Text = "[*] Stopped",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.Gray,
                Location = new Point(200, 7),
                AutoSize = true,
            };
            panel.Controls.Add(_lblStatus);
            
            return panel;
        }
        
        private Panel CreateControlBar()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(40, 40, 43),
                Padding = new Padding(10, 5, 10, 5),
            };
            
            // Play/Stop Button
            _btnPlayStop = new Button
            {
                Text = "Play",
                Location = new Point(10, 10),
                Size = new Size(60, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 120, 60),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            _btnPlayStop.Click += (s, e) => TogglePlayStop();
            panel.Controls.Add(_btnPlayStop);
            
            // Snapshot Button
            _btnSnapshot = new Button
            {
                Text = "Snap",
                Location = new Point(75, 10),
                Size = new Size(55, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(100, 100, 150),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
            };
            _btnSnapshot.Click += (s, e) => TakeSnapshot();
            panel.Controls.Add(_btnSnapshot);
            
            // External Button
            _btnExternal = new Button
            {
                Text = "VLC",
                Location = new Point(135, 10),
                Size = new Size(50, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(80, 80, 85),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
            };
            _btnExternal.Click += (s, e) => OpenExternal();
            panel.Controls.Add(_btnExternal);
            
            // Fullscreen Button
            _btnFullscreen = new Button
            {
                Text = "[+] Full",
                Location = new Point(185, 10),
                Size = new Size(65, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(80, 80, 85),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
            };
            _btnFullscreen.Click += (s, e) => ToggleFullscreen();
            panel.Controls.Add(_btnFullscreen);
            
            // Latency Slider
            var lblLatencyTitle = new Label
            {
                Text = "Latency:",
                Location = new Point(10, 48),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                AutoSize = true,
            };
            panel.Controls.Add(lblLatencyTitle);
            
            _trkLatency = new TrackBar
            {
                Location = new Point(60, 43),
                Size = new Size(150, 30),
                Minimum = 50,
                Maximum = 1000,
                Value = _networkCaching,
                TickFrequency = 100,
                SmallChange = 50,
                LargeChange = 100,
            };
            _trkLatency.ValueChanged += (s, e) =>
            {
                _networkCaching = _trkLatency.Value;
                _lblLatency.Text = $"{_networkCaching}ms";
            };
            panel.Controls.Add(_trkLatency);
            
            _lblLatency = new Label
            {
                Text = $"{_networkCaching}ms",
                Location = new Point(215, 48),
                ForeColor = Color.LightGray,
                Font = new Font("Segoe UI", 8),
                AutoSize = true,
            };
            panel.Controls.Add(_lblLatency);
            
            // Stream Selector (replaces fixed camera view selector)
            var lblStream = new Label
            {
                Text = "Stream:",
                Location = new Point(260, 15),
                ForeColor = Color.Cyan,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                AutoSize = true,
            };
            panel.Controls.Add(lblStream);
            
            _cmbStreamSelect = new ComboBox
            {
                Location = new Point(320, 10),
                Size = new Size(135, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
            };
            PopulateStreamSelector();
            _cmbStreamSelect.SelectedIndexChanged += CmbStreamSelect_SelectedIndexChanged;
            panel.Controls.Add(_cmbStreamSelect);
            
            // Refresh Streams Button
            _btnRefreshStreams = new Button
            {
                Text = "...",
                Location = new Point(460, 10),
                Size = new Size(30, 25),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
            };
            _btnRefreshStreams.Click += (s, e) => RefreshAvailableStreams();
            panel.Controls.Add(_btnRefreshStreams);
            
            // Quality is fixed at 720p - no selector needed
            _quality = "720p";
            
            return panel;
        }
        
        /// <summary>
        /// Populates the stream selector with available streams.
        /// </summary>
        private void PopulateStreamSelector()
        {
            _cmbStreamSelect.Items.Clear();
            foreach (var (name, displayName) in _availableStreams)
            {
                _cmbStreamSelect.Items.Add(displayName);
            }
            
            // Select current stream if available
            var currentIndex = _availableStreams.FindIndex(s => s.Name == _selectedStream);
            _cmbStreamSelect.SelectedIndex = currentIndex >= 0 ? currentIndex : 0;
        }
        
        /// <summary>
        /// Refreshes available streams from MediaMTX API.
        /// </summary>
        private async void RefreshAvailableStreams()
        {
            try
            {
                UpdateStatus("Refreshing streams...", Color.Yellow);
                
                // Query MediaMTX API for available paths
                var uri = new Uri(_baseRtspUrl);
                var apiUrl = $"http://{uri.Host}:9997/v3/paths/list";
                
                using (var client = new System.Net.Http.HttpClient())
                {
                    client.Timeout = TimeSpan.FromSeconds(5);
                    var response = await client.GetStringAsync(apiUrl);
                    
                    // Parse JSON response
                    var paths = Newtonsoft.Json.Linq.JObject.Parse(response);
                    var items = paths["items"] as Newtonsoft.Json.Linq.JArray;
                    
                    if (items != null && items.Count > 0)
                    {
                        _availableStreams.Clear();
                        foreach (var item in items)
                        {
                            var name = item["name"]?.ToString();
                            var ready = item["ready"]?.Value<bool>() ?? false;
                            var tracks = item["tracks"] as Newtonsoft.Json.Linq.JArray;
                            
                            if (!string.IsNullOrEmpty(name))
                            {
                                var status = ready ? "[LIVE]" : "[--]";
                                var trackInfo = tracks?.Count > 0 ? $" ({string.Join(",", tracks)})" : "";
                                _availableStreams.Add((name, $"{status} {name}{trackInfo}"));
                            }
                        }
                        
                        PopulateStreamSelector();
                        UpdateStatus($"Found {_availableStreams.Count} streams", Color.LimeGreen);
                    }
                    else
                    {
                        UpdateStatus("No streams found", Color.Orange);
                    }
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Failed to refresh streams - {ex.Message}");
                UpdateStatus("Could not fetch streams", Color.Red);
            }
        }
        
        /// <summary>
        /// Handles stream selection change.
        /// Restarts playback with the new stream.
        /// </summary>
        private void CmbStreamSelect_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (_cmbStreamSelect.SelectedIndex < 0 || _cmbStreamSelect.SelectedIndex >= _availableStreams.Count)
                return;
            
            var (name, displayName) = _availableStreams[_cmbStreamSelect.SelectedIndex];
            
            // Don't restart if same stream
            if (name == _selectedStream)
                return;
                
            _selectedStream = name;
            _streamUrl = $"{_baseRtspUrl}/{_selectedStream}";
            
            System.Diagnostics.Debug.WriteLine($"NOMAD Video: Stream changed to {displayName} -> {_streamUrl}");
            
            // Restart stream with new URL if currently playing
            if (_isPlaying)
            {
                StopStream();
                System.Threading.Thread.Sleep(100);
                StartStream();
            }
        }
        
        // ============================================================
        // Playback Methods
        // ============================================================
        
        public void StartStream()
        {
            if (_isPlaying) return;
            
            try
            {
                if (!_useGStreamer)
                {
                    var errorMsg = _embeddedInitErrorMessage ?? "GStreamer not available";
                    UpdateStatus($"Error: {errorMsg}", Color.Red);
                    System.Diagnostics.Debug.WriteLine($"NOMAD Video: Cannot start - {errorMsg}");
                    return;
                }

                // Create a fresh GStreamer instance for each session
                // This prevents "attempted to read or write protected memory" errors
                // that occur when reusing a stopped GStreamer instance
                if (_gst != null)
                {
                    try { _gst.OnNewImage -= OnGstNewImage; } catch { }
                    try { _gst.Stop(); } catch { }
                    _gst = null;
                }

                // Small delay to ensure cleanup
                System.Threading.Thread.Sleep(50);

                // Create new instance
                _gst = new GStreamer();
                _gst.OnNewImage += OnGstNewImage;

                var pipeline = BuildGStreamerPipeline();
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Starting pipeline: {pipeline}");
                _gst.Start(pipeline);

                _isPlaying = true;
                _frameCount = 0; // Reset frame counter
                
                // Update status to show current stream
                UpdateStatus($"Playing [{_selectedStream}]", Color.LimeGreen);
                UpdatePlayStopButton();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Play error - {ex.Message}\n{ex.StackTrace}");
                UpdateStatus($"Error: {ex.Message}", Color.Red);
            }
        }
        
        public void StopStream()
        {
            try
            {
                _isPlaying = false;
                
                if (_gst != null)
                {
                    // Unhook the event handler first to prevent callbacks during cleanup
                    try { _gst.OnNewImage -= OnGstNewImage; } catch { }
                    
                    // Stop the pipeline
                    try { _gst.Stop(); } catch { }
                    
                    // Give GStreamer a moment to clean up
                    System.Threading.Thread.Sleep(100);
                    
                    // Clear the reference - we'll create a new instance on next Start
                    _gst = null;
                }
                
                UpdateStatus("Stopped", Color.Gray);
                UpdatePlayStopButton();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Stop error - {ex.Message}");
            }
        }
        
        private void TogglePlayStop()
        {
            if (_isPlaying)
            {
                StopStream();
            }
            else
            {
                StartStream();
            }
        }
        
        private void UpdatePlayStopButton()
        {
            if (_btnPlayStop == null) return;
            
            if (_isPlaying)
            {
                _btnPlayStop.Text = "Stop";
                _btnPlayStop.BackColor = Color.FromArgb(150, 60, 60);
            }
            else
            {
                _btnPlayStop.Text = "Play";
                _btnPlayStop.BackColor = Color.FromArgb(60, 120, 60);
            }
        }
        
        public void OpenExternal()
        {
            // Build VLC/FFplay arguments based on stream type
            string vlcArgs;
            string ffplayArgs;
            
            if (_streamUrl.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
            {
                // UDP stream - use SDP file for best compatibility
                // VLC works better with SDP file than raw UDP URL
                var port = ExtractUdpPort(_streamUrl);
                
                // Create a temporary SDP file for VLC
                var sdpContent = $@"v=0
o=- 0 0 IN IP4 127.0.0.1
s=NOMAD ZED Camera
c=IN IP4 127.0.0.1
t=0 0
m=video {port} RTP/AVP 96
a=rtpmap:96 H264/90000
a=fmtp:96 profile-level-id=42c01f;packetization-mode=1
a=framerate:30
a=recvonly";
                
                var sdpPath = Path.Combine(Path.GetTempPath(), "nomad_stream.sdp");
                File.WriteAllText(sdpPath, sdpContent);
                
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Opening VLC with SDP file: {sdpPath}");
                vlcArgs = $"--network-caching={_networkCaching} --live-caching={_networkCaching} \"{sdpPath}\"";
                ffplayArgs = $"-fflags nobuffer -flags low_delay -protocol_whitelist file,udp,rtp -i \"{sdpPath}\"";
            }
            else
            {
                // RTSP stream - ultra-low latency settings
                // --avcodec-skiploopfilter=all: Skip deblocking for speed
                // --avcodec-skip-frame=0: Don't skip frames
                // --avcodec-hurry-up: Decode fast
                // --sout-mux-caching=0: Minimize mux buffering
                vlcArgs = $"--network-caching={_networkCaching} --live-caching={_networkCaching} " +
                          $"--clock-jitter=0 --avcodec-skiploopfilter=all --avcodec-hurry-up " +
                          $"--sout-mux-caching=0 --rtsp-tcp \"{_streamUrl}\"";
                ffplayArgs = $"-fflags nobuffer -flags low_delay -rtsp_transport tcp -i \"{_streamUrl}\"";
            }
            
            // Common VLC installation paths on Windows
            var vlcPaths = new[]
            {
                "vlc",  // PATH
                @"C:\Program Files\VideoLAN\VLC\vlc.exe",
                @"C:\Program Files (x86)\VideoLAN\VLC\vlc.exe",
                Environment.ExpandEnvironmentVariables(@"%LOCALAPPDATA%\Programs\VideoLAN\VLC\vlc.exe")
            };
            
            // Try VLC paths
            foreach (var vlcPath in vlcPaths)
            {
                try
                {
                    var psi = new System.Diagnostics.ProcessStartInfo
                    {
                        FileName = vlcPath,
                        Arguments = vlcArgs,
                        UseShellExecute = true,
                    };
                    System.Diagnostics.Process.Start(psi);
                    
                    UpdateStatus("Opened in VLC", Color.Yellow);
                    _isPlaying = true;
                    return;
                }
                catch
                {
                    // Try next path
                }
            }
            
            // Try FFplay (common with FFmpeg)
            var ffplayPaths = new[]
            {
                "ffplay",  // PATH
                @"C:\ffmpeg\bin\ffplay.exe",
                Environment.ExpandEnvironmentVariables(@"%LOCALAPPDATA%\Programs\ffmpeg\bin\ffplay.exe")
            };
            
            foreach (var ffplayPath in ffplayPaths)
            {
                try
                {
                    var psi = new System.Diagnostics.ProcessStartInfo
                    {
                        FileName = ffplayPath,
                        Arguments = ffplayArgs,
                        UseShellExecute = true,
                    };
                    System.Diagnostics.Process.Start(psi);
                    
                    UpdateStatus("Opened in FFplay", Color.Yellow);
                    _isPlaying = true;
                    return;
                }
                catch
                {
                    // Try next path
                }
            }
            
            // No player found - show error
            UpdateStatus("No player found", Color.Red);
            MessageBox.Show(
                $"Could not open video stream.\n\n" +
                $"Please install VLC or FFplay and ensure it's in your PATH.\n\n" +
                $"VLC download: https://www.videolan.org/vlc/\n" +
                $"FFmpeg download: https://ffmpeg.org/download.html\n\n" +
                $"Stream URL: {_streamUrl}",
                "Video Player Not Found",
                MessageBoxButtons.OK,
                MessageBoxIcon.Warning
            );
        }
        
        public void TakeSnapshot()
        {
            try
            {
                if (_lastFrame != null)
                {
                    var desktopPath = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
                    var filename = $"NOMAD_Snapshot_{DateTime.Now:yyyyMMdd_HHmmss}.png";
                    var path = System.IO.Path.Combine(desktopPath, filename);

                    _lastFrame.Save(path);
                    UpdateStatus($"[S] Saved: {filename}", Color.LimeGreen);
                }
                else
                {
                    UpdateStatus("[!] No frame available", Color.Yellow);
                }
            }
            catch (Exception ex)
            {
                UpdateStatus($"[X] Error: {ex.Message}", Color.Red);
            }
        }
        
        public void ToggleFullscreen()
        {
            try
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

                _fullscreenForm.KeyDown += (s, e) =>
                {
                    if (e.KeyCode == Keys.Escape)
                    {
                        _fullscreenForm.Close();
                        _fullscreenForm = null;
                        _fullscreenBox = null;
                    }
                };

                _fullscreenBox = new PictureBox
                {
                    Dock = DockStyle.Fill,
                    BackColor = Color.Black,
                    SizeMode = PictureBoxSizeMode.Zoom,
                };

                _fullscreenForm.Controls.Add(_fullscreenBox);
                _fullscreenForm.Show();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Fullscreen error - {ex.Message}");
            }
        }
        
        // ============================================================
        // Helper Methods
        // ============================================================
        
        private void UpdateStatus(string status, Color color)
        {
            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => UpdateStatus(status, color)));
                return;
            }
            
            _lblStatus.Text = status;
            _lblStatus.ForeColor = color;
        }
        
        public void UpdateStreamUrl(string newUrl)
        {
            var wasPlaying = _isPlaying;
            
            if (_isPlaying)
            {
                StopStream();
            }
            
            _streamUrl = newUrl;
            
            if (wasPlaying)
            {
                StartStream();
            }
        }
        
        // ============================================================
        // Cleanup
        // ============================================================
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                try
                {
                    // Stop the stream - this handles GStreamer cleanup
                    StopStream();
                    
                    // Dispose the last frame
                    if (_lastFrame != null)
                    {
                        _lastFrame.Dispose();
                        _lastFrame = null;
                    }
                    
                    // Close fullscreen form
                    if (_fullscreenForm != null && !_fullscreenForm.IsDisposed)
                    {
                        _fullscreenForm.Close();
                        _fullscreenForm = null;
                    }
                }
                catch { }
            }
            
            base.Dispose(disposing);
        }

        // ============================================================
        // GStreamer Helpers
        // ============================================================

        private string BuildGStreamerPipeline()
        {
            // Mission Planner's GStreamer class expects:
            // 1. appsink named "outsink" (not "appsink")
            // 2. format=BGRA (32-bit BGRA required by GStreamer.cs bitmap creation)
            // 3. sync=false at the end
            // 4. decodebin3 for automatic codec detection
            // 5. queue with leaky=2 for low latency
            
            // Check if the URL is UDP or RTSP
            if (_streamUrl.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
            {
                // UDP RTP stream (e.g., udp://@:5600)
                var port = ExtractUdpPort(_streamUrl);
                
                // Match Mission Planner's AutoConnect format for UDP H264
                // Uses decodebin3 for automatic codec detection
                return $"udpsrc port={port} buffer-size=90000 ! application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264 ! decodebin3 ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false";
            }
            else
            {
                // RTSP stream - Use Mission Planner's proven pipeline format
                // Key elements:
                // - rtspsrc: handles RTSP protocol
                // - latency: lower = less delay, but 41ms is stable minimum
                // - udp-reconnect=1: auto reconnect on UDP drops
                // - timeout=0: no timeout (important for network glitches)
                // - do-retransmission=false: don't request retransmission (reduces latency)
                // - application/x-rtp: explicit RTP media type
                // - decodebin3: automatic codec detection and decoding
                // - queue with leaky=2: drop old buffers if full (low latency)
                // - videoconvert to BGRA: required format for Mission Planner's bitmap
                // - appsink name=outsink sync=false: named sink without clock sync
                return $"rtspsrc location={_streamUrl} latency={_networkCaching} udp-reconnect=1 timeout=0 do-retransmission=false ! application/x-rtp ! decodebin3 ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false";
            }
        }

        private int _frameCount = 0;
        private DateTime _lastFrameTime = DateTime.MinValue;
        
        private void OnGstNewImage(object sender, MPBitmap frame)
        {
            if (frame == null)
            {
                System.Diagnostics.Debug.WriteLine("NOMAD Video: Received null frame (stream may have ended)");
                if (InvokeRequired)
                {
                    BeginInvoke(new Action(() => UpdateStatus("Stream ended", Color.Gray)));
                }
                else
                {
                    UpdateStatus("Stream ended", Color.Gray);
                }
                return;
            }

            if (frame.Width <= 0 || frame.Height <= 0)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Invalid frame dimensions: {frame.Width}x{frame.Height}");
                return;
            }

            _frameCount++;
            if (_frameCount % 30 == 1) // Log every 30 frames (once per second)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Frame #{_frameCount} received, Size: {frame.Width}x{frame.Height}, Stream: {_selectedStream}");
            }

            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => OnGstNewImage(sender, frame)));
                return;
            }

            try
            {
                // Create a System.Drawing.Bitmap from the SkiaSharp bitmap data
                var displayBitmap = new Bitmap(
                    frame.Width, 
                    frame.Height, 
                    4 * frame.Width,  // stride = 4 bytes per pixel (BGRA)
                    System.Drawing.Imaging.PixelFormat.Format32bppPArgb,
                    frame.LockBits(Rectangle.Empty, null, SkiaSharp.SKColorType.Bgra8888).Scan0
                );

                // Update video display
                var old = _videoBox?.Image;
                if (_videoBox != null)
                {
                    _videoBox.Image = displayBitmap;
                }
                old?.Dispose();

                // Update fullscreen display
                if (_fullscreenBox != null)
                {
                    var oldFull = _fullscreenBox.Image;
                    // Clone for fullscreen (displayBitmap is used by main view)
                    _fullscreenBox.Image = (Bitmap)displayBitmap.Clone();
                    oldFull?.Dispose();
                }

                // Keep last frame for snapshot (clone it since frame may be disposed)
                var oldLastFrame = _lastFrame;
                _lastFrame = (MPBitmap)frame.Clone();
                oldLastFrame?.Dispose();
                
                // Update status periodically
                if (_frameCount % 30 == 1)
                {
                    var viewName = _cameraView == "both" ? "Wide" : (_cameraView == "right" ? "Right" : "Left");
                    UpdateStatus($"Playing [{viewName}] ({cropRegion.Width}x{cropRegion.Height})", Color.LimeGreen);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Frame update error - {ex.Message}\n{ex.StackTrace}");
            }
        }
        
        /// <summary>
        /// Extract UDP port from URL (udp://@:5600, udp://5600, udp://@:5600, etc.)
        /// </summary>
        private int ExtractUdpPort(string url)
        {
            try
            {
                // Handle various formats:
                // udp://@:5600
                // udp://5600
                // udp://:5600
                var cleaned = url.Replace("udp://", "").Replace("@", "").TrimStart(':');
                if (int.TryParse(cleaned, out int port))
                {
                    return port;
                }
            }
            catch { }
            return 5600; // Default port
        }
    }
}
