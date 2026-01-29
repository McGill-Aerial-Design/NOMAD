extern alias MPDrawing;

// ============================================================
// NOMAD Embedded Video Player Control
// ============================================================
// Provides in-app RTSP video streaming for Mission Planner
// using the built-in GStreamer pipeline (same core as OSD video).
//
// NOTE: This class uses .NET Reflection to access internal Mission Planner fields
// (specifically the gstreamer pipeline and native bitmap conversion). If Mission Planner
// updates break these reflection points, the embedded player will show a graceful error
// message and fallback to external VLC/FFplay options instead of crashing. Video display
// will be disabled but the plugin will remain functional. See error logs for details.
// ============================================================

using System;
using System.Drawing;
using System.IO;
using System.Threading.Tasks;
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
        
        // SAFETY: Synchronization for thread-safe stream switching
        private readonly object _streamLock = new object();
        private volatile bool _isStreamSwitching = false;
        private volatile bool _disposed = false;
        
        // Connection safety
        private JetsonConnectionManager _jetsonConnectionManager;
        private bool _jetsonConnected = true;  // Default true if no manager provided
        
        // Playback settings - ultra-low latency defaults
        private int _networkCaching = 50; // ms - minimum for stable playback
        private string _quality = "auto";
        private string _selectedStream = "zed";  // Current selected stream name
        private bool _showControls = true;    // Whether to show control bar and title
        
        // Stream configuration - Zero-Copy Architecture
        // RTSP URL stays constant, content is switched via Jetson API
        private string _baseRtspUrl;  // Base URL without stream path (e.g., rtsp://ip:8554)
        private string _apiBaseUrl;   // Edge Core API URL (e.g., http://ip:8000)
        private System.Collections.Generic.List<(string Name, string DisplayName)> _availableStreams;
        private Button _btnRefreshStreams;
        private ComboBox _cmbStreamSelect;  // Stream selector (replaces camera view)
        private CheckBox _chkOverlay;       // Overlay toggle
        
        // ============================================================
        // Constructor
        // ============================================================
        
        /// <summary>
        /// Creates an embedded video player.
        /// </summary>
        /// <param name="title">Title to display</param>
        /// <param name="rtspUrl">RTSP stream URL (e.g., rtsp://ip:8554/zed)</param>
        /// <param name="showControls">Whether to show control bar and title (false for minimal view)</param>
        /// <param name="jetsonConnectionManager">Connection manager for safety-aware behavior</param>
        public EmbeddedVideoPlayer(string title, string rtspUrl, bool showControls = true, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _streamTitle = title;
            _streamUrl = rtspUrl;  // Full stream URL (e.g., rtsp://ip:8554/zed)
            _isPlaying = false;
            _showControls = showControls;
            _useGStreamer = CheckGStreamerAvailable();
            _jetsonConnectionManager = jetsonConnectionManager;
            
            // Subscribe to connection state if manager provided
            if (_jetsonConnectionManager != null)
            {
                _jetsonConnected = _jetsonConnectionManager.IsConnected;
                _jetsonConnectionManager.ConnectionStateChanged += OnJetsonConnectionStateChanged;
            }
            
            // Parse base URL and stream name from rtspUrl
            ParseStreamUrl(rtspUrl);
            
            // Initialize with persistent streams (primary/secondary)
            // In zero-copy architecture, these are always available
            _availableStreams = new System.Collections.Generic.List<(string, string)>
            {
                ("primary", "[*] Primary Stream"),
                ("secondary", "[*] Secondary Stream"),
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
        /// In zero-copy architecture, stream URLs are constant (primary/secondary).
        /// </summary>
        private void ParseStreamUrl(string rtspUrl)
        {
            try
            {
                var uri = new Uri(rtspUrl);
                _baseRtspUrl = $"{uri.Scheme}://{uri.Host}:{uri.Port}";
                _apiBaseUrl = $"http://{uri.Host}:8000";  // Edge Core API
                _selectedStream = uri.AbsolutePath.TrimStart('/');
                
                // Map legacy stream names to new persistent instances
                if (string.IsNullOrEmpty(_selectedStream) || _selectedStream == "zed" || _selectedStream == "live" || _selectedStream == "dynamic")
                    _selectedStream = "primary";
                else if (_selectedStream == "depth" || _selectedStream == "gimbal")
                    _selectedStream = "secondary";
            }
            catch
            {
                _baseRtspUrl = "rtsp://100.75.218.89:8554";
                _apiBaseUrl = "http://100.75.218.89:8000";
                _selectedStream = "primary";
            }
            
            // Always use constant stream URL (primary or secondary)
            // Content is controlled via API, not by changing URL
            _streamUrl = $"{_baseRtspUrl}/{_selectedStream}";
        }
        
        /// <summary>
        /// Handles Jetson connection state changes.
        /// </summary>
        private void OnJetsonConnectionStateChanged(object sender, JetsonConnectionStateChangedEventArgs e)
        {
            _jetsonConnected = e.NewState == JetsonConnectionState.Connected;
            
            if (!_jetsonConnected)
            {
                // Stop stream when Jetson disconnects
                if (InvokeRequired)
                    BeginInvoke(new Action(() => {
                        StopStream();
                        UpdateStatus("Stream stopped - Jetson offline", Color.Orange);
                    }));
                else
                {
                    StopStream();
                    UpdateStatus("Stream stopped - Jetson offline", Color.Orange);
                }
            }
            else
            {
                // Update status when connected
                if (InvokeRequired)
                    BeginInvoke(new Action(() => UpdateStatus("Jetson connected - ready", Color.Gray)));
                else
                    UpdateStatus("Jetson connected - ready", Color.Gray);
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
            
            // Overlay Checkbox
            _chkOverlay = new CheckBox
            {
                Text = "Overlay",
                Location = new Point(500, 13),
                ForeColor = Color.LightGray,
                Font = new Font("Segoe UI", 9),
                AutoSize = true,
                Checked = true
            };
            _chkOverlay.CheckedChanged += (s, e) => ToggleOverlay(_chkOverlay.Checked);
            panel.Controls.Add(_chkOverlay);
            
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
        /// Toggles the object detection overlay on the Jetson.
        /// Uses the new API endpoint: POST /api/video/overlay?enabled={}&instance={}
        /// </summary>
        private async void ToggleOverlay(bool enabled)
        {
            try
            {
                var apiUrl = $"{_apiBaseUrl}/api/video/overlay?enabled={enabled}&instance={_selectedStream}";
                
                using (var client = new System.Net.Http.HttpClient())
                {
                    client.Timeout = TimeSpan.FromSeconds(2);
                    await client.PostAsync(apiUrl, null);
                    System.Diagnostics.Debug.WriteLine($"NOMAD Video: Overlay set to {enabled} for {_selectedStream}");
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Overlay toggle failed - {ex.Message}");
            }
        }

        /// <summary>
        /// Refreshes available streams from Edge Core API.
        /// Gets both persistent bridges (primary/secondary) and available ROS topics.
        /// </summary>
        private async void RefreshAvailableStreams()
        {
            try
            {
                UpdateStatus("Refreshing streams...", Color.Yellow);
                
                // Preserve current selection if possible
                var currentSelection = _selectedStream;
                _availableStreams.Clear();

                // Always add persistent streams first
                _availableStreams.Add(("primary", "[*] Primary Stream"));
                _availableStreams.Add(("secondary", "[*] Secondary Stream"));

                using (var client = new System.Net.Http.HttpClient())
                {
                    client.Timeout = TimeSpan.FromSeconds(5);

                    // Fetch bridge status to check if they're healthy
                    try 
                    {
                        var bridgesUrl = $"{_apiBaseUrl}/api/video/bridges";
                        var response = await client.GetStringAsync(bridgesUrl);
                        var data = Newtonsoft.Json.Linq.JObject.Parse(response);
                        var bridges = data["bridges"] as Newtonsoft.Json.Linq.JObject;
                        
                        if (bridges != null)
                        {
                            // Update primary/secondary display names with current topic
                            _availableStreams.Clear();
                            
                            var primary = bridges["primary"];
                            var primaryTopic = primary?["current_topic"]?.ToString() ?? "unknown";
                            var primaryShort = primaryTopic.Contains("/") ? primaryTopic.Substring(primaryTopic.LastIndexOf('/') + 1) : primaryTopic;
                            _availableStreams.Add(("primary", $"[*] Primary ({primaryShort})"));
                            
                            var secondary = bridges["secondary"];
                            var secondaryTopic = secondary?["current_topic"]?.ToString() ?? "unknown";
                            var secondaryShort = secondaryTopic.Contains("/") ? secondaryTopic.Substring(secondaryTopic.LastIndexOf('/') + 1) : secondaryTopic;
                            _availableStreams.Add(("secondary", $"[*] Secondary ({secondaryShort})"));
                        }
                    }
                    catch (Exception ex) 
                    { 
                        System.Diagnostics.Debug.WriteLine($"Bridge status fetch failed: {ex.Message}");
                    }

                    // Fetch ROS Topics (Dynamic switching options)
                    try
                    {
                        var topicsUrl = $"{_apiBaseUrl}/api/video/topics";
                        var response = await client.GetStringAsync(topicsUrl);
                        var data = Newtonsoft.Json.Linq.JObject.Parse(response);
                        var topics = data["topics"] as Newtonsoft.Json.Linq.JArray;
                        
                        if (topics != null)
                        {
                            foreach (var t in topics)
                            {
                                var topic = t.ToString();
                                // Create shorter display name by removing common prefix
                                var displayName = topic;
                                if (topic.StartsWith("/zed/zed_node/"))
                                    displayName = topic.Substring("/zed/zed_node/".Length);
                                else if (topic.StartsWith("/"))
                                    displayName = topic.Substring(1);
                                _availableStreams.Add(("ros:" + topic, $"[ROS] {displayName}"));
                            }
                        }
                    }
                    catch (Exception ex) 
                    { 
                        System.Diagnostics.Debug.WriteLine($"ROS topics fetch failed: {ex.Message}"); 
                    }
                }

                PopulateStreamSelector();
                UpdateStatus($"Found {_availableStreams.Count} sources", Color.LimeGreen);
            }
            catch (Exception ex)
            {
                UpdateStatus($"Refresh Error: {ex.Message}", Color.Red);
            }
        }
        
        /// <summary>
        /// Handles stream selection change.
        /// Uses API to switch topic - RTSP URL stays constant.
        /// This enables instant content switching without video player reconnection.
        /// </summary>
        private async void CmbStreamSelect_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (_cmbStreamSelect.SelectedIndex < 0 || _cmbStreamSelect.SelectedIndex >= _availableStreams.Count)
                return;
            
            var (name, displayName) = _availableStreams[_cmbStreamSelect.SelectedIndex];
            
            // Don't switch if same stream (for standard streams)
            if (!name.StartsWith("ros:") && name == _selectedStream)
                return;
            
            // Handle ROS Topic selection (Dynamic switching via API)
            if (name.StartsWith("ros:"))
            {
                var topic = name.Substring(4); // Remove "ros:"
                UpdateStatus($"Switching to {topic}...", Color.Yellow);
                
                try
                {
                    // Call new API endpoint to switch topic without restarting stream
                    // POST /api/video/switch {"instance": "primary", "topic": "<topic>"}
                    var apiUrl = $"{_apiBaseUrl}/api/video/switch";
                    var payload = new { instance = _selectedStream, topic = topic };
                    var jsonPayload = Newtonsoft.Json.JsonConvert.SerializeObject(payload);
                    
                    using (var client = new System.Net.Http.HttpClient())
                    {
                        client.Timeout = TimeSpan.FromSeconds(5);
                        var content = new System.Net.Http.StringContent(jsonPayload, System.Text.Encoding.UTF8, "application/json");
                        var response = await client.PostAsync(apiUrl, content);
                        
                        if (!response.IsSuccessStatusCode)
                        {
                            UpdateStatus($"Switch failed: {response.StatusCode}", Color.Red);
                            return;
                        }
                    }
                    
                    // RTSP URL stays the same - no need to restart player
                    // The video content changes on the Jetson side instantly
                    UpdateStatus($"Switched to: {topic}", Color.LimeGreen);
                    System.Diagnostics.Debug.WriteLine($"NOMAD Video: Topic switched via API to {topic}");
                }
                catch (Exception ex)
                {
                    UpdateStatus($"Switch Failed: {ex.Message}", Color.Red);
                    System.Diagnostics.Debug.WriteLine($"NOMAD Video: API switch failed - {ex.Message}");
                }
            }
            else
            {
                // Standard MediaMTX Stream (primary/secondary)
                // These are persistent streams - just ensure we're connected to the right one
                if (name == "primary" || name == "secondary")
                {
                    _selectedStream = name;
                    var newUrl = $"{_baseRtspUrl}/{_selectedStream}";
                    
                    if (newUrl != _streamUrl)
                    {
                        _streamUrl = newUrl;
                        UpdateStatus($"Connecting to {name}...", Color.Yellow);
                        RestartStream();
                    }
                }
            }
        }

        /// <summary>
        /// Restarts the video stream.
        /// In zero-copy architecture, this is only needed when switching between
        /// primary and secondary streams, not for topic changes.
        /// </summary>
        private void RestartStream()
        {
            // SAFETY: Prevent concurrent stream switches
            lock (_streamLock)
            {
                if (_isStreamSwitching) return;
                _isStreamSwitching = true;
            }
            
            try
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Restarting stream -> {_streamUrl}");
                
                if (_isPlaying)
                {
                    // SAFETY: Full stop with cleanup before starting new stream
                    StopStreamSafe();
                    
                    // SAFETY: Allow time for GStreamer to fully release resources
                    System.Threading.Thread.Sleep(300);
                    
                    // Force garbage collection
                    GC.Collect();
                    GC.WaitForPendingFinalizers();
                    
                    System.Threading.Thread.Sleep(100);
                    
                    StartStream();
                }
            }
            finally
            {
                lock (_streamLock)
                {
                    _isStreamSwitching = false;
                }
            }
        }
        
        // ============================================================
        // Playback Methods
        // ============================================================
        
        public void StartStream()
        {
            // SAFETY: Check for disposed state
            if (_disposed)
            {
                System.Diagnostics.Debug.WriteLine("NOMAD Video: Cannot start - control is disposed");
                return;
            }
            
            // SAFETY: Don't start stream if Jetson is offline
            if (!_jetsonConnected)
            {
                UpdateStatus("Stream unavailable - Jetson offline", Color.Orange);
                System.Diagnostics.Debug.WriteLine("NOMAD Video: Cannot start - Jetson offline");
                return;
            }
            
            if (_isPlaying) return;
            
            // SAFETY: Prevent starting during a stream switch
            lock (_streamLock)
            {
                if (_isStreamSwitching && _gst != null)
                {
                    System.Diagnostics.Debug.WriteLine("NOMAD Video: Cannot start during stream switch");
                    return;
                }
            }
            
            try
            {
                if (!_useGStreamer)
                {
                    var errorMsg = _embeddedInitErrorMessage ?? "GStreamer not available";
                    UpdateStatus($"Error: {errorMsg}", Color.Red);
                    System.Diagnostics.Debug.WriteLine($"NOMAD Video: Cannot start - {errorMsg}");
                    return;
                }

                // SAFETY: Ensure previous instance is fully cleaned up
                lock (_streamLock)
                {
                    if (_gst != null)
                    {
                        try { _gst.OnNewImage -= OnGstNewImage; } catch { }
                        try { _gst.Stop(); } catch { }
                        _gst = null;
                        
                        // Extra cleanup time
                        System.Threading.Thread.Sleep(100);
                    }

                    // SAFETY: Create new instance within lock to prevent races
                    _gst = new GStreamer();
                    _gst.OnNewImage += OnGstNewImage;
                }

                var pipeline = BuildGStreamerPipeline();
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Starting pipeline: {pipeline}");
                
                lock (_streamLock)
                {
                    if (_gst != null)
                    {
                        _gst.Start(pipeline);
                    }
                }

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
                
                // SAFETY: Cleanup on failure
                lock (_streamLock)
                {
                    if (_gst != null)
                    {
                        try { _gst.OnNewImage -= OnGstNewImage; } catch { }
                        try { _gst.Stop(); } catch { }
                        _gst = null;
                    }
                }
            }
        }
        
        public void StopStream()
        {
            StopStreamSafe();
        }
        
        /// <summary>
        /// Safely stops the stream with proper resource cleanup.
        /// 
        /// SAFETY CRITICAL: This method must be called before switching streams
        /// to prevent illegal memory access in GStreamer. The GStreamer instance
        /// must be fully stopped and dereferenced before creating a new one.
        /// </summary>
        private void StopStreamSafe()
        {
            try
            {
                _isPlaying = false;
                
                lock (_streamLock)
                {
                    if (_gst != null)
                    {
                        // SAFETY: Unhook event handler FIRST to prevent callbacks during cleanup
                        // This is critical - callbacks during shutdown cause memory violations
                        try 
                        { 
                            _gst.OnNewImage -= OnGstNewImage; 
                        } 
                        catch (Exception ex)
                        {
                            System.Diagnostics.Debug.WriteLine($"NOMAD Video: Error unhooking event - {ex.Message}");
                        }
                        
                        // SAFETY: Stop the pipeline with error handling
                        try 
                        { 
                            _gst.Stop(); 
                        } 
                        catch (Exception ex)
                        {
                            System.Diagnostics.Debug.WriteLine($"NOMAD Video: Error stopping GStreamer - {ex.Message}");
                        }
                        
                        // SAFETY: Give GStreamer time to clean up internal resources
                        // This is essential to prevent memory access violations
                        System.Threading.Thread.Sleep(150);
                        
                        // Clear the reference - we'll create a new instance on next Start
                        _gst = null;
                    }
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
        
        /// <summary>
        /// Disposes the video player and all resources.
        /// 
        /// SAFETY: Sets disposed flag FIRST to prevent any further operations,
        /// then cleans up in safe order (GStreamer first, then UI resources).
        /// </summary>
        protected override void Dispose(bool disposing)
        {
            // SAFETY: Set disposed flag immediately to stop all callbacks
            _disposed = true;
            
            if (disposing)
            {
                try
                {
                    // Stop the stream - this handles GStreamer cleanup
                    StopStreamSafe();
                    
                    // Extra wait for GStreamer cleanup
                    System.Threading.Thread.Sleep(200);
                    
                    // Dispose the last frame
                    lock (_streamLock)
                    {
                        if (_lastFrame != null)
                        {
                            _lastFrame.Dispose();
                            _lastFrame = null;
                        }
                    }
                    
                    // Close fullscreen form
                    if (_fullscreenForm != null && !_fullscreenForm.IsDisposed)
                    {
                        _fullscreenForm.Close();
                        _fullscreenForm = null;
                    }
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"NOMAD Video: Dispose error - {ex.Message}");
                }
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
        
        /// <summary>
        /// Handles new frames from GStreamer.
        /// 
        /// SAFETY CRITICAL: This callback can be invoked from GStreamer's thread
        /// at any time. We must check for disposal and stream switching states
        /// to prevent accessing released memory.
        /// </summary>
        private void OnGstNewImage(object sender, MPBitmap frame)
        {
            // SAFETY: Check if we're shutting down or switching streams
            if (_disposed || _isStreamSwitching)
            {
                return;
            }
            
            if (frame == null)
            {
                System.Diagnostics.Debug.WriteLine("NOMAD Video: Received null frame (stream may have ended)");
                if (!_disposed && !_isStreamSwitching)
                {
                    if (InvokeRequired)
                    {
                        try { BeginInvoke(new Action(() => UpdateStatus("Stream ended", Color.Gray))); } catch { }
                    }
                    else
                    {
                        UpdateStatus("Stream ended", Color.Gray);
                    }
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

            // SAFETY: Double-check before invoking on UI thread
            if (_disposed || _isStreamSwitching)
            {
                return;
            }

            if (InvokeRequired)
            {
                try
                {
                    BeginInvoke(new Action(() => OnGstNewImage(sender, frame)));
                }
                catch (ObjectDisposedException)
                {
                    // Control was disposed - ignore
                }
                catch (InvalidOperationException)
                {
                    // Handle not created - ignore
                }
                return;
            }

            try
            {
                // SAFETY: Final check before UI operations
                if (_disposed || _isStreamSwitching || _videoBox == null)
                {
                    return;
                }
                
                // Create a System.Drawing.Bitmap from the MissionPlanner.Drawing.Bitmap
                // MPBitmap internally wraps SKBitmap - access via reflection or direct property
                try
                {
                    Bitmap displayBitmap = null;
                    
                    // Try to get the internal SkiaSharp bitmap via reflection
                    var frameType = frame.GetType();
                    SkiaSharp.SKBitmap skBitmap = null;
                    
                    // Try property first
                    var skBitmapProp = frameType.GetProperty("nativeSkBitmap");
                    if (skBitmapProp != null)
                    {
                        skBitmap = skBitmapProp.GetValue(frame) as SkiaSharp.SKBitmap;
                    }
                    
                    // Try field if property not found
                    if (skBitmap == null)
                    {
                        var skBitmapField = frameType.GetField("nativeSkBitmap", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                        if (skBitmapField != null)
                        {
                            skBitmap = skBitmapField.GetValue(frame) as SkiaSharp.SKBitmap;
                        }
                    }
                    
                    if (skBitmap != null)
                    {
                        // Direct pixel access from internal SKBitmap
                        var pixmap = skBitmap.PeekPixels();
                        if (pixmap != null && pixmap.GetPixels() != IntPtr.Zero)
                        {
                            displayBitmap = new Bitmap(
                                skBitmap.Width,
                                skBitmap.Height,
                                skBitmap.RowBytes,
                                System.Drawing.Imaging.PixelFormat.Format32bppPArgb,
                                pixmap.GetPixels()
                            );
                        }
                    }
                    else
                    {
                        // Fallback: Use Clone which returns MPBitmap, then try ToSKImage
                        // This is slower but more compatible
                        using (var ms = new MemoryStream())
                        {
                            // Try saving via reflection on the clone
                            var cloned = frame.Clone();
                            var saveMethod = cloned.GetType().GetMethod("Save", new[] { typeof(Stream), typeof(SkiaSharp.SKEncodedImageFormat), typeof(int) });
                            if (saveMethod != null)
                            {
                                saveMethod.Invoke(cloned, new object[] { ms, SkiaSharp.SKEncodedImageFormat.Png, 80 });
                                ms.Position = 0;
                                displayBitmap = new Bitmap(ms);
                            }
                            (cloned as IDisposable)?.Dispose();
                        }
                    }
                    
                    if (displayBitmap == null)
                    {
                        System.Diagnostics.Debug.WriteLine("NOMAD Video: Failed to convert frame - no compatible method found");
                        return;
                    }

                    // Update video display
                    var old = _videoBox?.Image;
                    if (_videoBox != null && !_disposed)
                    {
                        _videoBox.Image = displayBitmap;
                    }
                    else
                    {
                        displayBitmap.Dispose();
                    }
                    old?.Dispose();

                    // Update fullscreen display
                    if (_fullscreenBox != null && !_disposed)
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
                }
                finally
                {
                    // Stream-based conversion handles cleanup automatically
                }
            }
            catch (AccessViolationException ex)
            {
                // SAFETY: Log but don't crash - this indicates a GStreamer timing issue
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: ACCESS VIOLATION - {ex.Message}");
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
