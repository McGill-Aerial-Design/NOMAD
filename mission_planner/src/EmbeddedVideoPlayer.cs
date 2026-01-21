extern alias MPDrawing;

// ============================================================
// NOMAD Embedded Video Player Control
// ============================================================
// Provides in-app RTSP video streaming for Mission Planner
// using the built-in GStreamer pipeline (same core as OSD video).
// ============================================================

using System;
using System.Drawing;
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
        private string _streamUrl;
        private bool _isPlaying;
        private bool _useGStreamer;
        
        // UI Controls
        private Panel _videoPanel;
        private Panel _controlPanel;
        private Label _lblTitle;
        private Label _lblStatus;
        private Button _btnFullscreen;
        private Button _btnExternal;
        private Button _btnSnapshot;
        private ComboBox _cmbQuality;
        private TrackBar _trkLatency;
        private Label _lblLatency;
        
        // GStreamer (Mission Planner built-in)
        private GStreamer _gst;
        private PictureBox _videoBox;
        private Form _fullscreenForm;
        private PictureBox _fullscreenBox;
        private MPBitmap _lastFrame;
        private string _embeddedInitErrorMessage = null;
        
        // Playback settings
        private int _networkCaching = 100; // ms
        private string _quality = "auto";
        
        // ============================================================
        // Constructor
        // ============================================================
        
        public EmbeddedVideoPlayer(string title, string rtspUrl)
        {
            _streamTitle = title;
            _streamUrl = rtspUrl;
            _isPlaying = false;
            _useGStreamer = TryInitializeGStreamer();
            
            InitializeUI();
            
            // Auto-start streaming when control is loaded
            this.Load += (s, e) => {
                if (_useGStreamer)
                {
                    StartStream();
                }
            };
        }
        
        // ============================================================
        // LibVLC Initialization
        // ============================================================
        
        private bool TryInitializeGStreamer()
        {
            try
            {
                _gst = new GStreamer();
                var gstPath = GStreamer.LookForGstreamer();
                if (string.IsNullOrWhiteSpace(gstPath) || !GStreamer.GstLaunchExists)
                {
                    _embeddedInitErrorMessage = "GStreamer runtime not found. Install Mission Planner GStreamer support or run its GStreamer downloader.";
                    System.Diagnostics.Debug.WriteLine("NOMAD: GStreamer not available");
                    return false;
                }

                _gst.OnNewImage += OnGstNewImage;
                System.Diagnostics.Debug.WriteLine("NOMAD: GStreamer initialized successfully");
                return true;
            }
            catch (Exception ex)
            {
                _embeddedInitErrorMessage = ex.Message;
                System.Diagnostics.Debug.WriteLine($"NOMAD: GStreamer initialization failed - {ex}");
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
            this.Padding = new Padding(5);
            
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
                    Text = "[VIDEO] Video Feed\n\nClick Play to start stream\n\n" +
                           "(Embedded GStreamer unavailable)",
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
            
            // Snapshot Button
            _btnSnapshot = new Button
            {
                Text = "[S] Snapshot",
                Location = new Point(10, 10),
                Size = new Size(90, 30),
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
                Text = "[EXT] VLC",
                Location = new Point(105, 10),
                Size = new Size(75, 30),
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
            
            // Quality Selector
            var lblQuality = new Label
            {
                Text = "Quality:",
                Location = new Point(270, 48),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                AutoSize = true,
            };
            panel.Controls.Add(lblQuality);
            
            _cmbQuality = new ComboBox
            {
                Location = new Point(315, 45),
                Size = new Size(80, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
            _cmbQuality.Items.AddRange(new object[] { "Auto", "720p", "480p", "360p" });
            _cmbQuality.SelectedIndex = 0;
            _cmbQuality.SelectedIndexChanged += (s, e) => _quality = _cmbQuality.SelectedItem.ToString().ToLower();
            panel.Controls.Add(_cmbQuality);
            
            return panel;
        }
        
        // ============================================================
        // Playback Methods
        // ============================================================
        
        public void StartStream()
        {
            if (_isPlaying) return;
            
            try
            {
                if (_useGStreamer && _gst != null)
                {
                    var pipeline = BuildGStreamerPipeline();
                    System.Diagnostics.Debug.WriteLine($"NOMAD Video: Starting pipeline: {pipeline}");
                    _gst.Start(pipeline);

                    _isPlaying = true;
                    UpdateStatus("Playing (GStreamer)", Color.LimeGreen);
                }
                else
                {
                    var errorMsg = _embeddedInitErrorMessage ?? "GStreamer not available";
                    UpdateStatus($"Error: {errorMsg}", Color.Red);
                    System.Diagnostics.Debug.WriteLine($"NOMAD Video: Cannot start - {errorMsg}");
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Play error - {ex.Message}");
                UpdateStatus($"Error: {ex.Message}", Color.Red);
            }
        }
        
        public void StopStream()
        {
            try
            {
                if (_useGStreamer && _gst != null)
                {
                    // Unhook the event handler to prevent more callbacks
                    try { _gst.OnNewImage -= OnGstNewImage; } catch { }
                    
                    // Stop the pipeline
                    _gst.Stop();
                    
                    // Give GStreamer a moment to clean up
                    System.Threading.Thread.Sleep(100);
                }
                
                _isPlaying = false;
                UpdateStatus("Stopped", Color.Gray);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Stop error - {ex.Message}");
            }
        }
        
        public void OpenExternal()
        {
            // Build VLC/FFplay arguments based on stream type
            string vlcArgs;
            string ffplayArgs;
            
            if (_streamUrl.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
            {
                // UDP stream - VLC needs special format for UDP listen
                // VLC expects: udp://@:5600 (the @ means "listen on all interfaces")
                var port = ExtractUdpPort(_streamUrl);
                var vlcUrl = $"udp://@:{port}";
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Opening VLC with UDP URL: {vlcUrl}");
                vlcArgs = $"--network-caching={_networkCaching} {vlcUrl}";
                ffplayArgs = $"-fflags nobuffer -flags low_delay -i udp://0.0.0.0:{port}";
            }
            else
            {
                // RTSP stream - use TCP transport
                vlcArgs = $"--network-caching={_networkCaching} --rtsp-tcp \"{_streamUrl}\"";
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
                    // First stop the stream and unhook events
                    StopStream();
                    
                    // Force GStreamer cleanup on a background thread with timeout
                    if (_gst != null)
                    {
                        var cleanupTask = System.Threading.Tasks.Task.Run(() =>
                        {
                            try
                            {
                                _gst.Stop();
                            }
                            catch { }
                        });
                        
                        // Wait max 500ms for cleanup
                        cleanupTask.Wait(500);
                        _gst = null;
                    }

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
            var latency = Math.Max(50, _networkCaching);
            
            // Check if the URL is UDP or RTSP
            if (_streamUrl.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
            {
                // UDP RTP stream (e.g., udp://@:5600)
                // Extract port from URL like "udp://@:5600" or "udp://0.0.0.0:5600"
                var port = ExtractUdpPort(_streamUrl);
                
                // Simple UDP pipeline - let GStreamer auto-detect the stream format
                // This is more compatible with Mission Planner's GStreamer wrapper
                return $"udpsrc port={port} ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! " +
                       $"rtpjitterbuffer latency={latency} ! rtph264depay ! avdec_h264 ! " +
                       "videoconvert ! video/x-raw,format=BGRx ! appsink name=appsink";
            }
            else
            {
                // RTSP stream (default) - use same format as Mission Planner OSD
                return $"rtspsrc location={_streamUrl} latency={latency} ! " +
                       "rtph264depay ! avdec_h264 ! videoconvert ! " +
                       "video/x-raw,format=BGRx ! appsink name=appsink";
            }
        }

        private void OnGstNewImage(object sender, MPBitmap frame)
        {
            if (frame == null)
            {
                return;
            }

            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => OnGstNewImage(sender, frame)));
                return;
            }

            try
            {
                // Clone frames for display and snapshot
                // Use dynamic to handle the type mismatch between compile-time and runtime assemblies
                dynamic displayFrame = frame.Clone();
                var snapshotFrame = (MPBitmap)frame.Clone();

                // Update video display
                var old = _videoBox?.Image;
                if (_videoBox != null)
                {
                    _videoBox.Image = displayFrame;
                }
                (old as IDisposable)?.Dispose();

                // Update fullscreen display
                if (_fullscreenBox != null)
                {
                    var oldFull = _fullscreenBox.Image;
                    dynamic fullFrame = frame.Clone();
                    _fullscreenBox.Image = fullFrame;
                    (oldFull as IDisposable)?.Dispose();
                }

                // Keep last frame for snapshot
                if (_lastFrame != null)
                {
                    _lastFrame.Dispose();
                }
                _lastFrame = snapshotFrame;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Video: Frame update error - {ex.Message}");
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
