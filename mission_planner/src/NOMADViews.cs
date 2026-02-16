// ============================================================
// NOMAD Sidebar Views
// ============================================================
// Individual view implementations for each sidebar section:
// Dashboard, Task 1, Task 2, Boundary, Video, Terminal, Health, Links
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    // ============================================================
    // Base View Class
    // ============================================================
    
    /// <summary>
    /// Base class for NOMAD views with common styling
    /// </summary>
    public abstract class NOMADViewBase : UserControl
    {
        // Colors delegated to NOMADTheme for consistency
        protected static readonly Color CARD_BG = NOMADTheme.CARD_BG;
        protected static readonly Color ACCENT_COLOR = NOMADTheme.ACCENT;
        protected static readonly Color SUCCESS_COLOR = NOMADTheme.SUCCESS;
        protected static readonly Color WARNING_COLOR = NOMADTheme.WARNING;
        protected static readonly Color ERROR_COLOR = NOMADTheme.ERROR;
        protected static readonly Color INFO_COLOR = NOMADTheme.INFO;
        protected static readonly Color TEXT_PRIMARY = NOMADTheme.TEXT_PRIMARY;
        protected static readonly Color TEXT_SECONDARY = NOMADTheme.TEXT_SECONDARY;

        protected NOMADViewBase()
        {
            this.BackColor = NOMADTheme.BG_DARK;
            this.Dock = DockStyle.Fill;
            this.Padding = new Padding(20);
            this.AutoScroll = true;
        }
        
        protected Panel CreateCard(string title, int width = -1, int height = -1)
        {
            var card = new Panel
            {
                BackColor = CARD_BG,
                Margin = new Padding(5),
                Padding = new Padding(15),
            };
            
            if (width > 0) card.Width = width;
            if (height > 0) card.Height = height;
            
            var titleLabel = new Label
            {
                Text = title.ToUpper(),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(15, 15),
                AutoSize = true,
            };
            card.Controls.Add(titleLabel);
            
            return card;
        }
        
        protected Button CreateButton(string text, Color bgColor, int width = 150, int height = 45)
        {
            var btn = new Button
            {
                Text = text,
                Size = new Size(width, height),
                Margin = new Padding(5),
                FlatStyle = FlatStyle.Flat,
                BackColor = bgColor,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            btn.FlatAppearance.BorderSize = 0;
            return btn;
        }
    }
    
    // ============================================================
    // Task 1 View - Outdoor Reconnaissance
    // ============================================================
    
    public class NOMADTask1View : NOMADViewBase, IUpdatableView
    {
        private readonly DualLinkSender _sender;
        private readonly NOMADConfig _config;
        private readonly JetsonConnectionManager _jetsonConnectionManager;
        private Label _lblPosition;
        private Label _lblGpsStatus;
        private Button _btnCapture;
        private TextBox _txtResult;
        private EmbeddedVideoPlayer _videoPlayer;
        private FlowLayoutPanel _galleryPanel;
        
        public NOMADTask1View(DualLinkSender sender, NOMADConfig config, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _sender = sender;
            _config = config;
            _jetsonConnectionManager = jetsonConnectionManager;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            // Two-column layout: Video (left) | Controls + Gallery (right)
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                Margin = Padding.Empty,
                Padding = Padding.Empty,
            };
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            // ========== LEFT COLUMN: Video (no title label - player fills entirely) ==========
            var videoPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                Margin = new Padding(5),
            };
            try
            {
                string rtspUrl = $"rtsp://{_config.EffectiveIP}:8554/primary";
                _videoPlayer = new EmbeddedVideoPlayer("Task 1 Camera", rtspUrl, true, _jetsonConnectionManager);
                _videoPlayer.Dock = DockStyle.Fill;
                videoPanel.Controls.Add(_videoPlayer);
            }
            catch (Exception ex)
            {
                var lblVideoError = new Label
                {
                    Text = $"Video unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 10),
                    ForeColor = ERROR_COLOR,
                    Location = new Point(15, 15),
                    AutoSize = true,
                };
                videoPanel.Controls.Add(lblVideoError);
            }
            mainLayout.Controls.Add(videoPanel, 0, 0);

            // ========== RIGHT COLUMN: GPS + Capture + Gallery ==========
            var rightLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
                Margin = Padding.Empty,
                Padding = Padding.Empty,
            };
            rightLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 90));   // GPS status
            rightLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 50));     // Capture
            rightLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 50));     // Gallery

            // --- GPS Status ---
            var gpsCard = CreateCard("GPS STATUS");
            gpsCard.Dock = DockStyle.Fill;

            _lblGpsStatus = new Label
            {
                Text = "Fix: Waiting...",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 40),
                AutoSize = true,
            };
            gpsCard.Controls.Add(_lblGpsStatus);

            _lblPosition = new Label
            {
                Text = "Position: --",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 62),
                AutoSize = true,
            };
            gpsCard.Controls.Add(_lblPosition);

            rightLayout.Controls.Add(gpsCard, 0, 0);

            // --- Capture Card ---
            var captureCard = CreateCard("SNAPSHOT CAPTURE");
            captureCard.Dock = DockStyle.Fill;

            _btnCapture = CreateButton("CAPTURE PHOTO", ACCENT_COLOR, 200, 40);
            _btnCapture.Location = new Point(15, 45);
            _btnCapture.Click += BtnCapture_Click;
            captureCard.Controls.Add(_btnCapture);

            _txtResult = new TextBox
            {
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Bottom,
                Location = new Point(15, 95),
                Size = new Size(280, 80),
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Vertical,
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = SUCCESS_COLOR,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.FixedSingle,
                Text = "Ready to capture...",
            };
            captureCard.Controls.Add(_txtResult);

            // Anchor result textbox to fill available space
            captureCard.Resize += (s, e) =>
            {
                _txtResult.Width = captureCard.ClientSize.Width - 30;
                _txtResult.Height = captureCard.ClientSize.Height - 110;
            };

            rightLayout.Controls.Add(captureCard, 0, 1);

            // --- Gallery Card ---
            var galleryCard = CreateCard("CAPTURED IMAGES");
            galleryCard.Dock = DockStyle.Fill;

            _galleryPanel = new FlowLayoutPanel
            {
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Bottom,
                Location = new Point(15, 45),
                Size = new Size(280, 100),
                AutoScroll = true,
                BorderStyle = BorderStyle.FixedSingle,
                BackColor = Color.FromArgb(25, 25, 28),
            };
            galleryCard.Controls.Add(_galleryPanel);

            // Anchor gallery to fill available space
            galleryCard.Resize += (s, e) =>
            {
                _galleryPanel.Width = galleryCard.ClientSize.Width - 30;
                _galleryPanel.Height = galleryCard.ClientSize.Height - 60;
            };

            rightLayout.Controls.Add(galleryCard, 0, 2);

            mainLayout.Controls.Add(rightLayout, 1, 0);

            this.AutoScroll = false;
            this.Controls.Add(mainLayout);
        }
        
        private async void BtnCapture_Click(object sender, EventArgs e)
        {
            _btnCapture.Enabled = false;
            _btnCapture.Text = "Capturing...";
            _txtResult.Text = "Sending capture command...";
            _txtResult.ForeColor = WARNING_COLOR;
            
            try
            {
                var result = await _sender.SendTask1Capture();
                if (result.Success)
                {
                    _txtResult.ForeColor = SUCCESS_COLOR;
                    
                    // Download and display the captured image
                    try
                    {
                        // Parse response data to get image filename and metadata
                        if (!string.IsNullOrEmpty(result.Data))
                        {
                            var data = Newtonsoft.Json.Linq.JObject.Parse(result.Data);
                            var imageName = data["image_name"]?.ToString();
                            
                            // Extract enhanced metadata (with fallback for backward compatibility)
                            var timestamp = data["timestamp"]?.ToString() ?? DateTime.Now.ToString("yyyy-MM-ddTHH:mm:ssZ");
                            var headingDeg = data["heading_deg"]?.ToString() ?? "N/A";
                            var pitchDeg = data["pitch_deg"]?.ToString() ?? "N/A";
                            var rollDeg = data["roll_deg"]?.ToString() ?? "N/A";
                            var gimbalPitch = data["gimbal_pitch_deg"]?.ToString() ?? "N/A";
                            var gimbalYaw = data["gimbal_yaw_deg"]?.ToString() ?? "N/A";
                            var buildingLocation = data["building_location"]?.ToString() ?? "N/A";
                            var captureFolder = data["capture_folder"]?.ToString() ?? "N/A";
                            
                            var position = data["position"];
                            var latStr = position?["lat"]?.ToString() ?? "N/A";
                            var lonStr = position?["lon"]?.ToString() ?? "N/A";
                            var altStr = position?["alt"]?.ToString() ?? "N/A";
                            
                            // Format comprehensive metadata display
                            var metadataText = new StringBuilder();
                            metadataText.AppendLine("[OK] Capture Successful");
                            metadataText.AppendLine($"Time: {timestamp}");
                            metadataText.AppendLine($"Position: {latStr}, {lonStr} @ {altStr}m");
                            metadataText.AppendLine($"AHRS: Hdg={headingDeg} Pitch={pitchDeg} Roll={rollDeg}");
                            metadataText.AppendLine($"Gimbal: Pitch={gimbalPitch} Yaw={gimbalYaw}");
                            metadataText.AppendLine($"Building: {buildingLocation}");
                            metadataText.AppendLine($"Folder: {captureFolder}");
                            metadataText.Append($"Image: {imageName}");
                            
                            _txtResult.Text = metadataText.ToString();
                            
                            if (!string.IsNullOrEmpty(imageName))
                            {
                                // Construct download URL
                                string imageUrl = $"http://{_config.EffectiveIP}:{_config.JetsonPort}/api/task/1/images/{imageName}";
                                
                                // Download image
                                var imageBytes = await JetsonApiService.GetByteArrayAsync(imageUrl);
                                
                                // Save to local directory
                                var task1Dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "NOMAD", "Task1");
                                Directory.CreateDirectory(task1Dir);
                                var localPath = Path.Combine(task1Dir, imageName);
                                File.WriteAllBytes(localPath, imageBytes);

                                // Save metadata JSON file
                                var jsonPath = Path.ChangeExtension(localPath, ".json");
                                var metadata = new SnapshotMetadata
                                {
                                    FileName = imageName,
                                    CaptureTime = DateTime.TryParse(timestamp, out var captureTime) ? captureTime : DateTime.Now,
                                    Position = new PositionData
                                    {
                                        Lat = double.TryParse(latStr, out var lat) ? lat : 0,
                                        Lon = double.TryParse(lonStr, out var lon) ? lon : 0,
                                        Alt = double.TryParse(altStr, out var alt) ? alt : 0
                                    },
                                    HeadingDeg = double.TryParse(headingDeg, out var heading) ? heading : (double?)null,
                                    PitchDeg = double.TryParse(pitchDeg, out var pitch) ? pitch : (double?)null,
                                    RollDeg = double.TryParse(rollDeg, out var roll) ? roll : (double?)null,
                                    GimbalPitchDeg = double.TryParse(gimbalPitch, out var gPitch) ? gPitch : (double?)null,
                                    GimbalYawDeg = double.TryParse(gimbalYaw, out var gYaw) ? gYaw : (double?)null,
                                    BuildingLocation = buildingLocation
                                };

                                File.WriteAllText(jsonPath, Newtonsoft.Json.JsonConvert.SerializeObject(metadata, Newtonsoft.Json.Formatting.Indented));

                                // Auto-generate AI description if enabled
                                if (_config.AiAutoGenerate)
                                {
                                    _ = Task.Run(async () =>
                                    {
                                        try
                                        {
                                            var aiService = new AIDescriptionService(_config);
                                            var aiResult = await aiService.GenerateDescriptionAsync(localPath, metadata);

                                            if (aiResult.Success)
                                            {
                                                // Update metadata with AI description
                                                metadata.AiDescription = aiResult.Description;
                                                metadata.AiProvider = aiResult.Provider.ToString();
                                                metadata.AiModel = aiResult.Model;
                                                metadata.AiGeneratedAt = aiResult.GeneratedAt;

                                                // Save updated metadata
                                                File.WriteAllText(jsonPath, Newtonsoft.Json.JsonConvert.SerializeObject(metadata, Newtonsoft.Json.Formatting.Indented));

                                                // Update UI on main thread
                                                this.BeginInvoke(new Action(() =>
                                                {
                                                    _txtResult.Text += $"\n\n[AI] Description generated using {aiResult.Provider}:\n{aiResult.Description}";
                                                }));
                                            }
                                            else
                                            {
                                                this.BeginInvoke(new Action(() =>
                                                {
                                                    _txtResult.Text += $"\n\n[AI] Failed: {aiResult.ErrorMessage}";
                                                }));
                                            }
                                        }
                                        catch (Exception aiEx)
                                        {
                                            this.BeginInvoke(new Action(() =>
                                            {
                                                _txtResult.Text += $"\n\n[AI] Error: {aiEx.Message}";
                                            }));
                                        }
                                    });
                                }

                                // Add thumbnail to gallery with enhanced metadata tooltip
                                using (var ms = new MemoryStream(imageBytes))
                                {
                                    var originalImage = Image.FromStream(ms);
                                    var thumbnail = originalImage.GetThumbnailImage(120, 90, null, IntPtr.Zero);
                                    
                                    var picBox = new PictureBox
                                    {
                                        Image = thumbnail,
                                        Size = new Size(120, 90),
                                        SizeMode = PictureBoxSizeMode.Zoom,
                                        Margin = new Padding(5),
                                        Cursor = Cursors.Hand,
                                        Tag = new { Path = localPath, Metadata = metadataText.ToString() },
                                    };
                                    
                                    picBox.Click += (s, evt) =>
                                    {
                                        try
                                        {
                                            dynamic tagData = picBox.Tag;
                                            System.Diagnostics.Process.Start(tagData.Path);
                                        }
                                        catch { }
                                    };
                                    
                                    // Enhanced tooltip with full metadata
                                    var tooltip = new ToolTip();
                                    var tooltipText = $"{imageName}\n" +
                                                     $"Position: {latStr}, {lonStr}\n" +
                                                     $"Heading: {headingDeg} Pitch: {pitchDeg} Roll: {rollDeg}\n" +
                                                     $"Gimbal: P={gimbalPitch} Y={gimbalYaw}\n" +
                                                     $"Building: {buildingLocation}";
                                    tooltip.SetToolTip(picBox, tooltipText);
                                    
                                    _galleryPanel.Controls.Add(picBox);
                                    _galleryPanel.ScrollControlIntoView(picBox);
                                    
                                    originalImage.Dispose();
                                }
                            }
                        }
                        else
                        {
                            // Fallback for old API version
                            _txtResult.Text = $"[OK] Capture successful: {result.Message}";
                        }
                    }
                    catch (Exception imgEx)
                    {
                        _txtResult.Text += $"\n[WARN] Image download failed: {imgEx.Message}";
                    }
                }
                else
                {
                    _txtResult.Text = $"[FAIL] Capture failed: {result.Message}";
                    _txtResult.ForeColor = ERROR_COLOR;
                }
            }
            catch (Exception ex)
            {
                _txtResult.Text = $"[ERROR] {ex.Message}";
                _txtResult.ForeColor = ERROR_COLOR;
            }
            finally
            {
                _btnCapture.Enabled = true;
                _btnCapture.Text = "CAPTURE PHOTO WITH METADATA";
            }
        }
        
        public void UpdateData()
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke((MethodInvoker)UpdateData);
                return;
            }
            
            try
            {
                var cs = MainV2.comPort?.MAV?.cs;
                if (cs == null) return;
                
                int gpsFix = (int)cs.gpsstatus;
                string fixText = gpsFix switch
                {
                    3 => "3D Fix",
                    4 => "DGPS",
                    5 => "RTK Float",
                    6 => "RTK Fixed",
                    _ => $"Fix Type {gpsFix}"
                };
                _lblGpsStatus.Text = $"Fix: {fixText} | Satellites: {cs.satcount}";
                _lblGpsStatus.ForeColor = gpsFix >= 3 ? SUCCESS_COLOR : WARNING_COLOR;
                
                _lblPosition.Text = $"Position: {cs.lat:F6}, {cs.lng:F6} | Alt: {cs.alt:F1}m";
            }
            catch { }
        }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _videoPlayer?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
    
    // ============================================================
    // Task 2 View - Indoor Fire Extinguishing
    // ============================================================
    
    public class NOMADTask2View : NOMADViewBase, IUpdatableView
    {
        private readonly DualLinkSender _sender;
        private readonly NOMADConfig _config;
        private readonly JetsonConnectionManager _jetsonConnectionManager;
        private Label _lblVioStatus;
        private Label _lblTargetCount;
        private Button _btnResetMap;
        private Button _btnResetVio;
        private SLAM3DView _slam3DView;
        private TabControl _tabControl;
        
        public NOMADTask2View(DualLinkSender sender, NOMADConfig config, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _sender = sender;
            _config = config;
            _jetsonConnectionManager = jetsonConnectionManager;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            // Use TabControl to switch between Status view and 3D SLAM view
            _tabControl = new TabControl
            {
                Dock = DockStyle.Fill,
            };
            
            // Tab 1: Status & Controls
            var statusTab = new TabPage("Status & Controls")
            {
                BackColor = NOMADTheme.BG_DARK,
            };
            
            var statusLayout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = true,
                Padding = new Padding(10),
            };
            
            // Description
            var descLabel = new Label
            {
                Text = "Task 2: Indoor Fire Extinguishing\n\n" +
                       "VIO-based indoor navigation. GPS is disabled.\n" +
                       "Use the exclusion map to track extinguished targets.\n" +
                       "Switch to '3D SLAM View' tab for real-time 3D mapping.",
                Font = new Font("Segoe UI", 11),
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                MaximumSize = new Size(600, 0),
                Margin = new Padding(0, 0, 0, 20),
            };
            statusLayout.Controls.Add(descLabel);
            
            // VIO Status Card
            var vioCard = CreateCard("VIO STATUS");
            vioCard.Size = new Size(600, 100);
            
            _lblVioStatus = new Label
            {
                Text = "VIO: Inactive",
                Font = new Font("Consolas", 11),
                ForeColor = WARNING_COLOR,
                Location = new Point(15, 50),
                AutoSize = true,
            };
            vioCard.Controls.Add(_lblVioStatus);
            
            _btnResetVio = CreateButton("Reset VIO Origin", SUCCESS_COLOR, 180, 35);
            _btnResetVio.Location = new Point(400, 45);
            _btnResetVio.Click += async (s, e) => await _sender.ResetVioOriginAsync();
            vioCard.Controls.Add(_btnResetVio);
            
            statusLayout.Controls.Add(vioCard);
            
            // Exclusion Map Card
            var mapCard = CreateCard("TARGET EXCLUSION MAP");
            mapCard.Size = new Size(600, 130);
            
            _lblTargetCount = new Label
            {
                Text = "Targets tracked: 0",
                Font = new Font("Segoe UI", 12),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 50),
                AutoSize = true,
            };
            mapCard.Controls.Add(_lblTargetCount);
            
            _btnResetMap = CreateButton("RESET EXCLUSION MAP", ERROR_COLOR, 250, 45);
            _btnResetMap.Location = new Point(15, 80);
            _btnResetMap.Click += async (s, e) =>
            {
                var confirm = MessageBox.Show(
                    "Reset the exclusion map? All tracked targets will be cleared.",
                    "Confirm Reset",
                    MessageBoxButtons.YesNo,
                    MessageBoxIcon.Warning
                );
                if (confirm == DialogResult.Yes)
                {
                    await _sender.SendTask2ResetMap();
                    _lblTargetCount.Text = "Targets tracked: 0";
                }
            };
            mapCard.Controls.Add(_btnResetMap);
            
            statusLayout.Controls.Add(mapCard);
            
            // WASD Control hint
            var wasdLabel = new Label
            {
                Text = "Tip: For manual indoor control, use the dedicated WASD controller in the Quick Panel.",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                Margin = new Padding(0, 20, 0, 0),
            };
            statusLayout.Controls.Add(wasdLabel);
            
            statusTab.Controls.Add(statusLayout);
            _tabControl.TabPages.Add(statusTab);
            
            // Tab 2: 3D SLAM View
            var slam3DTab = new TabPage("3D SLAM View")
            {
                BackColor = NOMADTheme.BG_DARK,
            };
            
            try
            {
                _slam3DView = new SLAM3DView(_config);
                _slam3DView.Dock = DockStyle.Fill;
                slam3DTab.Controls.Add(_slam3DView);
            }
            catch (Exception ex)
            {
                var errorLabel = new Label
                {
                    Text = $"3D SLAM View unavailable: {ex.Message}\n\n" +
                           "This may be due to missing Helix Toolkit dependencies.\n" +
                           "Ensure HelixToolkit.Wpf NuGet package is installed.",
                    Font = new Font("Segoe UI", 11),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                    Padding = new Padding(20),
                };
                slam3DTab.Controls.Add(errorLabel);
            }
            
            _tabControl.TabPages.Add(slam3DTab);
            
            this.Controls.Add(_tabControl);
        }
        
        public void UpdateData()
        {
            // VIO status updates would come from Jetson API
        }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _slam3DView?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
    
    // ============================================================
    // Video View with WASD Controls
    // ============================================================
    
    public class NOMADVideoView : NOMADViewBase, IUpdatableView
    {
        private readonly DualLinkSender _sender;
        private readonly NOMADConfig _config;
        private readonly JetsonConnectionManager _jetsonConnectionManager;
        private EmbeddedVideoPlayer _videoPlayer;
        private EnhancedWASDControl _wasdControl;
        private Label _lblStatus;
        
        public NOMADVideoView(DualLinkSender sender, NOMADConfig config, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _sender = sender;
            _config = config;
            _jetsonConnectionManager = jetsonConnectionManager;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            // Main horizontal split: Video (left) + Controls (right)
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
            };
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 60));  // Video
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 40));  // Controls
            
            // Left side: Video with controls
            var videoSection = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 1,
            };
            videoSection.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            
            // Video player panel - left ZED camera
            var videoPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                Margin = new Padding(5),
            };
            
            try
            {
                // RTSP URL for ZED stream - left camera will be cropped
                string rtspUrl = $"rtsp://{_config.EffectiveIP}:8554/primary";
                _videoPlayer = new EmbeddedVideoPlayer("ZED Left Camera", rtspUrl, true, _jetsonConnectionManager);
                _videoPlayer.Dock = DockStyle.Fill;
                videoPanel.Controls.Add(_videoPlayer);
            }
            catch (Exception ex)
            {
                _lblStatus = new Label
                {
                    Text = $"Video player unavailable: {ex.Message}\n\n" +
                           $"Stream URL: rtsp://{_config.EffectiveIP}:8554/primary\n\n" +
                           "Use VLC or another player to view the stream.",
                    Font = new Font("Segoe UI", 12),
                    ForeColor = TEXT_SECONDARY,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                videoPanel.Controls.Add(_lblStatus);
            }
            
            videoSection.Controls.Add(videoPanel, 0, 0);
            
            // Video player has built-in controls - no need for duplicate controls panel
            
            mainLayout.Controls.Add(videoSection, 0, 0);
            
            // Right side: WASD Controls (full height - includes payload controls)
            var controlsSection = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 1,
            };
            controlsSection.RowStyles.Add(new RowStyle(SizeType.Percent, 100));  // WASD with payload controls

            try
            {
                _wasdControl = new EnhancedWASDControl(
                    _config,
                    _config.WasdNudgeSpeed,
                    _config.WasdAltSpeed,
                    15.0f,  // Default yaw rate
                    _jetsonConnectionManager
                );
                _wasdControl.Dock = DockStyle.Fill;
                controlsSection.Controls.Add(_wasdControl, 0, 0);
            }
            catch (Exception ex)
            {
                var errorPanel = new Panel
                {
                    Dock = DockStyle.Fill,
                    BackColor = CARD_BG,
                };
                var errorLabel = new Label
                {
                    Text = $"WASD controls unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 11),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                errorPanel.Controls.Add(errorLabel);
                controlsSection.Controls.Add(errorPanel, 0, 0);
            }

            mainLayout.Controls.Add(controlsSection, 1, 0);
            
            this.Controls.Add(mainLayout);
        }
        
        public void UpdateData() { }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _videoPlayer?.Dispose();
                _wasdControl?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
    
    // ============================================================
    // Terminal View
    // ============================================================
    
    public class NOMADTerminalView : NOMADViewBase
    {
        private readonly NOMADConfig _config;
        private JetsonTerminalControl _terminal;
        
        public NOMADTerminalView(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            try
            {
                _terminal = new JetsonTerminalControl(_config);
                _terminal.Dock = DockStyle.Fill;
                this.Controls.Add(_terminal);
            }
            catch (Exception ex)
            {
                var errorLabel = new Label
                {
                    Text = $"Terminal unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 12),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                this.Controls.Add(errorLabel);
            }
        }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _terminal?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
    
    // ============================================================
    // Health View
    // ============================================================
    
    public class NOMADHealthView : NOMADViewBase, IUpdatableView
    {
        private readonly NOMADConfig _config;
        private readonly DualLinkSender _sender;
        private EnhancedHealthDashboard _healthDashboard;
        private ServiceControlPanel _serviceControlPanel;
        private TabControl _tabControl;
        
        public NOMADHealthView(NOMADConfig config, DualLinkSender sender = null)
        {
            _config = config;
            _sender = sender;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            try
            {
                // Create a TabControl to hold both Health Dashboard and Service Control
                _tabControl = new TabControl
                {
                    Dock = DockStyle.Fill,
                    Font = new Font("Segoe UI", 10),
                };
                
                // Tab 1: Health Dashboard
                var healthTab = new TabPage("Jetson Health")
                {
                    BackColor = Color.FromArgb(30, 30, 33),
                };
                _healthDashboard = new EnhancedHealthDashboard(_config);
                _healthDashboard.Dock = DockStyle.Fill;
                healthTab.Controls.Add(_healthDashboard);
                _tabControl.TabPages.Add(healthTab);
                
                // Tab 2: Service Control
                var serviceTab = new TabPage("Service Control")
                {
                    BackColor = Color.FromArgb(30, 30, 33),
                    AutoScroll = true,
                };
                
                if (_sender != null)
                {
                    _serviceControlPanel = new ServiceControlPanel(_sender, _config.HealthPollInterval);
                    _serviceControlPanel.Dock = DockStyle.Fill;
                    serviceTab.Controls.Add(_serviceControlPanel);
                }
                else
                {
                    var noSenderLabel = new Label
                    {
                        Text = "Service control unavailable - no sender configured.\n\nConnect to Jetson to enable service control.",
                        Font = new Font("Segoe UI", 11),
                        ForeColor = WARNING_COLOR,
                        Dock = DockStyle.Fill,
                        TextAlign = ContentAlignment.MiddleCenter,
                    };
                    serviceTab.Controls.Add(noSenderLabel);
                }
                _tabControl.TabPages.Add(serviceTab);
                
                this.Controls.Add(_tabControl);
            }
            catch (Exception ex)
            {
                var errorLabel = new Label
                {
                    Text = $"Health dashboard unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 12),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                this.Controls.Add(errorLabel);
            }
        }
        
        public void UpdateData()
        {
            _healthDashboard?.RefreshHealth();
        }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _healthDashboard?.Dispose();
                _serviceControlPanel?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
    
    // ============================================================
    // Links View
    // ============================================================
    
    public class NOMADLinksView : NOMADViewBase, IUpdatableView
    {
        private readonly MAVLinkConnectionManager _connectionManager;
        private readonly NOMADConfig _config;
        private LinkHealthPanel _linkPanel;
        
        public NOMADLinksView(MAVLinkConnectionManager connectionManager, NOMADConfig config)
        {
            _connectionManager = connectionManager;
            _config = config;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            if (_connectionManager != null)
            {
                try
                {
                    _linkPanel = new LinkHealthPanel(_connectionManager, _config);
                    _linkPanel.Dock = DockStyle.Fill;
                    this.Controls.Add(_linkPanel);
                    return;
                }
                catch { }
            }
            
            // Fallback if no connection manager
            var layout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
            };
            
            var descLabel = new Label
            {
                Text = "Dual Link Failover Status\n\n" +
                       "Monitor the health of both communication links:\n" +
                       "• LTE/Tailscale: Primary long-range link via 4G\n" +
                       "• RadioMaster: Backup RC link via ELRS",
                Font = new Font("Segoe UI", 11),
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                MaximumSize = new Size(600, 0),
                Margin = new Padding(0, 0, 0, 20),
            };
            layout.Controls.Add(descLabel);
            
            var statusLabel = new Label
            {
                Text = _config.DualLinkEnabled 
                    ? "[OK] Dual link monitoring is enabled" 
                    : "[!] Dual link is disabled in settings",
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = _config.DualLinkEnabled ? SUCCESS_COLOR : WARNING_COLOR,
                AutoSize = true,
            };
            layout.Controls.Add(statusLabel);
            
            this.Controls.Add(layout);
        }
        
        public void UpdateData()
        {
            // Link panel updates itself
        }
    }
    
    // ============================================================
    // Boundary View - Flight Boundary Configuration & Monitoring
    // ============================================================
    
    /// <summary>
    /// Boundary preset for save/load functionality
    /// </summary>
    public class BoundaryPreset
    {
        public string Name { get; set; }
        public string Description { get; set; }
        public int TaskNumber { get; set; } // 0 = both, 1 = Task 1 only, 2 = Task 2 only
        public DateTime CreatedAt { get; set; }
        public List<GpsPoint> SoftBoundary { get; set; } = new List<GpsPoint>();
        public List<GpsPoint> HardBoundary { get; set; } = new List<GpsPoint>();
        public double MaxAltitudeMeters { get; set; } = 122.0; // 400ft
        public GpsPoint BuildingLocation { get; set; }
    }
    
    public class NOMADBoundaryView : NOMADViewBase, IUpdatableView
    {
        private readonly MissionConfig _missionConfig;
        private readonly NOMADConfig _config;
        private readonly BoundaryMonitor _monitor;
        
        // Status display
        private Panel _statusPanel;
        private Label _lblStatus;
        private Label _lblCountdown;
        private Label _lblAltitude;
        private Label _lblPosition;
        
        // Boundary grids
        private DataGridView _dgvSoftBoundary;
        private DataGridView _dgvHardBoundary;
        
        // Task selection
        private ComboBox _cmbTask;
        private CheckBox _chkEnableMonitoring;
        private NumericUpDown _nudMaxAlt;
        
        // Building location
        private TextBox _txtBuildingLat;
        private TextBox _txtBuildingLon;
        
        // Violation action controls
        private ComboBox _cmbSoftAction;
        private ComboBox _cmbHardAction;
        private NumericUpDown _nudKillDelay;
        
        // Preset management
        private ComboBox _cmbPresets;
        private List<BoundaryPreset> _presets = new List<BoundaryPreset>();
        private static readonly string PresetsDir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "Mission Planner", "plugins", "NOMAD", "boundary_presets");
        
        public NOMADBoundaryView(MissionConfig missionConfig, NOMADConfig config, BoundaryMonitor monitor)
        {
            _missionConfig = missionConfig;
            _config = config;
            _monitor = monitor;
            
            LoadPresets();
            InitializeUI();
            LoadBoundaries();
            
            // Subscribe to monitor events
            if (_monitor != null)
            {
                _monitor.BoundaryStatusChanged += Monitor_BoundaryStatusChanged;
                _monitor.BoundaryViolation += Monitor_BoundaryViolation;
            }
        }
        
        // Return location fields
        private TextBox _txtReturnLat;
        private TextBox _txtReturnLon;

        private void InitializeUI()
        {
            // ============================================================
            // Two-column layout: Left = boundaries, Right = settings
            // ============================================================
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 2,
                Padding = new Padding(0),
            };
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            // Row 0: status (fixed height), Row 1: content (fill)
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 75));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            // ============================================================
            // Status Panel (spans both columns)
            // ============================================================
            _statusPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(80, 80, 90),
                Margin = new Padding(3),
            };

            _lblStatus = new Label
            {
                Text = "[?] Waiting for GPS Position",
                Font = new Font("Segoe UI", 14, FontStyle.Bold),
                ForeColor = Color.White,
                Location = new Point(15, 8),
                AutoSize = true,
            };
            _statusPanel.Controls.Add(_lblStatus);

            _lblCountdown = new Label
            {
                Text = "",
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = Color.Yellow,
                Location = new Point(15, 35),
                AutoSize = true,
                Visible = false,
            };
            _statusPanel.Controls.Add(_lblCountdown);

            _lblPosition = new Label
            {
                Text = "Position: --",
                Font = new Font("Consolas", 8),
                ForeColor = Color.White,
                Location = new Point(15, 50),
                AutoSize = true,
            };
            _statusPanel.Controls.Add(_lblPosition);

            _lblAltitude = new Label
            {
                Text = "Alt: -- / 122m",
                Font = new Font("Consolas", 8),
                ForeColor = Color.White,
                Location = new Point(280, 50),
                AutoSize = true,
            };
            _statusPanel.Controls.Add(_lblAltitude);

            mainLayout.Controls.Add(_statusPanel, 0, 0);
            mainLayout.SetColumnSpan(_statusPanel, 2);

            // ============================================================
            // LEFT COLUMN: Boundary point grids
            // ============================================================
            var leftScroll = new Panel { Dock = DockStyle.Fill, AutoScroll = true, Padding = new Padding(3) };
            var leftContent = new FlowLayoutPanel
            {
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoSize = true,
                Dock = DockStyle.Top,
            };
            leftContent.SizeChanged += (s, e) => { leftContent.Width = leftScroll.ClientSize.Width - 8; };
            leftScroll.Resize += (s, e) => { leftContent.Width = leftScroll.ClientSize.Width - 8; };

            // -- Soft Boundary --
            var softCard = CreateCard("SOFT BOUNDARY (Warning)");
            softCard.Dock = DockStyle.Top;
            softCard.Height = 200;
            softCard.Margin = new Padding(0, 0, 0, 3);

            _dgvSoftBoundary = CreateBoundaryGrid();
            _dgvSoftBoundary.Location = new Point(10, 40);
            _dgvSoftBoundary.Size = new Size(200, 110);
            _dgvSoftBoundary.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
            _dgvSoftBoundary.CellValueChanged += (s, e) => SaveBoundaryFromGrid(_dgvSoftBoundary, _missionConfig.SoftBoundary);
            softCard.Controls.Add(_dgvSoftBoundary);

            // Buttons to the right of the grid
            var softBtnX = 10;
            var btnPasteSoft = CreateButton("Paste", ACCENT_COLOR, 65, 24);
            btnPasteSoft.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold);
            btnPasteSoft.Location = new Point(softBtnX, 158);
            btnPasteSoft.Click += (s, e) => PasteCoordinates(_dgvSoftBoundary, _missionConfig.SoftBoundary, "soft");
            softCard.Controls.Add(btnPasteSoft);

            var btnAddSoft = CreateButton("+ Add", SUCCESS_COLOR, 55, 24);
            btnAddSoft.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold);
            btnAddSoft.Location = new Point(80, 158);
            btnAddSoft.Click += (s, e) => AddManualPoint(_dgvSoftBoundary, _missionConfig.SoftBoundary);
            softCard.Controls.Add(btnAddSoft);

            var btnClearSoft = CreateButton("Clear", ERROR_COLOR, 50, 24);
            btnClearSoft.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold);
            btnClearSoft.Location = new Point(140, 158);
            btnClearSoft.Click += (s, e) => ClearBoundary(_dgvSoftBoundary, _missionConfig.SoftBoundary);
            softCard.Controls.Add(btnClearSoft);

            var lblSoftCount = new Label
            {
                Name = "lblSoftCount",
                Text = $"{_missionConfig.SoftBoundary.Vertices.Count} pts",
                Font = new Font("Segoe UI", 8),
                ForeColor = Color.Yellow,
                Location = new Point(200, 162),
                AutoSize = true,
            };
            softCard.Controls.Add(lblSoftCount);

            leftContent.Controls.Add(softCard);

            // -- Hard Boundary --
            var hardCard = CreateCard("HARD BOUNDARY (Kill)");
            hardCard.Dock = DockStyle.Top;
            hardCard.Height = 200;
            hardCard.Margin = new Padding(0, 0, 0, 3);

            _dgvHardBoundary = CreateBoundaryGrid();
            _dgvHardBoundary.Location = new Point(10, 40);
            _dgvHardBoundary.Size = new Size(200, 110);
            _dgvHardBoundary.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
            _dgvHardBoundary.CellValueChanged += (s, e) => SaveBoundaryFromGrid(_dgvHardBoundary, _missionConfig.HardBoundary);
            hardCard.Controls.Add(_dgvHardBoundary);

            var btnPasteHard = CreateButton("Paste", ACCENT_COLOR, 65, 24);
            btnPasteHard.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold);
            btnPasteHard.Location = new Point(10, 158);
            btnPasteHard.Click += (s, e) => PasteCoordinates(_dgvHardBoundary, _missionConfig.HardBoundary, "hard");
            hardCard.Controls.Add(btnPasteHard);

            var btnAddHard = CreateButton("+ Add", SUCCESS_COLOR, 55, 24);
            btnAddHard.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold);
            btnAddHard.Location = new Point(80, 158);
            btnAddHard.Click += (s, e) => AddManualPoint(_dgvHardBoundary, _missionConfig.HardBoundary);
            hardCard.Controls.Add(btnAddHard);

            var btnClearHard = CreateButton("Clear", ERROR_COLOR, 50, 24);
            btnClearHard.Font = new Font("Segoe UI", 7.5f, FontStyle.Bold);
            btnClearHard.Location = new Point(140, 158);
            btnClearHard.Click += (s, e) => ClearBoundary(_dgvHardBoundary, _missionConfig.HardBoundary);
            hardCard.Controls.Add(btnClearHard);

            var lblHardCount = new Label
            {
                Name = "lblHardCount",
                Text = $"{_missionConfig.HardBoundary.Vertices.Count} pts",
                Font = new Font("Segoe UI", 8),
                ForeColor = Color.Red,
                Location = new Point(200, 162),
                AutoSize = true,
            };
            hardCard.Controls.Add(lblHardCount);

            leftContent.Controls.Add(hardCard);

            // -- Import / Export --
            var importCard = CreateCard("IMPORT / EXPORT");
            importCard.Dock = DockStyle.Top;
            importCard.Height = 105;
            importCard.Margin = new Padding(0, 0, 0, 3);

            var lblImport = new Label { Text = "Import:", Font = new Font("Segoe UI", 7.5f, FontStyle.Bold), ForeColor = TEXT_SECONDARY, Location = new Point(10, 42), AutoSize = true };
            importCard.Controls.Add(lblImport);

            var btnImportKml = CreateButton("KML", ACCENT_COLOR, 48, 22);
            btnImportKml.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnImportKml.Location = new Point(60, 39);
            btnImportKml.Click += BtnImportKml_Click;
            importCard.Controls.Add(btnImportKml);

            var btnImportCSV = CreateButton("Coords", ACCENT_COLOR, 55, 22);
            btnImportCSV.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnImportCSV.Location = new Point(112, 39);
            btnImportCSV.Click += BtnImportGoogleMaps_Click;
            importCard.Controls.Add(btnImportCSV);

            var btnGetFromMP = CreateButton("MP Fence", ACCENT_COLOR, 65, 22);
            btnGetFromMP.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnGetFromMP.Location = new Point(171, 39);
            btnGetFromMP.Click += BtnGetFromMP_Click;
            importCard.Controls.Add(btnGetFromMP);

            var lblExport = new Label { Text = "Export:", Font = new Font("Segoe UI", 7.5f, FontStyle.Bold), ForeColor = TEXT_SECONDARY, Location = new Point(10, 68), AutoSize = true };
            importCard.Controls.Add(lblExport);

            var btnExportToMPFence = CreateButton("To MP", SUCCESS_COLOR, 55, 22);
            btnExportToMPFence.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnExportToMPFence.Location = new Point(60, 65);
            btnExportToMPFence.Click += BtnExportToMPFence_Click;
            importCard.Controls.Add(btnExportToMPFence);

            var btnExportFenFile = CreateButton("Save .fen", SUCCESS_COLOR, 65, 22);
            btnExportFenFile.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnExportFenFile.Location = new Point(119, 65);
            btnExportFenFile.Click += BtnExportPolyFile_Click;
            importCard.Controls.Add(btnExportFenFile);

            leftContent.Controls.Add(importCard);

            leftScroll.Controls.Add(leftContent);
            mainLayout.Controls.Add(leftScroll, 0, 1);

            // ============================================================
            // RIGHT COLUMN: Settings, return point, actions, presets
            // ============================================================
            var rightScroll = new Panel { Dock = DockStyle.Fill, AutoScroll = true, Padding = new Padding(3) };
            var rightContent = new FlowLayoutPanel
            {
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoSize = true,
                Dock = DockStyle.Top,
            };
            rightContent.SizeChanged += (s, e) => { rightContent.Width = rightScroll.ClientSize.Width - 8; };
            rightScroll.Resize += (s, e) => { rightContent.Width = rightScroll.ClientSize.Width - 8; };

            // -- Monitoring Settings --
            var settingsCard = CreateCard("MONITORING");
            settingsCard.Dock = DockStyle.Top;
            settingsCard.Height = 110;
            settingsCard.Margin = new Padding(0, 0, 0, 3);

            var lblTask = new Label { Text = "Task:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(10, 40), AutoSize = true };
            settingsCard.Controls.Add(lblTask);

            _cmbTask = new ComboBox
            {
                Location = new Point(50, 37), Size = new Size(170, 22),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
            _cmbTask.Items.AddRange(new object[] { "Task 1 - Outdoor", "Task 2 - Indoor" });
            _cmbTask.SelectedIndex = _missionConfig.CurrentTask - 1;
            _cmbTask.SelectedIndexChanged += (s, e) => { _missionConfig.CurrentTask = _cmbTask.SelectedIndex + 1; _missionConfig.Save(); };
            settingsCard.Controls.Add(_cmbTask);

            _chkEnableMonitoring = new CheckBox
            {
                Text = "Real-time Monitor",
                ForeColor = Color.White, Font = new Font("Segoe UI", 8),
                Location = new Point(10, 65), AutoSize = true,
                Checked = _monitor?.IsMonitoring ?? false,
            };
            _chkEnableMonitoring.CheckedChanged += (s, e) => { if (_chkEnableMonitoring.Checked) _monitor?.StartMonitoring(); else _monitor?.StopMonitoring(); };
            settingsCard.Controls.Add(_chkEnableMonitoring);

            var lblMaxAlt = new Label { Text = "Max Alt:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(10, 88), AutoSize = true };
            settingsCard.Controls.Add(lblMaxAlt);

            _nudMaxAlt = new NumericUpDown
            {
                Location = new Point(65, 85), Size = new Size(60, 22),
                Minimum = 10, Maximum = 150,
                Value = (decimal)_missionConfig.MaxAltitudeAglMeters,
                BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White,
            };
            _nudMaxAlt.ValueChanged += (s, e) => { _missionConfig.MaxAltitudeAglMeters = (double)_nudMaxAlt.Value; _missionConfig.Save(); };
            settingsCard.Controls.Add(_nudMaxAlt);

            var lblMeters = new Label { Text = "m AGL", Font = new Font("Segoe UI", 7), ForeColor = TEXT_SECONDARY, Location = new Point(128, 89), AutoSize = true };
            settingsCard.Controls.Add(lblMeters);

            rightContent.Controls.Add(settingsCard);

            // -- Return Location --
            var returnCard = CreateCard("RETURN LOCATION");
            returnCard.Dock = DockStyle.Top;
            returnCard.Height = 100;
            returnCard.Margin = new Padding(0, 0, 0, 3);

            var lblRetLat = new Label { Text = "Lat:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(10, 40), AutoSize = true };
            returnCard.Controls.Add(lblRetLat);
            _txtReturnLat = new TextBox
            {
                Location = new Point(35, 37), Size = new Size(95, 22),
                BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
            returnCard.Controls.Add(_txtReturnLat);

            var lblRetLon = new Label { Text = "Lon:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(135, 40), AutoSize = true };
            returnCard.Controls.Add(lblRetLon);
            _txtReturnLon = new TextBox
            {
                Location = new Point(162, 37), Size = new Size(95, 22),
                BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
            returnCard.Controls.Add(_txtReturnLon);

            // Load saved return point
            if (_missionConfig.ReturnPoint != null)
            {
                _txtReturnLat.Text = _missionConfig.ReturnPoint.Lat.ToString("F7");
                _txtReturnLon.Text = _missionConfig.ReturnPoint.Lon.ToString("F7");
            }

            var btnReturnCurrent = CreateButton("Use Current", ACCENT_COLOR, 85, 22);
            btnReturnCurrent.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnReturnCurrent.Location = new Point(10, 68);
            btnReturnCurrent.Click += (s, e) =>
            {
                double lat = MainV2.comPort?.MAV?.cs?.lat ?? 0;
                double lon = MainV2.comPort?.MAV?.cs?.lng ?? 0;
                if (lat != 0 || lon != 0)
                {
                    _txtReturnLat.Text = lat.ToString("F7");
                    _txtReturnLon.Text = lon.ToString("F7");
                    SaveReturnPoint();
                }
                else
                    CustomMessageBox.Show("No GPS position available.", "Warning");
            };
            returnCard.Controls.Add(btnReturnCurrent);

            var btnReturnSave = CreateButton("Save", SUCCESS_COLOR, 50, 22);
            btnReturnSave.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnReturnSave.Location = new Point(100, 68);
            btnReturnSave.Click += (s, e) => SaveReturnPoint();
            returnCard.Controls.Add(btnReturnSave);

            var btnReturnCentroid = CreateButton("Use Centroid", INFO_COLOR, 85, 22);
            btnReturnCentroid.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnReturnCentroid.Location = new Point(155, 68);
            btnReturnCentroid.Click += (s, e) =>
            {
                var boundary = _missionConfig.HardBoundary?.Vertices?.Count > 0
                    ? _missionConfig.HardBoundary : _missionConfig.SoftBoundary;
                if (boundary?.Vertices?.Count >= 3)
                {
                    double cLat = 0, cLon = 0;
                    foreach (var v in boundary.Vertices) { cLat += v.Lat; cLon += v.Lon; }
                    cLat /= boundary.Vertices.Count; cLon /= boundary.Vertices.Count;
                    _txtReturnLat.Text = cLat.ToString("F7");
                    _txtReturnLon.Text = cLon.ToString("F7");
                    SaveReturnPoint();
                }
                else
                    CustomMessageBox.Show("No boundary defined to compute centroid.", "Warning");
            };
            returnCard.Controls.Add(btnReturnCentroid);

            rightContent.Controls.Add(returnCard);

            // -- Violation Actions --
            var actionCard = CreateCard("VIOLATION ACTIONS");
            actionCard.Dock = DockStyle.Top;
            actionCard.Height = 105;
            actionCard.Margin = new Padding(0, 0, 0, 3);

            var lblSoftAction = new Label { Text = "Soft:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(10, 40), AutoSize = true };
            actionCard.Controls.Add(lblSoftAction);

            _cmbSoftAction = new ComboBox
            {
                Location = new Point(42, 37), Size = new Size(145, 22),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
            _cmbSoftAction.Items.AddRange(new object[] { "Warn (Audio)", "Warn (Visual)", "Warn (Both)", "Return to Boundary" });
            var softActionMap = new Dictionary<string, int> { { "warn_audio", 0 }, { "warn_visual", 1 }, { "warn_both", 2 }, { "return_to_boundary", 3 } };
            _cmbSoftAction.SelectedIndex = softActionMap.TryGetValue(_missionConfig.Failsafe.SoftBoundaryAction ?? "", out int softIdx) ? softIdx : 2;
            _cmbSoftAction.SelectedIndexChanged += (s, e) =>
            {
                _missionConfig.Failsafe.SoftBoundaryAction = _cmbSoftAction.SelectedItem.ToString().ToLower().Replace(" ", "_").Replace("(", "").Replace(")", "");
                _missionConfig.Save();
            };
            actionCard.Controls.Add(_cmbSoftAction);

            var lblHardAction = new Label { Text = "Hard:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(10, 65), AutoSize = true };
            actionCard.Controls.Add(lblHardAction);

            _cmbHardAction = new ComboBox
            {
                Location = new Point(42, 62), Size = new Size(145, 22),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
            _cmbHardAction.Items.AddRange(new object[] { "Warn and Kill", "Auto Kill", "Warn Only" });
            var hardActionMap = new Dictionary<string, int> { { "warn_and_kill", 0 }, { "auto_kill", 1 }, { "warn_only", 2 } };
            _cmbHardAction.SelectedIndex = hardActionMap.TryGetValue(_missionConfig.Failsafe.HardBoundaryAction ?? "", out int hardIdx) ? hardIdx : 0;
            _cmbHardAction.SelectedIndexChanged += (s, e) =>
            {
                _missionConfig.Failsafe.HardBoundaryAction = _cmbHardAction.SelectedItem.ToString().ToLower().Replace(" ", "_");
                _missionConfig.Save();
            };
            actionCard.Controls.Add(_cmbHardAction);

            var lblKillDelay = new Label { Text = "Kill:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(195, 65), AutoSize = true };
            actionCard.Controls.Add(lblKillDelay);

            _nudKillDelay = new NumericUpDown
            {
                Location = new Point(222, 62), Size = new Size(45, 22),
                Minimum = 1, Maximum = 30,
                Value = _missionConfig.Failsafe.HardBoundaryKillDelaySec,
                BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White,
            };
            _nudKillDelay.ValueChanged += (s, e) => { _missionConfig.Failsafe.HardBoundaryKillDelaySec = (int)_nudKillDelay.Value; _missionConfig.Save(); };
            actionCard.Controls.Add(_nudKillDelay);

            var lblSec = new Label { Text = "s", Font = new Font("Segoe UI", 7), ForeColor = TEXT_SECONDARY, Location = new Point(270, 67), AutoSize = true };
            actionCard.Controls.Add(lblSec);

            rightContent.Controls.Add(actionCard);

            // -- Building Location --
            var buildingCard = CreateCard("BUILDING LOCATION");
            buildingCard.Dock = DockStyle.Top;
            buildingCard.Height = 80;
            buildingCard.Margin = new Padding(0, 0, 0, 3);

            var lblBuildingLat = new Label { Text = "Lat:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(10, 42), AutoSize = true };
            buildingCard.Controls.Add(lblBuildingLat);
            _txtBuildingLat = new TextBox { Location = new Point(35, 39), Size = new Size(95, 22), BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White, Font = new Font("Segoe UI", 8) };
            buildingCard.Controls.Add(_txtBuildingLat);

            var lblBuildingLon = new Label { Text = "Lon:", Font = new Font("Segoe UI", 8), ForeColor = TEXT_PRIMARY, Location = new Point(135, 42), AutoSize = true };
            buildingCard.Controls.Add(lblBuildingLon);
            _txtBuildingLon = new TextBox { Location = new Point(162, 39), Size = new Size(95, 22), BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White, Font = new Font("Segoe UI", 8) };
            buildingCard.Controls.Add(_txtBuildingLon);

            var btnSaveBuilding = CreateButton("Save", SUCCESS_COLOR, 40, 22);
            btnSaveBuilding.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnSaveBuilding.Location = new Point(10, 65);
            btnSaveBuilding.Click += SaveBuildingLocation;
            buildingCard.Controls.Add(btnSaveBuilding);

            var btnShowBuilding = CreateButton("Show", ACCENT_COLOR, 45, 22);
            btnShowBuilding.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnShowBuilding.Location = new Point(55, 65);
            btnShowBuilding.Click += ShowBuildingOnMap;
            buildingCard.Controls.Add(btnShowBuilding);

            rightContent.Controls.Add(buildingCard);

            // -- Presets --
            var presetCard = CreateCard("PRESETS");
            presetCard.Dock = DockStyle.Top;
            presetCard.Height = 80;
            presetCard.Margin = new Padding(0, 0, 0, 3);

            _cmbPresets = new ComboBox
            {
                Location = new Point(10, 40), Size = new Size(150, 22),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(50, 50, 53), ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
            RefreshPresetCombo();
            presetCard.Controls.Add(_cmbPresets);

            var btnLoadPreset = CreateButton("Load", SUCCESS_COLOR, 45, 22);
            btnLoadPreset.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnLoadPreset.Location = new Point(165, 40);
            btnLoadPreset.Click += LoadSelectedPreset;
            presetCard.Controls.Add(btnLoadPreset);

            var btnSavePreset = CreateButton("Save", ACCENT_COLOR, 45, 22);
            btnSavePreset.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnSavePreset.Location = new Point(215, 40);
            btnSavePreset.Click += SaveCurrentAsPreset;
            presetCard.Controls.Add(btnSavePreset);

            var btnDeletePreset = CreateButton("Del", ERROR_COLOR, 35, 22);
            btnDeletePreset.Font = new Font("Segoe UI", 7, FontStyle.Bold);
            btnDeletePreset.Location = new Point(265, 40);
            btnDeletePreset.Click += DeleteSelectedPreset;
            presetCard.Controls.Add(btnDeletePreset);

            rightContent.Controls.Add(presetCard);

            rightScroll.Controls.Add(rightContent);
            mainLayout.Controls.Add(rightScroll, 1, 1);

            this.Controls.Add(mainLayout);
        }

        private void SaveReturnPoint()
        {
            if (double.TryParse(_txtReturnLat.Text, out double lat) &&
                double.TryParse(_txtReturnLon.Text, out double lon))
            {
                _missionConfig.ReturnPoint = new GpsPoint(lat, lon);
                _missionConfig.Save();
                CustomMessageBox.Show($"Return point saved: {lat:F7}, {lon:F7}", "Saved");
            }
            else
            {
                CustomMessageBox.Show("Enter valid latitude and longitude.", "Warning");
            }
        }
        
        private DataGridView CreateBoundaryGrid()
        {
            var dgv = new DataGridView
            {
                AllowUserToAddRows = false,
                AllowUserToDeleteRows = true,
                AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill,
                BackgroundColor = Color.FromArgb(30, 30, 30),
                BorderStyle = BorderStyle.None,
                CellBorderStyle = DataGridViewCellBorderStyle.SingleHorizontal,
                ColumnHeadersHeightSizeMode = DataGridViewColumnHeadersHeightSizeMode.AutoSize,
                DefaultCellStyle = new DataGridViewCellStyle
                {
                    BackColor = Color.FromArgb(40, 40, 43),
                    ForeColor = Color.White,
                    SelectionBackColor = Color.FromArgb(0, 122, 204),
                    SelectionForeColor = Color.White,
                },
                EnableHeadersVisualStyles = false,
                GridColor = Color.FromArgb(60, 60, 63),
                RowHeadersVisible = false,
                SelectionMode = DataGridViewSelectionMode.FullRowSelect,
            };
            
            dgv.Columns.Add(new DataGridViewTextBoxColumn { Name = "Lat", HeaderText = "Latitude", FillWeight = 50 });
            dgv.Columns.Add(new DataGridViewTextBoxColumn { Name = "Lon", HeaderText = "Longitude", FillWeight = 50 });
            
            dgv.ColumnHeadersDefaultCellStyle = new DataGridViewCellStyle
            {
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            
            return dgv;
        }
        
        private void LoadBoundaries()
        {
            // Load soft boundary
            _dgvSoftBoundary.Rows.Clear();
            foreach (var point in _missionConfig.SoftBoundary.Vertices)
            {
                _dgvSoftBoundary.Rows.Add(point.Lat.ToString("F8"), point.Lon.ToString("F8"));
            }
            
            // Load hard boundary
            _dgvHardBoundary.Rows.Clear();
            foreach (var point in _missionConfig.HardBoundary.Vertices)
            {
                _dgvHardBoundary.Rows.Add(point.Lat.ToString("F8"), point.Lon.ToString("F8"));
            }
            
            // Load building location based on current task
            var building = _missionConfig.CurrentTask == 1 
                ? _missionConfig.Task1Building 
                : _missionConfig.Task2Building;
            if (building?.Coordinates != null)
            {
                _txtBuildingLat.Text = building.Coordinates.Lat.ToString("F8");
                _txtBuildingLon.Text = building.Coordinates.Lon.ToString("F8");
            }
            
            UpdatePointCounts();
        }
        
        private void UpdatePointCounts()
        {
            var softLabel = this.Controls.Find("lblSoftCount", true).FirstOrDefault() as Label;
            var hardLabel = this.Controls.Find("lblHardCount", true).FirstOrDefault() as Label;
            
            if (softLabel != null)
                softLabel.Text = $"Points: {_missionConfig.SoftBoundary.Vertices.Count}";
            if (hardLabel != null)
                hardLabel.Text = $"Points: {_missionConfig.HardBoundary.Vertices.Count}";
        }
        
        private void PasteCoordinates(DataGridView dgv, FlightBoundary boundary, string boundaryType)
        {
            using (var inputForm = new Form())
            {
                inputForm.Width = 550;
                inputForm.Height = 400;
                inputForm.Text = $"Paste {boundaryType.ToUpper()} Boundary Coordinates";
                inputForm.StartPosition = FormStartPosition.CenterParent;
                inputForm.BackColor = Color.FromArgb(40, 40, 45);
                inputForm.FormBorderStyle = FormBorderStyle.FixedDialog;
                inputForm.MaximizeBox = false;
                
                var instructions = new Label
                {
                    Text = "Paste coordinates (one per line). Supported formats:\n" +
                           "- lon, lat (e.g., -75.7554276757985, 45.32367641417768)\n" +
                           "- lat, lon (e.g., 45.32367641417768, -75.7554276757985)\n" +
                           "Auto-detects format based on value ranges.",
                    Location = new Point(20, 20),
                    Size = new Size(500, 60),
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(instructions);
                
                var textBox = new TextBox
                {
                    Location = new Point(20, 90),
                    Size = new Size(500, 200),
                    Multiline = true,
                    ScrollBars = ScrollBars.Vertical,
                    BackColor = Color.FromArgb(30, 30, 33),
                    ForeColor = Color.White,
                    Font = new Font("Consolas", 10),
                };
                inputForm.Controls.Add(textBox);
                
                var chkReplace = new CheckBox
                {
                    Text = "Replace existing points (unchecked = append)",
                    Location = new Point(20, 300),
                    ForeColor = Color.White,
                    AutoSize = true,
                    Checked = true,
                };
                inputForm.Controls.Add(chkReplace);
                
                var btnOk = new Button
                {
                    Text = "Import",
                    Location = new Point(330, 330),
                    Size = new Size(90, 30),
                    DialogResult = DialogResult.OK,
                    BackColor = Color.FromArgb(0, 122, 204),
                    ForeColor = Color.White,
                    FlatStyle = FlatStyle.Flat,
                };
                inputForm.Controls.Add(btnOk);
                
                var btnCancel = new Button
                {
                    Text = "Cancel",
                    Location = new Point(430, 330),
                    Size = new Size(90, 30),
                    DialogResult = DialogResult.Cancel,
                    FlatStyle = FlatStyle.Flat,
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(btnCancel);
                
                inputForm.AcceptButton = btnOk;
                inputForm.CancelButton = btnCancel;
                
                if (inputForm.ShowDialog() == DialogResult.OK)
                {
                    var points = ParseCoordinates(textBox.Text);
                    if (points.Count > 0)
                    {
                        if (chkReplace.Checked)
                        {
                            boundary.Vertices.Clear();
                            dgv.Rows.Clear();
                        }
                        
                        foreach (var point in points)
                        {
                            boundary.Vertices.Add(point);
                            dgv.Rows.Add(point.Lat.ToString("F8"), point.Lon.ToString("F8"));
                        }
                        
                        _missionConfig.Save();
                        UpdatePointCounts();
                        AutoDrawBoundariesIfEnabled();
                        CustomMessageBox.Show($"Imported {points.Count} points to {boundaryType} boundary.", "Success");
                    }
                    else
                    {
                        CustomMessageBox.Show("No valid coordinates found.", "Warning");
                    }
                }
            }
        }
        
        private List<GpsPoint> ParseCoordinates(string input)
        {
            var points = new List<GpsPoint>();
            var lines = input.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
            
            foreach (var line in lines)
            {
                var cleanLine = line.Trim();
                if (string.IsNullOrEmpty(cleanLine)) continue;
                
                // Try to parse various formats
                var parts = cleanLine.Split(new[] { ',', '\t', ' ' }, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length >= 2)
                {
                    if (double.TryParse(parts[0].Trim(), out double val1) &&
                        double.TryParse(parts[1].Trim(), out double val2))
                    {
                        double lat, lon;
                        
                        // Auto-detect format:
                        // Longitude typically ranges -180 to 180 (but for Ottawa area ~-75)
                        // Latitude for North America is typically 24 to 70
                        // If abs(val1) > 90, it's likely longitude
                        if (Math.Abs(val1) > 90)
                        {
                            // lon, lat format
                            lon = val1;
                            lat = val2;
                        }
                        else if (Math.Abs(val2) > 90)
                        {
                            // lat, lon format
                            lat = val1;
                            lon = val2;
                        }
                        else
                        {
                            // Both could be valid lat or lon
                            // For Ottawa area (lat ~45, lon ~-75), check for negative values
                            if (val1 < 0)
                            {
                                lon = val1;
                                lat = val2;
                            }
                            else if (val2 < 0)
                            {
                                lat = val1;
                                lon = val2;
                            }
                            else
                            {
                                // Default to lat, lon
                                lat = val1;
                                lon = val2;
                            }
                        }
                        
                        points.Add(new GpsPoint(lat, lon));
                    }
                }
            }
            
            return points;
        }
        
        private void ClearBoundary(DataGridView dgv, FlightBoundary boundary)
        {
            if (CustomMessageBox.Show("Clear all boundary points?", "Confirm",
                CustomMessageBox.MessageBoxButtons.YesNo) == CustomMessageBox.DialogResult.Yes)
            {
                boundary.Vertices.Clear();
                dgv.Rows.Clear();
                _missionConfig.Save();
                UpdatePointCounts();
            }
        }
        
        private void AddManualPoint(DataGridView dgv, FlightBoundary boundary)
        {
            // Use current position or last point
            double lat = MainV2.comPort?.MAV?.cs?.lat ?? 45.0;
            double lon = MainV2.comPort?.MAV?.cs?.lng ?? -75.0;
            
            var point = new GpsPoint(lat, lon);
            boundary.Vertices.Add(point);
            dgv.Rows.Add(lat.ToString("F8"), lon.ToString("F8"));
            _missionConfig.Save();
            UpdatePointCounts();
            AutoDrawBoundariesIfEnabled();
        }
        
        private void AutoDrawBoundariesIfEnabled()
        {
            try
            {
                var chkAutoDraw = this.Controls.Find("chkAutoDraw", true);
                if (chkAutoDraw.Length > 0 && chkAutoDraw[0] is CheckBox chk && chk.Checked)
                {
                    MapOverlayManager.DrawBoundaries(_missionConfig);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Auto-draw error - {ex.Message}");
            }
        }
        
        // ============================================================
        // Import Methods (ported from BoundaryConfigPanel)
        // ============================================================
        
        private void BtnImportKml_Click(object sender, EventArgs e)
        {
            using (var ofd = new OpenFileDialog
            {
                Filter = "KML/KMZ Files|*.kml;*.kmz|All Files|*.*",
                Title = "Import Boundary from KML"
            })
            {
                if (ofd.ShowDialog() == DialogResult.OK)
                {
                    try
                    {
                        string content;
                        
                        // KMZ files are zipped KML - extract the KML content
                        if (ofd.FileName.EndsWith(".kmz", StringComparison.OrdinalIgnoreCase))
                        {
                            using (var zip = System.IO.Compression.ZipFile.OpenRead(ofd.FileName))
                            {
                                var kmlEntry = zip.Entries.FirstOrDefault(e =>
                                    e.Name.EndsWith(".kml", StringComparison.OrdinalIgnoreCase));
                                if (kmlEntry == null)
                                {
                                    CustomMessageBox.Show("No KML file found inside KMZ archive.", "Error");
                                    return;
                                }
                                using (var sr = new StreamReader(kmlEntry.Open()))
                                {
                                    content = sr.ReadToEnd();
                                }
                            }
                        }
                        else
                        {
                            content = File.ReadAllText(ofd.FileName);
                        }
                        
                        var points = ParseKmlCoordinates(content);
                        
                        if (points.Count > 0)
                        {
                            var result = CustomMessageBox.Show(
                                $"Import {points.Count} points as Soft (Yes) or Hard (No) boundary?",
                                "Select Boundary Type",
                                CustomMessageBox.MessageBoxButtons.YesNo);

                            if (result == CustomMessageBox.DialogResult.Yes)
                            {
                                _missionConfig.SoftBoundary.Vertices = points;
                            }
                            else
                            {
                                _missionConfig.HardBoundary.Vertices = points;
                            }
                            _missionConfig.Save();
                            LoadBoundaries();
                            AutoDrawBoundariesIfEnabled();
                            CustomMessageBox.Show($"Imported {points.Count} boundary points from KML.", "Success");
                        }
                        else
                        {
                            CustomMessageBox.Show("No valid coordinates found in KML file.", "Warning");
                        }
                    }
                    catch (Exception ex)
                    {
                        CustomMessageBox.Show($"Error importing KML: {ex.Message}", "Error");
                    }
                }
            }
        }
        
        private List<GpsPoint> ParseKmlCoordinates(string kmlContent)
        {
            var points = new List<GpsPoint>();
            
            var coordsMatch = System.Text.RegularExpressions.Regex.Match(
                kmlContent, @"<coordinates>\s*(.*?)\s*</coordinates>",
                System.Text.RegularExpressions.RegexOptions.Singleline);

            if (coordsMatch.Success)
            {
                var coordString = coordsMatch.Groups[1].Value;
                var coordPairs = coordString.Split(new[] { ' ', '\n', '\r', '\t' },
                    StringSplitOptions.RemoveEmptyEntries);

                foreach (var pair in coordPairs)
                {
                    var parts = pair.Split(',');
                    if (parts.Length >= 2)
                    {
                        if (double.TryParse(parts[0], out double lon) &&
                            double.TryParse(parts[1], out double lat))
                        {
                            points.Add(new GpsPoint(lat, lon));
                        }
                    }
                }
            }

            return points;
        }
        
        private void BtnImportGoogleMaps_Click(object sender, EventArgs e)
        {
            using (var inputForm = new Form())
            {
                inputForm.Width = 500;
                inputForm.Height = 300;
                inputForm.Text = "Import Coordinates";
                inputForm.StartPosition = FormStartPosition.CenterParent;
                inputForm.BackColor = Color.FromArgb(40, 40, 45);
                inputForm.FormBorderStyle = FormBorderStyle.FixedDialog;
                inputForm.MaximizeBox = false;
                inputForm.MinimizeBox = false;
                
                var label = new Label
                {
                    Left = 20, Top = 20, Width = 440,
                    Text = "Paste coordinates (one per line, format: lat,lon or lon,lat):",
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(label);
                
                var textBox = new TextBox
                {
                    Left = 20, Top = 50, Width = 440, Height = 120,
                    Multiline = true, ScrollBars = ScrollBars.Vertical,
                    BackColor = Color.FromArgb(30, 30, 33),
                    ForeColor = Color.White,
                    Font = new Font("Consolas", 10),
                };
                inputForm.Controls.Add(textBox);
                
                var chkReplace = new CheckBox
                {
                    Text = "Replace existing points (unchecked = append)",
                    Location = new Point(20, 180),
                    ForeColor = Color.White,
                    AutoSize = true,
                    Checked = true,
                };
                inputForm.Controls.Add(chkReplace);
                
                var cmbTarget = new ComboBox
                {
                    Location = new Point(20, 210),
                    Size = new Size(200, 25),
                    DropDownStyle = ComboBoxStyle.DropDownList,
                    BackColor = Color.FromArgb(50, 50, 53),
                    ForeColor = Color.White,
                };
                cmbTarget.Items.AddRange(new object[] { "Soft Boundary", "Hard Boundary" });
                cmbTarget.SelectedIndex = 0;
                inputForm.Controls.Add(cmbTarget);
                
                var btnOk = new Button
                {
                    Text = "Import", Left = 300, Width = 80, Top = 240,
                    DialogResult = DialogResult.OK,
                    BackColor = Color.FromArgb(0, 122, 204),
                    ForeColor = Color.White,
                    FlatStyle = FlatStyle.Flat,
                };
                inputForm.Controls.Add(btnOk);
                
                var btnCancel = new Button
                {
                    Text = "Cancel", Left = 390, Width = 80, Top = 240,
                    DialogResult = DialogResult.Cancel,
                    FlatStyle = FlatStyle.Flat,
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(btnCancel);
                
                inputForm.AcceptButton = btnOk;
                inputForm.CancelButton = btnCancel;
                
                if (inputForm.ShowDialog() == DialogResult.OK && !string.IsNullOrWhiteSpace(textBox.Text))
                {
                    var points = ParseCoordinates(textBox.Text);
                    if (points.Count > 0)
                    {
                        var boundary = cmbTarget.SelectedIndex == 0
                            ? _missionConfig.SoftBoundary
                            : _missionConfig.HardBoundary;
                        var dgv = cmbTarget.SelectedIndex == 0
                            ? _dgvSoftBoundary
                            : _dgvHardBoundary;
                        
                        if (chkReplace.Checked)
                        {
                            boundary.Vertices.Clear();
                            dgv.Rows.Clear();
                        }
                        
                        foreach (var point in points)
                        {
                            boundary.Vertices.Add(point);
                            dgv.Rows.Add(point.Lat.ToString("F8"), point.Lon.ToString("F8"));
                        }
                        
                        _missionConfig.Save();
                        UpdatePointCounts();
                        AutoDrawBoundariesIfEnabled();
                        CustomMessageBox.Show($"Imported {points.Count} points.", "Success");
                    }
                    else
                    {
                        CustomMessageBox.Show("No valid coordinates found.", "Warning");
                    }
                }
            }
        }
        
        private void BtnGetFromMP_Click(object sender, EventArgs e)
        {
            try
            {
                var mav = MainV2.comPort?.MAV;
                if (mav == null)
                {
                    CustomMessageBox.Show("Not connected to vehicle.", "Warning");
                    return;
                }
                
                var points = new List<GpsPoint>();
                
                // Try to access fencepoints via reflection (type varies by MP version)
                var fencepointsField = mav.GetType().GetProperty("fencepoints");
                if (fencepointsField != null)
                {
                    var fenceData = fencepointsField.GetValue(mav);
                    if (fenceData != null)
                    {
                        var valuesProperty = fenceData.GetType().GetProperty("Values");
                        if (valuesProperty != null)
                        {
                            var values = valuesProperty.GetValue(fenceData) as System.Collections.IEnumerable;
                            if (values != null)
                            {
                                foreach (var item in values)
                                {
                                    var latProp = item.GetType().GetField("lat");
                                    var lngProp = item.GetType().GetField("lng");
                                    if (latProp != null && lngProp != null)
                                    {
                                        var lat = Convert.ToDouble(latProp.GetValue(item));
                                        var lng = Convert.ToDouble(lngProp.GetValue(item));
                                        if (lat != 0 || lng != 0)
                                            points.Add(new GpsPoint(lat, lng));
                                    }
                                }
                            }
                        }
                    }
                }
                
                if (points.Count > 0)
                {
                    var result = CustomMessageBox.Show(
                        $"Import {points.Count} fence points as Soft (Yes) or Hard (No) boundary?",
                        "Select Boundary Type",
                        CustomMessageBox.MessageBoxButtons.YesNo);

                    if (result == CustomMessageBox.DialogResult.Yes)
                    {
                        _missionConfig.SoftBoundary.Vertices = points;
                    }
                    else
                    {
                        _missionConfig.HardBoundary.Vertices = points;
                    }
                    _missionConfig.Save();
                    LoadBoundaries();
                    AutoDrawBoundariesIfEnabled();
                    CustomMessageBox.Show($"Imported {points.Count} fence points.", "Success");
                }
                else
                {
                    CustomMessageBox.Show("No fence points found in Mission Planner.", "Warning");
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error getting fence: {ex.Message}", "Error");
            }
        }
        
        private List<GpsPoint> GetSelectedBoundaryVertices(out string boundaryName)
        {
            var soft = _missionConfig.SoftBoundary?.Vertices;
            var hard = _missionConfig.HardBoundary?.Vertices;
            bool hasSoft = soft != null && soft.Count > 0;
            bool hasHard = hard != null && hard.Count > 0;

            if (!hasSoft && !hasHard)
            {
                boundaryName = null;
                return null;
            }

            if (hasSoft && hasHard)
            {
                var result = CustomMessageBox.Show(
                    "Export Soft boundary (Yes) or Hard boundary (No)?",
                    "Select Boundary",
                    CustomMessageBox.MessageBoxButtons.YesNo);
                if (result == CustomMessageBox.DialogResult.Yes)
                {
                    boundaryName = "Soft";
                    return soft;
                }
                boundaryName = "Hard";
                return hard;
            }

            if (hasSoft) { boundaryName = "Soft"; return soft; }
            boundaryName = "Hard";
            return hard;
        }

        private void BtnExportToMPFence_Click(object sender, EventArgs e)
        {
            try
            {
                var vertices = GetSelectedBoundaryVertices(out string boundaryName);
                if (vertices == null || vertices.Count < 3)
                {
                    CustomMessageBox.Show("No boundary points to export (need at least 3).", "Warning");
                    return;
                }

                // Return point: use saved or compute centroid
                double cLat, cLon;
                if (_missionConfig.ReturnPoint != null)
                {
                    cLat = _missionConfig.ReturnPoint.Lat;
                    cLon = _missionConfig.ReturnPoint.Lon;
                }
                else
                {
                    cLat = 0; cLon = 0;
                    foreach (var v in vertices) { cLat += v.Lat; cLon += v.Lon; }
                    cLat /= vertices.Count; cLon /= vertices.Count;
                }

                // Save as temp .fen file in MP's fence directory
                var mpDir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
                    "Mission Planner");
                if (!Directory.Exists(mpDir))
                    mpDir = Path.GetTempPath();

                var fenPath = Path.Combine(mpDir, "nomad_fence.fen");
                using (var writer = new StreamWriter(fenPath))
                {
                    writer.WriteLine("#saved by NOMAD Plugin");
                    writer.WriteLine($"{cLat:F13} {cLon:F13}");
                    foreach (var v in vertices)
                        writer.WriteLine($"{v.Lat:F7} {v.Lon:F7}");
                    writer.WriteLine($"{vertices[0].Lat:F7} {vertices[0].Lon:F7}");
                }

                // Try to trigger MP's fence load via FlightPlanner reflection
                bool loaded = false;
                try
                {
                    var fpType = AppDomain.CurrentDomain.GetAssemblies()
                        .SelectMany(a => { try { return a.GetTypes(); } catch { return new Type[0]; } })
                        .FirstOrDefault(t => t.FullName == "MissionPlanner.GCSViews.FlightPlanner");

                    if (fpType != null)
                    {
                        var fpInstance = fpType.GetProperty("instance",
                            System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static |
                            System.Reflection.BindingFlags.NonPublic)?.GetValue(null)
                            ?? fpType.GetField("instance",
                            System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static |
                            System.Reflection.BindingFlags.NonPublic)?.GetValue(null);

                        if (fpInstance != null)
                        {
                            // Try calling loadFenceFromFile or similar method
                            foreach (var methodName in new[] { "loadFenceFromFile", "LoadFenceFromFile", "BUT_loadfence_Click" })
                            {
                                var method = fpType.GetMethod(methodName,
                                    System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance |
                                    System.Reflection.BindingFlags.NonPublic);
                                if (method != null)
                                {
                                    var parms = method.GetParameters();
                                    if (parms.Length == 1 && parms[0].ParameterType == typeof(string))
                                    {
                                        method.Invoke(fpInstance, new object[] { fenPath });
                                        loaded = true;
                                        break;
                                    }
                                    else if (parms.Length == 0)
                                    {
                                        // Some MP versions take no args - they prompt a file dialog
                                        // Skip these
                                    }
                                }
                            }
                        }
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"NOMAD: Could not auto-load fence: {ex.Message}");
                }

                // Also draw on FlightData map via NOMAD overlay
                bool isSoft = boundaryName == "Soft";
                var strokeColor = isSoft ? Color.Yellow : Color.Red;
                var fillColor = isSoft ? Color.FromArgb(60, Color.Yellow) : Color.FromArgb(60, Color.Red);
                string polyName = $"NOMAD_{boundaryName}_Fence";

                try
                {
                    MapOverlayManager.DrawPolygon(vertices, polyName, strokeColor, fillColor, isSoft ? 2 : 3);
                    MapOverlayManager.RefreshMap();
                }
                catch { }

                // Also try geofence overlay injection
                try { MapOverlayManager.ExportToMPGeoFence(vertices, polyName, strokeColor, fillColor, isSoft ? 2 : 3); } catch { }

                if (loaded)
                {
                    CustomMessageBox.Show(
                        $"Loaded {vertices.Count} {boundaryName} boundary points as MP fence.\nFence should be visible on Plan + Data maps.",
                        "Success");
                }
                else
                {
                    CustomMessageBox.Show(
                        $"Fence file saved to:\n{fenPath}\n\nTo load in Mission Planner:\nPlan view > Right-click map > Geo-Fence > Load from File\n\nBoundary also drawn on Data map overlay.",
                        "Fence Exported");
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error exporting fence: {ex.Message}", "Error");
            }
        }

        private void BtnExportPolyFile_Click(object sender, EventArgs e)
        {
            try
            {
                var vertices = GetSelectedBoundaryVertices(out string boundaryName);
                if (vertices == null || vertices.Count < 3)
                {
                    CustomMessageBox.Show("No boundary points to export (need at least 3).", "Warning");
                    return;
                }

                using (var sfd = new SaveFileDialog())
                {
                    sfd.Title = $"Save {boundaryName} Boundary as Fence File";
                    sfd.Filter = "Fence Files (*.fen)|*.fen|Poly Files (*.poly)|*.poly|All Files (*.*)|*.*";
                    sfd.FileName = $"NOMAD_{boundaryName}_Fence.fen";
                    sfd.DefaultExt = "fen";

                    if (sfd.ShowDialog() == DialogResult.OK)
                    {
                        // Return point: use saved return point or compute centroid
                        double cLat, cLon;
                        if (_missionConfig.ReturnPoint != null)
                        {
                            cLat = _missionConfig.ReturnPoint.Lat;
                            cLon = _missionConfig.ReturnPoint.Lon;
                        }
                        else
                        {
                            cLat = 0; cLon = 0;
                            foreach (var v in vertices) { cLat += v.Lat; cLon += v.Lon; }
                            cLat /= vertices.Count;
                            cLon /= vertices.Count;
                        }

                        using (var writer = new StreamWriter(sfd.FileName))
                        {
                            writer.WriteLine("#saved by NOMAD Plugin");
                            // Return point (centroid) - required first line
                            writer.WriteLine($"{cLat:F13} {cLon:F13}");
                            // Polygon vertices
                            foreach (var v in vertices)
                            {
                                writer.WriteLine($"{v.Lat:F7} {v.Lon:F7}");
                            }
                            // Close polygon (repeat first vertex)
                            writer.WriteLine($"{vertices[0].Lat:F7} {vertices[0].Lon:F7}");
                        }

                        CustomMessageBox.Show(
                            $"Saved {vertices.Count} fence points to:\n{sfd.FileName}\n\nTo load in Mission Planner:\nPlan view → Right-click → Geo-Fence → Load from File",
                            "Fence File Saved");
                    }
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error saving fence file: {ex.Message}", "Error");
            }
        }

        private void SaveBoundaryFromGrid(DataGridView dgv, FlightBoundary boundary)
        {
            try
            {
                boundary.Vertices.Clear();
                foreach (DataGridViewRow row in dgv.Rows)
                {
                    if (row.Cells["Lat"].Value != null && row.Cells["Lon"].Value != null)
                    {
                        if (double.TryParse(row.Cells["Lat"].Value.ToString(), out double lat) &&
                            double.TryParse(row.Cells["Lon"].Value.ToString(), out double lon))
                        {
                            boundary.Vertices.Add(new GpsPoint(lat, lon));
                        }
                    }
                }
                _missionConfig.Save();
                UpdatePointCounts();
            }
            catch { }
        }
        
        private void SaveBuildingLocation(object sender, EventArgs e)
        {
            try
            {
                if (double.TryParse(_txtBuildingLat.Text, out double lat) &&
                    double.TryParse(_txtBuildingLon.Text, out double lon))
                {
                    var building = _missionConfig.CurrentTask == 1 
                        ? _missionConfig.Task1Building 
                        : _missionConfig.Task2Building;
                    
                    if (building == null)
                    {
                        building = new BuildingInfo();
                        if (_missionConfig.CurrentTask == 1)
                            _missionConfig.Task1Building = building;
                        else
                            _missionConfig.Task2Building = building;
                    }
                    
                    building.Coordinates = new GpsPoint(lat, lon);
                    _missionConfig.Save();
                    CustomMessageBox.Show("Building location saved.", "Success");
                }
                else
                {
                    CustomMessageBox.Show("Invalid coordinates.", "Error");
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error saving building: {ex.Message}", "Error");
            }
        }
        
        private void ShowBuildingOnMap(object sender, EventArgs e)
        {
            try
            {
                if (double.TryParse(_txtBuildingLat.Text, out double lat) &&
                    double.TryParse(_txtBuildingLon.Text, out double lon))
                {
                    CustomMessageBox.Show($"Building Location (Task {_missionConfig.CurrentTask}):\n" +
                        $"Latitude: {lat:F8}\n" +
                        $"Longitude: {lon:F8}\n\n" +
                        "Use Mission Planner's map to add a marker at this location.", "Building Location");
                }
                else
                {
                    CustomMessageBox.Show("Invalid coordinates.", "Error");
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error: {ex.Message}", "Error");
            }
        }
        
        // ============================================================
        // Preset Management
        // ============================================================
        
        private void LoadPresets()
        {
            _presets.Clear();
            try
            {
                if (Directory.Exists(PresetsDir))
                {
                    foreach (var file in Directory.GetFiles(PresetsDir, "*.json"))
                    {
                        var json = File.ReadAllText(file);
                        var preset = JsonConvert.DeserializeObject<BoundaryPreset>(json);
                        if (preset != null)
                            _presets.Add(preset);
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Error loading presets - {ex.Message}");
            }
        }
        
        private void RefreshPresetCombo()
        {
            _cmbPresets?.Items.Clear();
            _cmbPresets?.Items.Add("-- Select Preset --");
            foreach (var preset in _presets.OrderByDescending(p => p.CreatedAt))
            {
                _cmbPresets?.Items.Add($"{preset.Name} (Task {preset.TaskNumber})");
            }
            if (_cmbPresets != null)
                _cmbPresets.SelectedIndex = 0;
        }
        
        private void LoadSelectedPreset(object sender, EventArgs e)
        {
            if (_cmbPresets.SelectedIndex <= 0) return;
            
            var preset = _presets[_cmbPresets.SelectedIndex - 1];
            
            if (CustomMessageBox.Show($"Load preset '{preset.Name}'?\nThis will replace current boundaries.",
                "Confirm", CustomMessageBox.MessageBoxButtons.YesNo) == CustomMessageBox.DialogResult.Yes)
            {
                _missionConfig.SoftBoundary.Vertices = preset.SoftBoundary.ToList();
                _missionConfig.HardBoundary.Vertices = preset.HardBoundary.ToList();
                _missionConfig.MaxAltitudeAglMeters = preset.MaxAltitudeMeters;
                
                if (preset.BuildingLocation != null)
                {
                    var building = _missionConfig.CurrentTask == 1 
                        ? _missionConfig.Task1Building 
                        : _missionConfig.Task2Building;
                    if (building == null)
                    {
                        building = new BuildingInfo();
                        if (_missionConfig.CurrentTask == 1)
                            _missionConfig.Task1Building = building;
                        else
                            _missionConfig.Task2Building = building;
                    }
                    building.Coordinates = preset.BuildingLocation;
                }
                
                _missionConfig.Save();
                LoadBoundaries();
                _nudMaxAlt.Value = (decimal)preset.MaxAltitudeMeters;
                
                CustomMessageBox.Show($"Preset '{preset.Name}' loaded.", "Success");
            }
        }
        
        private void SaveCurrentAsPreset(object sender, EventArgs e)
        {
            using (var inputForm = new Form())
            {
                inputForm.Width = 400;
                inputForm.Height = 200;
                inputForm.Text = "Save Boundary Preset";
                inputForm.StartPosition = FormStartPosition.CenterParent;
                inputForm.BackColor = Color.FromArgb(40, 40, 45);
                inputForm.FormBorderStyle = FormBorderStyle.FixedDialog;
                
                var lblName = new Label { Text = "Preset Name:", Location = new Point(20, 20), ForeColor = Color.White, AutoSize = true };
                inputForm.Controls.Add(lblName);
                
                var txtName = new TextBox
                {
                    Location = new Point(20, 45),
                    Size = new Size(340, 25),
                    BackColor = Color.FromArgb(50, 50, 53),
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(txtName);
                
                var lblDesc = new Label { Text = "Description:", Location = new Point(20, 75), ForeColor = Color.White, AutoSize = true };
                inputForm.Controls.Add(lblDesc);
                
                var txtDesc = new TextBox
                {
                    Location = new Point(20, 100),
                    Size = new Size(340, 25),
                    BackColor = Color.FromArgb(50, 50, 53),
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(txtDesc);
                
                var btnOk = new Button
                {
                    Text = "Save",
                    Location = new Point(180, 135),
                    Size = new Size(80, 30),
                    DialogResult = DialogResult.OK,
                    BackColor = Color.FromArgb(0, 122, 204),
                    ForeColor = Color.White,
                    FlatStyle = FlatStyle.Flat,
                };
                inputForm.Controls.Add(btnOk);
                
                var btnCancel = new Button
                {
                    Text = "Cancel",
                    Location = new Point(270, 135),
                    Size = new Size(80, 30),
                    DialogResult = DialogResult.Cancel,
                    FlatStyle = FlatStyle.Flat,
                    ForeColor = Color.White,
                };
                inputForm.Controls.Add(btnCancel);
                
                if (inputForm.ShowDialog() == DialogResult.OK && !string.IsNullOrWhiteSpace(txtName.Text))
                {
                    var building = _missionConfig.CurrentTask == 1 
                        ? _missionConfig.Task1Building 
                        : _missionConfig.Task2Building;
                    
                    var preset = new BoundaryPreset
                    {
                        Name = txtName.Text,
                        Description = txtDesc.Text,
                        TaskNumber = _missionConfig.CurrentTask,
                        CreatedAt = DateTime.Now,
                        SoftBoundary = _missionConfig.SoftBoundary.Vertices.ToList(),
                        HardBoundary = _missionConfig.HardBoundary.Vertices.ToList(),
                        MaxAltitudeMeters = _missionConfig.MaxAltitudeAglMeters,
                        BuildingLocation = building?.Coordinates,
                    };
                    
                    try
                    {
                        if (!Directory.Exists(PresetsDir))
                            Directory.CreateDirectory(PresetsDir);
                        
                        var fileName = $"{txtName.Text.Replace(" ", "_")}_{DateTime.Now:yyyyMMdd_HHmmss}.json";
                        var filePath = Path.Combine(PresetsDir, fileName);
                        var json = JsonConvert.SerializeObject(preset, Formatting.Indented);
                        File.WriteAllText(filePath, json);
                        
                        _presets.Add(preset);
                        RefreshPresetCombo();
                        
                        CustomMessageBox.Show($"Preset '{preset.Name}' saved.", "Success");
                    }
                    catch (Exception ex)
                    {
                        CustomMessageBox.Show($"Error saving preset: {ex.Message}", "Error");
                    }
                }
            }
        }
        
        private void DeleteSelectedPreset(object sender, EventArgs e)
        {
            if (_cmbPresets.SelectedIndex <= 0) return;
            
            var preset = _presets[_cmbPresets.SelectedIndex - 1];
            
            if (CustomMessageBox.Show($"Delete preset '{preset.Name}'?", "Confirm",
                CustomMessageBox.MessageBoxButtons.YesNo) == CustomMessageBox.DialogResult.Yes)
            {
                try
                {
                    // Find and delete file
                    var files = Directory.GetFiles(PresetsDir, "*.json");
                    foreach (var file in files)
                    {
                        var json = File.ReadAllText(file);
                        var p = JsonConvert.DeserializeObject<BoundaryPreset>(json);
                        if (p?.Name == preset.Name && p?.CreatedAt == preset.CreatedAt)
                        {
                            File.Delete(file);
                            break;
                        }
                    }
                    
                    _presets.Remove(preset);
                    RefreshPresetCombo();
                    CustomMessageBox.Show("Preset deleted.", "Success");
                }
                catch (Exception ex)
                {
                    CustomMessageBox.Show($"Error deleting preset: {ex.Message}", "Error");
                }
            }
        }
        
        // ============================================================
        // Monitor Events
        // ============================================================
        
        private void Monitor_BoundaryStatusChanged(object sender, BoundaryStatusEventArgs e)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => Monitor_BoundaryStatusChanged(sender, e)));
                return;
            }
            
            switch (e.Status)
            {
                case "inside":
                    _statusPanel.BackColor = Color.FromArgb(40, 100, 40);
                    _lblStatus.Text = "[OK] Inside Boundaries";
                    _lblCountdown.Visible = false;
                    break;
                    
                case "soft_violation":
                    _statusPanel.BackColor = Color.FromArgb(180, 150, 0);
                    _lblStatus.Text = "[!] SOFT BOUNDARY - Turn Around!";
                    _lblCountdown.Visible = false;
                    break;
                    
                case "hard_violation":
                    _statusPanel.BackColor = Color.FromArgb(180, 40, 40);
                    _lblStatus.Text = "[!!] HARD BOUNDARY VIOLATION!";
                    _lblCountdown.Visible = true;
                    break;

                case "no_position":
                    _statusPanel.BackColor = Color.FromArgb(80, 80, 90);
                    _lblStatus.Text = "[?] Waiting for GPS Position";
                    _lblCountdown.Visible = false;
                    break;
            }
        }
        
        private void Monitor_BoundaryViolation(object sender, BoundaryViolationEventArgs e)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => Monitor_BoundaryViolation(sender, e)));
                return;
            }
            
            if (e.BoundaryType == "hard" && _monitor?.KillCountdown != null)
            {
                _lblCountdown.Text = $"KILL IN {_monitor.KillCountdown} SECONDS!";
            }
        }
        
        public void UpdateData()
        {
            if (InvokeRequired)
            {
                BeginInvoke((MethodInvoker)UpdateData);
                return;
            }
            
            try
            {
                var cs = MainV2.comPort?.MAV?.cs;
                if (cs != null)
                {
                    _lblPosition.Text = $"Position: {cs.lat:F6}, {cs.lng:F6}";
                    _lblAltitude.Text = $"Alt: {cs.alt:F1}m / {_missionConfig.MaxAltitudeAglMeters:F0}m";
                    
                    // Color altitude warning
                    if (cs.alt > _missionConfig.MaxAltitudeAglMeters * 0.9)
                        _lblAltitude.ForeColor = Color.Red;
                    else if (cs.alt > _missionConfig.MaxAltitudeAglMeters * 0.8)
                        _lblAltitude.ForeColor = Color.Yellow;
                    else
                        _lblAltitude.ForeColor = Color.White;
                }
            }
            catch { }
        }
    }
}
