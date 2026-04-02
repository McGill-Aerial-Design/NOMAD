// ============================================================
// NOMAD Task 1 View - Outdoor Reconnaissance
// ============================================================

using System;
using System.Drawing;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    public class NOMADTask1View : NOMADViewBase, IUpdatableView
    {
        private readonly DualLinkSender _sender;
        private readonly NOMADConfig _config;
        private readonly MissionConfig _missionConfig;
        private readonly JetsonConnectionManager _jetsonConnectionManager;
        private Label _lblPosition;
        private Label _lblGpsStatus;
        private Label _lblBuildingStatus;
        private TextBox _txtBuildingLat;
        private TextBox _txtBuildingLon;
        private Button _btnBuildingSave;
        private Button _btnBuildingUseCurrent;
        private Button _btnCapture;
        private TextBox _txtResult;
        private EmbeddedVideoPlayer _videoPlayer;
        private FlowLayoutPanel _galleryPanel;
        private PayloadControlPanel _payloadControl;
        private Task1UploadPanel _uploadPanel;
        private TabControl _tabControl;
        
        public NOMADTask1View(
            DualLinkSender sender,
            NOMADConfig config,
            MissionConfig missionConfig = null,
            JetsonConnectionManager jetsonConnectionManager = null)
        {
            _sender = sender;
            _config = config;
            _missionConfig = missionConfig ?? MissionConfig.Load();
            _jetsonConnectionManager = jetsonConnectionManager;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            // Main layout: Video (left) | TabControl for Capture/Submit (right)
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

            // ========== LEFT COLUMN: Video ==========
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

            // ========== RIGHT COLUMN: TabControl with Capture/Submit/Configuration tabs ==========
            _tabControl = new TabControl
            {
                Dock = DockStyle.Fill,
                Margin = new Padding(5),
            };
            StyleTabControl(_tabControl);

            // --- Tab 1: Capture ---
            var captureTab = new TabPage("Capture")
            {
                BackColor = CARD_BG,
                Padding = new Padding(0),
            };
            captureTab.Controls.Add(CreateCapturePanel());
            _tabControl.TabPages.Add(captureTab);

            // --- Tab 2: Submit to Google Drive ---
            var submitTab = new TabPage("Submit")
            {
                BackColor = CARD_BG,
                Padding = new Padding(0),
            };
            _uploadPanel = new Task1UploadPanel(_config);
            _uploadPanel.Dock = DockStyle.Fill;
            submitTab.Controls.Add(_uploadPanel);
            _tabControl.TabPages.Add(submitTab);

            // --- Tab 3: Configuration ---
            var configTab = new TabPage("Configuration")
            {
                BackColor = CARD_BG,
                Padding = new Padding(0),
            };
            configTab.Controls.Add(CreateConfigurationSubtab());
            _tabControl.TabPages.Add(configTab);

            LoadBuildingLocationFields();

            mainLayout.Controls.Add(_tabControl, 1, 0);

            this.AutoScroll = false;
            this.Controls.Add(mainLayout);
        }

        private void StyleTabControl(TabControl tabControl)
        {
            tabControl.DrawMode = TabDrawMode.OwnerDrawFixed;
            tabControl.SizeMode = TabSizeMode.Fixed;
            tabControl.ItemSize = new Size(100, 30);
            tabControl.DrawItem += (s, e) =>
            {
                var tab = tabControl.TabPages[e.Index];
                var isSelected = tabControl.SelectedIndex == e.Index;
                var bgColor = isSelected ? ACCENT_COLOR : Color.FromArgb(45, 45, 48);
                var textColor = Color.White;

                using (var bgBrush = new SolidBrush(bgColor))
                using (var textBrush = new SolidBrush(textColor))
                {
                    e.Graphics.FillRectangle(bgBrush, e.Bounds);
                    var sf = new StringFormat
                    {
                        Alignment = StringAlignment.Center,
                        LineAlignment = StringAlignment.Center,
                    };
                    e.Graphics.DrawString(tab.Text, new Font("Segoe UI", 9, FontStyle.Bold), textBrush, e.Bounds, sf);
                }
            };
        }

        private Panel CreateCapturePanel()
        {
            var capturePanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
            };

            capturePanel.Controls.Add(CreateMainControlsSubtab());
            return capturePanel;
        }

        private Panel CreateMainControlsSubtab()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
            };

            var layout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 4,
                Margin = Padding.Empty,
                Padding = Padding.Empty,
            };
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 90));   // GPS status
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 105));  // Payload controls
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 45));    // Capture
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 55));    // Gallery

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

            layout.Controls.Add(gpsCard, 0, 0);

            // --- Payload Controls ---
            _payloadControl = new PayloadControlPanel(_config);
            _payloadControl.Dock = DockStyle.Fill;
            _payloadControl.Margin = new Padding(5);
            layout.Controls.Add(_payloadControl, 0, 1);

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

            captureCard.Resize += (s, e) =>
            {
                _txtResult.Width = captureCard.ClientSize.Width - 30;
                _txtResult.Height = captureCard.ClientSize.Height - 110;
            };

            layout.Controls.Add(captureCard, 0, 2);

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

            galleryCard.Resize += (s, e) =>
            {
                _galleryPanel.Width = galleryCard.ClientSize.Width - 30;
                _galleryPanel.Height = galleryCard.ClientSize.Height - 60;
            };

            layout.Controls.Add(galleryCard, 0, 3);

            panel.Controls.Add(layout);
            return panel;
        }

        private Panel CreateConfigurationSubtab()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                Padding = new Padding(12),
            };

            // --- Building Location ---
            var buildingCard = CreateCard("BUILDING LOCATION");
            buildingCard.Dock = DockStyle.Top;
            buildingCard.Height = 230;

            _lblBuildingStatus = new Label
            {
                Text = GetTask1BuildingLocationText() ?? "Not set",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(16, 34),
                AutoSize = true,
            };
            buildingCard.Controls.Add(_lblBuildingStatus);

            var lblBuildingLat = new Label
            {
                Text = "Lat:",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(16, 70),
                AutoSize = true,
            };
            buildingCard.Controls.Add(lblBuildingLat);

            _txtBuildingLat = new TextBox
            {
                Location = new Point(16, 88),
                Size = new Size(280, 28),
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
                BorderStyle = BorderStyle.FixedSingle,
                Font = new Font("Segoe UI", 9),
            };
            buildingCard.Controls.Add(_txtBuildingLat);

            var lblBuildingLon = new Label
            {
                Text = "Lon:",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(16, 124),
                AutoSize = true,
            };
            buildingCard.Controls.Add(lblBuildingLon);

            _txtBuildingLon = new TextBox
            {
                Location = new Point(16, 142),
                Size = new Size(280, 28),
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
                BorderStyle = BorderStyle.FixedSingle,
                Font = new Font("Segoe UI", 9),
            };
            buildingCard.Controls.Add(_txtBuildingLon);

            _btnBuildingUseCurrent = CreateButton("Use Current", ACCENT_COLOR, 120, 30);
            _btnBuildingUseCurrent.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            _btnBuildingUseCurrent.Location = new Point(16, 180);
            _btnBuildingUseCurrent.Click += (s, e) => UseCurrentGpsForBuildingLocation();
            buildingCard.Controls.Add(_btnBuildingUseCurrent);

            _btnBuildingSave = CreateButton("Save", SUCCESS_COLOR, 90, 30);
            _btnBuildingSave.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            _btnBuildingSave.Location = new Point(146, 180);
            _btnBuildingSave.Click += (s, e) => SaveBuildingLocationFromFields();
            buildingCard.Controls.Add(_btnBuildingSave);

            var btnBuildingClear = CreateButton("Clear", ERROR_COLOR, 90, 30);
            btnBuildingClear.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            btnBuildingClear.Location = new Point(246, 180);
            btnBuildingClear.Click += (s, e) => ClearBuildingLocationFields();
            buildingCard.Controls.Add(btnBuildingClear);

            buildingCard.Resize += (s, e) =>
            {
                int inputWidth = Math.Max(260, buildingCard.ClientSize.Width - 32);
                _txtBuildingLat.Width = inputWidth;
                _txtBuildingLon.Width = inputWidth;
            };

            panel.Controls.Add(buildingCard);
            return panel;
        }

        private string GetTask1BuildingLocationText()
        {
            var coords = _missionConfig?.Task1Building?.Coordinates;
            if (coords != null)
            {
                return $"{coords.Lat:F6}, {coords.Lon:F6}";
            }

            return null;
        }

        private void LoadBuildingLocationFields()
        {
            if (_txtBuildingLat != null)
                _txtBuildingLat.Text = _missionConfig?.Task1Building?.Coordinates?.Lat.ToString("F6") ?? string.Empty;
            if (_txtBuildingLon != null)
                _txtBuildingLon.Text = _missionConfig?.Task1Building?.Coordinates?.Lon.ToString("F6") ?? string.Empty;
            UpdateBuildingLocationStatus();
        }

        private void UpdateBuildingLocationStatus(string prefix = null)
        {
            if (_lblBuildingStatus == null)
                return;

            var text = GetTask1BuildingLocationText() ?? "Not set";
            _lblBuildingStatus.Text = string.IsNullOrWhiteSpace(prefix) ? text : $"{prefix}: {text}";
        }

        private void SaveBuildingLocationFromFields()
        {
            if (!double.TryParse(_txtBuildingLat?.Text, out var lat) ||
                !double.TryParse(_txtBuildingLon?.Text, out var lon))
            {
                _txtResult.Text = "[FAIL] Invalid building GPS coordinates.";
                _txtResult.ForeColor = ERROR_COLOR;
                return;
            }

            var building = _missionConfig.Task1Building ?? new BuildingInfo();
            building.Coordinates = new GpsPoint(lat, lon);
            _missionConfig.Task1Building = building;
            _missionConfig.Save();
            UpdateBuildingLocationStatus("Saved");
            _txtResult.Text = $"[OK] Building location saved: {lat:F6}, {lon:F6}";
            _txtResult.ForeColor = SUCCESS_COLOR;
        }

        private void UseCurrentGpsForBuildingLocation()
        {
            var cs = MainV2.comPort?.MAV?.cs;
            if (cs == null || (Math.Abs(cs.lat) < 0.000001 && Math.Abs(cs.lng) < 0.000001))
            {
                _txtResult.Text = "[FAIL] No GPS position available.";
                _txtResult.ForeColor = ERROR_COLOR;
                return;
            }

            _txtBuildingLat.Text = cs.lat.ToString("F6");
            _txtBuildingLon.Text = cs.lng.ToString("F6");
            SaveBuildingLocationFromFields();
        }

        private void ClearBuildingLocationFields()
        {
            _missionConfig.Task1Building = new BuildingInfo();
            _missionConfig.Save();
            if (_txtBuildingLat != null) _txtBuildingLat.Text = string.Empty;
            if (_txtBuildingLon != null) _txtBuildingLon.Text = string.Empty;
            UpdateBuildingLocationStatus();
            _txtResult.Text = "[OK] Building location cleared.";
            _txtResult.ForeColor = SUCCESS_COLOR;
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

                            // Helper: JValue(null).ToString() returns "" not null, so ?? doesn't catch it
                            string Val(string key) {
                                var t = data[key];
                                return t != null && t.Type != Newtonsoft.Json.Linq.JTokenType.Null ? t.ToString() : null;
                            }

                            // Check API-level success (distinct from HTTP success)
                            var apiSuccess = (bool?)data["success"] ?? false;
                            if (!apiSuccess)
                            {
                                var error = Val("error") ?? "Unknown error";
                                _txtResult.Text = $"[FAIL] {error}";
                                _txtResult.ForeColor = ERROR_COLOR;
                                return;
                            }

                            var imageName = Val("image_name");

                            if (string.IsNullOrEmpty(imageName))
                            {
                                var output = Val("output") ?? Val("message") ?? Val("detail");
                                var captureSummary = new StringBuilder();
                                captureSummary.AppendLine("[OK] Capture Successful");
                                if (!string.IsNullOrWhiteSpace(output))
                                {
                                    captureSummary.AppendLine(output);
                                }
                                else
                                {
                                    captureSummary.AppendLine("Target localization completed, but no image metadata was returned by the API.");
                                }

                                _txtResult.Text = captureSummary.ToString().TrimEnd();
                                return;
                            }

                            // Extract enhanced metadata
                            var timestamp = Val("timestamp") ?? DateTime.Now.ToString("yyyy-MM-ddTHH:mm:ssZ");
                            var headingDeg = Val("heading_deg") ?? "N/A";
                            var pitchDeg = Val("pitch_deg") ?? "N/A";
                            var rollDeg = Val("roll_deg") ?? "N/A";
                            var gimbalPitch = Val("gimbal_pitch_deg") ?? "N/A";
                            var gimbalYaw = Val("gimbal_yaw_deg") ?? "N/A";
                            var buildingLocation = GetTask1BuildingLocationText() ?? Val("building_location") ?? "N/A";
                            var captureFolder = Val("capture_folder") ?? "N/A";

                            var position = data["position"] as Newtonsoft.Json.Linq.JObject;
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
                                // Construct download URL (use folder-based endpoint)
                                var folderName = Path.GetFileName(captureFolder.TrimEnd('/'));
                                string imageUrl = $"http://{_config.EffectiveIP}:{_config.JetsonPort}/api/task/1/images/{folderName}/{imageName}";
                                
                                // Download image
                                var imageBytes = await JetsonApiService.GetByteArrayAsync(imageUrl);
                                
                                // Save to local directory (use folder name to avoid overwriting)
                                var task1Dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "NOMAD", "Task1");
                                Directory.CreateDirectory(task1Dir);
                                var localImageName = $"{folderName}_{imageName}";
                                var localPath = Path.Combine(task1Dir, localImageName);
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
                                        Tag = new { Path = localPath, JsonPath = jsonPath, Metadata = metadataText.ToString() },
                                    };
                                    
                                    picBox.MouseClick += (s, mevt) =>
                                    {
                                        ShowImageContextMenu(picBox, mevt.Location);
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
        
        /// <summary>
        /// Show a context menu on thumbnail click with options to open the image
        /// or view the AI-generated description.
        /// </summary>
        private void ShowImageContextMenu(PictureBox picBox, Point location)
        {
            dynamic tagData = picBox.Tag;
            string imagePath = tagData.Path;
            string jsonPath = tagData.JsonPath;

            var menu = new ContextMenuStrip();
            menu.BackColor = Color.FromArgb(35, 35, 38);
            menu.ForeColor = Color.White;
            menu.Renderer = new ToolStripProfessionalRenderer(new DarkMenuColorTable());

            var openItem = new ToolStripMenuItem("Open Image in Viewer");
            openItem.Click += (s, e) =>
            {
                try { System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo(imagePath) { UseShellExecute = true }); }
                catch (Exception ex) { MessageBox.Show($"Failed to open image: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error); }
            };
            menu.Items.Add(openItem);

            var descItem = new ToolStripMenuItem("View AI Description");
            descItem.Click += (s, e) => ShowAIDescriptionPopup(jsonPath, imagePath);
            menu.Items.Add(descItem);

            menu.Show(picBox, location);
        }

        /// <summary>
        /// Show a modal popup displaying the AI description for a captured image,
        /// with the ability to copy the text. Polls the JSON metadata file every
        /// 2 seconds so the description auto-appears when AI processing finishes.
        /// </summary>
        private void ShowAIDescriptionPopup(string jsonPath, string imagePath)
        {
            string currentDescription = null;

            // Helper: read AI fields from metadata JSON
            void ReadMetadata(out string desc, out string provider, out string model, out string generatedAt)
            {
                desc = null; provider = null; model = null; generatedAt = null;
                try
                {
                    if (File.Exists(jsonPath))
                    {
                        var json = File.ReadAllText(jsonPath);
                        var data = Newtonsoft.Json.Linq.JObject.Parse(json);
                        desc = data["ai_description"]?.ToString();
                        provider = data["ai_provider"]?.ToString();
                        model = data["ai_model"]?.ToString();
                        generatedAt = data["ai_generated_at"]?.ToString();
                    }
                }
                catch { }
            }

            ReadMetadata(out var aiDescription, out var aiProvider, out var aiModel, out var aiGeneratedAt);
            currentDescription = aiDescription;

            var popup = new Form
            {
                Text = "AI Description - " + Path.GetFileName(imagePath),
                Size = new Size(680, 520),
                MinimumSize = new Size(500, 380),
                StartPosition = FormStartPosition.CenterParent,
                BackColor = Color.FromArgb(30, 30, 33),
                ForeColor = Color.White,
                FormBorderStyle = FormBorderStyle.Sizable,
                MaximizeBox = false,
                MinimizeBox = false,
                ShowInTaskbar = false,
            };

            var layout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
                Padding = new Padding(20),
            };
            layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));

            var header = new Label
            {
                Text = !string.IsNullOrEmpty(aiProvider)
                    ? $"Provider: {aiProvider}  |  Model: {aiModel ?? "N/A"}  |  Generated: {aiGeneratedAt ?? "N/A"}"
                    : "Waiting for AI description...",
                AutoSize = true,
                ForeColor = Color.FromArgb(180, 180, 180),
                Padding = new Padding(4, 4, 4, 8),
            };
            layout.Controls.Add(header, 0, 0);

            var txtDesc = new RichTextBox
            {
                Dock = DockStyle.Fill,
                ReadOnly = true,
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = Color.FromArgb(220, 220, 220),
                BorderStyle = BorderStyle.FixedSingle,
                Font = new Font("Segoe UI", 10f),
                WordWrap = true,
                ScrollBars = RichTextBoxScrollBars.Vertical,
                Text = !string.IsNullOrEmpty(aiDescription)
                    ? aiDescription
                    : "Waiting for AI to generate description...",
                Margin = new Padding(8, 4, 8, 4),
            };
            // Remove the URL-detection underline behavior
            txtDesc.DetectUrls = false;
            layout.Controls.Add(txtDesc, 0, 1);

            var btnPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.RightToLeft,
                AutoSize = true,
                Padding = new Padding(0, 8, 0, 0),
            };

            var btnClose = new Button
            {
                Text = "Close",
                Size = new Size(80, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
            };
            btnClose.Click += (s, e) => popup.Close();

            var btnCopy = new Button
            {
                Text = "Copy Text",
                Size = new Size(90, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
                Enabled = !string.IsNullOrEmpty(aiDescription),
            };
            btnCopy.Click += (s, e) =>
            {
                try
                {
                    if (!string.IsNullOrEmpty(currentDescription))
                    {
                        Clipboard.SetText(currentDescription);
                        btnCopy.Text = "Copied!";
                        var resetTimer = new System.Windows.Forms.Timer { Interval = 1500 };
                        resetTimer.Tick += (ts, te) => { btnCopy.Text = "Copy Text"; resetTimer.Stop(); resetTimer.Dispose(); };
                        resetTimer.Start();
                    }
                }
                catch { }
            };

            btnPanel.Controls.Add(btnClose);
            btnPanel.Controls.Add(btnCopy);
            layout.Controls.Add(btnPanel, 0, 2);

            // Poll timer: re-read JSON every 2s until AI description appears
            System.Windows.Forms.Timer pollTimer = null;
            if (string.IsNullOrEmpty(aiDescription))
            {
                pollTimer = new System.Windows.Forms.Timer { Interval = 2000 };
                pollTimer.Tick += (s, e) =>
                {
                    ReadMetadata(out var desc, out var prov, out var mod, out var gen);
                    if (!string.IsNullOrEmpty(desc))
                    {
                        currentDescription = desc;
                        txtDesc.Text = desc;
                        header.Text = $"Provider: {prov}  |  Model: {mod ?? "N/A"}  |  Generated: {gen ?? "N/A"}";
                        btnCopy.Enabled = true;
                        pollTimer.Stop();
                        pollTimer.Dispose();
                    }
                };
                pollTimer.Start();
            }

            popup.FormClosed += (s, e) =>
            {
                pollTimer?.Stop();
                pollTimer?.Dispose();
            };

            popup.Controls.Add(layout);
            popup.ShowDialog(this);
        }

        /// <summary>Dark color table for context menu styling.</summary>
        private class DarkMenuColorTable : ProfessionalColorTable
        {
            public override Color MenuBorder => Color.FromArgb(60, 60, 63);
            public override Color MenuItemSelected => Color.FromArgb(60, 60, 63);
            public override Color MenuItemSelectedGradientBegin => Color.FromArgb(50, 50, 53);
            public override Color MenuItemSelectedGradientEnd => Color.FromArgb(50, 50, 53);
            public override Color MenuItemBorder => Color.FromArgb(70, 70, 73);
            public override Color MenuStripGradientBegin => Color.FromArgb(35, 35, 38);
            public override Color MenuStripGradientEnd => Color.FromArgb(35, 35, 38);
            public override Color ToolStripDropDownBackground => Color.FromArgb(35, 35, 38);
            public override Color ImageMarginGradientBegin => Color.FromArgb(35, 35, 38);
            public override Color ImageMarginGradientMiddle => Color.FromArgb(35, 35, 38);
            public override Color ImageMarginGradientEnd => Color.FromArgb(35, 35, 38);
            public override Color SeparatorDark => Color.FromArgb(60, 60, 63);
            public override Color SeparatorLight => Color.FromArgb(60, 60, 63);
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _videoPlayer?.Dispose();
                _payloadControl?.Dispose();
                _uploadPanel?.Dispose();
                _tabControl?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
