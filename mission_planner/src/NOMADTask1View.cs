// ============================================================
// NOMAD Task 1 View - Outdoor Reconnaissance
// ============================================================

using System;
using System.Drawing;
using System.IO;
using System.Linq;
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
        private Button _btnCapture;
        private TextBox _txtResult;
        private EmbeddedVideoPlayer _videoPlayer;
        private FlowLayoutPanel _galleryPanel;
        private PayloadControlPanel _payloadControl;
        private Task1UploadPanel _uploadPanel;
        private TabControl _tabControl;
    private TextBox _txtCornerName;
    private ListBox _lstCorners;
    private Label _lblCornerStatus;
    private Button _btnCaptureCorner;
    private Button _btnApplyCorners;
    private Button _btnClearCorners;
    private TextBox _txtBuildingHeight;
private ListBox _lstWalls;
        private TextBox _txtWallOverride;
        private Button _btnSetGroundAlt;
        private Label _lblGroundAlt;
        private Button _btnRegenDescriptions;
        private Label _lblDetectionStatus;
        private Label _lblTiltInfo;
        private int _detectionPollCounter = 0;
        private int _tiltPollCounter = 2; // Staggered from detection poll
        private int _lastDetectionCount = 0;
        
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
                // Pin the bridge to the Task 1 (HSV color) circle detector,
                // turning the Task 2 shape detector off so the two windows
                // can never display the wrong overlay.
                _ = SetOverlayModeAsync("task1");
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


            mainLayout.Controls.Add(_tabControl, 1, 0);

            this.AutoScroll = false;
            this.Controls.Add(mainLayout);

            // Restore gallery from locally saved captures after UI is ready
            this.Load += (s, e) => UiAsync.Run(this, RestoreGalleryAsync, nameof(RestoreGalleryAsync));
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
            // Scrollable container \u2014 cards stack top-to-bottom with fixed heights.
            // Scrolling kicks in automatically when the window is too short.
            var scroll = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                AutoScroll = true,
            };

            // Inner panel holds all cards stacked via DockStyle.Top (reverse add order).
            var inner = new Panel
            {
                Dock = DockStyle.Top,
                BackColor = CARD_BG,
                Height = 120 + 145 + 185 + 220,  // sum of card heights below
            };

            // --- Gallery Card (added first so it docks at bottom of inner, behind others) ---
            var galleryCard = CreateCard("CAPTURED IMAGES");
            galleryCard.Dock = DockStyle.Top;
            galleryCard.Height = 220;

            _galleryPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.None,
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Bottom,
                Location = new Point(15, 40),
                AutoScroll = true,
                BorderStyle = BorderStyle.FixedSingle,
                BackColor = Color.FromArgb(25, 25, 28),
            };
            galleryCard.Controls.Add(_galleryPanel);
            galleryCard.Resize += (s, e) =>
            {
                _galleryPanel.Width = galleryCard.ClientSize.Width - 30;
                _galleryPanel.Height = galleryCard.ClientSize.Height - 50;
            };

            // --- Capture Card ---
            var captureCard = CreateCard("SNAPSHOT CAPTURE");
            captureCard.Dock = DockStyle.Top;
            captureCard.Height = 185;

            _btnCapture = CreateButton("CAPTURE PHOTO", ACCENT_COLOR, 200, 32);
            _btnCapture.Location = new Point(15, 38);
            _btnCapture.Click += BtnCapture_Click;
            captureCard.Controls.Add(_btnCapture);

            _txtResult = new TextBox
            {
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
                Location = new Point(15, 78),
                Size = new Size(280, 90),
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
            };

            // --- Payload Controls ---
            _payloadControl = new PayloadControlPanel(_config);
            _payloadControl.Dock = DockStyle.Top;
            _payloadControl.Height = 145;
            _payloadControl.Margin = new Padding(5, 0, 5, 0);

            // --- GPS Status Card ---
            var gpsCard = CreateCard("GPS STATUS");
            gpsCard.Dock = DockStyle.Top;
            gpsCard.Height = 120;

            _lblGpsStatus = new Label
            {
                Text = "Fix: Waiting...",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 38),
                AutoSize = true,
            };
            gpsCard.Controls.Add(_lblGpsStatus);

            _lblPosition = new Label
            {
                Text = "Position: --",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 58),
                AutoSize = true,
            };
            gpsCard.Controls.Add(_lblPosition);

            _lblDetectionStatus = new Label
            {
                Text = "\u25CB No detection",
                Font = new Font("Consolas", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 78),
                AutoSize = true,
            };
            gpsCard.Controls.Add(_lblDetectionStatus);

            _lblTiltInfo = new Label
            {
                Text = "Tilt: -- | Gnd: --",
                Font = new Font("Consolas", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 96),
                AutoSize = true,
            };
            gpsCard.Controls.Add(_lblTiltInfo);

            // Add cards to inner in reverse order (DockStyle.Top stacks bottom-up in WinForms)
            inner.Controls.Add(galleryCard);
            inner.Controls.Add(captureCard);
            inner.Controls.Add(_payloadControl);
            inner.Controls.Add(gpsCard);

            // Resize inner panel to match total card height when scroll area resizes
            scroll.Resize += (s, e) =>
            {
                int totalH = gpsCard.Height + _payloadControl.Height + captureCard.Height + galleryCard.Height;
                inner.Height = Math.Max(totalH, scroll.ClientSize.Height);
            };

            scroll.Controls.Add(inner);
            return scroll;
        }

        private Panel CreateConfigurationSubtab()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                Padding = new Padding(12),
                AutoScroll = true,
            };

            // --- Building Dimensions Card ---
            var dimensionsCard = CreateCard("BUILDING DIMENSIONS");
            dimensionsCard.Dock = DockStyle.Top;
            dimensionsCard.Height = 380;

            var lblHeightInfo = new Label
            {
                Text = "Enter the building height and wall lengths from competition data",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(16, 34),
                AutoSize = true,
            };
            dimensionsCard.Controls.Add(lblHeightInfo);

            var lblHeight = new Label
            {
                Text = "Building Height (m):",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(16, 58),
                AutoSize = true,
            };
            dimensionsCard.Controls.Add(lblHeight);

            _txtBuildingHeight = new TextBox
            {
                Text = "5.0",
                Location = new Point(16, 76),
                Size = new Size(100, 28),
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
                BorderStyle = BorderStyle.FixedSingle,
                Font = new Font("Segoe UI", 9),
            };
            dimensionsCard.Controls.Add(_txtBuildingHeight);

            var btnSetHeight = CreateButton("Set Height", ACCENT_COLOR, 100, 28);
            btnSetHeight.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            btnSetHeight.Location = new Point(126, 76);
            btnSetHeight.Click += BtnSetHeight_Click;
            dimensionsCard.Controls.Add(btnSetHeight);

            var lblWalls = new Label
            {
                Text = "Wall Lengths (calculated from GPS, editable):",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(16, 114),
                AutoSize = true,
            };
            dimensionsCard.Controls.Add(lblWalls);

            _lstWalls = new ListBox
            {
                Location = new Point(16, 136),
                Size = new Size(280, 120),
                BackColor = Color.FromArgb(40, 40, 43),
                ForeColor = Color.White,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.FixedSingle,
                SelectionMode = SelectionMode.One,
            };
            dimensionsCard.Controls.Add(_lstWalls);

            var lblOverride = new Label
            {
                Text = "Manual Override (m):",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(16, 264),
                AutoSize = true,
            };
            dimensionsCard.Controls.Add(lblOverride);

            _txtWallOverride = new TextBox
            {
                Location = new Point(16, 282),
                Size = new Size(100, 28),
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
                BorderStyle = BorderStyle.FixedSingle,
                Font = new Font("Segoe UI", 9),
            };
            dimensionsCard.Controls.Add(_txtWallOverride);

            var btnSetWall = CreateButton("Set Wall", SUCCESS_COLOR, 80, 28);
            btnSetWall.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            btnSetWall.Location = new Point(126, 282);
            btnSetWall.Click += BtnSetWall_Click;
            dimensionsCard.Controls.Add(btnSetWall);

            var btnClearWall = CreateButton("Clear", ERROR_COLOR, 70, 28);
            btnClearWall.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            btnClearWall.Location = new Point(216, 282);
            btnClearWall.Click += BtnClearWall_Click;
            dimensionsCard.Controls.Add(btnClearWall);

            var btnRefreshWalls = CreateButton("Refresh Walls", Color.FromArgb(70, 70, 73), 120, 28);
            btnRefreshWalls.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            btnRefreshWalls.Location = new Point(16, 318);
            btnRefreshWalls.Click += (s, e) => UiAsync.Run(this, RefreshWallsAsync, nameof(RefreshWallsAsync));
            dimensionsCard.Controls.Add(btnRefreshWalls);

            dimensionsCard.Resize += (s, e) =>
            {
                int inputWidth = Math.Max(260, dimensionsCard.ClientSize.Width - 32);
                _lstWalls.Width = inputWidth;
            };

            panel.Controls.Add(dimensionsCard);

            // --- Building Corner Calibration ---
            var cornerCard = CreateCard("BUILDING CORNER CALIBRATION");
            cornerCard.Dock = DockStyle.Top;
            cornerCard.Height = 320;

            _lblCornerStatus = new Label
            {
                Text = "Fly above each building corner and click Capture Corner",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(16, 34),
                AutoSize = true,
            };
            cornerCard.Controls.Add(_lblCornerStatus);

            var lblCornerName = new Label
            {
                Text = "Corner Name:",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(16, 58),
                AutoSize = true,
            };
            cornerCard.Controls.Add(lblCornerName);

            _txtCornerName = new TextBox
            {
                Text = "NW",
                Location = new Point(16, 76),
                Size = new Size(120, 28),
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
                BorderStyle = BorderStyle.FixedSingle,
                Font = new Font("Segoe UI", 9),
            };
            cornerCard.Controls.Add(_txtCornerName);

            // Preset corner name buttons: cardinal (rectangles) + alphabetic (any polygon)
            string[] presetNames = { "NW", "NE", "SE", "SW", "A", "B", "C" };
            for (int i = 0; i < presetNames.Length; i++)
            {
                string name = presetNames[i];
                int btnW = name.Length > 1 ? 36 : 28;
                var btnPreset = CreateButton(name, Color.FromArgb(70, 70, 73), btnW, 26);
                btnPreset.Font = new Font("Segoe UI", 7, FontStyle.Bold);
                btnPreset.Location = new Point(144 + i * 38, 77);
                btnPreset.Click += (s, e) => _txtCornerName.Text = name;
                cornerCard.Controls.Add(btnPreset);
            }

            _btnCaptureCorner = CreateButton("CAPTURE CORNER GPS", ACCENT_COLOR, 250, 32);
            _btnCaptureCorner.Font = new Font("Segoe UI", 9, FontStyle.Bold);
            _btnCaptureCorner.Location = new Point(16, 110);
            _btnCaptureCorner.Click += BtnCaptureCorner_Click;
            cornerCard.Controls.Add(_btnCaptureCorner);

            _lstCorners = new ListBox
            {
                Location = new Point(16, 150),
                Size = new Size(280, 80),
                BackColor = Color.FromArgb(40, 40, 43),
                ForeColor = Color.White,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.FixedSingle,
                SelectionMode = SelectionMode.One,
            };
            cornerCard.Controls.Add(_lstCorners);

            _btnApplyCorners = CreateButton("Apply to Model", SUCCESS_COLOR, 130, 30);
            _btnApplyCorners.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            _btnApplyCorners.Location = new Point(16, 238);
            _btnApplyCorners.Click += BtnApplyCorners_Click;
            cornerCard.Controls.Add(_btnApplyCorners);

            _btnClearCorners = CreateButton("Clear All", ERROR_COLOR, 90, 30);
            _btnClearCorners.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            _btnClearCorners.Location = new Point(156, 238);
            _btnClearCorners.Click += BtnClearCorners_Click;
            cornerCard.Controls.Add(_btnClearCorners);

            var btnRefreshCorners = CreateButton("Refresh", Color.FromArgb(70, 70, 73), 90, 30);
            btnRefreshCorners.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            btnRefreshCorners.Location = new Point(256, 238);
            btnRefreshCorners.Click += (s, e) => UiAsync.Run(this, RefreshCornerListAsync, nameof(RefreshCornerListAsync));
            cornerCard.Controls.Add(btnRefreshCorners);

            cornerCard.Resize += (s, e) =>
            {
                int inputWidth = Math.Max(260, cornerCard.ClientSize.Width - 32);
                _btnCaptureCorner.Width = inputWidth;
                _lstCorners.Width = inputWidth;
            };

            panel.Controls.Add(cornerCard);

            // --- Ground Altitude Calibration ---
            var groundAltCard = CreateCard("GROUND ALTITUDE CALIBRATION");
            groundAltCard.Dock = DockStyle.Top;
            groundAltCard.Height = 140;

            var lblGroundAltInfo = new Label
            {
                Text = "Land the drone on the ground, then click to set 0m AGL reference",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(16, 34),
                AutoSize = true,
            };
            groundAltCard.Controls.Add(lblGroundAltInfo);

            _btnSetGroundAlt = CreateButton("SET AS GROUND ALT", ACCENT_COLOR, 200, 28);
            _btnSetGroundAlt.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            _btnSetGroundAlt.Location = new Point(16, 56);
            _btnSetGroundAlt.Click += BtnSetGroundAlt_Click;
            groundAltCard.Controls.Add(_btnSetGroundAlt);

            _lblGroundAlt = new Label
            {
                Text = "Offset: not set",
                Font = new Font("Consolas", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(226, 60),
                AutoSize = true,
            };
            groundAltCard.Controls.Add(_lblGroundAlt);

            _btnRegenDescriptions = CreateButton("REGENERATE DESCRIPTIONS", ACCENT_COLOR, 240, 28);
            _btnRegenDescriptions.Font = new Font("Segoe UI", 8, FontStyle.Bold);
            _btnRegenDescriptions.Location = new Point(16, 96);
            _btnRegenDescriptions.Click += BtnRegenDescriptions_Click;
            groundAltCard.Controls.Add(_btnRegenDescriptions);

            panel.Controls.Add(groundAltCard);

            return panel;
        }

        private void BtnCaptureCorner_Click(object sender, EventArgs e)
        {
            UiAsync.Run(this, () => BtnCaptureCornerAsync(sender, e), nameof(BtnCaptureCorner_Click));
        }

        private async Task BtnCaptureCornerAsync(object sender, EventArgs e)
        {
            var cs = MainV2.comPort?.MAV?.cs;
            if (cs == null || (Math.Abs(cs.lat) < 0.000001 && Math.Abs(cs.lng) < 0.000001))
            {
                _txtResult.Text = "[FAIL] No GPS position available. Fly above the corner first.";
                _txtResult.ForeColor = ERROR_COLOR;
                return;
            }

            string cornerName = _txtCornerName.Text.Trim();
            if (string.IsNullOrEmpty(cornerName))
            {
                _txtResult.Text = "[FAIL] Enter a corner name (e.g. NW, NE).";
                _txtResult.ForeColor = ERROR_COLOR;
                return;
            }

            _btnCaptureCorner.Enabled = false;
            _btnCaptureCorner.Text = "Capturing...";
            _txtResult.Text = $"Saving corner {cornerName} at {cs.lat:F6}, {cs.lng:F6}...";
            _txtResult.ForeColor = WARNING_COLOR;

            try
            {
                var body = new { name = cornerName, lat = cs.lat, lon = cs.lng };
                var json = Newtonsoft.Json.JsonConvert.SerializeObject(body);
                var httpContent = new System.Net.Http.StringContent(json, System.Text.Encoding.UTF8, "application/json");
                var response = await JetsonApiService.PostAsync("/api/task/1/building/corner", httpContent);

                if (response.IsSuccessStatusCode)
                {
                    var respBody = await response.Content.ReadAsStringAsync();
                    var data = Newtonsoft.Json.Linq.JObject.Parse(respBody);
                    int total = (int?)data["total_corners"] ?? 0;
                    bool canApply = (bool?)data["can_apply"] ?? false;

                    _txtResult.Text = $"[OK] Corner '{cornerName}' saved ({cs.lat:F6}, {cs.lng:F6}). {total} corners total." + (canApply ? " Ready to Apply!" : " Need >= 3 corners to apply.");
                    _txtResult.ForeColor = SUCCESS_COLOR;

                    // Auto-advance to next corner name suggestion
                    AutoAdvanceCornerName(cornerName);

                    await RefreshCornerListAsync();
                }
                else
                {
                    _txtResult.Text = $"[FAIL] API returned {response.StatusCode}";
                    _txtResult.ForeColor = ERROR_COLOR;
                }
            }
            catch (Exception ex)
            {
                _txtResult.Text = $"[FAIL] {ex.Message}";
                _txtResult.ForeColor = ERROR_COLOR;
            }
            finally
            {
                _btnCaptureCorner.Enabled = true;
                _btnCaptureCorner.Text = "CAPTURE CORNER GPS";
            }
        }

        private void AutoAdvanceCornerName(string current)
        {
            // Cardinal sequence: NW -> NE -> SE -> SW (good for rectangles)
            var cardinal = new[] { "NW", "NE", "SE", "SW" };
            int idx = Array.IndexOf(cardinal, current.ToUpper());
            if (idx >= 0 && idx < cardinal.Length - 1)
            {
                _txtCornerName.Text = cardinal[idx + 1];
                return;
            }

            // Alphabetic sequence: A -> B -> C -> ... -> Z (scales to any polygon)
            if (current.Length == 1 && char.IsLetter(current[0]))
            {
                char next = (char)(char.ToUpper(current[0]) + 1);
                if (next <= 'Z')
                    _txtCornerName.Text = next.ToString();
            }
        }

        private async Task RefreshCornerListAsync()
        {
            try
            {
                var response = await JetsonApiService.GetAsync("/api/task/1/building/corners");
                if (response.IsSuccessStatusCode)
                {
                    var body = await response.Content.ReadAsStringAsync();
                    var data = Newtonsoft.Json.Linq.JObject.Parse(body);
                    var corners = data["corners"] as Newtonsoft.Json.Linq.JArray;
                    int total = (int?)data["total_corners"] ?? 0;
                    bool canApply = (bool?)data["can_apply"] ?? false;

                    _lstCorners.Items.Clear();
                    if (corners != null)
                    {
                        foreach (var c in corners)
                        {
                            string name = c["name"]?.ToString() ?? "?";
                            double lat = (double?)c["lat"] ?? 0;
                            double lon = (double?)c["lon"] ?? 0;
                            _lstCorners.Items.Add($"{name}: {lat:F6}, {lon:F6}");
                        }
                    }

                    _lblCornerStatus.Text = total == 0
                        ? "Fly above each building corner and click Capture Corner"
                        : $"{total} corners captured" + (canApply ? " - Ready to Apply!" : " - Need >= 3");
                    _lblCornerStatus.ForeColor = canApply ? SUCCESS_COLOR : TEXT_SECONDARY;
                    
                    // Also refresh walls when corners change
                    await RefreshWallsAsync();
                }
            }
            catch
            {
                // Silently fail - API may not be available
            }
        }

        private void BtnApplyCorners_Click(object sender, EventArgs e)
        {
            UiAsync.Run(this, () => BtnApplyCornersAsync(sender, e), nameof(BtnApplyCorners_Click));
        }

        private async Task BtnApplyCornersAsync(object sender, EventArgs e)
        {
            _btnApplyCorners.Enabled = false;
            _txtResult.Text = "Applying corners to building model...";
            _txtResult.ForeColor = WARNING_COLOR;

            try
            {
                var response = await JetsonApiService.PostAsync("/api/task/1/building/corners/apply", null);
                if (response.IsSuccessStatusCode)
                {
                    var body = await response.Content.ReadAsStringAsync();
                    var data = Newtonsoft.Json.Linq.JObject.Parse(body);
                    int count = (data["corners"] as Newtonsoft.Json.Linq.JArray)?.Count ?? 0;
                    _txtResult.Text = $"[OK] Building model rebuilt with {count} corners!";
                    _txtResult.ForeColor = SUCCESS_COLOR;
                }
                else
                {
                    var errorBody = await response.Content.ReadAsStringAsync();
                    _txtResult.Text = $"[FAIL] Apply corners failed: {response.StatusCode} - {errorBody}";
                    _txtResult.ForeColor = ERROR_COLOR;
                }
            }
            catch (Exception ex)
            {
                _txtResult.Text = $"[FAIL] {ex.Message}";
                _txtResult.ForeColor = ERROR_COLOR;
            }
            finally
            {
                _btnApplyCorners.Enabled = true;
            }
        }

        private void BtnClearCorners_Click(object sender, EventArgs e)
        {
            UiAsync.Run(this, () => BtnClearCornersAsync(sender, e), nameof(BtnClearCorners_Click));
        }

        private async Task BtnClearCornersAsync(object sender, EventArgs e)
        {
            try
            {
                var response = await JetsonApiService.DeleteAsync("/api/task/1/building/corners");
                if (response.IsSuccessStatusCode)
                {
                    _lstCorners.Items.Clear();
                    _lblCornerStatus.Text = "Fly above each building corner and click Capture Corner";
                    _lblCornerStatus.ForeColor = TEXT_SECONDARY;
                    _txtResult.Text = "[OK] Building corners cleared.";
                    _txtResult.ForeColor = SUCCESS_COLOR;
                }
                else
                {
                    _txtResult.Text = $"[FAIL] Clear failed: {response.StatusCode}";
                    _txtResult.ForeColor = ERROR_COLOR;
                }
            }
            catch (Exception ex)
            {
                _txtResult.Text = $"[FAIL] {ex.Message}";
                _txtResult.ForeColor = ERROR_COLOR;
            }
        }

        private void BtnSetGroundAlt_Click(object sender, EventArgs e)
        {
            UiAsync.Run(this, () => BtnSetGroundAltAsync(sender, e), nameof(BtnSetGroundAlt_Click));
        }

        private async Task BtnSetGroundAltAsync(object sender, EventArgs e)
        {
            _btnSetGroundAlt.Enabled = false;
            _btnSetGroundAlt.Text = "Setting...";
            try
            {
                var response = await JetsonApiService.PostLongRunAsync("/api/task/1/target/ground_alt", null);
                if (response.IsSuccessStatusCode)
                {
                    var body = await response.Content.ReadAsStringAsync();
                    // Extract the altitude value from "Ground altitude set to X.XXm..."
                    string altStr = "?";
                    try
                    {
                        var json = Newtonsoft.Json.Linq.JObject.Parse(body);
                        var msg = json["output"]?.ToString() ?? json["message"]?.ToString() ?? "";
                        // Backend formats: "Ground altitude set to -78.50m. All heights..."
                        // The leading minus must be captured; the old regex dropped it and
                        // displayed e.g. "78.50m" instead of "-78.50m".
                        var m = System.Text.RegularExpressions.Regex.Match(msg, @"(-?\d+(?:\.\d+)?)\s*m");
                        if (m.Success) altStr = m.Groups[1].Value + "m";
                    }
                    catch { }
                    _lblGroundAlt.Text = $"Offset: {altStr}";
                    _lblGroundAlt.ForeColor = SUCCESS_COLOR;
                    _txtResult.Text = $"[OK] Ground alt set to {altStr}. Heights now relative to ground.";
                    _txtResult.ForeColor = SUCCESS_COLOR;
                }
                else
                {
                    _lblGroundAlt.Text = "Offset: failed";
                    _lblGroundAlt.ForeColor = ERROR_COLOR;
                    _txtResult.Text = $"[FAIL] Ground altitude failed: {response.StatusCode}";
                    _txtResult.ForeColor = ERROR_COLOR;
                }
            }
            catch (Exception ex)
            {
                _lblGroundAlt.Text = "Offset: error";
                _lblGroundAlt.ForeColor = ERROR_COLOR;
                _txtResult.Text = $"[FAIL] {ex.Message}";
                _txtResult.ForeColor = ERROR_COLOR;
            }
            finally
            {
                _btnSetGroundAlt.Enabled = true;
                _btnSetGroundAlt.Text = "SET AS GROUND ALT";
            }
        }

        private void BtnRegenDescriptions_Click(object sender, EventArgs e)
        {
            UiAsync.Run(this, () => BtnRegenDescriptionsAsync(sender, e), nameof(BtnRegenDescriptions_Click));
        }

        private async Task BtnRegenDescriptionsAsync(object sender, EventArgs e)
        {
            _btnRegenDescriptions.Enabled = false;
            _btnRegenDescriptions.Text = "Regenerating...";
            try
            {
                // PostLongRunAsync: the regenerate service may iterate over every
                // captured target. Even with the in-container service proxy
                // (sub-100ms typical) this is the right client to use — short
                // 5s timeout caused this button to silently TaskCancel.
                var response = await JetsonApiService.PostLongRunAsync("/api/task/1/target/regenerate", null);
                var body = await response.Content.ReadAsStringAsync();
                if (response.IsSuccessStatusCode)
                {
                    // Surface the backend's "Regenerated N/M target description(s)."
                    // so the operator sees real feedback instead of a generic OK.
                    string detail = null;
                    try
                    {
                        var json = Newtonsoft.Json.Linq.JObject.Parse(body);
                        detail = json["output"]?.ToString() ?? json["message"]?.ToString();
                    }
                    catch { }
                    var ok = string.IsNullOrWhiteSpace(detail)
                        ? "Target descriptions regenerated from raw data."
                        : detail.Trim();
                    _txtResult.Text = $"[OK] {ok} Reopen the Submit panel to see updated text.";
                    _txtResult.ForeColor = SUCCESS_COLOR;
                    // _txtResult lives further down the layout and can scroll
                    // out of view; show a MessageBox so the operator always
                    // gets visible confirmation.
                    MessageBox.Show(
                        ok + "\n\nReopen the Submit panel to see updated descriptions.",
                        "Descriptions Regenerated",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information);
                }
                else
                {
                    string detail = null;
                    try
                    {
                        var json = Newtonsoft.Json.Linq.JObject.Parse(body);
                        detail = json["detail"]?.ToString();
                    }
                    catch { }
                    var err = string.IsNullOrWhiteSpace(detail)
                        ? $"Regenerate failed: {response.StatusCode}"
                        : $"Regenerate failed: {detail}";
                    _txtResult.Text = "[FAIL] " + err;
                    _txtResult.ForeColor = ERROR_COLOR;
                    MessageBox.Show(err, "Regenerate Failed",
                        MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            catch (Exception ex)
            {
                _txtResult.Text = $"[FAIL] {ex.Message}";
                _txtResult.ForeColor = ERROR_COLOR;
                MessageBox.Show(ex.Message, "Regenerate Failed",
                    MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                _btnRegenDescriptions.Enabled = true;
                _btnRegenDescriptions.Text = "REGENERATE DESCRIPTIONS";
            }
        }

        private void BtnSetHeight_Click(object sender, EventArgs e)
        {
            UiAsync.Run(this, () => BtnSetHeightAsync(sender, e), nameof(BtnSetHeight_Click));
        }

        private async Task BtnSetHeightAsync(object sender, EventArgs e)
        {
            if (!double.TryParse(_txtBuildingHeight.Text, out double height) || height <= 0)
            {
                _txtResult.Text = "[FAIL] Enter a valid building height (e.g., 5.0)";
                _txtResult.ForeColor = ERROR_COLOR;
                return;
            }

            try
            {
                var response = await JetsonApiService.PostAsync($"/api/task/1/building/height?height={height}", null);
                if (response.IsSuccessStatusCode)
                {
                    _txtResult.Text = $"[OK] Building height set to {height}m";
                    _txtResult.ForeColor = SUCCESS_COLOR;
                    if (_uploadPanel != null) _uploadPanel.BuildingHeight = height;
                    await RefreshWallsAsync();
                }
                else
                {
                    _txtResult.Text = $"[FAIL] API returned {response.StatusCode}";
                    _txtResult.ForeColor = ERROR_COLOR;
                }
            }
            catch (Exception ex)
            {
                _txtResult.Text = $"[FAIL] {ex.Message}";
                _txtResult.ForeColor = ERROR_COLOR;
            }
        }

        private async Task RefreshWallsAsync()
        {
            try
            {
                var response = await JetsonApiService.GetAsync("/api/task/1/building/corners");
                if (response.IsSuccessStatusCode)
                {
                    var body = await response.Content.ReadAsStringAsync();
                    var data = Newtonsoft.Json.Linq.JObject.Parse(body);
                    var walls = data["walls"] as Newtonsoft.Json.Linq.JArray;
                    double? height = (double?)data["height"];

                    if (height.HasValue)
                    {
                        _txtBuildingHeight.Text = height.Value.ToString("F1");
                    }

                    _lstWalls.Items.Clear();
                    if (walls != null && walls.Count > 0)
                    {
                        foreach (var w in walls)
                        {
                            string name = w["name"]?.ToString() ?? "?";
                            double lengthM = (double?)w["length_m"] ?? 0;
                            double? overrideM = (double?)w["manual_override_m"];

                            string display = overrideM.HasValue
                                ? $"{name}: {lengthM:F2}m → {overrideM.Value:F2}m (manual)"
                                : $"{name}: {lengthM:F2}m";
                            
                            _lstWalls.Items.Add(display);
                        }
                    }
                    else
                    {
                        _lstWalls.Items.Add("(No walls - add >= 3 corners first)");
                    }
                }
            }
            catch
            {
                // Silently fail - API may not be available
            }
        }

        private void BtnSetWall_Click(object sender, EventArgs e)
        {
            UiAsync.Run(this, () => BtnSetWallAsync(sender, e), nameof(BtnSetWall_Click));
        }

        private async Task BtnSetWallAsync(object sender, EventArgs e)
        {
            if (_lstWalls.SelectedIndex < 0)
            {
                _txtResult.Text = "[FAIL] Select a wall from the list first";
                _txtResult.ForeColor = ERROR_COLOR;
                return;
            }

            if (!double.TryParse(_txtWallOverride.Text, out double length) || length <= 0)
            {
                _txtResult.Text = "[FAIL] Enter a valid wall length (e.g., 10.5)";
                _txtResult.ForeColor = ERROR_COLOR;
                return;
            }

            try
            {
                // Extract wall name from selected item (format: "NW-NE: 10.5m" or "NW-NE: 10.5m → 12.0m (manual)")
                string selectedItem = _lstWalls.SelectedItem.ToString();
                string wallName = selectedItem.Split(':')[0].Trim();

                var response = await JetsonApiService.PostAsync(
                    $"/api/task/1/building/wall/override?wall_name={Uri.EscapeDataString(wallName)}&length_m={length}",
                    null);

                if (response.IsSuccessStatusCode)
                {
                    _txtResult.Text = $"[OK] Wall '{wallName}' set to {length}m";
                    _txtResult.ForeColor = SUCCESS_COLOR;
                    _txtWallOverride.Clear();
                    await RefreshWallsAsync();
                }
                else
                {
                    var errorBody = await response.Content.ReadAsStringAsync();
                    _txtResult.Text = $"[FAIL] {response.StatusCode}: {errorBody}";
                    _txtResult.ForeColor = ERROR_COLOR;
                }
            }
            catch (Exception ex)
            {
                _txtResult.Text = $"[FAIL] {ex.Message}";
                _txtResult.ForeColor = ERROR_COLOR;
            }
        }

        private void BtnClearWall_Click(object sender, EventArgs e)
        {
            UiAsync.Run(this, () => BtnClearWallAsync(sender, e), nameof(BtnClearWall_Click));
        }

        private async Task BtnClearWallAsync(object sender, EventArgs e)
        {
            if (_lstWalls.SelectedIndex < 0)
            {
                _txtResult.Text = "[FAIL] Select a wall from the list first";
                _txtResult.ForeColor = ERROR_COLOR;
                return;
            }

            try
            {
                // Extract wall name from selected item
                string selectedItem = _lstWalls.SelectedItem.ToString();
                string wallName = selectedItem.Split(':')[0].Trim();

                var response = await JetsonApiService.PostAsync(
                    $"/api/task/1/building/wall/override?wall_name={Uri.EscapeDataString(wallName)}",
                    null);

                if (response.IsSuccessStatusCode)
                {
                    _txtResult.Text = $"[OK] Wall '{wallName}' override cleared (using GPS-calculated length)";
                    _txtResult.ForeColor = SUCCESS_COLOR;
                    _txtWallOverride.Clear();
                    await RefreshWallsAsync();
                }
                else
                {
                    _txtResult.Text = $"[FAIL] API returned {response.StatusCode}";
                    _txtResult.ForeColor = ERROR_COLOR;
                }
            }
            catch (Exception ex)
            {
                _txtResult.Text = $"[FAIL] {ex.Message}";
                _txtResult.ForeColor = ERROR_COLOR;
            }
        }

        private void RestoreCaptureButton()
        {
            _btnCapture.Enabled = true;
            if (_lastDetectionCount > 0)
            {
                _btnCapture.BackColor = SUCCESS_COLOR;
                _btnCapture.Text = $"● CAPTURE ({_lastDetectionCount} circle{(_lastDetectionCount > 1 ? "s" : "")})";
            }
            else
            {
                _btnCapture.BackColor = ACCENT_COLOR;
                _btnCapture.Text = "CAPTURE (crosshair)";
            }
        }

        private void BtnCapture_Click(object sender, EventArgs e)
        {
            UiAsync.Run(this, () => BtnCaptureAsync(sender, e), nameof(BtnCapture_Click));
        }

        private async Task BtnCaptureAsync(object sender, EventArgs e)
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
                    if (!string.IsNullOrEmpty(result.Data))
                    {
                        var data = Newtonsoft.Json.Linq.JObject.Parse(result.Data);

                        string Val(string key) {
                            var t = data[key];
                            return t != null && t.Type != Newtonsoft.Json.Linq.JTokenType.Null ? t.ToString() : null;
                        }

                        var apiSuccess = (bool?)data["success"] ?? false;
                        if (!apiSuccess)
                        {
                            _txtResult.Text = $"[FAIL] {Val("error") ?? "Unknown error"}";
                            _txtResult.ForeColor = ERROR_COLOR;
                            return;
                        }

                        var imageName = Val("image_name");

                        if (string.IsNullOrEmpty(imageName))
                        {
                            var output = Val("output") ?? Val("message") ?? Val("detail");
                            _txtResult.Text = "[OK] Capture Successful\n" + (string.IsNullOrWhiteSpace(output)
                                ? "Target localization completed, but no image metadata was returned by the API."
                                : output);
                            _txtResult.ForeColor = SUCCESS_COLOR;
                            return;
                        }

                        var timestamp = Val("timestamp") ?? DateTime.Now.ToString("yyyy-MM-ddTHH:mm:ssZ");
                        var headingDeg = Val("heading_deg") ?? "N/A";
                        var pitchDeg = Val("pitch_deg") ?? "N/A";
                        var rollDeg = Val("roll_deg") ?? "N/A";
                        var cameraPitch = Val("camera_pitch_deg") ?? "N/A";
                        var captureFolder = Val("capture_folder") ?? "N/A";
                        var distanceM = Val("distance_m");
                        var altAgl = Val("alt_agl_m");
                        var rawOutput = Val("output") ?? Val("message") ?? Val("detail");
                        var outputText = System.Text.RegularExpressions.Regex.Replace(
                            rawOutput ?? "", @"(?i)^Added \d+ target\(s\):(\\n|\n|\r\n)?", "").TrimStart();

                        var position = data["position"] as Newtonsoft.Json.Linq.JObject;
                        var latStr = position?["lat"]?.ToString() ?? "N/A";
                        var lonStr = position?["lon"]?.ToString() ?? "N/A";
                        var altStr = position?["alt"]?.ToString() ?? "N/A";

                        var metadataText = new StringBuilder();
                        metadataText.AppendLine("[OK] Capture Successful");
                        metadataText.AppendLine($"Time: {timestamp}");
                        metadataText.AppendLine($"Position: {latStr}, {lonStr} @ {altStr}m");
                        metadataText.AppendLine($"AHRS: Hdg={headingDeg} Pitch={pitchDeg} Roll={rollDeg}");
                        metadataText.AppendLine($"Cam pitch: {cameraPitch}°  AGL: {(altAgl != null ? altAgl + "m" : "N/A")}");
                        if (distanceM != null) metadataText.AppendLine($"Distance to target: {distanceM}m");
                        metadataText.AppendLine($"Folder: {captureFolder}");
                        int targetNum = (_uploadPanel?.TargetCount ?? 0) + 1;
                        metadataText.Append($"Image: Target {targetNum} ({imageName}) — downloading...");

                        _txtResult.Text = metadataText.ToString();
                        _txtResult.ForeColor = SUCCESS_COLOR;

                        // Re-enable button immediately — image download runs in background
                        RestoreCaptureButton();

                        var folderName = Path.GetFileName(captureFolder.TrimEnd('/'));
                        string imageUrl = $"http://{_config.EffectiveIP}:{_config.JetsonPort}/api/task/1/images/{folderName}/{imageName}";
                        var task1Dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "NOMAD", "Task1");
                        var ext = Path.GetExtension(imageName);
                        if (string.IsNullOrEmpty(ext)) ext = ".jpg";
                        var localImageName = $"target_{targetNum}_{folderName}{ext}";
                        var localPath = Path.Combine(task1Dir, localImageName);

                        // Snapshot all captures needed by the background task
                        var snap_imageName = imageName;
                        var snap_timestamp = timestamp;
                        var snap_latStr = latStr; var snap_lonStr = lonStr; var snap_altStr = altStr;
                        var snap_headingDeg = headingDeg; var snap_pitchDeg = pitchDeg; var snap_rollDeg = rollDeg;
                        var snap_cameraPitch = cameraPitch; var snap_altAgl = altAgl; var snap_distanceM = distanceM;
                        var snap_outputText = outputText;
                        var snap_metaStr = metadataText.ToString().Replace("— downloading...", "");
                        var snap_targetNum = targetNum;
                        var snap_folderName = folderName;
                        var snap_tooltipText = $"{imageName}\nPosition: {latStr}, {lonStr}\nHeading: {headingDeg} Pitch: {pitchDeg} Roll: {rollDeg}\nCamera pitch: {cameraPitch}";

                        _ = DownloadCaptureImageAsync(
                            imageUrl, localPath, snap_imageName, snap_timestamp, snap_latStr, snap_lonStr, snap_altStr,
                            snap_headingDeg, snap_pitchDeg, snap_rollDeg, snap_cameraPitch, snap_altAgl, snap_distanceM,
                            snap_outputText, snap_metaStr, snap_targetNum, snap_tooltipText);
                    }
                    else
                    {
                        _txtResult.Text = $"[OK] Capture successful: {result.Message}";
                        _txtResult.ForeColor = SUCCESS_COLOR;
                    }
                }
                else
                {
                    string detail = null;
                    if (!string.IsNullOrEmpty(result.Data))
                    {
                        try
                        {
                            var errJson = Newtonsoft.Json.Linq.JObject.Parse(result.Data);
                            detail = errJson["detail"]?.ToString()
                                  ?? errJson["error"]?.ToString()
                                  ?? errJson["message"]?.ToString();
                        }
                        catch { /* response body wasn't JSON */ }
                    }
                    var combined = string.IsNullOrWhiteSpace(detail)
                        ? result.Message
                        : $"{result.Message} — {detail}";
                    _txtResult.Text = $"[FAIL] Capture failed: {combined}";
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
                // Safety net — button may already be restored above for the success path
                if (!_btnCapture.Enabled)
                    RestoreCaptureButton();
            }
        }

        private async Task DownloadCaptureImageAsync(
            string imageUrl, string localPath, string imageName, string timestamp,
            string latStr, string lonStr, string altStr,
            string headingDeg, string pitchDeg, string rollDeg, string cameraPitch,
            string altAgl, string distanceM, string outputText, string metaStr,
            int targetNum, string tooltipText)
        {
            try
            {
                var imageBytes = await JetsonApiService.GetByteArrayAsync(imageUrl);

                Directory.CreateDirectory(Path.GetDirectoryName(localPath)!);
                File.WriteAllBytes(localPath, imageBytes);

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
                    CameraPitchDeg = double.TryParse(cameraPitch, out var cPitch) ? cPitch : (double?)null,
                    AltAglM = double.TryParse(altAgl, out var vAlt) ? vAlt : (double?)null,
                    DistanceM = double.TryParse(distanceM, out var dM) ? dM : (double?)null,
                    BuildingLocation = null,
                    RelativeDescription = outputText,
                };
                File.WriteAllText(jsonPath, Newtonsoft.Json.JsonConvert.SerializeObject(metadata, Newtonsoft.Json.Formatting.Indented));

                var capturedColor = "Red";
                var capturedPlane = "wall";
                var outputLower = (outputText ?? "").ToLower();
                if (outputLower.Contains("ground")) capturedPlane = "ground";
                else if (outputLower.Contains("roof")) capturedPlane = "roof";
                var capturedHeight = "";
                var heightMatch = System.Text.RegularExpressions.Regex.Match(
                    outputText ?? "", @"(\d+\.?\d*)\s*m\s+above\s+ground");
                if (heightMatch.Success) capturedHeight = heightMatch.Groups[1].Value;

                var suggestedDesc = ExtractSuggestedTask1Description(outputText);
                var capturedImageBytes = imageBytes;
                var capturedLocalPath = localPath;
                var capturedJsonPath = jsonPath;

                this.BeginInvoke(() =>
                {
                    _uploadPanel?.AddCapturedImage(capturedLocalPath, suggestedDesc, capturedColor, capturedPlane, capturedHeight);

                    // Draw the thumbnail onto an independent Bitmap so it does
                    // not retain a lazy reference to the disposed MemoryStream.
                    // GDI+ otherwise reads from the freed stream during paint
                    // and brings down the host with an uncatchable exception.
                    Bitmap thumb = new Bitmap(120, 90);
                    using (var ms = new MemoryStream(capturedImageBytes))
                    using (var originalImage = Image.FromStream(ms))
                    using (var g = Graphics.FromImage(thumb))
                    {
                        g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                        g.DrawImage(originalImage, 0, 0, 120, 90);
                    }
                    var picBox = new PictureBox
                    {
                        Image = thumb,
                        Size = new Size(120, 90),
                        SizeMode = PictureBoxSizeMode.Zoom,
                        Margin = new Padding(5),
                        Cursor = Cursors.Hand,
                        Tag = new { Path = capturedLocalPath, JsonPath = capturedJsonPath, Metadata = metaStr },
                    };
                    picBox.MouseClick += (s, mevt) => ShowImageContextMenu(picBox, mevt.Location);
                    var tip = new ToolTip();
                    tip.SetToolTip(picBox, tooltipText);
                    _galleryPanel.Controls.Add(picBox);
                    _galleryPanel.ScrollControlIntoView(picBox);

                    // Update result text to confirm image saved
                    if (_txtResult.Text.Contains("— downloading..."))
                        _txtResult.Text = _txtResult.Text.Replace("— downloading...", "");
                });
            }
            catch (Exception ex)
            {
                this.BeginInvoke(() =>
                {
                    _txtResult.Text = _txtResult.Text.Replace("— downloading...", $"[WARN] image download failed: {ex.Message}");
                });
            }
        }

        private string ExtractSuggestedTask1Description(string outputText)
        {
            if (string.IsNullOrWhiteSpace(outputText))
                return string.Empty;

            var normalized = outputText.Replace("\r", string.Empty);
            var lines = normalized.Split('\n');
            foreach (var rawLine in lines)
            {
                var line = rawLine?.Trim();
                if (string.IsNullOrEmpty(line))
                    continue;

                if (line.IndexOf("target on", StringComparison.OrdinalIgnoreCase) >= 0)
                    return line;
            }

            return string.Empty;
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

                // Update detection status + capture button color every ~2s
                _detectionPollCounter++;
                if (_detectionPollCounter >= 4)
                {
                    _detectionPollCounter = 0;
                    _ = Task.Run(async () =>
                    {
                        try
                        {
                            var resp = await JetsonApiService.GetAsync("/api/task/1/target/detections");
                            if (resp.IsSuccessStatusCode)
                            {
                                var body = await resp.Content.ReadAsStringAsync();
                                var data = Newtonsoft.Json.Linq.JObject.Parse(body);
                                int count = (int?)data["circle_count"] ?? 0;
                                double? topDist = (double?)data["top_distance_m"];
                                double? centerDist = (double?)data["center_distance_m"];
                                // ground_alt_offset is published continuously by the
                                // target_localizer node. Poll it here so the offset
                                // label survives a Mission Planner restart -- the value
                                // lives in the ROS node, not in this process.
                                double? groundOffset = (double?)data["ground_alt_offset"];
                                this.BeginInvoke(new Action(() =>
                                {
                                    _lastDetectionCount = count;

                                    if (_lblGroundAlt != null && !_lblGroundAlt.IsDisposed
                                        && groundOffset.HasValue && Math.Abs(groundOffset.Value) > 1e-6)
                                    {
                                        var newText = $"Offset: {groundOffset.Value:F2}m";
                                        if (_lblGroundAlt.Text != newText)
                                        {
                                            _lblGroundAlt.Text = newText;
                                            _lblGroundAlt.ForeColor = SUCCESS_COLOR;
                                        }
                                    }

                                    if (_lblDetectionStatus != null && !_lblDetectionStatus.IsDisposed)
                                    {
                                        if (count > 0)
                                        {
                                            string distStr = topDist.HasValue ? $" @ {topDist.Value:F2}m" : "";
                                            _lblDetectionStatus.Text = $"\u25CF {count} circle(s){distStr}";
                                            _lblDetectionStatus.ForeColor = SUCCESS_COLOR;
                                        }
                                        else if (centerDist.HasValue)
                                        {
                                            _lblDetectionStatus.Text = $"\u25CB No detection (crosshair: {centerDist.Value:F2}m)";
                                            _lblDetectionStatus.ForeColor = TEXT_SECONDARY;
                                        }
                                        else
                                        {
                                            _lblDetectionStatus.Text = "\u25CB No detection";
                                            _lblDetectionStatus.ForeColor = TEXT_SECONDARY;
                                        }
                                    }

                                    // Update capture button: green when circles found, amber for crosshair fallback.
                                    // Always enabled \u2014 crosshair fallback handles the no-circle case.
                                    if (_btnCapture != null && !_btnCapture.IsDisposed && _btnCapture.Enabled)
                                    {
                                        if (count > 0)
                                        {
                                            _btnCapture.BackColor = SUCCESS_COLOR;
                                            _btnCapture.Text = $"\u25CF CAPTURE ({count} circle{(count > 1 ? "s" : "")})";
                                        }
                                        else
                                        {
                                            _btnCapture.BackColor = ACCENT_COLOR;
                                            _btnCapture.Text = "CAPTURE (crosshair)";
                                        }
                                    }
                                }));
                            }
                        }
                        catch { }
                    });
                }

                // Poll camera tilt angle every ~2s, staggered from detection poll
                _tiltPollCounter++;
                if (_tiltPollCounter >= 4)
                {
                    _tiltPollCounter = 0;
                    double droneAlt = cs.alt;
                    _ = Task.Run(async () =>
                    {
                        try
                        {
                            var resp = await JetsonApiService.GetAsync("/api/servo/camera/tilt");
                            if (resp.IsSuccessStatusCode)
                            {
                                var body = await resp.Content.ReadAsStringAsync();
                                var data = Newtonsoft.Json.Linq.JObject.Parse(body);
                                float feedbackAngle = (float?)data["feedback_angle"]
                                    ?? (float?)data["angle"] ?? 90f;
                                float pitchDeg = feedbackAngle - 90f;
                                string tiltText;
                                Color tiltColor;
                                if (pitchDeg < -1f)
                                {
                                    double groundDist = droneAlt / Math.Tan(Math.Abs(pitchDeg) * Math.PI / 180.0);
                                    tiltText = $"Tilt: {pitchDeg:F0}\u00B0 | Gnd: {groundDist:F1}m fwd";
                                }
                                else if (pitchDeg > 1f)
                                {
                                    tiltText = $"Tilt: +{pitchDeg:F0}\u00B0 (up)";
                                }
                                else
                                {
                                    tiltText = "Tilt: 0\u00B0 (level)";
                                }
                                tiltColor = TEXT_SECONDARY;
                                this.BeginInvoke(new Action(() =>
                                {
                                    if (_lblTiltInfo != null && !_lblTiltInfo.IsDisposed)
                                    {
                                        _lblTiltInfo.Text = tiltText;
                                        _lblTiltInfo.ForeColor = tiltColor;
                                    }
                                }));
                            }
                        }
                        catch { }
                    });
                }
            }
            catch { }
        }
        
        /// <summary>
        /// Show a context menu on thumbnail click with options to open the image
        /// or view/edit the target description.
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

            var descItem = new ToolStripMenuItem("View / Edit Description");
            descItem.Click += (s, e) => ShowDescriptionPopup(jsonPath, imagePath);
            menu.Items.Add(descItem);

            menu.Show(picBox, location);
        }

        /// <summary>
        /// Show a modal popup displaying the deterministic target description for a
        /// captured image. The operator can manually override the text and save it
        /// back to the local JSON metadata file.
        /// </summary>
        private void ShowDescriptionPopup(string jsonPath, string imagePath)
        {
            string ReadDescription()
            {
                try
                {
                    if (File.Exists(jsonPath))
                    {
                        var json = File.ReadAllText(jsonPath);
                        var data = Newtonsoft.Json.Linq.JObject.Parse(json);
                        return data["relative_description"]?.ToString() ?? "";
                    }
                }
                catch { }
                return "";
            }

            void WriteDescription(string desc)
            {
                try
                {
                    if (File.Exists(jsonPath))
                    {
                        var json = File.ReadAllText(jsonPath);
                        var data = Newtonsoft.Json.Linq.JObject.Parse(json);
                        data["relative_description"] = desc;
                        File.WriteAllText(jsonPath, data.ToString(Newtonsoft.Json.Formatting.Indented));
                    }
                }
                catch { }
            }

            var description = ReadDescription();

            var popup = new Form
            {
                Text = "Target Description - " + Path.GetFileName(imagePath),
                Size = new Size(680, 480),
                MinimumSize = new Size(500, 350),
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
                Text = "Deterministic description from target localizer. Edit to override before submission.",
                AutoSize = true,
                ForeColor = Color.FromArgb(180, 180, 180),
                Padding = new Padding(4, 4, 4, 8),
            };
            layout.Controls.Add(header, 0, 0);

            var txtDesc = new RichTextBox
            {
                Dock = DockStyle.Fill,
                ReadOnly = false,
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = Color.FromArgb(220, 220, 220),
                BorderStyle = BorderStyle.FixedSingle,
                Font = new Font("Segoe UI", 10f),
                WordWrap = true,
                ScrollBars = RichTextBoxScrollBars.Vertical,
                Text = description,
                Margin = new Padding(8, 4, 8, 4),
            };
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

            var btnSave = new Button
            {
                Text = "Save",
                Size = new Size(80, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 150, 100),
                ForeColor = Color.White,
            };
            btnSave.Click += (s, e) =>
            {
                WriteDescription(txtDesc.Text);
                btnSave.Text = "Saved!";
                var t = new System.Windows.Forms.Timer { Interval = 1500 };
                t.Tick += (ts, te) => { btnSave.Text = "Save"; t.Stop(); t.Dispose(); };
                t.Start();
            };

            var btnCopy = new Button
            {
                Text = "Copy",
                Size = new Size(80, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
            };
            btnCopy.Click += (s, e) =>
            {
                try
                {
                    if (!string.IsNullOrEmpty(txtDesc.Text))
                    {
                        Clipboard.SetText(txtDesc.Text);
                        btnCopy.Text = "Copied!";
                        var t = new System.Windows.Forms.Timer { Interval = 1500 };
                        t.Tick += (ts, te) => { btnCopy.Text = "Copy"; t.Stop(); t.Dispose(); };
                        t.Start();
                    }
                }
                catch { }
            };

            btnPanel.Controls.Add(btnClose);
            btnPanel.Controls.Add(btnSave);
            btnPanel.Controls.Add(btnCopy);
            layout.Controls.Add(btnPanel, 0, 2);

            popup.Controls.Add(layout);
            popup.ShowDialog(this);
        }

        /// <summary>
        /// Restore the gallery panel from locally saved captures in Documents\NOMAD\Task1\.
        /// Called once after UI is shown so the window survives reloads.
        /// </summary>
        private async Task RestoreGalleryAsync()
        {
            try
            {
                var task1Dir = Path.Combine(
                    Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "NOMAD", "Task1");
                if (!Directory.Exists(task1Dir))
                    return;

                var jsonFiles = await Task.Run(() =>
                    Directory.GetFiles(task1Dir, "*.json")
                        .Select(p => new { Path = p, Info = new FileInfo(p) })
                        .OrderBy(x => x.Info.LastWriteTime)
                        .ToArray());

                foreach (var jf in jsonFiles)
                {
                    try
                    {
                        var imagePath = Path.ChangeExtension(jf.Path, ".jpg");
                        if (!File.Exists(imagePath))
                            imagePath = Path.ChangeExtension(jf.Path, ".jpeg");
                        if (!File.Exists(imagePath))
                            continue;

                        var jsonText = await Task.Run(() => File.ReadAllText(jf.Path));
                        var meta = Newtonsoft.Json.JsonConvert.DeserializeObject<SnapshotMetadata>(jsonText);

                        var imageBytes = await Task.Run(() => File.ReadAllBytes(imagePath));
                        var jsonPath = jf.Path;
                        var localPath = imagePath;

                        this.BeginInvoke(new Action(() =>
                        {
                            try
                            {
                                Bitmap thumb = new Bitmap(120, 90);
                                using (var ms = new MemoryStream(imageBytes))
                                using (var originalImage = Image.FromStream(ms))
                                using (var g = Graphics.FromImage(thumb))
                                {
                                    g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                                    g.DrawImage(originalImage, 0, 0, 120, 90);
                                }

                                var picBox = new PictureBox
                                {
                                    Image = thumb,
                                    Size = new Size(120, 90),
                                    SizeMode = PictureBoxSizeMode.Zoom,
                                    Margin = new Padding(5),
                                    Cursor = Cursors.Hand,
                                    Tag = new { Path = localPath, JsonPath = jsonPath, Metadata = "" },
                                };

                                picBox.MouseClick += (s, mevt) => ShowImageContextMenu(picBox, mevt.Location);

                                var tooltip = new ToolTip();
                                var lat = meta?.Position?.Lat.ToString("F6") ?? "N/A";
                                var lon = meta?.Position?.Lon.ToString("F6") ?? "N/A";
                                tooltip.SetToolTip(picBox, $"{Path.GetFileName(imagePath)}\nPos: {lat}, {lon}");

                                _galleryPanel.Controls.Add(picBox);
                            }
                            catch { }
                        }));
                    }
                    catch { }
                }
            }
            catch { }
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

        private static async Task SetOverlayModeAsync(string mode)
        {
            try
            {
                await JetsonApiService.PostAsync($"/api/video/overlay/mode?mode={mode}");
            }
            catch (Exception ex) { System.Diagnostics.Debug.WriteLine(ex); }
        }

        private static async Task DisableOverlayDetectorsAsync()
        {
            try
            {
                await JetsonApiService.PostAsync(
                    "/api/video/overlay/detectors?task1=false&task2=false");
            }
            catch (Exception ex) { System.Diagnostics.Debug.WriteLine(ex); }
        }

        protected override void OnVisibleChanged(EventArgs e)
        {
            base.OnVisibleChanged(e);
            if (Visible)
            {
                // Re-pin task1 in case another view switched the bridge while
                // this one was hidden.
                _ = SetOverlayModeAsync("task1");
            }
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                // Best-effort: leave the bridge with both detectors off so a
                // stale detector doesn't keep running after the view closes.
                _ = DisableOverlayDetectorsAsync();
                _videoPlayer?.Dispose();
                _payloadControl?.Dispose();
                _uploadPanel?.Dispose();
                _tabControl?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
