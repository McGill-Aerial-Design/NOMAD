// ============================================================
// NOMAD Control Panel - Flight Data Tab UI
// ============================================================
// Custom control panel added to Mission Planner's Flight Data screen.
// Provides buttons for Task 1 and Task 2 operations.
// Includes Indoor Nudge controls for manual velocity commands.
// Includes RTSP video streaming from Jetson.
// ============================================================

using System;
using System.Diagnostics;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Comms;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Control panel for NOMAD operations in Mission Planner.
    /// </summary>
    public class NOMADControlPanel : UserControl
    {
        // ============================================================
        // Fields
        // ============================================================

        private readonly DualLinkSender _sender;
        private NOMADConfig _config;

        // New Feature Instances
        private TelemetryInjector _telemetryInjector;
        private WASDNudgeControl _wasdControl;
        private JetsonHealthTab _healthTab;

        // UI Controls
        private Label _lblTitle;
        private Label _lblStatus;
        private Label _lblMode;
        private Label _lblPosition;

        // Task 1 Controls
        private GroupBox _grpTask1;
        private Button _btnTask1Capture;
        private TextBox _txtTask1Result;

        // Task 2 Controls
        private GroupBox _grpTask2;
        private Button _btnTask2Reset;
        private Button _btnTask2Hit;
        private NumericUpDown _numX, _numY, _numZ;
        private Label _lblExclusionCount;

        // Indoor Nudge Controls (WASD)
        private GroupBox _grpNudge;
        private CheckBox _chkEnableWASD;
        private Label _lblWASDStatus;
        private NumericUpDown _numNudgeSpeed;

        // Video Stream Controls
        private GroupBox _grpVideo;
        private Button _btnOpenPrimaryVideo;
        private Button _btnOpenSecondaryVideo;
        private Label _lblVideoStatus;
        private ComboBox _cmbVideoPlayer;

        // Nudge State
        private bool _nudgeEnabled = false;
        private const float DEFAULT_NUDGE_SPEED = 0.5f;  // m/s (safe indoor speed)

        // ============================================================
        // Constructor
        // ============================================================

        public NOMADControlPanel(DualLinkSender sender, NOMADConfig config)
        {
            _sender = sender ?? throw new ArgumentNullException(nameof(sender));
            _config = config ?? throw new ArgumentNullException(nameof(config));

            // Initialize new features
            try
            {
                _telemetryInjector = new TelemetryInjector(null); // Will use MainV2.comPort internally
                _wasdControl = new WASDNudgeControl(null); // Will use MainV2.comPort internally
                _wasdControl.NudgeSpeed = DEFAULT_NUDGE_SPEED;
                
                _healthTab = new JetsonHealthTab(); // Initialize health tab
                
                // Send initial status
                _telemetryInjector?.SendCustomStatus("Control Panel Loaded");
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD: Feature init error - {ex.Message}");
            }

            InitializeComponents();
            UpdateModeDisplay();

            // Enable keyboard input for WASD controls
            this.SetStyle(ControlStyles.Selectable, true);
            this.TabStop = true;
        }

        // ============================================================
        // Public Methods
        // ============================================================

        /// <summary>
        /// Update status display with current vehicle state.
        /// </summary>
        public void UpdateStatus(bool connected, double lat, double lng, double alt)
        {
            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => UpdateStatus(connected, lat, lng, alt)));
                return;
            }

            _lblStatus.Text = connected ? "[*] Connected" : "[ ] Disconnected";
            _lblStatus.ForeColor = connected ? Color.LimeGreen : Color.Red;
            _lblPosition.Text = connected
                ? $"Position: {lat:F6}, {lng:F6} @ {alt:F1}m"
                : "Position: --";
        }

        /// <summary>
        /// Update configuration reference.
        /// </summary>
        public void UpdateConfig(NOMADConfig config)
        {
            _config = config;
            UpdateModeDisplay();
        }

        // ============================================================
        // UI Initialization
        // ============================================================

        private void InitializeComponents()
        {
            this.BackColor = Color.FromArgb(45, 45, 48);
            this.Padding = new Padding(10);
            this.AutoScroll = true;  // Enable vertical scrolling
            this.AutoScrollMinSize = new Size(350, 1200);  // Set minimum size for scroll

            int yOffset = 10;

            // Title
            _lblTitle = new Label
            {
                Text = "NOMAD Control",
                Font = new Font("Segoe UI", 16, FontStyle.Bold),
                ForeColor = Color.FromArgb(0, 122, 204),
                Location = new Point(10, yOffset),
                AutoSize = true
            };
            this.Controls.Add(_lblTitle);
            yOffset += 40;

            // Status Row
            var pnlStatus = new FlowLayoutPanel
            {
                Location = new Point(10, yOffset),
                Size = new Size(350, 25),
                FlowDirection = FlowDirection.LeftToRight,
                BackColor = Color.Transparent
            };

            _lblStatus = new Label
            {
                Text = "[ ] Disconnected",
                ForeColor = Color.Red,
                AutoSize = true,
                Font = new Font("Segoe UI", 10)
            };
            pnlStatus.Controls.Add(_lblStatus);

            _lblMode = new Label
            {
                Text = "| Mode: HTTP",
                ForeColor = Color.Gray,
                AutoSize = true,
                Font = new Font("Segoe UI", 10),
                Margin = new Padding(20, 0, 0, 0)
            };
            pnlStatus.Controls.Add(_lblMode);

            this.Controls.Add(pnlStatus);
            yOffset += 30;

            // Position Label
            _lblPosition = new Label
            {
                Text = "Position: --",
                ForeColor = Color.LightGray,
                Font = new Font("Consolas", 9),
                Location = new Point(10, yOffset),
                AutoSize = true
            };
            this.Controls.Add(_lblPosition);
            yOffset += 30;

            // ============================================================
            // Task 1: Recon
            // ============================================================

            _grpTask1 = new GroupBox
            {
                Text = "Task 1: Recon (Outdoor)",
                ForeColor = Color.FromArgb(0, 150, 200),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(350, 150),
                BackColor = Color.FromArgb(55, 55, 58)
            };

            _btnTask1Capture = new Button
            {
                Text = "[CAP] Capture Snapshot",
                Location = new Point(15, 25),
                Size = new Size(320, 40),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand
            };
            _btnTask1Capture.FlatAppearance.BorderSize = 0;
            _btnTask1Capture.Click += BtnTask1Capture_Click;
            _grpTask1.Controls.Add(_btnTask1Capture);

            var lblResult = new Label
            {
                Text = "Result:",
                Location = new Point(15, 75),
                ForeColor = Color.Gray,
                AutoSize = true
            };
            _grpTask1.Controls.Add(lblResult);

            _txtTask1Result = new TextBox
            {
                Location = new Point(15, 95),
                Size = new Size(320, 40),
                Multiline = true,
                ReadOnly = true,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.LimeGreen,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.FixedSingle
            };
            _grpTask1.Controls.Add(_txtTask1Result);

            this.Controls.Add(_grpTask1);
            yOffset += 160;

            // ============================================================
            // Task 2: Extinguish
            // ============================================================

            _grpTask2 = new GroupBox
            {
                Text = "Task 2: Extinguish (Indoor)",
                ForeColor = Color.FromArgb(255, 150, 50),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(350, 180),
                BackColor = Color.FromArgb(55, 55, 58)
            };

            _btnTask2Reset = new Button
            {
                Text = "[CLR] Reset Exclusion Map",
                Location = new Point(15, 25),
                Size = new Size(320, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(200, 50, 50),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Cursor = Cursors.Hand
            };
            _btnTask2Reset.FlatAppearance.BorderSize = 0;
            _btnTask2Reset.Click += BtnTask2Reset_Click;
            _grpTask2.Controls.Add(_btnTask2Reset);

            _lblExclusionCount = new Label
            {
                Text = "Targets: 0",
                Location = new Point(15, 65),
                ForeColor = Color.Gray,
                AutoSize = true
            };
            _grpTask2.Controls.Add(_lblExclusionCount);

            // Manual Hit Registration
            var lblManualHit = new Label
            {
                Text = "Manual Hit (X, Y, Z):",
                Location = new Point(15, 90),
                ForeColor = Color.Gray,
                AutoSize = true
            };
            _grpTask2.Controls.Add(lblManualHit);

            _numX = CreateNumericUpDown(15, 110);
            _numY = CreateNumericUpDown(90, 110);
            _numZ = CreateNumericUpDown(165, 110);
            _grpTask2.Controls.Add(_numX);
            _grpTask2.Controls.Add(_numY);
            _grpTask2.Controls.Add(_numZ);

            _btnTask2Hit = new Button
            {
                Text = "Hit",
                Location = new Point(245, 110),
                Size = new Size(90, 25),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(255, 150, 50),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
                Cursor = Cursors.Hand
            };
            _btnTask2Hit.FlatAppearance.BorderSize = 0;
            _btnTask2Hit.Click += BtnTask2Hit_Click;
            _grpTask2.Controls.Add(_btnTask2Hit);

            this.Controls.Add(_grpTask2);
            yOffset += 190;

            // ============================================================
            // Indoor Nudge Controls
            // ============================================================

            _grpNudge = new GroupBox
            {
                Text = "Indoor Nudge (Task 2)",
                ForeColor = Color.FromArgb(100, 200, 100),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(350, 140),
                BackColor = Color.FromArgb(55, 55, 58)
            };

            _chkEnableWASD = new CheckBox
            {
                Text = "Enable WASD Indoor Control",
                Location = new Point(15, 25),
                Size = new Size(320, 25),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10),
                Checked = false
            };
            _chkEnableWASD.CheckedChanged += ChkEnableWASD_CheckedChanged;
            _grpNudge.Controls.Add(_chkEnableWASD);

            var lblSpeed = new Label
            {
                Text = "Nudge Speed (m/s):",
                Location = new Point(15, 55),
                ForeColor = Color.Gray,
                AutoSize = true
            };
            _grpNudge.Controls.Add(lblSpeed);

            _numNudgeSpeed = new NumericUpDown
            {
                Location = new Point(140, 52),
                Size = new Size(70, 25),
                DecimalPlaces = 2,
                Minimum = 0.1M,
                Maximum = 2.0M,
                Increment = 0.1M,
                Value = (decimal)DEFAULT_NUDGE_SPEED,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            _grpNudge.Controls.Add(_numNudgeSpeed);

            var lblNudgeHelp = new Label
            {
                Text = "W=Forward  S=Back  A=Left  D=Right",
                Location = new Point(15, 85),
                ForeColor = Color.Gray,
                Font = new Font("Consolas", 9),
                AutoSize = true
            };
            _grpNudge.Controls.Add(lblNudgeHelp);

            _lblWASDStatus = new Label
            {
                Text = "Status: Disabled",
                Location = new Point(15, 110),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 9),
                AutoSize = true
            };
            _grpNudge.Controls.Add(_lblWASDStatus);

            this.Controls.Add(_grpNudge);
            yOffset += 150;

            // ============================================================
            // Video Streaming Controls
            // ============================================================

            _grpVideo = new GroupBox
            {
                Text = "Video Streams (RTSP)",
                ForeColor = Color.FromArgb(200, 100, 200),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Location = new Point(10, yOffset),
                Size = new Size(350, 130),
                BackColor = Color.FromArgb(55, 55, 58)
            };

            _btnOpenPrimaryVideo = new Button
            {
                Text = "[VID] Open Primary (ZED/Nav)",
                Location = new Point(15, 25),
                Size = new Size(155, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(100, 50, 150),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Cursor = Cursors.Hand
            };
            _btnOpenPrimaryVideo.FlatAppearance.BorderSize = 0;
            _btnOpenPrimaryVideo.Click += BtnOpenPrimaryVideo_Click;
            _grpVideo.Controls.Add(_btnOpenPrimaryVideo);

            _btnOpenSecondaryVideo = new Button
            {
                Text = "[TGT] Open Secondary (Gimbal)",
                Location = new Point(180, 25),
                Size = new Size(155, 35),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(150, 100, 50),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Cursor = Cursors.Hand
            };
            _btnOpenSecondaryVideo.FlatAppearance.BorderSize = 0;
            _btnOpenSecondaryVideo.Click += BtnOpenSecondaryVideo_Click;
            _grpVideo.Controls.Add(_btnOpenSecondaryVideo);

            var lblPlayer = new Label
            {
                Text = "Video Player:",
                Location = new Point(15, 70),
                ForeColor = Color.Gray,
                AutoSize = true
            };
            _grpVideo.Controls.Add(lblPlayer);

            _cmbVideoPlayer = new ComboBox
            {
                Location = new Point(100, 67),
                Size = new Size(150, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
            _cmbVideoPlayer.Items.AddRange(new object[] { "VLC", "FFplay", "System Default" });
            _cmbVideoPlayer.SelectedIndex = 0;
            _grpVideo.Controls.Add(_cmbVideoPlayer);

            _lblVideoStatus = new Label
            {
                Text = "Status: Ready",
                Location = new Point(15, 100),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 9),
                AutoSize = true
            };
            _grpVideo.Controls.Add(_lblVideoStatus);

            this.Controls.Add(_grpVideo);
            yOffset += 140;

            // ============================================================
            // Jetson Health Monitor Tab
            // ============================================================

            if (_healthTab != null)
            {
                _healthTab.Location = new Point(10, yOffset);
                _healthTab.Size = new Size(350, 450);
                _healthTab.SetJetsonUrl(_config?.JetsonBaseUrl ?? "http://127.0.0.1:8000");
                this.Controls.Add(_healthTab);
            }
        }

        private NumericUpDown CreateNumericUpDown(int x, int y)
        {
            return new NumericUpDown
            {
                Location = new Point(x, y),
                Size = new Size(70, 25),
                DecimalPlaces = 2,
                Minimum = -100,
                Maximum = 100,
                Value = 0,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White
            };
        }

        private void UpdateModeDisplay()
        {
            if (_lblMode == null) return;

            var mode = _config.UseELRS ? "ELRS/MAVLink" : "HTTP";
            _lblMode.Text = $"| Mode: {mode}";
            _lblMode.ForeColor = _config.UseELRS ? Color.Orange : Color.LightBlue;
        }

        // ============================================================
        // Event Handlers
        // ============================================================

        private async void BtnTask1Capture_Click(object sender, EventArgs e)
        {
            _btnTask1Capture.Enabled = false;
            _btnTask1Capture.Text = "Capturing...";
            _txtTask1Result.Text = "";
            
            // Send telemetry
            _telemetryInjector?.SendTaskStatus(1, "Capturing");

            try
            {
                var result = await _sender.SendTask1Capture();

                if (result.Success)
                {
                    // Try to extract target_text from response
                    try
                    {
                        dynamic json = Newtonsoft.Json.JsonConvert.DeserializeObject(result.Data);
                        _txtTask1Result.Text = json?.target_text ?? "Capture successful";
                        _txtTask1Result.ForeColor = Color.LimeGreen;
                        
                        // Send success telemetry
                        _telemetryInjector?.SendTaskStatus(1, "Snapshot Captured");
                    }
                    catch
                    {
                        _txtTask1Result.Text = result.Message;
                        _txtTask1Result.ForeColor = Color.LimeGreen;
                        _telemetryInjector?.SendTaskStatus(1, "Captured");
                    }
                }
                else
                {
                    _txtTask1Result.Text = result.Message;
                    _txtTask1Result.ForeColor = Color.Red;
                }
            }
            catch (Exception ex)
            {
                _txtTask1Result.Text = $"Error: {ex.Message}";
                _txtTask1Result.ForeColor = Color.Red;
            }
            finally
            {
                _btnTask1Capture.Enabled = true;
                _btnTask1Capture.Text = "[CAP] Capture Snapshot";
            }
        }

        private async void BtnTask2Reset_Click(object sender, EventArgs e)
        {
            var confirm = MessageBox.Show(
                "Reset the exclusion map?\nAll stored targets will be cleared.",
                "Confirm Reset",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning
            );

            if (confirm != DialogResult.Yes) return;

            _btnTask2Reset.Enabled = false;
            
            // Send telemetry
            _telemetryInjector?.SendTaskStatus(2, "Resetting Map");

            try
            {
                var result = await _sender.SendTask2ResetMap();

                if (result.Success)
                {
                    _lblExclusionCount.Text = "Targets: 0";
                    _lblExclusionCount.ForeColor = Color.LimeGreen;
                    _telemetryInjector?.SendTaskStatus(2, "Map Cleared");
                    MessageBox.Show("Exclusion map cleared!", "Success", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                else
                {
                    _telemetryInjector?.SendCustomStatus("Map Reset Failed");
                    MessageBox.Show($"Reset failed: {result.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                _btnTask2Reset.Enabled = true;
            }
        }

        private async void BtnTask2Hit_Click(object sender, EventArgs e)
        {
            _btnTask2Hit.Enabled = false;

            try
            {
                var result = await _sender.SendTask2TargetHit(
                    (float)_numX.Value,
                    (float)_numY.Value,
                    (float)_numZ.Value
                );

                if (result.Success)
                {
                    try
                    {
                        dynamic json = Newtonsoft.Json.JsonConvert.DeserializeObject(result.Data);
                        int total = json?.total_targets ?? 0;
                        _lblExclusionCount.Text = $"Targets: {total}";
                        _lblExclusionCount.ForeColor = Color.Orange;
                    }
                    catch
                    {
                        _lblExclusionCount.ForeColor = Color.LimeGreen;
                    }
                }
                else
                {
                    MessageBox.Show($"Hit registration failed: {result.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                _btnTask2Hit.Enabled = true;
            }
        }

        // ============================================================
        // WASD Control Event Handlers
        // ============================================================

        private void ChkEnableWASD_CheckedChanged(object sender, EventArgs e)
        {
            _nudgeEnabled = _chkEnableWASD.Checked;
            
            if (_wasdControl != null)
            {
                _wasdControl.Enabled = _chkEnableWASD.Checked;
            }

            if (_nudgeEnabled)
            {
                _lblWASDStatus.Text = "Status: ENABLED - W/A/S/D/Q/E to nudge";
                _lblWASDStatus.ForeColor = Color.LimeGreen;
                _chkEnableWASD.ForeColor = Color.LimeGreen;

                // Send telemetry status
                _telemetryInjector?.SendCustomStatus("WASD Control Enabled");

                // Show warning message
                CustomMessageBox.Show(
                    "WASD Control Enabled\n\n" +
                    "W/S = Forward/Back\n" +
                    "A/D = Left/Right\n" +
                    "Q/E = Up/Down\n\n" +
                    "CAUTION: Use only in Guided mode!\n" +
                    "Keep RC transmitter ready for manual override.",
                    "WASD Control",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Warning
                );

                // Focus this control to receive keyboard events
                this.Focus();
            }
            else
            {
                _lblWASDStatus.Text = "Status: Disabled";
                _lblWASDStatus.ForeColor = Color.Gray;
                _chkEnableWASD.ForeColor = Color.White;

                // Send telemetry status
                _telemetryInjector?.SendCustomStatus("WASD Control Disabled");
            }
        }

        // Override keyboard handling for WASD
        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            if (_wasdControl?.Enabled == true)
            {
                // Handle WASD Q E keys
                if (keyData == Keys.W || keyData == Keys.A || 
                    keyData == Keys.S || keyData == Keys.D || 
                    keyData == Keys.Q || keyData == Keys.E)
                {
                    _wasdControl.HandleKeyDown(keyData);
                    return true; // Handled
                }
            }
            return base.ProcessCmdKey(ref msg, keyData);
        }

        protected override void OnKeyUp(KeyEventArgs e)
        {
            if (_wasdControl?.Enabled == true)
            {
                _wasdControl.HandleKeyUp(e.KeyCode);
            }
            base.OnKeyUp(e);
        }

        // ============================================================
        // Video Streaming Event Handlers
        // ============================================================

        private void BtnOpenPrimaryVideo_Click(object sender, EventArgs e)
        {
            OpenVideoStream(_config.RtspUrlPrimary, "Primary (ZED/Navigation)");
        }

        private void BtnOpenSecondaryVideo_Click(object sender, EventArgs e)
        {
            OpenVideoStream(_config.RtspUrlSecondary, "Secondary (Gimbal/Targeting)");
        }

        /// <summary>
        /// Open an RTSP video stream in the selected video player.
        /// VLC or FFplay are recommended for low-latency RTSP playback.
        /// </summary>
        /// <param name="rtspUrl">RTSP stream URL from Jetson</param>
        /// <param name="streamName">Display name for status updates</param>
        private void OpenVideoStream(string rtspUrl, string streamName)
        {
            try
            {
                _lblVideoStatus.Text = $"Opening {streamName}...";
                _lblVideoStatus.ForeColor = Color.Yellow;

                string player = _cmbVideoPlayer.SelectedItem?.ToString() ?? "VLC";
                ProcessStartInfo psi = null;

                switch (player)
                {
                    case "VLC":
                        // VLC with low-latency settings for RTSP
                        psi = new ProcessStartInfo
                        {
                            FileName = "vlc",
                            Arguments = $"--network-caching=100 --rtsp-tcp \"{rtspUrl}\"",
                            UseShellExecute = true
                        };
                        break;

                    case "FFplay":
                        // FFplay with low-latency flags
                        psi = new ProcessStartInfo
                        {
                            FileName = "ffplay",
                            Arguments = $"-fflags nobuffer -flags low_delay -rtsp_transport tcp \"{rtspUrl}\"",
                            UseShellExecute = true
                        };
                        break;

                    case "System Default":
                    default:
                        // Open with system default handler
                        psi = new ProcessStartInfo
                        {
                            FileName = rtspUrl,
                            UseShellExecute = true
                        };
                        break;
                }

                Process.Start(psi);

                _lblVideoStatus.Text = $"Opened: {streamName}";
                _lblVideoStatus.ForeColor = Color.LimeGreen;
            }
            catch (Exception ex)
            {
                _lblVideoStatus.Text = $"Error: {ex.Message}";
                _lblVideoStatus.ForeColor = Color.Red;
                MessageBox.Show(
                    $"Failed to open video stream.\n\n" +
                    $"URL: {rtspUrl}\n" +
                    $"Error: {ex.Message}\n\n" +
                    "Make sure VLC or FFplay is installed and in your PATH.",
                    "Video Stream Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
        }
    }
}
