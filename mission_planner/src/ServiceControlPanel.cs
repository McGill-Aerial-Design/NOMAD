using System;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Service Control Panel for NOMAD - manages Jetson services including:
    /// - MAVLink Router
    /// - MediaMTX (RTSP Server)
    /// - noVNC
    /// - Edge Core
    /// - Isaac ROS Container
    /// - VIO Pipeline Status
    /// </summary>
    public class ServiceControlPanel : UserControl
    {
        private readonly DualLinkSender _sender;
        private readonly System.Threading.Timer _pollTimer;
        private readonly int _pollIntervalMs;
        private int _isPolling = 0;
        private int _pollCycle = 0;

        // Track transient failures so we can avoid UI flapping and log spam.
        private int _servicesFailStreak = 0;
        private int _isaacFailStreak = 0;
        private int _vioFailStreak = 0;
        private int _videoFailStreak = 0;
        private int _slamFailStreak = 0;

        private static bool ShouldLogStreak(int streak)
        {
            // Ignore one-off timeouts/cancels. Log only when persistent.
            return streak >= 5 && (streak == 5 || streak % 10 == 0);
        }
        
        // Service status indicators
        private Label _lblMavlinkStatus;
        private Label _lblMediamtxStatus;
        private Label _lblNoVncStatus;
        private Label _lblEdgeCoreStatus;
        private Label _lblIsaacRosStatus;
        private Label _lblTargetLocalizerStatus;
        private Label _lblVioStatus;
        
        // Service control buttons
        private Button _btnMavlinkRestart;
        private Button _btnMediamtxRestart;
        private Button _btnEdgeCoreRestart;
        private Button _btnNoVncStart;
        private Button _btnNoVncStop;
        private Button _btnIsaacRosStart;
        private Button _btnVioReset;
        private Button _btnTargetLocalizerStart;
        private Button _btnTargetLocalizerStop;
        
        // VIO trajectory info
        private Label _lblVioTrajectoryPoints;
        private Button _btnClearTrajectory;
        
        // ROS HTTP Bridge
        private Label _lblRosBridgeStatus;
        private Button _btnStartRosBridge;
        private Button _btnStopRosBridge;

        // Nvblox
        private Label _lblNvbloxStatus;
        private Button _btnNvbloxLaunch;
        private Button _btnNvbloxStop;

        // Video bridges
        private Label _lblVideoBridgesStatus;
        private Button _btnStartBridges;

        // SLAM
        private Label _lblSlamStatus;
        private Button _btnStopSlam;

        // Detector overlays — let the operator turn each circle detector
        // on/off independently to free CPU when one isn't needed.
        private CheckBox _chkDetectorTask1;
        private CheckBox _chkDetectorTask2;
        private Label    _lblDetectorStatus;
        
        // Status text
        private Label _lblLastUpdate;
        private TextBox _txtLog;
        private Panel _servicesPanel;
        private Panel _logPanel;

        // Left service pane layout constants
        private const int ServiceLeftCol = 15;
        private const int ServiceStatusCol = 140;
        private const int ServiceStatusWidth = 280;
        private const int ServiceActionCol = 430;
        private const int ServiceStartCol = 385;
        private const int ServiceStopCol = 460;
        
        public ServiceControlPanel(DualLinkSender sender, int pollIntervalMs = 3000)
        {
            _sender = sender ?? throw new ArgumentNullException(nameof(sender));
            // Service Control performs multiple network calls per cycle.
            // Clamp to a sane minimum to avoid timeout churn under load.
            _pollIntervalMs = Math.Max(5000, pollIntervalMs);
            
            InitializeUI();
            
            // Poll using configured interval
            _pollTimer = new System.Threading.Timer(
                _ => PollServicesAsync(),
                null,
                TimeSpan.FromSeconds(2),
                TimeSpan.FromMilliseconds(_pollIntervalMs)
            );

            LogMessage("Service poller v2 active (stability patch loaded)");
        }
        
        private void InitializeUI()
        {
            this.BackColor = Color.FromArgb(45, 45, 48);
            this.Dock = DockStyle.Fill;
            this.Size = new Size(920, 650);
            this.MinimumSize = new Size(860, 550);
            this.AutoScroll = false;

            var rootLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                BackColor = Color.FromArgb(45, 45, 48),
                Margin = new Padding(0),
                Padding = new Padding(0),
            };
            rootLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 560));
            rootLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));

            _servicesPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                AutoScroll = true,
                Padding = new Padding(0),
            };

            _logPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(38, 38, 42),
                Padding = new Padding(10),
            };

            rootLayout.Controls.Add(_servicesPanel, 0, 0);
            rootLayout.Controls.Add(_logPanel, 1, 0);
            this.Controls.Add(rootLayout);
            
            int yOffset = 10;
            int leftCol = ServiceLeftCol;
            int rightCol = ServiceActionCol;
            
            // Title
            var lblTitle = new Label
            {
                Text = "NOMAD Service Control",
                Location = new Point(leftCol, yOffset),
                Size = new Size(520, 25),
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = Color.White,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(lblTitle);
            yOffset += 35;
            
            // === MAVLink Router ===
            AddServiceRow("MAVLink Router", ref _lblMavlinkStatus, ref _btnMavlinkRestart, ref yOffset);
            _btnMavlinkRestart.Click += async (s, e) => await RestartServiceAsync("mavlink-router", _lblMavlinkStatus);
            
            // === MediaMTX ===
            AddServiceRow("MediaMTX (RTSP)", ref _lblMediamtxStatus, ref _btnMediamtxRestart, ref yOffset);
            _btnMediamtxRestart.Click += async (s, e) => await RestartServiceAsync("mediamtx", _lblMediamtxStatus);

            // === noVNC (with Start/Stop) ===
            AddNoVncRow(ref yOffset);
            
            // === NOMAD Services (Full Restart) ===
            AddServiceRow("NOMAD Services", ref _lblEdgeCoreStatus, ref _btnEdgeCoreRestart, ref yOffset, "Restart All");
            _btnEdgeCoreRestart.Click += async (s, e) => await RestartAllServicesAsync();
            
            // === Isaac ROS (with Start/Stop) ===
            AddIsaacRosRow(ref yOffset);

            // === ROS HTTP Bridge (with Start/Stop) ===
            AddRosBridgeRow(ref yOffset);

            // === Target Localizer (Task 1 capture backend) ===
            AddTargetLocalizerRow(ref yOffset);

            // === Nvblox (with Launch/Stop) ===
            AddNvbloxRow(ref yOffset);

            // === Video Bridges ===
            AddServiceRow("Video Bridges", ref _lblVideoBridgesStatus, ref _btnStartBridges, ref yOffset, "Start");
            _btnStartBridges.Click += async (s, e) => await StartVideoBridgesAsync();
            
            // === SLAM Service ===
            AddServiceRow("SLAM / Mesh", ref _lblSlamStatus, ref _btnStopSlam, ref yOffset, "Stop SLAM");
            _btnStopSlam.Click += async (s, e) => await StopSlamAsync();

            // === Circle Detector Toggles ===
            yOffset += 8;
            var lblDetectorTitle = new Label
            {
                Text = "Circle Detectors:",
                Location = new Point(ServiceLeftCol, yOffset),
                Size = new Size(110, 22),
                ForeColor = Color.LightGray,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            _servicesPanel.Controls.Add(lblDetectorTitle);

            // Checkboxes shift right of the title label so the indicator boxes
            // aren't hidden behind the (transparent but click-blocking) label
            // bounds. Spacing here matches the screenshot-driven layout.
            int detectorCol = ServiceLeftCol + 130;
            _chkDetectorTask1 = new CheckBox
            {
                Text = "Task 1 (HSV color)",
                Location = new Point(detectorCol, yOffset + 2),
                Size = new Size(150, 20),
                ForeColor = Color.LightGray,
                BackColor = Color.Transparent,
            };
            _chkDetectorTask1.CheckedChanged += async (s, e) => await PushDetectorState();
            _servicesPanel.Controls.Add(_chkDetectorTask1);

            _chkDetectorTask2 = new CheckBox
            {
                Text = "Task 2 (shape)",
                Location = new Point(detectorCol + 155, yOffset + 2),
                Size = new Size(130, 20),
                ForeColor = Color.LightGray,
                BackColor = Color.Transparent,
            };
            _chkDetectorTask2.CheckedChanged += async (s, e) => await PushDetectorState();
            _servicesPanel.Controls.Add(_chkDetectorTask2);

            yOffset += 26;
            _lblDetectorStatus = new Label
            {
                Text = "—",
                Location = new Point(ServiceStatusCol, yOffset),
                Size = new Size(380, 18),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
            };
            _servicesPanel.Controls.Add(_lblDetectorStatus);
            yOffset += 22;
            _ = RefreshDetectorState();
            
            // Separator
            yOffset += 10;
            var separator = new Label
            {
                BorderStyle = BorderStyle.Fixed3D,
                Location = new Point(leftCol, yOffset),
                Size = new Size(520, 2),
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(separator);
            yOffset += 15;
            
            // === VIO Status Section ===
            var lblVioTitle = new Label
            {
                Text = "VIO / VSLAM Status",
                Location = new Point(leftCol, yOffset),
                Size = new Size(200, 20),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = Color.LightBlue
            };
            _servicesPanel.Controls.Add(lblVioTitle);
            yOffset += 25;
            
            // VIO Status
            var lblVioLabel = new Label
            {
                Text = "Status:",
                Location = new Point(leftCol, yOffset),
                Size = new Size(80, 20),
                ForeColor = Color.LightGray
            };
            _servicesPanel.Controls.Add(lblVioLabel);
            
            _lblVioStatus = new Label
            {
                Text = "Unknown",
                Location = new Point(ServiceStatusCol, yOffset),
                Size = new Size(ServiceStatusWidth, 20),
                ForeColor = Color.Yellow,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblVioStatus);
            
            _btnVioReset = new Button
            {
                Text = "Reset Origin",
                Location = new Point(rightCol, yOffset - 3),
                Size = new Size(100, 25),
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
            };
            _btnVioReset.Click += async (s, e) => await ResetVioOriginAsync();
            _servicesPanel.Controls.Add(_btnVioReset);
            yOffset += 30;
            
            // VIO Trajectory Points
            var lblTrajLabel = new Label
            {
                Text = "Trajectory:",
                Location = new Point(leftCol, yOffset),
                Size = new Size(80, 20),
                ForeColor = Color.LightGray
            };
            _servicesPanel.Controls.Add(lblTrajLabel);
            
            _lblVioTrajectoryPoints = new Label
            {
                Text = "0 points",
                Location = new Point(ServiceStatusCol, yOffset),
                Size = new Size(ServiceStatusWidth, 20),
                ForeColor = Color.White,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblVioTrajectoryPoints);
            
            _btnClearTrajectory = new Button
            {
                Text = "Clear",
                Location = new Point(rightCol, yOffset - 3),
                Size = new Size(100, 25),
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
            };
            _btnClearTrajectory.Click += async (s, e) => await ClearTrajectoryAsync();
            _servicesPanel.Controls.Add(_btnClearTrajectory);
            yOffset += 40;
            
            // Last update
            _lblLastUpdate = new Label
            {
                Text = "Last update: Never",
                Location = new Point(leftCol, yOffset),
                Size = new Size(520, 20),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblLastUpdate);

            var lblLogTitle = new Label
            {
                Text = "Activity Log:",
                Dock = DockStyle.Top,
                Height = 22,
                ForeColor = Color.LightGray,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleLeft,
            };
            
            _txtLog = new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ScrollBars = ScrollBars.Vertical,
                ReadOnly = true,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.LightGreen,
                Font = new Font("Consolas", 8),
            };

            _logPanel.Controls.Add(_txtLog);
            _logPanel.Controls.Add(lblLogTitle);
        }
        
        // Suppresses CheckedChanged → PushDetectorState while we mirror server
        // state into the checkboxes from a periodic poll.
        private bool _suppressDetectorPush;

        private async Task PushDetectorState()
        {
            if (_suppressDetectorPush) return;
            try
            {
                bool t1 = _chkDetectorTask1?.Checked ?? false;
                bool t2 = _chkDetectorTask2?.Checked ?? false;
                bool any = t1 || t2;

                // Make sure overlay is enabled when at least one detector is on,
                // and disabled (saves CPU) when both are off.
                if (any)
                {
                    // /overlay/detectors enables the overlay with the exact
                    // requested detector mask. Calling /overlay/enable first
                    // briefly selects the legacy Task 1 detector and can draw
                    // false HSV targets for a few frames.
                    var resp = await JetsonApiService.PostAsync(
                        $"/api/video/overlay/detectors?task1={(t1 ? "true" : "false")}&task2={(t2 ? "true" : "false")}");
                    SetDetectorStatus(resp.IsSuccessStatusCode
                        ? $"Active — task1={t1} task2={t2}"
                        : $"HTTP {(int)resp.StatusCode}");
                }
                else
                {
                    var resp = await JetsonApiService.PostAsync("/api/video/overlay/disable");
                    SetDetectorStatus(resp.IsSuccessStatusCode
                        ? "Both detectors disabled"
                        : $"HTTP {(int)resp.StatusCode}");
                }
            }
            catch (Exception ex)
            {
                SetDetectorStatus($"Error: {ex.Message}");
            }
        }

        private async Task RefreshDetectorState()
        {
            try
            {
                var resp = await JetsonApiService.GetAsync("/api/video/overlay/status");
                if (!resp.IsSuccessStatusCode) return;
                var json = JObject.Parse(await resp.Content.ReadAsStringAsync());
                bool t1 = json["task1_enabled"]?.Value<bool>() ?? false;
                bool t2 = json["task2_enabled"]?.Value<bool>() ?? false;
                BeginInvoke(new Action(() =>
                {
                    _suppressDetectorPush = true;
                    try
                    {
                        if (_chkDetectorTask1 != null) _chkDetectorTask1.Checked = t1;
                        if (_chkDetectorTask2 != null) _chkDetectorTask2.Checked = t2;
                    }
                    finally { _suppressDetectorPush = false; }
                    SetDetectorStatus(t1 || t2
                        ? $"Active — task1={t1} task2={t2}"
                        : "Both detectors disabled");
                }));
            }
            catch { }
        }

        private void SetDetectorStatus(string text)
        {
            if (_lblDetectorStatus == null) return;
            if (InvokeRequired) { BeginInvoke(new Action(() => SetDetectorStatus(text))); return; }
            _lblDetectorStatus.Text = text;
        }

        private void AddServiceRow(string serviceName, ref Label statusLabel, ref Button actionButton, ref int yOffset, string buttonText = "Restart")
        {
            int leftCol = ServiceLeftCol;
            int rightCol = ServiceActionCol;
            
            var lblName = new Label
            {
                Text = serviceName + ":",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(120, 20),
                ForeColor = Color.LightGray
            };
            _servicesPanel.Controls.Add(lblName);
            
            statusLabel = new Label
            {
                Text = "Checking...",
                Location = new Point(ServiceStatusCol, yOffset + 3),
                Size = new Size(ServiceStatusWidth, 20),
                ForeColor = Color.Yellow,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(statusLabel);
            
            actionButton = new Button
            {
                Text = buttonText,
                Location = new Point(rightCol, yOffset),
                Size = new Size(100, 25),
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(actionButton);
            
            yOffset += 35;
        }
        
        private Button _btnIsaacRosStop;
        
        private void AddIsaacRosRow(ref int yOffset)
        {
            int leftCol = ServiceLeftCol;
            int startCol = ServiceStartCol;
            int stopCol = ServiceStopCol;
            
            var lblName = new Label
            {
                Text = "Isaac ROS:",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(120, 20),
                ForeColor = Color.LightGray
            };
            _servicesPanel.Controls.Add(lblName);
            
            _lblIsaacRosStatus = new Label
            {
                Text = "Checking...",
                Location = new Point(ServiceStatusCol, yOffset + 3),
                Size = new Size(240, 20),
                ForeColor = Color.Yellow,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblIsaacRosStatus);
            
            _btnIsaacRosStart = new Button
            {
                Text = "Start",
                Location = new Point(startCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(0, 120, 60),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnIsaacRosStart.Click += async (s, e) => await StartIsaacRosAsync();
            _servicesPanel.Controls.Add(_btnIsaacRosStart);
            
            _btnIsaacRosStop = new Button
            {
                Text = "Stop",
                Location = new Point(stopCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(150, 50, 50),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnIsaacRosStop.Click += async (s, e) => await StopIsaacRosAsync();
            _servicesPanel.Controls.Add(_btnIsaacRosStop);
            
            yOffset += 35;
        }

        private void AddRosBridgeRow(ref int yOffset)
        {
            int leftCol = ServiceLeftCol;
            int startCol = ServiceStartCol;
            int stopCol = ServiceStopCol;

            var lblName = new Label
            {
                Text = "ROS HTTP Bridge:",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(130, 20),
                ForeColor = Color.LightGray
            };
            _servicesPanel.Controls.Add(lblName);

            _lblRosBridgeStatus = new Label
            {
                Text = "Unknown",
                Location = new Point(ServiceStatusCol, yOffset + 3),
                Size = new Size(240, 20),
                ForeColor = Color.Yellow,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblRosBridgeStatus);

            _btnStartRosBridge = new Button
            {
                Text = "Start",
                Location = new Point(startCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(0, 120, 60),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnStartRosBridge.Click += async (s, e) => await StartRosBridgeAsync();
            _servicesPanel.Controls.Add(_btnStartRosBridge);

            _btnStopRosBridge = new Button
            {
                Text = "Stop",
                Location = new Point(stopCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(150, 50, 50),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnStopRosBridge.Click += async (s, e) => await StopRosBridgeAsync();
            _servicesPanel.Controls.Add(_btnStopRosBridge);

            yOffset += 35;
        }

        private void AddTargetLocalizerRow(ref int yOffset)
        {
            int leftCol = ServiceLeftCol;
            int startCol = ServiceStartCol;
            int stopCol = ServiceStopCol;

            var lblName = new Label
            {
                Text = "Target Localizer:",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(130, 20),
                ForeColor = Color.LightGray,
            };
            _servicesPanel.Controls.Add(lblName);

            _lblTargetLocalizerStatus = new Label
            {
                Text = "Checking...",
                Location = new Point(ServiceStatusCol, yOffset + 3),
                Size = new Size(240, 20),
                ForeColor = Color.Yellow,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblTargetLocalizerStatus);

            _btnTargetLocalizerStart = new Button
            {
                Text = "Start",
                Location = new Point(startCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(0, 120, 60),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnTargetLocalizerStart.Click += async (s, e) => await StartTargetLocalizerAsync();
            _servicesPanel.Controls.Add(_btnTargetLocalizerStart);

            _btnTargetLocalizerStop = new Button
            {
                Text = "Stop",
                Location = new Point(stopCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(150, 50, 50),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnTargetLocalizerStop.Click += async (s, e) => await StopTargetLocalizerAsync();
            _servicesPanel.Controls.Add(_btnTargetLocalizerStop);

            yOffset += 35;
        }

        private void AddNoVncRow(ref int yOffset)
        {
            int leftCol = ServiceLeftCol;
            int startCol = ServiceStartCol;
            int stopCol = ServiceStopCol;

            var lblName = new Label
            {
                Text = "noVNC:",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(120, 20),
                ForeColor = Color.LightGray
            };
            _servicesPanel.Controls.Add(lblName);

            _lblNoVncStatus = new Label
            {
                Text = "Checking...",
                Location = new Point(ServiceStatusCol, yOffset + 3),
                Size = new Size(240, 20),
                ForeColor = Color.Yellow,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblNoVncStatus);

            _btnNoVncStart = new Button
            {
                Text = "Start",
                Location = new Point(startCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(0, 120, 60),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnNoVncStart.Click += async (s, e) => await StartNoVncAsync();
            _servicesPanel.Controls.Add(_btnNoVncStart);

            _btnNoVncStop = new Button
            {
                Text = "Stop",
                Location = new Point(stopCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(150, 50, 50),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnNoVncStop.Click += async (s, e) => await StopNoVncAsync();
            _servicesPanel.Controls.Add(_btnNoVncStop);

            yOffset += 35;
        }
        
        private void AddNvbloxRow(ref int yOffset)
        {
            int leftCol = ServiceLeftCol;
            int launchCol = ServiceStartCol;
            int stopCol = ServiceStopCol;

            var lblName = new Label
            {
                Text = "Nvblox:",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(120, 20),
                ForeColor = Color.LightGray
            };
            _servicesPanel.Controls.Add(lblName);

            _lblNvbloxStatus = new Label
            {
                Text = "Checking...",
                Location = new Point(ServiceStatusCol, yOffset + 3),
                Size = new Size(240, 20),
                ForeColor = Color.Yellow,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _servicesPanel.Controls.Add(_lblNvbloxStatus);

            _btnNvbloxLaunch = new Button
            {
                Text = "Launch",
                Location = new Point(launchCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(0, 120, 60),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnNvbloxLaunch.Click += async (s, e) => await LaunchNvbloxAsync();
            _servicesPanel.Controls.Add(_btnNvbloxLaunch);

            _btnNvbloxStop = new Button
            {
                Text = "Stop",
                Location = new Point(stopCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(150, 50, 50),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Left,
            };
            _btnNvbloxStop.Click += async (s, e) => await StopNvbloxAsync();
            _servicesPanel.Controls.Add(_btnNvbloxStop);

            yOffset += 35;
        }

        private async void PollServicesAsync()
        {
            // Prevent overlapping async polls (Timer can fire again before prior await chain completes).
            if (Interlocked.Exchange(ref _isPolling, 1) == 1)
                return;

            try
            {
                int cycle = Interlocked.Increment(ref _pollCycle);
                bool pollIsaac = (cycle == 1) || (cycle % 2 == 0);
                bool pollVio = (cycle == 1) || (cycle % 2 == 0);
                bool pollVideoAndSlam = (cycle == 1) || (cycle % 3 == 0);
                bool servicesFresh = false;
                bool isaacRunningFromServices = false;

                // Primary source of truth for service states.
                var servicesResult = await _sender.GetServicesStatusAsync();
                if (servicesResult.Success)
                {
                    try
                    {
                        var services = JObject.Parse(servicesResult.Data);

                        bool edgeRunning = services["edge_core"]?["running"]?.Value<bool>() ?? false;
                        UpdateStatusLabel(_lblEdgeCoreStatus, edgeRunning, edgeRunning ? "Running" : "Stopped");

                        bool mavRunning = services["mavlink_router"]?["running"]?.Value<bool>() ?? false;
                        string mavRaw = services["mavlink_router"]?["status"]?.Value<string>() ?? string.Empty;
                        string mavText;
                        if (mavRunning)
                        {
                            mavText = "Running";
                        }
                        else if (mavRaw.Equals("no_cubepilot", StringComparison.OrdinalIgnoreCase))
                        {
                            mavText = "No CubePilot";
                        }
                        else
                        {
                            mavText = !string.IsNullOrWhiteSpace(mavRaw) ? $"Stopped ({mavRaw})" : "Stopped";
                        }
                        UpdateStatusLabel(_lblMavlinkStatus, mavRunning, mavText);

                        bool mediamtxRunning = services["mediamtx"]?["running"]?.Value<bool>() ?? false;
                        UpdateStatusLabel(_lblMediamtxStatus, mediamtxRunning, mediamtxRunning ? "Running" : "Stopped");

                        var novncToken = services["novnc"];
                        if (novncToken != null)
                        {
                            bool noVncRunning = novncToken["running"]?.Value<bool>() ?? false;
                            string noVncStatus = novncToken["status"]?.Value<string>() ?? string.Empty;
                            string noVncText = noVncRunning
                                ? "Running (port 6080)"
                                : (!string.IsNullOrWhiteSpace(noVncStatus) ? $"Stopped ({noVncStatus})" : "Stopped");
                            UpdateStatusLabel(_lblNoVncStatus, noVncRunning, noVncText);
                        }
                        else
                        {
                            UpdateStatusLabel(_lblNoVncStatus, false, "Unavailable");
                        }

                        bool isaacRunning = services["isaac_ros"]?["running"]?.Value<bool>() ?? false;
                        string isaacMessage = services["isaac_ros"]?["message"]?.Value<string>();
                        string isaacText = isaacRunning
                            ? "Running"
                            : (string.IsNullOrWhiteSpace(isaacMessage) ? "Not Running" : isaacMessage);
                        UpdateStatusLabel(_lblIsaacRosStatus, isaacRunning, isaacText);
                        isaacRunningFromServices = isaacRunning;

                        bool localizerRunning = services["target_localizer"]?["running"]?.Value<bool>() ?? false;
                        UpdateStatusLabel(_lblTargetLocalizerStatus, localizerRunning,
                            localizerRunning ? "Running" : "Stopped");

                        servicesFresh = true;
                        _servicesFailStreak = 0;
                    }
                    catch (Exception parseEx)
                    {
                        _servicesFailStreak++;
                        if (ShouldLogStreak(_servicesFailStreak))
                            LogMessage($"Services parse warning (streak {_servicesFailStreak}): {parseEx.Message}");
                    }
                }
                else
                {
                    _servicesFailStreak++;
                    if (ShouldLogStreak(_servicesFailStreak))
                        LogMessage($"Services poll warning (streak {_servicesFailStreak}): {servicesResult.Message}");
                }

                // If the primary snapshot is unavailable, keep last-known labels and
                // skip the rest of endpoint-specific probes this cycle.
                if (!servicesFresh)
                {
                    UpdateStatusPendingIfChecking(_lblNoVncStatus, "Waiting...");
                    UpdateStatusPendingIfChecking(_lblIsaacRosStatus, "Waiting...");
                    UpdateStatusPendingIfChecking(_lblTargetLocalizerStatus, "Waiting...");
                    UpdateStatusPendingIfChecking(_lblNvbloxStatus, "Waiting...");
                    UpdateStatusPendingIfChecking(_lblVioStatus, "Waiting...");
                    UpdateStatusPendingIfChecking(_lblVideoBridgesStatus, "Waiting...");
                    UpdateStatusPendingIfChecking(_lblSlamStatus, "Waiting...");
                    UpdateLabel(_lblLastUpdate, $"Last update: {DateTime.Now:HH:mm:ss} (partial/stale)");
                    return;
                }
                
                // Check Isaac ROS status (container + nvblox + bridge)
                if (pollIsaac)
                {
                    var isaacResult = await _sender.GetIsaacStatusAsync();
                    if (isaacResult.Success)
                    {
                        try
                        {
                            var isaacData = JObject.Parse(isaacResult.Data);
                            var containerRunning = isaacData["container_running"]?.Value<bool>() ?? false;
                            var nvbloxRunning = isaacData["nvblox_running"]?.Value<bool>() ?? false;
                            var bridgeRunning = isaacData["bridge_running"]?.Value<bool>() ?? false;

                            UpdateStatusLabel(_lblIsaacRosStatus, containerRunning, containerRunning ? "Running" : "Not Running");

                            UpdateStatusLabel(_lblRosBridgeStatus, bridgeRunning,
                                bridgeRunning ? "Running" : (containerRunning ? "Stopped" : "No Container"));

                            if (nvbloxRunning)
                                UpdateStatusLabel(_lblNvbloxStatus, true, "Running");
                            else if (containerRunning)
                                UpdateStatusLabel(_lblNvbloxStatus, false, "Stopped");
                            else
                                UpdateStatusLabel(_lblNvbloxStatus, false, "No Container");

                            _isaacFailStreak = 0;
                        }
                        catch (Exception parseEx)
                        {
                            _isaacFailStreak++;
                            if (ShouldLogStreak(_isaacFailStreak))
                                LogMessage($"Isaac status parse warning (streak {_isaacFailStreak}): {parseEx.Message}");
                        }
                    }
                    else
                    {
                        _isaacFailStreak++;
                        if (ShouldLogStreak(_isaacFailStreak))
                            LogMessage($"Isaac status warning (streak {_isaacFailStreak}): {isaacResult.Message}");
                    }
                }
                
                // Check VIO status
                if (pollVio)
                {
                    var vioResult = await _sender.GetVioStatusAsync();
                    if (vioResult.Success)
                    {
                        try
                        {
                            var vioData = JObject.Parse(vioResult.Data);
                            var health = vioData["health"]?.Value<string>() ?? "unknown";
                            var source = vioData["source"]?.Value<string>() ?? "none";
                            var confidence = vioData["tracking_confidence"]?.Value<double>() ?? 0;
                            
                            bool healthy = health == "healthy";
                            string statusText = $"{health} ({source})";
                            if (health.Equals("unknown", StringComparison.OrdinalIgnoreCase) && isaacRunningFromServices)
                                statusText = "warming up (isaac_ros)";
                            if (confidence > 0)
                                statusText += $" {confidence:P0}";
                            
                            UpdateStatusLabel(_lblVioStatus, healthy, statusText);
                            _vioFailStreak = 0;
                        }
                        catch (Exception parseEx)
                        {
                            _vioFailStreak++;
                            if (ShouldLogStreak(_vioFailStreak))
                                LogMessage($"VIO parse warning (streak {_vioFailStreak}): {parseEx.Message}");
                        }
                    }
                    else
                    {
                        _vioFailStreak++;
                        if (ShouldLogStreak(_vioFailStreak))
                            LogMessage($"VIO status warning (streak {_vioFailStreak}): {vioResult.Message}");
                    }

                    // Get trajectory points less aggressively (same cadence as VIO poll)
                    var trajResult = await _sender.GetVioTrajectoryAsync(10);
                    if (trajResult.Success)
                    {
                        try
                        {
                            var trajData = JObject.Parse(trajResult.Data);
                            var totalPoints = trajData["total_points"]?.Value<int>() ?? 0;
                            UpdateLabel(_lblVioTrajectoryPoints, $"{totalPoints} points");
                        }
                        catch { }
                    }
                }
                
                // Video bridge status (single bridge configuration)
                if (pollVideoAndSlam)
                {
                    var bridgesResult = await _sender.GetVideoBridgesStatusAsync();
                    if (bridgesResult.Success)
                    {
                        try
                        {
                            var data = JObject.Parse(bridgesResult.Data);
                            var primary = data["bridges"]?["primary"]?["state"]?.ToString() ?? "stopped";
                            // Only check primary bridge (we simplified to single bridge)
                            bool isStreaming = primary == "playing";
                            var fps = data["bridges"]?["primary"]?["fps"]?.Value<float>() ?? 0;
                            string statusText = isStreaming ? $"Streaming ({fps:F1} fps)" : "Stopped";
                            UpdateStatusLabel(_lblVideoBridgesStatus, isStreaming, statusText);
                            _videoFailStreak = 0;
                        }
                        catch (Exception parseEx)
                        {
                            _videoFailStreak++;
                            if (ShouldLogStreak(_videoFailStreak))
                                LogMessage($"Video status parse warning (streak {_videoFailStreak}): {parseEx.Message}");
                        }
                    }
                    else
                    {
                        _videoFailStreak++;
                        if (ShouldLogStreak(_videoFailStreak))
                            LogMessage($"Video status warning (streak {_videoFailStreak}): {bridgesResult.Message}");
                    }

                    // SLAM status
                    var slamResult = await _sender.GetSlamStatusAsync();
                    if (slamResult.Success)
                    {
                        try
                        {
                            var data = JObject.Parse(slamResult.Data);
                            var available = data["available"]?.Value<bool>() ?? false;
                            var running = data["running"]?.Value<bool>() ?? false;
                            if (running)
                            {
                                var blocks = data["block_count"]?.Value<int>() ?? 0;
                                UpdateStatusLabel(_lblSlamStatus, true, $"Active ({blocks} blocks)");
                            }
                            else if (available)
                            {
                                UpdateStatusLabel(_lblSlamStatus, false, "Available (no data)");
                            }
                            else
                            {
                                var error = (string)data["error"];
                                if (isaacRunningFromServices &&
                                    string.Equals(error, "No mesh data available", StringComparison.OrdinalIgnoreCase))
                                {
                                    UpdateStatusLabel(_lblSlamStatus, false, "Waiting for mesh");
                                }
                                else
                                {
                                    UpdateStatusLabel(_lblSlamStatus, false, error ?? "Inactive");
                                }
                            }

                            _slamFailStreak = 0;
                        }
                        catch (Exception parseEx)
                        {
                            _slamFailStreak++;
                            if (ShouldLogStreak(_slamFailStreak))
                                LogMessage($"SLAM status parse warning (streak {_slamFailStreak}): {parseEx.Message}");
                        }
                    }
                    else
                    {
                        _slamFailStreak++;
                        if (ShouldLogStreak(_slamFailStreak))
                            LogMessage($"SLAM status warning (streak {_slamFailStreak}): {slamResult.Message}");
                    }
                }
                
                // Re-sync the circle-detector checkboxes from the bridge so the
                // panel reflects toggles made by Task 1 / Task 2 views.
                await RefreshDetectorState();

                // Update timestamp
                var suffix = servicesFresh ? string.Empty : " (partial/stale)";
                UpdateLabel(_lblLastUpdate, $"Last update: {DateTime.Now:HH:mm:ss}{suffix}");
            }
            catch (Exception ex)
            {
                LogMessage($"Poll error: {ex.Message}");
            }
            finally
            {
                Interlocked.Exchange(ref _isPolling, 0);
            }
        }
        
        private void UpdateStatusLabel(Label label, bool isOk, string customText = null)
        {
            if (label == null || label.IsDisposed) return;
            
            try
            {
                if (label.InvokeRequired)
                {
                    label.BeginInvoke(new Action(() => UpdateStatusLabel(label, isOk, customText)));
                    return;
                }
                
                label.Text = customText ?? (isOk ? "Running" : "Stopped");
                label.ForeColor = isOk ? Color.LimeGreen : Color.OrangeRed;
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
        }

        private void UpdateStatusPendingIfChecking(Label label, string text)
        {
            if (label == null || label.IsDisposed) return;

            try
            {
                if (label.InvokeRequired)
                {
                    label.BeginInvoke(new Action(() => UpdateStatusPendingIfChecking(label, text)));
                    return;
                }

                if (string.Equals(label.Text, "Checking...", StringComparison.OrdinalIgnoreCase))
                {
                    label.Text = text;
                    label.ForeColor = Color.Goldenrod;
                }
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
        }
        
        private void UpdateLabel(Label label, string text)
        {
            if (label == null || label.IsDisposed) return;
            
            try
            {
                if (label.InvokeRequired)
                {
                    label.BeginInvoke(new Action(() => UpdateLabel(label, text)));
                    return;
                }
                
                label.Text = text;
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
        }
        
        private void LogMessage(string message)
        {
            if (_txtLog == null || _txtLog.IsDisposed) return;
            
            try
            {
                if (_txtLog.InvokeRequired)
                {
                    _txtLog.BeginInvoke(new Action(() => LogMessage(message)));
                    return;
                }
                
                var timestamp = DateTime.Now.ToString("HH:mm:ss");
                _txtLog.AppendText($"[{timestamp}] {message}\r\n");
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
        }
        
        private async Task RestartServiceAsync(string serviceName, Label statusLabel)
        {
            LogMessage($"Restarting {serviceName}...");
            UpdateStatusLabel(statusLabel, false, "Restarting...");
            
            var result = await _sender.RestartServiceAsync(serviceName);
            
            if (result.Success)
            {
                LogMessage($"{serviceName} restart command sent");
                await Task.Delay(2000);
                // Will be updated by next poll
            }
            else
            {
                LogMessage($"Failed to restart {serviceName}: {result.Message}");
            }
        }
        
        private async Task RestartAllServicesAsync()
        {
            LogMessage("Restarting all NOMAD services via SSH...");
            UpdateStatusLabel(_lblEdgeCoreStatus, false, "Restarting...");
            UpdateStatusLabel(_lblMavlinkStatus, false, "Restarting...");
            UpdateStatusLabel(_lblMediamtxStatus, false, "Restarting...");
            UpdateStatusLabel(_lblIsaacRosStatus, false, "Restarting...");
            UpdateStatusLabel(_lblVideoBridgesStatus, false, "Restarting...");
            
            // Use SSH instead of HTTP API (since we're killing edge_core)
            LogMessage("Using SSH for restart (HTTP API will be unavailable during restart)");
            var result = await _sender.RestartAllServicesViaSSHAsync();
            
            if (result.Success || result.Message.Contains("executed") || string.IsNullOrEmpty(result.Message))
            {
                LogMessage("NOMAD services restart command sent successfully");
                LogMessage("All services starting (MAVLink, MediaMTX, Edge Core, Isaac ROS, Video Bridge)...");
                LogMessage("Full startup takes 30-45 seconds. Status will update automatically.");
                
                // Show info message on UI thread
                if (this.IsHandleCreated && !this.IsDisposed)
                {
                    this.BeginInvoke(new Action(() =>
                    {
                        MessageBox.Show(
                            this,
                            "NOMAD Services Restart Initiated!\n\n" +
                            "Restarting all services via SSH:\n" +
                            "1. MAVLink Router\n" +
                            "2. MediaMTX (RTSP)\n" +
                            "3. Edge Core API\n" +
                            "4. Isaac ROS + ZED Camera\n" +
                            "5. Video Bridge\n\n" +
                            "Full startup: 30-45 seconds.\n" +
                            "Status will update automatically.",
                            "NOMAD Restarting",
                            MessageBoxButtons.OK,
                            MessageBoxIcon.Information
                        );
                    }));
                }
            }
            else
            {
                LogMessage($"SSH restart command may have issues: {result.Message}");
                LogMessage("Note: This is normal if SSH keys aren't configured. Services may still be restarting.");
            }
        }
        
        private async Task StartIsaacRosAsync()
        {
            LogMessage("Starting Isaac ROS container and services...");
            UpdateStatusLabel(_lblIsaacRosStatus, false, "Starting...");
            
            // Use the dedicated Isaac ROS API endpoint instead of raw shell command
            var result = await _sender.StartIsaacRosAsync();
            
            if (result.Success)
            {
                LogMessage("Isaac ROS startup initiated");
                LogMessage("Container starting with ROS2 environment...");
                LogMessage("Full startup takes 30-60 seconds");
                
                // Show non-blocking message on UI thread
                if (this.IsHandleCreated && !this.IsDisposed)
                {
                    this.BeginInvoke(new Action(() =>
                    {
                        MessageBox.Show(
                            this,
                            "Isaac ROS startup initiated!\n\n" +
                            "The following will start automatically:\n" +
                            "1. Docker container\n" +
                            "2. ROS2 dependencies installation\n\n" +
                            "Note: Nvblox VSLAM must be launched separately\n" +
                            "using the Nvblox controls in this panel.\n\n" +
                            "Full startup takes 30-60 seconds.\n" +
                            "Status will update automatically.",
                            "Isaac ROS Starting",
                            MessageBoxButtons.OK,
                            MessageBoxIcon.Information
                        );
                    }));
                }
            }
            else
            {
                LogMessage($"Failed to start Isaac ROS: {result.Message}");
                UpdateStatusLabel(_lblIsaacRosStatus, false, "Start Failed");
            }
        }

        private async Task StartTargetLocalizerAsync()
        {
            LogMessage("Starting target_localizer node...");
            UpdateStatusLabel(_lblTargetLocalizerStatus, false, "Starting...");
            var result = await _sender.StartTargetLocalizerAsync();
            if (result.Success)
            {
                LogMessage("target_localizer start command sent");
            }
            else
            {
                LogMessage($"Failed to start target_localizer: {result.Message}");
                UpdateStatusLabel(_lblTargetLocalizerStatus, false, "Start Failed");
            }
        }

        private async Task StopTargetLocalizerAsync()
        {
            LogMessage("Stopping target_localizer node...");
            UpdateStatusLabel(_lblTargetLocalizerStatus, false, "Stopping...");
            var result = await _sender.StopTargetLocalizerAsync();
            if (result.Success)
            {
                LogMessage("target_localizer stopped");
                UpdateStatusLabel(_lblTargetLocalizerStatus, false, "Stopped");
            }
            else
            {
                LogMessage($"Failed to stop target_localizer: {result.Message}");
            }
        }

        private async Task StartNoVncAsync()
        {
            LogMessage("Starting noVNC...");
            UpdateStatusLabel(_lblNoVncStatus, false, "Starting...");

            var result = await _sender.StartServiceAsync("novnc");

            if (result.Success)
            {
                LogMessage("noVNC start command sent");
            }
            else
            {
                LogMessage($"Failed to start noVNC: {result.Message}");
                UpdateStatusLabel(_lblNoVncStatus, false, "Start Failed");
            }
        }

        private async Task StopNoVncAsync()
        {
            LogMessage("Stopping noVNC...");
            UpdateStatusLabel(_lblNoVncStatus, false, "Stopping...");

            var result = await _sender.StopServiceAsync("novnc");

            if (result.Success)
            {
                LogMessage("noVNC stop command sent");
            }
            else
            {
                LogMessage($"Failed to stop noVNC: {result.Message}");
            }
        }

        private async Task StopIsaacRosAsync()
        {
            LogMessage("Stopping Isaac ROS...");
            UpdateStatusLabel(_lblIsaacRosStatus, false, "Stopping...");
            
            var result = await _sender.StopIsaacRosAsync();
            
            if (result.Success)
            {
                LogMessage("Isaac ROS stopped");
                UpdateStatusLabel(_lblIsaacRosStatus, false, "Stopped");
            }
            else
            {
                LogMessage($"Failed to stop Isaac ROS: {result.Message}");
            }
        }
        
        private async Task ResetVioOriginAsync()
        {
            LogMessage("Resetting VIO origin...");
            var result = await _sender.ResetVioOriginAsync();
            
            if (result.Success)
            {
                LogMessage("VIO origin reset successful");
            }
            else
            {
                LogMessage($"VIO reset failed: {result.Message}");
            }
        }
        
        private async Task ClearTrajectoryAsync()
        {
            LogMessage("Clearing VIO trajectory...");
            // Use the dedicated HTTP DELETE endpoint for VIO trajectory
            var result = await _sender.ClearVioTrajectoryAsync();
            
            if (result.Success)
            {
                LogMessage("Trajectory cleared");
                UpdateLabel(_lblVioTrajectoryPoints, "0 points");
            }
            else
            {
                LogMessage($"Clear failed: {result.Message}");
            }
        }
        
        private async Task StartRosBridgeAsync()
        {
            LogMessage("Starting ROS HTTP bridge...");
            UpdateStatusLabel(_lblRosBridgeStatus, false, "Starting...");
            var result = await _sender.StartRosBridgeAsync();
            if (result.Success)
            {
                LogMessage("ROS HTTP bridge started");
                UpdateStatusLabel(_lblRosBridgeStatus, true, "Running");
            }
            else
            {
                LogMessage($"Failed to start ROS bridge: {result.Message}");
                UpdateStatusLabel(_lblRosBridgeStatus, false, "Start Failed");
            }
        }

        private async Task StopRosBridgeAsync()
        {
            LogMessage("Stopping ROS HTTP bridge...");
            UpdateStatusLabel(_lblRosBridgeStatus, false, "Stopping...");
            var result = await _sender.StopRosBridgeAsync();
            if (result.Success)
            {
                LogMessage("ROS HTTP bridge stopped");
                UpdateStatusLabel(_lblRosBridgeStatus, false, "Stopped");
            }
            else
            {
                LogMessage($"Failed to stop ROS bridge: {result.Message}");
            }
        }

        private async Task LaunchNvbloxAsync()
        {
            LogMessage("Launching nvblox...");
            UpdateStatusLabel(_lblNvbloxStatus, false, "Launching...");

            var result = await _sender.LaunchNvbloxAsync();
            if (result.Success)
            {
                LogMessage("nvblox launch initiated (~15s for ZED init)");
            }
            else
            {
                LogMessage($"Failed to launch nvblox: {result.Message}");
                UpdateStatusLabel(_lblNvbloxStatus, false, "Launch Failed");
            }
        }

        private async Task StopNvbloxAsync()
        {
            LogMessage("Stopping nvblox...");
            UpdateStatusLabel(_lblNvbloxStatus, false, "Stopping...");

            var result = await _sender.StopNvbloxAsync();
            if (result.Success)
            {
                LogMessage("nvblox stopped");
                UpdateStatusLabel(_lblNvbloxStatus, false, "Stopped");
            }
            else
            {
                LogMessage($"Failed to stop nvblox: {result.Message}");
            }
        }

        private async Task StartVideoBridgesAsync()
        {
            LogMessage("Starting video bridges...");
            UpdateStatusLabel(_lblVideoBridgesStatus, false, "Starting...");
            var result = await _sender.StartVideoBridgesAsync();
            if (result.Success)
                LogMessage("Video bridges start command sent");
            else
                LogMessage($"Failed to start bridges: {result.Message}");
        }

        private async Task StopSlamAsync()
        {
            LogMessage("Stopping SLAM / nvblox...");
            UpdateStatusLabel(_lblSlamStatus, false, "Stopping...");
            var result = await _sender.StopSlamAsync();
            if (result.Success)
            {
                LogMessage("SLAM / nvblox stopped");
                UpdateStatusLabel(_lblSlamStatus, false, "Stopped");
            }
            else
            {
                LogMessage($"Stop failed: {result.Message}");
                UpdateStatusLabel(_lblSlamStatus, false, "Stop Failed");
            }
        }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _pollTimer?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
