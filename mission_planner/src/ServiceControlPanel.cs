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
        private Label _lblEdgeCoreStatus;
        private Label _lblIsaacRosStatus;
        private Label _lblVioStatus;
        
        // Service control buttons
        private Button _btnMavlinkRestart;
        private Button _btnMediamtxRestart;
        private Button _btnEdgeCoreRestart;
        private Button _btnIsaacRosStart;
        private Button _btnVioReset;
        
        // VIO trajectory info
        private Label _lblVioTrajectoryPoints;
        private Button _btnClearTrajectory;
        
        // Nvblox + Bridge
        private Label _lblNvbloxStatus;
        private Button _btnNvbloxLaunch;
        private Button _btnNvbloxStop;

        // Video bridges
        private Label _lblVideoBridgesStatus;
        private Button _btnStartBridges;

        // SLAM
        private Label _lblSlamStatus;
        private Button _btnClearSlam;
        
        // Status text
        private Label _lblLastUpdate;
        private TextBox _txtLog;
        
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
            this.AutoScroll = true;
            
            int yOffset = 10;
            int leftCol = 15;
            int rightCol = 790;
            
            // Title
            var lblTitle = new Label
            {
                Text = "NOMAD Service Control",
                Location = new Point(leftCol, yOffset),
                Size = new Size(880, 25),
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = Color.White,
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
            };
            this.Controls.Add(lblTitle);
            yOffset += 35;
            
            // === MAVLink Router ===
            AddServiceRow("MAVLink Router", ref _lblMavlinkStatus, ref _btnMavlinkRestart, ref yOffset);
            _btnMavlinkRestart.Click += async (s, e) => await RestartServiceAsync("mavlink-router", _lblMavlinkStatus);
            
            // === MediaMTX ===
            AddServiceRow("MediaMTX (RTSP)", ref _lblMediamtxStatus, ref _btnMediamtxRestart, ref yOffset);
            _btnMediamtxRestart.Click += async (s, e) => await RestartServiceAsync("mediamtx", _lblMediamtxStatus);
            
            // === NOMAD Services (Full Restart) ===
            AddServiceRow("NOMAD Services", ref _lblEdgeCoreStatus, ref _btnEdgeCoreRestart, ref yOffset, "Restart All");
            _btnEdgeCoreRestart.Click += async (s, e) => await RestartAllServicesAsync();
            
            // === Isaac ROS (with Start/Stop) ===
            AddIsaacRosRow(ref yOffset);

            // === Nvblox + Bridge (with Launch/Stop) ===
            AddNvbloxRow(ref yOffset);

            // === Video Bridges ===
            AddServiceRow("Video Bridges", ref _lblVideoBridgesStatus, ref _btnStartBridges, ref yOffset, "Start");
            _btnStartBridges.Click += async (s, e) => await StartVideoBridgesAsync();
            
            // === SLAM Service ===
            AddServiceRow("SLAM / Mesh", ref _lblSlamStatus, ref _btnClearSlam, ref yOffset, "Clear");
            _btnClearSlam.Click += async (s, e) => await ClearSlamAsync();
            
            // Separator
            yOffset += 10;
            var separator = new Label
            {
                BorderStyle = BorderStyle.Fixed3D,
                Location = new Point(leftCol, yOffset),
                Size = new Size(870, 2),
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
            };
            this.Controls.Add(separator);
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
            this.Controls.Add(lblVioTitle);
            yOffset += 25;
            
            // VIO Status
            var lblVioLabel = new Label
            {
                Text = "Status:",
                Location = new Point(leftCol, yOffset),
                Size = new Size(80, 20),
                ForeColor = Color.LightGray
            };
            this.Controls.Add(lblVioLabel);
            
            _lblVioStatus = new Label
            {
                Text = "Unknown",
                Location = new Point(140, yOffset),
                Size = new Size(620, 20),
                ForeColor = Color.Yellow,
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
            };
            this.Controls.Add(_lblVioStatus);
            
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
            this.Controls.Add(_btnVioReset);
            yOffset += 30;
            
            // VIO Trajectory Points
            var lblTrajLabel = new Label
            {
                Text = "Trajectory:",
                Location = new Point(leftCol, yOffset),
                Size = new Size(80, 20),
                ForeColor = Color.LightGray
            };
            this.Controls.Add(lblTrajLabel);
            
            _lblVioTrajectoryPoints = new Label
            {
                Text = "0 points",
                Location = new Point(140, yOffset),
                Size = new Size(620, 20),
                ForeColor = Color.White,
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
            };
            this.Controls.Add(_lblVioTrajectoryPoints);
            
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
            this.Controls.Add(_btnClearTrajectory);
            yOffset += 40;
            
            // Last update
            _lblLastUpdate = new Label
            {
                Text = "Last update: Never",
                Location = new Point(leftCol, yOffset),
                Size = new Size(870, 20),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
            };
            this.Controls.Add(_lblLastUpdate);
            yOffset += 25;
            
            // Log output
            var lblLogTitle = new Label
            {
                Text = "Activity Log:",
                Location = new Point(leftCol, yOffset),
                Size = new Size(100, 20),
                ForeColor = Color.LightGray
            };
            this.Controls.Add(lblLogTitle);
            yOffset += 22;
            
            _txtLog = new TextBox
            {
                Location = new Point(leftCol, yOffset),
                Size = new Size(870, 220),
                Multiline = true,
                ScrollBars = ScrollBars.Vertical,
                ReadOnly = true,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.LightGreen,
                Font = new Font("Consolas", 8),
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Bottom,
            };
            this.Controls.Add(_txtLog);
        }
        
        private void AddServiceRow(string serviceName, ref Label statusLabel, ref Button actionButton, ref int yOffset, string buttonText = "Restart")
        {
            int leftCol = 15;
            int rightCol = 790;
            
            var lblName = new Label
            {
                Text = serviceName + ":",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(120, 20),
                ForeColor = Color.LightGray
            };
            this.Controls.Add(lblName);
            
            statusLabel = new Label
            {
                Text = "Checking...",
                Location = new Point(140, yOffset + 3),
                Size = new Size(620, 20),
                ForeColor = Color.Yellow,
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
            };
            this.Controls.Add(statusLabel);
            
            actionButton = new Button
            {
                Text = buttonText,
                Location = new Point(rightCol, yOffset),
                Size = new Size(100, 25),
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
            };
            this.Controls.Add(actionButton);
            
            yOffset += 35;
        }
        
        private Button _btnIsaacRosStop;
        
        private void AddIsaacRosRow(ref int yOffset)
        {
            int leftCol = 15;
            int startCol = 715;
            int stopCol = 790;
            
            var lblName = new Label
            {
                Text = "Isaac ROS:",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(120, 20),
                ForeColor = Color.LightGray
            };
            this.Controls.Add(lblName);
            
            _lblIsaacRosStatus = new Label
            {
                Text = "Checking...",
                Location = new Point(140, yOffset + 3),
                Size = new Size(560, 20),
                ForeColor = Color.Yellow,
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
            };
            this.Controls.Add(_lblIsaacRosStatus);
            
            _btnIsaacRosStart = new Button
            {
                Text = "Start",
                Location = new Point(startCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(0, 120, 60),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
            };
            _btnIsaacRosStart.Click += async (s, e) => await StartIsaacRosAsync();
            this.Controls.Add(_btnIsaacRosStart);
            
            _btnIsaacRosStop = new Button
            {
                Text = "Stop",
                Location = new Point(stopCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(150, 50, 50),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
            };
            _btnIsaacRosStop.Click += async (s, e) => await StopIsaacRosAsync();
            this.Controls.Add(_btnIsaacRosStop);
            
            yOffset += 35;
        }
        
        private void AddNvbloxRow(ref int yOffset)
        {
            int leftCol = 15;
            int launchCol = 715;
            int stopCol = 790;

            var lblName = new Label
            {
                Text = "Nvblox + Bridge:",
                Location = new Point(leftCol, yOffset + 3),
                Size = new Size(120, 20),
                ForeColor = Color.LightGray
            };
            this.Controls.Add(lblName);

            _lblNvbloxStatus = new Label
            {
                Text = "Checking...",
                Location = new Point(140, yOffset + 3),
                Size = new Size(560, 20),
                ForeColor = Color.Yellow,
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
            };
            this.Controls.Add(_lblNvbloxStatus);

            _btnNvbloxLaunch = new Button
            {
                Text = "Launch",
                Location = new Point(launchCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(0, 120, 60),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
            };
            _btnNvbloxLaunch.Click += async (s, e) => await LaunchNvbloxAsync();
            this.Controls.Add(_btnNvbloxLaunch);

            _btnNvbloxStop = new Button
            {
                Text = "Stop",
                Location = new Point(stopCol, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(150, 50, 50),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Anchor = AnchorStyles.Top | AnchorStyles.Right,
            };
            _btnNvbloxStop.Click += async (s, e) => await StopNvbloxAsync();
            this.Controls.Add(_btnNvbloxStop);

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

                        bool isaacRunning = services["isaac_ros"]?["running"]?.Value<bool>() ?? false;
                        string isaacMessage = services["isaac_ros"]?["message"]?.Value<string>();
                        string isaacText = isaacRunning
                            ? "Running"
                            : (string.IsNullOrWhiteSpace(isaacMessage) ? "Not Running" : isaacMessage);
                        UpdateStatusLabel(_lblIsaacRosStatus, isaacRunning, isaacText);
                        isaacRunningFromServices = isaacRunning;

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
                    UpdateStatusPendingIfChecking(_lblIsaacRosStatus, "Waiting...");
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

                            if (nvbloxRunning && bridgeRunning)
                                UpdateStatusLabel(_lblNvbloxStatus, true, "Running");
                            else if (nvbloxRunning)
                                UpdateStatusLabel(_lblNvbloxStatus, false, "No Bridge");
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
        
        private async Task LaunchNvbloxAsync()
        {
            LogMessage("Launching nvblox + ROS-HTTP bridge...");
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
            LogMessage("Stopping nvblox + bridge...");
            UpdateStatusLabel(_lblNvbloxStatus, false, "Stopping...");

            var result = await _sender.StopNvbloxAsync();
            if (result.Success)
            {
                LogMessage("nvblox + bridge stopped");
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

        private async Task ClearSlamAsync()
        {
            LogMessage("Clearing SLAM mesh...");
            var result = await _sender.ClearSlamAsync();
            if (result.Success)
            {
                LogMessage("SLAM mesh cleared");
                UpdateStatusLabel(_lblSlamStatus, true, "Cleared");
            }
            else
            {
                LogMessage($"Clear failed: {result.Message}");
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
