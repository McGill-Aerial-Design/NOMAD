using System;
using System.Drawing;
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
            _pollIntervalMs = pollIntervalMs;
            
            InitializeUI();
            
            // Poll using configured interval
            _pollTimer = new System.Threading.Timer(
                _ => PollServicesAsync(),
                null,
                TimeSpan.FromSeconds(2),
                TimeSpan.FromMilliseconds(_pollIntervalMs)
            );
        }
        
        private void InitializeUI()
        {
            this.BackColor = Color.FromArgb(45, 45, 48);
            this.Size = new Size(420, 550);
            this.AutoScroll = true;
            
            int yOffset = 10;
            int leftCol = 15;
            int rightCol = 280;
            
            // Title
            var lblTitle = new Label
            {
                Text = "NOMAD Service Control",
                Location = new Point(leftCol, yOffset),
                Size = new Size(390, 25),
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = Color.White
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
                Size = new Size(380, 2)
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
                Location = new Point(100, yOffset),
                Size = new Size(150, 20),
                ForeColor = Color.Yellow
            };
            this.Controls.Add(_lblVioStatus);
            
            _btnVioReset = new Button
            {
                Text = "Reset Origin",
                Location = new Point(rightCol, yOffset - 3),
                Size = new Size(100, 25),
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
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
                Location = new Point(100, yOffset),
                Size = new Size(150, 20),
                ForeColor = Color.White
            };
            this.Controls.Add(_lblVioTrajectoryPoints);
            
            _btnClearTrajectory = new Button
            {
                Text = "Clear",
                Location = new Point(rightCol, yOffset - 3),
                Size = new Size(100, 25),
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            _btnClearTrajectory.Click += async (s, e) => await ClearTrajectoryAsync();
            this.Controls.Add(_btnClearTrajectory);
            yOffset += 40;
            
            // Last update
            _lblLastUpdate = new Label
            {
                Text = "Last update: Never",
                Location = new Point(leftCol, yOffset),
                Size = new Size(380, 20),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8)
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
                Size = new Size(380, 100),
                Multiline = true,
                ScrollBars = ScrollBars.Vertical,
                ReadOnly = true,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.LightGreen,
                Font = new Font("Consolas", 8)
            };
            this.Controls.Add(_txtLog);
        }
        
        private void AddServiceRow(string serviceName, ref Label statusLabel, ref Button actionButton, ref int yOffset, string buttonText = "Restart")
        {
            int leftCol = 15;
            int rightCol = 280;
            
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
                Size = new Size(120, 20),
                ForeColor = Color.Yellow
            };
            this.Controls.Add(statusLabel);
            
            actionButton = new Button
            {
                Text = buttonText,
                Location = new Point(rightCol, yOffset),
                Size = new Size(100, 25),
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            this.Controls.Add(actionButton);
            
            yOffset += 35;
        }
        
        private Button _btnIsaacRosStop;
        
        private void AddIsaacRosRow(ref int yOffset)
        {
            int leftCol = 15;
            
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
                Size = new Size(90, 20),
                ForeColor = Color.Yellow
            };
            this.Controls.Add(_lblIsaacRosStatus);
            
            _btnIsaacRosStart = new Button
            {
                Text = "Start",
                Location = new Point(235, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(0, 120, 60),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            _btnIsaacRosStart.Click += async (s, e) => await StartIsaacRosAsync();
            this.Controls.Add(_btnIsaacRosStart);
            
            _btnIsaacRosStop = new Button
            {
                Text = "Stop",
                Location = new Point(310, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(150, 50, 50),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            _btnIsaacRosStop.Click += async (s, e) => await StopIsaacRosAsync();
            this.Controls.Add(_btnIsaacRosStop);
            
            yOffset += 35;
        }
        
        private void AddNvbloxRow(ref int yOffset)
        {
            int leftCol = 15;

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
                Size = new Size(90, 20),
                ForeColor = Color.Yellow
            };
            this.Controls.Add(_lblNvbloxStatus);

            _btnNvbloxLaunch = new Button
            {
                Text = "Launch",
                Location = new Point(235, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(0, 120, 60),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            _btnNvbloxLaunch.Click += async (s, e) => await LaunchNvbloxAsync();
            this.Controls.Add(_btnNvbloxLaunch);

            _btnNvbloxStop = new Button
            {
                Text = "Stop",
                Location = new Point(310, yOffset),
                Size = new Size(70, 25),
                BackColor = Color.FromArgb(150, 50, 50),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            _btnNvbloxStop.Click += async (s, e) => await StopNvbloxAsync();
            this.Controls.Add(_btnNvbloxStop);

            yOffset += 35;
        }

        private async void PollServicesAsync()
        {
            try
            {
                // Check Edge Core (via health endpoint)
                var healthResult = await _sender.GetHealthAsync();
                UpdateStatusLabel(_lblEdgeCoreStatus, healthResult.Success);
                
                // Check MAVLink Router service status (is the service running?)
                var mavlinkServiceResult = await _sender.GetServiceStatusAsync("mavlink-router");
                bool mavlinkServiceActive = mavlinkServiceResult.Success &&
                    mavlinkServiceResult.Data?.Trim().Equals("active", StringComparison.OrdinalIgnoreCase) == true;
                
                if (healthResult.Success && mavlinkServiceActive)
                {
                    try
                    {
                        var healthData = JObject.Parse(healthResult.Data);
                        var fcConnected = healthData["connected"]?.Value<bool>() ?? false;
                        string mavStatus = fcConnected ? "Running (FC linked)" : "Running (no FC)";
                        UpdateStatusLabel(_lblMavlinkStatus, true, mavStatus);
                    }
                    catch
                    {
                        UpdateStatusLabel(_lblMavlinkStatus, true, "Running");
                    }
                }
                else if (mavlinkServiceActive)
                {
                    UpdateStatusLabel(_lblMavlinkStatus, true, "Running (no FC)");
                }
                else if (healthResult.Success)
                {
                    UpdateStatusLabel(_lblMavlinkStatus, false, "Service Stopped");
                }
                else
                {
                    UpdateStatusLabel(_lblMavlinkStatus, false, "Offline");
                }
                
                // Check MediaMTX
                var mediamtxResult = await _sender.GetServiceStatusAsync("mediamtx");
                bool mediamtxActive = mediamtxResult.Success && 
                    mediamtxResult.Data?.Trim().Equals("active", StringComparison.OrdinalIgnoreCase) == true;
                UpdateStatusLabel(_lblMediamtxStatus, mediamtxActive);
                
                // Check Isaac ROS status (container + nvblox + bridge)
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
                    }
                    catch
                    {
                        UpdateStatusLabel(_lblIsaacRosStatus, false, "Not Running");
                        UpdateStatusLabel(_lblNvbloxStatus, false, "Unknown");
                    }
                }
                else
                {
                    UpdateStatusLabel(_lblIsaacRosStatus, false, "Not Running");
                    UpdateStatusLabel(_lblNvbloxStatus, false, "Offline");
                }
                
                // Check VIO status
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
                        if (confidence > 0)
                            statusText += $" {confidence:P0}";
                        
                        UpdateStatusLabel(_lblVioStatus, healthy, statusText);
                    }
                    catch
                    {
                        UpdateStatusLabel(_lblVioStatus, false, "Error");
                    }
                }
                else
                {
                    UpdateStatusLabel(_lblVioStatus, false, "Unavailable");
                }
                
                // Get trajectory points
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
                
                // Video bridge status (single bridge configuration)
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
                    }
                    catch { UpdateStatusLabel(_lblVideoBridgesStatus, false, "Parse Error"); }
                }
                else
                {
                    UpdateStatusLabel(_lblVideoBridgesStatus, false, "Offline");
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
                            UpdateStatusLabel(_lblSlamStatus, false, error ?? "Inactive");
                        }
                    }
                    catch { UpdateStatusLabel(_lblSlamStatus, false, "Error"); }
                }
                else
                {
                    UpdateStatusLabel(_lblSlamStatus, false, "Unavailable");
                }
                
                // Update timestamp
                UpdateLabel(_lblLastUpdate, $"Last update: {DateTime.Now:HH:mm:ss}");
            }
            catch (Exception ex)
            {
                LogMessage($"Poll error: {ex.Message}");
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
