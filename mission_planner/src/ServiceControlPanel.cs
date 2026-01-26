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
        
        // Status text
        private Label _lblLastUpdate;
        private TextBox _txtLog;
        
        public ServiceControlPanel(DualLinkSender sender)
        {
            _sender = sender ?? throw new ArgumentNullException(nameof(sender));
            
            InitializeUI();
            
            // Poll every 3 seconds
            _pollTimer = new System.Threading.Timer(
                _ => PollServicesAsync(),
                null,
                TimeSpan.FromSeconds(2),
                TimeSpan.FromSeconds(3)
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
            
            // === Edge Core ===
            AddServiceRow("Edge Core API", ref _lblEdgeCoreStatus, ref _btnEdgeCoreRestart, ref yOffset);
            _btnEdgeCoreRestart.Click += async (s, e) => await RestartEdgeCoreAsync();
            
            // === Isaac ROS (with Start/Stop) ===
            AddIsaacRosRow(ref yOffset);
            
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
        
        private async void PollServicesAsync()
        {
            try
            {
                // Check Edge Core (via health endpoint)
                var healthResult = await _sender.GetHealthAsync();
                UpdateStatusLabel(_lblEdgeCoreStatus, healthResult.Success);
                
                if (healthResult.Success)
                {
                    try
                    {
                        var healthData = JObject.Parse(healthResult.Data);
                        var connected = healthData["connected"]?.Value<bool>() ?? false;
                        UpdateStatusLabel(_lblMavlinkStatus, connected, connected ? "Connected" : "Disconnected");
                    }
                    catch
                    {
                        UpdateStatusLabel(_lblMavlinkStatus, false, "Parse Error");
                    }
                }
                else
                {
                    UpdateStatusLabel(_lblMavlinkStatus, false, "Offline");
                }
                
                // Check MediaMTX
                var mediamtxResult = await _sender.GetServiceStatusAsync("mediamtx");
                bool mediamtxActive = mediamtxResult.Success && 
                    mediamtxResult.Data?.Trim().Contains("active") == true;
                UpdateStatusLabel(_lblMediamtxStatus, mediamtxActive);
                
                // Check Isaac ROS status
                var isaacResult = await _sender.GetIsaacStatusAsync();
                if (isaacResult.Success)
                {
                    try
                    {
                        var isaacData = JObject.Parse(isaacResult.Data);
                        var available = isaacData["available"]?.Value<bool>() ?? false;
                        UpdateStatusLabel(_lblIsaacRosStatus, available, available ? "Running" : "Not Running");
                    }
                    catch
                    {
                        UpdateStatusLabel(_lblIsaacRosStatus, false, "Not Running");
                    }
                }
                else
                {
                    UpdateStatusLabel(_lblIsaacRosStatus, false, "Not Running");
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
            if (label == null) return;
            
            if (label.InvokeRequired)
            {
                label.BeginInvoke(new Action(() => UpdateStatusLabel(label, isOk, customText)));
                return;
            }
            
            label.Text = customText ?? (isOk ? "Running" : "Stopped");
            label.ForeColor = isOk ? Color.LimeGreen : Color.OrangeRed;
        }
        
        private void UpdateLabel(Label label, string text)
        {
            if (label == null) return;
            
            if (label.InvokeRequired)
            {
                label.BeginInvoke(new Action(() => UpdateLabel(label, text)));
                return;
            }
            
            label.Text = text;
        }
        
        private void LogMessage(string message)
        {
            if (_txtLog == null) return;
            
            if (_txtLog.InvokeRequired)
            {
                _txtLog.BeginInvoke(new Action(() => LogMessage(message)));
                return;
            }
            
            var timestamp = DateTime.Now.ToString("HH:mm:ss");
            _txtLog.AppendText($"[{timestamp}] {message}\r\n");
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
        
        private async Task RestartEdgeCoreAsync()
        {
            LogMessage("Restarting Edge Core...");
            UpdateStatusLabel(_lblEdgeCoreStatus, false, "Restarting...");
            
            // Kill and restart edge_core
            var killResult = await _sender.ExecuteTerminalCommandAsync("pkill -f 'edge_core.main'", 5);
            await Task.Delay(1000);
            
            var startResult = await _sender.ExecuteTerminalCommandAsync(
                "cd ~/NOMAD && nohup python3 -m edge_core.main > /tmp/edge_core.log 2>&1 &", 5);
            
            LogMessage("Edge Core restart initiated");
            await Task.Delay(3000);
        }
        
        private async Task StartIsaacRosAsync()
        {
            LogMessage("Starting Isaac ROS (automatic)...");
            UpdateStatusLabel(_lblIsaacRosStatus, false, "Starting...");
            
            var result = await _sender.StartIsaacRosAsync();
            
            if (result.Success)
            {
                LogMessage("Isaac ROS startup initiated");
                LogMessage("Container starting, ZED + Nvblox launching...");
                LogMessage("Full startup takes 30-60 seconds");
                
                // Show non-blocking message
                Task.Run(() => {
                    MessageBox.Show(
                        "Isaac ROS startup initiated!\n\n" +
                        "The following will start automatically:\n" +
                        "1. Docker container\n" +
                        "2. ROS2 dependencies installation\n" +
                        "3. ZED + Nvblox VSLAM\n" +
                        "4. ROS-HTTP bridge to Edge Core\n\n" +
                        "Full startup takes 30-60 seconds.\n" +
                        "Status will update automatically.",
                        "Isaac ROS Starting",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information
                    );
                });
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
            var result = await _sender.ExecuteTerminalCommandAsync(
                "curl -X DELETE http://localhost:8000/api/vio/trajectory", 5);
            
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
