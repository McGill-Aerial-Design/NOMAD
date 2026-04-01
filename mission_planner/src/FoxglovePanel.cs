using System;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Foxglove Bridge control panel - start/stop Foxglove bridge inside
    /// the Isaac ROS container and get the WebSocket URL for Foxglove Studio.
    /// </summary>
    public class FoxglovePanel : UserControl
    {
        private readonly DualLinkSender _sender;
        private readonly System.Threading.Timer _pollTimer;
        private int _isPolling = 0;
        private int _failStreak = 0;

        // Status
        private Label _lblStatus;
        private Label _lblUrl;
        private Label _lblLastUpdate;

        // Controls
        private Button _btnStart;
        private Button _btnStop;
        private Button _btnCopyUrl;
        private Button _btnViewLogs;

        // Logs
        private TextBox _txtLog;

        public FoxglovePanel(DualLinkSender sender)
        {
            _sender = sender ?? throw new ArgumentNullException(nameof(sender));
            InitializeUI();

            _pollTimer = new System.Threading.Timer(
                _ => PollStatusAsync(),
                null,
                TimeSpan.FromSeconds(3),
                TimeSpan.FromSeconds(5)
            );
        }

        private void InitializeUI()
        {
            this.BackColor = Color.FromArgb(45, 45, 48);
            this.Dock = DockStyle.Fill;
            this.AutoScroll = true;

            int y = 15;
            int leftCol = 20;

            // Title
            var lblTitle = new Label
            {
                Text = "Foxglove Bridge",
                Location = new Point(leftCol, y),
                Size = new Size(380, 28),
                Font = new Font("Segoe UI", 13, FontStyle.Bold),
                ForeColor = Color.White
            };
            this.Controls.Add(lblTitle);
            y += 35;

            // Description
            var lblDesc = new Label
            {
                Text = "Exposes all ROS2 topics via WebSocket so Foxglove Studio can\nvisualize pointclouds, meshes, TF, images, ESDF, and odometry.",
                Location = new Point(leftCol, y),
                Size = new Size(380, 36),
                Font = new Font("Segoe UI", 8.5f),
                ForeColor = Color.FromArgb(180, 180, 180)
            };
            this.Controls.Add(lblDesc);
            y += 48;

            // Separator
            var sep1 = new Label
            {
                BorderStyle = BorderStyle.Fixed3D,
                Location = new Point(leftCol, y),
                Size = new Size(380, 2)
            };
            this.Controls.Add(sep1);
            y += 15;

            // Status row
            var lblStatusTitle = new Label
            {
                Text = "Status:",
                Location = new Point(leftCol, y),
                Size = new Size(60, 20),
                Font = new Font("Segoe UI", 10),
                ForeColor = Color.LightGray
            };
            this.Controls.Add(lblStatusTitle);

            _lblStatus = new Label
            {
                Text = "Checking...",
                Location = new Point(85, y),
                Size = new Size(160, 20),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = Color.Yellow
            };
            this.Controls.Add(_lblStatus);

            // Start/Stop buttons on same row
            _btnStart = new Button
            {
                Text = "Start",
                Location = new Point(250, y - 3),
                Size = new Size(70, 26),
                BackColor = Color.FromArgb(0, 120, 60),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 9)
            };
            _btnStart.Click += async (s, e) => await StartFoxgloveAsync();
            this.Controls.Add(_btnStart);

            _btnStop = new Button
            {
                Text = "Stop",
                Location = new Point(325, y - 3),
                Size = new Size(70, 26),
                BackColor = Color.FromArgb(150, 50, 50),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 9)
            };
            _btnStop.Click += async (s, e) => await StopFoxgloveAsync();
            this.Controls.Add(_btnStop);
            y += 38;

            // URL row
            var lblUrlTitle = new Label
            {
                Text = "WebSocket URL:",
                Location = new Point(leftCol, y),
                Size = new Size(110, 20),
                Font = new Font("Segoe UI", 9.5f),
                ForeColor = Color.LightGray
            };
            this.Controls.Add(lblUrlTitle);

            _lblUrl = new Label
            {
                Text = "Not running",
                Location = new Point(135, y),
                Size = new Size(200, 20),
                Font = new Font("Consolas", 9.5f),
                ForeColor = Color.FromArgb(120, 120, 120)
            };
            this.Controls.Add(_lblUrl);

            _btnCopyUrl = new Button
            {
                Text = "Copy",
                Location = new Point(340, y - 3),
                Size = new Size(55, 24),
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 8),
                Enabled = false
            };
            _btnCopyUrl.Click += (s, e) => CopyUrlToClipboard();
            this.Controls.Add(_btnCopyUrl);
            y += 35;

            // Separator
            var sep2 = new Label
            {
                BorderStyle = BorderStyle.Fixed3D,
                Location = new Point(leftCol, y),
                Size = new Size(380, 2)
            };
            this.Controls.Add(sep2);
            y += 15;

            // Instructions
            var lblInstructions = new Label
            {
                Text = "How to connect:",
                Location = new Point(leftCol, y),
                Size = new Size(200, 20),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = Color.White
            };
            this.Controls.Add(lblInstructions);
            y += 25;

            var steps = new string[]
            {
                "1. Click Start above to launch the bridge",
                "2. Open Foxglove Studio (desktop app or app.foxglove.dev)",
                "3. Click 'Open connection' and select 'Foxglove WebSocket'",
                "4. Paste the WebSocket URL shown above",
                "5. Add panels: 3D, Image, Plot, Raw Messages, etc."
            };

            foreach (var step in steps)
            {
                var lblStep = new Label
                {
                    Text = step,
                    Location = new Point(leftCol + 10, y),
                    Size = new Size(370, 18),
                    Font = new Font("Segoe UI", 8.5f),
                    ForeColor = Color.FromArgb(170, 170, 170)
                };
                this.Controls.Add(lblStep);
                y += 19;
            }
            y += 12;

            // Useful topics
            var lblTopics = new Label
            {
                Text = "Key ROS2 topics to visualize:",
                Location = new Point(leftCol, y),
                Size = new Size(250, 20),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = Color.White
            };
            this.Controls.Add(lblTopics);
            y += 25;

            var topics = new string[]
            {
                "/nvblox_node/mesh                          3D mesh",
                "/nvblox_node/color_layer_marker       Colored voxels",
                "/nvblox_node/combined_dynamic_map_slice  ESDF slice",
                "/zed/zed_node/odom                        VIO odometry",
                "/zed/zed_node/left/image_rect_color   Camera image",
                "/tf                                                        Transform tree",
            };

            foreach (var topic in topics)
            {
                var lblTopic = new Label
                {
                    Text = topic,
                    Location = new Point(leftCol + 10, y),
                    Size = new Size(380, 16),
                    Font = new Font("Consolas", 7.5f),
                    ForeColor = Color.FromArgb(140, 180, 220)
                };
                this.Controls.Add(lblTopic);
                y += 17;
            }
            y += 15;

            // Separator
            var sep3 = new Label
            {
                BorderStyle = BorderStyle.Fixed3D,
                Location = new Point(leftCol, y),
                Size = new Size(380, 2)
            };
            this.Controls.Add(sep3);
            y += 15;

            // Logs section
            var lblLogTitle = new Label
            {
                Text = "Bridge Logs:",
                Location = new Point(leftCol, y),
                Size = new Size(100, 20),
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.LightGray
            };
            this.Controls.Add(lblLogTitle);

            _btnViewLogs = new Button
            {
                Text = "Refresh Logs",
                Location = new Point(300, y - 3),
                Size = new Size(95, 24),
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 8)
            };
            _btnViewLogs.Click += async (s, e) => await RefreshLogsAsync();
            this.Controls.Add(_btnViewLogs);
            y += 25;

            _txtLog = new TextBox
            {
                Location = new Point(leftCol, y),
                Size = new Size(380, 120),
                Multiline = true,
                ScrollBars = ScrollBars.Vertical,
                ReadOnly = true,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.LightGreen,
                Font = new Font("Consolas", 8)
            };
            this.Controls.Add(_txtLog);
            y += 130;

            // Last update
            _lblLastUpdate = new Label
            {
                Text = "Last check: Never",
                Location = new Point(leftCol, y),
                Size = new Size(380, 16),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 7.5f)
            };
            this.Controls.Add(_lblLastUpdate);
        }

        private async void PollStatusAsync()
        {
            if (Interlocked.Exchange(ref _isPolling, 1) == 1)
                return;

            try
            {
                var result = await _sender.GetFoxgloveStatusAsync();
                if (result.Success)
                {
                    var data = JObject.Parse(result.Data);
                    var running = data["running"]?.Value<bool>() ?? false;
                    var url = data["url"]?.Value<string>();

                    UpdateUI(() =>
                    {
                        _lblStatus.Text = running ? "Running" : "Stopped";
                        _lblStatus.ForeColor = running ? Color.LimeGreen : Color.OrangeRed;

                        if (running && !string.IsNullOrEmpty(url))
                        {
                            _lblUrl.Text = url;
                            _lblUrl.ForeColor = Color.FromArgb(100, 200, 255);
                            _btnCopyUrl.Enabled = true;
                        }
                        else
                        {
                            _lblUrl.Text = "Not running";
                            _lblUrl.ForeColor = Color.FromArgb(120, 120, 120);
                            _btnCopyUrl.Enabled = false;
                        }

                        _lblLastUpdate.Text = $"Last check: {DateTime.Now:HH:mm:ss}";
                    });

                    _failStreak = 0;
                }
                else
                {
                    _failStreak++;
                    if (_failStreak >= 5 && (_failStreak == 5 || _failStreak % 10 == 0))
                    {
                        LogMessage($"Status poll failed (streak {_failStreak}): {result.Message}");
                    }
                }
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

        private async Task StartFoxgloveAsync()
        {
            LogMessage("Starting Foxglove bridge...");
            UpdateUI(() =>
            {
                _lblStatus.Text = "Starting...";
                _lblStatus.ForeColor = Color.Yellow;
                _btnStart.Enabled = false;
            });

            var result = await _sender.StartFoxgloveAsync();

            if (result.Success)
            {
                try
                {
                    var data = JObject.Parse(result.Data);
                    var url = data["url"]?.Value<string>() ?? "ws://100.85.121.98:8765";
                    var alreadyRunning = data["already_running"]?.Value<bool>() ?? false;

                    LogMessage(alreadyRunning ? "Foxglove bridge was already running" : "Foxglove bridge started");

                    UpdateUI(() =>
                    {
                        _lblStatus.Text = "Running";
                        _lblStatus.ForeColor = Color.LimeGreen;
                        _lblUrl.Text = url;
                        _lblUrl.ForeColor = Color.FromArgb(100, 200, 255);
                        _btnCopyUrl.Enabled = true;
                        _btnStart.Enabled = true;
                    });
                }
                catch
                {
                    LogMessage("Foxglove bridge started (could not parse response)");
                    UpdateUI(() => { _btnStart.Enabled = true; });
                }
            }
            else
            {
                LogMessage($"Failed to start: {result.Message}");
                // Try to show logs from the response
                try
                {
                    var data = JObject.Parse(result.Data ?? "{}");
                    var logs = data["logs"]?.Value<string>();
                    if (!string.IsNullOrEmpty(logs))
                        LogMessage($"Logs: {logs}");
                }
                catch { }

                UpdateUI(() =>
                {
                    _lblStatus.Text = "Start Failed";
                    _lblStatus.ForeColor = Color.OrangeRed;
                    _btnStart.Enabled = true;
                });
            }
        }

        private async Task StopFoxgloveAsync()
        {
            LogMessage("Stopping Foxglove bridge...");
            UpdateUI(() =>
            {
                _lblStatus.Text = "Stopping...";
                _lblStatus.ForeColor = Color.Yellow;
                _btnStop.Enabled = false;
            });

            var result = await _sender.StopFoxgloveAsync();

            if (result.Success)
            {
                LogMessage("Foxglove bridge stopped");
                UpdateUI(() =>
                {
                    _lblStatus.Text = "Stopped";
                    _lblStatus.ForeColor = Color.OrangeRed;
                    _lblUrl.Text = "Not running";
                    _lblUrl.ForeColor = Color.FromArgb(120, 120, 120);
                    _btnCopyUrl.Enabled = false;
                    _btnStop.Enabled = true;
                });
            }
            else
            {
                LogMessage($"Failed to stop: {result.Message}");
                UpdateUI(() => { _btnStop.Enabled = true; });
            }
        }

        private async Task RefreshLogsAsync()
        {
            _btnViewLogs.Enabled = false;
            var result = await _sender.GetFoxgloveLogsAsync();

            if (result.Success)
            {
                try
                {
                    var data = JObject.Parse(result.Data);
                    var logs = data["logs"]?.Value<string>() ?? data["log_type"]?.Value<string>() ?? "";
                    UpdateUI(() =>
                    {
                        _txtLog.Text = logs;
                        _txtLog.SelectionStart = _txtLog.Text.Length;
                        _txtLog.ScrollToCaret();
                    });
                }
                catch
                {
                    UpdateUI(() => { _txtLog.Text = result.Data ?? "No logs available"; });
                }
            }
            else
            {
                LogMessage($"Could not fetch logs: {result.Message}");
            }

            UpdateUI(() => { _btnViewLogs.Enabled = true; });
        }

        private void CopyUrlToClipboard()
        {
            var url = _lblUrl.Text;
            if (!string.IsNullOrEmpty(url) && url != "Not running")
            {
                try
                {
                    Clipboard.SetText(url);
                    LogMessage($"Copied to clipboard: {url}");
                }
                catch (Exception ex)
                {
                    LogMessage($"Clipboard error: {ex.Message}");
                }
            }
        }

        private void LogMessage(string message)
        {
            UpdateUI(() =>
            {
                if (_txtLog == null || _txtLog.IsDisposed) return;
                var timestamp = DateTime.Now.ToString("HH:mm:ss");
                _txtLog.AppendText($"[{timestamp}] {message}\r\n");
            });
        }

        private void UpdateUI(Action action)
        {
            try
            {
                if (this.IsDisposed) return;
                if (this.InvokeRequired)
                    this.BeginInvoke(action);
                else
                    action();
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
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
