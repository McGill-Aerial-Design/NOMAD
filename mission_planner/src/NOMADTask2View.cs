// ============================================================
// NOMAD Task 2 View - Indoor Fire Extinguishing
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Net.Http;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
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
        private WebBrowser _remoteScreenBrowser;
        private TextBox _txtRemoteScreenUrl;
        private Button _btnRemoteScreenConnect;
        private Button _btnRemoteScreenOpenExternal;

        // Mode selector controls
        private ComboBox _cmbMode;
        private Label _lblModeStatus;
        private Label _lblNvbloxWarning;
        private System.Threading.Timer _modePollTimer;

        // Spray controls
        private Button _btnSprayTarget;
        private Button _btnAbortSpray;
        private Label _lblSprayStatus;
        private volatile bool _sprayInProgress;
        private bool _modesPopulated;

        // Obstacle distance readout (from Jetson obstacle_distance_bridge)
        private Label _lblObstacleStatus;

        public NOMADTask2View(DualLinkSender sender, NOMADConfig config, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _sender = sender;
            _config = config;
            _jetsonConnectionManager = jetsonConnectionManager;
            InitializeUI();
            StartModePolling();

            VisibleChanged += (s, e) =>
            {
                if (Visible)
                    _modePollTimer?.Change(TimeSpan.FromSeconds(1), TimeSpan.FromSeconds(2));
                else
                    _modePollTimer?.Change(System.Threading.Timeout.Infinite, System.Threading.Timeout.Infinite);
            };
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

            // ==================== Obstacle Distance Card ====================
            var obstacleCard = CreateCard("OBSTACLE DISTANCE");
            obstacleCard.Size = new Size(600, 80);

            _lblObstacleStatus = new Label
            {
                Text = "Obstacles: no data",
                Font = new Font("Consolas", 11),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 45),
                AutoSize = true,
                MaximumSize = new Size(570, 0),
            };
            obstacleCard.Controls.Add(_lblObstacleStatus);

            statusLayout.Controls.Add(obstacleCard);

            // ==================== Operational Mode Card ====================
            var modeCard = CreateCard("OPERATIONAL MODE");
            modeCard.Size = new Size(600, 140);

            var lblModeTitle = new Label
            {
                Text = "Mode:",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 45),
                AutoSize = true,
            };
            modeCard.Controls.Add(lblModeTitle);

            _cmbMode = new ComboBox
            {
                Location = new Point(65, 42),
                Size = new Size(180, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10),
            };
            _cmbMode.Items.AddRange(new object[]
            {
                "outdoor_transit",
                "outdoor_survey",
                "indoor_nav",
                "spray_approach",
                "emergency",
            });
            _cmbMode.SelectedIndex = 0;
            _cmbMode.SelectedIndexChanged += async (s, e) => await SetMode(_cmbMode.SelectedItem?.ToString());
            modeCard.Controls.Add(_cmbMode);

            _lblModeStatus = new Label
            {
                Text = "Current: outdoor_transit",
                Font = new Font("Consolas", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(260, 45),
                AutoSize = true,
            };
            modeCard.Controls.Add(_lblModeStatus);

            _lblNvbloxWarning = new Label
            {
                Text = "",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = WARNING_COLOR,
                Location = new Point(15, 80),
                AutoSize = true,
                Visible = false,
            };
            modeCard.Controls.Add(_lblNvbloxWarning);

            statusLayout.Controls.Add(modeCard);

            // ==================== Spray Controller Card ====================
            var sprayCard = CreateCard("SPRAY CONTROLLER");
            sprayCard.Size = new Size(600, 130);

            _btnSprayTarget = CreateButton("Spray Target", ACCENT_COLOR, 140, 35);
            _btnSprayTarget.Location = new Point(15, 45);
            _btnSprayTarget.Click += async (s, e) => await TriggerSpray();
            sprayCard.Controls.Add(_btnSprayTarget);

            _btnAbortSpray = CreateButton("Abort", ERROR_COLOR, 80, 35);
            _btnAbortSpray.Location = new Point(165, 45);
            _btnAbortSpray.Click += async (s, e) => await AbortSpray();
            sprayCard.Controls.Add(_btnAbortSpray);

            _lblSprayStatus = new Label
            {
                Text = "State: idle",
                Font = new Font("Consolas", 9),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 90),
                AutoSize = true,
                MaximumSize = new Size(560, 0),
            };
            sprayCard.Controls.Add(_lblSprayStatus);

            statusLayout.Controls.Add(sprayCard);

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
                _slam3DView = new SLAM3DView(_config, _sender);
                _slam3DView.Dock = DockStyle.Fill;
                slam3DTab.Controls.Add(_slam3DView);
            }
            catch (Exception ex)
            {
                var errorLabel = new Label
                {
                    Text = $"3D SLAM View unavailable: {ex.Message}\n\n" +
                           "The 3D SLAM view uses OpenTK (cross-platform OpenGL).\n" +
                           "Ensure the OpenTK NuGet packages are present and the\n" +
                           "GPU supports OpenGL 2.1 or newer.",
                    Font = new Font("Segoe UI", 11),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                    Padding = new Padding(20),
                };
                slam3DTab.Controls.Add(errorLabel);
            }

            _tabControl.TabPages.Add(slam3DTab);

            // Remote Screen tab intentionally removed.
            // Remote desktop access is now handled from the Calibration view.

            this.Controls.Add(_tabControl);
        }

        private TabPage CreateRemoteScreenTab()
        {
            var tab = new TabPage("Remote Screen")
            {
                BackColor = NOMADTheme.BG_DARK,
            };

            var layout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 2,
                Padding = new Padding(8),
            };
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 44));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            var topBar = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(40, 40, 45),
                Padding = new Padding(6),
            };

            var lblUrl = new Label
            {
                Text = "noVNC URL:",
                ForeColor = TEXT_SECONDARY,
                Dock = DockStyle.Left,
                Width = 85,
                TextAlign = ContentAlignment.MiddleLeft,
                Font = new Font("Segoe UI", 9),
            };

            _btnRemoteScreenOpenExternal = CreateButton("Open External", Color.FromArgb(60, 60, 65), 115, 28);
            _btnRemoteScreenOpenExternal.Dock = DockStyle.Right;
            _btnRemoteScreenOpenExternal.Click += (s, e) => OpenRemoteScreenExternal();

            _btnRemoteScreenConnect = CreateButton("Connect", ACCENT_COLOR, 90, 28);
            _btnRemoteScreenConnect.Dock = DockStyle.Right;
            _btnRemoteScreenConnect.Click += async (s, e) => await NavigateRemoteScreenAsync();

            _txtRemoteScreenUrl = new TextBox
            {
                Dock = DockStyle.Fill,
                Text = BuildDefaultRemoteScreenUrl(),
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = Color.White,
                BorderStyle = BorderStyle.FixedSingle,
                Font = new Font("Segoe UI", 9),
            };

            topBar.Controls.Add(_txtRemoteScreenUrl);
            topBar.Controls.Add(_btnRemoteScreenConnect);
            topBar.Controls.Add(_btnRemoteScreenOpenExternal);
            topBar.Controls.Add(lblUrl);

            _remoteScreenBrowser = new WebBrowser
            {
                Dock = DockStyle.Fill,
                ScriptErrorsSuppressed = true,
            };
            _remoteScreenBrowser.DocumentText =
                "<html><body style='background:#1e1e21;color:#ddd;font-family:Segoe UI;padding:16px;'>" +
                "<h3 style='color:#36A2EB;margin-top:0;'>Remote Screen</h3>" +
                "<p>Click <b>Connect</b> to open the Jetson noVNC login page inside this tab.</p>" +
                "<p>Auto-connect is disabled so password prompt/login can appear normally.</p>" +
                "<p>If the embedded view still spins, click <b>Open External</b>.</p>" +
                "</body></html>";

            layout.Controls.Add(topBar, 0, 0);
            layout.Controls.Add(_remoteScreenBrowser, 0, 1);

            tab.Controls.Add(layout);
            return tab;
        }

        private string BuildDefaultRemoteScreenUrl()
        {
            string host = _config.UseTailscale ? _config.TailscaleIP : _config.JetsonIP;
            if (string.IsNullOrWhiteSpace(host))
                host = "100.85.121.98";

            return $"http://{host}:6080/vnc.html?autoconnect=0&reconnect=0&resize=scale";
        }

        private string NormalizeRemoteScreenUrl(string url)
        {
            if (string.IsNullOrWhiteSpace(url))
                return url;

            string normalized = url;
            string lower = normalized.ToLowerInvariant();

            int autoconnectIndex = lower.IndexOf("autoconnect=", StringComparison.Ordinal);
            if (autoconnectIndex >= 0)
            {
                int valueStart = autoconnectIndex + "autoconnect=".Length;
                int valueEnd = normalized.IndexOf('&', valueStart);
                if (valueEnd < 0)
                    valueEnd = normalized.Length;
                normalized = normalized.Substring(0, valueStart) + "0" + normalized.Substring(valueEnd);
                lower = normalized.ToLowerInvariant();
            }
            else if (lower.Contains("vnc.html"))
            {
                normalized += normalized.Contains("?") ? "&autoconnect=0" : "?autoconnect=0";
                lower = normalized.ToLowerInvariant();
            }

            if (lower.Contains("vnc.html") && !lower.Contains("reconnect="))
            {
                normalized += normalized.Contains("?") ? "&reconnect=0" : "?reconnect=0";
            }

            return normalized;
        }

        private async Task<bool> IsNoVncReachableAsync(string url)
        {
            try
            {
                using (var client = new HttpClient())
                {
                    client.Timeout = TimeSpan.FromSeconds(4);
                    using (var req = new HttpRequestMessage(HttpMethod.Get, url))
                    using (var resp = await client.SendAsync(req))
                    {
                        return resp.IsSuccessStatusCode;
                    }
                }
            }
            catch
            {
                return false;
            }
        }

        private async Task NavigateRemoteScreenAsync()
        {
            var url = _txtRemoteScreenUrl?.Text?.Trim();
            if (string.IsNullOrWhiteSpace(url) || _remoteScreenBrowser == null)
                return;

            if (!url.StartsWith("http://", StringComparison.OrdinalIgnoreCase) &&
                !url.StartsWith("https://", StringComparison.OrdinalIgnoreCase))
            {
                url = $"http://{url}";
            }

            url = NormalizeRemoteScreenUrl(url);
            _txtRemoteScreenUrl.Text = url;

            try
            {
                bool reachable = await IsNoVncReachableAsync(url);
                if (!reachable)
                {
                    var startResult = await _sender.StartServiceAsync("novnc");
                    if (startResult.Success)
                    {
                        await Task.Delay(1500);
                        reachable = await IsNoVncReachableAsync(url);
                    }
                }

                if (!reachable)
                {
                    MessageBox.Show(
                        "Cannot reach noVNC on port 6080.\n\n" +
                        "Checked URL: " + url + "\n\n" +
                        "Possible causes:\n" +
                        "- noVNC service is not running\n" +
                        "- Firewall is blocking port 6080\n" +
                        "- Network path to Jetson is down",
                        "Remote Screen Unreachable",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Warning
                    );
                    return;
                }

                OpenRemoteScreenExternal(url);

                _remoteScreenBrowser.DocumentText =
                    "<html><body style='background:#1e1e21;color:#ddd;font-family:Segoe UI;padding:16px;'>" +
                    "<h3 style='color:#36A2EB;margin-top:0;'>Remote Screen</h3>" +
                    "<p>Opened noVNC in your default browser for reliable password prompts.</p>" +
                    "<p>If needed, edit the URL above and click Connect again.</p>" +
                    "</body></html>";
            }
            catch (Exception ex)
            {
                MessageBox.Show(
                    $"Failed to open remote screen URL:\n\n{ex.Message}",
                    "Remote Screen Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
        }

        private void OpenRemoteScreenExternal(string urlOverride = null)
        {
            var url = string.IsNullOrWhiteSpace(urlOverride) ? _txtRemoteScreenUrl?.Text?.Trim() : urlOverride;
            if (string.IsNullOrWhiteSpace(url))
                return;

            if (!url.StartsWith("http://", StringComparison.OrdinalIgnoreCase) &&
                !url.StartsWith("https://", StringComparison.OrdinalIgnoreCase))
            {
                url = $"http://{url}";
                _txtRemoteScreenUrl.Text = url;
            }

            try
            {
                var psi = new System.Diagnostics.ProcessStartInfo
                {
                    FileName = url,
                    UseShellExecute = true,
                };
                System.Diagnostics.Process.Start(psi);
            }
            catch (Exception ex)
            {
                MessageBox.Show(
                    $"Failed to open browser:\n\n{ex.Message}",
                    "Remote Screen Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
        }

        #region Mode Polling

        private void StartModePolling()
        {
            _modePollTimer = new System.Threading.Timer(
                _ => PollModeAndSpray(),
                null,
                TimeSpan.FromSeconds(1),
                TimeSpan.FromSeconds(2)
            );
        }

        private async void PollModeAndSpray()
        {
            if (IsDisposed || !IsHandleCreated) return;

            try
            {
                var modeTask = JetsonApiService.GetAsync("/api/mode");
                var sprayTask = JetsonApiService.GetAsync("/api/spray/status");
                var vioTask = JetsonApiService.GetAsync("/api/vio/status");
                var obstacleTask = JetsonApiService.GetAsync("/api/obstacle_distance");

                await Task.WhenAll(modeTask, sprayTask, vioTask, obstacleTask);

                if (IsDisposed || !IsHandleCreated) return;

                var modeResp = await modeTask;
                var sprayResp = await sprayTask;
                var vioResp = await vioTask;
                var obstacleResp = await obstacleTask;

                JObject modeData = null;
                JObject sprayData = null;
                JObject vioData = null;
                JObject obstacleData = null;

                if (modeResp.IsSuccessStatusCode)
                {
                    var json = await modeResp.Content.ReadAsStringAsync();
                    modeData = JObject.Parse(json);
                }

                if (sprayResp.IsSuccessStatusCode)
                {
                    var json = await sprayResp.Content.ReadAsStringAsync();
                    sprayData = JObject.Parse(json);
                }

                if (vioResp.IsSuccessStatusCode)
                {
                    var json = await vioResp.Content.ReadAsStringAsync();
                    vioData = JObject.Parse(json);
                }

                if (obstacleResp.IsSuccessStatusCode)
                {
                    var json = await obstacleResp.Content.ReadAsStringAsync();
                    obstacleData = JObject.Parse(json);
                }

                if (!IsDisposed && IsHandleCreated)
                {
                    this.BeginInvoke((Action)(() => UpdateModeAndSprayUI(modeData, sprayData, vioData, obstacleData)));
                }
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
            catch (Exception)
            {
                // Connection lost — ignore until next poll
            }
        }

        private void UpdateModeAndSprayUI(JObject modeData, JObject sprayData, JObject vioData = null, JObject obstacleData = null)
        {
            try
            {
                // Update VIO status (tracking quality, rate, source)
                if (vioData != null && _lblVioStatus != null)
                {
                    var health = vioData["health"]?.ToString() ?? "unknown";
                    var confidence = vioData["tracking_confidence"]?.Value<double>() ?? 0.0;
                    var rateHz = vioData["message_rate_hz"]?.Value<double>() ?? 0.0;
                    var source = vioData["source"]?.ToString() ?? "none";

                    _lblVioStatus.Text = $"VIO: {health}  |  quality {confidence * 100.0:F0}%  |  {rateHz:F1} Hz  |  {source}";
                    if (health == "healthy")
                        _lblVioStatus.ForeColor = SUCCESS_COLOR;
                    else if (health == "degraded")
                        _lblVioStatus.ForeColor = WARNING_COLOR;
                    else
                        _lblVioStatus.ForeColor = ERROR_COLOR;
                }

                // Update nearest-obstacle readout
                if (obstacleData != null && _lblObstacleStatus != null)
                {
                    var valid = obstacleData["valid"]?.Value<bool>() ?? false;
                    if (!valid)
                    {
                        _lblObstacleStatus.Text = "Obstacles: no data";
                        _lblObstacleStatus.ForeColor = TEXT_SECONDARY;
                    }
                    else
                    {
                        var nearestCm = obstacleData["nearest_distance_cm"]?.Value<double?>();
                        var nearestBearing = obstacleData["nearest_bearing_deg"]?.Value<double?>();
                        var ageS = obstacleData["age_seconds"]?.Value<double>() ?? 0.0;

                        if (nearestCm.HasValue && nearestBearing.HasValue)
                        {
                            var meters = nearestCm.Value / 100.0;
                            _lblObstacleStatus.Text = $"Nearest obstacle: {meters:F2} m @ {nearestBearing.Value:F0}°  (age {ageS:F1}s)";
                            if (meters < 0.5)
                                _lblObstacleStatus.ForeColor = ERROR_COLOR;
                            else if (meters < 1.5)
                                _lblObstacleStatus.ForeColor = WARNING_COLOR;
                            else
                                _lblObstacleStatus.ForeColor = SUCCESS_COLOR;
                        }
                        else
                        {
                            _lblObstacleStatus.Text = $"Obstacles: clear  (age {ageS:F1}s)";
                            _lblObstacleStatus.ForeColor = SUCCESS_COLOR;
                        }
                    }
                }

                // Update mode status
                if (modeData != null)
                {
                    if (!_modesPopulated)
                    {
                        var availModes = modeData["available_modes"] as JArray;
                        if (availModes != null && availModes.Count > 0)
                        {
                            var selected = _cmbMode.SelectedItem?.ToString();
                            _cmbMode.Items.Clear();
                            foreach (var m in availModes)
                                _cmbMode.Items.Add(m.ToString());
                            if (selected != null && _cmbMode.Items.Contains(selected))
                                _cmbMode.SelectedItem = selected;
                            else if (_cmbMode.Items.Count > 0)
                                _cmbMode.SelectedIndex = 0;
                            _modesPopulated = true;
                        }
                    }

                    var status = modeData["status"];
                    if (status != null)
                    {
                        var currentMode = status["current_mode"]?.ToString() ?? "unknown";
                        _lblModeStatus.Text = $"Current: {currentMode}";

                        var nvbloxRestarting = status["nvblox_restarting"]?.Value<bool>() ?? false;
                        if (nvbloxRestarting)
                        {
                            _lblNvbloxWarning.Text = "nvblox restarting -- obstacle avoidance offline";
                            _lblNvbloxWarning.Visible = true;
                        }
                        else
                        {
                            _lblNvbloxWarning.Visible = false;
                        }
                    }
                }

                // Update spray status
                if (sprayData != null)
                {
                    var state = sprayData["state"]?.ToString() ?? "idle";
                    var sprayCount = sprayData["spray_count"]?.Value<int>() ?? 0;
                    var verified = sprayData["verification_passed"]?.Value<bool>() ?? false;
                    var error = sprayData["error"]?.ToString();

                    var statusText = $"State: {state}";
                    if (state != "idle")
                    {
                        statusText += $"  |  Sprays: {sprayCount}  |  Verified: {(verified ? "YES" : "no")}";
                    }
                    if (!string.IsNullOrEmpty(error))
                    {
                        statusText += $"  |  Error: {error}";
                    }
                    _lblSprayStatus.Text = statusText;

                    bool active = state != "idle" && state != "complete" && state != "failed" && state != "aborted";
                    _btnSprayTarget.Enabled = !active;
                    _btnAbortSpray.Enabled = active;
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Mode/Spray UI update error: {ex.Message}");
            }
        }

        #endregion

        #region Mode Actions

        private async Task SetMode(string mode)
        {
            if (string.IsNullOrEmpty(mode)) return;

            try
            {
                var response = await JetsonApiService.PostAsync($"/api/mode/set?mode={mode}");

                if (!response.IsSuccessStatusCode)
                {
                    var body = await response.Content.ReadAsStringAsync();
                    _lblModeStatus.Text = $"Switch failed: {response.StatusCode}";
                    _lblModeStatus.ForeColor = ERROR_COLOR;

                    try
                    {
                        var err = JObject.Parse(body);
                        var detail = err["detail"]?.ToString();
                        if (!string.IsNullOrEmpty(detail))
                            _lblModeStatus.Text = $"Failed: {detail}";
                    }
                    catch { }
                }
                else
                {
                    _lblModeStatus.ForeColor = TEXT_PRIMARY;
                }
            }
            catch (Exception ex)
            {
                _lblModeStatus.Text = $"Error: {ex.Message}";
                _lblModeStatus.ForeColor = ERROR_COLOR;
            }
        }

        #endregion

        #region Spray Actions

        /// <summary>
        /// Trigger spray on the first available detection from the detection list.
        /// </summary>
        private async Task TriggerSpray()
        {
            if (_sprayInProgress) return;
            _sprayInProgress = true;
            try
            {
                _btnSprayTarget.Enabled = false;

                // Fetch latest detections to find a target
                var detectResp = await JetsonApiService.GetAsync("/api/task/2/detections");
                if (!detectResp.IsSuccessStatusCode)
                {
                    _lblSprayStatus.Text = "No detections available";
                    _btnSprayTarget.Enabled = true;
                    return;
                }

                var detectJson = await detectResp.Content.ReadAsStringAsync();
                var detections = JArray.Parse(detectJson);
                if (detections.Count == 0)
                {
                    _lblSprayStatus.Text = "No detections — cannot spray";
                    _btnSprayTarget.Enabled = true;
                    return;
                }

                // Use first detection
                var det = detections[0];
                var payload = new JObject
                {
                    ["target_id"] = det["target_id"] ?? det["id"] ?? 0,
                    ["x"] = det["x"] ?? 0,
                    ["y"] = det["y"] ?? 0,
                    ["z"] = det["z"] ?? 0,
                    ["label"] = det["label"] ?? "",
                };

                var content = new StringContent(payload.ToString(), Encoding.UTF8, "application/json");
                var response = await JetsonApiService.PostAsync("/api/spray/trigger", content);

                if (response.IsSuccessStatusCode)
                {
                    _lblSprayStatus.Text = "Spray triggered...";
                }
                else
                {
                    var body = await response.Content.ReadAsStringAsync();
                    try
                    {
                        var err = JObject.Parse(body);
                        _lblSprayStatus.Text = $"Spray failed: {err["detail"]?.ToString() ?? body}";
                    }
                    catch
                    {
                        _lblSprayStatus.Text = $"Spray failed: HTTP {response.StatusCode}";
                    }
                    _btnSprayTarget.Enabled = true;
                }
            }
            catch (Exception ex)
            {
                _lblSprayStatus.Text = $"Error: {ex.Message}";
                _btnSprayTarget.Enabled = true;
            }
            finally
            {
                _sprayInProgress = false;
            }
        }

        private async Task AbortSpray()
        {
            try
            {
                _btnAbortSpray.Enabled = false;
                await JetsonApiService.PostAsync("/api/spray/abort");
                _lblSprayStatus.Text = "Spray aborted";
            }
            catch (Exception ex)
            {
                _lblSprayStatus.Text = $"Abort error: {ex.Message}";
            }
            finally
            {
                _btnAbortSpray.Enabled = true;
            }
        }

        #endregion

        public void UpdateData()
        {
            // VIO status updates would come from Jetson API
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _modePollTimer?.Dispose();
                _slam3DView?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
