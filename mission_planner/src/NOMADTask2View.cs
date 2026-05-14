// ============================================================
// NOMAD Task 2 View - Indoor Fire Extinguishing
// ============================================================
// Layout mirrors Task 1: video feed on the left, tabbed
// controls on the right.
//
// Tabs (right column):
//   1. Detect & Spray  — detection list, spray controls,
//                        slim payload panel (cam tilt + water),
//                        distance/error display.
//   2. Submit          — Task2UploadPanel (auto + manual flow).
//   3. Status          — VIO/Approach/Mode/Obstacles + spray
//                        sequence detail panels.
//   4. 3D SLAM         — SLAM3DView (unchanged).
//
// Tilt lock is driven from spray state (UpdateSprayUI) — the
// slider is locked only while the autonomous spray sequence
// is mid-run, and unlocked otherwise.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Net.Http;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public class NOMADTask2View : NOMADViewBase, IUpdatableView
    {
        private static readonly HashSet<string> ACTIVE_SPRAY_STATES = new HashSet<string>
        {
            "approach", "aim", "spray", "verify", "upload",
        };

        private readonly DualLinkSender _sender;
        private readonly NOMADConfig _config;
        private readonly JetsonConnectionManager _jetsonConnectionManager;

        // Top-level layout
        private TabControl _tabControl;
        private EmbeddedVideoPlayer _videoPlayer;
        private SLAM3DView _slam3DView;
        private Task2PayloadPanel _payloadPanel;
        private Task2UploadPanel _uploadPanel;

        // ---- Status tab ----
        private Label _lblVioStatus;
        private Label _lblApproachStatus;
        private Label _lblModeStatus;
        private Label _lblObstacleStatus;
        private Label _lblSprayState;
        private Label _lblApproachMethod;
        private Label _lblSprayCount;
        private Label _lblVerification;
        private Label _lblSprayTargets;
        private Label _lblSprayError;
        private Label _lblNvbloxWarning;

        // ---- Detect & Spray tab ----
        private ListBox _lstDetections;
        private Label _lblDetectionCount;
        private Button _btnRefreshDetections;
        private Button _btnSprayTarget;
        private Button _btnAbortSpray;
        private Label _lblDistToTarget;
        private Label _lblTargetCount;
        private Button _btnResetMap;
        private Button _btnResetVio;

        private System.Threading.Timer _modePollTimer;
        private volatile bool _sprayInProgress;
        private JArray _cachedDetections = new JArray();

        public NOMADTask2View(
            DualLinkSender sender,
            NOMADConfig config,
            JetsonConnectionManager jetsonConnectionManager = null)
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

        // ============================================================
        // Layout
        // ============================================================
        private void InitializeUI()
        {
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                Margin = Padding.Empty,
                Padding = Padding.Empty,
                BackColor = NOMADTheme.BG_DARK,
            };
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            // ---- Left: Video ----
            var videoPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                Margin = new Padding(5),
            };
            try
            {
                string rtspUrl = $"rtsp://{_config.EffectiveIP}:8554/primary";
                _videoPlayer = new EmbeddedVideoPlayer("Task 2 Camera", rtspUrl, true, _jetsonConnectionManager);
                _videoPlayer.Dock = DockStyle.Fill;
                videoPanel.Controls.Add(_videoPlayer);
                // Ask the bridge to use Task 2 (shape-based) overlay if available.
                _ = SetOverlayModeAsync("task2");
            }
            catch (Exception ex)
            {
                videoPanel.Controls.Add(new Label
                {
                    Text = $"Video unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 10),
                    ForeColor = ERROR_COLOR,
                    Location = new Point(15, 15),
                    AutoSize = true,
                });
            }
            mainLayout.Controls.Add(videoPanel, 0, 0);

            // ---- Right: TabControl ----
            _tabControl = new TabControl
            {
                Dock = DockStyle.Fill,
                Margin = new Padding(5),
            };
            StyleTabControl(_tabControl);

            // Tab 1: Detect & Spray
            var detectTab = new TabPage("Detect & Spray") { BackColor = CARD_BG, Padding = new Padding(0) };
            detectTab.Controls.Add(CreateDetectSprayPanel());
            _tabControl.TabPages.Add(detectTab);

            // Tab 2: Submit
            var submitTab = new TabPage("Submit") { BackColor = CARD_BG, Padding = new Padding(0) };
            _uploadPanel = new Task2UploadPanel(_config) { Dock = DockStyle.Fill };
            submitTab.Controls.Add(_uploadPanel);
            _tabControl.TabPages.Add(submitTab);

            // Tab 3: Status
            var statusTab = new TabPage("Status") { BackColor = NOMADTheme.BG_DARK, Padding = new Padding(0) };
            statusTab.Controls.Add(CreateStatusPanel());
            _tabControl.TabPages.Add(statusTab);

            // Tab 4: 3D SLAM View
            var slam3DTab = new TabPage("3D SLAM View") { BackColor = NOMADTheme.BG_DARK };
            try
            {
                _slam3DView = new SLAM3DView(_config, _sender) { Dock = DockStyle.Fill };
                slam3DTab.Controls.Add(_slam3DView);
            }
            catch (Exception ex)
            {
                slam3DTab.Controls.Add(new Label
                {
                    Text = $"3D SLAM View unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 11),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                    Padding = new Padding(20),
                });
            }
            _tabControl.TabPages.Add(slam3DTab);

            mainLayout.Controls.Add(_tabControl, 1, 0);
            this.Controls.Add(mainLayout);
        }

        private void StyleTabControl(TabControl tabControl)
        {
            tabControl.DrawMode = TabDrawMode.OwnerDrawFixed;
            tabControl.SizeMode = TabSizeMode.Fixed;
            tabControl.ItemSize = new Size(110, 30);
            tabControl.DrawItem += (s, e) =>
            {
                var tab = tabControl.TabPages[e.Index];
                var isSelected = tabControl.SelectedIndex == e.Index;
                var bgColor = isSelected ? ACCENT_COLOR : Color.FromArgb(45, 45, 48);
                using (var bgBrush = new SolidBrush(bgColor))
                using (var textBrush = new SolidBrush(Color.White))
                {
                    e.Graphics.FillRectangle(bgBrush, e.Bounds);
                    var sf = new StringFormat
                    {
                        Alignment = StringAlignment.Center,
                        LineAlignment = StringAlignment.Center,
                    };
                    e.Graphics.DrawString(tab.Text, new Font("Segoe UI", 9, FontStyle.Bold),
                        textBrush, e.Bounds, sf);
                }
            };
        }

        private async Task SetOverlayModeAsync(string mode)
        {
            try
            {
                await JetsonApiService.PostAsync($"/api/video/overlay/mode?mode={mode}");
            }
            catch { }
        }

        // ============================================================
        // Detect & Spray tab
        // ============================================================
        private Panel CreateDetectSprayPanel()
        {
            var scroll = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                AutoScroll = true,
            };

            var inner = new Panel
            {
                Dock = DockStyle.Top,
                BackColor = CARD_BG,
                Height = 280 + 130 + 150 + 80,
            };

            // ---- Detections card (docks at bottom of inner; added first) ----
            var detectCard = CreateCard("DETECTED TARGETS");
            detectCard.Dock = DockStyle.Top;
            detectCard.Height = 280;

            _lblDetectionCount = new Label
            {
                Text = "Targets: 0",
                Font = new Font("Consolas", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 38),
                AutoSize = true,
            };
            detectCard.Controls.Add(_lblDetectionCount);

            _btnRefreshDetections = CreateButton("Refresh", INFO_COLOR, 80, 26);
            _btnRefreshDetections.Location = new Point(220, 36);
            _btnRefreshDetections.Font = new Font("Segoe UI", 9, FontStyle.Bold);
            _btnRefreshDetections.Click += async (s, e) => await RefreshDetections();
            detectCard.Controls.Add(_btnRefreshDetections);

            _lstDetections = new ListBox
            {
                Location = new Point(15, 64),
                Size = new Size(480, 160),
                Anchor = AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = TEXT_PRIMARY,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.FixedSingle,
                SelectionMode = SelectionMode.One,
                HorizontalScrollbar = true,
            };
            detectCard.Controls.Add(_lstDetections);

            _lblDistToTarget = new Label
            {
                Text = "Distance: --",
                Font = new Font("Consolas", 10, FontStyle.Bold),
                ForeColor = WARNING_COLOR,
                Location = new Point(15, 232),
                AutoSize = true,
            };
            detectCard.Controls.Add(_lblDistToTarget);

            // ---- Spray card ----
            var sprayCard = CreateCard("SPRAY CONTROLS");
            sprayCard.Dock = DockStyle.Top;
            sprayCard.Height = 130;

            _btnSprayTarget = CreateButton("Spray Target", ACCENT_COLOR, 140, 38);
            _btnSprayTarget.Location = new Point(15, 38);
            _btnSprayTarget.Font = new Font("Segoe UI", 11, FontStyle.Bold);
            _btnSprayTarget.Click += async (s, e) => await TriggerSpray();
            sprayCard.Controls.Add(_btnSprayTarget);

            _btnAbortSpray = CreateButton("ABORT", ERROR_COLOR, 90, 38);
            _btnAbortSpray.Location = new Point(165, 38);
            _btnAbortSpray.Font = new Font("Segoe UI", 11, FontStyle.Bold);
            _btnAbortSpray.Click += async (s, e) => await AbortSpray();
            sprayCard.Controls.Add(_btnAbortSpray);

            sprayCard.Controls.Add(new Label
            {
                Text = "1. Position drone with WASD until target visible\n"
                     + "2. Select target in list above\n"
                     + "3. Click Spray Target",
                Font = new Font("Segoe UI", 8),
                ForeColor = TEXT_MUTED,
                Location = new Point(265, 38),
                AutoSize = true,
            });

            // ---- Payload (cam tilt + shoot water) ----
            _payloadPanel = new Task2PayloadPanel(_config)
            {
                Dock = DockStyle.Top,
                Height = 150,
                Margin = new Padding(5, 0, 5, 0),
            };

            // ---- Exclusion map card ----
            var mapCard = CreateCard("EXCLUSION MAP");
            mapCard.Dock = DockStyle.Top;
            mapCard.Height = 80;

            _lblTargetCount = new Label
            {
                Text = "Hit targets: 0",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 42),
                AutoSize = true,
            };
            mapCard.Controls.Add(_lblTargetCount);

            _btnResetMap = CreateButton("Reset Map", ERROR_COLOR, 100, 28);
            _btnResetMap.Location = new Point(170, 38);
            _btnResetMap.Font = new Font("Segoe UI", 9, FontStyle.Bold);
            _btnResetMap.Click += async (s, e) =>
            {
                var confirm = MessageBox.Show(
                    "Reset the exclusion map? All tracked targets will be cleared.",
                    "Confirm Reset",
                    MessageBoxButtons.YesNo, MessageBoxIcon.Warning);
                if (confirm == DialogResult.Yes)
                {
                    await _sender.SendTask2ResetMap();
                    _lblTargetCount.Text = "Hit targets: 0";
                }
            };
            mapCard.Controls.Add(_btnResetMap);

            _btnResetVio = CreateButton("Reset VIO", ERROR_COLOR, 100, 28);
            _btnResetVio.Location = new Point(285, 38);
            _btnResetVio.Font = new Font("Segoe UI", 9, FontStyle.Bold);
            _btnResetVio.Click += async (s, e) => await _sender.ResetVioOriginAsync();
            mapCard.Controls.Add(_btnResetVio);

            // Add cards in reverse-stack order (DockStyle.Top stacks bottom-up)
            inner.Controls.Add(detectCard);
            inner.Controls.Add(sprayCard);
            inner.Controls.Add(_payloadPanel);
            inner.Controls.Add(mapCard);

            scroll.Resize += (s, e) =>
            {
                int totalH = mapCard.Height + _payloadPanel.Height + sprayCard.Height + detectCard.Height;
                inner.Height = Math.Max(totalH, scroll.ClientSize.Height);
            };

            scroll.Controls.Add(inner);
            return scroll;
        }

        // ============================================================
        // Status tab
        // ============================================================
        private Panel CreateStatusPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.BG_DARK,
                AutoScroll = true,
                Padding = new Padding(8),
            };

            // ---- Top bar: VIO / Approach / Mode / Obstacles ----
            var bar = new Panel
            {
                Dock = DockStyle.Top,
                Height = 130,
                BackColor = CARD_BG,
                Padding = new Padding(12, 8, 12, 8),
            };

            _lblVioStatus = MakeRow(bar, "VIO: --", 10);
            _lblApproachStatus = MakeRow(bar, "Approach: --", 32);
            _lblModeStatus = MakeRow(bar, "Mode: --", 54);
            _lblObstacleStatus = MakeRow(bar, "Obstacles: --", 76);
            _lblNvbloxWarning = new Label
            {
                Text = "",
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                ForeColor = Color.Orange,
                Location = new Point(10, 100),
                AutoSize = true,
                Visible = false,
            };
            bar.Controls.Add(_lblNvbloxWarning);

            // ---- Spray sequence detail card ----
            var seqCard = CreateCard("SPRAY SEQUENCE STATUS");
            seqCard.Dock = DockStyle.Top;
            seqCard.Height = 240;

            _lblSprayState = new Label
            {
                Text = "State: idle",
                Font = new Font("Consolas", 14, FontStyle.Bold),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 42),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblSprayState);

            _lblApproachMethod = new Label
            {
                Text = "Approach: --",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 72),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblApproachMethod);

            _lblSprayCount = new Label
            {
                Text = "Sprays: 0 / 2",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 94),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblSprayCount);

            _lblVerification = new Label
            {
                Text = "Verified: --",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 116),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblVerification);

            _lblSprayTargets = new Label
            {
                Text = "Engaged: 0 | OK: 0 | Fail: 0",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 138),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblSprayTargets);

            _lblSprayError = new Label
            {
                Text = "",
                Font = new Font("Consolas", 9),
                ForeColor = ERROR_COLOR,
                Location = new Point(15, 165),
                AutoSize = true,
                MaximumSize = new Size(420, 60),
                Visible = false,
            };
            seqCard.Controls.Add(_lblSprayError);

            seqCard.Controls.Add(new Label
            {
                Text = "APPROACH (ZED-guided) → AIM (visual servo)\n"
                     + "  → SPRAY (500ms pump) → VERIFY (circle change)\n"
                     + "  → UPLOAD (Google Drive) → COMPLETE",
                Font = new Font("Consolas", 8),
                ForeColor = TEXT_MUTED,
                Location = new Point(15, 200),
                AutoSize = true,
            });

            panel.Controls.Add(seqCard);
            panel.Controls.Add(bar);
            return panel;
        }

        private Label MakeRow(Panel parent, string text, int y)
        {
            var lbl = new Label
            {
                Text = text,
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, y),
                AutoSize = true,
            };
            parent.Controls.Add(lbl);
            return lbl;
        }

        // ============================================================
        // Polling
        // ============================================================
        private void StartModePolling()
        {
            _modePollTimer = new System.Threading.Timer(
                _ => PollModeAndSpray(),
                null,
                TimeSpan.FromSeconds(1),
                TimeSpan.FromSeconds(2));
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
                var nav2Task = JetsonApiService.GetAsync("/api/nav2/status");
                var detectionTask = JetsonApiService.GetAsync("/api/task/2/detections");
                var exclMapTask = JetsonApiService.GetAsync("/api/task/2/exclusion_map");

                await Task.WhenAll(modeTask, sprayTask, vioTask, obstacleTask, nav2Task, detectionTask, exclMapTask);
                if (IsDisposed || !IsHandleCreated) return;

                JObject modeData = await ReadJson(modeTask);
                JObject sprayData = await ReadJson(sprayTask);
                JObject vioData = await ReadJson(vioTask);
                JObject obstacleData = await ReadJson(obstacleTask);
                JObject nav2Data = await ReadJson(nav2Task);
                JObject detectionData = await ReadJson(detectionTask);
                JObject exclMapData = await ReadJson(exclMapTask);

                if (!IsDisposed && IsHandleCreated)
                {
                    BeginInvoke((Action)(() => UpdateAllUI(
                        modeData, sprayData, vioData, obstacleData,
                        nav2Data, detectionData, exclMapData)));
                }
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
            catch (Exception) { }
        }

        private static async Task<JObject> ReadJson(Task<HttpResponseMessage> task)
        {
            try
            {
                var resp = await task;
                if (!resp.IsSuccessStatusCode) return null;
                return JObject.Parse(await resp.Content.ReadAsStringAsync());
            }
            catch { return null; }
        }

        private void UpdateAllUI(
            JObject modeData, JObject sprayData, JObject vioData,
            JObject obstacleData, JObject nav2Data, JObject detectionData,
            JObject exclMapData)
        {
            try
            {
                UpdateVioUI(vioData);
                UpdateNav2UI(nav2Data);
                UpdateObstacleUI(obstacleData);
                UpdateModeUI(modeData);
                UpdateSprayUI(sprayData);
                UpdateDetectionUI(detectionData);
                UpdateExclMapUI(exclMapData);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Task2 UI update error: {ex.Message}");
            }
        }

        // ============================================================
        // UI updates
        // ============================================================
        private void UpdateVioUI(JObject vioData)
        {
            if (vioData == null || _lblVioStatus == null) return;
            var health = vioData["health"]?.ToString() ?? "unknown";
            var confidence = vioData["tracking_confidence"]?.Value<double>() ?? 0.0;
            var rateHz = vioData["message_rate_hz"]?.Value<double>() ?? 0.0;
            var source = vioData["source"]?.ToString() ?? "none";

            _lblVioStatus.Text = $"VIO: {health} | {confidence * 100.0:F0}% | {rateHz:F1}Hz | {source}";
            _lblVioStatus.ForeColor = health == "healthy" ? SUCCESS_COLOR
                : health == "degraded" ? WARNING_COLOR : ERROR_COLOR;
        }

        private void UpdateNav2UI(JObject nav2Data)
        {
            if (_lblApproachStatus == null) return;
            if (nav2Data == null)
            {
                _lblApproachStatus.Text = "Approach: velocity";
                _lblApproachStatus.ForeColor = TEXT_SECONDARY;
                return;
            }

            var status = nav2Data["status"]?.ToString() ?? "unknown";
            var goalId = nav2Data["goal_id"]?.ToString() ?? "";
            _lblApproachStatus.Text = string.IsNullOrEmpty(goalId)
                ? "Approach: velocity"
                : $"Approach: legacy Nav2 {status} (goal: {goalId.Substring(0, Math.Min(8, goalId.Length))})";

            _lblApproachStatus.ForeColor =
                status == "navigating" || status == "active" ? ACCENT_COLOR :
                status == "succeeded" ? SUCCESS_COLOR :
                status == "failed" || status == "aborted" ? ERROR_COLOR :
                TEXT_SECONDARY;
        }

        private void UpdateObstacleUI(JObject obstacleData)
        {
            if (_lblObstacleStatus == null) return;
            if (obstacleData == null)
            {
                _lblObstacleStatus.Text = "Obstacles: --";
                _lblObstacleStatus.ForeColor = TEXT_SECONDARY;
                return;
            }
            var valid = obstacleData["valid"]?.Value<bool>() ?? false;
            if (!valid)
            {
                _lblObstacleStatus.Text = "Obstacles: no data";
                _lblObstacleStatus.ForeColor = TEXT_SECONDARY;
                return;
            }
            var nearestCm = obstacleData["nearest_distance_cm"]?.Value<double?>();
            var nearestBearing = obstacleData["nearest_bearing_deg"]?.Value<double?>();
            var ageS = obstacleData["age_seconds"]?.Value<double>() ?? 0.0;

            if (nearestCm.HasValue && nearestBearing.HasValue)
            {
                var meters = nearestCm.Value / 100.0;
                _lblObstacleStatus.Text = $"Obstacles: {meters:F2}m @ {nearestBearing.Value:F0}° ({ageS:F1}s)";
                _lblObstacleStatus.ForeColor = meters < 0.5 ? ERROR_COLOR
                    : meters < 1.5 ? WARNING_COLOR : SUCCESS_COLOR;
            }
            else
            {
                _lblObstacleStatus.Text = $"Obstacles: clear ({ageS:F1}s)";
                _lblObstacleStatus.ForeColor = SUCCESS_COLOR;
            }
        }

        private void UpdateModeUI(JObject modeData)
        {
            if (modeData == null || _lblModeStatus == null) return;
            var status = modeData["status"];
            if (status == null) return;

            var currentMode = status["current_mode"]?.ToString() ?? "unknown";
            _lblModeStatus.Text = $"Mode: {currentMode}";

            var nvbloxRestarting = status["nvblox_restarting"]?.Value<bool>() ?? false;
            if (_lblNvbloxWarning != null)
            {
                _lblNvbloxWarning.Visible = nvbloxRestarting;
                if (nvbloxRestarting)
                    _lblNvbloxWarning.Text = "nvblox restarting -- obstacle avoidance offline";
            }
        }

        private void UpdateSprayUI(JObject sprayData)
        {
            if (sprayData == null || _lblSprayState == null) return;

            var state = sprayData["state"]?.ToString() ?? "idle";
            var sprayCount = sprayData["spray_count"]?.Value<int>() ?? 0;
            var verified = sprayData["verification_passed"]?.Value<bool>() ?? false;
            var error = sprayData["error"]?.ToString();
            var distance = sprayData["distance_to_target"]?.Value<double>() ?? 0.0;
            var approachMethod = sprayData["approach_method"]?.ToString() ?? "";
            var nav2Active = sprayData["nav2_approach_active"]?.Value<bool>() ?? false;
            var nav2GoalId = sprayData["nav2_goal_id"]?.ToString() ?? "";
            var engaged = sprayData["targets_engaged"]?.Value<int>() ?? 0;
            var succeeded = sprayData["targets_succeeded"]?.Value<int>() ?? 0;
            var failed = sprayData["targets_failed"]?.Value<int>() ?? 0;

            _lblSprayState.Text = $"State: {state.ToUpper()}";
            _lblSprayState.ForeColor = state switch
            {
                "idle"     => TEXT_SECONDARY,
                "approach" => ACCENT_COLOR,
                "aim"      => WARNING_COLOR,
                "spray"    => ACCENT_COLOR,
                "verify"   => INFO_COLOR,
                "upload"   => INFO_COLOR,
                "complete" => SUCCESS_COLOR,
                "failed"   => ERROR_COLOR,
                "aborted"  => ERROR_COLOR,
                _          => TEXT_PRIMARY,
            };

            if (state == "approach")
            {
                _lblApproachMethod.Text = nav2Active
                    ? $"Approach: legacy Nav2 (goal: {(string.IsNullOrEmpty(nav2GoalId) ? "--" : nav2GoalId.Substring(0, Math.Min(8, nav2GoalId.Length)))})"
                    : (approachMethod == "velocity" ? "Approach: direct velocity (no obstacle avoid)" : "Approach: pending");
                _lblApproachMethod.ForeColor = nav2Active ? ACCENT_COLOR : WARNING_COLOR;
            }
            else
            {
                _lblApproachMethod.Text = string.IsNullOrEmpty(approachMethod) ? "Approach: --" : $"Approach: last={approachMethod}";
                _lblApproachMethod.ForeColor = TEXT_SECONDARY;
            }

            _lblSprayCount.Text = $"Sprays: {sprayCount} / 2";
            if (state == "idle")
            {
                _lblVerification.Text = "Verified: --";
                _lblVerification.ForeColor = TEXT_SECONDARY;
            }
            else
            {
                _lblVerification.Text = verified ? "Verified: YES" : "Verified: no";
                _lblVerification.ForeColor = verified ? SUCCESS_COLOR : WARNING_COLOR;
            }
            _lblSprayTargets.Text = $"Engaged: {engaged} | OK: {succeeded} | Fail: {failed}";

            _lblDistToTarget.Text = $"Distance: {distance:F2}m";
            _lblDistToTarget.ForeColor = distance > 3.0 ? ERROR_COLOR
                : distance > 2.0 ? WARNING_COLOR : SUCCESS_COLOR;

            if (!string.IsNullOrEmpty(error))
            {
                _lblSprayError.Text = $"Error: {error}";
                _lblSprayError.Visible = true;
            }
            else
            {
                _lblSprayError.Visible = false;
            }

            bool active = ACTIVE_SPRAY_STATES.Contains(state);
            _btnSprayTarget.Enabled = !active;
            _btnAbortSpray.Enabled = active;

            // Spray-state-driven tilt lock — only active when spray is mid-run.
            _payloadPanel?.SetTiltLocked(active);
        }

        private void UpdateDetectionUI(JObject detectionData)
        {
            if (detectionData == null || _lstDetections == null) return;
            try
            {
                var history = detectionData["history"]?["detections"] as JArray;
                var current = detectionData["current"]?["detections"] as JArray;
                var displayDetections = (current != null && current.Count > 0) ? current : history;
                if (displayDetections == null)
                {
                    _lblDetectionCount.Text = "Targets: 0";
                    return;
                }

                _cachedDetections = displayDetections;
                _lblDetectionCount.Text = $"Targets: {displayDetections.Count}";
                var prevSel = _lstDetections.SelectedIndex;
                _lstDetections.Items.Clear();

                foreach (var det in displayDetections)
                {
                    var label = det["label"]?.ToString() ?? "?";
                    var conf = det["confidence"]?.Value<double>() ?? 0;
                    var seen = det["seen_count"]?.Value<int>() ?? 1;
                    var x = det["x"]?.Value<double>() ?? 0;
                    var y = det["y"]?.Value<double>() ?? 0;
                    var z = det["z"]?.Value<double>() ?? 0;
                    var source = det["source"]?.ToString() ?? "";
                    _lstDetections.Items.Add($"{label} conf={conf:F0}% {source} seen={seen} ({x:F1},{y:F1},{z:F1})");
                }

                if (prevSel >= 0 && prevSel < _lstDetections.Items.Count)
                    _lstDetections.SelectedIndex = prevSel;
                else if (_lstDetections.Items.Count > 0 && _lstDetections.SelectedIndex < 0)
                    _lstDetections.SelectedIndex = 0;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Detection UI update error: {ex.Message}");
            }
        }

        private void UpdateExclMapUI(JObject exclMapData)
        {
            if (exclMapData == null || _lblTargetCount == null) return;
            try
            {
                var totalTargets = exclMapData["total_targets"]?.Value<int>() ?? 0;
                _lblTargetCount.Text = $"Hit targets: {totalTargets}";
            }
            catch { }
        }

        // ============================================================
        // Spray actions
        // ============================================================
        private async Task RefreshDetections()
        {
            try
            {
                _btnRefreshDetections.Enabled = false;
                var resp = await JetsonApiService.GetAsync("/api/task/2/detections");
                if (resp.IsSuccessStatusCode)
                {
                    var json = JObject.Parse(await resp.Content.ReadAsStringAsync());
                    UpdateDetectionUI(json);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Detection refresh error: {ex.Message}");
            }
            finally
            {
                _btnRefreshDetections.Enabled = true;
            }
        }

        private async Task TriggerSpray()
        {
            if (_sprayInProgress) return;
            _sprayInProgress = true;
            try
            {
                _btnSprayTarget.Enabled = false;

                JToken target = null;
                int selIdx = -1;
                BeginInvoke((Action)(() => { selIdx = _lstDetections.SelectedIndex; }));
                await Task.Yield();

                if (selIdx >= 0 && selIdx < _cachedDetections.Count)
                {
                    target = _cachedDetections[selIdx];
                }
                else
                {
                    var detectResp = await JetsonApiService.GetAsync("/api/task/2/detections");
                    if (!detectResp.IsSuccessStatusCode)
                    {
                        BeginInvoke((Action)(() => _lblSprayError.Text = "No detections available"));
                        _btnSprayTarget.Enabled = true;
                        return;
                    }
                    var detectJson = JObject.Parse(await detectResp.Content.ReadAsStringAsync());
                    var current = detectJson["current"]?["detections"] as JArray;
                    var history = detectJson["history"]?["detections"] as JArray;
                    var detections = (current != null && current.Count > 0) ? current : history;
                    if (detections == null || detections.Count == 0)
                    {
                        BeginInvoke((Action)(() => _lblSprayError.Text = "No detections -- cannot spray"));
                        _btnSprayTarget.Enabled = true;
                        return;
                    }
                    target = detections[0];
                }

                var payload = new JObject
                {
                    ["target_id"] = target["target_id"] ?? target["id"] ?? 0,
                    ["x"] = target["x"] ?? 0,
                    ["y"] = target["y"] ?? 0,
                    ["z"] = target["z"] ?? 0,
                    ["label"] = target["label"] ?? "",
                    ["confidence"] = target["confidence"] ?? 0,
                    ["image_only"] = target["image_only"]?.Value<bool>() ?? false,
                    ["range_m"] = target["range_m"] ?? target["distance_m"] ?? null,
                };

                var content = new StringContent(payload.ToString(), Encoding.UTF8, "application/json");
                var response = await JetsonApiService.PostAsync("/api/spray/trigger", content);

                if (response.IsSuccessStatusCode)
                {
                    var result = JObject.Parse(await response.Content.ReadAsStringAsync());
                    var skipApproach = result["skip_approach"]?.Value<bool>() ?? false;
                    BeginInvoke((Action)(() =>
                    {
                        _lblSprayState.Text = skipApproach
                            ? "State: SKIPPED APPROACH (already <2m)"
                            : "State: APPROACH STARTING";
                        _lblSprayState.ForeColor = ACCENT_COLOR;
                    }));
                }
                else
                {
                    var body = await response.Content.ReadAsStringAsync();
                    string detail;
                    try { detail = JObject.Parse(body)["detail"]?.ToString() ?? body; }
                    catch { detail = body; }
                    BeginInvoke((Action)(() =>
                    {
                        _lblSprayError.Text = $"Spray failed: {detail}";
                        _lblSprayError.Visible = true;
                    }));
                    _btnSprayTarget.Enabled = true;
                }
            }
            catch (Exception ex)
            {
                BeginInvoke((Action)(() =>
                {
                    _lblSprayError.Text = $"Error: {ex.Message}";
                    _lblSprayError.Visible = true;
                }));
                _btnSprayTarget.Enabled = true;
            }
            finally { _sprayInProgress = false; }
        }

        private async Task AbortSpray()
        {
            try
            {
                _btnAbortSpray.Enabled = false;
                await JetsonApiService.PostAsync("/api/spray/abort");
                BeginInvoke((Action)(() =>
                {
                    _lblSprayState.Text = "State: ABORTED";
                    _lblSprayState.ForeColor = ERROR_COLOR;
                }));
            }
            catch (Exception ex)
            {
                BeginInvoke((Action)(() =>
                {
                    _lblSprayError.Text = $"Abort error: {ex.Message}";
                    _lblSprayError.Visible = true;
                }));
            }
            finally { _btnAbortSpray.Enabled = true; }
        }

        // ============================================================
        public void UpdateData() { }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _modePollTimer?.Dispose();
                _slam3DView?.Dispose();
                _videoPlayer?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
