// ============================================================
// NOMAD Task 2 View - Indoor Fire Extinguishing
// ============================================================
// Layout:
//   Top:    Mission summary bar (VIO + approach + mode at a glance)
//   Left:   Detection list + target selection + spray controls
//   Right:  Detailed status panels (approach, spray sequence, exclusion map)
//   Tab 2:  3D SLAM View
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
        private readonly DualLinkSender _sender;
        private readonly NOMADConfig _config;
        private readonly JetsonConnectionManager _jetsonConnectionManager;

        // Tab control
        private TabControl _tabControl;
        private SLAM3DView _slam3DView;

        // ---- Top status bar ----
        private Label _lblVioStatus;
        private Label _lblApproachStatus;
        private Label _lblModeStatus;
        private Label _lblObstacleStatus;

        // ---- Left panel: Detection list + spray ----
        private ListBox _lstDetections;
        private Label _lblDetectionCount;
        private Button _btnRefreshDetections;
        private Button _btnSprayTarget;
        private Button _btnAbortSpray;
        private Label _lblDistToTarget;

        // ---- Right panel: Status cards ----
        private Label _lblSprayState;
        private Label _lblApproachMethod;
        private Label _lblSprayCount;
        private Label _lblVerification;
        private Label _lblSprayTargets;
        private Label _lblSprayError;

        // Mode display (read-only, no selector)
        private Label _lblNvbloxWarning;

        // Exclusion map
        private Label _lblTargetCount;
        private Button _btnResetMap;
        private Button _btnResetVio;

        // Polling timer
        private System.Threading.Timer _modePollTimer;
        private volatile bool _sprayInProgress;
        // _modesPopulated removed: mode selector removed for Task 2

        // Cached detection data
        private JArray _cachedDetections = new JArray();

        public NOMADTask2View(
            DualLinkSender sender,
            NOMADConfig config,
            JetsonConnectionManager jetsonConnectionManager = null
        )
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
                    _modePollTimer?.Change(
                        System.Threading.Timeout.Infinite,
                        System.Threading.Timeout.Infinite
                    );
            };
        }

        private void InitializeUI()
        {
            _tabControl = new TabControl { Dock = DockStyle.Fill };

            // ======== Tab 1: Status & Controls ========
            var statusTab = new TabPage("Status & Controls")
            {
                BackColor = NOMADTheme.BG_DARK,
            };

            var outerLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 2,
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(8),
            };
            // Row 0: top status bar (spans both columns)
            outerLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 45F));
            outerLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 55F));
            outerLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 110F));
            outerLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

            // ---- Top status bar (row 0, spans both columns) ----
            var topBar = CreateTopStatusBar();
            outerLayout.Controls.Add(topBar, 0, 0);
            outerLayout.SetColumnSpan(topBar, 2);

            // ---- Left column: Detection list + spray controls ----
            var leftPanel = CreateLeftPanel();
            outerLayout.Controls.Add(leftPanel, 0, 1);

            // ---- Right column: Status cards ----
            var rightPanel = CreateRightPanel();
            outerLayout.Controls.Add(rightPanel, 1, 1);

            statusTab.Controls.Add(outerLayout);
            _tabControl.TabPages.Add(statusTab);

            // ======== Tab 2: 3D SLAM View ========
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
                    Text =
                        $"3D SLAM View unavailable: {ex.Message}\n\n"
                        + "The 3D SLAM view uses OpenTK (cross-platform OpenGL).\n"
                        + "Ensure the OpenTK NuGet packages are present and the\n"
                        + "GPU supports OpenGL 2.1 or newer.",
                    Font = new Font("Segoe UI", 11),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                    Padding = new Padding(20),
                };
                slam3DTab.Controls.Add(errorLabel);
            }

            _tabControl.TabPages.Add(slam3DTab);

            this.Controls.Add(_tabControl);
        }

        // ============================================================
        // Top Status Bar
        // ============================================================
        private Panel CreateTopStatusBar()
        {
            var bar = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                Padding = new Padding(12, 8, 12, 8),
            };

            // VIO status
            _lblVioStatus = new Label
            {
                Text = "VIO: --",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 10),
                AutoSize = true,
            };
            bar.Controls.Add(_lblVioStatus);

            // Approach status
            _lblApproachStatus = new Label
            {
                Text = "Approach: --",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 32),
                AutoSize = true,
            };
            bar.Controls.Add(_lblApproachStatus);

            // Mode status
            _lblModeStatus = new Label
            {
                Text = "Mode: --",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 54),
                AutoSize = true,
            };
            bar.Controls.Add(_lblModeStatus);

            // Obstacle status
            _lblObstacleStatus = new Label
            {
                Text = "Obstacles: --",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 76),
                AutoSize = true,
            };
            bar.Controls.Add(_lblObstacleStatus);

        // Right side: Reset VIO + nvblox warning
        _btnResetVio = CreateButton("Reset VIO", ERROR_COLOR, 95, 28);
        _btnResetVio.Location = new Point(400, 10);
        _btnResetVio.Click += async (s, e) => await _sender.ResetVioOriginAsync();

            _btnResetVio = CreateButton("Reset VIO", ERROR_COLOR, 95, 28);
            _btnResetVio.Location = new Point(400, 64);
            _btnResetVio.Click += async (s, e) => await _sender.ResetVioOriginAsync();
            bar.Controls.Add(_btnResetVio);

            return bar;
        }

        // ============================================================
        // Left Panel: Detection List + Spray Controls
        // ============================================================
        private Panel CreateLeftPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(0, 8, 4, 0),
            };

            var layout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = true,
                Padding = new Padding(4),
            };

            // ---- Detection List Card ----
            var detectCard = CreateCard("DETected targets");
            detectCard.AutoSize = true; detectCard.AutoSizeMode = AutoSizeMode.GrowAndShrink; detectCard.MinimumSize = new Size(280, 240); detectCard.MaximumSize = new Size(500, 0);

            _lblDetectionCount = new Label
            {
                Text = "Targets: 0",
                Font = new Font("Consolas", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 42),
                AutoSize = true,
            };
            detectCard.Controls.Add(_lblDetectionCount);

            _btnRefreshDetections = CreateButton("Refresh", INFO_COLOR, 80, 26);
            _btnRefreshDetections.Location = new Point(245, 38);
            _btnRefreshDetections.Font = new Font("Segoe UI", 9, FontStyle.Bold);
            _btnRefreshDetections.Click += async (s, e) =>
                await RefreshDetections();
            detectCard.Controls.Add(_btnRefreshDetections);

            _lstDetections = new ListBox
            {
                Location = new Point(15, 68),
                Size = new Size(300, 140), Anchor = AnchorStyles.Left | AnchorStyles.Right,
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = TEXT_PRIMARY,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.FixedSingle,
                SelectionMode = SelectionMode.One,
                HorizontalScrollbar = true,
            };
            detectCard.Controls.Add(_lstDetections);

            // Distance indicator
            _lblDistToTarget = new Label
            {
                Text = "Distance: --",
                Font = new Font("Consolas", 10, FontStyle.Bold),
                ForeColor = WARNING_COLOR,
                Location = new Point(15, 215),
                AutoSize = true,
            };
            detectCard.Controls.Add(_lblDistToTarget);

            layout.Controls.Add(detectCard);

            // ---- Spray Control Card ----
            var sprayCard = CreateCard("SPRAY controls");
            sprayCard.AutoSize = true; sprayCard.AutoSizeMode = AutoSizeMode.GrowAndShrink; sprayCard.MinimumSize = new Size(280, 120);

            _btnSprayTarget = CreateButton("Spray Target", ACCENT_COLOR, 140, 38);
            _btnSprayTarget.Location = new Point(15, 42);
            _btnSprayTarget.Font = new Font("Segoe UI", 11, FontStyle.Bold);
            _btnSprayTarget.Click += async (s, e) => await TriggerSpray();
            sprayCard.Controls.Add(_btnSprayTarget);

            _btnAbortSpray = CreateButton("ABORT", ERROR_COLOR, 90, 38);
            _btnAbortSpray.Location = new Point(165, 42);
            _btnAbortSpray.Font = new Font("Segoe UI", 11, FontStyle.Bold);
            _btnAbortSpray.Click += async (s, e) => await AbortSpray();
            sprayCard.Controls.Add(_btnAbortSpray);

            // Workflow hint
            var workflowHint = new Label
            {
                Text =
                    "1. Fly until target is visible in ZED\n"
                    + "2. Select target in list above\n"
                    + "3. Click Spray Target",
                Font = new Font("Segoe UI", 8),
                ForeColor = TEXT_MUTED,
                Location = new Point(15, 88),
                AutoSize = true,
            };
            sprayCard.Controls.Add(workflowHint);

            layout.Controls.Add(sprayCard);

            // ---- Exclusion Map Card ----
            var mapCard = CreateCard("exclusion map");
            mapCard.AutoSize = true; mapCard.AutoSizeMode = AutoSizeMode.GrowAndShrink; mapCard.MinimumSize = new Size(280, 75);

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
            _btnResetMap.Location = new Point(200, 42);
            _btnResetMap.Font = new Font("Segoe UI", 9, FontStyle.Bold);
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
                    _lblTargetCount.Text = "Hit targets: 0";
                }
            };
            mapCard.Controls.Add(_btnResetMap);

            layout.Controls.Add(mapCard);

            panel.Controls.Add(layout);
            return panel;
        }

        // ============================================================
        // Right Panel: Spray Sequence Status
        // ============================================================
        private Panel CreateRightPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(4, 8, 0, 0),
            };

            var layout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = true,
                Padding = new Padding(4),
            };

            // ---- Spray Sequence Status Card ----
            var seqCard = CreateCard("spray sequence status");
            seqCard.AutoSize = true; seqCard.AutoSizeMode = AutoSizeMode.GrowAndShrink; seqCard.MinimumSize = new Size(350, 220); seqCard.MaximumSize = new Size(600, 0);

            // State indicator
            _lblSprayState = new Label
            {
                Text = "State: idle",
                Font = new Font("Consolas", 14, FontStyle.Bold),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 42),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblSprayState);

            // Approach method
            _lblApproachMethod = new Label
            {
                Text = "Approach: --",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 72),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblApproachMethod);

            // Spray count
            _lblSprayCount = new Label
            {
                Text = "Sprays: 0 / 2",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 94),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblSprayCount);

            // Verification
            _lblVerification = new Label
            {
                Text = "Verified: --",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 116),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblVerification);

            // Targets engaged/succeeded/failed
            _lblSprayTargets = new Label
            {
                Text = "Engaged: 0 | OK: 0 | Fail: 0",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 138),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblSprayTargets);

            // Error
            _lblSprayError = new Label
            {
                Text = "",
                Font = new Font("Consolas", 9),
                ForeColor = ERROR_COLOR,
                Location = new Point(15, 165),
                AutoSize = true,
                MaximumSize = new Size(390, 60),
                Visible = false,
            };
            seqCard.Controls.Add(_lblSprayError);

            // State machine visualization
            var stateFlowLabel = new Label
            {
                Text =
                    "APPROACH (ZED-guided) -> AIM (visual servo)\n"
                    + "  -> SPRAY (500ms pump) -> VERIFY (circle change)\n"
                    + "  -> UPLOAD (Google Drive) -> COMPLETE",
                Font = new Font("Consolas", 8),
                ForeColor = TEXT_MUTED,
                Location = new Point(15, 195),
                AutoSize = true,
            };
            seqCard.Controls.Add(stateFlowLabel);

            layout.Controls.Add(seqCard);

            // ---- Mission Info Card ----
            var infoCard = CreateCard("mission info");
            infoCard.AutoSize = true; infoCard.AutoSizeMode = AutoSizeMode.GrowAndShrink; infoCard.MinimumSize = new Size(350, 100); infoCard.MaximumSize = new Size(600, 0);

            var missionInfo = new Label
            {
                Text =
                    "Task 2: Indoor Fire Extinguishing\n\n"
                    + "Targets are purple circles (5-30cm) that turn BLUE when wet.\n"
                    + "Spray is slightly basic baking soda water.\n"
                    + "Autonomous approach from >2m earns 20 bonus points.\n"
                    + "Photo uploads must be real-time and fully autonomous.",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 42),
                AutoSize = true,
                MaximumSize = new Size(390, 80),
            };
            infoCard.Controls.Add(missionInfo);

            layout.Controls.Add(infoCard);

            // ---- WASD Hint Card ----
            var wasdCard = CreateCard("manual control");
            wasdCard.AutoSize = true; wasdCard.AutoSizeMode = AutoSizeMode.GrowAndShrink; wasdCard.MinimumSize = new Size(350, 55); wasdCard.MaximumSize = new Size(600, 0);

            var wasdHint = new Label
            {
                Text = "Use WASD controls in the Quick Panel until the target is visible in ZED.",
                Font = new Font("Segoe UI", 9),
                ForeColor = TEXT_MUTED,
                Location = new Point(15, 42),
                AutoSize = true,
            };
            wasdCard.Controls.Add(wasdHint);

            layout.Controls.Add(wasdCard);

            panel.Controls.Add(layout);
            return panel;
        }

        // ============================================================
        // Polling: Mode + Spray + VIO + Nav2 + Obstacles + Detections
        // ============================================================
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
            if (IsDisposed || !IsHandleCreated)
                return;

            try
            {
                var modeTask = JetsonApiService.GetAsync("/api/mode");
                var sprayTask = JetsonApiService.GetAsync("/api/spray/status");
                var vioTask = JetsonApiService.GetAsync("/api/vio/status");
                var obstacleTask = JetsonApiService.GetAsync("/api/obstacle_distance");
                var nav2Task = JetsonApiService.GetAsync("/api/nav2/status");
                var detectionTask = JetsonApiService.GetAsync("/api/detections");
                var exclMapTask = JetsonApiService.GetAsync("/api/task/2/exclusion_map");

                await Task.WhenAll(
                    modeTask,
                    sprayTask,
                    vioTask,
                    obstacleTask,
                    nav2Task,
                    detectionTask,
                    exclMapTask
                );

                if (IsDisposed || !IsHandleCreated)
                    return;

                var modeResp = await modeTask;
                var sprayResp = await sprayTask;
                var vioResp = await vioTask;
                var obstacleResp = await obstacleTask;
                var nav2Resp = await nav2Task;
                var detectionResp = await detectionTask;
                var exclMapResp = await exclMapTask;

                JObject modeData = null;
                JObject sprayData = null;
                JObject vioData = null;
                JObject obstacleData = null;
                JObject nav2Data = null;
                JObject detectionData = null;
                JObject exclMapData = null;

                if (modeResp.IsSuccessStatusCode)
                    modeData = JObject.Parse(await modeResp.Content.ReadAsStringAsync());
                if (sprayResp.IsSuccessStatusCode)
                    sprayData = JObject.Parse(await sprayResp.Content.ReadAsStringAsync());
                if (vioResp.IsSuccessStatusCode)
                    vioData = JObject.Parse(await vioResp.Content.ReadAsStringAsync());
                if (obstacleResp.IsSuccessStatusCode)
                    obstacleData = JObject.Parse(
                        await obstacleResp.Content.ReadAsStringAsync()
                    );
                if (nav2Resp.IsSuccessStatusCode)
                    nav2Data = JObject.Parse(await nav2Resp.Content.ReadAsStringAsync());
                if (detectionResp.IsSuccessStatusCode)
                    detectionData = JObject.Parse(
                        await detectionResp.Content.ReadAsStringAsync()
                    );
                if (exclMapResp.IsSuccessStatusCode)
                    exclMapData = JObject.Parse(
                        await exclMapResp.Content.ReadAsStringAsync()
                    );

                if (!IsDisposed && IsHandleCreated)
                {
                    this.BeginInvoke(
                        (Action)(
                            () =>
                                UpdateAllUI(
                                    modeData,
                                    sprayData,
                                    vioData,
                                    obstacleData,
                                    nav2Data,
                                    detectionData,
                                    exclMapData
                                )
                        )
                    );
                }
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
            catch (Exception) { }
        }

        // ============================================================
        // UI Update
        // ============================================================
        private void UpdateAllUI(
            JObject modeData,
            JObject sprayData,
            JObject vioData,
            JObject obstacleData,
            JObject nav2Data,
            JObject detectionData,
            JObject exclMapData
        )
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
                System.Diagnostics.Debug.WriteLine(
                    $"Task2 UI update error: {ex.Message}"
                );
            }
        }

        private void UpdateVioUI(JObject vioData)
        {
            if (vioData == null || _lblVioStatus == null)
                return;

            var health = vioData["health"]?.ToString() ?? "unknown";
            var confidence = vioData["tracking_confidence"]?.Value<double>() ?? 0.0;
            var rateHz = vioData["message_rate_hz"]?.Value<double>() ?? 0.0;
            var source = vioData["source"]?.ToString() ?? "none";

            _lblVioStatus.Text =
                $"VIO: {health} | {confidence * 100.0:F0}% | {rateHz:F1}Hz | {source}";

            if (health == "healthy")
                _lblVioStatus.ForeColor = SUCCESS_COLOR;
            else if (health == "degraded")
                _lblVioStatus.ForeColor = WARNING_COLOR;
            else
                _lblVioStatus.ForeColor = ERROR_COLOR;
        }

        private void UpdateNav2UI(JObject nav2Data)
        {
            if (_lblApproachStatus == null)
                return;

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

            if (status == "navigating" || status == "active")
                _lblApproachStatus.ForeColor = ACCENT_COLOR;
            else if (status == "succeeded")
                _lblApproachStatus.ForeColor = SUCCESS_COLOR;
            else if (status == "failed" || status == "aborted")
                _lblApproachStatus.ForeColor = ERROR_COLOR;
            else if (status == "idle" || status == "pending")
                _lblApproachStatus.ForeColor = TEXT_SECONDARY;
            else
                _lblApproachStatus.ForeColor = TEXT_SECONDARY;
        }

        private void UpdateObstacleUI(JObject obstacleData)
        {
            if (_lblObstacleStatus == null)
                return;

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
                _lblObstacleStatus.Text =
                    $"Obstacles: {meters:F2}m @ {nearestBearing.Value:F0}\u00B0 ({ageS:F1}s)";
                if (meters < 0.5)
                    _lblObstacleStatus.ForeColor = ERROR_COLOR;
                else if (meters < 1.5)
                    _lblObstacleStatus.ForeColor = WARNING_COLOR;
                else
                    _lblObstacleStatus.ForeColor = SUCCESS_COLOR;
            }
            else
            {
                _lblObstacleStatus.Text = $"Obstacles: clear ({ageS:F1}s)";
                _lblObstacleStatus.ForeColor = SUCCESS_COLOR;
            }
        }

        private void UpdateModeUI(JObject modeData)
        {
            if (modeData == null)
                return;

            // Mode selector removed - just display current mode

            var status = modeData["status"];
            if (status != null)
            {
                var currentMode = status["current_mode"]?.ToString() ?? "unknown";
                _lblModeStatus.Text = $"Mode: {currentMode}";

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

        private void UpdateSprayUI(JObject sprayData)
        {
            if (sprayData == null || _lblSprayState == null)
                return;

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

            // State indicator
            _lblSprayState.Text = $"State: {state.ToUpper()}";
            switch (state)
            {
                case "idle":
                    _lblSprayState.ForeColor = TEXT_SECONDARY;
                    break;
                case "approach":
                    _lblSprayState.ForeColor = ACCENT_COLOR;
                    break;
                case "aim":
                    _lblSprayState.ForeColor = WARNING_COLOR;
                    break;
                case "spray":
                    _lblSprayState.ForeColor = ACCENT_COLOR;
                    break;
                case "verify":
                    _lblSprayState.ForeColor = INFO_COLOR;
                    break;
                case "upload":
                    _lblSprayState.ForeColor = INFO_COLOR;
                    break;
                case "complete":
                    _lblSprayState.ForeColor = SUCCESS_COLOR;
                    break;
                case "failed":
                    _lblSprayState.ForeColor = ERROR_COLOR;
                    break;
                case "aborted":
                    _lblSprayState.ForeColor = ERROR_COLOR;
                    break;
                default:
                    _lblSprayState.ForeColor = TEXT_PRIMARY;
                    break;
            }

            // Approach method
            if (state == "approach")
            {
                var approachText = nav2Active
                    ? $"Approach: legacy Nav2 (goal: {(string.IsNullOrEmpty(nav2GoalId) ? "--" : nav2GoalId.Substring(0, Math.Min(8, nav2GoalId.Length)))})"
                    : (approachMethod == "velocity"
                            ? "Approach: direct velocity (no obstacle avoid)"
                            : "Approach: pending");
                _lblApproachMethod.Text = approachText;
                _lblApproachMethod.ForeColor = nav2Active ? ACCENT_COLOR : WARNING_COLOR;
            }
            else
            {
                _lblApproachMethod.Text = string.IsNullOrEmpty(approachMethod)
                    ? "Approach: --"
                    : $"Approach: last={approachMethod}";
                _lblApproachMethod.ForeColor = TEXT_SECONDARY;
            }

            // Spray count
            _lblSprayCount.Text = $"Sprays: {sprayCount} / 2";

            // Verification
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

            // Targets stats
            _lblSprayTargets.Text = $"Engaged: {engaged} | OK: {succeeded} | Fail: {failed}";

            // Distance to selected target
            _lblDistToTarget.Text = $"Distance: {distance:F2}m";
            if (distance > 3.0)
                _lblDistToTarget.ForeColor = ERROR_COLOR;
            else if (distance > 2.0)
                _lblDistToTarget.ForeColor = WARNING_COLOR;
            else
                _lblDistToTarget.ForeColor = SUCCESS_COLOR;

            // Error display
            if (!string.IsNullOrEmpty(error))
            {
                _lblSprayError.Text = $"Error: {error}";
                _lblSprayError.Visible = true;
            }
            else
            {
                _lblSprayError.Visible = false;
            }

            // Button enable/disable
            bool active =
                state != "idle"
                && state != "complete"
                && state != "failed"
                && state != "aborted";
            _btnSprayTarget.Enabled = !active;
            _btnAbortSpray.Enabled = active;
        }

        private void UpdateDetectionUI(JObject detectionData)
        {
            if (detectionData == null || _lstDetections == null)
                return;

            try
            {
                var history = detectionData["history"]?["detections"] as JArray;
                var current = detectionData["current"]?["detections"] as JArray;

                if (history != null)
                {
                    _cachedDetections = history;
                    _lblDetectionCount.Text = $"Targets: {history.Count}";

                    var prevSel = _lstDetections.SelectedIndex;
                    _lstDetections.Items.Clear();

                    foreach (var det in history)
                    {
                        var label = det["label"]?.ToString() ?? "?";
                        var conf = det["confidence"]?.Value<double>() ?? 0;
                        var seen = det["seen_count"]?.Value<int>() ?? 1;
                        var x = det["x"]?.Value<double>() ?? 0;
                        var y = det["y"]?.Value<double>() ?? 0;
                        var z = det["z"]?.Value<double>() ?? 0;

                        _lstDetections.Items.Add(
                            $"{label} conf={conf:F0}% seen={seen} ({x:F1},{y:F1},{z:F1})"
                        );
                    }

                    if (prevSel >= 0 && prevSel < _lstDetections.Items.Count)
                        _lstDetections.SelectedIndex = prevSel;
                    else if (_lstDetections.Items.Count > 0 && _lstDetections.SelectedIndex < 0)
                        _lstDetections.SelectedIndex = 0;
                }
                else
                {
                    _lblDetectionCount.Text = "Targets: 0";
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine(
                    $"Detection UI update error: {ex.Message}"
                );
            }
        }

        private void UpdateExclMapUI(JObject exclMapData)
        {
            if (exclMapData == null || _lblTargetCount == null)
                return;

            try
            {
                var totalTargets = exclMapData["total_targets"]?.Value<int>() ?? 0;
                _lblTargetCount.Text = $"Hit targets: {totalTargets}";
            }
            catch (Exception) { }
        }
        // Detection Refresh
        // ============================================================
        private async Task RefreshDetections()
        {
            try
            {
                _btnRefreshDetections.Enabled = false;
                var resp = await JetsonApiService.GetAsync("/api/detections");
                if (resp.IsSuccessStatusCode)
                {
                    var json = JObject.Parse(await resp.Content.ReadAsStringAsync());
                    UpdateDetectionUI(json);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine(
                    $"Detection refresh error: {ex.Message}"
                );
            }
            finally
            {
                _btnRefreshDetections.Enabled = true;
            }
        }

        // ============================================================
        // Spray Actions
        // ============================================================
        private async Task TriggerSpray()
        {
            if (_sprayInProgress)
                return;
            _sprayInProgress = true;

            try
            {
                _btnSprayTarget.Enabled = false;

                // Get selected detection, or fetch latest
                JToken target = null;
                int selIdx = -1;

                this.BeginInvoke(
                    (Action)(() => { selIdx = _lstDetections.SelectedIndex; })
                );
                await Task.Yield();

                if (
                    selIdx >= 0
                    && selIdx < _cachedDetections.Count
                )
                {
                    target = _cachedDetections[selIdx];
                }
                else
                {
                    // Fetch fresh detections
                    var detectResp = await JetsonApiService.GetAsync("/api/detections");
                    if (!detectResp.IsSuccessStatusCode)
                    {
                        this.BeginInvoke(
                            (Action)(
                                () =>
                                    _lblSprayError.Text = "No detections available"
                            )
                        );
                        _btnSprayTarget.Enabled = true;
                        return;
                    }

                    var detectJson = JObject.Parse(
                        await detectResp.Content.ReadAsStringAsync()
                    );
                    var history = detectJson["history"]?["detections"] as JArray;
                    if (history == null || history.Count == 0)
                    {
                        this.BeginInvoke(
                            (Action)(
                                () =>
                                    _lblSprayError.Text = "No detections -- cannot spray"
                            )
                        );
                        _btnSprayTarget.Enabled = true;
                        return;
                    }

                    target = history[0];
                }

                // Build spray trigger payload
                var payload = new JObject
                {
                    ["target_id"] = target["target_id"] ?? target["id"] ?? 0,
                    ["x"] = target["x"] ?? 0,
                    ["y"] = target["y"] ?? 0,
                    ["z"] = target["z"] ?? 0,
                    ["label"] = target["label"] ?? "",
                    ["confidence"] = target["confidence"] ?? 0,
                };

                var content = new StringContent(
                    payload.ToString(),
                    Encoding.UTF8,
                    "application/json"
                );
                var response = await JetsonApiService.PostAsync("/api/spray/trigger", content);

                if (response.IsSuccessStatusCode)
                {
                    var result = JObject.Parse(
                        await response.Content.ReadAsStringAsync()
                    );
                    var skipApproach = result["skip_approach"]?.Value<bool>() ?? false;
                    var dist = result["distance"]?.Value<double>() ?? 0;

                    this.BeginInvoke(
                        (Action)(
                            () =>
                            {
                                if (skipApproach)
                                    _lblSprayState.Text = "State: SKIPPED APPROACH (already <2m)";
                                else
                                    _lblSprayState.Text = "State: APPROACH STARTING";
                                _lblSprayState.ForeColor = ACCENT_COLOR;
                            }
                        )
                    );
                }
                else
                {
                    var body = await response.Content.ReadAsStringAsync();
                    try
                    {
                        var err = JObject.Parse(body);
                        this.BeginInvoke(
                            (Action)(
                                () =>
                                {
                                    _lblSprayError.Text =
                                        $"Spray failed: {err["detail"]?.ToString() ?? body}";
                                    _lblSprayError.Visible = true;
                                }
                            )
                        );
                    }
                    catch
                    {
                        this.BeginInvoke(
                            (Action)(
                                () =>
                                {
                                    _lblSprayError.Text = $"Spray failed: HTTP {response.StatusCode}";
                                    _lblSprayError.Visible = true;
                                }
                            )
                        );
                    }
                    _btnSprayTarget.Enabled = true;
                }
            }
            catch (Exception ex)
            {
                this.BeginInvoke(
                    (Action)(
                        () =>
                        {
                            _lblSprayError.Text = $"Error: {ex.Message}";
                            _lblSprayError.Visible = true;
                        }
                    )
                );
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
                this.BeginInvoke(
                    (Action)(
                        () =>
                        {
                            _lblSprayState.Text = "State: ABORTED";
                            _lblSprayState.ForeColor = ERROR_COLOR;
                        }
                    )
                );
            }
            catch (Exception ex)
            {
                this.BeginInvoke(
                    (Action)(
                        () =>
                        {
                            _lblSprayError.Text = $"Abort error: {ex.Message}";
                            _lblSprayError.Visible = true;
                        }
                    )
                );
            }
            finally
            {
                _btnAbortSpray.Enabled = true;
            }
        }

        // ============================================================
        // IUpdatableView
        // ============================================================
        public void UpdateData() { }

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
