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
using System.Threading;
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
        private JetsonStateStream _stateStream;

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

        // ---- Detect & Spray tab ----
        private ListBox _lstDetections;
        private Label _lblDetectionCount;
        private Button _btnRefreshDetections;
        private Button _btnAutoSpray;        // Autonomy-gated (1× per mission)
        private Button _btnManualSpray;      // Manual fire (no approach gate)
        private Button _btnAbortSpray;
        private Label _lblDistToTarget;
        private Label _lblAutonomyState;     // "Autonomy gate: not yet claimed" / "Claimed (target X)"
        private Label _lblTargetCount;
        private Button _btnResetMap;
        private Button _btnResetVio;

        private System.Threading.Timer _modePollTimer;
        private int _modePollInFlight;
        private volatile bool _sprayInProgress;
        private JArray _cachedDetections = new JArray();
        // True once an autonomous spray attempt has actually completed
        // successfully. The "Auto Spray" button is hidden after this so
        // the team can't accidentally re-trigger and risk failing the
        // claim on a second attempt.
        private bool _autonomyClaimed;

        public NOMADTask2View(
            DualLinkSender sender,
            NOMADConfig config,
            JetsonConnectionManager jetsonConnectionManager = null)
        {
            _sender = sender;
            _config = config;
            _jetsonConnectionManager = jetsonConnectionManager;
            InitializeUI();
            StartStateStream();
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

            // Tab 2: RTM Checklist (CONOPS Appendix F — 15 pts)
            var rtmTab = new TabPage("RTM SOPs") { BackColor = NOMADTheme.BG_DARK, Padding = new Padding(0) };
            rtmTab.Controls.Add(new RtmChecklistPanel("task2", RtmChecklistPanel.TASK2_ITEMS));
            _tabControl.TabPages.Add(rtmTab);

            // Tab 3: Submit
            var submitTab = new TabPage("Submit") { BackColor = CARD_BG, Padding = new Padding(0) };
            _uploadPanel = new Task2UploadPanel(_config) { Dock = DockStyle.Fill };
            submitTab.Controls.Add(_uploadPanel);
            _tabControl.TabPages.Add(submitTab);

            // Tab 4: Status
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
                // Re-pin task2 in case Task 1 (or anything else) switched the
                // bridge while this view was hidden.
                _ = SetOverlayModeAsync("task2");
            }
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
                Height = 280 + 175 + 150 + 80,
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
            _btnRefreshDetections.Click += (s, e) => UiAsync.Run(this, RefreshDetections, nameof(RefreshDetections));
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

            // ---- Spray card (autonomy-claim flow + manual flow) ----
            // Strategy for Task 2 scoring (CONOPS §5.2.4 + Q&A #10):
            //   - Pick ONE outdoor target near the doorway and run the
            //     autonomous spray sequence on it. That claims the
            //     20-pt autonomy gate.
            //   - Spray every other target manually (no autonomous
            //     approach), maximising the chance of hits on
            //     additional indoor + outdoor circles.
            // The two buttons below enforce this split.
            var sprayCard = CreateCard("SPRAY CONTROLS");
            sprayCard.Dock = DockStyle.Top;
            sprayCard.Height = 175;

            _lblAutonomyState = new Label
            {
                Text = "Autonomy gate: NOT CLAIMED — use Auto Spray on the first target",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = WARNING_COLOR,
                Location = new Point(15, 36),
                AutoSize = true,
            };
            sprayCard.Controls.Add(_lblAutonomyState);

            _btnAutoSpray = CreateButton("Auto Spray (1×)", ACCENT_COLOR, 180, 42);
            _btnAutoSpray.Location = new Point(15, 60);
            _btnAutoSpray.Font = new Font("Segoe UI", 11, FontStyle.Bold);
            _btnAutoSpray.Click += (s, e) => UiAsync.Run(this, TriggerAutoSpray, nameof(TriggerAutoSpray));
            sprayCard.Controls.Add(_btnAutoSpray);

            _btnManualSpray = CreateButton("Manual Spray", INFO_COLOR, 140, 42);
            _btnManualSpray.Location = new Point(205, 60);
            _btnManualSpray.Font = new Font("Segoe UI", 11, FontStyle.Bold);
            _btnManualSpray.Click += (s, e) => UiAsync.Run(this, TriggerManualSpray, nameof(TriggerManualSpray));
            sprayCard.Controls.Add(_btnManualSpray);

            _btnAbortSpray = CreateButton("ABORT", ERROR_COLOR, 90, 42);
            _btnAbortSpray.Location = new Point(355, 60);
            _btnAbortSpray.Font = new Font("Segoe UI", 11, FontStyle.Bold);
            _btnAbortSpray.Click += (s, e) => UiAsync.Run(this, AbortSpray, nameof(AbortSpray));
            sprayCard.Controls.Add(_btnAbortSpray);

            sprayCard.Controls.Add(new Label
            {
                Text = "Auto Spray:   pick outdoor target, drone ≥2.5m from it, full APPROACH→AIM→SPRAY→VERIFY→UPLOAD.\n"
                     + "Manual Spray: pilot is in firing range, water pump fires; servo aims at last detection.\n"
                     + "ABORT:        cancels any in-flight sequence and re-arms both buttons.",
                Font = new Font("Consolas", 8),
                ForeColor = TEXT_MUTED,
                Location = new Point(15, 112),
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
            _btnResetMap.Click += (s, e) => UiAsync.Run(this, async () =>
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
            }, "ResetTask2Map");
            mapCard.Controls.Add(_btnResetMap);

            _btnResetVio = CreateButton("Reset VIO", ERROR_COLOR, 100, 28);
            _btnResetVio.Location = new Point(285, 38);
            _btnResetVio.Font = new Font("Segoe UI", 9, FontStyle.Bold);
            _btnResetVio.Click += (s, e) => UiAsync.Run(this, () => _sender.ResetVioOriginAsync(), "ResetVioOrigin");
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

        private void StartStateStream()
        {
            _stateStream = JetsonStateStream.Shared;
            _stateStream.Configure(_config);
            _stateStream.StateUpdated += OnStateStreamUpdated;
            _stateStream.Start();
        }

        private void OnStateStreamUpdated(JObject state)
        {
            if (IsDisposed || !IsHandleCreated || state == null) return;
            try
            {
                BeginInvoke((Action)(() => UpdateFromStateStream(state)));
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
        }

        private void UpdateFromStateStream(JObject state)
        {
            UpdateModeUI(state["operational_mode"] as JObject);
            UpdateSprayUI(state["spray_status"] as JObject);
            UpdateVioUI(state["vio_status"] as JObject);
            UpdateObstacleUI(state["obstacle_distance"] as JObject);
        }

        private void PollModeAndSpray()
        {
            UiAsync.Run(this, PollModeAndSprayAsync, nameof(PollModeAndSpray));
        }

        private async Task PollModeAndSprayAsync()
        {
            if (IsDisposed || !IsHandleCreated) return;
            if (Interlocked.Exchange(ref _modePollInFlight, 1) == 1) return;
            try
            {
                JObject modeData = null;
                JObject sprayData = null;
                JObject vioData = null;
                JObject obstacleData = null;
                var state = _stateStream?.LatestState;
                if (_stateStream?.HasFreshState == true && state != null)
                {
                    modeData = state["operational_mode"] as JObject;
                    sprayData = state["spray_status"] as JObject;
                    vioData = state["vio_status"] as JObject;
                    obstacleData = state["obstacle_distance"] as JObject;
                }
                else
                {
                    var modeTask = JetsonApiService.GetAsync("/api/mode");
                    var sprayTask = JetsonApiService.GetAsync("/api/spray/status");
                    var vioTask = JetsonApiService.GetAsync("/api/vio/status");
                    var obstacleTask = JetsonApiService.GetAsync("/api/obstacle_distance");

                    await Task.WhenAll(modeTask, sprayTask, vioTask, obstacleTask);
                    modeData = await ReadJson(modeTask);
                    sprayData = await ReadJson(sprayTask);
                    vioData = await ReadJson(vioTask);
                    obstacleData = await ReadJson(obstacleTask);
                }

                var detectionTask = JetsonApiService.GetAsync("/api/task/2/detections");
                var exclMapTask = JetsonApiService.GetAsync("/api/task/2/exclusion_map");

                await Task.WhenAll(detectionTask, exclMapTask);
                if (IsDisposed || !IsHandleCreated) return;

                JObject detectionData = await ReadJson(detectionTask);
                JObject exclMapData = await ReadJson(exclMapTask);

                if (!IsDisposed && IsHandleCreated)
                {
                    BeginInvoke((Action)(() => UpdateAllUI(
                        modeData, sprayData, vioData, obstacleData,
                        detectionData, exclMapData)));
                }
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
            catch (Exception) { }
            finally
            {
                Interlocked.Exchange(ref _modePollInFlight, 0);
            }
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
            JObject obstacleData, JObject detectionData,
            JObject exclMapData)
        {
            try
            {
                UpdateVioUI(vioData);
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

        // Nav2 was removed; the spray controller now drives APPROACH via
        // visual servoing. UpdateSprayUI already reflects the chosen method.

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
                _lblApproachMethod.Text = approachMethod switch
                {
                    "image"    => "Approach: visual servoing (image)",
                    "velocity" => "Approach: direct velocity (no obstacle avoid)",
                    _          => "Approach: pending",
                };
                _lblApproachMethod.ForeColor = WARNING_COLOR;
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
            // Autonomy claim is recorded when a sequence that included
            // an APPROACH phase reaches the verified state. The
            // backend doesn't currently surface the "required_autonomy"
            // flag back on /api/spray/status; we proxy it via the fact
            // that a verified target was engaged via APPROACH (i.e.
            // approach_method is "image" or "velocity" — both started
            // from outside the 2 m envelope).
            if (state == "complete" && verified && !string.IsNullOrEmpty(approachMethod) && !_autonomyClaimed)
            {
                _autonomyClaimed = true;
                _lblAutonomyState.Text = $"Autonomy gate: CLAIMED (target {sprayData["target_id"]?.Value<int>() ?? -1}) ✓";
                _lblAutonomyState.ForeColor = SUCCESS_COLOR;
            }

            _btnAutoSpray.Enabled = !active && !_autonomyClaimed;
            _btnManualSpray.Enabled = !active;
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
                    bool imageOnly = det["image_only"]?.Value<bool>() ?? false;
                    string posStr;
                    if (imageOnly)
                    {
                        // Task 2 shape detector is 2D-only — show pixel
                        // coords + range so the list isn't just "(0,0,0)".
                        var px = det["pixel_x"]?.Value<double>() ?? 0;
                        var py = det["pixel_y"]?.Value<double>() ?? 0;
                        var rngTok = det["range_m"];
                        bool hasRng = rngTok != null && rngTok.Type != JTokenType.Null;
                        posStr = hasRng
                            ? $"px=({px:F0},{py:F0}) r={rngTok.Value<double>():F2}m"
                            : $"px=({px:F0},{py:F0})";
                    }
                    else
                    {
                        posStr = $"({x:F1},{y:F1},{z:F1})";
                    }
                    _lstDetections.Items.Add($"{label} conf={conf:F0}% {source} seen={seen} {posStr}");
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
            catch (Exception ex) { System.Diagnostics.Debug.WriteLine(ex); }
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

        private Task TriggerAutoSpray() => TriggerSprayInternal(requireAutonomy: true);
        private Task TriggerManualSpray() => TriggerSprayInternal(requireAutonomy: false);

        private async Task TriggerSprayInternal(bool requireAutonomy)
        {
            if (_sprayInProgress) return;
            _sprayInProgress = true;
            try
            {
                _btnAutoSpray.Enabled = false;
                _btnManualSpray.Enabled = false;

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
                        ReenableSprayButtons();
                        return;
                    }
                    var detectJson = JObject.Parse(await detectResp.Content.ReadAsStringAsync());
                    var current = detectJson["current"]?["detections"] as JArray;
                    var history = detectJson["history"]?["detections"] as JArray;
                    var detections = (current != null && current.Count > 0) ? current : history;
                    if (detections == null || detections.Count == 0)
                    {
                        BeginInvoke((Action)(() => _lblSprayError.Text = "No detections -- cannot spray"));
                        ReenableSprayButtons();
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
                    ["require_autonomy"] = requireAutonomy,
                };

                var content = new StringContent(payload.ToString(), Encoding.UTF8, "application/json");
                var response = await JetsonApiService.PostAsync("/api/spray/trigger", content);

                if (response.IsSuccessStatusCode)
                {
                    var result = JObject.Parse(await response.Content.ReadAsStringAsync());
                    var skipApproach = result["skip_approach"]?.Value<bool>() ?? false;
                    BeginInvoke((Action)(() =>
                    {
                        if (requireAutonomy)
                        {
                            _lblSprayState.Text = "State: APPROACH (autonomy claim in progress)";
                            _lblAutonomyState.Text = "Autonomy gate: claim in progress…";
                            _lblAutonomyState.ForeColor = ACCENT_COLOR;
                        }
                        else
                        {
                            _lblSprayState.Text = skipApproach
                                ? "State: MANUAL SPRAY (no approach)"
                                : "State: APPROACH (manual mode)";
                        }
                        _lblSprayState.ForeColor = ACCENT_COLOR;
                        _lblSprayError.Visible = false;
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
                    ReenableSprayButtons();
                }
            }
            catch (Exception ex)
            {
                BeginInvoke((Action)(() =>
                {
                    _lblSprayError.Text = $"Error: {ex.Message}";
                    _lblSprayError.Visible = true;
                }));
                ReenableSprayButtons();
            }
            finally { _sprayInProgress = false; }
        }

        private void ReenableSprayButtons()
        {
            BeginInvoke((Action)(() =>
            {
                _btnAutoSpray.Enabled = !_autonomyClaimed;
                _btnManualSpray.Enabled = true;
            }));
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
                // Best-effort: turn detectors off when the view closes so
                // overlay processing isn't left running.
                _ = DisableOverlayDetectorsAsync();
                _modePollTimer?.Dispose();
                if (_stateStream != null)
                {
                    _stateStream.StateUpdated -= OnStateStreamUpdated;
                    _stateStream = null;
                }
                _slam3DView?.Dispose();
                _videoPlayer?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
