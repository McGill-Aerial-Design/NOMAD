// ============================================================
// NOMAD Task 2 View - Indoor Fire Extinguishing
// ============================================================
// Layout uses a full-width tabbed workflow so Task 2 controls are not
// constrained by a side-by-side video/control split.
//
// Tabs (right column):
//   1. Detect & Spray  - autonomous flight workflow with status,
//                        detections, spray controls, payload, and
//                        autonomous artifacts.
//   2. Pre-Flight      - backend readiness checks and target color config.
//   3. Manual Spray    - Task2UploadPanel manual flow.
//   4. RTM SOPs        - checklist.
//   5. 3D SLAM         - SLAM3DView.
// Layout mirrors Task 1: live camera feed on the left, tabs on the right.
//
// Tilt lock is driven from spray state (UpdateSprayUI) - the
// slider is locked only while the autonomous spray sequence
// is mid-run, and unlocked otherwise.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
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
        private Button _btnAutoSpray;        // Autonomy-gated (1x per mission)
        private Button _btnAbortSpray;
        private Label _lblDistToTarget;
        private Label _lblAutonomyState;     // "Autonomy gate: not yet claimed" / "Claimed (target X)"

        private System.Threading.Timer _modePollTimer;
        private int _modePollInFlight;
        private volatile bool _sprayInProgress;
        private JArray _cachedDetections = new JArray();
        // True once an autonomous spray attempt has actually completed
        // successfully. The "Auto Spray" button is hidden after this so
        // the team can't accidentally re-trigger and risk failing the
        // claim on a second attempt.
        private bool _autonomyClaimed;

        // Spray-state transition tracking for TTS + popup. We only speak on
        // state CHANGES so the operator doesn't get spammed at the 0.5s poll
        // rate. _lastConfirmedTargetId guards against re-firing the popup
        // for the same target if the poll lands on COMPLETE more than once.
        private string _lastSprayState = "idle";
        private bool _lastAutonomyCompromised;
        private int _lastConfirmedTargetId = -1;
        private string _localSprayErrorMessage = "";
        private Button _btnAutoTakeoff;
        private Button _btnAutoLand;
        private NumericUpDown _numTakeoffAltitude;
        private Task2PreflightPanel _preflightPanel;

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
            BackColor = NOMADTheme.BG_DARK;
            Padding = Padding.Empty;

            // Two-column layout (mirrors Task 1): live video on the left,
            // tabbed controls on the right.
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                Margin = Padding.Empty,
                Padding = Padding.Empty,
            };
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            // ========== LEFT COLUMN: Video ==========
            mainLayout.Controls.Add(CreateCameraPanel(), 0, 0);

            // ========== RIGHT COLUMN: Tabs ==========
            _tabControl = new TabControl
            {
                Dock = DockStyle.Fill,
                Margin = new Padding(5),
            };
            StyleTabControl(_tabControl);

            var detectTab = new TabPage("Detect & Spray") { BackColor = CARD_BG, Padding = new Padding(0) };
            detectTab.Controls.Add(CreateDetectSprayPanel());
            _tabControl.TabPages.Add(detectTab);

            var preflightTab = new TabPage("Pre-Flight") { BackColor = NOMADTheme.BG_DARK, Padding = new Padding(0) };
            _preflightPanel = new Task2PreflightPanel { Dock = DockStyle.Fill };
            preflightTab.Controls.Add(_preflightPanel);
            _tabControl.TabPages.Add(preflightTab);

            var manualTab = new TabPage("Manual Spray") { BackColor = CARD_BG, Padding = new Padding(0) };
            manualTab.Controls.Add(CreateManualSprayPanel());
            _tabControl.TabPages.Add(manualTab);

            var rtmTab = new TabPage("RTM SOPs") { BackColor = NOMADTheme.BG_DARK, Padding = new Padding(0) };
            rtmTab.Controls.Add(new RtmChecklistPanel("task2", RtmChecklistPanel.TASK2_ITEMS));
            _tabControl.TabPages.Add(rtmTab);

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

            this.AutoScroll = false;
            this.Controls.Add(mainLayout);
        }

        private Panel CreateCameraPanel()
        {
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
                _ = EnableTask2OverlayAsync();
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
            return videoPanel;
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

        /// <summary>Enable overlay + Task 2 detector together so the circle
        /// detector is guaranteed to be running while this view is open.</summary>
        private async Task EnableTask2OverlayAsync()
        {
            try
            {
                // /overlay/detectors enables the overlay by itself. Avoid
                // /overlay/enable here because that legacy endpoint briefly
                // selects the Task 1 HSV detector before this Task 2 command.
                await JetsonApiService.PostAsync(
                    "/api/video/overlay/detectors?task1=false&task2=true");
                await JetsonApiService.PostAsync("/api/video/overlay/mode?mode=task2");
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
                // bridge while this view was hidden. Use the full enable path
                // so the detector is guaranteed on (not just the mode hint).
                _ = EnableTask2OverlayAsync();
            }
        }

        // ============================================================
        // Detect & Spray tab
        // ============================================================
        // Single-column layout. The outer panel is the only scrollable
        // container so the tab cannot clip half-width cards. Exclusion map /
        // reset-map / reset-vio are gone; they were leftovers from a
        // navigation flow we no longer use.
        private Panel CreateDetectSprayPanel()
        {
            var root = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                AutoScroll = true,
                Padding = new Padding(8),
            };

            var stack = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                BackColor = CARD_BG,
                Margin = Padding.Empty,
                Padding = Padding.Empty,
            };

            void SizeToStack(Control card)
            {
                if (card.IsDisposed) return;
                var available = root.ClientSize.Width - root.Padding.Horizontal - SystemInformation.VerticalScrollBarWidth - 4;
                card.Width = Math.Max(470, available);
            }

            void AddCard(Control card, int height)
            {
                card.Height = height;
                card.Margin = new Padding(0, 0, 0, 10);
                SizeToStack(card);
                root.Resize += (s, e) => SizeToStack(card);
                stack.Controls.Add(card);
            }

            // DETECTED TARGETS card
            var detectCard = CreateCard("DETECTED TARGETS");

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
                Size = new Size(380, 118),
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
                Location = new Point(15, 192),
                AutoSize = true,
            };
            detectCard.Controls.Add(_lblDistToTarget);
            AddCard(detectCard, 226);

            // SPRAY SEQUENCE STATUS card
            var seqCard = CreateCard("SPRAY SEQUENCE STATUS");

            _lblSprayState = new Label
            {
                Text = "State: idle",
                Font = new Font("Consolas", 14, FontStyle.Bold),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 38),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblSprayState);

            _lblApproachMethod = new Label
            {
                Text = "Approach: --",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 70),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblApproachMethod);

            _lblSprayCount = new Label
            {
                Text = "Sprays: 0 / 2",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 92),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblSprayCount);

            _lblVerification = new Label
            {
                Text = "Verified: --",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_SECONDARY,
                Location = new Point(15, 114),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblVerification);

            _lblSprayTargets = new Label
            {
                Text = "Engaged: 0 | OK: 0 | Fail: 0",
                Font = new Font("Consolas", 10),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 136),
                AutoSize = true,
            };
            seqCard.Controls.Add(_lblSprayTargets);

            _lblSprayError = new Label
            {
                Text = "",
                Font = new Font("Consolas", 9, FontStyle.Bold),
                ForeColor = ERROR_COLOR,
                Location = new Point(15, 165),
                AutoSize = true,
                MaximumSize = new Size(450, 60),
                Visible = false,
            };
            seqCard.Controls.Add(_lblSprayError);
            AddCard(seqCard, 210);

            // ---- Spray card (autonomy-claim flow + manual flow) ----
            // Strategy for Task 2 scoring (CONOPS section 5.2.4 + Q&A #10):
            //   - Pick ONE outdoor target near the doorway and run the
            //     autonomous spray sequence on it. That claims the
            //     20-pt autonomy gate.
            //   - Spray every other target manually (no autonomous
            //     approach), maximising the chance of hits on
            //     additional indoor + outdoor circles.
            // The two buttons below enforce this split.
            var sprayCard = CreateCard("SPRAY CONTROLS");

            _lblAutonomyState = new Label
            {
                Text = "Autonomy gate: NOT CLAIMED - use Auto Spray on the first target",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = WARNING_COLOR,
                Location = new Point(15, 36),
                AutoSize = true,
            };
            sprayCard.Controls.Add(_lblAutonomyState);

            _btnAutoSpray = CreateButton("Auto Spray (1x)", ACCENT_COLOR, 180, 42);
            _btnAutoSpray.Location = new Point(15, 60);
            _btnAutoSpray.Font = new Font("Segoe UI", 11, FontStyle.Bold);
            _btnAutoSpray.Click += (s, e) => UiAsync.Run(this, TriggerAutoSpray, nameof(TriggerAutoSpray));
            sprayCard.Controls.Add(_btnAutoSpray);

            _btnAbortSpray = CreateButton("ABORT", ERROR_COLOR, 90, 42);
            _btnAbortSpray.Location = new Point(205, 60);
            _btnAbortSpray.Font = new Font("Segoe UI", 11, FontStyle.Bold);
            _btnAbortSpray.Click += (s, e) => UiAsync.Run(this, AbortSpray, nameof(AbortSpray));
            sprayCard.Controls.Add(_btnAbortSpray);

            sprayCard.Controls.Add(new Label
            {
                Text = "Auto Spray: pick outdoor target, drone >=2.5m from it, full APPROACH->AIM->SPRAY->VERIFY->UPLOAD.\n"
                     + "ABORT:      cancels any in-flight sequence and re-arms Auto Spray.\n"
                     + "For hand-fired spray runs, switch to the Manual Spray tab.",
                Font = new Font("Consolas", 8),
                ForeColor = TEXT_MUTED,
                Location = new Point(15, 112),
                AutoSize = true,
            });
            AddCard(sprayCard, 172);

            // Payload (cam tilt + shoot water).
            _payloadPanel = new Task2PayloadPanel(_config);
            AddCard(_payloadPanel, 145);

            // AUTO-UPLOAD (compact view of Task2UploadPanel). Mounted here so
            // the operator sees the autonomous spray's before/after artifacts
            // uploading to Drive without tab-switching.
            _uploadPanel = new Task2UploadPanel(_config, Task2UploadPanel.PanelMode.Auto);
            AddCard(_uploadPanel, 350);

            // ---- Auto Takeoff / Auto Land card ----
            // Kept last so the operator works top-to-bottom: select target,
            // watch spray/upload, then use flight-window takeoff/landing
            // controls from the bottom of the tab.
            var flightCard = CreateCard("AUTONOMOUS TAKEOFF / LAND");

            flightCard.Controls.Add(new Label
            {
                Text = "Takeoff altitude (m AGL)",
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 38),
                AutoSize = true,
            });

            _numTakeoffAltitude = new NumericUpDown
            {
                Minimum = 1,
                Maximum = 60,
                DecimalPlaces = 1,
                Increment = 1,
                Value = 30,
                Width = 90,
                Location = new Point(170, 35),
                BackColor = NOMADTheme.INPUT_BG,
                ForeColor = TEXT_PRIMARY,
                Font = new Font("Consolas", 10, FontStyle.Bold),
            };
            flightCard.Controls.Add(_numTakeoffAltitude);

            _btnAutoTakeoff = CreateButton("Auto Takeoff", ACCENT_COLOR, 150, 42);
            _btnAutoTakeoff.Location = new Point(15, 72);
            _btnAutoTakeoff.Font = new Font("Segoe UI", 11, FontStyle.Bold);
            _btnAutoTakeoff.Click += (s, e) => UiAsync.Run(this, TriggerAutoTakeoff, nameof(TriggerAutoTakeoff));
            flightCard.Controls.Add(_btnAutoTakeoff);

            _btnAutoLand = CreateButton("Auto Land", INFO_COLOR, 140, 42);
            _btnAutoLand.Location = new Point(180, 72);
            _btnAutoLand.Font = new Font("Segoe UI", 11, FontStyle.Bold);
            _btnAutoLand.Click += (s, e) => UiAsync.Run(this, TriggerAutoLand, nameof(TriggerAutoLand));
            flightCard.Controls.Add(_btnAutoLand);

            flightCard.Controls.Add(new Label
            {
                Text = "Takeoff switches to GUIDED, arms, then climbs to the selected altitude. Land switches to LAND mode.\n"
                     + "Flip the RC mode switch any time to override.",
                Font = new Font("Consolas", 8),
                ForeColor = TEXT_MUTED,
                Location = new Point(15, 126),
                AutoSize = true,
            });
            AddCard(flightCard, 176);

            root.Controls.Add(stack);
            return root;
        }

        // ============================================================
        // Manual Spray tab - pilot positions in firing range, aims via
        // tilt slider, and fires by hand. Below the controls, the
        // Task2UploadPanel handles the manual session (before/after
        // capture + video record + Drive upload).
        // ============================================================
        private Panel CreateManualSprayPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.BG_DARK,
                AutoScroll = true,
                Padding = new Padding(0),
            };

            // Upload panel (Manual mode: start/stop session, before/after,
            // video, abort, and Drive upload all visible).
            var manualUpload = new Task2UploadPanel(_config, Task2UploadPanel.PanelMode.Manual)
            {
                Dock = DockStyle.Fill,
            };

            // Payload controls (tilt slider + shoot water). Docked to top
            // so it sits above the upload panel and the slider stays in
            // reach while the manual session is open.
            var manualPayload = new Task2PayloadPanel(_config)
            {
                Dock = DockStyle.Top,
                Height = 165,
            };

            panel.Controls.Add(manualUpload);
            panel.Controls.Add(manualPayload);
            return panel;
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
            UpdateSprayUI(state["spray_status"] as JObject);
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
                JObject sprayData = null;
                var state = _stateStream?.LatestState;
                if (_stateStream?.HasFreshState == true && state != null)
                {
                    sprayData = state["spray_status"] as JObject;
                }
                else
                {
                    var sprayTask = JetsonApiService.GetAsync("/api/spray/status");
                    await sprayTask;
                    sprayData = await ReadJson(sprayTask);
                }

                var detectionTask = JetsonApiService.GetAsync("/api/task/2/detections");
                await detectionTask;
                if (IsDisposed || !IsHandleCreated) return;

                JObject detectionData = await ReadJson(detectionTask);

                if (!IsDisposed && IsHandleCreated)
                {
                    BeginInvoke((Action)(() => UpdateAllUI(sprayData, detectionData)));
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

        private void UpdateAllUI(JObject sprayData, JObject detectionData)
        {
            try
            {
                UpdateSprayUI(sprayData);
                UpdateDetectionUI(detectionData);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Task2 UI update error: {ex.Message}");
            }
        }

        // ============================================================
        // UI updates
        // ============================================================
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
            var requireAutonomy = sprayData["require_autonomy"]?.Value<bool>() ?? false;
            var autonomyCompromised = sprayData["autonomy_compromised"]?.Value<bool>() ?? false;
            var targetId = sprayData["target_id"]?.Value<int>() ?? -1;

            // --- TTS on state TRANSITIONS only (not every poll) ---
            if (state != _lastSprayState)
            {
                AnnounceSprayState(state, requireAutonomy, verified, targetId);
                _lastSprayState = state;
            }
            // Pilot override detection: speak ONCE when the flag flips on.
            if (autonomyCompromised && !_lastAutonomyCompromised)
            {
                AudioAlerts.Speak(
                    "Autonomy override detected. Pilot has manual control. Claim forfeited for this target.",
                    ignoreRateLimit: true);
            }
            _lastAutonomyCompromised = autonomyCompromised;

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
            else if (!string.IsNullOrEmpty(_localSprayErrorMessage))
            {
                _lblSprayError.Text = _localSprayErrorMessage;
                _lblSprayError.Visible = true;
            }
            else
            {
                _lblSprayError.Visible = false;
            }

            bool active = ACTIVE_SPRAY_STATES.Contains(state);
            if (state == "complete"
                && verified
                && requireAutonomy
                && !autonomyCompromised
                && !_autonomyClaimed)
            {
                _autonomyClaimed = true;
                _lblAutonomyState.Text = $"Autonomy gate: CLAIMED (target {targetId})";
                _lblAutonomyState.ForeColor = SUCCESS_COLOR;
            }

            if (state == "complete"
                && verified
                && requireAutonomy
                && !autonomyCompromised
                && targetId != _lastConfirmedTargetId)
            {
                _lastConfirmedTargetId = targetId;
                ShowExtinguishedConfirmation(targetId);
            }

            _btnAutoSpray.Enabled = !active && !_autonomyClaimed;
            _btnAbortSpray.Enabled = active;

            // Spray-state-driven tilt lock - only active when spray is mid-run.
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
                        // Task 2 shape detector is 2D-only - show pixel
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

        private async Task TriggerSprayInternal(bool requireAutonomy)
        {
            if (_sprayInProgress) return;
            _sprayInProgress = true;
            try
            {
                _btnAutoSpray.Enabled = false;
                if (requireAutonomy)
                {
                    AudioAlerts.Speak(
                        "Autonomous spray requested. Switching to guided and starting the approach.",
                        ignoreRateLimit: true);
                }
                // Clear any previous error banner so the operator can
                // see the new failure (or that the trigger actually fired).
                ShowSprayError("");

                JToken target = null;
                // UiAsync.Run already invokes us on the UI thread (it uses
                // ConfigureAwait(true)) so we can read the selection directly
                // - the old BeginInvoke + Task.Yield was a race that almost
                // always returned -1 and silently fell through to the
                // fetch-from-server path.
                int selIdx = _lstDetections.SelectedIndex;

                if (selIdx >= 0 && selIdx < _cachedDetections.Count)
                {
                    target = _cachedDetections[selIdx];
                }
                else
                {
                    var detectResp = await JetsonApiService.GetAsync("/api/task/2/detections");
                    if (!detectResp.IsSuccessStatusCode)
                    {
                        var msg = $"Detector fetch failed (HTTP {(int)detectResp.StatusCode}). " +
                                  "Check Jetson + verify the Task 2 overlay is enabled.";
                        ShowSprayError(msg);
                        AudioAlerts.Speak("Detector fetch failed. Check Jetson and verify the Task 2 overlay is enabled.",
                            ignoreRateLimit: true);
                        ReenableSprayButtons();
                        return;
                    }
                    var detectJson = JObject.Parse(await detectResp.Content.ReadAsStringAsync());
                    var current = detectJson["current"]?["detections"] as JArray;
                    var history = detectJson["history"]?["detections"] as JArray;
                    var detections = (current != null && current.Count > 0) ? current : history;
                    if (detections == null || detections.Count == 0)
                    {
                        var msg = "No detections - the shape detector isn't returning circles. " +
                                  "Verify the video overlay shows boxed circles, then press Refresh before triggering.";
                        ShowSprayError(msg);
                        AudioAlerts.Speak("No Task 2 circle detections. Verify boxed circles on the video, then refresh detections.",
                            ignoreRateLimit: true);
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
                    if (requireAutonomy)
                    {
                        _lblSprayState.Text = "State: APPROACH (autonomy claim in progress)";
                        _lblAutonomyState.Text = "Autonomy gate: claim in progress...";
                        _lblAutonomyState.ForeColor = ACCENT_COLOR;
                        AudioAlerts.Speak(
                            "Autonomous spray sequence active. Hands off sticks unless overriding for safety.",
                            ignoreRateLimit: true);
                    }
                    else
                    {
                        _lblSprayState.Text = skipApproach
                            ? "State: MANUAL SPRAY (no approach)"
                            : "State: APPROACH (manual mode)";
                        AudioAlerts.Speak("Spray sequence started.", ignoreRateLimit: true);
                    }
                    _lblSprayState.ForeColor = ACCENT_COLOR;
                    ShowSprayError("");
                }
                else
                {
                    var body = await response.Content.ReadAsStringAsync();
                    string detail;
                    try { detail = JObject.Parse(body)["detail"]?.ToString() ?? body; }
                    catch { detail = body; }
                    ShowSprayError($"Spray failed: {detail}");
                    AudioAlerts.Speak("Spray trigger failed. Check the error banner.", ignoreRateLimit: true);
                    ReenableSprayButtons();
                }
            }
            catch (Exception ex)
            {
                ShowSprayError($"Error: {ex.Message}");
                AudioAlerts.Speak("Spray trigger error.", ignoreRateLimit: true);
                ReenableSprayButtons();
            }
            finally { _sprayInProgress = false; }
        }

        /// <summary>Surface an error in the spray-status card. Pass empty
        /// string to hide. Centralised so every failure path actually
        /// flips Visible=true - the old paths set Text without making the
        /// label visible, which is why Auto Spray looked silent.</summary>
        private void ShowSprayError(string message)
        {
            if (_lblSprayError == null) return;
            if (InvokeRequired)
            {
                BeginInvoke((Action)(() => ShowSprayError(message)));
                return;
            }
            if (string.IsNullOrEmpty(message))
            {
                _localSprayErrorMessage = "";
                _lblSprayError.Visible = false;
                return;
            }
            _localSprayErrorMessage = message;
            _lblSprayError.Text = message;
            _lblSprayError.Visible = true;
        }

        private void ReenableSprayButtons()
        {
            BeginInvoke((Action)(() =>
            {
                _btnAutoSpray.Enabled = !_autonomyClaimed;
            }));
        }

        private async Task AbortSpray()
        {
            try
            {
                _btnAbortSpray.Enabled = false;
                await JetsonApiService.PostAsync("/api/spray/abort");
                ShowSprayError("");
                _lblSprayState.Text = "State: ABORTED";
                _lblSprayState.ForeColor = ERROR_COLOR;
                AudioAlerts.Speak("Spray sequence aborted.", ignoreRateLimit: true);
            }
            catch (Exception ex)
            {
                ShowSprayError($"Abort error: {ex.Message}");
            }
            finally { _btnAbortSpray.Enabled = true; }
        }

        private async Task TriggerAutoTakeoff()
        {
            var altitudeM = _numTakeoffAltitude != null ? (double)_numTakeoffAltitude.Value : 30.0;
            var confirm = MessageBox.Show(
                $"Auto takeoff will switch the autopilot to GUIDED, arm motors, and climb to {altitudeM:F1} m AGL.\n\n" +
                "Keep the RC mode switch ready for override.",
                "Confirm Auto Takeoff",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning);
            if (confirm != DialogResult.Yes) return;

            try
            {
                _btnAutoTakeoff.Enabled = false;
                AudioAlerts.Speak($"Auto takeoff requested to {altitudeM:F0} meters. Switching to guided and arming.", ignoreRateLimit: true);

                var payload = new JObject { ["altitude_m"] = altitudeM };
                var content = new StringContent(payload.ToString(), Encoding.UTF8, "application/json");
                var resp = await JetsonApiService.PostAsync("/api/flight/takeoff", content);
                if (!resp.IsSuccessStatusCode)
                {
                    ShowSprayError("Auto takeoff failed: " + await ExtractError(resp));
                    AudioAlerts.Speak("Auto takeoff failed.", ignoreRateLimit: true);
                    return;
                }

                _lblSprayState.Text = "State: AUTO TAKEOFF COMMANDED";
                _lblSprayState.ForeColor = SUCCESS_COLOR;
                AudioAlerts.Speak($"Auto takeoff commanded to {altitudeM:F0} meters.", ignoreRateLimit: true);
            }
            catch (Exception ex)
            {
                ShowSprayError($"Auto takeoff error: {ex.Message}");
                AudioAlerts.Speak("Auto takeoff error.", ignoreRateLimit: true);
            }
            finally
            {
                _btnAutoTakeoff.Enabled = true;
            }
        }

        private async Task TriggerAutoLand()
        {
            var confirm = MessageBox.Show(
                "Auto land will switch the autopilot to LAND mode and descend at the current position.\n\n" +
                "Use only when the aircraft is over the landing area.",
                "Confirm Auto Land",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning);
            if (confirm != DialogResult.Yes) return;

            try
            {
                _btnAutoLand.Enabled = false;
                AudioAlerts.Speak("Auto land requested.", ignoreRateLimit: true);

                var resp = await JetsonApiService.PostAsync("/api/flight/land");
                if (!resp.IsSuccessStatusCode)
                {
                    ShowSprayError("Auto land failed: " + await ExtractError(resp));
                    AudioAlerts.Speak("Auto land failed.", ignoreRateLimit: true);
                    return;
                }

                _lblSprayState.Text = "State: AUTO LAND COMMANDED";
                _lblSprayState.ForeColor = SUCCESS_COLOR;
                AudioAlerts.Speak("Auto land commanded.", ignoreRateLimit: true);
            }
            catch (Exception ex)
            {
                ShowSprayError($"Auto land error: {ex.Message}");
                AudioAlerts.Speak("Auto land error.", ignoreRateLimit: true);
            }
            finally
            {
                _btnAutoLand.Enabled = true;
            }
        }

        private static async Task<string> ExtractError(HttpResponseMessage resp)
        {
            try
            {
                var body = await resp.Content.ReadAsStringAsync();
                if (string.IsNullOrWhiteSpace(body)) return $"HTTP {(int)resp.StatusCode}";
                try { return JObject.Parse(body)["detail"]?.ToString() ?? body; }
                catch { return body; }
            }
            catch
            {
                return $"HTTP {(int)resp.StatusCode}";
            }
        }

        private void AnnounceSprayState(string state, bool requireAutonomy, bool verified, int targetId)
        {
            switch (state)
            {
                case "approach":
                    AudioAlerts.Speak(requireAutonomy
                        ? "Autonomous spray started. Guided mode active. Hands off sticks unless overriding for safety."
                        : "Spray sequence started.",
                        ignoreRateLimit: true);
                    break;
                case "aim":
                    AudioAlerts.Speak("Aiming at target.", ignoreRateLimit: requireAutonomy);
                    break;
                case "spray":
                    AudioAlerts.Speak("Spraying target.", ignoreRateLimit: true);
                    break;
                case "verify":
                    AudioAlerts.Speak("Verifying extinguish.", ignoreRateLimit: requireAutonomy);
                    break;
                case "upload":
                    AudioAlerts.Speak("Uploading proof image.", ignoreRateLimit: requireAutonomy);
                    break;
                case "complete":
                    AudioAlerts.Speak(verified
                        ? $"Target {targetId} extinguished."
                        : $"Target {targetId} verification failed.",
                        ignoreRateLimit: true);
                    break;
                case "failed":
                    AudioAlerts.Speak("Spray sequence failed.", ignoreRateLimit: true);
                    break;
                case "aborted":
                    AudioAlerts.Speak("Spray sequence aborted.", ignoreRateLimit: true);
                    break;
            }
        }

        private async void ShowExtinguishedConfirmation(int targetId)
        {
            try
            {
                AudioAlerts.Speak($"Target {targetId} extinguished. Show judges the confirmation image.", ignoreRateLimit: true);

                var resp = await JetsonApiService.GetAsync("/api/task/2/spray/last_artifacts");
                if (!resp.IsSuccessStatusCode)
                {
                    MessageBox.Show(
                        $"TARGET {targetId} EXTINGUISHED\n\nProof image is not available yet.",
                        "Task 2 Autonomy",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information);
                    return;
                }

                var json = JObject.Parse(await resp.Content.ReadAsStringAsync());
                var afterPath = json["after_image_path"]?.ToString();
                if (string.IsNullOrEmpty(afterPath))
                {
                    MessageBox.Show(
                        $"TARGET {targetId} EXTINGUISHED\n\nNo post-spray image path returned.",
                        "Task 2 Autonomy",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information);
                    return;
                }

                var artifactUrl = $"{JetsonApiService.BaseUrl}/api/task/2/spray/artifact?path={Uri.EscapeDataString(afterPath)}";
                var bytes = await JetsonApiService.LongRunClient.GetByteArrayAsync(artifactUrl);
                using (var ms = new MemoryStream(bytes))
                using (var img = Image.FromStream(ms))
                {
                    ShowImageConfirmationForm(targetId, new Bitmap(img));
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(
                    $"TARGET {targetId} EXTINGUISHED\n\nConfirmation image popup failed: {ex.Message}",
                    "Task 2 Autonomy",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Information);
            }
        }

        private void ShowImageConfirmationForm(int targetId, Image image)
        {
            var form = new Form
            {
                Text = $"TARGET {targetId} EXTINGUISHED",
                BackColor = Color.Black,
                Width = 980,
                Height = 760,
                StartPosition = FormStartPosition.CenterScreen,
                TopMost = true,
            };

            var header = new Label
            {
                Text = $"TARGET {targetId} - EXTINGUISHED",
                Dock = DockStyle.Top,
                Height = 64,
                ForeColor = Color.White,
                BackColor = SUCCESS_COLOR,
                Font = new Font("Segoe UI", 24, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleCenter,
            };

            var pic = new PictureBox
            {
                Dock = DockStyle.Fill,
                Image = image,
                SizeMode = PictureBoxSizeMode.Zoom,
                BackColor = Color.Black,
            };
            pic.Disposed += (s, e) => image.Dispose();

            form.Controls.Add(pic);
            form.Controls.Add(header);
            form.Show(this);
            form.BringToFront();
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
