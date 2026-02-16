// ============================================================
// NOMAD ZED Calibration View
// ============================================================
// Interactive ZED 2i sensor calibration wizard for magnetometer
// and IMU calibration. Provides step-by-step guidance with
// real-time visualization of calibration progress.
// ============================================================

using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// ZED Camera Calibration view with step-by-step wizard,
    /// real-time progress visualization, and IMU health checks.
    /// </summary>
    public class ZedCalibrationView : NOMADViewBase, IUpdatableView
    {
        private readonly NOMADConfig _config;

        // Tab control
        private TabControl _tabControl;

        // === Magnetometer Calibration Tab ===
        private Panel _magWizardPanel;
        private Label _lblMagStep;
        private Label _lblMagInstruction;
        private Label _lblMagSamples;
        private Label _lblMagCoverage;
        private Label _lblMagFitness;
        private Label _lblMagElapsed;
        private ProgressBar _magProgressBar;
        private CoverageVisualization _coverageViz;
        private Button _btnMagStart;
        private Button _btnMagStop;
        private Button _btnMagCancel;
        private Panel _magStatusPanel;
        private Label _lblMagStatus;
        private TextBox _txtMagResult;
        private int _magWizardStep = 0;

        // === IMU Check Tab ===
        private Button _btnImuCheck;
        private Label _lblImuStatus;
        private Panel _imuResultPanel;
        private Label _lblGravity;
        private Label _lblAccelBias;
        private Label _lblGyroBias;
        private Label _lblAccelNoise;
        private Label _lblGyroNoise;
        private Label _lblTemperature;
        private Label _lblImuHealthy;
        private TextBox _txtImuIssues;

        // === Saved Calibration Tab ===
        private TextBox _txtSavedCal;
        private Button _btnRefreshSaved;
        private Label _lblSavedStatus;

        // Polling timer for mag calibration status
        private Timer _pollTimer;
        private bool _isMagCalibrating = false;
        private bool _isPolling = false;

        public ZedCalibrationView(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
        }

        private void InitializeUI()
        {
            this.AutoScroll = false;

            _tabControl = new TabControl
            {
                Dock = DockStyle.Fill,
                Font = new Font("Segoe UI", 10),
            };
            // Apply dark theme to tabs
            _tabControl.DrawMode = TabDrawMode.OwnerDrawFixed;
            _tabControl.DrawItem += TabControl_DrawItem;

            // --- Tab 1: Magnetometer Calibration ---
            var magTab = new TabPage("Magnetometer Calibration")
            {
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(15),
            };
            BuildMagnetometerTab(magTab);
            _tabControl.TabPages.Add(magTab);

            // --- Tab 2: IMU Health Check ---
            var imuTab = new TabPage("IMU Health Check")
            {
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(15),
            };
            BuildImuTab(imuTab);
            _tabControl.TabPages.Add(imuTab);

            // --- Tab 3: Saved Calibration ---
            var savedTab = new TabPage("Saved Calibration")
            {
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(15),
            };
            BuildSavedCalibrationTab(savedTab);
            _tabControl.TabPages.Add(savedTab);

            this.Controls.Add(_tabControl);

            // Polling timer for mag calibration progress
            _pollTimer = new Timer { Interval = 500 };
            _pollTimer.Tick += PollTimer_Tick;
        }

        // ============================================================
        // Tab Drawing (Dark Theme)
        // ============================================================

        private void TabControl_DrawItem(object sender, DrawItemEventArgs e)
        {
            var tab = _tabControl.TabPages[e.Index];
            var bounds = _tabControl.GetTabRect(e.Index);
            bool selected = (_tabControl.SelectedIndex == e.Index);

            using (var bgBrush = new SolidBrush(selected ? NOMADTheme.CARD_BG : NOMADTheme.BG_DARK))
            {
                e.Graphics.FillRectangle(bgBrush, bounds);
            }

            var textColor = selected ? NOMADTheme.ACCENT : NOMADTheme.TEXT_SECONDARY;
            TextRenderer.DrawText(e.Graphics, tab.Text, _tabControl.Font, bounds, textColor,
                TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter);
        }

        // ============================================================
        // Magnetometer Calibration Tab
        // ============================================================

        private void BuildMagnetometerTab(TabPage tab)
        {
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
            };
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 55));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 45));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            // ---- Left side: Wizard steps + controls ----
            var leftPanel = new Panel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
                Padding = new Padding(10),
            };

            // Status banner
            _magStatusPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 50,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(15, 10, 15, 10),
                Margin = new Padding(0, 0, 0, 10),
            };
            _lblMagStatus = new Label
            {
                Text = "Ready - Press Start to begin magnetometer calibration",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft,
            };
            _magStatusPanel.Controls.Add(_lblMagStatus);
            leftPanel.Controls.Add(_magStatusPanel);

            // Wizard step panel
            _magWizardPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 220,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(15),
                Margin = new Padding(0, 5, 0, 10),
            };

            var wizardTitle = new Label
            {
                Text = "CALIBRATION STEPS",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                Location = new Point(15, 10),
                AutoSize = true,
            };
            _magWizardPanel.Controls.Add(wizardTitle);

            _lblMagStep = new Label
            {
                Text = "Step 1 of 3",
                Font = new Font("Segoe UI", 9),
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Location = new Point(15, 35),
                AutoSize = true,
            };
            _magWizardPanel.Controls.Add(_lblMagStep);

            _lblMagInstruction = new Label
            {
                Text = "1. Ensure the ZED camera is connected and accessible.\n" +
                       "2. Place the drone in an open area away from metal objects.\n" +
                       "3. Press START to begin collecting magnetometer data.",
                Font = new Font("Segoe UI", 10),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Location = new Point(15, 60),
                Size = new Size(380, 100),
            };
            _magWizardPanel.Controls.Add(_lblMagInstruction);

            // Step indicators
            var stepFlow = new FlowLayoutPanel
            {
                Location = new Point(15, 170),
                Size = new Size(380, 35),
                FlowDirection = FlowDirection.LeftToRight,
            };
            for (int i = 1; i <= 3; i++)
            {
                var stepLabel = new Label
                {
                    Name = $"stepIndicator_{i}",
                    Text = i.ToString(),
                    Size = new Size(30, 30),
                    Font = new Font("Segoe UI", 10, FontStyle.Bold),
                    ForeColor = i == 1 ? Color.White : NOMADTheme.TEXT_MUTED,
                    BackColor = i == 1 ? NOMADTheme.ACCENT : NOMADTheme.CARD_BG,
                    TextAlign = ContentAlignment.MiddleCenter,
                    Margin = new Padding(2),
                };
                stepFlow.Controls.Add(stepLabel);

                if (i < 3)
                {
                    var dash = new Label
                    {
                        Text = "---",
                        Size = new Size(30, 30),
                        Font = new Font("Segoe UI", 10),
                        ForeColor = NOMADTheme.TEXT_MUTED,
                        TextAlign = ContentAlignment.MiddleCenter,
                        Margin = new Padding(0),
                    };
                    stepFlow.Controls.Add(dash);
                }
            }
            _magWizardPanel.Controls.Add(stepFlow);
            leftPanel.Controls.Add(_magWizardPanel);

            // Progress metrics panel
            var metricsPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 140,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(15),
                Margin = new Padding(0, 5, 0, 10),
            };

            var metricsTitle = new Label
            {
                Text = "PROGRESS",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                Location = new Point(15, 10),
                AutoSize = true,
            };
            metricsPanel.Controls.Add(metricsTitle);

            _magProgressBar = new ProgressBar
            {
                Location = new Point(15, 35),
                Size = new Size(350, 20),
                Style = ProgressBarStyle.Continuous,
                Maximum = 100,
            };
            metricsPanel.Controls.Add(_magProgressBar);

            _lblMagSamples = new Label
            {
                Text = "Samples: 0 / 500",
                Font = new Font("Consolas", 10),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Location = new Point(15, 65),
                AutoSize = true,
            };
            metricsPanel.Controls.Add(_lblMagSamples);

            _lblMagCoverage = new Label
            {
                Text = "Coverage: 0 / 8 octants (0%)",
                Font = new Font("Consolas", 10),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Location = new Point(15, 88),
                AutoSize = true,
            };
            metricsPanel.Controls.Add(_lblMagCoverage);

            _lblMagElapsed = new Label
            {
                Text = "Elapsed: 0s",
                Font = new Font("Consolas", 10),
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Location = new Point(15, 111),
                AutoSize = true,
            };
            metricsPanel.Controls.Add(_lblMagElapsed);

            leftPanel.Controls.Add(metricsPanel);

            // Buttons panel
            var btnPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                Height = 55,
                FlowDirection = FlowDirection.LeftToRight,
                Padding = new Padding(0, 5, 0, 5),
            };

            _btnMagStart = new Button
            {
                Text = "START CALIBRATION",
                Size = new Size(180, 40),
                FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.BTN_START,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand,
                Margin = new Padding(0, 0, 10, 0),
            };
            _btnMagStart.FlatAppearance.BorderSize = 0;
            _btnMagStart.Click += BtnMagStart_Click;
            btnPanel.Controls.Add(_btnMagStart);

            _btnMagStop = new Button
            {
                Text = "STOP & COMPUTE",
                Size = new Size(160, 40),
                FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.ACCENT,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand,
                Enabled = false,
                Margin = new Padding(0, 0, 10, 0),
            };
            _btnMagStop.FlatAppearance.BorderSize = 0;
            _btnMagStop.Click += BtnMagStop_Click;
            btnPanel.Controls.Add(_btnMagStop);

            _btnMagCancel = new Button
            {
                Text = "CANCEL",
                Size = new Size(100, 40),
                FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.BTN_STOP,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand,
                Enabled = false,
            };
            _btnMagCancel.FlatAppearance.BorderSize = 0;
            _btnMagCancel.Click += BtnMagCancel_Click;
            btnPanel.Controls.Add(_btnMagCancel);

            leftPanel.Controls.Add(btnPanel);

            // Result text area
            _txtMagResult = new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Vertical,
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = NOMADTheme.SUCCESS,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.FixedSingle,
                Text = "Calibration results will appear here after computation.",
            };
            leftPanel.Controls.Add(_txtMagResult);

            mainLayout.Controls.Add(leftPanel, 0, 0);

            // ---- Right side: 3D Coverage Visualization ----
            var rightPanel = new Panel
            {
                Dock = DockStyle.Fill,
                Padding = new Padding(10),
            };

            var vizTitle = new Label
            {
                Text = "SPATIAL COVERAGE",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                Dock = DockStyle.Top,
                Height = 25,
                TextAlign = ContentAlignment.MiddleLeft,
                Padding = new Padding(5, 0, 0, 0),
            };
            rightPanel.Controls.Add(vizTitle);

            _coverageViz = new CoverageVisualization
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(25, 25, 28),
            };
            rightPanel.Controls.Add(_coverageViz);

            // Fitness label below viz
            _lblMagFitness = new Label
            {
                Text = "Fitness: --",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                Dock = DockStyle.Bottom,
                Height = 30,
                TextAlign = ContentAlignment.MiddleCenter,
            };
            rightPanel.Controls.Add(_lblMagFitness);

            mainLayout.Controls.Add(rightPanel, 1, 0);

            tab.Controls.Add(mainLayout);
        }

        // ============================================================
        // IMU Health Check Tab
        // ============================================================

        private void BuildImuTab(TabPage tab)
        {
            var layout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
            };
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 120));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 120));

            // Instructions card
            var instrCard = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(15),
            };
            var instrTitle = new Label
            {
                Text = "IMU HEALTH CHECK",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                Location = new Point(15, 10),
                AutoSize = true,
            };
            instrCard.Controls.Add(instrTitle);

            var instrText = new Label
            {
                Text = "This check verifies accelerometer and gyroscope health.\n" +
                       "IMPORTANT: Keep the camera completely stationary and level during the 5-second check.",
                Font = new Font("Segoe UI", 10),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Location = new Point(15, 35),
                Size = new Size(600, 50),
            };
            instrCard.Controls.Add(instrText);

            _btnImuCheck = new Button
            {
                Text = "RUN IMU CHECK",
                Size = new Size(180, 35),
                Location = new Point(15, 80),
                FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.ACCENT,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Cursor = Cursors.Hand,
            };
            _btnImuCheck.FlatAppearance.BorderSize = 0;
            _btnImuCheck.Click += BtnImuCheck_Click;
            instrCard.Controls.Add(_btnImuCheck);

            _lblImuStatus = new Label
            {
                Text = "",
                Font = new Font("Segoe UI", 10),
                ForeColor = NOMADTheme.WARNING,
                Location = new Point(210, 85),
                AutoSize = true,
            };
            instrCard.Controls.Add(_lblImuStatus);

            layout.Controls.Add(instrCard, 0, 0);

            // Results card
            _imuResultPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(15),
                Visible = false,
            };

            var resTitle = new Label
            {
                Text = "RESULTS",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                Location = new Point(15, 10),
                AutoSize = true,
            };
            _imuResultPanel.Controls.Add(resTitle);

            _lblImuHealthy = new Label
            {
                Text = "Status: --",
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                Location = new Point(15, 35),
                AutoSize = true,
            };
            _imuResultPanel.Controls.Add(_lblImuHealthy);

            int yOff = 65;
            _lblGravity = CreateMetricLabel("Gravity:", ref yOff);
            _imuResultPanel.Controls.Add(_lblGravity);
            _lblAccelBias = CreateMetricLabel("Accel Bias:", ref yOff);
            _imuResultPanel.Controls.Add(_lblAccelBias);
            _lblGyroBias = CreateMetricLabel("Gyro Bias:", ref yOff);
            _imuResultPanel.Controls.Add(_lblGyroBias);
            _lblAccelNoise = CreateMetricLabel("Accel Noise:", ref yOff);
            _imuResultPanel.Controls.Add(_lblAccelNoise);
            _lblGyroNoise = CreateMetricLabel("Gyro Noise:", ref yOff);
            _imuResultPanel.Controls.Add(_lblGyroNoise);
            _lblTemperature = CreateMetricLabel("Temperature:", ref yOff);
            _imuResultPanel.Controls.Add(_lblTemperature);

            layout.Controls.Add(_imuResultPanel, 0, 1);

            // Issues card
            var issuesCard = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = NOMADTheme.CARD_BG,
                Padding = new Padding(15),
            };
            var issuesTitle = new Label
            {
                Text = "DIAGNOSTICS",
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                Dock = DockStyle.Top,
                Height = 25,
            };
            issuesCard.Controls.Add(issuesTitle);

            _txtImuIssues = new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Vertical,
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.FixedSingle,
                Text = "Run the IMU check to see diagnostics.",
            };
            issuesCard.Controls.Add(_txtImuIssues);

            layout.Controls.Add(issuesCard, 0, 2);

            tab.Controls.Add(layout);
        }

        private Label CreateMetricLabel(string prefix, ref int yOffset)
        {
            var lbl = new Label
            {
                Text = $"{prefix} --",
                Font = new Font("Consolas", 10),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Location = new Point(15, yOffset),
                AutoSize = true,
            };
            yOffset += 23;
            return lbl;
        }

        // ============================================================
        // Saved Calibration Tab
        // ============================================================

        private void BuildSavedCalibrationTab(TabPage tab)
        {
            var layout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
            };
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 50));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 35));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

            _lblSavedStatus = new Label
            {
                Text = "View the currently saved magnetometer calibration.",
                Font = new Font("Segoe UI", 10),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft,
                Padding = new Padding(10, 0, 0, 0),
            };
            layout.Controls.Add(_lblSavedStatus, 0, 0);

            _btnRefreshSaved = new Button
            {
                Text = "REFRESH",
                Size = new Size(120, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.ACCENT,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                Cursor = Cursors.Hand,
                Margin = new Padding(10, 0, 0, 5),
            };
            _btnRefreshSaved.FlatAppearance.BorderSize = 0;
            _btnRefreshSaved.Click += async (s, e) => await LoadSavedCalibration();
            layout.Controls.Add(_btnRefreshSaved, 0, 1);

            _txtSavedCal = new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Vertical,
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = NOMADTheme.TEXT_PRIMARY,
                Font = new Font("Consolas", 10),
                BorderStyle = BorderStyle.FixedSingle,
                Text = "Press REFRESH to load saved calibration data.",
            };
            layout.Controls.Add(_txtSavedCal, 0, 2);

            tab.Controls.Add(layout);
        }

        // ============================================================
        // Magnetometer Calibration Logic
        // ============================================================

        private async void BtnMagStart_Click(object sender, EventArgs e)
        {
            _btnMagStart.Enabled = false;
            _btnMagStop.Enabled = false;
            _btnMagCancel.Enabled = false;
            _lblMagStatus.Text = "Starting magnetometer calibration...";
            _lblMagStatus.ForeColor = NOMADTheme.WARNING;
            _txtMagResult.Text = "";

            try
            {
                var resp = await JetsonApiService.PostAsync("/api/calibration/magnetometer/start");
                var body = await resp.Content.ReadAsStringAsync();
                var data = JObject.Parse(body);

                if (resp.IsSuccessStatusCode)
                {
                    _isMagCalibrating = true;
                    _pollTimer.Start();

                    // Transition to step 2
                    SetMagWizardStep(2);

                    _lblMagStatus.Text = "COLLECTING - Rotate the camera slowly in all directions";
                    _lblMagStatus.ForeColor = NOMADTheme.SUCCESS;
                    _magStatusPanel.BackColor = Color.FromArgb(30, 70, 30);

                    _btnMagStart.Enabled = false;
                    _btnMagStop.Enabled = true;
                    _btnMagCancel.Enabled = true;
                }
                else
                {
                    var detail = data["detail"]?.ToString() ?? "Unknown error";
                    _lblMagStatus.Text = $"Failed: {detail}";
                    _lblMagStatus.ForeColor = NOMADTheme.ERROR;
                    _btnMagStart.Enabled = true;
                }
            }
            catch (Exception ex)
            {
                _lblMagStatus.Text = $"Connection error: {ex.Message}";
                _lblMagStatus.ForeColor = NOMADTheme.ERROR;
                _btnMagStart.Enabled = true;
            }
        }

        private async void BtnMagStop_Click(object sender, EventArgs e)
        {
            _btnMagStop.Enabled = false;
            _btnMagCancel.Enabled = false;
            _pollTimer.Stop();
            _lblMagStatus.Text = "Computing calibration (ellipsoid fitting)...";
            _lblMagStatus.ForeColor = NOMADTheme.WARNING;

            // Transition to step 3
            SetMagWizardStep(3);

            try
            {
                var resp = await JetsonApiService.PostLongRunAsync("/api/calibration/magnetometer/stop");
                var body = await resp.Content.ReadAsStringAsync();
                var data = JObject.Parse(body);

                _isMagCalibrating = false;

                bool success = (bool?)data["success"] ?? false;
                if (success)
                {
                    var result = data["result"];
                    _lblMagStatus.Text = "Calibration COMPLETE - Results saved";
                    _lblMagStatus.ForeColor = NOMADTheme.SUCCESS;
                    _magStatusPanel.BackColor = Color.FromArgb(30, 70, 30);

                    float fitness = (float?)result?["fitness"] ?? 0;
                    _lblMagFitness.Text = $"Fitness: {fitness:P1}";
                    _lblMagFitness.ForeColor = fitness > 0.9f ? NOMADTheme.SUCCESS
                        : fitness > 0.7f ? NOMADTheme.WARNING : NOMADTheme.ERROR;

                    var hardIron = result?["hard_iron"];
                    var softIron = result?["soft_iron"];

                    _txtMagResult.ForeColor = NOMADTheme.SUCCESS;
                    _txtMagResult.Text = $"=== Magnetometer Calibration Complete ===\r\n\r\n" +
                        $"Fitness:        {fitness:F4}\r\n" +
                        $"Samples Used:   {result?["samples_used"]}\r\n" +
                        $"Duration:       {result?["duration_s"]}s\r\n\r\n" +
                        $"Hard-Iron Offset (uT):\r\n" +
                        $"  X: {hardIron?[0]}\r\n" +
                        $"  Y: {hardIron?[1]}\r\n" +
                        $"  Z: {hardIron?[2]}\r\n\r\n" +
                        $"Soft-Iron Correction Matrix:\r\n" +
                        $"  [{FormatRow(softIron?[0])}]\r\n" +
                        $"  [{FormatRow(softIron?[1])}]\r\n" +
                        $"  [{FormatRow(softIron?[2])}]\r\n\r\n" +
                        $"Saved to: config/calibration/magnetometer_cal.json";
                }
                else
                {
                    var msg = data["message"]?.ToString() ?? "Unknown error";
                    _lblMagStatus.Text = $"Calibration failed: {msg}";
                    _lblMagStatus.ForeColor = NOMADTheme.ERROR;
                    _magStatusPanel.BackColor = Color.FromArgb(80, 30, 30);
                    _txtMagResult.ForeColor = NOMADTheme.ERROR;
                    _txtMagResult.Text = msg;
                }
            }
            catch (Exception ex)
            {
                _lblMagStatus.Text = $"Error: {ex.Message}";
                _lblMagStatus.ForeColor = NOMADTheme.ERROR;
                _txtMagResult.ForeColor = NOMADTheme.ERROR;
                _txtMagResult.Text = $"Compute error: {ex.Message}";
            }

            _btnMagStart.Enabled = true;
            _btnMagStop.Enabled = false;
            _btnMagCancel.Enabled = false;
        }

        private async void BtnMagCancel_Click(object sender, EventArgs e)
        {
            _pollTimer.Stop();
            _isMagCalibrating = false;
            _btnMagCancel.Enabled = false;
            _btnMagStop.Enabled = false;

            try
            {
                await JetsonApiService.PostAsync("/api/calibration/magnetometer/cancel");
            }
            catch { }

            _lblMagStatus.Text = "Calibration cancelled";
            _lblMagStatus.ForeColor = NOMADTheme.TEXT_SECONDARY;
            _magStatusPanel.BackColor = NOMADTheme.CARD_BG;

            SetMagWizardStep(1);
            ResetMagProgress();

            _btnMagStart.Enabled = true;
        }

        private async void PollTimer_Tick(object sender, EventArgs e)
        {
            if (!_isMagCalibrating || _isPolling) return;
            _isPolling = true;

            try
            {
                var resp = await JetsonApiService.GetAsync("/api/calibration/magnetometer/status");
                var body = await resp.Content.ReadAsStringAsync();
                var data = JObject.Parse(body);

                int samples = (int?)data["samples"] ?? 0;
                int target = (int?)data["target_samples"] ?? 500;
                float progress = (float?)data["progress"] ?? 0;
                float elapsed = (float?)data["elapsed_s"] ?? 0;
                string state = data["state"]?.ToString() ?? "unknown";

                _lblMagSamples.Text = $"Samples: {samples} / {target}";
                _magProgressBar.Value = Math.Min(100, (int)(progress * 100));
                _lblMagElapsed.Text = $"Elapsed: {elapsed:F0}s";

                var coverage = data["coverage"];
                if (coverage != null)
                {
                    int octants = (int?)coverage["octants_covered"] ?? 0;
                    float pct = (float?)coverage["coverage_pct"] ?? 0;
                    _lblMagCoverage.Text = $"Coverage: {octants} / 8 octants ({pct:F0}%)";

                    // Update visualization
                    _coverageViz.UpdateCoverage(octants, GetOctantFlags(coverage));

                    // Color code coverage
                    _lblMagCoverage.ForeColor = octants >= 6 ? NOMADTheme.SUCCESS
                        : octants >= 4 ? NOMADTheme.WARNING : NOMADTheme.ERROR;
                }

                if (state == "failed")
                {
                    _pollTimer.Stop();
                    _isMagCalibrating = false;
                    _lblMagStatus.Text = $"Collection failed: {data["error"]}";
                    _lblMagStatus.ForeColor = NOMADTheme.ERROR;
                    _btnMagStart.Enabled = true;
                    _btnMagStop.Enabled = false;
                    _btnMagCancel.Enabled = false;
                }
            }
            catch { /* Ignore transient poll failures */ }
            finally { _isPolling = false; }
        }

        private bool[] GetOctantFlags(JToken coverage)
        {
            // The API reports coverage as octants_covered count
            // We approximate which octants are covered based on count
            // (precise per-octant data could be added to the API later)
            int covered = (int?)coverage?["octants_covered"] ?? 0;
            var flags = new bool[8];
            for (int i = 0; i < Math.Min(covered, 8); i++)
                flags[i] = true;
            return flags;
        }

        private void SetMagWizardStep(int step)
        {
            _magWizardStep = step;
            _lblMagStep.Text = $"Step {step} of 3";

            // Update step indicators
            for (int i = 1; i <= 3; i++)
            {
                var ctrl = _magWizardPanel.Controls.Find($"stepIndicator_{i}", true);
                if (ctrl.Length > 0)
                {
                    ctrl[0].BackColor = i <= step ? NOMADTheme.ACCENT : NOMADTheme.CARD_BG;
                    ctrl[0].ForeColor = i <= step ? Color.White : NOMADTheme.TEXT_MUTED;
                }
            }

            switch (step)
            {
                case 1:
                    _lblMagInstruction.Text =
                        "1. Ensure the ZED camera is connected and accessible.\n" +
                        "2. Place the drone in an open area away from metal objects.\n" +
                        "3. Press START to begin collecting magnetometer data.";
                    break;
                case 2:
                    _lblMagInstruction.Text =
                        "ROTATING: Slowly rotate the drone in all orientations.\n\n" +
                        "- Roll, pitch, and yaw slowly through full range\n" +
                        "- Try to cover all 8 spatial octants (see visualization)\n" +
                        "- Continue until coverage is green (6+ octants)\n" +
                        "- Press STOP & COMPUTE when satisfied";
                    break;
                case 3:
                    _lblMagInstruction.Text =
                        "COMPUTING: Fitting ellipsoid to collected data...\n\n" +
                        "This computes hard-iron offset and soft-iron\n" +
                        "correction matrix using least-squares fitting.\n" +
                        "Results will be saved automatically.";
                    break;
            }
        }

        private void ResetMagProgress()
        {
            _magProgressBar.Value = 0;
            _lblMagSamples.Text = "Samples: 0 / 500";
            _lblMagCoverage.Text = "Coverage: 0 / 8 octants (0%)";
            _lblMagCoverage.ForeColor = NOMADTheme.TEXT_PRIMARY;
            _lblMagElapsed.Text = "Elapsed: 0s";
            _lblMagFitness.Text = "Fitness: --";
            _lblMagFitness.ForeColor = NOMADTheme.TEXT_SECONDARY;
            _coverageViz.Reset();
        }

        private string FormatRow(JToken row)
        {
            if (row == null) return "N/A";
            try
            {
                return string.Join(", ", row.Select(v => $"{(float)v,10:F6}"));
            }
            catch { return row.ToString(); }
        }

        // ============================================================
        // IMU Check Logic
        // ============================================================

        private async void BtnImuCheck_Click(object sender, EventArgs e)
        {
            _btnImuCheck.Enabled = false;
            _btnImuCheck.Text = "Running...";
            _lblImuStatus.Text = "Keep the camera STILL for 5 seconds...";
            _lblImuStatus.ForeColor = NOMADTheme.WARNING;
            _imuResultPanel.Visible = false;
            _txtImuIssues.Text = "";

            try
            {
                // IMU check takes ~5 seconds on the Jetson; use the long-run client to avoid timeout
                var resp = await JetsonApiService.LongRunClient.GetAsync(
                    $"{JetsonApiService.BaseUrl}/api/calibration/imu/check");
                var body = await resp.Content.ReadAsStringAsync();
                var data = JObject.Parse(body);

                bool healthy = (bool?)data["healthy"] ?? false;

                _lblImuHealthy.Text = healthy ? "HEALTHY" : "ISSUES DETECTED";
                _lblImuHealthy.ForeColor = healthy ? NOMADTheme.SUCCESS : NOMADTheme.ERROR;

                float gravity = (float?)data["gravity_magnitude"] ?? 0;
                float gravError = (float?)data["gravity_error_pct"] ?? 0;
                _lblGravity.Text = $"Gravity:     {gravity:F4} m/s^2 (error: {gravError:F1}%)";
                _lblGravity.ForeColor = gravError < 5 ? NOMADTheme.SUCCESS : NOMADTheme.ERROR;

                var accelBias = data["accel_bias"];
                _lblAccelBias.Text = $"Accel Bias:  ({accelBias?[0]:F4}, {accelBias?[1]:F4}, {accelBias?[2]:F4}) m/s^2";

                var gyroBias = data["gyro_bias"];
                float gyroBiasMag = 0;
                if (gyroBias != null)
                {
                    float gx = (float?)gyroBias[0] ?? 0;
                    float gy = (float?)gyroBias[1] ?? 0;
                    float gz = (float?)gyroBias[2] ?? 0;
                    gyroBiasMag = (float)Math.Sqrt(gx * gx + gy * gy + gz * gz);
                }
                _lblGyroBias.Text = $"Gyro Bias:   ({gyroBias?[0]:F4}, {gyroBias?[1]:F4}, {gyroBias?[2]:F4}) rad/s";
                _lblGyroBias.ForeColor = gyroBiasMag < 0.05f ? NOMADTheme.TEXT_PRIMARY : NOMADTheme.ERROR;

                float accelNoise = (float?)data["accel_noise"] ?? 0;
                _lblAccelNoise.Text = $"Accel Noise: {accelNoise:F4} m/s^2";
                _lblAccelNoise.ForeColor = accelNoise < 0.5f ? NOMADTheme.TEXT_PRIMARY : NOMADTheme.WARNING;

                float gyroNoise = (float?)data["gyro_noise"] ?? 0;
                _lblGyroNoise.Text = $"Gyro Noise:  {gyroNoise:F4} rad/s";
                _lblGyroNoise.ForeColor = gyroNoise < 0.02f ? NOMADTheme.TEXT_PRIMARY : NOMADTheme.WARNING;

                float temp = (float?)data["temperature"] ?? 0;
                _lblTemperature.Text = $"Temperature: {temp:F1} C";

                // Show issues
                var issues = data["issues"] as JArray;
                if (issues != null && issues.Count > 0)
                {
                    _txtImuIssues.Text = string.Join("\r\n", issues.Select(i => $"  - {i}"));
                    _txtImuIssues.ForeColor = healthy ? NOMADTheme.SUCCESS : NOMADTheme.WARNING;
                }

                _imuResultPanel.Visible = true;
                _lblImuStatus.Text = healthy ? "Check passed" : "Check completed with issues";
                _lblImuStatus.ForeColor = healthy ? NOMADTheme.SUCCESS : NOMADTheme.WARNING;
            }
            catch (Exception ex)
            {
                _lblImuStatus.Text = $"Error: {ex.Message}";
                _lblImuStatus.ForeColor = NOMADTheme.ERROR;
                _txtImuIssues.Text = $"Connection error: {ex.Message}";
                _txtImuIssues.ForeColor = NOMADTheme.ERROR;
            }

            _btnImuCheck.Enabled = true;
            _btnImuCheck.Text = "RUN IMU CHECK";
        }

        // ============================================================
        // Saved Calibration Logic
        // ============================================================

        private async Task LoadSavedCalibration()
        {
            _btnRefreshSaved.Enabled = false;
            _txtSavedCal.Text = "Loading...";

            try
            {
                var resp = await JetsonApiService.GetAsync("/api/calibration/magnetometer/saved");
                var body = await resp.Content.ReadAsStringAsync();
                var data = JObject.Parse(body);

                bool available = (bool?)data["available"] ?? false;
                if (available)
                {
                    float fitness = (float?)data["fitness"] ?? 0;
                    var hardIron = data["hard_iron"];
                    var softIron = data["soft_iron"];

                    _lblSavedStatus.Text = $"Calibration available (fitness: {fitness:P1})";
                    _lblSavedStatus.ForeColor = fitness > 0.9f ? NOMADTheme.SUCCESS
                        : fitness > 0.7f ? NOMADTheme.WARNING : NOMADTheme.ERROR;

                    _txtSavedCal.ForeColor = NOMADTheme.TEXT_PRIMARY;
                    _txtSavedCal.Text =
                        $"=== Saved Magnetometer Calibration ===\r\n\r\n" +
                        $"Timestamp:      {data["timestamp"]}\r\n" +
                        $"Fitness:        {fitness:F4}\r\n" +
                        $"Samples Used:   {data["samples_used"]}\r\n" +
                        $"Duration:       {data["duration_s"]}s\r\n\r\n" +
                        $"Hard-Iron Offset (uT):\r\n" +
                        $"  X: {hardIron?[0]}\r\n" +
                        $"  Y: {hardIron?[1]}\r\n" +
                        $"  Z: {hardIron?[2]}\r\n\r\n" +
                        $"Soft-Iron Correction Matrix:\r\n" +
                        $"  [{FormatRow(softIron?[0])}]\r\n" +
                        $"  [{FormatRow(softIron?[1])}]\r\n" +
                        $"  [{FormatRow(softIron?[2])}]\r\n";
                }
                else
                {
                    _lblSavedStatus.Text = "No calibration saved";
                    _lblSavedStatus.ForeColor = NOMADTheme.WARNING;
                    _txtSavedCal.ForeColor = NOMADTheme.TEXT_SECONDARY;
                    _txtSavedCal.Text = data["message"]?.ToString() ?? "No magnetometer calibration found on the Jetson.";
                }
            }
            catch (Exception ex)
            {
                _lblSavedStatus.Text = "Error loading calibration";
                _lblSavedStatus.ForeColor = NOMADTheme.ERROR;
                _txtSavedCal.ForeColor = NOMADTheme.ERROR;
                _txtSavedCal.Text = $"Error: {ex.Message}";
            }

            _btnRefreshSaved.Enabled = true;
        }

        // ============================================================
        // IUpdatableView
        // ============================================================

        public void UpdateData()
        {
            // No periodic update needed -- mag calibration polls via its own timer
        }

        // ============================================================
        // Cleanup
        // ============================================================

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _pollTimer?.Stop();
                _pollTimer?.Dispose();
            }
            base.Dispose(disposing);
        }
    }

    // ============================================================
    // Coverage Visualization Control
    // ============================================================

    /// <summary>
    /// Custom painting control that shows octant coverage as an
    /// isometric cube diagram. Each octant is color-coded based on
    /// whether it has been covered during magnetometer calibration.
    /// </summary>
    internal class CoverageVisualization : Control
    {
        private int _octantsCovered = 0;
        private bool[] _octantFlags = new bool[8];

        public CoverageVisualization()
        {
            this.DoubleBuffered = true;
            this.SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer, true);
        }

        public void UpdateCoverage(int octantsCovered, bool[] flags)
        {
            _octantsCovered = octantsCovered;
            if (flags != null && flags.Length == 8)
                _octantFlags = flags;
            Invalidate();
        }

        public void Reset()
        {
            _octantsCovered = 0;
            _octantFlags = new bool[8];
            Invalidate();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;

            int w = this.Width;
            int h = this.Height;
            int cx = w / 2;
            int cy = h / 2;
            int size = Math.Min(w, h) / 3;

            // Draw the exploded cube diagram: 8 small cubes in octant positions
            // Isometric offsets
            float dx = size * 0.8f;
            float dy = size * 0.5f;

            // Octant positions (isometric projection)
            // Octant index: (sign_x >= 0, sign_y >= 0, sign_z >= 0)
            var octantPositions = new PointF[]
            {
                new PointF(cx - dx * 0.1f, cy - dy * 1.1f),  // (0,0,0) -> bottom-left-back
                new PointF(cx + dx * 0.9f, cy - dy * 0.6f),  // (1,0,0) -> bottom-right-back
                new PointF(cx - dx * 0.9f, cy - dy * 0.6f),  // (0,1,0) -> bottom-left-front
                new PointF(cx + dx * 0.1f, cy - dy * 0.1f),  // (1,1,0) -> bottom-right-front
                new PointF(cx - dx * 0.1f, cy - dy * 0.1f - size), // (0,0,1) -> top-left-back
                new PointF(cx + dx * 0.9f, cy + dy * 0.4f - size), // (1,0,1) -> top-right-back
                new PointF(cx - dx * 0.9f, cy + dy * 0.4f - size), // (0,1,1) -> top-left-front
                new PointF(cx + dx * 0.1f, cy + dy * 0.9f - size), // (1,1,1) -> top-right-front
            };

            int cubeSize = size / 2;

            for (int i = 0; i < 8; i++)
            {
                var pos = octantPositions[i];
                bool covered = _octantFlags[i];

                Color fillColor = covered
                    ? Color.FromArgb(180, NOMADTheme.SUCCESS)
                    : Color.FromArgb(60, NOMADTheme.TEXT_MUTED);

                Color borderColor = covered
                    ? NOMADTheme.SUCCESS
                    : Color.FromArgb(100, NOMADTheme.TEXT_MUTED);

                // Draw a small isometric cube face (just a diamond for simplicity)
                var diamond = new PointF[]
                {
                    new PointF(pos.X, pos.Y - cubeSize / 2f),
                    new PointF(pos.X + cubeSize / 2f, pos.Y),
                    new PointF(pos.X, pos.Y + cubeSize / 2f),
                    new PointF(pos.X - cubeSize / 2f, pos.Y),
                };

                using (var brush = new SolidBrush(fillColor))
                    g.FillPolygon(brush, diamond);
                using (var pen = new Pen(borderColor, 1.5f))
                    g.DrawPolygon(pen, diamond);

                // Label
                string label = covered ? "OK" : $"{i + 1}";
                using (var font = new Font("Segoe UI", 7, FontStyle.Bold))
                using (var textBrush = new SolidBrush(covered ? Color.White : NOMADTheme.TEXT_MUTED))
                {
                    var textSize = g.MeasureString(label, font);
                    g.DrawString(label, font, textBrush,
                        pos.X - textSize.Width / 2, pos.Y - textSize.Height / 2);
                }
            }

            // Title at bottom
            string summary = $"{_octantsCovered}/8 Octants";
            using (var font = new Font("Segoe UI", 11, FontStyle.Bold))
            using (var brush = new SolidBrush(
                _octantsCovered >= 6 ? NOMADTheme.SUCCESS :
                _octantsCovered >= 4 ? NOMADTheme.WARNING : NOMADTheme.TEXT_SECONDARY))
            {
                var textSize = g.MeasureString(summary, font);
                g.DrawString(summary, font, brush,
                    cx - textSize.Width / 2, h - textSize.Height - 10);
            }

            // Axis labels
            using (var font = new Font("Segoe UI", 8))
            using (var brush = new SolidBrush(NOMADTheme.TEXT_MUTED))
            {
                g.DrawString("+X", font, brush, cx + dx + 5, cy - 10);
                g.DrawString("-X", font, brush, cx - dx - 25, cy - 10);
                g.DrawString("+Z", font, brush, cx - 10, cy - size - dy - 10);
            }
        }
    }
}
