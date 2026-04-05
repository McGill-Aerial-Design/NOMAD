// ============================================================
// SLAM3DView.cs - 3D SLAM Visualization for Mission Planner
// ============================================================
// Real-time 3D mesh visualization from nvblox SLAM.
// Uses OpenTK (OpenGL) for cross-platform rendering (Windows + Linux).
// ============================================================

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using NOMAD.MissionPlanner.SLAM3D.Camera;
using NOMAD.MissionPlanner.SLAM3D.Data;
using NOMAD.MissionPlanner.SLAM3D.Models;
using NOMAD.MissionPlanner.SLAM3D.Network;
using NOMAD.MissionPlanner.SLAM3D.Rendering;

namespace NOMAD.MissionPlanner
{
    // ==================== SLAM 3D View ====================

    /// <summary>
    /// 3D SLAM visualization using OpenTK (cross-platform OpenGL).
    /// Displays real-time nvblox mesh, drone model with servo camera, and detection markers.
    /// </summary>
    public class SLAM3DView : UserControl
    {
        // ---- Configuration ----
        private readonly NOMADConfig _config;
        private readonly DualLinkSender _sender;

        // ---- GL Control ----
        private GLControl _glControl;
        private bool _glInitialized;
        private System.Windows.Forms.Timer _renderTimer;

        // ---- SLAM components ----
        private readonly WebSocketClient _webSocketClient = new WebSocketClient();
        private readonly VoxelMeshBuilder _voxelMeshBuilder = new VoxelMeshBuilder();
        private readonly CameraController _cameraController = new CameraController();
        private readonly GridRenderer _gridRenderer = new GridRenderer();
        private readonly TrajectoryRenderer _trajectoryRenderer = new TrajectoryRenderer();
        private readonly DetectionRenderer _detectionRenderer = new DetectionRenderer();
        private readonly DroneRenderer _droneRenderer = new DroneRenderer();

        private volatile bool _autoUpdateEnabled = true;
        private readonly object _poseLock = new object();

        // ---- Servo polling ----
        private float _servoAngleDeg = 90.0f;
        private System.Windows.Forms.Timer _servoTimer;

        // ---- Perception/status polling ----
        private System.Windows.Forms.Timer _statusTimer;
        private bool _statusPollInFlight;
        private const float ScanStopThresholdMps = 0.10f;

        // ---- PoseState (anti-jitter filtering) ----
        private readonly PoseState _poseState = new PoseState();

        // ---- Drone pose (raw from WS, REP-103 odom frame: X-forward, Y-left, Z-up) ----
        private float _dronePosX, _dronePosY, _dronePosZ;
        private float _droneRollRaw, _dronePitchRaw, _droneYawRaw;
        private float _droneVelX, _droneVelY, _droneVelZ;
        private float _renderPosX, _renderPosY, _renderPosZ;
        private float _renderRollRaw, _renderPitchRaw, _renderYawRaw;
        private bool _hasBodyAttitude; // True if body_roll/pitch/yaw received (magnetometer-corrected)
        private long _lastRawPoseUpdateStamp = -1;
        private const double PoseSnapAfterGapSec = 0.40;

        // ---- Trajectory ----
        private const int MaxTrajectoryPoints = 500;

        // ---- Camera interaction ----
        private Point _lastMousePos;
        private bool _mouseRotating, _mousePanning;

        // ---- UI Controls ----
        private Panel _controlPanel;
        private Panel _statusLogPanel;
        private Button _btnToggleCamera, _btnResetView, _btnClearMesh, _btnResetImuBiases;
        private Button _btnSaveMap, _btnLoadMap, _btnRelocalizeMap, _btnCenterOnPose;
        private Label _lblStatus, _lblStats;
        private Label _lblPerceptionStatus;
        private Label _lblMapPath;
        private TextBox _txtMapPath;
        private TextBox _txtStatusLog;
        private CheckBox _chkShowGrid, _chkShowTrajectory, _chkAutoUpdate;
        private ComboBox _combDroneType, _combMeshMode;
        private NumericUpDown _numLength, _numWidth, _numHeight, _numHeadingOffset, _numFov;
        private string _meshOutputMode = "voxel";
        private bool _meshModeApplyInFlight;
        private bool _meshModeRefreshInFlight;
        private bool _meshModeSelectionInternal;
        private int _meshUpdateCount;
        private int _totalBlocks;
        private const int MaxStatusLogLines = 120;

        // ==================== Constructor ====================

        public SLAM3DView(NOMADConfig config, DualLinkSender sender)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            _sender = sender ?? throw new ArgumentNullException(nameof(sender));
            InitializeSlamComponents();
            InitializeComponents();
            StartUpdateLoop();
            StartServoPolling();
            StartPerceptionStatusPolling();
            _ = RefreshMeshModeFromServerAsync(updateStatus: false);
        }

        private void InitializeSlamComponents()
        {
            _cameraController.ViewMode = CameraViewMode.ThirdPerson;
            _cameraController.Reset();

            _trajectoryRenderer.MaxPoints = MaxTrajectoryPoints;
            _trajectoryRenderer.ShowTrajectory = true;
            _trajectoryRenderer.UseAgeGradient = false;
            _trajectoryRenderer.LineWidth = 2f;
            _trajectoryRenderer.SetGradientColors(1f, 0.78f, 0f, 1f, 0.78f, 0f);

            _gridRenderer.ShowGrid = true;
            _gridRenderer.GridSize = 10;
            _gridRenderer.CellSize = 1f;

            _webSocketClient.ReconnectDelayMs = 1000;
            _webSocketClient.MaxReconnectDelayMs = 10000;
            _webSocketClient.MaxMessageSize = 10 * 1024 * 1024;
            _webSocketClient.ReceiveTimeoutSec = 30;
            _webSocketClient.OnStatusChanged += HandleWebSocketStatusChanged;
            _webSocketClient.OnError += HandleWebSocketError;
            _webSocketClient.OnFrameReceived += HandleSlamFrame;
        }

        // ==================== UI Initialization ====================

        private void InitializeComponents()
        {
            BackColor = Color.FromArgb(30, 30, 33);
            Dock = DockStyle.Fill;

            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 2,
                BackColor = Color.FromArgb(30, 30, 33),
            };
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 170));

            var bottomLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                BackColor = Color.FromArgb(30, 30, 33),
                Margin = new Padding(0),
                Padding = new Padding(0),
            };
            bottomLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 76));
            bottomLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 24));

            // OpenGL viewport
            try
            {
                _glControl = new GLControl(new GraphicsMode(32, 24, 0, 4))
                {
                    Dock = DockStyle.Fill,
                    VSync = false,
                };
                _glControl.Load += GlControl_Load;
                _glControl.Paint += GlControl_Paint;
                _glControl.Resize += GlControl_Resize;
                _glControl.MouseDown += GlControl_MouseDown;
                _glControl.MouseUp += GlControl_MouseUp;
                _glControl.MouseMove += GlControl_MouseMove;
                _glControl.MouseWheel += GlControl_MouseWheel;
                mainLayout.Controls.Add(_glControl, 0, 0);
            }
            catch (Exception ex)
            {
                var errorLabel = new Label
                {
                    Text = $"OpenGL init failed: {ex.Message}\nEnsure OpenTK.dll is available.",
                    Dock = DockStyle.Fill,
                    ForeColor = Color.Red,
                    Font = new Font("Consolas", 11),
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                mainLayout.Controls.Add(errorLabel, 0, 0);
            }

            // Control panel
            _controlPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(40, 40, 45),
                Padding = new Padding(10),
                AutoScroll = true,
            };

            int x = 10, y = 8;

            _btnToggleCamera = CreateButton("View: TPV", x, y, 100, 28, Color.FromArgb(0, 122, 204));
            _btnToggleCamera.Click += BtnToggleCamera_Click;
            _controlPanel.Controls.Add(_btnToggleCamera);
            x += 105;

            _btnResetView = CreateButton("Reset View", x, y, 85, 28, Color.FromArgb(60, 60, 65));
            _btnResetView.Click += BtnResetView_Click;
            _controlPanel.Controls.Add(_btnResetView);
            x += 90;

            _btnClearMesh = CreateButton("Clear Mesh", x, y, 85, 28, Color.FromArgb(180, 60, 60));
            _btnClearMesh.Click += BtnClearMesh_Click;
            _controlPanel.Controls.Add(_btnClearMesh);
            x += 95;

            _btnResetImuBiases = CreateButton("Reset IMU", x, y, 85, 28, Color.FromArgb(120, 80, 0));
            _btnResetImuBiases.Click += BtnResetImuBiases_Click;
            _controlPanel.Controls.Add(_btnResetImuBiases);
            x += 95;

            _chkShowGrid = CreateCheckBox("Grid", x, y + 4, true);
            _controlPanel.Controls.Add(_chkShowGrid);
            x += 55;

            _chkShowTrajectory = CreateCheckBox("Trail", x, y + 4, true);
            _controlPanel.Controls.Add(_chkShowTrajectory);
            x += 55;

            _chkAutoUpdate = CreateCheckBox("Auto", x, y + 4, true);
            _chkAutoUpdate.CheckedChanged += (s, e) => _autoUpdateEnabled = _chkAutoUpdate.Checked;
            _controlPanel.Controls.Add(_chkAutoUpdate);
            x += 55;

            _controlPanel.Controls.Add(CreateLabel("Mesh:", x, y + 6));
            x += 38;
            _combMeshMode = new ComboBox
            {
                Location = new Point(x, y + 2),
                Size = new Size(78, 22),
                DropDownStyle = ComboBoxStyle.DropDownList,
                ForeColor = Color.White,
                BackColor = Color.FromArgb(60, 60, 60),
                Font = new Font("Segoe UI", 8.5f),
            };
            _combMeshMode.Items.Add("Voxel");
            _combMeshMode.SelectedIndexChanged += CombMeshMode_SelectedIndexChanged;
            _combMeshMode.SelectedIndex = 0;
            _combMeshMode.Enabled = false;
            _controlPanel.Controls.Add(_combMeshMode);

            // Second row: drone config
            y += 34;
            x = 10;

            _controlPanel.Controls.Add(CreateLabel("Drone (cm)  L:", x, y + 3));
            x += 82;
            _numLength = CreateNumericUpDown(x, y, 45, 1, 200, (decimal)_config.DroneLengthCm);
            _numLength.ValueChanged += (s, e) => { _config.DroneLengthCm = (float)_numLength.Value; _config.Save(); };
            _controlPanel.Controls.Add(_numLength);
            x += 50;

            _controlPanel.Controls.Add(CreateLabel("W:", x, y + 3));
            x += 18;
            _numWidth = CreateNumericUpDown(x, y, 45, 1, 200, (decimal)_config.DroneWidthCm);
            _numWidth.ValueChanged += (s, e) => { _config.DroneWidthCm = (float)_numWidth.Value; _config.Save(); };
            _controlPanel.Controls.Add(_numWidth);
            x += 50;

            _controlPanel.Controls.Add(CreateLabel("H:", x, y + 3));
            x += 18;
            _numHeight = CreateNumericUpDown(x, y, 45, 1, 100, (decimal)_config.DroneHeightCm);
            _numHeight.ValueChanged += (s, e) => { _config.DroneHeightCm = (float)_numHeight.Value; _config.Save(); };
            _controlPanel.Controls.Add(_numHeight);
            x += 55;

            _controlPanel.Controls.Add(CreateLabel("Type:", x, y + 3));
            x += 35;
            _combDroneType = new ComboBox
            {
                Location = new Point(x, y),
                Size = new Size(85, 20),
                DropDownStyle = ComboBoxStyle.DropDownList,
                ForeColor = Color.White,
                BackColor = Color.FromArgb(60, 60, 60),
            };
            _combDroneType.Items.AddRange(new[] { "Tricopter", "Quadcopter" });
            _combDroneType.SelectedItem = _config.DroneFrameType;
            _combDroneType.SelectedIndexChanged += (s, e) =>
            {
                _config.DroneFrameType = _combDroneType.SelectedItem?.ToString() ?? "Tricopter";
                _config.Save();
                _glControl?.Invalidate();
            };
            _controlPanel.Controls.Add(_combDroneType);
            x += 90;

            _controlPanel.Controls.Add(CreateLabel("Hdg Offset:", x, y + 3));
            x += 68;
            _numHeadingOffset = CreateNumericUpDown(x, y, 55, -180, 180, (decimal)_config.SlamHeadingOffsetDeg);
            _numHeadingOffset.ValueChanged += (s, e) => { _config.SlamHeadingOffsetDeg = (float)_numHeadingOffset.Value; _config.Save(); };
            _controlPanel.Controls.Add(_numHeadingOffset);
            x += 63;

            _controlPanel.Controls.Add(CreateLabel("FOV:", x, y + 3));
            x += 30;
            _numFov = CreateNumericUpDown(x, y, 50, 30, 140, (decimal)GetClampedFovDegrees());
            _numFov.ValueChanged += (s, e) =>
            {
                _config.SlamCameraFovDeg = (float)_numFov.Value;
                _config.Save();
                _glControl?.Invalidate();
            };
            _controlPanel.Controls.Add(_numFov);

            // Third row: area map controls
            y += 26;
            x = 10;

            _lblMapPath = CreateLabel("Area Map:", x, y + 3);
            _controlPanel.Controls.Add(_lblMapPath);
            x += 65;

            _txtMapPath = new TextBox
            {
                Location = new Point(x, y),
                Size = new Size(300, 22),
                Text = "/home/mad/NOMAD/data/area_maps/slam_area_map.area",
                BackColor = Color.FromArgb(50, 50, 55),
                ForeColor = Color.White,
                BorderStyle = BorderStyle.FixedSingle,
            };
            _controlPanel.Controls.Add(_txtMapPath);
            x += 310;

            _btnSaveMap = CreateButton("Save", x, y, 58, 24, Color.FromArgb(0, 122, 204));
            _btnSaveMap.Click += async (s, e) => await SaveAreaMapAsync();
            _controlPanel.Controls.Add(_btnSaveMap);
            x += 62;

            _btnLoadMap = CreateButton("Load", x, y, 58, 24, Color.FromArgb(60, 60, 65));
            _btnLoadMap.Click += async (s, e) => await LoadAreaMapAsync();
            _controlPanel.Controls.Add(_btnLoadMap);
            x += 62;

            _btnRelocalizeMap = CreateButton("Relocalize", x, y, 80, 24, Color.FromArgb(150, 90, 0));
            _btnRelocalizeMap.Click += async (s, e) => await RelocalizeAreaMapAsync();
            _controlPanel.Controls.Add(_btnRelocalizeMap);
            x += 86;

            _btnCenterOnPose = CreateButton("Center", x, y, 65, 24, Color.FromArgb(60, 60, 65));
            _btnCenterOnPose.Click += (s, e) => CenterOrbitOnCurrentPose();
            _controlPanel.Controls.Add(_btnCenterOnPose);

            // Third row: status
            y += 30;
            _lblStatus = new Label
            {
                Text = "Status: Connecting...",
                Location = new Point(10, y),
                ForeColor = Color.FromArgb(200, 200, 200),
                AutoSize = true,
                Font = new Font("Consolas", 9),
            };
            _controlPanel.Controls.Add(_lblStatus);

            _lblStats = new Label
            {
                Text = "Mesh: 0 voxels",
                Location = new Point(300, y),
                ForeColor = Color.FromArgb(150, 150, 150),
                AutoSize = true,
                Font = new Font("Consolas", 9),
            };
            _controlPanel.Controls.Add(_lblStats);

            y += 18;
            _lblPerceptionStatus = new Label
            {
                Text = "HSV: -- | Servo: -- | ScanStopScan: --",
                Location = new Point(10, y),
                ForeColor = Color.FromArgb(150, 150, 150),
                AutoSize = true,
                Font = new Font("Consolas", 9),
            };
            _controlPanel.Controls.Add(_lblPerceptionStatus);

            _statusLogPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(34, 34, 38),
                Padding = new Padding(8),
            };

            var lblStatusLogTitle = new Label
            {
                Text = "Status Log",
                Dock = DockStyle.Top,
                Height = 20,
                ForeColor = Color.LightSteelBlue,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleLeft,
            };

            _txtStatusLog = new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ScrollBars = ScrollBars.Vertical,
                ReadOnly = true,
                BackColor = Color.FromArgb(24, 24, 26),
                ForeColor = Color.Gainsboro,
                BorderStyle = BorderStyle.FixedSingle,
                Font = new Font("Consolas", 8),
            };

            _statusLogPanel.Controls.Add(_txtStatusLog);
            _statusLogPanel.Controls.Add(lblStatusLogTitle);

            bottomLayout.Controls.Add(_controlPanel, 0, 0);
            bottomLayout.Controls.Add(_statusLogPanel, 1, 0);
            mainLayout.Controls.Add(bottomLayout, 0, 1);
            Controls.Add(mainLayout);

            UpdateCameraToggleLabel();

            AppendStatusLogSafe("SLAM status log initialized");

            // Render timer ~30fps
            _renderTimer = new System.Windows.Forms.Timer { Interval = 33 };
            _renderTimer.Tick += (s, e) => { if (_glControl != null && _glInitialized) _glControl.Invalidate(); };
            _renderTimer.Start();
        }

        private static Button CreateButton(string text, int x, int y, int w, int h, Color bg)
        {
            return new Button
            {
                Text = text,
                Location = new Point(x, y),
                Size = new Size(w, h),
                FlatStyle = FlatStyle.Flat,
                BackColor = bg,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
            };
        }

        private static CheckBox CreateCheckBox(string text, int x, int y, bool chk)
        {
            return new CheckBox
            {
                Text = text,
                Location = new Point(x, y),
                ForeColor = Color.White,
                AutoSize = true,
                Checked = chk,
                Font = new Font("Segoe UI", 9),
            };
        }

        private static Label CreateLabel(string text, int x, int y)
        {
            return new Label
            {
                Text = text,
                Location = new Point(x, y),
                ForeColor = Color.FromArgb(180, 180, 180),
                AutoSize = true,
                Font = new Font("Segoe UI", 8.5f),
            };
        }

        private static NumericUpDown CreateNumericUpDown(int x, int y, int w, int min, int max, decimal val)
        {
            return new NumericUpDown
            {
                Location = new Point(x, y),
                Size = new Size(w, 22),
                Minimum = min,
                Maximum = max,
                Value = Math.Min(max, Math.Max(min, val)),
                DecimalPlaces = 0,
                BackColor = Color.FromArgb(50, 50, 55),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8.5f),
                BorderStyle = BorderStyle.FixedSingle,
            };
        }

        private float GetClampedFovDegrees()
        {
            float fov = Math.Max(30f, Math.Min(140f, _config.SlamCameraFovDeg));
            _cameraController.FieldOfView = fov;
            return fov;
        }

        private void ApplyProjectionMatrix(int width, int height)
        {
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();
            float aspect = (float)width / height;
            var proj = Matrix4.CreatePerspectiveFieldOfView(
                MathHelper.DegreesToRadians(GetClampedFovDegrees()), aspect, 0.05f, 500f);
            GL.LoadMatrix(ref proj);
            GL.MatrixMode(MatrixMode.Modelview);
        }

        // ==================== OpenGL Setup ====================

        private void GlControl_Load(object sender, EventArgs e)
        {
            GL.ClearColor(0.08f, 0.08f, 0.10f, 1.0f);
            GL.Enable(EnableCap.DepthTest);
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            GL.Enable(EnableCap.Lighting);
            GL.Enable(EnableCap.Light0);
            GL.Enable(EnableCap.Light1);
            GL.Enable(EnableCap.ColorMaterial);
            GL.ColorMaterial(MaterialFace.FrontAndBack, ColorMaterialParameter.AmbientAndDiffuse);

            // Main light from upper-right-front
            GL.Light(LightName.Light0, LightParameter.Position, new float[] { 1f, 2f, 1f, 0f });
            GL.Light(LightName.Light0, LightParameter.Diffuse, new float[] { 0.7f, 0.7f, 0.7f, 1f });
            GL.Light(LightName.Light0, LightParameter.Ambient, new float[] { 0.35f, 0.35f, 0.35f, 1f });

            // Fill light from opposite side
            GL.Light(LightName.Light1, LightParameter.Position, new float[] { -1f, 0.5f, -0.5f, 0f });
            GL.Light(LightName.Light1, LightParameter.Diffuse, new float[] { 0.25f, 0.25f, 0.3f, 1f });

            _glInitialized = true;
            GlControl_Resize(sender, e);
        }

        private void GlControl_Resize(object sender, EventArgs e)
        {
            if (!_glInitialized || _glControl == null) return;
            int w = Math.Max(1, _glControl.Width);
            int h = Math.Max(1, _glControl.Height);
            GL.Viewport(0, 0, w, h);
            ApplyProjectionMatrix(w, h);
        }

        // ==================== Main Render ====================

        private void GlControl_Paint(object sender, PaintEventArgs e)
        {
            if (!_glInitialized || _glControl == null) return;

            try
            {
                _glControl.MakeCurrent();
                ApplyProjectionMatrix(Math.Max(1, _glControl.Width), Math.Max(1, _glControl.Height));

                _voxelMeshBuilder.ProcessPendingRebuild();

                BlendRenderPose();

                GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

                GL.MatrixMode(MatrixMode.Modelview);
                GL.LoadIdentity();
                ApplyCameraControllerView();

                _gridRenderer.ShowGrid = _chkShowGrid != null && _chkShowGrid.Checked;
                _trajectoryRenderer.ShowTrajectory = _chkShowTrajectory != null && _chkShowTrajectory.Checked;

                _gridRenderer.Render();
                _voxelMeshBuilder.Render();
                _trajectoryRenderer.Render();
                DrawDroneModel();
                _detectionRenderer.Render();

                _glControl.SwapBuffers();

                // Draw HUD overlay (top-right)
                DrawHudOverlay(e.Graphics);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"SLAM3D render error: {ex.Message}");
            }
        }

        // ==================== Camera ====================

        /// <summary>
        /// Blends raw pose to render pose using PoseState's anti-jitter filtering.
        /// PoseState handles: jump rejection, zero-reset glitch rejection, and smoothing.
        /// </summary>
        private void BlendRenderPose()
        {
            // Read raw pose under lock
            float srcX, srcY, srcZ, srcRoll, srcPitch, srcYaw;
            bool hasBodyAttitude;
            lock (_poseLock)
            {
                srcX = _dronePosX;
                srcY = _dronePosY;
                srcZ = _dronePosZ;
                srcRoll = _droneRollRaw;
                srcPitch = _dronePitchRaw;
                srcYaw = _droneYawRaw;
                hasBodyAttitude = _hasBodyAttitude;
            }

            // Update PoseState (handles jump rejection, smoothing)
            // PoseState expects radians for angles (same as _droneRollRaw etc.)
            _poseState.Update(srcX, srcY, srcZ, srcRoll, srcPitch, srcYaw, hasBodyAttitude);

            // Read filtered output from PoseState (already in radians)
            _renderPosX = _poseState.X;
            _renderPosY = _poseState.Y;
            _renderPosZ = _poseState.Z;
            _renderRollRaw = _poseState.Roll;
            _renderPitchRaw = _poseState.Pitch;
            _renderYawRaw = _poseState.Yaw;

            // Log pose state stats periodically (every 5 seconds)
            LogPoseStateStats();
        }

        private DateTime _lastPoseStatsLogUtc = DateTime.MinValue;

        private void LogPoseStateStats()
        {
            if ((DateTime.UtcNow - _lastPoseStatsLogUtc).TotalSeconds < 5.0)
                return;
            _lastPoseStatsLogUtc = DateTime.UtcNow;

            var (total, rejected, accepted, streak) = _poseState.GetStats();
            if (total > 0)
            {
                double rejectRate = (double)rejected / total * 100.0;
                System.Diagnostics.Debug.WriteLine(
                    $"[PoseState] Total: {total}, Rejected: {rejected} ({rejectRate:F1}%), Accepted: {accepted}, Streak: {streak}");
            }
        }

        private static bool TryReadFloatToken(JToken token, out float value)
        {
            value = 0f;
            if (token == null || (token.Type != JTokenType.Integer && token.Type != JTokenType.Float))
                return false;

            value = token.Value<float>();
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        // Note: AngleMagnitudeDeg and ShouldRejectAttitudeResetGlitch moved to PoseState.cs

        private void ApplyCameraControllerView()
        {
            float servoDeg;
            lock (_poseLock) { servoDeg = _servoAngleDeg; }

            var (glX, glY, glZ) = CoordinateConverter.RosToOpenGL(_renderPosX, _renderPosY, _renderPosZ);

            _cameraController.HeadingOffsetDeg = _config.SlamHeadingOffsetDeg;
            _cameraController.Update(glX, glY, glZ, _renderYawRaw, _renderPitchRaw, _renderRollRaw, servoDeg);

            var view = Matrix4.LookAt(
                _cameraController.EyeX,
                _cameraController.EyeY,
                _cameraController.EyeZ,
                _cameraController.TargetX,
                _cameraController.TargetY,
                _cameraController.TargetZ,
                _cameraController.UpX,
                _cameraController.UpY,
                _cameraController.UpZ);
            GL.LoadMatrix(ref view);
        }

        // ==================== Mouse Controls ====================

        private void GlControl_MouseDown(object sender, MouseEventArgs e)
        {
            _lastMousePos = e.Location;
            if (e.Button == MouseButtons.Left) _mouseRotating = true;
            if (e.Button == MouseButtons.Right || e.Button == MouseButtons.Middle) _mousePanning = true;
        }

        private void GlControl_MouseUp(object sender, MouseEventArgs e)
        {
            _mouseRotating = false;
            _mousePanning = false;
        }

        private void GlControl_MouseMove(object sender, MouseEventArgs e)
        {
            if (_cameraController.ViewMode != CameraViewMode.FreeOrbit) return;
            float dx = e.X - _lastMousePos.X;
            float dy = e.Y - _lastMousePos.Y;
            _lastMousePos = e.Location;

            if (_mouseRotating)
            {
                _cameraController.RotateOrbit(dx * 0.3f, dy * 0.3f);
            }

            if (_mousePanning)
            {
                _cameraController.Pan(dx, dy);
            }
        }

        private void GlControl_MouseWheel(object sender, MouseEventArgs e)
        {
            if (_cameraController.ViewMode != CameraViewMode.FreeOrbit) return;
            _cameraController.Zoom(-e.Delta * 0.01f);
        }

        private void DrawDroneModel()
        {
            if (_cameraController.ViewMode == CameraViewMode.FirstPerson)
                return;

            float servoDeg;
            lock (_poseLock) { servoDeg = _servoAngleDeg; }

            var (glX, glY, glZ) = CoordinateConverter.RosToOpenGL(_renderPosX, _renderPosY, _renderPosZ);

            _droneRenderer.LengthM = _config.DroneLengthCm / 100f;
            _droneRenderer.WidthM = _config.DroneWidthCm / 100f;
            _droneRenderer.HeightM = _config.DroneHeightCm / 100f;
            _droneRenderer.FrameType = _config.DroneFrameType;
            _droneRenderer.HeadingOffsetDeg = _config.SlamHeadingOffsetDeg;
            _droneRenderer.CameraForwardOffsetM = _config.CameraForwardOffsetCm / 100f;
            _droneRenderer.CameraDownOffsetM = _config.CameraDownOffsetCm / 100f;

            _droneRenderer.Draw(glX, glY, glZ, _renderYawRaw, _renderPitchRaw, _renderRollRaw, servoDeg);
        }

        /// <summary>
        /// Draw a HUD overlay in the top-right corner showing pose and velocity.
        /// </summary>
        private void DrawHudOverlay(Graphics g)
        {
            float rollDeg, pitchDeg, yawDeg, vx, vy, vz;
            lock (_poseLock)
            {
                rollDeg = (float)(_droneRollRaw * 180.0 / Math.PI);
                pitchDeg = (float)(_dronePitchRaw * 180.0 / Math.PI);
                yawDeg = (float)(_droneYawRaw * 180.0 / Math.PI);
                vx = _droneVelX;
                vy = _droneVelY;
                vz = _droneVelZ;
            }

            int right = _glControl.Width - 12;
            int top = 12;
            int lineH = 16;
            int pad = 4;

            string[] lines = new string[7];
            lines[0] = "POSE (RAW ZED WS)";
            lines[1] = $"  Roll:  {rollDeg,8:F2}°";
            lines[2] = $"  Pitch: {pitchDeg,8:F2}°";
            lines[3] = $"  Yaw:   {yawDeg,8:F2}°";
            lines[4] = "VELOCITY (m/s)";
            lines[5] = $"  Vx: {vx,7:F3}";
            lines[6] = $"  Vy: {vy,7:F3}  Vz: {vz,7:F3}";

            using (var font = new Font("Consolas", 10f))
            using (var bgBrush = new SolidBrush(Color.FromArgb(140, 15, 15, 20)))
            using (var textBrush = new SolidBrush(Color.FromArgb(220, 220, 220)))
            using (var titleBrush = new SolidBrush(Color.FromArgb(0, 160, 230)))
            using (var pen = new Pen(Color.FromArgb(80, 80, 83), 1))
            {
                // Measure to size the background
                float maxWidth = 0;
                foreach (var line in lines)
                {
                    var sz = g.MeasureString(line, font);
                    if (sz.Width > maxWidth) maxWidth = sz.Width;
                }
                int boxW = (int)maxWidth + pad * 2;
                int boxH = lines.Length * lineH + pad * 2;

                int x = right - boxW;
                int y = top;

                // Background
                g.FillRectangle(bgBrush, x, y, boxW, boxH);
                g.DrawRectangle(pen, x, y, boxW, boxH);

                // Text
                int cy = y + pad;
                for (int i = 0; i < lines.Length; i++)
                {
                    var brush = (i == 0 || i == 4) ? titleBrush : textBrush;
                    g.DrawString(lines[i], font, brush, x + pad, cy);
                    cy += lineH;
                }
            }
        }

        private void HandleWebSocketStatusChanged(string status)
        {
            if (string.IsNullOrWhiteSpace(status))
                return;

            string text = status.StartsWith("Status:", StringComparison.OrdinalIgnoreCase)
                ? status
                : $"Status: {status}";
            UpdateStatusSafe(text);

            if (status.IndexOf("Connected", StringComparison.OrdinalIgnoreCase) >= 0)
                _ = RefreshMeshModeFromServerAsync(updateStatus: false);
        }

        private void HandleWebSocketError(string error)
        {
            if (string.IsNullOrWhiteSpace(error))
                return;
            UpdateStatusSafe($"Status: {error}");
        }

        // ==================== WebSocket Stream ====================

        private void StartUpdateLoop()
        {
            _webSocketClient.BaseUrl = JetsonApiService.BaseUrl ?? "http://100.85.121.98:8000";
            _webSocketClient.ApiKey = JetsonApiService.ApiKey;
            _webSocketClient.Start();
        }

        private void HandleSlamFrame(SlamFrame frame)
        {
            if (frame == null || !_autoUpdateEnabled)
                return;

            var frameJson = frame.RawJson ?? new JObject();
            string frameId = string.IsNullOrWhiteSpace(frame.FrameId) ? "ros_optical" : frame.FrameId;
            if (!string.Equals(frameId, "ros_optical", StringComparison.OrdinalIgnoreCase))
            {
                Debug.WriteLine($"[SLAM3D] Unexpected frame_id: {frameId} (expected ros_optical)");
            }

            bool hasPosePositionInFrame = false;
            float latestX = 0f, latestY = 0f, latestZ = 0f;

            lock (_poseLock)
            {
                if (TryReadFloatToken(frameJson["x"], out float xVal))
                {
                    _dronePosX = xVal;
                    hasPosePositionInFrame = true;
                }

                if (TryReadFloatToken(frameJson["y"], out float yVal))
                {
                    _dronePosY = yVal;
                    hasPosePositionInFrame = true;
                }

                if (TryReadFloatToken(frameJson["z"], out float zVal))
                {
                    _dronePosZ = zVal;
                    hasPosePositionInFrame = true;
                }

                if (hasPosePositionInFrame)
                {
                    long nowPoseStamp = Stopwatch.GetTimestamp();
                    if (_lastRawPoseUpdateStamp > 0)
                    {
                        double gapSec = (double)(nowPoseStamp - _lastRawPoseUpdateStamp) / Stopwatch.Frequency;
                        if (gapSec > PoseSnapAfterGapSec)
                            _poseState.Reset();
                    }
                    _lastRawPoseUpdateStamp = nowPoseStamp;
                }

                bool hasBodyRoll = TryReadFloatToken(frameJson["body_roll"], out float bodyRoll);
                bool hasBodyPitch = TryReadFloatToken(frameJson["body_pitch"], out float bodyPitch);
                bool hasBodyYaw = TryReadFloatToken(frameJson["body_yaw"], out float bodyYaw);

                bool hasRoll;
                bool hasPitch;
                bool hasYaw;
                float nextRoll;
                float nextPitch;
                float nextYaw;

                if (hasBodyRoll && hasBodyPitch && hasBodyYaw)
                {
                    hasRoll = hasPitch = hasYaw = true;
                    nextRoll = bodyRoll;
                    nextPitch = bodyPitch;
                    nextYaw = bodyYaw;
                    _hasBodyAttitude = true;
                }
                else
                {
                    hasRoll = TryReadFloatToken(frameJson["roll"], out nextRoll);
                    hasPitch = TryReadFloatToken(frameJson["pitch"], out nextPitch);
                    hasYaw = TryReadFloatToken(frameJson["yaw"], out nextYaw);
                    _hasBodyAttitude = false;
                }

                if (hasRoll && hasPitch && hasYaw)
                {
                    _droneRollRaw = nextRoll;
                    _dronePitchRaw = nextPitch;
                    _droneYawRaw = nextYaw;
                }

                if (TryReadFloatToken(frameJson["vx"], out float vxVal))
                    _droneVelX = vxVal;
                if (TryReadFloatToken(frameJson["vy"], out float vyVal))
                    _droneVelY = vyVal;
                if (TryReadFloatToken(frameJson["vz"], out float vzVal))
                    _droneVelZ = vzVal;

                latestX = _dronePosX;
                latestY = _dronePosY;
                latestZ = _dronePosZ;
            }

            if (hasPosePositionInFrame)
                _trajectoryRenderer.AddPointFromRos(latestX, latestY, latestZ);

            UpdateDetectionMarkersFromFrame(frameJson);

            if (frame.Type == SlamFrameType.Mesh && frame.MeshToken != null)
            {
                var meshData = frame.MeshToken.ToObject<MeshDataModel>();
                if (meshData == null)
                    return;

                _voxelMeshBuilder.UpdateMesh(meshData);
                _meshUpdateCount++;

                _totalBlocks = meshData.TotalBlocks > 0 ? meshData.TotalBlocks : meshData.TotalVoxels;
                string mode = (meshData.Mode == "voxel" || meshData.Mode == "voxels") ? "voxels" : "blocks";
                string statsText = $"Mesh: {_totalBlocks:N0} {mode} ({_voxelMeshBuilder.VoxelCount:N0} cached)";

                UpdateStatusSafe($"Status: Connected (30Hz) | Updates: {_meshUpdateCount}");
                UpdateStatsSafe(statsText);
            }
        }

        private void UpdateDetectionMarkersFromFrame(JObject frameJson)
        {
            if (frameJson == null)
                return;

            var detectionsToken = frameJson["detections"] as JArray;
            if (detectionsToken == null)
            {
                if (frameJson.ContainsKey("detections"))
                    _detectionRenderer.Clear();
                return;
            }

            var markers = new List<DetectionMarkerData>(detectionsToken.Count);
            foreach (var detection in detectionsToken)
            {
                var dx = detection["x"]?.Value<double?>();
                var dy = detection["y"]?.Value<double?>();
                var dz = detection["z"]?.Value<double?>();
                if (dx == null || dy == null || dz == null)
                    continue;

                var (gx, gy, gz) = CoordinateConverter.RosToOpenGL((float)dx.Value, (float)dy.Value, (float)dz.Value);
                markers.Add(new DetectionMarkerData
                {
                    Label = detection["label"]?.ToString() ?? string.Empty,
                    X = gx,
                    Y = gy,
                    Z = gz,
                    Confidence = detection["confidence"]?.Value<float>() ?? 0f,
                    SeenCount = detection["seen_count"]?.Value<int>() ?? 1,
                    ColorMatch = detection["color_match"]?.Value<bool?>() ?? true,
                    NeedsReview = detection["needs_review"]?.Value<bool>() ?? false,
                });
            }

            _detectionRenderer.UpdateMarkers(markers);
        }

        // ==================== Servo Polling ====================

        private void StartServoPolling()
        {
            _servoTimer = new System.Windows.Forms.Timer { Interval = 500 };
            _servoTimer.Tick += async (s, e) =>
            {
                try
                {
                    var response = await JetsonApiService.GetAsync("/api/servo/camera/tilt");
                    if (response.IsSuccessStatusCode)
                    {
                        var json = await response.Content.ReadAsStringAsync();
                        var obj = JObject.Parse(json);
                        lock (_poseLock)
                        {
                            _servoAngleDeg = obj["angle"]?.Value<float>() ?? 90f;
                        }
                    }
                }
                catch { }
            };
            _servoTimer.Start();
        }

        private void StartPerceptionStatusPolling()
        {
            _statusTimer = new System.Windows.Forms.Timer { Interval = 1000 };
            _statusTimer.Tick += async (s, e) =>
            {
                if (_statusPollInFlight) return;
                _statusPollInFlight = true;
                try
                {
                    await PollPerceptionStatusAsync();
                }
                finally
                {
                    _statusPollInFlight = false;
                }
            };
            _statusTimer.Start();
        }

        private async Task PollPerceptionStatusAsync()
        {
            string hsvText = "HSV: --";
            string servoText = "Servo: --";
            string scanText = "ScanStopScan: --";

            try
            {
                var detectionsResponse = await JetsonApiService.GetAsync("/api/detections");
                if (detectionsResponse.IsSuccessStatusCode)
                {
                    var detectionsJson = await detectionsResponse.Content.ReadAsStringAsync();
                    var detectionsObj = JObject.Parse(detectionsJson);
                    var current = detectionsObj["current"]? ["detections"] as JArray;
                    int total = current?.Count ?? 0;
                    int mismatches = 0;
                    string sampleMismatch = null;

                    if (current != null)
                    {
                        foreach (var det in current)
                        {
                            bool needsReview = det["needs_review"]?.Value<bool>() ?? false;
                            bool colorMatch = det["color_match"]?.Value<bool?>() ?? true;
                            string hsvColor = det["hsv_color"]?.ToString() ?? string.Empty;
                            if (needsReview || (!colorMatch && !string.IsNullOrEmpty(hsvColor)))
                            {
                                mismatches++;
                                if (sampleMismatch == null)
                                {
                                    string label = det["label"]?.ToString() ?? "unknown";
                                    sampleMismatch = string.IsNullOrEmpty(hsvColor) ? label : $"{label}->{hsvColor}";
                                }
                            }
                        }
                    }

                    if (total == 0)
                        hsvText = "HSV: No detections";
                    else if (mismatches > 0)
                        hsvText = sampleMismatch == null
                            ? $"HSV: {mismatches}/{total} mismatch"
                            : $"HSV: {mismatches}/{total} mismatch ({sampleMismatch})";
                    else
                        hsvText = $"HSV: OK ({total})";
                }
            }
            catch { }

            try
            {
                var servoResponse = await JetsonApiService.GetAsync("/api/servo/status");
                if (servoResponse.IsSuccessStatusCode)
                {
                    var servoJson = await servoResponse.Content.ReadAsStringAsync();
                    var servoObj = JObject.Parse(servoJson);
                    bool available = servoObj["available"]?.Value<bool>() ?? false;
                    var camTilt = servoObj["servos"]?["camera_tilt"];
                    bool enabled = camTilt?["enabled"]?.Value<bool>() ?? false;
                    float angle = camTilt?["angle"]?.Value<float>() ?? _servoAngleDeg;
                    servoText = available
                        ? $"Servo: {(enabled ? "Enabled" : "Disabled")} {angle:F1} deg"
                        : "Servo: Not available";
                }
            }
            catch { }

            try
            {
                var poseResponse = await JetsonApiService.GetAsync("/api/vio/pose");
                if (poseResponse.IsSuccessStatusCode)
                {
                    var poseJson = await poseResponse.Content.ReadAsStringAsync();
                    var poseObj = JObject.Parse(poseJson);
                    bool valid = poseObj["valid"]?.Value<bool?>() ?? true;
                    if (valid)
                    {
                        float vx = poseObj["vx"]?.Value<float>() ?? 0f;
                        float vy = poseObj["vy"]?.Value<float>() ?? 0f;
                        float vz = poseObj["vz"]?.Value<float>() ?? 0f;
                        float speed = (float)Math.Sqrt(vx * vx + vy * vy + vz * vz);
                        bool moving = speed > ScanStopThresholdMps;
                        scanText = moving
                            ? $"ScanStopScan: HOLD ({speed:F2} m/s)"
                            : $"ScanStopScan: SCAN ({speed:F2} m/s)";
                    }
                    else
                    {
                        scanText = "ScanStopScan: No VIO";
                    }
                }
            }
            catch { }

            UpdatePerceptionStatusSafe($"{hsvText} | {servoText} | {scanText}");
        }

        // ==================== Event Handlers ====================

        private void BtnToggleCamera_Click(object sender, EventArgs e)
        {
            var nextMode = _cameraController.CycleViewMode();
            if (nextMode == CameraViewMode.FreeOrbit)
            {
                var (glX, glY, glZ) = CoordinateConverter.RosToOpenGL(_renderPosX, _renderPosY, _renderPosZ);
                _cameraController.CenterOn(glX, glY, glZ);
            }

            UpdateCameraToggleLabel();
        }

        private void BtnResetView_Click(object sender, EventArgs e)
        {
            _cameraController.Reset();
        }

        private void CenterOrbitOnCurrentPose()
        {
            var (glX, glY, glZ) = CoordinateConverter.RosToOpenGL(_renderPosX, _renderPosY, _renderPosZ);
            _cameraController.CenterOn(glX, glY, glZ);
            _cameraController.ViewMode = CameraViewMode.FreeOrbit;
            UpdateCameraToggleLabel();
        }

        private void UpdateCameraToggleLabel()
        {
            if (_btnToggleCamera == null)
                return;

            string modeName = _cameraController.ViewMode switch
            {
                CameraViewMode.FirstPerson => "FPV",
                CameraViewMode.ThirdPerson => "TPV",
                CameraViewMode.FreeOrbit => "Orbit",
                _ => "?"
            };
            _btnToggleCamera.Text = $"View: {modeName}";
        }

        private async void CombMeshMode_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (_combMeshMode == null || _meshModeSelectionInternal || _meshModeApplyInFlight)
                return;

            // Voxel-only mode: keep the control as a status indicator.
            if (_meshOutputMode == "voxel")
                return;

            _meshModeApplyInFlight = true;
            _combMeshMode.Enabled = false;
            try
            {
                var response = await JetsonApiService.PostAsync("/api/task/2/slam/mesh/mode?mode=voxel");
                if (response.IsSuccessStatusCode)
                {
                    _meshOutputMode = "voxel";
                    UpdateStatusSafe("Status: Mesh mode set to voxel");
                    AppendStatusLogSafe("Mesh mode set to voxel");
                }
                else
                {
                    SetMeshModeSelection("voxel");
                    UpdateStatusSafe($"Status: Mesh mode change failed ({(int)response.StatusCode})");
                    AppendStatusLogSafe($"Mesh mode change failed ({(int)response.StatusCode})");
                }
            }
            catch (Exception ex)
            {
                SetMeshModeSelection("voxel");
                UpdateStatusSafe($"Status: Mesh mode change failed ({ex.Message})");
                AppendStatusLogSafe($"Mesh mode change failed ({ex.Message})");
            }
            finally
            {
                _combMeshMode.Enabled = true;
                _meshModeApplyInFlight = false;
            }
        }

        private async void BtnClearMesh_Click(object sender, EventArgs e)
        {
            _voxelMeshBuilder.Clear();
            _trajectoryRenderer.Clear();
            _detectionRenderer.Clear();
            _totalBlocks = 0;

            try
            {
                await JetsonApiService.PostAsync("/api/task/2/slam/clear");
                UpdateStatusSafe("Mesh cleared");
                AppendStatusLogSafe("Mesh cleared");
            }
            catch (Exception ex)
            {
                UpdateStatusSafe($"Mesh cleared locally (server: {ex.Message})");
                AppendStatusLogSafe($"Mesh clear sent locally (server warning: {ex.Message})");
            }
        }

        private async void BtnResetImuBiases_Click(object sender, EventArgs e)
        {
            var result = MessageBox.Show(
                "Reset IMU bias values stored in the ZED camera's internal EEPROM?\n\n" +
                "This is equivalent to running 'ZED Sensor Calibration.exe --cimu'.\n" +
                "Use this if camera orientation continues to drift after warmup.\n\n" +
                "The camera must be stationary during this operation.",
                "Reset IMU Biases",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning
            );

            if (result != DialogResult.Yes) return;

            // IMU EEPROM reset needs exclusive access to the ZED camera.
            // Block early if Isaac ROS/nvblox currently owns the camera.
            try
            {
                var isaacResponse = await JetsonApiService.GetAsync("/api/isaac/status");
                if (isaacResponse.IsSuccessStatusCode)
                {
                    var isaacBody = await isaacResponse.Content.ReadAsStringAsync();
                    var isaacData = JObject.Parse(isaacBody);
                    bool isaacRunning = isaacData["running"]?.Value<bool>() ?? false;
                    if (isaacRunning)
                    {
                        UpdateStatusSafe("Stop Isaac ROS before IMU reset");
                        AppendStatusLogSafe("IMU reset blocked: Isaac ROS/nvblox is running.");
                        MessageBox.Show(
                            "IMU reset requires exclusive camera access, but Isaac ROS/nvblox is currently using the ZED camera.\n\n" +
                            "Stop Isaac ROS first, then retry IMU reset.",
                            "IMU Reset Blocked",
                            MessageBoxButtons.OK,
                            MessageBoxIcon.Warning
                        );
                        return;
                    }
                }
            }
            catch
            {
                // If status probe fails, continue and let the reset endpoint return a concrete error.
            }

            _btnResetImuBiases.Enabled = false;
            _btnResetImuBiases.Text = "Resetting...";
            UpdateStatusSafe("Resetting IMU biases...");

            try
            {
                var response = await JetsonApiService.PostLongRunAsync("/api/calibration/imu/reset_biases");
                var body = await response.Content.ReadAsStringAsync();
                var data = Newtonsoft.Json.Linq.JObject.Parse(body);

                if (response.IsSuccessStatusCode)
                {
                    UpdateStatusSafe("IMU biases reset successfully");
                    AppendStatusLogSafe("IMU biases reset. Camera EEPROM cleared.");
                    MessageBox.Show(
                        "IMU biases reset successfully.\n\n" +
                        "The camera's internal bias values have been cleared.\n" +
                        "Allow the camera to warm up for a few minutes before use.",
                        "IMU Reset Complete",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information
                    );
                }
                else
                {
                    var detail = data["detail"]?.ToString() ?? "Unknown error";
                    UpdateStatusSafe($"IMU reset failed: {detail}");
                    AppendStatusLogSafe($"IMU reset failed: {detail}");
                    MessageBox.Show(
                        $"Failed to reset IMU biases:\n\n{detail}",
                        "IMU Reset Failed",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Error
                    );
                }
            }
            catch (Exception ex)
            {
                UpdateStatusSafe($"IMU reset error: {ex.Message}");
                AppendStatusLogSafe($"IMU reset error: {ex.Message}");
                MessageBox.Show(
                    $"Error resetting IMU biases:\n\n{ex.Message}",
                    "IMU Reset Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
            finally
            {
                _btnResetImuBiases.Enabled = true;
                _btnResetImuBiases.Text = "Reset IMU";
            }
        }

        // ==================== Helpers ====================

        private void UpdateStatusSafe(string text)
        {
            if (_lblStatus == null) return;
            if (_lblStatus.InvokeRequired)
                _lblStatus.BeginInvoke(new Action(() => { if (_lblStatus != null) _lblStatus.Text = text; }));
            else
                _lblStatus.Text = text;
        }

        private void UpdateStatsSafe(string text)
        {
            if (_lblStats == null) return;
            if (_lblStats.InvokeRequired)
                _lblStats.BeginInvoke(new Action(() => { if (_lblStats != null) _lblStats.Text = text; }));
            else
                _lblStats.Text = text;
        }

        private void UpdatePerceptionStatusSafe(string text)
        {
            if (_lblPerceptionStatus == null) return;
            if (_lblPerceptionStatus.InvokeRequired)
                _lblPerceptionStatus.BeginInvoke(new Action(() => { if (_lblPerceptionStatus != null) _lblPerceptionStatus.Text = text; }));
            else
                _lblPerceptionStatus.Text = text;
        }

        private void AppendStatusLogSafe(string text)
        {
            if (string.IsNullOrWhiteSpace(text) || _txtStatusLog == null || _txtStatusLog.IsDisposed)
                return;

            void Append()
            {
                if (_txtStatusLog == null || _txtStatusLog.IsDisposed)
                    return;

                string timestamp = DateTime.Now.ToString("HH:mm:ss");
                _txtStatusLog.AppendText($"[{timestamp}] {text}\r\n");

                var lines = _txtStatusLog.Lines;
                if (lines.Length > MaxStatusLogLines)
                {
                    _txtStatusLog.Lines = lines.Skip(lines.Length - MaxStatusLogLines).ToArray();
                    _txtStatusLog.SelectionStart = _txtStatusLog.TextLength;
                    _txtStatusLog.ScrollToCaret();
                }
            }

            if (_txtStatusLog.InvokeRequired)
                _txtStatusLog.BeginInvoke((Action)Append);
            else
                Append();
        }

        private static string SummarizeCommandResult(CommandResult result)
        {
            if (result == null)
                return "Unknown result";

            if (!string.IsNullOrWhiteSpace(result.Data))
            {
                try
                {
                    var json = JObject.Parse(result.Data);
                    var message = json["message"]?.ToString()
                        ?? json["detail"]?.ToString()
                        ?? json["output"]?.ToString();
                    if (!string.IsNullOrWhiteSpace(message))
                        return message;
                }
                catch
                {
                    // Fall back to the transport-level message below.
                }
            }

            return string.IsNullOrWhiteSpace(result.Message) ? "Completed" : result.Message;
        }

        private static bool IsServiceUnavailableMessage(string message)
        {
            string text = (message ?? string.Empty).ToLowerInvariant();
            return text.Contains("waiting for service")
                || text.Contains("service to be available")
                || text.Contains("service is not available")
                || text.Contains("service unavailable");
        }

        private static string BuildAreaMapFailureText(string actionName, string summary)
        {
            if (IsServiceUnavailableMessage(summary))
            {
                return $"{actionName} failed: map service not ready yet (start Isaac ROS + nvblox, then retry)";
            }

            if (!string.IsNullOrWhiteSpace(summary)
                && summary.IndexOf("FilePath_Response(success=False)", StringComparison.OrdinalIgnoreCase) >= 0)
            {
                return $"{actionName} failed: nvblox rejected the request (map not ready yet or file path invalid)";
            }

            return $"{actionName} failed ({summary})";
        }

        private async Task<CommandResult> ExecuteAreaMapCommandWithRetryAsync(
            Func<Task<CommandResult>> command,
            string actionName)
        {
            CommandResult lastResult = null;
            const int maxServiceReadyAttempts = 6;

            for (int attempt = 1; attempt <= maxServiceReadyAttempts; attempt++)
            {
                lastResult = await command();
                if (lastResult.Success)
                    return lastResult;

                string summary = SummarizeCommandResult(lastResult);
                if (!IsServiceUnavailableMessage(summary) || attempt >= maxServiceReadyAttempts)
                    return lastResult;

                AppendStatusLogSafe($"{actionName}: service not ready (attempt {attempt}/{maxServiceReadyAttempts}), retrying in 2s...");
                await Task.Delay(2000);
            }

            return lastResult ?? new CommandResult
            {
                Success = false,
                Message = $"{actionName} failed",
                Method = "HTTP",
            };
        }

        private static string NormalizeMeshMode(string mode)
        {
            return "voxel";
        }

        private void SetMeshModeSelection(string mode)
        {
            if (_combMeshMode == null)
                return;

            string target = NormalizeMeshMode(mode) == "voxel" ? "Voxel" : "Block";
            string current = _combMeshMode.SelectedItem?.ToString() ?? "";
            if (string.Equals(current, target, StringComparison.OrdinalIgnoreCase))
                return;

            _meshModeSelectionInternal = true;
            try
            {
                _combMeshMode.SelectedItem = target;
            }
            finally
            {
                _meshModeSelectionInternal = false;
            }
        }

        private async Task RefreshMeshModeFromServerAsync(bool updateStatus)
        {
            if (_meshModeRefreshInFlight)
                return;

            _meshModeRefreshInFlight = true;
            try
            {
                var response = await JetsonApiService.GetAsync("/api/task/2/slam/mesh/mode");
                if (!response.IsSuccessStatusCode)
                    return;

                var body = await response.Content.ReadAsStringAsync();
                var obj = JObject.Parse(body);
                string mode = NormalizeMeshMode(obj["mesh_output_mode"]?.ToString());
                _meshOutputMode = mode;

                if (_combMeshMode != null)
                {
                    if (_combMeshMode.InvokeRequired)
                        _combMeshMode.BeginInvoke(new Action(() => SetMeshModeSelection(mode)));
                    else
                        SetMeshModeSelection(mode);
                }

                if (updateStatus)
                    UpdateStatusSafe("Status: Mesh mode is voxel");
            }
            catch
            {
                // Keep last-known mode when endpoint is unavailable.
            }
            finally
            {
                _meshModeRefreshInFlight = false;
            }
        }

        private async Task SaveAreaMapAsync()
        {
            var path = (_txtMapPath?.Text ?? string.Empty).Trim();
            if (string.IsNullOrWhiteSpace(path))
            {
                UpdateStatusSafe("Status: Area map path is empty");
                AppendStatusLogSafe("Save area map failed: path is empty");
                return;
            }

            try
            {
                UpdateStatusSafe("Status: Saving area map...");
                AppendStatusLogSafe($"Save area map requested: {path}");

                var result = await ExecuteAreaMapCommandWithRetryAsync(
                    () => _sender.SaveAreaMapAsync(path),
                    "Save area map");
                var message = SummarizeCommandResult(result);

                if (result.Success)
                {
                    UpdateStatusSafe($"Status: {message}");
                    AppendStatusLogSafe($"Save area map succeeded: {message}");
                }
                else
                {
                    string failureText = BuildAreaMapFailureText("Save area map", message);
                    UpdateStatusSafe($"Status: {failureText}");
                    AppendStatusLogSafe(failureText);
                }
            }
            catch (Exception ex)
            {
                string failureText = $"Save area map failed ({ex.Message})";
                UpdateStatusSafe($"Status: {failureText}");
                AppendStatusLogSafe(failureText);
            }
        }

        private async Task LoadAreaMapAsync()
        {
            var path = (_txtMapPath?.Text ?? string.Empty).Trim();
            if (string.IsNullOrWhiteSpace(path))
            {
                UpdateStatusSafe("Status: Area map path is empty");
                AppendStatusLogSafe("Load area map failed: path is empty");
                return;
            }

            try
            {
                UpdateStatusSafe("Status: Loading area map...");
                AppendStatusLogSafe($"Load area map requested: {path}");

                var result = await ExecuteAreaMapCommandWithRetryAsync(
                    () => _sender.LoadAreaMapAsync(path),
                    "Load area map");
                if (result.Success)
                    CenterOrbitOnCurrentPose();

                var message = SummarizeCommandResult(result);
                if (result.Success)
                {
                    UpdateStatusSafe($"Status: {message}");
                    AppendStatusLogSafe($"Load area map succeeded: {message}");
                }
                else
                {
                    string failureText = BuildAreaMapFailureText("Load area map", message);
                    UpdateStatusSafe($"Status: {failureText}");
                    AppendStatusLogSafe(failureText);
                }
            }
            catch (Exception ex)
            {
                string failureText = $"Load area map failed ({ex.Message})";
                UpdateStatusSafe($"Status: {failureText}");
                AppendStatusLogSafe(failureText);
            }
        }

        private async Task RelocalizeAreaMapAsync()
        {
            var path = (_txtMapPath?.Text ?? string.Empty).Trim();
            if (string.IsNullOrWhiteSpace(path))
            {
                UpdateStatusSafe("Status: Area map path is empty");
                AppendStatusLogSafe("Relocalize area map failed: path is empty");
                return;
            }

            try
            {
                UpdateStatusSafe("Status: Relocalizing...");
                AppendStatusLogSafe($"Relocalize area map requested: {path}");

                var result = await ExecuteAreaMapCommandWithRetryAsync(
                    () => _sender.RelocalizeAreaMapAsync(path),
                    "Relocalize area map");
                if (result.Success)
                    CenterOrbitOnCurrentPose();

                var message = SummarizeCommandResult(result);
                if (result.Success)
                {
                    UpdateStatusSafe($"Status: {message}");
                    AppendStatusLogSafe($"Relocalize area map succeeded: {message}");
                }
                else
                {
                    string failureText = BuildAreaMapFailureText("Relocalize area map", message);
                    UpdateStatusSafe($"Status: {failureText}");
                    AppendStatusLogSafe(failureText);
                }
            }
            catch (Exception ex)
            {
                string failureText = $"Relocalize area map failed ({ex.Message})";
                UpdateStatusSafe($"Status: {failureText}");
                AppendStatusLogSafe(failureText);
            }
        }

        // ==================== Cleanup ====================

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _renderTimer?.Stop();
                _renderTimer?.Dispose();
                _servoTimer?.Stop();
                _servoTimer?.Dispose();
                _statusTimer?.Stop();
                _statusTimer?.Dispose();
                try { _webSocketClient.Stop(); } catch { }
                try { _webSocketClient.Dispose(); } catch { }
                _glControl?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
