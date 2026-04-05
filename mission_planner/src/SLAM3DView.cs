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
using System.IO;
using System.Linq;
using System.Net.WebSockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;

namespace NOMAD.MissionPlanner
{
    /// <summary>Camera view mode for 3D visualization.</summary>
    public enum CameraViewMode
    {
        FirstPerson,
        ThirdPerson,
        FreeOrbit
    }

    // ==================== Data Models ====================

    public class MeshDataModel
    {
        [JsonProperty("blocks")]
        public List<MeshBlockModel> Blocks { get; set; }
        [JsonProperty("block_size")]
        public double BlockSize { get; set; }
        [JsonProperty("total_blocks")]
        public int TotalBlocks { get; set; }
        [JsonProperty("mode")]
        public string Mode { get; set; }
        [JsonProperty("timestamp")]
        public double Timestamp { get; set; }
        [JsonProperty("frame_id")]
        public string FrameId { get; set; }
        [JsonProperty("clear")]
        public bool Clear { get; set; }
        [JsonProperty("voxels")]
        public List<VoxelModel> Voxels { get; set; }
        [JsonProperty("voxel_size")]
        public double VoxelSize { get; set; }
        [JsonProperty("total_voxels")]
        public int TotalVoxels { get; set; }
        [JsonProperty("removed")]
        public List<RemovedVoxelModel> Removed { get; set; }

    }

    public class VoxelModel
    {
        [JsonProperty("p")]
        public List<double> Position { get; set; }
        [JsonProperty("c")]
        public List<int> Color { get; set; }
    }

    public class RemovedVoxelModel
    {
        [JsonProperty("x")]
        public int X { get; set; }
        [JsonProperty("y")]
        public int Y { get; set; }
        [JsonProperty("z")]
        public int Z { get; set; }
    }

    public class MeshBlockModel
    {
        [JsonProperty("index")]
        public List<int> Index { get; set; }
        [JsonProperty("i")]
        private List<int> CompactIndex
        {
            set
            {
                if ((Index == null || Index.Count == 0) && value != null)
                    Index = value;
            }
        }
        [JsonProperty("color")]
        public List<int> Color { get; set; }
        [JsonProperty("c")]
        private List<int> CompactColor
        {
            set
            {
                if ((Color == null || Color.Count == 0) && value != null)
                    Color = value;
            }
        }
    }

    public class DetectionMarker3D
    {
        public string Label { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double Confidence { get; set; }
        public int SeenCount { get; set; }
        public string HsvColor { get; set; }
        public bool ColorMatch { get; set; } = true;
        public bool NeedsReview { get; set; }
    }

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

        // ---- WebSocket ----
        private ClientWebSocket _webSocket;
        private CancellationTokenSource _updateCts;
        private int _wsReconnectDelayMs = 1000;
        private const int MaxWsReconnectDelayMs = 10000;
        private volatile bool _autoUpdateEnabled = true;
        private readonly object _poseLock = new object();
        private readonly object _meshLock = new object();
        private readonly object _trajectoryLock = new object();
        private const int MaxWebSocketMessageSize = 10 * 1024 * 1024;

        // ---- Servo polling ----
        private float _servoAngleDeg = 90.0f;
        private System.Windows.Forms.Timer _servoTimer;

        // ---- Perception/status polling ----
        private System.Windows.Forms.Timer _statusTimer;
        private bool _statusPollInFlight;
        private const float ScanStopThresholdMps = 0.10f;

        // ---- Drone pose (raw from WS, ZED optical/odom frame) ----
        private float _dronePosX, _dronePosY, _dronePosZ;
        private float _droneRollRaw, _dronePitchRaw, _droneYawRaw;
        private float _droneVelX, _droneVelY, _droneVelZ;
        private float _renderPosX, _renderPosY, _renderPosZ;
        private float _renderRollRaw, _renderPitchRaw, _renderYawRaw;
        private bool _renderPoseInitialized;
        private long _lastPoseBlendStamp = -1;
        private long _lastRawPoseUpdateStamp = -1;
        private const float PoseBlendRateHz = 24.0f;
        private const double PoseSnapAfterGapSec = 0.40;
        private DateTime _lastPoseResetDropLogUtc = DateTime.MinValue;
        private const float PoseResetJumpThresholdDeg = 25.0f;
        private const float PoseResetNearZeroDeg = 8.0f;
        private const float PoseResetPrevMinDeg = 15.0f;
        private const float PoseResetMaxPositionDeltaM = 0.35f;
        private const int PoseResetRejectStreakLimit = 3;
        private int _poseResetRejectStreak = 0;

        // ---- Voxel storage ----
        private Dictionary<long, uint> _persistedBlocks = new Dictionary<long, uint>();
        private Dictionary<long, int> _voxelLastSeen = new Dictionary<long, int>(); // key -> update generation
        private int _meshGeneration = 0; // incremented each mesh update
        private const int VoxelMaxAge = 10; // expire after N updates without being re-seen
        private Queue<long> _voxelInsertionOrder = new Queue<long>();
        private HashSet<long> _queuedForEviction = new HashSet<long>();
        private HashSet<long> _occupancySet = new HashSet<long>();
        private const int MaxPersistedVoxels = 5000;
        private double _currentVoxelSize = 0.05;
        private string _currentMeshMode = ""; // "voxel" or "block" — clear data on mode switch

        // ---- GL vertex data (rebuilt when mesh changes) ----
        private float[] _voxelVerts;   // interleaved: pos(3) + color(3) + normal(3) = 9 floats/vert
        private int[] _voxelIndices;
        private int _voxelIndexCount;

        // ---- Mesh rebuild tracking ----
        private bool _meshDirty;
        private int _lastRenderedCount;
        private const int MinNewVoxelsForRebuild = 20;
        private DateTime _lastMeshRebuild = DateTime.MinValue;
        private static readonly TimeSpan MinRebuildInterval = TimeSpan.FromMilliseconds(250);
        private bool _pendingMeshUpdate = false;  // P3-7: Flag to mark queued updates during debounce
        private long _lastMeshRebuildStamp = -1;  // Stopwatch ticks (monotonic)
        // Use readonly (not const) so the compiler does not fold branches in LogMeshDebounce.
        private static readonly bool EnableMeshDebounceDebugLog = true;

        // ---- Trajectory ----
        private List<float[]> _trajectoryPoints = new List<float[]>(); // each [x,y,z] in ZED frame
        private const int MaxTrajectoryPoints = 500;

        // ---- Detection markers ----
        private List<DetectionMarker3D> _detectionMarkers = new List<DetectionMarker3D>();

        // ---- Camera ----
        private CameraViewMode _currentViewMode = CameraViewMode.ThirdPerson;
        private float _orbitYaw = 45f, _orbitPitch = 30f, _orbitDistance = 12f;
        private float _orbitCenterX, _orbitCenterY, _orbitCenterZ;
        private Point _lastMousePos;
        private bool _mouseRotating, _mousePanning;
        // Stored camera vectors for pan calculation
        private float _camPosX, _camPosY, _camPosZ;
        private float _camTgtX, _camTgtY, _camTgtZ;

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
        private DateTime _lastUpdateTime = DateTime.MinValue;
        private const int MaxStatusLogLines = 120;

        // ==================== Voxel Key Helpers ====================

        private static long PackKey(int x, int y, int z)
        {
            long lx = (long)(x + 0x100000) & 0xFFFFF;
            long ly = (long)(y + 0x100000) & 0xFFFFF;
            long lz = (long)(z + 0x100000) & 0xFFFFF;
            return (lx << 40) | (ly << 20) | lz;
        }

        private static long PackVoxelKey(int ix, int iy, int iz)
        {
            return ((long)(ix + 32768) << 32) | ((long)(iy + 32768) << 16) | (long)(iz + 32768);
        }

        private static void UnpackVoxelKey(long key, out int ix, out int iy, out int iz)
        {
            iz = (int)((key & 0xFFFF) - 32768);
            iy = (int)(((key >> 16) & 0xFFFF) - 32768);
            ix = (int)(((key >> 32) & 0xFFFF) - 32768);
        }

        private void ClearEvictionTracking()
        {
            _voxelInsertionOrder.Clear();
            _queuedForEviction.Clear();
        }

        private void QueueForEviction(long key)
        {
            if (_queuedForEviction.Add(key))
                _voxelInsertionOrder.Enqueue(key);
        }

        private void UnqueueForEviction(long key)
        {
            if (!_queuedForEviction.Remove(key) || _voxelInsertionOrder.Count == 0)
                return;

            var rebuilt = new Queue<long>(_voxelInsertionOrder.Count);
            while (_voxelInsertionOrder.Count > 0)
            {
                long queued = _voxelInsertionOrder.Dequeue();
                if (queued != key)
                    rebuilt.Enqueue(queued);
            }
            _voxelInsertionOrder = rebuilt;
        }

        // ==================== Coordinate Conversion ====================

        /// <summary>
        /// Convert position from ROS odom frame (X-forward, Y-left, Z-up)
        /// to OpenGL frame (X-right, Y-up, Z-toward-viewer).
        /// Mapping chosen to keep heading and mesh conventions consistent:
        ///   gx = -y, gy = z, gz = -x
        /// </summary>
        private static void ZedToGL(float x, float y, float z, out float gx, out float gy, out float gz)
        {
            gx = -y;
            gy = z;
            gz = -x;
        }

        // ==================== Constructor ====================

        public SLAM3DView(NOMADConfig config, DualLinkSender sender)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            _sender = sender ?? throw new ArgumentNullException(nameof(sender));
            InitializeComponents();
            StartUpdateLoop();
            StartServoPolling();
            StartPerceptionStatusPolling();
            _ = RefreshMeshModeFromServerAsync(updateStatus: false);
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
            return Math.Max(30f, Math.Min(140f, _config.SlamCameraFovDeg));
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
                
                // P3-7: Process any pending mesh updates if debounce window has elapsed
                ProcessPendingMeshUpdate();

                BlendRenderPose();
                
                GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

                GL.MatrixMode(MatrixMode.Modelview);
                GL.LoadIdentity();
                SetupCamera();

                if (_chkShowGrid != null && _chkShowGrid.Checked) DrawGrid();
                DrawVoxels();
                if (_chkShowTrajectory != null && _chkShowTrajectory.Checked) DrawTrajectory();
                DrawDrone();
                DrawDetectionMarkers();

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

        private void BlendRenderPose()
        {
            float srcX, srcY, srcZ, srcRoll, srcPitch, srcYaw;
            lock (_poseLock)
            {
                srcX = _dronePosX;
                srcY = _dronePosY;
                srcZ = _dronePosZ;
                srcRoll = _droneRollRaw;
                srcPitch = _dronePitchRaw;
                srcYaw = _droneYawRaw;
            }

            long nowStamp = Stopwatch.GetTimestamp();
            if (!_renderPoseInitialized)
            {
                _renderPosX = srcX;
                _renderPosY = srcY;
                _renderPosZ = srcZ;
                _renderRollRaw = srcRoll;
                _renderPitchRaw = srcPitch;
                _renderYawRaw = srcYaw;
                _renderPoseInitialized = true;
                _lastPoseBlendStamp = nowStamp;
                return;
            }

            double dtSec = 0.0;
            if (_lastPoseBlendStamp > 0)
                dtSec = (double)(nowStamp - _lastPoseBlendStamp) / Stopwatch.Frequency;
            _lastPoseBlendStamp = nowStamp;

            float alpha = 1.0f;
            if (dtSec > 0)
            {
                alpha = (float)(1.0 - Math.Exp(-PoseBlendRateHz * dtSec));
                alpha = Math.Max(0.02f, Math.Min(1.0f, alpha));
            }

            _renderPosX += (srcX - _renderPosX) * alpha;
            _renderPosY += (srcY - _renderPosY) * alpha;
            _renderPosZ += (srcZ - _renderPosZ) * alpha;
            _renderRollRaw = BlendAngleRadians(_renderRollRaw, srcRoll, alpha);
            _renderPitchRaw = BlendAngleRadians(_renderPitchRaw, srcPitch, alpha);
            _renderYawRaw = BlendAngleRadians(_renderYawRaw, srcYaw, alpha);
        }

        private static float BlendAngleRadians(float current, float target, float alpha)
        {
            float delta = target - current;
            float pi = (float)Math.PI;
            while (delta > pi) delta -= pi * 2.0f;
            while (delta < -pi) delta += pi * 2.0f;
            return current + delta * alpha;
        }

        private static bool TryReadFloatToken(JToken token, out float value)
        {
            value = 0f;
            if (token == null || (token.Type != JTokenType.Integer && token.Type != JTokenType.Float))
                return false;

            value = token.Value<float>();
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        private static float WrappedAngleDeltaRadians(float from, float to)
        {
            float delta = to - from;
            float pi = (float)Math.PI;
            while (delta > pi) delta -= 2f * pi;
            while (delta < -pi) delta += 2f * pi;
            return delta;
        }

        private static float AngleMagnitudeDeg(float roll, float pitch, float yaw)
        {
            return (float)((Math.Abs(roll) + Math.Abs(pitch) + Math.Abs(yaw)) * 180.0 / Math.PI);
        }

        private bool ShouldRejectAttitudeResetGlitch(
            float prevX,
            float prevY,
            float prevZ,
            float prevRoll,
            float prevPitch,
            float prevYaw,
            float nextX,
            float nextY,
            float nextZ,
            float nextRoll,
            float nextPitch,
            float nextYaw)
        {
            float prevMagDeg = AngleMagnitudeDeg(prevRoll, prevPitch, prevYaw);
            float nextMagDeg = AngleMagnitudeDeg(nextRoll, nextPitch, nextYaw);

            float rollJumpDeg = (float)(Math.Abs(WrappedAngleDeltaRadians(prevRoll, nextRoll)) * 180.0 / Math.PI);
            float pitchJumpDeg = (float)(Math.Abs(WrappedAngleDeltaRadians(prevPitch, nextPitch)) * 180.0 / Math.PI);
            float yawJumpDeg = (float)(Math.Abs(WrappedAngleDeltaRadians(prevYaw, nextYaw)) * 180.0 / Math.PI);
            float maxJumpDeg = Math.Max(rollJumpDeg, Math.Max(pitchJumpDeg, yawJumpDeg));

            float dx = nextX - prevX;
            float dy = nextY - prevY;
            float dz = nextZ - prevZ;
            float posDelta = (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);

            return prevMagDeg >= PoseResetPrevMinDeg
                && nextMagDeg <= PoseResetNearZeroDeg
                && maxJumpDeg >= PoseResetJumpThresholdDeg
                && posDelta <= PoseResetMaxPositionDeltaM;
        }

        private void SetupCamera()
        {
            float posX = _renderPosX;
            float posY = _renderPosY;
            float posZ = _renderPosZ;
            float rollRaw = _renderRollRaw;
            float pitchRaw = _renderPitchRaw;
            float yawRaw = _renderYawRaw;
            float servoDeg;
            lock (_poseLock) { servoDeg = _servoAngleDeg; }

            // Convert drone pos to GL frame
            ZedToGL(posX, posY, posZ, out float glX, out float glY, out float glZ);

            // Heading: ZED yaw (rotation around Z-forward) -> GL Y-up rotation
            float headingRad = -yawRaw + MathHelper.DegreesToRadians(_config.SlamHeadingOffsetDeg);

            float eyeX, eyeY, eyeZ, tgtX, tgtY, tgtZ, upX = 0, upY = 1, upZ = 0;

            switch (_currentViewMode)
            {
                case CameraViewMode.FirstPerson:
                {
                    // FPV attitude: heading from yaw, elevation from pitch+servo,
                    // and horizon bank from roll via dynamic camera up-vector.
                    float pitchRad = -pitchRaw;
                    float rollRad = rollRaw;
                    float elevRad = MathHelper.DegreesToRadians(servoDeg - 90f) + pitchRad;

                    float cosH = (float)Math.Cos(headingRad), sinH = (float)Math.Sin(headingRad);
                    float cosE = (float)Math.Cos(elevRad), sinE = (float)Math.Sin(elevRad);
                    float cosR = (float)Math.Cos(rollRad), sinR = (float)Math.Sin(rollRad);

                    eyeX = glX; eyeY = glY; eyeZ = glZ;

                    // Forward direction from heading + elevation.
                    float fwdX = sinH * cosE;
                    float fwdY = sinE;
                    float fwdZ = -cosH * cosE;

                    tgtX = glX + fwdX;
                    tgtY = glY + fwdY;
                    tgtZ = glZ + fwdZ;

                    // Build no-roll camera basis from forward + reference up, then bank around forward axis.
                    float refUpX = 0f, refUpY = 1f, refUpZ = 0f;
                    if (Math.Abs(fwdY) > 0.98f)
                    {
                        // Near-vertical forward; switch reference axis to avoid cross-product degeneracy.
                        refUpX = 1f; refUpY = 0f; refUpZ = 0f;
                    }

                    float rightX = (fwdY * refUpZ) - (fwdZ * refUpY);
                    float rightY = (fwdZ * refUpX) - (fwdX * refUpZ);
                    float rightZ = (fwdX * refUpY) - (fwdY * refUpX);
                    float rightLen = (float)Math.Sqrt(rightX * rightX + rightY * rightY + rightZ * rightZ);
                    if (rightLen > 1e-6f)
                    {
                        rightX /= rightLen;
                        rightY /= rightLen;
                        rightZ /= rightLen;

                        float upNoRollX = rightY * fwdZ - rightZ * fwdY;
                        float upNoRollY = rightZ * fwdX - rightX * fwdZ;
                        float upNoRollZ = rightX * fwdY - rightY * fwdX;

                        float upNoRollLen = (float)Math.Sqrt(upNoRollX * upNoRollX + upNoRollY * upNoRollY + upNoRollZ * upNoRollZ);
                        if (upNoRollLen > 1e-6f)
                        {
                            upNoRollX /= upNoRollLen;
                            upNoRollY /= upNoRollLen;
                            upNoRollZ /= upNoRollLen;
                        }

                        upX = upNoRollX * cosR + rightX * sinR;
                        upY = upNoRollY * cosR + rightY * sinR;
                        upZ = upNoRollZ * cosR + rightZ * sinR;

                        float upLen = (float)Math.Sqrt(upX * upX + upY * upY + upZ * upZ);
                        if (upLen > 1e-6f)
                        {
                            upX /= upLen;
                            upY /= upLen;
                            upZ /= upLen;
                        }
                    }
                    break;
                }
                case CameraViewMode.ThirdPerson:
                {
                    float dist = 3f, height = 1.5f;

                    // Use the same attitude convention as FPV so TPV reflects pitch/roll as well.
                    float pitchRad = -pitchRaw;
                    float rollRad = rollRaw;
                    float cosH = (float)Math.Cos(headingRad), sinH = (float)Math.Sin(headingRad);
                    float cosP = (float)Math.Cos(pitchRad), sinP = (float)Math.Sin(pitchRad);
                    float cosR = (float)Math.Cos(rollRad), sinR = (float)Math.Sin(rollRad);

                    // Drone forward from heading + pitch.
                    float fwdX = sinH * cosP;
                    float fwdY = sinP;
                    float fwdZ = -cosH * cosP;

                    // Build camera basis and bank it with roll.
                    float refUpX = 0f, refUpY = 1f, refUpZ = 0f;
                    if (Math.Abs(fwdY) > 0.98f)
                    {
                        refUpX = 1f; refUpY = 0f; refUpZ = 0f;
                    }

                    float rightX = (fwdY * refUpZ) - (fwdZ * refUpY);
                    float rightY = (fwdZ * refUpX) - (fwdX * refUpZ);
                    float rightZ = (fwdX * refUpY) - (fwdY * refUpX);
                    float rightLen = (float)Math.Sqrt(rightX * rightX + rightY * rightY + rightZ * rightZ);
                    if (rightLen > 1e-6f)
                    {
                        rightX /= rightLen; rightY /= rightLen; rightZ /= rightLen;
                    }

                    float upNoRollX = rightY * fwdZ - rightZ * fwdY;
                    float upNoRollY = rightZ * fwdX - rightX * fwdZ;
                    float upNoRollZ = rightX * fwdY - rightY * fwdX;
                    float upNoRollLen = (float)Math.Sqrt(upNoRollX * upNoRollX + upNoRollY * upNoRollY + upNoRollZ * upNoRollZ);
                    if (upNoRollLen > 1e-6f)
                    {
                        upNoRollX /= upNoRollLen; upNoRollY /= upNoRollLen; upNoRollZ /= upNoRollLen;
                    }

                    upX = upNoRollX * cosR + rightX * sinR;
                    upY = upNoRollY * cosR + rightY * sinR;
                    upZ = upNoRollZ * cosR + rightZ * sinR;

                    // Place camera behind forward direction and above drone in its rolled up direction.
                    eyeX = glX - fwdX * dist + upX * height;
                    eyeY = glY - fwdY * dist + upY * height;
                    eyeZ = glZ - fwdZ * dist + upZ * height;

                    // Keep drone centered while still reflecting its orientation in world motion.
                    tgtX = glX; tgtY = glY; tgtZ = glZ;
                    break;
                }
                default: // FreeOrbit
                {
                    float yawRad = MathHelper.DegreesToRadians(_orbitYaw);
                    float pitchRad2 = MathHelper.DegreesToRadians(_orbitPitch);
                    float cosPitch = (float)Math.Cos(pitchRad2);
                    eyeX = _orbitCenterX + (float)Math.Sin(yawRad) * cosPitch * _orbitDistance;
                    eyeY = _orbitCenterY + (float)Math.Sin(pitchRad2) * _orbitDistance;
                    eyeZ = _orbitCenterZ + (float)Math.Cos(yawRad) * cosPitch * _orbitDistance;
                    tgtX = _orbitCenterX; tgtY = _orbitCenterY; tgtZ = _orbitCenterZ;
                    break;
                }
            }

            _camPosX = eyeX; _camPosY = eyeY; _camPosZ = eyeZ;
            _camTgtX = tgtX; _camTgtY = tgtY; _camTgtZ = tgtZ;

            var view = Matrix4.LookAt(eyeX, eyeY, eyeZ, tgtX, tgtY, tgtZ, upX, upY, upZ);
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
            if (_currentViewMode != CameraViewMode.FreeOrbit) return;
            float dx = e.X - _lastMousePos.X;
            float dy = e.Y - _lastMousePos.Y;
            _lastMousePos = e.Location;

            if (_mouseRotating)
            {
                _orbitYaw += dx * 0.3f;
                _orbitPitch = Math.Max(-89f, Math.Min(89f, _orbitPitch + dy * 0.3f));
            }
            if (_mousePanning)
            {
                float scale = _orbitDistance * 0.002f;
                // Compute camera right and up from current view
                float fwdX = _camTgtX - _camPosX, fwdY = _camTgtY - _camPosY, fwdZ = _camTgtZ - _camPosZ;
                float fwdLen = (float)Math.Sqrt(fwdX * fwdX + fwdY * fwdY + fwdZ * fwdZ);
                if (fwdLen > 0.001f)
                {
                    fwdX /= fwdLen; fwdY /= fwdLen; fwdZ /= fwdLen;
                    // Right = forward x up(0,1,0)
                    float rx = fwdY * 0 - fwdZ * 1, ry = fwdZ * 0 - fwdX * 0, rz = fwdX * 1 - fwdY * 0;
                    // actually: right = cross(forward, (0,1,0))
                    rx = fwdZ; ry = 0; rz = -fwdX; // simplified cross for up=(0,1,0)
                    float rLen = (float)Math.Sqrt(rx * rx + rz * rz);
                    if (rLen > 0.001f) { rx /= rLen; rz /= rLen; }
                    // Up = cross(right, forward)
                    float ux = ry * fwdZ - rz * fwdY;
                    float uy = rz * fwdX - rx * fwdZ;
                    float uz = rx * fwdY - ry * fwdX;

                    _orbitCenterX -= rx * dx * scale + ux * dy * scale;
                    _orbitCenterY -= ry * dx * scale + uy * dy * scale;
                    _orbitCenterZ -= rz * dx * scale + uz * dy * scale;
                }
            }
        }

        private void GlControl_MouseWheel(object sender, MouseEventArgs e)
        {
            if (_currentViewMode != CameraViewMode.FreeOrbit) return;
            _orbitDistance = Math.Max(1f, Math.Min(100f, _orbitDistance - e.Delta * 0.01f));
        }

        // ==================== Drawing: Grid ====================

        private void DrawGrid()
        {
            GL.Disable(EnableCap.Lighting);

            // Ground grid at Y=0
            GL.Color4(0.3f, 0.3f, 0.3f, 0.4f);
            GL.Begin(PrimitiveType.Lines);
            float size = 10f;
            int lines = 20;
            float spacing = size * 2f / lines;
            for (int i = 0; i <= lines; i++)
            {
                float pos = -size + i * spacing;
                GL.Vertex3(-size, 0, pos); GL.Vertex3(size, 0, pos);
                GL.Vertex3(pos, 0, -size); GL.Vertex3(pos, 0, size);
            }
            GL.End();

            // Axis indicators
            GL.LineWidth(2f);
            GL.Begin(PrimitiveType.Lines);
            GL.Color3(1f, 0.2f, 0.2f); GL.Vertex3(0, 0, 0); GL.Vertex3(1, 0, 0); // X red
            GL.Color3(0.2f, 1f, 0.2f); GL.Vertex3(0, 0, 0); GL.Vertex3(0, 1, 0); // Y green
            GL.Color3(0.2f, 0.2f, 1f); GL.Vertex3(0, 0, 0); GL.Vertex3(0, 0, 1); // Z blue
            GL.End();
            GL.LineWidth(1f);

            GL.Enable(EnableCap.Lighting);
        }

        // ==================== Drawing: Voxels ====================

        private void DrawVoxels()
        {
            if (_voxelVerts == null || _voxelIndexCount == 0) return;

            GL.Enable(EnableCap.Lighting);
            GL.EnableClientState(ArrayCap.VertexArray);
            GL.EnableClientState(ArrayCap.ColorArray);
            GL.EnableClientState(ArrayCap.NormalArray);

            int stride = 9 * sizeof(float);
            var vertHandle = GCHandle.Alloc(_voxelVerts, GCHandleType.Pinned);
            var idxHandle = GCHandle.Alloc(_voxelIndices, GCHandleType.Pinned);
            try
            {
                IntPtr basePtr = vertHandle.AddrOfPinnedObject();
                GL.VertexPointer(3, VertexPointerType.Float, stride, basePtr);
                GL.ColorPointer(3, ColorPointerType.Float, stride, basePtr + 3 * sizeof(float));
                GL.NormalPointer(NormalPointerType.Float, stride, basePtr + 6 * sizeof(float));
                GL.DrawElements(PrimitiveType.Triangles, _voxelIndexCount,
                    DrawElementsType.UnsignedInt, idxHandle.AddrOfPinnedObject());
            }
            finally
            {
                idxHandle.Free();
                vertHandle.Free();
            }

            GL.DisableClientState(ArrayCap.NormalArray);
            GL.DisableClientState(ArrayCap.ColorArray);
            GL.DisableClientState(ArrayCap.VertexArray);
        }

        // ==================== Drawing: Drone (Tricopter) ====================

        private void DrawDrone()
        {
            if (_currentViewMode == CameraViewMode.FirstPerson) return;

            float posX = _renderPosX;
            float posY = _renderPosY;
            float posZ = _renderPosZ;
            float rollRaw = _renderRollRaw;
            float pitchRaw = _renderPitchRaw;
            float yawRaw = _renderYawRaw;
            float servoDeg;
            lock (_poseLock) { servoDeg = _servoAngleDeg; }

            ZedToGL(posX, posY, posZ, out float glX, out float glY, out float glZ);

            // Drone body orientation from camera pose
            // CORRECTED: ZED odom convention is standard aviation Euler (ZYX intrinsic):
            //   roll (X-rotation) = rotation about forward axis (nose roll)
            //   pitch (Y-rotation) = rotation about lateral axis (nose up/down)
            //   yaw (Z-rotation) = rotation about vertical axis (heading)
            // After position frame conversion (x,y,z)->(x,-y,-z), apply rotations correctly:
            // Model geometry alignment: reverse yaw direction and add 180° so model nose matches true heading.
            float headingDeg = (float)(yawRaw * 180.0 / Math.PI) + _config.SlamHeadingOffsetDeg + 180f;
            float bodyPitchDeg = (float)(pitchRaw * 180.0 / Math.PI);
            float bodyRollDeg = (float)(rollRaw * 180.0 / Math.PI);
            // NOTE: Servo angle affects camera gimbal only, applied separately below

            float lenM = _config.DroneLengthCm / 100f;
            float widM = _config.DroneWidthCm / 100f;
            float hgtM = _config.DroneHeightCm / 100f;

            GL.Disable(EnableCap.Lighting);
            GL.PushMatrix();
            GL.Translate(glX, glY, glZ);
            GL.Rotate(headingDeg, 0f, 1f, 0f);
            GL.Rotate(bodyPitchDeg, 1f, 0f, 0f);
            GL.Rotate(bodyRollDeg, 0f, 0f, 1f);

            // Draw drone body based on selected frame type
            if (_config.DroneFrameType == "Quadcopter")
            {
                DrawQuadropterBody(lenM, widM, hgtM);
            }
            else
            {
                DrawTricopterBody(lenM, widM, hgtM);
            }

            // --- Camera servo mount ---
            float camFwd = _config.CameraForwardOffsetCm / 100f;
            float camDown = _config.CameraDownOffsetCm / 100f;

            GL.PushMatrix();
            GL.Translate(0, -camDown, camFwd);
            // Servo tilts camera: rotation around X (lateral axis in body frame)
            // servo=90 is level, >90 tilts up
            GL.Rotate(90f - servoDeg, 1f, 0f, 0f);

            // Camera box (yellow)
            GL.Color3(0.9f, 0.85f, 0.2f);
            float camW = 0.04f, camH = 0.025f, camD = 0.03f;
            DrawWireBox(camW * 0.3f, 0, 0, camW, camH, camD);

            // Camera look direction (green)
            GL.Color3(0.2f, 1f, 0.2f);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0);
            GL.Vertex3(0, 0, 0.15f);
            GL.End();

            GL.PopMatrix(); // camera mount

            // --- Avoidance envelope ---
            float envPad = 0.1f;
            GL.Color4(0f, 0.86f, 0.86f, 0.2f);
            DrawWireBox(0, 0, 0, lenM + envPad * 2, hgtM + envPad * 2, widM + envPad * 2);

            GL.Enable(EnableCap.Lighting);
            GL.PopMatrix(); // drone body
        }

        private void DrawTricopterBody(float lenM, float widM, float hgtM)
        {
            // --- Tricopter body: 2 front motors, 1 rear motor ---
            GL.Color3(0f, 0.86f, 0.86f); // cyan

            // Central body wireframe
            DrawWireBox(0, 0, 0, widM * 0.3f, hgtM, lenM * 0.4f);

            // Front-left arm
            float frontX = widM * 0.4f;
            float frontZ = lenM * 0.35f;
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0); GL.Vertex3(-frontX, 0, frontZ);
            GL.End();

            // Front-right arm
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0); GL.Vertex3(frontX, 0, frontZ);
            GL.End();

            // Rear arm
            float rearZ = -lenM * 0.4f;
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0); GL.Vertex3(0, 0, rearZ);
            GL.End();

            // Motor discs
            float motorR = Math.Min(lenM, widM) * 0.15f;
            DrawMotorDisc(frontX, 0, frontZ, motorR);
            DrawMotorDisc(-frontX, 0, frontZ, motorR);
            DrawMotorDisc(0, 0, rearZ, motorR);

            // Forward direction indicator (red) - points along +Z (forward in body frame)
            GL.Color3(1f, 0.3f, 0.3f);
            GL.LineWidth(2f);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0);
            GL.Vertex3(0, 0, lenM * 0.6f);
            GL.End();
            GL.LineWidth(1f);
        }

        private void DrawQuadropterBody(float lenM, float widM, float hgtM)
        {
            // --- Quadcopter body: 4 motors in X configuration ---
            GL.Color3(1f, 0.65f, 0f); // orange

            // Central body wireframe
            DrawWireBox(0, 0, 0, widM * 0.3f, hgtM, lenM * 0.3f);

            // Front-left arm
            float armAngle = 45f * (float)Math.PI / 180f;
            float frontLeftX = -widM * 0.4f * (float)Math.Cos(armAngle);
            float frontLeftZ = lenM * 0.35f * (float)Math.Sin(armAngle);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0); GL.Vertex3(frontLeftX, 0, frontLeftZ);
            GL.End();

            // Front-right arm
            float frontRightX = widM * 0.4f * (float)Math.Cos(armAngle);
            float frontRightZ = lenM * 0.35f * (float)Math.Sin(armAngle);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0); GL.Vertex3(frontRightX, 0, frontRightZ);
            GL.End();

            // Rear-left arm
            float rearLeftX = -widM * 0.4f * (float)Math.Cos(armAngle);
            float rearLeftZ = -lenM * 0.35f * (float)Math.Sin(armAngle);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0); GL.Vertex3(rearLeftX, 0, rearLeftZ);
            GL.End();

            // Rear-right arm
            float rearRightX = widM * 0.4f * (float)Math.Cos(armAngle);
            float rearRightZ = -lenM * 0.35f * (float)Math.Sin(armAngle);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0); GL.Vertex3(rearRightX, 0, rearRightZ);
            GL.End();

            // Motor discs (4 motors)
            float motorR = Math.Min(lenM, widM) * 0.15f;
            DrawMotorDisc(frontLeftX, 0, frontLeftZ, motorR);
            DrawMotorDisc(frontRightX, 0, frontRightZ, motorR);
            DrawMotorDisc(rearLeftX, 0, rearLeftZ, motorR);
            DrawMotorDisc(rearRightX, 0, rearRightZ, motorR);

            // Forward direction indicator (red) - points along +Z (forward in body frame)
            GL.Color3(1f, 0.3f, 0.3f);
            GL.LineWidth(2f);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0);
            GL.Vertex3(0, 0, lenM * 0.6f);
            GL.End();
            GL.LineWidth(1f);
        }

        private static void DrawWireBox(float cx, float cy, float cz, float sx, float sy, float sz)
        {
            float hx = sx / 2, hy = sy / 2, hz = sz / 2;
            GL.Begin(PrimitiveType.Lines);
            for (int i = 0; i < 2; i++)
            {
                float y = (i == 0) ? cy - hy : cy + hy;
                GL.Vertex3(cx - hx, y, cz - hz); GL.Vertex3(cx + hx, y, cz - hz);
                GL.Vertex3(cx + hx, y, cz - hz); GL.Vertex3(cx + hx, y, cz + hz);
                GL.Vertex3(cx + hx, y, cz + hz); GL.Vertex3(cx - hx, y, cz + hz);
                GL.Vertex3(cx - hx, y, cz + hz); GL.Vertex3(cx - hx, y, cz - hz);
            }
            GL.Vertex3(cx - hx, cy - hy, cz - hz); GL.Vertex3(cx - hx, cy + hy, cz - hz);
            GL.Vertex3(cx + hx, cy - hy, cz - hz); GL.Vertex3(cx + hx, cy + hy, cz - hz);
            GL.Vertex3(cx + hx, cy - hy, cz + hz); GL.Vertex3(cx + hx, cy + hy, cz + hz);
            GL.Vertex3(cx - hx, cy - hy, cz + hz); GL.Vertex3(cx - hx, cy + hy, cz + hz);
            GL.End();
        }

        private static void DrawMotorDisc(float cx, float cy, float cz, float radius)
        {
            GL.Begin(PrimitiveType.LineLoop);
            for (int i = 0; i < 12; i++)
            {
                float angle = (float)(i * 2 * Math.PI / 12);
                GL.Vertex3(cx + radius * (float)Math.Cos(angle), cy,
                           cz + radius * (float)Math.Sin(angle));
            }
            GL.End();
        }

        // ==================== Drawing: Trajectory ====================

        private void DrawTrajectory()
        {
            List<float[]> points;
            lock (_trajectoryLock)
            {
                if (_trajectoryPoints.Count < 2) return;
                points = new List<float[]>(_trajectoryPoints);
            }

            GL.Disable(EnableCap.Lighting);
            GL.Color3(1f, 0.78f, 0f); // gold
            GL.LineWidth(2f);
            GL.Begin(PrimitiveType.LineStrip);
            foreach (var pt in points)
            {
                ZedToGL(pt[0], pt[1], pt[2], out float gx, out float gy, out float gz);
                GL.Vertex3(gx, gy, gz);
            }
            GL.End();
            GL.LineWidth(1f);
            GL.Enable(EnableCap.Lighting);
        }

        // ==================== Drawing: Detection Markers ====================

        private void DrawDetectionMarkers()
        {
            List<DetectionMarker3D> markers;
            lock (_poseLock) { markers = _detectionMarkers; }
            if (markers == null || markers.Count == 0) return;

            GL.Disable(EnableCap.Lighting);
            foreach (var det in markers)
            {
                ZedToGL((float)det.X, (float)det.Y, (float)det.Z, out float gx, out float gy, out float gz);
                GetDetectionColorRGB(det.Label, out float cr, out float cg, out float cb);
                GL.Color3(cr, cg, cb);

                float conf = (float)Math.Max(0, Math.Min(1, det.Confidence));
                float radius = 0.03f + conf * 0.05f;
                DrawSphere(gx, gy, gz, radius, 8);

                // Vertical reference line to ground
                GL.Color4(cr, cg, cb, 0.3f);
                GL.Begin(PrimitiveType.Lines);
                GL.Vertex3(gx, 0, gz);
                GL.Vertex3(gx, gy, gz);
                GL.End();
            }
            GL.Enable(EnableCap.Lighting);
        }

        private static void GetDetectionColorRGB(string label, out float r, out float g, out float b)
        {
            r = 0.8f; g = 0.4f; b = 0.8f; // default purple
            if (string.IsNullOrEmpty(label)) { r = 1; g = 1; b = 1; return; }
            string lower = label.ToLowerInvariant();
            if (lower.Contains("red")) { r = 0.86f; g = 0.16f; b = 0.16f; }
            else if (lower.Contains("blue")) { r = 0.16f; g = 0.4f; b = 0.86f; }
            else if (lower.Contains("green")) { r = 0.16f; g = 0.78f; b = 0.16f; }
            else if (lower.Contains("yellow")) { r = 0.94f; g = 0.86f; b = 0.16f; }
            else if (lower.Contains("black")) { r = 0.2f; g = 0.2f; b = 0.2f; }
            else if (lower.Contains("white")) { r = 0.94f; g = 0.94f; b = 0.94f; }
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

        private static void DrawSphere(float cx, float cy, float cz, float radius, int segments)
        {
            for (int lat = 0; lat < segments; lat++)
            {
                float theta1 = (float)(lat * Math.PI / segments);
                float theta2 = (float)((lat + 1) * Math.PI / segments);

                GL.Begin(PrimitiveType.TriangleStrip);
                for (int lon = 0; lon <= segments; lon++)
                {
                    float phi = (float)(lon * 2 * Math.PI / segments);
                    float cp = (float)Math.Cos(phi), sp = (float)Math.Sin(phi);

                    float x1 = radius * (float)Math.Sin(theta1) * cp;
                    float y1 = radius * (float)Math.Cos(theta1);
                    float z1 = radius * (float)Math.Sin(theta1) * sp;

                    float x2 = radius * (float)Math.Sin(theta2) * cp;
                    float y2 = radius * (float)Math.Cos(theta2);
                    float z2 = radius * (float)Math.Sin(theta2) * sp;

                    GL.Normal3(x1 / radius, y1 / radius, z1 / radius);
                    GL.Vertex3(cx + x1, cy + y1, cz + z1);
                    GL.Normal3(x2 / radius, y2 / radius, z2 / radius);
                    GL.Vertex3(cx + x2, cy + y2, cz + z2);
                }
                GL.End();
            }
        }

        // ==================== Mesh Building ====================

        /// <summary>
        /// Rebuild the voxel vertex arrays from persisted data.
        /// Uses adjacent-face culling and full 8-bit per-vertex color.
        /// </summary>
        private void RebuildVoxelMesh()
        {
            _lastMeshRebuild = DateTime.UtcNow;
            _lastMeshRebuildStamp = Stopwatch.GetTimestamp();
            _meshDirty = false;
            _lastRenderedCount = _persistedBlocks.Count;

            double vs = _currentVoxelSize;
            float half = (float)(vs * 0.5);

            // Pre-allocate: worst case 6 faces * 4 verts * 9 floats = 216 floats/voxel
            var verts = new List<float>(_persistedBlocks.Count * 100);
            var indices = new List<int>(_persistedBlocks.Count * 36);
            int vertOffset = 0;

            foreach (var kvp in _persistedBlocks)
            {
                UnpackVoxelKey(kvp.Key, out int ix, out int iy, out int iz);
                float cx = (float)(ix * vs);
                float cy = (float)(iy * vs);
                float cz = (float)(iz * vs);

                // Full 8-bit color
                uint ck = kvp.Value;
                float cr, cg, cb;
                if (ck == uint.MaxValue) { cr = 0.56f; cg = 0.56f; cb = 0.63f; }
                else
                {
                    cr = ((ck >> 16) & 0xFF) / 255f;
                    cg = ((ck >> 8) & 0xFF) / 255f;
                    cb = (ck & 0xFF) / 255f;
                }

                // +X face
                if (!_occupancySet.Contains(PackKey(ix + 1, iy, iz)))
                    AddQuad(verts, indices, ref vertOffset,
                        cx + half, cy - half, cz - half,
                        cx + half, cy + half, cz - half,
                        cx + half, cy + half, cz + half,
                        cx + half, cy - half, cz + half,
                        cr, cg, cb, 1, 0, 0);
                // -X face
                if (!_occupancySet.Contains(PackKey(ix - 1, iy, iz)))
                    AddQuad(verts, indices, ref vertOffset,
                        cx - half, cy - half, cz + half,
                        cx - half, cy + half, cz + half,
                        cx - half, cy + half, cz - half,
                        cx - half, cy - half, cz - half,
                        cr, cg, cb, -1, 0, 0);
                // +Y face
                if (!_occupancySet.Contains(PackKey(ix, iy + 1, iz)))
                    AddQuad(verts, indices, ref vertOffset,
                        cx - half, cy + half, cz - half,
                        cx - half, cy + half, cz + half,
                        cx + half, cy + half, cz + half,
                        cx + half, cy + half, cz - half,
                        cr, cg, cb, 0, 1, 0);
                // -Y face
                if (!_occupancySet.Contains(PackKey(ix, iy - 1, iz)))
                    AddQuad(verts, indices, ref vertOffset,
                        cx - half, cy - half, cz + half,
                        cx - half, cy - half, cz - half,
                        cx + half, cy - half, cz - half,
                        cx + half, cy - half, cz + half,
                        cr, cg, cb, 0, -1, 0);
                // +Z face
                if (!_occupancySet.Contains(PackKey(ix, iy, iz + 1)))
                    AddQuad(verts, indices, ref vertOffset,
                        cx - half, cy - half, cz + half,
                        cx + half, cy - half, cz + half,
                        cx + half, cy + half, cz + half,
                        cx - half, cy + half, cz + half,
                        cr, cg, cb, 0, 0, 1);
                // -Z face
                if (!_occupancySet.Contains(PackKey(ix, iy, iz - 1)))
                    AddQuad(verts, indices, ref vertOffset,
                        cx + half, cy - half, cz - half,
                        cx - half, cy - half, cz - half,
                        cx - half, cy + half, cz - half,
                        cx + half, cy + half, cz - half,
                        cr, cg, cb, 0, 0, -1);
            }

            // Atomic swap: the render thread reads these arrays
            _voxelVerts = verts.ToArray();
            _voxelIndices = indices.ToArray();
            _voxelIndexCount = indices.Count;
            LogMeshDebounce($"REBUILD complete: cached={_persistedBlocks.Count} verts={_voxelVerts.Length} indices={_voxelIndexCount} at={_lastMeshRebuild:O}");
        }

        private static void AddQuad(List<float> verts, List<int> indices, ref int offset,
            float x0, float y0, float z0, float x1, float y1, float z1,
            float x2, float y2, float z2, float x3, float y3, float z3,
            float r, float g, float b, float nx, float ny, float nz)
        {
            // 4 vertices: pos(3) + color(3) + normal(3)
            verts.Add(x0); verts.Add(y0); verts.Add(z0); verts.Add(r); verts.Add(g); verts.Add(b); verts.Add(nx); verts.Add(ny); verts.Add(nz);
            verts.Add(x1); verts.Add(y1); verts.Add(z1); verts.Add(r); verts.Add(g); verts.Add(b); verts.Add(nx); verts.Add(ny); verts.Add(nz);
            verts.Add(x2); verts.Add(y2); verts.Add(z2); verts.Add(r); verts.Add(g); verts.Add(b); verts.Add(nx); verts.Add(ny); verts.Add(nz);
            verts.Add(x3); verts.Add(y3); verts.Add(z3); verts.Add(r); verts.Add(g); verts.Add(b); verts.Add(nx); verts.Add(ny); verts.Add(nz);
            // 2 triangles
            indices.Add(offset); indices.Add(offset + 1); indices.Add(offset + 2);
            indices.Add(offset); indices.Add(offset + 2); indices.Add(offset + 3);
            offset += 4;
        }

        // ==================== Mesh Data Processing ====================

        private void LogMeshDebounce(string message)
        {
            if (!EnableMeshDebounceDebugLog) return;
            Debug.WriteLine($"[SLAM3D][P3-7] {message}");
        }

        private static long ElapsedMsSince(long startStamp)
        {
            if (startStamp < 0) return long.MaxValue;
            long elapsedTicks = Stopwatch.GetTimestamp() - startStamp;
            return (elapsedTicks * 1000) / Stopwatch.Frequency;
        }

        // P3-7: Process pending mesh updates after debounce window expires
        private void ProcessPendingMeshUpdate()
        {
            lock (_meshLock)
            {
                if (!_pendingMeshUpdate) return;

                if (!_meshDirty)
                {
                    _pendingMeshUpdate = false;
                    return;
                }

                long elapsedMs = ElapsedMsSince(_lastMeshRebuildStamp);
                long debounceMs = (long)MinRebuildInterval.TotalMilliseconds;

                if (elapsedMs >= debounceMs)
                {
                    // Debounce window has expired and updates are pending: rebuild now
                    LogMeshDebounce($"ALLOW pending rebuild: elapsed={elapsedMs}ms dirty={_meshDirty} pending={_pendingMeshUpdate} voxels={_persistedBlocks.Count}");
                    FillGaps();
                    RebuildVoxelMesh();
                    _pendingMeshUpdate = false;
                }
                else
                {
                    LogMeshDebounce($"BLOCK pending rebuild: elapsed={elapsedMs}ms < {debounceMs}ms dirty={_meshDirty} pending={_pendingMeshUpdate}");
                }
            }
        }

        private void UpdateMeshVisual(MeshDataModel meshData)
        {
            try
            {
                lock (_meshLock)
                {
                    if (meshData?.Clear == true)
                    {
                        _persistedBlocks.Clear();
                        ClearEvictionTracking();
                        _occupancySet.Clear();
                        _voxelLastSeen.Clear();
                        _voxelVerts = null;
                        _voxelIndices = null;
                        _voxelIndexCount = 0;
                        _meshDirty = false;
                        _pendingMeshUpdate = false;
                        _lastRenderedCount = 0;
                    }

                    // Handle removals (voxel/block mode only)
                    if (meshData?.Removed != null)
                    {
                        foreach (var r in meshData.Removed)
                        {
                            int rx = -r.Y, ry = r.Z, rz = -r.X;
                            var key = PackVoxelKey(rx, ry, rz);
                            if (_persistedBlocks.Remove(key))
                            {
                                UnqueueForEviction(key);
                                _occupancySet.Remove(PackKey(rx, ry, rz));
                                _meshDirty = true;
                            }
                        }
                    }

                    // Voxel/block cube mode
                    if ((meshData?.Mode == "voxel" || meshData?.Mode == "voxels") &&
                        meshData.Voxels != null && meshData.Voxels.Count > 0)
                    {
                        ProcessVoxels(meshData);
                    }
                    else if (meshData?.Blocks != null && meshData.Blocks.Count > 0)
                    {
                        ProcessBlocks(meshData);
                    }

                    if (_meshDirty)
                    {
                        // Queue rebuild work; timing gate is enforced in ProcessPendingMeshUpdate().
                        _pendingMeshUpdate = true;
                        LogMeshDebounce($"QUEUE mesh update: dirty=true pending=true mode={meshData?.Mode ?? "?"} voxels={meshData?.Voxels?.Count ?? 0} blocks={meshData?.Blocks?.Count ?? 0}");
                    }
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Mesh update error: {ex.Message}");
            }
        }

        private void ProcessVoxels(MeshDataModel meshData)
        {
            double vs = meshData.VoxelSize > 0 ? meshData.VoxelSize : 0.15;

            // Clear persisted data when switching from block→voxel mode
            // (block indices are in a different coordinate space)
            if (_currentMeshMode != "voxel")
            {
                _persistedBlocks.Clear();
                ClearEvictionTracking();
                _occupancySet.Clear();
                _voxelLastSeen.Clear();
                _currentMeshMode = "voxel";
            }
            _currentVoxelSize = vs;
            _meshGeneration++;

            foreach (var voxel in meshData.Voxels)
            {
                if (voxel.Position == null || voxel.Position.Count < 3) continue;

                int qx = (int)Math.Floor(-voxel.Position[1] / vs + 0.5);
                int qy = (int)Math.Floor(voxel.Position[2] / vs + 0.5);
                int qz = (int)Math.Floor(-voxel.Position[0] / vs + 0.5);
                long key = PackVoxelKey(qx, qy, qz);

                // Full 8-bit color (no quantization!)
                uint colorKey;
                if (voxel.Color != null && voxel.Color.Count >= 3)
                {
                    byte r = (byte)Math.Min(255, Math.Max(0, voxel.Color[0]));
                    byte g = (byte)Math.Min(255, Math.Max(0, voxel.Color[1]));
                    byte b = (byte)Math.Min(255, Math.Max(0, voxel.Color[2]));
                    colorKey = ((uint)r << 16) | ((uint)g << 8) | (uint)b;
                }
                else
                    colorKey = uint.MaxValue;

                _voxelLastSeen[key] = _meshGeneration;

                if (!_persistedBlocks.ContainsKey(key) || _persistedBlocks[key] != colorKey)
                {
                    if (!_persistedBlocks.ContainsKey(key))
                        QueueForEviction(key);
                    _persistedBlocks[key] = colorKey;
                    _occupancySet.Add(PackKey(qx, qy, qz));
                    _meshDirty = true;
                }
            }

            // Expire voxels not seen in recent updates
            ExpireOldVoxels();
        }

        private void ProcessBlocks(MeshDataModel meshData)
        {
            double bs = meshData.BlockSize > 0 ? meshData.BlockSize : 0.05;

            // Clear persisted data when switching from voxel→block mode
            if (_currentMeshMode != "block")
            {
                _persistedBlocks.Clear();
                ClearEvictionTracking();
                _occupancySet.Clear();
                _voxelLastSeen.Clear();
                _currentMeshMode = "block";
            }
            _currentVoxelSize = bs;
            _meshGeneration++;

            foreach (var block in meshData.Blocks)
            {
                if (block.Index == null || block.Index.Count < 3) continue;

                int bix = -block.Index[1];
                int biy = block.Index[2];
                int biz = -block.Index[0];
                long key = PackVoxelKey(bix, biy, biz);

                uint colorKey;
                if (block.Color != null && block.Color.Count >= 3)
                {
                    byte r = (byte)Math.Min(255, Math.Max(0, block.Color[0]));
                    byte g = (byte)Math.Min(255, Math.Max(0, block.Color[1]));
                    byte b = (byte)Math.Min(255, Math.Max(0, block.Color[2]));
                    colorKey = ((uint)r << 16) | ((uint)g << 8) | (uint)b;
                }
                else
                    colorKey = uint.MaxValue;

                if (!_persistedBlocks.ContainsKey(key) || _persistedBlocks[key] != colorKey)
                {
                    if (!_persistedBlocks.ContainsKey(key))
                        QueueForEviction(key);
                    _persistedBlocks[key] = colorKey;
                    _occupancySet.Add(PackKey(bix, biy, biz));
                    _meshDirty = true;
                }
            }

            EvictOldVoxels();
        }

        private void EvictOldVoxels()
        {
            while (_persistedBlocks.Count > MaxPersistedVoxels && _voxelInsertionOrder.Count > 0)
            {
                long key = _voxelInsertionOrder.Dequeue();
                _queuedForEviction.Remove(key);
                if (_persistedBlocks.Remove(key))
                {
                    _voxelLastSeen.Remove(key);
                    UnpackVoxelKey(key, out int ix, out int iy, out int iz);
                    _occupancySet.Remove(PackKey(ix, iy, iz));
                    _meshDirty = true;
                }
            }
        }

        /// <summary>
        /// Fill empty cells that have occupied neighbors on opposite sides
        /// using the mean color. Closes nvblox block-boundary gaps.
        /// </summary>
        private void FillGaps()
        {
            var fills = new Dictionary<long, uint>();

            foreach (var kvp in _persistedBlocks)
            {
                UnpackVoxelKey(kvp.Key, out int ix, out int iy, out int iz);

                // For each axis, check if the +1 neighbor is empty but +2 is occupied
                CheckAndFill(fills, ix, iy, iz, 1, 0, 0, kvp.Value);
                CheckAndFill(fills, ix, iy, iz, 0, 1, 0, kvp.Value);
                CheckAndFill(fills, ix, iy, iz, 0, 0, 1, kvp.Value);
            }

            foreach (var kvp in fills)
            {
                if (!_persistedBlocks.ContainsKey(kvp.Key))
                    QueueForEviction(kvp.Key);
                _persistedBlocks[kvp.Key] = kvp.Value;
                UnpackVoxelKey(kvp.Key, out int fx, out int fy, out int fz);
                _occupancySet.Add(PackKey(fx, fy, fz));
            }

            // Gap filling can add many synthetic cells in block mode.
            // Re-apply the memory cap after fills are inserted.
            if (_currentMeshMode == "block")
                EvictOldVoxels();
        }

        private void CheckAndFill(Dictionary<long, uint> fills, int ix, int iy, int iz,
            int dx, int dy, int dz, uint c1)
        {
            int nx = ix + dx, ny = iy + dy, nz = iz + dz;
            long nk = PackVoxelKey(nx, ny, nz);
            if (_persistedBlocks.ContainsKey(nk) || fills.ContainsKey(nk)) return;

            int fx = ix + dx * 2, fy = iy + dy * 2, fz = iz + dz * 2;
            long fk = PackVoxelKey(fx, fy, fz);
            uint c2;
            if (!_persistedBlocks.TryGetValue(fk, out c2)) return;

            // Mean color of both sides
            if (c1 == uint.MaxValue) c1 = c2;
            if (c2 == uint.MaxValue) c2 = c1;
            uint r = (((c1 >> 16) & 0xFF) + ((c2 >> 16) & 0xFF)) / 2;
            uint g = (((c1 >> 8) & 0xFF) + ((c2 >> 8) & 0xFF)) / 2;
            uint b = ((c1 & 0xFF) + (c2 & 0xFF)) / 2;
            fills[nk] = (r << 16) | (g << 8) | b;
        }

        /// <summary>
        /// Remove voxels that haven't been seen by nvblox in recent updates.
        /// This handles obstacles that move or disappear.
        /// </summary>
        private void ExpireOldVoxels()
        {
            int cutoff = _meshGeneration - VoxelMaxAge;
            if (cutoff < 0) return;

            var expired = new List<long>();
            foreach (var kvp in _voxelLastSeen)
            {
                if (kvp.Value < cutoff)
                    expired.Add(kvp.Key);
            }

            bool compactEvictionQueue = false;

            foreach (var key in expired)
            {
                _voxelLastSeen.Remove(key);
                if (_persistedBlocks.Remove(key))
                {
                    if (_queuedForEviction.Remove(key))
                        compactEvictionQueue = true;
                    UnpackVoxelKey(key, out int ix, out int iy, out int iz);
                    _occupancySet.Remove(PackKey(ix, iy, iz));
                    _meshDirty = true;
                }
            }

            if (compactEvictionQueue && _voxelInsertionOrder.Count > 0)
            {
                var rebuilt = new Queue<long>(_voxelInsertionOrder.Count);
                while (_voxelInsertionOrder.Count > 0)
                {
                    long queued = _voxelInsertionOrder.Dequeue();
                    if (_queuedForEviction.Contains(queued))
                        rebuilt.Enqueue(queued);
                }
                _voxelInsertionOrder = rebuilt;
            }
        }

        // ==================== WebSocket Stream ====================

        private void StartUpdateLoop()
        {
            _updateCts = new CancellationTokenSource();
            Task.Run(() => WebSocketStreamLoop(_updateCts.Token));
        }

        private async Task WebSocketStreamLoop(CancellationToken ct)
        {
            while (!ct.IsCancellationRequested)
            {
                try
                {
                    _webSocket = new ClientWebSocket();
                    _webSocket.Options.KeepAliveInterval = TimeSpan.FromSeconds(5);

                    string baseUrl = JetsonApiService.BaseUrl ?? "http://100.85.121.98:8000";
                    string wsUrl = baseUrl.Replace("https://", "wss://").Replace("http://", "ws://")
                        .TrimEnd('/') + "/ws/slam";
                    if (JetsonApiService.ApiKey != null)
                        wsUrl += $"?token={Uri.EscapeDataString(JetsonApiService.ApiKey)}";

                    UpdateStatusSafe("Connecting...");
                    await _webSocket.ConnectAsync(new Uri(wsUrl), ct);
                    UpdateStatusSafe("Status: Connected (30Hz)");
                    _wsReconnectDelayMs = 1000;
                    _ = RefreshMeshModeFromServerAsync(updateStatus: false);

                    var buffer = new byte[64 * 1024];
                    var messageBuffer = new MemoryStream();

                    while (_webSocket.State == WebSocketState.Open && !ct.IsCancellationRequested)
                    {
                        messageBuffer.SetLength(0);
                        WebSocketReceiveResult result = null;
                        bool timedOut = false, oversized = false;

                        do
                        {
                            var receiveTask = _webSocket.ReceiveAsync(new ArraySegment<byte>(buffer), ct);
                            var timeoutTask = Task.Delay(30000, ct);
                            var completed = await Task.WhenAny(receiveTask, timeoutTask);
                            if (completed == timeoutTask) { timedOut = true; break; }

                            result = await receiveTask;
                            if (result.MessageType == WebSocketMessageType.Close) break;

                            if (!oversized)
                            {
                                messageBuffer.Write(buffer, 0, result.Count);
                                if (messageBuffer.Length > MaxWebSocketMessageSize) oversized = true;
                            }
                        } while (result != null && !result.EndOfMessage);

                        if (timedOut) break;
                        if (result == null || result.MessageType == WebSocketMessageType.Close) break;
                        if (oversized || !_autoUpdateEnabled) continue;

                        string json = Encoding.UTF8.GetString(messageBuffer.GetBuffer(), 0, (int)messageBuffer.Length);
                        var frame = JObject.Parse(json);
                        string frameType = frame["type"]?.ToString() ?? "pose";
                        bool isMeshFrame = string.Equals(frameType, "mesh", StringComparison.OrdinalIgnoreCase);

                        // Validate frame_id (should be "ros_optical" for all SLAM data)
                        // Default to "ros_optical" if missing (backward compatibility)
                        string frameId = frame["frame_id"]?.ToString() ?? "ros_optical";
                        if (frameId != "ros_optical")
                        {
                            // Non-breaking: log unexpected frame but continue processing
                            System.Diagnostics.Debug.WriteLine($"[SLAM3D] Unexpected frame_id: {frameId} (expected ros_optical)");
                        }

                        bool hasPosePositionInFrame = false;
                        float latestX, latestY, latestZ;
                        lock (_poseLock)
                        {
                            float prevX = _dronePosX;
                            float prevY = _dronePosY;
                            float prevZ = _dronePosZ;
                            float prevRoll = _droneRollRaw;
                            float prevPitch = _dronePitchRaw;
                            float prevYaw = _droneYawRaw;

                            if (TryReadFloatToken(frame["x"], out float xVal))
                            {
                                _dronePosX = xVal;
                                hasPosePositionInFrame = true;
                            }

                            if (TryReadFloatToken(frame["y"], out float yVal))
                            {
                                _dronePosY = yVal;
                                hasPosePositionInFrame = true;
                            }

                            if (TryReadFloatToken(frame["z"], out float zVal))
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
                                    {
                                        // After upstream stalls, snap render pose to fresh raw pose
                                        // to avoid prolonged catch-up lag from stale samples.
                                        _renderPoseInitialized = false;
                                    }
                                }
                                _lastRawPoseUpdateStamp = nowPoseStamp;
                            }

                            bool hasBodyRoll = TryReadFloatToken(frame["body_roll"], out float bodyRoll);
                            bool hasBodyPitch = TryReadFloatToken(frame["body_pitch"], out float bodyPitch);
                            bool hasBodyYaw = TryReadFloatToken(frame["body_yaw"], out float bodyYaw);

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
                            }
                            else
                            {
                                hasRoll = TryReadFloatToken(frame["roll"], out nextRoll);
                                hasPitch = TryReadFloatToken(frame["pitch"], out nextPitch);
                                hasYaw = TryReadFloatToken(frame["yaw"], out nextYaw);
                            }

                            if (hasRoll && hasPitch && hasYaw)
                            {
                                bool rejectResetGlitch = ShouldRejectAttitudeResetGlitch(
                                    prevX,
                                    prevY,
                                    prevZ,
                                    prevRoll,
                                    prevPitch,
                                    prevYaw,
                                    _dronePosX,
                                    _dronePosY,
                                    _dronePosZ,
                                    nextRoll,
                                    nextPitch,
                                    nextYaw);

                                if (rejectResetGlitch && _poseResetRejectStreak < PoseResetRejectStreakLimit)
                                {
                                    _poseResetRejectStreak++;
                                    if ((DateTime.UtcNow - _lastPoseResetDropLogUtc).TotalSeconds >= 2)
                                    {
                                        _lastPoseResetDropLogUtc = DateTime.UtcNow;
                                        AppendStatusLogSafe("Ignored transient zero-attitude reset frame.");
                                    }
                                }
                                else
                                {
                                    _poseResetRejectStreak = 0;
                                    _droneRollRaw = nextRoll;
                                    _dronePitchRaw = nextPitch;
                                    _droneYawRaw = nextYaw;
                                }
                            }

                            // Velocity (optional, from external VIO state)
                            if (TryReadFloatToken(frame["vx"], out float vxVal))
                                _droneVelX = vxVal;
                            if (TryReadFloatToken(frame["vy"], out float vyVal))
                                _droneVelY = vyVal;
                            if (TryReadFloatToken(frame["vz"], out float vzVal))
                                _droneVelZ = vzVal;

                            latestX = _dronePosX;
                            latestY = _dronePosY;
                            latestZ = _dronePosZ;
                        }

                        // Trajectory
                        if (hasPosePositionInFrame)
                            AddTrajectoryPoint(latestX, latestY, latestZ);

                        // Detection markers
                        var detectionsToken = frame["detections"] as JArray;
                        if (detectionsToken != null)
                        {
                            var markers = new List<DetectionMarker3D>();
                            foreach (var d in detectionsToken)
                            {
                                var dx = d["x"]?.Value<double?>();
                                var dy = d["y"]?.Value<double?>();
                                var dz = d["z"]?.Value<double?>();
                                if (dx == null || dy == null || dz == null) continue;
                                markers.Add(new DetectionMarker3D
                                {
                                    Label = d["label"]?.ToString() ?? "",
                                    X = dx.Value, Y = dy.Value, Z = dz.Value,
                                    Confidence = d["confidence"]?.Value<double>() ?? 0,
                                    SeenCount = d["seen_count"]?.Value<int>() ?? 1,
                                    HsvColor = d["hsv_color"]?.ToString(),
                                    ColorMatch = d["color_match"]?.Value<bool?>() ?? true,
                                    NeedsReview = d["needs_review"]?.Value<bool>() ?? false,
                                });
                            }
                            lock (_poseLock) { _detectionMarkers = markers; }
                        }
                        else if (frame.ContainsKey("detections"))
                        {
                            lock (_poseLock) { _detectionMarkers = new List<DetectionMarker3D>(); }
                        }

                        if (frameType == "mesh")
                        {
                            var meshToken = frame["mesh"];
                            if (meshToken != null)
                            {
                                var meshData = meshToken.ToObject<MeshDataModel>();
                                UpdateMeshVisual(meshData);

                                _meshUpdateCount++;
                                _lastUpdateTime = DateTime.Now;
                                string statsText;
                                {
                                    if (meshData != null)
                                        _totalBlocks = meshData.TotalBlocks > 0 ? meshData.TotalBlocks : meshData.TotalVoxels;
                                    string mode = (meshData?.Mode == "voxel" || meshData?.Mode == "voxels") ? "voxels" : "blocks";
                                    int cachedVoxels;
                                    lock (_meshLock) { cachedVoxels = _persistedBlocks.Count; }
                                    statsText = $"Mesh: {_totalBlocks:N0} {mode} ({cachedVoxels:N0} cached)";
                                }
                                UpdateStatusSafe($"Status: Connected (30Hz) | Updates: {_meshUpdateCount}");
                                UpdateStatsSafe(statsText);
                            }
                        }
                    }
                }
                catch (OperationCanceledException) { break; }
                catch (WebSocketException ex) { UpdateStatusSafe($"WebSocket error: {ex.Message}"); }
                catch (Exception ex) { UpdateStatusSafe($"Stream error: {ex.Message}"); }
                finally
                {
                    try { _webSocket?.Dispose(); } catch { }
                    _webSocket = null;
                }

                if (ct.IsCancellationRequested) break;
                UpdateStatusSafe($"Reconnecting in {_wsReconnectDelayMs / 1000}s...");
                try { await Task.Delay(_wsReconnectDelayMs, ct); }
                catch (OperationCanceledException) { break; }
                _wsReconnectDelayMs = Math.Min(_wsReconnectDelayMs * 2, MaxWsReconnectDelayMs);
            }
        }

        private void AddTrajectoryPoint(float x, float y, float z)
        {
            lock (_trajectoryLock)
            {
                _trajectoryPoints.Add(new float[] { x, y, z });
                int overflow = _trajectoryPoints.Count - MaxTrajectoryPoints;
                if (overflow > 0)
                    _trajectoryPoints.RemoveRange(0, overflow);
            }
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
            _currentViewMode = _currentViewMode switch
            {
                CameraViewMode.ThirdPerson => CameraViewMode.FirstPerson,
                CameraViewMode.FirstPerson => CameraViewMode.FreeOrbit,
                CameraViewMode.FreeOrbit => CameraViewMode.ThirdPerson,
                _ => CameraViewMode.ThirdPerson
            };

            // Seed orbit camera from current drone position when entering orbit
            if (_currentViewMode == CameraViewMode.FreeOrbit)
            {
                ZedToGL(_renderPosX, _renderPosY, _renderPosZ,
                    out _orbitCenterX, out _orbitCenterY, out _orbitCenterZ);
            }

            string modeName = _currentViewMode switch
            {
                CameraViewMode.FirstPerson => "FPV",
                CameraViewMode.ThirdPerson => "TPV",
                CameraViewMode.FreeOrbit => "Orbit",
                _ => "?"
            };
            _btnToggleCamera.Text = $"View: {modeName}";
        }

        private void BtnResetView_Click(object sender, EventArgs e)
        {
            _orbitYaw = 45f;
            _orbitPitch = 30f;
            _orbitDistance = 12f;
            _orbitCenterX = _orbitCenterY = _orbitCenterZ = 0;
        }

        private void CenterOrbitOnCurrentPose()
        {
            ZedToGL(_renderPosX, _renderPosY, _renderPosZ,
                out _orbitCenterX, out _orbitCenterY, out _orbitCenterZ);
            _currentViewMode = CameraViewMode.FreeOrbit;
            if (_btnToggleCamera != null)
                _btnToggleCamera.Text = "View: Orbit";
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
            lock (_meshLock)
            {
                _persistedBlocks.Clear();
                ClearEvictionTracking();
                _occupancySet.Clear();
                _voxelLastSeen.Clear();
                _voxelVerts = null;
                _voxelIndices = null;
                _voxelIndexCount = 0;
                _meshDirty = false;
                _pendingMeshUpdate = false;
                _lastRenderedCount = 0;
            }
            lock (_trajectoryLock)
            {
                _trajectoryPoints.Clear();
            }
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
                _updateCts?.Cancel();
                try
                {
                    if (_webSocket?.State == WebSocketState.Open)
                        _webSocket.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, "", CancellationToken.None).Wait(500);
                }
                catch { }
                try { _webSocket?.Dispose(); } catch { }
                _webSocket = null;
                _updateCts?.Dispose();
                _glControl?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
