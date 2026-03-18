// ============================================================
// SLAM3DView.cs - 3D SLAM Visualization for Mission Planner
// ============================================================
// Real-time 3D mesh visualization from nvblox SLAM.
// Uses OpenTK (OpenGL) for cross-platform rendering (Windows + Linux).
// ============================================================

using System;
using System.Collections.Generic;
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
        [JsonProperty("color")]
        public List<int> Color { get; set; }
    }

    public class DetectionMarker3D
    {
        public string Label { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double Confidence { get; set; }
        public int SeenCount { get; set; }
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

        // ---- GL Control ----
        private GLControl _glControl;
        private bool _glInitialized;
        private System.Windows.Forms.Timer _renderTimer;

        // ---- WebSocket ----
        private ClientWebSocket _webSocket;
        private CancellationTokenSource _updateCts;
        private int _wsReconnectDelayMs = 1000;
        private const int MaxWsReconnectDelayMs = 10000;
        private volatile bool _disposed;
        private volatile bool _autoUpdateEnabled = true;
        private readonly object _poseLock = new object();
        private const int MaxWebSocketMessageSize = 10 * 1024 * 1024;

        // ---- Servo polling ----
        private float _servoAngleDeg = 90.0f;
        private System.Windows.Forms.Timer _servoTimer;

        // ---- Drone pose (raw from WS, ZED optical/odom frame) ----
        private float _dronePosX, _dronePosY, _dronePosZ;
        private float _droneRollRaw, _dronePitchRaw, _droneYawRaw;

        // ---- Voxel storage ----
        private Dictionary<long, uint> _persistedBlocks = new Dictionary<long, uint>();
        private Queue<long> _voxelInsertionOrder = new Queue<long>();
        private HashSet<long> _occupancySet = new HashSet<long>();
        private const int MaxPersistedVoxels = 5000;
        private double _currentVoxelSize = 0.05;

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

        // ---- Trajectory ----
        private List<float[]> _trajectoryPoints = new List<float[]>(); // each [x,y,z] in ZED frame
        private const int MaxTrajectoryPoints = 500;

        // ---- Detection markers ----
        private List<DetectionMarker3D> _detectionMarkers = new List<DetectionMarker3D>();
        private bool _detectionsDirty;

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
        private Button _btnToggleCamera, _btnResetView, _btnClearMesh;
        private Label _lblStatus, _lblStats;
        private CheckBox _chkShowGrid, _chkShowTrajectory, _chkAutoUpdate;
        private NumericUpDown _numLength, _numWidth, _numHeight, _numHeadingOffset;
        private int _meshUpdateCount;
        private int _totalBlocks;
        private DateTime _lastUpdateTime = DateTime.MinValue;

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

        public SLAM3DView(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            InitializeComponents();
            StartUpdateLoop();
            StartServoPolling();
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
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 110));

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

            _chkShowGrid = CreateCheckBox("Grid", x, y + 4, true);
            _controlPanel.Controls.Add(_chkShowGrid);
            x += 55;

            _chkShowTrajectory = CreateCheckBox("Trail", x, y + 4, true);
            _controlPanel.Controls.Add(_chkShowTrajectory);
            x += 55;

            _chkAutoUpdate = CreateCheckBox("Auto", x, y + 4, true);
            _chkAutoUpdate.CheckedChanged += (s, e) => _autoUpdateEnabled = _chkAutoUpdate.Checked;
            _controlPanel.Controls.Add(_chkAutoUpdate);

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

            _controlPanel.Controls.Add(CreateLabel("Hdg Offset:", x, y + 3));
            x += 68;
            _numHeadingOffset = CreateNumericUpDown(x, y, 55, -180, 180, (decimal)_config.SlamHeadingOffsetDeg);
            _numHeadingOffset.ValueChanged += (s, e) => { _config.SlamHeadingOffsetDeg = (float)_numHeadingOffset.Value; _config.Save(); };
            _controlPanel.Controls.Add(_numHeadingOffset);

            // Third row: status
            y += 28;
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

            mainLayout.Controls.Add(_controlPanel, 0, 1);
            Controls.Add(mainLayout);

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

            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();
            float aspect = (float)w / h;
            var proj = Matrix4.CreatePerspectiveFieldOfView(
                MathHelper.DegreesToRadians(60f), aspect, 0.05f, 500f);
            GL.LoadMatrix(ref proj);
            GL.MatrixMode(MatrixMode.Modelview);
        }

        // ==================== Main Render ====================

        private void GlControl_Paint(object sender, PaintEventArgs e)
        {
            if (!_glInitialized || _glControl == null) return;

            try
            {
                _glControl.MakeCurrent();
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
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"SLAM3D render error: {ex.Message}");
            }
        }

        // ==================== Camera ====================

        private void SetupCamera()
        {
            float posX, posY, posZ, rollRaw, pitchRaw, yawRaw, servoDeg;
            lock (_poseLock)
            {
                posX = _dronePosX;
                posY = _dronePosY;
                posZ = _dronePosZ;
                rollRaw = _droneRollRaw;
                pitchRaw = _dronePitchRaw;
                yawRaw = _droneYawRaw;
                servoDeg = _servoAngleDeg;
            }

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

            float posX, posY, posZ, rollRaw, pitchRaw, yawRaw, servoDeg;
            lock (_poseLock)
            {
                posX = _dronePosX; posY = _dronePosY; posZ = _dronePosZ;
                rollRaw = _droneRollRaw; pitchRaw = _dronePitchRaw; yawRaw = _droneYawRaw;
                servoDeg = _servoAngleDeg;
            }

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
            if (_trajectoryPoints.Count < 2) return;

            GL.Disable(EnableCap.Lighting);
            GL.Color3(1f, 0.78f, 0f); // gold
            GL.LineWidth(2f);
            GL.Begin(PrimitiveType.LineStrip);
            foreach (var pt in _trajectoryPoints)
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
            if (DateTime.UtcNow - _lastMeshRebuild < MinRebuildInterval) return;

            int newSince = _persistedBlocks.Count - _lastRenderedCount;
            if (newSince > 0 && newSince < MinNewVoxelsForRebuild && _lastRenderedCount > 0) return;

            _lastMeshRebuild = DateTime.UtcNow;
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

        private void UpdateMeshVisual(MeshDataModel meshData)
        {
            try
            {
                if (meshData?.Clear == true)
                {
                    _persistedBlocks.Clear();
                    _voxelInsertionOrder.Clear();
                    _occupancySet.Clear();
                    _voxelVerts = null;
                    _voxelIndices = null;
                    _voxelIndexCount = 0;
                    _meshDirty = false;
                    _lastRenderedCount = 0;
                }

                // Handle removals
                if (meshData?.Removed != null)
                {
                    foreach (var r in meshData.Removed)
                    {
                        int rx = -r.Y, ry = r.Z, rz = -r.X;
                        var key = PackVoxelKey(rx, ry, rz);
                        if (_persistedBlocks.Remove(key))
                        {
                            _occupancySet.Remove(PackKey(rx, ry, rz));
                            _meshDirty = true;
                        }
                    }
                }

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
                    RebuildVoxelMesh();
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Mesh update error: {ex.Message}");
            }
        }

        private void ProcessVoxels(MeshDataModel meshData)
        {
            double vs = meshData.VoxelSize > 0 ? meshData.VoxelSize : 0.15;
            _currentVoxelSize = vs;

            foreach (var voxel in meshData.Voxels)
            {
                if (voxel.Position == null || voxel.Position.Count < 3) continue;

                int qx = (int)Math.Round(-voxel.Position[1] / vs);
                int qy = (int)Math.Round(voxel.Position[2] / vs);
                int qz = (int)Math.Round(-voxel.Position[0] / vs);
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

                if (!_persistedBlocks.ContainsKey(key) || _persistedBlocks[key] != colorKey)
                {
                    if (!_persistedBlocks.ContainsKey(key))
                        _voxelInsertionOrder.Enqueue(key);
                    _persistedBlocks[key] = colorKey;
                    _occupancySet.Add(PackKey(qx, qy, qz));
                    _meshDirty = true;
                }
            }

            EvictOldVoxels();
        }

        private void ProcessBlocks(MeshDataModel meshData)
        {
            double bs = meshData.BlockSize > 0 ? meshData.BlockSize : 0.05;
            _currentVoxelSize = bs;

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
                        _voxelInsertionOrder.Enqueue(key);
                    _persistedBlocks[key] = colorKey;
                    _occupancySet.Add(PackKey(bix, biy, biz));
                    _meshDirty = true;
                }
            }

            EvictOldVoxels();
        }

        private void EvictOldVoxels()
        {
            if (_persistedBlocks.Count <= MaxPersistedVoxels) return;

            int toRemove = _persistedBlocks.Count - MaxPersistedVoxels;
            for (int i = 0; i < toRemove && _voxelInsertionOrder.Count > 0; i++)
            {
                long evictKey = _voxelInsertionOrder.Dequeue();
                if (_persistedBlocks.Remove(evictKey))
                {
                    UnpackVoxelKey(evictKey, out int ex, out int ey, out int ez);
                    _occupancySet.Remove(PackKey(ex, ey, ez));
                }
                else
                {
                    i--; // already removed, skip
                }
            }
            _meshDirty = true;
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

                        if (frame["x"] == null) continue;

                        lock (_poseLock)
                        {
                            var xToken = frame["x"];
                            if (xToken != null && (xToken.Type == JTokenType.Integer || xToken.Type == JTokenType.Float))
                                _dronePosX = xToken.Value<float>();

                            var yToken = frame["y"];
                            if (yToken != null && (yToken.Type == JTokenType.Integer || yToken.Type == JTokenType.Float))
                                _dronePosY = yToken.Value<float>();

                            var zToken = frame["z"];
                            if (zToken != null && (zToken.Type == JTokenType.Integer || zToken.Type == JTokenType.Float))
                                _dronePosZ = zToken.Value<float>();

                            var rollToken = frame["roll"];
                            if (rollToken != null && (rollToken.Type == JTokenType.Integer || rollToken.Type == JTokenType.Float))
                                _droneRollRaw = rollToken.Value<float>();

                            var pitchToken = frame["pitch"];
                            if (pitchToken != null && (pitchToken.Type == JTokenType.Integer || pitchToken.Type == JTokenType.Float))
                                _dronePitchRaw = pitchToken.Value<float>();

                            var yawToken = frame["yaw"];
                            if (yawToken != null && (yawToken.Type == JTokenType.Integer || yawToken.Type == JTokenType.Float))
                                _droneYawRaw = yawToken.Value<float>();
                        }

                        // Trajectory
                        AddTrajectoryPoint(_dronePosX, _dronePosY, _dronePosZ);

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
                                });
                            }
                            lock (_poseLock) { _detectionMarkers = markers; }
                            _detectionsDirty = true;
                        }
                        else if (frame.ContainsKey("detections"))
                        {
                            lock (_poseLock) { _detectionMarkers = new List<DetectionMarker3D>(); }
                            _detectionsDirty = true;
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
                                if (meshData != null)
                                    _totalBlocks = meshData.TotalBlocks > 0 ? meshData.TotalBlocks : meshData.TotalVoxels;
                                string mode = (meshData?.Mode == "voxel" || meshData?.Mode == "voxels") ? "voxels" : "blocks";
                                UpdateStatusSafe($"Status: Connected (30Hz) | Updates: {_meshUpdateCount}");
                                UpdateStatsSafe($"Mesh: {_totalBlocks:N0} {mode} ({_persistedBlocks.Count:N0} cached)");
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
            if (_trajectoryPoints.Count > 0)
            {
                var last = _trajectoryPoints[_trajectoryPoints.Count - 1];
                float dx = x - last[0], dy = y - last[1], dz = z - last[2];
                if (dx * dx + dy * dy + dz * dz < 0.0025f) return; // < 5cm
            }
            _trajectoryPoints.Add(new float[] { x, y, z });
            if (_trajectoryPoints.Count > MaxTrajectoryPoints)
                _trajectoryPoints.RemoveAt(0);
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
                lock (_poseLock)
                {
                    ZedToGL(_dronePosX, _dronePosY, _dronePosZ,
                        out _orbitCenterX, out _orbitCenterY, out _orbitCenterZ);
                }
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

        private async void BtnClearMesh_Click(object sender, EventArgs e)
        {
            _persistedBlocks.Clear();
            _voxelInsertionOrder.Clear();
            _occupancySet.Clear();
            _voxelVerts = null;
            _voxelIndices = null;
            _voxelIndexCount = 0;
            _meshDirty = false;
            _lastRenderedCount = 0;
            _trajectoryPoints.Clear();
            _totalBlocks = 0;

            try
            {
                await JetsonApiService.PostAsync("/api/task/2/slam/clear");
                UpdateStatusSafe("Mesh cleared");
            }
            catch (Exception ex)
            {
                UpdateStatusSafe($"Mesh cleared locally (server: {ex.Message})");
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

        // ==================== Cleanup ====================

        protected override void Dispose(bool disposing)
        {
            _disposed = true;
            if (disposing)
            {
                _renderTimer?.Stop();
                _renderTimer?.Dispose();
                _servoTimer?.Stop();
                _servoTimer?.Dispose();
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
