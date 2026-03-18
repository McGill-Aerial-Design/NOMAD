// ============================================================
// SLAM3DView.cs - 3D SLAM Visualization for Mission Planner
// ============================================================
// Real-time 3D mesh visualization from nvblox SLAM.
// Uses Helix Toolkit WPF for 3D rendering, hosted in WinForms.
// ============================================================

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Forms;
using System.Windows.Forms.Integration;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Brush = System.Windows.Media.Brush;
using Brushes = System.Windows.Media.Brushes;
using Color = System.Windows.Media.Color;
using Control = System.Windows.Forms.Control;
using Panel = System.Windows.Forms.Panel;
using Point = System.Windows.Point;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Camera view mode for 3D visualization.
    /// </summary>
    public enum CameraViewMode
    {
        /// <summary>First-person view from drone perspective.</summary>
        FirstPerson,
        /// <summary>Third-person view following the drone.</summary>
        ThirdPerson,
        /// <summary>Free orbit camera controlled by user.</summary>
        FreeOrbit
    }

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
        // Per-voxel mode (mode == "voxel")
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
        /// <summary>Position [x, y, z] in meters.</summary>
        [JsonProperty("p")]
        public List<double> Position { get; set; }
        /// <summary>Color [R, G, B] (0-255).</summary>
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
        /// <summary>Block-only mode: average [R, G, B] color (0-255).</summary>
        [JsonProperty("color")]
        public List<int> Color { get; set; }
    }

    /// <summary>
    /// Represents a detected object marker in the 3D SLAM view.
    /// </summary>
    public class DetectionMarker3D
    {
        public string Label { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double Confidence { get; set; }
        public int SeenCount { get; set; }
    }


    /// <summary>
    /// 3D SLAM visualization control using Helix Toolkit.
    /// Displays real-time nvblox mesh from Jetson with drone position overlay.
    /// </summary>
    public class SLAM3DView : System.Windows.Forms.UserControl
    {
        // ==================== Fields ====================
        private readonly NOMADConfig _config;
        private CancellationTokenSource _updateCts;
        
        // WebSocket streaming (30Hz)
        private ClientWebSocket _webSocket;
        private int _wsReconnectDelayMs = 1000;
        private const int MaxWsReconnectDelayMs = 10000;
        private volatile bool _disposed;
        private volatile bool _autoUpdateEnabled = true;
        private readonly object _poseLock = new object();
        private const int MaxWebSocketMessageSize = 10 * 1024 * 1024; // 10MB
        
        // WPF hosting
        private ElementHost _elementHost;
        private HelixViewport3D _viewport;
        
        // 3D Models
        private Model3DGroup _meshModelGroup;
        private ModelVisual3D _meshVisual;
        private ModelVisual3D _droneVisual;
        private Model3D _droneModelContent; // Stored to restore after hiding in FPV
        private ModelVisual3D _gridVisual;
        private ModelVisual3D _trajectoryVisual;
        private ModelVisual3D _detectionMarkersVisual;
        
        // Detection marker state
        private List<DetectionMarker3D> _detectionMarkers = new List<DetectionMarker3D>();
        
        // Cameras
        private PerspectiveCamera _fpvCamera;
        private PerspectiveCamera _tpvCamera;
        private PerspectiveCamera _orbitCamera;
        private CameraViewMode _currentViewMode = CameraViewMode.ThirdPerson;
        
        // Drone state
        private Point3D _dronePosition = new Point3D(0, 0, 0);
        private double _droneYaw = 0;
        private double _dronePitch = 0;
        private double _droneRoll = 0;
        
        // Cached drone transforms (P3-8)
        private Transform3DGroup _droneTransformGroup;
        private AxisAngleRotation3D _droneRotX, _droneRotY, _droneRotZ;
        private TranslateTransform3D _droneTranslation;
        
        private List<Point3D> _trajectoryPoints = new List<Point3D>();
        private const int MaxTrajectoryPoints = 500;
        private int _lastTrajectoryRendered = 0; // incremental trajectory rendering
        
        // UI Controls (WinForms)
        private Panel _controlPanel;
        private System.Windows.Forms.Button _btnToggleCamera;
        private System.Windows.Forms.Button _btnResetView;
        private System.Windows.Forms.Button _btnClearMesh;
        private System.Windows.Forms.Label _lblStatus;
        private System.Windows.Forms.Label _lblStats;
        private System.Windows.Forms.CheckBox _chkShowGrid;
        private System.Windows.Forms.CheckBox _chkShowTrajectory;
        private System.Windows.Forms.CheckBox _chkAutoUpdate;
        
        // Statistics
        private int _meshUpdateCount = 0;
        private int _totalBlocks = 0;
        private DateTime _lastUpdateTime = DateTime.MinValue;
        
        // Persisted block map: key = packed long (ix,iy,iz), value = packed ARGB color
        private Dictionary<long, uint> _persistedBlocks = new Dictionary<long, uint>();
        private Queue<long> _voxelInsertionOrder = new Queue<long>();
        private const int MaxPersistedVoxels = 5000; // Hard cap with face-culling keeps triangle count low

        // Material cache to avoid recreating WPF resources every frame
        private Dictionary<uint, Material> _materialCache = new Dictionary<uint, Material>();

        // Dirty tracking: only rebuild mesh when significant changes arrive
        private bool _meshDirty = false;
        private int _lastRenderedCount = 0;
        private const int MinNewVoxelsForRebuild = 20; // Only rebuild after 20+ new voxels

        // Time-based debounce for mesh rebuilds (P3-7)
        private DateTime _lastMeshRebuild = DateTime.MinValue;
        private static readonly TimeSpan MinRebuildInterval = TimeSpan.FromMilliseconds(250);

        // Occupancy set for fast neighbor lookups (adjacent-face culling)
        private HashSet<long> _occupancySet = new HashSet<long>();

        // Pack three ints into a single long key for O(1) neighbor lookup.
        // Each axis masked to 20 bits; valid range per axis: [-1048576, 1048575].
        private static long PackKey(int x, int y, int z)
        {
            long lx = (long)(x + 0x100000) & 0xFFFFF;
            long ly = (long)(y + 0x100000) & 0xFFFFF;
            long lz = (long)(z + 0x100000) & 0xFFFFF;
            return (lx << 40) | (ly << 20) | lz;
        }

        // Pack three grid indices into a single long for dictionary keys.
        // Each axis uses 16 bits with +32768 offset; valid range: [-32768, 32767].
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

        // ==================== Constructor ====================
        
        /// <summary>
        /// Create a new SLAM 3D visualization control.
        /// </summary>
        /// <param name="config">NOMAD configuration with Jetson connection details.</param>
        public SLAM3DView(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            
            InitializeComponents();
            Initialize3DViewport();
            StartUpdateLoop();
        }

        // ==================== Initialization ====================
        
        private void InitializeComponents()
        {
            this.BackColor = System.Drawing.Color.FromArgb(30, 30, 33);
            this.Dock = DockStyle.Fill;
            
            // Main layout - viewport takes most space, controls at bottom
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 2,
                BackColor = System.Drawing.Color.FromArgb(30, 30, 33),
            };
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));  // Viewport
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 80));  // Controls
            
            // WPF Host for Helix Toolkit
            _elementHost = new ElementHost
            {
                Dock = DockStyle.Fill,
                BackColor = System.Drawing.Color.Black,
            };
            mainLayout.Controls.Add(_elementHost, 0, 0);
            
            // Control panel at bottom
            _controlPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = System.Drawing.Color.FromArgb(40, 40, 45),
                Padding = new Padding(10),
            };
            
            // Toggle camera button
            _btnToggleCamera = new System.Windows.Forms.Button
            {
                Text = "Toggle View (TPV)",
                Location = new System.Drawing.Point(10, 10),
                Size = new System.Drawing.Size(120, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = System.Drawing.Color.FromArgb(0, 122, 204),
                ForeColor = System.Drawing.Color.White,
            };
            _btnToggleCamera.Click += BtnToggleCamera_Click;
            _controlPanel.Controls.Add(_btnToggleCamera);
            
            // Reset view button
            _btnResetView = new System.Windows.Forms.Button
            {
                Text = "Reset View",
                Location = new System.Drawing.Point(140, 10),
                Size = new System.Drawing.Size(90, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = System.Drawing.Color.FromArgb(60, 60, 65),
                ForeColor = System.Drawing.Color.White,
            };
            _btnResetView.Click += BtnResetView_Click;
            _controlPanel.Controls.Add(_btnResetView);
            
            // Clear mesh button
            _btnClearMesh = new System.Windows.Forms.Button
            {
                Text = "Clear Mesh",
                Location = new System.Drawing.Point(240, 10),
                Size = new System.Drawing.Size(90, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = System.Drawing.Color.FromArgb(180, 60, 60),
                ForeColor = System.Drawing.Color.White,
            };
            _btnClearMesh.Click += BtnClearMesh_Click;
            _controlPanel.Controls.Add(_btnClearMesh);
            
            // Checkboxes
            _chkShowGrid = new System.Windows.Forms.CheckBox
            {
                Text = "Grid",
                Location = new System.Drawing.Point(350, 15),
                ForeColor = System.Drawing.Color.White,
                AutoSize = true,
                Checked = true,
            };
            _chkShowGrid.CheckedChanged += ChkShowGrid_CheckedChanged;
            _controlPanel.Controls.Add(_chkShowGrid);
            
            _chkShowTrajectory = new System.Windows.Forms.CheckBox
            {
                Text = "Trajectory",
                Location = new System.Drawing.Point(410, 15),
                ForeColor = System.Drawing.Color.White,
                AutoSize = true,
                Checked = true,
            };
            _chkShowTrajectory.CheckedChanged += ChkShowTrajectory_CheckedChanged;
            _controlPanel.Controls.Add(_chkShowTrajectory);
            
            _chkAutoUpdate = new System.Windows.Forms.CheckBox
            {
                Text = "Auto Update",
                Location = new System.Drawing.Point(490, 15),
                ForeColor = System.Drawing.Color.White,
                AutoSize = true,
                Checked = true,
            };
            _chkAutoUpdate.CheckedChanged += (s, e) => _autoUpdateEnabled = _chkAutoUpdate.Checked;
            _controlPanel.Controls.Add(_chkAutoUpdate);
            
            // Status labels
            _lblStatus = new System.Windows.Forms.Label
            {
                Text = "Status: Connecting...",
                Location = new System.Drawing.Point(10, 50),
                ForeColor = System.Drawing.Color.FromArgb(200, 200, 200),
                AutoSize = true,
                Font = new System.Drawing.Font("Consolas", 9),
            };
            _controlPanel.Controls.Add(_lblStatus);
            
            _lblStats = new System.Windows.Forms.Label
            {
                Text = "Mesh: 0 vertices, 0 triangles",
                Location = new System.Drawing.Point(250, 50),
                ForeColor = System.Drawing.Color.FromArgb(150, 150, 150),
                AutoSize = true,
                Font = new System.Drawing.Font("Consolas", 9),
            };
            _controlPanel.Controls.Add(_lblStats);
            
            mainLayout.Controls.Add(_controlPanel, 0, 1);
            this.Controls.Add(mainLayout);
        }
        
        private void Initialize3DViewport()
        {
            // Create WPF viewport
            _viewport = new HelixViewport3D
            {
                Background = new SolidColorBrush(Color.FromRgb(20, 20, 25)),
                ShowCoordinateSystem = true,
                CoordinateSystemLabelForeground = Brushes.White,
                ShowViewCube = true,
                ViewCubeBackText = "S",
                ViewCubeFrontText = "N",
                ViewCubeLeftText = "W",
                ViewCubeRightText = "E",
                ZoomExtentsWhenLoaded = true,
                RotateAroundMouseDownPoint = true,
                CameraRotationMode = CameraRotationMode.Turntable,
            };
            
            // Initialize cameras
            // --------------------------------------------------------
            // Frame convention:
            // SLAM data arrives in ROS optical frame: X-right, Y-down, Z-forward.
            // WPF 3D coordinate system: X-right, Y-up, Z-toward-viewer.
            // Conversion: WPF_X = ROS_X, WPF_Y = -ROS_Y, WPF_Z = -ROS_Z
            // --------------------------------------------------------
            _tpvCamera = new PerspectiveCamera
            {
                Position = new Point3D(0, 5, 10),
                LookDirection = new Vector3D(0, -3, -8),
                UpDirection = new Vector3D(0, 1, 0),
                FieldOfView = 60,
                NearPlaneDistance = 0.1,
                FarPlaneDistance = 1000,
            };
            
            _fpvCamera = new PerspectiveCamera
            {
                Position = new Point3D(0, 0, 0),
                LookDirection = new Vector3D(0, 0, -1),
                UpDirection = new Vector3D(0, 1, 0),
                FieldOfView = 90,
                NearPlaneDistance = 0.01,
                FarPlaneDistance = 500,
            };
            
            _orbitCamera = new PerspectiveCamera
            {
                Position = new Point3D(8, 6, 10),
                LookDirection = new Vector3D(-8, -4, -10),
                UpDirection = new Vector3D(0, 1, 0),
                FieldOfView = 45,
                NearPlaneDistance = 0.1,
                FarPlaneDistance = 1000,
            };
            
            _viewport.Camera = _tpvCamera;
            
            // Add lighting
            var ambientLight = new AmbientLight(Color.FromRgb(80, 80, 80));
            var directionalLight = new DirectionalLight(
                Color.FromRgb(255, 255, 255),
                new Vector3D(-1, -1, -1)
            );
            var directionalLight2 = new DirectionalLight(
                Color.FromRgb(100, 100, 120),
                new Vector3D(1, 0.5, 0.5)
            );
            
            var lightGroup = new Model3DGroup();
            lightGroup.Children.Add(ambientLight);
            lightGroup.Children.Add(directionalLight);
            lightGroup.Children.Add(directionalLight2);
            _viewport.Children.Add(new ModelVisual3D { Content = lightGroup });
            
            // Initialize mesh model group
            _meshModelGroup = new Model3DGroup();
            _meshVisual = new ModelVisual3D { Content = _meshModelGroup };
            _viewport.Children.Add(_meshVisual);
            
            // Add grid
            CreateGrid();
            
            // Add drone marker
            CreateDroneMarker();
            
            // Add trajectory
            CreateTrajectoryVisual();
            
            // Add detection markers layer
            CreateDetectionMarkersVisual();
            
            // Host in WinForms
            _elementHost.Child = _viewport;
        }
        
        private void CreateGrid()
        {
            var gridBuilder = new MeshBuilder();
            
            // Create lightweight grid lines -- 20 lines instead of 82 pipes
            double gridSize = 10;
            int gridLines = 20;
            double spacing = gridSize * 2 / gridLines;
            
            for (int i = 0; i <= gridLines; i++)
            {
                double pos = -gridSize + i * spacing;
                
                // X lines -- thin cylinders with 4 segments (minimal)
                gridBuilder.AddPipe(
                    new Point3D(pos, -gridSize, 0),
                    new Point3D(pos, gridSize, 0),
                    0, 0.005, 4
                );
                
                // Y lines
                gridBuilder.AddPipe(
                    new Point3D(-gridSize, pos, 0),
                    new Point3D(gridSize, pos, 0),
                    0, 0.005, 4
                );
            }
            
            var gridMesh = gridBuilder.ToMesh();
            gridMesh.Freeze();
            var gridBrush = new SolidColorBrush(Color.FromArgb(40, 100, 100, 100));
            gridBrush.Freeze();
            var gridMaterial = new DiffuseMaterial(gridBrush);
            gridMaterial.Freeze();
            var gridModel = new GeometryModel3D(gridMesh, gridMaterial);
            gridModel.Freeze();
            
            _gridVisual = new ModelVisual3D { Content = gridModel };
            _viewport.Children.Add(_gridVisual);
        }
        
        private void CreateDroneMarker()
        {
            var droneBuilder = new MeshBuilder();
            
            // Arrow points along -Z in WPF (which is +Z forward in ROS optical frame)
            droneBuilder.AddArrow(
                new Point3D(0, 0, 0.04),
                new Point3D(0, 0, -0.08),
                0.015, 3
            );
            
            // Create material - bright cyan for visibility (P3-11: frozen)
            var droneMesh = droneBuilder.ToMesh();
            droneMesh.Freeze();
            var droneBrush = new SolidColorBrush(Color.FromRgb(0, 220, 220));
            droneBrush.Freeze();
            var droneMaterial = new DiffuseMaterial(droneBrush);
            droneMaterial.Freeze();
            var droneModel = new GeometryModel3D(droneMesh, droneMaterial);
            droneModel.Freeze();
            
            var droneGroup = new Model3DGroup();
            droneGroup.Children.Add(droneModel);
            
            // Initialize cached transforms (P3-8)
            _droneRotZ = new AxisAngleRotation3D(new Vector3D(0, 0, 1), 0);
            _droneRotX = new AxisAngleRotation3D(new Vector3D(1, 0, 0), 0);
            _droneRotY = new AxisAngleRotation3D(new Vector3D(0, 1, 0), 0);
            _droneTranslation = new TranslateTransform3D(0, 0, 0);
            
            _droneTransformGroup = new Transform3DGroup();
            _droneTransformGroup.Children.Add(new RotateTransform3D(_droneRotZ));
            _droneTransformGroup.Children.Add(new RotateTransform3D(_droneRotX));
            _droneTransformGroup.Children.Add(new RotateTransform3D(_droneRotY));
            _droneTransformGroup.Children.Add(_droneTranslation);
            
            _droneVisual = new ModelVisual3D { Content = droneGroup };
            _droneVisual.Transform = _droneTransformGroup;
            _droneModelContent = droneGroup;
            _viewport.Children.Add(_droneVisual);
        }
        
        private void CreateTrajectoryVisual()
        {
            // Initialize empty trajectory
            var trajectoryGroup = new Model3DGroup();
            _trajectoryVisual = new ModelVisual3D { Content = trajectoryGroup };
            _viewport.Children.Add(_trajectoryVisual);
        }

        private void CreateDetectionMarkersVisual()
        {
            var group = new Model3DGroup();
            _detectionMarkersVisual = new ModelVisual3D { Content = group };
            _viewport.Children.Add(_detectionMarkersVisual);
        }

        /// <summary>
        /// Get WPF color for a circle detection label.
        /// Maps the YOLO26 class names to their actual circle colors.
        /// </summary>
        private static Color GetDetectionColor(string label)
        {
            if (string.IsNullOrEmpty(label)) return Colors.White;
            
            string lower = label.ToLowerInvariant();
            if (lower.Contains("red")) return Color.FromRgb(220, 40, 40);
            if (lower.Contains("blue")) return Color.FromRgb(40, 100, 220);
            if (lower.Contains("green")) return Color.FromRgb(40, 200, 40);
            if (lower.Contains("yellow")) return Color.FromRgb(240, 220, 40);
            if (lower.Contains("black")) return Color.FromRgb(50, 50, 50);
            if (lower.Contains("white")) return Color.FromRgb(240, 240, 240);
            return Color.FromRgb(200, 100, 200); // Purple for unknown
        }

        /// <summary>
        /// Rebuild all detection markers in the 3D view.
        /// Each detected circle target is shown as a colored sphere at its 3D position.
        /// Uses a dirty flag set when new detection data arrives from the WebSocket.
        /// </summary>
        private bool _detectionsDirty = false;
        
        private void UpdateDetectionMarkers()
        {
            if (_detectionMarkersVisual == null)
                return;
            
            // Only rebuild when data actually changed
            if (!_detectionsDirty)
                return;
            _detectionsDirty = false;
            
            // Handle empty / cleared detections
            if (_detectionMarkers == null || _detectionMarkers.Count == 0)
            {
                _detectionMarkersVisual.Content = new Model3DGroup();
                return;
            }
            
            var group = new Model3DGroup();
            
            foreach (var det in _detectionMarkers)
            {
                var color = GetDetectionColor(det.Label);
                
                // ROS-to-WPF frame conversion for detection position
                double wx = det.X, wy = -det.Y, wz = -det.Z;
                
                // Sphere radius scales with confidence (0.03-0.08m)
                double conf = Math.Max(0.0, Math.Min(1.0, det.Confidence));
                double radius = 0.03 + conf * 0.05;
                
                var builder = new MeshBuilder();
                builder.AddSphere(
                    new Point3D(wx, wy, wz),
                    radius, 8, 8);
                
                var mesh = builder.ToMesh();
                mesh.Freeze();
                
                var brush = new SolidColorBrush(color);
                brush.Freeze();
                var material = new DiffuseMaterial(brush);
                material.Freeze();
                
                // Add emissive glow for visibility
                var emissiveBrush = new SolidColorBrush(
                    Color.FromArgb(80, color.R, color.G, color.B));
                emissiveBrush.Freeze();
                var emissive = new EmissiveMaterial(emissiveBrush);
                emissive.Freeze();
                
                var materialGroup = new MaterialGroup();
                materialGroup.Children.Add(material);
                materialGroup.Children.Add(emissive);
                materialGroup.Freeze();
                
                var model = new GeometryModel3D(mesh, materialGroup);
                model.Freeze();
                group.Children.Add(model);
                
                // Add a thin vertical line from floor to marker for depth perception
                var lineBuilder = new MeshBuilder();
                lineBuilder.AddPipe(
                    new Point3D(wx, 0, wz),
                    new Point3D(wx, wy, wz),
                    0, 0.003, 4);
                var lineMesh = lineBuilder.ToMesh();
                lineMesh.Freeze();
                
                var lineColor = Color.FromArgb(60, color.R, color.G, color.B);
                var lineBrush = new SolidColorBrush(lineColor);
                lineBrush.Freeze();
                var lineMaterial = new DiffuseMaterial(lineBrush);
                lineMaterial.Freeze();
                var lineModel = new GeometryModel3D(lineMesh, lineMaterial);
                lineModel.Freeze();
                group.Children.Add(lineModel);
            }
            
            _detectionMarkersVisual.Content = group;
        }

        // ==================== Update Loops ====================
        
        private void StartUpdateLoop()
        {
            _updateCts = new CancellationTokenSource();
            Task.Run(() => WebSocketStreamLoop(_updateCts.Token));
        }

        /// <summary>
        /// Connects to ws://jetson:8000/ws/slam and receives 30Hz frames.
        /// Frame types: "pose" (drone only) and "mesh" (mesh + drone).
        /// Reconnects with exponential backoff on disconnect.
        /// </summary>
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
                        bool timedOut = false;
                        bool oversized = false;
                        do
                        {
                            var receiveTask = _webSocket.ReceiveAsync(
                                new ArraySegment<byte>(buffer), ct);
                            var timeoutTask = Task.Delay(30000, ct);
                            var completed = await Task.WhenAny(receiveTask, timeoutTask);
                            if (completed == timeoutTask)
                            {
                                System.Diagnostics.Debug.WriteLine("SLAM3D: WebSocket receive timeout, reconnecting...");
                                timedOut = true;
                                break;
                            }
                            result = await receiveTask;
                            if (result.MessageType == WebSocketMessageType.Close)
                                break;
                            if (!oversized)
                            {
                                messageBuffer.Write(buffer, 0, result.Count);
                                if (messageBuffer.Length > MaxWebSocketMessageSize)
                                {
                                    System.Diagnostics.Debug.WriteLine($"SLAM3D: Message too large ({messageBuffer.Length} bytes), skipping");
                                    oversized = true;
                                }
                            }
                        } while (result != null && !result.EndOfMessage);

                        if (timedOut)
                            break;

                        if (result == null || result.MessageType == WebSocketMessageType.Close)
                            break;

                        if (oversized)
                            continue;

                        if (!_autoUpdateEnabled)
                            continue;

                        string json = Encoding.UTF8.GetString(
                            messageBuffer.GetBuffer(), 0, (int)messageBuffer.Length);
                        var frame = JObject.Parse(json);
                        string frameType = frame["type"]?.ToString() ?? "pose";

                        // Skip frames with no position data
                        if (frame["x"] == null)
                            continue;

                        lock (_poseLock)
                        {
                            _dronePosition = new Point3D(
                                frame["x"]?.Value<double>() ?? 0,
                                frame["y"]?.Value<double>() ?? 0,
                                frame["z"]?.Value<double>() ?? 0);
                            _droneRoll = frame["roll"]?.Value<double>() ?? 0;
                            _dronePitch = frame["pitch"]?.Value<double>() ?? 0;
                            _droneYaw = frame["yaw"]?.Value<double>() ?? 0;
                        }

                        // Parse detection markers if present (~5Hz from server)
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
                                    X = dx.Value,
                                    Y = dy.Value,
                                    Z = dz.Value,
                                    Confidence = d["confidence"]?.Value<double>() ?? 0,
                                    SeenCount = d["seen_count"]?.Value<int>() ?? 1,
                                });
                            }
                            _detectionMarkers = markers;
                            _detectionsDirty = true;
                        }
                        else if (frame.ContainsKey("detections"))
                        {
                            // Server explicitly sent null/empty detections -- clear markers
                            _detectionMarkers = new List<DetectionMarker3D>();
                            _detectionsDirty = true;
                        }

                        if (frameType == "mesh")
                        {
                            var meshToken = frame["mesh"];
                            if (meshToken != null)
                            {
                                var meshData = meshToken.ToObject<MeshDataModel>();
                                ProcessMeshAndPoseOnUiThread(meshData);
                            }
                            else
                            {
                                UpdatePoseVisualsOnUiThread();
                            }
                        }
                        else
                        {
                            UpdatePoseVisualsOnUiThread();
                        }
                    }
                }
                catch (OperationCanceledException) { break; }
                catch (WebSocketException ex)
                {
                    UpdateStatusSafe($"WebSocket error: {ex.Message}");
                }
                catch (Exception ex)
                {
                    UpdateStatusSafe($"Stream error: {ex.Message}");
                }
                finally
                {
                    if (_webSocket != null)
                    {
                        try { _webSocket.Dispose(); } catch { }
                        _webSocket = null;
                    }
                }

                if (ct.IsCancellationRequested) break;

                UpdateStatusSafe($"Reconnecting in {_wsReconnectDelayMs / 1000}s...");
                try { await Task.Delay(_wsReconnectDelayMs, ct); }
                catch (OperationCanceledException) { break; }

                _wsReconnectDelayMs = Math.Min(_wsReconnectDelayMs * 2, MaxWsReconnectDelayMs);
            }
        }

        private void UpdatePoseVisualsOnUiThread()
        {
            if (_disposed || !IsHandleCreated) return;
            Point3D pos;
            double yaw, pitch, roll;
            lock (_poseLock)
            {
                pos = _dronePosition;
                yaw = _droneYaw;
                pitch = _dronePitch;
                roll = _droneRoll;
            }
            try
            {
                this.BeginInvoke(new Action(() =>
                {
                    if (_disposed || _elementHost == null) return;
                    _elementHost.Invoke(new Action(() =>
                    {
                        AddTrajectoryPoint(pos);
                        UpdateDroneVisual(pos, yaw, pitch, roll);
                        UpdateTrajectoryVisual();
                        UpdateDetectionMarkers();
                        UpdateCameras(pos, yaw, pitch, roll);
                    }));
                }));
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
        }

        private void AddTrajectoryPoint(Point3D position)
        {
            if (_trajectoryPoints.Count == 0 ||
                (position - _trajectoryPoints[_trajectoryPoints.Count - 1]).Length > 0.05)
            {
                _trajectoryPoints.Add(position);
                if (_trajectoryPoints.Count > MaxTrajectoryPoints)
                {
                    _trajectoryPoints.RemoveAt(0);
                    _lastTrajectoryRendered = 0; // Force full rebuild after eviction
                }
            }
        }

        private void ProcessMeshAndPoseOnUiThread(MeshDataModel meshData)
        {
            if (_disposed || !IsHandleCreated) return;
            Point3D pos;
            double yaw, pitch, roll;
            lock (_poseLock)
            {
                pos = _dronePosition;
                yaw = _droneYaw;
                pitch = _dronePitch;
                roll = _droneRoll;
            }
            try
            {
                this.BeginInvoke(new Action(() =>
                {
                    if (_disposed || _elementHost == null) return;
                    _elementHost.Invoke(new Action(() =>
                    {
                        AddTrajectoryPoint(pos);
                        UpdateDroneVisual(pos, yaw, pitch, roll);
                        UpdateTrajectoryVisual();
                        UpdateDetectionMarkers();
                        UpdateCameras(pos, yaw, pitch, roll);
                        if (meshData != null)
                            UpdateMeshVisual(meshData);
                    }));

                    _meshUpdateCount++;
                    _lastUpdateTime = DateTime.Now;
                    if (meshData != null)
                        _totalBlocks = meshData.TotalBlocks > 0 ? meshData.TotalBlocks : meshData.TotalVoxels;
                    string mode = (meshData?.Mode == "voxel" || meshData?.Mode == "voxels") ? "voxels" : "blocks";
                    UpdateStatusSafe($"Status: Connected (30Hz) | Updates: {_meshUpdateCount}");
                    UpdateStatsSafe($"Mesh: {_totalBlocks:N0} {mode} ({_persistedBlocks.Count:N0} cached)");
                }));
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
        }
        
        private void UpdateMeshVisual(MeshDataModel meshData)
        {
            try
            {
                // Only clear when explicitly flagged by backend
                if (meshData?.Clear == true)
                {
                    _meshModelGroup.Children.Clear();
                    _persistedBlocks.Clear();
                    _voxelInsertionOrder.Clear();
                    _occupancySet.Clear();
                    _materialCache.Clear();
                    _meshDirty = false;
                    _lastRenderedCount = 0;
                }

                // Handle voxel removals (P2-7)
                if (meshData?.Removed != null && meshData.Removed.Count > 0)
                {
                    foreach (var r in meshData.Removed)
                    {
                        // Negate Y and Z: ROS odom (Y-down, Z-forward) → WPF (Y-up, Z-toward viewer)
                        int rx = r.X, ry = -r.Y, rz = -r.Z;
                        var key = PackVoxelKey(rx, ry, rz);
                        if (_persistedBlocks.Remove(key))
                        {
                            _occupancySet.Remove(PackKey(rx, ry, rz));
                            _meshDirty = true;
                        }
                    }
                }

                // Route to the appropriate renderer based on mode
                if ((meshData?.Mode == "voxel" || meshData?.Mode == "voxels") && meshData.Voxels != null && meshData.Voxels.Count > 0)
                {
                    UpdateMeshVisualVoxels(meshData);
                }
                else if (meshData?.Blocks != null && meshData.Blocks.Count > 0)
                {
                    UpdateMeshVisualBlocks(meshData);
                }
                else if (_meshDirty)
                {
                    // Removals-only frame: rebuild with existing persisted data
                    double vs = meshData?.VoxelSize > 0 ? meshData.VoxelSize :
                               (meshData?.BlockSize > 0 ? meshData.BlockSize : 0.15);
                    UpdateMeshVisualVoxels(new MeshDataModel
                    {
                        Mode = "voxels",
                        Voxels = new List<VoxelModel>(),
                        VoxelSize = vs,
                    });
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Mesh update error: {ex.Message}");
            }
        }

        /// <summary>
        /// Accept voxels into the persisted map. Rebuild is deferred until enough accumulate.
        /// Uses adjacent-face culling: only renders faces not touching another voxel (Minecraft-style).
        /// </summary>
        private void UpdateMeshVisualVoxels(MeshDataModel meshData)
        {
            double vs = meshData.VoxelSize > 0 ? meshData.VoxelSize : 0.15;

            foreach (var voxel in meshData.Voxels)
            {
                if (voxel.Position == null || voxel.Position.Count < 3)
                    continue;

                // Negate Y and Z before quantising: converts ROS odom (Y-down, Z-forward)
                // into WPF grid space (Y-up, Z-toward viewer) so all grid ops stay consistent.
                int qx = (int)Math.Round(voxel.Position[0] / vs);
                int qy = (int)Math.Round(-voxel.Position[1] / vs);
                int qz = (int)Math.Round(-voxel.Position[2] / vs);
                long key = PackVoxelKey(qx, qy, qz);

                // Flat color: quantize to 4-bit per channel (16 levels)
                uint colorKey;
                if (voxel.Color != null && voxel.Color.Count >= 3)
                {
                    byte r = (byte)(((voxel.Color[0] >> 4) & 0x0F) * 17);
                    byte g = (byte)(((voxel.Color[1] >> 4) & 0x0F) * 17);
                    byte b = (byte)(((voxel.Color[2] >> 4) & 0x0F) * 17);
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

            // Evict oldest entries via FIFO queue
            if (_persistedBlocks.Count > MaxPersistedVoxels)
            {
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
                        i--; // key was already removed (color update replaced it), skip
                    }
                }
                _meshDirty = true;
            }

            if (!_meshDirty) return;

            // Only defer rebuild when voxels are purely additive (count grew)
            // Always rebuild immediately on eviction (count shrank) or color changes
            int newSinceLastRender = _persistedBlocks.Count - _lastRenderedCount;
            if (newSinceLastRender > 0 && newSinceLastRender < MinNewVoxelsForRebuild && _lastRenderedCount > 0)
                return;

            // Time-based debounce: max ~4 rebuilds/sec (P3-7)
            if (DateTime.UtcNow - _lastMeshRebuild < MinRebuildInterval)
                return;

            _lastMeshRebuild = DateTime.UtcNow;
            _meshDirty = false;
            _lastRenderedCount = _persistedBlocks.Count;

            // Group by color for batched draw calls
            _meshModelGroup.Children.Clear();

            var colorGroups = new Dictionary<uint, List<long>>();
            foreach (var kvp in _persistedBlocks)
            {
                if (!colorGroups.ContainsKey(kvp.Value))
                    colorGroups[kvp.Value] = new List<long>();
                colorGroups[kvp.Value].Add(kvp.Key);
            }

            double half = vs * 0.5;

            foreach (var cg in colorGroups)
            {
                uint ck = cg.Key;
                byte cr = (byte)((ck >> 16) & 0xFF);
                byte cg2 = (byte)((ck >> 8) & 0xFF);
                byte cb = (byte)(ck & 0xFF);
                if (ck == uint.MaxValue) { cr = 144; cg2 = 144; cb = 160; }

                var positions = new Point3DCollection();
                var indices = new Int32Collection();
                int offset = 0;

                foreach (var key in cg.Value)
                {
                    UnpackVoxelKey(key, out int ix, out int iy, out int iz);
                    double cx = ix * vs;
                    double cy = iy * vs;
                    double cz = iz * vs;

                    // Check each face: only emit if neighbor is absent
                    // +X face
                    if (!_occupancySet.Contains(PackKey(ix+1, iy, iz)))
                    {
                        positions.Add(new Point3D(cx+half, cy-half, cz-half));
                        positions.Add(new Point3D(cx+half, cy+half, cz-half));
                        positions.Add(new Point3D(cx+half, cy+half, cz+half));
                        positions.Add(new Point3D(cx+half, cy-half, cz+half));
                        indices.Add(offset); indices.Add(offset+1); indices.Add(offset+2);
                        indices.Add(offset); indices.Add(offset+2); indices.Add(offset+3);
                        offset += 4;
                    }
                    // -X face
                    if (!_occupancySet.Contains(PackKey(ix-1, iy, iz)))
                    {
                        positions.Add(new Point3D(cx-half, cy-half, cz-half));
                        positions.Add(new Point3D(cx-half, cy-half, cz+half));
                        positions.Add(new Point3D(cx-half, cy+half, cz+half));
                        positions.Add(new Point3D(cx-half, cy+half, cz-half));
                        indices.Add(offset); indices.Add(offset+1); indices.Add(offset+2);
                        indices.Add(offset); indices.Add(offset+2); indices.Add(offset+3);
                        offset += 4;
                    }
                    // +Y face
                    if (!_occupancySet.Contains(PackKey(ix, iy+1, iz)))
                    {
                        positions.Add(new Point3D(cx-half, cy+half, cz-half));
                        positions.Add(new Point3D(cx-half, cy+half, cz+half));
                        positions.Add(new Point3D(cx+half, cy+half, cz+half));
                        positions.Add(new Point3D(cx+half, cy+half, cz-half));
                        indices.Add(offset); indices.Add(offset+1); indices.Add(offset+2);
                        indices.Add(offset); indices.Add(offset+2); indices.Add(offset+3);
                        offset += 4;
                    }
                    // -Y face
                    if (!_occupancySet.Contains(PackKey(ix, iy-1, iz)))
                    {
                        positions.Add(new Point3D(cx-half, cy-half, cz-half));
                        positions.Add(new Point3D(cx+half, cy-half, cz-half));
                        positions.Add(new Point3D(cx+half, cy-half, cz+half));
                        positions.Add(new Point3D(cx-half, cy-half, cz+half));
                        indices.Add(offset); indices.Add(offset+1); indices.Add(offset+2);
                        indices.Add(offset); indices.Add(offset+2); indices.Add(offset+3);
                        offset += 4;
                    }
                    // +Z face (top)
                    if (!_occupancySet.Contains(PackKey(ix, iy, iz+1)))
                    {
                        positions.Add(new Point3D(cx-half, cy-half, cz+half));
                        positions.Add(new Point3D(cx+half, cy-half, cz+half));
                        positions.Add(new Point3D(cx+half, cy+half, cz+half));
                        positions.Add(new Point3D(cx-half, cy+half, cz+half));
                        indices.Add(offset); indices.Add(offset+1); indices.Add(offset+2);
                        indices.Add(offset); indices.Add(offset+2); indices.Add(offset+3);
                        offset += 4;
                    }
                    // -Z face (bottom)
                    if (!_occupancySet.Contains(PackKey(ix, iy, iz-1)))
                    {
                        positions.Add(new Point3D(cx-half, cy-half, cz-half));
                        positions.Add(new Point3D(cx-half, cy+half, cz-half));
                        positions.Add(new Point3D(cx+half, cy+half, cz-half));
                        positions.Add(new Point3D(cx+half, cy-half, cz-half));
                        indices.Add(offset); indices.Add(offset+1); indices.Add(offset+2);
                        indices.Add(offset); indices.Add(offset+2); indices.Add(offset+3);
                        offset += 4;
                    }
                }

                if (positions.Count == 0) continue;

                positions.Freeze();
                indices.Freeze();

                var geom = new MeshGeometry3D
                {
                    Positions = positions,
                    TriangleIndices = indices,
                };
                geom.Freeze();

                // Opaque DiffuseMaterial: responds to scene lighting for natural shading
                Material mat;
                if (!_materialCache.TryGetValue(ck, out mat))
                {
                    var brush = new SolidColorBrush(Color.FromArgb(255, cr, cg2, cb));
                    brush.Freeze();
                    mat = new DiffuseMaterial(brush);
                    ((DiffuseMaterial)mat).Freeze();
                    _materialCache[ck] = mat;
                }

                var model = new GeometryModel3D(geom, mat);
                model.Freeze();
                _meshModelGroup.Children.Add(model);
            }
        }

        /// <summary>
        /// Render block-only cubes (fallback when per-voxel data is not available).
        /// Uses adjacent-face culling.
        /// </summary>
        private void UpdateMeshVisualBlocks(MeshDataModel meshData)
        {
                double bs = meshData.BlockSize > 0 ? meshData.BlockSize : 0.05;
                double half = bs * 0.5; // Full size for seamless joins

                // Merge incoming blocks into persisted map
                bool hasNewBlocks = false;
                foreach (var block in meshData.Blocks)
                {
                    if (block.Index == null || block.Index.Count < 3)
                        continue;

                    // Negate Y and Z: ROS odom (Y-down, Z-forward) → WPF grid (Y-up, Z-toward viewer)
                    int bix = block.Index[0];
                    int biy = -block.Index[1];
                    int biz = -block.Index[2];

                    long key = PackVoxelKey(bix, biy, biz);
                    uint colorKey;
                    if (block.Color != null && block.Color.Count >= 3)
                    {
                        byte r = (byte)(((block.Color[0] >> 4) & 0x0F) * 17);
                        byte g = (byte)(((block.Color[1] >> 4) & 0x0F) * 17);
                        byte b = (byte)(((block.Color[2] >> 4) & 0x0F) * 17);
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
                        hasNewBlocks = true;
                    }
                }

                if (!hasNewBlocks)
                    return;

                _meshModelGroup.Children.Clear();

                var colorGroups = new Dictionary<uint, List<int[]>>();
                foreach (var kvp in _persistedBlocks)
                {
                    UnpackVoxelKey(kvp.Key, out int bx, out int by, out int bz);
                    int[] idx = { bx, by, bz };
                    if (!colorGroups.ContainsKey(kvp.Value))
                        colorGroups[kvp.Value] = new List<int[]>();
                    colorGroups[kvp.Value].Add(idx);
                }

                foreach (var kvp in colorGroups)
                {
                    uint ck = kvp.Key;
                    byte r = (byte)((ck >> 16) & 0xFF);
                    byte g = (byte)((ck >> 8) & 0xFF);
                    byte b = (byte)(ck & 0xFF);
                    if (ck == uint.MaxValue) { r = 144; g = 144; b = 160; }

                    var positions = new Point3DCollection();
                    var indices = new Int32Collection();
                    int gOffset = 0;

                    foreach (var idx in kvp.Value)
                    {
                        int ix = idx[0], iy = idx[1], iz = idx[2];
                        double cx = ix * bs;
                        double cy = iy * bs;
                        double cz = iz * bs;

                        // Adjacent-face culling: only emit faces where no neighbor exists
                        if (!_occupancySet.Contains(PackKey(ix+1, iy, iz)))
                        {
                            positions.Add(new Point3D(cx+half, cy-half, cz-half));
                            positions.Add(new Point3D(cx+half, cy+half, cz-half));
                            positions.Add(new Point3D(cx+half, cy+half, cz+half));
                            positions.Add(new Point3D(cx+half, cy-half, cz+half));
                            indices.Add(gOffset); indices.Add(gOffset+1); indices.Add(gOffset+2);
                            indices.Add(gOffset); indices.Add(gOffset+2); indices.Add(gOffset+3);
                            gOffset += 4;
                        }
                        if (!_occupancySet.Contains(PackKey(ix-1, iy, iz)))
                        {
                            positions.Add(new Point3D(cx-half, cy-half, cz-half));
                            positions.Add(new Point3D(cx-half, cy-half, cz+half));
                            positions.Add(new Point3D(cx-half, cy+half, cz+half));
                            positions.Add(new Point3D(cx-half, cy+half, cz-half));
                            indices.Add(gOffset); indices.Add(gOffset+1); indices.Add(gOffset+2);
                            indices.Add(gOffset); indices.Add(gOffset+2); indices.Add(gOffset+3);
                            gOffset += 4;
                        }
                        if (!_occupancySet.Contains(PackKey(ix, iy+1, iz)))
                        {
                            positions.Add(new Point3D(cx-half, cy+half, cz-half));
                            positions.Add(new Point3D(cx-half, cy+half, cz+half));
                            positions.Add(new Point3D(cx+half, cy+half, cz+half));
                            positions.Add(new Point3D(cx+half, cy+half, cz-half));
                            indices.Add(gOffset); indices.Add(gOffset+1); indices.Add(gOffset+2);
                            indices.Add(gOffset); indices.Add(gOffset+2); indices.Add(gOffset+3);
                            gOffset += 4;
                        }
                        if (!_occupancySet.Contains(PackKey(ix, iy-1, iz)))
                        {
                            positions.Add(new Point3D(cx-half, cy-half, cz-half));
                            positions.Add(new Point3D(cx+half, cy-half, cz-half));
                            positions.Add(new Point3D(cx+half, cy-half, cz+half));
                            positions.Add(new Point3D(cx-half, cy-half, cz+half));
                            indices.Add(gOffset); indices.Add(gOffset+1); indices.Add(gOffset+2);
                            indices.Add(gOffset); indices.Add(gOffset+2); indices.Add(gOffset+3);
                            gOffset += 4;
                        }
                        if (!_occupancySet.Contains(PackKey(ix, iy, iz+1)))
                        {
                            positions.Add(new Point3D(cx-half, cy-half, cz+half));
                            positions.Add(new Point3D(cx+half, cy-half, cz+half));
                            positions.Add(new Point3D(cx+half, cy+half, cz+half));
                            positions.Add(new Point3D(cx-half, cy+half, cz+half));
                            indices.Add(gOffset); indices.Add(gOffset+1); indices.Add(gOffset+2);
                            indices.Add(gOffset); indices.Add(gOffset+2); indices.Add(gOffset+3);
                            gOffset += 4;
                        }
                        if (!_occupancySet.Contains(PackKey(ix, iy, iz-1)))
                        {
                            positions.Add(new Point3D(cx-half, cy-half, cz-half));
                            positions.Add(new Point3D(cx-half, cy+half, cz-half));
                            positions.Add(new Point3D(cx+half, cy+half, cz-half));
                            positions.Add(new Point3D(cx+half, cy-half, cz-half));
                            indices.Add(gOffset); indices.Add(gOffset+1); indices.Add(gOffset+2);
                            indices.Add(gOffset); indices.Add(gOffset+2); indices.Add(gOffset+3);
                            gOffset += 4;
                        }
                    }

                    if (positions.Count == 0) continue;

                    positions.Freeze();
                    indices.Freeze();

                    var geom = new MeshGeometry3D
                    {
                        Positions = positions,
                        TriangleIndices = indices,
                    };
                    geom.Freeze();

                    // Opaque DiffuseMaterial: responds to scene lighting for depth
                    Material mat;
                    if (!_materialCache.TryGetValue(ck, out mat))
                    {
                        var brush = new SolidColorBrush(Color.FromArgb(255, r, g, b));
                        brush.Freeze();
                        mat = new DiffuseMaterial(brush);
                        ((DiffuseMaterial)mat).Freeze();
                        _materialCache[ck] = mat;
                    }

                    var model = new GeometryModel3D(geom, mat);
                    model.Freeze();
                    _meshModelGroup.Children.Add(model);
                }
        }
        
        private void UpdateDroneVisual(Point3D pos, double yaw, double pitch, double roll)
        {
            if (_droneVisual == null) return;
            
            // In ZED odom (X-right, Y-down, Z-forward), _quat_to_euler gives:
            //   roll  = rotation around X = physical camera tilt (elevation, up/down)
            //   pitch = rotation around Y = physical heading (left/right pan)
            //   yaw   = rotation around Z = physical optical roll
            // WPF frame is Y-up, Z-toward viewer, so Y and Z senses are inverted.
            // Transform3DGroup order [RotZ, RotX, RotY] = extrinsic ZXY = intrinsic YXZ
            // (heading first, then elevation, then optical roll) when values are assigned:
            _droneRotY.Angle = -pitch * 180.0 / Math.PI;  // heading (pan) → WPF Y rotation
            _droneRotX.Angle = roll * 180.0 / Math.PI;    // elevation (tilt) → WPF X rotation
            _droneRotZ.Angle = -yaw * 180.0 / Math.PI;    // optical roll → WPF Z rotation
            _droneTranslation.OffsetX = pos.X;
            _droneTranslation.OffsetY = -pos.Y;
            _droneTranslation.OffsetZ = -pos.Z;
        }
        
        private void UpdateTrajectoryVisual()
        {
            if (_trajectoryVisual == null || !_chkShowTrajectory.Checked || _trajectoryPoints.Count < 2)
                return;
            
            // Skip if no new points since last render
            if (_trajectoryPoints.Count == _lastTrajectoryRendered)
                return;

            try
            {
                var positions = new Point3DCollection();
                var indices = new Int32Collection();
                int offset = 0;
                double r = 0.015;

                for (int i = 1; i < _trajectoryPoints.Count; i++)
                {
                    // Convert stored ROS points to WPF frame at render time
                    var rosP0 = _trajectoryPoints[i - 1];
                    var rosP1 = _trajectoryPoints[i];
                    var p0 = new Point3D(rosP0.X, -rosP0.Y, -rosP0.Z);
                    var p1 = new Point3D(rosP1.X, -rosP1.Y, -rosP1.Z);
                    var dir = p1 - p0;
                    if (dir.Length < 0.001) continue;

                    var up = new Vector3D(0, 1, 0); // WPF Y-up
                    var right = Vector3D.CrossProduct(dir, up);
                    if (right.Length < 0.001) { right = Vector3D.CrossProduct(dir, new Vector3D(1, 0, 0)); }
                    right.Normalize();
                    right *= r;

                    positions.Add(p0 + right);
                    positions.Add(p0 - right);
                    positions.Add(p1 - right);
                    positions.Add(p1 + right);

                    indices.Add(offset); indices.Add(offset+1); indices.Add(offset+2);
                    indices.Add(offset); indices.Add(offset+2); indices.Add(offset+3);
                    offset += 4;
                }

                if (positions.Count == 0) return;

                _lastTrajectoryRendered = _trajectoryPoints.Count;

                positions.Freeze();
                indices.Freeze();

                var geom = new MeshGeometry3D
                {
                    Positions = positions,
                    TriangleIndices = indices,
                };
                geom.Freeze();

                // Reuse cached material
                Material mat;
                uint trajColorKey = 0xFFC800FF; // yellow marker
                if (!_materialCache.TryGetValue(trajColorKey, out mat))
                {
                    var brush = new SolidColorBrush(Color.FromArgb(255, 255, 200, 0));
                    brush.Freeze();
                    mat = new DiffuseMaterial(brush);
                    ((DiffuseMaterial)mat).Freeze();
                    _materialCache[trajColorKey] = mat;
                }

                var trajectoryModel = new GeometryModel3D(geom, mat);
                trajectoryModel.BackMaterial = mat;
                trajectoryModel.Freeze();
                
                var group = new Model3DGroup();
                group.Children.Add(trajectoryModel);
                _trajectoryVisual.Content = group;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Trajectory update error: {ex.Message}");
            }
        }
        
        private void UpdateCameras(Point3D pos, double yaw, double pitch, double roll)
        {
            switch (_currentViewMode)
            {
                case CameraViewMode.FirstPerson:
                    UpdateFPVCamera(pos, yaw, pitch, roll);
                    // Hide drone arrow in FPV so it doesn't obscure the view
                    if (_droneVisual != null)
                        _droneVisual.Content = new Model3DGroup();
                    break;
                case CameraViewMode.ThirdPerson:
                    UpdateTPVCamera(pos, pitch);  // pitch = physical heading in ZED odom frame
                    RestoreDroneVisual();
                    break;
                case CameraViewMode.FreeOrbit:
                    RestoreDroneVisual();
                    break;
            }
        }
        
        private void UpdateFPVCamera(Point3D pos, double yaw, double pitch, double roll)
        {
            // ROS-to-WPF frame conversion
            var wpfPos = new Point3D(pos.X, -pos.Y, -pos.Z);
            _fpvCamera.Position = wpfPos;

            // In ZED odom: pitch=heading, roll=elevation, yaw=optical roll.
            // WPF: Y-up, Z-toward-viewer.  Forward = (0,0,-1) in WPF.
            // lookDir formula: heading around Y-up, elevation around X-right.
            double cosY = Math.Cos(-pitch), sinY = Math.Sin(-pitch);   // heading (negated: odom Y-down → WPF Y-up)
            double cosP = Math.Cos(-roll),  sinP = Math.Sin(-roll);    // elevation (negated for WPF sign convention)

            var lookDir = new Vector3D(
                -sinY * cosP,
                -sinP,
                -cosY * cosP
            );
            _fpvCamera.LookDirection = lookDir;

            // Up direction: optical roll around look-direction
            double cosR = Math.Cos(yaw), sinR = Math.Sin(yaw);  // yaw = physical optical roll
            var right = Vector3D.CrossProduct(lookDir, new Vector3D(0, 1, 0));
            if (right.Length < 0.001) right = new Vector3D(1, 0, 0);
            right.Normalize();
            var baseUp = Vector3D.CrossProduct(right, lookDir);
            baseUp.Normalize();
            // Apply roll around look direction
            _fpvCamera.UpDirection = baseUp * cosR + right * sinR;
        }

        private void RestoreDroneVisual()
        {
            if (_droneVisual != null && _droneModelContent != null &&
                _droneVisual.Content != _droneModelContent)
            {
                _droneVisual.Content = _droneModelContent;
            }
        }
        
        private void UpdateTPVCamera(Point3D pos, double heading)
        {
            // ROS-to-WPF frame conversion
            var wpfPos = new Point3D(pos.X, -pos.Y, -pos.Z);
            double distance = 3.0, height = 1.5;
            // heading is the physical heading (ros_pitch in ZED odom), negated for WPF Y-up
            double cosY = Math.Cos(-heading), sinY = Math.Sin(-heading);
            // Behind the drone in WPF frame (forward is -Z in WPF)
            var camPos = new Point3D(
                wpfPos.X + sinY * distance,
                wpfPos.Y + height,
                wpfPos.Z + cosY * distance
            );
            _tpvCamera.Position = camPos;
            _tpvCamera.LookDirection = wpfPos - camPos;
        }

        // ==================== Event Handlers ====================
        
        private void BtnToggleCamera_Click(object sender, EventArgs e)
        {
            // Cycle through camera modes
            _currentViewMode = _currentViewMode switch
            {
                CameraViewMode.ThirdPerson => CameraViewMode.FirstPerson,
                CameraViewMode.FirstPerson => CameraViewMode.FreeOrbit,
                CameraViewMode.FreeOrbit => CameraViewMode.ThirdPerson,
                _ => CameraViewMode.ThirdPerson
            };
            
            // Update camera
            _elementHost.Invoke(new Action(() =>
            {
                // When switching to FreeOrbit, seed from current active camera
                if (_currentViewMode == CameraViewMode.FreeOrbit)
                {
                    var prevCam = _viewport.Camera as PerspectiveCamera;
                    if (prevCam != null)
                    {
                        _orbitCamera.Position = prevCam.Position;
                        _orbitCamera.LookDirection = prevCam.LookDirection;
                        _orbitCamera.UpDirection = prevCam.UpDirection;
                    }
                }
                _viewport.Camera = _currentViewMode switch
                {
                    CameraViewMode.FirstPerson => _fpvCamera,
                    CameraViewMode.ThirdPerson => _tpvCamera,
                    CameraViewMode.FreeOrbit => _orbitCamera,
                    _ => _tpvCamera
                };
            }));
            
            // Update button text
            string modeName = _currentViewMode switch
            {
                CameraViewMode.FirstPerson => "FPV",
                CameraViewMode.ThirdPerson => "TPV",
                CameraViewMode.FreeOrbit => "Orbit",
                _ => "Unknown"
            };
            _btnToggleCamera.Text = $"Toggle View ({modeName})";
        }
        
        private void BtnResetView_Click(object sender, EventArgs e)
        {
            _elementHost.Invoke(new Action(() =>
            {
                // Reset orbit camera (WPF Y-up frame)
                _orbitCamera.Position = new Point3D(8, 6, 10);
                _orbitCamera.LookDirection = new Vector3D(-8, -4, -10);
                
                // Reset TPV (WPF Y-up frame)
                _tpvCamera.Position = new Point3D(0, 5, 10);
                _tpvCamera.LookDirection = new Vector3D(0, -3, -8);
                
                _viewport.ZoomExtents();
            }));
        }
        
        private async void BtnClearMesh_Click(object sender, EventArgs e)
        {
            // Always clear local data regardless of API success
            _elementHost.Invoke(new Action(() =>
            {
                _meshModelGroup.Children.Clear();
                _persistedBlocks.Clear();
                _occupancySet.Clear();
                _materialCache.Clear();
                _meshDirty = false;
                _lastRenderedCount = 0;
                var emptyGroup = new Model3DGroup();
                _trajectoryVisual.Content = emptyGroup;
            }));
            _trajectoryPoints.Clear();
            _lastTrajectoryRendered = 0;
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
        
        private void ChkShowGrid_CheckedChanged(object sender, EventArgs e)
        {
            if (_gridVisual != null)
            {
                _elementHost.Invoke(new Action(() =>
                {
                    if (_chkShowGrid.Checked)
                    {
                        if (!_viewport.Children.Contains(_gridVisual))
                            _viewport.Children.Add(_gridVisual);
                    }
                    else
                    {
                        _viewport.Children.Remove(_gridVisual);
                    }
                }));
            }
        }
        
        private void ChkShowTrajectory_CheckedChanged(object sender, EventArgs e)
        {
            if (_trajectoryVisual != null)
            {
                _elementHost.Invoke(new Action(() =>
                {
                    if (_chkShowTrajectory.Checked)
                    {
                        if (!_viewport.Children.Contains(_trajectoryVisual))
                            _viewport.Children.Add(_trajectoryVisual);
                    }
                    else
                    {
                        _viewport.Children.Remove(_trajectoryVisual);
                    }
                }));
            }
        }

        // ==================== Helpers ====================
        
        private void UpdateStatusSafe(string text)
        {
            if (_lblStatus == null) return;
            
            if (_lblStatus.InvokeRequired)
            {
                _lblStatus.BeginInvoke(new Action(() => _lblStatus.Text = text));
            }
            else
            {
                _lblStatus.Text = text;
            }
        }
        
        private void UpdateStatsSafe(string text)
        {
            if (_lblStats == null) return;
            
            if (_lblStats.InvokeRequired)
            {
                _lblStats.BeginInvoke(new Action(() => _lblStats.Text = text));
            }
            else
            {
                _lblStats.Text = text;
            }
        }

        // ==================== Cleanup ====================
        
        protected override void Dispose(bool disposing)
        {
            _disposed = true;
            if (disposing)
            {
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
                _elementHost?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
