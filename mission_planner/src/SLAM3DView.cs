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
using System.Linq;
using System.Net.Http;
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

    /// <summary>
    /// Data model for SLAM mesh received from Jetson API.
    /// </summary>
    public class SLAMMeshData
    {
        [JsonProperty("available")]
        public bool Available { get; set; }
        [JsonProperty("error")]
        public string Error { get; set; }
        [JsonProperty("timestamp")]
        public string Timestamp { get; set; }
        [JsonProperty("mesh")]
        public MeshDataModel Mesh { get; set; }
        [JsonProperty("drone_position")]
        public DronePositionModel DronePosition { get; set; }
        [JsonProperty("drone_attitude")]
        public DroneAttitudeModel DroneAttitude { get; set; }
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
        // Per-voxel mode (mode == "voxels")
        [JsonProperty("voxels")]
        public List<VoxelModel> Voxels { get; set; }
        [JsonProperty("voxel_size")]
        public double VoxelSize { get; set; }
        [JsonProperty("total_voxels")]
        public int TotalVoxels { get; set; }
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

    public class MeshBlockModel
    {
        [JsonProperty("index")]
        public List<int> Index { get; set; }
        /// <summary>Block-only mode: average [R, G, B] color (0-255).</summary>
        [JsonProperty("color")]
        public List<int> Color { get; set; }
    }

    public class DronePositionModel
    {
        [JsonProperty("x")]
        public double X { get; set; }
        [JsonProperty("y")]
        public double Y { get; set; }
        [JsonProperty("z")]
        public double Z { get; set; }
    }

    public class DroneAttitudeModel
    {
        [JsonProperty("roll")]
        public double Roll { get; set; }
        [JsonProperty("pitch")]
        public double Pitch { get; set; }
        [JsonProperty("yaw")]
        public double Yaw { get; set; }
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
        private List<Point3D> _trajectoryPoints = new List<Point3D>();
        private const int MaxTrajectoryPoints = 500;
        
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
        
        // Persisted block map: key = "ix,iy,iz", value = packed ARGB color
        private Dictionary<string, uint> _persistedBlocks = new Dictionary<string, uint>();
        private const int MaxPersistedVoxels = 2000; // Hard cap: prevents WPF overload

        // Material cache to avoid recreating WPF resources every frame
        private Dictionary<uint, Material> _materialCache = new Dictionary<uint, Material>();

        // Dirty tracking: only rebuild mesh when significant changes arrive
        private bool _meshDirty = false;
        private int _lastRenderedCount = 0;
        private const int MinNewVoxelsForRebuild = 100; // Only rebuild after 100+ new voxels

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
            _tpvCamera = new PerspectiveCamera
            {
                Position = new Point3D(-5, -5, 5),
                LookDirection = new Vector3D(5, 5, -3),
                UpDirection = new Vector3D(0, 0, 1),
                FieldOfView = 60,
                NearPlaneDistance = 0.1,
                FarPlaneDistance = 1000,
            };
            
            _fpvCamera = new PerspectiveCamera
            {
                Position = new Point3D(0, 0, 0),
                LookDirection = new Vector3D(1, 0, 0),
                UpDirection = new Vector3D(0, 0, 1),
                FieldOfView = 90,
                NearPlaneDistance = 0.01,
                FarPlaneDistance = 500,
            };
            
            _orbitCamera = new PerspectiveCamera
            {
                Position = new Point3D(-10, -10, 8),
                LookDirection = new Vector3D(10, 10, -5),
                UpDirection = new Vector3D(0, 0, 1),
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
            
            // Host in WinForms
            _elementHost.Child = _viewport;
        }
        
        private void CreateGrid()
        {
            var gridBuilder = new MeshBuilder();
            
            // Create grid lines
            double gridSize = 20;
            int gridLines = 40;
            double spacing = gridSize * 2 / gridLines;
            
            for (int i = 0; i <= gridLines; i++)
            {
                double pos = -gridSize + i * spacing;
                
                // X lines
                gridBuilder.AddPipe(
                    new Point3D(pos, -gridSize, 0),
                    new Point3D(pos, gridSize, 0),
                    0, 0.01, 8
                );
                
                // Y lines
                gridBuilder.AddPipe(
                    new Point3D(-gridSize, pos, 0),
                    new Point3D(gridSize, pos, 0),
                    0, 0.01, 8
                );
            }
            
            var gridMaterial = new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(60, 100, 100, 100)));
            var gridModel = new GeometryModel3D(gridBuilder.ToMesh(), gridMaterial);
            
            _gridVisual = new ModelVisual3D { Content = gridModel };
            _viewport.Children.Add(_gridVisual);
        }
        
        private void CreateDroneMarker()
        {
            var droneBuilder = new MeshBuilder();
            
            // Small arrow only - proportional to mesh voxels
            droneBuilder.AddArrow(
                new Point3D(-0.04, 0, 0),
                new Point3D(0.08, 0, 0),
                0.015, 3
            );
            
            // Create material - bright cyan for visibility
            var droneMaterial = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(0, 220, 220)));
            var droneModel = new GeometryModel3D(droneBuilder.ToMesh(), droneMaterial);
            
            var droneGroup = new Model3DGroup();
            droneGroup.Children.Add(droneModel);
            
            _droneVisual = new ModelVisual3D { Content = droneGroup };
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

        // ==================== Update Loop ====================
        
        private void StartUpdateLoop()
        {
            _updateCts = new CancellationTokenSource();
            Task.Run(() => MeshUpdateLoop(_updateCts.Token));
        }
        
        private async Task MeshUpdateLoop(CancellationToken ct)
        {
            while (!ct.IsCancellationRequested)
            {
                try
                {
                    if (_chkAutoUpdate?.Checked == true)
                    {
                        await FetchAndUpdateMesh();
                    }
                    
                    // 0.5 Hz update rate (2s) to reduce CPU/GC pressure
                    await Task.Delay(2000, ct);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    UpdateStatusSafe($"Error: {ex.Message}");
                    await Task.Delay(1000, ct); // Back off on error
                }
            }
        }
        
        private async Task FetchAndUpdateMesh()
        {
            try
            {
                var response = await JetsonApiService.GetStringAsync("/api/task/2/slam/mesh");
                var data = JsonConvert.DeserializeObject<SLAMMeshData>(response);
                
                if (data == null)
                {
                    UpdateStatusSafe("Received null data");
                    return;
                }
                
                if (!data.Available)
                {
                    UpdateStatusSafe($"SLAM unavailable: {data.Error ?? "Unknown"}");
                    return;
                }
                
                // Update mesh on UI thread
                if (this.InvokeRequired)
                {
                    this.BeginInvoke(new Action(() => ProcessMeshData(data)));
                }
                else
                {
                    ProcessMeshData(data);
                }
            }
            catch (HttpRequestException ex)
            {
                UpdateStatusSafe($"Connection error: {ex.Message}");
            }
            catch (TaskCanceledException)
            {
                UpdateStatusSafe("Request timeout");
            }
            catch (Exception ex)
            {
                UpdateStatusSafe($"Fetch error: {ex.Message}");
            }
        }
        
        private void ProcessMeshData(SLAMMeshData data)
        {
            try
            {
                // Update drone position
                if (data.DronePosition != null)
                {
                    _dronePosition = new Point3D(
                        data.DronePosition.X,
                        data.DronePosition.Y,
                        data.DronePosition.Z
                    );
                    
                    // Add to trajectory
                    _trajectoryPoints.Add(_dronePosition);
                    if (_trajectoryPoints.Count > MaxTrajectoryPoints)
                    {
                        _trajectoryPoints.RemoveAt(0);
                    }
                }
                
                if (data.DroneAttitude != null)
                {
                    _droneYaw = data.DroneAttitude.Yaw;
                    _dronePitch = data.DroneAttitude.Pitch;
                    _droneRoll = data.DroneAttitude.Roll;
                }
                
                // Update 3D visuals on WPF dispatcher
                _elementHost.Invoke(new Action(() =>
                {
                    UpdateDroneVisual();
                    UpdateTrajectoryVisual();
                    UpdateCameras();
                    
                    if (data.Mesh != null)
                    {
                        UpdateMeshVisual(data.Mesh);
                    }
                }));
                
                _meshUpdateCount++;
                _lastUpdateTime = DateTime.Now;
                
                // Update stats
                if (data.Mesh != null)
                {
                    _totalBlocks = data.Mesh.TotalBlocks > 0 ? data.Mesh.TotalBlocks : data.Mesh.TotalVoxels;
                }

                string mode = data.Mesh?.Mode == "voxels" ? "voxels" : "blocks";
                UpdateStatusSafe($"Status: Connected | Updates: {_meshUpdateCount}");
                UpdateStatsSafe($"Mesh: {_totalBlocks:N0} {mode} ({_persistedBlocks.Count:N0} cached)");
            }
            catch (Exception ex)
            {
                UpdateStatusSafe($"Process error: {ex.Message}");
            }
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
                    _materialCache.Clear();
                    _meshDirty = false;
                    _lastRenderedCount = 0;
                }

                // Route to the appropriate renderer based on mode
                if (meshData?.Mode == "voxels" && meshData.Voxels != null && meshData.Voxels.Count > 0)
                {
                    UpdateMeshVisualVoxels(meshData);
                }
                else if (meshData?.Blocks != null && meshData.Blocks.Count > 0)
                {
                    UpdateMeshVisualBlocks(meshData);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Mesh update error: {ex.Message}");
            }
        }

        /// <summary>
        /// Accept voxels into the persisted map. Rebuild is deferred until enough accumulate.
        /// </summary>
        private void UpdateMeshVisualVoxels(MeshDataModel meshData)
        {
            double vs = meshData.VoxelSize > 0 ? meshData.VoxelSize : 0.15;

            foreach (var voxel in meshData.Voxels)
            {
                if (voxel.Position == null || voxel.Position.Count < 3)
                    continue;

                int qx = (int)Math.Round(voxel.Position[0] / vs);
                int qy = (int)Math.Round(voxel.Position[1] / vs);
                int qz = (int)Math.Round(voxel.Position[2] / vs);
                string key = $"{qx},{qy},{qz}";

                // Quantize color to 4-bit per channel (16 levels) to reduce unique colors
                uint colorKey;
                if (voxel.Color != null && voxel.Color.Count >= 3)
                {
                    byte r = (byte)((voxel.Color[0] >> 4) << 4);
                    byte g = (byte)((voxel.Color[1] >> 4) << 4);
                    byte b = (byte)((voxel.Color[2] >> 4) << 4);
                    colorKey = ((uint)r << 16) | ((uint)g << 8) | (uint)b;
                }
                else
                    colorKey = (144u << 16) | (144u << 8) | 160u;

                if (!_persistedBlocks.ContainsKey(key) || _persistedBlocks[key] != colorKey)
                {
                    _persistedBlocks[key] = colorKey;
                    _meshDirty = true;
                }
            }

            // Evict oldest entries if over cap
            if (_persistedBlocks.Count > MaxPersistedVoxels)
            {
                int toRemove = _persistedBlocks.Count - MaxPersistedVoxels;
                var keysToRemove = _persistedBlocks.Keys.Take(toRemove).ToList();
                foreach (var k in keysToRemove)
                    _persistedBlocks.Remove(k);
                _meshDirty = true;
            }

            if (!_meshDirty) return;

            // Only rebuild when enough new voxels accumulated to justify the cost
            int newSinceLastRender = _persistedBlocks.Count - _lastRenderedCount;
            if (newSinceLastRender < MinNewVoxelsForRebuild && _lastRenderedCount > 0)
                return;

            _meshDirty = false;
            _lastRenderedCount = _persistedBlocks.Count;

            // Full rebuild: one MeshGeometry3D per unique color (single draw call per color)
            // With 4-bit quantization, max ~4096 unique colors, typically 20-50 in practice
            _meshModelGroup.Children.Clear();

            var colorGroups = new Dictionary<uint, List<string>>();
            foreach (var kvp in _persistedBlocks)
            {
                if (!colorGroups.ContainsKey(kvp.Value))
                    colorGroups[kvp.Value] = new List<string>();
                colorGroups[kvp.Value].Add(kvp.Key);
            }

            double half = vs * 0.5;
            foreach (var cg in colorGroups)
            {
                uint ck = cg.Key;
                byte cr = (byte)((ck >> 16) & 0xFF);
                byte cg2 = (byte)((ck >> 8) & 0xFF);
                byte cb = (byte)(ck & 0xFF);
                if (cr == 0 && cg2 == 0 && cb == 0) { cr = 144; cg2 = 144; cb = 160; }

                var geom = new MeshGeometry3D();
                int offset = 0;

                foreach (var key in cg.Value)
                {
                    string[] parts = key.Split(',');
                    double cx = int.Parse(parts[0]) * vs;
                    double cy = int.Parse(parts[1]) * vs;
                    double cz = int.Parse(parts[2]) * vs;

                    // 8 vertices per cube
                    geom.Positions.Add(new Point3D(cx - half, cy - half, cz - half));
                    geom.Positions.Add(new Point3D(cx + half, cy - half, cz - half));
                    geom.Positions.Add(new Point3D(cx + half, cy + half, cz - half));
                    geom.Positions.Add(new Point3D(cx - half, cy + half, cz - half));
                    geom.Positions.Add(new Point3D(cx - half, cy - half, cz + half));
                    geom.Positions.Add(new Point3D(cx + half, cy - half, cz + half));
                    geom.Positions.Add(new Point3D(cx + half, cy + half, cz + half));
                    geom.Positions.Add(new Point3D(cx - half, cy + half, cz + half));

                    // 12 triangles (6 faces)
                    int[] faces = {
                        0,1,2, 0,2,3, 4,6,5, 4,7,6,
                        0,4,5, 0,5,1, 2,6,7, 2,7,3,
                        0,3,7, 0,7,4, 1,5,6, 1,6,2
                    };
                    foreach (int fi in faces)
                        geom.TriangleIndices.Add(offset + fi);
                    offset += 8;
                }
                geom.Freeze();

                Material mat;
                if (!_materialCache.TryGetValue(ck, out mat))
                {
                    var brush = new SolidColorBrush(Color.FromArgb(230, cr, cg2, cb));
                    brush.Freeze();
                    mat = new DiffuseMaterial(brush);
                    ((DiffuseMaterial)mat).Freeze();
                    _materialCache[ck] = mat;
                }

                var model = new GeometryModel3D(geom, mat);
                model.BackMaterial = mat;
                model.Freeze();
                _meshModelGroup.Children.Add(model);
            }
        }

        /// <summary>
        /// Render block-only cubes (fallback when per-voxel data is not available).
        /// </summary>
        private void UpdateMeshVisualBlocks(MeshDataModel meshData)
        {
                double bs = meshData.BlockSize > 0 ? meshData.BlockSize : 0.05;
                double cubeSize = bs * 0.96;
                double gap = (bs - cubeSize) * 0.5;

                // Merge incoming blocks into persisted map
                bool hasNewBlocks = false;
                foreach (var block in meshData.Blocks)
                {
                    if (block.Index == null || block.Index.Count < 3)
                        continue;

                    string key = $"{block.Index[0]},{block.Index[1]},{block.Index[2]}";
                    uint colorKey;
                    if (block.Color != null && block.Color.Count >= 3)
                        colorKey = ((uint)block.Color[0] << 16) | ((uint)block.Color[1] << 8) | (uint)block.Color[2];
                    else
                        colorKey = (150u << 16) | (150u << 8) | 160u;

                    if (!_persistedBlocks.ContainsKey(key) || _persistedBlocks[key] != colorKey)
                    {
                        _persistedBlocks[key] = colorKey;
                        hasNewBlocks = true;
                    }
                }

                if (!hasNewBlocks)
                    return;

                _meshModelGroup.Children.Clear();

                var colorGroups = new Dictionary<uint, List<int[]>>();
                foreach (var kvp in _persistedBlocks)
                {
                    string[] parts = kvp.Key.Split(',');
                    int[] idx = { int.Parse(parts[0]), int.Parse(parts[1]), int.Parse(parts[2]) };
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

                    var groupGeometry = new MeshGeometry3D();
                    int gOffset = 0;

                    foreach (var idx in kvp.Value)
                    {
                        double ox = idx[0] * bs + gap;
                        double oy = idx[1] * bs + gap;
                        double oz = idx[2] * bs + gap;

                        groupGeometry.Positions.Add(new Point3D(ox,            oy,            oz));
                        groupGeometry.Positions.Add(new Point3D(ox + cubeSize, oy,            oz));
                        groupGeometry.Positions.Add(new Point3D(ox + cubeSize, oy + cubeSize, oz));
                        groupGeometry.Positions.Add(new Point3D(ox,            oy + cubeSize, oz));
                        groupGeometry.Positions.Add(new Point3D(ox,            oy,            oz + cubeSize));
                        groupGeometry.Positions.Add(new Point3D(ox + cubeSize, oy,            oz + cubeSize));
                        groupGeometry.Positions.Add(new Point3D(ox + cubeSize, oy + cubeSize, oz + cubeSize));
                        groupGeometry.Positions.Add(new Point3D(ox,            oy + cubeSize, oz + cubeSize));

                        int[] faces = {
                            0,1,2, 0,2,3,
                            4,6,5, 4,7,6,
                            0,4,5, 0,5,1,
                            2,6,7, 2,7,3,
                            0,3,7, 0,7,4,
                            1,5,6, 1,6,2,
                        };
                        foreach (int fi in faces)
                            groupGeometry.TriangleIndices.Add(gOffset + fi);
                        gOffset += 8;
                    }

                    var material = new DiffuseMaterial(new SolidColorBrush(
                        Color.FromArgb(220, r, g, b)
                    ));
                    var model = new GeometryModel3D(groupGeometry, material);
                    model.BackMaterial = material;
                    _meshModelGroup.Children.Add(model);
                }
        }
        
        private void UpdateDroneVisual()
        {
            if (_droneVisual == null) return;
            
            var transform = new Transform3DGroup();
            
            // Rotation (yaw, pitch, roll -> ZYX Euler)
            var rotZ = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), _droneYaw * 180 / Math.PI));
            var rotY = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), _dronePitch * 180 / Math.PI));
            var rotX = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), _droneRoll * 180 / Math.PI));
            
            transform.Children.Add(rotX);
            transform.Children.Add(rotY);
            transform.Children.Add(rotZ);
            
            // Translation
            transform.Children.Add(new TranslateTransform3D(_dronePosition.X, _dronePosition.Y, _dronePosition.Z));
            
            _droneVisual.Transform = transform;
        }
        
        private void UpdateTrajectoryVisual()
        {
            if (_trajectoryVisual == null || !_chkShowTrajectory.Checked || _trajectoryPoints.Count < 2)
                return;
            
            try
            {
                var trajectoryBuilder = new MeshBuilder();
                
                // Create tube along trajectory
                for (int i = 1; i < _trajectoryPoints.Count; i++)
                {
                    trajectoryBuilder.AddPipe(
                        _trajectoryPoints[i - 1],
                        _trajectoryPoints[i],
                        0, 0.02, 6
                    );
                }
                
                var trajectoryMaterial = new DiffuseMaterial(new SolidColorBrush(
                    Color.FromArgb(180, 255, 200, 0)  // Yellow trajectory
                ));
                var trajectoryModel = new GeometryModel3D(trajectoryBuilder.ToMesh(), trajectoryMaterial);
                
                var group = new Model3DGroup();
                group.Children.Add(trajectoryModel);
                _trajectoryVisual.Content = group;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Trajectory update error: {ex.Message}");
            }
        }
        
        private void UpdateCameras()
        {
            switch (_currentViewMode)
            {
                case CameraViewMode.FirstPerson:
                    UpdateFPVCamera();
                    // Hide drone arrow in FPV so it doesn't obscure the view
                    if (_droneVisual != null)
                        _droneVisual.Content = new Model3DGroup();
                    break;
                case CameraViewMode.ThirdPerson:
                    UpdateTPVCamera();
                    RestoreDroneVisual();
                    break;
                case CameraViewMode.FreeOrbit:
                    RestoreDroneVisual();
                    break;
            }
        }
        
        private void UpdateFPVCamera()
        {
            _fpvCamera.Position = _dronePosition;
            
            double yaw = _droneYaw;
            double pitch = _dronePitch;
            double roll = _droneRoll;

            // Look direction from yaw + pitch
            _fpvCamera.LookDirection = new Vector3D(
                Math.Cos(yaw) * Math.Cos(pitch),
                Math.Sin(yaw) * Math.Cos(pitch),
                -Math.Sin(pitch)
            );

            // Up direction rotated by roll around the look axis
            // Default up is (0,0,1); roll rotates it in the plane perpendicular to look
            double upX = -Math.Cos(yaw) * Math.Sin(pitch) * Math.Cos(roll) - Math.Sin(yaw) * Math.Sin(roll);
            double upY = -Math.Sin(yaw) * Math.Sin(pitch) * Math.Cos(roll) + Math.Cos(yaw) * Math.Sin(roll);
            double upZ = Math.Cos(pitch) * Math.Cos(roll);
            _fpvCamera.UpDirection = new Vector3D(upX, upY, upZ);
        }

        private void RestoreDroneVisual()
        {
            if (_droneVisual != null && _droneModelContent != null &&
                _droneVisual.Content != _droneModelContent)
            {
                _droneVisual.Content = _droneModelContent;
            }
        }
        
        private void UpdateTPVCamera()
        {
            // Camera behind and above drone
            double distance = 5.0;
            double height = 3.0;
            double yaw = _droneYaw;
            
            double camX = _dronePosition.X - distance * Math.Cos(yaw);
            double camY = _dronePosition.Y - distance * Math.Sin(yaw);
            double camZ = _dronePosition.Z + height;
            
            _tpvCamera.Position = new Point3D(camX, camY, camZ);
            _tpvCamera.LookDirection = new Vector3D(
                _dronePosition.X - camX,
                _dronePosition.Y - camY,
                _dronePosition.Z - camZ
            );
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
                // Reset orbit camera
                _orbitCamera.Position = new Point3D(-10, -10, 8);
                _orbitCamera.LookDirection = new Vector3D(10, 10, -5);
                
                // Reset TPV
                _tpvCamera.Position = new Point3D(-5, -5, 5);
                _tpvCamera.LookDirection = new Vector3D(5, 5, -3);
                
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
                _materialCache.Clear();
                _meshDirty = false;
                _lastRenderedCount = 0;
                var emptyGroup = new Model3DGroup();
                _trajectoryVisual.Content = emptyGroup;
            }));
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
            if (disposing)
            {
                _updateCts?.Cancel();
                _updateCts?.Dispose();
                _elementHost?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
