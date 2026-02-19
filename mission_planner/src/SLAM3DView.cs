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
        
        // Persisted block map: key = "ix,iy,iz", value = packed ARGB color
        private Dictionary<string, uint> _persistedBlocks = new Dictionary<string, uint>();
        private const int MaxPersistedVoxels = 5000; // Hard cap with face-culling keeps triangle count low

        // Material cache to avoid recreating WPF resources every frame
        private Dictionary<uint, Material> _materialCache = new Dictionary<uint, Material>();

        // Dirty tracking: only rebuild mesh when significant changes arrive
        private bool _meshDirty = false;
        private int _lastRenderedCount = 0;
        private const int MinNewVoxelsForRebuild = 20; // Only rebuild after 20+ new voxels

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

                    UpdateStatusSafe("Connecting...");
                    await _webSocket.ConnectAsync(new Uri(wsUrl), ct);
                    UpdateStatusSafe("Status: Connected (30Hz)");
                    _wsReconnectDelayMs = 1000;

                    var buffer = new byte[64 * 1024];
                    var messageBuffer = new MemoryStream();

                    while (_webSocket.State == WebSocketState.Open && !ct.IsCancellationRequested)
                    {
                        messageBuffer.SetLength(0);
                        WebSocketReceiveResult result;
                        do
                        {
                            result = await _webSocket.ReceiveAsync(
                                new ArraySegment<byte>(buffer), ct);
                            if (result.MessageType == WebSocketMessageType.Close)
                                break;
                            messageBuffer.Write(buffer, 0, result.Count);
                        } while (!result.EndOfMessage);

                        if (result.MessageType == WebSocketMessageType.Close)
                            break;

                        if (_chkAutoUpdate?.Checked != true)
                            continue;

                        string json = Encoding.UTF8.GetString(
                            messageBuffer.GetBuffer(), 0, (int)messageBuffer.Length);
                        var frame = JObject.Parse(json);
                        string frameType = frame["type"]?.ToString() ?? "pose";

                        // Skip frames with no position data
                        if (frame["x"] == null)
                            continue;

                        _dronePosition = new Point3D(
                            frame["x"]?.Value<double>() ?? 0,
                            frame["y"]?.Value<double>() ?? 0,
                            frame["z"]?.Value<double>() ?? 0);
                        _droneRoll = frame["roll"]?.Value<double>() ?? 0;
                        _dronePitch = frame["pitch"]?.Value<double>() ?? 0;
                        _droneYaw = frame["yaw"]?.Value<double>() ?? 0;

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
            try
            {
                this.BeginInvoke(new Action(() =>
                {
                    if (_disposed || _elementHost == null) return;
                    _elementHost.Invoke(new Action(() =>
                    {
                        AddTrajectoryPoint(_dronePosition);
                        UpdateDroneVisual();
                        UpdateTrajectoryVisual();
                        UpdateCameras();
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
            try
            {
                this.BeginInvoke(new Action(() =>
                {
                    if (_disposed || _elementHost == null) return;
                    _elementHost.Invoke(new Action(() =>
                    {
                        AddTrajectoryPoint(_dronePosition);
                        UpdateDroneVisual();
                        UpdateTrajectoryVisual();
                        UpdateCameras();
                        if (meshData != null)
                            UpdateMeshVisual(meshData);
                    }));

                    _meshUpdateCount++;
                    _lastUpdateTime = DateTime.Now;
                    if (meshData != null)
                        _totalBlocks = meshData.TotalBlocks > 0 ? meshData.TotalBlocks : meshData.TotalVoxels;
                    string mode = meshData?.Mode == "voxels" ? "voxels" : "blocks";
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
                    _occupancySet.Clear();
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
        /// Uses adjacent-face culling: only renders faces not touching another voxel (Minecraft-style).
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

                // Flat color: quantize to 4-bit per channel (16 levels)
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
                    _occupancySet.Add(PackKey(qx, qy, qz));
                    _meshDirty = true;
                }
            }

            // Evict oldest entries if over cap
            if (_persistedBlocks.Count > MaxPersistedVoxels)
            {
                int toRemove = _persistedBlocks.Count - MaxPersistedVoxels;
                var keysToRemove = _persistedBlocks.Keys.Take(toRemove).ToList();
                foreach (var k in keysToRemove)
                {
                    _persistedBlocks.Remove(k);
                    string[] p = k.Split(',');
                    _occupancySet.Remove(PackKey(int.Parse(p[0]), int.Parse(p[1]), int.Parse(p[2])));
                }
                _meshDirty = true;
            }

            if (!_meshDirty) return;

            // Only defer rebuild when voxels are purely additive (count grew)
            // Always rebuild immediately on eviction (count shrank) or color changes
            int newSinceLastRender = _persistedBlocks.Count - _lastRenderedCount;
            if (newSinceLastRender > 0 && newSinceLastRender < MinNewVoxelsForRebuild && _lastRenderedCount > 0)
                return;

            _meshDirty = false;
            _lastRenderedCount = _persistedBlocks.Count;

            // Group by color for batched draw calls
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

                var positions = new Point3DCollection();
                var indices = new Int32Collection();
                int offset = 0;

                foreach (var key in cg.Value)
                {
                    string[] parts = key.Split(',');
                    int ix = int.Parse(parts[0]);
                    int iy = int.Parse(parts[1]);
                    int iz = int.Parse(parts[2]);
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
                model.BackMaterial = mat;
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

                    string key = $"{block.Index[0]},{block.Index[1]},{block.Index[2]}";
                    uint colorKey;
                    if (block.Color != null && block.Color.Count >= 3)
                        colorKey = ((uint)block.Color[0] << 16) | ((uint)block.Color[1] << 8) | (uint)block.Color[2];
                    else
                        colorKey = (150u << 16) | (150u << 8) | 160u;

                    if (!_persistedBlocks.ContainsKey(key) || _persistedBlocks[key] != colorKey)
                    {
                        _persistedBlocks[key] = colorKey;
                        _occupancySet.Add(PackKey(block.Index[0], block.Index[1], block.Index[2]));
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
                    model.BackMaterial = mat;
                    model.Freeze();
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
                    var p0 = _trajectoryPoints[i - 1];
                    var p1 = _trajectoryPoints[i];
                    var dir = p1 - p0;
                    if (dir.Length < 0.001) continue;

                    var up = new Vector3D(0, 0, 1);
                    var right = Vector3D.CrossProduct(dir, up);
                    if (right.Length < 0.001) { up = new Vector3D(0, 1, 0); right = Vector3D.CrossProduct(dir, up); }
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
