// ==========================================================
// SLAM3DManager.cs - Central Manager for SLAM3D Components
// ==========================================================
// This manager coordinates all SLAM3D modular components and
// provides a clean interface for SLAM3DView to use.
// ==========================================================

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using NOMAD.MissionPlanner.SLAM3D.Data;
using NOMAD.MissionPlanner.SLAM3D.Network;
using NOMAD.MissionPlanner.SLAM3D.Rendering;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner.SLAM3D
{
    /// <summary>
    /// Central manager coordinating all SLAM3D subsystems.
    /// Provides unified interface for pose updates, mesh data, and rendering.
    /// </summary>
    public class SLAM3DManager : IDisposable
    {
        // ---- Components ----
        private readonly PoseState _poseState;
        private readonly VoxelStorage _voxelStorage;
        private readonly WebSocketClient _webSocketClient;
        private readonly Camera.CameraController _cameraController;

        // ---- Renderers ----
        private readonly GridRenderer _gridRenderer;
        private readonly VoxelRenderer _voxelRenderer;
        private readonly TrajectoryRenderer _trajectoryRenderer;
        private readonly DetectionRenderer _detectionRenderer;
        private readonly DroneRenderer _droneRenderer;

        // ---- State ----
        private bool _disposed;
        private readonly object _stateLock = new object();
        private int _meshUpdateCount;
        private DateTime _lastMeshRebuild = DateTime.MinValue;
        private bool _meshDirty;
        private double _currentVoxelSize = 0.05;
        private readonly Stopwatch _uptimeWatch = Stopwatch.StartNew();

        // ---- Configuration ----
        private readonly string _edgeCoreUrl;
        private bool _autoConnect = true;

        // ---- Events ----
        public event Action<string> OnStatusChanged;
        public event Action<string> OnError;
        public event Action OnMeshUpdated;
        public event Action OnPoseUpdated;

        // ---- Properties ----

        /// <summary>Current smoothed X position (OpenGL coords).</summary>
        public float PoseX => _poseState.X;

        /// <summary>Current smoothed Y position (OpenGL coords).</summary>
        public float PoseY => _poseState.Y;

        /// <summary>Current smoothed Z position (OpenGL coords).</summary>
        public float PoseZ => _poseState.Z;

        /// <summary>Current smoothed roll (degrees).</summary>
        public float RollDeg => _poseState.Roll;

        /// <summary>Current smoothed pitch (degrees).</summary>
        public float PitchDeg => _poseState.Pitch;

        /// <summary>Current smoothed yaw (degrees).</summary>
        public float YawDeg => _poseState.Yaw;

        /// <summary>Total mesh update count.</summary>
        public int MeshUpdateCount => _meshUpdateCount;

        /// <summary>Total voxel count.</summary>
        public int VoxelCount => _voxelStorage.Count;

        /// <summary>WebSocket connection status.</summary>
        public bool IsConnected => _webSocketClient?.IsConnected ?? false;

        /// <summary>Pose state statistics (total, rejected, accepted, streak).</summary>
        public (int total, int rejected, int accepted, int streak) PoseStats => _poseState.GetStats();

        /// <summary>Grid renderer.</summary>
        public GridRenderer Grid => _gridRenderer;

        /// <summary>Trajectory renderer.</summary>
        public TrajectoryRenderer Trajectory => _trajectoryRenderer;

        /// <summary>Detection renderer.</summary>
        public DetectionRenderer Detections => _detectionRenderer;

        /// <summary>Camera controller.</summary>
        public Camera.CameraController Camera => _cameraController;

        /// <summary>
        /// Initialize SLAM3D manager with Edge Core URL.
        /// </summary>
        /// <param name="edgeCoreUrl">Base URL (e.g., "http://100.85.121.98:8000")</param>
        public SLAM3DManager(string edgeCoreUrl)
        {
            _edgeCoreUrl = edgeCoreUrl ?? throw new ArgumentNullException(nameof(edgeCoreUrl));

            // Initialize components
            _poseState = new PoseState();
            _voxelStorage = new VoxelStorage(maxVoxels: 5000);
            _cameraController = new Camera.CameraController();

            // Initialize renderers
            _gridRenderer = new GridRenderer();
            _voxelRenderer = new VoxelRenderer();
            _trajectoryRenderer = new TrajectoryRenderer();
            _detectionRenderer = new DetectionRenderer();
            _droneRenderer = new DroneRenderer();

            // Initialize WebSocket client
            _webSocketClient = new WebSocketClient();
            _webSocketClient.BaseUrl = edgeCoreUrl;
            _webSocketClient.OnFrameReceived += HandleSlamFrame;
            _webSocketClient.OnStatusChanged += status => OnStatusChanged?.Invoke(status);
            _webSocketClient.OnError += error => OnError?.Invoke(error);
        }

        /// <summary>
        /// Start WebSocket connection and update loop.
        /// </summary>
        public void Start()
        {
            if (_autoConnect)
            {
                _webSocketClient.Start();
            }
        }

        /// <summary>
        /// Stop WebSocket connection.
        /// </summary>
        public void Stop()
        {
            _webSocketClient.Stop();
        }

        /// <summary>
        /// Handle incoming SLAM frame from WebSocket.
        /// </summary>
        private void HandleSlamFrame(SlamFrame frame)
        {
            if (frame == null) return;

            // Update pose with anti-jitter filtering
            if (frame.AttitudeValid)
            {
                // Convert ROS odom frame (X-forward, Y-left, Z-up) to OpenGL (X-right, Y-up, Z-toward)
                float rosX = frame.X;
                float rosY = frame.Y;
                float rosZ = frame.Z;

                float glX = -rosY;
                float glY = rosZ;
                float glZ = -rosX;

                // Use body attitude if available (magnetometer-corrected yaw)
                float rollDeg, pitchDeg, yawDeg;
                if (frame.BodyYaw != 0 || frame.BodyRoll != 0 || frame.BodyPitch != 0)
                {
                    rollDeg = frame.BodyRoll * 180f / MathHelper.PI;
                    pitchDeg = frame.BodyPitch * 180f / MathHelper.PI;
                    yawDeg = frame.BodyYaw * 180f / MathHelper.PI;
                }
                else
                {
                    rollDeg = frame.Roll * 180f / MathHelper.PI;
                    pitchDeg = frame.Pitch * 180f / MathHelper.PI;
                    yawDeg = frame.Yaw * 180f / MathHelper.PI;
                }

                bool updated = _poseState.Update(glX, glY, glZ, rollDeg, pitchDeg, yawDeg, attitudeValid: true);

                if (updated)
                {
                    // Add to trajectory
                    _trajectoryRenderer.AddPoint(glX, glY, glZ);
                    OnPoseUpdated?.Invoke();
                }
            }

            // Process mesh data if present
            if (frame.Type == SlamFrameType.Mesh && frame.MeshToken != null)
            {
                ProcessMeshToken(frame.MeshToken);
            }
        }

        /// <summary>
        /// Process mesh data from JToken.
        /// </summary>
        private void ProcessMeshToken(JToken meshToken)
        {
            if (meshToken == null) return;

            try
            {
                lock (_stateLock)
                {
                    // Handle clear command
                    bool clear = meshToken["clear"]?.Value<bool>() ?? false;
                    if (clear)
                    {
                        _voxelStorage.Clear();
                        _meshDirty = true;
                    }

                    // Update voxel size
                    double voxelSize = meshToken["voxel_size"]?.Value<double>() ?? 0;
                    if (voxelSize > 0)
                    {
                        _currentVoxelSize = voxelSize;
                    }

                    // Process voxels array
                    var voxelsArray = meshToken["voxels"] as JArray;
                    if (voxelsArray != null && voxelsArray.Count > 0)
                    {
                        var voxelList = new List<VoxelData>();
                        foreach (var v in voxelsArray)
                        {
                            var pos = v["p"] as JArray;
                            if (pos == null || pos.Count < 3) continue;

                            // Convert from ROS frame to OpenGL frame
                            float rosX = pos[0].Value<float>();
                            float rosY = pos[1].Value<float>();
                            float rosZ = pos[2].Value<float>();

                            // ROS odom: X-forward, Y-left, Z-up
                            // OpenGL: X-right, Y-up, Z-toward
                            // Mapping: gx = -y, gy = z, gz = -x
                            float gx = -rosY;
                            float gy = rosZ;
                            float gz = -rosX;

                            // Quantize to voxel indices
                            int ix = (int)Math.Round(gx / _currentVoxelSize);
                            int iy = (int)Math.Round(gy / _currentVoxelSize);
                            int iz = (int)Math.Round(gz / _currentVoxelSize);

                            uint color = 0x808080; // Default gray
                            var colorArray = v["c"] as JArray;
                            if (colorArray != null && colorArray.Count >= 3)
                            {
                                byte r = (byte)MathHelper.Clamp(colorArray[0].Value<int>(), 0, 255);
                                byte g = (byte)MathHelper.Clamp(colorArray[1].Value<int>(), 0, 255);
                                byte b = (byte)MathHelper.Clamp(colorArray[2].Value<int>(), 0, 255);
                                color = ((uint)r << 16) | ((uint)g << 8) | b;
                            }

                            voxelList.Add(new VoxelData { X = ix, Y = iy, Z = iz, Color = color });
                        }

                        _voxelStorage.AddVoxels(voxelList);
                        _meshDirty = true;
                    }

                    // Process removed voxels
                    var removedArray = meshToken["removed"] as JArray;
                    if (removedArray != null && removedArray.Count > 0)
                    {
                        var removeList = new List<(int, int, int)>();
                        foreach (var r in removedArray)
                        {
                            int rx = r["x"]?.Value<int>() ?? 0;
                            int ry = r["y"]?.Value<int>() ?? 0;
                            int rz = r["z"]?.Value<int>() ?? 0;
                            // Convert from ROS frame indices to GL frame indices
                            int gx = -ry;
                            int gy = rz;
                            int gz = -rx;
                            removeList.Add((gx, gy, gz));
                        }
                        _voxelStorage.RemoveVoxels(removeList);
                        _meshDirty = true;
                    }

                    _meshUpdateCount++;
                    OnMeshUpdated?.Invoke();
                }
            }
            catch (Exception ex)
            {
                OnError?.Invoke($"Mesh parse error: {ex.Message}");
            }
        }

        /// <summary>
        /// Rebuild mesh if dirty and debounce interval has passed.
        /// Call from render thread.
        /// </summary>
        /// <param name="minIntervalMs">Minimum interval between rebuilds</param>
        /// <returns>True if mesh was rebuilt</returns>
        public bool RebuildMeshIfNeeded(int minIntervalMs = 250)
        {
            if (!_meshDirty) return false;

            var now = DateTime.UtcNow;
            if ((now - _lastMeshRebuild).TotalMilliseconds < minIntervalMs)
                return false;

            lock (_stateLock)
            {
                if (!_meshDirty) return false;

                var voxels = _voxelStorage.GetVoxelDictionary();
                _voxelRenderer.BuildMesh(voxels, _currentVoxelSize, VoxelStorage.UnpackKey);

                _meshDirty = false;
                _lastMeshRebuild = now;
            }

            return true;
        }

        /// <summary>
        /// Initialize OpenGL resources. Call after GL context is created.
        /// </summary>
        public void InitializeGL()
        {
            _voxelRenderer.InitializeBuffers();
        }

        /// <summary>
        /// Render all 3D elements.
        /// </summary>
        public void Render()
        {
            // Rebuild mesh if needed
            RebuildMeshIfNeeded();

            // Render grid
            _gridRenderer.Render(0f);

            // Render voxels
            _voxelRenderer.Render();

            // Render trajectory
            _trajectoryRenderer.Render();

            // Render drone (Draw takes radians, convert from stored degrees)
            float rollRad = _poseState.Roll * MathHelper.PI / 180f;
            float pitchRad = _poseState.Pitch * MathHelper.PI / 180f;
            float yawRad = _poseState.Yaw * MathHelper.PI / 180f;
            _droneRenderer.Draw(
                _poseState.X, _poseState.Y, _poseState.Z,
                rollRad, pitchRad, yawRad);

            // Render detection markers
            _detectionRenderer.Render();
        }

        /// <summary>
        /// Clear all mesh data.
        /// </summary>
        public void ClearMesh()
        {
            lock (_stateLock)
            {
                _voxelStorage.Clear();
                _meshDirty = true;
            }
        }

        /// <summary>
        /// Clear trajectory.
        /// </summary>
        public void ClearTrajectory()
        {
            _trajectoryRenderer.Clear();
        }

        /// <summary>
        /// Reset pose to origin.
        /// </summary>
        public void ResetPose()
        {
            _poseState.Reset();
        }

        /// <summary>
        /// Get status summary string.
        /// </summary>
        public string GetStatusSummary()
        {
            var stats = _poseState.GetStats();
            return $"Voxels: {_voxelStorage.Count} | Updates: {_meshUpdateCount} | " +
                   $"Rejected: {stats.rejected} | Connected: {IsConnected}";
        }

        /// <summary>
        /// Dispose resources.
        /// </summary>
        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            _webSocketClient?.Stop();
            _voxelRenderer?.Dispose();
        }
    }
}
