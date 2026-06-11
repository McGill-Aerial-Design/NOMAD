// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
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
    public partial class SLAM3DView : UserControl
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
        // Track the camera tilt as PWM microseconds — the server's 0-180 deg
        // value is a linear projection onto a 500-2500us range that doesn't
        // match the rig's mechanical limits (700us=down, 1250us=level, 1450us=up).
        // We display PWM directly and remap to the renderer's "90deg=level" space.
        private const int ServoPulseDownUs  = 700;
        private const int ServoPulseLevelUs = 1250;
        private const int ServoPulseUpUs    = 1450;
        private const int ServoApiPulseMinUs = 500;
        private const int ServoApiPulseMaxUs = 2500;
        private int _servoPulseUs = ServoPulseLevelUs;
        private System.Windows.Forms.Timer _servoTimer;

        // ---- Perception/status polling ----
        private System.Windows.Forms.Timer _statusTimer;
        private bool _statusPollInFlight;

        // ---- PoseState (anti-jitter filtering) ----
        private readonly PoseState _poseState = new PoseState();

        /// <summary>
        /// When true, bypass all client-side pose smoothing, jump rejection,
        /// and voxel retention/quantization so the 3D view matches RViz exactly.
        /// Controlled by environment variable NOMAD_SLAM3D_PARITY=1 or the
        /// ParityMode property at runtime.
        /// </summary>
        public bool ParityMode
        {
            get => _poseState.ParityMode;
            set
            {
                _poseState.ParityMode = value;
                _voxelMeshBuilder.ParityMode = value;
            }
        }

        // ---- Drone pose (raw from WS, REP-103 map frame: X-forward, Y-left, Z-up) ----
        private float _dronePosX, _dronePosY, _dronePosZ;
        private float _droneRollRaw, _dronePitchRaw, _droneYawRaw;
        private bool _attitudeValid = true; // Tracks server-reported attitude_valid flag
        private float _droneVelX, _droneVelY, _droneVelZ;
        private float _renderPosX, _renderPosY, _renderPosZ;
        private float _renderRollRaw, _renderPitchRaw, _renderYawRaw;
        private bool _hasBodyAttitude; // True if body_roll/pitch/yaw received (magnetometer-corrected)
        private bool _hasPoseFrame;

        // ---- Trajectory ----
        private const int MaxTrajectoryPoints = 500;

        // ---- Camera interaction ----
        private Point _lastMousePos;
        private bool _mouseRotating, _mousePanning;

        // ---- UI Controls ----
        private Panel _controlPanel;
        private Panel _statusLogPanel;
        private Button _btnToggleCamera, _btnResetView, _btnClearMesh, _btnResetImuBiases;
        private Button _btnCenterOnPose;
        private Label _lblStatus, _lblStats;
        private Label _lblPerceptionStatus;
        private TextBox _txtStatusLog;
        private CheckBox _chkShowGrid, _chkShowTrajectory, _chkAutoUpdate;
        private ComboBox _combDroneType;
        private NumericUpDown _numLength, _numWidth, _numHeight, _numHeadingOffset, _numFov, _numMapRadius;
        private int _meshUpdateCount;
        private int _totalBlocks;
        private const int MaxStatusLogLines = 120;

        // ==================== Constructor ====================

        public SLAM3DView(NOMADConfig config, DualLinkSender sender)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            _sender = sender ?? throw new ArgumentNullException(nameof(sender));

            // Parity mode opt-in via environment variable for RViz comparison runs.
            // Accepts "1", "true", "yes" (case-insensitive).
            try
            {
                string parityEnv = Environment.GetEnvironmentVariable("NOMAD_SLAM3D_PARITY");
                if (!string.IsNullOrWhiteSpace(parityEnv))
                {
                    string v = parityEnv.Trim().ToLowerInvariant();
                    if (v == "1" || v == "true" || v == "yes")
                    {
                        ParityMode = true;
                        Debug.WriteLine("[SLAM3D] ParityMode enabled via NOMAD_SLAM3D_PARITY");
                    }
                }
            }
            catch { /* ignore env access failures */ }

            InitializeSlamComponents();
            InitializeComponents();
            StartUpdateLoop();
            StartServoPolling();
            StartPerceptionStatusPolling();
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
            _webSocketClient.MaxMessageSize = 64 * 1024 * 1024;
            _webSocketClient.ReceiveTimeoutSec = 30;
            _webSocketClient.OnStatusChanged += HandleWebSocketStatusChanged;
            _webSocketClient.OnError += HandleWebSocketError;
            _webSocketClient.OnFrameReceived += HandleSlamFrame;
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
