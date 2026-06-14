// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// BuildingViewer3D.cs - 3D building model with target markers
// ============================================================
// OpenTK-based 3D viewer. Renders the building polygon
// (extruded by configured height), a surround circle, and
// one sphere per captured target marker.
//
// Mouse:    left-drag = orbit, wheel = zoom, right-drag = pan
// Hover:    nearest target within 14 px fires TargetHovered;
//           the panel uses this to highlight rows in the grid
//           (and the panel calls SetHighlightedTarget on us to
//           do the reverse).
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;
using OpenTK;
using OpenTK.Graphics;
using NOMAD.MissionPlanner.SLAM3D.Rendering;

namespace NOMAD.MissionPlanner
{
    public partial class BuildingViewer3D : UserControl
    {
        // ==================== Public API ====================

        public event Action<string> TargetHovered;
        public event Action<string> TargetClicked;
        public event Action<Placement> PlacementClicked;
        public string HighlightedTargetId { get; private set; }
        public bool PlacementMode { get; set; }
        public bool DronePovEnabled
        {
            get => _dronePovEnabled;
            set
            {
                if (_dronePovEnabled == value) return;
                _dronePovEnabled = value;
                _heldKeys.Clear();
                _navTimer.Stop();
                _glControl?.Invalidate();
            }
        }
        public float BuildingHeightM => _buildingHeight;

        // ==================== Internal state ====================

        private readonly GLControl _glControl;
        private readonly List<Corner> _corners = new List<Corner>();
        private readonly List<Target> _targets = new List<Target>();
        private readonly HashSet<Keys> _heldKeys = new HashSet<Keys>();
        private readonly DroneRenderer _droneRenderer = new DroneRenderer();
        private readonly Timer _navTimer;

        private float _buildingHeight = 5f;
        private float _searchBufferM = 15f;
        private float _sceneHalfExtentM = 25f;
        private float _minEast = -10f, _maxEast = 10f, _minNorth = -10f, _maxNorth = 10f;
        private bool _hasProjectionOrigin;
        private double _originLat;
        private double _originLon;
        private double _originCosLat = 1.0;
        private float _recenterEast;
        private float _recenterNorth;
        private bool _hasDronePose;
        private float _droneEast;
        private float _droneNorth;
        private float _droneUp;
        private float _droneYawRad;
        private float _dronePitchRad;
        private float _droneRollRad;
        private bool _dronePovEnabled;

        // Orbit camera state.
        private float _yawDeg = 35f;
        private float _pitchDeg = 30f;
        private float _distance = 25f;
        private Vector3 _panTarget = Vector3.Zero;

        private Point _lastMouse;
        private Point _mouseDownPoint;
        private MouseButtons _dragButton;
        private string _hoverId;

        // Cached matrices for picking (set during paint).
        private Matrix4 _viewProj;
        private int _viewW, _viewH;

        // ==================== Construction ====================

        public BuildingViewer3D()
        {
            BackColor = Color.FromArgb(20, 20, 22);
            Dock = DockStyle.Fill;

            _glControl = new GLControl(new GraphicsMode(32, 24, 0, 4))
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                TabStop = true,
            };
            _glControl.Load += GlControl_Load;
            _glControl.Resize += GlControl_Resize;
            _glControl.Paint += GlControl_Paint;
            _glControl.MouseDown += GlControl_MouseDown;
            _glControl.MouseUp += GlControl_MouseUp;
            _glControl.MouseEnter += (s, e) => _glControl.Focus();
            _glControl.MouseMove += GlControl_MouseMove;
            _glControl.MouseLeave += (s, e) => SetHover(null);
            _glControl.LostFocus += (s, e) =>
            {
                _heldKeys.Clear();
                _navTimer.Stop();
            };
            _glControl.MouseWheel += GlControl_MouseWheel;
            _glControl.PreviewKeyDown += GlControl_PreviewKeyDown;
            _glControl.KeyDown += GlControl_KeyDown;
            _glControl.KeyUp += GlControl_KeyUp;

            Controls.Add(_glControl);

            _navTimer = new Timer { Interval = 16 };
            _navTimer.Tick += NavTimer_Tick;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                try { _navTimer?.Stop(); } catch { }
                try { _navTimer?.Dispose(); } catch { }
                try { _glControl?.Dispose(); } catch { }
            }
            base.Dispose(disposing);
        }
    }
}
