// ============================================================
// BuildingViewer3D.cs - 3D building model with target markers
// ============================================================
// OpenTK-based viewer for the Task 1 submit page. Renders the
// building polygon (extruded by configured height), a 15 m
// surround circle and one sphere per captured target.
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
using System.Linq;
using System.Windows.Forms;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using NOMAD.MissionPlanner.SLAM3D.Rendering;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// 3D viewer for the building footprint, surround radius and captured targets.
    /// Self-contained — does not depend on the SLAM3D stack.
    /// </summary>
    public class BuildingViewer3D : UserControl
    {
        // ==================== Data classes ====================

        public class Corner
        {
            public string Name;
            public double Lat;
            public double Lon;
            // ENU coords relative to building centroid (filled by SetCorners).
            public float East;
            public float North;
        }

        public class Target
        {
            public string Id;
            public string Color;
            public string Description;
            public float East;
            public float North;
            public float Up;
        }

        public class Placement
        {
            public string Surface;
            public string WallName;
            public string NearestCornerName;
            public float East;
            public float North;
            public float Up;
            public float DistanceFromCornerM;
            public string StructureLabel;
            public string ReferenceWallName;
            public float DistanceFromReferenceWallM;
        }

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

        public void SetHighlightedTarget(string id)
        {
            if (HighlightedTargetId == id) return;
            HighlightedTargetId = id;
            _glControl?.Invalidate();
        }

        public void SetBuildingHeight(double heightM)
        {
            _buildingHeight = (float)Math.Max(0.5, heightM);
            _glControl?.Invalidate();
        }

        /// <summary>
        /// Replace the building corners. Lat/lon → ENU is computed here using
        /// an equirectangular projection at the centroid latitude — accurate
        /// to ~cm over the &lt;30 m scales we care about.
        /// </summary>
        public void SetCorners(IList<Corner> corners, double? centerLat = null, double? centerLon = null)
        {
            _corners.Clear();
            if (corners == null || corners.Count == 0)
            {
                _hasProjectionOrigin = false;
                _hasDronePose = false;
                _glControl?.Invalidate();
                return;
            }

            double lat0 = centerLat ?? corners.Average(c => c.Lat);
            double lon0 = centerLon ?? corners.Average(c => c.Lon);
            double cosLat = Math.Cos(lat0 * Math.PI / 180.0);
            _originLat = lat0;
            _originLon = lon0;
            _originCosLat = cosLat;
            _hasProjectionOrigin = true;
            const double METERS_PER_DEG = 111320.0;

            foreach (var c in corners)
            {
                _corners.Add(new Corner
                {
                    Name = c.Name,
                    Lat = c.Lat,
                    Lon = c.Lon,
                    East = (float)((c.Lon - lon0) * cosLat * METERS_PER_DEG),
                    North = (float)((c.Lat - lat0) * METERS_PER_DEG),
                });
            }

            // Recenter on centroid so the model is easy to orbit.
            float cx = _corners.Average(c => c.East);
            float cy = _corners.Average(c => c.North);
            _recenterEast = cx;
            _recenterNorth = cy;
            foreach (var c in _corners) { c.East -= cx; c.North -= cy; }
            UpdateSceneBounds();

            _glControl?.Invalidate();
        }

        public void SetDronePoseGps(double lat, double lon, double altAglM,
            double yawDeg, double pitchDeg, double rollDeg)
        {
            if (!_hasProjectionOrigin || Math.Abs(lat) < 0.000001 || Math.Abs(lon) < 0.000001)
            {
                _hasDronePose = false;
                _glControl?.Invalidate();
                return;
            }

            const double METERS_PER_DEG = 111320.0;
            _droneEast = (float)((lon - _originLon) * _originCosLat * METERS_PER_DEG - _recenterEast);
            _droneNorth = (float)((lat - _originLat) * METERS_PER_DEG - _recenterNorth);
            _droneUp = (float)Math.Max(0.0, altAglM);
            _droneYawRad = (float)(yawDeg * Math.PI / 180.0);
            _dronePitchRad = (float)(pitchDeg * Math.PI / 180.0);
            _droneRollRad = (float)(rollDeg * Math.PI / 180.0);
            _hasDronePose = true;
            _glControl?.Invalidate();
        }

        public bool TryGetLocalFromGps(double lat, double lon, out float east, out float north)
        {
            east = 0f;
            north = 0f;
            if (!_hasProjectionOrigin) return false;

            const double METERS_PER_DEG = 111320.0;
            east = (float)((lon - _originLon) * _originCosLat * METERS_PER_DEG - _recenterEast);
            north = (float)((lat - _originLat) * METERS_PER_DEG - _recenterNorth);
            return true;
        }

        public Placement CreatePlacementFromLocal(string surface, float east, float north, float up)
        {
            if (_corners.Count < 3) return null;
            string normalized = string.IsNullOrWhiteSpace(surface)
                ? "ground"
                : surface.Trim().ToLowerInvariant();
            float clampedUp = normalized == "roof"
                ? _buildingHeight
                : Math.Max(0f, Math.Min(_buildingHeight, up));
            return BuildPlacement(
                normalized,
                normalized == "wall" ? (int?)NearestWallIndex(east, north) : null,
                new Vector3(east, clampedUp, -north));
        }

        public void SetTargets(IList<Target> targets)
        {
            _targets.Clear();
            if (targets != null)
            {
                // Recenter targets the same way corners were recentered. The
                // python pipeline already returns ENU relative to the building
                // centroid, so no extra offset is needed unless the user-set
                // corners drift from the python centroid.
                foreach (var t in targets) _targets.Add(t);
            }
            _glControl?.Invalidate();
        }

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

        private void GlControl_Load(object sender, EventArgs e)
        {
            try
            {
                _glControl.MakeCurrent();
                GL.ClearColor(0.08f, 0.08f, 0.10f, 1f);
                GL.Enable(EnableCap.DepthTest);
                GL.Enable(EnableCap.Blend);
                GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
                GL.Enable(EnableCap.LineSmooth);
                GL.Hint(HintTarget.LineSmoothHint, HintMode.Nicest);
            }
            catch { }
        }

        private void GlControl_Resize(object sender, EventArgs e)
        {
            try
            {
                _glControl.MakeCurrent();
                int w = Math.Max(1, _glControl.Width);
                int h = Math.Max(1, _glControl.Height);
                GL.Viewport(0, 0, w, h);
                _viewW = w; _viewH = h;
            }
            catch { }
        }

        // ==================== Camera + matrices ====================

        private (Matrix4 view, Matrix4 proj, Vector3 eye) BuildMatrices()
        {
            float aspect = _viewW > 0 && _viewH > 0 ? (float)_viewW / _viewH : 1f;
            var proj = Matrix4.CreatePerspectiveFieldOfView(
                MathHelperToRad(55f), aspect, 0.1f, 500f);

            if (_dronePovEnabled && _hasDronePose)
            {
                float glX = _droneEast;
                float glY = Math.Max(0.08f, _droneUp);
                float glZ = -_droneNorth;

                // Match the SLAM FPV convention: ROS yaw maps to GL heading,
                // pitch is inverted into GL elevation, roll controls camera up.
                float headingRad = -_droneYawRad;
                float elevRad = -_dronePitchRad;
                float cosH = (float)Math.Cos(headingRad);
                float sinH = (float)Math.Sin(headingRad);
                float cosE = (float)Math.Cos(elevRad);
                float sinE = (float)Math.Sin(elevRad);

                var povEye = new Vector3(glX, glY, glZ);
                var forward = new Vector3(sinH * cosE, sinE, -cosH * cosE);
                if (forward.LengthSquared < 0.0001f)
                    forward = new Vector3(0f, 0f, -1f);
                forward.Normalize();

                var up = BuildRolledUpVector(forward, _droneRollRad);
                var povView = Matrix4.LookAt(povEye, povEye + forward, up);
                return (povView, proj, povEye);
            }

            float yaw = _yawDeg * (float)Math.PI / 180f;
            float pitch = MathHelperClamp(_pitchDeg, -85f, 85f) * (float)Math.PI / 180f;

            float cy = (float)Math.Cos(yaw);
            float sy = (float)Math.Sin(yaw);
            float cp = (float)Math.Cos(pitch);
            float sp = (float)Math.Sin(pitch);

            // GL frame: +Y up, +X east, -Z north (we use +Z south to keep right-handed).
            // Map ENU (east, north, up) to GL (east, up, -north).
            Vector3 dir = new Vector3(cp * sy, sp, -cp * cy);
            Vector3 eye = _panTarget + dir * _distance;

            var view = Matrix4.LookAt(eye, _panTarget, Vector3.UnitY);
            return (view, proj, eye);
        }

        private static float MathHelperToRad(float deg) => deg * (float)Math.PI / 180f;
        private static float MathHelperClamp(float v, float lo, float hi) => Math.Max(lo, Math.Min(hi, v));

        private static Vector3 BuildRolledUpVector(Vector3 forward, float rollRad)
        {
            Vector3 worldUp = Vector3.UnitY;
            Vector3 right = Vector3.Cross(forward, worldUp);
            if (right.LengthSquared < 0.0001f)
                right = Vector3.UnitX;
            right.Normalize();

            Vector3 up = Vector3.Cross(right, forward);
            up.Normalize();

            float cosR = (float)Math.Cos(rollRad);
            float sinR = (float)Math.Sin(rollRad);
            var rolled = up * cosR + right * sinR;
            if (rolled.LengthSquared < 0.0001f)
                return Vector3.UnitY;
            rolled.Normalize();
            return rolled;
        }

        // ==================== Rendering ====================

        private void GlControl_Paint(object sender, PaintEventArgs e)
        {
            try
            {
                _glControl.MakeCurrent();
                GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

                var (view, proj, _) = BuildMatrices();
                _viewProj = view * proj;

                GL.MatrixMode(MatrixMode.Projection);
                GL.LoadMatrix(ref proj);
                GL.MatrixMode(MatrixMode.Modelview);
                GL.LoadMatrix(ref view);

                DrawGroundGrid();
                DrawSearchBoundary();
                DrawBuilding();
                DrawTargets();
                DrawDronePose();
                DrawCornerLabels();

                _glControl.SwapBuffers();

                // GDI overlay (text). Drawn after SwapBuffers so it sits on top
                // of the GL surface and survives until the next Paint cycle.
                DrawOverlay2D();
            }
            catch
            {
                // Suppress paint errors so a transient GL issue doesn't crash the GCS.
            }
        }

        private void DrawGroundGrid()
        {
            GL.LineWidth(1f);
            GL.Color4(0.18f, 0.18f, 0.22f, 1f);
            GL.Begin(PrimitiveType.Lines);
            int n = Math.Max(20, (int)Math.Ceiling(_sceneHalfExtentM));
            const float step = 1f;
            for (int i = -n; i <= n; i++)
            {
                GL.Vertex3(i * step, 0f, -n * step);
                GL.Vertex3(i * step,  0f,  n * step);
                GL.Vertex3(-n * step, 0f, i * step);
                GL.Vertex3( n * step, 0f, i * step);
            }
            GL.End();

            // Origin axes (subtle).
            GL.Begin(PrimitiveType.Lines);
            GL.Color4(0.6f, 0.2f, 0.2f, 0.7f); GL.Vertex3(0, 0.01f, 0); GL.Vertex3(2, 0.01f, 0); // east
            GL.Color4(0.2f, 0.6f, 0.2f, 0.7f); GL.Vertex3(0, 0.01f, 0); GL.Vertex3(0, 0.01f, -2); // north
            GL.End();
        }

        private void DrawSearchBoundary()
        {
            float minE = _minEast - _searchBufferM;
            float maxE = _maxEast + _searchBufferM;
            float minN = _minNorth - _searchBufferM;
            float maxN = _maxNorth + _searchBufferM;

            GL.LineWidth(2f);
            GL.Color4(1.0f, 0.55f, 0.0f, 0.85f);
            GL.Begin(PrimitiveType.LineLoop);
            GL.Vertex3(minE, 0.02f, -minN);
            GL.Vertex3(maxE, 0.02f, -minN);
            GL.Vertex3(maxE, 0.02f, -maxN);
            GL.Vertex3(minE, 0.02f, -maxN);
            GL.End();
        }

        private void DrawBuilding()
        {
            if (_corners.Count < 3) return;

            // Top + bottom rings; walls as quads. ENU → GL: (east, up, -north).
            var pts = _corners.Select(c => new Vector3(c.East, 0f, -c.North)).ToArray();
            float h = _buildingHeight;

            // Floor (filled).
            GL.Color4(0.12f, 0.18f, 0.28f, 0.9f);
            GL.Begin(PrimitiveType.TriangleFan);
            float cx = pts.Average(p => p.X);
            float cz = pts.Average(p => p.Z);
            GL.Vertex3(cx, 0.005f, cz);
            for (int i = 0; i <= pts.Length; i++)
            {
                var p = pts[i % pts.Length];
                GL.Vertex3(p.X, 0.005f, p.Z);
            }
            GL.End();

            // Walls (semi-transparent so targets behind a wall are still visible).
            GL.Color4(0.30f, 0.45f, 0.65f, 0.45f);
            GL.Begin(PrimitiveType.Quads);
            for (int i = 0; i < pts.Length; i++)
            {
                var a = pts[i];
                var b = pts[(i + 1) % pts.Length];
                GL.Vertex3(a.X, 0f, a.Z);
                GL.Vertex3(b.X, 0f, b.Z);
                GL.Vertex3(b.X, h,  b.Z);
                GL.Vertex3(a.X, h,  a.Z);
            }
            GL.End();

            // Wall edges + roof outline.
            GL.LineWidth(2f);
            GL.Color4(0.65f, 0.85f, 1.0f, 1f);
            GL.Begin(PrimitiveType.Lines);
            for (int i = 0; i < pts.Length; i++)
            {
                var a = pts[i];
                var b = pts[(i + 1) % pts.Length];
                // Bottom edge.
                GL.Vertex3(a.X, 0f, a.Z); GL.Vertex3(b.X, 0f, b.Z);
                // Top edge.
                GL.Vertex3(a.X, h, a.Z);  GL.Vertex3(b.X, h, b.Z);
                // Vertical.
                GL.Vertex3(a.X, 0f, a.Z); GL.Vertex3(a.X, h, a.Z);
            }
            GL.End();
        }

        private void DrawTargets()
        {
            foreach (var t in _targets)
            {
                Vector3 p = new Vector3(t.East, t.Up, -t.North);
                Color c = ColorForTarget(t.Color);

                bool isHover = t.Id == _hoverId;
                bool isHi    = t.Id == HighlightedTargetId;

                // Ground stem so floating targets are easy to localize.
                GL.LineWidth(1.5f);
                GL.Color4(0.5f, 0.5f, 0.5f, 0.6f);
                GL.Begin(PrimitiveType.Lines);
                GL.Vertex3(p.X, 0f, p.Z);
                GL.Vertex3(p.X, p.Y, p.Z);
                GL.End();

                float radius = isHover || isHi ? 0.42f : 0.30f;
                DrawSphere(p, radius, c, isHover || isHi ? 1f : 0.9f);

                if (isHover || isHi)
                {
                    // Highlight ring around the marker.
                    GL.LineWidth(2.5f);
                    GL.Color4(1f, 1f, 0f, 0.95f);
                    GL.Begin(PrimitiveType.LineLoop);
                    for (int i = 0; i < 32; i++)
                    {
                        float a = (float)(i * 2 * Math.PI / 32);
                        GL.Vertex3(p.X + (float)Math.Cos(a) * (radius + 0.12f),
                                   p.Y,
                                   p.Z + (float)Math.Sin(a) * (radius + 0.12f));
                    }
                    GL.End();
                }
            }
        }

        private void DrawSphere(Vector3 center, float r, Color color, float alpha)
        {
            const int LAT = 8;
            const int LON = 12;
            GL.Color4(color.R / 255f, color.G / 255f, color.B / 255f, alpha);
            for (int i = 0; i < LAT; i++)
            {
                float lat0 = (float)(Math.PI * (-0.5 + (double)i / LAT));
                float lat1 = (float)(Math.PI * (-0.5 + (double)(i + 1) / LAT));
                float y0 = (float)Math.Sin(lat0) * r;
                float y1 = (float)Math.Sin(lat1) * r;
                float ring0 = (float)Math.Cos(lat0) * r;
                float ring1 = (float)Math.Cos(lat1) * r;
                GL.Begin(PrimitiveType.QuadStrip);
                for (int j = 0; j <= LON; j++)
                {
                    float lon = (float)(2 * Math.PI * (double)j / LON);
                    float cx = (float)Math.Cos(lon);
                    float sx = (float)Math.Sin(lon);
                    GL.Vertex3(center.X + cx * ring1, center.Y + y1, center.Z + sx * ring1);
                    GL.Vertex3(center.X + cx * ring0, center.Y + y0, center.Z + sx * ring0);
                }
                GL.End();
            }
        }

        private void DrawDronePose()
        {
            if (!_hasDronePose || _dronePovEnabled) return;

            float glX = _droneEast;
            float glY = Math.Max(0.08f, _droneUp);
            float glZ = -_droneNorth;

            _droneRenderer.LengthM = 0.35f;
            _droneRenderer.WidthM = 0.35f;
            _droneRenderer.HeightM = 0.08f;
            _droneRenderer.FrameType = "Quadcopter";
            _droneRenderer.ShowAvoidanceEnvelope = false;
            _droneRenderer.Draw(glX, glY, glZ, _droneYawRad, _dronePitchRad, _droneRollRad, 90f);
            GL.Disable(EnableCap.Lighting);

            GL.LineWidth(1.5f);
            GL.Color4(1.0f, 0.65f, 0.0f, 0.65f);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(glX, 0f, glZ);
            GL.Vertex3(glX, glY, glZ);
            GL.End();
        }

        // ==================== 2D overlay (compass + wall labels) ====================

        private void DrawOverlay2D()
        {
            if (_viewW <= 0 || _viewH <= 0) return;
            try
            {
                using (var g = Graphics.FromHwnd(_glControl.Handle))
                {
                    g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
                    g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.AntiAlias;
                    DrawWallLabels2D(g);
                    DrawCornerLabels2D(g);
                    DrawTargetLabels2D(g);
                    DrawViewModeLabel2D(g);
                    DrawCompass2D(g);
                }
            }
            catch { /* GDI/GL race on resize — skip this frame. */ }
        }

        private void DrawViewModeLabel2D(Graphics g)
        {
            if (!_dronePovEnabled) return;

            string text = _hasDronePose ? "DRONE POV" : "DRONE POV - WAITING FOR POSE";
            using (var font = new Font("Segoe UI", 8, FontStyle.Bold))
            {
                var size = g.MeasureString(text, font);
                var rect = new RectangleF(10, 10, size.Width + 16, size.Height + 8);
                using (var bg = new SolidBrush(Color.FromArgb(180, 18, 18, 22)))
                    g.FillRectangle(bg, rect);
                using (var pen = new Pen(Color.FromArgb(220, 255, 193, 7), 1f))
                    g.DrawRectangle(pen, rect.X, rect.Y, rect.Width, rect.Height);
                using (var brush = new SolidBrush(Color.FromArgb(255, 193, 7)))
                    g.DrawString(text, font, brush, rect.X + 8, rect.Y + 4);
            }
        }

        // Compass rose in the top-right that rotates with the camera so the user
        // always knows where world-north is. yaw=0 looks north; rotating the
        // camera right (yaw+) makes N swing left on the compass.
        private void DrawCompass2D(Graphics g)
        {
            const int RADIUS = 34;
            const int MARGIN = 12;
            float cx = _viewW - MARGIN - RADIUS;
            float cy = MARGIN + RADIUS;

            using (var bg = new SolidBrush(Color.FromArgb(160, 18, 18, 22)))
                g.FillEllipse(bg, cx - RADIUS, cy - RADIUS, RADIUS * 2, RADIUS * 2);
            using (var ring = new Pen(Color.FromArgb(220, 200, 200, 210), 1.5f))
                g.DrawEllipse(ring, cx - RADIUS, cy - RADIUS, RADIUS * 2, RADIUS * 2);

            float yaw = _yawDeg * (float)Math.PI / 180f;
            // Screen-space unit vector pointing to world-north in current view.
            float nx = -(float)Math.Sin(yaw);
            float ny = -(float)Math.Cos(yaw); // screen Y is inverted (down = +)

            // Tick marks at the 4 cardinals.
            var ticks = new (string label, float ang, Color color)[]
            {
                ("N", 0f,                       Color.FromArgb(255, 90, 90)),
                ("E", (float)(Math.PI * 0.5),   Color.White),
                ("S", (float)Math.PI,           Color.White),
                ("W", (float)(Math.PI * 1.5),   Color.White),
            };

            using (var font = new Font("Segoe UI", 8.5f, FontStyle.Bold))
            using (var pen = new Pen(Color.FromArgb(200, 200, 200, 210), 1f))
            {
                foreach (var (label, ang, color) in ticks)
                {
                    float c = (float)Math.Cos(ang);
                    float s = (float)Math.Sin(ang);
                    // Rotate the cardinal direction (originally +Y for N) by the
                    // camera yaw so N tracks the world-north direction onscreen.
                    float ux =  nx * c - ny * s;
                    float uy =  ny * c + nx * s;
                    float ix = cx + ux * (RADIUS - 3);
                    float iy = cy + uy * (RADIUS - 3);
                    float ox = cx + ux * (RADIUS - 9);
                    float oy = cy + uy * (RADIUS - 9);
                    g.DrawLine(pen, ox, oy, ix, iy);

                    float tx = cx + ux * (RADIUS - 18);
                    float ty = cy + uy * (RADIUS - 18);
                    using (var br = new SolidBrush(color))
                    {
                        var sz = g.MeasureString(label, font);
                        g.DrawString(label, font, br, tx - sz.Width / 2, ty - sz.Height / 2);
                    }
                }

                // N-pointing arrow (red) on top of ticks.
                using (var arrowPen = new Pen(Color.FromArgb(255, 90, 90), 2.2f))
                {
                    g.DrawLine(arrowPen, cx, cy, cx + nx * (RADIUS - 12), cy + ny * (RADIUS - 12));
                }
                using (var dot = new SolidBrush(Color.FromArgb(220, 220, 220)))
                    g.FillEllipse(dot, cx - 2.5f, cy - 2.5f, 5, 5);
            }
        }

        // Wall label = "{corner1}-{corner2}", drawn at the wall midpoint projected
        // to screen. Walls behind the camera or off-screen are skipped.
        private void DrawWallLabels2D(Graphics g)
        {
            if (_corners.Count < 2) return;

            using (var font = new Font("Segoe UI", 9f, FontStyle.Bold))
            using (var br = new SolidBrush(Color.FromArgb(245, 245, 250)))
            using (var shadow = new SolidBrush(Color.FromArgb(180, 0, 0, 0)))
            {
                for (int i = 0; i < _corners.Count; i++)
                {
                    var a = _corners[i];
                    var b = _corners[(i + 1) % _corners.Count];
                    string label = WallDisplayLabelForEdge(i);

                    // Mid-wall at half-height in GL coords (E, up, -N).
                    Vector3 mid = new Vector3(
                        (a.East + b.East) * 0.5f,
                        _buildingHeight * 0.5f,
                        -(a.North + b.North) * 0.5f);

                    if (!WorldToScreen(mid, out var pt)) continue;

                    var sz = g.MeasureString(label, font);
                    float x = pt.X - sz.Width / 2;
                    float y = pt.Y - sz.Height / 2;
                    g.FillRectangle(shadow, x - 3, y - 1, sz.Width + 6, sz.Height + 2);
                    g.DrawString(label, font, br, x, y);
                }
            }
        }

        private static string NameOrIndex(string name, int idx)
            => string.IsNullOrWhiteSpace(name) ? (idx + 1).ToString() : name;

        private void DrawCornerLabels2D(Graphics g)
        {
            if (_corners.Count == 0) return;

            using (var font = new Font("Segoe UI", 9.5f, FontStyle.Bold))
            using (var textBrush = new SolidBrush(Color.Black))
            using (var fill = new SolidBrush(Color.FromArgb(245, 245, 220, 80)))
            using (var border = new Pen(Color.FromArgb(230, 40, 40, 35), 1f))
            {
                for (int i = 0; i < _corners.Count; i++)
                {
                    var c = _corners[i];
                    if (!WorldToScreen(new Vector3(c.East, 0.35f, -c.North), out var pt)) continue;

                    string label = (i + 1).ToString();
                    var sz = g.MeasureString(label, font);
                    float diameter = Math.Max(20f, Math.Max(sz.Width, sz.Height) + 7f);
                    float x = pt.X - diameter / 2f;
                    float y = pt.Y - diameter / 2f;
                    g.FillEllipse(fill, x, y, diameter, diameter);
                    g.DrawEllipse(border, x, y, diameter, diameter);
                    g.DrawString(label, font, textBrush, pt.X - sz.Width / 2f, pt.Y - sz.Height / 2f);
                }
            }
        }

        private void DrawTargetLabels2D(Graphics g)
        {
            if (_targets.Count == 0) return;

            using (var font = new Font("Segoe UI", 8.5f, FontStyle.Bold))
            using (var textBrush = new SolidBrush(Color.White))
            using (var borderPen = new Pen(Color.FromArgb(230, 245, 245, 245), 1f))
            using (var shadow = new SolidBrush(Color.FromArgb(205, 0, 0, 0)))
            {
                foreach (var t in _targets)
                {
                    string id = string.IsNullOrWhiteSpace(t.Id) ? "?" : t.Id.Trim();
                    string color = string.IsNullOrWhiteSpace(t.Color) ? "Unknown" : t.Color.Trim();
                    string label = $"{id} {color}";
                    float labelUp = Math.Max(t.Up, 0.15f) + 0.45f;
                    if (!WorldToScreen(new Vector3(t.East, labelUp, -t.North), out var pt)) continue;

                    var sz = g.MeasureString(label, font);
                    float x = pt.X - sz.Width / 2f;
                    float y = pt.Y - sz.Height - 8f;
                    var rect = new RectangleF(x - 5f, y - 2f, sz.Width + 10f, sz.Height + 4f);

                    using (var fill = new SolidBrush(Color.FromArgb(225, ColorForTarget(t.Color))))
                    {
                        g.FillRectangle(shadow, rect.X + 1f, rect.Y + 1f, rect.Width, rect.Height);
                        g.FillRectangle(fill, rect);
                    }
                    g.DrawRectangle(borderPen, rect.X, rect.Y, rect.Width, rect.Height);
                    g.DrawString(label, font, textBrush, x, y);
                }
            }
        }

        private bool WorldToScreen(Vector3 world, out PointF screen)
        {
            screen = PointF.Empty;
            Vector4 clip = Vector4.Transform(new Vector4(world, 1f), _viewProj);
            if (clip.W <= 0.0001f) return false;
            float ndcX = clip.X / clip.W;
            float ndcY = clip.Y / clip.W;
            if (ndcX < -1.2f || ndcX > 1.2f || ndcY < -1.2f || ndcY > 1.2f) return false;
            screen = new PointF(
                (ndcX * 0.5f + 0.5f) * _viewW,
                (1f - (ndcY * 0.5f + 0.5f)) * _viewH);
            return true;
        }

        private void DrawCornerLabels()
        {
            if (_corners.Count == 0) return;
            // Small spheres at each corner to help orient the user.
            foreach (var c in _corners)
            {
                DrawSphere(new Vector3(c.East, 0.05f, -c.North), 0.18f,
                    Color.FromArgb(220, 220, 80), 0.95f);
            }
        }

        private static Color ColorForTarget(string name)
        {
            if (string.IsNullOrEmpty(name)) return Color.OrangeRed;
            switch (name.Trim().ToLowerInvariant())
            {
                case "red": return Color.FromArgb(230, 60, 60);
                case "blue": return Color.FromArgb(60, 130, 230);
                case "green": return Color.FromArgb(60, 200, 90);
                case "yellow": return Color.FromArgb(240, 220, 50);
                case "orange": return Color.FromArgb(240, 140, 30);
                case "purple": return Color.FromArgb(180, 80, 200);
                case "white": return Color.FromArgb(230, 230, 230);
                case "black": return Color.FromArgb(60, 60, 60);
                default: return Color.FromArgb(200, 200, 200);
            }
        }

        // ==================== Mouse + picking ====================

        private void GlControl_MouseDown(object sender, MouseEventArgs e)
        {
            _lastMouse = e.Location;
            _mouseDownPoint = e.Location;
            _dragButton = e.Button;
            _glControl.Focus();
        }

        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            var key = keyData & Keys.KeyCode;
            if (IsNavigationKey(key))
            {
                _glControl.Focus();
                SetNavigationKey(key, true);
                return true;
            }
            return base.ProcessCmdKey(ref msg, keyData);
        }

        private void GlControl_PreviewKeyDown(object sender, PreviewKeyDownEventArgs e)
        {
            if (IsNavigationKey(e.KeyCode))
                e.IsInputKey = true;
        }

        private static bool IsNavigationKey(Keys key)
        {
            return key == Keys.W || key == Keys.A || key == Keys.S || key == Keys.D
                || key == Keys.Up || key == Keys.Down || key == Keys.Home;
        }

        private void SetNavigationKey(Keys key, bool down)
        {
            if (_dronePovEnabled)
                return;

            if (key == Keys.Home && down)
            {
                _panTarget = Vector3.Zero;
                _heldKeys.Clear();
                _navTimer.Stop();
                _glControl.Invalidate();
                return;
            }

            if (!IsNavigationKey(key) || key == Keys.Home)
                return;

            if (down) _heldKeys.Add(key);
            else _heldKeys.Remove(key);

            if (_heldKeys.Count > 0 && !_navTimer.Enabled)
                _navTimer.Start();
            else if (_heldKeys.Count == 0 && _navTimer.Enabled)
                _navTimer.Stop();
        }

        private void GlControl_KeyDown(object sender, KeyEventArgs e)
        {
            if (!IsNavigationKey(e.KeyCode)) return;
            SetNavigationKey(e.KeyCode, true);
            e.Handled = true;
            e.SuppressKeyPress = true;
        }

        private void GlControl_KeyUp(object sender, KeyEventArgs e)
        {
            if (!IsNavigationKey(e.KeyCode)) return;
            SetNavigationKey(e.KeyCode, false);
            e.Handled = true;
            e.SuppressKeyPress = true;
        }

        private void NavTimer_Tick(object sender, EventArgs e)
        {
            if (_dronePovEnabled)
            {
                _heldKeys.Clear();
                _navTimer.Stop();
                return;
            }

            if (_heldKeys.Count == 0)
            {
                _navTimer.Stop();
                return;
            }

            float yaw = _yawDeg * (float)Math.PI / 180f;
            Vector3 forward = new Vector3(-(float)Math.Sin(yaw), 0f, (float)Math.Cos(yaw));
            Vector3 right = new Vector3((float)Math.Cos(yaw), 0f, (float)Math.Sin(yaw));
            Vector3 delta = Vector3.Zero;

            if (_heldKeys.Contains(Keys.W)) delta += forward;
            if (_heldKeys.Contains(Keys.S)) delta -= forward;
            if (_heldKeys.Contains(Keys.D)) delta -= right;
            if (_heldKeys.Contains(Keys.A)) delta += right;
            if (_heldKeys.Contains(Keys.Up)) delta += Vector3.UnitY;
            if (_heldKeys.Contains(Keys.Down)) delta -= Vector3.UnitY;

            if (delta.LengthSquared <= 0.0001f)
                return;

            delta.Normalize();
            float speed = Math.Max(1.5f, _distance * 0.55f);
            var modifiers = Control.ModifierKeys;
            if ((modifiers & Keys.Shift) == Keys.Shift) speed *= 3.0f;
            if ((modifiers & Keys.Control) == Keys.Control) speed *= 0.25f;

            _panTarget += delta * speed * (_navTimer.Interval / 1000f);
            _glControl.Invalidate();
        }

        private void GlControl_MouseUp(object sender, MouseEventArgs e)
        {
            if (_dragButton == MouseButtons.Left)
            {
                int dx = e.X - _mouseDownPoint.X;
                int dy = e.Y - _mouseDownPoint.Y;
                bool isClick = dx * dx + dy * dy <= 25;
                if (isClick)
                {
                    if (PlacementMode)
                    {
                        var placement = PickPlacement(e.Location);
                        if (placement != null)
                            try { PlacementClicked?.Invoke(placement); } catch { }
                    }
                    else
                    {
                        // Click on (or very near) a target marker selects it
                        // in the submission table. We pick BEFORE clearing the
                        // drag state so the hit-test uses the same screen
                        // projection the user just saw.
                        string id = PickTarget(e.Location);
                        if (!string.IsNullOrEmpty(id))
                            try { TargetClicked?.Invoke(id); } catch { }
                    }
                }
            }
            _dragButton = MouseButtons.None;
        }

        private void GlControl_MouseMove(object sender, MouseEventArgs e)
        {
            int dx = e.X - _lastMouse.X;
            int dy = e.Y - _lastMouse.Y;
            _lastMouse = e.Location;

            if (_dronePovEnabled)
            {
                if (_dragButton == MouseButtons.None)
                    SetHover(PickTarget(e.Location));
                return;
            }

            if (_dragButton == MouseButtons.Left)
            {
                _yawDeg = (_yawDeg + dx * 0.4f) % 360f;
                _pitchDeg = MathHelperClamp(_pitchDeg + dy * 0.3f, -85f, 85f);
                _glControl.Invalidate();
            }
            else if (_dragButton == MouseButtons.Right || _dragButton == MouseButtons.Middle)
            {
                // Pan in the screen plane proportional to distance.
                float scale = _distance * 0.0025f;
                float yaw = _yawDeg * (float)Math.PI / 180f;
                Vector3 right = new Vector3((float)Math.Cos(yaw), 0, (float)Math.Sin(yaw));
                _panTarget -= right * (dx * scale);
                _panTarget += new Vector3(0, dy * scale, 0);
                _glControl.Invalidate();
            }
            else
            {
                // Hover detection — only when the mouse is idle.
                var hit = PickTarget(e.Location);
                SetHover(hit);
            }
        }

        private void GlControl_MouseWheel(object sender, MouseEventArgs e)
        {
            if (_dronePovEnabled) return;

            float factor = e.Delta > 0 ? 0.85f : 1.18f;
            _distance = MathHelperClamp(_distance * factor, 4f, 120f);
            _glControl.Invalidate();
        }

        private void SetHover(string id)
        {
            if (_hoverId == id) return;
            _hoverId = id;
            _glControl.Invalidate();
            try { TargetHovered?.Invoke(id); } catch { }
        }

        /// <summary>
        /// Project every target into screen space and return the id of the
        /// nearest one within 14 pixels of the cursor — null otherwise.
        /// </summary>
        private string PickTarget(Point screen)
        {
            if (_targets.Count == 0 || _viewW == 0 || _viewH == 0) return null;

            string best = null;
            float bestDist = 14f * 14f;

            foreach (var t in _targets)
            {
                Vector4 p = new Vector4(t.East, t.Up, -t.North, 1f);
                Vector4 clip = Vector4.Transform(p, _viewProj);
                if (clip.W <= 0.0001f) continue;
                float ndcX = clip.X / clip.W;
                float ndcY = clip.Y / clip.W;
                if (ndcX < -1f || ndcX > 1f || ndcY < -1f || ndcY > 1f) continue;

                float sx = (ndcX * 0.5f + 0.5f) * _viewW;
                float sy = (1f - (ndcY * 0.5f + 0.5f)) * _viewH;
                float dx = sx - screen.X;
                float dy = sy - screen.Y;
                float d2 = dx * dx + dy * dy;
                if (d2 < bestDist)
                {
                    bestDist = d2;
                    best = t.Id;
                }
            }
            return best;
        }

        private Placement PickPlacement(Point screen)
        {
            if (_corners.Count < 3 || _viewW == 0 || _viewH == 0) return null;
            if (!TryBuildPickRay(screen, out var origin, out var dir)) return null;

            Placement best = null;
            float bestT = float.MaxValue;

            TryPickHorizontal("ground", 0f, origin, dir, false, ref best, ref bestT);
            TryPickHorizontal("roof", _buildingHeight, origin, dir, true, ref best, ref bestT);
            TryPickWalls(origin, dir, ref best, ref bestT);

            return best;
        }

        private bool TryBuildPickRay(Point screen, out Vector3 origin, out Vector3 dir)
        {
            origin = Vector3.Zero;
            dir = Vector3.Zero;

            float ndcX = (2f * screen.X) / Math.Max(1, _viewW) - 1f;
            float ndcY = 1f - (2f * screen.Y) / Math.Max(1, _viewH);

            Matrix4 viewProj = _viewProj;
            Matrix4.Invert(ref viewProj, out var inv);
            var near = Vector4.Transform(new Vector4(ndcX, ndcY, -1f, 1f), inv);
            var far = Vector4.Transform(new Vector4(ndcX, ndcY, 1f, 1f), inv);
            if (Math.Abs(near.W) < 0.0001f || Math.Abs(far.W) < 0.0001f) return false;
            near /= near.W;
            far /= far.W;

            origin = new Vector3(near.X, near.Y, near.Z);
            dir = Vector3.Normalize(new Vector3(far.X - near.X, far.Y - near.Y, far.Z - near.Z));
            return dir.LengthSquared > 0.0001f;
        }

        private void TryPickHorizontal(string surface, float y, Vector3 origin, Vector3 dir,
            bool requireInsideFootprint, ref Placement best, ref float bestT)
        {
            if (Math.Abs(dir.Y) < 0.0001f) return;
            float t = (y - origin.Y) / dir.Y;
            if (t <= 0f || t >= bestT) return;

            var p = origin + dir * t;
            bool inside = PointInsideFootprint(p.X, p.Z);
            if (requireInsideFootprint && !inside) return;
            if (!requireInsideFootprint && DistanceToFootprintSquared(p.X, p.Z) > _searchBufferM * _searchBufferM) return;

            bestT = t;
            best = BuildPlacement(surface, null, p);
        }

        private void TryPickWalls(Vector3 origin, Vector3 dir, ref Placement best, ref float bestT)
        {
            for (int i = 0; i < _corners.Count; i++)
            {
                var ca = _corners[i];
                var cb = _corners[(i + 1) % _corners.Count];
                var a = new Vector3(ca.East, 0f, -ca.North);
                var b = new Vector3(cb.East, 0f, -cb.North);
                var edge = b - a;
                if (edge.LengthSquared < 0.0001f) continue;

                var normal = Vector3.Normalize(new Vector3(edge.Z, 0f, -edge.X));
                float denom = Vector3.Dot(dir, normal);
                if (Math.Abs(denom) < 0.0001f) continue;

                float t = Vector3.Dot(a - origin, normal) / denom;
                if (t <= 0f || t >= bestT) continue;

                var p = origin + dir * t;
                if (p.Y < -0.01f || p.Y > _buildingHeight + 0.01f) continue;

                float along = Vector3.Dot(p - a, edge) / edge.LengthSquared;
                if (along < -0.01f || along > 1.01f) continue;

                bestT = t;
                best = BuildPlacement("wall", i, p);
            }
        }

        private Placement BuildPlacement(string surface, int? wallIndex, Vector3 glPoint)
        {
            float east = glPoint.X;
            float north = -glPoint.Z;
            int faceIndex = wallIndex ?? NearestWallIndex(east, north);
            string nearestName = "";
            float nearest = float.MaxValue;

            for (int i = 0; i < _corners.Count; i++)
            {
                var c = _corners[i];
                float d2 = Distance2(east, north, c.East, c.North);
                if (d2 < nearest)
                {
                    nearest = d2;
                    nearestName = NameOrIndex(c.Name, i);
                }
            }

            var reference = ReferenceWallForPlacement(faceIndex, east, north);
            bool onProtrusion = IsProtrudingWallIndex(faceIndex) ||
                ((surface == "roof" || surface == "ground") && PointInProtrudingPortion(east, north));

            return new Placement
            {
                Surface = surface,
                WallName = WallLabelForEdge(faceIndex),
                NearestCornerName = nearestName,
                East = east,
                North = north,
                Up = Math.Max(0f, Math.Min(_buildingHeight, glPoint.Y)),
                DistanceFromCornerM = (float)Math.Sqrt(Math.Max(0f, nearest)),
                StructureLabel = onProtrusion ? "portion of the building that sticks out" : "building",
                ReferenceWallName = reference.wall,
                DistanceFromReferenceWallM = reference.distance,
            };
        }

        private bool PointInsideFootprint(float glX, float glZ)
        {
            bool inside = false;
            for (int i = 0, j = _corners.Count - 1; i < _corners.Count; j = i++)
            {
                float xi = _corners[i].East;
                float zi = -_corners[i].North;
                float xj = _corners[j].East;
                float zj = -_corners[j].North;
                bool intersect = ((zi > glZ) != (zj > glZ)) &&
                    (glX < (xj - xi) * (glZ - zi) / ((zj - zi) == 0f ? 0.0001f : (zj - zi)) + xi);
                if (intersect) inside = !inside;
            }
            return inside;
        }

        private float DistanceToFootprintSquared(float glX, float glZ)
        {
            if (PointInsideFootprint(glX, glZ)) return 0f;
            float best = float.MaxValue;
            for (int i = 0; i < _corners.Count; i++)
            {
                var a = _corners[i];
                var b = _corners[(i + 1) % _corners.Count];
                float ax = a.East, az = -a.North;
                float bx = b.East, bz = -b.North;
                float vx = bx - ax, vz = bz - az;
                float len2 = vx * vx + vz * vz;
                if (len2 < 0.0001f) continue;
                float t = ((glX - ax) * vx + (glZ - az) * vz) / len2;
                t = Math.Max(0f, Math.Min(1f, t));
                float px = ax + vx * t;
                float pz = az + vz * t;
                best = Math.Min(best, Distance2(glX, glZ, px, pz));
            }
            return best;
        }

        private string NearestWallLabel(float east, float north)
            => WallLabelForEdge(NearestWallIndex(east, north));

        private int NearestWallIndex(float east, float north)
        {
            if (_corners.Count < 2) return 0;
            int bestIdx = 0;
            float best = float.MaxValue;
            for (int i = 0; i < _corners.Count; i++)
            {
                var a = _corners[i];
                var b = _corners[(i + 1) % _corners.Count];
                float vx = b.East - a.East;
                float vy = b.North - a.North;
                float len2 = vx * vx + vy * vy;
                if (len2 < 0.0001f) continue;
                float t = ((east - a.East) * vx + (north - a.North) * vy) / len2;
                t = Math.Max(0f, Math.Min(1f, t));
                float px = a.East + vx * t;
                float py = a.North + vy * t;
                float d = Distance2(east, north, px, py);
                if (d < best)
                {
                    best = d;
                    bestIdx = i;
                }
            }
            return bestIdx;
        }

        private (string wall, float distance) ReferenceWallForPlacement(int edgeIndex, float east, float north)
        {
            if (_corners.Count < 2)
                return ("nearest", 0f);

            var a = _corners[edgeIndex];
            var b = _corners[(edgeIndex + 1) % _corners.Count];
            float vx = b.East - a.East;
            float vy = b.North - a.North;
            float len2 = Math.Max(0.0001f, vx * vx + vy * vy);
            float t = ((east - a.East) * vx + (north - a.North) * vy) / len2;
            t = Math.Max(0f, Math.Min(1f, t));
            float px = a.East + vx * t;
            float py = a.North + vy * t;

            string face = WallLabelForEdge(edgeIndex);
            bool eastWestFace = face.Contains("north") || face.Contains("south");
            bool useFirst = eastWestFace
                ? a.East <= b.East
                : a.North <= b.North;
            var refCorner = useFirst ? a : b;
            string refWall = eastWestFace ? "western" : "southern";
            float distance = (float)Math.Sqrt(Distance2(px, py, refCorner.East, refCorner.North));
            return (refWall, distance);
        }

        private string WallLabelForEdge(int i)
        {
            if (_corners.Count < 2) return "wall";
            // The protrusion's two side faces literally face north and south,
            // but they belong to the west wall of the building — so report them
            // under the parent wall direction instead of their local normal.
            if (IsProtrudingWallIndex(i))
                return ProtrudingSectionParentWall() ?? EdgeOutwardCompass(i);
            return EdgeOutwardCompass(i);
        }

        // Compass label from the edge's outward normal — robust to concavities
        // (the previous midpoint-vs-centroid heuristic mislabels edges of any
        // section that protrudes past the polygon's centroid).
        private string EdgeOutwardCompass(int i)
        {
            if (!TryEdgeOutwardNormal(i, out float ox, out float oy))
                return "wall";
            float heading = (float)((90.0 - Math.Atan2(oy, ox) * 180.0 / Math.PI + 360.0) % 360.0);
            return SnapHeadingToCompass(heading);
        }

        private bool TryEdgeOutwardNormal(int i, out float nx, out float ny)
        {
            nx = ny = 0f;
            int n = _corners.Count;
            if (n < 2) return false;
            var a = _corners[i];
            var b = _corners[(i + 1) % n];
            float vx = b.East - a.East;
            float vy = b.North - a.North;
            float len = (float)Math.Sqrt(vx * vx + vy * vy);
            if (len < 1e-6f) return false;
            // Perpendicular candidate.
            nx = vy / len;
            ny = -vx / len;
            // Flip if it points into the polygon interior.
            float mx = (a.East + b.East) * 0.5f;
            float my = (a.North + b.North) * 0.5f;
            const float probe = 0.5f;
            if (PointInPolygon(mx + nx * probe, my + ny * probe))
            {
                nx = -nx;
                ny = -ny;
            }
            return true;
        }

        // Dominant compass direction of the protrusion: sum of outward normals
        // across its edges. For a westward bump, the south + north side normals
        // cancel and the outer (west) face dominates → "west".
        private string ProtrudingSectionParentWall()
        {
            int n = _corners.Count;
            float sx = 0, sy = 0;
            for (int i = 0; i < n; i++)
            {
                if (!IsProtrudingWallIndex(i)) continue;
                if (!TryEdgeOutwardNormal(i, out float nx, out float ny)) continue;
                sx += nx;
                sy += ny;
            }
            if (sx * sx + sy * sy < 1e-6f) return null;
            float heading = (float)((90.0 - Math.Atan2(sy, sx) * 180.0 / Math.PI + 360.0) % 360.0);
            return SnapHeadingToCompass(heading);
        }

        private bool PointInPolygon(float px, float py)
        {
            bool inside = false;
            int n = _corners.Count;
            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                var pi = _corners[i];
                var pj = _corners[j];
                if (((pi.North > py) != (pj.North > py)) &&
                    (px < (pj.East - pi.East) * (py - pi.North) / (pj.North - pi.North) + pi.East))
                    inside = !inside;
            }
            return inside;
        }

        private string WallDisplayLabelForEdge(int i)
        {
            string face = WallLabelForEdge(i);
            return IsProtrudingWallIndex(i)
                ? $"{face} - portion that sticks out"
                : face;
        }

        private bool IsProtrudingWallIndex(int edgeIndex)
        {
            return _corners.Count == 8 && edgeIndex >= 4 && edgeIndex <= 6;
        }

        private bool PointInProtrudingPortion(float east, float north)
        {
            if (_corners.Count != 8) return false;
            var protrusion = new[] { _corners[4], _corners[5], _corners[6], _corners[7] };
            float minE = protrusion.Min(c => c.East) - 0.05f;
            float maxE = protrusion.Max(c => c.East) + 0.05f;
            float minN = protrusion.Min(c => c.North) - 0.05f;
            float maxN = protrusion.Max(c => c.North) + 0.05f;
            return east >= minE && east <= maxE && north >= minN && north <= maxN;
        }

        private float EdgeHeading(int i)
        {
            var a = _corners[i];
            var b = _corners[(i + 1) % _corners.Count];
            float cx = _corners.Average(c => c.East);
            float cy = _corners.Average(c => c.North);
            float ox = (a.East + b.East) * 0.5f - cx;
            float oy = (a.North + b.North) * 0.5f - cy;
            return (float)((90.0 - Math.Atan2(oy, ox) * 180.0 / Math.PI + 360.0) % 360.0);
        }

        private static string SnapHeadingToCompass(float headingDeg)
        {
            string[] names = { "north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest" };
            int idx = (int)Math.Round((((headingDeg % 360f) + 360f) % 360f) / 45f) % 8;
            return names[idx];
        }

        private void UpdateSceneBounds()
        {
            if (_corners.Count == 0)
            {
                _minEast = _minNorth = -10f;
                _maxEast = _maxNorth = 10f;
                _sceneHalfExtentM = 25f;
                _distance = Math.Max(_distance, 25f);
                return;
            }
            _minEast = _corners.Min(c => c.East);
            _maxEast = _corners.Max(c => c.East);
            _minNorth = _corners.Min(c => c.North);
            _maxNorth = _corners.Max(c => c.North);
            float halfW = Math.Max(Math.Abs(_minEast), Math.Abs(_maxEast));
            float halfH = Math.Max(Math.Abs(_minNorth), Math.Abs(_maxNorth));
            _sceneHalfExtentM = Math.Max(25f, Math.Max(halfW, halfH) + _searchBufferM + 5f);
            _distance = Math.Max(_distance, _sceneHalfExtentM * 1.4f);
        }

        private static float Distance2(float x1, float y1, float x2, float y2)
        {
            float dx = x1 - x2;
            float dy = y1 - y2;
            return dx * dx + dy * dy;
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
