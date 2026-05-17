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

        // ==================== Public API ====================

        public event Action<string> TargetHovered;
        public string HighlightedTargetId { get; private set; }

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
                _glControl?.Invalidate();
                return;
            }

            double lat0 = centerLat ?? corners.Average(c => c.Lat);
            double lon0 = centerLon ?? corners.Average(c => c.Lon);
            double cosLat = Math.Cos(lat0 * Math.PI / 180.0);
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
            foreach (var c in _corners) { c.East -= cx; c.North -= cy; }

            _glControl?.Invalidate();
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

        private float _buildingHeight = 5f;

        // Orbit camera state.
        private float _yawDeg = 35f;
        private float _pitchDeg = 30f;
        private float _distance = 25f;
        private Vector3 _panTarget = Vector3.Zero;

        private Point _lastMouse;
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
            };
            _glControl.Load += GlControl_Load;
            _glControl.Resize += GlControl_Resize;
            _glControl.Paint += GlControl_Paint;
            _glControl.MouseDown += GlControl_MouseDown;
            _glControl.MouseUp += GlControl_MouseUp;
            _glControl.MouseMove += GlControl_MouseMove;
            _glControl.MouseLeave += (s, e) => SetHover(null);
            _glControl.MouseWheel += GlControl_MouseWheel;

            Controls.Add(_glControl);
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

            float aspect = _viewW > 0 && _viewH > 0 ? (float)_viewW / _viewH : 1f;
            var proj = Matrix4.CreatePerspectiveFieldOfView(
                MathHelperToRad(55f), aspect, 0.1f, 500f);
            var view = Matrix4.LookAt(eye, _panTarget, Vector3.UnitY);
            return (view, proj, eye);
        }

        private static float MathHelperToRad(float deg) => deg * (float)Math.PI / 180f;
        private static float MathHelperClamp(float v, float lo, float hi) => Math.Max(lo, Math.Min(hi, v));

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
                DrawSurroundCircle(15f);
                DrawBuilding();
                DrawTargets();
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
            const int N = 20;
            const float step = 1f;
            for (int i = -N; i <= N; i++)
            {
                GL.Vertex3(i * step, 0f, -N * step);
                GL.Vertex3(i * step,  0f,  N * step);
                GL.Vertex3(-N * step, 0f, i * step);
                GL.Vertex3( N * step, 0f, i * step);
            }
            GL.End();

            // Origin axes (subtle).
            GL.Begin(PrimitiveType.Lines);
            GL.Color4(0.6f, 0.2f, 0.2f, 0.7f); GL.Vertex3(0, 0.01f, 0); GL.Vertex3(2, 0.01f, 0); // east
            GL.Color4(0.2f, 0.6f, 0.2f, 0.7f); GL.Vertex3(0, 0.01f, 0); GL.Vertex3(0, 0.01f, -2); // north
            GL.End();
        }

        private void DrawSurroundCircle(float radius)
        {
            GL.LineWidth(2f);
            GL.Color4(1.0f, 0.55f, 0.0f, 0.85f);
            GL.Begin(PrimitiveType.LineLoop);
            const int SEG = 96;
            for (int i = 0; i < SEG; i++)
            {
                float a = (float)(i * 2 * Math.PI / SEG);
                GL.Vertex3((float)Math.Cos(a) * radius, 0.02f, -(float)Math.Sin(a) * radius);
            }
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
                    DrawCompass2D(g);
                }
            }
            catch { /* GDI/GL race on resize — skip this frame. */ }
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
                    string label = $"{NameOrIndex(a.Name, i)}-{NameOrIndex(b.Name, (i + 1) % _corners.Count)}";

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
            => string.IsNullOrWhiteSpace(name) ? ((char)('A' + (idx % 26))).ToString() : name;

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
            _dragButton = e.Button;
            _glControl.Focus();
        }

        private void GlControl_MouseUp(object sender, MouseEventArgs e)
        {
            _dragButton = MouseButtons.None;
        }

        private void GlControl_MouseMove(object sender, MouseEventArgs e)
        {
            int dx = e.X - _lastMouse.X;
            int dy = e.Y - _lastMouse.Y;
            _lastMouse = e.Location;

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

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                try { _glControl?.Dispose(); } catch { }
            }
            base.Dispose(disposing);
        }
    }
}
