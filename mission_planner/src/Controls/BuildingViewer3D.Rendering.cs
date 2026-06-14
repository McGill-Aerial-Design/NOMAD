// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// BuildingViewer3D.Rendering.cs - OpenGL rendering pipeline
// ============================================================

using System;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using NOMAD.MissionPlanner.SLAM3D.Rendering;

namespace NOMAD.MissionPlanner
{
    public partial class BuildingViewer3D
    {
        // ==================== OpenGL Setup ====================

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

                DrawOverlay2D();
            }
            catch { }
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

            GL.Begin(PrimitiveType.Lines);
            GL.Color4(0.6f, 0.2f, 0.2f, 0.7f); GL.Vertex3(0, 0.01f, 0); GL.Vertex3(2, 0.01f, 0);
            GL.Color4(0.2f, 0.6f, 0.2f, 0.7f); GL.Vertex3(0, 0.01f, 0); GL.Vertex3(0, 0.01f, -2);
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

            var pts = _corners.Select(c => new Vector3(c.East, 0f, -c.North)).ToArray();
            float h = _buildingHeight;

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

            GL.LineWidth(2f);
            GL.Color4(0.65f, 0.85f, 1.0f, 1f);
            GL.Begin(PrimitiveType.Lines);
            for (int i = 0; i < pts.Length; i++)
            {
                var a = pts[i];
                var b = pts[(i + 1) % pts.Length];
                GL.Vertex3(a.X, 0f, a.Z); GL.Vertex3(b.X, 0f, b.Z);
                GL.Vertex3(a.X, h, a.Z);  GL.Vertex3(b.X, h, b.Z);
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

        private void DrawCornerLabels()
        {
            if (_corners.Count == 0) return;
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

        // ==================== 2D overlay ====================

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
            catch { }
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
            float nx = -(float)Math.Sin(yaw);
            float ny = -(float)Math.Cos(yaw);

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

                using (var arrowPen = new Pen(Color.FromArgb(255, 90, 90), 2.2f))
                {
                    g.DrawLine(arrowPen, cx, cy, cx + nx * (RADIUS - 12), cy + ny * (RADIUS - 12));
                }
                using (var dot = new SolidBrush(Color.FromArgb(220, 220, 220)))
                    g.FillEllipse(dot, cx - 2.5f, cy - 2.5f, 5, 5);
            }
        }

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
    }
}
