// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    internal class JoystickPad : Control
    {
        public event Action<float, float> StickChanged;

        private bool _dragging;
        private PointF _stickNorm; // [-1,1] each axis
        private readonly float _drawDeadzone;

        // Gutters reserved between the ring and the control edge so the axis
        // labels sit OUTSIDE the circle yet stay within the pad; the X gutter is
        // wider because "ROLL+/ROLL-" are wider than they are tall. The radius is
        // also capped so the joystick stays a sensible size on a large pad. Both
        // OnPaint and the hit-test use Radius() so visual + interaction agree.
        private const int LABEL_MARGIN_X = 38;
        private const int LABEL_MARGIN_Y = 18;
        private const int MAX_RADIUS = 120;
        private const int PUCK = 16;

        public JoystickPad(float drawDeadzone)
        {
            _drawDeadzone = drawDeadzone;
            DoubleBuffered = true;
            SetStyle(ControlStyles.UserPaint | ControlStyles.AllPaintingInWmPaint
                     | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
        }

        private int Radius()
        {
            int r = Math.Min(Width / 2 - LABEL_MARGIN_X, Height / 2 - LABEL_MARGIN_Y);
            return Math.Min(r, MAX_RADIUS);
        }

        protected override void OnMouseDown(MouseEventArgs e)
        {
            if (e.Button != MouseButtons.Left) return;
            _dragging = true;
            UpdateFromMouse(e.Location);
            base.OnMouseDown(e);
        }

        protected override void OnMouseMove(MouseEventArgs e)
        {
            if (_dragging) UpdateFromMouse(e.Location);
            base.OnMouseMove(e);
        }

        protected override void OnMouseUp(MouseEventArgs e)
        {
            _dragging = false;
            _stickNorm = PointF.Empty;
            StickChanged?.Invoke(0, 0);
            Invalidate();
            base.OnMouseUp(e);
        }

        protected override void OnMouseLeave(EventArgs e)
        {
            if (_dragging)
            {
                _dragging = false;
                _stickNorm = PointF.Empty;
                StickChanged?.Invoke(0, 0);
                Invalidate();
            }
            base.OnMouseLeave(e);
        }

        private void UpdateFromMouse(Point p)
        {
            int cx = Width / 2, cy = Height / 2;
            int r = Radius();
            if (r <= 0) return;
            float dx = (p.X - cx) / (float)r;
            float dy = (p.Y - cy) / (float)r;
            float mag = (float)Math.Sqrt(dx * dx + dy * dy);
            if (mag > 1f) { dx /= mag; dy /= mag; }
            _stickNorm = new PointF(dx, dy);
            // Up-in-pixels means -Y, but operator expects "stick forward = look down" or
            // "stick up = look up". Convention: stick UP (negative pixel-y) → pitch UP (+).
            StickChanged?.Invoke(dx, -dy);
            Invalidate();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.Clear(BackColor);

            int cx = Width / 2, cy = Height / 2;
            int r = Radius();
            if (r <= 0) return;

            // Outer ring
            using (var ringPen = new Pen(NOMADTheme.TEXT_SECONDARY, 2))
                g.DrawEllipse(ringPen, cx - r, cy - r, r * 2, r * 2);

            // Inner cross + deadzone
            using (var p2 = new Pen(Color.FromArgb(60, 60, 70), 1))
            {
                g.DrawLine(p2, cx - r, cy, cx + r, cy);
                g.DrawLine(p2, cx, cy - r, cx, cy + r);
                int dz = (int)(r * _drawDeadzone);
                g.DrawEllipse(p2, cx - dz, cy - dz, dz * 2, dz * 2);
            }

            // Stick puck
            int px = cx + (int)(_stickNorm.X * r);
            int py = cy + (int)(_stickNorm.Y * r);
            using (var brush = new SolidBrush(NOMADTheme.ACCENT))
                g.FillEllipse(brush, px - PUCK, py - PUCK, PUCK * 2, PUCK * 2);
            using (var pen = new Pen(Color.White, 2))
                g.DrawEllipse(pen, px - PUCK, py - PUCK, PUCK * 2, PUCK * 2);

            // Axis labels — drawn just OUTSIDE the ring, in the reserved gutter,
            // so they sit clear of the circle without spilling past the pad edge.
            using (var brush = new SolidBrush(NOMADTheme.TEXT_SECONDARY))
            using (var f = new Font(NOMADTheme.FONT_FAMILY, NOMADTheme.SIZE_SMALL))
            {
                var p = g.MeasureString("PITCH+", f);
                var roll = g.MeasureString("ROLL+", f);
                g.DrawString("PITCH+", f, brush, cx - p.Width / 2, cy - r - p.Height + 1);
                g.DrawString("PITCH-", f, brush, cx - p.Width / 2, cy + r + 1);
                g.DrawString("ROLL+", f, brush, cx - r - roll.Width - 1, cy - roll.Height / 2);
                g.DrawString("ROLL-", f, brush, cx + r + 1, cy - roll.Height / 2);
            }
        }
    }
}
