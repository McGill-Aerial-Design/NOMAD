// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// MAVLink Link Status — throughput sparkline
// ============================================================
// Internal lightweight Panel that plots a rolling throughput history.
// Used by LinkCard inside LinkHealthPanel.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    // ================================================================
    // Throughput sparkline
    // ================================================================

    internal class SparklinePanel : Panel
    {
        private readonly LinkedList<double> _values = new LinkedList<double>();
        private Color _lineColor = NOMADTheme.ACCENT;
        private const int MAX_POINTS = 80;

        public SparklinePanel() { DoubleBuffered = true; }

        public void Push(double value, Color color)
        {
            _values.AddLast(value);
            while (_values.Count > MAX_POINTS) _values.RemoveFirst();
            _lineColor = color;
            Invalidate();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.Clear(BackColor);

            // baseline
            using (var basePen = new Pen(Color.FromArgb(60, 60, 60)))
            {
                g.DrawLine(basePen, 0, ClientSize.Height - 1, ClientSize.Width, ClientSize.Height - 1);
            }

            if (_values.Count < 2) return;

            double max = Math.Max(1, _values.Max());
            int w = ClientSize.Width;
            int h = ClientSize.Height;
            float dx = (float)w / Math.Max(1, MAX_POINTS - 1);

            var pts = new List<PointF>();
            int i = 0;
            int offset = MAX_POINTS - _values.Count;
            foreach (var v in _values)
            {
                float x = (offset + i) * dx;
                float y = (float)(h - 2 - (v / max) * (h - 6));
                pts.Add(new PointF(x, y));
                i++;
            }

            // soft fill under the line
            using (var fillPath = new GraphicsPath())
            using (var fillBrush = new SolidBrush(Color.FromArgb(50, _lineColor)))
            {
                fillPath.AddLine(pts[0].X, h, pts[0].X, pts[0].Y);
                for (int k = 1; k < pts.Count; k++) fillPath.AddLine(pts[k - 1], pts[k]);
                fillPath.AddLine(pts[pts.Count - 1].X, pts[pts.Count - 1].Y, pts[pts.Count - 1].X, h);
                g.FillPath(fillBrush, fillPath);
            }

            using (var pen = new Pen(_lineColor, 1.5f))
            {
                for (int k = 1; k < pts.Count; k++)
                    g.DrawLine(pen, pts[k - 1], pts[k]);
            }
        }
    }
}
