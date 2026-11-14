// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Media;

namespace NOMAD.MissionPlanner
{
    internal sealed class SparklinePanel : FrameworkElement
    {
        private const int MaxPoints = 80;
        private readonly LinkedList<double> _values = new LinkedList<double>();
        private Brush _line = WpfTheme.Accent;

        public void Push(double value, Brush color)
        {
            _values.AddLast(value);
            while (_values.Count > MaxPoints) _values.RemoveFirst();
            _line = color;
            InvalidateVisual();
        }

        protected override void OnRender(DrawingContext drawing)
        {
            base.OnRender(drawing);
            double width = ActualWidth, height = ActualHeight;
            drawing.DrawRectangle(new SolidColorBrush(Color.FromRgb(28, 28, 30)), null, new Rect(0, 0, width, height));
            drawing.DrawLine(new Pen(Brushes.DimGray, 1), new Point(0, height - 1), new Point(width, height - 1));
            if (_values.Count < 2) return;
            double max = Math.Max(1, _values.Max());
            double dx = width / (MaxPoints - 1);
            int offset = MaxPoints - _values.Count, index = 0;
            var points = _values.Select(value => new Point((offset + index++) * dx,
                height - 2 - value / max * (height - 6))).ToArray();
            var geometry = new StreamGeometry();
            using (var context = geometry.Open())
            {
                context.BeginFigure(new Point(points[0].X, height), true, true);
                context.PolyLineTo(points, true, false);
                context.LineTo(new Point(points[points.Length - 1].X, height), true, false);
            }
            drawing.DrawGeometry(new SolidColorBrush(((SolidColorBrush)_line).Color) { Opacity = 0.2 }, null, geometry);
            var pen = new Pen(_line, 1.5);
            for (int i = 1; i < points.Length; i++) drawing.DrawLine(pen, points[i - 1], points[i]);
        }
    }
}
