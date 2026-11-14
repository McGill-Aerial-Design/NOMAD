// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Globalization;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;

namespace NOMAD.MissionPlanner
{
    internal class JoystickPad : FrameworkElement
    {
        public event Action<float, float> StickChanged;
        internal string HorizontalLabel { get; set; } = "YAW";
        internal string VerticalLabel { get; set; } = "PITCH";
        internal bool HorizontalInverted { get; set; }
        internal bool VerticalInverted { get; set; }
        private bool _dragging;
        private Point _stickNorm;
        private readonly float _drawDeadzone;
        private readonly Func<MouseEventArgs, Point> _position;
        private readonly Func<bool> _capture;
        private readonly Action _release;
        public JoystickPad(float drawDeadzone) : this(drawDeadzone, null, null, null) { }
        internal JoystickPad(float drawDeadzone, Func<MouseEventArgs, Point> position, Func<bool> capture, Action release)
        {
            _drawDeadzone = drawDeadzone;
            _position = position ?? (args => args.GetPosition(this));
            _capture = capture ?? CaptureMouse;
            _release = release ?? ReleaseMouseCapture;
            Focusable = true;
            IsEnabledChanged += (sender, args) => { if (!IsEnabled) { ResetStick(); _release(); } };
            Unloaded += (sender, args) => { ResetStick(); _release(); };
        }
        private double Radius() => Math.Min(Math.Min(ActualWidth / 2 - 38, ActualHeight / 2 - 18), 120);
        protected override void OnMouseLeftButtonDown(MouseButtonEventArgs e)
        {
            base.OnMouseLeftButtonDown(e);
            if (!_capture()) return;
            _dragging = true;
            UpdateFromMouse(_position(e));
        }
        protected override void OnMouseMove(MouseEventArgs e)
        {
            base.OnMouseMove(e);
            if (_dragging) UpdateFromMouse(_position(e));
        }
        protected override void OnMouseLeftButtonUp(MouseButtonEventArgs e)
        {
            base.OnMouseLeftButtonUp(e);
            ResetStick();
            _release();
        }
        protected override void OnMouseLeave(MouseEventArgs e)
        {
            base.OnMouseLeave(e);
            if (_dragging) { ResetStick(); _release(); }
        }
        protected override void OnLostMouseCapture(MouseEventArgs e) { base.OnLostMouseCapture(e); ResetStick(); }
        private void ResetStick()
        {
            if (!_dragging) return;
            _dragging = false; _stickNorm = new Point();
            StickChanged?.Invoke(0, 0); InvalidateVisual();
        }
        private void UpdateFromMouse(Point point)
        {
            double r = Radius();
            if (r <= 0) return;
            double x = (point.X - ActualWidth / 2) / r, y = (point.Y - ActualHeight / 2) / r;
            double magnitude = Math.Sqrt(x * x + y * y);
            if (magnitude > 1) { x /= magnitude; y /= magnitude; }
            _stickNorm = new Point(x, y);
            StickChanged?.Invoke((float)x, (float)-y); InvalidateVisual();
        }
        protected override void OnRender(DrawingContext dc)
        {
            base.OnRender(dc);
            dc.DrawRectangle(WpfTheme.Card, null, new Rect(RenderSize));
            double r = Radius(); if (r <= 0) return;
            var center = new Point(ActualWidth / 2, ActualHeight / 2);
            var ring = new Pen(WpfTheme.SecondaryText, 2);
            var cross = new Pen(WpfTheme.Outline, 1);
            dc.DrawEllipse(null, ring, center, r, r);
            dc.DrawLine(cross, new Point(center.X - r, center.Y), new Point(center.X + r, center.Y));
            dc.DrawLine(cross, new Point(center.X, center.Y - r), new Point(center.X, center.Y + r));
            dc.DrawEllipse(null, cross, center, r * _drawDeadzone, r * _drawDeadzone);
            dc.DrawEllipse(WpfTheme.Accent, new Pen(Brushes.White, 2),
                new Point(center.X + _stickNorm.X * r, center.Y + _stickNorm.Y * r), 16, 16);
            DrawLabel(dc, AxisLabel(VerticalLabel, !VerticalInverted), center.X, center.Y - r, 0, -1);
            DrawLabel(dc, AxisLabel(VerticalLabel, VerticalInverted), center.X, center.Y + r, 0, 1);
            DrawLabel(dc, AxisLabel(HorizontalLabel, HorizontalInverted), center.X - r, center.Y, -1, 0);
            DrawLabel(dc, AxisLabel(HorizontalLabel, !HorizontalInverted), center.X + r, center.Y, 1, 0);
        }
        private void DrawLabel(DrawingContext dc, string label, double x, double y, int horizontal, int vertical)
        {
            var text = new FormattedText(label, CultureInfo.CurrentUICulture, FlowDirection.LeftToRight,
                new Typeface("Segoe UI"), 10, WpfTheme.SecondaryText, VisualTreeHelper.GetDpi(this).PixelsPerDip);
            dc.DrawText(text, new Point(x - (horizontal < 0 ? text.Width + 1 : horizontal == 0 ? text.Width / 2 : -1),
                y - (vertical < 0 ? text.Height : vertical == 0 ? text.Height / 2 : -1)));
        }
        private static string AxisLabel(string name, bool positive) =>
            name == "None" ? "OFF" : name.ToUpperInvariant() + (positive ? "+" : "−");
    }
}
