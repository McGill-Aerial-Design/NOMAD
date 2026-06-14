// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public sealed class TimeSeriesPoint
    {
        public double Time { get; set; }
        public double Value { get; set; }
    }

    public sealed class TimeSeriesData
    {
        public string Key { get; set; } = "";
        public string Label { get; set; } = "";
        public string Unit { get; set; } = "";
        public Color Color { get; set; } = Color.White;
        public List<TimeSeriesPoint> Points { get; } = new List<TimeSeriesPoint>();
    }

    public sealed class TimeSeriesPlot : Panel
    {
        private static readonly Color[] Palette =
        {
            NOMADTheme.ACCENT,
            Color.FromArgb(255, 82, 92),
            Color.FromArgb(178, 36, 44),
            Color.FromArgb(255, 150, 156),
            NOMADTheme.TEXT_PRIMARY,
            Color.FromArgb(190, 190, 194),
            Color.FromArgb(125, 125, 130),
            Color.FromArgb(118, 18, 26),
        };

        private readonly Dictionary<string, TimeSeriesData> _series =
            new Dictionary<string, TimeSeriesData>(StringComparer.OrdinalIgnoreCase);
        private readonly List<(double Value, Color Color, string Label)> _thresholds =
            new List<(double, Color, string)>();
        private IReadOnlyList<ModeSpan> _modes = Array.Empty<ModeSpan>();
        private double _viewStart;
        private double _viewEnd = 1;
        private bool _dragging;
        private Point _dragStart;
        private double _dragViewStart;
        private double _dragViewEnd;
        private Point? _hoverPoint;

        public TimeSeriesPlot()
        {
            DoubleBuffered = true;
            BackColor = NOMADTheme.BG_DARK;
            ForeColor = NOMADTheme.TEXT_PRIMARY;
            SetStyle(ControlStyles.ResizeRedraw, true);
        }

        public bool Normalized { get; set; }
        public int MaxPointsPerSeries { get; set; } = 600;
        public IReadOnlyCollection<TimeSeriesData> Series => _series.Values.ToList();

        public Color NextColor => Palette[_series.Count % Palette.Length];

        public void SetModes(IReadOnlyList<ModeSpan> modes)
        {
            _modes = modes ?? Array.Empty<ModeSpan>();
            ResetView();
        }

        public void SetSeries(TimeSeriesData data)
        {
            if (data == null || string.IsNullOrWhiteSpace(data.Key)) return;
            if (data.Color == Color.Empty) data.Color = NextColor;
            _series[data.Key] = data;
            ResetView();
        }

        public void RemoveSeries(string key)
        {
            if (key == null) return;
            _series.Remove(key);
            Invalidate();
        }

        public void ClearSeries()
        {
            _series.Clear();
            _thresholds.Clear();
            ResetView();
        }

        public void AppendPoint(string key, string label, string unit, double time, double value, Color? color = null)
        {
            if (!_series.TryGetValue(key, out TimeSeriesData data))
            {
                data = new TimeSeriesData
                {
                    Key = key,
                    Label = label,
                    Unit = unit,
                    Color = color ?? NextColor,
                };
                _series[key] = data;
            }

            data.Points.Add(new TimeSeriesPoint { Time = time, Value = value });
            while (data.Points.Count > Math.Max(2, MaxPointsPerSeries))
                data.Points.RemoveAt(0);
            _viewStart = 0;
            _viewEnd = 1;
            Invalidate();
        }

        public void SetThresholds(params (double Value, Color Color, string Label)[] thresholds)
        {
            _thresholds.Clear();
            if (thresholds != null) _thresholds.AddRange(thresholds);
            Invalidate();
        }

        public void ResetView()
        {
            _viewStart = 0;
            _viewEnd = 1;
            Invalidate();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.Clear(BackColor);

            Rectangle plot = PlotBounds();
            DrawGrid(g, plot);
            var visible = VisibleSeries();
            if (visible.Count == 0 && _modes.Count == 0)
            {
                DrawCentered(g, "Select a field to plot", NOMADTheme.TEXT_MUTED);
                return;
            }

            (double minTime, double maxTime) = visible.Count > 0
                ? TimeBounds(visible)
                : ModeTimeBounds();
            if (maxTime <= minTime) maxTime = minTime + 1;
            double fullRange = maxTime - minTime;
            double viewMin = minTime + fullRange * _viewStart;
            double viewMax = minTime + fullRange * _viewEnd;
            DrawModeTrack(g, ModeTrackBounds(plot), viewMin, viewMax);
            if (visible.Count == 0)
            {
                DrawCentered(g, "Select a field to plot", NOMADTheme.TEXT_MUTED);
                DrawHover(g, plot, visible, viewMin, viewMax);
                return;
            }

            var valueBounds = ValueBounds(visible, viewMin, viewMax);
            if (valueBounds.Max <= valueBounds.Min)
            {
                valueBounds.Min -= 1;
                valueBounds.Max += 1;
            }

            DrawAxes(g, plot, viewMin, viewMax, valueBounds.Min, valueBounds.Max);
            DrawThresholds(g, plot, valueBounds.Min, valueBounds.Max);
            using (var clip = new Region(plot))
            {
                Region previous = g.Clip;
                g.Clip = clip;
                foreach (TimeSeriesData data in visible)
                    DrawSeries(g, plot, data, viewMin, viewMax, valueBounds.Min, valueBounds.Max);
                g.Clip = previous;
            }
            DrawLegend(g, visible);
            DrawHover(g, plot, visible, viewMin, viewMax);
        }

        protected override void OnMouseWheel(MouseEventArgs e)
        {
            base.OnMouseWheel(e);
            Rectangle plot = PlotBounds();
            double span = _viewEnd - _viewStart;
            double factor = e.Delta > 0 ? 0.75 : 1.33;
            double newSpan = Math.Max(0.02, Math.Min(1, span * factor));
            double ratio = plot.Width <= 0
                ? 0.5
                : Math.Max(0, Math.Min(1, (e.X - plot.Left) / (double)plot.Width));
            double anchor = _viewStart + span * ratio;
            _viewStart = anchor - newSpan * ratio;
            _viewEnd = _viewStart + newSpan;
            ClampView();
            Invalidate();
        }

        protected override void OnMouseDown(MouseEventArgs e)
        {
            base.OnMouseDown(e);
            if (e.Button != MouseButtons.Left) return;
            _dragging = true;
            _dragStart = e.Location;
            _dragViewStart = _viewStart;
            _dragViewEnd = _viewEnd;
            Cursor = Cursors.SizeWE;
        }

        protected override void OnMouseMove(MouseEventArgs e)
        {
            base.OnMouseMove(e);
            _hoverPoint = e.Location;
            if (_dragging && ClientSize.Width > 0)
            {
                double span = _dragViewEnd - _dragViewStart;
                double shift = -(e.X - _dragStart.X) / (double)ClientSize.Width * span;
                _viewStart = _dragViewStart + shift;
                _viewEnd = _dragViewEnd + shift;
                ClampView();
            }
            Invalidate();
        }

        protected override void OnMouseUp(MouseEventArgs e)
        {
            base.OnMouseUp(e);
            _dragging = false;
            Cursor = Cursors.Default;
        }

        protected override void OnMouseLeave(EventArgs e)
        {
            base.OnMouseLeave(e);
            _hoverPoint = null;
            _dragging = false;
            Cursor = Cursors.Default;
            Invalidate();
        }

        protected override void OnMouseDoubleClick(MouseEventArgs e)
        {
            base.OnMouseDoubleClick(e);
            ResetView();
        }

        private List<TimeSeriesData> VisibleSeries()
            => _series.Values.Where(series => series.Points.Count > 0).ToList();

        private static (double Min, double Max) TimeBounds(IEnumerable<TimeSeriesData> series)
        {
            double min = series.Min(item => item.Points.Min(point => point.Time));
            double max = series.Max(item => item.Points.Max(point => point.Time));
            return (min, max);
        }

        private (double Min, double Max) ModeTimeBounds()
        {
            double min = _modes.Count == 0 ? 0 : _modes.Min(mode => mode.StartSeconds);
            double max = _modes.Count == 0 ? 1 : _modes.Max(mode => mode.EndSeconds);
            return (min, max);
        }

        private Rectangle PlotBounds()
        {
            int left = ClientSize.Width < 420 ? 42 : 55;
            int top = _modes.Count > 0 ? 54 : 30;
            return new Rectangle(
                left,
                top,
                Math.Max(1, ClientSize.Width - left - 15),
                Math.Max(1, ClientSize.Height - top - 35));
        }

        private static Rectangle ModeTrackBounds(Rectangle plot)
            => new Rectangle(plot.Left, plot.Top - 23, plot.Width, 19);

        private void DrawModeTrack(Graphics g, Rectangle track, double minTime, double maxTime)
        {
            if (_modes.Count == 0 || maxTime <= minTime)
                return;

            using (var background = new SolidBrush(NOMADTheme.INPUT_BG))
                g.FillRectangle(background, track);
            using (var clip = new Region(track))
            using (var font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Bold))
            {
                Region previous = g.Clip;
                g.Clip = clip;
                foreach (ModeSpan mode in _modes)
                {
                    double start = Math.Max(minTime, mode.StartSeconds);
                    double end = Math.Min(maxTime, mode.EndSeconds);
                    if (end <= start)
                        continue;

                    int left = track.Left + (int)((start - minTime) / (maxTime - minTime) * track.Width);
                    int right = track.Left + (int)((end - minTime) / (maxTime - minTime) * track.Width);
                    var segment = new Rectangle(left, track.Top, Math.Max(2, right - left), track.Height);
                    Color color = FlightModeVisuals.ColorFor(mode.Mode);
                    using (var brush = new SolidBrush(color))
                        g.FillRectangle(brush, segment);

                    SizeF labelSize = g.MeasureString(mode.Mode, font);
                    if (segment.Width >= labelSize.Width + 8)
                    {
                        using (var brush = new SolidBrush(ReadableTextColor(color)))
                        {
                            g.DrawString(
                                mode.Mode,
                                font,
                                brush,
                                segment.Left + 4,
                                segment.Top + (segment.Height - labelSize.Height) / 2);
                        }
                    }
                }
                g.Clip = previous;
            }
            using (var border = new Pen(NOMADTheme.CARD_BORDER))
                g.DrawRectangle(border, track.Left, track.Top, track.Width - 1, track.Height - 1);
        }

        private (double Min, double Max) ValueBounds(
            IEnumerable<TimeSeriesData> series,
            double minTime,
            double maxTime)
        {
            if (Normalized) return (0, 1);
            var values = series.SelectMany(item => item.Points)
                .Where(point => point.Time >= minTime && point.Time <= maxTime)
                .Select(point => point.Value)
                .Where(value => !double.IsNaN(value) && !double.IsInfinity(value))
                .ToList();
            if (values.Count == 0) return (0, 1);
            double min = values.Min();
            double max = values.Max();
            double padding = Math.Max(0.001, (max - min) * 0.08);
            return (min - padding, max + padding);
        }

        private void DrawSeries(
            Graphics g,
            Rectangle plot,
            TimeSeriesData data,
            double minTime,
            double maxTime,
            double minValue,
            double maxValue)
        {
            var points = data.Points
                .Where(point => point.Time >= minTime && point.Time <= maxTime)
                .ToList();
            if (points.Count < 2) return;

            double localMin = minValue;
            double localMax = maxValue;
            if (Normalized)
            {
                localMin = points.Min(point => point.Value);
                localMax = points.Max(point => point.Value);
                if (localMax <= localMin) localMax = localMin + 1;
            }

            var screen = points.Select(point => new PointF(
                plot.Left + (float)((point.Time - minTime) / (maxTime - minTime) * plot.Width),
                plot.Bottom - (float)((point.Value - localMin) / (localMax - localMin) * plot.Height)))
                .ToArray();
            using (var pen = new Pen(data.Color, 1.7f))
                g.DrawLines(pen, screen);
        }

        private static void DrawGrid(Graphics g, Rectangle plot)
        {
            using (var brush = new SolidBrush(NOMADTheme.CARD_BG))
                g.FillRectangle(brush, plot);
            using (var pen = new Pen(NOMADTheme.CARD_BORDER))
            {
                for (int i = 0; i <= 5; i++)
                {
                    float x = plot.Left + plot.Width * i / 5f;
                    float y = plot.Top + plot.Height * i / 5f;
                    g.DrawLine(pen, x, plot.Top, x, plot.Bottom);
                    g.DrawLine(pen, plot.Left, y, plot.Right, y);
                }
            }
        }

        private void DrawAxes(
            Graphics g,
            Rectangle plot,
            double minTime,
            double maxTime,
            double minValue,
            double maxValue)
        {
            using (var brush = new SolidBrush(NOMADTheme.TEXT_MUTED))
            using (var font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL))
            {
                for (int i = 0; i <= 5; i++)
                {
                    double time = minTime + (maxTime - minTime) * i / 5;
                    string timeText = FormatTime(time);
                    SizeF timeSize = g.MeasureString(timeText, font);
                    float x = plot.Left + plot.Width * i / 5f - timeSize.Width / 2;
                    g.DrawString(timeText, font, brush, x, plot.Bottom + 5);

                    double value = maxValue - (maxValue - minValue) * i / 5;
                    string valueText = Normalized ? $"{value:P0}" : value.ToString("0.##");
                    SizeF valueSize = g.MeasureString(valueText, font);
                    g.DrawString(valueText, font, brush, plot.Left - valueSize.Width - 5,
                        plot.Top + plot.Height * i / 5f - valueSize.Height / 2);
                }
            }
        }

        private void DrawThresholds(Graphics g, Rectangle plot, double minValue, double maxValue)
        {
            if (Normalized) return;
            using (var font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL))
            {
                foreach (var threshold in _thresholds)
                {
                    if (threshold.Value < minValue || threshold.Value > maxValue) continue;
                    float y = plot.Bottom - (float)((threshold.Value - minValue) / (maxValue - minValue) * plot.Height);
                    using (var pen = new Pen(Color.FromArgb(180, threshold.Color), 1) { DashStyle = DashStyle.Dash })
                    using (var brush = new SolidBrush(threshold.Color))
                    {
                        g.DrawLine(pen, plot.Left, y, plot.Right, y);
                        g.DrawString(threshold.Label, font, brush, plot.Right - 90, y - 14);
                    }
                }
            }
        }

        private static void DrawLegend(Graphics g, IReadOnlyList<TimeSeriesData> series)
        {
            int x = 58;
            using (var font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL))
            {
                foreach (TimeSeriesData item in series)
                {
                    string text = string.IsNullOrWhiteSpace(item.Unit) ? item.Label : $"{item.Label} ({item.Unit})";
                    using (var brush = new SolidBrush(item.Color))
                        g.FillRectangle(brush, x, 10, 10, 10);
                    using (var brush = new SolidBrush(NOMADTheme.TEXT_SECONDARY))
                        g.DrawString(text, font, brush, x + 14, 6);
                    x += 22 + (int)g.MeasureString(text, font).Width;
                    if (x > g.VisibleClipBounds.Width - 150) break;
                }
            }
        }

        private void DrawHover(
            Graphics g,
            Rectangle plot,
            IReadOnlyList<TimeSeriesData> series,
            double minTime,
            double maxTime)
        {
            Rectangle interaction = _modes.Count == 0
                ? plot
                : Rectangle.FromLTRB(plot.Left, ModeTrackBounds(plot).Top, plot.Right, plot.Bottom);
            if (!_hoverPoint.HasValue || !interaction.Contains(_hoverPoint.Value)) return;
            float x = _hoverPoint.Value.X;
            double time = minTime + (x - plot.Left) / plot.Width * (maxTime - minTime);
            using (var pen = new Pen(Color.FromArgb(180, NOMADTheme.ACCENT)))
                g.DrawLine(pen, x, interaction.Top, x, plot.Bottom);

            var labels = new List<string> { FormatTime(time) };
            ModeSpan mode = _modes.FirstOrDefault(item =>
                time >= item.StartSeconds && time < item.EndSeconds);
            if (mode != null)
                labels.Add("Mode: " + mode.Mode);
            foreach (TimeSeriesData item in series)
            {
                TimeSeriesPoint nearest = item.Points.OrderBy(point => Math.Abs(point.Time - time)).FirstOrDefault();
                if (nearest != null)
                    labels.Add($"{item.Label}: {nearest.Value:0.###} {item.Unit}".TrimEnd());
            }
            string text = string.Join(Environment.NewLine, labels);
            using (var font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL))
            {
                SizeF size = g.MeasureString(text, font);
                float left = Math.Min(plot.Right - size.Width - 8, x + 8);
                if (left < plot.Left) left = plot.Left;
                var box = new RectangleF(left, plot.Top + 8, size.Width + 8, size.Height + 6);
                using (var brush = new SolidBrush(Color.FromArgb(238, NOMADTheme.PANEL_ALT)))
                    g.FillRectangle(brush, box);
                using (var pen = new Pen(NOMADTheme.ACCENT))
                    g.DrawRectangle(pen, box.X, box.Y, box.Width, box.Height);
                using (var brush = new SolidBrush(NOMADTheme.TEXT_PRIMARY))
                    g.DrawString(text, font, brush, box.X + 4, box.Y + 3);
            }
        }

        private void DrawCentered(Graphics g, string text, Color color)
        {
            using (var font = NOMADTheme.Font(NOMADTheme.SIZE_HEADING))
            using (var brush = new SolidBrush(color))
            {
                SizeF size = g.MeasureString(text, font);
                g.DrawString(text, font, brush,
                    (ClientSize.Width - size.Width) / 2,
                    (ClientSize.Height - size.Height) / 2);
            }
        }

        private void ClampView()
        {
            double span = Math.Max(0.02, Math.Min(1, _viewEnd - _viewStart));
            if (_viewStart < 0)
            {
                _viewStart = 0;
                _viewEnd = span;
            }
            if (_viewEnd > 1)
            {
                _viewEnd = 1;
                _viewStart = 1 - span;
            }
        }

        private static string FormatTime(double seconds)
            => TimeSpan.FromSeconds(Math.Max(0, seconds)).ToString(seconds >= 3600 ? @"h\:mm\:ss" : @"m\:ss");

        private static Color ReadableTextColor(Color background)
        {
            double luminance = 0.299 * background.R + 0.587 * background.G + 0.114 * background.B;
            return luminance > 155 ? Color.Black : Color.White;
        }
    }
}
