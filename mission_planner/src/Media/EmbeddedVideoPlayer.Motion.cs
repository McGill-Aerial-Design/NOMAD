// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private string MotionEstimateDetail()
        {
            var frame = _displayedVideoFrame;
            bool current = frame?.Fresh == true && frame.Motion?.IsCurrent == true;
            var heading = current ? frame.Motion.Heading : null;
            string detail = heading == null ? " | Aircraft heading unavailable" :
                " | Aircraft heading relative to camera: " + heading.RelativeDegrees.ToString("0") +
                "° (estimated); " + heading.Source;
            if (current && _trackingMode == VisualTrackingMode.Area && frame.Region?.Revision == _regionRevision &&
                SelectedRegion.HasValue && frame.Region.GroundSearchHint.HasValue)
            { detail += " | Estimated area search hint; horizontal plane " + _input.GroundPlaneHeightMeters +
                " m above HOME, camera delay " + _input.CameraDelayEstimateMs + " ms; no steering authority"; }
            _videoToolTips.SetToolTip(_videoBox, detail);
            _videoBox.AccessibleDescription = detail;
            if (_fullscreenBox != null) { _videoToolTips.SetToolTip(_fullscreenBox, detail); }
            return detail;
        }

        private void PaintMotionEstimates(PictureBox box, Graphics graphics)
        {
            var frame = _displayedVideoFrame;
            bool current = frame?.Fresh == true && frame.Motion?.IsCurrent == true;
            PaintHeadingMarker(box, graphics, current ? frame.Motion.Heading : null);
            if (current) { PaintGroundSearchHint(box, graphics, frame); }
        }

        private static void PaintHeadingMarker(PictureBox box, Graphics graphics, VideoHeadingEstimate heading)
        {
            if (heading == null || !VideoDetection.Finite(heading.RelativeDegrees)) { return; }
            float centerX = box.Width / 2f, centerY = box.Height / 2f;
            var offset = heading.RingOffset;
            var inner = new PointF(centerX + offset.X * 25, centerY + offset.Y * 25);
            var outer = new PointF(centerX + offset.X * 38, centerY + offset.Y * 38);
            using (var halo = new Pen(Color.FromArgb(170, Color.Black), 4))
            using (var marker = new Pen(NOMADTheme.INFO, 2))
            {
                graphics.DrawLine(halo, inner, outer);
                graphics.DrawLine(marker, inner, outer);
            }
        }

        private void PaintGroundSearchHint(PictureBox box, Graphics graphics, VideoFrame frame)
        {
            if (_trackingMode != VisualTrackingMode.Area || frame.Region?.Revision != _regionRevision ||
                !frame.Region.GroundSearchHint.HasValue || !SelectedRegion.HasValue) { return; }
            var hint = frame.Region.GroundSearchHint.Value;
            var area = VideoGeometry.ImageRectangle(frame.Image.Size, box.ClientSize);
            float x = area.X + hint.X * area.Width / frame.Image.Width;
            float y = area.Y + hint.Y * area.Height / frame.Image.Height;
            x = Math.Max(area.Left + 8, Math.Min(area.Right - 8, x));
            y = Math.Max(area.Top + 8, Math.Min(area.Bottom - 8, y));
            using (var pen = new Pen(NOMADTheme.WARNING, 2) { DashStyle = DashStyle.Dash })
            using (var brush = new SolidBrush(NOMADTheme.WARNING))
            {
                graphics.DrawPolygon(pen, new[] { new PointF(x, y - 7), new PointF(x + 7, y),
                    new PointF(x, y + 7), new PointF(x - 7, y) });
                graphics.DrawString("Estimated area hint", Font, brush,
                    Math.Min(area.Right - 120, Math.Max(area.Left, x + 10)), Math.Max(area.Top, y - Font.Height));
            }
        }
    }
}
