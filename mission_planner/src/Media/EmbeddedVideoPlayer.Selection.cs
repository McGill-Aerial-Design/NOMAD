// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        private PictureBox _dragBox;
        private Point _dragStart, _dragEnd;
        private Size _dragViewport, _dragImageSize;
        private long _dragGeneration;

        private void AttachSelectionMouse(PictureBox box)
        {
            box.MouseDown += OnSelectionDown;
            box.MouseMove += OnSelectionMove;
            box.MouseUp += OnSelectionUp;
        }

        private void OnSelectionDown(object sender, MouseEventArgs args)
        {
            var frame = _displayedVideoFrame;
            var box = (PictureBox)sender;
            if (!_showControls || args.Button != MouseButtons.Left || frame?.Fresh != true ||
                !VideoGeometry.TryImagePoint(args.Location, frame.Image.Size, box.ClientSize, out _)) { return; }
            RetireTracking("Selecting a new region");
            _dragBox = box; _dragStart = args.Location; _dragEnd = args.Location;
            _dragViewport = box.ClientSize; _dragImageSize = frame.Image.Size; _dragGeneration = frame.Generation;
            box.Capture = true;
        }

        private void OnSelectionMove(object sender, MouseEventArgs args)
        {
            if (!ReferenceEquals(sender, _dragBox)) { return; }
            _dragEnd = args.Location;
            _dragBox.Invalidate();
        }

        private void OnSelectionUp(object sender, MouseEventArgs args)
        {
            if (args.Button != MouseButtons.Left || !ReferenceEquals(sender, _dragBox)) { return; }
            var box = _dragBox; _dragBox = null;
            box.Capture = false;
            var frame = _displayedVideoFrame;
            if (frame?.Fresh != true || frame.Generation != _dragGeneration ||
                frame.Image.Size != _dragImageSize || box.ClientSize != _dragViewport) { return; }
            if (Math.Abs(args.X - _dragStart.X) < 6 && Math.Abs(args.Y - _dragStart.Y) < 6)
            { SelectTargetAt(args.Location, box.ClientSize); }
            else if (VideoGeometry.TryImageRegion(_dragStart, args.Location, frame.Image.Size, box.ClientSize, out var region))
            { SelectRegion(region); }
            box.Invalidate();
        }

        private void PaintSelectedRegion(PictureBox box, Graphics graphics, RectangleF area, float scale)
        {
            RectangleF? region = SelectedRegion;
            bool dragging = ReferenceEquals(box, _dragBox);
            if (dragging && VideoGeometry.TryImageRegion(_dragStart, _dragEnd, _dragImageSize,
                _dragViewport, out var drag)) { region = drag; }
            if (!region.HasValue) { return; }
            var bounds = region.Value;
            using (var pen = new Pen(Color.Yellow, 2))
            {
                if (dragging || !RegionReady) { pen.DashStyle = DashStyle.Dash; }
                graphics.DrawRectangle(pen, area.X + bounds.X * scale, area.Y + bounds.Y * scale,
                    bounds.Width * scale, bounds.Height * scale);
            }
        }

        protected override void OnVisibleChanged(EventArgs args)
        {
            base.OnVisibleChanged(args);
            if (!_showControls || Visible) { return; }
            lock (_visionGate) { _trackingVisible = false; }
            RetireTracking("Video hidden; select again");
        }
    }
}
