// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    // One owner carries the image and its inference results together, including through replacement and shutdown.
    internal sealed class VideoFrame : IDisposable
    {
        internal Bitmap Image { get; }
        internal IReadOnlyList<VideoDetection> Detections { get; }
        internal long Sequence { get; }
        internal long Generation { get; }
        internal long ReceivedTicks { get; }
        internal DateTime ReceivedUtc { get; }
        internal string DetectorIdentity { get; }
        internal VideoRegionResult Region { get; }
        internal bool Fresh => AgeSeconds >= 0 && AgeSeconds < 0.5;
        internal double AgeSeconds => (Stopwatch.GetTimestamp() - ReceivedTicks) / (double)Stopwatch.Frequency;

        internal VideoFrame(Bitmap image, IReadOnlyList<VideoDetection> detections, long sequence,
            long generation, long receivedTicks, DateTime receivedUtc, string detectorIdentity,
            VideoRegionResult region = null)
        {
            Image = image ?? throw new ArgumentNullException(nameof(image));
            var copy = (detections ?? Array.Empty<VideoDetection>()).ToArray();
            if (copy.Any(detection => detection == null || detection.Bounds.Right > image.Width ||
                detection.Bounds.Bottom > image.Height))
            { throw new ArgumentException("Detection exceeds its image."); }
            Detections = Array.AsReadOnly(copy);
            Sequence = sequence;
            Generation = generation;
            ReceivedTicks = receivedTicks;
            ReceivedUtc = receivedUtc;
            DetectorIdentity = detectorIdentity ?? "";
            Region = region;
        }

        public void Dispose() => Image.Dispose();
    }

    internal sealed class VideoRegionResult
    {
        internal long Revision { get; }
        internal bool Found { get; }
        internal RectangleF? Bounds { get; }
        internal string Status { get; }
        internal VideoRegionResult(long revision, bool found, RectangleF? bounds, string status)
        { Revision = revision; Found = found; Bounds = bounds; Status = status; }
    }

    internal static class VideoGeometry
    {
        internal static RectangleF ImageRectangle(Size image, Size viewport)
        {
            if (image.Width <= 0 || image.Height <= 0 || viewport.Width <= 0 || viewport.Height <= 0)
            { return RectangleF.Empty; }
            float scale = Math.Min(viewport.Width / (float)image.Width, viewport.Height / (float)image.Height);
            float width = image.Width * scale, height = image.Height * scale;
            return new RectangleF((viewport.Width - width) / 2, (viewport.Height - height) / 2, width, height);
        }

        internal static bool TryImagePoint(Point point, Size image, Size viewport, out PointF pixel)
        {
            var area = ImageRectangle(image, viewport);
            pixel = PointF.Empty;
            if (area.IsEmpty || !area.Contains(point)) { return false; }
            pixel = new PointF((point.X - area.X) * image.Width / area.Width,
                (point.Y - area.Y) * image.Height / area.Height);
            return true;
        }

        internal static RectangleF DefaultRegion(PointF pixel, Size image)
        {
            float size = 64 * Math.Max(1, Math.Max(image.Width, image.Height) / 640f);
            float width = Math.Min(size, image.Width), height = Math.Min(size, image.Height);
            return new RectangleF(Math.Max(0, Math.Min(image.Width - width, pixel.X - width / 2)),
                Math.Max(0, Math.Min(image.Height - height, pixel.Y - height / 2)), width, height);
        }

        internal static bool TryImageRegion(Point start, Point end, Size image, Size viewport, out RectangleF region)
        {
            region = RectangleF.Empty;
            if (!TryImagePoint(start, image, viewport, out var first)) { return false; }
            var area = ImageRectangle(image, viewport);
            float x = Math.Max(0, Math.Min(image.Width, (end.X - area.X) * image.Width / area.Width));
            float y = Math.Max(0, Math.Min(image.Height, (end.Y - area.Y) * image.Height / area.Height));
            region = RectangleF.FromLTRB(Math.Min(first.X, x), Math.Min(first.Y, y),
                Math.Max(first.X, x), Math.Max(first.Y, y));
            return true;
        }
    }
}
