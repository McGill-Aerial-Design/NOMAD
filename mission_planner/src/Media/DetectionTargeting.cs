// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;

namespace NOMAD.MissionPlanner
{
    // Pure choices that link detector boxes to the visual tracker. Detection proposes; the tracker still verifies.
    internal static class DetectionTargeting
    {
        internal const float ConfirmOverlap = .3f;
        internal const float ReacquireReach = 1.5f;

        // The smallest box under the point is the most specific target when boxes overlap.
        internal static VideoDetection At(IReadOnlyList<VideoDetection> detections, PointF point, float minimumSize)
        {
            VideoDetection best = null;
            foreach (var detection in detections)
            {
                var bounds = detection.Bounds;
                if (!bounds.Contains(point) || bounds.Width < minimumSize || bounds.Height < minimumSize) { continue; }
                if (best == null || Area(bounds) < Area(best.Bounds)) { best = detection; }
            }
            return best;
        }

        // A detection agreeing with the tracked region; a null label accepts any class.
        internal static VideoDetection Confirming(IReadOnlyList<VideoDetection> detections, RectangleF tracked,
            string label)
        {
            VideoDetection best = null;
            float bestOverlap = ConfirmOverlap;
            foreach (var detection in detections)
            {
                if (label != null && detection.Label != label) { continue; }
                float overlap = Overlap(detection.Bounds, tracked);
                if (overlap >= bestOverlap) { best = detection; bestOverlap = overlap; }
            }
            return best;
        }

        // The same class nearest the last verified region, within a reach scaled by that region's size.
        internal static VideoDetection Reacquire(IReadOnlyList<VideoDetection> detections, string label,
            RectangleF last, float minimumSize)
        {
            float reach = ReacquireReach * Math.Max(last.Width, last.Height);
            VideoDetection best = null;
            float bestDistance = reach;
            foreach (var detection in detections)
            {
                if (detection.Label != label || detection.Bounds.Width < minimumSize ||
                    detection.Bounds.Height < minimumSize) { continue; }
                float distance = Distance(Center(detection.Bounds), Center(last));
                if (distance <= bestDistance) { best = detection; bestDistance = distance; }
            }
            return best;
        }

        // Auto follow prefers confident detections near the image center.
        internal static VideoDetection Auto(IReadOnlyList<VideoDetection> detections, Size image, float minimumSize)
        {
            var center = new PointF(image.Width / 2f, image.Height / 2f);
            float halfDiagonal = Distance(PointF.Empty, center);
            VideoDetection best = null;
            float bestScore = 0;
            foreach (var detection in detections)
            {
                if (detection.Bounds.Width < minimumSize || detection.Bounds.Height < minimumSize) { continue; }
                float centered = 1 - .5f * Distance(Center(detection.Bounds), center) / halfDiagonal;
                float score = detection.Confidence * centered;
                if (score > bestScore) { best = detection; bestScore = score; }
            }
            return best;
        }

        // Selections smaller than this are too few pixels for the trackers to verify.
        internal static float MinimumSize(Size image) => 8 * Math.Max(1, Math.Max(image.Width, image.Height) / 640f);

        internal static float Overlap(RectangleF left, RectangleF right)
        {
            var shared = RectangleF.Intersect(left, right);
            if (shared.IsEmpty) { return 0; }
            float union = Area(left) + Area(right) - Area(shared);
            return union <= 0 ? 0 : Area(shared) / union;
        }

        private static float Area(RectangleF bounds) => bounds.Width * bounds.Height;
        private static PointF Center(RectangleF bounds) =>
            new PointF(bounds.X + bounds.Width / 2, bounds.Y + bounds.Height / 2);
        private static float Distance(PointF left, PointF right) =>
            (float)Math.Sqrt((left.X - right.X) * (left.X - right.X) + (left.Y - right.Y) * (left.Y - right.Y));
    }
}
