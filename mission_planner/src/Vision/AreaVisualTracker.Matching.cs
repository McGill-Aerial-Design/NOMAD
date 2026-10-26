// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Threading;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    internal sealed partial class AreaVisualTracker
    {
        private sealed class Placement
        {
            internal PointF Point;
            internal RectangleF Bounds;
            internal Point2f[] Context;
        }

        private static void BoundFeatures(Mat descriptors, ref KeyPoint[] points)
        {
            // Equal-response ORB corners can exceed its requested feature count.
            if (points.Length <= 1000) { return; }
            Array.Resize(ref points, 1000); descriptors.Resize(1000);
        }

        private void RemoveRepeatedDescriptors()
        {
            if (_descriptors.Rows < 2) { return; }
            using (var matcher = new BFMatcher(NormTypes.Hamming))
            using (var distinct = new Mat())
            {
                var neighbors = matcher.KnnMatch(_descriptors, _descriptors, 2);
                int localFeatures = _referencePoints.Count(point => _selection.Contains(point.Pt.X, point.Pt.Y));
                var points = new List<KeyPoint>();
                for (int index = 0; index < neighbors.Length; index++)
                {
                    if (neighbors[index].Length < 2 || neighbors[index][1].Distance < 24) { continue; }
                    using (var row = _descriptors.Row(index)) { distinct.PushBack(row); }
                    points.Add(_referencePoints[index]);
                }
                _descriptors.Dispose(); _descriptors = distinct.Clone(); _referencePoints = points.ToArray();
                if (points.Count(point => _selection.Contains(point.Pt.X, point.Pt.Y)) < localFeatures * .5)
                { throw new ArgumentException("Area texture repeats; select a distinctive marked surface."); }
            }
        }

        private Placement FindNearPrior(Mat current, KeyPoint[] points, Mat descriptors, PointF hint,
            CancellationToken token)
        {
            if (!VideoDetection.Finite(hint.X) || !VideoDetection.Finite(hint.Y)) { return null; }
            float x = hint.X * current.Width / _imageSize.Width, y = hint.Y * current.Height / _imageSize.Height;
            float radius = VisualFrameContext.PriorSearchRadius(new System.Drawing.Size(current.Width, current.Height),
                _selection);
            using (var nearby = new Mat())
            {
                var nearbyPoints = new List<KeyPoint>();
                for (int index = 0; index < points.Length; index++)
                {
                    if (Math.Abs(points[index].Pt.X - x) > radius || Math.Abs(points[index].Pt.Y - y) > radius) { continue; }
                    using (var row = descriptors.Row(index)) { nearby.PushBack(row); }
                    nearbyPoints.Add(points[index]);
                }
                return FindPlacement(current, nearbyPoints.ToArray(), nearby, token);
            }
        }

        private Placement FindPlacement(Mat current, KeyPoint[] points, Mat descriptors, CancellationToken token)
        {
            if (descriptors.Rows < MinimumInliers) { return null; }
            var matches = MutualMatches(descriptors);
            if (matches.Length < MinimumInliers) { return null; }
            var source = matches.Select(match => _referencePoints[match.QueryIdx].Pt).ToArray();
            var destination = matches.Select(match => points[match.TrainIdx].Pt).ToArray();
            LastHomographyFits++;
            using (var inliers = new Mat())
            using (var homography = Cv2.FindHomography(source.Select(point => new Point2d(point.X, point.Y)),
                destination.Select(point => new Point2d(point.X, point.Y)), HomographyMethods.Ransac, 2.5, inliers, 750, .995))
            {
                token.ThrowIfCancellationRequested();
                if (homography.Empty() || inliers.Empty()) { return null; }
                var accepted = Enumerable.Range(0, matches.Length).Where(index => inliers.At<byte>(index) != 0).ToArray();
                if (accepted.Length < MinimumInliers || accepted.Length < matches.Length * .55 ||
                    !DistributedSupport(accepted.Select(index => _referencePoints[matches[index].QueryIdx]).ToArray()))
                { return null; }
                var projected = Cv2.PerspectiveTransform(source, homography);
                double meanError = accepted.Average(index => Distance(projected[index], destination[index]));
                if (meanError > 1.5) { return null; }
                var corners = Cv2.PerspectiveTransform(Corners(_selection), homography);
                var context = Cv2.PerspectiveTransform(Corners(_context), homography);
                var aim = Cv2.PerspectiveTransform(new[] { new Point2f(_canonicalPoint.X, _canonicalPoint.Y) }, homography)[0];
                if (!ValidGeometry(corners, context, aim, current.Size()) || !TargetVisible(current, homography))
                { return null; }
                return new Placement { Point = new PointF(aim.X, aim.Y), Bounds = Bounds(corners), Context = context };
            }
        }

        private DMatch[] MutualMatches(Mat current)
        {
            using (var matcher = new BFMatcher(NormTypes.Hamming))
            {
                var forward = matcher.KnnMatch(_descriptors, current, 2);
                var reverse = matcher.KnnMatch(current, _descriptors, 2);
                return forward.Where(pair => pair.Length == 2 && pair[0].Distance < 64 &&
                    pair[0].Distance < pair[1].Distance * .72 && reverse[pair[0].TrainIdx].Length == 2 &&
                    reverse[pair[0].TrainIdx][0].TrainIdx == pair[0].QueryIdx &&
                    reverse[pair[0].TrainIdx][0].Distance < reverse[pair[0].TrainIdx][1].Distance * .72)
                    .Select(pair => pair[0]).ToArray();
            }
        }

        private bool HasAnotherPlacement(Mat current, KeyPoint[] points, Mat descriptors, Placement first,
            CancellationToken token)
        {
            using (var remaining = new Mat())
            {
                var remainingPoints = new List<KeyPoint>();
                for (int index = 0; index < points.Length; index++)
                {
                    if (Cv2.PointPolygonTest(first.Context, points[index].Pt, false) >= 0) { continue; }
                    using (var row = descriptors.Row(index)) { remaining.PushBack(row); }
                    remainingPoints.Add(points[index]);
                }
                token.ThrowIfCancellationRequested();
                // Rematch the same frozen reference outside the first placement, including ratio-test rejections.
                var second = FindPlacement(current, remainingPoints.ToArray(), remaining, token);
                return second != null && Math.Sqrt(Math.Pow(first.Point.X - second.Point.X, 2) +
                    Math.Pow(first.Point.Y - second.Point.Y, 2)) > Math.Max(12,
                    Math.Min(first.Bounds.Width, first.Bounds.Height) * .5);
            }
        }

        private bool DistributedSupport(KeyPoint[] points)
        {
            int local = 0, context = 0, quadrants = 0;
            float left = float.MaxValue, top = float.MaxValue, right = float.MinValue, bottom = float.MinValue;
            foreach (var keypoint in points)
            {
                var point = keypoint.Pt;
                if (_selection.Contains(point.X, point.Y))
                {
                    local++;
                    int quadrant = (point.X >= _canonicalPoint.X ? 1 : 0) + (point.Y >= _canonicalPoint.Y ? 2 : 0);
                    quadrants |= 1 << quadrant;
                    left = Math.Min(left, point.X); right = Math.Max(right, point.X);
                    top = Math.Min(top, point.Y); bottom = Math.Max(bottom, point.Y);
                }
                else { context++; }
            }
            bool surroundsPoint = quadrants == 7 || quadrants == 11 || quadrants == 13 || quadrants == 14 || quadrants == 15;
            return local >= 8 && context >= 6 && surroundsPoint &&
                right - left >= _selection.Width * .45 && bottom - top >= _selection.Height * .45;
        }

        private bool TargetVisible(Mat current, Mat homography)
        {
            using (var aligned = new Mat())
            using (var scores = new Mat())
            {
                Cv2.WarpPerspective(current, aligned, homography, _reference.Size(),
                    InterpolationFlags.Linear | InterpolationFlags.WarpInverseMap);
                using (var reference = new Mat(_reference, PixelBounds(_selection)))
                using (var candidate = new Mat(aligned, PixelBounds(_selection)))
                {
                    Cv2.MatchTemplate(candidate, reference, scores, TemplateMatchModes.CCoeffNormed);
                    return scores.At<float>(0) >= .5;
                }
            }
        }

        private bool ValidGeometry(Point2f[] corners, Point2f[] context, Point2f aim, OpenCvSharp.Size imageSize)
        {
            if (!Finite(aim) || corners.Any(point => !Finite(point)) || context.Any(point => !Finite(point))) { return false; }
            if (!Cv2.IsContourConvex(corners) || !Cv2.IsContourConvex(context)) { return false; }
            double area = Cv2.ContourArea(corners, true), original = _selection.Width * _selection.Height;
            if (area < original * .16 || area > original * 6.25 || area > imageSize.Width * imageSize.Height * .7)
            { return false; }
            for (int index = 0; index < 4; index++)
            {
                double edge = Distance(corners[index], corners[(index + 1) % 4]);
                if (edge < 8 || edge > Math.Max(imageSize.Width, imageSize.Height) * 2) { return false; }
            }
            return Cv2.PointPolygonTest(corners, aim, false) >= 0;
        }

        private static Point2f[] Corners(RectangleF rectangle) => new[] {
            new Point2f(rectangle.Left, rectangle.Top), new Point2f(rectangle.Right, rectangle.Top),
            new Point2f(rectangle.Right, rectangle.Bottom), new Point2f(rectangle.Left, rectangle.Bottom) };

        private static RectangleF Bounds(Point2f[] points) => RectangleF.FromLTRB(points.Min(point => point.X),
            points.Min(point => point.Y), points.Max(point => point.X), points.Max(point => point.Y));

        private static bool Finite(Point2f point) => VideoDetection.Finite(point.X) && VideoDetection.Finite(point.Y);
        private static double Distance(Point2f left, Point2f right) =>
            Math.Sqrt((left.X - right.X) * (left.X - right.X) + (left.Y - right.Y) * (left.Y - right.Y));
    }
}
