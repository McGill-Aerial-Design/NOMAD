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
            internal Point2f[] Corners, Context;
            internal Point2f[] InlierPoints;
            internal Reference Reference;
        }

        private static void BoundFeatures(Mat descriptors, ref KeyPoint[] points, int limit = MaximumFeatures)
        {
            // Equal-response ORB corners can exceed its requested feature count.
            if (points.Length <= limit) { return; }
            Array.Resize(ref points, limit); descriptors.Resize(limit);
        }

        private void RemoveRepeatedDescriptors()
        {
            var view = _original;
            if (view.Descriptors.Rows < 2) { return; }
            using (var matcher = new BFMatcher(NormTypes.Hamming))
            using (var distinct = new Mat())
            {
                var neighbors = matcher.KnnMatch(view.Descriptors, view.Descriptors, 2);
                int localFeatures = view.Points.Count(point => Contains(view.Corners, point.Pt));
                var points = new List<KeyPoint>();
                for (int index = 0; index < neighbors.Length; index++)
                {
                    if (neighbors[index].Length < 2 || neighbors[index][1].Distance < 24) { continue; }
                    using (var row = view.Descriptors.Row(index)) { distinct.PushBack(row); }
                    points.Add(view.Points[index]);
                }
                view.Descriptors.Dispose(); view.Descriptors = distinct.Clone(); view.Points = points.ToArray();
                if (points.Count(point => Contains(view.Corners, point.Pt)) < localFeatures * .5)
                { throw new ArgumentException("Area texture repeats; select a distinctive marked surface."); }
            }
        }

        private Placement FindNearPrior(Reference view, Mat current, KeyPoint[] points, Mat descriptors, PointF hint,
            CancellationToken token)
        {
            if (!VideoDetection.Finite(hint.X) || !VideoDetection.Finite(hint.Y)) { return null; }
            float x = hint.X * current.Width / _imageSize.Width, y = hint.Y * current.Height / _imageSize.Height;
            float radius = VisualFrameContext.PriorSearchRadius(new System.Drawing.Size(current.Width, current.Height),
                view.Selection);
            using (var nearby = new Mat())
            {
                var nearbyPoints = new List<KeyPoint>();
                for (int index = 0; index < points.Length; index++)
                {
                    if (Math.Abs(points[index].Pt.X - x) > radius || Math.Abs(points[index].Pt.Y - y) > radius) { continue; }
                    using (var row = descriptors.Row(index)) { nearby.PushBack(row); }
                    nearbyPoints.Add(points[index]);
                }
                return FindPlacement(view, current, nearbyPoints.ToArray(), nearby, token);
            }
        }

        private Placement FindPlacement(Reference view, Mat current, KeyPoint[] points, Mat descriptors, CancellationToken token)
        {
            if (descriptors.Rows < MinimumInliers) { LastMatchDiagnostic = "Insufficient current descriptors"; return null; }
            var matches = VisualFeatureMatching.Mutual(view.Descriptors, descriptors);
            if (matches.Length < MinimumInliers) { LastMatchDiagnostic = "Insufficient mutual matches: " + matches.Length; return null; }
            var source = matches.Select(match => view.Points[match.QueryIdx].Pt).ToArray();
            var destination = matches.Select(match => points[match.TrainIdx].Pt).ToArray();
            return FitPlacement(view, current, source, destination, token);
        }

        private Placement FollowVisibleSurface(Reference view, Mat current, CancellationToken token)
        {
            var available = view.FlowPoints ?? view.Points.Select(value => value.Pt).ToArray();
            int count = Math.Min(160, available.Length);
            var source = Enumerable.Range(0, count).Select(index => available[index * available.Length / count]).ToArray();
            using (var previous = Mat.FromArray(source))
            using (var next = new Mat())
            using (var back = new Mat())
            using (var forwardStatus = new Mat())
            using (var backwardStatus = new Mat())
            using (var error = new Mat())
            {
                Cv2.CalcOpticalFlowPyrLK(view.Image, current, previous, next, forwardStatus, error,
                    new OpenCvSharp.Size(21, 21), 3, new TermCriteria(CriteriaTypes.Count | CriteriaTypes.Eps, 10, .03));
                token.ThrowIfCancellationRequested();
                next.GetArray(out Point2f[] destination); forwardStatus.GetArray(out byte[] forward);
                var measured = Enumerable.Range(0, source.Length).Where(index => forward[index] != 0 &&
                    Finite(destination[index]) && destination[index].X >= 0 && destination[index].Y >= 0 &&
                    destination[index].X < current.Width && destination[index].Y < current.Height).ToArray();
                if (measured.Length < MinimumInliers) { return null; }
                using (var measuredPoints = Mat.FromArray(measured.Select(index => destination[index]).ToArray()))
                {
                    Cv2.CalcOpticalFlowPyrLK(current, view.Image, measuredPoints, back, backwardStatus, error,
                        new OpenCvSharp.Size(21, 21), 3, new TermCriteria(CriteriaTypes.Count | CriteriaTypes.Eps, 10, .03));
                    token.ThrowIfCancellationRequested();
                    back.GetArray(out Point2f[] returned); backwardStatus.GetArray(out byte[] backward);
                    var reliable = Enumerable.Range(0, measured.Length).Where(index => backward[index] != 0 &&
                        Finite(returned[index]) && Distance(source[measured[index]], returned[index]) <= .75)
                        .Select(index => measured[index]).ToArray();
                    if (reliable.Length < MinimumInliers) { return null; }
                    return FitPlacement(view, current, reliable.Select(index => source[index]).ToArray(),
                        reliable.Select(index => destination[index]).ToArray(), token);
                }
            }
        }

        private Placement FitPlacement(Reference view, Mat current, Point2f[] source, Point2f[] destination,
            CancellationToken token)
        {
            LastHomographyFits++;
            using (var inliers = new Mat())
            using (var homography = Cv2.FindHomography(source.Select(point => new Point2d(point.X, point.Y)),
                destination.Select(point => new Point2d(point.X, point.Y)), HomographyMethods.Ransac, 2.5, inliers, 750, .995))
            {
                token.ThrowIfCancellationRequested();
                if (homography.Empty() || inliers.Empty()) { LastMatchDiagnostic = "Homography unavailable"; return null; }
                var accepted = Enumerable.Range(0, source.Length).Where(index => inliers.At<byte>(index) != 0).ToArray();
                if (accepted.Length < MinimumInliers || accepted.Length < source.Length * .55)
                { LastMatchDiagnostic = "Insufficient inliers: " + accepted.Length + "/" + source.Length; return null; }
                if (!DistributedSupport(view, accepted.Select(index => new KeyPoint(source[index].X, source[index].Y, 1)).ToArray()))
                { return null; }
                var projected = Cv2.PerspectiveTransform(source, homography);
                double meanError = accepted.Average(index => Distance(projected[index], destination[index]));
                if (meanError > 1.5) { LastMatchDiagnostic = "Reprojection error: " + meanError; return null; }
                var corners = Cv2.PerspectiveTransform(view.Corners, homography);
                var context = Cv2.PerspectiveTransform(view.Context, homography);
                var aim = Cv2.PerspectiveTransform(new[] { new Point2f(view.Point.X, view.Point.Y) }, homography)[0];
                if (!ValidGeometry(corners, context, aim, current.Size()))
                { LastMatchDiagnostic = "Invalid projected geometry"; return null; }
                if (!TargetVisible(view, current, corners))
                { LastMatchDiagnostic = "Selected target appearance not verified"; return null; }
                return new Placement { Point = new PointF(aim.X, aim.Y), Bounds = Bounds(corners),
                    Corners = corners, Context = context, Reference = view,
                    InlierPoints = accepted.Select(index => destination[index]).ToArray() };
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
                    // Keep surrounding evidence available to distinguish a nearby competing placement.
                    if (Cv2.PointPolygonTest(first.Corners, points[index].Pt, false) >= 0) { continue; }
                    using (var row = descriptors.Row(index)) { remaining.PushBack(row); }
                    remainingPoints.Add(points[index]);
                }
                token.ThrowIfCancellationRequested();
                // Rematch the same frozen reference outside the first placement, including ratio-test rejections.
                foreach (var view in _active == _original ? new[] { _original } : new[] { _active, _original })
                {
                    var second = FindPlacement(view, current, remainingPoints.ToArray(), remaining, token);
                    if (second != null && Math.Sqrt(Math.Pow(first.Point.X - second.Point.X, 2) +
                        Math.Pow(first.Point.Y - second.Point.Y, 2)) > Math.Max(12,
                        Math.Min(first.Bounds.Width, first.Bounds.Height) * .5)) { return true; }
                }
                return false;
            }
        }

        private bool DistributedSupport(Reference view, KeyPoint[] points)
        {
            if (view != _original)
            {
                using (var mapping = Cv2.GetPerspectiveTransform(view.Corners, _original.Corners))
                {
                    var projected = Cv2.PerspectiveTransform(points.Select(value => value.Pt).ToArray(), mapping);
                    points = projected.Select(point => new KeyPoint(point.X, point.Y, 1)).ToArray();
                }
                view = _original;
            }
            int local = 0, context = 0, quadrants = 0;
            float left = float.MaxValue, top = float.MaxValue, right = float.MinValue, bottom = float.MinValue;
            foreach (var keypoint in points)
            {
                var point = keypoint.Pt;
                if (Contains(view.Corners, point))
                {
                    local++;
                    int quadrant = (point.X >= view.Point.X ? 1 : 0) + (point.Y >= view.Point.Y ? 2 : 0);
                    quadrants |= 1 << quadrant;
                    left = Math.Min(left, point.X); right = Math.Max(right, point.X);
                    top = Math.Min(top, point.Y); bottom = Math.Max(bottom, point.Y);
                }
                else { context++; }
            }
            bool surroundsPoint = quadrants == 7 || quadrants == 11 || quadrants == 13 || quadrants == 14 || quadrants == 15;
            LastMatchDiagnostic = "Support local=" + local + ", context=" + context + ", quadrants=" + quadrants +
                ", span=" + (right - left) + "/" + view.Selection.Width + "," + (bottom - top) + "/" + view.Selection.Height;
            return local >= 8 && context >= 6 && surroundsPoint &&
                right - left >= view.Selection.Width * .45 && bottom - top >= view.Selection.Height * .45;
        }

        private static Mat Rectify(Mat image, Point2f[] corners)
        {
            var result = new Mat();
            try
            {
                using (var transform = Cv2.GetPerspectiveTransform(corners, new[] { new Point2f(0, 0),
                    new Point2f(95, 0), new Point2f(95, 95), new Point2f(0, 95) }))
                { Cv2.WarpPerspective(image, result, transform, new OpenCvSharp.Size(96, 96)); }
                return result;
            }
            catch { result.Dispose(); throw; }
        }

        private bool TargetVisible(Reference view, Mat current, Point2f[] corners)
        {
            using (var candidate = Rectify(current, corners))
            {
                // Learn feature views, while retaining a contrast-normalized identity witness against slow takeover.
                return FrozenAppearance.SameSurface(candidate, view.Appearance, .5) &&
                    (view == _original || FrozenAppearance.SameSurface(candidate, _original.Appearance, .5));
            }
        }

        private bool ValidGeometry(Point2f[] corners, Point2f[] context, Point2f aim, OpenCvSharp.Size imageSize)
        {
            if (!Finite(aim) || corners.Any(point => !Finite(point)) || context.Any(point => !Finite(point))) { return false; }
            if (!Cv2.IsContourConvex(corners) || !Cv2.IsContourConvex(context)) { return false; }
            double area = Cv2.ContourArea(corners, true), original = _original.Selection.Width * _original.Selection.Height;
            if (area < original * .16 || area > original * 6.25 || area > imageSize.Width * imageSize.Height * .7)
            { return false; }
            for (int index = 0; index < 4; index++)
            {
                double edge = Distance(corners[index], corners[(index + 1) % 4]);
                if (edge < 8 || edge > Math.Max(imageSize.Width, imageSize.Height) * 2) { return false; }
            }
            return Cv2.PointPolygonTest(corners, aim, false) >= 0;
        }

        private void DetectReferenceFeatures(Reference view, CancellationToken token, Point2f[] support = null)
        {
            var retained = new List<KeyPoint>();
            view.Descriptors = new Mat();
            // Reserve independent local/context quotas, including each detector's coarse pyramid levels.
            foreach (bool local in new[] { true, false })
            using (var mask = new Mat(view.Image.Size(), MatType.CV_8UC1, Scalar.All(0)))
            using (var descriptors = new Mat())
            using (var orb = CreateOrb(ReferenceFeatures / 2))
            {
                var polygon = (local ? view.Corners : view.Context).Select(point => new OpenCvSharp.Point(
                    (int)Math.Round(Math.Max(-32768, Math.Min(32768, point.X))),
                    (int)Math.Round(Math.Max(-32768, Math.Min(32768, point.Y))))).ToArray();
                Cv2.FillConvexPoly(mask, polygon, Scalar.All(255));
                orb.DetectAndCompute(view.Image, mask, out KeyPoint[] points, descriptors);
                token.ThrowIfCancellationRequested();
                int added = 0;
                for (int index = 0; index < points.Length && added < ReferenceFeatures / 2; index++)
                {
                    if (Contains(view.Corners, points[index].Pt) != local) { continue; }
                    if (support != null && !support.Any(point => Distance(point, points[index].Pt) <= 2.5)) { continue; }
                    using (var row = descriptors.Row(index)) { view.Descriptors.PushBack(row); }
                    retained.Add(points[index]); added++;
                }
            }
            view.Points = retained.ToArray();
        }

        private void Promote(Mat current, Placement candidate, KeyPoint[] points, Mat descriptors, CancellationToken token)
        {
            // Keep measured plane tracks; re-detecting ORB corners alone can erode their spatial support.
            var view = new Reference { Image = current.Clone(),
                Selection = candidate.Bounds, Corners = candidate.Corners, Context = candidate.Context, Point = candidate.Point,
                FlowPoints = candidate.InlierPoints };
            try
            {
                view.Descriptors = new Mat();
                var retained = new List<KeyPoint>();
                foreach (bool local in new[] { true, false })
                {
                    int added = 0;
                    for (int index = 0; index < points.Length && added < ReferenceFeatures / 2; index++)
                    {
                        if (Contains(view.Corners, points[index].Pt) != local ||
                            !Contains(view.Context, points[index].Pt)) { continue; }
                        // Reuse this frame's descriptors only near measured plane inliers.
                        if (!candidate.InlierPoints.Any(point => Distance(point, points[index].Pt) <= 2.5)) { continue; }
                        using (var row = descriptors.Row(index)) { view.Descriptors.PushBack(row); }
                        retained.Add(points[index]); added++;
                    }
                }
                token.ThrowIfCancellationRequested(); view.Points = retained.ToArray();
                if (view.Descriptors.Rows < 24 || !DistributedSupport(view, view.Points))
                {
                    // A sparse global budget may omit local corners needed to retain this verified view.
                    view.Descriptors.Dispose();
                    DetectReferenceFeatures(view, token, candidate.InlierPoints);
                    if (view.Descriptors.Rows < 24 || !DistributedSupport(view, view.Points)) { return; }
                }
                view.Appearance = Rectify(current, view.Corners);
                if (_active != _original) { _active.Dispose(); }
                _active = view; view = null; _keyframeMatches = 0; LearnedViews++;
            }
            finally { view?.Dispose(); }
        }

        private static bool Contains(Point2f[] polygon, Point2f point) => Cv2.PointPolygonTest(polygon, point, false) >= 0;

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
