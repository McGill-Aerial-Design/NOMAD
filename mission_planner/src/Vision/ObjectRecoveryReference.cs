// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    // Context proposes geometry; foreground inliers and masked appearance establish visual identity.
    internal sealed class ObjectRecoveryReference : IDisposable
    {
        private readonly Mat _descriptors = new Mat();
        private readonly KeyPoint[] _points;
        private readonly bool[] _foregroundPoints;
        private readonly System.Drawing.Rectangle _bounds;
        private readonly System.Drawing.Rectangle _supportBounds;
        private readonly FrozenAppearance _appearance;
        internal bool CanLocate => _points.Length >= 12 &&
            _foregroundPoints.Count(value => value) >= 8 && Distributed(_points.Where((point, i) => _foregroundPoints[i])
                .Select(point => point.Pt).ToArray());
        private sealed class Match
        {
            internal System.Drawing.Rectangle Bounds;
            internal Point2f[] Corners;
        }

        internal ObjectRecoveryReference(Mat image, Rect bounds, FrozenAppearance appearance, Mat foregroundMask)
        {
            _bounds = new System.Drawing.Rectangle(bounds.X, bounds.Y, bounds.Width, bounds.Height);
            _supportBounds = _bounds;
            _appearance = appearance;
            try
            {
                using (var gray = new Mat())
                using (var mask = new Mat(image.Size(), MatType.CV_8UC1, Scalar.All(0)))
                using (var orb = Features(600))
                {
                    Cv2.CvtColor(image, gray, ColorConversionCodes.BGR2GRAY);
                    Cv2.Rectangle(mask, bounds, Scalar.All(255), -1);
                    using (var selected = new Mat())
                    {
                        Cv2.BitwiseAnd(mask, foregroundMask, selected);
                        var support = Cv2.BoundingRect(selected);
                        _supportBounds = new System.Drawing.Rectangle(support.X, support.Y, support.Width, support.Height);
                    }
                    orb.DetectAndCompute(gray, mask, out _points, _descriptors);
                    if (_points.Length > 600)
                    { Array.Resize(ref _points, 600); _descriptors.Resize(600); }
                    _foregroundPoints = _points.Select(point => foregroundMask.At<byte>((int)point.Pt.Y, (int)point.Pt.X) != 0).ToArray();
                }
            }
            catch { Dispose(); throw; }
        }

        internal Rect? Find(Mat image, CancellationToken token)
        {
            if (!CanLocate) { return null; }
            using (var gray = new Mat())
            using (var descriptors = new Mat())
            using (var orb = Features(2000))
            {
                Cv2.CvtColor(image, gray, ColorConversionCodes.BGR2GRAY);
                orb.DetectAndCompute(gray, null, out KeyPoint[] points, descriptors);
                token.ThrowIfCancellationRequested();
                if (points.Length > 2000) { Array.Resize(ref points, 2000); descriptors.Resize(2000); }
                var first = FindPlacement(image, points, descriptors, token);
                if (first == null) { return null; }
                using (var remaining = new Mat())
                {
                    var others = new List<KeyPoint>();
                    for (int index = 0; index < points.Length; index++)
                    {
                        var p = points[index].Pt;
                        if (Cv2.PointPolygonTest(first.Corners, p, false) >= 0) { continue; }
                        others.Add(points[index]);
                        using (var row = descriptors.Row(index)) { remaining.PushBack(row); }
                    }
                    if (FindPlacement(image, others.ToArray(), remaining, token) != null) { return null; }
                }
                return NativeBox(first.Bounds);
            }
        }

        internal Rect? Refine(Mat image, Rect proposal, CancellationToken token)
        {
            if (!CanLocate) { return null; }
            var region = new Rect(Math.Max(0, proposal.X - proposal.Width / 2),
                Math.Max(0, proposal.Y - proposal.Height / 2), 0, 0);
            region.Width = Math.Min(image.Width, proposal.Right + proposal.Width / 2) - region.X;
            region.Height = Math.Min(image.Height, proposal.Bottom + proposal.Height / 2) - region.Y;
            if (region.Width <= 0 || region.Height <= 0) { return null; }
            using (var gray = new Mat())
            using (var mask = new Mat(image.Size(), MatType.CV_8UC1, Scalar.All(0)))
            using (var descriptors = new Mat())
            using (var orb = Features(600))
            {
                Cv2.CvtColor(image, gray, ColorConversionCodes.BGR2GRAY);
                Cv2.Rectangle(mask, region, Scalar.All(255), -1);
                orb.DetectAndCompute(gray, mask, out KeyPoint[] points, descriptors);
                if (points.Length > 600) { Array.Resize(ref points, 600); descriptors.Resize(600); }
                token.ThrowIfCancellationRequested();
                var found = FindPlacement(image, points, descriptors, token);
                return found == null ? (Rect?)null : NativeBox(found.Bounds);
            }
        }

        private Match FindPlacement(Mat image, KeyPoint[] points, Mat descriptors, CancellationToken token)
        {
            if (descriptors.Rows < 12) { return null; }

            var matches = VisualFeatureMatching.Mutual(_descriptors, descriptors);
            if (matches.Length < 12) { return null; }
            var source = matches.Select(match => _points[match.QueryIdx].Pt).ToArray();
            var destination = matches.Select(match => points[match.TrainIdx].Pt).ToArray();
            using (var inliers = new Mat())
            using (var mapping = Cv2.FindHomography(source.Select(p => new Point2d(p.X, p.Y)),
                destination.Select(p => new Point2d(p.X, p.Y)), HomographyMethods.Ransac, 2.5, inliers, 750, .995))
            {
                token.ThrowIfCancellationRequested();
                if (mapping.Empty() || inliers.Empty()) { return null; }
                var accepted = Enumerable.Range(0, source.Length).Where(i => inliers.At<byte>(i) != 0).ToArray();
                var foreground = accepted.Where(i => _foregroundPoints[matches[i].QueryIdx]).ToArray();
                if (accepted.Length < 12 || accepted.Length < source.Length * .55 || foreground.Length < 8 ||
                    !Distributed(foreground.Select(i => source[i]).ToArray())) { return null; }
                var projected = Cv2.PerspectiveTransform(source, mapping);
                if (accepted.Average(i => Distance(projected[i], destination[i])) > 1.5) { return null; }
                var corners = Cv2.PerspectiveTransform(new[] { new Point2f(_bounds.Left, _bounds.Top),
                    new Point2f(_bounds.Right, _bounds.Top), new Point2f(_bounds.Right, _bounds.Bottom),
                    new Point2f(_bounds.Left, _bounds.Bottom) }, mapping);
                if (corners.Any(p => !VideoDetection.Finite(p.X) || !VideoDetection.Finite(p.Y) || p.X < 0 || p.Y < 0 ||
                    p.X >= image.Width || p.Y >= image.Height) || !Cv2.IsContourConvex(corners)) { return null; }
                double scale = Cv2.ContourArea(corners, true) / (_bounds.Width * _bounds.Height);
                if (scale < .16 || scale > 6.25 || !_appearance.MatchesProjection(image, corners)) { return null; }
                int left = (int)Math.Floor(corners.Min(p => p.X)), top = (int)Math.Floor(corners.Min(p => p.Y));
                var result = new Rect(left, top, (int)Math.Ceiling(corners.Max(p => p.X)) - left,
                    (int)Math.Ceiling(corners.Max(p => p.Y)) - top);
                return result.Width >= 8 && result.Height >= 8 ? new Match { Bounds =
                    new System.Drawing.Rectangle(result.X, result.Y, result.Width, result.Height), Corners = corners } : null;
            }
        }

        private bool Distributed(Point2f[] points)
        {
            int quadrants = 0;
            foreach (var point in points)
            { quadrants |= 1 << ((point.X >= _supportBounds.X + _supportBounds.Width / 2f ? 1 : 0) +
                (point.Y >= _supportBounds.Y + _supportBounds.Height / 2f ? 2 : 0)); }
            bool surrounds = quadrants == 7 || quadrants == 11 || quadrants == 13 || quadrants == 14 || quadrants == 15;
            return surrounds && points.Max(p => p.X) - points.Min(p => p.X) >= _supportBounds.Width * .45 &&
                points.Max(p => p.Y) - points.Min(p => p.Y) >= _supportBounds.Height * .45;
        }

        private static ORB Features(int count) => ORB.Create(count, edgeThreshold: 12,
            scoreType: ORBScoreType.Fast, fastThreshold: 10);
        private static double Distance(Point2f a, Point2f b) => Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2));
        private static Rect NativeBox(System.Drawing.Rectangle bounds) => new Rect(bounds.X, bounds.Y, bounds.Width, bounds.Height);
        public void Dispose() { _descriptors.Dispose(); }
    }
}
