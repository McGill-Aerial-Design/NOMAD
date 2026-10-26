// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Threading;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    internal sealed partial class AreaVisualTracker : IVisualTracker
    {
        private const int MinimumInliers = 12;
        private Mat _reference, _descriptors;
        private KeyPoint[] _referencePoints;
        private RectangleF _selection, _context;
        private System.Drawing.Size _imageSize;
        private PointF _canonicalPoint;
        private PointF? _lastPoint, _motionHint;
        private int _consecutive;
        private bool _disposed;
        private bool _preferPrior;
        internal int LastHomographyFits { get; private set; }

        internal AreaVisualTracker() { Cv2.SetNumThreads(2); }

        public void Initialize(Bitmap frame, RectangleF bounds, CancellationToken token,
            VisualFrameContext context = default)
        {
            CheckDisposed(); Clear(); token.ThrowIfCancellationRequested();
            VisualTrackingImage.ValidateFrame(frame); VisualTrackingImage.ValidateSelection(frame, bounds);
            try
            {
                _reference = VisualTrackingImage.Gray(frame);
                float scaleX = (float)_reference.Width / frame.Width, scaleY = (float)_reference.Height / frame.Height;
                _selection = new RectangleF(bounds.X * scaleX, bounds.Y * scaleY,
                    bounds.Width * scaleX, bounds.Height * scaleY);
                if (_selection.Width < 24 || _selection.Height < 24)
                { throw new ArgumentException("Select a larger, distinctive area with visible surface texture."); }
                _canonicalPoint = new PointF(_selection.X + _selection.Width / 2,
                    _selection.Y + _selection.Height / 2);
                _context = RectangleF.Intersect(new RectangleF(0, 0, _reference.Width, _reference.Height),
                    new RectangleF(_selection.X - _selection.Width, _selection.Y - _selection.Height,
                        _selection.Width * 3, _selection.Height * 3));
                using (var mask = new Mat(_reference.Size(), MatType.CV_8UC1, Scalar.All(0)))
                using (var orb = CreateOrb())
                {
                    Cv2.Rectangle(mask, PixelBounds(_context), Scalar.All(255), -1);
                    _descriptors = new Mat();
                    orb.DetectAndCompute(_reference, mask, out _referencePoints, _descriptors);
                    BoundFeatures(_descriptors, ref _referencePoints);
                }
                token.ThrowIfCancellationRequested();
                RemoveRepeatedDescriptors();
                if (_descriptors.Rows < 24 || !DistributedSupport(_referencePoints))
                { throw new ArgumentException("Area lacks distinctive local texture; select a marked surface."); }
                _imageSize = frame.Size;
            }
            catch { Clear(); throw; }
        }

        public VisualTrackingResult Update(Bitmap frame, CancellationToken token, VisualFrameContext context = default)
        {
            CheckDisposed();
            LastHomographyFits = 0;
            if (token.IsCancellationRequested) { Clear(); }
            token.ThrowIfCancellationRequested(); VisualTrackingImage.ValidateFrame(frame);
            if (_reference == null) { return Lost("Select a distinctive surface area"); }
            if (frame.Size != _imageSize) { Clear(); return Lost("Image size changed; select the area again"); }
            try
            {
                using (var current = VisualTrackingImage.Gray(frame))
                using (var descriptors = new Mat())
                using (var orb = CreateOrb())
                {
                    orb.DetectAndCompute(current, null, out KeyPoint[] points, descriptors);
                    BoundFeatures(descriptors, ref points);
                    token.ThrowIfCancellationRequested();
                    var candidate = _preferPrior && context.SearchHint.HasValue
                        ? FindNearPrior(current, points, descriptors, context.SearchHint.Value, token)
                        : null;
                    // At most three bounded fits: local prior, global fallback, independent global ambiguity check.
                    candidate = candidate ?? FindPlacement(current, points, descriptors, token);
                    if (candidate == null)
                    {
                        _preferPrior = false;
                        return Search(context, "Area unverified; searching original reference");
                    }
                    if (HasAnotherPlacement(current, points, descriptors, candidate, token))
                    {
                        _consecutive = 0; _lastPoint = null; _preferPrior = false;
                        return new VisualTrackingResult(VisualTrackingState.Searching,
                            "Ambiguous repeated area; choose a distinctive surface feature");
                    }
                    token.ThrowIfCancellationRequested();
                    return Accept(candidate, frame.Size);
                }
            }
            catch { Clear(); throw; }
        }

        private VisualTrackingResult Accept(Placement candidate, System.Drawing.Size frameSize)
        {
            float scaleX = (float)frameSize.Width / _reference.Width, scaleY = (float)frameSize.Height / _reference.Height;
            var point = new PointF(candidate.Point.X * scaleX, candidate.Point.Y * scaleY);
            if (_lastPoint.HasValue)
            {
                var previous = _lastPoint.Value;
                _motionHint = new PointF(point.X + point.X - previous.X, point.Y + point.Y - previous.Y);
            }
            else { _motionHint = point; }
            _lastPoint = point;
            if (!new RectangleF(0, 0, frameSize.Width, frameSize.Height).Contains(point))
            { _consecutive = 0; return new VisualTrackingResult(VisualTrackingState.Outside,
                "Selected area point is outside the image", motionSearchHint: point); }
            if (++_consecutive < 3)
            { return new VisualTrackingResult(VisualTrackingState.Searching,
                "Confirming original area (" + _consecutive + "/3)", motionSearchHint: point); }
            var bounds = candidate.Bounds;
            _preferPrior = true;
            bounds = RectangleF.Intersect(bounds, new RectangleF(0, 0, _reference.Width, _reference.Height));
            return new VisualTrackingResult(VisualTrackingState.Verified, "Original surface area verified",
                new RectangleF(bounds.X * scaleX, bounds.Y * scaleY, bounds.Width * scaleX, bounds.Height * scaleY),
                point, _motionHint);
        }

        private VisualTrackingResult Search(VisualFrameContext context, string status)
        {
            _consecutive = 0; _lastPoint = null;
            var hint = context.SearchHint ?? _motionHint;
            if (hint.HasValue && (!VideoDetection.Finite(hint.Value.X) || !VideoDetection.Finite(hint.Value.Y)))
            { hint = null; }
            bool outside = hint.HasValue && !new RectangleF(0, 0, _imageSize.Width, _imageSize.Height).Contains(hint.Value);
            return new VisualTrackingResult(outside ? VisualTrackingState.Outside : VisualTrackingState.Searching,
                outside ? "Area unverified; coarse search hint is outside the image" : status, motionSearchHint: hint);
        }

        private static VisualTrackingResult Lost(string status) => new VisualTrackingResult(VisualTrackingState.Lost, status);
        private static ORB CreateOrb() => ORB.Create(1000, edgeThreshold: 12, fastThreshold: 10);
        private static Rect PixelBounds(RectangleF rectangle) => new Rect((int)rectangle.X, (int)rectangle.Y,
            Math.Max(1, (int)rectangle.Width), Math.Max(1, (int)rectangle.Height));

        private void CheckDisposed()
        { if (_disposed) { throw new ObjectDisposedException(nameof(AreaVisualTracker)); } }

        private void Clear()
        {
            _reference?.Dispose(); _reference = null; _descriptors?.Dispose(); _descriptors = null;
            _referencePoints = null; _lastPoint = null; _motionHint = null; _consecutive = 0; _preferPrior = false;
        }

        public void Dispose()
        { if (_disposed) { return; } _disposed = true; Clear(); }
    }
}
