// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    internal sealed partial class AreaVisualTracker : IVisualTracker
    {
        private const int MinimumInliers = 12;
        private const int MaximumFeatures = 1500;
        private const int ReferenceFeatures = 1000;
        private sealed class Reference : IDisposable
        {
            internal Mat Image, Descriptors, Appearance;
            internal KeyPoint[] Points;
            internal Point2f[] FlowPoints;
            internal RectangleF Selection;
            internal Point2f[] Corners, Context;
            internal PointF Point;
            public void Dispose() { Image?.Dispose(); Descriptors?.Dispose(); Appearance?.Dispose(); }
        }
        private Reference _original, _active;
        private System.Drawing.Size _imageSize;
        private PointF? _lastPoint, _motionHint;
        private int _consecutive, _keyframeMatches;
        private bool _disposed;
        private bool _preferPrior;
        private bool _localContinuity;
        internal int LastHomographyFits { get; private set; }
        internal int LearnedViews { get; private set; }
        internal string LastCandidateSource { get; private set; }
        internal string LastMatchDiagnostic { get; private set; }
        internal object LastTiming { get; private set; }

        internal AreaVisualTracker() { Cv2.SetNumThreads(2); }

        public void Initialize(Bitmap frame, RectangleF bounds, CancellationToken token,
            VisualFrameContext context = default)
        {
            CheckDisposed(); Clear(); token.ThrowIfCancellationRequested();
            VisualTrackingImage.ValidateFrame(frame); VisualTrackingImage.ValidateSelection(frame, bounds);
            try
            {
                var gray = VisualTrackingImage.Gray(frame);
                _original = _active = new Reference { Image = gray };
                float scaleX = (float)gray.Width / frame.Width, scaleY = (float)gray.Height / frame.Height;
                var selection = new RectangleF(bounds.X * scaleX, bounds.Y * scaleY,
                    bounds.Width * scaleX, bounds.Height * scaleY);
                if (selection.Width < 24 || selection.Height < 24)
                { throw new ArgumentException("Select a larger, distinctive area with visible surface texture."); }
                _original.Selection = selection;
                _original.Point = new PointF(selection.X + selection.Width / 2, selection.Y + selection.Height / 2);
                _original.Corners = Corners(selection);
                var contextBounds = RectangleF.Intersect(new RectangleF(0, 0, gray.Width, gray.Height),
                    new RectangleF(selection.X - selection.Width, selection.Y - selection.Height,
                        selection.Width * 3, selection.Height * 3));
                _original.Context = Corners(contextBounds);
                DetectReferenceFeatures(_original, token);
                token.ThrowIfCancellationRequested();
                RemoveRepeatedDescriptors();
                if (_original.Descriptors.Rows < 24 || !DistributedSupport(_original, _original.Points))
                { throw new ArgumentException("Area lacks distinctive local texture; select a marked surface."); }
                _original.Appearance = Rectify(gray, _original.Corners);
                _localContinuity = true;
                _imageSize = frame.Size;
            }
            catch { Clear(); throw; }
        }

        public VisualTrackingResult Update(Bitmap frame, CancellationToken token, VisualFrameContext context = default)
        {
            CheckDisposed();
            LastHomographyFits = 0;
            LastCandidateSource = "none";
            LastMatchDiagnostic = "No placement evaluated";
            if (token.IsCancellationRequested) { Clear(); }
            token.ThrowIfCancellationRequested(); VisualTrackingImage.ValidateFrame(frame);
            if (_original == null) { return Lost("Select a distinctive surface area"); }
            if (frame.Size != _imageSize) { Clear(); return Lost("Image size changed; select the area again"); }
            if (context.Discontinuous)
            { _consecutive = 0; _keyframeMatches = 0; _lastPoint = null; _localContinuity = false; }
            long started = Stopwatch.GetTimestamp();
            double featuresMs = 0, flowMs = 0, matchingMs = 0, ambiguityMs = 0, acceptanceMs = 0;
            int currentFeatures = 0;
            try
            {
                using (var current = VisualTrackingImage.Gray(frame))
                using (var descriptors = new Mat())
                {
                    long stage = Stopwatch.GetTimestamp();
                    // The operator's frozen reference can bootstrap checked flow; admission still needs three matches.
                    var candidate = _localContinuity ? FollowVisibleSurface(_active, current, token) : null;
                    flowMs = ElapsedMs(stage); stage = Stopwatch.GetTimestamp();
                    if (candidate != null) { LastCandidateSource = "flow"; }
                    // Retain the full recovery budget whenever checked local motion cannot locate the surface.
                    int featureBudget = _consecutive >= 3 && candidate != null ? 400 : MaximumFeatures;
                    KeyPoint[] points;
                    using (var orb = CreateOrb(featureBudget))
                    { orb.DetectAndCompute(current, null, out points, descriptors); }
                    BoundFeatures(descriptors, ref points, featureBudget);
                    currentFeatures = points.Length; featuresMs = ElapsedMs(stage);
                    token.ThrowIfCancellationRequested(); stage = Stopwatch.GetTimestamp();
                    candidate = candidate ?? (_active == _original && _preferPrior && context.SearchHint.HasValue
                        ? FindNearPrior(_active, current, points, descriptors, context.SearchHint.Value, token)
                        : null);
                    // At most five fits, including flow, global recovery and both-view ambiguity checks.
                    candidate = candidate ?? FindPlacement(_active, current, points, descriptors, token);
                    if (candidate == null && _active != _original)
                    {
                        string activeFailure = LastMatchDiagnostic;
                        candidate = FindPlacement(_original, current, points, descriptors, token);
                        if (candidate == null)
                        { LastMatchDiagnostic = "Working view: " + activeFailure + "; original: " + LastMatchDiagnostic; }
                    }
                    if (candidate == null)
                    {
                        _preferPrior = false;
                        return Search(context, "Area unverified; searching original reference");
                    }
                    matchingMs = ElapsedMs(stage); stage = Stopwatch.GetTimestamp();
                    if (LastCandidateSource == "none")
                    { LastCandidateSource = candidate.Reference == _original ? "original features" : "working features"; }
                    bool ambiguous = HasAnotherPlacement(current, points, descriptors, candidate, token);
                    ambiguityMs = ElapsedMs(stage); stage = Stopwatch.GetTimestamp();
                    if (ambiguous)
                    {
                        _consecutive = 0; _keyframeMatches = 0; _lastPoint = null; _preferPrior = false;
                        _localContinuity = false;
                        return new VisualTrackingResult(VisualTrackingState.Searching,
                            "Ambiguous repeated area; choose a distinctive surface feature");
                    }
                    token.ThrowIfCancellationRequested();
                    var result = Accept(candidate, current, points, descriptors, frame.Size, token);
                    acceptanceMs = ElapsedMs(stage);
                    return result;
                }
            }
            catch { Clear(); throw; }
            finally
            {
                LastTiming = new { features_ms = featuresMs, flow_ms = flowMs, matching_ms = matchingMs,
                    ambiguity_ms = ambiguityMs, acceptance_ms = acceptanceMs, total_ms = ElapsedMs(started),
                    current_features = currentFeatures, reference_features = _active?.Points.Length ?? 0,
                    fits = LastHomographyFits, candidate = LastCandidateSource };
            }
        }

        private static double ElapsedMs(long since) =>
            (Stopwatch.GetTimestamp() - since) * 1000.0 / Stopwatch.Frequency;

        private VisualTrackingResult Accept(Placement candidate, Mat current, KeyPoint[] points, Mat descriptors,
            System.Drawing.Size frameSize, CancellationToken token)
        {
            float scaleX = (float)frameSize.Width / current.Width, scaleY = (float)frameSize.Height / current.Height;
            var point = new PointF(candidate.Point.X * scaleX, candidate.Point.Y * scaleY);
            if (_lastPoint.HasValue)
            {
                var previous = _lastPoint.Value;
                _motionHint = new PointF(point.X + point.X - previous.X, point.Y + point.Y - previous.Y);
            }
            else { _motionHint = point; }
            _lastPoint = point;
            if (!new RectangleF(0, 0, frameSize.Width, frameSize.Height).Contains(point))
            { _consecutive = 0; _keyframeMatches = 0; _localContinuity = false;
                return new VisualTrackingResult(VisualTrackingState.Outside,
                "Selected area point is outside the image", motionSearchHint: point); }
            double viewChange = Math.Max(2, Math.Min(candidate.Reference.Selection.Width,
                candidate.Reference.Selection.Height) * .08);
            bool changed = candidate.Corners.Where((corner, index) =>
                Distance(corner, candidate.Reference.Corners[index]) >= viewChange).Any();
            _keyframeMatches = changed ? _keyframeMatches + 1 : 0;
            if (++_consecutive < 3)
            { return new VisualTrackingResult(VisualTrackingState.Searching,
                "Confirming original area (" + _consecutive + "/3)", motionSearchHint: point); }
            _localContinuity = true;
            if (_keyframeMatches >= 3)
            {
                if (candidate.Reference != _original && LastHomographyFits < 5)
                {
                    var anchor = FindPlacement(_original, current, points, descriptors, token);
                    if (anchor != null && Distance(new Point2f(anchor.Point.X, anchor.Point.Y),
                        new Point2f(candidate.Point.X, candidate.Point.Y)) > 6)
                    { return Search(default, "Working view disagrees with original area; learning paused"); }
                }
                Promote(current, candidate, points, descriptors, token);
            }
            LastMatchDiagnostic = "Verified geometry and selected-surface appearance";
            var bounds = candidate.Bounds;
            _preferPrior = true;
            bounds = RectangleF.Intersect(bounds, new RectangleF(0, 0, current.Width, current.Height));
            return new VisualTrackingResult(VisualTrackingState.Verified, "Selected surface area verified",
                new RectangleF(bounds.X * scaleX, bounds.Y * scaleY, bounds.Width * scaleX, bounds.Height * scaleY),
                point, _motionHint);
        }

        private VisualTrackingResult Search(VisualFrameContext context, string status)
        {
            _consecutive = 0; _keyframeMatches = 0; _lastPoint = null; _localContinuity = false;
            var hint = context.SearchHint ?? _motionHint;
            if (hint.HasValue && (!VideoDetection.Finite(hint.Value.X) || !VideoDetection.Finite(hint.Value.Y)))
            { hint = null; }
            bool outside = hint.HasValue && !new RectangleF(0, 0, _imageSize.Width, _imageSize.Height).Contains(hint.Value);
            return new VisualTrackingResult(outside ? VisualTrackingState.Outside : VisualTrackingState.Searching,
                outside ? "Area unverified; coarse search hint is outside the image" : status, motionSearchHint: hint);
        }

        private static VisualTrackingResult Lost(string status) => new VisualTrackingResult(VisualTrackingState.Lost, status);
        // FAST ranking avoids concentrating the feature budget on one high-gradient strip.
        private static ORB CreateOrb(int count = MaximumFeatures) => ORB.Create(count, edgeThreshold: 12,
            scoreType: ORBScoreType.Fast, fastThreshold: 10);
        private void CheckDisposed()
        { if (_disposed) { throw new ObjectDisposedException(nameof(AreaVisualTracker)); } }

        private void Clear()
        {
            if (_active != _original) { _active?.Dispose(); }
            _original?.Dispose(); _original = null; _active = null;
            _lastPoint = null; _motionHint = null; _consecutive = 0; _keyframeMatches = 0;
            _preferPrior = false; _localContinuity = false; LearnedViews = 0;
        }

        public void Dispose()
        { if (_disposed) { return; } _disposed = true; Clear(); }
    }
}
