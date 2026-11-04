// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    internal sealed partial class VideoSession
    {
        private readonly Func<CancellationToken, VisualTrackingMode, IVisualTracker> _createTracker;
        private RegionSelection _pendingSelection;
        private CancellationTokenSource _selectionCancellation;
        private long _selectionRevision;
        private bool _trackerReady, _selectionFound, _selectionActive, _trackerLoading;
        private string _trackerError;
        internal bool TrackerLoading { get { lock (_gate) { return _trackerLoading; } } }
        internal bool TrackerReady { get { lock (_gate) { return _trackerReady; } } }
        internal string TrackerError { get { lock (_gate) { return _trackerError; } } }
        internal bool IsRegionCurrent(long revision)
        { lock (_gate) { return _selectionFound && revision == _selectionRevision && _state == VideoState.Streaming; } }
        internal bool HasRegion(long revision)
        { lock (_gate) { return _selectionActive && revision == _selectionRevision && _state == VideoState.Streaming; } }

        internal long RequestRegion(VideoFrame source, RectangleF bounds, VisualTrackingMode mode = VisualTrackingMode.Object)
        {
            if (source?.Fresh != true || !ValidRegion(bounds, source.Image.Size, true)) { return 0; }
            lock (_gate)
            {
                if (!_trackerReady || _state != VideoState.Streaming || source.Generation != _generation || !source.Fresh)
                { return 0; }
                var retained = source.Retain();
                RetireRegion();
                _pendingSelection = new RegionSelection
                { Frame = retained, Bounds = bounds, Sequence = source.Sequence,
                    Generation = source.Generation, ReceivedTicks = source.ReceivedTicks, Mode = mode };
                _selectionActive = true;
                ClearPendingFrame();
                return _selectionRevision;
            }
        }

        internal void RetireRegion()
        {
            lock (_gate)
            {
                ++_selectionRevision;
                _selectionFound = false;
                _selectionActive = false;
                _pendingTracking = null; _trackingSequence = 0;
                CancelLater(_selectionCancellation);
                _pendingSelection?.Dispose(); _pendingSelection = null;
            }
        }

        private static bool ValidRegion(RectangleF bounds, Size image, bool initial)
        {
            float minimum = initial ? 8 * Math.Max(1, Math.Max(image.Width, image.Height) / 640f) : 1;
            return VideoDetection.Finite(bounds.X) && VideoDetection.Finite(bounds.Y) &&
                VideoDetection.Finite(bounds.Width) && VideoDetection.Finite(bounds.Height) &&
                bounds.X >= 0 && bounds.Y >= 0 && bounds.Width >= minimum && bounds.Height >= minimum &&
                bounds.Right <= image.Width && bounds.Bottom <= image.Height;
        }

        private sealed class RegionSelection : IDisposable
        {
            internal VideoFrame Frame;
            internal Bitmap Image => Frame.Image;
            internal RectangleF Bounds;
            internal VisualTrackingMode Mode;
            internal long Sequence, ReceivedTicks, Generation;
            public void Dispose() => Frame.Dispose();
        }

        // The tracking lane owns native state; UI requests retain one immutable source frame.
        private sealed class RegionWorker : IDisposable
        {
            private readonly VideoSession _owner;
            private readonly CancellationToken _token;
            private IVisualTracker _tracker;
            private CancellationTokenSource _cancellation;
            private long _revision;
            private VideoRegionResult _lastResult;
            private VisualTrackingMode _mode;
            private GroundAreaAnchor _groundAnchor;
            private RectangleF _referenceSelection;
            private bool _groundPriorDisabled;
            private long _lastReceivedTicks;

            internal RegionWorker(VideoSession owner, CancellationToken token)
            {
                _owner = owner; _token = token;
                lock (owner._gate) { owner._trackerReady = false; owner._trackerError = null; }

            }

            internal void Warm()
            {
                if (_owner._createTracker == null) { return; }
                if (_owner._plainRecovery)
                { lock (_owner._gate) { _owner._trackerReady = true; } return; }
                lock (_owner._gate) { _owner._trackerLoading = true; }
                try
                {
                    _tracker = _owner._createTracker(_token, VisualTrackingMode.Object);
                    lock (_owner._gate) { _owner._trackerReady = true; _owner._trackerError = null; }
                }
                catch (OperationCanceledException) { }
                catch (Exception error)
                {
                    lock (_owner._gate)
                    { _owner._trackerReady = false; _owner._trackerError = error.Message + " Video continues."; }
                    Log.Error("Tracker load unavailable: " + error.GetType().Name);
                }
                finally { lock (_owner._gate) { _owner._trackerLoading = false; } }
            }

            internal VideoRegionResult Update(Bitmap image, long received, long sequence, VideoMotionSnapshot motion)
            {
                RegionSelection selection;
                long revision;
                lock (_owner._gate)
                {
                    revision = _owner._selectionRevision;
                    selection = _owner._pendingSelection; _owner._pendingSelection = null;
                }
                using (selection)
                try
                {
                    if (revision != _revision) { Clear(); _revision = revision; }
                    if (selection != null)
                    {
                        double age = (Stopwatch.GetTimestamp() - selection.ReceivedTicks) / (double)Stopwatch.Frequency;
                        if (age < 0 || age >= .5 || image.Size != selection.Image.Size || sequence <= selection.Sequence ||
                            selection.Generation != _owner.Generation)
                        { return Lost("Selection expired; select the current image again"); }
                        _mode = selection.Mode;
                        _referenceSelection = selection.Bounds;
                        if (_tracker == null)
                        {
                            lock (_owner._gate) { _owner._trackerLoading = true; }
                            try { _tracker = _owner._createTracker(_token, _mode); }
                            finally { lock (_owner._gate) { _owner._trackerLoading = false; } }
                            if (!selection.Frame.Fresh)
                            { return Lost("Tracker ready; select the current image again"); }
                        }
                        _cancellation = CancellationTokenSource.CreateLinkedTokenSource(_token);
                        lock (_owner._gate)
                        {
                            if (revision != _owner._selectionRevision) { CancelLater(_cancellation); }
                            _owner._selectionCancellation = _cancellation;
                        }
                        _tracker.Initialize(selection.Image, selection.Bounds, _cancellation.Token);
                    }
                    if (_tracker == null || _cancellation == null) { return _lastResult; }
                    PointF? hint = GroundHint(image.Size, motion);
                    bool gap = _lastReceivedTicks != 0 && (received <= _lastReceivedTicks ||
                        (received - _lastReceivedTicks) / (double)Stopwatch.Frequency >= .5);
                    _lastReceivedTicks = received;
                    var result = _tracker.Update(image, _cancellation.Token, new VisualFrameContext(hint, gap));
                    bool found = result.Found && result.Bounds.HasValue && result.AimPoint.HasValue &&
                        ValidRegion(result.Bounds.Value, image.Size, false) &&
                        new RectangleF(0, 0, image.Width, image.Height).Contains(result.AimPoint.Value);
                    if (result.Found && !found) { return Lost("Invalid visual result; select again"); }
                    _token.ThrowIfCancellationRequested();
                    bool current;
                    lock (_owner._gate)
                    { current = _revision == _owner._selectionRevision; if (current) { _owner._selectionFound = found; } }
                    if (!current) { Clear(); return null; }
                    if (result.State == VisualTrackingState.Lost) { return Lost(result.Status); }
                    if (found && hint.HasValue)
                    {
                        float radius = VisualFrameContext.PriorSearchRadius(image.Size, _referenceSelection);
                        if (Math.Abs(result.AimPoint.Value.X - hint.Value.X) > radius ||
                            Math.Abs(result.AimPoint.Value.Y - hint.Value.Y) > radius)
                        { _groundPriorDisabled = true; _groundAnchor = null; hint = null; }
                    }
                    if (found && _mode == VisualTrackingMode.Area && _owner._trackingProfile.GroundPredictionEnabled &&
                        !_groundPriorDisabled && _groundAnchor == null && motion?.IsCurrent == true)
                    { GroundProjection.TryCreateAnchor(_owner._trackingProfile, image.Size, result.AimPoint.Value,
                        motion.Ground, _owner._trackingProfile.GroundPlaneHeightMeters, out _groundAnchor); }
                    return _lastResult = new VideoRegionResult(_revision, _mode, result, hint);
                }
                catch (OperationCanceledException) { Clear(); _token.ThrowIfCancellationRequested(); return null; }
                catch (Exception) { return Lost("Selection failed; choose a larger textured region"); }
            }

            private VideoRegionResult Lost(string status)
            {
                bool current;
                lock (_owner._gate)
                {
                    current = _revision == _owner._selectionRevision;
                    if (current) { _owner._selectionFound = false; _owner._selectionActive = false; }
                }
                Clear();
                if (!current) { return null; }
                return _lastResult = new VideoRegionResult(_revision, _mode,
                    new VisualTrackingResult(VisualTrackingState.Lost, status));
            }

            private PointF? GroundHint(Size image, VideoMotionSnapshot motion)
            {
                if (_mode != VisualTrackingMode.Area || !_owner._trackingProfile.GroundPredictionEnabled ||
                    _groundPriorDisabled || motion?.IsCurrent != true) { return null; }
                if (_groundAnchor != null && !ReferenceEquals(_groundAnchor.Epoch, motion.Epoch)) { _groundAnchor = null; }
                return GroundProjection.TryProjectAnchor(_owner._trackingProfile, image, _groundAnchor,
                    motion.Ground, out var point) ? point : (PointF?)null;
            }

            private void Clear()
            {
                var source = _cancellation;
                _cancellation = null;
                _lastResult = null;
                _groundAnchor = null;
                _groundPriorDisabled = false;
                _lastReceivedTicks = 0;
                lock (_owner._gate)
                { if (ReferenceEquals(_owner._selectionCancellation, source)) { _owner._selectionCancellation = null; } }
                try { _tracker?.Dispose(); }
                finally { _tracker = null; source?.Dispose(); }
            }

            public void Dispose()
            {
                Clear();
                lock (_owner._gate) { _owner._trackerReady = false; _owner._selectionFound = false; }
            }
        }
    }
}
