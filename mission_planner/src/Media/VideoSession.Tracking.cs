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
        private bool _trackerReady, _selectionFound, _selectionActive;
        private string _trackerError;
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
                var clone = (Bitmap)source.Image.Clone();
                RetireRegion();
                _pendingSelection = new RegionSelection
                { Image = clone, Bounds = bounds, Sequence = source.Sequence,
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
                _selectionCancellation?.Cancel();
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
            internal Bitmap Image;
            internal RectangleF Bounds;
            internal VisualTrackingMode Mode;
            internal long Sequence, ReceivedTicks, Generation;
            public void Dispose() => Image.Dispose();
        }

        // Constructed and used only by the decode worker; UI requests own one pending image clone.
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

            internal RegionWorker(VideoSession owner, CancellationToken token)
            {
                _owner = owner; _token = token;
                if (owner._createTracker == null) { return; }
                try
                {
                    _tracker = owner._createTracker(token, VisualTrackingMode.Object);
                    lock (owner._gate) { owner._trackerReady = true; owner._trackerError = null; }
                }
                catch (OperationCanceledException) { token.ThrowIfCancellationRequested(); }
                catch (Exception)
                { lock (owner._gate) { owner._trackerReady = false; owner._trackerError = "Visual tracking unavailable; video continues."; } }
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
                        _tracker = _tracker ?? _owner._createTracker(_token, _mode);
                        _cancellation = CancellationTokenSource.CreateLinkedTokenSource(_token);
                        lock (_owner._gate)
                        {
                            if (revision != _owner._selectionRevision) { _cancellation.Cancel(); }
                            _owner._selectionCancellation = _cancellation;
                        }
                        _tracker.Initialize(selection.Image, selection.Bounds, _cancellation.Token);
                    }
                    if (_tracker == null || _cancellation == null) { return _lastResult; }
                    PointF? hint = GroundHint(image.Size, motion);
                    var result = _tracker.Update(image, _cancellation.Token, new VisualFrameContext(hint));
                    bool found = result.Found && result.Bounds.HasValue && result.AimPoint.HasValue &&
                        ValidRegion(result.Bounds.Value, image.Size, false) &&
                        new RectangleF(0, 0, image.Width, image.Height).Contains(result.AimPoint.Value);
                    if (result.Found && !found) { return Lost("Invalid visual result; select again"); }
                    _token.ThrowIfCancellationRequested();
                    lock (_owner._gate)
                    {
                        if (_revision != _owner._selectionRevision) { return null; }
                        _owner._selectionFound = found;
                    }
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
                lock (_owner._gate)
                {
                    if (_revision != _owner._selectionRevision) { Clear(); return null; }
                    _owner._selectionFound = false;
                    _owner._selectionActive = false;
                }
                Clear();
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
