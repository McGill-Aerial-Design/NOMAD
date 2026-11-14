// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Threading;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    internal sealed partial class ObjectVisualTracker : IVisualTracker
    {
        private bool _following;
        private CpuTemplateTracker _templateTracker;
        private bool _templateAccepted;
        private FrozenAppearance _appearance;
        private ObjectRecoveryReference _reference;
        private Rectangle? _recoveryBounds;
        private int _recoveryMatches;
        private System.Drawing.Size _imageSize;
        private int _maximumImageDimension = 640;
        internal System.Drawing.Size ProcessingImageSize { get; private set; }
        private bool _disposed;

        internal ObjectVisualTracker() { Cv2.SetNumThreads(2); }

        public void Initialize(Bitmap frame, RectangleF bounds, CancellationToken token,
            VisualFrameContext context = default)
        {
            CheckDisposed();
            Clear();
            token.ThrowIfCancellationRequested();
            VisualTrackingImage.ValidateFrame(frame);
            VisualTrackingImage.ValidateSelection(frame, bounds);
            // Select the pixel budget once, retaining the original 640-coordinate foreground policy. Only an object
            // narrower than the 48 px appearance patch at 640 needs the 4x-pixel 1280 budget that GrabCut pays for.
            double baselineScale = Math.Min(1, 640.0 / Math.Max(frame.Width, frame.Height));
            int baselineWidth = (int)(bounds.Width * ((int)(frame.Width * baselineScale) / (double)frame.Width));
            int baselineHeight = (int)(bounds.Height * ((int)(frame.Height * baselineScale) / (double)frame.Height));
            _maximumImageDimension = Math.Min(baselineWidth, baselineHeight) < 48 ? 1280 : 640;
            bool smallSelection = Math.Max(baselineWidth, baselineHeight) <= 80;
            using (var image = VisualTrackingImage.Color(frame, _maximumImageDimension))
            {
                double scaleX = (double)image.Width / frame.Width, scaleY = (double)image.Height / frame.Height;
                var box = new OpenCvSharp.Rect((int)(bounds.X * scaleX), (int)(bounds.Y * scaleY),
                    (int)(bounds.Width * scaleX), (int)(bounds.Height * scaleY));
                if (box.Width < 8 || box.Height < 8 || !HasTexture(image, box))
                { throw new ArgumentException("Select a larger region with visible texture or edges."); }
                try
                {
                    token.ThrowIfCancellationRequested();
                    InitializeObject(image, box, smallSelection, token);
                    token.ThrowIfCancellationRequested();
                    _imageSize = frame.Size;
                    ProcessingImageSize = new System.Drawing.Size(image.Width, image.Height);
                }
                catch { Clear(); throw; }
            }
        }

        public VisualTrackingResult Update(Bitmap frame, CancellationToken token, VisualFrameContext context = default)
        {
            CheckDisposed();
            if (token.IsCancellationRequested) { Clear(); }
            token.ThrowIfCancellationRequested();
            VisualTrackingImage.ValidateFrame(frame);
            if (_appearance == null && _templateTracker == null) { return Lost("Select a distinctive object"); }
            if (frame.Size != _imageSize) { Clear(); return Lost("Image size changed; select the object again"); }
            if (context.Discontinuous) { ResetLocalTrack(); }
            using (var image = VisualTrackingImage.Color(frame, _maximumImageDimension))
            {
                try
                {
                    token.ThrowIfCancellationRequested();
                    var result = FollowVerifiedObject(image, token, context.Discontinuous);
                    if (_templateTracker != null) { LastObjectDiagnostic = _templateTracker.Diagnostic; }
                    token.ThrowIfCancellationRequested();
                    if (!result.Found) { return result; }
                    var box = result.Bounds.Value;
                    var bounds = new RectangleF(box.X * frame.Width / image.Width, box.Y * frame.Height / image.Height,
                        box.Width * frame.Width / image.Width, box.Height * frame.Height / image.Height);
                    return new VisualTrackingResult(VisualTrackingState.Verified, result.Status, bounds,
                        new PointF(result.AimPoint.Value.X * frame.Width / image.Width,
                            result.AimPoint.Value.Y * frame.Height / image.Height));
                }
                catch { Clear(); throw; }
            }
        }

        private VisualTrackingResult FollowVerifiedObject(Mat image, CancellationToken token, bool discontinuous)
        {
            if (_templateTracker == null) { return FollowObject(image, token); }
            var learned = _templateTracker.Update(image, token, discontinuous);
            if (learned.Found && !_templateAccepted) { _templateAccepted = true; ClearForeground(); }
            if (!_templateAccepted && !_templateTracker.HasCandidate && _appearance != null)
            { _templateTracker.Dispose(); _templateTracker = null; return FollowObject(image, token); }
            return learned;
        }

        private static bool HasTexture(Mat image, OpenCvSharp.Rect box)
        {
            int minimum = 255, maximum = 0;
            for (int y = 0; y < 8; y++)
            {
                for (int x = 0; x < 8; x++)
                {
                    var pixel = image.At<Vec3b>(box.Y + box.Height * y / 8, box.X + box.Width * x / 8);
                    int brightness = (pixel.Item0 + pixel.Item1 + pixel.Item2) / 3;
                    minimum = Math.Min(minimum, brightness); maximum = Math.Max(maximum, brightness);
                }
            }
            return maximum - minimum >= 12;
        }

        private static VisualTrackingResult Lost(string status) => new VisualTrackingResult(VisualTrackingState.Lost, status);
        private static VisualTrackingResult Searching(string status) => new VisualTrackingResult(VisualTrackingState.Searching, status);

        private static double Overlap(Rectangle a, Rectangle b)
        {
            double intersection = Math.Max(0, Math.Min(a.Right, b.Right) - Math.Max(a.Left, b.Left)) *
                Math.Max(0, Math.Min(a.Bottom, b.Bottom) - Math.Max(a.Top, b.Top));
            return intersection / ((double)a.Width * a.Height + (double)b.Width * b.Height - intersection);
        }

        private void CheckDisposed()
        { if (_disposed) { throw new ObjectDisposedException(nameof(ObjectVisualTracker)); } }

        private void ResetLocalTrack()
        {
            _following = false; _flowImage?.Dispose(); _flowImage = null; _flowPoints = null;
            _recoveryBounds = null; _recoveryMatches = 0; _acceptedViews = 0;
            _heldFrames = 0; _heldConfirmations = 0; _proposedFrames = 0;
        }

        private void Clear()
        {
            _templateTracker?.Dispose(); _templateTracker = null; _templateAccepted = false;
            ClearForeground();
            _maximumImageDimension = 640; ProcessingImageSize = System.Drawing.Size.Empty;
        }

        private void ClearForeground()
        {
            ResetLocalTrack(); _reference?.Dispose(); _reference = null; _appearance?.Dispose(); _appearance = null;
            _workingReference?.Dispose(); _workingReference = null; _workingAppearance?.Dispose(); _workingAppearance = null;
            _originalView?.Dispose(); _originalView = null; _workingView?.Dispose(); _workingView = null;
            _previousView?.Dispose(); _previousView = null; _proposer?.Dispose(); _proposer = null;
            _hasObjectCentre = false; _objectMotion = PointF.Empty;
        }

        public void Dispose()
        {
            if (_disposed) { return; }
            _disposed = true;
            Clear();
        }
    }
}
