// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Threading;
using OpenCvSharp;
using OpenCvSharp.Tracking;

namespace NOMAD.MissionPlanner
{
    internal sealed partial class CsrtVisualTracker : IVisualTracker
    {
        private TrackerCSRT _tracker;
        private FrozenAppearance _appearance;
        private ObjectRecoveryReference _reference;
        private Rectangle? _recoveryBounds;
        private int _recoveryMatches;
        private System.Drawing.Size _imageSize;
        private int _maximumImageDimension = 640;
        internal System.Drawing.Size ProcessingImageSize { get; private set; }
        private bool _disposed;

        internal CsrtVisualTracker() { Cv2.SetNumThreads(2); }

        public void Initialize(Bitmap frame, RectangleF bounds, CancellationToken token,
            VisualFrameContext context = default)
        {
            CheckDisposed();
            Clear();
            token.ThrowIfCancellationRequested();
            VisualTrackingImage.ValidateFrame(frame);
            VisualTrackingImage.ValidateSelection(frame, bounds);
            // Select the pixel budget once, retaining the original 640-coordinate foreground policy.
            double baselineScale = Math.Min(1, 640.0 / Math.Max(frame.Width, frame.Height));
            int baselineWidth = (int)(bounds.Width * ((int)(frame.Width * baselineScale) / (double)frame.Width));
            int baselineHeight = (int)(bounds.Height * ((int)(frame.Height * baselineScale) / (double)frame.Height));
            _maximumImageDimension = Math.Min(baselineWidth, baselineHeight) <= 80 ? 1280 : 640;
            bool smallSelection = Math.Max(baselineWidth, baselineHeight) <= 80;
            using (var image = VisualTrackingImage.Scale(frame, _maximumImageDimension))
            {
                double scaleX = (double)image.Width / frame.Width, scaleY = (double)image.Height / frame.Height;
                var box = new OpenCvSharp.Rect((int)(bounds.X * scaleX), (int)(bounds.Y * scaleY),
                    (int)(bounds.Width * scaleX), (int)(bounds.Height * scaleY));
                if (box.Width < 8 || box.Height < 8 || !HasTexture(image, box))
                { throw new ArgumentException("Select a larger region with visible texture or edges."); }
                var bits = image.LockBits(new Rectangle(0, 0, image.Width, image.Height), ImageLockMode.ReadOnly,
                    PixelFormat.Format24bppRgb);
                try
                {
                    using (var mat = Mat.FromPixelData(image.Height, image.Width, MatType.CV_8UC3,
                        bits.Scan0, bits.Stride))
                    {
                        token.ThrowIfCancellationRequested();
                        // OpenCV's default CSRT PSR threshold is 0.035; Update returns false below it.
                        InitializeObject(mat, box, smallSelection, token);
                        token.ThrowIfCancellationRequested();
                        _imageSize = frame.Size;
                        ProcessingImageSize = image.Size;
                    }
                }
                catch { Clear(); throw; }
                finally { image.UnlockBits(bits); }
            }
        }

        public VisualTrackingResult Update(Bitmap frame, CancellationToken token, VisualFrameContext context = default)
        {
            CheckDisposed();
            if (token.IsCancellationRequested) { Clear(); }
            token.ThrowIfCancellationRequested();
            VisualTrackingImage.ValidateFrame(frame);
            if (_appearance == null) { return Lost("Select a distinctive object"); }
            if (frame.Size != _imageSize) { Clear(); return Lost("Image size changed; select the object again"); }
            if (context.Discontinuous) { ResetNative(); }
            using (var image = VisualTrackingImage.Scale(frame, _maximumImageDimension))
            {
                var bits = image.LockBits(new Rectangle(0, 0, image.Width, image.Height), ImageLockMode.ReadOnly,
                    PixelFormat.Format24bppRgb);
                try
                {
                    using (var mat = Mat.FromPixelData(image.Height, image.Width, MatType.CV_8UC3,
                        bits.Scan0, bits.Stride))
                    {
                        token.ThrowIfCancellationRequested();
                        var result = FollowObject(mat, token);
                        token.ThrowIfCancellationRequested();
                        if (!result.Found) { return result; }
                        var box = result.Bounds.Value;
                        var bounds = new RectangleF(box.X * frame.Width / image.Width, box.Y * frame.Height / image.Height,
                            box.Width * frame.Width / image.Width, box.Height * frame.Height / image.Height);
                        return new VisualTrackingResult(VisualTrackingState.Verified, result.Status, bounds,
                            new PointF(result.AimPoint.Value.X * frame.Width / image.Width,
                                result.AimPoint.Value.Y * frame.Height / image.Height));
                    }
                }
                catch { Clear(); throw; }
                finally { image.UnlockBits(bits); }
            }
        }

        private static bool HasTexture(Bitmap image, OpenCvSharp.Rect box)
        {
            int minimum = 255, maximum = 0;
            for (int y = 0; y < 8; y++)
            {
                for (int x = 0; x < 8; x++)
                {
                    var pixel = image.GetPixel(box.X + box.Width * x / 8, box.Y + box.Height * y / 8);
                    int brightness = (pixel.R + pixel.G + pixel.B) / 3;
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
        { if (_disposed) { throw new ObjectDisposedException(nameof(CsrtVisualTracker)); } }

        private void ResetNative()
        {
            _tracker?.Dispose(); _tracker = null; _recoveryBounds = null; _recoveryMatches = 0; _acceptedViews = 0;
            _heldFrames = 0; _heldConfirmations = 0;
        }

        private void Clear()
        {
            ResetNative(); _reference?.Dispose(); _reference = null; _appearance?.Dispose(); _appearance = null;
            _workingReference?.Dispose(); _workingReference = null; _workingAppearance?.Dispose(); _workingAppearance = null;
            _originalView?.Dispose(); _originalView = null; _workingView?.Dispose(); _workingView = null;
            _previousView?.Dispose(); _previousView = null;
            _hasObjectCentre = false; _objectMotion = PointF.Empty;
            _maximumImageDimension = 640; ProcessingImageSize = System.Drawing.Size.Empty;
        }

        public void Dispose()
        {
            if (_disposed) { return; }
            _disposed = true;
            Clear();
        }
    }
}
