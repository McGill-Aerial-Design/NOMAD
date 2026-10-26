// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Threading;
using OpenCvSharp;
using OpenCvSharp.Tracking;

namespace NOMAD.MissionPlanner
{
    internal sealed class CsrtVisualTracker : IVisualTracker
    {
        private TrackerCSRT _tracker;
        private FrozenAppearance _appearance;
        private System.Drawing.Size _imageSize;
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
            using (var image = VisualTrackingImage.Scale(frame))
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
                        _tracker = TrackerCSRT.Create();
                        _tracker.Init(mat, box);
                        _appearance = new FrozenAppearance(mat, box);
                        token.ThrowIfCancellationRequested();
                        _imageSize = frame.Size;
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
            if (_tracker == null) { return Lost("Object lost; select it again"); }
            if (frame.Size != _imageSize) { Clear(); return Lost("Image size changed; select the object again"); }
            using (var image = VisualTrackingImage.Scale(frame))
            {
                var bits = image.LockBits(new Rectangle(0, 0, image.Width, image.Height), ImageLockMode.ReadOnly,
                    PixelFormat.Format24bppRgb);
                try
                {
                    using (var mat = Mat.FromPixelData(image.Height, image.Width, MatType.CV_8UC3,
                        bits.Scan0, bits.Stride))
                    {
                        token.ThrowIfCancellationRequested();
                        var box = new OpenCvSharp.Rect();
                        bool found = _tracker.Update(mat, ref box);
                        // CSRT has no native interruption API; a canceled result must never leave this worker.
                        token.ThrowIfCancellationRequested();
                        if (!found || box.Width < 1 || box.Height < 1 || box.X < 0 || box.Y < 0 ||
                            (long)box.X + box.Width > image.Width || (long)box.Y + box.Height > image.Height)
                        { Clear(); return Lost("Object lost; select it again"); }
                        if (!_appearance.Matches(mat, box))
                        { Clear(); return Lost("Object appearance changed or was occluded; select it again"); }
                        token.ThrowIfCancellationRequested();
                        var bounds = new RectangleF((float)box.X * frame.Width / image.Width,
                            (float)box.Y * frame.Height / image.Height, (float)box.Width * frame.Width / image.Width,
                            (float)box.Height * frame.Height / image.Height);
                        return new VisualTrackingResult(VisualTrackingState.Verified, "Selected object verified", bounds,
                            new PointF(bounds.X + bounds.Width / 2, bounds.Y + bounds.Height / 2));
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

        private void CheckDisposed()
        { if (_disposed) { throw new ObjectDisposedException(nameof(CsrtVisualTracker)); } }

        private void Clear()
        { _tracker?.Dispose(); _tracker = null; _appearance?.Dispose(); _appearance = null; }

        public void Dispose()
        {
            if (_disposed) { return; }
            _disposed = true;
            Clear();
        }
    }
}
