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
        private System.Drawing.Size _imageSize;
        private bool _disposed;

        internal CsrtVisualTracker() { Cv2.SetNumThreads(2); }

        public void Initialize(Bitmap frame, RectangleF bounds, CancellationToken token)
        {
            CheckDisposed();
            Clear();
            token.ThrowIfCancellationRequested();
            ValidateFrame(frame);
            if (!VideoDetection.Finite(bounds.X) || !VideoDetection.Finite(bounds.Y) ||
                !VideoDetection.Finite(bounds.Width) || !VideoDetection.Finite(bounds.Height) ||
                bounds.X < 0 || bounds.Y < 0 || bounds.Right > frame.Width || bounds.Bottom > frame.Height ||
                bounds.Width < 1 || bounds.Height < 1)
            { throw new ArgumentException("Select a region fully inside the current image."); }
            using (var image = Scale(frame))
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
                        token.ThrowIfCancellationRequested();
                        _imageSize = frame.Size;
                    }
                }
                catch { Clear(); throw; }
                finally { image.UnlockBits(bits); }
            }
        }

        public bool TryUpdate(Bitmap frame, CancellationToken token, out RectangleF bounds)
        {
            CheckDisposed();
            bounds = RectangleF.Empty;
            if (token.IsCancellationRequested) { Clear(); }
            token.ThrowIfCancellationRequested();
            ValidateFrame(frame);
            if (_tracker == null) { return false; }
            if (frame.Size != _imageSize) { Clear(); return false; }
            using (var image = Scale(frame))
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
                        { Clear(); return false; }
                        bounds = new RectangleF((float)box.X * frame.Width / image.Width,
                            (float)box.Y * frame.Height / image.Height, (float)box.Width * frame.Width / image.Width,
                            (float)box.Height * frame.Height / image.Height);
                        return true;
                    }
                }
                catch { Clear(); throw; }
                finally { image.UnlockBits(bits); }
            }
        }

        private static Bitmap Scale(Bitmap frame)
        {
            double scale = Math.Min(1, 640.0 / Math.Max(frame.Width, frame.Height));
            var result = new Bitmap(Math.Max(1, (int)(frame.Width * scale)), Math.Max(1, (int)(frame.Height * scale)),
                PixelFormat.Format24bppRgb);
            using (var graphics = Graphics.FromImage(result))
            { graphics.DrawImage(frame, new Rectangle(0, 0, result.Width, result.Height)); }
            return result;
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

        private static void ValidateFrame(Bitmap frame)
        {
            if (frame == null) { throw new ArgumentNullException(nameof(frame)); }
            if (frame.Width < 1 || frame.Height < 1 || frame.Width > 16384 || frame.Height > 16384 ||
                (long)frame.Width * frame.Height > 64 * 1024 * 1024)
            { throw new ArgumentException("Video frame exceeds visual tracking limits."); }
        }

        private void CheckDisposed()
        { if (_disposed) { throw new ObjectDisposedException(nameof(CsrtVisualTracker)); } }

        private void Clear() { _tracker?.Dispose(); _tracker = null; }

        public void Dispose()
        {
            if (_disposed) { return; }
            _disposed = true;
            Clear();
        }
    }
}
