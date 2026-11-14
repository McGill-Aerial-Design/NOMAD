// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Drawing.Imaging;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    internal static class VisualTrackingImage
    {
        internal static Mat Color(Bitmap frame, int maximumDimension = 640)
        {
            ValidateFrame(frame);
            if (maximumDimension < 1 || maximumDimension > 1280)
            { throw new ArgumentOutOfRangeException(nameof(maximumDimension)); }
            double scale = Math.Min(1, maximumDimension / (double)Math.Max(frame.Width, frame.Height));
            var size = new OpenCvSharp.Size(Math.Max(1, (int)(frame.Width * scale)),
                Math.Max(1, (int)(frame.Height * scale)));
            var result = new Mat();
            BitmapData bits = null;
            try
            {
                bool bgra = frame.PixelFormat == PixelFormat.Format32bppArgb || frame.PixelFormat == PixelFormat.Format32bppRgb;
                bits = frame.LockBits(new Rectangle(0, 0, frame.Width, frame.Height), ImageLockMode.ReadOnly,
                    bgra ? frame.PixelFormat : PixelFormat.Format24bppRgb);
                bool inverted = bits.Stride < 0;
                var start = inverted ? IntPtr.Add(bits.Scan0, bits.Stride * (frame.Height - 1)) : bits.Scan0;
                using (var source = Mat.FromPixelData(frame.Height, frame.Width, bgra ? MatType.CV_8UC4 : MatType.CV_8UC3,
                    start, Math.Abs(bits.Stride)))
                using (var resized = new Mat())
                {
                    // In-place BGRA->BGR makes OpenCV clone the source first (~1 ms per HD frame); convert across.
                    Cv2.Resize(source, bgra ? resized : result, size, interpolation: InterpolationFlags.Linear);
                    if (bgra) { Cv2.CvtColor(resized, result, ColorConversionCodes.BGRA2BGR); }
                    if (inverted) { Cv2.Flip(result, result, FlipMode.X); }
                }
                return result;
            }
            catch { result.Dispose(); throw; }
            finally { if (bits != null) { frame.UnlockBits(bits); } }
        }

        internal static Mat Gray(Bitmap frame)
        {
            using (var color = Color(frame))
            {
                var gray = new Mat();
                try
                {
                    Cv2.CvtColor(color, gray, ColorConversionCodes.BGR2GRAY);
                    return gray;
                }
                catch { gray.Dispose(); throw; }
            }
        }

        internal static void ValidateFrame(Bitmap frame)
        {
            if (frame == null) { throw new ArgumentNullException(nameof(frame)); }
            if (frame.Width < 1 || frame.Height < 1 || frame.Width > 16384 || frame.Height > 16384 ||
                (long)frame.Width * frame.Height > 64 * 1024 * 1024)
            { throw new ArgumentException("Video frame exceeds visual tracking limits."); }
        }

        internal static void ValidateSelection(Bitmap frame, RectangleF bounds)
        {
            if (!VideoDetection.Finite(bounds.X) || !VideoDetection.Finite(bounds.Y) ||
                !VideoDetection.Finite(bounds.Width) || !VideoDetection.Finite(bounds.Height) ||
                bounds.X < 0 || bounds.Y < 0 || bounds.Right > frame.Width || bounds.Bottom > frame.Height ||
                bounds.Width < 1 || bounds.Height < 1)
            { throw new ArgumentException("Select a region fully inside the current image."); }
        }
    }
}
