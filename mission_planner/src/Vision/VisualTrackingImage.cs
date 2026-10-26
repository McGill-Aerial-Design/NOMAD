// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Drawing.Imaging;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    internal static class VisualTrackingImage
    {
        internal static Bitmap Scale(Bitmap frame)
        {
            double scale = Math.Min(1, 640.0 / Math.Max(frame.Width, frame.Height));
            var result = new Bitmap(Math.Max(1, (int)(frame.Width * scale)), Math.Max(1, (int)(frame.Height * scale)),
                PixelFormat.Format24bppRgb);
            using (var graphics = Graphics.FromImage(result))
            { graphics.DrawImage(frame, new Rectangle(0, 0, result.Width, result.Height)); }
            return result;
        }

        internal static Mat Gray(Bitmap frame)
        {
            using (var image = Scale(frame))
            {
                var bits = image.LockBits(new Rectangle(0, 0, image.Width, image.Height), ImageLockMode.ReadOnly,
                    PixelFormat.Format24bppRgb);
                try
                {
                    using (var color = Mat.FromPixelData(image.Height, image.Width, MatType.CV_8UC3,
                        bits.Scan0, bits.Stride))
                    {
                        var gray = new Mat();
                        Cv2.CvtColor(color, gray, ColorConversionCodes.BGR2GRAY);
                        return gray;
                    }
                }
                finally { image.UnlockBits(bits); }
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
