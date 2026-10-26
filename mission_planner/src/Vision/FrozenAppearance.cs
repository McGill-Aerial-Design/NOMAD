// SPDX-License-Identifier: Apache-2.0
using System;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    internal sealed class FrozenAppearance : IDisposable
    {
        private readonly Mat _reference = new Mat();

        internal FrozenAppearance(Mat image, Rect bounds)
        {
            using (var color = new Mat(image, bounds))
            using (var gray = new Mat())
            {
                Cv2.CvtColor(color, gray, ColorConversionCodes.BGR2GRAY);
                Cv2.Resize(gray, _reference, new Size(48, 48), interpolation: InterpolationFlags.Area);
                Cv2.MeanStdDev(_reference, out Scalar _, out Scalar deviation);
                // Constant templates otherwise produce misleading normalized-correlation scores.
                if (double.IsNaN(deviation.Val0) || double.IsInfinity(deviation.Val0) || deviation.Val0 < 1)
                {
                    _reference.Dispose();
                    throw new ArgumentException("Select a region with distinctive texture or edges.");
                }
            }
        }

        internal bool Matches(Mat image, Rect bounds)
        {
            var expanded = new Rect(Math.Max(0, bounds.X - bounds.Width / 5),
                Math.Max(0, bounds.Y - bounds.Height / 5), 0, 0);
            expanded.Width = Math.Min(image.Width, bounds.Right + bounds.Width / 5) - expanded.X;
            expanded.Height = Math.Min(image.Height, bounds.Bottom + bounds.Height / 5) - expanded.Y;
            using (var color = new Mat(image, expanded))
            using (var gray = new Mat())
            using (var normalized = new Mat())
            using (var scores = new Mat())
            {
                Cv2.CvtColor(color, gray, ColorConversionCodes.BGR2GRAY);
                for (int step = -4; step <= 4; step++)
                {
                    double scale = 1 + step * .05;
                    int width = Math.Max(48, (int)(48 * expanded.Width / (bounds.Width * scale)));
                    int height = Math.Max(48, (int)(48 * expanded.Height / (bounds.Height * scale)));
                    Cv2.Resize(gray, normalized, new Size(width, height), interpolation: InterpolationFlags.Area);
                    Cv2.MatchTemplate(normalized, _reference, scores, TemplateMatchModes.CCoeffNormed);
                    Cv2.MinMaxLoc(scores, out _, out double maximum);
                    if (maximum >= .35) { return true; }
                }
            }
            return false;
        }

        public void Dispose() { _reference.Dispose(); }
    }
}
