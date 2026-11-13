// SPDX-License-Identifier: Apache-2.0
using System;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    internal sealed class FrozenAppearance : IDisposable
    {
        private readonly Mat _reference = new Mat();
        private readonly Mat _projectionReference = new Mat();
        private readonly bool _hasTexture;
        private Mat _projectionMask, _projectionColour;

        internal FrozenAppearance(Mat image, Rect bounds, Mat foregroundMask = null)
        {
            try
            {
                using (var color = new Mat(image, bounds))
                using (var gray = new Mat())
                {
                    Cv2.CvtColor(color, gray, ColorConversionCodes.BGR2GRAY);
                    Cv2.Resize(gray, _reference, new Size(48, 48), interpolation: InterpolationFlags.Area);
                    Cv2.MeanStdDev(_reference, out Scalar _, out Scalar deviation);
                    _hasTexture = deviation.Val0 >= 1;
                    // A masked uniform shaft can use local shape/colour; constant NCC cannot establish identity.
                    if (double.IsNaN(deviation.Val0) || double.IsInfinity(deviation.Val0) ||
                        (!_hasTexture && foregroundMask == null))
                    { throw new ArgumentException("Select a region with distinctive texture or edges."); }
                }
                using (var gray = new Mat())
                {
                    Cv2.CvtColor(image, gray, ColorConversionCodes.BGR2GRAY);
                    Project(gray, new[] { new Point2f(bounds.Left, bounds.Top), new Point2f(bounds.Right, bounds.Top),
                        new Point2f(bounds.Right, bounds.Bottom), new Point2f(bounds.Left, bounds.Bottom) }, _projectionReference);
                    if (foregroundMask != null)
                    {
                        var corners = new[] { new Point2f(bounds.Left, bounds.Top), new Point2f(bounds.Right, bounds.Top),
                            new Point2f(bounds.Right, bounds.Bottom), new Point2f(bounds.Left, bounds.Bottom) };
                        _projectionMask = new Mat(); _projectionColour = new Mat();
                        Project(foregroundMask, corners, _projectionMask);
                        Cv2.Threshold(_projectionMask, _projectionMask, 128, 255, ThresholdTypes.Binary);
                        Project(image, corners, _projectionColour);
                    }
                }
            }
            catch { Dispose(); throw; }
        }

        // Correlation peaks only propose seeds inside a window; callers still verify masked identity.
        internal System.Collections.Generic.List<Point> Peaks(Mat image, Rect window, Size objectSize, int count)
        {
            var peaks = new System.Collections.Generic.List<Point>();
            if (!_hasTexture || objectSize.Width < 2 || objectSize.Height < 2) { return peaks; }
            double sx = 48.0 / objectSize.Width, sy = 48.0 / objectSize.Height;
            var scaled = new Size((int)(window.Width * sx), (int)(window.Height * sy));
            if (scaled.Width <= 48 || scaled.Height <= 48) { return peaks; }
            using (var color = new Mat(image, window))
            using (var gray = new Mat())
            using (var normalized = new Mat())
            using (var scores = new Mat())
            {
                Cv2.CvtColor(color, gray, ColorConversionCodes.BGR2GRAY);
                Cv2.Resize(gray, normalized, scaled, interpolation: InterpolationFlags.Area);
                Cv2.MatchTemplate(normalized, _reference, scores, TemplateMatchModes.CCoeffNormed);
                for (int i = 0; i < count; i++)
                {
                    Cv2.MinMaxLoc(scores, out _, out double best, out _, out Point at);
                    if (best < .2) { break; }
                    peaks.Add(new Point(window.X + (int)((at.X + 24) / sx), window.Y + (int)((at.Y + 24) / sy)));
                    Cv2.Circle(scores, at, 36, Scalar.All(-1), -1);
                }
            }
            return peaks;
        }

        internal bool MatchesProjection(Mat image, Point2f[] corners)
        {
            using (var gray = new Mat())
            using (var normalized = new Mat())
            {
                Cv2.CvtColor(image, gray, ColorConversionCodes.BGR2GRAY);
                Project(gray, corners, normalized);
                if (_projectionMask != null)
                {
                    using (var colour = new Mat())
                    {
                        Project(image, corners, colour);
                        return MatchesMasked(_projectionReference, normalized, _projectionColour, colour,
                            _projectionMask, _projectionMask, .65, new System.Drawing.PointF(24, 24));
                    }
                }
                return SameSurface(normalized, _projectionReference, .65);
            }
        }

        private static void Project(Mat gray, Point2f[] corners, Mat result)
        {
            using (var mapping = Cv2.GetPerspectiveTransform(corners,
                new[] { new Point2f(0, 0), new Point2f(47, 0), new Point2f(47, 47), new Point2f(0, 47) }))
            { Cv2.WarpPerspective(gray, result, mapping, new Size(48, 48)); }
        }

        internal static bool SameSurface(Mat candidate, Mat appearance, double minimumCorrelation)
        {
            var middle = new Rect(candidate.Width / 3, candidate.Height / 3, candidate.Width / 3, candidate.Height / 3);
            using (var scores = new Mat())
            using (var center = new Mat(candidate, middle))
            using (var reference = new Mat(appearance, middle))
            {
                Cv2.MatchTemplate(candidate, appearance, scores, TemplateMatchModes.CCoeffNormed);
                if (scores.At<float>(0) < minimumCorrelation) { return false; }
                Cv2.MeanStdDev(reference, out Scalar referenceMean, out Scalar referenceDeviation);
                if (referenceDeviation.Val0 < 2)
                {
                    // Constant templates have degenerate NCC; normalize center brightness against the whole patch.
                    Cv2.MeanStdDev(center, out Scalar centerMean, out Scalar centerDeviation);
                    Cv2.MeanStdDev(candidate, out Scalar currentMean, out Scalar currentDeviation);
                    Cv2.MeanStdDev(appearance, out Scalar originalMean, out Scalar originalDeviation);
                    double gain = currentDeviation.Val0 / Math.Max(1, originalDeviation.Val0);
                    double expected = currentMean.Val0 + (referenceMean.Val0 - originalMean.Val0) * gain;
                    return centerDeviation.Val0 <= 3 + referenceDeviation.Val0 * gain &&
                        Math.Abs(centerMean.Val0 - expected) <= Math.Max(3, currentDeviation.Val0 * .25);
                }
                Cv2.MatchTemplate(center, reference, scores, TemplateMatchModes.CCoeffNormed);
                return scores.At<float>(0) >= .5;
            }
        }

        internal static bool MatchesMasked(Mat a, Mat b, Mat aColour, Mat bColour, Mat aMask, Mat bMask,
            double minimum, System.Drawing.PointF anchor)
        {
            a.GetArray(out byte[] aPixels); b.GetArray(out byte[] bPixels);
            aMask.GetArray(out byte[] aCoverage); bMask.GetArray(out byte[] bCoverage);
            aColour.GetArray(out Vec3b[] aColours); bColour.GetArray(out Vec3b[] bColours);
            int aWidth = a.Width, bWidth = b.Width;
            if (!Correlation(new Rect(0, 0, a.Width, a.Height), minimum, 0, 0)) { return false; }
            var centre = new Rect(Math.Max(3, Math.Min(a.Width - 19, (int)anchor.X - 8)),
                Math.Max(3, Math.Min(a.Height - 19, (int)anchor.Y - 8)), 16, 16);
            // Facial features deform within the foreground; a rigid center pixel layout is not identity.
            for (int dy = -3; dy <= 3; dy++)
            for (int dx = -3; dx <= 3; dx++)
            { if (Correlation(centre, .45, dx, dy)) { return true; } }
            return false;

            bool Correlation(Rect region, double threshold, int offsetX, int offsetY)
            {
                int possible = 0, count = 0;
                double sa = 0, sb = 0, aa = 0, bb = 0, ab = 0;
                var ca = new double[3]; var cb = new double[3];
                for (int y = region.Top; y < region.Bottom; y++)
                for (int x = region.Left; x < region.Right; x++)
                {
                    int ai = y * aWidth + x, bi = (y + offsetY) * bWidth + x + offsetX;
                    if (aCoverage[ai] == 0) { continue; }
                    possible++;
                    if (bCoverage[bi] == 0) { continue; }
                    count++;
                    double av = aPixels[ai], bv = bPixels[bi];
                    sa += av; sb += bv; aa += av * av; bb += bv * bv; ab += av * bv;
                    var ac = aColours[ai]; var bc = bColours[bi];
                    for (int c = 0; c < 3; c++) { ca[c] += ac[c]; cb[c] += bc[c]; }
                }
                if (count < 8 || count < possible * .6) { return false; }
                double colourError = 0;
                for (int c = 0; c < 3; c++) { colourError += Math.Pow((ca[c] - cb[c]) / count, 2); }
                if (Math.Sqrt(colourError / 3) > 40) { return false; }
                double va = aa - sa * sa / count, vb = bb - sb * sb / count;
                if (va < count * 4 || vb < count * 4)
                { return region.Width < a.Width && va < count * 16 && vb < count * 16 && Math.Sqrt(colourError / 3) < 25; }
                return (ab - sa * sb / count) / Math.Sqrt(va * vb) >= threshold;
            }
        }

        public void Dispose()
        { _reference.Dispose(); _projectionReference.Dispose(); _projectionMask?.Dispose(); _projectionColour?.Dispose(); }
    }
}
