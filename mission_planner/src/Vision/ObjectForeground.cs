// SPDX-License-Identifier: Apache-2.0
using System;
using System.Linq;
using System.Threading;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    // Selection-time RGB evidence, not a semantic object detector or a source of target identity.
    internal sealed class ObjectForeground : IDisposable
    {
        internal Mat Mask, Neutralized, PatchColor, PatchGray, PatchMask;
        internal System.Drawing.Rectangle Bounds;
        internal System.Drawing.PointF Centre;
        internal Point2f[] Quad;
        internal double AngleDegrees;
        internal double[] ColourMean;
        internal int Area;
        internal bool Projected; // Previous outline moved into a proposal, not measured from this frame.

        private ObjectForeground() { }

        internal static ObjectForeground TryExtract(Mat image, System.Drawing.Rectangle selection,
            System.Drawing.PointF seed, double? previousAngle, CancellationToken token, double searchExpansion = 1.8,
            double[] expectedColour = null, bool restrictToSelection = false)
        {
            token.ThrowIfCancellationRequested();
            if (image.Empty() || image.Type() != MatType.CV_8UC3 || selection.Width < 2 || selection.Height < 2 ||
                !VideoDetection.Finite(seed.X) || !VideoDetection.Finite(seed.Y) ||
                seed.X < 0 || seed.Y < 0 || seed.X >= image.Width || seed.Y >= image.Height ||
                double.IsNaN(searchExpansion) || searchExpansion < 1.2 || searchExpansion > 4) { return null; }
            int side = (int)Math.Min(Math.Max(image.Width, image.Height),
                Math.Max(48, Math.Ceiling(Math.Max(selection.Width, selection.Height) * searchExpansion)));
            int left = Math.Max(0, Math.Min(image.Width - side, (int)Math.Floor(seed.X - side / 2.0)));
            int top = Math.Max(0, Math.Min(image.Height - side, (int)Math.Floor(seed.Y - side / 2.0)));
            var search = new Rect(left, top, Math.Min(image.Width, left + side) - left,
                Math.Min(image.Height, top + side) - top);
            var point = new Point((int)Math.Round(seed.X) - left, (int)Math.Round(seed.Y) - top);
            if (point.X < 3 || point.Y < 3 || point.X >= search.Width - 3 || point.Y >= search.Height - 3) { return null; }
            ObjectForeground result = null;
            try
            {
                using (var crop = new Mat(image, search))
                using (var classes = new Mat(crop.Size(), MatType.CV_8UC1, Scalar.All(restrictToSelection ? 0 : 2)))
                using (var background = new Mat())
                using (var foreground = new Mat())
                using (var binary = new Mat())
                using (var probable = new Mat())
                using (var labels = new Mat())
                using (var component = new Mat())
                using (var points = new Mat())
                {
                    var prior = Rect.Intersect(new Rect(selection.X - left, selection.Y - top,
                        selection.Width, selection.Height), new Rect(0, 0, crop.Width, crop.Height));
                    if (prior.Width < 2 || prior.Height < 2) { return null; }
                    Cv2.Rectangle(classes, prior, Scalar.All(3), -1);
                    if (expectedColour != null)
                    {
                        binary.Create(crop.Size(), MatType.CV_8UC1); binary.SetTo(Scalar.All(0));
                        // Uniform shafts have too little interior texture for a rectangle-trained colour mixture.
                        for (int y = 2; y < crop.Height - 2; y++)
                        for (int x = 2; x < crop.Width - 2; x++)
                        {
                            var pixel = crop.At<Vec3b>(y, x);
                            double distance = 0;
                            for (int c = 0; c < 3; c++) { distance += Math.Pow(pixel[c] - expectedColour[c], 2); }
                            if (distance < 3 * 35 * 35) { binary.Set(y, x, (byte)255); }
                        }
                        using (var kernel = Cv2.GetStructuringElement(MorphShapes.Rect, new Size(3, 3)))
                        { Cv2.MorphologyEx(binary, binary, MorphTypes.Close, kernel); }
                    }
                    else
                    {
                        Cv2.Rectangle(classes, new Rect(0, 0, crop.Width, crop.Height), Scalar.All(0), 2);
                        int radius = Math.Max(1, Math.Min(2, Math.Min(selection.Width, selection.Height) / 40));
                        Cv2.Circle(classes, point, radius, Scalar.All(1), -1);
                        // GrabCut initializes colour clusters randomly; fix its declared seed rather than test-order state.
                        ulong randomState = Cv2.GetTheRNG().State;
                        Cv2.SetTheRNG(173);
                        try { Cv2.GrabCut(crop, classes, default, background, foreground, 3, GrabCutModes.InitWithMask); }
                        finally { Cv2.SetTheRNG(randomState); }
                        Cv2.InRange(classes, Scalar.All(1), Scalar.All(1), binary);
                        Cv2.InRange(classes, Scalar.All(3), Scalar.All(3), probable);
                        Cv2.BitwiseOr(binary, probable, binary);
                    }
                    token.ThrowIfCancellationRequested();
                    Cv2.ConnectedComponents(binary, labels, PixelConnectivity.Connectivity8, MatType.CV_32S);
                    int selected = labels.At<int>(point.Y, point.X);
                    if (selected == 0) { return null; }
                    Cv2.InRange(labels, Scalar.All(selected), Scalar.All(selected), component);
                    int area = Cv2.CountNonZero(component);
                    var box = Cv2.BoundingRect(component);
                    if (area < Math.Max(12, selection.Width * (double)selection.Height * .005) ||
                        area > crop.Width * (double)crop.Height * .65 || box.Width < 2 || box.Height < 2 ||
                        box.Left <= 2 || box.Top <= 2 || box.Right >= crop.Width - 2 || box.Bottom >= crop.Height - 2)
                    { return null; }
                    result = new ObjectForeground { Area = area, Mask = new Mat(image.Size(), MatType.CV_8UC1, Scalar.All(0)) };
                    using (var destination = new Mat(result.Mask, search)) { component.CopyTo(destination); }
                    result.Bounds = new System.Drawing.Rectangle(box.X + left, box.Y + top, box.Width, box.Height);
                    var moments = Cv2.Moments(component, true);
                    result.Centre = new System.Drawing.PointF((float)(moments.M10 / moments.M00 + left),
                        (float)(moments.M01 / moments.M00 + top));
                    Cv2.FindNonZero(component, points); points.GetArray(out Point[] outline);
                    var rotated = Cv2.MinAreaRect(outline.Select(p => new Point2f(p.X + left, p.Y + top)));
                    result.SetQuad(rotated, previousAngle);
                    var mean = Cv2.Mean(image, result.Mask);
                    result.ColourMean = new[] { mean.Val0, mean.Val1, mean.Val2 };
                    result.Neutralized = new Mat(image.Size(), image.Type(), mean);
                    image.CopyTo(result.Neutralized, result.Mask);
                    result.PatchColor = new Mat(); result.PatchMask = new Mat(); result.PatchGray = new Mat();
                    using (var transform = Cv2.GetPerspectiveTransform(result.Quad,
                        new[] { new Point2f(0, 0), new Point2f(47, 0), new Point2f(47, 47), new Point2f(0, 47) }))
                    {
                        Cv2.WarpPerspective(result.Neutralized, result.PatchColor, transform, new Size(48, 48));
                        Cv2.WarpPerspective(result.Mask, result.PatchMask, transform, new Size(48, 48),
                            InterpolationFlags.Nearest);
                    }
                    Cv2.CvtColor(result.PatchColor, result.PatchGray, ColorConversionCodes.BGR2GRAY);
                    token.ThrowIfCancellationRequested();
                    return result;
                }
            }
            catch (OpenCVException) { result?.Dispose(); return null; }
            catch { result?.Dispose(); throw; }
        }

        private void SetQuad(RotatedRect rectangle, double? previousAngle)
        {
            double major = Math.Max(rectangle.Size.Width, rectangle.Size.Height);
            double minor = Math.Min(rectangle.Size.Width, rectangle.Size.Height);
            double angle = rectangle.Angle + (rectangle.Size.Width >= rectangle.Size.Height ? 0 : 90);
            if (major < minor * 1.15) { angle = previousAngle ?? 90; }
            if (previousAngle.HasValue)
            {
                while (angle - previousAngle.Value > 90) { angle -= 180; }
                while (angle - previousAngle.Value < -90) { angle += 180; }
            }
            AngleDegrees = angle;
            double radians = angle * Math.PI / 180, x = Math.Cos(radians), y = Math.Sin(radians);
            var centre = new System.Drawing.PointF(rectangle.Center.X, rectangle.Center.Y);
            Quad = new[] { Corner(-1, -1), Corner(1, -1), Corner(1, 1), Corner(-1, 1) };
            Point2f Corner(int horizontal, int vertical) => new Point2f(
                (float)(centre.X + horizontal * y * minor / 2 + vertical * x * major / 2),
                (float)(centre.Y - horizontal * x * minor / 2 + vertical * y * major / 2));
        }

        internal Mat CreateNativeMask(System.Drawing.Rectangle bounds)
        {
            var result = new Mat();
            try
            {
                using (var crop = new Mat(Mask, new Rect(bounds.X, bounds.Y, bounds.Width, bounds.Height)))
                { crop.ConvertTo(result, MatType.CV_32FC1, 1.0 / 255); }
                return result;
            }
            catch { result.Dispose(); throw; }
        }

        internal static ObjectForeground ProjectPrevious(Mat image, ObjectForeground previous,
            System.Drawing.Rectangle current, CancellationToken token)
        {
            if (current.Width < 2 || current.Height < 2 || current.Left < 0 || current.Top < 0 ||
                current.Right > image.Width || current.Bottom > image.Height) { return null; }
            var result = new ObjectForeground();
            try
            {
                double sx = current.Width / (double)previous.Bounds.Width;
                double sy = current.Height / (double)previous.Bounds.Height;
                using (var mapping = Mat.FromArray(new double[,] { { sx, 0, current.X - sx * previous.Bounds.X },
                    { 0, sy, current.Y - sy * previous.Bounds.Y } }))
                {
                    result.Mask = new Mat();
                    Cv2.WarpAffine(previous.Mask, result.Mask, mapping, image.Size(), InterpolationFlags.Nearest);
                    result.Quad = previous.Quad.Select(p => new Point2f((float)(current.X + (p.X - previous.Bounds.X) * sx),
                        (float)(current.Y + (p.Y - previous.Bounds.Y) * sy))).ToArray();
                }
                result.Bounds = current; result.AngleDegrees = previous.AngleDegrees; result.Projected = true;
                result.Area = Cv2.CountNonZero(result.Mask);
                var moments = Cv2.Moments(result.Mask, true);
                result.Centre = new System.Drawing.PointF((float)(moments.M10 / moments.M00), (float)(moments.M01 / moments.M00));
                var mean = Cv2.Mean(image, result.Mask);
                result.ColourMean = new[] { mean.Val0, mean.Val1, mean.Val2 };
                result.Neutralized = new Mat(image.Size(), image.Type(), mean); image.CopyTo(result.Neutralized, result.Mask);
                result.PatchColor = new Mat(); result.PatchMask = new Mat(); result.PatchGray = new Mat();
                using (var mapping = Cv2.GetPerspectiveTransform(result.Quad,
                    new[] { new Point2f(0, 0), new Point2f(47, 0), new Point2f(47, 47), new Point2f(0, 47) }))
                {
                    Cv2.WarpPerspective(result.Neutralized, result.PatchColor, mapping, new Size(48, 48));
                    Cv2.WarpPerspective(result.Mask, result.PatchMask, mapping, new Size(48, 48), InterpolationFlags.Nearest);
                }
                Cv2.CvtColor(result.PatchColor, result.PatchGray, ColorConversionCodes.BGR2GRAY);
                token.ThrowIfCancellationRequested(); return result;
            }
            catch { result.Dispose(); throw; }
        }

        internal ObjectForeground Clone()
        {
            var result = new ObjectForeground { Bounds = Bounds, Centre = Centre, AngleDegrees = AngleDegrees, Area = Area,
                Projected = Projected,
                Quad = (Point2f[])Quad.Clone(), ColourMean = (double[])ColourMean.Clone() };
            try
            {
                result.Mask = Mask.Clone(); result.Neutralized = Neutralized.Clone();
                result.PatchColor = PatchColor.Clone(); result.PatchGray = PatchGray.Clone(); result.PatchMask = PatchMask.Clone();
                return result;
            }
            catch { result.Dispose(); throw; }
        }

        public void Dispose()
        { Mask?.Dispose(); Neutralized?.Dispose(); PatchColor?.Dispose(); PatchGray?.Dispose(); PatchMask?.Dispose(); }
    }
}
