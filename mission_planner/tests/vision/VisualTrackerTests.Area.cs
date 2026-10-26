// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.Threading;
using NOMAD.MissionPlanner;
using OpenCvSharp;

internal static partial class VisualTrackerTests
{
    private static readonly RectangleF AreaSelection = new RectangleF(220, 170, 180, 140);

    private static void AreaTracking()
    {
        var elapsed = Stopwatch.StartNew();
        using (var reference = GroundTexture())
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None, VisualTrackingMode.Area))
        {
            Check(tracker.Update(reference, CancellationToken.None).State == VisualTrackingState.Lost,
                "An uninitialized area tracker must have no reference.");
            using (var flat = new Bitmap(640, 480))
            {
                Throws<ArgumentException>(() => tracker.Initialize(flat, AreaSelection, CancellationToken.None),
                    "A plain wall/featureless ground selection cannot establish area identity.");
            }
            Throws<ArgumentException>(() => tracker.Initialize(reference, new RectangleF(10, 10, 8, 8),
                CancellationToken.None), "A tiny area cannot establish a local plane.");
            Throws<ArgumentException>(() => tracker.Initialize(reference, new RectangleF(float.NaN, 10, 100, 100),
                CancellationToken.None), "A nonfinite area selection must fail.");
            Throws<ArgumentException>(() => tracker.Initialize(reference, new RectangleF(600, 10, 100, 100),
                CancellationToken.None), "An out-of-image area selection must fail.");
            tracker.Initialize(reference, AreaSelection, CancellationToken.None);
            long total = 0, maximum = 0;
            for (int index = 1; index <= 6; index++)
            {
                var transform = new double[,] { { 1.3, .12, index * 3 }, { .04, 1.2, index * -2 },
                    { .0006, -.0003, 1 } };
                using (var projected = Project(reference, transform))
                {
                    var updateTime = Stopwatch.StartNew();
                    var result = tracker.Update(projected, CancellationToken.None);
                    maximum = Math.Max(maximum, updateTime.ElapsedMilliseconds); total += updateTime.ElapsedMilliseconds;
                    Check(updateTime.ElapsedMilliseconds < 1000, "Area matching must complete within the fixture deadline.");
                    Check(result.Found == (index >= 3), "Area needs three independent matched frames before verification: " +
                        index + ", " + result.Status);
                    if (!result.Found) { continue; }
                    var truth = ProjectPoint(Center(AreaSelection), transform);
                    Check(PointDistance(result.AimPoint.Value, truth) < 3,
                        "Area aim must be the independently projected canonical point: " + result.AimPoint + " / " + truth);
                    Check(PointDistance(Center(result.Bounds.Value), truth) > 4,
                        "Perspective fixture must distinguish the target point from the bounding-box center.");
                }
            }
            Console.WriteLine("Area perspective: 6 updates, mean " + total / 6.0 + " ms, max " + maximum + " ms.");
            RejectWrongPrior(tracker, reference);
            RejectOcclusionAndRecover(tracker, reference);
            RejectForegroundPlane(tracker, reference);
            using (var resized = new Bitmap(800, 600))
            { Check(tracker.Update(resized, CancellationToken.None).State == VisualTrackingState.Lost,
                "A resolution change retires the area reference."); }
            Check(tracker.Update(reference, CancellationToken.None).State == VisualTrackingState.Lost,
                "A retired reference cannot recover at a later image size.");
            tracker.Initialize(reference, AreaSelection, CancellationToken.None);
            using (var canceled = new CancellationTokenSource())
            {
                canceled.Cancel();
                Throws<OperationCanceledException>(() => tracker.Update(reference, canceled.Token),
                    "Canceled area work must publish no result.");
                Check(tracker.Update(reference, CancellationToken.None).State == VisualTrackingState.Lost,
                    "Cancellation retires the area reference.");
                Throws<OperationCanceledException>(() => tracker.Initialize(reference, AreaSelection, canceled.Token),
                    "Canceled area initialization must not establish a reference.");
            }
            tracker.Dispose();
            Throws<ObjectDisposedException>(() => tracker.Update(reference, CancellationToken.None),
                "Disposed area tracking refuses frames.");
        }
        RejectDuplicateAreas();
        RejectRepeatingTexture();
        Console.WriteLine("Area reference, perspective point, occlusion, outside/return, ambiguity and parallax: " +
            elapsed.ElapsedMilliseconds + " ms.");
    }

    private static void RejectWrongPrior(IVisualTracker tracker, Bitmap reference)
    {
        long maximumMilliseconds = 0;
        int maximumFits = 0;
        for (int index = 0; index < 12; index++)
        {
            var wrong = new VisualFrameContext(index % 2 == 0 ? new PointF(-1000, -1000) : new PointF(70, 450));
            using (var current = Project(reference, new double[,] { { 1, 0, index }, { 0, 1, index }, { 0, 0, 1 } }))
            {
                var elapsed = Stopwatch.StartNew();
                var result = tracker.Update(current, CancellationToken.None, wrong);
                Check(result.Found, "A persistent wrong prior must not interrupt unique global verification: " + result.Status);
                int fits = ((AreaVisualTracker)tracker).LastHomographyFits;
                Check(fits <= 3 && elapsed.ElapsedMilliseconds < 1000,
                    "Wrong-prior fallback exceeded the three-fit/fixture-time bound");
                maximumMilliseconds = Math.Max(maximumMilliseconds, elapsed.ElapsedMilliseconds);
                maximumFits = Math.Max(maximumFits, fits);
            }
        }
        Console.WriteLine("Persistent wrong prior: 12 continuously verified frames; max " + maximumFits +
            "/3 homography fits and " + maximumMilliseconds + " ms.");
    }

    private static void RejectRepeatingTexture()
    {
        using (var repeated = new Bitmap(640, 480))
        using (var graphics = Graphics.FromImage(repeated))
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None, VisualTrackingMode.Area))
        {
            graphics.Clear(Color.Gray);
            for (int y = 0; y < 480; y += 24)
            {
                for (int x = 0; x < 640; x += 24)
                {
                    graphics.FillRectangle(Brushes.DarkGray, x, y, 12, 12);
                    graphics.FillEllipse(Brushes.Black, x + 5, y + 5, 10, 14);
                }
            }
            try { tracker.Initialize(repeated, AreaSelection, CancellationToken.None); }
            catch (ArgumentException) { Check(true, "Repetitive texture rejected at initialization."); return; }
            for (int index = 0; index < 3; index++)
            { Check(!tracker.Update(repeated, CancellationToken.None).Found,
                "An ambiguous repeating wall/grass-like texture cannot verify a unique selected ground point."); }
        }
    }

    private static void RejectOcclusionAndRecover(IVisualTracker tracker, Bitmap reference)
    {
        using (var occluded = new Bitmap(reference))
        using (var graphics = Graphics.FromImage(occluded))
        {
            graphics.FillRectangle(Brushes.DarkSlateGray, AreaSelection);
            var result = tracker.Update(occluded, CancellationToken.None);
            Check(!result.Found && !result.Bounds.HasValue && !result.AimPoint.HasValue,
                "Even matching surrounding context must not authorize an occluded target.");
        }
        using (var blank = Project(reference, new double[,] { { 1, 0, 700 }, { 0, 1, 0 }, { 0, 0, 1 } }))
        {
            var result = tracker.Update(blank, CancellationToken.None, new VisualFrameContext(new PointF(850, 240)));
            Check(result.State == VisualTrackingState.Outside && !result.Found && !result.AimPoint.HasValue,
                "An out-of-image coarse hint may report Outside but cannot establish target identity.");
            Check(tracker.Update(blank, CancellationToken.None, new VisualFrameContext(new PointF(300, 240))).State ==
                VisualTrackingState.Searching, "An in-image GPS hint must not verify missing visual evidence.");
        }
        for (int index = 1; index <= 3; index++)
        {
            var transform = new double[,] { { 1, 0, 18 + index }, { 0, 1, 10 + index }, { 0, 0, 1 } };
            using (var returned = Project(reference, transform))
            {
                var result = tracker.Update(returned, CancellationToken.None);
                Check(result.Found == (index == 3), "Retained area must require three new matches after leaving view: " +
                    result.Status);
            }
        }
    }

    private static void RejectForegroundPlane(IVisualTracker tracker, Bitmap reference)
    {
        using (var foreground = GroundTexture(313))
        using (var mixed = new Bitmap(foreground))
        using (var graphics = Graphics.FromImage(mixed))
        {
            // Copy the original broad context onto another plane, but cover the selected ground itself.
            graphics.DrawImage(reference, new Rectangle(35, 0, 605, 480), new Rectangle(0, 0, 605, 480), GraphicsUnit.Pixel);
            graphics.DrawImage(foreground, AreaSelection, AreaSelection, GraphicsUnit.Pixel);
            var result = tracker.Update(mixed, CancellationToken.None);
            Check(!result.Found && !result.AimPoint.HasValue,
                "A dominant matching foreground/context plane cannot replace selected-ground support.");
        }
    }

    private static void RejectDuplicateAreas()
    {
        var selection = new RectangleF(270, 210, 100, 80);
        var patch = new Rectangle(170, 130, 300, 240);
        using (var reference = GroundTexture())
        using (var duplicated = new Bitmap(640, 480))
        using (var graphics = Graphics.FromImage(duplicated))
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None, VisualTrackingMode.Area))
        {
            graphics.Clear(Color.DimGray);
            graphics.DrawImage(reference, new Rectangle(0, 100, 300, 240), patch, GraphicsUnit.Pixel);
            graphics.DrawImage(reference, new Rectangle(340, 100, 300, 240), patch, GraphicsUnit.Pixel);
            tracker.Initialize(reference, selection, CancellationToken.None);
            using (var onePlacement = new Bitmap(duplicated))
            using (var oneGraphics = Graphics.FromImage(onePlacement))
            {
                oneGraphics.FillRectangle(Brushes.DimGray, 340, 100, 300, 240);
                for (int index = 0; index < 3; index++)
                {
                    var result = tracker.Update(onePlacement, CancellationToken.None);
                    Check(result.Found == (index == 2), "One repeated-patch placement must independently qualify: " + result.Status);
                }
                oneGraphics.DrawImage(reference, new Rectangle(340, 100, 300, 240), patch, GraphicsUnit.Pixel);
                oneGraphics.FillRectangle(Brushes.DimGray, 0, 100, 300, 240);
                tracker.Initialize(reference, selection, CancellationToken.None);
                for (int index = 0; index < 3; index++)
                {
                    var result = tracker.Update(onePlacement, CancellationToken.None);
                    Check(result.Found == (index == 2), "The second repeated-patch placement must independently qualify: " + result.Status);
                }
            }
            for (int index = 0; index < 4; index++)
            {
                var result = tracker.Update(duplicated, CancellationToken.None,
                    new VisualFrameContext(index % 2 == 0 ? new PointF(150, 220) : new PointF(850, 220)));
                Check(result.State == VisualTrackingState.Searching && !result.Found && !result.AimPoint.HasValue,
                    "Two repeated placements must stay ambiguous even with a hint on one copy.");
                Check(result.Status.StartsWith("Ambiguous", StringComparison.Ordinal),
                    "The duplicate fixture must exercise the second-valid-placement rejection: " + result.Status);
            }
            graphics.FillRectangle(Brushes.DimGray, 340, 100, 300, 240);
            for (int index = 0; index < 3; index++)
            { Check(tracker.Update(duplicated, CancellationToken.None).Found == (index == 2),
                "Recovering from ambiguity must require three fresh unambiguous matches."); }
        }
    }

    private static void RejectObjectOccluder()
    {
        using (var texture = Texture())
        using (var first = Frame(texture, 0))
        using (var different = GroundTexture(712))
        using (var occluded = new Bitmap(first))
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            tracker.Initialize(first, Truth(0), CancellationToken.None);
            using (var graphics = Graphics.FromImage(occluded))
            { graphics.DrawImage(different, Truth(0), new Rectangle(0, 0, 144, 144), GraphicsUnit.Pixel); }
            Check(!tracker.Update(occluded, CancellationToken.None).Found,
                "A different textured foreground object cannot take over the selected object's frozen appearance.");
            Check(!tracker.Update(first, CancellationToken.None).Found,
                "An appearance-rejected object tracker must be discarded without automatic reacquisition.");
        }
    }

    private static Bitmap GroundTexture(int seed = 173)
    {
        var bitmap = new Bitmap(640, 480, PixelFormat.Format24bppRgb);
        var random = new Random(seed);
        using (var graphics = Graphics.FromImage(bitmap))
        {
            graphics.Clear(Color.FromArgb(90, 105, 80));
            for (int index = 0; index < 2200; index++)
            {
                int x = random.Next(640), y = random.Next(480), size = random.Next(3, 13);
                using (var brush = new SolidBrush(Color.FromArgb(random.Next(256), random.Next(256), random.Next(256))))
                { graphics.FillRectangle(brush, x, y, size, random.Next(3, 13)); }
            }
        }
        return bitmap;
    }

    private static Bitmap Project(Bitmap source, double[,] transform)
    {
        using (var input = VisualTrackingImage.Gray(source))
        using (var homography = Mat.FromArray(transform))
        using (var projected = new Mat())
        {
            Cv2.WarpPerspective(input, projected, homography, new OpenCvSharp.Size(640, 480));
            var result = new Bitmap(640, 480, PixelFormat.Format24bppRgb);
            for (int y = 0; y < 480; y++)
            {
                for (int x = 0; x < 640; x++)
                { int pixel = projected.At<byte>(y, x); result.SetPixel(x, y, Color.FromArgb(pixel, pixel, pixel)); }
            }
            return result;
        }
    }

    private static PointF ProjectPoint(PointF point, double[,] transform)
    {
        double denominator = transform[2, 0] * point.X + transform[2, 1] * point.Y + transform[2, 2];
        return new PointF((float)((transform[0, 0] * point.X + transform[0, 1] * point.Y + transform[0, 2]) / denominator),
            (float)((transform[1, 0] * point.X + transform[1, 1] * point.Y + transform[1, 2]) / denominator));
    }

    private static PointF Center(RectangleF rectangle) =>
        new PointF(rectangle.X + rectangle.Width / 2, rectangle.Y + rectangle.Height / 2);
    private static double PointDistance(PointF left, PointF right) =>
        Math.Sqrt((left.X - right.X) * (left.X - right.X) + (left.Y - right.Y) * (left.Y - right.Y));
}
