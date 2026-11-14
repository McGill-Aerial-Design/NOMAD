// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using NOMAD.MissionPlanner;

internal static partial class VisualTrackerTests
{
    private static int _checks;

    internal static void Run()
    {
        VisionRuntime.EnsureOpenCvLoaded(CancellationToken.None);
        TrackingImagePixels();
        FeatureMatchingParity();
        ObjectBackgroundAndTurning();
        using (var tile = Texture())
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            Check(!Process.GetCurrentProcess().Modules.Cast<ProcessModule>().Any(module =>
                module.ModuleName.Equals("DirectML.dll", StringComparison.OrdinalIgnoreCase)),
                "CPU Object tracking must not load a GPU execution provider.");
            using (var first = Frame(tile, 0))
            {
                Check(!tracker.Update(first, CancellationToken.None).Found, "Uninitialized tracker has no target.");
                Throws<ArgumentException>(() => tracker.Initialize(first, new RectangleF(-1, 100, 100, 100),
                    CancellationToken.None), "Out-of-frame selection must fail.");
                Throws<ArgumentException>(() => tracker.Initialize(first, new RectangleF(20, 20, 2, 2),
                    CancellationToken.None), "Tiny selection must fail.");
                Throws<ArgumentException>(() => tracker.Initialize(first, new RectangleF(float.NaN, 20, 100, 100),
                    CancellationToken.None), "Nonfinite selection must fail.");
                Throws<ArgumentException>(() => tracker.Initialize(first, new RectangleF(20, 20, 100, 100),
                    CancellationToken.None), "A flat region must fail.");
                tracker.Initialize(first, Truth(0), CancellationToken.None);
            }
            long maximum = 0, total = 0;
            for (int index = 1; index <= 10; index++)
            {
                using (var frame = Frame(tile, index))
                {
                    var elapsed = Stopwatch.StartNew();
                    var result = tracker.Update(frame, CancellationToken.None);
                    Check(result.Found,
                        "Object tracker must follow the moving textured region without detections: frame " + index + ", " + result.Status);
                    long milliseconds = elapsed.ElapsedMilliseconds;
                    maximum = Math.Max(maximum, milliseconds); total += milliseconds;
                    Check(Overlap(result.Bounds.Value, Truth(index)) > .45,
                        "Object tracker region must overlap the independently rendered moving/scaling target.");
                }
            }
            using (var blank = Frame(null, 0))
            using (var targetReturns = Frame(tile, 10))
            {
                var lost = tracker.Update(blank, CancellationToken.None);
                Check(!lost.Found && !lost.Bounds.HasValue && !lost.AimPoint.HasValue,
                    "Object appearance loss must return no target.");
                for (int index = 1; index <= 3; index++)
                {
                    var recovered = tracker.Update(targetReturns, CancellationToken.None);
                    Check(recovered.Found == (index == 3),
                        "A returning Object needs three matches, then must recover without reselection: " + recovered.Status);
                    if (recovered.Found)
                    { Check(Overlap(recovered.Bounds.Value, Truth(10)) > .65, "Recovered Object matched the wrong region"); }
                }
                tracker.Initialize(targetReturns, Truth(10), CancellationToken.None);
                using (var resized = new Bitmap(800, 600))
                { Check(!tracker.Update(resized, CancellationToken.None).Found, "Image-size change must retire tracking."); }
                tracker.Initialize(targetReturns, Truth(10), CancellationToken.None);
                using (var canceled = new CancellationTokenSource())
                {
                    canceled.Cancel();
                    Throws<OperationCanceledException>(() => tracker.Update(targetReturns, canceled.Token),
                        "Canceled tracking must publish no result.");
                    Check(!tracker.Update(targetReturns, CancellationToken.None).Found,
                        "Cancellation must retire the visual target.");
                }
            }
            Console.WriteLine("Object tracker texture/scale: 10 updates, mean " + total / 10.0 + " ms, max " + maximum +
                " ms; target loss confirmed.");
        }
        using (var disposed = VisualTrackerFactory.Create(CancellationToken.None))
        {
            disposed.Dispose();
            using (var frame = Frame(null, 0))
            {
                Throws<ObjectDisposedException>(() => disposed.Update(frame, CancellationToken.None),
                    "Disposed tracker must refuse frames.");
            }
        }
        RejectObjectOccluder();
        RejectObjectCenterOccluder();
        RejectDuplicateObjects();
        RejectFlatAppearanceReference();
        AreaTracking();
        PanoramaReturn();
        CpuRenderedDeer();
        FastMotion();
        BenchmarkPerson();
        GpuObjectTracker(); // Last: it loads the ONNX runtime the checks above must run without.
        Console.WriteLine("PASS: " + _checks + " visual tracker checks.");
    }

    private static void RejectFlatAppearanceReference()
    {
        using (var color = new OpenCvSharp.Mat(64, 64, OpenCvSharp.MatType.CV_8UC3))
        using (var gray = new OpenCvSharp.Mat())
        {
            for (int y = 0; y < 64; y++)
            {
                for (int x = 0; x < 64; x++)
                { color.Set(y, x, (x + y) % 2 == 0 ? new OpenCvSharp.Vec3b(0, 0, 84) :
                    new OpenCvSharp.Vec3b(0, 43, 0)); }
            }
            OpenCvSharp.Cv2.CvtColor(color, gray, OpenCvSharp.ColorConversionCodes.BGR2GRAY);
            OpenCvSharp.Cv2.MeanStdDev(gray, out OpenCvSharp.Scalar _, out OpenCvSharp.Scalar deviation);
            Check(deviation.Val0 == 0, "The constant-luminance colour fixture must have zero grayscale variance.");
            Throws<ArgumentException>(() =>
            {
                using (var appearance = new FrozenAppearance(color, new OpenCvSharp.Rect(0, 0, 64, 64))) { }
            }, "A constant appearance template must not produce a false perfect correlation.");
        }
    }

    private static void RejectDuplicateObjects()
    {
        using (var tile = Texture())
        using (var reference = Frame(tile, 0))
        using (var blank = Frame(null, 0))
        using (var second = Frame(null, 0))
        using (var duplicates = new Bitmap(reference))
        {
            var other = Truth(0); other.Offset(250, 0);
            using (var graphics = Graphics.FromImage(duplicates)) { graphics.DrawImage(tile, other); }
            using (var graphics = Graphics.FromImage(second)) { graphics.DrawImage(tile, other); }
            foreach (var frame in new[] { reference, second })
            using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
            {
                tracker.Initialize(reference, Truth(0), CancellationToken.None);
                tracker.Update(blank, CancellationToken.None);
                for (int i = 0; i < 3; i++)
                { Check(tracker.Update(frame, CancellationToken.None).Found == (i == 2),
                    "Each unique Object placement must recover positively before ambiguity testing"); }
            }
            using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
            {
                tracker.Initialize(reference, Truth(0), CancellationToken.None);
                tracker.Update(blank, CancellationToken.None);
                for (int i = 0; i < 4; i++)
                { Check(!tracker.Update(duplicates, CancellationToken.None).Found,
                    "Two identical Object placements were accepted as a unique returning target"); }
            }
        }
    }

    private static void RejectObjectCenterOccluder()
    {
        using (var tile = Texture())
        using (var reference = Frame(tile, 0))
        foreach (int size in new[] { 48, 96 })
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        using (var occluded = new Bitmap(reference))
        {
            tracker.Initialize(reference, Truth(0), CancellationToken.None);
            Check(tracker.Update(reference, CancellationToken.None).Found, "Clear Object positive never verified");
            using (var graphics = Graphics.FromImage(occluded))
            { graphics.FillRectangle(Brushes.DarkSlateGray, 272 - size / 2, 272 - size / 2, size, size); }
            Check(!tracker.Update(occluded, CancellationToken.None).Found,
                "A foreground covering the Object aiming point was accepted while outer appearance stayed visible");
            for (int index = 1; index <= 3; index++)
            { Check(tracker.Update(reference, CancellationToken.None).Found == (index == 3),
                "Object center-occlusion recovery needs three new matches"); }
        }
    }

    private static Bitmap Texture()
    {
        var result = new Bitmap(72, 72);
        var random = new Random(812);
        for (int y = 0; y < result.Height; y++)
        {
            for (int x = 0; x < result.Width; x++)
            { result.SetPixel(x, y, Color.FromArgb(random.Next(256), random.Next(256), random.Next(256))); }
        }
        return result;
    }

    private static RectangleF Truth(int index) => new RectangleF(200 + index * 12, 200 + index * 6,
        144 + index * 2, 144 + index * 2);

    private static Bitmap Frame(Bitmap tile, int index)
    {
        var result = new Bitmap(960, 720, PixelFormat.Format24bppRgb);
        using (var graphics = Graphics.FromImage(result))
        {
            graphics.Clear(Color.FromArgb(40, 40, 40));
            if (tile != null) { graphics.DrawImage(tile, Truth(index)); }
        }
        return result;
    }

    private static void BenchmarkPerson()
    {
        string directory = Environment.GetEnvironmentVariable("NOMAD_VISION_TRACKING_FRAMES");
        string truthPath = Environment.GetEnvironmentVariable("NOMAD_VISION_TRACKING_GT");
        if (string.IsNullOrEmpty(directory) || string.IsNullOrEmpty(truthPath))
        { Console.WriteLine("SKIPPED: optional owned/external person tracking benchmark."); return; }
        var files = Directory.GetFiles(directory, "*.png").OrderBy(path => path, StringComparer.Ordinal).Take(120).ToArray();
        var truth = File.ReadAllLines(truthPath).Where(line => !string.IsNullOrWhiteSpace(line)).Select(line =>
        {
            var values = line.Split(new[] { ',', ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries)
                .Select(value => float.Parse(value, CultureInfo.InvariantCulture)).ToArray();
            if (values.Length != 4) { throw new FormatException("Expected benchmark x,y,width,height rows."); }
            return new RectangleF(values[0], values[1], values[2], values[3]);
        }).ToArray();
        Check(files.Length == 120 && truth.Length >= files.Length, "The benchmark requires 120 aligned frames.");
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            using (var first = new Bitmap(files[0])) { tracker.Initialize(first, truth[0], CancellationToken.None); }
            double sum = 0, minimum = 1;
            int tracked = 0;
            long maximum = 0;
            for (int index = 1; index < files.Length; index++)
            {
                using (var frame = new Bitmap(files[index]))
                {
                    var elapsed = Stopwatch.StartNew();
                    var result = tracker.Update(frame, CancellationToken.None);
                    if (!result.Found)
                    {
                        Console.WriteLine("Person benchmark loss at " + index + ": " + result.Status + "; " +
                            ((ObjectVisualTracker)tracker).LastObjectDiagnostic);
                        break;
                    }
                    maximum = Math.Max(maximum, elapsed.ElapsedMilliseconds);
                    double overlap = Overlap(result.Bounds.Value, truth[index]);
                    minimum = Math.Min(minimum, overlap); sum += overlap; tracked++;
                }
            }
            Console.WriteLine("Person benchmark coverage " + tracked + "/" + (files.Length - 1));
            Check(tracked == files.Length - 1 && sum / Math.Max(1, tracked) > .4,
                "Person tracking must complete every frame and overlap the independent ground truth.");
            Console.WriteLine("Object tracker person benchmark, no reseeding: " + tracked + "/" + (files.Length - 1) +
                " updates; mean IoU " + (sum / tracked).ToString("F3", CultureInfo.InvariantCulture) +
                ", min IoU " + minimum.ToString("F3", CultureInfo.InvariantCulture) + ", max " + maximum + " ms.");
        }
    }

    private static double Overlap(RectangleF left, RectangleF right)
    {
        var overlap = RectangleF.Intersect(left, right);
        double intersection = Math.Max(0, overlap.Width) * Math.Max(0, overlap.Height);
        return intersection / (left.Width * left.Height + right.Width * right.Height - intersection);
    }

    private static void Check(bool condition, string message)
    { if (!condition) { throw new Exception(message); } _checks++; }

    private static void Throws<T>(Action action, string message) where T : Exception
    { try { action(); } catch (T) { _checks++; return; } throw new Exception(message); }
}
