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

internal static class VisualTrackerTests
{
    private static int _checks;

    internal static void Run()
    {
        using (var tile = Texture())
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            Check(VisionRuntime.BuiltinModelPath == null, "Visual tracking must load without the detector runtime.");
            using (var first = Frame(tile, 0))
            {
                RectangleF ignored;
                Check(!tracker.TryUpdate(first, CancellationToken.None, out ignored), "Uninitialized tracker has no target.");
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
                    RectangleF bounds;
                    Check(tracker.TryUpdate(frame, CancellationToken.None, out bounds),
                        "CSRT must follow the moving textured region without detections.");
                    long milliseconds = elapsed.ElapsedMilliseconds;
                    maximum = Math.Max(maximum, milliseconds); total += milliseconds;
                    Check(Overlap(bounds, Truth(index)) > .45,
                        "CSRT region must overlap the independently rendered moving/scaling target.");
                }
            }
            using (var blank = Frame(null, 0))
            using (var targetReturns = Frame(tile, 10))
            {
                RectangleF bounds;
                Check(!tracker.TryUpdate(blank, CancellationToken.None, out bounds) && bounds.IsEmpty,
                    "Native CSRT confidence loss must return no target.");
                Check(!tracker.TryUpdate(targetReturns, CancellationToken.None, out bounds),
                    "A lost target must never be reacquired automatically.");
                tracker.Initialize(targetReturns, Truth(10), CancellationToken.None);
                using (var resized = new Bitmap(800, 600))
                { Check(!tracker.TryUpdate(resized, CancellationToken.None, out bounds), "Image-size change must retire tracking."); }
                tracker.Initialize(targetReturns, Truth(10), CancellationToken.None);
                using (var canceled = new CancellationTokenSource())
                {
                    canceled.Cancel();
                    Throws<OperationCanceledException>(() => tracker.TryUpdate(targetReturns, canceled.Token, out bounds),
                        "Canceled tracking must publish no result.");
                    Check(!tracker.TryUpdate(targetReturns, CancellationToken.None, out bounds),
                        "Cancellation must retire the visual target.");
                }
            }
            Console.WriteLine("CSRT texture/scale: 10 updates, mean " + total / 10.0 + " ms, max " + maximum +
                " ms; native target loss confirmed.");
        }
        using (var disposed = VisualTrackerFactory.Create(CancellationToken.None))
        {
            disposed.Dispose();
            using (var frame = Frame(null, 0))
            {
                RectangleF bounds;
                Throws<ObjectDisposedException>(() => disposed.TryUpdate(frame, CancellationToken.None, out bounds),
                    "Disposed tracker must refuse frames.");
            }
        }
        BenchmarkPerson();
        Console.WriteLine("PASS: " + _checks + " visual tracker checks.");
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
                    RectangleF bounds;
                    if (!tracker.TryUpdate(frame, CancellationToken.None, out bounds)) { break; }
                    maximum = Math.Max(maximum, elapsed.ElapsedMilliseconds);
                    double overlap = Overlap(bounds, truth[index]);
                    minimum = Math.Min(minimum, overlap); sum += overlap; tracked++;
                }
            }
            Check(tracked == files.Length - 1 && sum / tracked > .4,
                "Person tracking must complete every frame and overlap the independent ground truth.");
            Console.WriteLine("CSRT person benchmark, no reseeding: " + tracked + "/" + (files.Length - 1) +
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
