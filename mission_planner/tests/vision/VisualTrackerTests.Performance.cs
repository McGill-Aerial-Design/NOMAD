// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Threading;
using Newtonsoft.Json;
using NOMAD.MissionPlanner;
using OpenCvSharp;

internal static partial class VisualTrackerTests
{
    private static void TrackingImagePixels()
    {
        var memory = System.Runtime.InteropServices.Marshal.AllocHGlobal(24);
        try
        {
            using (var inverted = new Bitmap(3, 2, -12, System.Drawing.Imaging.PixelFormat.Format24bppRgb,
                IntPtr.Add(memory, 12)))
            using (var argb = new Bitmap(3, 2, System.Drawing.Imaging.PixelFormat.Format32bppArgb))
            foreach (var image in new[] { inverted, argb })
            {
                image.SetPixel(0, 0, System.Drawing.Color.Red); image.SetPixel(2, 1, System.Drawing.Color.Blue);
                using (var color = VisualTrackingImage.Color(image))
                {
                    var top = color.At<Vec3b>(0, 0); var bottom = color.At<Vec3b>(1, 2);
                    Check(top.Item0 == 0 && top.Item1 == 0 && top.Item2 == 255 &&
                        bottom.Item0 == 255 && bottom.Item1 == 0 && bottom.Item2 == 0,
                        "Native image preparation inverted rows or changed BGR channels: " + image.PixelFormat);
                }
            }
        }
        finally { System.Runtime.InteropServices.Marshal.FreeHGlobal(memory); }
    }

    private static void FeatureMatchingParity()
    {
        var random = new Random(723);
        foreach (bool duplicates in new[] { false, true })
        {
            var reference = new byte[96, 32]; var current = new byte[256, 32];
            for (int row = 0; row < 96; row++)
            for (int col = 0; col < 32; col++) { reference[row, col] = (byte)random.Next(256); }
            for (int row = 0; row < 256; row++)
            for (int col = 0; col < 32; col++)
            { current[row, col] = row < 64 ? reference[row, col] : (byte)random.Next(256); }
            for (int row = 0; row < 64; row++) { current[row, 0] ^= 1; }
            if (duplicates)
            {
                for (int row = 0; row < 16; row++)
                for (int col = 0; col < 32; col++) { current[row + 64, col] = current[row, col]; }
            }
            using (var a = Mat.FromArray(reference))
            using (var b = Mat.FromArray(current))
            using (var matcher = new BFMatcher(NormTypes.Hamming))
            {
                var forward = matcher.KnnMatch(a, b, 2); var reverse = matcher.KnnMatch(b, a, 2);
                var expected = forward.Where(pair => pair.Length == 2 && pair[0].Distance < 64 &&
                    pair[0].Distance < pair[1].Distance * .72 && reverse[pair[0].TrainIdx].Length == 2 &&
                    reverse[pair[0].TrainIdx][0].TrainIdx == pair[0].QueryIdx &&
                    reverse[pair[0].TrainIdx][0].Distance < reverse[pair[0].TrainIdx][1].Distance * .72)
                    .Select(pair => pair[0].QueryIdx + ":" + pair[0].TrainIdx).ToArray();
                var actual = VisualFeatureMatching.Mutual(a, b).Select(m => m.QueryIdx + ":" + m.TrainIdx).ToArray();
                Check(expected.Length >= 48 && actual.SequenceEqual(expected),
                    "Candidate-only reverse matching changed the full mutual/ratio result; duplicates=" + duplicates);
                using (var empty = new Mat())
                { Check(VisualFeatureMatching.Mutual(a, empty).Length == 0, "Empty features produced a placement"); }
            }
        }
    }

    internal static void BenchmarkCpu()
    {
        using (var tile = Texture())
        using (var ground = GroundTexture())
        {
            MeasureCpu("Object texture/scale", VisualTrackingMode.Object,
                index => Frame(tile, index), Truth(0), index => Center(Truth(index)));
            MeasureCpu("Object HD thin", VisualTrackingMode.Object,
                index => SmallObjectHdFrame(index), new RectangleF(949, 465, 22, 150),
                index => new PointF(960 + index * 6, 540));
            MeasureCpu("Area HD perspective", VisualTrackingMode.Area, index =>
            {
                using (var projected = Project(ground, AreaMotion(index)))
                { return new Bitmap(projected, 1920, 1440); }
            }, new RectangleF(AreaSelection.X * 3, AreaSelection.Y * 3, AreaSelection.Width * 3, AreaSelection.Height * 3),
                index =>
                {
                    var point = ProjectPoint(Center(AreaSelection), AreaMotion(index));
                    return new PointF(point.X * 3, point.Y * 3);
                });
            var deer = CpuDeerFixture();
            var deerFrames = (Newtonsoft.Json.Linq.JArray)deer["frames"];
            MeasureCpu("Object rendered deer / CPU template", VisualTrackingMode.Object,
                index => CpuDeerFrame((string)deerFrames[index % deerFrames.Count]["file"]),
                new RectangleF(944, 522, 32, 32), index =>
                {
                    var box = deerFrames[index % deerFrames.Count]["bounds"];
                    return new PointF(880 + (float)box[0] + (float)box[2] / 2,
                        455 + (float)box[1] + (float)box[3] / 2);
                });
        }
    }

    private static double[,] AreaMotion(int index) => new double[,]
        { { 1, .001 * index, index }, { -.001 * index, 1, -index / 2.0 }, { .00001 * index, 0, 1 } };

    private static void MeasureCpu(string name, VisualTrackingMode mode, Func<int, Bitmap> render,
        RectangleF selection, Func<int, PointF> aim)
    {
        var times = new List<double>();
        double cpuMs = 0, maximumError = 0;
        object lastTiming = null;
        object slowestTiming = null;
        double slowest = 0;
        int verified = 0;
        using (var process = Process.GetCurrentProcess())
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None, mode))
        using (var first = render(0))
        {
            tracker.Initialize(first, selection, CancellationToken.None);
            for (int frame = 0; frame < 128; frame++)
            {
                // Reversing motion exercises changes in velocity without reselection or artificial frame jumps.
                int index = frame % 20; if (index > 10) { index = 20 - index; }
                using (var image = render(index))
                {
                    double cpuStart = process.TotalProcessorTime.TotalMilliseconds;
                    var timer = Stopwatch.StartNew();
                    var result = tracker.Update(image, CancellationToken.None);
                    double milliseconds = timer.Elapsed.TotalMilliseconds;
                    double cpu = process.TotalProcessorTime.TotalMilliseconds - cpuStart;
                    lastTiming = (tracker as AreaVisualTracker)?.LastTiming;
                    if (frame >= 8 && milliseconds > slowest) { slowest = milliseconds; slowestTiming = lastTiming; }
                    if (frame < 8) { continue; }
                    times.Add(milliseconds); cpuMs += cpu;
                    if (result.Found)
                    { verified++; maximumError = Math.Max(maximumError, PointDistance(result.AimPoint.Value, aim(index))); }
                }
            }
        }
        times.Sort();
        Console.WriteLine(JsonConvert.SerializeObject(new { benchmark = name, samples = times.Count, verified,
            mean_ms = times.Average(), p50_ms = times[times.Count / 2],
            p95_ms = times[(int)Math.Ceiling(times.Count * .95) - 1], max_ms = times.Last(),
            mean_cpu_ms = cpuMs / times.Count, maximum_aim_error_px = maximumError,
            last_stage_timings = lastTiming, slowest_stage_timings = slowestTiming }));
        Check(verified == times.Count && maximumError < 8,
            name + " failed independent aim truth: " + verified + "/" + times.Count + "; error=" + maximumError);
        Check(times[(int)Math.Ceiling(times.Count * .95) - 1] <= 1000.0 / 30,
            name + " exceeded the 30 Hz p95 update budget; see the timing report above");
    }
}
