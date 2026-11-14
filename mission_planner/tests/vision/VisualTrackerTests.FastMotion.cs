// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.Threading;
using Newtonsoft.Json.Linq;
using NOMAD.MissionPlanner;

internal static partial class VisualTrackerTests
{
    // Cosine sweeps start at rest, then reach peak speed = amplitude * 2pi / period. A fast target must stay
    // verified on its independent truth; a verified aim anywhere else is always a failure.
    // ponytail: two speeds (about 1 and 2 deer lengths per frame); at period 27 (3 lengths) the template keeps ~50%.
    private static void FastMotion()
    {
        bool passed = true;
        foreach (int period in new[] { 80, 40 })
        {
            var deer = SweepDeer(period);
            var tile = SweepTexture(period);
            Console.WriteLine("  Fast sweep period=" + period + ": deer " + deer + "; texture " + tile);
            passed &= deer.Wrong == 0 && tile.Wrong == 0 &&
                deer.Followed >= deer.Frames * .95 && tile.Followed >= tile.Frames * .95;
        }
        Check(passed, "A smoothly accelerating fast target was lost or verified in the wrong place; see report above");
        DeerLeavesFrame();
    }

    // A fast deer accelerates out of the left edge, so its coasting search window then leaves the image entirely.
    // That must report no target, never fail the tracker.
    private static void DeerLeavesFrame()
    {
        var fixture = CpuDeerFixture();
        var selection = (JArray)fixture["selection"];
        string file = (string)((JArray)fixture["frames"])[0]["file"];
        var box = (JArray)((JArray)fixture["frames"])[0]["bounds"];
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            using (var first = CpuDeerFrame(file))
            {
                tracker.Initialize(first, new RectangleF(880 + (float)selection[0], 455 + (float)selection[1],
                    (float)selection[2], (float)selection[3]), CancellationToken.None);
            }
            bool nearEdge = false;
            for (int frame = 1; frame <= 45; frame++)
            {
                // Three admission frames at rest, then 3 px/frame^2: about 70 px/frame when it reaches the edge.
                int step = Math.Max(0, frame - 3), offset = -3 * step * step / 2;
                using (var image = CpuDeerFrame(file, offset))
                {
                    var result = tracker.Update(image, CancellationToken.None);
                    Check(!result.Found || 880 + offset + (float)box[0] + (float)box[2] > 0,
                        "A deer that left the image was still verified at frame " + frame);
                    nearEdge |= result.Found && result.AimPoint.Value.X < 200;
                }
            }
            Check(nearEdge, "The fast deer was not followed to the image edge, so its search never coasted off-image: " +
                ((ObjectVisualTracker)tracker).LastObjectDiagnostic);
        }
    }

    private struct Sweep
    {
        internal int Frames, Followed, Wrong; internal double Milliseconds, PeakSpeed;
        public override string ToString() => Followed + "/" + Frames + " followed, wrong=" + Wrong +
            ", peak " + PeakSpeed.ToString("F0") + " px/frame, mean " +
            (Milliseconds / Math.Max(1, Frames)).ToString("F1") + " ms";
    }

    // 26 px rendered deer in HD: the CPU template path.
    private static Sweep SweepDeer(int period)
    {
        var fixture = CpuDeerFixture();
        var selection = (JArray)fixture["selection"];
        string file = (string)((JArray)fixture["frames"])[0]["file"];
        var box = (JArray)((JArray)fixture["frames"])[0]["bounds"];
        return RunSweep(period, 400, offset => CpuDeerFrame(file, offset),
            offset => new RectangleF(880 + offset + (float)selection[0], 455 + (float)selection[1],
                (float)selection[2], (float)selection[3]),
            offset => new RectangleF(880 + offset + (float)box[0], 455 + (float)box[1], (float)box[2], (float)box[3]));
    }

    // 120 px texture in 960x720: the foreground/appearance path.
    private static Sweep SweepTexture(int period)
    {
        using (var tile = Texture())
        {
            Func<int, RectangleF> truth = offset => new RectangleF(420 + offset, 300, 120, 120);
            return RunSweep(period, 250, offset =>
            {
                var image = new Bitmap(960, 720, PixelFormat.Format24bppRgb);
                using (var graphics = Graphics.FromImage(image))
                { graphics.Clear(Color.FromArgb(40, 40, 40)); graphics.DrawImage(tile, truth(offset)); }
                return image;
            }, truth, truth);
        }
    }

    private static Sweep RunSweep(int period, double amplitude, Func<int, Bitmap> render,
        Func<int, RectangleF> selection, Func<int, RectangleF> truth)
    {
        var sweep = new Sweep { PeakSpeed = amplitude * 2 * Math.PI / period };
        Func<int, int> offset = frame => (int)Math.Round(-amplitude * Math.Cos(2 * Math.PI * frame / period));
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            using (var first = render(offset(0))) { tracker.Initialize(first, selection(offset(0)), CancellationToken.None); }
            // Template admission takes three frames; they fall in the slow start of the sweep.
            for (int frame = 1; frame <= 2 * period; frame++)
            using (var image = render(offset(frame)))
            {
                var clock = Stopwatch.StartNew();
                var result = tracker.Update(image, CancellationToken.None);
                sweep.Milliseconds += clock.Elapsed.TotalMilliseconds;
                if (frame <= 3) { continue; }
                sweep.Frames++;
                if (!result.Found) { continue; }
                var expected = truth(offset(frame));
                if (PointDistance(result.AimPoint.Value, Center(expected)) <= Math.Max(6, expected.Width / 4))
                { sweep.Followed++; }
                else { sweep.Wrong++; }
            }
        }
        return sweep;
    }
}
