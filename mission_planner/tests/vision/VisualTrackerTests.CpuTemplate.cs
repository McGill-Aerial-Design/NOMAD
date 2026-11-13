// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Threading;
using Newtonsoft.Json.Linq;
using NOMAD.MissionPlanner;

internal static partial class VisualTrackerTests
{
    private static JObject CpuDeerFixture()
    {
        using (var stream = System.Reflection.Assembly.GetExecutingAssembly().GetManifestResourceStream("NOMAD.Tests.cpu-deer.json"))
        using (var reader = new StreamReader(stream)) { return JObject.Parse(reader.ReadToEnd()); }
    }

    private static void CpuRenderedDeer()
    {
        var fixture = CpuDeerFixture();
        var selection = (JArray)fixture["selection"];
        var bounds = new RectangleF(880 + (float)selection[0], 455 + (float)selection[1], (float)selection[2], (float)selection[3]);
        var frames = (JArray)fixture["frames"];
        using (var first = CpuDeerFrame((string)frames[0]["file"]))
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            tracker.Initialize(first, bounds, CancellationToken.None);
            CpuTemplateAlternatives();
            for (int index = 1; index < frames.Count; index++)
            using (var image = CpuDeerFrame((string)frames[index]["file"]))
            {
                var result = tracker.Update(image, CancellationToken.None);
                Check(result.Found == (index >= 3), "CPU deer admission needs three real frames: " + result.Status);
                if (result.Found) { CheckCpuDeerAim(result, frames[index]); }
            }
            // Move beyond the original search crop; admission must release its provisional search position.
            foreach (int step in Enumerable.Range(0, 41).Concat(Enumerable.Range(0, 40).Reverse()))
            using (var image = CpuDeerFrame((string)frames[0]["file"], step * 4))
            {
                var result = tracker.Update(image, CancellationToken.None);
                Check(result.Found, "CPU deer stopped following a continuous translation: " + result.Status);
                CheckCpuDeerAim(result, frames[0], step * 4);
            }
            using (var blank = new Bitmap(first.Width, first.Height))
            {
                using (var graphics = Graphics.FromImage(blank)) { graphics.Clear(Color.DarkSlateGray); }
                for (int index = 0; index < 5; index++)
                {
                    var result = tracker.Update(blank, CancellationToken.None);
                    Check(!result.Found && !result.AimPoint.HasValue, "An opaque replacement was accepted as the CPU deer");
                }
            }
            for (int index = 1; index <= 3; index++)
            {
                var result = tracker.Update(first, CancellationToken.None);
                Check(result.Found == (index == 3), "CPU deer recovery failed at " + index + ": " + result.Status +
                    "; " + ((ObjectVisualTracker)tracker).LastObjectDiagnostic);
                if (result.Found) { CheckCpuDeerAim(result, frames[0]); }
            }
            for (int index = 0; index < 3; index++)
            {
                var result = tracker.Update(first, CancellationToken.None, new VisualFrameContext(null, index == 0));
                Check(result.Found == (index == 2), "Receipt discontinuity retained CPU template admission");
            }
            using (var cancellation = new CancellationTokenSource())
            {
                cancellation.Cancel();
                Throws<OperationCanceledException>(() => tracker.Update(first, cancellation.Token), "CPU template cancellation was ignored");
                Check(!tracker.Update(first, CancellationToken.None).Found, "Canceled CPU template retained a target");
            }
        }
        using (var first = CpuDeerFrame((string)frames[0]["file"]))
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            tracker.Initialize(first, bounds, CancellationToken.None);
            for (int index = 1; index <= 3; index++)
            using (var image = CpuDeerFrame((string)frames[0]["file"], index * 8))
            {
                var result = tracker.Update(image, CancellationToken.None);
                Check(result.Found == (index == 3), "Moving CPU deer confirmation used an obsolete position: " + result.Status);
                if (result.Found) { CheckCpuDeerAim(result, frames[0], index * 8); }
            }
        }
        Console.WriteLine("CPU rendered/decode deer: independent whole-object bounds, opaque loss, recovery and gap fences passed.");
    }

    private static void CpuTemplateAlternatives()
    {
        var confidence = new float[256]; var sizes = new float[512]; var offsets = new float[512];
        int best = 8 * 16 + 8, other = best + 2;
        confidence[best] = .9f; confidence[other] = .85f;
        sizes[best] = sizes[256 + best] = sizes[other] = sizes[256 + other] = .1f;
        var crop = new RectangleF(0, 0, 160, 160); var target = new RectangleF(72, 72, 16, 16);
        offsets[other] = -2;
        Check(CpuTemplateTracker.CompetingScore(crop, target, best, confidence, sizes, offsets, 160, 160) == 0,
            "Two cells regressing the same object were treated as competing identities");
        offsets[other] = 2;
        Check(CpuTemplateTracker.CompetingScore(crop, target, best, confidence, sizes, offsets, 160, 160) == .85f,
            "A distinct plausible object hypothesis was hidden");
        sizes[other] = sizes[256 + other] = .5f;
        Check(CpuTemplateTracker.CompetingScore(crop, target, best, confidence, sizes, offsets, 160, 160) == 0,
            "An incompatible object scale was treated as the selected object");
        sizes[other] = sizes[256 + other] = .1f; offsets[other] = 200;
        Check(CpuTemplateTracker.CompetingScore(crop, target, best, confidence, sizes, offsets, 160, 160) == 0,
            "An off-image prediction was treated as a competing object");
    }

    private static void CheckCpuDeerAim(VisualTrackingResult result, JToken frame, int offset = 0)
    {
        var expected = (JArray)frame["bounds"];
        var truth = new RectangleF(880 + offset + (float)expected[0], 455 + (float)expected[1],
            (float)expected[2], (float)expected[3]);
        Check(Overlap(result.Bounds.Value, truth) >= .5 && PointDistance(result.AimPoint.Value, Center(truth)) <= 6,
            "CPU template accepted wrong rendered deer bounds/centre: " + result.Bounds + " / " + truth +
            "; IoU=" + Overlap(result.Bounds.Value, truth) + "; aim=" + result.AimPoint);
    }

    private static Bitmap CpuDeerFrame(string file, int offset = 0)
    {
        var image = new Bitmap(1920, 1080);
        using (var crop = RenderedAreaImage(file))
        using (var graphics = Graphics.FromImage(image)) { graphics.DrawImageUnscaled(crop, 880 + offset, 455); }
        return image;
    }
}
