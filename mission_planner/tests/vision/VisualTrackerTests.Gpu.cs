// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.IO;
using System.Threading;
using NOMAD.MissionPlanner;

internal static partial class VisualTrackerTests
{
    // Opt-in: NOMAD_TEST_TAM_MODEL names the exported, hash-pinned EfficientTAM folder. Needs an NVIDIA DX12 GPU.
    internal static void GpuObjectTracker()
    {
        GpuTrackerSetting();
        string model = Environment.GetEnvironmentVariable("NOMAD_TEST_TAM_MODEL");
        if (string.IsNullOrEmpty(model))
        {
            Console.WriteLine("SKIP: GPU Object tracker checks (set NOMAD_TEST_TAM_MODEL to the exported model folder).");
            return;
        }
        GpuFactoryIncluded(model);
        using (var tracker = new EfficientTamVisualTracker(model, CancellationToken.None))
        {
            Console.WriteLine("GPU Object tracker provider: " + tracker.Provider);
            Check(tracker.Provider.Contains("NVIDIA"), "GPU tracker did not select the NVIDIA adapter: " + tracker.Provider);
            using (var first = TurningObject(false, 0)) { tracker.Initialize(first, new RectangleF(180, 105, 170, 200), CancellationToken.None); }
            int verified = 0;
            for (int frame = 1; frame <= 12; frame++)
            using (var image = TurningObject(false, frame))
            {
                var result = tracker.Update(image, CancellationToken.None);
                if (!result.Found) { continue; }
                verified++;
                Check(PointDistance(result.AimPoint.Value, new PointF(265 + frame * 3, 203)) < 15,
                    "GPU tracker aim left the turning face at frame " + frame + ": " + result.AimPoint + "; " + tracker.LastDiagnostic);
            }
            Check(verified >= 11, "GPU tracker lost a moving, turning face: " + verified + "/12; " + tracker.LastDiagnostic);

            // An occluder over the aiming point may leave the visible face verified, but never moves the aim onto it
            // or extends the box over the occluder.
            foreach (bool similar in new[] { false, true })
            for (int frame = 13; frame <= 16; frame++)
            using (var image = TurningObject(false, frame))
            using (var graphics = Graphics.FromImage(image))
            {
                var block = new RectangleF(265 + frame * 3 - 44, 300, 88, 90);
                using (var brush = new SolidBrush(similar ? Color.FromArgb(190, 143, 102) : Color.DarkSlateGray))
                { graphics.FillRectangle(brush, block); }
                var result = tracker.Update(image, CancellationToken.None);
                Check(!result.Found || (result.Bounds.Value.Bottom < block.Top + 8 &&
                    PointDistance(result.AimPoint.Value, new PointF(265 + frame * 3, 203)) < 25),
                    "GPU tracker accepted a " + (similar ? "similar-colour " : "") + "occluder beside the face: " +
                    result.Bounds + "; " + tracker.LastDiagnostic);
            }

            // An identical twin beside the tracked face must not take over the track or the memory.
            using (var twinned = Twin(TurningObject(false, 16), 260))
            {
                for (int frame = 0; frame < 4; frame++)
                {
                    var result = tracker.Update(twinned, CancellationToken.None);
                    Check(result.Found && PointDistance(result.AimPoint.Value, new PointF(313, 203)) < 25,
                        "GPU tracker left the selected face for its twin: " + result.AimPoint + "; " + tracker.LastDiagnostic);
                }
            }
            int committed = tracker.CommittedFrames;
            using (var empty = new Bitmap(640, 480))
            using (var graphics = Graphics.FromImage(empty))
            {
                graphics.Clear(Color.FromArgb(25, 25, 25));
                for (int frame = 0; frame < 5; frame++)
                {
                    Check(!tracker.Update(empty, CancellationToken.None).Found && tracker.CommittedFrames == committed,
                        "GPU tracker verified or learned an empty scene after the object was removed: " + tracker.LastDiagnostic);
                }
            }
            // Return to the last position needs three consecutive admissions before steering resumes.
            using (var back = TurningObject(false, 16))
            {
                var first = tracker.Update(back, CancellationToken.None, new VisualFrameContext(null, true));
                var second = tracker.Update(back, CancellationToken.None);
                int confirming = tracker.CommittedFrames;
                var third = tracker.Update(back, CancellationToken.None);
                Check(!first.Found && !second.Found && confirming == committed,
                    "GPU tracker resumed or learned before three confirmations: " + first.Status + " | " + second.Status +
                    " | committed " + committed + "->" + confirming + "; " + tracker.LastDiagnostic);
                Check(third.Found && tracker.CommittedFrames == committed + 1,
                    "GPU tracker did not resume after three consistent confirmations: " + third.Status + "; " + tracker.LastDiagnostic);
            }
            using (var other = new Bitmap(320, 240))
            {
                Check(tracker.Update(other, CancellationToken.None).State == VisualTrackingState.Lost,
                    "GPU tracker accepted a different image size without reselection");
            }
            // Reselection must discard the previous target's video memory.
            using (var pencil = TurningObject(true, 0))
            { tracker.Initialize(pencil, new RectangleF(225, 135, 80, 170), CancellationToken.None); }
            Check(tracker.CommittedFrames == 1, "Reselection kept the previous target's memory: " + tracker.CommittedFrames);
        }
        GpuSmallTurningObject(model);
    }

    // Same sub-patch animal blob as the CPU case: whole-object centre and box while it turns side-on to head-on.
    private static void GpuSmallTurningObject(string model)
    {
        using (var field = SmallObjectField())
        using (var first = SmallObjectFrame(field, 960, 44))
        using (var tracker = new EfficientTamVisualTracker(model, CancellationToken.None))
        {
            tracker.Initialize(first, new RectangleF(922, 502, 76, 76), CancellationToken.None);
            int verified = 0;
            for (int frame = 1; frame <= 40; frame++)
            {
                float cx = 960 + frame * 4, width = 44 - frame * .7f;
                using (var image = SmallObjectFrame(field, cx, width))
                {
                    var result = tracker.Update(image, CancellationToken.None);
                    if (!result.Found) { continue; }
                    verified++;
                    var truth = SmallObjectBounds(cx, width);
                    var centre = new PointF(truth.X + truth.Width / 2, truth.Y + truth.Height / 2);
                    Check(PointDistance(result.AimPoint.Value, centre) <= 8 && BoxOverlap(result.Bounds.Value, truth) >= .5,
                        "GPU small turning object left the whole object at frame " + frame + ": " + result.Bounds +
                        " versus " + truth + "; " + tracker.LastDiagnostic);
                }
            }
            Check(verified >= 38, "GPU tracker lost the small turning object: " + verified + "/40; " + tracker.LastDiagnostic);
            // After a long loss, a lone lookalike near the stale position must not be accepted by position alone.
            using (var empty = new Bitmap(field))
            { for (int frame = 0; frame < 20; frame++) { tracker.Update(empty, CancellationToken.None); } }
            using (var lookalike = SmallObjectFrame(field, 960 + 160 + 40, 16))
            {
                for (int frame = 0; frame < 6; frame++)
                {
                    Check(!tracker.Update(lookalike, CancellationToken.None).Found,
                        "GPU tracker accepted a lookalike long after the selected object was lost: " + tracker.LastDiagnostic);
                }
            }
        }
    }

    // The setting selects the tracker, Area never changes, and a broken model folder fails instead of falling back.
    private static void GpuTrackerSetting()
    {
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        { Check(tracker is ObjectVisualTracker, "The default Object tracker must stay on the CPU tracker"); }
        using (var area = VisualTrackerFactory.Create(CancellationToken.None, VisualTrackingMode.Area, true))
        { Check(area is AreaVisualTracker, "The GPU Object setting replaced the Area tracker"); }
        Throws<IOException>(() => new EfficientTamVisualTracker(
            Path.Combine(Path.GetTempPath(), "missing-" + Guid.NewGuid().ToString("N")), CancellationToken.None).Dispose(),
            "A missing GPU model folder did not fail");
    }

    // The embedded, hash-checked model (not a folder path) backs the GPU setting.
    private static void GpuFactoryIncluded(string model)
    {
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None, VisualTrackingMode.Object, true))
        { Check(tracker is EfficientTamVisualTracker, "The GPU setting did not create the GPU Object tracker"); }
    }

    private static Bitmap Twin(Bitmap source, int dx)
    {
        using (var copy = (Bitmap)source.Clone())
        using (var graphics = Graphics.FromImage(source))
        {
            var face = new Rectangle(255, 120, 116, 170);
            graphics.DrawImage(copy, new Rectangle(face.X + dx, face.Y, face.Width, face.Height), face, GraphicsUnit.Pixel);
        }
        return source;
    }
}
