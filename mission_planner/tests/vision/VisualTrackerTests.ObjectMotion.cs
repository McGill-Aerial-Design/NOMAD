// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Diagnostics;
using System.Threading;
using NOMAD.MissionPlanner;

internal static partial class VisualTrackerTests
{
    private static void ObjectBackgroundAndTurning()
    {
        foreach (bool pencil in new[] { false, true })
        using (var first = TurningObject(pencil, 0))
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            var selection = pencil ? new RectangleF(225, 135, 80, 170) : new RectangleF(180, 105, 170, 200);
            tracker.Initialize(first, selection, CancellationToken.None);
            int verified = 0;
            for (int frame = 1; frame <= 24; frame++)
            using (var image = TurningObject(pencil, frame))
            {
                var result = tracker.Update(image, CancellationToken.None);
                var truth = pencil ? new PointF(265 + frame * 3, 220) : new PointF(265 + frame * 3, 205);
                if (result.Found)
                {
                    verified++;
                    Check(PointDistance(result.AimPoint.Value, truth) < 15,
                        "Changing background took over the " + (pencil ? "pencil" : "turning foreground") + " aim");
                }
                Check(result.Found, (pencil ? "Thin pencil" : "Turning foreground") +
                    " lost on independent background/appearance change at frame " + frame + ": " + result.Status +
                    "; " + ((ObjectVisualTracker)tracker).LastObjectDiagnostic);
            }
            Check(((ObjectVisualTracker)tracker).LearnedViews > 0, "Verified foreground views never adapted");
            Check(verified >= 23, "Always-searching Object tracker cannot pass rotation/background positives");
        }
        RejectOccluderAfterLearning();
        RejectRemovedThinObject();
        ThinForegroundSupport();
        ForegroundGeometryNeedsTheObject();
        HighResolutionSmallObject();
        WholeObjectCentre();
        UniformWholeObjectCentre();
        SmallTurningObject();
    }

    // A distant animal-sized blob turns from side-on to head-on while moving. At this size the 48x48 comparison
    // patch only upsamples it, so the turning outline must be followed without accepting removal or duplicates.
    private static void SmallTurningObject()
    {
        using (var field = SmallObjectField())
        using (var first = SmallObjectFrame(field, 960, 44))
        using (var tracker = new ObjectVisualTracker())
        {
            tracker.Initialize(first, new RectangleF(922, 502, 76, 76), CancellationToken.None);
            int verified = 0;
            float cx = 960, width = 44;
            for (int frame = 1; frame <= 40; frame++)
            {
                cx = 960 + frame * 4; width = 44 - frame * .7f;
                using (var image = SmallObjectFrame(field, cx, width))
                {
                    var result = tracker.Update(image, CancellationToken.None);
                    var truth = SmallObjectBounds(cx, width);
                    if (!result.Found) { continue; }
                    verified++;
                    var centre = new PointF(truth.X + truth.Width / 2, truth.Y + truth.Height / 2);
                    Check(PointDistance(result.AimPoint.Value, centre) <= 8 && BoxOverlap(result.Bounds.Value, truth) >= .5,
                        "Small turning object aim/box left the whole object at frame " + frame + ": " + result.Bounds +
                        " versus " + truth + "; " + tracker.LastObjectDiagnostic);
                }
            }
            Check(verified >= 38, "Small turning object was not followed through its outline change: " + verified +
                "/40; " + tracker.LastObjectDiagnostic);
            int learned = tracker.LearnedViews;
            using (var empty = new Bitmap(field))
            {
                for (int frame = 0; frame < 5; frame++)
                {
                    Check(!tracker.Update(empty, CancellationToken.None).Found && tracker.LearnedViews == learned,
                        "Background replaced a removed small object or was learned as its appearance");
                }
            }
            // A stale-video gap retires local continuity; return near the last position needs three matches.
            using (var back = SmallObjectFrame(field, cx + 20, width))
            {
                Check(!tracker.Update(back, CancellationToken.None, new VisualFrameContext(null, true)).Found &&
                    !tracker.Update(back, CancellationToken.None).Found,
                    "A small object returned without three consistent recovery matches");
                bool returned = false;
                for (int frame = 0; frame < 4 && !returned; frame++)
                { returned = tracker.Update(back, CancellationToken.None).Found; }
                Check(returned, "A small object did not return near its last position: " + tracker.LastObjectDiagnostic);
            }
            using (var twins = SmallObjectFrame(field, cx - 30, width, cx + 70))
            {
                tracker.Update(twins, CancellationToken.None, new VisualFrameContext(null, true));
                for (int frame = 0; frame < 6; frame++)
                {
                    Check(!tracker.Update(twins, CancellationToken.None).Found,
                        "Nearby recovery chose one of two indistinguishable small objects");
                }
            }
            // After a long loss, a lone lookalike near the stale position is not the selected object by position
            // alone (a live orbit locked onto a decoy deer this way after 13.6 s).
            using (var empty = new Bitmap(field))
            {
                for (int frame = 0; frame < 20; frame++) { tracker.Update(empty, CancellationToken.None); }
            }
            using (var lookalike = SmallObjectFrame(field, cx + 40, width))
            {
                for (int frame = 0; frame < 6; frame++)
                {
                    Check(!tracker.Update(lookalike, CancellationToken.None).Found,
                        "Nearby recovery accepted a lookalike long after the selected object was lost: " +
                        tracker.LastObjectDiagnostic);
                }
            }
        }
    }

    private static Bitmap SmallObjectField()
    {
        var image = new Bitmap(1920, 1080);
        var random = new Random(173);
        using (var graphics = Graphics.FromImage(image))
        {
            graphics.Clear(Color.FromArgb(70, 110, 50));
            for (int patch = 0; patch < 6000; patch++)
            {
                int shade = random.Next(-18, 19);
                using (var brush = new SolidBrush(Color.FromArgb(70 + shade, 110 + shade, 50 + shade / 2)))
                { graphics.FillEllipse(brush, random.Next(1920), random.Next(1080), random.Next(4, 24), random.Next(3, 12)); }
            }
        }
        return image;
    }

    private static Bitmap SmallObjectFrame(Bitmap field, float cx, float width, float? twin = null)
    {
        var image = new Bitmap(field);
        using (var graphics = Graphics.FromImage(image))
        using (var body = new SolidBrush(Color.FromArgb(105, 72, 40)))
        using (var dark = new SolidBrush(Color.FromArgb(70, 48, 28)))
        {
            foreach (float x in twin.HasValue ? new[] { cx, twin.Value } : new[] { cx })
            {
                graphics.FillEllipse(body, x - width / 2, 528, width, 20);
                for (int leg = 0; leg < 4; leg++)
                { graphics.FillRectangle(dark, x - width / 2 + 3 + leg * (width - 9) / 3, 544, 3, 14); }
                graphics.FillEllipse(dark, x + width / 2 - 8, 520, 10, 10);
            }
        }
        return image;
    }

    private static RectangleF SmallObjectBounds(float cx, float width) =>
        RectangleF.FromLTRB(cx - width / 2, 520, Math.Max(cx + width / 2, cx + width / 2 + 2), 558);

    private static double BoxOverlap(RectangleF a, RectangleF b)
    {
        var both = RectangleF.Intersect(a, b);
        double shared = both.Width * both.Height;
        return shared / (a.Width * a.Height + b.Width * b.Height - shared);
    }

    private static void WholeObjectCentre()
    {
        using (var tile = Texture())
        using (var first = Frame(tile, 0))
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            tracker.Initialize(first, new RectangleF(170, 100, 204, 300), CancellationToken.None);
            for (int index = 1; index <= 8; index++)
            using (var image = Frame(tile, index))
            {
                var result = tracker.Update(image, CancellationToken.None);
                var truth = Truth(index);
                var centre = new PointF(truth.X + truth.Width / 2, truth.Y + truth.Height / 2);
                Check(result.Found && PointDistance(result.AimPoint.Value, centre) <= 8,
                    "Object kept the off-centre selection point instead of the visible object centre: " + index +
                    "; state=" + result.State + "; aim=" + result.AimPoint + "; expected=" + centre);
            }
        }
        using (var first = TurningObject(true, 0))
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            tracker.Initialize(first, new RectangleF(225, 60, 80, 280), CancellationToken.None);
            for (int index = 1; index <= 12; index++)
            using (var image = TurningObject(true, index))
            {
                var result = tracker.Update(image, CancellationToken.None);
                Check(result.Found && PointDistance(result.AimPoint.Value, new PointF(265 + index * 3, 220)) < 15,
                    "Rotating pencil aim followed an old surface offset instead of its centre: " + index +
                    "; state=" + result.State + "; aim=" + result.AimPoint);
            }
        }
    }

    private static void UniformWholeObjectCentre()
    {
        Bitmap Shaft(int index)
        {
            var image = new Bitmap(640, 480);
            using (var graphics = Graphics.FromImage(image))
            {
                graphics.Clear(Color.FromArgb(40, 40, 40));
                using (var brush = new SolidBrush(Color.FromArgb(180, 140, 30)))
                { graphics.FillRectangle(brush, 260 + index * 3, 150, 10, 140); }
            }
            return image;
        }
        using (var first = Shaft(0))
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            tracker.Initialize(first, new RectangleF(225, 135, 80, 170), CancellationToken.None);
            for (int index = 1; index <= 6; index++)
            using (var image = Shaft(index))
            {
                var result = tracker.Update(image, CancellationToken.None);
                Check(result.Found && PointDistance(result.AimPoint.Value, new PointF(265 + index * 3, 220)) < 8,
                    "A uniform shaft lost its contrasted outline or centre after foreground promotion");
            }
        }
    }

    private static void HighResolutionSmallObject()
    {
        using (var first = SmallObjectHdFrame(0))
        using (var tracker = new ObjectVisualTracker())
        {
            tracker.Initialize(first, new RectangleF(949, 465, 22, 150), CancellationToken.None);
            Check(tracker.ProcessingImageSize == new Size(1280, 720),
                "A thin HD object discarded the source pixels needed to establish its identity");
            long maximum = 0;
            for (int frame = 1; frame <= 12; frame++)
            using (var image = SmallObjectHdFrame(frame))
            {
                var clock = Stopwatch.StartNew();
                var result = tracker.Update(image, CancellationToken.None);
                maximum = Math.Max(maximum, clock.ElapsedMilliseconds);
                Check(result.Found && PointDistance(result.AimPoint.Value, new PointF(960 + frame * 6, 540)) < 8,
                    "Thin HD foreground was lost or replaced by the independently changing background at frame " + frame +
                    ": " + result.Status + "; " + tracker.LastObjectDiagnostic);
                Check(clock.ElapsedMilliseconds < 1000 && tracker.ProcessingImageSize == new Size(1280, 720),
                    "HD Object update exceeded its bounded time/image budget or changed processing scale mid-selection");
            }
            int learned = tracker.LearnedViews;
            for (int frame = 13; frame <= 16; frame++)
            using (var image = SmallObjectHdFrame(frame, true))
            {
                Check(!tracker.Update(image, CancellationToken.None).Found && tracker.LearnedViews == learned,
                    "A covering foreground object was accepted or learned as the selected HD target");
            }
            using (var low = TurningObject(true, 0))
            {
                tracker.Initialize(low, new RectangleF(225, 135, 80, 170), CancellationToken.None);
                Check(tracker.ProcessingImageSize == low.Size, "Reselecting a 640-pixel source upscaled it");
            }
            using (var low = TurningObject(false, 0))
            using (var large = new Bitmap(1920, 1440))
            using (var graphics = Graphics.FromImage(large))
            {
                graphics.DrawImage(low, 0, 0, large.Width, large.Height);
                tracker.Initialize(large, new RectangleF(540, 315, 510, 600), CancellationToken.None);
                Check(tracker.ProcessingImageSize == low.Size,
                    "Reselecting a large HD target retained the previous selection's expanded pixel budget");
            }
            tracker.Dispose();
            Check(tracker.ProcessingImageSize.IsEmpty, "Retiring the selection retained its processing image state");
            Console.WriteLine("HD thin Object: 12 verified moving frames; max " + maximum + " ms, 1280x720 budget.");
        }
        using (var high = new Bitmap(1920, 1080))
        using (var normal = VisualTrackingImage.Color(high))
        using (var detailed = VisualTrackingImage.Color(high, 1280))
        using (var low = new Bitmap(320, 240))
        using (var unchanged = VisualTrackingImage.Color(low, 1280))
        {
            Check(normal.Width == 640 && normal.Height == 360 && detailed.Width == 1280 && detailed.Height == 720 &&
                unchanged.Width == low.Width && unchanged.Height == low.Height,
                "Tracking image budgets changed the default Area scale or invented pixels by upscaling");
            Throws<ArgumentOutOfRangeException>(() => VisualTrackingImage.Color(high, 1281),
                "Object processing accepted an unbounded image dimension");
        }
    }

    private static Bitmap SmallObjectHdFrame(int frame, bool blocked = false)
    {
        var image = new Bitmap(1920, 1080);
        using (var graphics = Graphics.FromImage(image))
        {
            int light = frame == 0 ? 70 : frame % 2 == 0 ? 215 : 25;
            graphics.Clear(Color.FromArgb(light, light, light));
            for (int y = 0; y < image.Height; y += 48)
            for (int x = 0; x < image.Width; x += 48)
            {
                int value = light + ((x + y + frame * 48) % 96 == 0 ? 12 : -12);
                using (var brush = new SolidBrush(Color.FromArgb(value, value, value)))
                { graphics.FillRectangle(brush, x, y, 48, 48); }
            }
            float cx = 960 + frame * 6;
            graphics.SmoothingMode = SmoothingMode.AntiAlias;
            graphics.TranslateTransform(cx, 540); graphics.RotateTransform(frame * 1.5f);
            using (var body = new SolidBrush(Color.FromArgb(180, 140, 30)))
            { graphics.FillRectangle(body, -8, -60, 16, 120); }
            graphics.ResetTransform();
            if (blocked) { graphics.FillRectangle(Brushes.DarkSlateGray, cx - 60, 440, 120, 200); }
        }
        return image;
    }

    private static void ForegroundGeometryNeedsTheObject()
    {
        using (var background = new OpenCvSharp.Mat(300, 400, OpenCvSharp.MatType.CV_8UC3))
        using (var source = new OpenCvSharp.Mat())
        using (var positive = new OpenCvSharp.Mat(300, 400, OpenCvSharp.MatType.CV_8UC3, new OpenCvSharp.Scalar(90, 40, 120)))
        using (var mask = new OpenCvSharp.Mat(300, 400, OpenCvSharp.MatType.CV_8UC1, OpenCvSharp.Scalar.All(0)))
        using (var tile = new OpenCvSharp.Mat(96, 96, OpenCvSharp.MatType.CV_8UC3))
        {
            var random = new Random(271);
            foreach (var image in new[] { background, tile })
            for (int y = 0; y < image.Height; y++)
            for (int x = 0; x < image.Width; x++)
            { image.Set(y, x, new OpenCvSharp.Vec3b((byte)random.Next(256), (byte)random.Next(256), (byte)random.Next(256))); }
            background.CopyTo(source);
            var objectBox = new OpenCvSharp.Rect(152, 102, 96, 96);
            using (var a = new OpenCvSharp.Mat(source, objectBox)) { tile.CopyTo(a); }
            var moved = new OpenCvSharp.Rect(172, 112, 96, 96);
            using (var b = new OpenCvSharp.Mat(positive, moved)) { tile.CopyTo(b); }
            OpenCvSharp.Cv2.Rectangle(mask, objectBox, OpenCvSharp.Scalar.All(255), -1);
            var selection = new OpenCvSharp.Rect(120, 70, 160, 160);
            using (var appearance = new FrozenAppearance(source, selection, mask))
            using (var reference = new ObjectRecoveryReference(source, selection, appearance, mask))
            {
                Check(reference.CanLocate, "Foreground/context geometry fixture has no usable identity");
                Check(reference.Find(positive, CancellationToken.None).HasValue,
                    "A changed background prevented foreground geometry recovery");
                Check(!reference.Find(background, CancellationToken.None).HasValue,
                    "Unchanged surrounding features established identity after the foreground was removed");
                Check(!reference.Refine(source, new OpenCvSharp.Rect(800, 700, 30, 30), CancellationToken.None).HasValue,
                    "A completely outside native proposal did not fail safely");
            }
        }
    }

    private static void RejectOccluderAfterLearning()
    {
        foreach (bool similarColour in new[] { false, true })
        using (var first = TurningObject(false, 0))
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
        {
            tracker.Initialize(first, new RectangleF(180, 105, 170, 200), CancellationToken.None);
            // Segmentation refreshes every fourth frame between ViTTrack proposals; three fresh views promote one.
            for (int frame = 1; frame <= 16; frame++)
            using (var image = TurningObject(false, frame))
            { Check(tracker.Update(image, CancellationToken.None).Found, "Pre-occlusion foreground was never verified"); }
            int learned = ((ObjectVisualTracker)tracker).LearnedViews;
            Check(learned > 0, "Occluder test did not exercise an adapted appearance");
            for (int frame = 17; frame <= 20; frame++)
            using (var image = TurningObject(false, frame))
            using (var graphics = Graphics.FromImage(image))
            {
                using (var obstruction = new SolidBrush(similarColour ? Color.FromArgb(190, 143, 102) : Color.DarkSlateGray))
                { graphics.FillRectangle(obstruction, 265 + frame * 3 - 44, 164, 88, 90); }
                Check(!tracker.Update(image, CancellationToken.None).Found &&
                    ((ObjectVisualTracker)tracker).LearnedViews == learned,
                    "An aiming-point occluder was accepted or learned after foreground adaptation");
            }
        }
    }

    private static void ThinForegroundSupport()
    {
        foreach (int width in new[] { 2, 4, 6 })
        using (var image = new OpenCvSharp.Mat(480, 640, OpenCvSharp.MatType.CV_8UC3, new OpenCvSharp.Scalar(40, 40, 40)))
        {
            OpenCvSharp.Cv2.Rectangle(image, new OpenCvSharp.Rect(250, 150, width, 120), new OpenCvSharp.Scalar(30, 120, 180), -1);
            using (var target = ObjectForeground.TryExtract(image, new Rectangle(230, 135, 50, 150),
                new PointF(250 + width / 2f, 210), null, CancellationToken.None))
            {
                Check(target != null, "Thin target foreground failed before mask support validation");
                Check(target.Mask.Width == image.Width && target.Mask.Height == image.Height &&
                    OpenCvSharp.Cv2.CountNonZero(target.Mask) == target.Area,
                    "Thin foreground mask lost source coordinates or admitted surrounding background");
            }
        }
    }

    private static void RejectRemovedThinObject()
    {
        using (var background = new Bitmap(640, 480))
        using (var graphics = Graphics.FromImage(background))
        {
            graphics.Clear(Color.FromArgb(130, 130, 130));
            for (int y = 0; y < 480; y += 6)
            for (int x = 0; x < 640; x += 6)
            { graphics.FillRectangle((x + y) % 12 == 0 ? Brushes.DimGray : Brushes.LightGray, x, y, 2, 2); }
            using (var reference = new Bitmap(background))
            using (var paint = Graphics.FromImage(reference))
            using (var tracker = VisualTrackerFactory.Create(CancellationToken.None))
            {
                paint.FillRectangle(Brushes.Gray, 260, 150, 10, 140);
                tracker.Initialize(reference, new RectangleF(225, 135, 80, 170), CancellationToken.None);
                // This near-colour scene is ambiguous; the coloured rotating shaft above is the paired positive.
                tracker.Update(reference, CancellationToken.None);
                int learned = ((ObjectVisualTracker)tracker).LearnedViews;
                for (int frame = 0; frame < 5; frame++)
                {
                    var result = tracker.Update(background, CancellationToken.None);
                    Check(!result.Found && ((ObjectVisualTracker)tracker).LearnedViews == learned,
                        "The unchanged near-colour background replaced the removed shaft or was learned as identity");
                }
            }
        }
    }

    private static Bitmap TurningObject(bool pencil, int frame)
    {
        var image = new Bitmap(640, 480);
        using (var graphics = Graphics.FromImage(image))
        {
            int light = frame == 0 ? 70 : frame % 2 == 0 ? 215 : 25;
            graphics.Clear(Color.FromArgb(light, light, light));
            for (int y = 0; y < 480; y += 24)
            for (int x = 0; x < 640; x += 24)
            {
                int value = Math.Max(0, Math.Min(255, light + ((x + y + frame * 24) % 48 == 0 ? 18 : -18)));
                using (var brush = new SolidBrush(Color.FromArgb(value, value, value)))
                { graphics.FillRectangle(brush, x, y, 24, 24); }
            }
            graphics.SmoothingMode = SmoothingMode.AntiAlias;
            float cx = 265 + frame * 3;
            if (pencil)
            {
                graphics.TranslateTransform(cx, 220); graphics.RotateTransform(frame * 2);
                int shade = 100 + frame * 3;
                using (var body = new SolidBrush(Color.FromArgb(180, shade, 30)))
                { graphics.FillRectangle(body, -5, -70, 10, 140); }
                graphics.DrawLine(Pens.Black, -2, -60, -2, 60);
                graphics.FillPolygon(Brushes.Black, new[] { new PointF(-5, 70), new PointF(5, 70), new PointF(0, 82) });
            }
            else
            {
                float width = 110 - frame * 1.1f;
                using (var skin = new SolidBrush(Color.FromArgb(190, 143, 102)))
                { graphics.FillEllipse(skin, cx - width / 2, 130, width, 150); }
                graphics.FillEllipse(Brushes.DarkSlateGray, cx - width / 2, 125, width, 38);
                using (var face = new GraphicsPath())
                {
                    face.AddEllipse(cx - width / 2, 130, width, 150);
                    graphics.SetClip(face);
                    float turn = frame * 1.2f;
                    graphics.FillEllipse(Brushes.Black, cx - 25 + turn * .4f, 176, 9, 7);
                    graphics.FillEllipse(Brushes.Black, cx + 21 + turn * .8f, 176, 9, 7);
                    graphics.DrawLine(Pens.SaddleBrown, cx + turn, 185, cx + turn + 5, 220);
                    graphics.DrawArc(Pens.Maroon, cx - 22 + turn * .5f, 230, 45, 18, 0, 180);
                    for (int dot = 0; dot < 10; dot++)
                    { graphics.FillEllipse(Brushes.SaddleBrown, cx - 32 + dot * 6, 210 + (dot % 3) * 6, 2, 2); }
                    graphics.ResetClip();
                }
            }
        }
        return image;
    }
}
