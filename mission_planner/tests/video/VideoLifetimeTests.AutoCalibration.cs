// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void CameraDelayProbeRules()
    {
        using (var dark = new Bitmap(64, 48))
        using (var light = new Bitmap(64, 48))
        {
            using (var g = Graphics.FromImage(dark)) { g.Clear(Color.FromArgb(20, 20, 20)); }
            using (var g = Graphics.FromImage(light)) { g.Clear(Color.FromArgb(220, 220, 220)); }
            var a = CameraDelayProbe.Sample(dark);
            var b = CameraDelayProbe.Sample(light);
            Check(a.Length == CameraDelayProbe.Columns * CameraDelayProbe.Rows, "Sample grid size changed");
            Check(CameraDelayProbe.Difference(a, a) == 0, "Identical frames differ");
            Check(Math.Abs(CameraDelayProbe.Difference(a, b) - 200) < 1, "Whole-scene change not measured");
        }
        Check(CameraDelayProbe.ChangeThreshold(new[] { 0f, .5f }) == 6.25f, "Quiet scene threshold wrong");
        Check(CameraDelayProbe.ChangeThreshold(new[] { 5f, 7f }) == 14, "Noisy scene threshold not 2× max");
        Check(CameraDelayProbe.Estimate(new[] { .30, .10, .20, .90 }, 50) == 200, "Median less latency wrong");
        Check(CameraDelayProbe.Estimate(new[] { .01 }, 200) == 0, "Negative delay not clamped");
        Check(CameraDelayProbe.Estimate(new[] { 5.0 }, 0) == 2000, "Large delay not clamped");
        Check(CameraDelayProbe.StepTarget(-45, -90, 30) == -35, "Step not toward range");
        Check(CameraDelayProbe.StepTarget(25, -90, 30) == 15, "Step at upper limit not reversed");
        Check(CameraDelayProbe.StepTarget(-85, -90, -80) == -90, "Narrow range step not clamped");
    }

    private static void GroundHeightFromTelemetry()
    {
        const double tilted = 40 * Math.PI / 180;
        Check(VideoMotionHistory.GroundHeightAboveHome(50, 45, 0, 0, 130, 100) == 5, "Rangefinder not preferred");
        Check(VideoMotionHistory.GroundHeightAboveHome(50, 45, tilted, 0, 130, 100) == 30,
            "Tilted rangefinder not replaced by terrain");
        Check(VideoMotionHistory.GroundHeightAboveHome(50, null, 0, 0, null, 100) == 0, "Missing data not HOME level");
        Check(VideoMotionHistory.GroundHeightAboveHome(50, null, 0, 0, double.NaN, 100) == 0, "NaN terrain used");
        Check(VideoMotionHistory.GroundHeightAboveHome(50, null, 0, 0, 20000, 100) == 0, "Absurd terrain used");
    }
}
