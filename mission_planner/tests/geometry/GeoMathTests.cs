// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// GeoMath unit tests
// ============================================================
// Compiled together with src/Geofence/GeoMath.cs and
// GeofenceTypes.cs by scripts/build/test_plugin_geometry.ps1
// (plain csc, no test framework — exits non-zero on failure).
// Run via `pixi run test-plugin-geometry`.
// ============================================================

using System;
using System.Collections.Generic;
using NOMAD.MissionPlanner;

internal static class GeoMathTests
{
    private static int _failures;
    private const double LAT0 = 45.0;
    private const double LON0 = -75.0;
    private static readonly double M_PER_DEG_LAT = 110540.0;
    private static readonly double M_PER_DEG_LON = 111320.0 * Math.Cos(LAT0 * Math.PI / 180.0);

    private static int Main()
    {
        InsetSquareMovesEachVertexInward();
        InsetIsWindingAgnostic();
        InsetPreservesVertexCount();
        OversizedInsetFallsBackToShrink();
        IsInsideBasicSquare();
        FewerThanThreeVerticesCountsAsInside();
        AdvisoryOutlineStatusUsesVisualPolygonsOnly();
        AdvisoryAltitudeStatusUsesConfiguredThreshold();

        Console.WriteLine(_failures == 0
            ? "All geometry tests passed."
            : $"{_failures} geometry test(s) FAILED.");
        return _failures == 0 ? 0 : 1;
    }

    // ============================================================
    // Test cases
    // ============================================================

    private static void InsetSquareMovesEachVertexInward()
    {
        // 100 m square, inset 5 m: every vertex of the result must sit
        // 5 m inside both adjacent edges, i.e. at (5,5), (5,95), ...
        var square = Square(100);
        var inset = GeoMath.InsetPolygon(square, 5.0);

        var expected = new[]
        {
            (5.0, 5.0), (95.0, 5.0), (95.0, 95.0), (5.0, 95.0)
        };
        for (int i = 0; i < 4; i++)
        {
            var (ex, ey) = expected[i];
            var (ax, ay) = ToLocal(inset[i]);
            AssertNear(ax, ex, 0.05, $"inset square vertex {i} x");
            AssertNear(ay, ey, 0.05, $"inset square vertex {i} y");
        }
    }

    private static void InsetIsWindingAgnostic()
    {
        var ccw = Square(100);
        var cw = new List<GpsPoint>(ccw);
        cw.Reverse();

        var insetCcw = GeoMath.InsetPolygon(ccw, 8.0);
        var insetCw = GeoMath.InsetPolygon(cw, 8.0);

        // Same vertex set regardless of winding (order differs; compare by
        // containment: every CW-inset vertex appears in the CCW-inset set).
        foreach (var v in insetCw)
        {
            bool found = false;
            foreach (var w in insetCcw)
            {
                var (vx, vy) = ToLocal(v);
                var (wx, wy) = ToLocal(w);
                if (Math.Abs(vx - wx) < 0.05 && Math.Abs(vy - wy) < 0.05) { found = true; break; }
            }
            Assert(found, "winding-agnostic inset: CW vertex matches a CCW vertex");
        }
    }

    private static void InsetPreservesVertexCount()
    {
        var pentagon = new List<GpsPoint>
        {
            Pt(0, 50), Pt(48, 15), Pt(29, -40), Pt(-29, -40), Pt(-48, 15)
        };
        var inset = GeoMath.InsetPolygon(pentagon, 5.0);
        Assert(inset.Count == 5, "inset preserves vertex count");
        foreach (var v in inset)
        {
            Assert(GeoMath.IsInside(pentagon, v), "inset pentagon vertex lies inside original");
        }
    }

    private static void OversizedInsetFallsBackToShrink()
    {
        // 60 m inset on a 100 m square would invert the polygon — the
        // fallback must still return points strictly inside the original.
        var square = Square(100);
        var inset = GeoMath.InsetPolygon(square, 60.0);

        Assert(inset.Count == 4, "oversized inset preserves vertex count");
        foreach (var v in inset)
        {
            Assert(GeoMath.IsInside(square, v), "oversized inset vertex stays inside original");
        }
    }

    private static void IsInsideBasicSquare()
    {
        var square = Square(100);
        Assert(GeoMath.IsInside(square, Pt(50, 50)), "center is inside");
        Assert(!GeoMath.IsInside(square, Pt(150, 50)), "east of square is outside");
        Assert(!GeoMath.IsInside(square, Pt(-1, -1)), "southwest corner exterior is outside");
    }

    private static void FewerThanThreeVerticesCountsAsInside()
    {
        Assert(GeoMath.IsInside(null, Pt(0, 0)), "null polygon counts as inside (no boundary)");
        Assert(GeoMath.IsInside(new List<GpsPoint> { Pt(0, 0), Pt(10, 0) }, Pt(500, 500)),
            "2-vertex polygon counts as inside (no boundary)");
    }

    private static void AdvisoryOutlineStatusUsesVisualPolygonsOnly()
    {
        var outer = Square(100);
        var inner = Square(60);

        Assert(GeoMath.GetAdvisoryOutlineStatus(inner, outer, Pt(30, 30)) ==
            AdvisoryOutlineStatus.InsideOutlines, "position within both advisory outlines");
        Assert(GeoMath.GetAdvisoryOutlineStatus(inner, outer, Pt(80, 50)) ==
            AdvisoryOutlineStatus.OutsideInnerOutline, "position between advisory outlines");
        Assert(GeoMath.GetAdvisoryOutlineStatus(inner, outer, Pt(110, 50)) ==
            AdvisoryOutlineStatus.OutsideOuterOutline, "position outside outer advisory outline");
        Assert(GeoMath.GetAdvisoryOutlineStatus(null, null, Pt(30, 30)) ==
            AdvisoryOutlineStatus.NoOutline, "no configured outline has no visual status");
        Assert(GeoMath.GetAdvisoryOutlineStatus(null, outer, null) ==
            AdvisoryOutlineStatus.NoPosition, "missing telemetry has no visual status");
    }

    private static void AdvisoryAltitudeStatusUsesConfiguredThreshold()
    {
        Assert(AdvisoryAltitudeStatus.IsAboveThreshold(91, 90),
            "configured 90m advisory threshold colors 91m as above threshold");
        Assert(!AdvisoryAltitudeStatus.IsAboveThreshold(90, 90),
            "altitude equal to configured advisory threshold is not above it");
        Assert(!AdvisoryAltitudeStatus.IsAboveThreshold(89, 90),
            "altitude below configured advisory threshold remains normal");
        Assert(!AdvisoryAltitudeStatus.IsAboveThreshold(double.NaN, 90),
            "invalid telemetry does not create an above-threshold status");
    }

    // ============================================================
    // Helpers
    // ============================================================

    /// <summary>Square with corners (0,0)..(size,size) in local meters, CCW.</summary>
    private static List<GpsPoint> Square(double size) => new List<GpsPoint>
    {
        Pt(0, 0), Pt(size, 0), Pt(size, size), Pt(0, size)
    };

    /// <summary>GpsPoint from local meters east (x) / north (y) of the test origin.</summary>
    private static GpsPoint Pt(double xMeters, double yMeters) =>
        new GpsPoint(LAT0 + yMeters / M_PER_DEG_LAT, LON0 + xMeters / M_PER_DEG_LON);

    private static (double X, double Y) ToLocal(GpsPoint p) =>
        ((p.Lon - LON0) * M_PER_DEG_LON, (p.Lat - LAT0) * M_PER_DEG_LAT);

    private static void Assert(bool condition, string name)
    {
        if (condition)
        {
            Console.WriteLine($"  PASS  {name}");
        }
        else
        {
            Console.WriteLine($"  FAIL  {name}");
            _failures++;
        }
    }

    private static void AssertNear(double actual, double expected, double tolerance, string name)
    {
        Assert(Math.Abs(actual - expected) <= tolerance,
            $"{name} (expected {expected:F2} ± {tolerance}, got {actual:F2})");
    }
}
