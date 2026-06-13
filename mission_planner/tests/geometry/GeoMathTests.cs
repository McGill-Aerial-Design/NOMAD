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
        NearestPointInsideTargetsNearestEdge();
        NearestPointInsideFromCornerIsInside();
        NearestPointInsideNullWithoutPolygon();

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

    private static void NearestPointInsideTargetsNearestEdge()
    {
        // Drone 10 m east of the square's east edge, halfway up: nearest edge
        // point is (100, 50); with a 1 m margin the target is (99, 50) — not
        // anywhere near the centroid (50, 50).
        var square = Square(100);
        var target = GeoMath.NearestPointInside(square, Pt(110, 50), 1.0);

        Assert(target != null, "nearest-inside returns a point");
        var (tx, ty) = ToLocal(target);
        AssertNear(tx, 99.0, 0.1, "nearest-inside x (1 m inside east edge)");
        AssertNear(ty, 50.0, 0.1, "nearest-inside y (level with drone)");
        Assert(GeoMath.IsInside(square, target), "nearest-inside target is inside polygon");
    }

    private static void NearestPointInsideFromCornerIsInside()
    {
        // Outside the corner diagonally: closest outline point is the corner
        // vertex itself; the margin step must still land inside.
        var square = Square(100);
        var target = GeoMath.NearestPointInside(square, Pt(110, 110), 1.0);

        Assert(target != null, "corner nearest-inside returns a point");
        Assert(GeoMath.IsInside(square, target), "corner nearest-inside target is inside polygon");
        var (tx, ty) = ToLocal(target);
        AssertNear(tx, 99.3, 0.5, "corner target near (99.3, 99.3)");
        AssertNear(ty, 99.3, 0.5, "corner target near (99.3, 99.3)");
    }

    private static void NearestPointInsideNullWithoutPolygon()
    {
        Assert(GeoMath.NearestPointInside(null, Pt(0, 0), 1.0) == null,
            "nearest-inside null for null polygon");
        Assert(GeoMath.NearestPointInside(new List<GpsPoint> { Pt(0, 0) }, Pt(5, 5), 1.0) == null,
            "nearest-inside null for degenerate polygon");
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
