// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Geofence Geometry
// ============================================================
// Pure polygon math for the geofence subsystem: point-in-polygon,
// inward polygon offset (derived soft boundary), and nearest-
// inside-point (soft-boundary return target). No Mission Planner
// dependencies — unit-tested standalone via
// `pixi run test-plugin-geometry`.
//
// All functions work in a local equirectangular approximation,
// which is accurate to well under 0.1% for boundary-sized
// (sub-kilometer) polygons.
// ============================================================

using System;
using System.Collections.Generic;

namespace NOMAD.MissionPlanner
{
    public static class GeoMath
    {
        /// <summary>
        /// Ray-casting point-in-polygon test on raw lat/lon.
        /// Fewer than 3 vertices counts as "inside" (no boundary defined).
        /// </summary>
        public static bool IsInside(List<GpsPoint> vertices, GpsPoint point)
        {
            if (vertices == null || vertices.Count < 3)
                return true;

            int n = vertices.Count;
            bool inside = false;

            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                var vi = vertices[i];
                var vj = vertices[j];

                if (((vi.Lon > point.Lon) != (vj.Lon > point.Lon)) &&
                    (point.Lat < (vj.Lat - vi.Lat) * (point.Lon - vi.Lon) / (vj.Lon - vi.Lon) + vi.Lat))
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        /// <summary>
        /// Offset a polygon inward by <paramref name="meters"/>: each edge is
        /// shifted along its inward normal and adjacent shifted edges are
        /// intersected (winding-agnostic). If the inset is too large for the
        /// polygon (area would invert or grow — e.g. miter blowup at a sharp
        /// corner), falls back to pulling each vertex toward the centroid by
        /// the same distance (capped at 90% of the way).
        /// </summary>
        public static List<GpsPoint> InsetPolygon(List<GpsPoint> verts, double meters)
        {
            int n = verts.Count;
            double lat0 = verts[0].Lat;
            double mPerDegLat = 110540.0;
            double mPerDegLon = 111320.0 * Math.Cos(lat0 * Math.PI / 180.0);

            var x = new double[n];
            var y = new double[n];
            for (int i = 0; i < n; i++)
            {
                x[i] = (verts[i].Lon - verts[0].Lon) * mPerDegLon;
                y[i] = (verts[i].Lat - lat0) * mPerDegLat;
            }

            // Winding via shoelace: positive = CCW, whose interior is to the
            // left of each edge. Scale the offset by the sign so the left
            // normal always points inward.
            double area2 = Shoelace(x, y);
            double d = meters * (area2 >= 0 ? 1.0 : -1.0);

            var outX = new double[n];
            var outY = new double[n];
            for (int i = 0; i < n; i++)
            {
                int prev = (i + n - 1) % n;
                int next = (i + 1) % n;
                OffsetVertex(x[prev], y[prev], x[i], y[i], x[next], y[next], d, out outX[i], out outY[i]);
            }

            // Sanity: a valid inset keeps orientation and shrinks. Sharp
            // corners (miter blowup) or an inset bigger than the polygon
            // violate this — use the centroid-pull fallback instead.
            double newArea2 = Shoelace(outX, outY);
            if (Math.Sign(newArea2) != Math.Sign(area2) || Math.Abs(newArea2) >= Math.Abs(area2))
            {
                double cx = 0, cy = 0;
                for (int i = 0; i < n; i++) { cx += x[i]; cy += y[i]; }
                cx /= n; cy /= n;
                for (int i = 0; i < n; i++)
                {
                    double dx = cx - x[i], dy = cy - y[i];
                    double dist = Math.Sqrt(dx * dx + dy * dy);
                    double t = dist < 1e-6 ? 0 : Math.Min(0.9, meters / dist);
                    outX[i] = x[i] + dx * t;
                    outY[i] = y[i] + dy * t;
                }
            }

            var result = new List<GpsPoint>(n);
            for (int i = 0; i < n; i++)
            {
                result.Add(new GpsPoint(lat0 + outY[i] / mPerDegLat, verts[0].Lon + outX[i] / mPerDegLon));
            }
            return result;
        }

        /// <summary>
        /// Closest point on the polygon outline to <paramref name="pos"/>,
        /// nudged <paramref name="marginM"/> meters toward the polygon centroid
        /// so the target sits just inside. Null without a valid polygon.
        /// </summary>
        public static GpsPoint NearestPointInside(List<GpsPoint> verts, GpsPoint pos, double marginM)
        {
            if (verts == null || verts.Count < 3) return null;

            // Local equirectangular frame centered on the drone.
            double mPerDegLat = 110540.0;
            double mPerDegLon = 111320.0 * Math.Cos(pos.Lat * Math.PI / 180.0);

            int n = verts.Count;
            var x = new double[n];
            var y = new double[n];
            double cx = 0, cy = 0;
            for (int i = 0; i < n; i++)
            {
                x[i] = (verts[i].Lon - pos.Lon) * mPerDegLon;
                y[i] = (verts[i].Lat - pos.Lat) * mPerDegLat;
                cx += x[i];
                cy += y[i];
            }
            cx /= n;
            cy /= n;

            // Closest point on any edge to the drone (the local origin).
            double bestD2 = double.MaxValue, bx = 0, by = 0;
            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                double ex = x[j] - x[i], ey = y[j] - y[i];
                double len2 = ex * ex + ey * ey;
                double t = len2 < 1e-9 ? 0 : Math.Max(0, Math.Min(1, -(x[i] * ex + y[i] * ey) / len2));
                double px = x[i] + t * ex, py = y[i] + t * ey;
                double d2 = px * px + py * py;
                if (d2 < bestD2) { bestD2 = d2; bx = px; by = py; }
            }

            // Step off the edge toward the interior (centroid direction).
            double dx = cx - bx, dy = cy - by;
            double dist = Math.Sqrt(dx * dx + dy * dy);
            if (dist > 1e-6)
            {
                bx += dx / dist * marginM;
                by += dy / dist * marginM;
            }

            return new GpsPoint(pos.Lat + by / mPerDegLat, pos.Lon + bx / mPerDegLon);
        }

        private static double Shoelace(double[] x, double[] y)
        {
            double a = 0;
            for (int i = 0; i < x.Length; i++)
            {
                int j = (i + 1) % x.Length;
                a += x[i] * y[j] - x[j] * y[i];
            }
            return a;
        }

        /// <summary>
        /// New position of vertex B after shifting edges A→B and B→C by their
        /// left normals * d and intersecting the two shifted lines.
        /// </summary>
        private static void OffsetVertex(double ax, double ay, double bx, double by,
            double cx, double cy, double d, out double ox, out double oy)
        {
            double d1x = bx - ax, d1y = by - ay;
            double d2x = cx - bx, d2y = cy - by;
            double l1 = Math.Sqrt(d1x * d1x + d1y * d1y);
            double l2 = Math.Sqrt(d2x * d2x + d2y * d2y);
            if (l1 < 1e-9 || l2 < 1e-9)
            {
                ox = bx; oy = by;
                return;
            }
            double n1x = -d1y / l1, n1y = d1x / l1;   // left normal of A→B
            double n2x = -d2y / l2, n2y = d2x / l2;   // left normal of B→C

            double denom = d1x * d2y - d1y * d2x;
            if (Math.Abs(denom) < 1e-9)
            {
                // Collinear edges: just shift the vertex along the normal.
                ox = bx + n1x * d;
                oy = by + n1y * d;
                return;
            }

            double p1x = ax + n1x * d, p1y = ay + n1y * d; // shifted A→B line
            double p2x = bx + n2x * d, p2y = by + n2y * d; // shifted B→C line
            double t = ((p2x - p1x) * d2y - (p2y - p1y) * d2x) / denom;
            ox = p1x + t * d1x;
            oy = p1y + t * d1y;
        }
    }
}
