// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using GMap.NET;

namespace NOMAD.MissionPlanner
{
    internal static class FenceCoordinates
    {
        internal static bool TryParse(string input, bool longitudeFirst,
            out List<PointLatLng> points, out string error)
        {
            points = new List<PointLatLng>();
            var lines = (input ?? "").Replace("\r\n", "\n").Replace('\r', '\n').Split('\n');
            for (int index = 0; index < lines.Length; index++)
            {
                string line = lines[index].Trim();
                if (line.Length == 0) { continue; }
                var parts = line.Contains(",") ? line.Split(',') :
                    line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                error = $"Line {index + 1}: enter exactly two finite numbers using . for decimals.";
                if (parts.Length != 2 ||
                    !double.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out double first) ||
                    !double.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out double second))
                {
                    points = null;
                    return false;
                }
                points.Add(new PointLatLng(longitudeFirst ? second : first, longitudeFirst ? first : second));
            }
            if (points.Count > 1 && points[0] == points[points.Count - 1]) { points.RemoveAt(points.Count - 1); }
            if (Validate(points, out error)) { return true; }
            points = null;
            return false;
        }

        internal static bool Validate(IReadOnlyList<PointLatLng> points, out string error)
        {
            error = "Enter one polygon with 3 to 255 distinct vertices.";
            if (points == null || points.Count < 3 || points.Count > 255) { return false; }
            error = "Latitude must be -90 to 90 and longitude -180 to 180, both finite.";
            if (points.Any(p => double.IsNaN(p.Lat) || double.IsNaN(p.Lng) ||
                double.IsInfinity(p.Lat) || double.IsInfinity(p.Lng) ||
                p.Lat < -90 || p.Lat > 90 || p.Lng < -180 || p.Lng > 180)) { return false; }
            error = "Use a nonzero-area polygon without repeated vertices, crossings or longitude-seam edges.";
            var seen = new HashSet<PointLatLng>();
            decimal area = 0;
            for (int index = 0; index < points.Count; index++)
            {
                var a = points[index];
                var b = points[(index + 1) % points.Count];
                if (!seen.Add(a) || Math.Abs(a.Lng - b.Lng) > 180) { return false; }
                area += (decimal)a.Lat * (decimal)b.Lng - (decimal)b.Lat * (decimal)a.Lng;
                for (int other = index + 1; other < points.Count; other++)
                {
                    if (other == index + 1 || (index == 0 && other == points.Count - 1)) { continue; }
                    if (Intersects(a, b, points[other], points[(other + 1) % points.Count])) { return false; }
                }
            }
            if (area == 0) { return false; }
            error = null;
            return true;
        }

        private static decimal Cross(PointLatLng a, PointLatLng b, PointLatLng c) =>
            ((decimal)b.Lat - (decimal)a.Lat) * ((decimal)c.Lng - (decimal)a.Lng) -
            ((decimal)b.Lng - (decimal)a.Lng) * ((decimal)c.Lat - (decimal)a.Lat);

        private static bool Intersects(PointLatLng a, PointLatLng b, PointLatLng c, PointLatLng d)
        {
            if (Math.Max(a.Lat, b.Lat) < Math.Min(c.Lat, d.Lat) ||
                Math.Max(c.Lat, d.Lat) < Math.Min(a.Lat, b.Lat) ||
                Math.Max(a.Lng, b.Lng) < Math.Min(c.Lng, d.Lng) ||
                Math.Max(c.Lng, d.Lng) < Math.Min(a.Lng, b.Lng)) { return false; }
            return Math.Sign(Cross(a, b, c)) * Math.Sign(Cross(a, b, d)) <= 0 &&
                Math.Sign(Cross(c, d, a)) * Math.Sign(Cross(c, d, b)) <= 0;
        }
    }
}
