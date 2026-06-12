// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Geofence Configuration
// ============================================================
// General, reusable flight-geofence and failsafe configuration.
// Holds soft/hard boundary polygons, altitude limits, failsafe
// behavior, and boundary-violation checking. Task-agnostic.
// ============================================================

using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// GPS coordinate (lat/lon with optional altitude).
    /// </summary>
    public class GpsPoint
    {
        public double Lat { get; set; }
        public double Lon { get; set; }
        public double? Alt { get; set; }

        public GpsPoint() { }

        public GpsPoint(double lat, double lon, double? alt = null)
        {
            Lat = lat;
            Lon = lon;
            Alt = alt;
        }

        public override string ToString() => Alt.HasValue
            ? $"{Lat:F6}, {Lon:F6} @ {Alt:F1}m"
            : $"{Lat:F6}, {Lon:F6}";
    }

    /// <summary>
    /// Flight boundary polygon (soft or hard).
    /// </summary>
    public class FlightBoundary
    {
        /// <summary>
        /// Boundary name (e.g., "Soft Boundary", "Hard Boundary").
        /// </summary>
        public string Name { get; set; } = "Boundary";

        /// <summary>
        /// Boundary type: "soft" = warning, "hard" = kill required.
        /// </summary>
        public string BoundaryType { get; set; } = "soft";

        /// <summary>
        /// Polygon vertices in order (lat/lon).
        /// </summary>
        public List<GpsPoint> Vertices { get; set; } = new List<GpsPoint>();

        /// <summary>
        /// Color for display (ARGB hex).
        /// </summary>
        public string DisplayColor { get; set; } = "#FFFF00"; // Yellow for soft

        /// <summary>
        /// Maximum altitude AGL in meters (null = no limit).
        /// </summary>
        public double? MaxAltitudeAgl { get; set; } = 122.0; // 400ft default

        /// <summary>
        /// Minimum altitude AGL in meters.
        /// </summary>
        public double MinAltitudeAgl { get; set; } = 0.0;
    }

    /// <summary>
    /// A logged boundary violation event.
    /// </summary>
    public class BoundaryViolation
    {
        public DateTime Timestamp { get; set; }
        public string BoundaryName { get; set; }
        public string BoundaryType { get; set; }
        public GpsPoint DronePosition { get; set; }
        public string Action { get; set; } // "warning", "kill_required"
        public bool Acknowledged { get; set; }
    }

    /// <summary>
    /// Failsafe behavior configuration.
    /// </summary>
    public class FailsafeBehavior
    {
        /// <summary>
        /// Action when crossing soft boundary.
        /// Options: "warn_audio", "warn_visual", "warn_both", "return_to_boundary".
        /// </summary>
        public string SoftBoundaryAction { get; set; } = "warn_both";

        /// <summary>
        /// Action when crossing hard boundary.
        /// Options: "warn_and_kill", "auto_kill", "warn_only".
        /// </summary>
        public string HardBoundaryAction { get; set; } = "warn_and_kill";

        /// <summary>
        /// Seconds to wait before auto-kill after a hard boundary violation.
        /// </summary>
        public int HardBoundaryKillDelaySec { get; set; } = 10;

        /// <summary>
        /// Action on communication loss.
        /// </summary>
        public string CommLossAction { get; set; } = "hover_and_wait";

        /// <summary>
        /// Enable audible warnings.
        /// </summary>
        public bool EnableAudioWarnings { get; set; } = true;

        /// <summary>
        /// Enable visual overlay warnings.
        /// </summary>
        public bool EnableVisualWarnings { get; set; } = true;
    }

    /// <summary>
    /// General geofence + failsafe configuration with boundary checking and
    /// JSON persistence. Task-agnostic; adapt as needed per deployment.
    /// </summary>
    public class GeofenceConfig
    {
        /// <summary>
        /// Soft flight boundary (yellow - warning).
        /// </summary>
        public FlightBoundary SoftBoundary { get; set; } = new FlightBoundary
        {
            Name = "Soft Boundary",
            BoundaryType = "soft",
            DisplayColor = "#FFFF00"
        };

        /// <summary>
        /// Hard flight boundary (red - kill required).
        /// </summary>
        public FlightBoundary HardBoundary { get; set; } = new FlightBoundary
        {
            Name = "Hard Boundary",
            BoundaryType = "hard",
            DisplayColor = "#FF0000"
        };

        /// <summary>
        /// Return point for geofence breach (centroid used if null).
        /// </summary>
        public GpsPoint ReturnPoint { get; set; }

        /// <summary>
        /// Maximum altitude AGL (400ft = 122m default).
        /// </summary>
        public double MaxAltitudeAglMeters { get; set; } = 122.0;

        /// <summary>
        /// Failsafe behavior settings.
        /// </summary>
        public FailsafeBehavior Failsafe { get; set; } = new FailsafeBehavior();

        /// <summary>
        /// Whether real-time boundary monitoring is active. Persisted so the
        /// monitor survives Mission Planner page switches and restarts.
        /// </summary>
        public bool MonitoringEnabled { get; set; }

        /// <summary>
        /// When true the soft boundary is derived automatically by insetting
        /// the hard boundary by <see cref="SoftBoundaryInsetMeters"/> instead
        /// of using manually entered coordinates.
        /// </summary>
        public bool SoftBoundaryFromHard { get; set; }

        /// <summary>
        /// Inward offset (meters) applied to the hard boundary to produce the
        /// derived soft boundary when <see cref="SoftBoundaryFromHard"/> is on.
        /// </summary>
        public double SoftBoundaryInsetMeters { get; set; } = 5.0;

        /// <summary>
        /// Descent rate (m/s) commanded on flight termination (maps to
        /// LAND_SPEED when pushing the fence to the vehicle).
        /// </summary>
        public double TerminationDescentRateMps { get; set; } = 2.0;

        /// <summary>
        /// Boundary violations log.
        /// </summary>
        public List<BoundaryViolation> BoundaryViolations { get; set; } = new List<BoundaryViolation>();

        // ============================================================
        // Persistence
        // ============================================================

        private static readonly string ConfigDir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "Mission Planner", "plugins", "NOMAD");

        private static readonly string ConfigPath = Path.Combine(ConfigDir, "geofence_config.json");

        /// <summary>
        /// Load geofence configuration from file (defaults if missing/invalid).
        /// </summary>
        public static GeofenceConfig Load()
        {
            try
            {
                if (File.Exists(ConfigPath))
                {
                    var json = File.ReadAllText(ConfigPath);
                    return JsonConvert.DeserializeObject<GeofenceConfig>(json) ?? new GeofenceConfig();
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Failed to load geofence config — {ex.Message}");
            }
            return new GeofenceConfig();
        }

        /// <summary>
        /// Save geofence configuration to file.
        /// </summary>
        public void Save()
        {
            try
            {
                if (!Directory.Exists(ConfigDir))
                {
                    Directory.CreateDirectory(ConfigDir);
                }

                var json = JsonConvert.SerializeObject(this, Formatting.Indented);
                File.WriteAllText(ConfigPath, json);
            }
            catch (Exception ex)
            {
                Log.Error($"Failed to save geofence config — {ex.Message}");
            }
        }

        // ============================================================
        // Derived soft boundary (hard boundary inset)
        // ============================================================

        /// <summary>
        /// Recompute the soft boundary from the hard boundary when
        /// <see cref="SoftBoundaryFromHard"/> is enabled. Returns true if the
        /// soft boundary was (re)generated. Caller is responsible for Save().
        /// </summary>
        public bool RegenerateSoftFromHard()
        {
            if (!SoftBoundaryFromHard) return false;

            if (HardBoundary?.Vertices == null || HardBoundary.Vertices.Count < 3)
            {
                SoftBoundary.Vertices.Clear();
                return true;
            }

            SoftBoundary.Vertices = InsetPolygon(HardBoundary.Vertices, SoftBoundaryInsetMeters);
            return true;
        }

        /// <summary>
        /// Offset a polygon inward by <paramref name="meters"/>. Works in a
        /// local equirectangular frame (fine for boundary-sized polygons) by
        /// shifting each edge along its inward normal and intersecting
        /// adjacent edges. If the inset is too large for the polygon (area
        /// would invert or grow), falls back to pulling each vertex toward
        /// the centroid by the same distance.
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

        // ============================================================
        // Boundary checking
        // ============================================================

        /// <summary>
        /// Check if a GPS point is inside a boundary polygon (ray casting).
        /// </summary>
        public bool IsInsideBoundary(GpsPoint point, FlightBoundary boundary)
        {
            if (boundary?.Vertices == null || boundary.Vertices.Count < 3)
                return true; // No boundary defined

            int n = boundary.Vertices.Count;
            bool inside = false;

            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                var vi = boundary.Vertices[i];
                var vj = boundary.Vertices[j];

                if (((vi.Lon > point.Lon) != (vj.Lon > point.Lon)) &&
                    (point.Lat < (vj.Lat - vi.Lat) * (point.Lon - vi.Lon) / (vj.Lon - vi.Lon) + vi.Lat))
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        /// <summary>
        /// Check boundary status for a GPS point.
        /// Returns: "inside", "soft_violation", "hard_violation".
        /// </summary>
        public string CheckBoundaryStatus(GpsPoint point, double? altitudeAgl = null)
        {
            if (altitudeAgl.HasValue && altitudeAgl > MaxAltitudeAglMeters)
            {
                return "hard_violation";
            }

            if (!IsInsideBoundary(point, HardBoundary))
            {
                return "hard_violation";
            }

            if (!IsInsideBoundary(point, SoftBoundary))
            {
                return "soft_violation";
            }

            return "inside";
        }
    }
}
