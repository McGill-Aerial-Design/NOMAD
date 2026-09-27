// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Geofence Types
// ============================================================
// Plain data types for the geofence subsystem. Deliberately free
// of Mission Planner / Newtonsoft dependencies so they (and the
// GeoMath helpers) can be compiled standalone by the geometry
// unit tests (pixi run test-plugin-geometry).
// ============================================================

using System;
using System.Collections.Generic;

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
        /// Boundary type: "soft" = warning, "hard" = termination required; plugin activation unavailable.
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
        /// Seconds before reporting the unavailable termination request; not an aircraft safety deadline.
        /// </summary>
        public int HardBoundaryKillDelaySec { get; set; } = 10;

        /// <summary>
        /// Enable audible warnings.
        /// </summary>
        public bool EnableAudioWarnings { get; set; } = true;

        /// <summary>
        /// Enable visual overlay warnings.
        /// </summary>
        public bool EnableVisualWarnings { get; set; } = true;
    }
}
