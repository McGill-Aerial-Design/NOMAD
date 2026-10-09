// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Geofence Types
// ============================================================
// Plain data types for local advisory outlines. Deliberately free
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
    /// Polygon used only to draw and preview a local advisory outline.
    /// </summary>
    public class FlightBoundary
    {
        /// <summary>Boundary name shown by Mission Planner.</summary>
        public string Name { get; set; } = "Boundary";

        /// <summary>Polygon vertices in order (latitude/longitude).</summary>
        public List<GpsPoint> Vertices { get; set; } = new List<GpsPoint>();
    }

    public enum AdvisoryOutlineStatus
    {
        NoPosition,
        NoOutline,
        InsideOutlines,
        OutsideInnerOutline,
        OutsideOuterOutline,
    }

    /// <summary>Presentation state for a local altitude reference, not a vehicle limit.</summary>
    public static class AdvisoryAltitudeStatus
    {
        public static bool IsAboveThreshold(double altitudeMeters, double thresholdMeters)
        {
            if (double.IsNaN(altitudeMeters) || double.IsInfinity(altitudeMeters))
            {
                return false;
            }

            if (double.IsNaN(thresholdMeters) || double.IsInfinity(thresholdMeters))
            {
                return false;
            }

            return altitudeMeters > thresholdMeters;
        }
    }
}
