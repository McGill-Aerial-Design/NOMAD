// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Geofence Configuration
// ============================================================
// General, reusable flight-geofence and failsafe configuration.
// Holds soft/hard boundary polygons, altitude limits, failsafe
// behavior, and boundary-violation checking. Task-agnostic.
//
// Data types live in GeofenceTypes.cs; pure polygon math lives in
// GeoMath.cs (both Mission Planner-free and unit-tested via
// `pixi run test-plugin-geometry`).
// ============================================================

using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
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
        /// Hard flight boundary (red - forced descent required).
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

            SoftBoundary.Vertices = GeoMath.InsetPolygon(HardBoundary.Vertices, SoftBoundaryInsetMeters);
            return true;
        }

        // ============================================================
        // Boundary checking
        // ============================================================

        /// <summary>
        /// Check if a GPS point is inside a boundary polygon (ray casting).
        /// </summary>
        public bool IsInsideBoundary(GpsPoint point, FlightBoundary boundary)
        {
            return GeoMath.IsInside(boundary?.Vertices, point);
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
