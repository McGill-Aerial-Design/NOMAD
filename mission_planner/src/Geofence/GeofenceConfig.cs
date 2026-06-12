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
