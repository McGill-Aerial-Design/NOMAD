// ============================================================
// NOMAD Mission Configuration
// ============================================================
// Handles AEAC 2026 competition-specific mission settings.
// Includes flight boundaries, building coordinates, equipment,
// and all competition-related configuration.
// ============================================================

using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// GPS Coordinate for mission configuration.
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
    /// Staging area for equipment delivery (Task 1).
    /// </summary>
    public class StagingArea
    {
        public string Id { get; set; }
        public string Name { get; set; }
        public GpsPoint Location { get; set; }
        public double RadiusMeters { get; set; } = 1.0; // 32-inch landing pad ~0.4m
        public string DeliveredEquipment { get; set; } // null if empty
    }

    /// <summary>
    /// Target detected during reconnaissance.
    /// </summary>
    public class DetectedTarget
    {
        public string Id { get; set; }
        public DateTime DetectedAt { get; set; }
        public GpsPoint EstimatedLocation { get; set; }
        public string Color { get; set; }
        public string RelativeDescription { get; set; }
        public double? DiameterCm { get; set; }
        public string SnapshotPath { get; set; }
        public bool IsSubmitted { get; set; }
        public double? AccuracyMeters { get; set; }
    }

    /// <summary>
    /// Equipment item for Task 1 delivery.
    /// </summary>
    public class EquipmentItem
    {
        public string Type { get; set; } // "radio", "oxygen_tank", "ladder"
        public string Name { get; set; }
        public double WeightKg { get; set; }
        public string Dimensions { get; set; } // "WxHxD cm"
        public bool IsLoaded { get; set; }
        public bool IsDelivered { get; set; }
        public string DeliveredToStaging { get; set; }
        public DateTime? DeliveredAt { get; set; }
        public int PointValue { get; set; }
    }

    /// <summary>
    /// Lap tracking for Task 1 distance scoring.
    /// </summary>
    public class LapRecord
    {
        public int LapNumber { get; set; }
        public DateTime StartTime { get; set; }
        public DateTime? EndTime { get; set; }
        public TimeSpan? Duration => EndTime.HasValue ? EndTime - StartTime : null;
        public List<GpsPoint> Waypoints { get; set; } = new List<GpsPoint>();
        public int WaypointsCompleted { get; set; }
    }

    /// <summary>
    /// Task 2 fire target for extinguishing.
    /// </summary>
    public class FireTarget
    {
        public string Id { get; set; }
        public GpsPoint Location { get; set; }
        public bool IsIndoor { get; set; }
        public bool IsExtinguished { get; set; }
        public DateTime? ExtinguishedAt { get; set; }
        public string PhotoPath { get; set; }
        public bool IsDeclared { get; set; }
        public int PointValue { get; set; }
    }

    /// <summary>
    /// Building information for both tasks.
    /// </summary>
    public class BuildingInfo
    {
        public string Name { get; set; }
        public GpsPoint Coordinates { get; set; }
        public double? WidthMeters { get; set; }
        public double? DepthMeters { get; set; }
        public double? HeightMeters { get; set; }
        public string DoorOrientation { get; set; } // N, S, E, W
        public GpsPoint DoorLocation { get; set; }
        public double DoorWidthMeters { get; set; } = 4.0;
        public double DoorHeightMeters { get; set; } = 4.0;
    }

    /// <summary>
    /// Search volume around a building.
    /// </summary>
    public class SearchVolume
    {
        public string TaskName { get; set; }
        public GpsPoint Center { get; set; }
        public double HorizontalRadiusMeters { get; set; } = 15.0;
        public double MaxAltitudeAgl { get; set; } = 10.0;
    }

    /// <summary>
    /// Boundary violation event.
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
        /// Options: "warn_audio", "warn_visual", "warn_both", "return_to_boundary"
        /// </summary>
        public string SoftBoundaryAction { get; set; } = "warn_both";

        /// <summary>
        /// Action when crossing hard boundary.
        /// Options: "warn_and_kill", "auto_kill", "warn_only"
        /// </summary>
        public string HardBoundaryAction { get; set; } = "warn_and_kill";

        /// <summary>
        /// Seconds to wait before auto-kill after hard boundary violation.
        /// </summary>
        public int HardBoundaryKillDelaySec { get; set; } = 10;

        /// <summary>
        /// Action on VIO failure during Task 2.
        /// Options: "alt_hold", "land", "return_to_door", "operator_takeover"
        /// </summary>
        public string VioFailureAction { get; set; } = "alt_hold";

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
    /// Autonomy tracking for scoring.
    /// </summary>
    public class AutonomyTracking
    {
        public bool AutonomousTakeoff { get; set; }
        public DateTime? TakeoffTime { get; set; }
        
        public bool AutonomousLanding { get; set; }
        public DateTime? LandingTime { get; set; }

        public bool AutonomousTargetExtinguish { get; set; }
        public int AutonomousExtinguishCount { get; set; }

        public List<string> ManualInterventions { get; set; } = new List<string>();
    }

    /// <summary>
    /// Complete mission configuration for AEAC 2026.
    /// </summary>
    public class MissionConfig
    {
        // ============================================================
        // Mission Identification
        // ============================================================

        /// <summary>
        /// Team name/identifier.
        /// </summary>
        // Single source of truth for the team name. Used to build:
        //   - Task_1_<TeamName>_targets.txt (CONOPS §5.2.3.6.f)
        //   - Task_2_<TeamName>_target_<n>.jpg (CONOPS §5.2.4.4.f)
        // Match this to what is registered with AEAC. "MAD" matches the
        // Jetson-side default (target_localizer node param) so the file
        // name is consistent across the GCS and the drone.
        public string TeamName { get; set; } = "MAD";

        /// <summary>
        /// Team callsign for RTM communications (e.g., "Nomad 101A").
        /// </summary>
        public string Callsign { get; set; } = "Nomad 101A";

        /// <summary>
        /// Current task number (1 or 2).
        /// </summary>
        public int CurrentTask { get; set; } = 1;

        /// <summary>
        /// Mission date/time.
        /// </summary>
        public DateTime? MissionDateTime { get; set; }

        /// <summary>
        /// Flight window start time.
        /// </summary>
        public DateTime? FlightWindowStart { get; set; }

        /// <summary>
        /// Flight window duration in minutes.
        /// </summary>
        public int FlightWindowDurationMinutes { get; set; } = 30;

        // ============================================================
        // Flight Boundaries
        // ============================================================

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

        // ============================================================
        // Task 1: Fire Reconnaissance
        // ============================================================

        /// <summary>
        /// Building on fire for Task 1.
        /// </summary>
        public BuildingInfo Task1Building { get; set; } = new BuildingInfo();

        /// <summary>
        /// Search volume around Task 1 building.
        /// </summary>
        public SearchVolume Task1SearchVolume { get; set; } = new SearchVolume
        {
            TaskName = "Task 1",
            HorizontalRadiusMeters = 15.0,
            MaxAltitudeAgl = 10.0
        };

        /// <summary>
        /// Lap course waypoints.
        /// </summary>
        public List<GpsPoint> LapCourseWaypoints { get; set; } = new List<GpsPoint>();

        /// <summary>
        /// Staging areas for equipment delivery.
        /// </summary>
        public List<StagingArea> StagingAreas { get; set; } = new List<StagingArea>();

        /// <summary>
        /// Equipment available for delivery.
        /// </summary>
        public List<EquipmentItem> Equipment { get; set; } = new List<EquipmentItem>
        {
            new EquipmentItem { Type = "radio", Name = "Handheld Radio", WeightKg = 0.5, Dimensions = "7.5x7.5x20", PointValue = 5 },
            new EquipmentItem { Type = "oxygen_tank", Name = "Oxygen Tank", WeightKg = 1.0, Dimensions = "15x15x30", PointValue = 5 },
            new EquipmentItem { Type = "ladder", Name = "Ladder", WeightKg = 3.0, Dimensions = "15x60x120", PointValue = 10 }
        };

        /// <summary>
        /// Detected targets during recon.
        /// </summary>
        public List<DetectedTarget> DetectedTargets { get; set; } = new List<DetectedTarget>();

        /// <summary>
        /// Completed laps.
        /// </summary>
        public List<LapRecord> CompletedLaps { get; set; } = new List<LapRecord>();

        /// <summary>
        /// Current lap in progress.
        /// </summary>
        public LapRecord CurrentLap { get; set; }

        // ============================================================
        // Task 2: Fire Extinguishing
        // ============================================================

        /// <summary>
        /// Building on fire for Task 2.
        /// </summary>
        public BuildingInfo Task2Building { get; set; } = new BuildingInfo();

        /// <summary>
        /// Search volume around Task 2 building.
        /// </summary>
        public SearchVolume Task2SearchVolume { get; set; } = new SearchVolume
        {
            TaskName = "Task 2",
            HorizontalRadiusMeters = 15.0,
            MaxAltitudeAgl = 10.0
        };

        /// <summary>
        /// Fire targets for extinguishing.
        /// </summary>
        public List<FireTarget> FireTargets { get; set; } = new List<FireTarget>();

        /// <summary>
        /// Autonomy tracking for scoring.
        /// </summary>
        public AutonomyTracking Autonomy { get; set; } = new AutonomyTracking();

        // ============================================================
        // Failsafe and Behavior Configuration
        // ============================================================

        /// <summary>
        /// Failsafe behavior settings.
        /// </summary>
        public FailsafeBehavior Failsafe { get; set; } = new FailsafeBehavior();

        /// <summary>
        /// Boundary violations log.
        /// </summary>
        public List<BoundaryViolation> BoundaryViolations { get; set; } = new List<BoundaryViolation>();

        // ============================================================
        // RTM/ATC Configuration
        // ============================================================

        /// <summary>
        /// UAM corridor minimum altitude (meters AGL).
        /// </summary>
        public double UamCorridorMinAlt { get; set; } = 20.0;

        /// <summary>
        /// UAM corridor maximum altitude (meters AGL).
        /// </summary>
        public double UamCorridorMaxAlt { get; set; } = 35.0;

        /// <summary>
        /// Yield altitude when giving way to medevac (meters AGL).
        /// </summary>
        public double YieldAltitude { get; set; } = 50.0;

        /// <summary>
        /// Is currently in UAM corridor.
        /// </summary>
        public bool InUamCorridor { get; set; }

        /// <summary>
        /// ATC radio communications log.
        /// </summary>
        public List<string> AtcCommLog { get; set; } = new List<string>();

        // ============================================================
        // Scoring/Stats
        // ============================================================

        /// <summary>
        /// Total takeoff weight (kg) for payload fraction calculation.
        /// </summary>
        public double TakeoffWeightKg { get; set; }

        /// <summary>
        /// Payload weight (kg).
        /// </summary>
        public double PayloadWeightKg { get; set; }

        /// <summary>
        /// Calculate payload fraction (0-1).
        /// </summary>
        [JsonIgnore]
        public double PayloadFraction => TakeoffWeightKg > 0 
            ? PayloadWeightKg / TakeoffWeightKg 
            : 0;

        // ============================================================
        // File Paths
        // ============================================================

        /// <summary>
        /// Directory for saving snapshots.
        /// </summary>
        public string SnapshotDirectory { get; set; } = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "NOMAD", "Snapshots");

        /// <summary>
        /// Directory for mission logs.
        /// </summary>
        public string MissionLogDirectory { get; set; } = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "NOMAD", "MissionLogs");

        /// <summary>
        /// Path to target localization output file.
        /// </summary>
        [JsonIgnore]
        public string TargetLocalizationFile => Path.Combine(
            MissionLogDirectory, 
            $"Task_1_{TeamName.Replace(" ", "_")}_targets.txt");

        // ============================================================
        // Persistence
        // ============================================================

        private static readonly string ConfigDir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "Mission Planner", "plugins", "NOMAD");

        private static readonly string ConfigPath = Path.Combine(ConfigDir, "mission_config.json");

        /// <summary>
        /// Load mission configuration from file.
        /// </summary>
        public static MissionConfig Load()
        {
            try
            {
                if (File.Exists(ConfigPath))
                {
                    var json = File.ReadAllText(ConfigPath);
                    return JsonConvert.DeserializeObject<MissionConfig>(json) ?? new MissionConfig();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to load mission config - {ex.Message}");
            }
            return new MissionConfig();
        }

        /// <summary>
        /// Save mission configuration to file.
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
                Console.WriteLine($"NOMAD: Failed to save mission config - {ex.Message}");
            }
        }

        /// <summary>
        /// Export target localizations to text file for submission.
        /// </summary>
        public void ExportTargetLocalizations()
        {
            try
            {
                if (!Directory.Exists(MissionLogDirectory))
                {
                    Directory.CreateDirectory(MissionLogDirectory);
                }

                using (var writer = new StreamWriter(TargetLocalizationFile))
                {
                    writer.WriteLine($"# AEAC 2026 - Task 1 Target Localizations");
                    writer.WriteLine($"# Team: {TeamName}");
                    writer.WriteLine($"# Generated: {DateTime.UtcNow:yyyy-MM-dd HH:mm:ss} UTC");
                    writer.WriteLine();

                    foreach (var target in DetectedTargets)
                    {
                        if (!string.IsNullOrEmpty(target.RelativeDescription))
                        {
                            writer.WriteLine($"Target {target.Id} ({target.Color}): {target.RelativeDescription}");
                        }
                    }
                }

                Console.WriteLine($"NOMAD: Exported targets to {TargetLocalizationFile}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to export targets - {ex.Message}");
            }
        }

        /// <summary>
        /// Check if a GPS point is inside a boundary polygon.
        /// </summary>
        public bool IsInsideBoundary(GpsPoint point, FlightBoundary boundary)
        {
            if (boundary?.Vertices == null || boundary.Vertices.Count < 3)
                return true; // No boundary defined

            // Ray casting algorithm
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
        /// Returns: "inside", "soft_violation", "hard_violation"
        /// </summary>
        public string CheckBoundaryStatus(GpsPoint point, double? altitudeAgl = null)
        {
            // Check altitude
            if (altitudeAgl.HasValue && altitudeAgl > MaxAltitudeAglMeters)
            {
                return "hard_violation";
            }

            // Check hard boundary first (stricter)
            if (!IsInsideBoundary(point, HardBoundary))
            {
                return "hard_violation";
            }

            // Check soft boundary
            if (!IsInsideBoundary(point, SoftBoundary))
            {
                return "soft_violation";
            }

            return "inside";
        }

        /// <summary>
        /// Initialize default equipment for competition.
        /// </summary>
        public void InitializeDefaultEquipment()
        {
            Equipment = new List<EquipmentItem>
            {
                new EquipmentItem 
                { 
                    Type = "radio", 
                    Name = "Handheld Radio", 
                    WeightKg = 0.5, 
                    Dimensions = "7.5x7.5x20 cm", 
                    PointValue = 5 
                },
                new EquipmentItem 
                { 
                    Type = "oxygen_tank", 
                    Name = "Oxygen Tank", 
                    WeightKg = 1.0, 
                    Dimensions = "15x15x30 cm", 
                    PointValue = 5 
                },
                new EquipmentItem 
                { 
                    Type = "ladder", 
                    Name = "Ladder", 
                    WeightKg = 3.0, 
                    Dimensions = "15x60x120 cm", 
                    PointValue = 10 
                }
            };
        }

        /// <summary>
        /// Ensure required directories exist.
        /// </summary>
        public void EnsureDirectories()
        {
            if (!Directory.Exists(SnapshotDirectory))
            {
                Directory.CreateDirectory(SnapshotDirectory);
            }
            if (!Directory.Exists(MissionLogDirectory))
            {
                Directory.CreateDirectory(MissionLogDirectory);
            }
        }
    }
}
