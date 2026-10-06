// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Mission Planner advisory boundary configuration
// ============================================================
// Stores local polygons for map drawing and position preview. The runtime
// protocol does not expose boundary enforcement or evaluation.
// ============================================================

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Mission Planner-only boundary outlines and display preferences.
    /// These values are not sent to the runtime or flight controller.
    /// </summary>
    public class GeofenceConfig
    {
        public FlightBoundary SoftBoundary { get; set; } = new FlightBoundary
        {
            Name = "Inner Advisory Outline",
        };

        public FlightBoundary HardBoundary { get; set; } = new FlightBoundary
        {
            Name = "Outer Advisory Outline",
        };

        /// <summary>Altitude display threshold for Mission Planner-reported altitude telemetry.</summary>
        public double AdvisoryAltitudeDisplayThresholdMeters { get; set; } = 122.0;

        /// <summary>Whether to show local position relative to the saved outlines.</summary>
        public bool AdvisoryPreviewEnabled { get; set; }

        /// <summary>Whether a local advisory-outline transition can speak an alert.</summary>
        public bool AdvisoryAudioAlertsEnabled { get; set; } = true;

        /// <summary>
        /// When true, derive the inner display outline from the outer outline.
        /// This changes only Mission Planner's saved map geometry.
        /// </summary>
        public bool SoftBoundaryFromHard { get; set; }

        public double SoftBoundaryInsetMeters { get; set; } = 5.0;

        /// <summary>Shown to the operator after legacy boundary settings are migrated.</summary>
        [JsonIgnore]
        public string MigrationNotice { get; private set; }

        private static readonly string ConfigDir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "Mission Planner", "plugins", "NOMAD");

        private static readonly string ConfigPath = Path.Combine(ConfigDir, "geofence_config.json");

        public static GeofenceConfig Load()
        {
            if (!File.Exists(ConfigPath))
            {
                return new GeofenceConfig();
            }

            try
            {
                var config = LoadFromJson(File.ReadAllText(ConfigPath), out bool migrated);
                if (migrated && !config.Save())
                {
                    config.MigrationNotice += " The migrated settings could not be saved; " +
                        "review them again next start.";
                }

                return config;
            }
            catch (Exception ex)
            {
                Log.Error($"Failed to load advisory boundary config — {ex.Message}");
                return new GeofenceConfig
                {
                    MigrationNotice = "Saved boundary settings could not be read. Defaults are shown; " +
                        "review the advisory outlines.",
                };
            }
        }

        internal static GeofenceConfig LoadFromJson(string json, out bool migrated)
        {
            var source = JObject.Parse(json);
            var safeSource = (JObject)source.DeepClone();
            SanitizeAltitudeThreshold(safeSource);
            var config = safeSource.ToObject<GeofenceConfig>() ?? new GeofenceConfig();
            var changes = new List<string>();

            config.SoftBoundary ??= new FlightBoundary { Name = "Inner Advisory Outline" };
            config.HardBoundary ??= new FlightBoundary { Name = "Outer Advisory Outline" };
            config.SoftBoundary.Vertices ??= new List<GpsPoint>();
            config.HardBoundary.Vertices ??= new List<GpsPoint>();

            MigrateAltitudeThreshold(source, config, changes);
            MigratePreviewEnabled(source, config, changes);
            MigrateAudioPreference(source, config, changes);
            ReportRemovedSettings(source, changes);

            migrated = changes.Count > 0;
            if (migrated)
            {
                config.MigrationNotice = "Legacy boundary settings were migrated: " + string.Join(", ", changes) +
                    ". Saved outlines and altitude are visual advisory data only; they are not installed on the " +
                    "runtime or aircraft. Review them before use.";
            }

            return config;
        }

        private static void SanitizeAltitudeThreshold(JObject source)
        {
            var propertyName = nameof(AdvisoryAltitudeDisplayThresholdMeters);
            var value = source[propertyName];
            if (value == null)
            {
                return;
            }

            source[propertyName] = GetFiniteNumber(value) ?? 122.0;
        }

        public bool Save()
        {
            try
            {
                Directory.CreateDirectory(ConfigDir);
                var json = JsonConvert.SerializeObject(this, Formatting.Indented);
                File.WriteAllText(ConfigPath, json);
                return true;
            }
            catch (Exception ex)
            {
                Log.Error($"Failed to save advisory boundary config — {ex.Message}");
                return false;
            }
        }

        public bool RegenerateSoftFromHard()
        {
            if (!SoftBoundaryFromHard)
            {
                return false;
            }

            SoftBoundary ??= new FlightBoundary { Name = "Inner Advisory Outline" };
            if (HardBoundary?.Vertices == null || HardBoundary.Vertices.Count < 3)
            {
                SoftBoundary.Vertices.Clear();
                return true;
            }

            SoftBoundary.Vertices = GeoMath.InsetPolygon(HardBoundary.Vertices, SoftBoundaryInsetMeters);
            return true;
        }

        private static void MigrateAltitudeThreshold(JObject source, GeofenceConfig config, List<string> changes)
        {
            var currentThreshold = source[nameof(AdvisoryAltitudeDisplayThresholdMeters)];
            var advisoryThreshold = GetFiniteNumber(source[nameof(AdvisoryAltitudeDisplayThresholdMeters)]);
            if (advisoryThreshold.HasValue)
            {
                config.AdvisoryAltitudeDisplayThresholdMeters = advisoryThreshold.Value;
            }
            else if (currentThreshold != null)
            {
                config.AdvisoryAltitudeDisplayThresholdMeters = 122.0;
                changes.Add("an invalid advisory altitude value was reset to the display default");
            }

            var oldThreshold = GetFiniteNumber(source["MaxAltitudeAglMeters"]);
            if (source["MaxAltitudeAglMeters"] != null)
            {
                if (currentThreshold == null && oldThreshold.HasValue)
                {
                    config.AdvisoryAltitudeDisplayThresholdMeters = oldThreshold.Value;
                    changes.Add("the saved altitude reference was retained only as a display threshold " +
                        "for Mission Planner-reported altitude");
                }
                else if (!oldThreshold.HasValue)
                {
                    changes.Add("an invalid retired altitude value was ignored");
                }
                else
                {
                    changes.Add("the retired altitude-policy field was removed");
                }
            }

            if (HasAltitudePolicy(source["SoftBoundary"]) || HasAltitudePolicy(source["HardBoundary"]))
            {
                changes.Add("per-outline altitude rules were removed");
            }

            if (HasUnusedOutlineMetadata(source["SoftBoundary"]) || HasUnusedOutlineMetadata(source["HardBoundary"]))
            {
                changes.Add("unused outline type and color fields were removed");
            }
        }

        private static void MigratePreviewEnabled(JObject source, GeofenceConfig config, List<string> changes)
        {
            var advisoryEnabled = source[nameof(AdvisoryPreviewEnabled)];
            if (advisoryEnabled != null && advisoryEnabled.Type == JTokenType.Boolean)
            {
                config.AdvisoryPreviewEnabled = advisoryEnabled.Value<bool>();
            }

            var oldEnabled = source["MonitoringEnabled"];
            if (oldEnabled != null && oldEnabled.Type == JTokenType.Boolean)
            {
                if (advisoryEnabled == null)
                {
                    config.AdvisoryPreviewEnabled = oldEnabled.Value<bool>();
                    changes.Add("the saved monitor choice was retained as a local advisory preview preference");
                }
                else
                {
                    changes.Add("the retired monitor field was removed");
                }
            }
        }

        private static void MigrateAudioPreference(JObject source, GeofenceConfig config, List<string> changes)
        {
            var advisoryAudio = source[nameof(AdvisoryAudioAlertsEnabled)];
            if (advisoryAudio != null && advisoryAudio.Type == JTokenType.Boolean)
            {
                config.AdvisoryAudioAlertsEnabled = advisoryAudio.Value<bool>();
            }

            var oldAudio = source["Failsafe"]?["EnableAudioWarnings"];
            if (oldAudio != null && oldAudio.Type == JTokenType.Boolean)
            {
                if (advisoryAudio == null)
                {
                    config.AdvisoryAudioAlertsEnabled = oldAudio.Value<bool>();
                    changes.Add("the audio preference was retained for local alerts");
                }
                else
                {
                    changes.Add("the retired audio setting was removed");
                }
            }
        }

        private static void ReportRemovedSettings(JObject source, List<string> changes)
        {
            if (source["ReturnPoint"] != null)
            {
                changes.Add("the unused return location was removed");
            }

            var failsafe = source["Failsafe"] as JObject;
            if (failsafe != null && (failsafe["SoftBoundaryAction"] != null ||
                failsafe["HardBoundaryAction"] != null || failsafe["HardBoundaryKillDelaySec"] != null))
            {
                changes.Add("frontend boundary actions and termination timing were removed");
            }

            if (source["BoundaryViolations"] != null)
            {
                changes.Add("the local violation history was removed");
            }
        }

        private static bool HasAltitudePolicy(JToken boundary)
        {
            var item = boundary as JObject;
            return item != null && (item["MaxAltitudeAgl"] != null || item["MinAltitudeAgl"] != null);
        }

        private static bool HasUnusedOutlineMetadata(JToken boundary)
        {
            var item = boundary as JObject;
            return item != null && (item["BoundaryType"] != null || item["DisplayColor"] != null);
        }

        private static double? GetFiniteNumber(JToken value)
        {
            if (value == null || !double.TryParse(value.ToString(), NumberStyles.Float,
                CultureInfo.InvariantCulture, out double result))
            {
                return null;
            }

            return double.IsNaN(result) || double.IsInfinity(result) || result < 0 || result > 10000
                ? (double?)null
                : result;
        }
    }
}
