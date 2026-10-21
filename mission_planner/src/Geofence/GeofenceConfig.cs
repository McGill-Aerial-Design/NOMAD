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

            return LoadFromJson(File.ReadAllText(ConfigPath));
        }

        internal static GeofenceConfig LoadFromJson(string json)
        {
            var document = JObject.Parse(json, new JsonLoadSettings
            {
                DuplicatePropertyNameHandling = DuplicatePropertyNameHandling.Error,
            });
            NormalizeLegacyAdvisoryData(document);
            var config = document.ToObject<GeofenceConfig>(JsonSerializer.Create(new JsonSerializerSettings
            {
                MissingMemberHandling = MissingMemberHandling.Error,
            }));
            if (config == null || config.SoftBoundary?.Vertices == null || config.HardBoundary?.Vertices == null)
            {
                throw new JsonSerializationException("Advisory boundary outlines must contain vertex arrays.");
            }
            if (double.IsNaN(config.AdvisoryAltitudeDisplayThresholdMeters) ||
                double.IsInfinity(config.AdvisoryAltitudeDisplayThresholdMeters) ||
                config.AdvisoryAltitudeDisplayThresholdMeters < 0 || config.AdvisoryAltitudeDisplayThresholdMeters > 10000)
            {
                throw new JsonSerializationException("Advisory altitude display threshold must be between 0 and 10000 m.");
            }
            return config;
        }

        private static void NormalizeLegacyAdvisoryData(JObject document)
        {
            // Retire old frontend flight policy in memory; preserve the saved advisory geometry.
            foreach (string name in new[] { "SoftBoundary", "HardBoundary" })
            {
                if (!(document[name] is JObject boundary))
                {
                    continue;
                }
                foreach (string retired in new[] { "BoundaryType", "DisplayColor", "MaxAltitudeAgl", "MinAltitudeAgl" })
                {
                    boundary.Remove(retired);
                }
            }
            const string altitude = nameof(AdvisoryAltitudeDisplayThresholdMeters);
            if (document[altitude] == null && document["MaxAltitudeAglMeters"] != null)
            {
                document[altitude] = document["MaxAltitudeAglMeters"].DeepClone();
            }
            foreach (string retired in new[] { "MaxAltitudeAglMeters", "ReturnPoint", "Failsafe", "MonitoringEnabled",
                "TerminationDescentRateMps", "BoundaryViolations" })
            {
                document.Remove(retired);
            }
        }

        public bool Save()
        {
            return SaveToFile(ConfigPath);
        }

        internal GeofenceConfig Clone()
        {
            return LoadFromJson(JsonConvert.SerializeObject(this));
        }

        // Explicit path keeps persistence qualification out of the operator's AppData.
        internal bool SaveToFile(string path)
        {
            string temporary = path + ".tmp";
            try
            {
                var json = JsonConvert.SerializeObject(this, Formatting.Indented);
                var directory = Path.GetDirectoryName(path);
                if (!string.IsNullOrEmpty(directory))
                {
                    Directory.CreateDirectory(directory);
                }
                File.WriteAllText(temporary, json);
                if (File.Exists(path))
                {
                    File.Replace(temporary, path, path + ".bak", ignoreMetadataErrors: true);
                }
                else
                {
                    File.Move(temporary, path);
                }
                return true;
            }
            catch (Exception ex)
            {
                Log.Error($"Failed to save advisory boundary config — {ex.Message}");
                try { File.Delete(temporary); }
                catch (IOException) { }
                catch (UnauthorizedAccessException) { }
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

    }
}
