// ============================================================
// NOMAD Building Corner Store (GCS-local)
// ============================================================
// Owns the Task 1 building footprint on the ground station.
// The Jetson used to expose /api/task/1/building/* for this; we
// pulled the model entirely off the drone so the GCS is now the
// single source of truth.
//
// Persists to <MyDocuments>/NOMAD/Task1/building_corners.json
// via atomic-write (.tmp -> Replace -> .bak) so a Mission Planner
// crash mid-edit can never leave the file half-written.
//
// Calibration: 1 observation translates the preset so that corner
// lands on the observed GPS; 2 observations also rotate the preset
// around the first observation so the second corner also lines up.
// Matches the algorithm previously implemented on the Jetson in
// edge_core/api_routes/task1.py:_apply_corner_calibration.
// ============================================================

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    public static class BuildingCornerStore
    {
        private const double DefaultHeightM = 2.4;
        private const double EarthRadiusM = 6_371_000.0;

        public static string StorePath => Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "NOMAD", "Task1", "building_corners.json");

        public class Corner
        {
            [JsonProperty("name")] public string Name { get; set; }
            [JsonProperty("lat")] public double Lat { get; set; }
            [JsonProperty("lon")] public double Lon { get; set; }
        }

        public class Observation
        {
            [JsonProperty("name")] public string Name { get; set; }
            [JsonProperty("lat")] public double Lat { get; set; }
            [JsonProperty("lon")] public double Lon { get; set; }
        }

        public class CalibrationInfo
        {
            [JsonProperty("mode")] public string Mode { get; set; } = "none";
            [JsonProperty("observation_count")] public int ObservationCount { get; set; }
            [JsonProperty("rotation_deg")] public double RotationDeg { get; set; }
        }

        public class WallOverride
        {
            [JsonProperty("name")] public string Name { get; set; }
            [JsonProperty("manual_override_m")] public double? ManualOverrideM { get; set; }
        }

        public class Model
        {
            [JsonProperty("preset")] public string Preset { get; set; } = "aeac_2026_task1";
            [JsonProperty("height")] public double Height { get; set; } = DefaultHeightM;
            [JsonProperty("center_lat")] public double CenterLat { get; set; }
            [JsonProperty("center_lon")] public double CenterLon { get; set; }
            [JsonProperty("base_center_lat")] public double BaseCenterLat { get; set; }
            [JsonProperty("base_center_lon")] public double BaseCenterLon { get; set; }
            [JsonProperty("base_corners")] public List<Corner> BaseCorners { get; set; } = new List<Corner>();
            [JsonProperty("corners")] public List<Corner> Corners { get; set; } = new List<Corner>();
            [JsonProperty("calibration_observations")] public List<Observation> Observations { get; set; } = new List<Observation>();
            [JsonProperty("calibration")] public CalibrationInfo Calibration { get; set; } = new CalibrationInfo();
            [JsonProperty("walls")] public List<WallOverride> Walls { get; set; } = new List<WallOverride>();
        }

        // ----- preset (competition footprint) -----

        private static readonly (string name, double lat, double lon)[] PresetCorners = new[]
        {
            ("1", 45.316743567764945, -75.75773827279546),
            ("2", 45.31671371473123, -75.75759833217171),
            ("3", 45.31615424384856, -75.75781374638878),
            ("4", 45.31618520285556, -75.75796312121189),
            ("5", 45.31641794771017, -75.75787506868524),
            ("6", 45.316440061185425, -75.75798592052764),
            ("7", 45.31652353947904, -75.75795525937997),
            ("8", 45.316500873200205, -75.75784362135427),
        };

        public static Model BuildPreset()
        {
            var corners = PresetCorners
                .Select(c => new Corner { Name = c.name, Lat = c.lat, Lon = c.lon })
                .ToList();
            double cLat = corners.Average(c => c.Lat);
            double cLon = corners.Average(c => c.Lon);
            return new Model
            {
                Preset = "aeac_2026_task1",
                Height = DefaultHeightM,
                CenterLat = cLat,
                CenterLon = cLon,
                BaseCenterLat = cLat,
                BaseCenterLon = cLon,
                BaseCorners = corners.Select(c => new Corner { Name = c.Name, Lat = c.Lat, Lon = c.Lon }).ToList(),
                Corners = corners,
                Observations = new List<Observation>(),
                Calibration = new CalibrationInfo { Mode = "none" },
                Walls = new List<WallOverride>(),
            };
        }

        // ----- persistence -----

        public static Model Load()
        {
            try
            {
                if (!File.Exists(StorePath)) return null;
                var raw = File.ReadAllText(StorePath);
                if (string.IsNullOrWhiteSpace(raw)) return null;
                return JsonConvert.DeserializeObject<Model>(raw);
            }
            catch
            {
                try
                {
                    var bak = StorePath + ".bak";
                    if (File.Exists(bak))
                    {
                        var raw = File.ReadAllText(bak);
                        return JsonConvert.DeserializeObject<Model>(raw);
                    }
                }
                catch { }
                return null;
            }
        }

        public static Model LoadOrPreset()
        {
            var m = Load();
            if (m != null && m.Corners != null && m.Corners.Count > 0) return m;
            return BuildPreset();
        }

        public static void Save(Model model)
        {
            if (model == null) return;
            var path = StorePath;
            var dir = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                Directory.CreateDirectory(dir);

            var json = JsonConvert.SerializeObject(model, Formatting.Indented);
            var tmp = path + ".tmp";
            var bak = path + ".bak";
            File.WriteAllText(tmp, json);
            if (File.Exists(path))
                File.Replace(tmp, path, bak, ignoreMetadataErrors: true);
            else
                File.Move(tmp, path);
        }

        public static void Delete()
        {
            try { if (File.Exists(StorePath)) File.Delete(StorePath); } catch { }
            try { if (File.Exists(StorePath + ".bak")) File.Delete(StorePath + ".bak"); } catch { }
        }

        // ----- calibration math -----

        // Equirectangular GPS to local ENU around (refLat, refLon). Matches
        // the previous Jetson helper so calibration files round-trip cleanly.
        private static (double east, double north) GpsToLocal(double lat, double lon, double refLat, double refLon)
        {
            double north = ToRad(lat - refLat) * EarthRadiusM;
            double east = ToRad(lon - refLon) * EarthRadiusM * Math.Cos(ToRad(refLat));
            return (east, north);
        }

        private static (double lat, double lon) OffsetGps(double refLat, double refLon, double east, double north)
        {
            double outLat = refLat + ToDeg(north / EarthRadiusM);
            double outLon = refLon + ToDeg(east / (EarthRadiusM * Math.Cos(ToRad(refLat))));
            return (outLat, outLon);
        }

        private static double ToRad(double d) => d * Math.PI / 180.0;
        private static double ToDeg(double r) => r * 180.0 / Math.PI;

        /// <summary>
        /// Record an observed GPS for one preset corner and recompute the
        /// transform. First observation = pure translation. Second observation
        /// with > 1 m base distance from the first = translation + rotation
        /// about the first observation.
        /// </summary>
        public static Model AddObservationAndCalibrate(Model model, string cornerName, double obsLat, double obsLon)
        {
            if (model == null) throw new ArgumentNullException(nameof(model));
            if (string.IsNullOrWhiteSpace(cornerName)) throw new ArgumentException("Corner name required.");

            // Make sure the base/center fields are populated (legacy files may not have them).
            if (model.BaseCorners == null || model.BaseCorners.Count == 0)
                model.BaseCorners = model.Corners.Select(c => new Corner { Name = c.Name, Lat = c.Lat, Lon = c.Lon }).ToList();
            if (model.BaseCenterLat == 0 && model.BaseCenterLon == 0)
            {
                model.BaseCenterLat = model.CenterLat;
                model.BaseCenterLon = model.CenterLon;
            }
            if (model.Observations == null) model.Observations = new List<Observation>();

            string name = cornerName.Trim();
            if (!model.BaseCorners.Any(c => string.Equals(c.Name, name, StringComparison.OrdinalIgnoreCase)))
            {
                var available = string.Join(", ", model.BaseCorners.Select(c => c.Name));
                throw new ArgumentException($"Corner '{name}' is not in the preset (have: {available}).");
            }

            var entry = new Observation { Name = name, Lat = obsLat, Lon = obsLon };
            int existing = model.Observations.FindIndex(o => string.Equals(o.Name, name, StringComparison.OrdinalIgnoreCase));
            if (existing >= 0) model.Observations[existing] = entry;
            else model.Observations.Add(entry);

            ApplyCalibration(model);
            return model;
        }

        public static void ApplyCalibration(Model model)
        {
            var usable = (model.Observations ?? new List<Observation>())
                .Where(o => model.BaseCorners.Any(c => string.Equals(c.Name, o.Name, StringComparison.OrdinalIgnoreCase)))
                .ToList();
            if (model.BaseCorners == null || model.BaseCorners.Count == 0 || usable.Count == 0)
            {
                model.Calibration = new CalibrationInfo { Mode = "none" };
                return;
            }

            double refLat = model.BaseCenterLat;
            double refLon = model.BaseCenterLon;

            var baseLocal = model.BaseCorners.ToDictionary(
                c => c.Name,
                c => GpsToLocal(c.Lat, c.Lon, refLat, refLon),
                StringComparer.OrdinalIgnoreCase);

            var obs0 = usable[0];
            var p0 = baseLocal[obs0.Name];
            var q0 = GpsToLocal(obs0.Lat, obs0.Lon, refLat, refLon);
            double theta = 0.0;
            string mode = "translation";

            if (usable.Count >= 2)
            {
                foreach (var obs1 in usable.Skip(1))
                {
                    if (!baseLocal.TryGetValue(obs1.Name, out var p1)) continue;
                    var q1 = GpsToLocal(obs1.Lat, obs1.Lon, refLat, refLon);
                    double baseDx = p1.east - p0.east;
                    double baseDy = p1.north - p0.north;
                    if (Math.Sqrt(baseDx * baseDx + baseDy * baseDy) > 1.0)
                    {
                        double baseHeading = Math.Atan2(baseDy, baseDx);
                        double obsHeading = Math.Atan2(q1.north - q0.north, q1.east - q0.east);
                        theta = obsHeading - baseHeading;
                        mode = "translation_rotation";
                        break;
                    }
                }
            }

            double cosT = Math.Cos(theta);
            double sinT = Math.Sin(theta);
            (double east, double north) Transform((double east, double north) pt)
            {
                double x = cosT * pt.east - sinT * pt.north;
                double y = sinT * pt.east + cosT * pt.north;
                double x0 = cosT * p0.east - sinT * p0.north;
                double y0 = sinT * p0.east + cosT * p0.north;
                return (x + (q0.east - x0), y + (q0.north - y0));
            }

            var calibrated = new List<Corner>(model.BaseCorners.Count);
            double sumE = 0, sumN = 0;
            foreach (var c in model.BaseCorners)
            {
                var t = Transform(baseLocal[c.Name]);
                var (lat, lon) = OffsetGps(refLat, refLon, t.east, t.north);
                calibrated.Add(new Corner { Name = c.Name, Lat = lat, Lon = lon });
                sumE += t.east;
                sumN += t.north;
            }
            var (centerLat, centerLon) = OffsetGps(refLat, refLon,
                sumE / model.BaseCorners.Count,
                sumN / model.BaseCorners.Count);

            model.Corners = calibrated;
            model.CenterLat = centerLat;
            model.CenterLon = centerLon;
            model.Calibration = new CalibrationInfo
            {
                Mode = mode,
                ObservationCount = usable.Count,
                RotationDeg = theta * 180.0 / Math.PI,
            };
        }

        public static void ResetCalibration(Model model)
        {
            if (model == null) return;
            model.Observations?.Clear();
            if (model.BaseCorners != null && model.BaseCorners.Count > 0)
            {
                model.Corners = model.BaseCorners
                    .Select(c => new Corner { Name = c.Name, Lat = c.Lat, Lon = c.Lon })
                    .ToList();
                model.CenterLat = model.BaseCenterLat;
                model.CenterLon = model.BaseCenterLon;
            }
            model.Calibration = new CalibrationInfo { Mode = "none" };
        }

        // ----- wall lengths -----

        public class WallLength
        {
            public string Name { get; set; }
            public double LengthM { get; set; }
            public double? ManualOverrideM { get; set; }
        }

        public static List<WallLength> ComputeWallLengths(Model model)
        {
            var result = new List<WallLength>();
            if (model?.Corners == null || model.Corners.Count < 3) return result;
            double refLat = model.CenterLat;
            double refLon = model.CenterLon;
            var locals = model.Corners
                .Select(c => GpsToLocal(c.Lat, c.Lon, refLat, refLon))
                .ToList();
            for (int i = 0; i < locals.Count; i++)
            {
                int j = (i + 1) % locals.Count;
                double dx = locals[j].east - locals[i].east;
                double dy = locals[j].north - locals[i].north;
                double len = Math.Sqrt(dx * dx + dy * dy);
                string name = $"{model.Corners[i].Name}-{model.Corners[j].Name}";
                double? overrideM = model.Walls?
                    .FirstOrDefault(w => string.Equals(w.Name, name, StringComparison.OrdinalIgnoreCase))
                    ?.ManualOverrideM;
                result.Add(new WallLength { Name = name, LengthM = len, ManualOverrideM = overrideM });
            }
            return result;
        }

        public static void SetWallOverride(Model model, string wallName, double? lengthM)
        {
            if (model == null) return;
            if (model.Walls == null) model.Walls = new List<WallOverride>();
            var existing = model.Walls.FirstOrDefault(w => string.Equals(w.Name, wallName, StringComparison.OrdinalIgnoreCase));
            if (existing == null)
            {
                if (lengthM.HasValue)
                    model.Walls.Add(new WallOverride { Name = wallName, ManualOverrideM = lengthM });
            }
            else
            {
                existing.ManualOverrideM = lengthM;
            }
        }
    }
}
