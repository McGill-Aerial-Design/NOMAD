// ============================================================
// NOMAD Task 1 Lap Mission Planner
// ============================================================
// Builds the Task 1 payload lap course as native Mission Planner
// mission rows so the route uses MP's normal waypoint rendering and
// optional vehicle mission upload path.
// ============================================================

using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Windows.Forms;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    public static class Task1LapMissionPlanner
    {
        public const double OutsideOffsetMeters = 2.0;
        public const double DefaultAltitudeMeters = 15.0;
        public const double AcceptanceRadiusMeters = 4.0;
        public const double PassRadiusMeters = 8.0;

        public class Result
        {
            public bool Success;
            public string Message;
        }

        public static List<GpsPoint> BuildOutsideOffsetCourse(IEnumerable<GpsPoint> referencePoints, double offsetMeters, double altitudeMeters)
        {
            var points = referencePoints?.ToList() ?? new List<GpsPoint>();
            if (points.Count < 3)
                return points;

            double centerLat = points.Average(p => p.Lat);
            double centerLon = points.Average(p => p.Lon);
            double latMetersPerDeg = 111320.0;
            double lonMetersPerDeg = 111320.0 * Math.Cos(centerLat * Math.PI / 180.0);

            return points.Select(p =>
            {
                double x = (p.Lon - centerLon) * lonMetersPerDeg;
                double y = (p.Lat - centerLat) * latMetersPerDeg;
                double len = Math.Sqrt(x * x + y * y);
                if (len < 0.001)
                    return new GpsPoint(p.Lat, p.Lon, altitudeMeters);

                double outX = x / len * offsetMeters;
                double outY = y / len * offsetMeters;
                return new GpsPoint(
                    p.Lat + outY / latMetersPerDeg,
                    p.Lon + outX / lonMetersPerDeg,
                    altitudeMeters);
            }).ToList();
        }

        public static Result LoadIntoFlightPlanner(List<GpsPoint> course, int laps)
        {
            var result = new Result();
            if (course == null || course.Count < 3)
            {
                result.Message = "Need at least 3 lap waypoints.";
                return result;
            }

            try
            {
                object fp = GetFlightPlannerInstance();
                if (fp == null)
                {
                    result.Message = "Mission Planner Flight Planner is not available yet.";
                    return result;
                }

                ClearCommandGrid(fp);

                var addCommand = fp.GetType().GetMethod(
                    "AddCommand",
                    BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance,
                    null,
                    new[]
                    {
                        typeof(MAVLink.MAV_CMD),
                        typeof(double), typeof(double), typeof(double), typeof(double),
                        typeof(double), typeof(double), typeof(double),
                        typeof(object)
                    },
                    null);

                if (addCommand == null)
                {
                    result.Message = "Mission Planner AddCommand API was not found.";
                    return result;
                }

                for (int i = 0; i < course.Count; i++)
                {
                    var p = course[i];
                    var next = course[(i + 1) % course.Count];
                    double yawDeg = BearingDegrees(p.Lat, p.Lon, next.Lat, next.Lon);
                    addCommand.Invoke(fp, new object[]
                    {
                        MAVLink.MAV_CMD.WAYPOINT,
                        0.0,
                        AcceptanceRadiusMeters,
                        PassRadiusMeters,
                        yawDeg,
                        // FlightPlanner.AddCommand names these x/y, but the
                        // waypoint grid maps them as Lon/Lat. Passing Lat/Lon
                        // puts the mission near -75 latitude.
                        p.Lon,
                        p.Lat,
                        p.Alt,
                        $"NOMAD Lap WP {i + 1}"
                    });
                }

                if (laps > 1)
                {
                    addCommand.Invoke(fp, new object[]
                    {
                        MAVLink.MAV_CMD.DO_JUMP,
                        1.0,
                        Math.Max(0, laps - 1),
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        "NOMAD Lap Repeat"
                    });
                }

                RedrawMission(fp);

                result.Success = true;
                result.Message = laps > 1
                    ? $"Loaded {course.Count} lap waypoints plus DO_JUMP for {laps} laps into Mission Planner."
                    : $"Loaded {course.Count} lap waypoints into Mission Planner.";
                return result;
            }
            catch (TargetInvocationException tie)
            {
                result.Message = tie.InnerException?.Message ?? tie.Message;
                return result;
            }
            catch (Exception ex)
            {
                result.Message = ex.Message;
                return result;
            }
        }

        public static Result UploadLoadedMissionToVehicle()
        {
            var result = new Result();

            try
            {
                object comPort = global::MissionPlanner.MainV2.comPort;
                if (comPort == null)
                {
                    result.Message = "Not connected to vehicle.";
                    return result;
                }

                if (!IsComPortOpen(comPort))
                {
                    result.Message = "Vehicle link is not open.";
                    return result;
                }

                object fp = GetFlightPlannerInstance();
                if (fp == null)
                {
                    result.Message = "Mission Planner Flight Planner is not available.";
                    return result;
                }

                var saveMethod = fp.GetType().GetMethod(
                    "saveWPs",
                    BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
                if (saveMethod == null)
                {
                    result.Message = "Mission Planner saveWPs API was not found.";
                    return result;
                }

                object reporter = CreateNoUiProgressReporter(comPort);
                if (reporter == null)
                {
                    result.Message = "Mission Planner progress reporter could not be created.";
                    return result;
                }

                saveMethod.Invoke(fp, new[] { reporter });

                result.Success = true;
                result.Message = "Uploaded loaded lap mission to the vehicle. It will not run until AUTO/mission start is commanded.";
                return result;
            }
            catch (TargetInvocationException tie)
            {
                result.Message = tie.InnerException?.Message ?? tie.Message;
                return result;
            }
            catch (Exception ex)
            {
                result.Message = ex.Message;
                return result;
            }
        }

        private static object GetFlightPlannerInstance()
        {
            var fpType = FindType("MissionPlanner.GCSViews.FlightPlanner");
            if (fpType == null)
                return null;

            var instanceProp = fpType.GetProperty("instance", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static);
            var instance = instanceProp?.GetValue(null);
            if (instance != null)
                return instance;

            var instanceField = fpType.GetField("instance", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static);
            instance = instanceField?.GetValue(null);
            if (instance != null)
                return instance;

            var main = global::MissionPlanner.MainV2.instance;
            var fpProp = main?.GetType().GetProperty("FlightPlanner", BindingFlags.Public | BindingFlags.Instance);
            return fpProp?.GetValue(main);
        }

        private static void ClearCommandGrid(object fp)
        {
            var commandsField = fp.GetType().GetField("Commands", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            var grid = commandsField?.GetValue(fp) as DataGridView;
            if (grid == null)
                return;

            grid.Rows.Clear();
        }

        private static void RedrawMission(object fp)
        {
            try
            {
                var gridField = fp.GetType().GetField("Commands", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
                var grid = gridField?.GetValue(fp) as DataGridView;
                var dataViewToLocation = fp.GetType().GetMethod("DataViewtoLocationwp", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
                var wpToScreen = fp.GetType().GetMethod("WPtoScreen", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
                if (grid == null || dataViewToLocation == null || wpToScreen == null)
                    return;

                var cmds = new List<Locationwp>();
                for (int i = 0; i < grid.Rows.Count; i++)
                {
                    if (grid.Rows[i].IsNewRow)
                        continue;
                    var loc = (Locationwp)dataViewToLocation.Invoke(fp, new object[] { i });
                    cmds.Add(loc);
                }

                wpToScreen.Invoke(fp, new object[] { cmds });
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Task 1 mission redraw warning - {ex.Message}");
            }
        }

        private static object CreateNoUiProgressReporter(object comPort)
        {
            var reporterType = comPort.GetType().GetNestedType("NoUIReporter", BindingFlags.Public | BindingFlags.NonPublic);
            if (reporterType == null)
                reporterType = FindType("MissionPlanner.MAVLinkInterface+NoUIReporter");
            return reporterType == null ? null : Activator.CreateInstance(reporterType, true);
        }

        private static bool IsComPortOpen(object comPort)
        {
            try
            {
                var bsProp = comPort.GetType().GetProperty("BaseStream");
                var bs = bsProp?.GetValue(comPort);
                var isOpenProp = bs?.GetType().GetProperty("IsOpen");
                return (bool?)isOpenProp?.GetValue(bs) ?? true;
            }
            catch
            {
                return true;
            }
        }

        private static Type FindType(string fullName)
        {
            foreach (var asm in AppDomain.CurrentDomain.GetAssemblies())
            {
                try
                {
                    var type = asm.GetType(fullName, false);
                    if (type != null)
                        return type;
                }
                catch { }
            }
            return null;
        }

        private static double BearingDegrees(double lat1, double lon1, double lat2, double lon2)
        {
            double phi1 = lat1 * Math.PI / 180.0;
            double phi2 = lat2 * Math.PI / 180.0;
            double dLon = (lon2 - lon1) * Math.PI / 180.0;

            double y = Math.Sin(dLon) * Math.Cos(phi2);
            double x = Math.Cos(phi1) * Math.Sin(phi2)
                     - Math.Sin(phi1) * Math.Cos(phi2) * Math.Cos(dLon);
            double brng = Math.Atan2(y, x) * 180.0 / Math.PI;
            return (brng + 360.0) % 360.0;
        }
    }
}
