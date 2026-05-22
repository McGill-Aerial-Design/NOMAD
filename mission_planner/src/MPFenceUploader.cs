// ============================================================
// NOMAD MP Fence Uploader
// ============================================================
// Pushes a polygon boundary into Mission Planner's native geofence
// system: uploads vertices to the connected vehicle as MAVLink fence
// mission items (MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION) and sets
// the related FENCE_* parameters (ENABLE, TYPE, ACTION, ALT_MAX).
// ============================================================

using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public static class MPFenceUploader
    {
        public class UploadResult
        {
            public bool Success;
            public string Message;
        }

        /// <summary>
        /// Upload polygon fence to vehicle and set FENCE_* params.
        /// Returns success + human-readable status for the caller to surface.
        /// </summary>
        public static UploadResult UploadPolygon(
            List<GpsPoint> vertices,
            GpsPoint returnPoint,
            double maxAltMeters,
            int fenceAction,
            bool enableFence,
            int landSpeedCmS = 0)
        {
            var result = new UploadResult();
            var verifyTargets = new List<(string Name, double Expected)>();

            if (vertices == null || vertices.Count < 3)
            {
                result.Message = "Need at least 3 vertices.";
                return result;
            }

            var comPort = MainV2.comPort;
            if (comPort == null)
            {
                result.Message = "Not connected to vehicle.";
                return result;
            }

            // Check baseStream open
            try
            {
                var bsProp = comPort.GetType().GetProperty("BaseStream");
                var bs = bsProp?.GetValue(comPort);
                var isOpenProp = bs?.GetType().GetProperty("IsOpen");
                var isOpen = (bool?)isOpenProp?.GetValue(bs) ?? false;
                if (!isOpen)
                {
                    result.Message = "Vehicle link not open.";
                    return result;
                }
            }
            catch { /* fall through and try anyway */ }

            // Compute return point (centroid if not provided)
            double rLat, rLon;
            if (returnPoint != null)
            {
                rLat = returnPoint.Lat;
                rLon = returnPoint.Lon;
            }
            else
            {
                rLat = vertices.Average(v => v.Lat);
                rLon = vertices.Average(v => v.Lon);
            }

            try
            {
                int fenceType = maxAltMeters > 0 ? 5 : 4; // bitmask: 1=alt max, 4=polygon

                // 1) Disable fence first so we can rewrite cleanly
                if (!TrySetParam(comPort, "FENCE_ENABLE", 0))
                {
                    result.Message = "Could not disable FENCE_ENABLE before upload.";
                    return result;
                }

                // ArduPilot's current polygon fence storage is MAVLink
                // mission type FENCE. The older FENCE_POINT/FENCE_TOTAL path
                // can leave Copter with FENCE_ENABLE=1 but no selected fence
                // items, triggering: "PreArm: Fences enabled, but none selected".
                TryUploadFenceMission(comPort, vertices, rLat, rLon);

                if (!TrySetParam(comPort, "FENCE_TYPE", fenceType))
                {
                    result.Message = "Could not set FENCE_TYPE.";
                    return result;
                }
                if (!TrySetParam(comPort, "FENCE_ACTION", fenceAction))
                {
                    result.Message = "Could not set FENCE_ACTION.";
                    return result;
                }
                verifyTargets.Add(("FENCE_TYPE", fenceType));
                verifyTargets.Add(("FENCE_ACTION", fenceAction));
                if (maxAltMeters > 0)
                {
                    if (!TrySetParam(comPort, "FENCE_ALT_MAX", maxAltMeters))
                    {
                        result.Message = "Could not set FENCE_ALT_MAX.";
                        return result;
                    }
                    verifyTargets.Add(("FENCE_ALT_MAX", maxAltMeters));
                }
                if (landSpeedCmS > 0)
                {
                    // ArduCopter LAND_SPEED is in cm/s. CONOPS §4.5 requires
                    // ≥2 m/s vertical descent on termination — default 50 (0.5 m/s)
                    // is way too slow.
                    if (!TrySetParam(comPort, "LAND_SPEED", landSpeedCmS))
                    {
                        result.Message = "Could not set LAND_SPEED.";
                        return result;
                    }
                    verifyTargets.Add(("LAND_SPEED", landSpeedCmS));
                }

                // 2) Re-enable after the fence mission and type are present.
                if (enableFence)
                {
                    if (!TrySetParam(comPort, "FENCE_ENABLE", 1))
                    {
                        result.Message = "Could not re-enable FENCE_ENABLE.";
                        return result;
                    }
                    verifyTargets.Add(("FENCE_ENABLE", 1));
                }
                else
                {
                    verifyTargets.Add(("FENCE_ENABLE", 0));
                }

                // 3) Verify params actually took. ArduPilot acks param sets
                // by echoing PARAM_VALUE, which MP caches on MAV.param.
                // Give the link a beat to receive the echoes, then compare.
                System.Threading.Thread.Sleep(400);
                var mismatches = new List<string>();
                foreach (var (name, expected) in verifyTargets)
                {
                    if (!TryGetParam(comPort, name, out double actual))
                    {
                        mismatches.Add($"{name}: no readback");
                        continue;
                    }
                    if (Math.Abs(actual - expected) > 0.01)
                        mismatches.Add($"{name}: got {actual}, expected {expected}");
                }

                string fenceSummary = TryGetFenceMissionSummary(comPort, vertices.Count);

                if (mismatches.Count == 0)
                {
                    result.Success = true;
                    result.Message = $"Uploaded {vertices.Count} inclusion fence vertices; {verifyTargets.Count} params verified. {fenceSummary}";
                }
                else
                {
                    result.Success = false;
                    result.Message = $"Uploaded {vertices.Count} fence points, but {mismatches.Count} param(s) failed verification:\n  "
                                   + string.Join("\n  ", mismatches);
                }
                return result;
            }
            catch (Exception ex)
            {
                result.Message = $"Upload error: {ex.Message}";
                return result;
            }
        }

        /// <summary>
        /// Clear the vehicle fence mission and disable the fence.
        /// </summary>
        public static UploadResult ClearFence()
        {
            var result = new UploadResult();
            var comPort = MainV2.comPort;
            if (comPort == null)
            {
                result.Message = "Not connected.";
                return result;
            }
            try
            {
                TrySetParam(comPort, "FENCE_ENABLE", 0);
                TryUploadEmptyFenceMission(comPort);
                result.Success = true;
                result.Message = "Fence cleared on vehicle.";
            }
            catch (Exception ex)
            {
                result.Message = ex.Message;
            }
            return result;
        }

        private static bool TrySetParam(object comPort, string name, double value)
        {
            try
            {
                // Look for setParam(string, double, bool) or setParam(string, double) etc.
                var methods = comPort.GetType().GetMethods(BindingFlags.Public | BindingFlags.Instance)
                    .Where(m => m.Name == "setParam").ToList();
                foreach (var m in methods)
                {
                    var p = m.GetParameters();
                    if (p.Length >= 2 && p[0].ParameterType == typeof(string))
                    {
                        var args = new object[p.Length];
                        args[0] = name;
                        args[1] = Convert.ChangeType(value, p[1].ParameterType);
                        for (int i = 2; i < p.Length; i++)
                        {
                            // Force the write even when Mission Planner's local
                            // param cache believes the value is already current.
                            if (p[i].ParameterType == typeof(bool)) args[i] = true;
                            else if (p[i].HasDefaultValue) args[i] = p[i].DefaultValue;
                            else args[i] = p[i].ParameterType.IsValueType
                                ? Activator.CreateInstance(p[i].ParameterType)
                                : null;
                        }
                        try
                        {
                            object invoked = m.Invoke(comPort, args);
                            if (m.ReturnType == typeof(bool))
                                return (bool)invoked;
                            return true;
                        }
                        catch { /* try next overload */ }
                    }
                }
                Console.WriteLine($"NOMAD: No setParam overload accepted for {name}={value}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: setParam({name}) error - {ex.Message}");
            }
            return false;
        }

        private static void TryUploadFenceMission(object comPort, List<GpsPoint> vertices, double returnLat, double returnLon)
        {
            var fence = new global::MissionPlanner.Utilities.Fence();

            fence.Fences.Add(new global::MissionPlanner.Utilities.FenceReturn
            {
                Return = new global::MissionPlanner.Utilities.PointLatLngAlt(returnLat, returnLon, 0)
            });

            fence.Fences.Add(new global::MissionPlanner.Utilities.FencePolygon
            {
                Mode = global::MissionPlanner.Utilities.FencePolygon.PolyType.Inclusive,
                Points = vertices.Select(v => new global::MissionPlanner.Utilities.PointLatLngAlt(v.Lat, v.Lon, 0)).ToList()
            });

            fence.UploadFence((global::MissionPlanner.MAVLinkInterface)comPort, (progress, status) =>
                Console.WriteLine($"NOMAD: fence upload {progress}% {status}"));
        }

        private static void TryUploadEmptyFenceMission(object comPort)
        {
            var fence = new global::MissionPlanner.Utilities.Fence();
            fence.UploadFence((global::MissionPlanner.MAVLinkInterface)comPort, (progress, status) =>
                Console.WriteLine($"NOMAD: fence clear {progress}% {status}"));
        }

        private static string TryGetFenceMissionSummary(object comPort, int expectedVertexCount)
        {
            try
            {
                var fence = new global::MissionPlanner.Utilities.Fence();
                fence.DownloadFence((global::MissionPlanner.MAVLinkInterface)comPort, (progress, status) =>
                    Console.WriteLine($"NOMAD: fence verify {progress}% {status}"));

                var polygon = fence.Fences
                    .OfType<global::MissionPlanner.Utilities.FencePolygon>()
                    .FirstOrDefault(p => p.Mode == global::MissionPlanner.Utilities.FencePolygon.PolyType.Inclusive);

                if (polygon == null)
                    return "Warning: fence mission readback did not include an inclusion polygon.";

                int actual = polygon.Points?.Count ?? 0;
                if (actual != expectedVertexCount)
                    return $"Warning: fence mission readback has {actual} inclusion vertices, expected {expectedVertexCount}.";

                return $"Fence mission readback: {actual} inclusion vertices.";
            }
            catch (Exception ex)
            {
                return $"Warning: fence mission readback failed ({ex.Message}).";
            }
        }

        /// <summary>Read a param value back from MP's cached PARAM_VALUE
        /// dictionary on comPort.MAV.param. Returns false if the key is
        /// missing or unreadable — caller surfaces this as a verification
        /// failure rather than guessing.</summary>
        private static bool TryGetParam(object comPort, string name, out double value)
        {
            value = 0;
            try
            {
                var mavProp = comPort.GetType().GetProperty("MAV");
                var mav = mavProp?.GetValue(comPort);
                if (mav == null) return false;

                var paramProp = mav.GetType().GetProperty("param");
                var paramDict = paramProp?.GetValue(mav);
                if (paramDict == null) return false;

                // MP exposes param as a MAVLinkParamList / dictionary-like.
                // Try indexer [string] first.
                var indexer = paramDict.GetType().GetProperty("Item", new[] { typeof(string) });
                if (indexer != null)
                {
                    object raw;
                    try { raw = indexer.GetValue(paramDict, new object[] { name }); }
                    catch { return false; }
                    if (raw == null) return false;
                    // Param entries can be MAVLinkParam with .Value, or a primitive.
                    var valProp = raw.GetType().GetProperty("Value");
                    object boxed = valProp != null ? valProp.GetValue(raw) : raw;
                    if (boxed == null) return false;
                    value = Convert.ToDouble(boxed);
                    return true;
                }

                // Fallback: ContainsKey + then index
                var contains = paramDict.GetType().GetMethod("ContainsKey", new[] { typeof(string) });
                if (contains != null)
                {
                    bool has = (bool)contains.Invoke(paramDict, new object[] { name });
                    if (!has) return false;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: TryGetParam({name}) error - {ex.Message}");
            }
            return false;
        }

    }
}
