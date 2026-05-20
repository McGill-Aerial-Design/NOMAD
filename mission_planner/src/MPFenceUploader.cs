// ============================================================
// NOMAD MP Fence Uploader
// ============================================================
// Pushes a polygon boundary into Mission Planner's native geofence
// system: uploads vertices to the connected vehicle via MAVLink
// (FENCE_TOTAL + fence_point messages) and sets the related FENCE_*
// parameters (ENABLE, ACTION, ALT_MAX). Uses reflection on
// MainV2.comPort so it works across MP versions that ship different
// MAVLinkInterface method signatures.
// ============================================================

using System;
using System.Collections;
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
                // Total point count: 1 return point + N vertices + 1 closing vertex
                int total = vertices.Count + 2;

                // 1) Disable fence first so we can rewrite cleanly
                TrySetParam(comPort, "FENCE_ENABLE", 0);
                TrySetParam(comPort, "FENCE_TOTAL", total);
                TrySetParam(comPort, "FENCE_TYPE", 4); // polygon fence (bitmask: 4 = polygon)
                TrySetParam(comPort, "FENCE_ACTION", fenceAction);
                verifyTargets.Add(("FENCE_TYPE", 4));
                verifyTargets.Add(("FENCE_ACTION", fenceAction));
                verifyTargets.Add(("FENCE_TOTAL", total));
                if (maxAltMeters > 0)
                {
                    TrySetParam(comPort, "FENCE_ALT_MAX", maxAltMeters);
                    verifyTargets.Add(("FENCE_ALT_MAX", maxAltMeters));
                }
                if (landSpeedCmS > 0)
                {
                    // ArduCopter LAND_SPEED is in cm/s. CONOPS §4.5 requires
                    // ≥2 m/s vertical descent on termination — default 50 (0.5 m/s)
                    // is way too slow.
                    TrySetParam(comPort, "LAND_SPEED", landSpeedCmS);
                    verifyTargets.Add(("LAND_SPEED", landSpeedCmS));
                }

                // 2) Upload points
                bool ok = TrySetFencePoint(comPort, 0, rLat, rLon, (byte)total);
                if (!ok)
                {
                    result.Message = "Could not invoke setFencePoint on MAVLinkInterface (MP API mismatch).";
                    return result;
                }
                for (int i = 0; i < vertices.Count; i++)
                {
                    TrySetFencePoint(comPort, (byte)(i + 1), vertices[i].Lat, vertices[i].Lon, (byte)total);
                }
                // Closing point repeats first vertex
                TrySetFencePoint(comPort, (byte)(vertices.Count + 1), vertices[0].Lat, vertices[0].Lon, (byte)total);

                // 3) Re-enable
                if (enableFence)
                {
                    TrySetParam(comPort, "FENCE_ENABLE", 1);
                    verifyTargets.Add(("FENCE_ENABLE", 1));
                }
                else
                {
                    verifyTargets.Add(("FENCE_ENABLE", 0));
                }

                // 4) Verify params actually took. ArduPilot acks param sets
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

                if (mismatches.Count == 0)
                {
                    result.Success = true;
                    result.Message = $"Uploaded {vertices.Count} fence points; {verifyTargets.Count} params verified.";
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
        /// Clear the vehicle fence (set FENCE_TOTAL=0, disable).
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
                TrySetParam(comPort, "FENCE_TOTAL", 0);
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
                            // Default extra params (e.g. bool saveToBackstop, byte sysid/compid)
                            if (p[i].HasDefaultValue) args[i] = p[i].DefaultValue;
                            else if (p[i].ParameterType == typeof(bool)) args[i] = true;
                            else args[i] = p[i].ParameterType.IsValueType
                                ? Activator.CreateInstance(p[i].ParameterType)
                                : null;
                        }
                        try
                        {
                            m.Invoke(comPort, args);
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

        private static bool TrySetFencePoint(object comPort, byte index, double lat, double lon, byte total)
        {
            try
            {
                // MP signature: setFencePoint(byte index, PointLatLngAlt plla, byte fencepointcount)
                var methods = comPort.GetType().GetMethods(BindingFlags.Public | BindingFlags.Instance)
                    .Where(m => m.Name == "setFencePoint").ToList();

                foreach (var m in methods)
                {
                    var p = m.GetParameters();
                    if (p.Length < 2) continue;

                    object plla = BuildPointLatLngAlt(p[1].ParameterType, lat, lon);
                    if (plla == null) continue;

                    var args = new object[p.Length];
                    args[0] = index;
                    args[1] = plla;
                    for (int i = 2; i < p.Length; i++)
                    {
                        if (p[i].ParameterType == typeof(byte)) args[i] = total;
                        else if (p[i].HasDefaultValue) args[i] = p[i].DefaultValue;
                        else args[i] = p[i].ParameterType.IsValueType
                            ? Activator.CreateInstance(p[i].ParameterType)
                            : null;
                    }
                    try
                    {
                        m.Invoke(comPort, args);
                        return true;
                    }
                    catch { /* try next */ }
                }
                Console.WriteLine($"NOMAD: setFencePoint not invocable on comPort");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: setFencePoint error - {ex.Message}");
            }
            return false;
        }

        private static object BuildPointLatLngAlt(Type t, double lat, double lon)
        {
            try
            {
                // Try (double lat, double lng) ctor
                var ctor = t.GetConstructor(new[] { typeof(double), typeof(double) });
                if (ctor != null) return ctor.Invoke(new object[] { lat, lon });

                // Try (double lat, double lng, double alt)
                ctor = t.GetConstructor(new[] { typeof(double), typeof(double), typeof(double) });
                if (ctor != null) return ctor.Invoke(new object[] { lat, lon, 0.0 });

                // Try (double lat, double lng, double alt, string tag)
                ctor = t.GetConstructor(new[] { typeof(double), typeof(double), typeof(double), typeof(string) });
                if (ctor != null) return ctor.Invoke(new object[] { lat, lon, 0.0, "" });

                // Fallback: empty ctor and set fields
                var obj = Activator.CreateInstance(t);
                t.GetProperty("Lat")?.SetValue(obj, lat);
                t.GetProperty("Lng")?.SetValue(obj, lon);
                return obj;
            }
            catch { return null; }
        }
    }
}
