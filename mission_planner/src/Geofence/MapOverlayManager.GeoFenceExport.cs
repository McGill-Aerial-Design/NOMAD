// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Map Overlay Manager — native Mission Planner geofence export
// ============================================================
// Partial of MapOverlayManager: injects NOMAD boundary vertices into Mission
// Planner's own FlightPlanner geofence overlay so the fence renders as a real
// MP fence on the Plan and Data maps. See MapOverlayManager.cs for the NOMAD
// overlay drawing and the GMap.NET reflection helpers.
// ============================================================

using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Reflection;

namespace NOMAD.MissionPlanner
{
    public static partial class MapOverlayManager
    {
        /// <summary>
        /// Export boundary vertices into Mission Planner's native geofence system
        /// by injecting into FlightPlanner's geofenceoverlay and geofencepolygon.
        /// This makes the fence visible on both Plan and Data maps as a real MP fence.
        /// </summary>
        public static bool ExportToMPGeoFence(List<GpsPoint> vertices, string name, Color strokeColor, Color fillColor, int strokeWidth = 2)
        {
            if (vertices == null || vertices.Count < 3)
                return false;

            // Ensure GMap types are resolved
            if (_pointType == null || _polygonType == null || _overlayType == null)
            {
                if (!Initialize())
                    return false;
            }

            try
            {
                // Find FlightPlanner type
                var fpType = FindTypeInLoadedAssemblies(
                    "MissionPlanner.GCSViews.FlightPlanner",
                    "MissionPlanner.FlightPlanner");

                if (fpType == null)
                {
                    Log.Warn("FlightPlanner type not found");
                    return false;
                }

                // Get FlightPlanner instance
                object fpInstance = null;

                var instanceProp = fpType.GetProperty("instance",
                    BindingFlags.Public | BindingFlags.Static | BindingFlags.NonPublic);
                if (instanceProp != null)
                    fpInstance = instanceProp.GetValue(null);

                if (fpInstance == null)
                {
                    var instanceField = fpType.GetField("instance",
                        BindingFlags.Public | BindingFlags.Static | BindingFlags.NonPublic);
                    if (instanceField != null)
                        fpInstance = instanceField.GetValue(null);
                }

                if (fpInstance == null)
                {
                    Log.Warn("FlightPlanner instance not found");
                    return false;
                }

                // Build the point list for the polygon
                var pointListType = typeof(List<>).MakeGenericType(_pointType);
                var points = Activator.CreateInstance(pointListType);
                var addMethod = pointListType.GetMethod("Add");

                foreach (var v in vertices)
                {
                    var point = Activator.CreateInstance(_pointType, new object[] { v.Lat, v.Lon });
                    addMethod.Invoke(points, new[] { point });
                }
                // Close polygon (first vertex repeated)
                var firstPt = Activator.CreateInstance(_pointType, new object[] { vertices[0].Lat, vertices[0].Lon });
                addMethod.Invoke(points, new[] { firstPt });

                bool injected = false;

                // Strategy 1: Inject into MP's existing geofenceoverlay + geofencepolygon
                // These are the fields MP uses to render its native geofence display
                var geoOverlayField = fpType.GetField("geofenceoverlay",
                    BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
                var geoPolygonField = fpType.GetField("geofencepolygon",
                    BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);

                if (geoOverlayField != null && geoPolygonField != null)
                {
                    var geoOverlay = geoOverlayField.GetValue(fpInstance);

                    // Create a new GMapPolygon with our points
                    var polygon = Activator.CreateInstance(_polygonType, new object[] { points, name });

                    var fillProp = _polygonType.GetProperty("Fill");
                    var strokeProp = _polygonType.GetProperty("Stroke");
                    fillProp?.SetValue(polygon, new SolidBrush(fillColor));
                    strokeProp?.SetValue(polygon, new Pen(strokeColor, strokeWidth));

                    // Set as the geofence polygon
                    geoPolygonField.SetValue(fpInstance, polygon);

                    // If the overlay exists, clear old polygons and add new one
                    if (geoOverlay != null)
                    {
                        var polygonsProp = _overlayType.GetProperty("Polygons");
                        if (polygonsProp != null)
                        {
                            var polygons = polygonsProp.GetValue(geoOverlay) as IList;
                            if (polygons != null)
                            {
                                polygons.Clear();
                                polygons.Add(polygon);
                            }
                        }
                    }

                    injected = true;
                    Log.Info($"Injected fence into FlightPlanner.geofencepolygon ({vertices.Count} pts)");
                }

                // Strategy 2: Also try drawnpolygon / drawnpolygons overlay (alternate fence display)
                foreach (var fieldName in new[] { "drawnpolygon", "drawnpolygonsoverlay" })
                {
                    var dpField = fpType.GetField(fieldName,
                        BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
                    if (dpField != null)
                    {
                        var dpOverlay = dpField.GetValue(fpInstance);
                        if (dpOverlay != null)
                        {
                            var polygonsProp = dpOverlay.GetType().GetProperty("Polygons");
                            if (polygonsProp != null)
                            {
                                var polygons = polygonsProp.GetValue(dpOverlay) as IList;
                                if (polygons != null)
                                {
                                    var polygon = Activator.CreateInstance(_polygonType, new object[] { points, name });
                                    var fillProp = _polygonType.GetProperty("Fill");
                                    var strokeProp = _polygonType.GetProperty("Stroke");
                                    fillProp?.SetValue(polygon, new SolidBrush(fillColor));
                                    strokeProp?.SetValue(polygon, new Pen(strokeColor, strokeWidth));
                                    polygons.Add(polygon);
                                    Log.Info($"Also added fence to {fieldName}");
                                }
                            }
                        }
                    }
                }

                // Strategy 3: If strategies 1-2 didn't find the fields, add to the map's overlay list directly
                if (!injected)
                {
                    object planMap = null;
                    foreach (var fieldName in new[] { "MainMap", "mymap", "gMapControl1" })
                    {
                        var field = fpType.GetField(fieldName,
                            BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
                        if (field != null)
                        {
                            planMap = field.GetValue(fpInstance);
                            if (planMap != null) break;
                        }
                    }

                    if (planMap != null)
                    {
                        var overlaysProp = planMap.GetType().GetProperty("Overlays");
                        if (overlaysProp != null)
                        {
                            var overlays = overlaysProp.GetValue(planMap) as IList;
                            if (overlays != null)
                            {
                                // Find or create a geofence overlay
                                object targetOverlay = null;
                                foreach (var overlay in overlays)
                                {
                                    var idProp = overlay.GetType().GetProperty("Id");
                                    var id = idProp?.GetValue(overlay) as string;
                                    // Try to find MP's own geofence overlay first
                                    if (id == "geofence" || id == "GeoFence" || id == "geofenceoverlay")
                                    {
                                        targetOverlay = overlay;
                                        break;
                                    }
                                }

                                if (targetOverlay == null)
                                {
                                    // Create our own overlay on the plan map
                                    targetOverlay = Activator.CreateInstance(_overlayType, new object[] { "nomad_fence" });
                                    overlays.Add(targetOverlay);
                                }

                                var polygonsProp = _overlayType.GetProperty("Polygons");
                                if (polygonsProp != null)
                                {
                                    var polygons = polygonsProp.GetValue(targetOverlay) as IList;
                                    if (polygons != null)
                                    {
                                        var polygon = Activator.CreateInstance(_polygonType, new object[] { points, name });
                                        var fillProp = _polygonType.GetProperty("Fill");
                                        var strokeProp = _polygonType.GetProperty("Stroke");
                                        fillProp?.SetValue(polygon, new SolidBrush(fillColor));
                                        strokeProp?.SetValue(polygon, new Pen(strokeColor, strokeWidth));
                                        polygons.Add(polygon);
                                        injected = true;
                                        Log.Info($"Added fence to plan map overlay");
                                    }
                                }
                            }
                        }

                        // Refresh plan map
                        var invalidateMethod = planMap.GetType().GetMethod("Invalidate", new Type[0]);
                        invalidateMethod?.Invoke(planMap, null);
                    }
                }

                // Also try to trigger MP's UpdateOverlayPolygons or redraw
                try
                {
                    var redrawMethod = fpType.GetMethod("UpdateOverlayPolygons",
                        BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
                    redrawMethod?.Invoke(fpInstance, null);
                }
                catch { } // best-effort reflection
                try
                {
                    var redrawMethod = fpType.GetMethod("redrawPolygonSurvey",
                        BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
                    redrawMethod?.Invoke(fpInstance, null);
                }
                catch { } // best-effort reflection

                // Log all available fields for debugging (remove after confirmed working)
                if (!injected)
                {
                    var allFields = fpType.GetFields(BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
                    var geoFields = allFields.Where(f =>
                        f.Name.IndexOf("geo", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        f.Name.IndexOf("fence", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        f.Name.IndexOf("polygon", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        f.Name.IndexOf("drawn", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        f.Name.IndexOf("overlay", StringComparison.OrdinalIgnoreCase) >= 0);
                    Log.Debug($"FlightPlanner fence-related fields: {string.Join(", ", geoFields.Select(f => $"{f.Name}:{f.FieldType.Name}"))}");
                }

                return injected;
            }
            catch (Exception ex)
            {
                Log.Error($"Error exporting to MP GeoFence - {ex.Message}");
                return false;
            }
        }
    }
}
