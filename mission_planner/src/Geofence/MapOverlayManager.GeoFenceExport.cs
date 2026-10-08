// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Map Overlay Manager — local Mission Planner plan-map outline export
// ============================================================
// Partial of MapOverlayManager: draws NOMAD outline vertices in Mission
// Planner's Plan map. This does not upload a fence to the vehicle or runtime.
// ============================================================

using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;

namespace NOMAD.MissionPlanner
{
    public static partial class MapOverlayManager
    {
        /// <summary>
        /// Draw outline vertices in Mission Planner's Plan map. No vehicle fence
        /// configuration or runtime safety policy is changed.
        /// </summary>
        public static bool ExportToMPGeoFence(List<GpsPoint> vertices, string name, Color strokeColor, Color fillColor, int strokeWidth = 2)
        {
            if (vertices == null || vertices.Count < 3)
                return false;

            if (_pointType == null || _polygonType == null || _overlayType == null)
            {
                if (!Initialize())
                    return false;
            }

            try
            {
                var planner = GetFlightPlannerInstance();
                if (planner == null)
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

                var geofenceOverlay = GetMemberValue(planner, "geofenceoverlay");
                var polygons = GetOverlayCollection(geofenceOverlay, "Polygons");
                if (geofenceOverlay == null || polygons == null)
                {
                    Log.Warn("FlightPlanner geofence overlay not found");
                    return false;
                }

                var polygon = Activator.CreateInstance(_polygonType, new object[] { points, name });
                SetMemberValue(polygon, "Fill", new SolidBrush(fillColor));
                SetMemberValue(polygon, "Stroke", new Pen(strokeColor, strokeWidth));
                SetMemberValue(planner, "geofencepolygon", polygon);

                polygons.Clear();
                polygons.Add(polygon);

                var planMap = GetPlanMapControl();
                planMap?.GetType().GetMethod("UpdatePolygonLocalPosition")?.Invoke(planMap, new[] { polygon });
                planMap?.GetType().GetMethod("ForceUpdateOverlays", new Type[0])?.Invoke(planMap, null);
                planMap?.GetType().GetMethod("Invalidate", new Type[0])?.Invoke(planMap, null);

                MakeLegacyNomadFenceOutlineOnly();
                Log.Info($"Injected outline fence into Plan map ({vertices.Count} pts)");
                return true;
            }
            catch (Exception ex)
            {
                Log.Error($"Error exporting to MP GeoFence - {ex.Message}");
                return false;
            }
        }
    }
}
