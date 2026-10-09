// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using GMap.NET;
using GMap.NET.WindowsForms;
using MissionPlanner.GCSViews;

namespace NOMAD.MissionPlanner
{
    public static partial class MapOverlayManager
    {
        public static bool ExportToMPGeoFence(List<GpsPoint> vertices, string name,
            Color strokeColor, Color fillColor, int strokeWidth = 3)
        {
            var planner = FlightPlanner.instance;
            if (planner?.MainMap == null || planner.geofenceoverlay == null || vertices == null || vertices.Count < 3)
            {
                return false;
            }
            try
            {
                var points = vertices.Select(point => new PointLatLng(point.Lat, point.Lon)).ToList();
                var polygon = new GMapPolygon(points, name)
                {
                    Fill = new SolidBrush(fillColor),
                    Stroke = new Pen(strokeColor, strokeWidth),
                };
                planner.geofencepolygon = polygon;
                planner.geofenceoverlay.Polygons.Clear();
                planner.geofenceoverlay.Polygons.Add(polygon);
                planner.MainMap.UpdatePolygonLocalPosition(polygon);
                planner.MainMap.Invalidate();
                return true;
            }
            catch (Exception ex)
            {
                Log.Error($"Could not export advisory outline to the Plan map: {ex.Message}");
                return false;
            }
        }
    }
}
