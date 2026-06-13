// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Map Overlay Manager
// ============================================================
// Manages NOMAD boundary visualization on Mission Planner's map.
// Draws soft (yellow) and hard (red) boundary polygons.
// Uses reflection to access GMap.NET types for compatibility.
//
// NOTE: This class uses .NET Reflection to access internal Mission Planner and GMap.NET fields.
// If Mission Planner or GMap.NET updates break this, the plugin will continue to work but
// boundary visualization will be disabled (no errors thrown). The code includes automatic
// fallback from primary field names (mymap) to alternate names (gMapControl1) for robustness.
// See error logs if boundaries do not appear on the map.
// ============================================================

using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Reflection;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Manages NOMAD boundary overlays on Mission Planner's map.
    /// Uses reflection to access GMap.NET types for maximum compatibility.
    /// </summary>
    public static partial class MapOverlayManager
    {
        private static object _boundaryOverlay; // GMapOverlay instance via reflection
        private static bool _initialized = false;
        private static bool _initFailed = false;    // Track if init has permanently failed
        private static bool _fallbackUsedLogged = false;  // Track if fallback logging has been shown (prevent spam)
        private static Type _overlayType;
        private static Type _polygonType;
        private static Type _routeType;
        private static Type _markerType;
        private static Type _markerGoogleType;
        private static Type _markerTooltipModeType;
        private static Type _pointType;
        private static object _mapControl;

        // Boundary style constants
        private static readonly Color SOFT_BOUNDARY_STROKE = Color.Yellow;
        private static readonly int SOFT_BOUNDARY_WIDTH = 2;

        private static readonly Color HARD_BOUNDARY_STROKE = Color.Red;
        private static readonly int HARD_BOUNDARY_WIDTH = 3;

        // Reflection plumbing (type discovery, Initialize) lives in
        // MapOverlayManager.Reflection.cs.

        /// <summary>
        /// Draw a single polygon on the map using reflection.
        /// </summary>
        public static bool DrawPolygon(
            List<GpsPoint> vertices,
            string name,
            Color strokeColor,
            Color fillColor,
            int strokeWidth = 2)
        {
            if (!_initialized && !Initialize())
                return false;

            if (vertices == null || vertices.Count < 3)
                return false;

            try
            {
                // Create List<PointLatLng>
                var pointListType = typeof(List<>).MakeGenericType(_pointType);
                var points = Activator.CreateInstance(pointListType);
                var addMethod = pointListType.GetMethod("Add");

                foreach (var v in vertices)
                {
                    // Create PointLatLng(lat, lng)
                    var point = Activator.CreateInstance(_pointType, new object[] { v.Lat, v.Lon });
                    addMethod.Invoke(points, new[] { point });
                }

                // Close polygon
                if (vertices.Count > 0)
                {
                    var first = Activator.CreateInstance(_pointType, new object[] { vertices[0].Lat, vertices[0].Lon });
                    addMethod.Invoke(points, new[] { first });
                }

                // Create GMapPolygon(points, name)
                var polygon = Activator.CreateInstance(_polygonType, new object[] { points, name });

                // Set Fill and Stroke
                if (!SetMemberValue(polygon, "Fill", new SolidBrush(fillColor)))
                    throw new MissingMemberException(_polygonType.FullName, "Fill");
                if (!SetMemberValue(polygon, "Stroke", new Pen(strokeColor, strokeWidth)))
                    throw new MissingMemberException(_polygonType.FullName, "Stroke");

                // Remove existing polygon with same name
                RemovePolygonByName(name);

                // Add to overlay.Polygons
                var polygons = GetOverlayCollection(_boundaryOverlay, "Polygons");
                if (polygons == null)
                    throw new MissingMemberException(_overlayType.FullName, "Polygons");

                polygons.Add(polygon);
                if (!polygons.Contains(polygon))
                    throw new InvalidOperationException($"Polygon {name} was not retained by the overlay");

                // GMap.NET only computes the polygon's pixel-space LocalPoints
                // after the overlay is bound to the control. Recompute explicitly
                // for compatibility with older Mission Planner GMap builds.
                var map = _mapControl ?? GetMapControl();
                map?.GetType().GetMethod("UpdatePolygonLocalPosition")
                    ?.Invoke(map, new[] { polygon });
                return true;
            }
            catch (Exception ex)
            {
                Log.Error($"Error drawing polygon {name} - {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Draw a visual-only route with numbered waypoint markers.
        /// Nothing is uploaded to the vehicle mission.
        /// </summary>
        public static bool DrawWaypointCourse(
            List<GpsPoint> vertices,
            string name,
            Color strokeColor,
            Color fillColor,
            int strokeWidth = 3)
        {
            if (!_initialized && !Initialize())
                return false;

            if (vertices == null || vertices.Count == 0)
                return false;

            try
            {
                DrawPolygon(vertices, name + "_polygon", strokeColor, fillColor, strokeWidth);
                DrawRoute(vertices, name + "_route", strokeColor, strokeWidth + 1);
                DrawNumberedMarkers(vertices, name + "_marker");
                RefreshMap();
                Log.Debug($"Drew waypoint course {name} with {vertices.Count} marker(s)");
                return true;
            }
            catch (Exception ex)
            {
                Log.Error($"Error drawing waypoint course {name} - {ex.Message}");
                return false;
            }
        }

        private static object BuildPointList(IEnumerable<GpsPoint> vertices, bool closeLoop)
        {
            var pointListType = typeof(List<>).MakeGenericType(_pointType);
            var points = Activator.CreateInstance(pointListType);
            var addMethod = pointListType.GetMethod("Add");

            var list = vertices.ToList();
            foreach (var v in list)
            {
                var point = Activator.CreateInstance(_pointType, new object[] { v.Lat, v.Lon });
                addMethod.Invoke(points, new[] { point });
            }

            if (closeLoop && list.Count > 0)
            {
                var first = Activator.CreateInstance(_pointType, new object[] { list[0].Lat, list[0].Lon });
                addMethod.Invoke(points, new[] { first });
            }

            return points;
        }

        private static void DrawRoute(List<GpsPoint> vertices, string name, Color strokeColor, int strokeWidth)
        {
            if (_routeType == null || vertices == null || vertices.Count < 2) return;

            try
            {
                RemoveRouteByName(name);
                var points = BuildPointList(vertices.Concat(new[] { vertices[0] }), false);
                var route = Activator.CreateInstance(_routeType, new object[] { points, name });
                SetMemberValue(route, "Stroke", new Pen(strokeColor, strokeWidth));

                var routes = GetOverlayCollection(_boundaryOverlay, "Routes");
                routes?.Add(route);
            }
            catch (Exception ex)
            {
                Log.Error($"Error drawing route {name} - {ex.Message}");
            }
        }

        private static void DrawNumberedMarkers(List<GpsPoint> vertices, string tagPrefix)
        {
            if (_markerType == null || _pointType == null || vertices == null || vertices.Count == 0) return;

            try
            {
                RemoveMarkersByTagPrefix(tagPrefix);

                var markers = GetOverlayCollection(_boundaryOverlay, "Markers");
                if (markers == null) return;

                object markerKind = null;
                if (_markerGoogleType != null)
                {
                    foreach (var candidate in new[] { "red_dot", "blue_dot", "green_dot", "red_small", "arrow" })
                    {
                        try
                        {
                            markerKind = Enum.Parse(_markerGoogleType, candidate, ignoreCase: true);
                            break;
                        }
                        catch { }
                    }
                }

                object tooltipAlways = null;
                if (_markerTooltipModeType != null)
                {
                    try { tooltipAlways = Enum.Parse(_markerTooltipModeType, "Always", ignoreCase: true); }
                    catch { }
                }

                for (int i = 0; i < vertices.Count; i++)
                {
                    var v = vertices[i];
                    var point = Activator.CreateInstance(_pointType, new object[] { v.Lat, v.Lon });
                    object marker = markerKind == null
                        ? Activator.CreateInstance(_markerType, new object[] { point })
                        : Activator.CreateInstance(_markerType, new object[] { point, markerKind });

                    string label = (i + 1).ToString();
                    SetMemberValue(marker, "ToolTipText", label);
                    SetMemberValue(marker, "Tag", $"{tagPrefix}_{label}");
                    if (tooltipAlways != null)
                        SetMemberValue(marker, "ToolTipMode", tooltipAlways);

                    markers.Add(marker);
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Error drawing numbered markers - {ex.Message}");
            }
        }

        private static void RemoveRouteByName(string name)
        {
            if (_boundaryOverlay == null || _overlayType == null) return;

            try
            {
                var routes = GetOverlayCollection(_boundaryOverlay, "Routes");
                if (routes == null) return;
                for (int i = routes.Count - 1; i >= 0; i--)
                {
                    var route = routes[i];
                    if ((string)GetMemberValue(route, "Name") == name)
                        routes.RemoveAt(i);
                }
            }
            catch { } // best-effort reflection
        }

        private static void RemoveMarkersByTagPrefix(string tagPrefix)
        {
            if (_boundaryOverlay == null || _overlayType == null) return;

            try
            {
                var markers = GetOverlayCollection(_boundaryOverlay, "Markers");
                if (markers == null) return;
                for (int i = markers.Count - 1; i >= 0; i--)
                {
                    var marker = markers[i];
                    var tag = GetMemberValue(marker, "Tag")?.ToString() ?? "";
                    if (tag.StartsWith(tagPrefix, StringComparison.OrdinalIgnoreCase))
                        markers.RemoveAt(i);
                }
            }
            catch { } // best-effort reflection
        }

        /// <summary>
        /// Remove a polygon by name.
        /// </summary>
        private static void RemovePolygonByName(string name)
        {
            if (_boundaryOverlay == null || _overlayType == null) return;

            try
            {
                var polygons = GetOverlayCollection(_boundaryOverlay, "Polygons");
                if (polygons != null)
                {
                    for (int i = polygons.Count - 1; i >= 0; i--)
                    {
                        var polygon = polygons[i];
                        if ((string)GetMemberValue(polygon, "Name") == name)
                        {
                            polygons.RemoveAt(i);
                        }
                    }
                }
            }
            catch { } // best-effort reflection
        }

        /// <summary>
        /// Draw safety zones from the saved geofence config on the Data and Plan maps.
        /// </summary>
        public static void DrawBoundaries(GeofenceConfig config)
        {
            if (config == null) return;

            if (!_initialized && !Initialize())
            {
                Log.Warn("Cannot draw boundaries - map overlay not initialized");
                return;
            }

            var polygons = GetOverlayCollection(_boundaryOverlay, "Polygons");
            polygons?.Clear();

            ConfigureBoundaryZoneRendering(config);
            Log.Info($"Drew boundary zones (soft: {config.SoftBoundary?.Vertices?.Count ?? 0} pts, "
                + $"hard: {config.HardBoundary?.Vertices?.Count ?? 0} pts, maps: {EnsureBoundaryMaps()})");
        }

        /// <summary>
        /// Clear all NOMAD boundary polygons and markers from the map.
        /// </summary>
        public static void ClearBoundaries()
        {
            try
            {
                if (_boundaryOverlay != null && _overlayType != null)
                {
                    var polygons = GetOverlayCollection(_boundaryOverlay, "Polygons");
                    polygons?.Clear();

                    var markers = GetOverlayCollection(_boundaryOverlay, "Markers");
                    markers?.Clear();
                }

                _renderHardBoundary.Clear();
                _renderSoftBoundary.Clear();
                InvalidateBoundaryMaps();
                Log.Debug("Cleared map boundaries");
            }
            catch { } // best-effort reflection
        }

        /// <summary>
        /// Force the map to refresh/redraw.
        /// </summary>
        public static void RefreshMap()
        {
            try
            {
                var mymap = _mapControl ?? GetMapControl();
                if (mymap != null)
                {
                    // Recompute every overlay item's pixel position, then repaint.
                    // Invalidate alone is not enough when items were added through
                    // reflection: their LocalPoints may have never been computed.
                    var forceUpdate = mymap.GetType().GetMethod("ForceUpdateOverlays", new Type[0]);
                    forceUpdate?.Invoke(mymap, null);

                    var invalidateMethod = mymap.GetType().GetMethod("Invalidate", new Type[0]);
                    invalidateMethod?.Invoke(mymap, null);
                }
            }
            catch { } // best-effort reflection
        }

        /// <summary>
        /// Center the map on specific coordinates.
        /// </summary>
        public static void CenterMapOn(double lat, double lon, int zoom = 17)
        {
            try
            {
                var mymap = _mapControl ?? GetMapControl();
                if (mymap != null && _pointType != null)
                {
                    var position = Activator.CreateInstance(_pointType, new object[] { lat, lon });
                    var positionProp = mymap.GetType().GetProperty("Position");
                    positionProp?.SetValue(mymap, position);

                    var zoomProp = mymap.GetType().GetProperty("Zoom");
                    zoomProp?.SetValue(mymap, (double)zoom);

                    Log.Debug($"Centered map on {lat:F6}, {lon:F6}");
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Error centering map - {ex.Message}");
            }
        }

        /// <summary>
        /// Check if the overlay is initialized and ready.
        /// </summary>
        public static bool IsInitialized => _initialized;
    }
}
