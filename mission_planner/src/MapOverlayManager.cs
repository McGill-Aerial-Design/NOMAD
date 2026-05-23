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
    public static class MapOverlayManager
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
        private static readonly Color SOFT_BOUNDARY_FILL = Color.FromArgb(40, Color.Yellow);
        private static readonly int SOFT_BOUNDARY_WIDTH = 2;
        
        private static readonly Color HARD_BOUNDARY_STROKE = Color.Red;
        private static readonly Color HARD_BOUNDARY_FILL = Color.FromArgb(50, Color.Red);
        private static readonly int HARD_BOUNDARY_WIDTH = 3;

        /// <summary>
        /// Find a type from loaded assemblies by partial name.
        /// </summary>
        private static Type FindTypeInLoadedAssemblies(params string[] typeNames)
        {
            foreach (var assembly in AppDomain.CurrentDomain.GetAssemblies())
            {
                foreach (var typeName in typeNames)
                {
                    try
                    {
                        var type = assembly.GetType(typeName, false);
                        if (type != null)
                        {
                            Console.WriteLine($"NOMAD: Found type {typeName} in {assembly.GetName().Name}");
                            return type;
                        }
                    }
                    catch { }
                }
            }
            return null;
        }

        /// <summary>
        /// Find an assembly by partial name from loaded assemblies.
        /// </summary>
        private static Assembly FindLoadedAssembly(params string[] partialNames)
        {
            foreach (var assembly in AppDomain.CurrentDomain.GetAssemblies())
            {
                var name = assembly.GetName().Name;
                foreach (var partial in partialNames)
                {
                    if (name.Equals(partial, StringComparison.OrdinalIgnoreCase) ||
                        name.StartsWith(partial, StringComparison.OrdinalIgnoreCase))
                    {
                        return assembly;
                    }
                }
            }
            return null;
        }

        /// <summary>
        /// Get the map control via reflection from FlightData.
        /// Includes fallback from 'mymap' to 'gMapControl1' field name for version compatibility.
        /// </summary>
        private static object GetMapControl()
        {
            if (_mapControl != null) return _mapControl;

            try
            {
                // Method 1: Try to find mymap field in FlightData
                var flightDataType = FindTypeInLoadedAssemblies(
                    "MissionPlanner.GCSViews.FlightData",
                    "MissionPlanner.FlightData");
                
                if (flightDataType != null)
                {
                    // Try static field first
                    var mymapField = flightDataType.GetField("mymap", 
                        BindingFlags.Public | BindingFlags.Static | BindingFlags.NonPublic);
                    if (mymapField != null)
                    {
                        _mapControl = mymapField.GetValue(null);
                        if (_mapControl != null)
                        {
                            Console.WriteLine($"NOMAD: Found map control via static field (type: {_mapControl.GetType().FullName})");
                            return _mapControl;
                        }
                    }

                    // Try instance through MainV2.instance
                    var mainV2Type = FindTypeInLoadedAssemblies("MissionPlanner.MainV2");
                    if (mainV2Type != null)
                    {
                        var instanceProp = mainV2Type.GetProperty("instance", BindingFlags.Public | BindingFlags.Static);
                        var instance = instanceProp?.GetValue(null);
                        if (instance != null)
                        {
                            // Try to get FlightData instance
                            var flightDataProp = mainV2Type.GetProperty("FlightData", BindingFlags.Public | BindingFlags.Instance);
                            var flightData = flightDataProp?.GetValue(instance);
                            
                            if (flightData != null)
                            {
                                // Get mymap from FlightData instance (primary field name)
                                var mymapInstanceField = flightData.GetType().GetField("mymap", 
                                    BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
                                if (mymapInstanceField != null)
                                {
                                    _mapControl = mymapInstanceField.GetValue(flightData);
                                    if (_mapControl != null)
                                    {
                                        Console.WriteLine($"NOMAD: Found map control via instance (type: {_mapControl.GetType().FullName})");
                                        return _mapControl;
                                    }
                                }

                                // FALLBACK: Try gMapControl1 as alternate name (handles version differences)
                                // Log this once to indicate we're using a fallback field name
                                var gMapField = flightData.GetType().GetField("gMapControl1", 
                                    BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
                                if (gMapField != null)
                                {
                                    _mapControl = gMapField.GetValue(flightData);
                                    if (_mapControl != null)
                                    {
                                        // Log fallback only once to avoid console spam
                                        if (!_fallbackUsedLogged)
                                        {
                                            Console.WriteLine($"NOMAD: Primary field 'mymap' not found, using fallback field 'gMapControl1' (type: {_mapControl.GetType().FullName})");
                                            _fallbackUsedLogged = true;
                                        }
                                        return _mapControl;
                                    }
                                }
                            }
                        }
                    }
                }

                Console.WriteLine("NOMAD: Could not find map control - FlightData may not be loaded yet");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Error getting map control - {ex.Message}");
            }
            return null;
        }

        /// <summary>
        /// Initialize the NOMAD overlay on the map using reflection.
        /// Must be called after FlightData is loaded.
        /// </summary>
        public static bool Initialize()
        {
            if (_initFailed) return false;  // Don't retry if permanently failed
            if (_initialized) return true;

            try
            {
                // Find GMap types from loaded assemblies
                // Try multiple possible type names for compatibility
                _overlayType = FindTypeInLoadedAssemblies(
                    "GMap.NET.WindowsForms.GMapOverlay",
                    "GMap.NET.GMapOverlay");
                    
                _polygonType = FindTypeInLoadedAssemblies(
                    "GMap.NET.WindowsForms.GMapPolygon",
                    "GMap.NET.GMapPolygon");

                _routeType = FindTypeInLoadedAssemblies(
                    "GMap.NET.WindowsForms.GMapRoute",
                    "GMap.NET.GMapRoute");

                _markerType = FindTypeInLoadedAssemblies(
                    "GMap.NET.WindowsForms.Markers.GMarkerGoogle",
                    "GMap.NET.Markers.GMarkerGoogle");

                _markerGoogleType = FindTypeInLoadedAssemblies(
                    "GMap.NET.WindowsForms.Markers.GMarkerGoogleType",
                    "GMap.NET.Markers.GMarkerGoogleType");

                _markerTooltipModeType = FindTypeInLoadedAssemblies(
                    "GMap.NET.WindowsForms.MarkerTooltipMode",
                    "GMap.NET.MarkerTooltipMode");
                    
                _pointType = FindTypeInLoadedAssemblies(
                    "GMap.NET.PointLatLng",
                    "GMap.NET.Core.PointLatLng");
                
                if (_overlayType == null || _polygonType == null || _pointType == null)
                {
                    // Log what we found and what we're missing
                    Console.WriteLine($"NOMAD: GMap types status - Overlay:{_overlayType != null}, Polygon:{_polygonType != null}, Point:{_pointType != null}");
                    
                    // List all assemblies that might be GMap related
                    var gmapAssemblies = AppDomain.CurrentDomain.GetAssemblies()
                        .Where(a => a.GetName().Name.IndexOf("GMap", StringComparison.OrdinalIgnoreCase) >= 0)
                        .Select(a => a.GetName().Name);
                    Console.WriteLine($"NOMAD: Found GMap assemblies: {string.Join(", ", gmapAssemblies)}");
                    
                    Console.WriteLine("NOMAD: Failed to load required GMap.NET types via reflection");
                    _initFailed = true;
                    return false;
                }
                
                // Get the map control
                var mymap = GetMapControl();
                if (mymap == null)
                {
                    Console.WriteLine("NOMAD: Map not available yet, will retry on next draw");
                    return false;  // Don't mark as permanently failed - might succeed later
                }

                // Create overlay if needed: new GMapOverlay("nomad_boundaries")
                if (_boundaryOverlay == null)
                {
                    _boundaryOverlay = Activator.CreateInstance(_overlayType, new object[] { "nomad_boundaries" });
                }

                // Get Overlays property from map
                var overlaysProp = mymap.GetType().GetProperty("Overlays");
                if (overlaysProp != null)
                {
                    var overlays = overlaysProp.GetValue(mymap) as IList;
                    if (overlays != null)
                    {
                        // Check if already added
                        bool found = false;
                        foreach (var overlay in overlays)
                        {
                            var idProp = overlay.GetType().GetProperty("Id");
                            if (idProp != null && (string)idProp.GetValue(overlay) == "nomad_boundaries")
                            {
                                found = true;
                                _boundaryOverlay = overlay;
                                break;
                            }
                        }

                        if (!found)
                        {
                            overlays.Add(_boundaryOverlay);
                        }
                    }
                }

                _initialized = true;
                Console.WriteLine("NOMAD: Map overlay initialized successfully via reflection");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to initialize map overlay - {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Draw both soft and hard boundaries on the map.
        /// </summary>
        public static void DrawBoundaries(MissionConfig config)
        {
            if (config == null) return;
            
            if (!_initialized && !Initialize())
            {
                Console.WriteLine("NOMAD: Cannot draw boundaries - map not initialized");
                return;
            }

            // Clear existing NOMAD boundaries before redrawing
            ClearBoundaries();

            // Draw soft boundary (yellow, outer warning zone)
            if (config.SoftBoundary?.Vertices?.Count >= 3)
            {
                DrawPolygon(
                    config.SoftBoundary.Vertices,
                    "NOMAD_Soft_Boundary",
                    SOFT_BOUNDARY_STROKE,
                    SOFT_BOUNDARY_FILL,
                    SOFT_BOUNDARY_WIDTH
                );
                Console.WriteLine($"NOMAD: Drew soft boundary with {config.SoftBoundary.Vertices.Count} vertices");
            }

            // Draw hard boundary (red, kill zone)
            if (config.HardBoundary?.Vertices?.Count >= 3)
            {
                DrawPolygon(
                    config.HardBoundary.Vertices,
                    "NOMAD_Hard_Boundary",
                    HARD_BOUNDARY_STROKE,
                    HARD_BOUNDARY_FILL,
                    HARD_BOUNDARY_WIDTH
                );
                Console.WriteLine($"NOMAD: Drew hard boundary with {config.HardBoundary.Vertices.Count} vertices");
            }

            RefreshMap();
        }

        /// <summary>
        /// Draw a single polygon on the map using reflection.
        /// </summary>
        public static void DrawPolygon(
            List<GpsPoint> vertices,
            string name,
            Color strokeColor,
            Color fillColor,
            int strokeWidth = 2)
        {
            if (!_initialized && !Initialize())
                return;

            if (vertices == null || vertices.Count < 3)
                return;

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
                var fillProp = _polygonType.GetProperty("Fill");
                var strokeProp = _polygonType.GetProperty("Stroke");
                
                if (fillProp != null)
                    fillProp.SetValue(polygon, new SolidBrush(fillColor));
                if (strokeProp != null)
                    strokeProp.SetValue(polygon, new Pen(strokeColor, strokeWidth));

                // Remove existing polygon with same name
                RemovePolygonByName(name);

                // Add to overlay.Polygons
                var polygonsProp = _overlayType.GetProperty("Polygons");
                if (polygonsProp != null)
                {
                    var polygons = polygonsProp.GetValue(_boundaryOverlay) as IList;
                    polygons?.Add(polygon);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Error drawing polygon {name} - {ex.Message}");
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
                Console.WriteLine($"NOMAD: Drew waypoint course {name} with {vertices.Count} marker(s)");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Error drawing waypoint course {name} - {ex.Message}");
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
                var strokeProp = _routeType.GetProperty("Stroke");
                strokeProp?.SetValue(route, new Pen(strokeColor, strokeWidth));

                var routesProp = _overlayType.GetProperty("Routes");
                var routes = routesProp?.GetValue(_boundaryOverlay) as IList;
                routes?.Add(route);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Error drawing route {name} - {ex.Message}");
            }
        }

        private static void DrawNumberedMarkers(List<GpsPoint> vertices, string tagPrefix)
        {
            if (_markerType == null || _pointType == null || vertices == null || vertices.Count == 0) return;

            try
            {
                RemoveMarkersByTagPrefix(tagPrefix);

                var markersProp = _overlayType.GetProperty("Markers");
                var markers = markersProp?.GetValue(_boundaryOverlay) as IList;
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
                    SetPropertyIfExists(marker, "ToolTipText", label);
                    SetPropertyIfExists(marker, "Tag", $"{tagPrefix}_{label}");
                    if (tooltipAlways != null)
                        SetPropertyIfExists(marker, "ToolTipMode", tooltipAlways);

                    markers.Add(marker);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Error drawing numbered markers - {ex.Message}");
            }
        }

        private static void SetPropertyIfExists(object target, string propertyName, object value)
        {
            try
            {
                var prop = target.GetType().GetProperty(propertyName);
                if (prop != null && prop.CanWrite)
                    prop.SetValue(target, value);
            }
            catch { }
        }

        private static void RemoveRouteByName(string name)
        {
            if (_boundaryOverlay == null || _overlayType == null) return;

            try
            {
                var routesProp = _overlayType.GetProperty("Routes");
                var routes = routesProp?.GetValue(_boundaryOverlay) as IList;
                if (routes == null) return;
                for (int i = routes.Count - 1; i >= 0; i--)
                {
                    var route = routes[i];
                    var nameProp = route.GetType().GetProperty("Name");
                    if (nameProp != null && (string)nameProp.GetValue(route) == name)
                        routes.RemoveAt(i);
                }
            }
            catch { }
        }

        private static void RemoveMarkersByTagPrefix(string tagPrefix)
        {
            if (_boundaryOverlay == null || _overlayType == null) return;

            try
            {
                var markersProp = _overlayType.GetProperty("Markers");
                var markers = markersProp?.GetValue(_boundaryOverlay) as IList;
                if (markers == null) return;
                for (int i = markers.Count - 1; i >= 0; i--)
                {
                    var marker = markers[i];
                    var tagProp = marker.GetType().GetProperty("Tag");
                    var tag = tagProp?.GetValue(marker)?.ToString() ?? "";
                    if (tag.StartsWith(tagPrefix, StringComparison.OrdinalIgnoreCase))
                        markers.RemoveAt(i);
                }
            }
            catch { }
        }

        /// <summary>
        /// Remove a polygon by name.
        /// </summary>
        private static void RemovePolygonByName(string name)
        {
            if (_boundaryOverlay == null || _overlayType == null) return;

            try
            {
                var polygonsProp = _overlayType.GetProperty("Polygons");
                if (polygonsProp != null)
                {
                    var polygons = polygonsProp.GetValue(_boundaryOverlay) as IList;
                    if (polygons != null)
                    {
                        for (int i = polygons.Count - 1; i >= 0; i--)
                        {
                            var polygon = polygons[i];
                            var nameProp = polygon.GetType().GetProperty("Name");
                            if (nameProp != null && (string)nameProp.GetValue(polygon) == name)
                            {
                                polygons.RemoveAt(i);
                            }
                        }
                    }
                }
            }
            catch { }
        }

        /// <summary>
        /// Clear all NOMAD boundary polygons and markers from the map.
        /// </summary>
        public static void ClearBoundaries()
        {
            if (_boundaryOverlay == null || _overlayType == null) return;
            
            try
            {
                var polygonsProp = _overlayType.GetProperty("Polygons");
                if (polygonsProp != null)
                {
                    var polygons = polygonsProp.GetValue(_boundaryOverlay) as IList;
                    polygons?.Clear();
                }
                
                var markersProp = _overlayType.GetProperty("Markers");
                if (markersProp != null)
                {
                    var markers = markersProp.GetValue(_boundaryOverlay) as IList;
                    markers?.Clear();
                }
                
                Console.WriteLine("NOMAD: Cleared map boundaries");
            }
            catch { }
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
                    var invalidateMethod = mymap.GetType().GetMethod("Invalidate", new Type[0]);
                    invalidateMethod?.Invoke(mymap, null);
                }
            }
            catch { }
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
                    
                    Console.WriteLine($"NOMAD: Centered map on {lat:F6}, {lon:F6}");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Error centering map - {ex.Message}");
            }
        }

        /// <summary>
        /// Center the map on the boundaries.
        /// </summary>
        public static void ZoomToBoundaries(MissionConfig config)
        {
            if (config == null) return;

            try
            {
                // Find the center of all boundary points
                var allPoints = new List<GpsPoint>();
                
                if (config.SoftBoundary?.Vertices != null)
                    allPoints.AddRange(config.SoftBoundary.Vertices);
                if (config.HardBoundary?.Vertices != null)
                    allPoints.AddRange(config.HardBoundary.Vertices);

                if (allPoints.Count == 0) return;

                double avgLat = 0, avgLon = 0;
                foreach (var p in allPoints)
                {
                    avgLat += p.Lat;
                    avgLon += p.Lon;
                }
                avgLat /= allPoints.Count;
                avgLon /= allPoints.Count;

                CenterMapOn(avgLat, avgLon, 16);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Error zooming to boundaries - {ex.Message}");
            }
        }

        /// <summary>
        /// Check if the overlay is initialized and ready.
        /// </summary>
        public static bool IsInitialized => _initialized;

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
                    Console.WriteLine("NOMAD: FlightPlanner type not found");
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
                    Console.WriteLine("NOMAD: FlightPlanner instance not found");
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
                    Console.WriteLine($"NOMAD: Injected fence into FlightPlanner.geofencepolygon ({vertices.Count} pts)");
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
                                    Console.WriteLine($"NOMAD: Also added fence to {fieldName}");
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
                                        Console.WriteLine($"NOMAD: Added fence to plan map overlay");
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
                catch { }
                try
                {
                    var redrawMethod = fpType.GetMethod("redrawPolygonSurvey",
                        BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
                    redrawMethod?.Invoke(fpInstance, null);
                }
                catch { }

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
                    Console.WriteLine($"NOMAD: FlightPlanner fence-related fields: {string.Join(", ", geoFields.Select(f => $"{f.Name}:{f.FieldType.Name}"))}");
                }

                return injected;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Error exporting to MP GeoFence - {ex.Message}");
                return false;
            }
        }
    }
}
