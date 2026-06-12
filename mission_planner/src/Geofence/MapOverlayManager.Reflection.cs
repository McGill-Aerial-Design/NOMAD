// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// MapOverlayManager.Reflection.cs - GMap.NET reflection plumbing
// ============================================================
// Type/assembly discovery, map-control lookup (with mymap ->
// gMapControl1 fallback), and overlay initialization. Drawing
// and removal helpers live in MapOverlayManager.cs.
// ============================================================

using System;
using System.Collections;
using System.Linq;
using System.Reflection;

namespace NOMAD.MissionPlanner
{
    public static partial class MapOverlayManager
    {
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
                            Log.Debug($"Found type {typeName} in {assembly.GetName().Name}");
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
                    // Try static field/property first
                    var mymapField = flightDataType.GetField("mymap",
                        BindingFlags.Public | BindingFlags.Static | BindingFlags.NonPublic);
                    _mapControl = mymapField?.GetValue(null)
                        ?? flightDataType.GetProperty("mymap",
                            BindingFlags.Public | BindingFlags.Static | BindingFlags.NonPublic)?.GetValue(null);
                    if (_mapControl != null)
                    {
                        Log.Info($"Found map control via static member (type: {_mapControl.GetType().FullName})");
                        return _mapControl;
                    }

                    // Try instance through MainV2.instance. MP exposes 'instance'
                    // and 'FlightData' as fields in some versions and properties in
                    // others — probe both member kinds.
                    var mainV2Type = FindTypeInLoadedAssemblies("MissionPlanner.MainV2");
                    if (mainV2Type != null)
                    {
                        var instanceProp = mainV2Type.GetProperty("instance", BindingFlags.Public | BindingFlags.Static);
                        var instance = instanceProp?.GetValue(null)
                            ?? mainV2Type.GetField("instance", BindingFlags.Public | BindingFlags.Static)?.GetValue(null);
                        if (instance != null)
                        {
                            // Try to get FlightData instance (property, then field,
                            // then static variants)
                            var flightDataProp = mainV2Type.GetProperty("FlightData", BindingFlags.Public | BindingFlags.Instance);
                            var flightData = flightDataProp?.GetValue(instance)
                                ?? mainV2Type.GetField("FlightData", BindingFlags.Public | BindingFlags.Instance)?.GetValue(instance)
                                ?? mainV2Type.GetProperty("FlightData", BindingFlags.Public | BindingFlags.Static)?.GetValue(null)
                                ?? mainV2Type.GetField("FlightData", BindingFlags.Public | BindingFlags.Static)?.GetValue(null);

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
                                        Log.Debug($"Found map control via instance (type: {_mapControl.GetType().FullName})");
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
                                            Log.Debug($"Primary field 'mymap' not found, using fallback field 'gMapControl1' (type: {_mapControl.GetType().FullName})");
                                            _fallbackUsedLogged = true;
                                        }
                                        return _mapControl;
                                    }
                                }
                            }
                        }
                    }
                }

                Log.Warn("Could not find map control - FlightData may not be loaded yet "
                    + "(probed FlightData.mymap static, MainV2.instance.FlightData.mymap/gMapControl1)");
            }
            catch (Exception ex)
            {
                Log.Error($"Error getting map control - {ex.Message}");
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
                    Log.Warn($"GMap types status - Overlay:{_overlayType != null}, Polygon:{_polygonType != null}, Point:{_pointType != null}");

                    // List all assemblies that might be GMap related
                    var gmapAssemblies = AppDomain.CurrentDomain.GetAssemblies()
                        .Where(a => a.GetName().Name.IndexOf("GMap", StringComparison.OrdinalIgnoreCase) >= 0)
                        .Select(a => a.GetName().Name);
                    Log.Warn($"Found GMap assemblies: {string.Join(", ", gmapAssemblies)}");

                    Log.Warn("Failed to load required GMap.NET types via reflection");
                    _initFailed = true;
                    return false;
                }

                // Get the map control
                var mymap = GetMapControl();
                if (mymap == null)
                {
                    Log.Warn("Map not available yet, will retry on next draw");
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
                Log.Info("Map overlay initialized successfully via reflection");
                return true;
            }
            catch (Exception ex)
            {
                Log.Error($"Failed to initialize map overlay - {ex.Message}");
                return false;
            }
        }
    }
}
