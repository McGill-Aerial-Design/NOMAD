// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// MapOverlayManager.ZoneRendering.cs - safety-zone map masks
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Reflection;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public static partial class MapOverlayManager
    {
        private static readonly List<Control> BoundaryMapControls = new List<Control>();
        private static List<GpsPoint> _renderHardBoundary = new List<GpsPoint>();
        private static List<GpsPoint> _renderSoftBoundary = new List<GpsPoint>();
        private static bool _boundaryRenderingConfigured;
        private static bool _paintErrorLogged;

        private static readonly Color OUTSIDE_HARD_FILL = Color.FromArgb(62, Color.Red);
        private static readonly Color WARNING_BAND_FILL = Color.FromArgb(82, Color.Yellow);

        private static void ConfigureBoundaryZoneRendering(GeofenceConfig config)
        {
            _renderHardBoundary = CloneVertices(config?.HardBoundary?.Vertices);
            _renderSoftBoundary = CloneVertices(config?.SoftBoundary?.Vertices);
            _boundaryRenderingConfigured = true;
            _paintErrorLogged = false;

            EnsureBoundaryMaps();
            InvalidateBoundaryMaps();
        }

        /// <summary>
        /// Attach the saved boundary renderer to the Data and Plan map controls.
        /// Safe to call repeatedly while Mission Planner creates its screens.
        /// </summary>
        public static int EnsureBoundaryMaps()
        {
            RemoveDisposedMapControls();
            BindBoundaryMap(GetMapControl(), "Data");
            BindBoundaryMap(GetPlanMapControl(), "Plan");
            MakeLegacyNomadFenceOutlineOnly();
            return BoundaryMapControls.Count;
        }

        /// <summary>
        /// Detach paint handlers when the plugin unloads.
        /// </summary>
        public static void StopBoundaryRendering()
        {
            foreach (var control in BoundaryMapControls.ToArray())
            {
                try { control.Paint -= BoundaryMap_Paint; }
                catch { }
            }
            BoundaryMapControls.Clear();
            _renderHardBoundary.Clear();
            _renderSoftBoundary.Clear();
            _boundaryRenderingConfigured = false;
        }

        public static bool BoundaryRenderingConfigured => _boundaryRenderingConfigured;

        private static List<GpsPoint> CloneVertices(IEnumerable<GpsPoint> vertices)
        {
            return vertices?.Select(point => new GpsPoint(point.Lat, point.Lon, point.Alt)).ToList()
                ?? new List<GpsPoint>();
        }

        private static void BindBoundaryMap(object map, string label)
        {
            var control = map as Control;
            if (control == null || control.IsDisposed || BoundaryMapControls.Contains(control))
                return;

            control.Paint += BoundaryMap_Paint;
            BoundaryMapControls.Add(control);
            Log.Info($"Boundary zone renderer attached to {label} map");
            control.Invalidate();
        }

        private static void RemoveDisposedMapControls()
        {
            for (int i = BoundaryMapControls.Count - 1; i >= 0; i--)
            {
                if (BoundaryMapControls[i] == null || BoundaryMapControls[i].IsDisposed)
                    BoundaryMapControls.RemoveAt(i);
            }
        }

        private static void InvalidateBoundaryMaps()
        {
            foreach (var control in BoundaryMapControls.ToArray())
            {
                if (!control.IsDisposed)
                    control.Invalidate();
            }
        }

        private static void BoundaryMap_Paint(object sender, PaintEventArgs e)
        {
            var map = sender as Control;
            if (map == null || _renderHardBoundary.Count < 3)
                return;

            try
            {
                var hardPoints = ProjectBoundary(map, _renderHardBoundary);
                if (hardPoints.Length < 3)
                    return;

                var softPoints = ProjectBoundary(map, _renderSoftBoundary);
                var oldSmoothing = e.Graphics.SmoothingMode;
                e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;

                using (var outsidePath = new GraphicsPath(FillMode.Alternate))
                using (var outsideBrush = new SolidBrush(OUTSIDE_HARD_FILL))
                {
                    outsidePath.AddRectangle(new RectangleF(-1, -1, map.Width + 2, map.Height + 2));
                    outsidePath.AddPolygon(hardPoints);
                    e.Graphics.FillPath(outsideBrush, outsidePath);
                }

                if (softPoints.Length >= 3)
                {
                    using (var warningPath = new GraphicsPath(FillMode.Alternate))
                    using (var warningBrush = new SolidBrush(WARNING_BAND_FILL))
                    {
                        warningPath.AddPolygon(hardPoints);
                        warningPath.AddPolygon(softPoints);
                        e.Graphics.FillPath(warningBrush, warningPath);
                    }
                }

                using (var hardPen = new Pen(HARD_BOUNDARY_STROKE, HARD_BOUNDARY_WIDTH))
                {
                    e.Graphics.DrawPolygon(hardPen, hardPoints);
                }

                if (softPoints.Length >= 3)
                {
                    using (var softPen = new Pen(SOFT_BOUNDARY_STROKE, SOFT_BOUNDARY_WIDTH))
                    {
                        e.Graphics.DrawPolygon(softPen, softPoints);
                    }
                }

                e.Graphics.SmoothingMode = oldSmoothing;
            }
            catch (Exception ex)
            {
                if (_paintErrorLogged)
                    return;

                _paintErrorLogged = true;
                Log.Error($"Boundary zone paint failed - {ex.Message}");
            }
        }

        private static PointF[] ProjectBoundary(object map, List<GpsPoint> vertices)
        {
            if (_pointType == null || vertices == null || vertices.Count < 3)
                return new PointF[0];

            var project = map.GetType().GetMethod(
                "FromLatLngToLocal",
                BindingFlags.Public | BindingFlags.Instance,
                null,
                new[] { _pointType },
                null);
            if (project == null)
                return new PointF[0];

            var points = new PointF[vertices.Count];
            for (int i = 0; i < vertices.Count; i++)
            {
                var gps = vertices[i];
                var latLng = Activator.CreateInstance(_pointType, new object[] { gps.Lat, gps.Lon });
                var local = project.Invoke(map, new[] { latLng });
                points[i] = new PointF(
                    ClampCoordinate(GetCoordinate(local, "X")),
                    ClampCoordinate(GetCoordinate(local, "Y")));
            }
            return points;
        }

        private static long GetCoordinate(object point, string memberName)
        {
            var value = GetMemberValue(point, memberName);
            return value == null ? 0L : Convert.ToInt64(value);
        }

        private static float ClampCoordinate(long value)
        {
            const long limit = 1000000;
            return Math.Max(-limit, Math.Min(limit, value));
        }

        private static void MakeLegacyNomadFenceOutlineOnly()
        {
            var planner = GetFlightPlannerInstance();
            if (planner == null)
                return;

            var geofenceOverlay = GetMemberValue(planner, "geofenceoverlay");
            var geofencePolygons = GetOverlayCollection(geofenceOverlay, "Polygons");
            if (geofencePolygons != null)
            {
                foreach (var polygon in geofencePolygons)
                {
                    var fill = GetMemberValue(polygon, "Fill") as SolidBrush;
                    if (IsNomadPolygon(polygon) && (fill == null || fill.Color.A != 0))
                        SetMemberValue(polygon, "Fill", new SolidBrush(Color.Transparent));
                }
            }

            var drawnOverlay = GetMemberValue(planner, "drawnpolygonsoverlay");
            var drawnPolygons = GetOverlayCollection(drawnOverlay, "Polygons");
            if (drawnPolygons == null)
                return;

            for (int i = drawnPolygons.Count - 1; i >= 0; i--)
            {
                if (IsNomadPolygon(drawnPolygons[i]))
                    drawnPolygons.RemoveAt(i);
            }
        }

        private static bool IsNomadPolygon(object polygon)
        {
            var name = GetMemberValue(polygon, "Name")?.ToString();
            return name != null && name.StartsWith("NOMAD_", StringComparison.OrdinalIgnoreCase);
        }
    }
}
