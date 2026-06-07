// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// BuildingViewer3D.Data.cs - Geometry, picking, and data model
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using OpenTK;

namespace NOMAD.MissionPlanner
{
    public partial class BuildingViewer3D
    {
        // ==================== Data classes ====================

        public class Corner
        {
            public string Name;
            public double Lat;
            public double Lon;
            public float East;
            public float North;
        }

        public class Target
        {
            public string Id;
            public string Color;
            public string Description;
            public float East;
            public float North;
            public float Up;
        }

        public class Placement
        {
            public string Surface;
            public string WallName;
            public string NearestCornerName;
            public float East;
            public float North;
            public float Up;
            public float DistanceFromCornerM;
            public string StructureLabel;
            public string ReferenceWallName;
            public float DistanceFromReferenceWallM;
        }

        // ==================== Public API ====================

        public void SetHighlightedTarget(string id)
        {
            if (HighlightedTargetId == id) return;
            HighlightedTargetId = id;
            _glControl?.Invalidate();
        }

        public void SetBuildingHeight(double heightM)
        {
            _buildingHeight = (float)Math.Max(0.5, heightM);
            _glControl?.Invalidate();
        }

        public void SetCorners(IList<Corner> corners, double? centerLat = null, double? centerLon = null)
        {
            _corners.Clear();
            if (corners == null || corners.Count == 0)
            {
                _hasProjectionOrigin = false;
                _hasDronePose = false;
                _glControl?.Invalidate();
                return;
            }

            double lat0 = centerLat ?? corners.Average(c => c.Lat);
            double lon0 = centerLon ?? corners.Average(c => c.Lon);
            double cosLat = Math.Cos(lat0 * Math.PI / 180.0);
            _originLat = lat0;
            _originLon = lon0;
            _originCosLat = cosLat;
            _hasProjectionOrigin = true;
            const double METERS_PER_DEG = 111320.0;

            foreach (var c in corners)
            {
                _corners.Add(new Corner
                {
                    Name = c.Name,
                    Lat = c.Lat,
                    Lon = c.Lon,
                    East = (float)((c.Lon - lon0) * cosLat * METERS_PER_DEG),
                    North = (float)((c.Lat - lat0) * METERS_PER_DEG),
                });
            }

            float cx = _corners.Average(c => c.East);
            float cy = _corners.Average(c => c.North);
            _recenterEast = cx;
            _recenterNorth = cy;
            foreach (var c in _corners) { c.East -= cx; c.North -= cy; }
            UpdateSceneBounds();

            _glControl?.Invalidate();
        }

        public void SetDronePoseGps(double lat, double lon, double altAglM,
            double yawDeg, double pitchDeg, double rollDeg)
        {
            if (!_hasProjectionOrigin || Math.Abs(lat) < 0.000001 || Math.Abs(lon) < 0.000001)
            {
                _hasDronePose = false;
                _glControl?.Invalidate();
                return;
            }

            const double METERS_PER_DEG = 111320.0;
            _droneEast = (float)((lon - _originLon) * _originCosLat * METERS_PER_DEG - _recenterEast);
            _droneNorth = (float)((lat - _originLat) * METERS_PER_DEG - _recenterNorth);
            _droneUp = (float)Math.Max(0.0, altAglM);
            _droneYawRad = (float)(yawDeg * Math.PI / 180.0);
            _dronePitchRad = (float)(pitchDeg * Math.PI / 180.0);
            _droneRollRad = (float)(rollDeg * Math.PI / 180.0);
            _hasDronePose = true;
            _glControl?.Invalidate();
        }

        public bool TryGetLocalFromGps(double lat, double lon, out float east, out float north)
        {
            east = 0f;
            north = 0f;
            if (!_hasProjectionOrigin) return false;

            const double METERS_PER_DEG = 111320.0;
            east = (float)((lon - _originLon) * _originCosLat * METERS_PER_DEG - _recenterEast);
            north = (float)((lat - _originLat) * METERS_PER_DEG - _recenterNorth);
            return true;
        }

        public Placement CreatePlacementFromLocal(string surface, float east, float north, float up)
        {
            if (_corners.Count < 3) return null;
            string normalized = string.IsNullOrWhiteSpace(surface)
                ? "ground"
                : surface.Trim().ToLowerInvariant();
            float clampedUp = normalized == "roof"
                ? _buildingHeight
                : Math.Max(0f, Math.Min(_buildingHeight, up));
            return BuildPlacement(
                normalized,
                normalized == "wall" ? (int?)NearestWallIndex(east, north) : null,
                new Vector3(east, clampedUp, -north));
        }

        public void SetTargets(IList<Target> targets)
        {
            _targets.Clear();
            if (targets != null)
            {
                foreach (var t in targets) _targets.Add(t);
            }
            _glControl?.Invalidate();
        }

        // ==================== Picking ====================

        private string PickTarget(Point screen)
        {
            if (_targets.Count == 0 || _viewW == 0 || _viewH == 0) return null;

            string best = null;
            float bestDist = 14f * 14f;

            foreach (var t in _targets)
            {
                Vector4 p = new Vector4(t.East, t.Up, -t.North, 1f);
                Vector4 clip = Vector4.Transform(p, _viewProj);
                if (clip.W <= 0.0001f) continue;
                float ndcX = clip.X / clip.W;
                float ndcY = clip.Y / clip.W;
                if (ndcX < -1f || ndcX > 1f || ndcY < -1f || ndcY > 1f) continue;

                float sx = (ndcX * 0.5f + 0.5f) * _viewW;
                float sy = (1f - (ndcY * 0.5f + 0.5f)) * _viewH;
                float dx = sx - screen.X;
                float dy = sy - screen.Y;
                float d2 = dx * dx + dy * dy;
                if (d2 < bestDist)
                {
                    bestDist = d2;
                    best = t.Id;
                }
            }
            return best;
        }

        private Placement PickPlacement(Point screen)
        {
            if (_corners.Count < 3 || _viewW == 0 || _viewH == 0) return null;
            if (!TryBuildPickRay(screen, out var origin, out var dir)) return null;

            Placement best = null;
            float bestT = float.MaxValue;

            TryPickHorizontal("ground", 0f, origin, dir, false, ref best, ref bestT);
            TryPickHorizontal("roof", _buildingHeight, origin, dir, true, ref best, ref bestT);
            TryPickWalls(origin, dir, ref best, ref bestT);

            return best;
        }

        private bool TryBuildPickRay(Point screen, out Vector3 origin, out Vector3 dir)
        {
            origin = Vector3.Zero;
            dir = Vector3.Zero;

            float ndcX = (2f * screen.X) / Math.Max(1, _viewW) - 1f;
            float ndcY = 1f - (2f * screen.Y) / Math.Max(1, _viewH);

            Matrix4 viewProj = _viewProj;
            Matrix4.Invert(ref viewProj, out var inv);
            var near = Vector4.Transform(new Vector4(ndcX, ndcY, -1f, 1f), inv);
            var far = Vector4.Transform(new Vector4(ndcX, ndcY, 1f, 1f), inv);
            if (Math.Abs(near.W) < 0.0001f || Math.Abs(far.W) < 0.0001f) return false;
            near /= near.W;
            far /= far.W;

            origin = new Vector3(near.X, near.Y, near.Z);
            dir = Vector3.Normalize(new Vector3(far.X - near.X, far.Y - near.Y, far.Z - near.Z));
            return dir.LengthSquared > 0.0001f;
        }

        private void TryPickHorizontal(string surface, float y, Vector3 origin, Vector3 dir,
            bool requireInsideFootprint, ref Placement best, ref float bestT)
        {
            if (Math.Abs(dir.Y) < 0.0001f) return;
            float t = (y - origin.Y) / dir.Y;
            if (t <= 0f || t >= bestT) return;

            var p = origin + dir * t;
            bool inside = PointInsideFootprint(p.X, p.Z);
            if (requireInsideFootprint && !inside) return;
            if (!requireInsideFootprint && DistanceToFootprintSquared(p.X, p.Z) > _searchBufferM * _searchBufferM) return;

            bestT = t;
            best = BuildPlacement(surface, null, p);
        }

        private void TryPickWalls(Vector3 origin, Vector3 dir, ref Placement best, ref float bestT)
        {
            for (int i = 0; i < _corners.Count; i++)
            {
                var ca = _corners[i];
                var cb = _corners[(i + 1) % _corners.Count];
                var a = new Vector3(ca.East, 0f, -ca.North);
                var b = new Vector3(cb.East, 0f, -cb.North);
                var edge = b - a;
                if (edge.LengthSquared < 0.0001f) continue;

                var normal = Vector3.Normalize(new Vector3(edge.Z, 0f, -edge.X));
                float denom = Vector3.Dot(dir, normal);
                if (Math.Abs(denom) < 0.0001f) continue;

                float t = Vector3.Dot(a - origin, normal) / denom;
                if (t <= 0f || t >= bestT) continue;

                var p = origin + dir * t;
                if (p.Y < -0.01f || p.Y > _buildingHeight + 0.01f) continue;

                float along = Vector3.Dot(p - a, edge) / edge.LengthSquared;
                if (along < -0.01f || along > 1.01f) continue;

                bestT = t;
                best = BuildPlacement("wall", i, p);
            }
        }

        private Placement BuildPlacement(string surface, int? wallIndex, Vector3 glPoint)
        {
            float east = glPoint.X;
            float north = -glPoint.Z;
            int faceIndex = wallIndex ?? NearestWallIndex(east, north);
            string nearestName = "";
            float nearest = float.MaxValue;

            for (int i = 0; i < _corners.Count; i++)
            {
                var c = _corners[i];
                float d2 = Distance2(east, north, c.East, c.North);
                if (d2 < nearest)
                {
                    nearest = d2;
                    nearestName = NameOrIndex(c.Name, i);
                }
            }

            var reference = ReferenceWallForPlacement(faceIndex, east, north);
            bool onProtrusion = IsProtrudingWallIndex(faceIndex) ||
                ((surface == "roof" || surface == "ground") && PointInProtrudingPortion(east, north));

            return new Placement
            {
                Surface = surface,
                WallName = WallLabelForEdge(faceIndex),
                NearestCornerName = nearestName,
                East = east,
                North = north,
                Up = Math.Max(0f, Math.Min(_buildingHeight, glPoint.Y)),
                DistanceFromCornerM = (float)Math.Sqrt(Math.Max(0f, nearest)),
                StructureLabel = onProtrusion ? "portion of the building that sticks out" : "building",
                ReferenceWallName = reference.wall,
                DistanceFromReferenceWallM = reference.distance,
            };
        }

        // ==================== Geometry helpers ====================

        private bool PointInsideFootprint(float glX, float glZ)
        {
            bool inside = false;
            for (int i = 0, j = _corners.Count - 1; i < _corners.Count; j = i++)
            {
                float xi = _corners[i].East;
                float zi = -_corners[i].North;
                float xj = _corners[j].East;
                float zj = -_corners[j].North;
                bool intersect = ((zi > glZ) != (zj > glZ)) &&
                    (glX < (xj - xi) * (glZ - zi) / ((zj - zi) == 0f ? 0.0001f : (zj - zi)) + xi);
                if (intersect) inside = !inside;
            }
            return inside;
        }

        private float DistanceToFootprintSquared(float glX, float glZ)
        {
            if (PointInsideFootprint(glX, glZ)) return 0f;
            float best = float.MaxValue;
            for (int i = 0; i < _corners.Count; i++)
            {
                var a = _corners[i];
                var b = _corners[(i + 1) % _corners.Count];
                float ax = a.East, az = -a.North;
                float bx = b.East, bz = -b.North;
                float vx = bx - ax, vz = bz - az;
                float len2 = vx * vx + vz * vz;
                if (len2 < 0.0001f) continue;
                float t = ((glX - ax) * vx + (glZ - az) * vz) / len2;
                t = Math.Max(0f, Math.Min(1f, t));
                float px = ax + vx * t;
                float pz = az + vz * t;
                best = Math.Min(best, Distance2(glX, glZ, px, pz));
            }
            return best;
        }

        private string NearestWallLabel(float east, float north)
            => WallLabelForEdge(NearestWallIndex(east, north));

        private int NearestWallIndex(float east, float north)
        {
            if (_corners.Count < 2) return 0;
            int bestIdx = 0;
            float best = float.MaxValue;
            for (int i = 0; i < _corners.Count; i++)
            {
                var a = _corners[i];
                var b = _corners[(i + 1) % _corners.Count];
                float vx = b.East - a.East;
                float vy = b.North - a.North;
                float len2 = vx * vx + vy * vy;
                if (len2 < 0.0001f) continue;
                float t = ((east - a.East) * vx + (north - a.North) * vy) / len2;
                t = Math.Max(0f, Math.Min(1f, t));
                float px = a.East + vx * t;
                float py = a.North + vy * t;
                float d = Distance2(east, north, px, py);
                if (d < best)
                {
                    best = d;
                    bestIdx = i;
                }
            }
            return bestIdx;
        }

        private (string wall, float distance) ReferenceWallForPlacement(int edgeIndex, float east, float north)
        {
            if (_corners.Count < 2)
                return ("nearest", 0f);

            var a = _corners[edgeIndex];
            var b = _corners[(edgeIndex + 1) % _corners.Count];
            float vx = b.East - a.East;
            float vy = b.North - a.North;
            float len2 = Math.Max(0.0001f, vx * vx + vy * vy);
            float t = ((east - a.East) * vx + (north - a.North) * vy) / len2;
            t = Math.Max(0f, Math.Min(1f, t));
            float px = a.East + vx * t;
            float py = a.North + vy * t;

            string face = WallLabelForEdge(edgeIndex);
            bool eastWestFace = face.Contains("north") || face.Contains("south");
            bool useFirst = eastWestFace
                ? a.East <= b.East
                : a.North <= b.North;
            var refCorner = useFirst ? a : b;
            string refWall = eastWestFace ? "western" : "southern";
            float distance = (float)Math.Sqrt(Distance2(px, py, refCorner.East, refCorner.North));
            return (refWall, distance);
        }

        private string WallLabelForEdge(int i)
        {
            if (_corners.Count < 2) return "wall";
            if (IsProtrudingWallIndex(i))
                return ProtrudingSectionParentWall() ?? EdgeOutwardCompass(i);
            return EdgeOutwardCompass(i);
        }

        private string EdgeOutwardCompass(int i)
        {
            if (!TryEdgeOutwardNormal(i, out float ox, out float oy))
                return "wall";
            float heading = (float)((90.0 - Math.Atan2(oy, ox) * 180.0 / Math.PI + 360.0) % 360.0);
            return SnapHeadingToCompass(heading);
        }

        private bool TryEdgeOutwardNormal(int i, out float nx, out float ny)
        {
            nx = ny = 0f;
            int n = _corners.Count;
            if (n < 2) return false;
            var a = _corners[i];
            var b = _corners[(i + 1) % n];
            float vx = b.East - a.East;
            float vy = b.North - a.North;
            float len = (float)Math.Sqrt(vx * vx + vy * vy);
            if (len < 1e-6f) return false;
            nx = vy / len;
            ny = -vx / len;
            float mx = (a.East + b.East) * 0.5f;
            float my = (a.North + b.North) * 0.5f;
            const float probe = 0.5f;
            if (PointInPolygon(mx + nx * probe, my + ny * probe))
            {
                nx = -nx;
                ny = -ny;
            }
            return true;
        }

        private string ProtrudingSectionParentWall()
        {
            int n = _corners.Count;
            float sx = 0, sy = 0;
            for (int i = 0; i < n; i++)
            {
                if (!IsProtrudingWallIndex(i)) continue;
                if (!TryEdgeOutwardNormal(i, out float nx, out float ny)) continue;
                sx += nx;
                sy += ny;
            }
            if (sx * sx + sy * sy < 1e-6f) return null;
            float heading = (float)((90.0 - Math.Atan2(sy, sx) * 180.0 / Math.PI + 360.0) % 360.0);
            return SnapHeadingToCompass(heading);
        }

        private bool PointInPolygon(float px, float py)
        {
            bool inside = false;
            int n = _corners.Count;
            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                var pi = _corners[i];
                var pj = _corners[j];
                if (((pi.North > py) != (pj.North > py)) &&
                    (px < (pj.East - pi.East) * (py - pi.North) / (pj.North - pi.North) + pi.East))
                    inside = !inside;
            }
            return inside;
        }

        private string WallDisplayLabelForEdge(int i)
        {
            string face = WallLabelForEdge(i);
            return IsProtrudingWallIndex(i)
                ? $"{face} - portion that sticks out"
                : face;
        }

        private bool IsProtrudingWallIndex(int edgeIndex)
        {
            return _corners.Count == 8 && edgeIndex >= 4 && edgeIndex <= 6;
        }

        private bool PointInProtrudingPortion(float east, float north)
        {
            if (_corners.Count != 8) return false;
            var protrusion = new[] { _corners[4], _corners[5], _corners[6], _corners[7] };
            float minE = protrusion.Min(c => c.East) - 0.05f;
            float maxE = protrusion.Max(c => c.East) + 0.05f;
            float minN = protrusion.Min(c => c.North) - 0.05f;
            float maxN = protrusion.Max(c => c.North) + 0.05f;
            return east >= minE && east <= maxE && north >= minN && north <= maxN;
        }

        private float EdgeHeading(int i)
        {
            var a = _corners[i];
            var b = _corners[(i + 1) % _corners.Count];
            float cx = _corners.Average(c => c.East);
            float cy = _corners.Average(c => c.North);
            float ox = (a.East + b.East) * 0.5f - cx;
            float oy = (a.North + b.North) * 0.5f - cy;
            return (float)((90.0 - Math.Atan2(oy, ox) * 180.0 / Math.PI + 360.0) % 360.0);
        }

        private static string SnapHeadingToCompass(float headingDeg)
        {
            string[] names = { "north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest" };
            int idx = (int)Math.Round((((headingDeg % 360f) + 360f) % 360f) / 45f) % 8;
            return names[idx];
        }

        private void UpdateSceneBounds()
        {
            if (_corners.Count == 0)
            {
                _minEast = _minNorth = -10f;
                _maxEast = _maxNorth = 10f;
                _sceneHalfExtentM = 25f;
                _distance = Math.Max(_distance, 25f);
                return;
            }
            _minEast = _corners.Min(c => c.East);
            _maxEast = _corners.Max(c => c.East);
            _minNorth = _corners.Min(c => c.North);
            _maxNorth = _corners.Max(c => c.North);
            float halfW = Math.Max(Math.Abs(_minEast), Math.Abs(_maxEast));
            float halfH = Math.Max(Math.Abs(_minNorth), Math.Abs(_maxNorth));
            _sceneHalfExtentM = Math.Max(25f, Math.Max(halfW, halfH) + _searchBufferM + 5f);
            _distance = Math.Max(_distance, _sceneHalfExtentM * 1.4f);
        }

        private static float Distance2(float x1, float y1, float x2, float y2)
        {
            float dx = x1 - x2;
            float dy = y1 - y2;
            return dx * dx + dy * dy;
        }
    }
}
