// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>Decode one cached advisory polygon. This does not verify an aircraft fence transfer.</summary>
    internal static class NativeFenceImport
    {
        // Reviewed ArduPilot encoding uses a byte polygon count, independently of aircraft storage capacity.
        internal const int MaximumVertices = 255;
        private const ushort ReturnPointCommand = 5000;
        private const ushort InclusionVertexCommand = 5001;

        internal static bool TryDecode(KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>[] snapshot,
            out List<GpsPoint> points, out string error)
        {
            points = null;
            error = "Only one complete cached inclusion polygon is supported.";
            if (snapshot == null || snapshot.Length < 3 || snapshot.Length > MaximumVertices + 1)
            {
                return false;
            }
            var ordered = snapshot.OrderBy(entry => entry.Key).ToArray();
            for (int index = 0; index < ordered.Length; index++)
            {
                var item = ordered[index].Value;
                // Native float MISSION_ITEM passes through Locationwp, losing mission_type to zero.
                // The selected fencepoints field owns cache classification; other mission types remain invalid.
                if (ordered[index].Key != index || item.seq != index || !IsGlobalFrame(item.frame) ||
                    (item.mission_type != 0 && item.mission_type != (byte)MAVLink.MAV_MISSION_TYPE.FENCE) ||
                    !GeoMath.IsValidCoordinate(item.x / 1e7, item.y / 1e7))
                {
                    error = "Cached fence has missing sequences, unsupported metadata or invalid coordinates.";
                    return false;
                }
            }
            return DecodePolygon(ordered.Select(entry => entry.Value).ToArray(), out points, out error);
        }

        private static bool DecodePolygon(MAVLink.mavlink_mission_item_int_t[] items,
            out List<GpsPoint> points, out string error)
        {
            points = null;
            error = "Cached fence contains unsupported shapes, incomplete counts or inconsistent groups.";
            int start = items[0].command == ReturnPointCommand ? 1 : 0;
            int end = items.Length;
            if (start == 0 && items[end - 1].command == ReturnPointCommand)
            {
                if (items[end - 1].param1 != 0)
                {
                    return false;
                }
                end--;
            }
            if (end - start < 3)
            {
                return false;
            }
            var first = items[start];
            int count = end - start;
            if (first.param1 != count || count > MaximumVertices || !IsGroup(first.param2))
            {
                return false;
            }
            for (int index = start; index < end; index++)
            {
                var item = items[index];
                if (item.command != InclusionVertexCommand ||
                    item.param1 != count || item.param2 != first.param2)
                {
                    return false;
                }
            }
            var polygon = items.Skip(start).Take(count).ToArray();
            if (IsLegacy(items, start, end))
            {
                polygon = polygon.Take(count - 1).ToArray();
            }
            else if (start == 1 && items[0].param1 != 0)
            {
                return false;
            }
            if (!IsSimple(polygon))
            {
                error = "Cached polygon is degenerate, repeated, self-intersecting or crosses the longitude seam.";
                return false;
            }
            points = polygon.Select(item => new GpsPoint(item.x / 1e7, item.y / 1e7)).ToList();
            error = null;
            return true;
        }

        private static bool IsLegacy(MAVLink.mavlink_mission_item_int_t[] items, int start, int end)
        {
            return items.Length <= byte.MaxValue && start == 1 && end == items.Length &&
                items[0].frame == (byte)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT &&
                items[0].param1 == end - start && items.All(item =>
                    item.frame == (byte)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT && item.param2 == 0) &&
                items[start].x == items[end - 1].x && items[start].y == items[end - 1].y;
        }

        private static bool IsGlobalFrame(byte frame)
        {
            return frame == (byte)MAVLink.MAV_FRAME.GLOBAL || frame == (byte)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT ||
                frame == (byte)MAVLink.MAV_FRAME.GLOBAL_INT || frame == (byte)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT ||
                frame == (byte)MAVLink.MAV_FRAME.GLOBAL_TERRAIN_ALT || frame == (byte)MAVLink.MAV_FRAME.GLOBAL_TERRAIN_ALT_INT;
        }

        private static bool IsGroup(float group)
        {
            return !float.IsNaN(group) && !float.IsInfinity(group) && group >= 0 && group == Math.Truncate(group);
        }

        private static bool IsSimple(MAVLink.mavlink_mission_item_int_t[] polygon)
        {
            if (polygon.Length < 3)
            {
                return false;
            }
            decimal area = 0;
            var seen = new HashSet<Tuple<int, int>>();
            for (int index = 0; index < polygon.Length; index++)
            {
                var a = polygon[index];
                var b = polygon[(index + 1) % polygon.Length];
                if (!seen.Add(Tuple.Create(a.x, a.y)) || Math.Abs((long)a.y - b.y) > 1800000000L)
                {
                    return false;
                }
                area += (decimal)a.x * b.y - (decimal)b.x * a.y;
                for (int other = index + 1; other < polygon.Length; other++)
                {
                    if (other == index + 1 || (index == 0 && other == polygon.Length - 1))
                    {
                        continue;
                    }
                    if (Intersects(a, b, polygon[other], polygon[(other + 1) % polygon.Length]))
                    {
                        return false;
                    }
                }
            }
            return area != 0;
        }

        // Decimal products keep exact E7 orientation without overflowing signed 64-bit arithmetic.
        private static decimal Cross(MAVLink.mavlink_mission_item_int_t a,
            MAVLink.mavlink_mission_item_int_t b, MAVLink.mavlink_mission_item_int_t c)
        {
            return ((decimal)b.x - a.x) * ((decimal)c.y - a.y) -
                ((decimal)b.y - a.y) * ((decimal)c.x - a.x);
        }

        private static bool Intersects(MAVLink.mavlink_mission_item_int_t a, MAVLink.mavlink_mission_item_int_t b,
            MAVLink.mavlink_mission_item_int_t c, MAVLink.mavlink_mission_item_int_t d)
        {
            if (Math.Max(a.x, b.x) < Math.Min(c.x, d.x) || Math.Max(c.x, d.x) < Math.Min(a.x, b.x) ||
                Math.Max(a.y, b.y) < Math.Min(c.y, d.y) || Math.Max(c.y, d.y) < Math.Min(a.y, b.y))
            {
                return false;
            }
            return Math.Sign(Cross(a, b, c)) * Math.Sign(Cross(a, b, d)) <= 0 &&
                Math.Sign(Cross(c, d, a)) * Math.Sign(Cross(c, d, b)) <= 0;
        }
    }
}
