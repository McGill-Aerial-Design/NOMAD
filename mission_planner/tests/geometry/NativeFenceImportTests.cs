// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Linq;
using NOMAD.MissionPlanner;

internal static class NativeFenceImportTests
{
    internal static KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>[] Polygon(int count = 5)
    {
        var coordinates = new[] { (0, 0), (0, 20000000), (10000000, 10000000),
            (20000000, 20000000), (20000000, 0) };
        return Enumerable.Range(0, count).Select(index =>
        {
            var item = new MAVLink.mavlink_mission_item_int_t
            {
                seq = (ushort)index, command = 5001, param1 = count, mission_type = 1,
                x = count == 5 ? coordinates[index].Item1 : (int)(Math.Cos(index * 2 * Math.PI / count) * 10000000),
                y = count == 5 ? coordinates[index].Item2 : (int)(Math.Sin(index * 2 * Math.PI / count) * 10000000),
            };
            return new KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>(index, item);
        }).ToArray();
    }

    private static void Require(bool condition, string message)
    {
        if (!condition) { throw new Exception(message); }
    }

    internal static void AssertConcave(List<GpsPoint> points, string message)
    {
        var expected = new[] { (0.0, 0.0), (0.0, 2.0), (1.0, 1.0), (2.0, 2.0), (2.0, 0.0) };
        Require(points != null && points.Count == expected.Length, message + " count");
        for (int index = 0; index < expected.Length; index++)
        {
            Require(points[index].Lat == expected[index].Item1 && points[index].Lon == expected[index].Item2,
                message + " coordinate " + index);
        }
    }

    private static void Reject(KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>[] entries, string message)
    {
        bool accepted = NativeFenceImport.TryDecode(entries, out var points, out var error);
        Require(!accepted && points == null && !string.IsNullOrEmpty(error), message);
    }

    private static KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>[] Change(
        KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>[] entries, int index,
        Func<MAVLink.mavlink_mission_item_int_t, MAVLink.mavlink_mission_item_int_t> change)
    {
        var copy = entries.ToArray();
        copy[index] = new KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>(copy[index].Key, change(copy[index].Value));
        return copy;
    }

    internal static KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>[] WithReturn(
        KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>[] polygon, bool last = false, bool legacy = false)
    {
        var items = polygon.Select(entry => entry.Value).ToList();
        if (legacy) { items.Add(items[0]); }
        var point = new MAVLink.mavlink_mission_item_int_t
        {
            command = 5000, mission_type = 1, param1 = legacy ? items.Count : 0,
            frame = (byte)(legacy ? 3 : 0), x = 12300000, y = 45600000,
        };
        if (last) { items.Add(point); } else { items.Insert(0, point); }
        return items.Select((item, index) =>
        {
            item.seq = (ushort)index;
            if (legacy) { item.frame = 3; item.param1 = items.Count - 1; }
            return new KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>(index, item);
        }).ToArray();
    }

    internal static void Run()
    {
        var original = Polygon();
        Require(NativeFenceImport.TryDecode(original.Reverse().ToArray(), out var points, out _), "shuffled concave import");
        AssertConcave(points, "exact ordered E7 coordinates and legal zeros");
        var fractional = original.Select(entry => { var item = entry.Value; item.x += 12345678; item.y -= 23456789;
            return new KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>(entry.Key, item); }).ToArray();
        var fractionalBefore = fractional.ToArray();
        Require(NativeFenceImport.TryDecode(fractional.Reverse().ToArray(), out points, out _), "fractional negative E7 import");
        var fractionalExpected = new[] { (1.2345678, -2.3456789), (1.2345678, -0.3456789),
            (2.2345678, -1.3456789), (3.2345678, -0.3456789), (3.2345678, -2.3456789) };
        for (int index = 0; index < fractionalExpected.Length; index++)
        {
            Require(points[index].Lat == fractionalExpected[index].Item1 && points[index].Lon == fractionalExpected[index].Item2,
                "fractional E7 coordinate " + index);
        }
        Require(fractionalBefore.SequenceEqual(fractional), "fractional source payload unchanged");
        foreach (byte frame in new byte[] { 0, 3, 5, 6, 10, 11 })
        {
            var entries = original.Select(entry =>
            {
                var item = entry.Value; item.frame = frame;
                return new KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>(entry.Key, item);
            }).ToArray();
            Require(NativeFenceImport.TryDecode(entries, out points, out _), "supported frame " + frame);
            AssertConcave(points, "frame " + frame);
        }
        foreach (bool last in new[] { false, true })
        {
            Require(NativeFenceImport.TryDecode(WithReturn(original, last), out points, out _) && points.Count == 5 &&
                points[0].Lat == 0, "modern return excluded before/after polygon");
            AssertConcave(points, "modern return " + last);
        }
        Require(NativeFenceImport.TryDecode(WithReturn(original, legacy: true), out points, out _) && points.Count == 5,
            "legacy distinct return and closing point");
        AssertConcave(points, "legacy polygon");
        Require(NativeFenceImport.TryDecode(Polygon(255), out points, out _) && points.Count == 255, "255 supported vertices");
        Require(NativeFenceImport.TryDecode(WithReturn(Polygon(255), last: true), out points, out _) && points.Count == 255,
            "255 modern vertices plus return");
        Reject(Polygon(256), "256 vertex encoding rejected");
        Require(NativeFenceImport.TryDecode(WithReturn(Polygon(253), legacy: true), out points, out _) && points.Count == 253,
            "legacy total255 accepted");
        Reject(WithReturn(Polygon(254), legacy: true), "legacy total256 rejected");
        Reject(null, "absent cache"); Reject(Array.Empty<KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>>(), "empty cache");
        Reject(original.Take(2).ToArray(), "incomplete vertex count");
        Reject(original.Where(entry => entry.Key != 2).ToArray(), "gap");
        var duplicateKey = original.ToArray(); duplicateKey[1] = duplicateKey[0]; Reject(duplicateKey, "duplicate key");
        foreach (ushort command in new ushort[] { 5000, 5002, 5003, 5004, 16 })
        {
            Reject(Change(original, 2, item => { item.command = command; return item; }), "unsupported/interrupted shape");
        }
        Reject(Change(original, 1, item => { item.seq = 4; return item; }), "key/sequence mismatch");
        Reject(Change(original, 1, item => { item.frame = 1; return item; }), "local frame");
        Reject(Change(original, 1, item => { item.mission_type = 2; return item; }), "rally metadata");
        Reject(Change(original, 1, item => { item.x = int.MaxValue; return item; }), "latitude sentinel");
        Reject(Change(original, 1, item => { item.y = -1800000001; return item; }), "longitude range");
        foreach (float value in new[] { float.NaN, float.PositiveInfinity, -1, 4.5f, 256 })
        {
            Reject(Change(original, 0, item => { item.param1 = value; return item; }), "bad count");
        }
        foreach (float value in new[] { float.NaN, float.PositiveInfinity, -1, .5f, 1 })
        {
            Reject(Change(original, 0, item => { item.param2 = value; return item; }), "invalid/inconsistent group");
        }
        var two = Polygon(6).Select(entry => { var item = entry.Value; item.param1 = 3;
            return new KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>(entry.Key, item); }).ToArray();
        Reject(two, "two polygons cannot be flattened");
        Reject(Change(original, 1, item => { item.x = original[0].Value.x; item.y = original[0].Value.y; return item; }),
            "repeated vertex");
        var bowtie = Polygon(4); var swap = bowtie[1].Value; var other = bowtie[2].Value;
        swap.x = other.x; swap.y = other.y; other.x = bowtie[1].Value.x; other.y = bowtie[1].Value.y;
        bowtie = Change(bowtie, 1, _ => swap); bowtie = Change(bowtie, 2, _ => other); Reject(bowtie, "self intersection");
        var collinear = Polygon(3).Select((entry, index) => { var item = entry.Value; item.x = index; item.y = index;
            return new KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>(index, item); }).ToArray();
        Reject(collinear, "zero area");
        var wrapped = Polygon(4);
        var seam = new[] { (10000000, 1790000000), (10000000, -1790000000),
            (20000000, -1790000000), (20000000, 1790000000) };
        wrapped = wrapped.Select((entry, index) => { var item = entry.Value; item.x = seam[index].Item1;
            item.y = seam[index].Item2; return new KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>(index, item); }).ToArray();
        Reject(wrapped, "wrapped longitude seam is unsupported by planar display");
        var nearby = wrapped.Select(entry => { var item = entry.Value; if (item.y < 0) { item.y = 1780000000; }
            return new KeyValuePair<int, MAVLink.mavlink_mission_item_int_t>(entry.Key, item); }).ToArray();
        Require(NativeFenceImport.TryDecode(nearby, out points, out _) && points.Count == 4 && points[1].Lon == 178,
            "nearby nonwrapped polygon remains accepted");
        Console.WriteLine("PASS native fence: ordering, concavity, E7/zeros, six frames, returns, encoding bounds and malformed shapes");
    }
}
