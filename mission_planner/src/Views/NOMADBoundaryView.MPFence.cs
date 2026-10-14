// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADBoundaryView
    {
        private void BtnGetFromMP_Click(object sender, EventArgs e)
        {
            try
            {
                var port = MainV2.comPort;
                var mav = port?.MAV;
                var cache = mav?.fencepoints;
                if (cache == null || cache.Count > NativeFenceImport.MaximumVertices + 1)
                {
                    CustomMessageBox.Show("No supported cached fence polygon. Use native MP tools to read the fence.",
                        "Cached Advisory Import");
                    return;
                }
                var snapshot = cache.ToArray();
                if (!NativeFenceImport.TryDecode(snapshot, out var points, out var error))
                {
                    CustomMessageBox.Show(error, "Cached Advisory Import");
                    return;
                }
                byte system = mav.sysid, component = mav.compid;
                var stream = port.BaseStream;
                bool wasOpen = stream?.IsOpen ?? false;
                if (IsDisposed || Disposing)
                {
                    return;
                }
                var ownerHandle = Handle;
                var choice = MessageBox.Show(this,
                    $"Copy {points.Count} cached polygon vertices into the INNER (Yes) or OUTER (No) advisory outline?\n" +
                    "Cancel preserves both outlines. This replaces local geometry only; cached data does not verify " +
                    "a complete/current aircraft fence or containment.", "Cached Advisory Import",
                    MessageBoxButtons.YesNoCancel, MessageBoxIcon.Question, MessageBoxDefaultButton.Button3);
                if (choice != DialogResult.Yes && choice != DialogResult.No)
                {
                    return;
                }
                if (IsDisposed || Disposing || !IsHandleCreated || Handle != ownerHandle ||
                    !ReferenceEquals(MainV2.comPort, port) || !ReferenceEquals(port.MAV, mav) ||
                    mav.sysid != system || mav.compid != component || !ReferenceEquals(mav.fencepoints, cache) ||
                    !ReferenceEquals(port.BaseStream, stream) || (stream?.IsOpen ?? false) != wasOpen ||
                    !snapshot.OrderBy(entry => entry.Key).SequenceEqual(cache.ToArray().OrderBy(entry => entry.Key)))
                {
                    CustomMessageBox.Show("Mission Planner context/cache changed; import cancelled.", "Cached Advisory Import");
                    return;
                }
                if (!ApplyOutlinePoints(points, choice, out error))
                {
                    CustomMessageBox.Show(error, "Cached Advisory Import");
                    return;
                }
                LoadBoundaries(regenerateSoft: false);
                AutoDrawBoundariesIfEnabled();
                CustomMessageBox.Show($"Copied {points.Count} vertices into a local advisory outline.", "Cached Advisory Import");
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Cached fence import failed: {ex.Message}", "Cached Advisory Import");
            }
        }

        // The only qualification seam is a file path, never a dialog or persistence provider.
        private bool ApplyOutlinePoints(List<GpsPoint> points, DialogResult choice, out string error,
            string savePath = null, bool append = false)
        {
            error = "Import cancelled.";
            if (choice != DialogResult.Yes && choice != DialogResult.No)
            {
                return false;
            }
            if (choice == DialogResult.Yes && _missionConfig.SoftBoundaryFromHard)
            {
                error = "The inner outline is derived from the outer outline. Import outer, or disable auto derivation first.";
                return false;
            }
            if (points == null || points.Count == 0 || points.Any(point => point == null ||
                !GeoMath.IsValidCoordinate(point.Lat, point.Lon)))
            {
                error = "Coordinate points must be finite latitude/longitude pairs in range.";
                return false;
            }
            var candidate = _missionConfig.Clone();
            var target = choice == DialogResult.Yes ? candidate.SoftBoundary : candidate.HardBoundary;
            target.Vertices = append ? target.Vertices.Concat(points).ToList() : points.ToList();
            if (choice == DialogResult.No)
            {
                candidate.RegenerateSoftFromHard();
                if (candidate.SoftBoundaryFromHard && candidate.SoftBoundary.Vertices.Any(point => point == null ||
                    !GeoMath.IsValidCoordinate(point.Lat, point.Lon)))
                {
                    error = "The derived inner outline is invalid; original outlines preserved.";
                    return false;
                }
            }
            bool saved = savePath == null ? candidate.Save() : candidate.SaveToFile(savePath);
            if (!saved)
            {
                error = "Could not save the import; live outlines are unchanged. Check the primary and backup files.";
                return false;
            }
            if (choice == DialogResult.Yes)
            {
                _missionConfig.SoftBoundary.Vertices = candidate.SoftBoundary.Vertices;
            }
            else
            {
                _missionConfig.HardBoundary.Vertices = candidate.HardBoundary.Vertices;
                if (candidate.SoftBoundaryFromHard)
                {
                    _missionConfig.SoftBoundary.Vertices = candidate.SoftBoundary.Vertices;
                }
            }
            error = null;
            return true;
        }
    }
}
