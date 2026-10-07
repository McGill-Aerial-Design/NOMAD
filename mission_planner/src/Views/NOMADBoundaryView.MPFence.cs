// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Boundary View - Mission Planner fence exchange
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Reflection;
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
                var mav = MainV2.comPort?.MAV;
                if (mav == null)
                {
                    CustomMessageBox.Show("Not connected to vehicle.", "Warning");
                    return;
                }

                var points = new List<GpsPoint>();

                // Try to access fencepoints via reflection (type varies by MP version)
                var fencepointsField = mav.GetType().GetProperty("fencepoints");
                if (fencepointsField != null)
                {
                    var fenceData = fencepointsField.GetValue(mav);
                    if (fenceData != null)
                    {
                        var valuesProperty = fenceData.GetType().GetProperty("Values");
                        if (valuesProperty != null)
                        {
                            var values = valuesProperty.GetValue(fenceData) as System.Collections.IEnumerable;
                            if (values != null)
                            {
                                foreach (var item in values)
                                {
                                    var latProp = item.GetType().GetField("lat");
                                    var lngProp = item.GetType().GetField("lng");
                                    if (latProp != null && lngProp != null)
                                    {
                                        var lat = Convert.ToDouble(latProp.GetValue(item));
                                        var lng = Convert.ToDouble(lngProp.GetValue(item));
                                        if (lat != 0 || lng != 0)
                                            points.Add(new GpsPoint(lat, lng));
                                    }
                                }
                            }
                        }
                    }
                }

                if (points.Count > 0)
                {
                    var result = CustomMessageBox.Show(
                        $"Import {points.Count} fence points as Soft (Yes) or Hard (No) boundary?",
                        "Select Boundary Type",
                        CustomMessageBox.MessageBoxButtons.YesNo);

                    if (result == CustomMessageBox.DialogResult.Yes)
                    {
                        _missionConfig.SoftBoundary.Vertices = points;
                    }
                    else
                    {
                        _missionConfig.HardBoundary.Vertices = points;
                    }
                    _missionConfig.Save();
                    LoadBoundaries();
                    AutoDrawBoundariesIfEnabled();
                    CustomMessageBox.Show($"Imported {points.Count} fence points.", "Success");
                }
                else
                {
                    CustomMessageBox.Show("No fence points found in Mission Planner.", "Warning");
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error getting fence: {ex.Message}", "Error");
            }
        }

        private List<GpsPoint> GetSelectedBoundaryVertices(out string boundaryName)
        {
            var soft = ReadGridVertices(_dgvSoftBoundary);
            var hard = ReadGridVertices(_dgvHardBoundary);
            bool hasSoft = soft != null && soft.Count > 0;
            bool hasHard = hard != null && hard.Count > 0;

            if (!hasSoft && !hasHard)
            {
                boundaryName = null;
                return null;
            }

            if (hasSoft && hasHard)
            {
                var result = CustomMessageBox.Show(
                    "Export Soft boundary (Yes) or Hard boundary (No)?",
                    "Select Boundary",
                    CustomMessageBox.MessageBoxButtons.YesNo);
                if (result == CustomMessageBox.DialogResult.Yes)
                {
                    boundaryName = "Soft";
                    return soft;
                }
                boundaryName = "Hard";
                return hard;
            }

            if (hasSoft) { boundaryName = "Soft"; return soft; }
            boundaryName = "Hard";
            return hard;
        }

        private void BtnExportToMPPlan_Click(object sender, EventArgs e)
        {
            try
            {
                var hardVerts = _missionConfig.HardBoundary?.Vertices;
                if (hardVerts == null || hardVerts.Count < 3)
                {
                    CustomMessageBox.Show(
                        "Hard boundary needs at least 3 points before exporting to the Plan map.",
                        "Warning");
                    return;
                }
                var vertices = hardVerts;
                var strokeColor = Color.Red;
                var fillColor = Color.Transparent;
                string polyName = "NOMAD_Hard_Fence";

                // 1) Refresh the saved-config zone masks on both maps.
                try
                {
                    MapOverlayManager.DrawBoundaries(_missionConfig);
                }
                catch (Exception ex) { Log.Error($"Boundary zone draw failed - {ex.Message}"); }

                // 2) Keep Mission Planner's native Plan fence as an outline.
                bool planInjected = false;
                try
                {
                    planInjected = MapOverlayManager.ExportToMPGeoFence(
                        vertices,
                        polyName,
                        strokeColor,
                        fillColor,
                        3);
                }
                catch (Exception ex) { Log.Error($"Plan map inject failed - {ex.Message}"); }

                CustomMessageBox.Show(
                    planInjected ? "Hard boundary exported to the Plan map. No aircraft fence was changed."
                                 : "Plan map export unavailable. No aircraft fence was changed.",
                    "Plan Map Export");
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error exporting fence outline: {ex.Message}", "Error");
            }
        }

    }
}
