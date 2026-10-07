// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Boundary View - Boundary grid state and editing
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
        private void LoadBoundaries()
        {
            // Load soft boundary
            _dgvSoftBoundary.Rows.Clear();
            foreach (var point in _missionConfig.SoftBoundary.Vertices)
            {
                _dgvSoftBoundary.Rows.Add(point.Lat.ToString("F8"), point.Lon.ToString("F8"));
            }

            // Load hard boundary
            _dgvHardBoundary.Rows.Clear();
            foreach (var point in _missionConfig.HardBoundary.Vertices)
            {
                _dgvHardBoundary.Rows.Add(point.Lat.ToString("F8"), point.Lon.ToString("F8"));
            }

            UpdatePointCounts();
        }

        private bool _syncingSoftFromHard;

        /// <summary>
        /// Recompute the derived soft boundary when "auto from hard" is on and
        /// refresh its grid. Hooked into UpdatePointCounts so every hard-boundary
        /// mutation path (grid edit, paste, import, clear, add) picks it up.
        /// </summary>
        private void SyncSoftFromHard()
        {
            if (!_missionConfig.SoftBoundaryFromHard || _syncingSoftFromHard) return;
            try
            {
                _syncingSoftFromHard = true;
                _missionConfig.RegenerateSoftFromHard();
                _missionConfig.Save();
                _dgvSoftBoundary.Rows.Clear();
                foreach (var p in _missionConfig.SoftBoundary.Vertices)
                {
                    _dgvSoftBoundary.Rows.Add(p.Lat.ToString("F8"), p.Lon.ToString("F8"));
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Soft-from-hard sync failed - {ex.Message}");
            }
            finally
            {
                _syncingSoftFromHard = false;
            }
        }

        private void UpdatePointCounts()
        {
            SyncSoftFromHard();

            var softLabel = this.Controls.Find("lblSoftCount", true).FirstOrDefault() as Label;
            var hardLabel = this.Controls.Find("lblHardCount", true).FirstOrDefault() as Label;
            var softSaved = this.Controls.Find("lblSoftSaved", true).FirstOrDefault() as Label;
            var hardSaved = this.Controls.Find("lblHardSaved", true).FirstOrDefault() as Label;
            string stamp = $"Saved {DateTime.Now:HH:mm:ss} to plugin config";

            if (softLabel != null)
                softLabel.Text = $"{_missionConfig.SoftBoundary.Vertices.Count} pts";
            if (hardLabel != null)
                hardLabel.Text = $"{_missionConfig.HardBoundary.Vertices.Count} pts";
            if (softSaved != null)
                softSaved.Text = _missionConfig.SoftBoundary.Vertices.Count > 0 ? stamp : "No points";
            if (hardSaved != null)
                hardSaved.Text = _missionConfig.HardBoundary.Vertices.Count > 0 ? stamp : "No points";
        }

        private void DeleteSelectedPoint(DataGridView dgv, FlightBoundary boundary)
        {
            try
            {
                var rows = dgv.SelectedRows.Cast<DataGridViewRow>().OrderByDescending(r => r.Index).ToList();
                if (rows.Count == 0 && dgv.CurrentCell != null)
                {
                    var r = dgv.Rows[dgv.CurrentCell.RowIndex];
                    if (r != null) rows.Add(r);
                }
                if (rows.Count == 0)
                {
                    CustomMessageBox.Show("Select a row in the grid first.", "Delete Point");
                    return;
                }
                foreach (var r in rows)
                {
                    int idx = r.Index;
                    if (idx >= 0 && idx < boundary.Vertices.Count)
                        boundary.Vertices.RemoveAt(idx);
                    dgv.Rows.RemoveAt(idx);
                }
                _missionConfig.Save();
                UpdatePointCounts();
                AutoDrawBoundariesIfEnabled();
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Delete failed: {ex.Message}", "Error");
            }
        }

        private void ClearBoundary(DataGridView dgv, FlightBoundary boundary)
        {
            if (CustomMessageBox.Show("Clear all boundary points?", "Confirm",
                CustomMessageBox.MessageBoxButtons.YesNo) == CustomMessageBox.DialogResult.Yes)
            {
                boundary.Vertices.Clear();
                dgv.Rows.Clear();
                _missionConfig.Save();
                UpdatePointCounts();
            }
        }

        private void AddManualPoint(DataGridView dgv, FlightBoundary boundary)
        {
            // Use current position or last point
            double lat = MainV2.comPort?.MAV?.cs?.lat ?? 45.0;
            double lon = MainV2.comPort?.MAV?.cs?.lng ?? -75.0;

            var point = new GpsPoint(lat, lon);
            boundary.Vertices.Add(point);
            dgv.Rows.Add(lat.ToString("F8"), lon.ToString("F8"));
            _missionConfig.Save();
            UpdatePointCounts();
            AutoDrawBoundariesIfEnabled();
        }

        private void AutoDrawBoundariesIfEnabled()
        {
            try
            {
                var chkAutoDraw = this.Controls.Find("chkAutoDraw", true);
                if (chkAutoDraw.Length > 0 && chkAutoDraw[0] is CheckBox chk && chk.Checked)
                {
                    MapOverlayManager.DrawBoundaries(_missionConfig);
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Auto-draw error - {ex.Message}");
            }
        }

        /// <summary>Read an editable boundary grid into a vertex list (skips blank/invalid rows).</summary>
        private static List<GpsPoint> ReadGridVertices(DataGridView dgv)
        {
            var points = new List<GpsPoint>();
            if (dgv == null) return points;
            foreach (DataGridViewRow row in dgv.Rows)
            {
                if (row.IsNewRow) continue;
                if (double.TryParse(Convert.ToString(row.Cells["Lat"].Value), out var lat) &&
                    double.TryParse(Convert.ToString(row.Cells["Lon"].Value), out var lon))
                {
                    points.Add(new GpsPoint(lat, lon));
                }
            }
            return points;
        }

        private void SaveBoundaryFromGrid(DataGridView dgv, FlightBoundary boundary)
        {
            // The derived-soft sync repopulates the soft grid itself; its
            // CellValueChanged storm must not write partial rows back.
            if (_syncingSoftFromHard && dgv == _dgvSoftBoundary) return;
            try
            {
                boundary.Vertices.Clear();
                foreach (DataGridViewRow row in dgv.Rows)
                {
                    if (row.Cells["Lat"].Value != null && row.Cells["Lon"].Value != null)
                    {
                        if (double.TryParse(row.Cells["Lat"].Value.ToString(), out double lat) &&
                            double.TryParse(row.Cells["Lon"].Value.ToString(), out double lon))
                        {
                            boundary.Vertices.Add(new GpsPoint(lat, lon));
                        }
                    }
                }
                _missionConfig.Save();
                UpdatePointCounts();
            }
            catch (Exception ex)
            {
                Log.Error($"Save boundary from grid failed - {ex.Message}");
            }
        }
    }
}
