// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using GMap.NET;
using MissionPlanner;
using MissionPlanner.GCSViews;

namespace NOMAD.MissionPlanner
{
    // A pending replacement of the native Plan grid, never a fence transfer.
    internal sealed class NativeFenceDraft
    {
        private readonly FlightPlanner _planner;
        private readonly DataGridView _grid;
        private readonly DataGridViewRow[] _originalRows;
        private readonly DataGridViewRow[] _savedRows;
        private readonly DataGridViewColumn[] _columns;
        private readonly List<PointLatLng> _points;
        private readonly MAVLinkInterface _port;
        private readonly MAVState _mav;
        private readonly object _stream;
        private readonly byte _system, _component;
        private readonly long _generation;

        internal NativeFenceDraft(FlightPlanner planner, IReadOnlyList<PointLatLng> points)
        {
            if (!FenceCoordinates.Validate(points, out string error)) { throw new ArgumentException(error); }
            _planner = planner ?? throw new ArgumentNullException(nameof(planner));
            if (!IsFence(planner)) { throw new InvalidOperationException("Select Fence in native Plan first."); }
            _grid = planner.Commands;
            if (_grid == null || _grid.IsDisposed || planner.quickadd || _grid.IsCurrentCellInEditMode ||
                MainV2.comPort?.giveComport == true)
            {
                throw new InvalidOperationException("Finish the Plan edit or wait for the native request before staging.");
            }
            _generation = OutputController.Generation;
            _port = MainV2.comPort;
            _mav = _port?.MAV;
            _stream = _port?.BaseStream;
            _system = _mav?.sysid ?? 0;
            _component = _mav?.compid ?? 0;
            _points = points.ToList();
            _originalRows = _grid.Rows.Cast<DataGridViewRow>().ToArray();
            _savedRows = _originalRows.Select(CopyRow).ToArray();
            _columns = _grid.Columns.Cast<DataGridViewColumn>().ToArray();
        }

        internal static bool IsFence(FlightPlanner planner) =>
            planner != null && !planner.IsDisposed &&
            planner.cmb_missiontype?.SelectedItem is MAVLink.MAV_MISSION_TYPE type &&
            type == MAVLink.MAV_MISSION_TYPE.FENCE;

        internal bool TryStage(out string message)
        {
            message = "Plan or aircraft context changed; staging cancelled. Review and try again.";
            if (!IsCurrent()) { return false; }
            bool wasQuickAdd = _planner.quickadd;
            try
            {
                _planner.quickadd = true;
                _grid.CurrentCell = null;
                _grid.Rows.Clear();
                foreach (var point in _points)
                {
                    _planner.AddCommand(MAVLink.MAV_CMD.FENCE_POLYGON_VERTEX_INCLUSION,
                        _points.Count, 0, 0, 0, point.Lng, point.Lat, 0);
                }
            }
            catch (Exception)
            {
                try
                {
                    _grid.Rows.Clear();
                    foreach (var row in _savedRows) { _grid.Rows.Add(CopyRow(row)); }
                    message = "Could not stage the fence; the previous Plan rows were restored.";
                }
                catch (Exception)
                {
                    message = "Staging and restoration failed. Review the native Plan draft before any Write.";
                }
                return false;
            }
            finally
            {
                _planner.quickadd = wasQuickAdd;
            }
            message = "Draft staged in MP Plan. Review it, then use native Write and Read to verify the aircraft fence.";
            return true;
        }

        private bool IsCurrent()
        {
            if (!IsFence(_planner) || !ReferenceEquals(FlightPlanner.instance, _planner) ||
                !ReferenceEquals(_planner.Commands, _grid) || _grid.IsDisposed || _planner.quickadd ||
                MainV2.comPort?.giveComport == true ||
                !_grid.Columns.Cast<DataGridViewColumn>().SequenceEqual(_columns) ||
                _grid.IsCurrentCellInEditMode || OutputController.Generation != _generation ||
                !ReferenceEquals(MainV2.comPort, _port) || !ReferenceEquals(_port?.MAV, _mav) ||
                !ReferenceEquals(_port?.BaseStream, _stream) || (_mav?.sysid ?? 0) != _system ||
                (_mav?.compid ?? 0) != _component || _grid.Rows.Count != _originalRows.Length) { return false; }
            for (int index = 0; index < _originalRows.Length; index++)
            {
                var row = _grid.Rows[index];
                var saved = _savedRows[index];
                if (!ReferenceEquals(row, _originalRows[index]) || !Equals(row.Tag, saved.Tag)) { return false; }
                for (int cell = 0; cell < row.Cells.Count; cell++)
                {
                    if (!Equals(row.Cells[cell].Value, saved.Cells[cell].Value) ||
                        !Equals(row.Cells[cell].Tag, saved.Cells[cell].Tag)) { return false; }
                }
            }
            return true;
        }

        internal static DataGridViewRow CopyRow(DataGridViewRow row)
        {
            var copy = (DataGridViewRow)row.Clone();
            copy.Tag = row.Tag;
            for (int index = 0; index < row.Cells.Count; index++)
            {
                copy.Cells[index].Value = row.Cells[index].Value;
                copy.Cells[index].Tag = row.Cells[index].Tag;
            }
            return copy;
        }
    }
}
