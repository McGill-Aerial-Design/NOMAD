// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.GCSViews;

namespace NOMAD.MissionPlanner
{
    // MP 1.3.83 reloads aircraft caches on mode changes; keep unsaved editor rows separately.
    internal sealed class NativePlanDrafts : IDisposable
    {
        private readonly FlightPlanner _planner;
        private readonly Dictionary<MAVLink.MAV_MISSION_TYPE, DataGridViewRow[]> _rows =
            new Dictionary<MAVLink.MAV_MISSION_TYPE, DataGridViewRow[]>();
        private MAVLink.MAV_MISSION_TYPE _mode;
        private MAVLinkInterface _port;
        private MAVState _mav;
        private object _stream;
        private byte _system, _component;
        private long _generation;
        private bool _changing, _disposed;
        internal string Error { get; private set; }
        internal bool IsFor(FlightPlanner planner) => !_disposed && ReferenceEquals(planner, _planner);

        internal NativePlanDrafts(FlightPlanner planner)
        {
            _planner = planner ?? throw new ArgumentNullException(nameof(planner));
            _mode = (MAVLink.MAV_MISSION_TYPE)planner.cmb_missiontype.SelectedItem;
            CaptureContext();
            planner.cmb_missiontype.SelectedIndexChanged -= planner.Cmb_missiontype_SelectedIndexChanged;
            planner.cmb_missiontype.SelectedIndexChanged += Switch;
            planner.Disposed += OnDisposed;
        }

        internal bool SelectFence()
        {
            Error = null;
            if (_disposed || _planner.IsDisposed) { Error = "Native Plan is unavailable."; return false; }
            if (MainV2.comPort?.giveComport == true)
            { Error = "Wait for Mission Planner's native request before changing mission type."; return false; }
            _planner.cmb_missiontype.SelectedItem = MAVLink.MAV_MISSION_TYPE.FENCE;
            return Error == null && NativeFenceDraft.IsFence(_planner);
        }

        private void CaptureContext()
        {
            _port = MainV2.comPort;
            _mav = _port?.MAV;
            _stream = _port?.BaseStream;
            _system = _mav?.sysid ?? 0;
            _component = _mav?.compid ?? 0;
            _generation = OutputController.Generation;
        }

        private bool SameContext => ReferenceEquals(MainV2.comPort, _port) && ReferenceEquals(_port?.MAV, _mav) &&
            ReferenceEquals(_port?.BaseStream, _stream) && (_mav?.sysid ?? 0) == _system &&
            (_mav?.compid ?? 0) == _component && OutputController.Generation == _generation;

        private void Switch(object sender, EventArgs args)
        {
            if (_changing || _disposed) { return; }
            var requested = (MAVLink.MAV_MISSION_TYPE)_planner.cmb_missiontype.SelectedItem;
            var grid = _planner.Commands;
            _changing = true;
            try
            {
                Error = null;
                if (grid.IsCurrentCellInEditMode || _planner.quickadd || MainV2.comPort?.giveComport == true)
                {
                    _planner.cmb_missiontype.SelectedItem = _mode;
                    Error = "Finish the Plan edit or wait for the native request before changing mission type.";
                    return;
                }
                if (!SameContext)
                {
                    _rows.Clear();
                    CaptureContext();
                }
                else
                {
                    _rows[_mode] = grid.Rows.Cast<DataGridViewRow>().Where(row => !row.IsNewRow)
                        .Select(NativeFenceDraft.CopyRow).ToArray();
                }
                _planner.Cmb_missiontype_SelectedIndexChanged(sender, args);
                if (!SameContext)
                {
                    _rows.Clear();
                    CaptureContext();
                    _mode = requested;
                    Error = "Aircraft context changed during the native switch. Review Plan and try again.";
                    return;
                }
                if (_rows.TryGetValue(requested, out var saved)) { Restore(saved); }
                _mode = requested;
            }
            catch (Exception error)
            {
                Error = "Native Plan switch failed: " + error.Message;
                try
                {
                    _planner.cmb_missiontype.SelectedItem = _mode;
                    _planner.Cmb_missiontype_SelectedIndexChanged(sender, args);
                    if (_rows.TryGetValue(_mode, out var saved)) { Restore(saved); }
                }
                catch { Error += " Review the native editor before any Write."; }
            }
            finally { _changing = false; }
        }

        private void Restore(DataGridViewRow[] saved)
        {
            bool quickadd = _planner.quickadd;
            try
            {
                _planner.quickadd = true;
                _planner.Commands.CurrentCell = null;
                _planner.Commands.Rows.Clear();
                foreach (var row in saved) { _planner.Commands.Rows.Add(NativeFenceDraft.CopyRow(row)); }
            }
            finally { _planner.quickadd = quickadd; }
            _planner.writeKML();
        }

        private void OnDisposed(object sender, EventArgs args) { Dispose(); }
        public void Dispose()
        {
            if (_disposed) { return; }
            _disposed = true;
            _planner.Disposed -= OnDisposed;
            if (!_planner.cmb_missiontype.IsDisposed)
            {
                _planner.cmb_missiontype.SelectedIndexChanged -= Switch;
                _planner.cmb_missiontype.SelectedIndexChanged += _planner.Cmb_missiontype_SelectedIndexChanged;
            }
            _rows.Clear();
        }
    }
}
