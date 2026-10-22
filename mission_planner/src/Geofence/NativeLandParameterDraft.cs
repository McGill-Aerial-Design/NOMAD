// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Globalization;
using System.Linq;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.ArduPilot;
using MissionPlanner.Controls.BackstageView;
using MissionPlanner.GCSViews;
using MissionPlanner.GCSViews.ConfigurationView;

namespace NOMAD.MissionPlanner
{
    // Edits only the already-active native draft. MP owns Write Params and Refresh Params.
    internal sealed class NativeLandParameterDraft
    {
        private readonly ConfigRawParams _page;
        private readonly DataGridView _grid;
        private readonly DataGridViewColumn _command, _value;
        private readonly DataGridViewRow[] _rows;
        private readonly string[] _names;
        private readonly string[] _otherRates;
        private readonly double[] _desired;
        private readonly object[] _expected;
        private readonly double[] _cached;
        private readonly MAVLinkInterface _port;
        private readonly MAVState _mav;
        private readonly CurrentState _state;
        private readonly MAVLink.MAVLinkParamList _parameters;
        private readonly object _stream;
        private readonly int _system, _component;
        private readonly long _generation;
        private readonly IntPtr _pageHandle, _gridHandle;
        private readonly bool _readOnly;

        internal NativeLandParameterDraft()
        {
            _page = ActivePage() ?? throw new InvalidOperationException(
                "Open native Config/Tuning → Full Parameter List first, then use NOMAD → Tools → Stage Copter LAND settings.");
            _grid = _page.Controls.Find("Params", true).OfType<DataGridView>().SingleOrDefault();
            if (_grid == null || !_page.IsHandleCreated || !_grid.IsHandleCreated || !_grid.Enabled ||
                !_page.Visible || _grid.ReadOnly || _grid.IsCurrentCellInEditMode)
            { throw new InvalidOperationException("Finish the active native parameter edit before staging."); }
            _command = _grid.Columns["Command"]; _value = _grid.Columns["Value"];
            if (_command == null || _value == null) { throw new InvalidOperationException("Native parameter columns are unavailable."); }
            _generation = OutputController.Generation;
            _port = MainV2.comPort; _mav = _port?.MAV; _state = _mav?.cs;
            _stream = _port?.BaseStream; _parameters = _mav?.param;
            _system = _port?.sysidcurrent ?? 0; _component = _port?.compidcurrent ?? 0;
            _readOnly = _port?.ReadOnly ?? false;
            if (!Eligible()) { throw new InvalidOperationException("A fresh, connected, disarmed Copter with its parameter list is required."); }
            var modern = new[] { "LAND_SPD_MS", "LAND_SPD_HIGH_MS" };
            var legacy = new[] { "LAND_SPEED", "LAND_SPEED_HIGH" };
            bool hasModern = modern.Any(_parameters.ContainsKey), hasLegacy = legacy.Any(_parameters.ContainsKey);
            if (hasModern == hasLegacy || !(hasModern ? modern : legacy).All(_parameters.ContainsKey) ||
                (hasModern ? legacy : modern).Any(name => FindRows(name).Length != 0))
            { throw new InvalidOperationException("Landing-rate parameters are missing, mixed or unsupported. Review native parameters manually."); }
            _names = new[] { "FENCE_ACTION" }.Concat(hasModern ? modern : legacy).ToArray();
            _otherRates = hasModern ? legacy : modern;
            _desired = new[] { 2d, hasModern ? 2d : 200d, hasModern ? 2d : 200d };
            _rows = _names.Select(name =>
            {
                var matches = FindRows(name);
                if (!_parameters.ContainsKey(name) || matches.Length != 1 || matches[0].Cells[_value.Index].ReadOnly)
                { throw new InvalidOperationException("Required parameters must exist once in the native grid/cache and have editable Value cells."); }
                if (!matches[0].Visible)
                { throw new InvalidOperationException("Clear native parameter filters so all three required rows are visible before staging."); }
                return matches[0];
            }).ToArray();
            _expected = _rows.Select(row => row.Cells[_value.Index].Value).ToArray();
            _cached = _names.Select(name => _parameters[name].Value).ToArray();
            if (_expected.Any(value => !Number(value, out _)) || _cached.Any(value => double.IsNaN(value) || double.IsInfinity(value)))
            { throw new InvalidOperationException("Current native parameter values are unavailable or invalid."); }
            _pageHandle = _page.Handle; _gridHandle = _grid.Handle;
        }

        internal string Preview => "Fence breach: LAND; initial/final LAND descent: 2 m/s. " +
            "These global rates also affect ordinary landings.\n\n" +
            string.Join("\n", _names.Select((name, index) => name + ": " +
                Convert.ToString(_expected[index], CultureInfo.CurrentCulture) + " → " +
                _desired[index].ToString(CultureInfo.InvariantCulture))) +
            "\n\nThese are current native GRID values, which may already be edited. " +
            "All three values will be queued again, even when the displayed values match, replacing prior drafts for them. " +
            "Other queued edits are preserved. " +
            "Native Write Params may include those other edits. No aircraft write is performed here. " +
            "This is not a separate termination mode or a compliance claim.";

        private static ConfigRawParams ActivePage()
        {
            var software = MainV2.View?.current?.Control as SoftwareConfig;
            var backstage = software?.Controls.OfType<BackstageView>().SingleOrDefault();
            var page = backstage?.SelectedPage;
            return page?.PageType == typeof(ConfigRawParams) && page.isPageCreated ? page.Page as ConfigRawParams : null;
        }

        private DataGridViewRow[] FindRows(string name) => _grid.Rows.Cast<DataGridViewRow>()
            .Where(row => !row.IsNewRow && Equals(row.Cells[_command.Index].Value, name)).ToArray();

        private bool Eligible()
        {
            if (_port?.BaseStream?.IsOpen != true || _port.logreadmode || _port.giveComport || _state == null || _state.armed ||
                _state.firmware != Firmwares.ArduCopter2 || _parameters == null || _parameters.TotalReported <= 0 ||
                _parameters.TotalReceived < _parameters.TotalReported || _system < 1 || _component < 1 ||
                _system != _mav.sysid || _component != _mav.compid) { return false; }
            var heartbeat = _mav.getPacketLast((uint)MAVLink.MAVLINK_MSG_ID.HEARTBEAT);
            if (heartbeat == null || heartbeat.sysid != _system || heartbeat.compid != _component) { return false; }
            double age = (DateTime.UtcNow - heartbeat.rxtime.ToUniversalTime()).TotalSeconds;
            if (age < 0 || age > 3) { return false; }
            var status = heartbeat.ToStructure<MAVLink.mavlink_heartbeat_t>();
            bool rotor = new[] { MAVLink.MAV_TYPE.QUADROTOR, MAVLink.MAV_TYPE.COAXIAL, MAVLink.MAV_TYPE.HELICOPTER,
                MAVLink.MAV_TYPE.HEXAROTOR, MAVLink.MAV_TYPE.OCTOROTOR, MAVLink.MAV_TYPE.TRICOPTER,
                MAVLink.MAV_TYPE.DODECAROTOR, MAVLink.MAV_TYPE.DECAROTOR }.Contains((MAVLink.MAV_TYPE)status.type);
            return rotor && status.autopilot == (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA &&
                (status.base_mode & (byte)MAVLink.MAV_MODE_FLAG.SAFETY_ARMED) == 0;
        }

        private bool IsCurrent(bool ownEdit = false)
        {
            if (!ReferenceEquals(ActivePage(), _page) || _page.IsDisposed || _grid.IsDisposed ||
                !_page.IsHandleCreated || !_grid.IsHandleCreated || _page.Handle != _pageHandle || _grid.Handle != _gridHandle ||
                !_page.Visible || !_grid.Enabled || _grid.ReadOnly || (!ownEdit && _grid.IsCurrentCellInEditMode) ||
                !ReferenceEquals(_grid.Columns["Command"], _command) || !ReferenceEquals(_grid.Columns["Value"], _value) ||
                OutputController.Generation != _generation || !ReferenceEquals(MainV2.comPort, _port) ||
                !ReferenceEquals(_port.MAV, _mav) || !ReferenceEquals(_mav.cs, _state) ||
                !ReferenceEquals(_port.BaseStream, _stream) || !ReferenceEquals(_mav.param, _parameters) ||
                _port.sysidcurrent != _system || _port.compidcurrent != _component || _port.ReadOnly != _readOnly || !Eligible() ||
                _otherRates.Any(name => _parameters.ContainsKey(name) || FindRows(name).Length != 0))
            { return false; }
            for (int index = 0; index < _rows.Length; index++)
            {
                var matches = FindRows(_names[index]);
                if (matches.Length != 1 || !ReferenceEquals(matches[0], _rows[index]) ||
                    !_rows[index].Visible || _rows[index].Cells[_value.Index].ReadOnly || !_parameters.ContainsKey(_names[index]) ||
                    !Equals(_rows[index].Cells[_value.Index].Value, _expected[index]) ||
                    _parameters[_names[index]].Value != _cached[index]) { return false; }
            }
            return true;
        }

        internal bool TryStage(bool confirmed, out string message)
        {
            message = "Staging cancelled; the native parameter draft is unchanged.";
            if (!confirmed) { return false; }
            int staged = 0;
            bool attempted = false;
            try
            {
                for (int index = 0; index < _rows.Length; index++)
                {
                    if (!IsCurrent()) { break; }
                    var cell = _rows[index].Cells[_value.Index];
                    _grid.CurrentCell = cell;
                    if (!IsCurrent()) { break; }
                    if (!_grid.BeginEdit(false)) { break; }
                    if (!IsCurrent(ownEdit: true)) { _grid.CancelEdit(); break; }
                    if (!(_grid.EditingControl is DataGridViewTextBoxEditingControl editor))
                    { _grid.CancelEdit(); break; }
                    attempted = true;
                    // Native Refresh can hide a different queued value. An equal numeric expression forces a fresh callback.
                    editor.Text = _desired[index].ToString(CultureInfo.InvariantCulture) + "+0";
                    // Require this edit's native callback to provide the accepted-draft cue.
                    cell.Style.BackColor = Color.Empty;
                    if (!_grid.EndEdit() || cell.Style.BackColor != Color.Green ||
                        !Number(cell.Value, out double after) || after != _desired[index]) { break; }
                    _expected[index] = cell.Value;
                    staged++;
                }
                if (staged == 3 && IsCurrent())
                {
                    message = "Three native parameter values queued for review. " +
                        "Review native Write Params (including other queued changes), then Refresh Params to verify. " +
                        "No aircraft write was made here.";
                    return true;
                }
            }
            catch (Exception) { /* Native edit failures can leave a partial draft; never claim rollback. */ }
            if (!attempted)
            {
                message = "Native staging is unavailable or the context/draft changed. No new edits or aircraft writes were made.";
                return false;
            }
            message = $"Staging stopped. Native edits confirmed: {staged}. " +
                "Some target cells may have changed. Review the native draft before Write Params; no aircraft write was made here.";
            return false;
        }

        private static bool Number(object value, out double number)
        {
            string text = Convert.ToString(value, CultureInfo.InvariantCulture);
            return (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out number) ||
                double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out number)) &&
                !double.IsNaN(number) && !double.IsInfinity(number);
        }
    }
}
