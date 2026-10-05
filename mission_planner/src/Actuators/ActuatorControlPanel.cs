// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;
using NOMAD.MissionPlanner.Connectivity;

namespace NOMAD.MissionPlanner
{
    // Suppress OS key auto-repeat; each physical press/release remains one input intent.
    internal sealed class ActuatorButton : Button
    {
        private static bool IsRepeatedKey(Message message) =>
            (message.Msg == 0x100 || message.Msg == 0x104) &&
            (message.WParam.ToInt64() == 13 || message.WParam.ToInt64() == 32) &&
            (message.LParam.ToInt64() & (1L << 30)) != 0;
        public override bool PreProcessMessage(ref Message message)
        {
            return IsRepeatedKey(message) || base.PreProcessMessage(ref message);
        }
        protected override void WndProc(ref Message message)
        {
            if (IsRepeatedKey(message))
            {
                return;
            }
            base.WndProc(ref message);
        }

    }

    /// <summary>Displays runtime-provided actions and software evidence without owning actuator policy.</summary>
    public sealed class ActuatorControlPanel : UserControl
    {
        private readonly FlowLayoutPanel _rows = new FlowLayoutPanel { Dock = DockStyle.Fill,
            FlowDirection = FlowDirection.TopDown, WrapContents = false, AutoScroll = true, Padding = new Padding(8) };
        private readonly Label _status = new Label { AutoSize = true, ForeColor = Color.Silver };
        private readonly Dictionary<string, Label> _states = new Dictionary<string, Label>();
        private readonly Dictionary<string, TrackBar> _positions = new Dictionary<string, TrackBar>();
        private readonly Dictionary<string, double?> _shownPositions = new Dictionary<string, double?>();
        private bool _disposed;
        internal string OperatorStatus => _status.Text;

        public ActuatorControlPanel(NOMADConfig config)
        {
            BackColor = Color.FromArgb(35, 35, 38);
            Dock = DockStyle.Fill;
            Controls.Add(_rows);
            OutputController.ActuatorStateChanged += OnStateChanged;
            Load += async (s, e) => await RefreshActuatorsAsync();
            Render(new List<NomadActuator>());
        }

        internal async Task RefreshActuatorsAsync()
        {
            var result = await OutputController.GetActuatorsAsync();
            if (_disposed || !result.PresentationCurrent)
            {
                return;
            }
            if (!result.Succeeded)
            {
                _status.Text = OutputController.DescribeFailure("Discover actuators", result); return;
            }
            Render(result.Actuators);
            _status.Text = result.ConfigurationRecoveryRequired ?
                "Runtime configuration requires operator review; see runtime diagnostics." : result.Message;
        }

        internal void Render(IReadOnlyList<NomadActuator> actuators)
        {
            while (_rows.Controls.Count > 0)
            {
                var control = _rows.Controls[0];
                _rows.Controls.RemoveAt(0);
                if (control != _status)
                {
                    control.Dispose();
                }
            }
            _states.Clear();
            _positions.Clear();
            _shownPositions.Clear();
            var refresh = MakeButton("Refresh actuator configuration", "Refresh");
            refresh.Click += async (s, e) => await RefreshActuatorsAsync();
            _rows.Controls.Add(refresh);
            foreach (var actuator in actuators)
            {
                BuildRow(actuator);
            }
            if (actuators.Count == 0)
            {
                _rows.Controls.Add(new Label { Text = "No runtime actuators discovered. Use Settings > Actuators.",
                    AutoSize = true, ForeColor = Color.Silver });
            }
            _rows.Controls.Add(_status);
        }

        private void BuildRow(NomadActuator actuator)
        {
            var currentState = OutputController.GetDisplayState(actuator.Id) ?? actuator.State;
            var row = new FlowLayoutPanel { AutoSize = true, WrapContents = true, Name = actuator.Id };
            row.Controls.Add(new Label { Text = actuator.Name, Width = 130, ForeColor = Color.White });
            foreach (var action in actuator.Actions)
            {
                TrackBar position = null;
                if (action.Control == "position")
                {
                    position = new TrackBar { Name = actuator.Id + ":Position", Minimum = 0, Maximum = 1000,
                        Value = (int)Math.Round((currentState.CommandedPosition ?? 0.5) * 1000),
                        TickStyle = TickStyle.None, Size = new Size(150, 28), AutoSize = false };
                    _positions[actuator.Id] = position;
                    _shownPositions[actuator.Id] = currentState.CommandedPosition;
                    row.Controls.Add(position);
                }
                var slider = position;
                var button = MakeButton(action.Label, actuator.Id + ":" + action.Operation);
                button.Click += async (s, e) => await SendActionAsync(actuator.Id, action.Operation,
                    slider == null ? (double?)null : slider.Value / 1000.0);
                row.Controls.Add(button);
            }
            _rows.Controls.Add(row);
            var state = new Label { AutoSize = true, ForeColor = Color.Silver, Name = actuator.Id + ":State" };
            _states[actuator.Id] = state;
            _rows.Controls.Add(state);
            OnStateChanged(currentState);
        }

        internal async Task SendActionAsync(string id, string operation, double? value = null)
        {
            var result = await OutputController.ActuatorActionAsync(id, operation, value: value);
            if (_disposed)
            {
                return;
            }
            _status.Text = result.Succeeded ? result.Message + " Physical actuator state is unverified." :
                OutputController.DescribeFailure("Actuator request", result);
        }

        private static Button MakeButton(string text, string name) => new ActuatorButton
        {
            Text = text, Name = name, AutoSize = true, MinimumSize = new Size(75, 28),
            FlatStyle = FlatStyle.Flat, BackColor = Color.FromArgb(55, 65, 75), ForeColor = Color.White
        };

        private void OnStateChanged(NomadActuatorState state)
        {
            if (_disposed)
            {
                return;
            }
            UiAsync.RunSync(this, () =>
            {
                state = OutputController.GetDisplayState(state.Id) ?? state;
                if (!_states.TryGetValue(state.Id, out var label))
                {
                    return;
                }
                label.Text = "Runtime: confirmations remaining=" + state.ConfirmationRemaining +
                    "; pending=" + state.Pending + "; recovery required=" + state.RecoveryRequired +
                    "; software command success=" + state.SoftwareCommandSuccess +
                    "; activation=" + state.ActivationOutcome + "; safe=" + state.SafeOutcome +
                    ". Physical state is unverified.";
                if (state.CommandedPosition.HasValue && _positions.TryGetValue(state.Id, out var position) &&
                    (!_shownPositions.TryGetValue(state.Id, out var shown) || shown != state.CommandedPosition))
                {
                    _shownPositions[state.Id] = state.CommandedPosition;
                    position.Value = (int)Math.Round(state.CommandedPosition.Value * 1000);
                }
            }, "ActuatorDisplayState");
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _disposed = true; OutputController.ActuatorStateChanged -= OnStateChanged;
            }
            base.Dispose(disposing);
        }
    }
}
