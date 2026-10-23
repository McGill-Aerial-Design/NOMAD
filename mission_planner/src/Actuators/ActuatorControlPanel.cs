// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    // Suppress key auto-repeat: confirmations represent distinct operator presses.
    internal sealed class ActuatorButton : Button
    {
        private static bool Repeated(Message message) => (message.Msg == 0x100 || message.Msg == 0x104) &&
            (message.WParam.ToInt64() == 13 || message.WParam.ToInt64() == 32) &&
            (message.LParam.ToInt64() & (1L << 30)) != 0;
        public override bool PreProcessMessage(ref Message message) => Repeated(message) || base.PreProcessMessage(ref message);
        protected override void WndProc(ref Message message)
        { if (!Repeated(message)) { base.WndProc(ref message); } }
    }

    public sealed class ActuatorControlPanel : UserControl
    {
        private readonly NOMADConfig _config;
        private readonly PayloadController _owner;
        private readonly Label _status = new Label { Visible = false, AutoSize = true, MaximumSize = new Size(650, 0), ForeColor = Color.Gold };
        public ActuatorControlPanel(NOMADConfig config)
        {
            _config = config;
            _owner = PayloadController.Current;
            var rows = new FlowLayoutPanel { Dock = DockStyle.Fill, AutoScroll = true, WrapContents = false,
                FlowDirection = FlowDirection.TopDown };
            rows.SizeChanged += (sender, args) => _status.MaximumSize = new Size(
                Math.Max(1, rows.ClientSize.Width - _status.Margin.Horizontal - SystemInformation.VerticalScrollBarWidth), 0);
            Controls.Add(rows);
            if (_owner != null && _owner.Owns(config))
            {
                foreach (var definition in _owner.Definitions)
                {
                    var row = new FlowLayoutPanel { AutoSize = true };
                    row.Controls.Add(new Label { Text = definition.Name, ForeColor = Color.White, AutoSize = true });
                    TrackBar position = null;
                    if (definition.Behavior == "ServoPosition")
                    { position = new TrackBar { Minimum = 0, Maximum = 1000, Value = 500, TickStyle = TickStyle.None }; row.Controls.Add(position); }
                    var slider = position;
                    foreach (string operation in definition.Operations)
                    {
                        string label = operation == "negative" ? definition.NegativeLabel : operation == "positive" ? definition.PositiveLabel : definition.PrimaryLabel;
                        AddButton(row, label, definition.Id, operation, () => slider == null ? (double?)null : slider.Value / 1000.0);
                    }
                    AddButton(row, definition.Behavior == "ServoBidirectional" ? definition.NeutralLabel : definition.SafeLabel,
                        definition.Id, "safe", () => null);
                    rows.Controls.Add(row);
                }
                _owner.Changed += OnChanged;
            }
            rows.Controls.Add(_status);
        }
        private void AddButton(FlowLayoutPanel row, string label, string id, string operation, Func<double?> value)
        {
            var button = new ActuatorButton { Text = label, UseMnemonic = false, AutoSize = true, Name = id + ":" + operation };
            button.Click += async (sender, args) => await SendAsync(id, operation, value());
            row.Controls.Add(button);
        }
        private async Task SendAsync(string id, string operation, double? value)
        {
            if (IsDisposed || _owner?.Owns(_config) != true) { return; }
            string status = await _owner.ActAsync(id, operation, value,
                inputStillCurrent: () => !IsDisposed && _owner.Owns(_config));
            if (!IsDisposed && _owner.Owns(_config)) { _status.Text = status; _status.Visible = !string.IsNullOrWhiteSpace(status); }
        }
        private void OnChanged(string id)
        {
            if (!IsDisposed && _owner.Owns(_config))
            { UiAsync.RunSync(this, () => { if (!IsDisposed) { _status.Text = _owner.Status(id); _status.Visible = !string.IsNullOrWhiteSpace(_status.Text); } }, "Payload state"); }
        }
        protected override void Dispose(bool disposing)
        { if (disposing && _owner != null) { _owner.Changed -= OnChanged; } base.Dispose(disposing); }
    }
}
