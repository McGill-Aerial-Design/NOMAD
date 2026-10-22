// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.Globalization;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    internal sealed class RouterLinkEditorForm : Form
    {
        private readonly NOMADConfig _draft;
        private readonly int _index;
        private readonly LinkConfig _original;
        private readonly TextBox _name, _bind, _host, _device;
        private readonly ComboBox _transport, _baud;
        private readonly NumericUpDown _port, _remotePort, _priority;
        private readonly TextBox _reconnect;
        private readonly CheckBox _enabled;
        private readonly Control _bindRow, _hostRow, _portRow, _remotePortRow, _deviceRow, _baudRow;
        private readonly Label _portLabel, _hint, _error;
        internal NOMADConfig Candidate { get; private set; }

        internal RouterLinkEditorForm(NOMADConfig draft, int index, string newId)
        {
            _draft = JObject.FromObject(draft).ToObject<NOMADConfig>(); _index = index;
            _original = index < 0 ? new LinkConfig { Id = newId, Name = "New link", Port = 14750 } : _draft.RouterLinks[index].Snapshot();
            Text = index < 0 ? "Add aircraft link" : "Edit aircraft link";
            Font = NOMADTheme.Font(); BackColor = NOMADTheme.BG_DARK; ForeColor = NOMADTheme.TEXT_PRIMARY;
            ClientSize = new Size(650, 600); MinimumSize = new Size(560, 490); StartPosition = FormStartPosition.CenterParent;
            var scroll = new Panel { Dock = DockStyle.Fill, AutoScroll = true, Padding = new Padding(12) };
            var body = new TableLayoutPanel { Dock = DockStyle.Top, AutoSize = true, ColumnCount = 1 };
            body.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100)); scroll.Controls.Add(body); Controls.Add(scroll);
            Add(body, "Stable ID", new TextBox { Text = _original.Id, ReadOnly = true,
                BackColor = NOMADTheme.INPUT_BG, ForeColor = NOMADTheme.TEXT_SECONDARY });
            _name = TextInput(_original.Name); Add(body, "Name", _name);
            _enabled = ControlFactory.CheckBox("Enable this physical link", _original.Enabled); Add(body, "Availability", _enabled);
            _transport = ControlFactory.Combo(new[] { "UDP", "TCP", "COM" }); _transport.SelectedItem = _original.Transport;
            Add(body, "Transport", _transport);
            _priority = ControlFactory.Numeric(int.MinValue, int.MaxValue, _original.Priority); Add(body, "Priority (higher wins)", _priority);
            _bind = TextInput(_original.BindAddress); _bindRow = Add(body, "Local bind IPv4", _bind);
            _host = TextInput(_original.RemoteHost); _hostRow = Add(body, "Remote host", _host);
            _port = ControlFactory.Numeric(0, 65535, _original.Port); _portRow = Add(body, "Local listening port", _port);
            _portLabel = (Label)_portRow.Controls[0];
            _remotePort = ControlFactory.Numeric(0, 65535, _original.RemotePort); _remotePortRow = Add(body, "Remote UDP port (0 if learning)", _remotePort);
            _device = TextInput(_original.Device); _deviceRow = Add(body, "COM device (explicit)", _device);
            _baud = ControlFactory.Combo(new[] { "9600", "57600", "115200", "230400", "420000", "460800", "921600" });
            _baud.DropDownStyle = ComboBoxStyle.DropDown;
            _baud.Text = _original.BaudRate.ToString(CultureInfo.InvariantCulture);
            _baudRow = Add(body, "Baud (common or custom)", _baud);
            _reconnect = TextInput(_original.ReconnectSeconds.ToString("R", CultureInfo.InvariantCulture));
            Add(body, "Reconnect interval (seconds)", _reconnect);
            _hint = ControlFactory.Label("", foreColor: NOMADTheme.TEXT_SECONDARY, fontSize: NOMADTheme.SIZE_SMALL);
            _hint.MaximumSize = new Size(590, 0); body.Controls.Add(_hint);
            var footer = new Panel { Dock = DockStyle.Bottom, Height = 100, Padding = new Padding(12) };
            _error = new Label { Dock = DockStyle.Fill, ForeColor = NOMADTheme.WARNING };
            var save = ControlFactory.Button("Use link", width: 100, height: 32);
            save.Click += (sender, args) =>
            {
                try { Candidate = BuildCandidate(); DialogResult = DialogResult.OK; }
                catch (Exception error) { _error.Text = error.Message; }
            };
            var cancel = ControlFactory.Button("Cancel", width: 90, height: 32); cancel.DialogResult = DialogResult.Cancel;
            var buttons = ControlFactory.ButtonRow(save, cancel); buttons.Dock = DockStyle.Bottom;
            footer.Controls.Add(_error); footer.Controls.Add(buttons); Controls.Add(footer);
            AcceptButton = save; CancelButton = cancel;
            _transport.SelectedIndexChanged += (sender, args) => ShowTransport(); ShowTransport();
        }
        private static TableLayoutPanel Add(TableLayoutPanel body, string label, Control input)
        { var row = ControlFactory.LabeledRow(label, input); body.Controls.Add(row); return row; }
        private static TextBox TextInput(string text) => new TextBox
            { Text = text, BackColor = NOMADTheme.INPUT_BG, ForeColor = NOMADTheme.TEXT_PRIMARY };

        private void ShowTransport()
        {
            string transport = (string)_transport.SelectedItem;
            _bindRow.Visible = _remotePortRow.Visible = transport == "UDP";
            _hostRow.Visible = _portRow.Visible = transport != "COM";
            _deviceRow.Visible = _baudRow.Visible = transport == "COM";
            _portLabel.Text = transport == "TCP" ? "Remote server port" : "Local listening port";
            _hint.Text = transport == "UDP" ? "Leave remote host empty to learn the sender. A fixed peer needs host and port." :
                transport == "TCP" ? "Connects to the remote server. The saved remote UDP port is not used for TCP." :
                "Enter a device explicitly; this dialog never scans or opens serial ports. Example baud rates: 57600, 115200, 420000.";
        }

        internal NOMADConfig BuildCandidate()
        {
            var link = _original.Snapshot();
            link.Name = _name.Text.Trim(); link.Enabled = _enabled.Checked; link.Transport = (string)_transport.SelectedItem;
            link.Priority = (int)_priority.Value;
            if (!double.TryParse(_reconnect.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double reconnect))
            { throw new ArgumentException("Enter a positive reconnect interval in seconds."); }
            link.ReconnectSeconds = reconnect;
            if (link.Transport == "UDP") { link.BindAddress = _bind.Text.Trim(); link.RemotePort = (int)_remotePort.Value; }
            if (link.Transport != "COM") { link.RemoteHost = _host.Text.Trim(); link.Port = (int)_port.Value; }
            else
            {
                link.Device = _device.Text.Trim();
                if (!int.TryParse(_baud.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out int baud))
                { throw new ArgumentException("Enter a positive integer baud rate."); }
                link.BaudRate = baud;
            }
            var candidate = JObject.FromObject(_draft).ToObject<NOMADConfig>();
            if (_index < 0) { candidate.RouterLinks.Add(link); } else { candidate.RouterLinks[_index] = link; }
            candidate.Validate();
            return candidate;
        }
    }
}
