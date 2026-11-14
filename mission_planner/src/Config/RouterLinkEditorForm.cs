// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.Globalization;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;
using Wpf = System.Windows.Controls;
using Thickness = System.Windows.Thickness;
using Visibility = System.Windows.Visibility;

namespace NOMAD.MissionPlanner
{
    [global::MissionPlanner.Utilities.PreventTheming]
    internal sealed class RouterLinkEditorForm : Form
    {
        private readonly NOMADConfig _draft;
        private readonly int _index;
        private readonly LinkConfig _original;
        private readonly Wpf.TextBox _name, _bind, _host, _device, _port, _remotePort, _priority, _reconnect;
        private readonly Wpf.ComboBox _transport, _baud;
        private readonly Wpf.CheckBox _enabled;
        private readonly Wpf.StackPanel _bindRow, _hostRow, _portRow, _remotePortRow, _deviceRow, _baudRow;
        private readonly Wpf.TextBlock _portLabel, _error;
        internal NOMADConfig Candidate { get; private set; }

        internal RouterLinkEditorForm(NOMADConfig draft, int index, string newId)
        {
            _draft = JObject.FromObject(draft).ToObject<NOMADConfig>(); _index = index;
            _original = index < 0 ? new LinkConfig { Id = newId, Name = "New link", Port = 14750 } : _draft.RouterLinks[index].Snapshot();
            Text = index < 0 ? "Add aircraft link" : "Edit aircraft link";
            Font = NOMADTheme.Font(); BackColor = NOMADTheme.BG_DARK; ForeColor = NOMADTheme.TEXT_PRIMARY;
            ClientSize = new Size(650, 600); MinimumSize = new Size(560, 490); StartPosition = FormStartPosition.CenterParent;
            var root = new Wpf.DockPanel { Margin = new Thickness(12) };
            var body = new Wpf.StackPanel();
            var scroll = new Wpf.ScrollViewer { Content = body, VerticalScrollBarVisibility = Wpf.ScrollBarVisibility.Auto };
            _name = TextInput(_original.Name); Add(body, "Name", _name);
            _enabled = new Wpf.CheckBox { Content = "Enabled", IsChecked = _original.Enabled }; Add(body, "Availability", _enabled);
            _transport = Combo(new[] { "UDP", "TCP", "COM" }); _transport.SelectedItem = _original.Transport;
            Add(body, "Transport", _transport);
            _priority = TextInput(_original.Priority.ToString(CultureInfo.InvariantCulture)); Add(body, "Priority (higher wins)", _priority);
            _bind = TextInput(_original.BindAddress); _bindRow = Add(body, "Local bind IPv4", _bind);
            _host = TextInput(_original.RemoteHost); _hostRow = Add(body, "Remote host", _host);
            _port = TextInput(_original.Port.ToString(CultureInfo.InvariantCulture)); _portRow = Add(body, "Local listening port", _port);
            _portLabel = (Wpf.TextBlock)_portRow.Children[0];
            _remotePort = TextInput(_original.RemotePort.ToString(CultureInfo.InvariantCulture)); _remotePortRow = Add(body, "Remote UDP port (0 if learning)", _remotePort);
            _device = TextInput(_original.Device); _deviceRow = Add(body, "Device", _device);
            _baud = Combo(new[] { "9600", "57600", "115200", "230400", "420000", "460800", "921600" });
            _baud.IsEditable = true; _baud.Style = null;
            _baud.Text = _original.BaudRate.ToString(CultureInfo.InvariantCulture);
            _baudRow = Add(body, "Baud rate", _baud);
            _reconnect = TextInput(_original.ReconnectSeconds.ToString("R", CultureInfo.InvariantCulture));
            Add(body, "Reconnect interval (seconds)", _reconnect);
            var footer = new Wpf.StackPanel();
            _error = WpfUi.Text("");
            var save = WpfUi.Action("Use link", () =>
            {
                try { Candidate = BuildCandidate(); DialogResult = DialogResult.OK; }
                catch (Exception error) { _error.Text = error.Message; }
            });
            var cancel = WpfUi.Action("Cancel", () => { DialogResult = DialogResult.Cancel; Close(); });
            save.IsDefault = true; cancel.IsCancel = true;
            root.PreviewKeyDown += (sender, args) =>
            {
                if (args.Key == System.Windows.Input.Key.Escape) { DialogResult = DialogResult.Cancel; Close(); args.Handled = true; }
            };
            var buttons = new Wpf.WrapPanel(); buttons.Children.Add(save); buttons.Children.Add(cancel);
            footer.Children.Add(_error); footer.Children.Add(buttons);
            Wpf.DockPanel.SetDock(footer, Wpf.Dock.Bottom); root.Children.Add(footer); root.Children.Add(scroll);
            Controls.Add(WpfUi.Host(root));
            _transport.SelectionChanged += (sender, args) => ShowTransport(); ShowTransport();
        }
        private static Wpf.StackPanel Add(Wpf.StackPanel body, string label, System.Windows.FrameworkElement input)
        {
            var row = new Wpf.StackPanel { Margin = new Thickness(0, 0, 0, 8) };
            row.Children.Add(WpfUi.Text(label)); row.Children.Add(input); body.Children.Add(row);
            System.Windows.Automation.AutomationProperties.SetName(input, label); return row;
        }
        private static Wpf.TextBox TextInput(string text) => new Wpf.TextBox { Text = text };
        private static Wpf.ComboBox Combo(string[] choices)
        { var combo = new Wpf.ComboBox(); foreach (string choice in choices) { combo.Items.Add(choice); } return combo; }

        private void ShowTransport()
        {
            string transport = (string)_transport.SelectedItem;
            _bindRow.Visibility = _remotePortRow.Visibility = transport == "UDP" ? Visibility.Visible : Visibility.Collapsed;
            _hostRow.Visibility = _portRow.Visibility = transport != "COM" ? Visibility.Visible : Visibility.Collapsed;
            _deviceRow.Visibility = _baudRow.Visibility = transport == "COM" ? Visibility.Visible : Visibility.Collapsed;
            _portLabel.Text = transport == "TCP" ? "Remote server port" : "Local listening port";
            _transport.ToolTip = transport == "UDP" ? "Leave remote host empty to learn the sender. A fixed peer needs host and port." :
                transport == "TCP" ? "Connects to the remote server. The saved remote UDP port is not used for TCP." :
                "Enter the device and baud rate.";
        }

        internal NOMADConfig BuildCandidate()
        {
            var link = _original.Snapshot();
            link.Name = _name.Text.Trim(); link.Enabled = _enabled.IsChecked == true; link.Transport = (string)_transport.SelectedItem;
            link.Priority = Integer(_priority, "priority", int.MinValue, int.MaxValue);
            if (!double.TryParse(_reconnect.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double reconnect))
            { throw new ArgumentException("Enter a positive reconnect interval in seconds."); }
            link.ReconnectSeconds = reconnect;
            if (link.Transport == "UDP") { link.BindAddress = _bind.Text.Trim(); link.RemotePort = Integer(_remotePort, "remote port", 0, 65535); }
            if (link.Transport != "COM") { link.RemoteHost = _host.Text.Trim(); link.Port = Integer(_port, "port", 0, 65535); }
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

        private static int Integer(Wpf.TextBox field, string name, int min, int max)
        {
            if (!int.TryParse(field.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out int value) ||
                value < min || value > max) { throw new ArgumentException("Enter a valid " + name + "."); }
            return value;
        }
    }
}
