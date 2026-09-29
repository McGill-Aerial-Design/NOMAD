// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private TabPage CreateConnectionTab()
        {
            var tab = CreateTabPage("Core");
            int y = 15;

            AddSectionLabel(tab, "C++ Core Command Boundary", ref y);

            AddLabel(tab, "Runtime IPC port:", 20, y);
            _numCoreRuntimePort = AddNumericUpDown(tab, 170, y, 90, 1, 65535, 14611);
            y += 30;

            AddLabel(tab, "Local actuation gate:", 20, y);
            _txtCoreApiKey = AddTextBox(tab, 170, y, 360);
            _txtCoreApiKey.UseSystemPasswordChar = true;
            y += 35;

            var hint = new Label
            {
                Text = "Core command actions use versioned loopback IPC only. The configured key is a " +
                       "local nonempty gate, not IPC authentication. A separately supervised " +
                       "nomad-runtime process must be running; these commands fail closed when it is " +
                       "unavailable. GuidedGoto is unavailable until the runtime adds a typed request.",
                Font = new Font("Segoe UI", 8, FontStyle.Italic),
                ForeColor = Color.FromArgb(170, 170, 170),
                Location = new Point(20, y),
                AutoSize = true,
                MaximumSize = new Size(560, 0),
            };
            tab.Controls.Add(hint);
            y += 60;

            AddSectionLabel(tab, "Runtime software authority", ref y);
            AddAuthorityButton(tab, "Admit", 20, y, "admit");
            AddAuthorityButton(tab, "Revoke", 145, y, "revoke");
            AddAuthorityButton(tab, "Handback", 270, y, "handback");
            y += 40;
            AddLabel(tab, "Save core settings first. Reconnect never admits authority automatically.", 20, y);

            return tab;
        }

        private void AddAuthorityButton(TabPage tab, string label, int x, int y, string action)
        {
            var button = new Button { Text = label, Location = new Point(x, y), Size = new Size(110, 30) };
            button.Click += async (sender, args) => await RunAuthorityControlAsync(tab, action);
            tab.Controls.Add(button);
        }

        private async Task RunAuthorityControlAsync(TabPage tab, string action)
        {
            var client = OutputController.CreateCoreClient();
            if (client == null)
            {
                MessageBox.Show("Save core settings before changing runtime authority.", "NOMAD Runtime");
                return;
            }
            tab.Enabled = false;
            bool accepted;
            try
            {
                accepted = await Task.Run(() => action switch
                {
                    "admit" => client.AdmitAuthority(),
                    "revoke" => client.RevokeAuthority(),
                    _ => client.HandbackAuthority()
                });
            }
            finally
            {
                if (!IsDisposed) tab.Enabled = true;
            }
            if (IsDisposed) return;
            var message = accepted ? "Runtime authority changed." : client.LastMessage;
            MessageBox.Show(message, "NOMAD Runtime", MessageBoxButtons.OK,
                            accepted ? MessageBoxIcon.Information : MessageBoxIcon.Warning);
        }

        private TabPage CreateDualLinkTab()
        {
            var tab = CreateTabPage("Multi-Link");
            int y = 15;

            AddSectionLabel(tab, "Standalone ground router", ref y);
            _chkDualLinkEnabled = AddCheckBox(tab, "Connect to router status and controls", 20, y, Color.LimeGreen);
            y += 40;

            AddLabel(tab, "Mission Planner UDP port:", 20, y);
            _numRouterLocalPort = AddNumericUpDown(tab, 190, y, 80, 1024, 65535, 14600);
            y += 35;

            AddLabel(tab, "Management TCP port:", 20, y);
            _numManagementPort = AddNumericUpDown(tab, 190, y, 80, 1, 65535, 14610);
            y += 40;

            var routerHint = new Label
            {
                Text = "Mission Planner never starts or stops the router. Configure physical links, " +
                       "failover and duplicate suppression in the standalone router JSON, then " +
                       "supervise that host separately. The host rejects mission_planner " +
                       "AllowOutbound=true; nomad_core remains command-capable.",
                Font = new Font("Segoe UI", 8, FontStyle.Italic),
                ForeColor = Color.FromArgb(150, 150, 150),
                Location = new Point(40, y),
                AutoSize = true,
                MaximumSize = new Size(500, 0),
            };
            tab.Controls.Add(routerHint);

            return tab;
        }
    }
}
