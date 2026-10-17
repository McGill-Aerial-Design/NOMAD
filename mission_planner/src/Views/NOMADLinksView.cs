// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Links View
// ============================================================

using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public class NOMADLinksView : NOMADViewBase, IUpdatableView
    {
        private readonly MAVLinkConnectionManager _connectionManager;
        private readonly NOMADConfig _config;
        private LinkHealthPanel _linkPanel;

        public NOMADLinksView(MAVLinkConnectionManager connectionManager, NOMADConfig config)
        {
            _connectionManager = connectionManager;
            _config = config;
            InitializeUI();
        }

        private void InitializeUI()
        {
            bool disabled = _config != null && !_config.RouterClientEnabled;
            if (!disabled && _connectionManager != null)
            {
                try
                {
                    _linkPanel = new LinkHealthPanel(_connectionManager, _config);
                    _linkPanel.Dock = DockStyle.Fill;
                    Controls.Add(_linkPanel);
                    return;
                }
                catch (System.Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"NOMAD: LinkHealthPanel failed to initialize - {ex}");
                    /* fall through to unavailable guidance */
                }
            }

            // Show guidance when the router status panel is unavailable.
            var layout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                BackColor = NOMADTheme.BG_DARK,
                Padding = new Padding(24),
                AutoScroll = true,
                WrapContents = false,
            };

            layout.Controls.Add(new Label
            {
                Text = "MAVLink Multi-Link Router",
                Font = new Font("Segoe UI", 14, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                AutoSize = true,
                Margin = new Padding(0, 0, 0, 12),
            });

            layout.Controls.Add(new Label
            {
                Text = (disabled ? "Router status client is disabled in settings.\n\n"
                                 : "Router status is unavailable.\n\n") +
                    "When enabled, Mission Planner connects to the separately supervised ground router.\n" +
                    "The router selects one physical link, supplies telemetry, and reports link health.\n" +
                    "Mission Planner does not start, stop, or configure the router host.\n\n" +
                    (disabled ? "Enable the router status client in NOMAD → Settings → Multi-Link."
                              : "Check the router process and management endpoint in NOMAD → Settings → Multi-Link."),
                Font = new Font("Segoe UI", 10),
                ForeColor = disabled ? NOMADTheme.TEXT_SECONDARY : NOMADTheme.WARNING,
                AutoSize = true,
                MaximumSize = new Size(720, 0),
            });

            Controls.Add(layout);
        }

        public void UpdateData() { /* panel refreshes itself */ }
    }
}
