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
            if (_connectionManager != null)
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
                    /* fall through to disabled view */
                }
            }

            // Fallback: dual link not initialised
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
                Text = "MAVLink Dual Link Router",
                Font = new Font("Segoe UI", 14, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                AutoSize = true,
                Margin = new Padding(0, 0, 0, 12),
            });

            layout.Controls.Add(new Label
            {
                Text =
                    "Dual link is disabled in settings.\n\n" +
                    "When enabled, NOMAD opens both the LTE/Tailscale and RadioMaster sockets directly,\n" +
                    "merges the two streams to a single local UDP endpoint and switches the outbound\n" +
                    "side to whichever link is healthiest. Mission Planner connects to that local endpoint\n" +
                    "as a UDP client — failover happens with no reconnect and no dropped packets.\n\n" +
                    "Enable it in NOMAD → Settings → Connection.",
                Font = new Font("Segoe UI", 10),
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                AutoSize = true,
                MaximumSize = new Size(720, 0),
            });

            Controls.Add(layout);
        }

        public void UpdateData() { /* panel refreshes itself */ }
    }
}
