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
                Text = "Links",
                Font = new Font("Segoe UI", 14, FontStyle.Bold),
                ForeColor = NOMADTheme.ACCENT,
                AutoSize = true,
                Margin = new Padding(0, 0, 0, 12),
            });

            layout.Controls.Add(new Label
            {
                Text = disabled ? "Router disabled. Enable it in Settings → Links." :
                    "Router unavailable. Check Settings → Links.",
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
