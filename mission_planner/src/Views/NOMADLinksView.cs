// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Links View
// ============================================================

using System.Windows.Forms;
using System.Windows;
using System.Windows.Controls;

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
            var layout = new StackPanel { Margin = new Thickness(24) };
            layout.Children.Add(WpfUi.Section("Links"));
            layout.Children.Add(WpfUi.Text(disabled ? "Router disabled. Enable it in Settings → Links." :
                "Router unavailable. Check Settings → Links."));
            Controls.Add(WpfUi.Host(new ScrollViewer { Content = layout }));
        }

        public void UpdateData() { /* panel refreshes itself */ }
    }
}
