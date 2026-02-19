// ============================================================
// NOMAD Links View
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;
using Newtonsoft.Json;

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
                    this.Controls.Add(_linkPanel);
                    return;
                }
                catch { }
            }
            
            // Fallback if no connection manager
            var layout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
            };
            
            var descLabel = new Label
            {
                Text = "Dual Link Failover Status\n\n" +
                       "Monitor the health of both communication links:\n" +
                       "- LTE/Tailscale: Primary long-range link via 4G\n" +
                       "- RadioMaster: Backup RC link via ELRS",
                Font = new Font("Segoe UI", 11),
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                MaximumSize = new Size(600, 0),
                Margin = new Padding(0, 0, 0, 20),
            };
            layout.Controls.Add(descLabel);
            
            var statusLabel = new Label
            {
                Text = _config.DualLinkEnabled 
                    ? "[OK] Dual link monitoring is enabled" 
                    : "[!] Dual link is disabled in settings",
                Font = new Font("Segoe UI", 12, FontStyle.Bold),
                ForeColor = _config.DualLinkEnabled ? SUCCESS_COLOR : WARNING_COLOR,
                AutoSize = true,
            };
            layout.Controls.Add(statusLabel);
            
            this.Controls.Add(layout);
        }
        
        public void UpdateData()
        {
            // Link panel updates itself
        }
    }
}
