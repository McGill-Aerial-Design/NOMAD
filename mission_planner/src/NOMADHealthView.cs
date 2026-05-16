// ============================================================
// NOMAD Health View
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
    public class NOMADHealthView : NOMADViewBase, IUpdatableView
    {
        private readonly NOMADConfig _config;
        private readonly DualLinkSender _sender;
        private EnhancedHealthDashboard _healthDashboard;
        private ServiceControlPanel _serviceControlPanel;
        private TabControl _tabControl;
        
        public NOMADHealthView(NOMADConfig config, DualLinkSender sender = null)
        {
            _config = config;
            _sender = sender;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            try
            {
                // Create a TabControl to hold both Health Dashboard and Service Control
                _tabControl = new TabControl
                {
                    Dock = DockStyle.Fill,
                    Font = new Font("Segoe UI", 10),
                };
                
                // Tab 1: Health Dashboard
                var healthTab = new TabPage("Jetson Health")
                {
                    BackColor = Color.FromArgb(30, 30, 33),
                };
                _healthDashboard = new EnhancedHealthDashboard(_config);
                _healthDashboard.Dock = DockStyle.Fill;
                healthTab.Controls.Add(_healthDashboard);
                _tabControl.TabPages.Add(healthTab);
                
                // Tab 2: Service Control
                var serviceTab = new TabPage("Service Control")
                {
                    BackColor = Color.FromArgb(30, 30, 33),
                    AutoScroll = true,
                };
                
                if (_sender != null)
                {
                    _serviceControlPanel = new ServiceControlPanel(_sender, _config.HealthPollInterval);
                    _serviceControlPanel.Dock = DockStyle.Fill;
                    serviceTab.Controls.Add(_serviceControlPanel);
                }
                else
                {
                    var noSenderLabel = new Label
                    {
                        Text = "Service control unavailable - no sender configured.\n\nConnect to Jetson to enable service control.",
                        Font = new Font("Segoe UI", 11),
                        ForeColor = WARNING_COLOR,
                        Dock = DockStyle.Fill,
                        TextAlign = ContentAlignment.MiddleCenter,
                    };
                    serviceTab.Controls.Add(noSenderLabel);
                }
            _tabControl.TabPages.Add(serviceTab);

            this.Controls.Add(_tabControl);
            }
            catch (Exception ex)
            {
                var errorLabel = new Label
                {
                    Text = $"Health dashboard unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 12),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                this.Controls.Add(errorLabel);
            }
        }
        
        public void UpdateData()
        {
            _healthDashboard?.RefreshHealth();
        }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _healthDashboard?.Dispose();
                _serviceControlPanel?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
