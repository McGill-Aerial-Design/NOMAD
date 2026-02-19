// ============================================================
// NOMAD Task 2 View - Indoor Fire Extinguishing
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
    public class NOMADTask2View : NOMADViewBase, IUpdatableView
    {
        private readonly DualLinkSender _sender;
        private readonly NOMADConfig _config;
        private readonly JetsonConnectionManager _jetsonConnectionManager;
        private Label _lblVioStatus;
        private Label _lblTargetCount;
        private Button _btnResetMap;
        private Button _btnResetVio;
        private SLAM3DView _slam3DView;
        private TabControl _tabControl;
        
        public NOMADTask2View(DualLinkSender sender, NOMADConfig config, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _sender = sender;
            _config = config;
            _jetsonConnectionManager = jetsonConnectionManager;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            // Use TabControl to switch between Status view and 3D SLAM view
            _tabControl = new TabControl
            {
                Dock = DockStyle.Fill,
            };
            
            // Tab 1: Status & Controls
            var statusTab = new TabPage("Status & Controls")
            {
                BackColor = NOMADTheme.BG_DARK,
            };
            
            var statusLayout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = true,
                Padding = new Padding(10),
            };
            
            // Description
            var descLabel = new Label
            {
                Text = "Task 2: Indoor Fire Extinguishing\n\n" +
                       "VIO-based indoor navigation. GPS is disabled.\n" +
                       "Use the exclusion map to track extinguished targets.\n" +
                       "Switch to '3D SLAM View' tab for real-time 3D mapping.",
                Font = new Font("Segoe UI", 11),
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                MaximumSize = new Size(600, 0),
                Margin = new Padding(0, 0, 0, 20),
            };
            statusLayout.Controls.Add(descLabel);
            
            // VIO Status Card
            var vioCard = CreateCard("VIO STATUS");
            vioCard.Size = new Size(600, 100);
            
            _lblVioStatus = new Label
            {
                Text = "VIO: Inactive",
                Font = new Font("Consolas", 11),
                ForeColor = WARNING_COLOR,
                Location = new Point(15, 50),
                AutoSize = true,
            };
            vioCard.Controls.Add(_lblVioStatus);
            
            _btnResetVio = CreateButton("Reset VIO Origin", SUCCESS_COLOR, 180, 35);
            _btnResetVio.Location = new Point(400, 45);
            _btnResetVio.Click += async (s, e) => await _sender.ResetVioOriginAsync();
            vioCard.Controls.Add(_btnResetVio);
            
            statusLayout.Controls.Add(vioCard);
            
            // Exclusion Map Card
            var mapCard = CreateCard("TARGET EXCLUSION MAP");
            mapCard.Size = new Size(600, 130);
            
            _lblTargetCount = new Label
            {
                Text = "Targets tracked: 0",
                Font = new Font("Segoe UI", 12),
                ForeColor = TEXT_PRIMARY,
                Location = new Point(15, 50),
                AutoSize = true,
            };
            mapCard.Controls.Add(_lblTargetCount);
            
            _btnResetMap = CreateButton("RESET EXCLUSION MAP", ERROR_COLOR, 250, 45);
            _btnResetMap.Location = new Point(15, 80);
            _btnResetMap.Click += async (s, e) =>
            {
                var confirm = MessageBox.Show(
                    "Reset the exclusion map? All tracked targets will be cleared.",
                    "Confirm Reset",
                    MessageBoxButtons.YesNo,
                    MessageBoxIcon.Warning
                );
                if (confirm == DialogResult.Yes)
                {
                    await _sender.SendTask2ResetMap();
                    _lblTargetCount.Text = "Targets tracked: 0";
                }
            };
            mapCard.Controls.Add(_btnResetMap);
            
            statusLayout.Controls.Add(mapCard);
            
            // WASD Control hint
            var wasdLabel = new Label
            {
                Text = "Tip: For manual indoor control, use the dedicated WASD controller in the Quick Panel.",
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                Margin = new Padding(0, 20, 0, 0),
            };
            statusLayout.Controls.Add(wasdLabel);
            
            statusTab.Controls.Add(statusLayout);
            _tabControl.TabPages.Add(statusTab);
            
            // Tab 2: 3D SLAM View
            var slam3DTab = new TabPage("3D SLAM View")
            {
                BackColor = NOMADTheme.BG_DARK,
            };
            
            try
            {
                _slam3DView = new SLAM3DView(_config);
                _slam3DView.Dock = DockStyle.Fill;
                slam3DTab.Controls.Add(_slam3DView);
            }
            catch (Exception ex)
            {
                var errorLabel = new Label
                {
                    Text = $"3D SLAM View unavailable: {ex.Message}\n\n" +
                           "This may be due to missing Helix Toolkit dependencies.\n" +
                           "Ensure HelixToolkit.Wpf NuGet package is installed.",
                    Font = new Font("Segoe UI", 11),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                    Padding = new Padding(20),
                };
                slam3DTab.Controls.Add(errorLabel);
            }
            
            _tabControl.TabPages.Add(slam3DTab);
            
            this.Controls.Add(_tabControl);
        }
        
        public void UpdateData()
        {
            // VIO status updates would come from Jetson API
        }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _slam3DView?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
