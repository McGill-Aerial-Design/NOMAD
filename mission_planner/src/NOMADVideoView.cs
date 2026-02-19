// ============================================================
// NOMAD Video View with WASD Controls
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
    public class NOMADVideoView : NOMADViewBase, IUpdatableView
    {
        private readonly DualLinkSender _sender;
        private readonly NOMADConfig _config;
        private readonly JetsonConnectionManager _jetsonConnectionManager;
        private EmbeddedVideoPlayer _videoPlayer;
        private EnhancedWASDControl _wasdControl;
        private Label _lblStatus;
        
        public NOMADVideoView(DualLinkSender sender, NOMADConfig config, JetsonConnectionManager jetsonConnectionManager = null)
        {
            _sender = sender;
            _config = config;
            _jetsonConnectionManager = jetsonConnectionManager;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            // Main horizontal split: Video (left) + Controls (right)
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
            };
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 60));  // Video
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 40));  // Controls
            
            // Left side: Video with controls
            var videoSection = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 1,
            };
            videoSection.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            
            // Video player panel - left ZED camera
            var videoPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                Margin = new Padding(5),
            };
            
            try
            {
                // RTSP URL for ZED stream - left camera will be cropped
                string rtspUrl = $"rtsp://{_config.EffectiveIP}:8554/primary";
                _videoPlayer = new EmbeddedVideoPlayer("ZED Left Camera", rtspUrl, true, _jetsonConnectionManager);
                _videoPlayer.Dock = DockStyle.Fill;
                videoPanel.Controls.Add(_videoPlayer);
            }
            catch (Exception ex)
            {
                _lblStatus = new Label
                {
                    Text = $"Video player unavailable: {ex.Message}\n\n" +
                           $"Stream URL: rtsp://{_config.EffectiveIP}:8554/primary\n\n" +
                           "Use VLC or another player to view the stream.",
                    Font = new Font("Segoe UI", 12),
                    ForeColor = TEXT_SECONDARY,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                videoPanel.Controls.Add(_lblStatus);
            }
            
            videoSection.Controls.Add(videoPanel, 0, 0);
            
            // Video player has built-in controls - no need for duplicate controls panel
            
            mainLayout.Controls.Add(videoSection, 0, 0);
            
            // Right side: WASD Controls (full height - includes payload controls)
            var controlsSection = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 1,
            };
            controlsSection.RowStyles.Add(new RowStyle(SizeType.Percent, 100));  // WASD with payload controls

            try
            {
                _wasdControl = new EnhancedWASDControl(
                    _config,
                    _config.WasdNudgeSpeed,
                    _config.WasdAltSpeed,
                    15.0f,  // Default yaw rate
                    _jetsonConnectionManager
                );
                _wasdControl.Dock = DockStyle.Fill;
                controlsSection.Controls.Add(_wasdControl, 0, 0);
            }
            catch (Exception ex)
            {
                var errorPanel = new Panel
                {
                    Dock = DockStyle.Fill,
                    BackColor = CARD_BG,
                };
                var errorLabel = new Label
                {
                    Text = $"WASD controls unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 11),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                errorPanel.Controls.Add(errorLabel);
                controlsSection.Controls.Add(errorPanel, 0, 0);
            }

            mainLayout.Controls.Add(controlsSection, 1, 0);
            
            this.Controls.Add(mainLayout);
        }
        
        public void UpdateData() { }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _videoPlayer?.Dispose();
                _wasdControl?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
