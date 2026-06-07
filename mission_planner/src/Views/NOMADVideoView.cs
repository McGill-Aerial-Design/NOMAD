// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Video View (ZED stream + payload controls)
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
        private PayloadControlPanel _payloadPanel;
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

            // Right side: payload controls (drop mechanisms, servo sliders, pump)
            var controlsSection = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
            };

            try
            {
                _payloadPanel = new PayloadControlPanel(_config) { Dock = DockStyle.Fill };
                controlsSection.Controls.Add(_payloadPanel);
            }
            catch (Exception ex)
            {
                var errorLabel = new Label
                {
                    Text = $"Payload controls unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 11),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Top,
                    Height = 60,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                controlsSection.Controls.Add(errorLabel);
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
                _payloadPanel?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
