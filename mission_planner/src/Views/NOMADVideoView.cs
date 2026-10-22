// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Video View (direct video stream + payload controls)
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public class NOMADVideoView : NOMADViewBase, IUpdatableView
    {
        private readonly System.Threading.CancellationToken _shutdown;
        private readonly Func<IVideoPipeline> _createPipeline;
        private readonly NOMADConfig _config;
        private EmbeddedVideoPlayer _videoPlayer;
        private ActuatorControlPanel _actuatorPanel;

        public NOMADVideoView(NOMADConfig config)
            : this(config, System.Threading.CancellationToken.None) { }

        internal NOMADVideoView(NOMADConfig config, System.Threading.CancellationToken shutdown)
            : this(config, shutdown, () => new GStreamerVideoPipeline()) { }

        internal NOMADVideoView(NOMADConfig config, System.Threading.CancellationToken shutdown,
            Func<IVideoPipeline> createPipeline)
        {
            _createPipeline = createPipeline;
            _shutdown = shutdown;
            _config = config ?? new NOMADConfig();
            Font = NOMADTheme.Font();
            InitializeUI();
        }

        private void InitializeUI()
        {
            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
            };
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 60));
            mainLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 40));

            var videoPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                Margin = new Padding(5),
            };

            var input = _config.SelectedVideoInput;
            try
            {
                if (input == null)
                {
                    videoPanel.Controls.Add(new Label
                    {
                        Text = "No video input configured. Add an input in Settings → Video.",
                        ForeColor = TEXT_SECONDARY, Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleCenter,
                    });
                }
                else
                {
                    _videoPlayer = new EmbeddedVideoPlayer(input.Name, input, true, _shutdown, _createPipeline)
                    { Dock = DockStyle.Fill };
                    videoPanel.Controls.Add(_videoPlayer);
                }
            }
            catch (Exception ex)
            {
                videoPanel.Controls.Add(new Label
                {
                    Text = $"Video player unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 12),
                    ForeColor = TEXT_SECONDARY,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                });
            }

            mainLayout.Controls.Add(videoPanel, 0, 0);

            var controlsSection = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                Padding = new Padding(NOMADTheme.PAD),
            };
            try
            {
                _actuatorPanel = new ActuatorControlPanel(_config) { Dock = DockStyle.Fill };
                _actuatorPanel.BackColor = CARD_BG;
                controlsSection.Controls.Add(_actuatorPanel);
            }
            catch (Exception ex)
            {
                controlsSection.Controls.Add(new Label
                {
                    Text = $"Actuator controls unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 11),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Top,
                    Height = 60,
                    TextAlign = ContentAlignment.MiddleCenter,
                });
            }

            if (_config.ActuatorDefinitions == null || _config.ActuatorDefinitions.Count == 0)
            {
                controlsSection.Controls.Add(new Label
                {
                    Text = "No payloads configured. Add mappings in Settings → Payloads.",
                    ForeColor = TEXT_SECONDARY, Dock = DockStyle.Top, Height = 60,
                });
            }
            var payloadsTitle = ControlFactory.SectionTitle("Payloads");
            payloadsTitle.Dock = DockStyle.Top;
            controlsSection.Controls.Add(payloadsTitle);

            mainLayout.Controls.Add(controlsSection, 1, 0);
            Controls.Add(mainLayout);
        }

        public void UpdateData()
        {
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _videoPlayer?.Dispose();
                _actuatorPanel?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
