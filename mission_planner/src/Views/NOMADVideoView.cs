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
        private VideoInputPlayer _videoHost;
        private ActuatorControlPanel _actuatorPanel;

        internal NOMADVideoView(NOMADConfig config, System.Threading.CancellationToken shutdown)
            : this(config, shutdown, MediaWorkerPipeline.Create) { }

        internal NOMADVideoView(NOMADConfig config, System.Threading.CancellationToken shutdown,
            Func<IVideoPipeline> createPipeline)
        {
            _createPipeline = createPipeline;
            _shutdown = shutdown;
            _config = config ?? new NOMADConfig();
            Font = NOMADTheme.Font();
            InitializeUI();
        }

        private const int SidebarWidth = 340;
        private SplitContainer _split;
        private Control _sidebarTab;

        private void InitializeUI()
        {
            // Video fills the left; payloads are a resizable right sidebar the operator can hide.
            _split = new SplitContainer
            {
                Dock = DockStyle.Fill, Orientation = Orientation.Vertical, FixedPanel = FixedPanel.Panel2,
                SplitterWidth = 6, BackColor = NOMADTheme.CARD_BORDER, Panel1MinSize = 200,
            };
            _split.Panel1.BackColor = Color.Black;
            _split.Panel2.BackColor = CARD_BG;

            _videoHost = new VideoInputPlayer(_config, true, _shutdown, _createPipeline,
                NOMADMainScreen.SelectVideoInput, NOMADMainScreen.UpdateVideoInput);
            _split.Panel1.Controls.Add(_videoHost);

            try
            {
                _actuatorPanel = new ActuatorControlPanel(_config) { Dock = DockStyle.Fill, BackColor = CARD_BG };
                _split.Panel2.Controls.Add(_actuatorPanel);
            }
            catch (Exception ex)
            {
                var error = WpfUi.Text($"Actuator controls unavailable: {ex.Message}");
                error.Foreground = System.Windows.Media.Brushes.OrangeRed;
                error.Margin = new System.Windows.Thickness(NOMADTheme.PAD);
                var errorHost = WpfUi.Host(error); errorHost.BackColor = CARD_BG;
                _split.Panel2.Controls.Add(errorHost);
            }
            _split.Panel2.Controls.Add(SidebarHeader());

            var tab = WpfUi.Action("◂ Payloads", () => ShowSidebar(true));
            tab.ToolTip = "Show payload controls.";
            tab.Margin = new System.Windows.Thickness(0, NOMADTheme.PAD, 0, 0);
            tab.VerticalAlignment = System.Windows.VerticalAlignment.Top;
            tab.LayoutTransform = new System.Windows.Media.RotateTransform(-90);
            var tabRoot = new System.Windows.Controls.Grid { Background = WpfTheme.Card };
            tabRoot.Children.Add(tab);
            var tabHost = WpfUi.Host(tabRoot);
            tabHost.Dock = DockStyle.Right; tabHost.Width = 40; tabHost.BackColor = CARD_BG; tabHost.Visible = false;
            _sidebarTab = tabHost;

            Controls.Add(_split);
            Controls.Add(tabHost);
            _split.HandleCreated += (sender, args) => _split.SplitterDistance =
                Math.Max(_split.Panel1MinSize, _split.Width - SidebarWidth - _split.SplitterWidth);
        }

        private Control SidebarHeader()
        {
            var header = new System.Windows.Controls.DockPanel
            {
                Background = WpfTheme.Card, Margin = new System.Windows.Thickness(NOMADTheme.PAD, 0, NOMADTheme.PAD, 0),
            };
            var hide = WpfUi.Action("Hide ▸", () => ShowSidebar(false));
            hide.ToolTip = "Hide payload controls and widen the video.";
            hide.VerticalAlignment = System.Windows.VerticalAlignment.Top;
            hide.Margin = new System.Windows.Thickness(8, NOMADTheme.PAD - 4, 0, 0);
            System.Windows.Controls.DockPanel.SetDock(hide, System.Windows.Controls.Dock.Right);
            header.Children.Add(hide);
            var text = new System.Windows.Controls.StackPanel();
            text.Children.Add(WpfUi.Section("Payloads"));
            if (_config.ActuatorDefinitions == null || _config.ActuatorDefinitions.Count == 0)
            {
                var hint = WpfUi.Text("Add payloads in Settings → Payloads.");
                hint.Foreground = WpfTheme.SecondaryText;
                text.Children.Add(hint);
            }
            header.Children.Add(text);
            var host = WpfUi.Host(header);
            host.Dock = DockStyle.Top; host.AutoSize = true; host.BackColor = CARD_BG;
            return host;
        }

        private void ShowSidebar(bool visible)
        {
            _split.Panel2Collapsed = !visible;
            _sidebarTab.Visible = !visible;
        }

        public void UpdateData() { _videoHost?.RefreshInput(); }
        internal void StopTracking() => _videoHost?.Player?.StopTracking();

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _videoHost?.Dispose();
                _actuatorPanel?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
