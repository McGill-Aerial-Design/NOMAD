// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.Linq;
using System.Threading;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    internal sealed class VideoInputPlayer : UserControl
    {
        private readonly NOMADConfig _config;
        private readonly Func<NOMADConfig, string, bool> _selectInput;
        private readonly ComboBox _selector;
        private readonly Label _empty;
        private readonly ToolTip _toolTip = new ToolTip();
        private string _inputName;
        private bool _loading;
        internal EmbeddedVideoPlayer Player { get; }

        internal VideoInputPlayer(NOMADConfig config, bool showControls, CancellationToken shutdown,
            Func<IVideoPipeline> createPipeline, Func<NOMADConfig, string, bool> selectInput)
        {
            _config = config;
            _selectInput = selectInput;
            Dock = DockStyle.Fill;
            BackColor = Color.Black;
            _empty = new Label
            {
                Text = "No video inputs. Configure Settings → Video.", Dock = DockStyle.Fill,
                ForeColor = NOMADTheme.TEXT_SECONDARY, TextAlign = ContentAlignment.MiddleCenter,
            };
            Controls.Add(_empty);
            if (config.SelectedVideoInput != null)
            {
                var input = config.SelectedVideoInput;
                Player = new EmbeddedVideoPlayer(input, showControls, shutdown, createPipeline, config);
                Controls.Add(Player);
                Player.BringToFront();
            }
            _selector = ControlFactory.Combo(config.VideoInputs.Select(input => input.Name).ToArray(),
                config.VideoInputs.Count == 0 ? -1 : 0);
            if (config.VideoInputs.Count == 0) { _selector.Items.Add("No inputs"); }
            _selector.Name = "ActiveVideoInput";
            _selector.AccessibleName = "Active video input";
            _selector.Width = 180;
            _selector.Enabled = selectInput != null && config.VideoInputs.Count > 1 &&
                !shutdown.IsCancellationRequested;
            _toolTip.SetToolTip(_selector,
                "Active input for HUD, Dashboard and Video. Configure inputs in Settings → Video.");
            if (showControls && Player != null)
            {
                _selector.Anchor = AnchorStyles.Top | AnchorStyles.Right;
                _selector.Margin = new Padding(5);
                Player.Toolbar.Controls.Add(_selector, 1, 0);
            }
            else
            {
                var header = new Panel { Dock = DockStyle.Top, Height = 32, BackColor = NOMADTheme.CARD_BG };
                _selector.Dock = DockStyle.Right;
                header.Controls.Add(_selector);
                Controls.Add(header);
            }
            RefreshInput();
            _selector.SelectedIndexChanged += SelectionChanged;
        }

        private void SelectionChanged(object sender, EventArgs args)
        {
            if (_loading || !(_selector.SelectedItem is string name) || name == _config.SelectedVideoInputName)
            { return; }
            if (_selectInput?.Invoke(_config, name) != true)
            {
                const string error = "Video input was not changed or saved. Review the active profile and try again.";
                _toolTip.SetToolTip(_selector, error);
                if (FindForm()?.ContainsFocus == true) { _toolTip.Show(error, _selector, 4000); }
            }
            RefreshInput();
        }

        internal void RefreshInput()
        {
            if (IsDisposed) { return; }
            var input = _config.SelectedVideoInput;
            _loading = true;
            try { _selector.SelectedItem = input?.Name ?? "No inputs"; }
            finally { _loading = false; }
            if (_inputName == input?.Name) { return; }
            if (_inputName != null && input != null) { Player?.UpdateInput(input); }
            _inputName = input?.Name;
            _empty.Visible = input == null;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing) { _toolTip.Dispose(); }
            base.Dispose(disposing);
        }
    }
}
