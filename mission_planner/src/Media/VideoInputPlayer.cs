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
        private readonly Func<NOMADConfig, string, string, bool> _selectModel;
        private ComboBox _models;
        private string _modelPath;
        private readonly ComboBox _selector;
        private readonly Label _empty;
        private readonly ToolTip _toolTip = new ToolTip();
        private string _inputName;
        private bool _loading;
        internal EmbeddedVideoPlayer Player { get; }

        internal VideoInputPlayer(NOMADConfig config, bool showControls, CancellationToken shutdown,
            Func<IVideoPipeline> createPipeline, Func<NOMADConfig, string, bool> selectInput,
            Func<NOMADConfig, string, string, bool> selectModel = null)
        {
            _config = config;
            _selectInput = selectInput;
            _selectModel = selectModel;
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
                AddModelSelection(shutdown);
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
            if (_inputName == input?.Name)
            {
                RefreshModel(input);
                return;
            }
            if (_inputName != null && input != null) { Player?.UpdateInput(input); }
            _inputName = input?.Name;
            _empty.Visible = input == null;
            RefreshModel(input);
        }

        private void AddModelSelection(CancellationToken shutdown)
        {
            _models = ControlFactory.Combo(Array.Empty<string>(), -1);
            _models.Width = 220; _models.AccessibleName = "Detection model"; _models.Name = "DetectionModel";
            _models.Enabled = _selectModel != null && !shutdown.IsCancellationRequested;
            _toolTip.SetToolTip(_models, "Select the detection model for this camera. Video and generic region tracking continue.");
            var row = (FlowLayoutPanel)Player.Toolbar.GetControlFromPosition(0, 1);
            row.Controls.Add(_models);
            var manage = ControlFactory.Button("Manage models", (sender, args) =>
            {
                try
                {
                    using (var dialog = new VisionModelsForm(_config.SelectedVideoInput?.DetectorModelProfilePath ?? ""))
                    { dialog.ShowDialog(this); }
                }
                catch (Exception failure) when (failure is System.IO.IOException ||
                    failure is UnauthorizedAccessException || failure is Newtonsoft.Json.JsonException ||
                    failure is FormatException || failure is ArgumentException)
                { _toolTip.SetToolTip(_models, "Model catalog unavailable; current selection and video continue."); }
                _modelPath = null; RefreshModel(_config.SelectedVideoInput);
            }, width: 115, height: 28);
            manage.AccessibleName = "Manage detection models"; row.Controls.Add(manage);
            _models.SelectedIndexChanged += (sender, args) =>
            {
                if (_loading || !(_models.SelectedItem is VisionModelChoice selected)) { return; }
                var input = _config.SelectedVideoInput;
                if (input == null || VisionModelCatalog.SamePath(input.DetectorModelProfilePath, selected.Path)) { return; }
                if (_selectModel?.Invoke(_config, input.Name, selected.Path) != true)
                {
                    _toolTip.SetToolTip(_models, "Model selection was not saved. Review the active aircraft and try again.");
                }
                RefreshModel(_config.SelectedVideoInput);
            };
        }

        private void RefreshModel(VideoInputProfile input)
        {
            if (_models == null || input == null) { return; }
            bool changed = !VisionModelCatalog.SamePath(_modelPath, input.DetectorModelProfilePath);
            if (!changed)
            {
                _loading = true;
                try { _models.SelectedItem = _models.Items.Cast<VisionModelChoice>().Single(choice =>
                    VisionModelCatalog.SamePath(choice.Path, input.DetectorModelProfilePath)); }
                finally { _loading = false; }
                return;
            }
            _loading = true;
            try
            {
                _models.Items.Clear();
                var choices = new VisionModelCatalog().SelectionChoices(input.DetectorModelProfilePath, out string error);
                if (error != null) { _toolTip.SetToolTip(_models, error); }
                _models.Items.AddRange(choices.ToArray());
                _models.SelectedItem = choices.Single(choice =>
                    VisionModelCatalog.SamePath(choice.Path, input.DetectorModelProfilePath));
            }
            finally { _loading = false; }
            if (changed) { Player.UpdateDetectionModel(input.DetectorModelProfilePath); }
            _modelPath = input.DetectorModelProfilePath;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing) { _toolTip.Dispose(); }
            base.Dispose(disposing);
        }
    }
}
