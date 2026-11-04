// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    internal sealed class VideoInputPlayer : UserControl
    {
        private readonly NOMADConfig _config;
        private readonly Func<NOMADConfig, string, bool> _selectInput;
        private readonly Func<NOMADConfig, string, string, Task<bool>> _selectModel;
        private ComboBox _models;
        private string _modelPath;
        private readonly ComboBox _selector;
        private readonly Label _empty;
        private readonly ToolTip _toolTip = new ToolTip();
        private string _inputName;
        private bool _loading;
        private bool _savingModel;
        private long _modelRevision;
        private readonly CancellationToken _shutdown;
        internal EmbeddedVideoPlayer Player { get; }

        internal VideoInputPlayer(NOMADConfig config, bool showControls, CancellationToken shutdown,
            Func<IVideoPipeline> createPipeline, Func<NOMADConfig, string, bool> selectInput,
            Func<NOMADConfig, string, string, Task<bool>> selectModel = null)
        {
            _config = config;
            _selectInput = selectInput;
            _selectModel = selectModel;
            _shutdown = shutdown;
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
            var manage = ControlFactory.Button("Manage models", async (sender, args) =>
            {
                try
                {
                    string path = _config.SelectedVideoInput?.DetectorModelProfilePath ?? "";
                    await Task.Run(() => new VisionModelCatalog().Choices(path));
                    if (IsDisposed || _shutdown.IsCancellationRequested) { return; }
                    using (var dialog = new VisionModelsForm(path))
                    { dialog.ShowDialog(this); }
                }
                catch (Exception failure) when (failure is System.IO.IOException ||
                    failure is UnauthorizedAccessException || failure is Newtonsoft.Json.JsonException ||
                    failure is FormatException || failure is ArgumentException)
                {
                    if (!IsDisposed) { _toolTip.SetToolTip(_models,
                        "Model catalog unavailable; current selection and video continue."); }
                }
                if (!IsDisposed) { _modelPath = null; RefreshModel(_config.SelectedVideoInput); }
            }, width: 115, height: 28);
            manage.AccessibleName = "Manage detection models"; row.Controls.Add(manage);
            _models.SelectedIndexChanged += async (sender, args) =>
            {
                if (_loading || _savingModel || !(_models.SelectedItem is VisionModelChoice selected)) { return; }
                var input = _config.SelectedVideoInput;
                if (input == null || VisionModelCatalog.SamePath(input.DetectorModelProfilePath, selected.Path)) { return; }
                _savingModel = true; _models.Enabled = false;
                try
                {
                    if (_selectModel == null || !await _selectModel(_config, input.Name, selected.Path))
                    {
                        if (!IsDisposed) { _toolTip.SetToolTip(_models,
                            "Model selection was not saved. Review the active aircraft and try again."); }
                    }
                }
                catch (Exception error) { Log.Error("Detection model selection failed: " + error); }
                finally
                {
                    _savingModel = false;
                    if (!IsDisposed)
                    {
                        _models.Enabled = _selectModel != null && !_shutdown.IsCancellationRequested;
                        RefreshModel(_config.SelectedVideoInput);
                    }
                }
            };
        }

        private void RefreshModel(VideoInputProfile input)
        {
            if (_models == null || input == null) { return; }
            bool changed = !VisionModelCatalog.SamePath(_modelPath, input.DetectorModelProfilePath);
            if (!changed)
            {
                _loading = true;
                try { _models.SelectedItem = _models.Items.Cast<VisionModelChoice>().FirstOrDefault(choice =>
                    VisionModelCatalog.SamePath(choice.Path, input.DetectorModelProfilePath)); }
                finally { _loading = false; }
                return;
            }
            _modelPath = input.DetectorModelProfilePath;
            long revision = ++_modelRevision;
            Player.UpdateDetectionModel(_modelPath);
            _models.Enabled = false;
            _ = LoadModelChoicesAsync(input.Name, _modelPath, revision);
        }

        private async Task LoadModelChoicesAsync(string inputName, string path, long revision)
        {
            List<VisionModelChoice> choices = null;
            string error = null;
            try
            {
                await Task.Run(() => choices = new VisionModelCatalog().SelectionChoices(path, out error));
            }
            catch (Exception failure)
            {
                Log.Error("Detection model catalog failed: " + failure);
                error = "Model catalog unavailable; current model selection is preserved.";
                choices = new List<VisionModelChoice> { new VisionModelChoice(path, "Current model (catalog unavailable)") };
            }
            if (IsDisposed || _shutdown.IsCancellationRequested || revision != _modelRevision ||
                !string.Equals(_config.SelectedVideoInput?.Name, inputName, StringComparison.OrdinalIgnoreCase) ||
                !VisionModelCatalog.SamePath(_config.SelectedVideoInput?.DetectorModelProfilePath, path)) { return; }
            _loading = true;
            try
            {
                _models.Items.Clear();
                if (error != null) { _toolTip.SetToolTip(_models, error); }
                _models.Items.AddRange(choices.ToArray());
                _models.SelectedItem = choices.Single(choice =>
                    VisionModelCatalog.SamePath(choice.Path, path));
            }
            finally
            {
                _loading = false;
                _models.Enabled = _selectModel != null && !_savingModel && !_shutdown.IsCancellationRequested;
            }
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing) { _toolTip.Dispose(); }
            base.Dispose(disposing);
        }
    }
}
