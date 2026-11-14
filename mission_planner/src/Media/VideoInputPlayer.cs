// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using ComboBox = System.Windows.Controls.ComboBox;

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
        private readonly System.Windows.Controls.TextBlock _selectionError;
        private readonly System.Windows.Forms.Integration.ElementHost _empty;
        private string _inputName;
        private bool _loading;
        private bool _savingModel;
        private long _modelRevision;
        private readonly CancellationToken _shutdown;
        internal EmbeddedVideoPlayer Player { get; }

        internal VideoInputPlayer(NOMADConfig config, bool showControls, CancellationToken shutdown,
            Func<IVideoPipeline> createPipeline, Func<NOMADConfig, string, bool> selectInput,
            Func<NOMADConfig, string, string, Task<bool>> selectModel = null,
            Func<CancellationToken, VisualTrackingMode, IVisualTracker> createTracker = null)
        {
            _config = config;
            _selectInput = selectInput;
            _selectModel = selectModel;
            _shutdown = shutdown;
            Dock = DockStyle.Fill;
            BackColor = Color.Black;
            var emptyMessage = WpfUi.Text("No video inputs. Configure Settings → Video.");
            emptyMessage.TextAlignment = System.Windows.TextAlignment.Center;
            emptyMessage.VerticalAlignment = System.Windows.VerticalAlignment.Center;
            _empty = WpfUi.Host(emptyMessage);
            Controls.Add(_empty);
            if (config.SelectedVideoInput != null)
            {
                var input = config.SelectedVideoInput;
                Player = new EmbeddedVideoPlayer(input, showControls, shutdown, createPipeline, config,
                    createTracker: createTracker);
                Controls.Add(Player);
                Player.BringToFront();
            }
            _selector = showControls && Player != null ? Player.VideoControls.InputSelector : new ComboBox
            {
                Resources = WpfTheme.Resources(), MinWidth = 180, MaxWidth = 320,
                Margin = new System.Windows.Thickness(8, 4, 8, 4),
                HorizontalAlignment = System.Windows.HorizontalAlignment.Right,
            };
            foreach (var input in config.VideoInputs) { _selector.Items.Add(input.Name); }
            if (config.VideoInputs.Count == 0) { _selector.Items.Add("No inputs"); }
            _selector.Name = "ActiveVideoInput";
            System.Windows.Automation.AutomationProperties.SetName(_selector, "Active video input");
            _selector.IsEnabled = selectInput != null && config.VideoInputs.Count > 1 &&
                !shutdown.IsCancellationRequested;
            _selector.ToolTip = "Active input for HUD, Dashboard and Video. Configure inputs in Settings → Video.";
            _selectionError = showControls && Player != null ? Player.VideoControls.SelectionError : WpfUi.Text("");
            _selectionError.Foreground = WpfTheme.Warning;
            _selectionError.Visibility = System.Windows.Visibility.Collapsed;
            if (showControls && Player != null)
            {
                AddModelSelection(shutdown);
            }
            else
            {
                var row = new System.Windows.Controls.Grid { Background = WpfTheme.Card };
                row.RowDefinitions.Add(new System.Windows.Controls.RowDefinition { Height = System.Windows.GridLength.Auto });
                row.RowDefinitions.Add(new System.Windows.Controls.RowDefinition { Height = System.Windows.GridLength.Auto });
                row.Children.Add(_selector);
                _selectionError.Margin = new System.Windows.Thickness(8, 0, 8, 6);
                System.Windows.Controls.Grid.SetRow(_selectionError, 1); row.Children.Add(_selectionError);
                var header = new System.Windows.Forms.Integration.ElementHost
                { Dock = DockStyle.Top, AutoSize = true, Child = row, BackColor = NOMADTheme.CARD_BG,
                    Font = NOMADTheme.Font() };
                Controls.Add(header);
            }
            RefreshInput();
            _selector.SelectionChanged += SelectionChanged;
        }

        private void SelectionChanged(object sender, EventArgs args)
        {
            if (_loading || !(_selector.SelectedItem is string name) || name == _config.SelectedVideoInputName)
            { return; }
            if (_selectInput?.Invoke(_config, name) != true)
            {
                const string error = "Video input was not changed or saved. Review the active profile and try again.";
                _selector.ToolTip = error;
                _selectionError.Text = error;
                _selectionError.Visibility = System.Windows.Visibility.Visible;
            }
            else
            {
                _selector.ToolTip = "Active input for HUD, Dashboard and Video. Configure inputs in Settings → Video.";
                _selectionError.Text = ""; _selectionError.Visibility = System.Windows.Visibility.Collapsed;
            }
            RefreshInput();
            Player?.Toolbar.Parent?.PerformLayout();
            PerformLayout();
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
            _models = Player.VideoControls.ModelSelector;
            _models.IsEnabled = _selectModel != null && !shutdown.IsCancellationRequested;
            Player.VideoControls.ManageModelsButton.Click += async (sender, args) =>
            {
                try
                {
                    string path = _config.SelectedVideoInput?.DetectorModelProfilePath ?? "";
                    await Task.Run(() => new VisionModelCatalog().Choices(path));
                    if (IsDisposed || _shutdown.IsCancellationRequested) { return; }
                    using (var dialog = new VisionModelsForm(path))
                    { dialog.ShowDialog(Player.Toolbar.FindForm() ?? FindForm()); }
                }
                catch (Exception failure) when (failure is System.IO.IOException ||
                    failure is UnauthorizedAccessException || failure is Newtonsoft.Json.JsonException ||
                    failure is FormatException || failure is ArgumentException)
                {
                    if (!IsDisposed) { _models.ToolTip = "Model catalog unavailable; current selection and video continue."; }
                }
                if (!IsDisposed) { _modelPath = null; RefreshModel(_config.SelectedVideoInput); }
            };
            _models.SelectionChanged += async (sender, args) =>
            {
                if (_loading || _savingModel || !(_models.SelectedItem is VisionModelChoice selected)) { return; }
                var input = _config.SelectedVideoInput;
                if (input == null || VisionModelCatalog.SamePath(input.DetectorModelProfilePath, selected.Path)) { return; }
                _savingModel = true; _models.IsEnabled = false;
                try
                {
                    if (_selectModel == null || !await _selectModel(_config, input.Name, selected.Path))
                    {
                        if (!IsDisposed) { _models.ToolTip =
                            "Model selection was not saved. Review the active aircraft and try again."; }
                    }
                }
                catch (Exception error) { Log.Error("Detection model selection failed: " + error); }
                finally
                {
                    _savingModel = false;
                    if (!IsDisposed)
                    {
                        _models.IsEnabled = _selectModel != null && !_shutdown.IsCancellationRequested;
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
            _models.IsEnabled = false;
            _ = LoadModelChoicesAsync(_modelPath, revision);
        }

        private async Task LoadModelChoicesAsync(string path, long revision)
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
                !VisionModelCatalog.SamePath(_config.SelectedVideoInput?.DetectorModelProfilePath, path)) { return; }
            _loading = true;
            try
            {
                _models.Items.Clear();
                if (error != null) { _models.ToolTip = error; }
                foreach (var choice in choices) { _models.Items.Add(choice); }
                _models.SelectedItem = choices.Single(choice =>
                    VisionModelCatalog.SamePath(choice.Path, path));
            }
            finally
            {
                _loading = false;
                _models.IsEnabled = _selectModel != null && !_savingModel && !_shutdown.IsCancellationRequested;
            }
        }

    }
}
