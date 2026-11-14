// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json;
using Wpf = System.Windows.Controls;
using Thickness = System.Windows.Thickness;

namespace NOMAD.MissionPlanner
{
    internal sealed class VisionModelsForm : NOMADForm
    {
        private readonly VisionModelCatalog _catalog;
        private readonly Wpf.ListBox _models = new Wpf.ListBox { Background = WpfTheme.Input, Foreground = WpfTheme.Text };
        private readonly string _current;
        private readonly Wpf.Button _edit, _export, _remove, _duplicate, _close;
        private readonly Wpf.WrapPanel _actions;
        private readonly CancellationTokenSource _shutdown = new CancellationTokenSource();
        private bool _preparing;

        internal VisionModelsForm(string current = "", VisionModelCatalog catalog = null)
        {
            _current = current; _catalog = catalog ?? new VisionModelCatalog();
            Text = "Detection models · this groundstation"; ClientSize = new Size(690, 340);
            MinimumSize = new Size(460, 260);
            StartPosition = FormStartPosition.CenterParent;
            var root = new Wpf.DockPanel { Margin = new Thickness(8) };
            System.Windows.Automation.AutomationProperties.SetName(_models, "Detection models");
            var actions = _actions = new Wpf.WrapPanel();
            Wpf.DockPanel.SetDock(actions, Wpf.Dock.Bottom);
            actions.Children.Add(WpfUi.Action("Add", async () => await Run(() => Edit(null))));
            _duplicate = WpfUi.Action("Duplicate", async () => await DuplicateSelected());
            actions.Children.Add(_duplicate);
            _edit = WpfUi.Action("Edit", async () => await Run(() => Edit(Selected())));
            actions.Children.Add(_edit);
            actions.Children.Add(WpfUi.Action("Import", async () => await Run(Import)));
            _export = WpfUi.Action("Export", async () => await Run(Export));
            _remove = WpfUi.Action("Remove", async () => await Run(Remove));
            actions.Children.Add(_export); actions.Children.Add(_remove);
            _close = WpfUi.Action("Close", Close);
            _close.IsCancel = true;
            actions.Children.Add(_close);
            root.PreviewKeyDown += (sender, args) =>
            {
                if (args.Key == System.Windows.Input.Key.Escape) { Close(); args.Handled = true; }
            };
            root.Children.Add(actions); root.Children.Add(_models);
            Controls.Add(WpfUi.Host(root));
            _models.SelectionChanged += (sender, args) => UpdateActions();
            Shown += async (sender, args) => await Run(() => RefreshModels(_current));
        }

        private VisionModelChoice Selected() => _models.SelectedItem as VisionModelChoice;
        private async Task RefreshModels(string preferred = null)
        {
            string selected = preferred ?? Selected()?.Path ?? "";
            var choices = await Task.Run(() => _catalog.Choices(_current));
            if (IsDisposed || _shutdown.IsCancellationRequested) { return; }
            _models.Items.Clear();
            foreach (var choice in choices) { _models.Items.Add(choice); }
            _models.SelectedIndex = 0;
            for (int index = 0; index < _models.Items.Count; index++)
            {
                if (VisionModelCatalog.SamePath(((VisionModelChoice)_models.Items[index]).Path, selected))
                { _models.SelectedIndex = index; break; }
            }
        }

        private void UpdateActions()
        {
            if (_preparing) { return; }
            string path = Selected()?.Path;
            _duplicate.IsEnabled = Selected() != null;
            _edit.IsEnabled = _export.IsEnabled = !string.IsNullOrEmpty(path);
            _remove.IsEnabled = !string.IsNullOrEmpty(path) &&
                VisionModelCatalog.SamePath(Path.GetDirectoryName(path), _catalog.DirectoryPath);
        }

        internal static VisionModelProfile PrepareDuplicate(VisionModelChoice choice, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            var profile = choice.Path == "" ? VisionModelProfile.Builtin() : VisionModelProfile.Load(choice.Path);
            if (choice.Path == "")
            {
                VisionRuntime.EnsureBuiltinModel(token);
                profile.Model = VisionRuntime.BuiltinModelPath;
            }
            token.ThrowIfCancellationRequested();
            profile.Name = profile.Name.Substring(0, Math.Min(75, profile.Name.Length)) + " copy";
            return profile;
        }

        private async Task DuplicateSelected()
        {
            var choice = Selected();
            if (_preparing || choice == null) { return; }
            _preparing = true; _models.IsEnabled = false;
            foreach (Wpf.Control control in _actions.Children) { control.IsEnabled = ReferenceEquals(control, _close); }
            _duplicate.Content = "Preparing…";
            try
            {
                var token = _shutdown.Token;
                var profile = await Task.Run(() => PrepareDuplicate(choice, token), token);
                if (!IsDisposed && !token.IsCancellationRequested) { await EditProfile(profile); }
            }
            catch (OperationCanceledException) { }
            catch (Exception error) when (!(error is OutOfMemoryException) && !_shutdown.IsCancellationRequested)
            { MessageBox.Show(this, "Model copy unavailable. Check the model profile and local vision runtime.",
                "Duplicate model", MessageBoxButtons.OK, MessageBoxIcon.Warning); }
            finally
            {
                if (!IsDisposed)
                {
                    _preparing = false; _models.IsEnabled = true; _duplicate.Content = "Duplicate";
                    foreach (Wpf.Control control in _actions.Children) { control.IsEnabled = true; }
                    UpdateActions();
                }
            }
        }

        private async Task Edit(VisionModelChoice choice)
        {
            if (choice?.Path == "") { throw new InvalidOperationException("The built-in model is read-only. Use Add for another model."); }
            var profile = choice == null ? VisionModelProfile.Builtin() :
                await Task.Run(() => VisionModelProfile.Load(choice.Path));
            if (IsDisposed) { return; }
            if (choice == null) { profile.Name = "New YOLOX model"; profile.Model = ""; profile.Sha256 = ""; }
            bool managed = choice != null && VisionModelCatalog.SamePath(Path.GetDirectoryName(choice.Path),
                _catalog.DirectoryPath);
            await EditProfile(profile, managed ? choice.Path : null);
        }

        private async Task EditProfile(VisionModelProfile profile, string path = null)
        {
            while (true)
            {
                using (var editor = new VisionModelEditorForm(profile))
                {
                    if (editor.ShowDialog(this) != DialogResult.OK) { return; }
                    profile = editor.Draft;
                }
                try
                {
                    var draft = profile;
                    string saved = await Task.Run(() => _catalog.Save(draft, path));
                    if (!IsDisposed) { await RefreshModels(saved); }
                    return;
                }
                catch (FormatException error)
                {
                    if (IsDisposed) { return; }
                    MessageBox.Show(this, error.Message, "Review model profile", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
            }
        }

        private async Task Import()
        {
            using (var dialog = new OpenFileDialog { Filter = "Model profile (*.json)|*.json" })
            {
                if (dialog.ShowDialog(this) != DialogResult.OK) { return; }
                string path = dialog.FileName;
                var profile = await Task.Run(() => VisionModelProfile.Load(path));
                if (!IsDisposed) { await EditProfile(profile); }
            }
        }

        private async Task Export()
        {
            var selected = Selected(); if (selected == null) { return; }
            if (selected.Path == "") { throw new InvalidOperationException("Built-in weights are embedded; export a custom profile instead."); }
            using (var dialog = new SaveFileDialog { Filter = "Model profile (*.json)|*.json", FileName = "vision-model.json" })
            {
                if (dialog.ShowDialog(this) == DialogResult.OK)
                {
                    string output = dialog.FileName;
                    await Task.Run(() => NOMADConfig.WriteAtomically(output,
                        JsonConvert.SerializeObject(VisionModelProfile.Load(selected.Path), Formatting.Indented), keepBackup: false));
                }
            }
        }

        private async Task Remove()
        {
            var selected = Selected(); if (selected == null) { return; }
            if (MessageBox.Show(this, "Remove this profile? Cameras referencing it will report detection unavailable. " +
                "The ONNX weights remain in place.", "Remove model profile", MessageBoxButtons.OKCancel,
                MessageBoxIcon.Warning) != DialogResult.OK) { return; }
            await Task.Run(() => _catalog.Remove(selected.Path));
            if (!IsDisposed) { await RefreshModels(); }
        }

        private async Task Run(Func<Task> action)
        {
            if (_preparing) { return; }
            _preparing = true; _models.IsEnabled = false;
            foreach (Wpf.Control control in _actions.Children) { control.IsEnabled = ReferenceEquals(control, _close); }
            try { await action(); }
            catch (Exception error) when (!(error is OutOfMemoryException))
            {
                if (!IsDisposed && !_shutdown.IsCancellationRequested)
                { MessageBox.Show(this, error.Message, "Model profile requires review", MessageBoxButtons.OK, MessageBoxIcon.Warning); }
            }
            finally
            {
                if (!IsDisposed)
                {
                    _preparing = false; _models.IsEnabled = true;
                    foreach (Wpf.Control control in _actions.Children) { control.IsEnabled = true; }
                    UpdateActions();
                }
            }
        }

        protected override void OnFormClosing(FormClosingEventArgs args)
        { _shutdown.Cancel(); base.OnFormClosing(args); }

        protected override void Dispose(bool disposing)
        { if (disposing) { _shutdown.Cancel(); } base.Dispose(disposing); }
    }
}
