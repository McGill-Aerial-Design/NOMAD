// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    internal sealed class VisionModelsForm : NOMADForm
    {
        private readonly VisionModelCatalog _catalog;
        private readonly ListBox _models = new ListBox { Dock = DockStyle.Fill, AccessibleName = "Detection models",
            BackColor = NOMADTheme.INPUT_BG, ForeColor = NOMADTheme.TEXT_PRIMARY };
        private readonly string _current;
        private readonly Button _edit, _export, _remove, _duplicate;
        private readonly FlowLayoutPanel _actions;
        private readonly CancellationTokenSource _shutdown = new CancellationTokenSource();
        private bool _preparing;

        internal VisionModelsForm(string current = "", VisionModelCatalog catalog = null)
        {
            _current = current; _catalog = catalog ?? new VisionModelCatalog();
            Text = "Detection models · this groundstation"; ClientSize = new Size(690, 340);
            MinimumSize = new Size(460, 260);
            StartPosition = FormStartPosition.CenterParent;
            var actions = _actions = new FlowLayoutPanel { Dock = DockStyle.Bottom, AutoSize = true, Padding = new Padding(8) };
            actions.Controls.Add(ControlFactory.Button("Add", (sender, args) => Run(() => Edit(null))));
            _duplicate = ControlFactory.Button("Duplicate", async (sender, args) => await DuplicateSelected());
            actions.Controls.Add(_duplicate);
            _edit = ControlFactory.Button("Edit", (sender, args) => Run(() => Edit(Selected())));
            actions.Controls.Add(_edit);
            actions.Controls.Add(ControlFactory.Button("Import", (sender, args) => Run(Import)));
            _export = ControlFactory.Button("Export", (sender, args) => Run(Export));
            _remove = ControlFactory.Button("Remove", (sender, args) => Run(Remove));
            actions.Controls.Add(_export); actions.Controls.Add(_remove);
            var close = ControlFactory.Button("Close", (sender, args) => Close());
            close.DialogResult = DialogResult.Cancel; CancelButton = close;
            actions.Controls.Add(close);
            Controls.Add(_models); Controls.Add(actions);
            _models.SelectedIndexChanged += (sender, args) => UpdateActions();
            RefreshModels(_current);
        }

        private VisionModelChoice Selected() => _models.SelectedItem as VisionModelChoice;
        private void RefreshModels(string preferred = null)
        {
            string selected = preferred ?? Selected()?.Path ?? "";
            _models.Items.Clear();
            _models.Items.AddRange(_catalog.Choices(_current).ToArray());
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
            _duplicate.Enabled = Selected() != null;
            _edit.Enabled = _export.Enabled = !string.IsNullOrEmpty(path);
            _remove.Enabled = !string.IsNullOrEmpty(path) &&
                VisionModelCatalog.SamePath(Path.GetDirectoryName(path), _catalog.DirectoryPath);
        }

        internal static VisionModelProfile PrepareDuplicate(VisionModelChoice choice, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            var profile = choice.Path == "" ? VisionModelProfile.Builtin() : VisionModelProfile.Load(choice.Path);
            if (choice.Path == "")
            {
                VisionRuntime.EnsureLoaded(token);
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
            _preparing = true; _models.Enabled = false;
            foreach (Control action in _actions.Controls) { action.Enabled = ReferenceEquals(action, CancelButton); }
            _duplicate.Text = "Preparing…";
            try
            {
                var token = _shutdown.Token;
                var profile = await Task.Run(() => PrepareDuplicate(choice, token), token);
                if (!IsDisposed && !token.IsCancellationRequested) { EditProfile(profile); }
            }
            catch (OperationCanceledException) { }
            catch (Exception error) when (!(error is OutOfMemoryException) && !_shutdown.IsCancellationRequested)
            { MessageBox.Show(this, "Model copy unavailable. Check the model profile and local vision runtime.",
                "Duplicate model", MessageBoxButtons.OK, MessageBoxIcon.Warning); }
            finally
            {
                if (!IsDisposed)
                {
                    _preparing = false; _models.Enabled = true; _duplicate.Text = "Duplicate";
                    foreach (Control action in _actions.Controls) { action.Enabled = true; }
                    UpdateActions();
                }
            }
        }

        private void Edit(VisionModelChoice choice)
        {
            if (choice?.Path == "") { throw new InvalidOperationException("The built-in model is read-only. Use Add for another model."); }
            var profile = choice == null ? VisionModelProfile.Builtin() : VisionModelProfile.Load(choice.Path);
            if (choice == null) { profile.Name = "New YOLOX model"; profile.Model = ""; profile.Sha256 = ""; }
            bool managed = choice != null && VisionModelCatalog.SamePath(Path.GetDirectoryName(choice.Path),
                _catalog.DirectoryPath);
            EditProfile(profile, managed ? choice.Path : null);
        }

        private void EditProfile(VisionModelProfile profile, string path = null)
        {
            while (true)
            {
                using (var editor = new VisionModelEditorForm(profile))
                {
                    if (editor.ShowDialog(this) != DialogResult.OK) { return; }
                    profile = editor.Draft;
                }
                try { string saved = _catalog.Save(profile, path); RefreshModels(saved); return; }
                catch (FormatException error)
                { MessageBox.Show(this, error.Message, "Review model profile", MessageBoxButtons.OK, MessageBoxIcon.Warning); }
            }
        }

        private void Import()
        {
            using (var dialog = new OpenFileDialog { Filter = "Model profile (*.json)|*.json" })
            {
                if (dialog.ShowDialog(this) != DialogResult.OK) { return; }
                EditProfile(VisionModelProfile.Load(dialog.FileName));
            }
        }

        private void Export()
        {
            var selected = Selected(); if (selected == null) { return; }
            if (selected.Path == "") { throw new InvalidOperationException("Built-in weights are embedded; export a custom profile instead."); }
            var profile = VisionModelProfile.Load(selected.Path);
            using (var dialog = new SaveFileDialog { Filter = "Model profile (*.json)|*.json", FileName = "vision-model.json" })
            {
                if (dialog.ShowDialog(this) == DialogResult.OK)
                { NOMADConfig.WriteAtomically(dialog.FileName, JsonConvert.SerializeObject(profile, Formatting.Indented)); }
            }
        }

        private void Remove()
        {
            var selected = Selected(); if (selected == null) { return; }
            if (MessageBox.Show(this, "Remove this profile? Cameras referencing it will report detection unavailable. " +
                "The ONNX weights remain in place.", "Remove model profile", MessageBoxButtons.OKCancel,
                MessageBoxIcon.Warning) != DialogResult.OK) { return; }
            _catalog.Remove(selected.Path); RefreshModels();
        }

        private void Run(Action action)
        {
            try { action(); }
            catch (Exception error) when (!(error is OutOfMemoryException))
            { MessageBox.Show(this, error.Message, "Model profile requires review", MessageBoxButtons.OK, MessageBoxIcon.Warning); }
        }

        protected override void OnFormClosing(FormClosingEventArgs args)
        { _shutdown.Cancel(); base.OnFormClosing(args); }

        protected override void Dispose(bool disposing)
        { if (disposing) { _shutdown.Cancel(); } base.Dispose(disposing); }
    }
}
