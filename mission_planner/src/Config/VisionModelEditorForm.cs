// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Collections.ObjectModel;
using System.Globalization;
using Wpf = System.Windows.Controls;
using Thickness = System.Windows.Thickness;

namespace NOMAD.MissionPlanner
{
    internal sealed class VisionModelEditorForm : NOMADForm
    {
        private readonly Wpf.TextBox _name, _model, _hash, _size, _nms;
        private readonly Wpf.Slider _score;
        private readonly Wpf.DataGrid _classes;
        private readonly ObservableCollection<VisionClassRow> _rows = new ObservableCollection<VisionClassRow>();
        private readonly Wpf.WrapPanel _classActions;
        private readonly Wpf.Button _save, _browse, _test;
        private readonly CancellationTokenSource _shutdown = new CancellationTokenSource();
        private bool _hashing;
        internal VisionModelProfile Draft { get; private set; }

        // Everyday choices (classes, confidence) come first; weights and tensor details sit under Advanced.
        // The built-in model keeps its embedded weights, labels and input size fixed.
        internal VisionModelEditorForm(VisionModelProfile profile, bool builtin = false)
        {
            Text = builtin ? VisionModelCatalog.BuiltinName : "Detection model"; ClientSize = new Size(650, 640);
            MinimumSize = new Size(570, 460);
            StartPosition = FormStartPosition.CenterParent;
            var root = new Wpf.DockPanel { Margin = new Thickness(12) };
            var fields = new Wpf.StackPanel();
            var scroll = new Wpf.ScrollViewer { Content = fields, VerticalScrollBarVisibility = Wpf.ScrollBarVisibility.Auto };
            _name = TextField(fields, "Name", profile.Name);
            _name.IsReadOnly = builtin;
            for (int index = 0; index < profile.Labels.Length; index++)
            { _rows.Add(new VisionClassRow { Id = index, Label = profile.Labels[index], Detect = profile.AllowedClasses.Contains(index) }); }
            fields.Children.Add(WpfUi.Section("Classes to detect"));
            fields.Children.Add(new VisionClassPicker(_rows, builtin ? VisionModelProfile.Builtin().AllowedClasses : null));
            fields.Children.Add(WpfUi.Section("Minimum confidence"));
            _score = ConfidenceSlider(fields, profile.ScoreThreshold);

            var advanced = new Wpf.StackPanel();
            _model = TextField(advanced, "ONNX weights", profile.Model); _model.IsReadOnly = true;
            _browse = WpfUi.Action("Browse…", async () => await BrowseModel());
            _browse.ToolTip = "Choose a trusted local ONNX file, up to 256 MiB.";
            AddField(advanced, "Weights", _browse);
            _hash = TextField(advanced, "SHA256", profile.Sha256); _hash.IsReadOnly = true;
            _size = TextField(advanced, "Input size", profile.InputSize.ToString(CultureInfo.InvariantCulture));
            _classes = CreateClassTable();
            AddField(advanced, "Class table", _classes);
            _classes.ToolTip = "Rows must match the model's output class order. Detect selects which classes to show.";
            _classActions = new Wpf.WrapPanel();
            _classActions.Children.Add(WpfUi.Action("Add class", () =>
            { if (_rows.Count < 256) { _rows.Add(new VisionClassRow { Id = _rows.Count, Label = "" }); } }));
            _classActions.Children.Add(WpfUi.Action("Remove class", RemoveClass));
            var import = WpfUi.Action("Import labels", BrowseLabels);
            import.ToolTip = "Replace classes with a text file, one label per line in model output order. All imported classes start selected.";
            _classActions.Children.Add(import);
            AddField(advanced, "", _classActions);
            if (builtin)
            {
                foreach (var fixedPart in new System.Windows.UIElement[] { _model, _browse, _hash, _size, _classActions })
                { fixedPart.IsEnabled = false; }
                _classes.IsReadOnly = true;
            }
            _nms = TextField(advanced, "NMS overlap", profile.NmsThreshold.ToString("R", CultureInfo.CurrentCulture));
            _nms.ToolTip = "Boxes overlapping more than this are merged; lower values remove more duplicates.";
            var format = WpfUi.Text("YOLOX grid (ONNX)");
            AddField(advanced, "Format", format);
            format.ToolTip = "One float input/output. BGR NCHW 0–255, top-left letterbox, padding 114, " +
                "strides 8/16/32. Other ONNX architectures are unsupported.";
            fields.Children.Add(new Wpf.Expander { Header = builtin ? "Advanced" : "Model weights and advanced",
                Content = advanced, IsExpanded = !builtin, Foreground = WpfTheme.Text, Margin = new Thickness(0, 12, 0, 0) });

            var actions = new Wpf.WrapPanel();
            _save = WpfUi.Action("Save", AcceptDraft);
            _test = WpfUi.Action("Test model", async () => await TestModel());
            var cancel = WpfUi.Action("Cancel", Close);
            _save.IsDefault = true; cancel.IsCancel = true;
            actions.Children.Add(_test); actions.Children.Add(_save); actions.Children.Add(cancel);
            Wpf.DockPanel.SetDock(actions, Wpf.Dock.Bottom); root.Children.Add(actions); root.Children.Add(scroll);
            root.PreviewKeyDown += (sender, args) => { if (args.Key == System.Windows.Input.Key.Escape)
                { Close(); args.Handled = true; } };
            Controls.Add(WpfUi.Host(root));
        }

        private static Wpf.Slider ConfidenceSlider(Wpf.StackPanel panel, float value)
        {
            var row = new Wpf.DockPanel { Margin = new Thickness(0, 0, 0, 8) };
            var slider = new Wpf.Slider { Minimum = .01, Maximum = .99, Value = value, SmallChange = .01, LargeChange = .05,
                TickFrequency = .01, IsSnapToTickEnabled = true, VerticalAlignment = System.Windows.VerticalAlignment.Center,
                ToolTip = "Lower shows more detections, including false ones; higher shows only confident ones." };
            System.Windows.Automation.AutomationProperties.SetName(slider, "Minimum confidence");
            var shown = new Wpf.TextBlock { Width = 44, Margin = new Thickness(8, 0, 0, 0),
                VerticalAlignment = System.Windows.VerticalAlignment.Center };
            slider.ValueChanged += (sender, args) => shown.Text = slider.Value.ToString("0.00", CultureInfo.CurrentCulture);
            shown.Text = slider.Value.ToString("0.00", CultureInfo.CurrentCulture);
            Wpf.DockPanel.SetDock(shown, Wpf.Dock.Right);
            row.Children.Add(shown); row.Children.Add(slider);
            panel.Children.Add(row);
            return slider;
        }

        private Wpf.DataGrid CreateClassTable()
        {
            var table = new Wpf.DataGrid { Height = 170, AutoGenerateColumns = false, CanUserAddRows = false,
                CanUserDeleteRows = false, CanUserSortColumns = false, SelectionMode = Wpf.DataGridSelectionMode.Single,
                HeadersVisibility = Wpf.DataGridHeadersVisibility.Column, ItemsSource = _rows };
            table.Columns.Add(new Wpf.DataGridTextColumn { Header = "ID", IsReadOnly = true, Binding = new System.Windows.Data.Binding("Id") });
            table.Columns.Add(new Wpf.DataGridTextColumn { Header = "Class", Width = new Wpf.DataGridLength(1, Wpf.DataGridLengthUnitType.Star), Binding = new System.Windows.Data.Binding("Label") });
            table.Columns.Add(new Wpf.DataGridCheckBoxColumn { Header = "Detect", Binding = new System.Windows.Data.Binding("Detect") });
            return table;
        }

        private void RemoveClass()
        {
            int index = _classes.SelectedIndex; if (index < 0) { return; }
            _classes.CancelEdit(Wpf.DataGridEditingUnit.Cell); _classes.CancelEdit(Wpf.DataGridEditingUnit.Row);
            _rows.RemoveAt(index);
            for (int i = 0; i < _rows.Count; i++) { _rows[i].Id = i; }
        }

        private void BrowseLabels()
        {
            using (var dialog = new OpenFileDialog { Filter = "Class labels (*.txt)|*.txt", CheckFileExists = true })
            {
                if (dialog.ShowDialog(this) != DialogResult.OK) { return; }
                try { ImportLabels(dialog.FileName); }
                catch (Exception error) when (error is IOException || error is UnauthorizedAccessException ||
                    error is FormatException || error is ArgumentException)
                { MessageBox.Show(this, "Choose a text file up to 64 KiB with 1–256 labels, one per line " +
                    "and up to 80 characters each.", "Review class labels", MessageBoxButtons.OK, MessageBoxIcon.Warning); }
            }
        }

        internal void ImportLabels(string path)
        {
            string[] labels;
            using (var stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read))
            using (var reader = new StreamReader(stream))
            {
                if (stream.Length > 65536) { throw new FormatException("Class label file exceeds 64 KiB."); }
                var lines = new System.Collections.Generic.List<string>();
                string line;
                while ((line = reader.ReadLine()) != null)
                {
                    line = line.Trim();
                    if (lines.Count == 256 || line.Length == 0 || line.Length > 80 || line.Any(char.IsControl))
                    { throw new FormatException("Invalid class label list."); }
                    lines.Add(line);
                }
                if (lines.Count == 0) { throw new FormatException("Class label list is empty."); }
                labels = lines.ToArray();
            }
            _classes.CancelEdit(Wpf.DataGridEditingUnit.Cell); _classes.CancelEdit(Wpf.DataGridEditingUnit.Row); _rows.Clear();
            for (int index = 0; index < labels.Length; index++) { _rows.Add(new VisionClassRow { Id = index, Label = labels[index], Detect = true }); }
        }

        private static void AddField(Wpf.StackPanel panel, string label, System.Windows.FrameworkElement control)
        {
            panel.Children.Add(WpfUi.Text(label)); control.Margin = new Thickness(0, 0, 0, 8);
            System.Windows.Automation.AutomationProperties.SetName(control, label); panel.Children.Add(control);
        }

        private static Wpf.TextBox TextField(Wpf.StackPanel panel, string label, string text)
        {
            var field = new Wpf.TextBox { Text = text };
            AddField(panel, label, field); return field;
        }

        private async Task BrowseModel()
        {
            using (var dialog = new OpenFileDialog { Filter = "ONNX model (*.onnx)|*.onnx", CheckFileExists = true })
            {
                if (dialog.ShowDialog(this) != DialogResult.OK) { return; }
                string path = Path.GetFullPath(dialog.FileName);
                SetBusy(true);
                try
                {
                    var token = _shutdown.Token;
                    string hash = await Task.Run(() => HashModel(path, token));
                    if (IsDisposed || _shutdown.IsCancellationRequested) { return; }
                    _model.Text = path; _hash.Text = hash;
                }
                catch (OperationCanceledException) { }
                catch (Exception) when (!_shutdown.IsCancellationRequested)
                { MessageBox.Show(this, "Model could not be read. Choose a local ONNX file up to 256 MiB.", Text); }
                finally
                {
                    if (!IsDisposed) { SetBusy(false); }
                }
            }
        }

        internal static string HashModel(string path, CancellationToken token)
        {
            using (var input = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read))
            using (var hash = SHA256.Create())
            {
                if (input.Length < 1 || input.Length > 256 * 1024 * 1024)
                { throw new FormatException("Model must be between 1 byte and 256 MiB."); }
                byte[] buffer = new byte[65536];
                int count;
                while ((count = input.Read(buffer, 0, buffer.Length)) != 0)
                { token.ThrowIfCancellationRequested(); hash.TransformBlock(buffer, 0, count, buffer, 0); }
                token.ThrowIfCancellationRequested(); hash.TransformFinalBlock(Array.Empty<byte>(), 0, 0);
                return BitConverter.ToString(hash.Hash).Replace("-", "").ToLowerInvariant();
            }
        }

        private VisionModelProfile ReadDraft()
        {
            var profile = VisionModelProfile.Builtin();
            profile.Name = _name.Text.Trim(); profile.Model = _model.Text; profile.Sha256 = _hash.Text;
            if (!int.TryParse(_size.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out int size) || size < 160 || size > 1280)
            { throw new FormatException("Enter an input size from 160 to 1280."); }
            profile.InputSize = size;
            if (!_classes.CommitEdit(Wpf.DataGridEditingUnit.Cell, true) || !_classes.CommitEdit(Wpf.DataGridEditingUnit.Row, true))
            { throw new FormatException("Finish editing the class table."); }
            profile.Labels = _rows.Select(row => (row.Label ?? "").Trim()).ToArray();
            profile.AllowedClasses = _rows.Select((row, index) => new { row, index }).Where(item => item.row.Detect)
                .Select(item => item.index).ToArray();
            profile.ScoreThreshold = (float)Math.Round(_score.Value, 2);
            if (profile.AllowedClasses.Length == 0) { throw new FormatException("Select at least one class to detect."); }
            profile.NmsThreshold = Number(_nms, "NMS overlap", .001f, .999f);
            profile.Validate();
            if (!Path.IsPathRooted(profile.Model)) { throw new FormatException("Choose the model weights."); }
            return profile;
        }

        private static float Number(Wpf.TextBox field, string name, float min, float max)
        {
            if (!float.TryParse(field.Text, NumberStyles.Float, CultureInfo.CurrentCulture, out float value) ||
                float.IsNaN(value) || float.IsInfinity(value) || value < min || value > max)
            { throw new FormatException("Enter a valid " + name + "."); }
            return value;
        }

        private void SetBusy(bool busy)
        {
            _hashing = busy;
            foreach (System.Windows.UIElement control in new System.Windows.UIElement[] { _save, _browse, _test, _name, _model, _hash,
                _size, _classes, _classActions, _score, _nms }) { control.IsEnabled = !busy; }
        }

        private async Task TestModel()
        {
            if (_hashing) { return; }
            try
            {
                var profile = ReadDraft();
                SetBusy(true);
                var token = _shutdown.Token;
                await Task.Run(() => VisionDetectorFactory.ValidateModel(profile, token));
                if (!IsDisposed && !token.IsCancellationRequested)
                { MessageBox.Show(this, "Model loads with the configured YOLOX input and output tensors.", "Model test passed"); }
            }
            catch (OperationCanceledException) { }
            catch (Exception error) when (!_shutdown.IsCancellationRequested && !(error is OutOfMemoryException))
            { MessageBox.Show(this, error.Message, "Model test failed", MessageBoxButtons.OK, MessageBoxIcon.Warning); }
            finally
            {
                if (!IsDisposed) { SetBusy(false); }
            }
        }

        private void AcceptDraft()
        {
            if (_hashing) { return; }
            try
            {
                Draft = ReadDraft(); DialogResult = DialogResult.OK; Close();
            }
            catch (Exception error) when (error is FormatException || error is OverflowException)
            { MessageBox.Show(this, error.Message, "Review model profile", MessageBoxButtons.OK, MessageBoxIcon.Warning); }
        }

        protected override void OnFormClosing(FormClosingEventArgs args)
        { _shutdown.Cancel(); base.OnFormClosing(args); }

        protected override void Dispose(bool disposing)
        {
            if (disposing) { _shutdown.Cancel(); }
            base.Dispose(disposing);
        }
    }
}
