// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    internal sealed class VisionModelEditorForm : NOMADForm
    {
        private readonly TextBox _name, _model, _hash;
        private readonly DataGridView _classes;
        private readonly FlowLayoutPanel _classActions;
        private readonly ToolTip _tips = new ToolTip { AutoPopDelay = 15000 };
        private readonly NumericUpDown _size, _score, _nms;
        private readonly Button _save, _browse, _test;
        private readonly CancellationTokenSource _shutdown = new CancellationTokenSource();
        private bool _hashing;
        internal VisionModelProfile Draft { get; private set; }

        internal VisionModelEditorForm(VisionModelProfile profile)
        {
            Text = "Detection model"; ClientSize = new Size(650, 570);
            MinimumSize = new Size(570, 430);
            StartPosition = FormStartPosition.CenterParent;
            var fields = new TableLayoutPanel
            { Dock = DockStyle.Fill, AutoScroll = true, ColumnCount = 2, Padding = new Padding(12) };
            fields.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 155));
            fields.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            Controls.Add(fields);
            _name = TextField(fields, "Name", profile.Name);
            _model = TextField(fields, "ONNX weights", profile.Model); _model.ReadOnly = true;
            _browse = ControlFactory.Button("Browse…", async (sender, args) => await BrowseModel());
            _tips.SetToolTip(_browse, "Choose a trusted local ONNX file, up to 256 MiB.");
            AddField(fields, "Weights", _browse);
            _hash = TextField(fields, "SHA256", profile.Sha256); _hash.ReadOnly = true;
            _size = NumberField(fields, "Input size", 160, 1280, profile.InputSize, 0); _size.Increment = 32;
            _classes = CreateClassTable(profile);
            AddField(fields, "Classes", _classes);
            _tips.SetToolTip(_classes, "Rows must match the model's output class order. Detect selects which classes to show.");
            _classActions = new FlowLayoutPanel { AutoSize = true };
            _classActions.Controls.Add(ControlFactory.Button("Add class", (sender, args) =>
            { if (_classes.Rows.Count < 256) { _classes.Rows.Add(_classes.Rows.Count, "", false); } }));
            _classActions.Controls.Add(ControlFactory.Button("Remove class", (sender, args) => RemoveClass()));
            var import = ControlFactory.Button("Import labels", (sender, args) => BrowseLabels());
            _tips.SetToolTip(import, "Replace classes with a text file, one label per line in model output order. All imported classes start selected.");
            _classActions.Controls.Add(import);
            AddField(fields, "", _classActions);
            _score = NumberField(fields, "Minimum score", .01m, 1, (decimal)profile.ScoreThreshold, 3);
            _nms = NumberField(fields, "NMS overlap", .001m, .999m, (decimal)profile.NmsThreshold, 3);
            var format = new Label { AutoSize = true, Text = "YOLOX grid (ONNX)" };
            AddField(fields, "Format", format);
            _tips.SetToolTip(format, "One float input/output. BGR NCHW 0–255, top-left letterbox, padding 114, " +
                "strides 8/16/32. Other ONNX architectures are unsupported.");
            var actions = new FlowLayoutPanel { AutoSize = true, Dock = DockStyle.Bottom, Padding = new Padding(12, 6, 12, 8) };
            _save = ControlFactory.Button("Save", (sender, args) => AcceptDraft());
            _test = ControlFactory.Button("Test model", async (sender, args) => await TestModel());
            var cancel = ControlFactory.Button("Cancel", (sender, args) => Close());
            cancel.DialogResult = DialogResult.Cancel;
            actions.Controls.AddRange(new Control[] { _test, _save, cancel });
            Controls.Add(actions); AcceptButton = _save; CancelButton = cancel;
        }

        private static DataGridView CreateClassTable(VisionModelProfile profile)
        {
            var table = new DataGridView
            {
                Height = 170, AllowUserToAddRows = false, AllowUserToDeleteRows = false,
                AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill, RowHeadersVisible = false,
                MultiSelect = false, BackgroundColor = NOMADTheme.INPUT_BG, EnableHeadersVisualStyles = false,
                DefaultCellStyle = new DataGridViewCellStyle { BackColor = NOMADTheme.INPUT_BG,
                    ForeColor = NOMADTheme.TEXT_PRIMARY, SelectionBackColor = NOMADTheme.ACCENT },
                ColumnHeadersDefaultCellStyle = new DataGridViewCellStyle { BackColor = NOMADTheme.BUTTON_BG,
                    ForeColor = NOMADTheme.TEXT_PRIMARY },
            };
            table.Columns.Add(new DataGridViewTextBoxColumn { Name = "Id", HeaderText = "ID", ReadOnly = true, FillWeight = 20 });
            table.Columns.Add(new DataGridViewTextBoxColumn { Name = "Label", HeaderText = "Class", MaxInputLength = 80 });
            table.Columns.Add(new DataGridViewCheckBoxColumn { Name = "Detect", HeaderText = "Detect", FillWeight = 30 });
            foreach (DataGridViewColumn column in table.Columns) { column.SortMode = DataGridViewColumnSortMode.NotSortable; }
            for (int index = 0; index < profile.Labels.Length; index++)
            { table.Rows.Add(index, profile.Labels[index], profile.AllowedClasses.Contains(index)); }
            return table;
        }

        private void RemoveClass()
        {
            if (_classes.CurrentRow == null) { return; }
            _classes.Rows.RemoveAt(_classes.CurrentRow.Index);
            foreach (DataGridViewRow row in _classes.Rows) { row.Cells[0].Value = row.Index; }
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
            _classes.CancelEdit();
            _classes.Rows.Clear();
            for (int index = 0; index < labels.Length; index++) { _classes.Rows.Add(index, labels[index], true); }
        }

        private static void AddField(TableLayoutPanel panel, string label, Control control)
        {
            int row = panel.RowCount++;
            panel.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            panel.Controls.Add(new Label { Text = label, AutoSize = true, Margin = new Padding(0, 7, 8, 7) }, 0, row);
            control.Dock = DockStyle.Fill; control.Margin = new Padding(0, 3, 0, 5);
            control.AccessibleName = label;
            panel.Controls.Add(control, 1, row);
        }

        private static TextBox TextField(TableLayoutPanel panel, string label, string text)
        {
            var field = new TextBox { Text = text, BackColor = NOMADTheme.INPUT_BG, ForeColor = NOMADTheme.TEXT_PRIMARY };
            AddField(panel, label, field); return field;
        }

        private static NumericUpDown NumberField(TableLayoutPanel panel, string label, decimal min, decimal max,
            decimal value, int decimals)
        {
            var field = new NumericUpDown { Minimum = min, Maximum = max, DecimalPlaces = decimals, Value = value,
                BackColor = NOMADTheme.INPUT_BG, ForeColor = NOMADTheme.TEXT_PRIMARY,
                Increment = decimals == 0 ? 1 : .01m };
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
            profile.InputSize = (int)_size.Value;
            _classes.EndEdit();
            var rows = _classes.Rows.Cast<DataGridViewRow>().ToArray();
            profile.Labels = rows.Select(row => Convert.ToString(row.Cells[1].Value).Trim()).ToArray();
            profile.AllowedClasses = rows.Where(row => Equals(row.Cells[2].Value, true)).Select(row => row.Index).ToArray();
            profile.ScoreThreshold = (float)_score.Value; profile.NmsThreshold = (float)_nms.Value;
            profile.Validate();
            if (!Path.IsPathRooted(profile.Model)) { throw new FormatException("Choose the model weights."); }
            return profile;
        }

        private void SetBusy(bool busy)
        {
            _hashing = busy;
            foreach (Control control in new Control[] { _save, _browse, _test, _name, _model, _hash,
                _size, _classes, _classActions, _score, _nms }) { control.Enabled = !busy; }
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
            if (disposing) { _shutdown.Cancel(); _tips.Dispose(); }
            base.Dispose(disposing);
        }
    }
}
