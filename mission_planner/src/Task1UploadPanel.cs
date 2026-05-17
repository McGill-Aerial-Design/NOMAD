// ============================================================
// NOMAD Task 1 Upload Panel
// ============================================================
// Submission table with image previews, approval workflow,
// orange/red highlighting for warnings, and Google Drive upload.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Net.Http;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    public class Task1UploadPanel : UserControl
    {
        private static readonly Color CARD_BG = Color.FromArgb(35, 35, 38);
        private static readonly Color TEXT_PRIMARY = Color.FromArgb(220, 220, 220);
        private static readonly Color TEXT_SECONDARY = Color.FromArgb(160, 160, 160);
        private static readonly Color ACCENT_COLOR = Color.FromArgb(0, 122, 204);
        private static readonly Color SUCCESS_COLOR = Color.FromArgb(76, 175, 80);
        private static readonly Color WARNING_COLOR = Color.FromArgb(255, 193, 7);
        private static readonly Color ERROR_COLOR = Color.FromArgb(244, 67, 54);
        private static readonly Color ORANGE_WARN = Color.FromArgb(255, 152, 0);
        private static readonly Color RED_DUPE = Color.FromArgb(244, 67, 54);
        private static readonly Color UNAPPROVED_BG = Color.FromArgb(60, 30, 30);

        private readonly NOMADConfig _config;
        private DataGridView _targetGrid;
        private Button _btnAddTarget;
        private Button _btnRemoveTarget;
        private Button _btnApprove;
        private Button _btnPreview;
        private Button _btnUpload;
        private TextBox _txtPreview;
        private Label _lblStatus;
        private ProgressBar _progressBar;
        private double _buildingHeight = 5.0;

        // 3D building viewer (below the preview area).
        private BuildingViewer3D _viewer;
        private System.Windows.Forms.Timer _viewerRefreshTimer;
        private bool _viewerRefreshInFlight;

        // Triple-click guard for the Remove button
        private int _removeClickCount;
        private System.Windows.Forms.Timer _removeClickTimer;

        // Crash-safe persistence of user edits (approve/desc/color/plane/height)
        // keyed by image path. Sits next to the per-capture JSON files so a
        // GCS crash never loses 20 minutes of approval work.
        private System.Windows.Forms.Timer _stateSaveTimer;
        private bool _restoringState;

        private static string SubmitStatePath => Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "NOMAD", "Task1", "submit_state.json");

        private class SubmitRowState
        {
            public bool Approved { get; set; }
            public string Color { get; set; }
            public string Plane { get; set; }
            public string Height { get; set; }
            public string Description { get; set; }
        }

        public double BuildingHeight
        {
            get => _buildingHeight;
            set
            {
                _buildingHeight = value;
                _viewer?.SetBuildingHeight(value);
            }
        }

        /// <summary>
        /// Current number of targets in the queue (before adding the next capture).
        /// </summary>
        public int TargetCount => _targetGrid?.Rows.Count ?? 0;

        public Task1UploadPanel(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            InitializeUI();
            // Auto-restore any previously saved captures on creation
            this.Load += (s, e) =>
            {
                RestoreFromSavedCaptures();
                StartViewerRefresh();
            };
            this.VisibleChanged += (s, e) =>
            {
                if (Visible) StartViewerRefresh();
                else StopViewerRefresh();
            };
            // Flush any pending debounced save when the panel is torn down.
            this.Disposed += (s, e) =>
            {
                try
                {
                    if (_stateSaveTimer != null && _stateSaveTimer.Enabled)
                    {
                        _stateSaveTimer.Stop();
                        SaveSubmitState();
                    }
                    _stateSaveTimer?.Dispose();
                    StopViewerRefresh();
                }
                catch { }
            };
        }

        private void RestoreFromSavedCaptures()
        {
            var task1Dir = Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
                "NOMAD", "Task1");
            if (!Directory.Exists(task1Dir)) return;

            // Skip the submit_state sidecar — it isn't a capture metadata file.
            var jsonFiles = Directory.GetFiles(task1Dir, "*.json")
                .Where(f => !string.Equals(Path.GetFileName(f), "submit_state.json", StringComparison.OrdinalIgnoreCase))
                .OrderBy(f => f).ToArray();
            if (jsonFiles.Length == 0) return;

            // Load per-row user edits keyed by image path (if any).
            Dictionary<string, SubmitRowState> savedState = LoadSubmitState();

            _restoringState = true;
            try
            {
                _targetGrid.Rows.Clear();
                int targetNum = 1;
                foreach (var jsonFile in jsonFiles)
                {
                    try
                    {
                        var json = File.ReadAllText(jsonFile);
                        var metadata = JsonConvert.DeserializeObject<SnapshotMetadata>(json);

                        var imagePath = Path.ChangeExtension(jsonFile, ".jpg");
                        if (!File.Exists(imagePath)) imagePath = Path.ChangeExtension(jsonFile, ".jpeg");
                        if (!File.Exists(imagePath)) imagePath = null;

                        // Overlay any user edits stored in submit_state.json on top
                        // of the capture-time metadata so an earlier approval session
                        // survives a GCS crash.
                        SubmitRowState saved = null;
                        if (imagePath != null)
                            savedState.TryGetValue(imagePath, out saved);

                        int rowIndex = _targetGrid.Rows.Add();
                        var row = _targetGrid.Rows[rowIndex];
                        row.Cells["Approved"].Value = saved?.Approved ?? false;
                        row.Cells["Number"].Value = targetNum++;
                        row.Cells["Color"].Value = saved?.Color ?? metadata?.TargetColor ?? "Red";
                        row.Cells["Plane"].Value = saved?.Plane ?? "wall";
                        row.Cells["Height"].Value = saved?.Height ?? "";
                        row.Cells["Description"].Value = saved?.Description ?? metadata?.RelativeDescription ?? "";
                        row.Cells["ImagePath"].Value = imagePath ?? "";
                        row.Cells["Warning"].Value = "";

                        if (imagePath != null)
                        {
                            try
                            {
                                using (var img = Image.FromFile(imagePath))
                                    row.Cells["Preview"].Value = img.GetThumbnailImage(45, 45, null, IntPtr.Zero);
                            }
                            catch { }
                        }
                        row.DefaultCellStyle.BackColor = UNAPPROVED_BG;
                    }
                    catch { }
                }
            }
            finally
            {
                _restoringState = false;
            }

            if (_targetGrid.Rows.Count > 0)
            {
                ApplyRowHighlighting();
                int approvedCount = _targetGrid.Rows.Cast<DataGridViewRow>()
                    .Count(r => (bool?)r.Cells["Approved"].Value ?? false);
                _lblStatus.Text = approvedCount > 0
                    ? $"{_targetGrid.Rows.Count} capture(s) restored ({approvedCount} pre-approved)"
                    : $"{_targetGrid.Rows.Count} capture(s) restored — approve before uploading";

                // Fire-and-forget overlay of fresh backend descriptions so the
                // operator sees regenerate-button results without reopening the
                // panel. Local user edits (saved state) still win.
                _ = OverlayBackendDescriptionsAsync(LoadSubmitState());
            }
        }

        // Pull /api/task/1/target/list_structured and replace each row's
        // Description cell with the backend-side text, unless the operator has
        // an explicit saved override. The mapping is positional: backend
        // targets are A,B,C,...; grid rows are loaded in capture order so row
        // 0 → A, row 1 → B, etc. Failures are non-fatal — we keep the
        // capture-time RelativeDescription already in the cell.
        private async Task OverlayBackendDescriptionsAsync(Dictionary<string, SubmitRowState> savedState)
        {
            try
            {
                var resp = await JetsonApiService.GetAsync("/api/task/1/target/list_structured");
                if (!resp.IsSuccessStatusCode) return;
                var body = await resp.Content.ReadAsStringAsync();
                var json = Newtonsoft.Json.Linq.JObject.Parse(body);
                var targets = json["targets"] as Newtonsoft.Json.Linq.JArray;
                if (targets == null) return;
                var byLetter = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
                foreach (var t in targets)
                {
                    var id = t["id"]?.ToString();
                    var desc = t["description"]?.ToString();
                    if (!string.IsNullOrEmpty(id) && !string.IsNullOrEmpty(desc))
                        byLetter[id] = desc;
                }
                if (byLetter.Count == 0) return;

                if (_targetGrid.IsDisposed) return;
                if (_targetGrid.InvokeRequired)
                {
                    _targetGrid.Invoke((Action)(() => ApplyBackendDescriptions(byLetter, savedState)));
                }
                else
                {
                    ApplyBackendDescriptions(byLetter, savedState);
                }
            }
            catch { /* network/parse failure: keep local descriptions */ }
        }

        private void ApplyBackendDescriptions(
            Dictionary<string, string> byLetter,
            Dictionary<string, SubmitRowState> savedState)
        {
            _restoringState = true;
            try
            {
                for (int i = 0; i < _targetGrid.Rows.Count; i++)
                {
                    var letter = IndexToTargetLetter(i);
                    if (!byLetter.TryGetValue(letter, out var desc)) continue;
                    var row = _targetGrid.Rows[i];
                    var imagePath = row.Cells["ImagePath"].Value?.ToString();
                    if (!string.IsNullOrEmpty(imagePath) && savedState.TryGetValue(imagePath, out var saved)
                        && !string.IsNullOrEmpty(saved?.Description))
                    {
                        // Operator explicitly edited this row — don't clobber.
                        continue;
                    }
                    // Strip any legacy color/Target-letter/distance crud so the
                    // cell text is the canonical color-free body the upload step
                    // will prefix with the table's Color value.
                    row.Cells["Description"].Value = NormalizeBackendDescription(desc);
                }
            }
            finally
            {
                _restoringState = false;
            }
        }

        // ============================================================
        // Submit-state persistence (crash-safe sidecar)
        // ============================================================

        private Dictionary<string, SubmitRowState> LoadSubmitState()
        {
            try
            {
                var path = SubmitStatePath;
                if (!File.Exists(path)) return new Dictionary<string, SubmitRowState>(StringComparer.OrdinalIgnoreCase);
                var raw = File.ReadAllText(path);
                if (string.IsNullOrWhiteSpace(raw))
                    return new Dictionary<string, SubmitRowState>(StringComparer.OrdinalIgnoreCase);
                var loaded = JsonConvert.DeserializeObject<Dictionary<string, SubmitRowState>>(raw);
                return loaded ?? new Dictionary<string, SubmitRowState>(StringComparer.OrdinalIgnoreCase);
            }
            catch
            {
                // Try the .bak written by the last good save.
                try
                {
                    var bak = SubmitStatePath + ".bak";
                    if (File.Exists(bak))
                    {
                        var raw = File.ReadAllText(bak);
                        var loaded = JsonConvert.DeserializeObject<Dictionary<string, SubmitRowState>>(raw);
                        if (loaded != null) return loaded;
                    }
                }
                catch { }
            }
            return new Dictionary<string, SubmitRowState>(StringComparer.OrdinalIgnoreCase);
        }

        /// <summary>
        /// Schedule a save of the current grid state. Debounced (250ms)
        /// so a flurry of edits doesn't hammer the disk.
        /// </summary>
        private void ScheduleSubmitStateSave()
        {
            if (_restoringState) return;
            if (_stateSaveTimer == null)
            {
                _stateSaveTimer = new System.Windows.Forms.Timer { Interval = 250 };
                _stateSaveTimer.Tick += (s, e) =>
                {
                    _stateSaveTimer.Stop();
                    SaveSubmitState();
                };
            }
            _stateSaveTimer.Stop();
            _stateSaveTimer.Start();
        }

        private void SaveSubmitState()
        {
            try
            {
                var dict = new Dictionary<string, SubmitRowState>(StringComparer.OrdinalIgnoreCase);
                foreach (DataGridViewRow row in _targetGrid.Rows)
                {
                    if (row.IsNewRow) continue;
                    var imagePath = row.Cells["ImagePath"].Value?.ToString();
                    if (string.IsNullOrEmpty(imagePath)) continue;
                    dict[imagePath] = new SubmitRowState
                    {
                        Approved = (bool?)row.Cells["Approved"].Value ?? false,
                        Color = row.Cells["Color"].Value?.ToString(),
                        Plane = row.Cells["Plane"].Value?.ToString(),
                        Height = row.Cells["Height"].Value?.ToString(),
                        Description = row.Cells["Description"].Value?.ToString(),
                    };
                }

                var path = SubmitStatePath;
                var dir = Path.GetDirectoryName(path);
                if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                    Directory.CreateDirectory(dir);

                var json = JsonConvert.SerializeObject(dict, Formatting.Indented);
                var tmp = path + ".tmp";
                var bak = path + ".bak";

                File.WriteAllText(tmp, json);
                if (File.Exists(path))
                    File.Replace(tmp, path, bak, ignoreMetadataErrors: true);
                else
                    File.Move(tmp, path);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Submit state save failed: {ex.Message}");
            }
        }

        private void InitializeUI()
        {
            this.BackColor = CARD_BG;
            this.Padding = new Padding(0);
            this.AutoScroll = true;

            // Fixed-height inner panel — scrolls when window is too short
            const int GRID_H    = 210;  // target table (~4 rows)
            const int BTN_H     = 42;
            const int PREVIEW_H = 110;
            const int VIEWER_H  = 220;
            const int STATUS_H  = 36;
            const int TITLE_H   = 32;
            const int TOTAL_H   = TITLE_H + GRID_H + BTN_H + PREVIEW_H + VIEWER_H + STATUS_H + 20;

            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                ColumnCount = 1,
                RowCount = 6,
                Height = TOTAL_H,
                Padding = new Padding(8),
            };
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, TITLE_H));    // 0: title
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, GRID_H));     // 1: grid
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, BTN_H));      // 2: buttons
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, PREVIEW_H));  // 3: preview text
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, VIEWER_H));   // 4: 3D viewer
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, STATUS_H));   // 5: status

            // Stretch inner panel width with the scroll container
            this.Resize += (s, e) => mainLayout.Width = this.ClientSize.Width;

            var lblTitle = new Label
            {
                Text = "TASK 1 SUBMISSION",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                AutoSize = true,
                Padding = new Padding(0, 4, 0, 0),
            };
            mainLayout.Controls.Add(lblTitle, 0, 0);

            _targetGrid = new DataGridView
            {
                Dock = DockStyle.Fill,
                BackgroundColor = Color.FromArgb(25, 25, 28),
                ForeColor = TEXT_PRIMARY,
                GridColor = Color.FromArgb(60, 60, 63),
                BorderStyle = BorderStyle.FixedSingle,
                CellBorderStyle = DataGridViewCellBorderStyle.SingleHorizontal,
                ColumnHeadersBorderStyle = DataGridViewHeaderBorderStyle.Single,
                EnableHeadersVisualStyles = false,
                RowHeadersVisible = false,
                AllowUserToAddRows = false,
                AllowUserToDeleteRows = false,
                SelectionMode = DataGridViewSelectionMode.FullRowSelect,
                MultiSelect = true,
                AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill,
                EditMode = DataGridViewEditMode.EditOnEnter,
                RowTemplate = { Height = 50 },
            };

            _targetGrid.ColumnHeadersDefaultCellStyle.BackColor = Color.FromArgb(45, 45, 48);
            _targetGrid.ColumnHeadersDefaultCellStyle.ForeColor = TEXT_PRIMARY;
            _targetGrid.ColumnHeadersDefaultCellStyle.Font = new Font("Segoe UI", 9, FontStyle.Bold);
            _targetGrid.ColumnHeadersDefaultCellStyle.SelectionBackColor = Color.FromArgb(45, 45, 48);
            _targetGrid.ColumnHeadersHeight = 30;

            _targetGrid.DefaultCellStyle.BackColor = Color.FromArgb(30, 30, 33);
            _targetGrid.DefaultCellStyle.ForeColor = TEXT_PRIMARY;
            _targetGrid.DefaultCellStyle.SelectionBackColor = Color.FromArgb(0, 100, 180);
            _targetGrid.DefaultCellStyle.SelectionForeColor = Color.White;
            _targetGrid.AlternatingRowsDefaultCellStyle.BackColor = Color.FromArgb(35, 35, 38);

            var colApproved = new DataGridViewCheckBoxColumn
            {
                Name = "Approved",
                HeaderText = "\u2713",
                Width = 35,
                FalseValue = false,
                TrueValue = true,
            };
            _targetGrid.Columns.Add(colApproved);

            var colNumber = new DataGridViewTextBoxColumn
            {
                Name = "Number",
                HeaderText = "#",
                Width = 35,
                ReadOnly = true,
            };
            _targetGrid.Columns.Add(colNumber);

            var colPreview = new DataGridViewImageColumn
            {
                Name = "Preview",
                HeaderText = "Image",
                Width = 55,
                ImageLayout = DataGridViewImageCellLayout.Zoom,
            };
            _targetGrid.Columns.Add(colPreview);

            var colColor = new DataGridViewComboBoxColumn
            {
                Name = "Color",
                HeaderText = "Color",
                Width = 75,
                FlatStyle = FlatStyle.Flat,
            };
            colColor.Items.AddRange("Red", "Blue", "Green", "Yellow", "Orange", "Purple", "White", "Black", "Unknown");
            _targetGrid.Columns.Add(colColor);

            var colPlane = new DataGridViewComboBoxColumn
            {
                Name = "Plane",
                HeaderText = "Plane",
                Width = 70,
                FlatStyle = FlatStyle.Flat,
            };
            colPlane.Items.AddRange("wall", "ground", "roof");
            _targetGrid.Columns.Add(colPlane);

            var colHeight = new DataGridViewTextBoxColumn
            {
                Name = "Height",
                HeaderText = "H(AGL)",
                Width = 55,
                ReadOnly = true,
            };
            _targetGrid.Columns.Add(colHeight);

            var colDescription = new DataGridViewTextBoxColumn
            {
                Name = "Description",
                HeaderText = "Description (ConOps format)",
                FillWeight = 100,
            };
            _targetGrid.Columns.Add(colDescription);

            var colImagePath = new DataGridViewTextBoxColumn
            {
                Name = "ImagePath",
                HeaderText = "Path",
                Width = 100,
                ReadOnly = true,
                Visible = false,
            };
            _targetGrid.Columns.Add(colImagePath);

            var colWarning = new DataGridViewTextBoxColumn
            {
                Name = "Warning",
                HeaderText = "!",
                Width = 25,
                ReadOnly = true,
            };
            _targetGrid.Columns.Add(colWarning);

            _targetGrid.CellClick += TargetGrid_CellClick;
            _targetGrid.CellEndEdit += TargetGrid_CellEndEdit;
            _targetGrid.CellValueChanged += TargetGrid_CellValueChanged;
            _targetGrid.CellMouseEnter += (s, e) =>
            {
                if (e.RowIndex < 0 || _viewer == null) return;
                _viewer.SetHighlightedTarget(TargetIdForRow(e.RowIndex));
            };
            _targetGrid.MouseLeave += (s, e) => _viewer?.SetHighlightedTarget(null);
            _targetGrid.RowsAdded += (s, e) => { ApplyRowHighlighting(); ScheduleSubmitStateSave(); };
            _targetGrid.RowsRemoved += (s, e) => ScheduleSubmitStateSave();
            _targetGrid.DataBindingComplete += (s, e) => ApplyRowHighlighting();

            mainLayout.Controls.Add(_targetGrid, 0, 1);

            var buttonPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.LeftToRight,
                AutoSize = true,
                Padding = new Padding(0, 5, 0, 5),
            };

            _btnAddTarget = CreateButton("+ Add", SUCCESS_COLOR, 55, 28);
            _btnAddTarget.Click += BtnAddTarget_Click;
            buttonPanel.Controls.Add(_btnAddTarget);

            _btnRemoveTarget = CreateButton("- Remove", ERROR_COLOR, 80, 28);
            _btnRemoveTarget.Click += BtnRemoveTarget_Click;
            buttonPanel.Controls.Add(_btnRemoveTarget);

            _btnApprove = CreateButton("\u2713 Approve", SUCCESS_COLOR, 85, 28);
            _btnApprove.Click += BtnApprove_Click;
            buttonPanel.Controls.Add(_btnApprove);

            buttonPanel.Controls.Add(new Panel { Width = 20, Height = 28 });

            _btnPreview = CreateButton("Preview TXT", Color.FromArgb(80, 80, 83), 100, 28);
            _btnPreview.Click += BtnPreview_Click;
            buttonPanel.Controls.Add(_btnPreview);

            _btnUpload = CreateButton("Upload to Google Drive", ACCENT_COLOR, 160, 28);
            _btnUpload.Click += BtnUpload_Click;
            buttonPanel.Controls.Add(_btnUpload);

            mainLayout.Controls.Add(buttonPanel, 0, 2);

            _txtPreview = new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Vertical,
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = TEXT_SECONDARY,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.FixedSingle,
                Text = "Click 'Preview TXT' to see the submission file content...\n\nOnly approved targets (checked) will be included.",
            };
            mainLayout.Controls.Add(_txtPreview, 0, 3);

            // ---- 3D building viewer ----
            var viewerHost = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(20, 20, 22),
                BorderStyle = BorderStyle.FixedSingle,
                Padding = new Padding(0),
            };
            var viewerHeader = new Label
            {
                Text = "3D BUILDING MODEL  —  drag = orbit, wheel = zoom, right-drag = pan",
                Dock = DockStyle.Top,
                Height = 18,
                ForeColor = ACCENT_COLOR,
                BackColor = Color.FromArgb(30, 30, 33),
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleLeft,
                Padding = new Padding(6, 0, 0, 0),
            };
            _viewer = new BuildingViewer3D();
            _viewer.Dock = DockStyle.Fill;
            _viewer.TargetHovered += OnViewerTargetHovered;
            viewerHost.Controls.Add(_viewer);
            viewerHost.Controls.Add(viewerHeader);
            mainLayout.Controls.Add(viewerHost, 0, 4);

            var statusPanel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                Padding = new Padding(0, 5, 0, 0),
            };
            statusPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            statusPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 150));

            _lblStatus = new Label
            {
                Text = "Ready",
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                Padding = new Padding(0, 5, 0, 0),
            };
            statusPanel.Controls.Add(_lblStatus, 0, 0);

            _progressBar = new ProgressBar
            {
                Dock = DockStyle.Fill,
                Style = ProgressBarStyle.Continuous,
                Visible = false,
            };
            statusPanel.Controls.Add(_progressBar, 1, 0);

            mainLayout.Controls.Add(statusPanel, 0, 5);

            this.Controls.Add(mainLayout);
        }

        private Button CreateButton(string text, Color backColor, int width, int height)
        {
            return new Button
            {
                Text = text,
                Size = new Size(width, height),
                FlatStyle = FlatStyle.Flat,
                BackColor = backColor,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
                Cursor = Cursors.Hand,
                Margin = new Padding(3),
            };
        }

        private void BtnAddTarget_Click(object sender, EventArgs e)
        {
            int nextNumber = _targetGrid.Rows.Count + 1;
            int rowIndex = _targetGrid.Rows.Add();
            var row = _targetGrid.Rows[rowIndex];
            row.Cells["Approved"].Value = false;
            row.Cells["Number"].Value = nextNumber;
            row.Cells["Color"].Value = "Red";
            row.Cells["Plane"].Value = "wall";
            row.Cells["Height"].Value = "";
            row.Cells["Description"].Value = "";
            row.Cells["ImagePath"].Value = "";
            row.Cells["Warning"].Value = "";
            row.DefaultCellStyle.BackColor = UNAPPROVED_BG;

            _targetGrid.ClearSelection();
            row.Selected = true;
            _targetGrid.CurrentCell = row.Cells["Description"];
        }

        private void BtnRemoveTarget_Click(object sender, EventArgs e)
        {
            if (_targetGrid.SelectedRows.Count == 0) return;

            // Require three clicks within 1.5 s to prevent accidental deletion.
            _removeClickCount++;

            if (_removeClickTimer == null)
            {
                _removeClickTimer = new System.Windows.Forms.Timer { Interval = 1500 };
                _removeClickTimer.Tick += (s, _) =>
                {
                    _removeClickTimer.Stop();
                    _removeClickCount = 0;
                    _btnRemoveTarget.Text = "- Remove";
                    _lblStatus.Text = "Remove cancelled (triple-click timeout).";
                    _lblStatus.ForeColor = TEXT_SECONDARY;
                };
            }

            if (_removeClickCount == 1)
            {
                _removeClickTimer.Stop();
                _removeClickTimer.Start();
                _btnRemoveTarget.Text = "- Remove (2 more)";
                _lblStatus.Text = "Click 2 more times within 1.5 s to delete selected target(s).";
                _lblStatus.ForeColor = WARNING_COLOR;
                return;
            }
            if (_removeClickCount == 2)
            {
                _btnRemoveTarget.Text = "- Remove (1 more)";
                return;
            }

            // Third click — actually delete.
            _removeClickTimer.Stop();
            _removeClickCount = 0;
            _btnRemoveTarget.Text = "- Remove";
            _ = DeleteSelectedTargetsAsync();
        }

        private async Task DeleteSelectedTargetsAsync()
        {
            var rows = _targetGrid.SelectedRows.Cast<DataGridViewRow>()
                .Where(r => !r.IsNewRow)
                .OrderByDescending(r => r.Index)
                .ToList();

            if (rows.Count == 0) return;

            _btnRemoveTarget.Enabled = false;
            _lblStatus.ForeColor = WARNING_COLOR;

            // Collect target letters before removing rows (letters = row position A, B, C...)
            // Must be computed from current order before any removal.
            var allRows = _targetGrid.Rows.Cast<DataGridViewRow>().Where(r => !r.IsNewRow).ToList();
            var deletions = rows.Select(row => new
            {
                Row = row,
                Letter = IndexToTargetLetter(allRows.IndexOf(row)),
                ImagePath = row.Cells["ImagePath"].Value?.ToString() ?? "",
            }).ToList();

            foreach (var d in deletions)
            {
                _lblStatus.Text = $"Deleting Target {d.Letter}...";

                // 1. Delete local GCS files
                var imgPath = d.ImagePath;
                if (!string.IsNullOrEmpty(imgPath) && File.Exists(imgPath))
                {
                    try { File.Delete(imgPath); } catch { }
                    var jsonPath = Path.ChangeExtension(imgPath, ".json");
                    if (File.Exists(jsonPath)) try { File.Delete(jsonPath); } catch { }
                }

                // 2. Tell Jetson to remove target + capture folder
                try
                {
                    var resp = await JetsonApiService.DeleteAsync($"/api/task/1/target/{d.Letter}");
                    if (!resp.IsSuccessStatusCode)
                    {
                        var body = await resp.Content.ReadAsStringAsync();
                        _lblStatus.Text = $"Jetson delete failed for {d.Letter}: HTTP {(int)resp.StatusCode} — {body}";
                        _lblStatus.ForeColor = ERROR_COLOR;
                    }
                }
                catch (Exception ex)
                {
                    _lblStatus.Text = $"Jetson unreachable during delete: {ex.Message}";
                    _lblStatus.ForeColor = WARNING_COLOR;
                    // Still remove locally even if Jetson call fails.
                }

                // 3. Remove from grid
                if (_targetGrid.Rows.Contains(d.Row) && !d.Row.IsNewRow)
                    _targetGrid.Rows.Remove(d.Row);
            }

            RenumberTargets();
            SaveSubmitState();
            _btnRemoveTarget.Enabled = true;
            _lblStatus.Text = $"Deleted {deletions.Count} target(s). Local files and Jetson captures removed.";
            _lblStatus.ForeColor = SUCCESS_COLOR;
        }

        private void BtnApprove_Click(object sender, EventArgs e)
        {
            foreach (DataGridViewRow row in _targetGrid.SelectedRows)
            {
                bool current = (bool?)row.Cells["Approved"].Value ?? false;
                row.Cells["Approved"].Value = !current;
            }
            ApplyRowHighlighting();
        }

        private void RenumberTargets()
        {
            for (int i = 0; i < _targetGrid.Rows.Count; i++)
                _targetGrid.Rows[i].Cells["Number"].Value = i + 1;
        }

        public void AddCapturedImage(string imagePath, string suggestedDescription = null,
            string color = "Red", string plane = "wall", string heightAgl = "")
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke(new Action(() => AddCapturedImage(imagePath, suggestedDescription, color, plane, heightAgl)));
                return;
            }

            int nextNumber = _targetGrid.Rows.Count + 1;
            int rowIndex = _targetGrid.Rows.Add();
            var row = _targetGrid.Rows[rowIndex];
            row.Cells["Approved"].Value = false;
            row.Cells["Number"].Value = nextNumber;
            row.Cells["Color"].Value = color;
            row.Cells["Plane"].Value = plane;
            row.Cells["Height"].Value = heightAgl;
            row.Cells["Description"].Value = suggestedDescription ?? "";
            row.Cells["ImagePath"].Value = imagePath ?? "";
            row.Cells["Warning"].Value = "";

            if (!string.IsNullOrEmpty(imagePath) && File.Exists(imagePath))
            {
                try
                {
                    using (var img = Image.FromFile(imagePath))
                    {
                        row.Cells["Preview"].Value = img.GetThumbnailImage(45, 45, null, IntPtr.Zero);
                    }
                }
                catch { }
            }

            row.DefaultCellStyle.BackColor = UNAPPROVED_BG;

            _targetGrid.ClearSelection();
            row.Selected = true;
            _targetGrid.CurrentCell = row.Cells["Description"];
        }

        private void ApplyRowHighlighting()
        {
            foreach (DataGridViewRow row in _targetGrid.Rows)
            {
                if (row.IsNewRow) continue;

                bool approved = (bool?)row.Cells["Approved"].Value ?? false;
                var heightStr = row.Cells["Height"].Value?.ToString() ?? "";
                double heightVal = 0;
                double.TryParse(heightStr, out heightVal);
                var warning = row.Cells["Warning"].Value?.ToString() ?? "";

                if (!approved)
                {
                    row.DefaultCellStyle.BackColor = UNAPPROVED_BG;
                }
                else if (heightVal > _buildingHeight && _buildingHeight > 0)
                {
                    row.DefaultCellStyle.BackColor = Color.FromArgb(60, 40, 15);
                    row.Cells["Warning"].Value = "\u26A0";
                }
                else if (!string.IsNullOrEmpty(warning) && warning.Contains("dup"))
                {
                    row.DefaultCellStyle.BackColor = Color.FromArgb(60, 20, 20);
                }
                else
                {
                    row.DefaultCellStyle.BackColor = row.Index % 2 == 0
                        ? Color.FromArgb(30, 30, 33)
                        : Color.FromArgb(35, 35, 38);
                }
            }
        }

        private void TargetGrid_CellClick(object sender, DataGridViewCellEventArgs e)
        {
            if (e.RowIndex < 0) return;
        }

        private void TargetGrid_CellEndEdit(object sender, DataGridViewCellEventArgs e)
        {
            if (_targetGrid.Columns[e.ColumnIndex].Name == "Color")
            {
                var descCell = _targetGrid.Rows[e.RowIndex].Cells["Description"];
                var colorCell = _targetGrid.Rows[e.RowIndex].Cells["Color"];
                if (string.IsNullOrEmpty(descCell.Value?.ToString()) && colorCell.Value != null)
                {
                    descCell.Value = $"{colorCell.Value} target on the [face] of the building, [X]m above ground, [Y]m from [landmark].";
                }
            }
        }

        private void TargetGrid_CellValueChanged(object sender, DataGridViewCellEventArgs e)
        {
            if (e.RowIndex < 0) return;
            var colName = _targetGrid.Columns[e.ColumnIndex].Name;
            if (colName == "Approved")
                ApplyRowHighlighting();
            else if (colName == "Plane")
                _ = SendPlaneOverrideAsync(e.RowIndex);

            // Persist any user-visible field change so a crash never loses
            // approval/edit work. Image-path/preview/warning are derived,
            // not user-edited, so skip them.
            if (colName == "Approved" || colName == "Color" || colName == "Plane"
                || colName == "Height" || colName == "Description")
            {
                ScheduleSubmitStateSave();
            }
        }

        private async Task SendPlaneOverrideAsync(int rowIndex)
        {
            if (rowIndex < 0 || rowIndex >= _targetGrid.Rows.Count) return;
            var row = _targetGrid.Rows[rowIndex];
            var plane = row.Cells["Plane"].Value?.ToString();
            if (string.IsNullOrEmpty(plane)) return;

            // Target letters are assigned in capture order: row 0 = A, row 1 = B, ...
            string targetId = IndexToTargetLetter(rowIndex);
            try
            {
                _lblStatus.Text = $"Sending plane override for target {targetId} ({plane})...";
                _lblStatus.ForeColor = TEXT_SECONDARY;
                var json = JsonConvert.SerializeObject(new { plane_kind = plane });
                var content = new StringContent(json, Encoding.UTF8, "application/json");
                var resp = await JetsonApiService.PostLongRunAsync(
                    $"/api/task/1/target/{targetId}/plane_override", content);
                if (!resp.IsSuccessStatusCode)
                    throw new Exception($"HTTP {(int)resp.StatusCode}");
                _lblStatus.Text = $"Target {targetId} plane set to {plane}";
                _lblStatus.ForeColor = SUCCESS_COLOR;
            }
            catch (Exception ex)
            {
                _lblStatus.Text = $"Plane override failed: {ex.Message}";
                _lblStatus.ForeColor = ERROR_COLOR;
            }
        }

        private void BtnPreview_Click(object sender, EventArgs e)
        {
            var content = GenerateTxtContent();
            _txtPreview.Text = string.IsNullOrEmpty(content)
                ? "No approved targets. Check the Approved column to include targets."
                : content;
        }

        private string GenerateTxtContent()
        {
            var lines = new List<string>();
            int letterIndex = 0;

            foreach (DataGridViewRow row in _targetGrid.Rows)
            {
                bool approved = (bool?)row.Cells["Approved"].Value ?? false;
                if (!approved) continue;

                string letter = IndexToTargetLetter(letterIndex++);
                var color = row.Cells["Color"].Value?.ToString() ?? "";
                var description = row.Cells["Description"].Value?.ToString() ?? "";

                if (!string.IsNullOrEmpty(description))
                {
                    // Backend returns the color-free spatial body, e.g.
                    //   "on the south face of the building, 1.5m above ground, 1.7m from the SE corner."
                    // The table's Color column is the single source of truth for target color.
                    // NormalizeBackendDescription only strips legacy capture text saved before
                    // the backend was changed (color word, "Target X:" prefix, [distance=...] tag).
                    var body = NormalizeBackendDescription(description);
                    var fullDesc = string.IsNullOrEmpty(color)
                        ? body
                        : $"{color} target {body}".TrimEnd();
                    lines.Add($"Target {letter}: {fullDesc}");
                }
            }

            return string.Join("\n\n", lines);
        }

        // Legacy stripper for descriptions saved before the backend dropped
        // color from _generate_description. Removes (in order):
        //   - leading "Target X:" / "Target X -" prefix
        //   - leading "<color> target" or "Unknown target" phrase
        //   - trailing "[distance=NNcm]" / "[center_distance=NNcm]" tag
        // Color words use a closed vocabulary so freeform operator text like
        // "near red door" isn't accidentally stripped.
        private static readonly System.Text.RegularExpressions.Regex _backendPrefixRegex =
            new System.Text.RegularExpressions.Regex(
                @"^\s*(?:Target\s+[A-Za-z]+\s*[:\-]\s*)?" +
                @"(?:(?:red|blue|green|yellow|orange|purple|pink|black|white|brown|gray|grey|unknown)" +
                @"\s+target\s*(?:-\s*)?)?",
                System.Text.RegularExpressions.RegexOptions.IgnoreCase);

        private static readonly System.Text.RegularExpressions.Regex _distanceTagRegex =
            new System.Text.RegularExpressions.Regex(
                @"\s*\[(?:center_)?distance=\d+(?:\.\d+)?\s*cm\]\s*$",
                System.Text.RegularExpressions.RegexOptions.IgnoreCase);

        private static string NormalizeBackendDescription(string raw)
        {
            if (string.IsNullOrWhiteSpace(raw)) return string.Empty;
            var s = _backendPrefixRegex.Replace(raw.Trim(), string.Empty).Trim();
            s = _distanceTagRegex.Replace(s, string.Empty).Trim();
            return s;
        }

        private void BtnUpload_Click(object sender, EventArgs e)
        {
            UiAsync.Run(this, () => BtnUploadAsync(sender, e), nameof(BtnUpload_Click));
        }

        private async Task BtnUploadAsync(object sender, EventArgs e)
        {
            int approvedCount = 0;
            foreach (DataGridViewRow row in _targetGrid.Rows)
            {
                if ((bool?)row.Cells["Approved"].Value ?? false)
                    approvedCount++;
            }

            if (approvedCount == 0)
            {
                MessageBox.Show("No approved targets to upload. Check the Approved column first.",
                    "No Approved Targets", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            int checkIdx = 0;
            foreach (DataGridViewRow row in _targetGrid.Rows)
            {
                if (!((bool?)row.Cells["Approved"].Value ?? false)) continue;
                string checkLetter = IndexToTargetLetter(checkIdx++);
                var desc = row.Cells["Description"].Value?.ToString();
                if (string.IsNullOrWhiteSpace(desc))
                {
                    MessageBox.Show($"Target {checkLetter} has no description.",
                        "Missing Description", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
            }

            var result = MessageBox.Show(
                $"Upload {approvedCount} approved target(s) to Google Drive?\n\nUnapproved targets will NOT be included.",
                "Confirm Upload", MessageBoxButtons.YesNo, MessageBoxIcon.Question);

            if (result != DialogResult.Yes) return;

            await UploadToGoogleDrive();
        }

        private async Task UploadToGoogleDrive()
        {
            _btnUpload.Enabled = false;
            _progressBar.Visible = true;
            _progressBar.Style = ProgressBarStyle.Marquee;
            _lblStatus.Text = "Preparing submission...";
            _lblStatus.ForeColor = WARNING_COLOR;

            try
            {
                var gdrive = new GoogleDriveUploadService();
                var tokenDiag = gdrive.DiagnoseToken();
                if (tokenDiag != null)
                    throw new Exception($"Google Drive token problem:\n\n{tokenDiag}");


                var txtContent = GenerateTxtContent();
                if (string.IsNullOrEmpty(txtContent))
                    throw new Exception("No approved targets to upload.");

                var tempTxtPath = Path.Combine(Path.GetTempPath(), "Task_1_MAD_targets.txt");
                try
                {
                    File.WriteAllText(tempTxtPath, txtContent);

                    _lblStatus.Text = "Uploading targets file...";
                    var txtFileId = await gdrive.UploadFileAsync(tempTxtPath, "Task_1_MAD_targets.txt");
                    if (string.IsNullOrEmpty(txtFileId))
                        throw new Exception("Failed to upload Task_1_MAD_targets.txt to Google Drive.");

                    var imageResults = new List<(string letter, string filename, string fileId)>();
                    var errors = new List<string>();
                    int imgIdx = 0;

                    foreach (DataGridViewRow row in _targetGrid.Rows)
                    {
                        if (!((bool?)row.Cells["Approved"].Value ?? false)) continue;

                        string letter = IndexToTargetLetter(imgIdx++);
                        var imagePath = row.Cells["ImagePath"].Value?.ToString() ?? "";

                        if (!string.IsNullOrEmpty(imagePath) && File.Exists(imagePath))
                        {
                            _lblStatus.Text = $"Uploading Target {letter} image...";
                            var ext = Path.GetExtension(imagePath) ?? ".jpg";
                            var filename = $"Target_{letter}{ext}";

                            try
                            {
                                var fileId = await gdrive.UploadFileAsync(imagePath, filename);
                                imageResults.Add((letter, filename, fileId));
                            }
                            catch (Exception imgEx)
                            {
                                errors.Add($"Target {letter} image failed: {imgEx.Message}");
                            }
                        }
                    }

                    _lblStatus.Text = "Upload complete!";
                    _lblStatus.ForeColor = SUCCESS_COLOR;

                    var sb = new StringBuilder();
                    sb.AppendLine("=== UPLOAD SUCCESSFUL ===");
                    sb.AppendLine();
                    sb.AppendLine($"Task_1_MAD_targets.txt uploaded (ID: {txtFileId})");
                    sb.AppendLine();

                    if (imageResults.Count > 0)
                    {
                        sb.AppendLine("Images uploaded:");
                        foreach (var img in imageResults)
                        {
                            sb.AppendLine($" - {img.filename} (ID: {img.fileId})");
                        }
                    }

                    if (errors.Count > 0)
                    {
                        sb.AppendLine();
                        sb.AppendLine("Warnings:");
                        foreach (var err in errors)
                        {
                            sb.AppendLine($" - {err}");
                        }
                    }

                    _txtPreview.Text = sb.ToString();
                    _txtPreview.ForeColor = SUCCESS_COLOR;

                    MessageBox.Show(
                        $"Successfully uploaded Task 1 submission!\n\nText file ID: {txtFileId}\nImages uploaded: {imageResults.Count}",
                        "Upload Complete", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                finally
                {
                    if (File.Exists(tempTxtPath))
                        File.Delete(tempTxtPath);
                }
            }
            catch (Exception ex)
            {
                _lblStatus.Text = "Upload failed!";
                _lblStatus.ForeColor = ERROR_COLOR;
                _txtPreview.Text = $"=== UPLOAD FAILED ===\n\n{ex.Message}";
                _txtPreview.ForeColor = ERROR_COLOR;

                MessageBox.Show(
                    $"Failed to upload to Google Drive:\n\n{ex.Message}",
                    "Upload Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                _btnUpload.Enabled = true;
                _progressBar.Visible = false;
            }
        }

        public void LoadFromCaptures()
        {
            var task1Dir = Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
                "NOMAD", "Task1"
            );

            if (!Directory.Exists(task1Dir))
            {
                MessageBox.Show("No Task 1 captures found.", "No Captures",
                    MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            var jsonFiles = Directory.GetFiles(task1Dir, "*.json");
            if (jsonFiles.Length == 0)
            {
                MessageBox.Show("No capture metadata found.", "No Captures",
                    MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            _targetGrid.Rows.Clear();
            int targetNum = 1;

            foreach (var jsonFile in jsonFiles.OrderBy(f => f))
            {
                try
                {
                    var json = File.ReadAllText(jsonFile);
                    var metadata = JsonConvert.DeserializeObject<SnapshotMetadata>(json);

                    var imagePath = Path.ChangeExtension(jsonFile, ".jpg");
                    if (!File.Exists(imagePath))
                        imagePath = Path.ChangeExtension(jsonFile, ".jpeg");
                    if (!File.Exists(imagePath))
                        imagePath = Path.ChangeExtension(jsonFile, ".png");

                    int rowIndex = _targetGrid.Rows.Add();
                    var row = _targetGrid.Rows[rowIndex];
                    row.Cells["Approved"].Value = false;
                    row.Cells["Number"].Value = targetNum++;
                    row.Cells["Color"].Value = metadata?.TargetColor ?? "Red";
                    row.Cells["Plane"].Value = "wall";
                    row.Cells["Height"].Value = "";
                    row.Cells["Description"].Value =
                        metadata?.RelativeDescription ?? "";
                    row.Cells["ImagePath"].Value = File.Exists(imagePath) ? imagePath : "";
                    row.Cells["Warning"].Value = "";

                    if (File.Exists(imagePath))
                    {
                        try
                        {
                            using (var img = Image.FromFile(imagePath))
                            {
                                row.Cells["Preview"].Value = img.GetThumbnailImage(45, 45, null, IntPtr.Zero);
                            }
                        }
                        catch { }
                    }

                    row.DefaultCellStyle.BackColor = UNAPPROVED_BG;
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"Failed to load {jsonFile}: {ex.Message}");
                }
            }

            ApplyRowHighlighting();
            _lblStatus.Text = $"Loaded {_targetGrid.Rows.Count} captures (unapproved)";
            _lblStatus.ForeColor = WARNING_COLOR;
        }

        // ============================================================
        // 3D viewer integration (hover + auto-refresh)
        // ============================================================

        /// <summary>
        /// Targets are assigned letters in capture order: row 0 → A, row 1 → B, ...
        /// Same convention as SendPlaneOverrideAsync, so the IDs match what the
        /// Jetson's debug file emits.
        /// </summary>
        private string TargetIdForRow(int rowIndex)
        {
            if (rowIndex < 0 || rowIndex >= _targetGrid.Rows.Count) return null;
            return IndexToTargetLetter(rowIndex);
        }

        /// <summary>
        /// Map 0->A, 25->Z, 26->AA, 27->AB, 51->AZ, 52->BA, ... Mirrors the
        /// backend's target_letter_from_index() so IDs stay consistent across
        /// the GCS table, the Jetson debug file, and the uploaded .txt.
        /// Past Z the old (char)('A' + n) produced '[', '\', ']', etc.
        /// </summary>
        internal static string IndexToTargetLetter(int index)
        {
            if (index < 0) return "?";
            string letters = string.Empty;
            int n = index;
            while (true)
            {
                letters = ((char)('A' + (n % 26))).ToString() + letters;
                n = n / 26 - 1;
                if (n < 0) break;
            }
            return letters;
        }

        private void OnViewerTargetHovered(string targetId)
        {
            if (InvokeRequired) { BeginInvoke(new Action(() => OnViewerTargetHovered(targetId))); return; }
            if (string.IsNullOrEmpty(targetId))
            {
                _targetGrid.ClearSelection();
                return;
            }
            int idx = targetId[0] - 'A';
            if (idx < 0 || idx >= _targetGrid.Rows.Count) return;
            _targetGrid.ClearSelection();
            _targetGrid.Rows[idx].Selected = true;
            try { _targetGrid.FirstDisplayedScrollingRowIndex = Math.Max(0, idx - 2); } catch { }
        }

        private void StartViewerRefresh()
        {
            if (_viewerRefreshTimer != null) return;
            _viewerRefreshTimer = new System.Windows.Forms.Timer { Interval = 3000 };
            _viewerRefreshTimer.Tick += (s, e) => _ = RefreshViewerDataAsync();
            _viewerRefreshTimer.Start();
            _ = RefreshViewerDataAsync(); // Kick off an immediate refresh.
        }

        private void StopViewerRefresh()
        {
            if (_viewerRefreshTimer == null) return;
            _viewerRefreshTimer.Stop();
            _viewerRefreshTimer.Dispose();
            _viewerRefreshTimer = null;
        }

        private async Task RefreshViewerDataAsync()
        {
            if (_viewerRefreshInFlight || _viewer == null) return;
            _viewerRefreshInFlight = true;
            try
            {
                // 1. Building corners + height.
                var cornersResp = await JetsonApiService.GetAsync("/api/task/1/building/corners");
                if (cornersResp.IsSuccessStatusCode)
                {
                    var body = await cornersResp.Content.ReadAsStringAsync();
                    var data = Newtonsoft.Json.Linq.JObject.Parse(body);
                    var arr = data["corners"] as Newtonsoft.Json.Linq.JArray;

                    var corners = new List<BuildingViewer3D.Corner>();
                    if (arr != null)
                    {
                        foreach (var c in arr)
                        {
                            corners.Add(new BuildingViewer3D.Corner
                            {
                                Name = c["name"]?.ToString() ?? "?",
                                Lat = (double?)c["lat"] ?? 0,
                                Lon = (double?)c["lon"] ?? 0,
                            });
                        }
                    }
                    double? centerLat = (double?)data["center_lat"];
                    double? centerLon = (double?)data["center_lon"];
                    double? heightM = (double?)data["height"];
                    if (heightM.HasValue) _viewer.SetBuildingHeight(heightM.Value);
                    _viewer.SetCorners(corners, centerLat, centerLon);
                }

                // 2. Captured targets.
                var targetsResp = await JetsonApiService.GetAsync("/api/task/1/target/list_structured");
                if (targetsResp.IsSuccessStatusCode)
                {
                    var body = await targetsResp.Content.ReadAsStringAsync();
                    var data = Newtonsoft.Json.Linq.JObject.Parse(body);
                    var arr = data["targets"] as Newtonsoft.Json.Linq.JArray;

                    var targets = new List<BuildingViewer3D.Target>();
                    if (arr != null)
                    {
                        foreach (var t in arr)
                        {
                            // Only plot targets the API gave us a 3D position for.
                            var east = (float?)t["east"];
                            var north = (float?)t["north"];
                            var up = (float?)t["up"];
                            if (!east.HasValue || !north.HasValue) continue;
                            targets.Add(new BuildingViewer3D.Target
                            {
                                Id = t["id"]?.ToString() ?? "?",
                                Color = t["color"]?.ToString(),
                                Description = t["description"]?.ToString(),
                                East = east.Value,
                                North = north.Value,
                                Up = up ?? 0f,
                            });
                        }
                    }
                    _viewer.SetTargets(targets);
                }
            }
            catch
            {
                // Silently ignore — Jetson may be unreachable. The viewer keeps
                // its previous data so the user still sees the last good model.
            }
            finally
            {
                _viewerRefreshInFlight = false;
            }
        }
    }
}
