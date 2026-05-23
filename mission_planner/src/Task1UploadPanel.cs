// ============================================================
// NOMAD Task 1 Upload Panel
// ============================================================
// Submission table with image previews, approval workflow,
// orange/red highlighting for warnings, and Google Drive upload.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
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
        private Button _btnPlacementMode;
        private Button _btnGpsGround;
        private Button _btnGpsRoof;
        private TextBox _txtPreview;
        private Label _lblStatus;
        private ProgressBar _progressBar;
        private double _buildingHeight = 5.0;
        private double _groundAltM = 0.0;
        private bool _placementMode;

        // 3D building viewer (below the preview area).
        private BuildingViewer3D _viewer;
        private System.Windows.Forms.Timer _viewerRefreshTimer;
        private bool _viewerRefreshInFlight;
        private readonly List<BuildingViewer3D.Target> _lastBackendTargets = new List<BuildingViewer3D.Target>();

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

        private const double CompetitionBuildingHeightM = 2.4;

        private static (string name, double lat, double lon)[] GetCompetitionBuildingCorners()
        {
            return new[]
            {
                ("1", 45.316743567764945, -75.75773827279546),
                ("2", 45.31671371473123, -75.75759833217171),
                ("3", 45.31615424384856, -75.75781374638878),
                ("4", 45.31618520285556, -75.75796312121189),
                ("5", 45.31641794771017, -75.75787506868524),
                ("6", 45.316440061185425, -75.75798592052764),
                ("7", 45.31652353947904, -75.75795525937997),
                ("8", 45.316500873200205, -75.75784362135427),
            };
        }

        private class SubmitRowState
        {
            public bool Approved { get; set; }
            public string Color { get; set; }
            public string Plane { get; set; }
            public string Height { get; set; }
            public string Description { get; set; }
            public string StateKey { get; set; }
            public float? East { get; set; }
            public float? North { get; set; }
            public float? Up { get; set; }
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
            LoadCompetitionPresetModel();
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
            Dictionary<string, SubmitRowState> savedState = LoadSubmitState();
            if (!Directory.Exists(task1Dir))
            {
                RestoreManualTargets(savedState);
                return;
            }

            // Skip the submit_state sidecar — it isn't a capture metadata file.
            var jsonFiles = Directory.GetFiles(task1Dir, "*.json")
                .Where(f => !string.Equals(Path.GetFileName(f), "submit_state.json", StringComparison.OrdinalIgnoreCase))
                .OrderBy(f => f).ToArray();
            if (jsonFiles.Length == 0)
            {
                RestoreManualTargets(savedState);
                return;
            }

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
                        row.Cells["StateKey"].Value = imagePath ?? "";
                        row.Cells["East"].Value = saved?.East?.ToString(CultureInfo.InvariantCulture) ?? "";
                        row.Cells["North"].Value = saved?.North?.ToString(CultureInfo.InvariantCulture) ?? "";
                        row.Cells["Up"].Value = saved?.Up?.ToString(CultureInfo.InvariantCulture) ?? "";
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

            RestoreManualTargets(savedState);
        }

        private void RestoreManualTargets(Dictionary<string, SubmitRowState> savedState)
        {
            var manual = savedState
                .Where(kvp => kvp.Key.StartsWith("manual:", StringComparison.OrdinalIgnoreCase))
                .OrderBy(kvp => kvp.Key)
                .ToList();
            if (manual.Count == 0) return;

            _restoringState = true;
            try
            {
                foreach (var kvp in manual)
                {
                    var s = kvp.Value;
                    int rowIndex = _targetGrid.Rows.Add();
                    var row = _targetGrid.Rows[rowIndex];
                    row.Cells["Approved"].Value = s.Approved;
                    row.Cells["Number"].Value = rowIndex + 1;
                    row.Cells["Color"].Value = s.Color ?? "Red";
                    row.Cells["Plane"].Value = s.Plane ?? "ground";
                    row.Cells["Height"].Value = s.Height ?? "";
                    row.Cells["Description"].Value = s.Description ?? "";
                    row.Cells["ImagePath"].Value = "";
                    row.Cells["StateKey"].Value = kvp.Key;
                    row.Cells["East"].Value = s.East?.ToString(CultureInfo.InvariantCulture) ?? "";
                    row.Cells["North"].Value = s.North?.ToString(CultureInfo.InvariantCulture) ?? "";
                    row.Cells["Up"].Value = s.Up?.ToString(CultureInfo.InvariantCulture) ?? "";
                    row.Cells["Warning"].Value = "";
                }
            }
            finally
            {
                _restoringState = false;
            }
            RenumberTargets();
            ApplyRowHighlighting();
            PushGridTargetsToViewer();
        }

        private class BackendTargetInfo
        {
            public string Id;
            public string Color;
            public string Surface;
            public float East;
            public float North;
            public float Up;
        }

        // Pull /api/task/1/target/list_structured and use only the target's
        // final model position. Spatial descriptions are generated locally from
        // the Mission Planner building model so the Jetson does not own display
        // or submission wording.
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
                var byLetter = new Dictionary<string, BackendTargetInfo>(StringComparer.OrdinalIgnoreCase);
                foreach (var t in targets)
                {
                    var id = t["id"]?.ToString();
                    var east = (float?)t["east"];
                    var north = (float?)t["north"];
                    if (string.IsNullOrEmpty(id) || !east.HasValue || !north.HasValue) continue;
                    var up = (float?)t["up"] ?? 0f;
                    byLetter[id] = new BackendTargetInfo
                    {
                        Id = id,
                        Color = t["color"]?.ToString(),
                        Surface = InferSurfaceFromBackendTarget(t, up),
                        East = east.Value,
                        North = north.Value,
                        Up = up,
                    };
                }
                if (byLetter.Count == 0) return;

                if (_targetGrid.IsDisposed) return;
                if (_targetGrid.InvokeRequired)
                {
                    _targetGrid.Invoke((Action)(() => ApplyBackendTargetPositions(byLetter, savedState)));
                }
                else
                {
                    ApplyBackendTargetPositions(byLetter, savedState);
                }
            }
            catch { /* network/parse failure: keep local descriptions */ }
        }

        private void ApplyBackendTargetPositions(
            Dictionary<string, BackendTargetInfo> byLetter,
            Dictionary<string, SubmitRowState> savedState)
        {
            bool changed = false;
            _restoringState = true;
            try
            {
                for (int i = 0; i < _targetGrid.Rows.Count; i++)
                {
                    var letter = IndexToTargetLetter(i);
                    if (!byLetter.TryGetValue(letter, out var target)) continue;
                    var row = _targetGrid.Rows[i];
                    row.Cells["East"].Value = target.East.ToString(CultureInfo.InvariantCulture);
                    row.Cells["North"].Value = target.North.ToString(CultureInfo.InvariantCulture);
                    row.Cells["Up"].Value = target.Up.ToString(CultureInfo.InvariantCulture);
                    row.Cells["Height"].Value = target.Up.ToString("F1", CultureInfo.InvariantCulture);
                    row.Cells["Plane"].Value = target.Surface;
                    if (!string.IsNullOrWhiteSpace(target.Color))
                        row.Cells["Color"].Value = NormalizeTargetColor(target.Color);
                    changed = true;

                    var stateKey = row.Cells["StateKey"].Value?.ToString();
                    bool hasSavedDescription = !string.IsNullOrEmpty(stateKey)
                        && savedState.TryGetValue(stateKey, out var saved)
                        && !string.IsNullOrWhiteSpace(saved?.Description);
                    if (hasSavedDescription)
                    {
                        continue;
                    }

                    var placement = _viewer?.CreatePlacementFromLocal(target.Surface, target.East, target.North, target.Up);
                    if (placement != null)
                        row.Cells["Description"].Value = GeneratePlacementDescription(placement);
                }
                PushGridTargetsToViewer();
            }
            finally
            {
                _restoringState = false;
            }
            if (changed) ScheduleSubmitStateSave();
        }

        private static string InferSurfaceFromBackendTarget(Newtonsoft.Json.Linq.JToken t, float up)
        {
            string raw = t["plane"]?.ToString()
                ?? t["plane_kind"]?.ToString()
                ?? t["surface"]?.ToString()
                ?? t["face"]?.ToString()
                ?? "";
            raw = raw.Trim().ToLowerInvariant();
            if (raw.Contains("roof")) return "roof";
            if (raw.Contains("ground") || raw.Contains("floor")) return "ground";
            if (raw.Contains("wall")) return "wall";
            if (up <= 0.25f) return "ground";
            return "wall";
        }

        private static string NormalizeTargetColor(string color)
        {
            if (string.IsNullOrWhiteSpace(color)) return "Unknown";
            switch (color.Trim().ToLowerInvariant())
            {
                case "red": return "Red";
                case "blue": return "Blue";
                case "green": return "Green";
                case "yellow": return "Yellow";
                case "orange": return "Orange";
                case "purple": return "Purple";
                case "white": return "White";
                case "black": return "Black";
                default: return "Unknown";
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
                    var stateKey = row.Cells["StateKey"].Value?.ToString();
                    var imagePath = row.Cells["ImagePath"].Value?.ToString();
                    if (string.IsNullOrEmpty(stateKey)) stateKey = imagePath;
                    if (string.IsNullOrEmpty(stateKey)) continue;
                    dict[stateKey] = new SubmitRowState
                    {
                        Approved = (bool?)row.Cells["Approved"].Value ?? false,
                        Color = row.Cells["Color"].Value?.ToString(),
                        Plane = row.Cells["Plane"].Value?.ToString(),
                        Height = row.Cells["Height"].Value?.ToString(),
                        Description = row.Cells["Description"].Value?.ToString(),
                        StateKey = stateKey,
                        East = TryParseFloatCell(row, "East"),
                        North = TryParseFloatCell(row, "North"),
                        Up = TryParseFloatCell(row, "Up"),
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
            const int GRID_H    = 360;
            const int BTN_H     = 42;
            const int PREVIEW_H = 170;
            const int VIEWER_H  = 620;
            const int STATUS_H  = 36;
            const int TITLE_H   = 32;
            int totalH = TITLE_H + Math.Max(GRID_H + BTN_H + PREVIEW_H, VIEWER_H) + STATUS_H + 20;

            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 6,
                Height = totalH,
                Padding = new Padding(8),
            };
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, TITLE_H));    // 0: title
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));         // 1: 3D viewer
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, GRID_H));     // 2: target table
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, BTN_H));      // 3: buttons
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, PREVIEW_H));  // 4: preview text
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, STATUS_H));   // 5: status

            // Stretch inner panel width with the scroll container
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

            _targetGrid.Columns.Add(new DataGridViewTextBoxColumn
            {
                Name = "StateKey",
                HeaderText = "StateKey",
                ReadOnly = true,
                Visible = false,
            });

            _targetGrid.Columns.Add(new DataGridViewTextBoxColumn
            {
                Name = "East",
                HeaderText = "E",
                ReadOnly = true,
                Visible = false,
            });

            _targetGrid.Columns.Add(new DataGridViewTextBoxColumn
            {
                Name = "North",
                HeaderText = "N",
                ReadOnly = true,
                Visible = false,
            });

            _targetGrid.Columns.Add(new DataGridViewTextBoxColumn
            {
                Name = "Up",
                HeaderText = "U",
                ReadOnly = true,
                Visible = false,
            });

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
            _targetGrid.DataError += (s, e) => { e.ThrowException = false; };
            _targetGrid.CellMouseEnter += (s, e) =>
            {
                if (e.RowIndex < 0 || _viewer == null) return;
                _viewer.SetHighlightedTarget(TargetIdForRow(e.RowIndex));
            };
            _targetGrid.MouseLeave += (s, e) => _viewer?.SetHighlightedTarget(null);
            _targetGrid.RowsAdded += (s, e) => { ApplyRowHighlighting(); ScheduleSubmitStateSave(); };
            _targetGrid.RowsRemoved += (s, e) =>
            {
                ScheduleSubmitStateSave();
                PushGridTargetsToViewer();
            };
            _targetGrid.DataBindingComplete += (s, e) => ApplyRowHighlighting();

            mainLayout.Controls.Add(_targetGrid, 0, 2);

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

            _btnPlacementMode = CreateButton("Place: Off", Color.FromArgb(80, 80, 83), 82, 28);
            _btnPlacementMode.Click += BtnPlacementMode_Click;
            buttonPanel.Controls.Add(_btnPlacementMode);

            _btnGpsGround = CreateButton("GPS Ground", ACCENT_COLOR, 92, 28);
            _btnGpsGround.Click += (s, e) => AddGpsTarget("ground");
            buttonPanel.Controls.Add(_btnGpsGround);

            _btnGpsRoof = CreateButton("GPS Roof", ACCENT_COLOR, 75, 28);
            _btnGpsRoof.Click += (s, e) => AddGpsTarget("roof");
            buttonPanel.Controls.Add(_btnGpsRoof);

            _btnPreview = CreateButton("Preview TXT", Color.FromArgb(80, 80, 83), 100, 28);
            _btnPreview.Click += BtnPreview_Click;
            buttonPanel.Controls.Add(_btnPreview);

            _btnUpload = CreateButton("Upload to Google Drive", ACCENT_COLOR, 160, 28);
            _btnUpload.Click += BtnUpload_Click;
            buttonPanel.Controls.Add(_btnUpload);

            mainLayout.Controls.Add(buttonPanel, 0, 3);

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
            mainLayout.Controls.Add(_txtPreview, 0, 4);

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
                Text = "3D BUILDING MODEL - drag = orbit, wheel = zoom, right-drag/WASD/arrows = move view",
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
            _viewer.PlacementClicked += OnViewerPlacementClicked;
            viewerHost.Controls.Add(_viewer);
            viewerHost.Controls.Add(viewerHeader);
            mainLayout.Controls.Add(viewerHost, 0, 1);

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

            buttonPanel.Controls.Remove(_btnPlacementMode);
            buttonPanel.Controls.Remove(_btnGpsGround);
            buttonPanel.Controls.Remove(_btnGpsRoof);

            var modelTools = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.LeftToRight,
                Padding = new Padding(0, 4, 0, 4),
                BackColor = CARD_BG,
            };
            modelTools.Controls.Add(_btnPlacementMode);
            modelTools.Controls.Add(_btnGpsGround);
            modelTools.Controls.Add(_btnGpsRoof);

            var modelLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 2,
                Padding = new Padding(0),
            };
            modelLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, BTN_H));
            modelLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            modelLayout.Controls.Add(modelTools, 0, 0);
            modelLayout.Controls.Add(viewerHost, 0, 1);

            var submissionLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
                Padding = new Padding(0),
            };
            submissionLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            submissionLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, BTN_H));
            submissionLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, PREVIEW_H));
            submissionLayout.Controls.Add(_targetGrid, 0, 0);
            submissionLayout.Controls.Add(buttonPanel, 0, 1);
            submissionLayout.Controls.Add(_txtPreview, 0, 2);

            var contentTabs = new TabControl
            {
                Dock = DockStyle.Fill,
                Appearance = TabAppearance.Normal,
            };
            var modelPage = new TabPage("3D Model") { BackColor = CARD_BG, Padding = new Padding(0) };
            modelPage.Controls.Add(modelLayout);
            var tablePage = new TabPage("Submission Table") { BackColor = CARD_BG, Padding = new Padding(0) };
            tablePage.Controls.Add(submissionLayout);
            contentTabs.TabPages.Add(modelPage);
            contentTabs.TabPages.Add(tablePage);

            mainLayout.Controls.Clear();
            mainLayout.RowStyles.Clear();
            mainLayout.RowCount = 3;
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, TITLE_H));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, STATUS_H));
            mainLayout.Controls.Add(lblTitle, 0, 0);
            mainLayout.Controls.Add(contentTabs, 0, 1);
            mainLayout.Controls.Add(statusPanel, 0, 2);

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

        public void LoadCompetitionPresetModel()
        {
            var corners = GetCompetitionBuildingCorners()
                .Select(c => new BuildingViewer3D.Corner { Name = c.name, Lat = c.lat, Lon = c.lon })
                .ToList();
            SetBuildingModel(corners, CompetitionBuildingHeightM);
        }

        public void SetBuildingModel(IList<BuildingViewer3D.Corner> corners, double heightM)
        {
            _buildingHeight = Math.Max(0.5, heightM);
            _viewer?.SetBuildingHeight(_buildingHeight);
            _viewer?.SetCorners(corners);
            PushGridTargetsToViewer();
        }

        public void ClearBuildingModel()
        {
            _viewer?.SetCorners(new List<BuildingViewer3D.Corner>());
            _viewer?.SetTargets(new List<BuildingViewer3D.Target>());
            _lastBackendTargets.Clear();
            _lblStatus.Text = "3D building model cleared.";
            _lblStatus.ForeColor = TEXT_SECONDARY;
        }

        public void UpdateDronePose(double lat, double lon, double altMsl, double yawDeg, double pitchDeg, double rollDeg)
        {
            if (_viewer == null) return;
            double agl = _groundAltM == 0.0 ? Math.Max(0.0, altMsl) : Math.Max(0.0, altMsl - _groundAltM);
            _viewer.SetDronePoseGps(lat, lon, agl, yawDeg, pitchDeg, rollDeg);
        }

        public void SetGroundAltitudeReference(double groundAltM)
        {
            _groundAltM = groundAltM;
            if (_lblStatus != null && !_lblStatus.IsDisposed)
            {
                _lblStatus.Text = $"Ground-station altitude reference set to {groundAltM:F2}m.";
                _lblStatus.ForeColor = SUCCESS_COLOR;
            }
        }

        private void BtnPlacementMode_Click(object sender, EventArgs e)
        {
            _placementMode = !_placementMode;
            _viewer.PlacementMode = _placementMode;
            _btnPlacementMode.Text = _placementMode ? "Place: On" : "Place: Off";
            _btnPlacementMode.BackColor = _placementMode ? SUCCESS_COLOR : Color.FromArgb(80, 80, 83);
            _lblStatus.Text = _placementMode
                ? "Placement mode on: click the building, roof, or ground search area to add a target."
                : "Placement mode off.";
            _lblStatus.ForeColor = _placementMode ? SUCCESS_COLOR : TEXT_SECONDARY;
        }

        private void OnViewerPlacementClicked(BuildingViewer3D.Placement placement)
        {
            if (placement == null) return;
            AddManualPlacement(placement);
        }

        private void AddGpsTarget(string surface)
        {
            var cs = global::MissionPlanner.MainV2.comPort?.MAV?.cs;
            if (cs == null || Math.Abs(cs.lat) < 0.000001 || Math.Abs(cs.lng) < 0.000001)
            {
                _lblStatus.Text = "No GPS position available for GPS-only target.";
                _lblStatus.ForeColor = ERROR_COLOR;
                return;
            }

            if (!_viewer.TryGetLocalFromGps(cs.lat, cs.lng, out float east, out float north))
            {
                _lblStatus.Text = "Load or apply a building model before GPS-only placement.";
                _lblStatus.ForeColor = ERROR_COLOR;
                return;
            }
            double yawRad = ReadDouble(cs, "yaw") * Math.PI / 180.0;
            east += (float)(0.35 * Math.Sin(yawRad));
            north += (float)(0.35 * Math.Cos(yawRad));

            float up = surface == "roof" ? _viewer.BuildingHeightM : 0f;
            var placement = _viewer.CreatePlacementFromLocal(surface, east, north, up);
            if (placement == null)
            {
                _lblStatus.Text = "Could not project GPS onto the current building model.";
                _lblStatus.ForeColor = ERROR_COLOR;
                return;
            }
            AddManualPlacement(placement);
        }

        private void AddManualPlacement(BuildingViewer3D.Placement placement)
        {
            string defaultColor = "Red";
            string defaultDescription = GeneratePlacementDescription(placement);
            using (var dialog = new TargetPlacementDialog(defaultColor, defaultDescription))
            {
                if (dialog.ShowDialog(this) != DialogResult.OK) return;
                int rowIndex = _targetGrid.Rows.Add();
                var row = _targetGrid.Rows[rowIndex];
                row.Cells["Approved"].Value = true;
                row.Cells["Number"].Value = rowIndex + 1;
                row.Cells["Color"].Value = dialog.TargetColor;
                row.Cells["Plane"].Value = placement.Surface;
                row.Cells["Height"].Value = placement.Up.ToString("F1", CultureInfo.InvariantCulture);
                row.Cells["Description"].Value = dialog.DescriptionText;
                row.Cells["ImagePath"].Value = "";
                row.Cells["StateKey"].Value = $"manual:{DateTime.UtcNow:yyyyMMddHHmmssfff}:{Guid.NewGuid():N}";
                row.Cells["East"].Value = placement.East.ToString(CultureInfo.InvariantCulture);
                row.Cells["North"].Value = placement.North.ToString(CultureInfo.InvariantCulture);
                row.Cells["Up"].Value = placement.Up.ToString(CultureInfo.InvariantCulture);
                row.Cells["Warning"].Value = "";
                row.DefaultCellStyle.BackColor = Color.FromArgb(30, 30, 33);

                RenumberTargets();
                ApplyRowHighlighting();
                ScheduleSubmitStateSave();
                PushGridTargetsToViewer();
                _lblStatus.Text = $"Manual {placement.Surface} target added.";
                _lblStatus.ForeColor = SUCCESS_COLOR;
            }
        }

        private static string GeneratePlacementDescription(BuildingViewer3D.Placement p)
        {
            string structure = string.IsNullOrWhiteSpace(p.StructureLabel)
                ? "building"
                : p.StructureLabel;
            string face = string.IsNullOrWhiteSpace(p.WallName) ? "nearest" : p.WallName;
            string refWall = string.IsNullOrWhiteSpace(p.ReferenceWallName) ? "nearest" : p.ReferenceWallName;
            string refSuffix = structure == "building" ? "" : " of that portion";
            string refText = $"{p.DistanceFromReferenceWallM:F1}m from the {refWall} wall{refSuffix}";

            if (p.Surface == "roof")
                return $"on the roof of the {structure} near the {face} face, {p.Up:F1}m above ground and {refText}.";
            if (p.Surface == "ground")
                return $"on the ground near the {face} face of the {structure}, {p.Up:F1}m above ground and {refText}.";
            return $"on the {face} face of the {structure}, {p.Up:F1}m above ground and {refText}.";
        }

        public int RegenerateLocalDescriptions()
        {
            if (_targetGrid == null || _viewer == null) return 0;
            int count = 0;
            _restoringState = true;
            try
            {
                foreach (DataGridViewRow row in _targetGrid.Rows)
                {
                    if (row.IsNewRow) continue;
                    float? east = TryParseFloatCell(row, "East");
                    float? north = TryParseFloatCell(row, "North");
                    if (!east.HasValue || !north.HasValue) continue;
                    float up = TryParseFloatCell(row, "Up") ?? TryParseFloatCell(row, "Height") ?? 0f;
                    string surface = row.Cells["Plane"].Value?.ToString() ?? "wall";
                    var placement = _viewer.CreatePlacementFromLocal(surface, east.Value, north.Value, up);
                    if (placement == null) continue;
                    row.Cells["Description"].Value = GeneratePlacementDescription(placement);
                    row.Cells["Height"].Value = placement.Up.ToString("F1", CultureInfo.InvariantCulture);
                    row.Cells["Up"].Value = placement.Up.ToString(CultureInfo.InvariantCulture);
                    count++;
                }
            }
            finally
            {
                _restoringState = false;
            }
            if (count > 0)
            {
                ScheduleSubmitStateSave();
                PushGridTargetsToViewer();
            }
            return count;
        }

        private static float? TryParseFloatCell(DataGridViewRow row, string columnName)
        {
            if (!row.DataGridView.Columns.Contains(columnName)) return null;
            var raw = row.Cells[columnName].Value?.ToString();
            if (string.IsNullOrWhiteSpace(raw)) return null;
            if (float.TryParse(raw, NumberStyles.Float, CultureInfo.InvariantCulture, out var v)) return v;
            if (float.TryParse(raw, NumberStyles.Float, CultureInfo.CurrentCulture, out v)) return v;
            return null;
        }

        private static double ReadDouble(object obj, string propertyName)
        {
            try
            {
                var prop = obj.GetType().GetProperty(propertyName);
                if (prop == null) return 0.0;
                var value = prop.GetValue(obj, null);
                return value == null ? 0.0 : Convert.ToDouble(value);
            }
            catch { return 0.0; }
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
            row.Cells["StateKey"].Value = $"manual:{Guid.NewGuid():N}";
            row.Cells["East"].Value = "";
            row.Cells["North"].Value = "";
            row.Cells["Up"].Value = "";
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
                HasImage = !string.IsNullOrWhiteSpace(row.Cells["ImagePath"].Value?.ToString()),
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

                // 2. Captured-image rows also exist on the Jetson. Manual /
                // GPS-only rows without an image are ground-station-only, so
                // deleting them should not call the Jetson target endpoint.
                if (d.HasImage)
                {
                    _lastBackendTargets.RemoveAll(t =>
                        string.Equals(t.Id, d.Letter, StringComparison.OrdinalIgnoreCase));
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
                }

                // 3. Remove from grid
                if (_targetGrid.Rows.Contains(d.Row) && !d.Row.IsNewRow)
                    _targetGrid.Rows.Remove(d.Row);
            }

            RenumberTargets();
            PushGridTargetsToViewer();
            SaveSubmitState();
            _btnRemoveTarget.Enabled = true;
            _lblStatus.Text = $"Deleted {deletions.Count} target(s).";
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
            row.Cells["Color"].Value = NormalizeTargetColor(color);
            row.Cells["Plane"].Value = string.IsNullOrWhiteSpace(plane) ? "wall" : plane.Trim().ToLowerInvariant();
            row.Cells["Height"].Value = heightAgl;
            row.Cells["Description"].Value = suggestedDescription ?? "";
            row.Cells["ImagePath"].Value = imagePath ?? "";
            row.Cells["StateKey"].Value = imagePath ?? "";
            row.Cells["East"].Value = "";
            row.Cells["North"].Value = "";
            row.Cells["Up"].Value = "";
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
            PushGridTargetsToViewer();
            _ = RefreshViewerDataAsync();
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
                    descCell.Value = "on the [face] face of the building, [X]m above ground and [Y]m from the [western/eastern/southern/northern] wall.";
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
            {
                var stateKey = _targetGrid.Rows[e.RowIndex].Cells["StateKey"].Value?.ToString() ?? "";
                if (!stateKey.StartsWith("manual:", StringComparison.OrdinalIgnoreCase))
                    _ = SendPlaneOverrideAsync(e.RowIndex);
            }

            // Persist any user-visible field change so a crash never loses
            // approval/edit work. Image-path/preview/warning are derived,
            // not user-edited, so skip them.
            if (colName == "Approved" || colName == "Color" || colName == "Plane"
                || colName == "Height" || colName == "Description")
            {
                ScheduleSubmitStateSave();
                PushGridTargetsToViewer();
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
                    // Descriptions are color-free spatial bodies, e.g.
                    //   "on the north face of the building, 3.2m above ground and 1.6m from the western wall."
                    // The table's Color column is the single source of truth for target color.
                    // NormalizeBackendDescription only strips legacy capture text saved before
                    // the backend was changed (color word, "Target X:" prefix, [distance=...] tag).
                    var body = NormalizeBackendDescription(description);
                    string line = $"Target {letter} is {body}".TrimEnd();
                    if (!line.EndsWith(".")) line += ".";
                    if (!string.IsNullOrWhiteSpace(color))
                        line += $" The colour is {color.Trim().ToLowerInvariant()}.";
                    lines.Add(line);
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

                // CONOPS §5.2.3.6.f: file MUST be named Task_1_<TeamName>_targets.txt.
                var teamSlug = (MissionConfig.Load().TeamName ?? "MAD").Replace(" ", "_");
                var targetsFileName = $"Task_1_{teamSlug}_targets.txt";
                var tempTxtPath = Path.Combine(Path.GetTempPath(), targetsFileName);
                try
                {
                    File.WriteAllText(tempTxtPath, txtContent);

                    _lblStatus.Text = "Uploading targets file...";
                    var txtFileId = await gdrive.UploadFileAsync(tempTxtPath, targetsFileName);
                    if (string.IsNullOrEmpty(txtFileId))
                        throw new Exception($"Failed to upload {targetsFileName} to Google Drive.");

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
                    sb.AppendLine($"{targetsFileName} uploaded (ID: {txtFileId})");
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
                    row.Cells["StateKey"].Value = File.Exists(imagePath) ? imagePath : "";
                    row.Cells["East"].Value = "";
                    row.Cells["North"].Value = "";
                    row.Cells["Up"].Value = "";
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
            int idx = TargetLetterToIndex(targetId);
            if (idx < 0 || idx >= _targetGrid.Rows.Count) return;
            _targetGrid.ClearSelection();
            _targetGrid.Rows[idx].Selected = true;
            try { _targetGrid.FirstDisplayedScrollingRowIndex = Math.Max(0, idx - 2); } catch { }
        }

        private static int TargetLetterToIndex(string letters)
        {
            if (string.IsNullOrWhiteSpace(letters)) return -1;
            int n = 0;
            foreach (char ch in letters.Trim().ToUpperInvariant())
            {
                if (ch < 'A' || ch > 'Z') return -1;
                n = n * 26 + (ch - 'A' + 1);
            }
            return n - 1;
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
                var targetsResp = await JetsonApiService.GetAsync("/api/task/1/target/list_structured");
                if (targetsResp.IsSuccessStatusCode)
                {
                    var body = await targetsResp.Content.ReadAsStringAsync();
                    var data = Newtonsoft.Json.Linq.JObject.Parse(body);
                    var arr = data["targets"] as Newtonsoft.Json.Linq.JArray;

                    var targets = new List<BuildingViewer3D.Target>();
                    var byLetter = new Dictionary<string, BackendTargetInfo>(StringComparer.OrdinalIgnoreCase);
                    if (arr != null)
                    {
                        foreach (var t in arr)
                        {
                            var east = (float?)t["east"];
                            var north = (float?)t["north"];
                            var up = (float?)t["up"];
                            if (!east.HasValue || !north.HasValue) continue;
                            string id = t["id"]?.ToString() ?? "?";
                            targets.Add(new BuildingViewer3D.Target
                            {
                                Id = id,
                                Color = t["color"]?.ToString(),
                                Description = t["description"]?.ToString(),
                                East = east.Value,
                                North = north.Value,
                                Up = up ?? 0f,
                            });
                            byLetter[id] = new BackendTargetInfo
                            {
                                Id = id,
                                Color = t["color"]?.ToString(),
                                Surface = InferSurfaceFromBackendTarget(t, up ?? 0f),
                                East = east.Value,
                                North = north.Value,
                                Up = up ?? 0f,
                            };
                        }
                    }
                    _lastBackendTargets.Clear();
                    _lastBackendTargets.AddRange(targets);
                    ApplyBackendTargetPositions(byLetter, LoadSubmitState());
                    PushGridTargetsToViewer();
                }
            }
            catch
            {
                // Silently ignore — Jetson may be unreachable. The local model
                // remains usable for manual and GPS-only placement.
            }
            finally
            {
                _viewerRefreshInFlight = false;
            }
        }

        private void PushGridTargetsToViewer()
        {
            if (_viewer == null) return;
            var merged = new List<BuildingViewer3D.Target>(_lastBackendTargets);
            var ids = new HashSet<string>(merged.Select(t => t.Id ?? ""), StringComparer.OrdinalIgnoreCase);
            for (int i = 0; i < _targetGrid.Rows.Count; i++)
            {
                var row = _targetGrid.Rows[i];
                if (row.IsNewRow) continue;
                float? east = TryParseFloatCell(row, "East");
                float? north = TryParseFloatCell(row, "North");
                if (!east.HasValue || !north.HasValue) continue;
                string id = IndexToTargetLetter(i);
                var target = new BuildingViewer3D.Target
                {
                    Id = id,
                    Color = row.Cells["Color"].Value?.ToString(),
                    Description = row.Cells["Description"].Value?.ToString(),
                    East = east.Value,
                    North = north.Value,
                    Up = TryParseFloatCell(row, "Up") ?? 0f,
                };
                int existing = merged.FindIndex(t => string.Equals(t.Id, id, StringComparison.OrdinalIgnoreCase));
                if (existing >= 0) merged[existing] = target;
                else if (!ids.Contains(id)) merged.Add(target);
            }
            _viewer.SetTargets(merged);
        }

        private class TargetPlacementDialog : Form
        {
            private readonly ComboBox _color;
            private readonly TextBox _description;

            public string TargetColor => _color.SelectedItem?.ToString() ?? "Red";
            public string DescriptionText => _description.Text.Trim();

            public TargetPlacementDialog(string color, string description)
            {
                Text = "Task 1 Target";
                FormBorderStyle = FormBorderStyle.FixedDialog;
                StartPosition = FormStartPosition.CenterParent;
                MinimizeBox = false;
                MaximizeBox = false;
                ClientSize = new Size(430, 230);
                BackColor = CARD_BG;

                Controls.Add(new Label
                {
                    Text = "Color",
                    Location = new Point(12, 14),
                    AutoSize = true,
                    ForeColor = TEXT_SECONDARY,
                });

                _color = new ComboBox
                {
                    Location = new Point(12, 36),
                    Size = new Size(160, 24),
                    DropDownStyle = ComboBoxStyle.DropDownList,
                    BackColor = Color.FromArgb(25, 25, 28),
                    ForeColor = TEXT_PRIMARY,
                    FlatStyle = FlatStyle.Flat,
                };
                _color.Items.AddRange(new object[] { "Red", "Blue", "Green", "Yellow", "Orange", "Purple", "White", "Black", "Unknown" });
                _color.SelectedItem = string.IsNullOrWhiteSpace(color) ? "Red" : color;
                if (_color.SelectedIndex < 0) _color.SelectedIndex = 0;
                Controls.Add(_color);

                Controls.Add(new Label
                {
                    Text = "Description",
                    Location = new Point(12, 70),
                    AutoSize = true,
                    ForeColor = TEXT_SECONDARY,
                });

                _description = new TextBox
                {
                    Location = new Point(12, 92),
                    Size = new Size(406, 88),
                    Multiline = true,
                    ScrollBars = ScrollBars.Vertical,
                    Text = description ?? "",
                    BackColor = Color.FromArgb(25, 25, 28),
                    ForeColor = TEXT_PRIMARY,
                    BorderStyle = BorderStyle.FixedSingle,
                };
                Controls.Add(_description);

                var ok = new Button
                {
                    Text = "Add",
                    DialogResult = DialogResult.OK,
                    Location = new Point(250, 190),
                    Size = new Size(80, 28),
                    BackColor = SUCCESS_COLOR,
                    ForeColor = Color.White,
                    FlatStyle = FlatStyle.Flat,
                };
                ok.FlatAppearance.BorderSize = 0;
                Controls.Add(ok);

                var cancel = new Button
                {
                    Text = "Cancel",
                    DialogResult = DialogResult.Cancel,
                    Location = new Point(338, 190),
                    Size = new Size(80, 28),
                    BackColor = Color.FromArgb(70, 70, 73),
                    ForeColor = Color.White,
                    FlatStyle = FlatStyle.Flat,
                };
                cancel.FlatAppearance.BorderSize = 0;
                Controls.Add(cancel);

                AcceptButton = ok;
                CancelButton = cancel;
            }
        }
    }
}
