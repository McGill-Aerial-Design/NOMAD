// ============================================================
// NOMAD Task 1 V2 View - GPS/IMU based manual target placement
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using MissionPlanner;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    public class NOMADTask1V2View : NOMADViewBase, IUpdatableView
    {
        private readonly NOMADConfig _config;
        private readonly MissionConfig _missionConfig;
        private readonly BuildingViewer3D _viewer;
        private readonly DataGridView _targetGrid;
        private readonly ListBox _cornerList;
        private readonly Label _statusLabel;
        private readonly Label _poseLabel;
        private readonly Label _gpsLabel;
        private readonly TextBox _cornerName;
        private readonly TextBox _cornerLat;
        private readonly TextBox _cornerLon;
        private readonly NumericUpDown _heightM;
        private readonly Button _placeModeButton;
        private readonly TextBox _previewText;
        private readonly Task1V2State _state = new Task1V2State();
        private readonly string _statePath;
        private bool _placementMode = true;
        private bool _refreshingGrid;

        public NOMADTask1V2View(NOMADConfig config, MissionConfig missionConfig = null)
        {
            _config = config ?? NOMADConfig.Load();
            _missionConfig = missionConfig ?? MissionConfig.Load();
            _statePath = Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
                "NOMAD", "Task1V2", "task1v2_state.json");

            AutoScroll = false;
            Padding = new Padding(0);

            var layout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                BackColor = NOMADTheme.BG_DARK,
            };
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 68));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 32));

            var viewerHost = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(20, 20, 22),
                Padding = new Padding(0),
            };

            var header = new Label
            {
                Dock = DockStyle.Top,
                Height = 24,
                Text = "3D BUILDING MODEL - left-click places target, drag orbits, wheel zooms, right-drag pans",
                BackColor = Color.FromArgb(30, 30, 33),
                ForeColor = ACCENT_COLOR,
                Font = new Font("Segoe UI", 8.5f, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleLeft,
                Padding = new Padding(8, 0, 0, 0),
            };

            _viewer = new BuildingViewer3D { Dock = DockStyle.Fill, PlacementMode = true };
            _viewer.PlacementClicked += OnViewerPlacementClicked;
            _viewer.TargetHovered += id => SelectTargetById(id);
            viewerHost.Controls.Add(_viewer);
            viewerHost.Controls.Add(header);
            layout.Controls.Add(viewerHost, 0, 0);

            var side = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 5,
                BackColor = CARD_BG,
                Padding = new Padding(8),
            };
            side.RowStyles.Add(new RowStyle(SizeType.Absolute, 86));
            side.RowStyles.Add(new RowStyle(SizeType.Absolute, 220));
            side.RowStyles.Add(new RowStyle(SizeType.Percent, 48));
            side.RowStyles.Add(new RowStyle(SizeType.Percent, 52));
            side.RowStyles.Add(new RowStyle(SizeType.Absolute, 28));

            var poseCard = CreatePlainPanel();
            _gpsLabel = MakeLabel("GPS: waiting...", 10, 8, 9.5f, TEXT_PRIMARY);
            _poseLabel = MakeLabel("Pose: --", 10, 30, 9.5f, TEXT_PRIMARY);
            var groundButton = SmallButton("Set Ground Alt", 10, 54, 112, ACCENT_COLOR);
            groundButton.Click += (s, e) => SetGroundAltitudeFromCurrent();
            _placeModeButton = SmallButton("Placement: ON", 130, 54, 112, SUCCESS_COLOR);
            _placeModeButton.Click += (s, e) => TogglePlacementMode();
            poseCard.Controls.Add(_gpsLabel);
            poseCard.Controls.Add(_poseLabel);
            poseCard.Controls.Add(groundButton);
            poseCard.Controls.Add(_placeModeButton);
            side.Controls.Add(poseCard, 0, 0);

            var cornerCard = CreatePlainPanel();
            cornerCard.Controls.Add(MakeLabel("CORNERS", 10, 8, 9f, ACCENT_COLOR, true));
            cornerCard.Controls.Add(MakeLabel("Name", 10, 34, 8.5f, TEXT_SECONDARY));
            _cornerName = new TextBox
            {
                Location = new Point(55, 30),
                Size = new Size(56, 22),
                Text = "NW",
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = TEXT_PRIMARY,
                BorderStyle = BorderStyle.FixedSingle,
            };
            cornerCard.Controls.Add(_cornerName);
            var captureCorner = SmallButton("Capture", 118, 28, 72, ACCENT_COLOR);
            captureCorner.Click += (s, e) => CaptureCornerFromCurrent();
            cornerCard.Controls.Add(captureCorner);
            var removeCorner = SmallButton("Remove", 196, 28, 72, Color.FromArgb(110, 60, 60));
            removeCorner.Click += (s, e) => RemoveSelectedCorner();
            cornerCard.Controls.Add(removeCorner);

            cornerCard.Controls.Add(MakeLabel("Lat", 10, 62, 8.5f, TEXT_SECONDARY));
            _cornerLat = new TextBox
            {
                Location = new Point(55, 58),
                Size = new Size(106, 22),
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = TEXT_PRIMARY,
                BorderStyle = BorderStyle.FixedSingle,
            };
            cornerCard.Controls.Add(_cornerLat);

            cornerCard.Controls.Add(MakeLabel("Lon", 168, 62, 8.5f, TEXT_SECONDARY));
            _cornerLon = new TextBox
            {
                Location = new Point(204, 58),
                Size = new Size(106, 22),
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = TEXT_PRIMARY,
                BorderStyle = BorderStyle.FixedSingle,
            };
            cornerCard.Controls.Add(_cornerLon);

            var saveManualCorner = SmallButton("Save Manual", 55, 86, 106, ACCENT_COLOR);
            saveManualCorner.Click += (s, e) => SaveManualCorner();
            cornerCard.Controls.Add(saveManualCorner);

            _heightM = new NumericUpDown
            {
                Location = new Point(74, 116),
                Size = new Size(64, 22),
                Minimum = 1,
                Maximum = 50,
                DecimalPlaces = 1,
                Increment = 0.5M,
                Value = 5,
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = TEXT_PRIMARY,
            };
            _heightM.ValueChanged += (s, e) =>
            {
                _state.BuildingHeightM = (double)_heightM.Value;
                _viewer.SetBuildingHeight(_state.BuildingHeightM);
                SaveState();
                RefreshTargets();
            };
            cornerCard.Controls.Add(MakeLabel("Height", 10, 119, 8.5f, TEXT_SECONDARY));
            cornerCard.Controls.Add(_heightM);
            var clearCorners = SmallButton("Clear", 146, 114, 58, Color.FromArgb(90, 70, 40));
            clearCorners.Click += (s, e) => ClearCorners();
            cornerCard.Controls.Add(clearCorners);
            _cornerList = new ListBox
            {
                Location = new Point(10, 146),
                Size = new Size(300, 62),
                Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right,
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = TEXT_PRIMARY,
                BorderStyle = BorderStyle.FixedSingle,
                Font = new Font("Consolas", 8.5f),
            };
            _cornerList.SelectedIndexChanged += (s, e) => PopulateManualCornerFieldsFromSelection();
            cornerCard.Controls.Add(_cornerList);
            cornerCard.Resize += (s, e) => _cornerList.Width = Math.Max(80, cornerCard.ClientSize.Width - 20);
            side.Controls.Add(cornerCard, 0, 1);

            _targetGrid = BuildTargetGrid();
            side.Controls.Add(_targetGrid, 0, 2);

            var submissionPanel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 2,
                BackColor = CARD_BG,
            };
            submissionPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 34));
            submissionPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            var buttons = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.LeftToRight,
                WrapContents = false,
                BackColor = CARD_BG,
            };
            var removeTarget = SmallButton("Remove Target", 0, 0, 105, Color.FromArgb(110, 60, 60));
            removeTarget.Click += (s, e) => RemoveSelectedTarget();
            var preview = SmallButton("Preview TXT", 0, 0, 90, Color.FromArgb(70, 70, 75));
            preview.Click += (s, e) => _previewText.Text = GenerateSubmissionText();
            var upload = SmallButton("Upload", 0, 0, 74, ACCENT_COLOR);
            upload.Click += (s, e) => UploadSubmission();
            buttons.Controls.Add(removeTarget);
            buttons.Controls.Add(preview);
            buttons.Controls.Add(upload);
            submissionPanel.Controls.Add(buttons, 0, 0);
            _previewText = new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Vertical,
                BackColor = Color.FromArgb(25, 25, 28),
                ForeColor = TEXT_SECONDARY,
                Font = new Font("Consolas", 9),
                BorderStyle = BorderStyle.FixedSingle,
            };
            submissionPanel.Controls.Add(_previewText, 0, 1);
            side.Controls.Add(submissionPanel, 0, 3);

            _statusLabel = new Label
            {
                Dock = DockStyle.Fill,
                Text = "Ready",
                ForeColor = TEXT_SECONDARY,
                Font = new Font("Segoe UI", 8.5f),
                TextAlign = ContentAlignment.MiddleLeft,
            };
            side.Controls.Add(_statusLabel, 0, 4);

            layout.Controls.Add(side, 1, 0);
            Controls.Add(layout);

            LoadState();
            RefreshCorners();
            RefreshTargets();
        }

        public void UpdateData()
        {
            if (InvokeRequired)
            {
                BeginInvoke((MethodInvoker)UpdateData);
                return;
            }

            try
            {
                var cs = MainV2.comPort?.MAV?.cs;
                if (cs == null) return;

                int fix = (int)cs.gpsstatus;
                _gpsLabel.Text = $"GPS: fix {fix} | sats {cs.satcount} | {cs.lat:F7}, {cs.lng:F7}";
                _gpsLabel.ForeColor = fix >= 3 ? SUCCESS_COLOR : WARNING_COLOR;

                double yaw = ReadDouble(cs, "yaw");
                double pitch = ReadDouble(cs, "pitch");
                double roll = ReadDouble(cs, "roll");
                double agl = Math.Max(0, (double)cs.alt - _state.GroundAltM);
                _poseLabel.Text = $"Pose: alt {agl:F1}m AGL | yaw {yaw:F0} | pitch {pitch:F0} | roll {roll:F0}";

                _viewer.SetDronePoseGps(cs.lat, cs.lng, agl, yaw, pitch, roll);
            }
            catch { }
        }

        private DataGridView BuildTargetGrid()
        {
            var grid = new DataGridView
            {
                Dock = DockStyle.Fill,
                AllowUserToAddRows = false,
                AllowUserToDeleteRows = false,
                RowHeadersVisible = false,
                SelectionMode = DataGridViewSelectionMode.FullRowSelect,
                MultiSelect = false,
                BackgroundColor = Color.FromArgb(25, 25, 28),
                BorderStyle = BorderStyle.FixedSingle,
                AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill,
                ForeColor = TEXT_PRIMARY,
                EnableHeadersVisualStyles = false,
            };
            grid.ColumnHeadersDefaultCellStyle.BackColor = Color.FromArgb(45, 45, 48);
            grid.ColumnHeadersDefaultCellStyle.ForeColor = TEXT_PRIMARY;
            grid.DefaultCellStyle.BackColor = Color.FromArgb(30, 30, 33);
            grid.DefaultCellStyle.ForeColor = TEXT_PRIMARY;
            grid.DefaultCellStyle.SelectionBackColor = Color.FromArgb(0, 100, 180);
            grid.DefaultCellStyle.SelectionForeColor = Color.White;

            grid.Columns.Add(new DataGridViewCheckBoxColumn { Name = "Approved", HeaderText = "", Width = 32, FillWeight = 16 });
            grid.Columns.Add(new DataGridViewTextBoxColumn { Name = "Id", HeaderText = "ID", ReadOnly = true, FillWeight = 20 });
            grid.Columns.Add(new DataGridViewTextBoxColumn { Name = "Color", HeaderText = "Color", FillWeight = 32 });
            grid.Columns.Add(new DataGridViewTextBoxColumn { Name = "Surface", HeaderText = "Surface", ReadOnly = true, FillWeight = 40 });
            grid.Columns.Add(new DataGridViewTextBoxColumn { Name = "Height", HeaderText = "H", ReadOnly = true, FillWeight = 22 });
            grid.Columns.Add(new DataGridViewTextBoxColumn { Name = "Description", HeaderText = "Description", FillWeight = 150 });
            grid.CellEndEdit += (s, e) => SyncGridToState();
            grid.CellValueChanged += (s, e) => SyncGridToState();
            grid.CurrentCellDirtyStateChanged += (s, e) =>
            {
                if (grid.IsCurrentCellDirty) grid.CommitEdit(DataGridViewDataErrorContexts.Commit);
            };
            grid.SelectionChanged += (s, e) =>
            {
                if (grid.SelectedRows.Count == 0) return;
                _viewer.SetHighlightedTarget(grid.SelectedRows[0].Cells["Id"].Value?.ToString());
            };
            return grid;
        }

        private void OnViewerPlacementClicked(BuildingViewer3D.Placement placement)
        {
            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => OnViewerPlacementClicked(placement)));
                return;
            }

            string color = "Red";
            string description = GenerateDescription(placement);
            using (var dialog = new TargetPlacementDialog(color, description))
            {
                if (dialog.ShowDialog(this) != DialogResult.OK) return;
                color = dialog.TargetColor;
                description = dialog.DescriptionText;
            }

            var id = Task1UploadPanel.IndexToTargetLetter(_state.Targets.Count);
            _state.Targets.Add(new Task1V2Target
            {
                Id = id,
                Color = color,
                Description = description,
                Surface = placement.Surface,
                WallName = placement.WallName,
                East = placement.East,
                North = placement.North,
                Up = placement.Up,
                Approved = true,
            });
            SaveState();
            RefreshTargets();
            _statusLabel.Text = $"Target {id} placed on {placement.Surface}.";
            _statusLabel.ForeColor = SUCCESS_COLOR;
        }

        private string GenerateDescription(BuildingViewer3D.Placement p)
        {
            string corner = string.IsNullOrWhiteSpace(p.NearestCornerName) ? "nearest" : p.NearestCornerName;
            if (p.Surface == "wall")
            {
                string wall = string.IsNullOrWhiteSpace(p.WallName) ? "selected wall" : $"wall {p.WallName}";
                return $"on {wall} of the building, {p.Up:F1}m above ground, {p.DistanceFromCornerM:F1}m from the {corner} corner.";
            }
            if (p.Surface == "roof")
                return $"on the roof of the building, {p.DistanceFromCornerM:F1}m from the {corner} corner.";
            return $"on the ground, {p.DistanceFromCornerM:F1}m from the {corner} corner.";
        }

        private void CaptureCornerFromCurrent()
        {
            var cs = MainV2.comPort?.MAV?.cs;
            if (cs == null || (Math.Abs(cs.lat) < 0.000001 && Math.Abs(cs.lng) < 0.000001))
            {
                SetStatus("No GPS position available for corner capture.", ERROR_COLOR);
                return;
            }

            string name = _cornerName.Text.Trim();
            if (string.IsNullOrWhiteSpace(name))
            {
                SetStatus("Enter a corner name first.", WARNING_COLOR);
                return;
            }

            AddOrUpdateCorner(name, cs.lat, cs.lng);
            _cornerLat.Text = cs.lat.ToString("F7", CultureInfo.InvariantCulture);
            _cornerLon.Text = cs.lng.ToString("F7", CultureInfo.InvariantCulture);
            AutoAdvanceCornerName(name);
            SetStatus($"Corner {name} captured at {cs.lat:F7}, {cs.lng:F7}.", SUCCESS_COLOR);
        }

        private void SaveManualCorner()
        {
            string name = _cornerName.Text.Trim();
            if (string.IsNullOrWhiteSpace(name))
            {
                SetStatus("Enter a corner name first.", WARNING_COLOR);
                return;
            }

            if (!TryParseCoordinate(_cornerLat.Text, -90.0, 90.0, out double lat))
            {
                SetStatus("Enter a valid latitude between -90 and 90.", ERROR_COLOR);
                return;
            }

            if (!TryParseCoordinate(_cornerLon.Text, -180.0, 180.0, out double lon))
            {
                SetStatus("Enter a valid longitude between -180 and 180.", ERROR_COLOR);
                return;
            }

            AddOrUpdateCorner(name, lat, lon);
            SetStatus($"Corner {name} saved at {lat:F7}, {lon:F7}.", SUCCESS_COLOR);
        }

        private void AddOrUpdateCorner(string name, double lat, double lon)
        {
            var existing = _state.Corners.FirstOrDefault(c => string.Equals(c.Name, name, StringComparison.OrdinalIgnoreCase));
            if (existing != null)
            {
                existing.Lat = lat;
                existing.Lon = lon;
            }
            else
            {
                _state.Corners.Add(new Task1V2Corner { Name = name, Lat = lat, Lon = lon });
            }

            SaveState();
            RefreshCorners();
        }

        private static bool TryParseCoordinate(string raw, double min, double max, out double value)
        {
            raw = (raw ?? "").Trim();
            bool parsed = double.TryParse(raw, NumberStyles.Float, CultureInfo.InvariantCulture, out value)
                || double.TryParse(raw, NumberStyles.Float, CultureInfo.CurrentCulture, out value);
            return parsed && !double.IsNaN(value) && !double.IsInfinity(value) && value >= min && value <= max;
        }

        private void RefreshCorners()
        {
            _cornerList.Items.Clear();
            foreach (var c in _state.Corners)
                _cornerList.Items.Add($"{c.Name}: {c.Lat:F7}, {c.Lon:F7}");

            _heightM.Value = (decimal)Math.Max((double)_heightM.Minimum, Math.Min((double)_heightM.Maximum, _state.BuildingHeightM));
            _viewer.SetBuildingHeight(_state.BuildingHeightM);
            _viewer.SetCorners(_state.Corners.Select(c => new BuildingViewer3D.Corner
            {
                Name = c.Name,
                Lat = c.Lat,
                Lon = c.Lon,
            }).ToList());
        }

        private void PopulateManualCornerFieldsFromSelection()
        {
            int idx = _cornerList.SelectedIndex;
            if (idx < 0 || idx >= _state.Corners.Count) return;

            var corner = _state.Corners[idx];
            _cornerName.Text = corner.Name ?? "";
            _cornerLat.Text = corner.Lat.ToString("F7", CultureInfo.InvariantCulture);
            _cornerLon.Text = corner.Lon.ToString("F7", CultureInfo.InvariantCulture);
        }

        private void RefreshTargets()
        {
            for (int i = 0; i < _state.Targets.Count; i++)
                _state.Targets[i].Id = Task1UploadPanel.IndexToTargetLetter(i);

            _refreshingGrid = true;
            _targetGrid.Rows.Clear();
            try
            {
                foreach (var t in _state.Targets)
                {
                    int idx = _targetGrid.Rows.Add();
                    var row = _targetGrid.Rows[idx];
                    row.Cells["Approved"].Value = t.Approved;
                    row.Cells["Id"].Value = t.Id;
                    row.Cells["Color"].Value = t.Color;
                    row.Cells["Surface"].Value = string.IsNullOrWhiteSpace(t.WallName) ? t.Surface : t.WallName;
                    row.Cells["Height"].Value = t.Up.ToString("F1");
                    row.Cells["Description"].Value = t.Description;
                }
            }
            finally
            {
                _refreshingGrid = false;
            }

            _viewer.SetTargets(_state.Targets.Select(t => new BuildingViewer3D.Target
            {
                Id = t.Id,
                Color = t.Color,
                Description = t.Description,
                East = t.East,
                North = t.North,
                Up = t.Up,
            }).ToList());

            _previewText.Text = GenerateSubmissionText();
        }

        private void SyncGridToState()
        {
            if (_refreshingGrid) return;
            if (_targetGrid.Rows.Count != _state.Targets.Count) return;
            for (int i = 0; i < _targetGrid.Rows.Count; i++)
            {
                var row = _targetGrid.Rows[i];
                var t = _state.Targets[i];
                t.Approved = (bool?)row.Cells["Approved"].Value ?? false;
                t.Color = row.Cells["Color"].Value?.ToString() ?? t.Color;
                t.Description = row.Cells["Description"].Value?.ToString() ?? t.Description;
            }
            SaveState();
            _viewer.SetTargets(_state.Targets.Select(t => new BuildingViewer3D.Target
            {
                Id = t.Id,
                Color = t.Color,
                Description = t.Description,
                East = t.East,
                North = t.North,
                Up = t.Up,
            }).ToList());
        }

        private string GenerateSubmissionText()
        {
            var lines = new List<string>();
            int approvedIndex = 0;
            foreach (var target in _state.Targets.Where(t => t.Approved))
            {
                string letter = Task1UploadPanel.IndexToTargetLetter(approvedIndex++);
                string color = string.IsNullOrWhiteSpace(target.Color) ? "Unknown" : target.Color.Trim();
                string desc = (target.Description ?? "").Trim();
                if (string.IsNullOrWhiteSpace(desc)) continue;
                lines.Add($"Target {letter}: {color} target {desc}");
            }
            return lines.Count == 0 ? "No approved targets." : string.Join("\r\n\r\n", lines);
        }

        private async void UploadSubmission()
        {
            string text = GenerateSubmissionText();
            if (text == "No approved targets.")
            {
                MessageBox.Show("No approved targets to upload.", "Task 1 V2", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            try
            {
                SetStatus("Uploading Task 1 v2 targets file...", WARNING_COLOR);
                var gdrive = new GoogleDriveUploadService();
                var diag = gdrive.DiagnoseToken();
                if (diag != null) throw new Exception(diag);

                string teamSlug = (_missionConfig.TeamName ?? "MAD").Replace(" ", "_");
                string filename = $"Task_1_{teamSlug}_targets.txt";
                string tempPath = Path.Combine(Path.GetTempPath(), filename);
                File.WriteAllText(tempPath, text);
                try
                {
                    string fileId = await gdrive.UploadFileAsync(tempPath, filename);
                    SetStatus($"Uploaded {filename} ({fileId}).", SUCCESS_COLOR);
                    MessageBox.Show($"Uploaded {filename}.", "Task 1 V2", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                finally
                {
                    try { if (File.Exists(tempPath)) File.Delete(tempPath); } catch { }
                }
            }
            catch (Exception ex)
            {
                SetStatus($"Upload failed: {ex.Message}", ERROR_COLOR);
                MessageBox.Show(ex.Message, "Task 1 V2 Upload Failed", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void RemoveSelectedCorner()
        {
            int idx = _cornerList.SelectedIndex;
            if (idx < 0 || idx >= _state.Corners.Count) return;
            _state.Corners.RemoveAt(idx);
            SaveState();
            RefreshCorners();
            SetStatus("Corner removed.", WARNING_COLOR);
        }

        private void ClearCorners()
        {
            if (MessageBox.Show("Clear all Task 1 v2 corners?", "Task 1 V2", MessageBoxButtons.YesNo, MessageBoxIcon.Warning) != DialogResult.Yes)
                return;
            _state.Corners.Clear();
            SaveState();
            RefreshCorners();
            SetStatus("Corners cleared.", WARNING_COLOR);
        }

        private void RemoveSelectedTarget()
        {
            if (_targetGrid.SelectedRows.Count == 0) return;
            int idx = _targetGrid.SelectedRows[0].Index;
            if (idx < 0 || idx >= _state.Targets.Count) return;
            _state.Targets.RemoveAt(idx);
            SaveState();
            RefreshTargets();
            SetStatus("Target removed.", WARNING_COLOR);
        }

        private void SelectTargetById(string id)
        {
            if (InvokeRequired)
            {
                BeginInvoke(new Action(() => SelectTargetById(id)));
                return;
            }
            if (string.IsNullOrWhiteSpace(id)) return;
            foreach (DataGridViewRow row in _targetGrid.Rows)
            {
                if (row.Cells["Id"].Value?.ToString() != id) continue;
                row.Selected = true;
                try { _targetGrid.FirstDisplayedScrollingRowIndex = Math.Max(0, row.Index - 1); } catch { }
                return;
            }
        }

        private void SetGroundAltitudeFromCurrent()
        {
            var cs = MainV2.comPort?.MAV?.cs;
            if (cs == null)
            {
                SetStatus("No altitude available.", ERROR_COLOR);
                return;
            }
            _state.GroundAltM = cs.alt;
            SaveState();
            SetStatus($"Ground altitude set to {cs.alt:F1}m.", SUCCESS_COLOR);
        }

        private void TogglePlacementMode()
        {
            _placementMode = !_placementMode;
            _viewer.PlacementMode = _placementMode;
            _placeModeButton.Text = _placementMode ? "Placement: ON" : "Placement: OFF";
            _placeModeButton.BackColor = _placementMode ? SUCCESS_COLOR : Color.FromArgb(70, 70, 75);
        }

        private void LoadState()
        {
            try
            {
                if (File.Exists(_statePath))
                {
                    var loaded = JsonConvert.DeserializeObject<Task1V2State>(File.ReadAllText(_statePath));
                    if (loaded != null)
                    {
                        _state.BuildingHeightM = loaded.BuildingHeightM <= 0 ? 5.0 : loaded.BuildingHeightM;
                        _state.GroundAltM = loaded.GroundAltM;
                        _state.Corners = loaded.Corners ?? new List<Task1V2Corner>();
                        _state.Targets = loaded.Targets ?? new List<Task1V2Target>();
                    }
                }
            }
            catch (Exception ex)
            {
                SetStatus($"Failed to load v2 state: {ex.Message}", WARNING_COLOR);
            }
        }

        private void SaveState()
        {
            try
            {
                Directory.CreateDirectory(Path.GetDirectoryName(_statePath));
                File.WriteAllText(_statePath, JsonConvert.SerializeObject(_state, Formatting.Indented));
            }
            catch { }
        }

        private void SetStatus(string text, Color color)
        {
            _statusLabel.Text = text;
            _statusLabel.ForeColor = color;
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

        private void AutoAdvanceCornerName(string current)
        {
            var names = new[] { "NW", "NE", "SE", "SW" };
            int idx = Array.IndexOf(names, current.ToUpperInvariant());
            if (idx >= 0 && idx < names.Length - 1)
            {
                _cornerName.Text = names[idx + 1];
                return;
            }
            if (current.Length == 1 && char.IsLetter(current[0]))
            {
                char next = (char)(char.ToUpperInvariant(current[0]) + 1);
                if (next <= 'Z') _cornerName.Text = next.ToString();
            }
        }

        private Panel CreatePlainPanel()
        {
            return new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CARD_BG,
                BorderStyle = BorderStyle.FixedSingle,
            };
        }

        private Label MakeLabel(string text, int x, int y, float size, Color color, bool bold = false)
        {
            return new Label
            {
                Text = text,
                Location = new Point(x, y),
                AutoSize = true,
                ForeColor = color,
                Font = new Font("Segoe UI", size, bold ? FontStyle.Bold : FontStyle.Regular),
            };
        }

        private Button SmallButton(string text, int x, int y, int w, Color bg)
        {
            var btn = new Button
            {
                Text = text,
                Location = new Point(x, y),
                Size = new Size(w, 24),
                FlatStyle = FlatStyle.Flat,
                BackColor = bg,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8.5f, FontStyle.Bold),
                Margin = new Padding(2),
            };
            btn.FlatAppearance.BorderSize = 0;
            return btn;
        }

        private class Task1V2State
        {
            public double BuildingHeightM { get; set; } = 5.0;
            public double GroundAltM { get; set; }
            public List<Task1V2Corner> Corners { get; set; } = new List<Task1V2Corner>();
            public List<Task1V2Target> Targets { get; set; } = new List<Task1V2Target>();
        }

        private class Task1V2Corner
        {
            public string Name { get; set; }
            public double Lat { get; set; }
            public double Lon { get; set; }
        }

        private class Task1V2Target
        {
            public string Id { get; set; }
            public string Color { get; set; }
            public string Description { get; set; }
            public string Surface { get; set; }
            public string WallName { get; set; }
            public float East { get; set; }
            public float North { get; set; }
            public float Up { get; set; }
            public bool Approved { get; set; }
        }

        private class TargetPlacementDialog : Form
        {
            private readonly ComboBox _color;
            private readonly TextBox _description;

            public string TargetColor => _color.SelectedItem?.ToString() ?? "Red";
            public string DescriptionText => _description.Text.Trim();

            public TargetPlacementDialog(string color, string description)
            {
                Text = "Place Task 1 Target";
                FormBorderStyle = FormBorderStyle.FixedDialog;
                StartPosition = FormStartPosition.CenterParent;
                MinimizeBox = false;
                MaximizeBox = false;
                ClientSize = new Size(430, 230);
                BackColor = NOMADTheme.CARD_BG;

                Controls.Add(new Label
                {
                    Text = "Color",
                    Location = new Point(12, 14),
                    AutoSize = true,
                    ForeColor = NOMADTheme.TEXT_SECONDARY,
                });
                _color = new ComboBox
                {
                    Location = new Point(72, 10),
                    Size = new Size(130, 24),
                    DropDownStyle = ComboBoxStyle.DropDownList,
                    BackColor = Color.FromArgb(25, 25, 28),
                    ForeColor = NOMADTheme.TEXT_PRIMARY,
                };
                _color.Items.AddRange(new object[] { "Red", "Blue", "Green", "Yellow", "Orange", "Purple", "White", "Black" });
                _color.SelectedItem = _color.Items.Contains(color) ? color : "Red";
                Controls.Add(_color);

                Controls.Add(new Label
                {
                    Text = "Description",
                    Location = new Point(12, 48),
                    AutoSize = true,
                    ForeColor = NOMADTheme.TEXT_SECONDARY,
                });
                _description = new TextBox
                {
                    Location = new Point(12, 70),
                    Size = new Size(406, 104),
                    Multiline = true,
                    ScrollBars = ScrollBars.Vertical,
                    Text = description,
                    BackColor = Color.FromArgb(25, 25, 28),
                    ForeColor = NOMADTheme.TEXT_PRIMARY,
                    BorderStyle = BorderStyle.FixedSingle,
                };
                Controls.Add(_description);

                var ok = new Button
                {
                    Text = "Add Target",
                    DialogResult = DialogResult.OK,
                    Location = new Point(230, 190),
                    Size = new Size(90, 28),
                    BackColor = NOMADTheme.ACCENT,
                    ForeColor = Color.White,
                    FlatStyle = FlatStyle.Flat,
                };
                ok.FlatAppearance.BorderSize = 0;
                var cancel = new Button
                {
                    Text = "Cancel",
                    DialogResult = DialogResult.Cancel,
                    Location = new Point(328, 190),
                    Size = new Size(90, 28),
                    BackColor = Color.FromArgb(70, 70, 75),
                    ForeColor = Color.White,
                    FlatStyle = FlatStyle.Flat,
                };
                cancel.FlatAppearance.BorderSize = 0;
                Controls.Add(ok);
                Controls.Add(cancel);
                AcceptButton = ok;
                CancelButton = cancel;
            }
        }
    }
}
