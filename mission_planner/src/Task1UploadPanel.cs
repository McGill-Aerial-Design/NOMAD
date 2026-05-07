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

        public double BuildingHeight
        {
            get => _buildingHeight;
            set => _buildingHeight = value;
        }

        public Task1UploadPanel(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            InitializeUI();
        }

        private void InitializeUI()
        {
            this.BackColor = CARD_BG;
            this.Padding = new Padding(10);

            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 5,
                Padding = new Padding(5),
            };
            mainLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 55));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 45));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));

            var lblTitle = new Label
            {
                Text = "TASK 1 SUBMISSION",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                AutoSize = true,
                Padding = new Padding(0, 0, 0, 10),
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
            _targetGrid.RowsAdded += (s, e) => ApplyRowHighlighting();
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

            mainLayout.Controls.Add(statusPanel, 0, 4);

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
            if (_targetGrid.SelectedRows.Count > 0)
            {
                foreach (DataGridViewRow row in _targetGrid.SelectedRows)
                    if (!row.IsNewRow)
                        _targetGrid.Rows.RemoveAt(row.Index);
                RenumberTargets();
            }
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
        }

        private async Task SendPlaneOverrideAsync(int rowIndex)
        {
            if (rowIndex < 0 || rowIndex >= _targetGrid.Rows.Count) return;
            var row = _targetGrid.Rows[rowIndex];
            var plane = row.Cells["Plane"].Value?.ToString();
            if (string.IsNullOrEmpty(plane)) return;

            // Target letters are assigned in capture order: row 0 = A, row 1 = B, ...
            string targetId = ((char)('A' + rowIndex)).ToString();
            try
            {
                _lblStatus.Text = $"Sending plane override for target {targetId} ({plane})...";
                _lblStatus.ForeColor = TEXT_SECONDARY;
                var json = JsonConvert.SerializeObject(new { plane_kind = plane });
                var content = new StringContent(json, Encoding.UTF8, "application/json");
                var resp = await JetsonApiService.PostAsync(
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

            foreach (DataGridViewRow row in _targetGrid.Rows)
            {
                bool approved = (bool?)row.Cells["Approved"].Value ?? false;
                if (!approved) continue;

                var number = row.Cells["Number"].Value?.ToString();
                var color = row.Cells["Color"].Value?.ToString() ?? "";
                var description = row.Cells["Description"].Value?.ToString() ?? "";

                if (!string.IsNullOrEmpty(number) && !string.IsNullOrEmpty(description))
                {
                    string fullDesc = description;
                    if (!description.StartsWith(color, StringComparison.OrdinalIgnoreCase))
                    {
                        fullDesc = $"{color} target - {description}";
                    }
                    lines.Add($"Target {number}: {fullDesc}");
                }
            }

            return string.Join("\n\n", lines);
        }

        private async void BtnUpload_Click(object sender, EventArgs e)
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

            foreach (DataGridViewRow row in _targetGrid.Rows)
            {
                if (!((bool?)row.Cells["Approved"].Value ?? false)) continue;
                var desc = row.Cells["Description"].Value?.ToString();
                if (string.IsNullOrWhiteSpace(desc))
                {
                    MessageBox.Show($"Target {row.Cells["Number"].Value} has no description.",
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
                if (!gdrive.HasToken())
                {
                    throw new Exception(
                        "No Google Drive token found.\n\n" +
                        "Place your gdrive_token.json in ~/.nomad/ or upload it via Settings > Google Drive.");
                }

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

                    var imageResults = new List<(int number, string filename, string fileId)>();
                    var errors = new List<string>();

                    foreach (DataGridViewRow row in _targetGrid.Rows)
                    {
                        if (!((bool?)row.Cells["Approved"].Value ?? false)) continue;

                        var number = int.Parse(row.Cells["Number"].Value?.ToString() ?? "0");
                        var imagePath = row.Cells["ImagePath"].Value?.ToString() ?? "";

                        if (!string.IsNullOrEmpty(imagePath) && File.Exists(imagePath))
                        {
                            _lblStatus.Text = $"Uploading Target {number} image...";
                            var ext = Path.GetExtension(imagePath) ?? ".jpg";
                            var filename = $"Target_{number}{ext}";

                            var fileId = await gdrive.UploadFileAsync(imagePath, filename);
                            if (!string.IsNullOrEmpty(fileId))
                            {
                                imageResults.Add((number, filename, fileId));
                            }
                            else
                            {
                                errors.Add($"Failed to upload image for Target {number}");
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
                        metadata?.AiDescription ?? metadata?.RelativeDescription ?? "";
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
    }
}
