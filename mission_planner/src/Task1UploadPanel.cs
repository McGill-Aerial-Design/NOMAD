// ============================================================
// NOMAD Task 1 Upload Panel
// ============================================================
// Allows operators to manually enter/edit target descriptions
// and upload Task_1_MAD_targets.txt + images to Google Drive.
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
    /// <summary>
    /// Panel for managing and uploading Task 1 target submissions to Google Drive.
    /// </summary>
    public class Task1UploadPanel : UserControl
    {
        // Theme colors (matching NOMADViewBase)
        private static readonly Color CARD_BG = Color.FromArgb(35, 35, 38);
        private static readonly Color TEXT_PRIMARY = Color.FromArgb(220, 220, 220);
        private static readonly Color TEXT_SECONDARY = Color.FromArgb(160, 160, 160);
        private static readonly Color ACCENT_COLOR = Color.FromArgb(0, 122, 204);
        private static readonly Color SUCCESS_COLOR = Color.FromArgb(76, 175, 80);
        private static readonly Color WARNING_COLOR = Color.FromArgb(255, 193, 7);
        private static readonly Color ERROR_COLOR = Color.FromArgb(244, 67, 54);

        private readonly NOMADConfig _config;
        private DataGridView _targetGrid;
        private Button _btnAddTarget;
        private Button _btnRemoveTarget;
        private Button _btnPreview;
        private Button _btnUpload;
        private TextBox _txtPreview;
        private Label _lblStatus;
        private ProgressBar _progressBar;

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
            mainLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));      // Title
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 50));   // Grid
            mainLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));      // Buttons
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 50));   // Preview
            mainLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));      // Status

            // Title
            var lblTitle = new Label
            {
                Text = "TASK 1 SUBMISSION",
                Font = new Font("Segoe UI", 11, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                AutoSize = true,
                Padding = new Padding(0, 0, 0, 10),
            };
            mainLayout.Controls.Add(lblTitle, 0, 0);

            // Target Grid
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
                MultiSelect = false,
                AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill,
                EditMode = DataGridViewEditMode.EditOnEnter,
            };

            // Style the header
            _targetGrid.ColumnHeadersDefaultCellStyle.BackColor = Color.FromArgb(45, 45, 48);
            _targetGrid.ColumnHeadersDefaultCellStyle.ForeColor = TEXT_PRIMARY;
            _targetGrid.ColumnHeadersDefaultCellStyle.Font = new Font("Segoe UI", 9, FontStyle.Bold);
            _targetGrid.ColumnHeadersDefaultCellStyle.SelectionBackColor = Color.FromArgb(45, 45, 48);
            _targetGrid.ColumnHeadersHeight = 30;

            // Style rows
            _targetGrid.DefaultCellStyle.BackColor = Color.FromArgb(30, 30, 33);
            _targetGrid.DefaultCellStyle.ForeColor = TEXT_PRIMARY;
            _targetGrid.DefaultCellStyle.SelectionBackColor = Color.FromArgb(0, 100, 180);
            _targetGrid.DefaultCellStyle.SelectionForeColor = Color.White;
            _targetGrid.AlternatingRowsDefaultCellStyle.BackColor = Color.FromArgb(35, 35, 38);
            _targetGrid.RowTemplate.Height = 28;

            // Define columns
            var colNumber = new DataGridViewTextBoxColumn
            {
                Name = "Number",
                HeaderText = "#",
                Width = 40,
                ReadOnly = true,
            };
            _targetGrid.Columns.Add(colNumber);

            var colColor = new DataGridViewComboBoxColumn
            {
                Name = "Color",
                HeaderText = "Color",
                Width = 80,
                FlatStyle = FlatStyle.Flat,
            };
            colColor.Items.AddRange("Red", "Blue", "Green", "Yellow", "Orange", "Purple", "White", "Black");
            _targetGrid.Columns.Add(colColor);

            var colDescription = new DataGridViewTextBoxColumn
            {
                Name = "Description",
                HeaderText = "Description (ConOps format)",
                FillWeight = 100,
            };
            _targetGrid.Columns.Add(colDescription);

            var colImage = new DataGridViewButtonColumn
            {
                Name = "ImageBtn",
                HeaderText = "Image",
                Text = "Browse...",
                UseColumnTextForButtonValue = true,
                Width = 80,
            };
            _targetGrid.Columns.Add(colImage);

            var colImagePath = new DataGridViewTextBoxColumn
            {
                Name = "ImagePath",
                HeaderText = "Image Path",
                Width = 150,
                ReadOnly = true,
            };
            _targetGrid.Columns.Add(colImagePath);

            _targetGrid.CellClick += TargetGrid_CellClick;
            _targetGrid.CellEndEdit += TargetGrid_CellEndEdit;

            mainLayout.Controls.Add(_targetGrid, 0, 1);

            // Button Panel
            var buttonPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.LeftToRight,
                AutoSize = true,
                Padding = new Padding(0, 5, 0, 5),
            };

            _btnAddTarget = CreateButton("+ Add Target", SUCCESS_COLOR, 100, 28);
            _btnAddTarget.Click += BtnAddTarget_Click;
            buttonPanel.Controls.Add(_btnAddTarget);

            _btnRemoveTarget = CreateButton("- Remove", ERROR_COLOR, 80, 28);
            _btnRemoveTarget.Click += BtnRemoveTarget_Click;
            buttonPanel.Controls.Add(_btnRemoveTarget);

            // Spacer
            buttonPanel.Controls.Add(new Panel { Width = 20, Height = 28 });

            _btnPreview = CreateButton("Preview TXT", Color.FromArgb(80, 80, 83), 100, 28);
            _btnPreview.Click += BtnPreview_Click;
            buttonPanel.Controls.Add(_btnPreview);

            _btnUpload = CreateButton("Upload to Google Drive", ACCENT_COLOR, 160, 28);
            _btnUpload.Click += BtnUpload_Click;
            buttonPanel.Controls.Add(_btnUpload);

            mainLayout.Controls.Add(buttonPanel, 0, 2);

            // Preview TextBox
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
                Text = "Click 'Preview TXT' to see the submission file content...",
            };
            mainLayout.Controls.Add(_txtPreview, 0, 3);

            // Status Panel
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
            _targetGrid.Rows[rowIndex].Cells["Number"].Value = nextNumber;
            _targetGrid.Rows[rowIndex].Cells["Color"].Value = "Red";
            _targetGrid.Rows[rowIndex].Cells["Description"].Value = "";
            _targetGrid.Rows[rowIndex].Cells["ImagePath"].Value = "";

            // Select the new row for editing
            _targetGrid.ClearSelection();
            _targetGrid.Rows[rowIndex].Selected = true;
            _targetGrid.CurrentCell = _targetGrid.Rows[rowIndex].Cells["Description"];
        }

        private void BtnRemoveTarget_Click(object sender, EventArgs e)
        {
            if (_targetGrid.SelectedRows.Count > 0)
            {
                _targetGrid.Rows.RemoveAt(_targetGrid.SelectedRows[0].Index);
                RenumberTargets();
            }
        }

        private void RenumberTargets()
        {
            for (int i = 0; i < _targetGrid.Rows.Count; i++)
            {
                _targetGrid.Rows[i].Cells["Number"].Value = i + 1;
            }
        }

        /// <summary>
        /// Add a captured image as a new submission row.
        /// </summary>
        public void AddCapturedImage(string imagePath, string suggestedDescription = null)
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke(new Action(() => AddCapturedImage(imagePath, suggestedDescription)));
                return;
            }

            int nextNumber = _targetGrid.Rows.Count + 1;
            int rowIndex = _targetGrid.Rows.Add();
            _targetGrid.Rows[rowIndex].Cells["Number"].Value = nextNumber;
            _targetGrid.Rows[rowIndex].Cells["Color"].Value = "Red";
            _targetGrid.Rows[rowIndex].Cells["Description"].Value = suggestedDescription ?? "";
            _targetGrid.Rows[rowIndex].Cells["ImagePath"].Value = imagePath ?? "";

            _targetGrid.ClearSelection();
            _targetGrid.Rows[rowIndex].Selected = true;
            _targetGrid.CurrentCell = _targetGrid.Rows[rowIndex].Cells["Description"];
        }

        private void TargetGrid_CellClick(object sender, DataGridViewCellEventArgs e)
        {
            if (e.RowIndex < 0) return;

            // Handle image browse button click
            if (_targetGrid.Columns[e.ColumnIndex].Name == "ImageBtn")
            {
                using (var dialog = new OpenFileDialog())
                {
                    dialog.Title = "Select Target Image";
                    dialog.Filter = "Image Files|*.jpg;*.jpeg;*.png;*.bmp|All Files|*.*";

                    // Default to Task1 captures folder
                    var task1Dir = Path.Combine(
                        Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
                        "NOMAD", "Task1"
                    );
                    if (Directory.Exists(task1Dir))
                        dialog.InitialDirectory = task1Dir;

                    if (dialog.ShowDialog() == DialogResult.OK)
                    {
                        _targetGrid.Rows[e.RowIndex].Cells["ImagePath"].Value = dialog.FileName;
        }
    }
}

        }

        private void TargetGrid_CellEndEdit(object sender, DataGridViewCellEventArgs e)
        {
            // Auto-format description based on color if description is empty
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

        private void BtnPreview_Click(object sender, EventArgs e)
        {
            var content = GenerateTxtContent();
            _txtPreview.Text = string.IsNullOrEmpty(content) 
                ? "No targets defined. Add targets using the '+ Add Target' button."
                : content;
        }

        private string GenerateTxtContent()
        {
            var lines = new List<string>();

            foreach (DataGridViewRow row in _targetGrid.Rows)
            {
                var number = row.Cells["Number"].Value?.ToString();
                var color = row.Cells["Color"].Value?.ToString() ?? "";
                var description = row.Cells["Description"].Value?.ToString() ?? "";

                if (!string.IsNullOrEmpty(number) && !string.IsNullOrEmpty(description))
                {
                    // Ensure description starts with color if not already
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
            if (_targetGrid.Rows.Count == 0)
            {
                MessageBox.Show("No targets to upload. Add at least one target.", "No Targets",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            // Validate all targets have descriptions
            foreach (DataGridViewRow row in _targetGrid.Rows)
            {
                var desc = row.Cells["Description"].Value?.ToString();
                if (string.IsNullOrWhiteSpace(desc))
                {
                    MessageBox.Show($"Target {row.Cells["Number"].Value} has no description.",
                        "Missing Description", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
            }

            // Confirm upload
            var result = MessageBox.Show(
                $"Upload {_targetGrid.Rows.Count} target(s) to Google Drive?\n\nThis will create:\n- Task_1_MAD_targets.txt\n- Target images (if selected)",
                "Confirm Upload",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Question
            );

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

                // Generate and upload Task_1_MAD_targets.txt
                var txtContent = GenerateTxtContent();
                if (string.IsNullOrEmpty(txtContent))
                    throw new Exception("No targets to upload.");

                var tempTxtPath = Path.Combine(Path.GetTempPath(), "Task_1_MAD_targets.txt");
                try
                {
                    File.WriteAllText(tempTxtPath, txtContent);

                    _lblStatus.Text = "Uploading targets file...";
                    var txtFileId = await gdrive.UploadFileAsync(tempTxtPath, "Task_1_MAD_targets.txt");
                    if (string.IsNullOrEmpty(txtFileId))
                        throw new Exception("Failed to upload Task_1_MAD_targets.txt to Google Drive.");

                    // Upload images
                    var imageResults = new List<(int number, string filename, string fileId)>();
                    var errors = new List<string>();

                    foreach (DataGridViewRow row in _targetGrid.Rows)
                    {
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
                        else if (!string.IsNullOrEmpty(imagePath))
                        {
                            errors.Add($"Image not found for Target {number}: {imagePath}");
                        }
                    }

                    // Report results
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
                            sb.AppendLine($"  - {img.filename} (ID: {img.fileId})");
                        }
                    }

                    if (errors.Count > 0)
                    {
                        sb.AppendLine();
                        sb.AppendLine("Warnings:");
                        foreach (var err in errors)
                        {
                            sb.AppendLine($"  - {err}");
                        }
                    }

                    _txtPreview.Text = sb.ToString();
                    _txtPreview.ForeColor = SUCCESS_COLOR;

                    MessageBox.Show(
                        $"Successfully uploaded Task 1 submission!\n\nText file ID: {txtFileId}\nImages uploaded: {imageResults.Count}",
                        "Upload Complete",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information
                    );
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
                    "Upload Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
            finally
            {
                _btnUpload.Enabled = true;
                _progressBar.Visible = false;
            }
        }

        /// <summary>
        /// Load targets from local Task1 captures folder (auto-populate from captures).
        /// </summary>
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

                    // Find corresponding image
                    var imagePath = Path.ChangeExtension(jsonFile, ".jpg");
                    if (!File.Exists(imagePath))
                        imagePath = Path.ChangeExtension(jsonFile, ".jpeg");
                    if (!File.Exists(imagePath))
                        imagePath = Path.ChangeExtension(jsonFile, ".png");

                    int rowIndex = _targetGrid.Rows.Add();
                    _targetGrid.Rows[rowIndex].Cells["Number"].Value = targetNum++;
                    _targetGrid.Rows[rowIndex].Cells["Color"].Value = metadata?.TargetColor ?? "Red";
                    _targetGrid.Rows[rowIndex].Cells["Description"].Value = 
                        metadata?.AiDescription ?? metadata?.RelativeDescription ?? "";
                    _targetGrid.Rows[rowIndex].Cells["ImagePath"].Value = 
                        File.Exists(imagePath) ? imagePath : "";
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"Failed to load {jsonFile}: {ex.Message}");
                }
            }

            _lblStatus.Text = $"Loaded {_targetGrid.Rows.Count} captures";
            _lblStatus.ForeColor = SUCCESS_COLOR;
        }
    }
}

