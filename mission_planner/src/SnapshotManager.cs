// ============================================================
// NOMAD Snapshot Manager
// ============================================================
// Manages Task 1 snapshots with file explorer integration.
// Allows viewing, organizing, and exporting target images.
// ============================================================

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using MissionPlanner.Utilities;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// GPS position data.
    /// </summary>
    public class PositionData
    {
        public double Lat { get; set; }
        public double Lon { get; set; }
        public double Alt { get; set; }
    }

    /// <summary>
    /// Snapshot metadata structure (matches JSON file format).
    /// </summary>
    public class SnapshotMetadata
    {
        [JsonProperty("file_name")]
        public string FileName { get; set; }

        [JsonProperty("capture_time")]
        public DateTime CaptureTime { get; set; }

        [JsonProperty("position")]
        public PositionData Position { get; set; }

        [JsonProperty("heading_deg")]
        public double? HeadingDeg { get; set; }

        [JsonProperty("pitch_deg")]
        public double? PitchDeg { get; set; }

        [JsonProperty("roll_deg")]
        public double? RollDeg { get; set; }

        [JsonProperty("camera_pitch_deg")]
        public double? CameraPitchDeg { get; set; }

        [JsonProperty("alt_agl_m")]
        public double? AltAglM { get; set; }

        [JsonProperty("distance_m")]
        public double? DistanceM { get; set; }

        [JsonProperty("building_location")]
        public string BuildingLocation { get; set; }

        [JsonProperty("relative_description")]
        public string RelativeDescription { get; set; }

        [JsonProperty("target_color")]
        public string TargetColor { get; set; }
    }

    /// <summary>
    /// Snapshot metadata for display.
    /// </summary>
    public class SnapshotInfo
    {
        public string FilePath { get; set; }
        public string FileName { get; set; }
        public DateTime CaptureTime { get; set; }
        public double? Latitude { get; set; }
        public double? Longitude { get; set; }
        public double? Altitude { get; set; }
        public double? Heading { get; set; }
        public string TargetColor { get; set; }
        public string RelativeDescription { get; set; }
        public string JsonDataPath { get; set; }
    public bool HasMetadata => !string.IsNullOrEmpty(JsonDataPath) && File.Exists(JsonDataPath);
}

    /// <summary>
    /// Snapshot manager panel for viewing and organizing captures.
    /// </summary>
    public class SnapshotManager : UserControl
    {
        private readonly NOMADConfig _config;
        private readonly MissionConfig _missionConfig;
        
        private ListView _listView;
        private PictureBox _previewBox;
        private Panel _detailsPanel;
        private Label _lblDetails;
        private TextBox _txtRelativeDesc;
        private ComboBox _cmbTargetColor;
        private Button _btnSaveDescription;
        private Button _btnOpenFolder;
        private Button _btnOpenFile;
        private Button _btnRefresh;
        private Button _btnExportTargets;
        private Button _btnDeleteSelected;
        
        private ImageList _thumbnails;
        private List<SnapshotInfo> _snapshots = new List<SnapshotInfo>();
        private SnapshotInfo _selectedSnapshot;

        public SnapshotManager(NOMADConfig config, MissionConfig missionConfig)
        {
            _config = config;
            _missionConfig = missionConfig;
            
            BackColor = Color.FromArgb(30, 30, 30);
            Dock = DockStyle.Fill;
            
            InitializeComponents();
            LoadSnapshots();
        }

        private void InitializeComponents()
        {
            // Main split container
            var splitContainer = new SplitContainer
            {
                Dock = DockStyle.Fill,
                Orientation = Orientation.Vertical,
                SplitterDistance = 400,
                BackColor = Color.FromArgb(45, 45, 48),
                Panel1MinSize = 200,
                Panel2MinSize = 200,
            };

            // ============================================================
            // Left Panel: Snapshot List
            // ============================================================
            var leftPanel = new Panel
            {
                Dock = DockStyle.Fill,
                Padding = new Padding(10),
            };

            // Toolbar
            var toolbar = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                Height = 40,
                FlowDirection = FlowDirection.LeftToRight,
                BackColor = Color.FromArgb(45, 45, 48),
            };

            _btnRefresh = CreateToolButton("Refresh", RefreshSnapshots);
            _btnOpenFolder = CreateToolButton("Open Folder", OpenSnapshotFolder);
            _btnExportTargets = CreateToolButton("Export Targets", ExportTargetLocalizations);
            _btnDeleteSelected = CreateToolButton("Delete", DeleteSelectedSnapshot);
            _btnDeleteSelected.BackColor = Color.FromArgb(150, 50, 50);

            toolbar.Controls.AddRange(new Control[] { _btnRefresh, _btnOpenFolder, _btnExportTargets, _btnDeleteSelected });
            leftPanel.Controls.Add(toolbar);

            // Thumbnail list
            _thumbnails = new ImageList
            {
                ImageSize = new Size(80, 60),
                ColorDepth = ColorDepth.Depth32Bit,
            };

            _listView = new ListView
            {
                Dock = DockStyle.Fill,
                View = View.Tile,
                LargeImageList = _thumbnails,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
                BorderStyle = BorderStyle.None,
                TileSize = new Size(180, 80),
                FullRowSelect = true,
            };
            _listView.SelectedIndexChanged += ListView_SelectedIndexChanged;
            _listView.DoubleClick += ListView_DoubleClick;

            var listContainer = new Panel
            {
                Dock = DockStyle.Fill,
                Padding = new Padding(0, 5, 0, 0),
            };
            listContainer.Controls.Add(_listView);
            leftPanel.Controls.Add(listContainer);

            splitContainer.Panel1.Controls.Add(leftPanel);

            // ============================================================
            // Right Panel: Preview and Details
            // ============================================================
            var rightPanel = new Panel
            {
                Dock = DockStyle.Fill,
                Padding = new Padding(10),
                BackColor = Color.FromArgb(35, 35, 38),
            };

            // Preview image
            var previewGroup = new GroupBox
            {
                Text = "Preview",
                ForeColor = Color.FromArgb(0, 150, 200),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Dock = DockStyle.Top,
                Height = 300,
                BackColor = Color.FromArgb(40, 40, 43),
            };

            _previewBox = new PictureBox
            {
                Dock = DockStyle.Fill,
                SizeMode = PictureBoxSizeMode.Zoom,
                BackColor = Color.Black,
            };
            _previewBox.DoubleClick += (s, e) => OpenSelectedFile();
            previewGroup.Controls.Add(_previewBox);

            // Open file button
            _btnOpenFile = new Button
            {
                Text = "Open in Default App",
                Dock = DockStyle.Bottom,
                Height = 30,
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
            };
            _btnOpenFile.Click += (s, e) => OpenSelectedFile();
            previewGroup.Controls.Add(_btnOpenFile);

            rightPanel.Controls.Add(previewGroup);

            // Details panel
            _detailsPanel = new Panel
            {
                Dock = DockStyle.Fill,
                Padding = new Padding(0, 10, 0, 0),
            };

            var detailsGroup = new GroupBox
            {
                Text = "Capture Details",
                ForeColor = Color.FromArgb(0, 150, 200),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Dock = DockStyle.Top,
                Height = 200,
                BackColor = Color.FromArgb(40, 40, 43),
            };

            _lblDetails = new Label
            {
                Text = "Select a snapshot to view details",
                Font = new Font("Consolas", 9),
                ForeColor = Color.LightGray,
                Location = new Point(15, 25),
                Size = new Size(350, 170),
            };
            detailsGroup.Controls.Add(_lblDetails);
            _detailsPanel.Controls.Add(detailsGroup);

            // Target description group
            var descGroup = new GroupBox
            {
                Text = "Target Localization (Task 1)",
                ForeColor = Color.FromArgb(255, 200, 50),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Dock = DockStyle.Bottom,
                Height = 180,
                BackColor = Color.FromArgb(40, 40, 43),
            };

            var lblColor = new Label
            {
                Text = "Target Color:",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.White,
                Location = new Point(15, 25),
                AutoSize = true,
            };
            descGroup.Controls.Add(lblColor);

            _cmbTargetColor = new ComboBox
            {
                Location = new Point(110, 22),
                Size = new Size(120, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
            };
            _cmbTargetColor.Items.AddRange(new object[] { "Black", "White", "Red", "Yellow", "Blue", "Green" });
            descGroup.Controls.Add(_cmbTargetColor);

            var lblDesc = new Label
            {
                Text = "Relative Description:",
                Font = new Font("Segoe UI", 9),
                ForeColor = Color.White,
                Location = new Point(15, 55),
                AutoSize = true,
            };
            descGroup.Controls.Add(lblDesc);

            _txtRelativeDesc = new TextBox
            {
                Location = new Point(15, 75),
                Size = new Size(330, 60),
                Multiline = true,
                BackColor = Color.FromArgb(50, 50, 53),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
                ScrollBars = ScrollBars.Vertical,
            };
            _txtRelativeDesc.TextChanged += (s, e) => UpdateSaveButtonState();
            descGroup.Controls.Add(_txtRelativeDesc);

            _btnSaveDescription = new Button
            {
                Text = "Save Description",
                Location = new Point(15, 140),
                Size = new Size(150, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(50, 150, 50),
                ForeColor = Color.White,
                Enabled = false,
            };
            _btnSaveDescription.Click += BtnSaveDescription_Click;
            descGroup.Controls.Add(_btnSaveDescription);

            _detailsPanel.Controls.Add(descGroup);
            rightPanel.Controls.Add(_detailsPanel);

            splitContainer.Panel2.Controls.Add(rightPanel);

            Controls.Add(splitContainer);
        }

        private Button CreateToolButton(string text, Action onClick)
        {
            var btn = new Button
            {
                Text = text,
                Size = new Size(100, 30),
                Margin = new Padding(3),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 63),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8),
            };
            btn.Click += (s, e) => onClick();
            return btn;
        }

        private void LoadSnapshots()
        {
            _snapshots.Clear();
            _thumbnails.Images.Clear();
            _listView.Items.Clear();

            try
            {
                _missionConfig.EnsureDirectories();
                var snapshotDir = _missionConfig.SnapshotDirectory;

                if (!Directory.Exists(snapshotDir))
                    return;

                var imageExtensions = new[] { ".png", ".jpg", ".jpeg", ".bmp" };
                var files = Directory.GetFiles(snapshotDir)
                    .Where(f => imageExtensions.Contains(Path.GetExtension(f).ToLower()))
                    .OrderByDescending(f => File.GetCreationTime(f))
                    .ToList();

                int index = 0;
                foreach (var file in files)
                {
                    var snapshot = new SnapshotInfo
                    {
                        FilePath = file,
                        FileName = Path.GetFileName(file),
                        CaptureTime = File.GetCreationTime(file),
                    };

                    // Look for associated JSON metadata
                    var jsonPath = Path.ChangeExtension(file, ".json");
                    if (File.Exists(jsonPath))
                    {
                        snapshot.JsonDataPath = jsonPath;
                        LoadSnapshotMetadata(snapshot);
                    }

                    _snapshots.Add(snapshot);

                    // Create thumbnail
                    try
                    {
                        using (var img = Image.FromFile(file))
                        {
                            var thumb = img.GetThumbnailImage(80, 60, null, IntPtr.Zero);
                            _thumbnails.Images.Add(thumb);
                        }
                    }
                    catch
                    {
                        _thumbnails.Images.Add(SystemIcons.Warning.ToBitmap());
                    }

                    // Add to list view
                    var item = new ListViewItem(snapshot.FileName, index)
                    {
                        Tag = snapshot,
                        ToolTipText = $"{snapshot.CaptureTime:g}\n{snapshot.RelativeDescription ?? "No description"}",
                    };
                    _listView.Items.Add(item);

                    index++;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD SnapshotManager: Error loading snapshots - {ex.Message}");
            }
        }

        private void LoadSnapshotMetadata(SnapshotInfo snapshot)
        {
            try
            {
                var json = File.ReadAllText(snapshot.JsonDataPath);
                dynamic data = JsonConvert.DeserializeObject(json);

                snapshot.Latitude = data?.position?.lat;
                snapshot.Longitude = data?.position?.lon;
                snapshot.Altitude = data?.position?.alt;
                snapshot.Heading = data?.heading_deg;
                snapshot.TargetColor = data?.target_color;
                snapshot.RelativeDescription = data?.relative_description;

            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Error loading metadata for {snapshot.FileName} - {ex.Message}");
            }
        }

        private void RefreshSnapshots()
        {
            LoadSnapshots();
            ClearSelection();
        }

        private void ClearSelection()
        {
            _selectedSnapshot = null;
            _previewBox.Image = null;
            _lblDetails.Text = "Select a snapshot to view details";
            _txtRelativeDesc.Text = "";
            _cmbTargetColor.SelectedIndex = -1;
            _btnSaveDescription.Enabled = false;
        }

        private void ListView_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (_listView.SelectedItems.Count == 0)
            {
                ClearSelection();
                return;
            }

            _selectedSnapshot = _listView.SelectedItems[0].Tag as SnapshotInfo;
            if (_selectedSnapshot == null) return;

            // Load preview image
            try
            {
                if (_previewBox.Image != null)
                {
                    _previewBox.Image.Dispose();
                }
                _previewBox.Image = Image.FromFile(_selectedSnapshot.FilePath);
            }
            catch (Exception ex)
            {
                _previewBox.Image = null;
                Console.WriteLine($"NOMAD: Error loading preview - {ex.Message}");
            }

            // Update details
            var details = $"File: {_selectedSnapshot.FileName}\n" +
                         $"Captured: {_selectedSnapshot.CaptureTime:g}\n";

            if (_selectedSnapshot.Latitude.HasValue && _selectedSnapshot.Longitude.HasValue)
            {
                details += $"\nPosition: {_selectedSnapshot.Latitude:F6}, {_selectedSnapshot.Longitude:F6}";
                if (_selectedSnapshot.Altitude.HasValue)
                    details += $"\nAltitude: {_selectedSnapshot.Altitude:F1}m";
                if (_selectedSnapshot.Heading.HasValue)
                    details += $"\nHeading: {_selectedSnapshot.Heading:F1}°";
            }
            else
            {
                details += "\nNo GPS metadata available";
            }

            _lblDetails.Text = details;

            // Load description fields
            _txtRelativeDesc.Text = _selectedSnapshot.RelativeDescription ?? "";
            if (!string.IsNullOrEmpty(_selectedSnapshot.TargetColor))
            {
                _cmbTargetColor.SelectedItem = _selectedSnapshot.TargetColor;
            }
            else
            {
                _cmbTargetColor.SelectedIndex = -1;
            }

            UpdateSaveButtonState();
        }

        private void ListView_DoubleClick(object sender, EventArgs e)
        {
            OpenSelectedFile();
        }

        private void UpdateSaveButtonState()
        {
            _btnSaveDescription.Enabled = _selectedSnapshot != null &&
                (!string.IsNullOrEmpty(_txtRelativeDesc.Text) || _cmbTargetColor.SelectedIndex >= 0);
        }

        private void BtnSaveDescription_Click(object sender, EventArgs e)
        {
            if (_selectedSnapshot == null) return;

            try
            {
                // Update snapshot info
                _selectedSnapshot.RelativeDescription = _txtRelativeDesc.Text;
                _selectedSnapshot.TargetColor = _cmbTargetColor.SelectedItem?.ToString();

                // Save to JSON file
                var jsonPath = Path.ChangeExtension(_selectedSnapshot.FilePath, ".json");
                
                object data;
                if (File.Exists(jsonPath))
                {
                    var existingJson = File.ReadAllText(jsonPath);
                    data = JsonConvert.DeserializeObject<Dictionary<string, object>>(existingJson) 
                        ?? new Dictionary<string, object>();
                    var dict = (Dictionary<string, object>)data;
                    dict["relative_description"] = _selectedSnapshot.RelativeDescription;
                    dict["target_color"] = _selectedSnapshot.TargetColor;
                }
                else
                {
                    data = new
                    {
                        file_name = _selectedSnapshot.FileName,
                        capture_time = _selectedSnapshot.CaptureTime,
                        relative_description = _selectedSnapshot.RelativeDescription,
                        target_color = _selectedSnapshot.TargetColor,
                        position = _selectedSnapshot.Latitude.HasValue ? new
                        {
                            lat = _selectedSnapshot.Latitude,
                            lon = _selectedSnapshot.Longitude,
                            alt = _selectedSnapshot.Altitude,
                        } : null,
                        heading_deg = _selectedSnapshot.Heading,
                    };
                }

                var json = JsonConvert.SerializeObject(data, Formatting.Indented);
                File.WriteAllText(jsonPath, json);
                _selectedSnapshot.JsonDataPath = jsonPath;

                // Also update mission config detected targets
                var existingTarget = _missionConfig.DetectedTargets
                    .FirstOrDefault(t => t.SnapshotPath == _selectedSnapshot.FilePath);

                if (existingTarget != null)
                {
                    existingTarget.RelativeDescription = _selectedSnapshot.RelativeDescription;
                    existingTarget.Color = _selectedSnapshot.TargetColor;
                }
                else
                {
                    _missionConfig.DetectedTargets.Add(new DetectedTarget
                    {
                        Id = $"T{_missionConfig.DetectedTargets.Count + 1}",
                        DetectedAt = _selectedSnapshot.CaptureTime,
                        EstimatedLocation = _selectedSnapshot.Latitude.HasValue 
                            ? new GpsPoint(_selectedSnapshot.Latitude.Value, _selectedSnapshot.Longitude.Value, _selectedSnapshot.Altitude)
                            : null,
                        Color = _selectedSnapshot.TargetColor,
                        RelativeDescription = _selectedSnapshot.RelativeDescription,
                        SnapshotPath = _selectedSnapshot.FilePath,
                    });
                }

                _missionConfig.Save();

                CustomMessageBox.Show("Description saved successfully!", "Success");
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error saving description: {ex.Message}", "Error");
            }
        }

        private void OpenSnapshotFolder()
        {
            try
            {
                _missionConfig.EnsureDirectories();
                var dir = _missionConfig.SnapshotDirectory;

                if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                {
                    Process.Start("explorer.exe", dir);
                }
                else if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
                {
                    Process.Start("xdg-open", dir);
                }
                else if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
                {
                    Process.Start("open", dir);
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error opening folder: {ex.Message}", "Error");
            }
        }

        private void OpenSelectedFile()
        {
            if (_selectedSnapshot == null) return;

            try
            {
                if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                {
                    Process.Start(new ProcessStartInfo
                    {
                        FileName = _selectedSnapshot.FilePath,
                        UseShellExecute = true,
                    });
                }
                else if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
                {
                    Process.Start("xdg-open", _selectedSnapshot.FilePath);
                }
                else if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
                {
                    Process.Start("open", _selectedSnapshot.FilePath);
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error opening file: {ex.Message}", "Error");
            }
        }

        private void DeleteSelectedSnapshot()
        {
            if (_selectedSnapshot == null) return;

            var result = CustomMessageBox.Show(
                $"Delete snapshot '{_selectedSnapshot.FileName}'?\n\nThis cannot be undone.",
                "Confirm Delete",
                CustomMessageBox.MessageBoxButtons.YesNo);

            if (result == CustomMessageBox.DialogResult.Yes)
            {
                try
                {
                    // Delete image file
                    if (File.Exists(_selectedSnapshot.FilePath))
                    {
                        File.Delete(_selectedSnapshot.FilePath);
                    }

                    // Delete metadata file
                    if (File.Exists(_selectedSnapshot.JsonDataPath))
                    {
                        File.Delete(_selectedSnapshot.JsonDataPath);
                    }

                    // Remove from mission config
                    _missionConfig.DetectedTargets.RemoveAll(t => t.SnapshotPath == _selectedSnapshot.FilePath);
                    _missionConfig.Save();

                    RefreshSnapshots();
                }
                catch (Exception ex)
                {
                    CustomMessageBox.Show($"Error deleting file: {ex.Message}", "Error");
                }
            }
        }

        private void ExportTargetLocalizations()
        {
            try
            {
                _missionConfig.ExportTargetLocalizations();
                
                // Ask if user wants to open the file
                var result = CustomMessageBox.Show(
                    $"Target localizations exported to:\n{_missionConfig.TargetLocalizationFile}\n\nOpen file?",
                    "Export Complete",
                    CustomMessageBox.MessageBoxButtons.YesNo);

                if (result == CustomMessageBox.DialogResult.Yes)
                {
                    if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                    {
                        Process.Start(new ProcessStartInfo
                        {
                            FileName = _missionConfig.TargetLocalizationFile,
                            UseShellExecute = true,
                        });
                    }
                    else
                    {
                        Process.Start("xdg-open", _missionConfig.TargetLocalizationFile);
                    }
                }
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error exporting targets: {ex.Message}", "Error");
            }
        }

        /// <summary>
        /// Add a new snapshot from capture.
        /// </summary>
        public void AddSnapshot(string filePath, double? lat = null, double? lon = null, 
            double? alt = null, double? heading = null)
        {
            try
            {
                // Copy to snapshot directory if not already there
                var destDir = _missionConfig.SnapshotDirectory;
                _missionConfig.EnsureDirectories();

                string destPath;
                if (!filePath.StartsWith(destDir, StringComparison.OrdinalIgnoreCase))
                {
                    var fileName = $"NOMAD_Snapshot_{DateTime.Now:yyyyMMdd_HHmmss}{Path.GetExtension(filePath)}";
                    destPath = Path.Combine(destDir, fileName);
                    File.Copy(filePath, destPath, true);
                }
                else
                {
                    destPath = filePath;
                }

                // Save metadata
                var jsonPath = Path.ChangeExtension(destPath, ".json");
                var metadata = new
                {
                    file_name = Path.GetFileName(destPath),
                    capture_time = DateTime.Now,
                    position = lat.HasValue ? new { lat, lon, alt } : null,
                    heading_deg = heading,
                };
                File.WriteAllText(jsonPath, JsonConvert.SerializeObject(metadata, Formatting.Indented));

                // Refresh list
                RefreshSnapshots();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD SnapshotManager: Error adding snapshot - {ex.Message}");
            }
        }
    }
}
