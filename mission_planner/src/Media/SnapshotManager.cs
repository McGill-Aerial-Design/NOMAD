// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Snapshot Manager
// ============================================================
// Basic, task-agnostic image snapshot browser: lists captured
// images from the snapshot directory, previews the selection,
// and opens/deletes files. Adapt as needed.
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

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Minimal snapshot record (file on disk).
    /// </summary>
    public class SnapshotInfo
    {
        public string FilePath { get; set; }
        public string FileName { get; set; }
        public DateTime CaptureTime { get; set; }
    }

    /// <summary>
    /// Basic snapshot browser: thumbnails, preview, open, delete.
    /// </summary>
    public class SnapshotManager : UserControl
    {
        private static readonly string[] ImageExtensions = { ".jpg", ".jpeg", ".png", ".bmp" };

        private readonly string _snapshotDir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "NOMAD", "Snapshots");

        private ListView _listView;
        private PictureBox _previewBox;
        private Label _lblDetails;
        private ImageList _thumbnails;
        private readonly List<SnapshotInfo> _snapshots = new List<SnapshotInfo>();
        private SnapshotInfo _selected;

        public SnapshotManager(NOMADConfig config = null)
        {
            BackColor = Color.FromArgb(30, 30, 30);
            Dock = DockStyle.Fill;

            InitializeComponents();
            LoadSnapshots();
        }

        private void InitializeComponents()
        {
            var split = new SplitContainer
            {
                Dock = DockStyle.Fill,
                Orientation = Orientation.Vertical,
                SplitterDistance = 400,
                BackColor = Color.FromArgb(45, 45, 48),
                Panel1MinSize = 200,
                Panel2MinSize = 200,
            };

            // Left: toolbar + thumbnail list
            var leftPanel = new Panel { Dock = DockStyle.Fill, Padding = new Padding(10) };

            var toolbar = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                Height = 40,
                FlowDirection = FlowDirection.LeftToRight,
                BackColor = Color.FromArgb(45, 45, 48),
            };
            var btnRefresh = CreateToolButton("Refresh", LoadSnapshots);
            var btnOpenFolder = CreateToolButton("Open Folder", OpenSnapshotFolder);
            var btnDelete = CreateToolButton("Delete", DeleteSelected);
            btnDelete.BackColor = Color.FromArgb(150, 50, 50);
            toolbar.Controls.AddRange(new Control[] { btnRefresh, btnOpenFolder, btnDelete });
            leftPanel.Controls.Add(toolbar);

            _thumbnails = new ImageList { ImageSize = new Size(80, 60), ColorDepth = ColorDepth.Depth32Bit };
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
            _listView.SelectedIndexChanged += (s, e) => ShowSelected();
            _listView.DoubleClick += (s, e) => OpenSelectedFile();

            var listContainer = new Panel { Dock = DockStyle.Fill, Padding = new Padding(0, 5, 0, 0) };
            listContainer.Controls.Add(_listView);
            leftPanel.Controls.Add(listContainer);
            split.Panel1.Controls.Add(leftPanel);

            // Right: preview + details
            var rightPanel = new Panel { Dock = DockStyle.Fill, Padding = new Padding(10), BackColor = Color.FromArgb(35, 35, 38) };

            var previewGroup = new GroupBox
            {
                Text = "Preview",
                ForeColor = Color.FromArgb(0, 150, 200),
                Font = new Font("Segoe UI", 10, FontStyle.Bold),
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(40, 40, 43),
            };
            _previewBox = new PictureBox { Dock = DockStyle.Fill, SizeMode = PictureBoxSizeMode.Zoom, BackColor = Color.Black };
            _previewBox.DoubleClick += (s, e) => OpenSelectedFile();
            previewGroup.Controls.Add(_previewBox);

            var btnOpenFile = new Button
            {
                Text = "Open in Default App",
                Dock = DockStyle.Bottom,
                Height = 30,
                FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.ACCENT,
                ForeColor = Color.White,
            };
            btnOpenFile.Click += (s, e) => OpenSelectedFile();
            previewGroup.Controls.Add(btnOpenFile);

            _lblDetails = new Label
            {
                Text = "Select a snapshot to view details",
                Font = new Font("Consolas", 9),
                ForeColor = Color.LightGray,
                Dock = DockStyle.Bottom,
                Height = 60,
            };

            rightPanel.Controls.Add(previewGroup);
            rightPanel.Controls.Add(_lblDetails);
            split.Panel2.Controls.Add(rightPanel);

            Controls.Add(split);
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

        /// <summary>
        /// Scan the snapshot directory and populate the thumbnail list.
        /// </summary>
        public void LoadSnapshots()
        {
            _snapshots.Clear();
            _listView.Items.Clear();
            foreach (Image img in _thumbnails.Images) img?.Dispose();
            _thumbnails.Images.Clear();
            ClearSelection();

            try
            {
                if (!Directory.Exists(_snapshotDir)) return;

                var files = Directory.EnumerateFiles(_snapshotDir)
                    .Where(f => ImageExtensions.Contains(Path.GetExtension(f).ToLowerInvariant()))
                    .OrderByDescending(File.GetLastWriteTime);

                foreach (var file in files)
                {
                    var info = new SnapshotInfo
                    {
                        FilePath = file,
                        FileName = Path.GetFileName(file),
                        CaptureTime = File.GetLastWriteTime(file),
                    };
                    _snapshots.Add(info);

                    int imageIndex = -1;
                    try
                    {
                        using (var src = Image.FromFile(file))
                        {
                            _thumbnails.Images.Add(new Bitmap(src, _thumbnails.ImageSize));
                            imageIndex = _thumbnails.Images.Count - 1;
                        }
                    }
                    catch (Exception ex)
                    {
                        Log.Error($"Snapshot thumbnail failed for {info.FileName} - {ex.Message}");
                    }

                    _listView.Items.Add(new ListViewItem(info.FileName, imageIndex) { Tag = info });
                }
            }
            catch (Exception ex)
            {
                Log.Error($"SnapshotManager: failed to load snapshots - {ex.Message}");
            }
        }

        private void ClearSelection()
        {
            _selected = null;
            _previewBox.Image?.Dispose();
            _previewBox.Image = null;
            if (_lblDetails != null) _lblDetails.Text = "Select a snapshot to view details";
        }

        private void ShowSelected()
        {
            if (_listView.SelectedItems.Count == 0)
            {
                ClearSelection();
                return;
            }

            _selected = _listView.SelectedItems[0].Tag as SnapshotInfo;
            if (_selected == null) return;

            try
            {
                _previewBox.Image?.Dispose();
                _previewBox.Image = Image.FromFile(_selected.FilePath);
            }
            catch (Exception ex)
            {
                _previewBox.Image = null;
                Log.Error($"Snapshot preview failed - {ex.Message}");
            }

            _lblDetails.Text = $"File: {_selected.FileName}\nCaptured: {_selected.CaptureTime:g}";
        }

        private void OpenSnapshotFolder()
        {
            try
            {
                Directory.CreateDirectory(_snapshotDir);
                OpenWithShell(_snapshotDir);
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error opening folder: {ex.Message}", "Error");
            }
        }

        private void OpenSelectedFile()
        {
            if (_selected == null) return;
            try
            {
                OpenWithShell(_selected.FilePath);
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error opening file: {ex.Message}", "Error");
            }
        }

        private static void OpenWithShell(string path)
        {
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                Process.Start(new ProcessStartInfo { FileName = path, UseShellExecute = true });
            else if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
                Process.Start("xdg-open", path);
            else if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
                Process.Start("open", path);
        }

        private void DeleteSelected()
        {
            if (_selected == null) return;

            var result = CustomMessageBox.Show(
                $"Delete snapshot '{_selected.FileName}'?\n\nThis cannot be undone.",
                "Confirm Delete",
                CustomMessageBox.MessageBoxButtons.YesNo);

            if (result != CustomMessageBox.DialogResult.Yes) return;

            try
            {
                _previewBox.Image?.Dispose();
                _previewBox.Image = null;
                if (File.Exists(_selected.FilePath)) File.Delete(_selected.FilePath);
                LoadSnapshots();
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"Error deleting file: {ex.Message}", "Error");
            }
        }

        /// <summary>
        /// Copy an image into the snapshot directory and refresh the list.
        /// </summary>
        public void AddSnapshot(string filePath)
        {
            try
            {
                if (string.IsNullOrEmpty(filePath) || !File.Exists(filePath)) return;
                Directory.CreateDirectory(_snapshotDir);

                var fileName = $"NOMAD_Snapshot_{DateTime.Now:yyyyMMdd_HHmmss}{Path.GetExtension(filePath)}";
                var destPath = Path.Combine(_snapshotDir, fileName);
                if (!string.Equals(Path.GetFullPath(filePath), Path.GetFullPath(destPath), StringComparison.OrdinalIgnoreCase))
                {
                    File.Copy(filePath, destPath, true);
                }
                LoadSnapshots();
            }
            catch (Exception ex)
            {
                Log.Error($"SnapshotManager: error adding snapshot - {ex.Message}");
            }
        }
    }
}
