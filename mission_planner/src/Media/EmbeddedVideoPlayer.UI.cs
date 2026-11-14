// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

// ============================================================
// NOMAD Embedded Video Player - UI partial
// ============================================================

using System;
using System.Drawing;
using System.IO;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using System.Windows.Forms.Integration;
using Brushes = System.Windows.Media.Brushes;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        internal ElementHost Toolbar { get; private set; }
        internal VideoToolbarView VideoControls { get; private set; }
        internal System.Windows.Controls.WrapPanel VisionOptions => VideoControls.OptionsRow;
        private ElementHost _statusPanel;

        private void InitializeUI()
        {
            BackColor = Color.Black;
            Dock = DockStyle.Fill;

            _videoBox = new PictureBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                SizeMode = PictureBoxSizeMode.Zoom,
            };
            _videoBox.DoubleClick += (s, e) => ToggleFullscreen();
            _videoBox.Paint += OnVideoPaint;

            var status = new VideoStatusView();
            _lblStatus = status.PlaybackStatus;
            _lblVisionStatus = status.TrackingStatus;
            _statusPanel = new ElementHost
            {
                Name = "VideoStatus", Dock = DockStyle.Bottom, AutoSize = true, Child = status,
                BackColor = NOMADTheme.CARD_BG, Font = NOMADTheme.Font(),
            };
            VideoControls = new VideoToolbarView();
            VideoControls.PlayButton.Click += (sender, args) => StartStream();
            VideoControls.StopButton.Click += (sender, args) => StopStream();
            VideoControls.FullscreenButton.Click += (sender, args) => ToggleFullscreen();
            VideoControls.SnapshotButton.Click += (sender, args) => TakeSnapshot();
            Toolbar = new ElementHost
            {
                Name = "VideoToolbar", Dock = DockStyle.Top, AutoSize = true, Child = VideoControls,
                BackColor = NOMADTheme.CARD_BG, Font = NOMADTheme.Font(),
            };
            Controls.Add(_videoBox);
            Controls.Add(_statusPanel);
            Controls.Add(Toolbar);
            _statusPanel.Visible = _showControls;
            Toolbar.Visible = _showControls;
        }

        private void OnVideoPaint(object sender, PaintEventArgs e)
        {
            var box = (PictureBox)sender;
            if (box.Image == null)
            {
                return;
            }

            PaintDetections(box, e.Graphics);
            PaintMotionEstimates(box, e.Graphics);
            int centerX = box.Width / 2;
            int centerY = box.Height / 2;
            const int arm = 18;
            const int gap = 5;
            e.Graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

            using (var outline = new Pen(Color.FromArgb(140, Color.Black), 4f))
            using (var foreground = new Pen(Color.FromArgb(220, Color.Cyan), 1.5f))
            {
                foreach (var pen in new[] { outline, foreground })
                {
                    e.Graphics.DrawLine(pen, centerX - arm, centerY, centerX - gap, centerY);
                    e.Graphics.DrawLine(pen, centerX + gap, centerY, centerX + arm, centerY);
                    e.Graphics.DrawLine(pen, centerX, centerY - arm, centerX, centerY - gap);
                    e.Graphics.DrawLine(pen, centerX, centerY + gap, centerX, centerY + arm);
                }
            }
        }

        public void ToggleFullscreen()
        {
            ValidateUiThread();
            if (IsDisposed)
            {
                return;
            }
            if (_fullscreenForm != null && !_fullscreenForm.IsDisposed)
            {
                if (_fullscreenBox != null && !_fullscreenBox.IsDisposed)
                {
                    _fullscreenBox.Image = null;
                }
                _fullscreenForm.Close();
                _fullscreenForm = null;
                _fullscreenBox = null;
                return;
            }

            _fullscreenForm = new NOMADForm
            {
                FormBorderStyle = FormBorderStyle.None,
                WindowState = FormWindowState.Maximized,
                BackColor = Color.Black,
                KeyPreview = true,
            };
            _fullscreenForm.KeyDown += (s, e) =>
            {
                if (e.KeyCode == Keys.Escape)
                {
                    ToggleFullscreen();
                }
            };
            VideoControls.PreviewKeyDown += OnFullscreenKey;

            _fullscreenBox = new PictureBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Black,
                SizeMode = PictureBoxSizeMode.Zoom,
            };
            _fullscreenBox.DoubleClick += (s, e) => ToggleFullscreen();
            _fullscreenBox.Paint += OnVideoPaint;
            AttachSelectionMouse(_fullscreenBox);

            _fullscreenBox.Image = _displayFrame;

            _fullscreenForm.Controls.Add(_fullscreenBox);
            if (_showControls)
            {
                _fullscreenForm.Controls.Add(_statusPanel);
                _fullscreenForm.Controls.Add(Toolbar);
            }
            _fullscreenForm.FormClosed += (sender, args) =>
            {
                _fullscreenBox.Image = null;
                VideoControls.PreviewKeyDown -= OnFullscreenKey;
                if (_showControls)
                {
                    Controls.Add(_statusPanel);
                    Controls.Add(Toolbar);
                }
                _fullscreenForm = null;
                _fullscreenBox = null;
            };
            _fullscreenForm.Show();
        }

        private void OnFullscreenKey(object sender, System.Windows.Input.KeyEventArgs args)
        {
            if (args.Key == System.Windows.Input.Key.Escape)
            { args.Handled = true; ToggleFullscreen(); }
        }

        public void TakeSnapshot() => TakeSnapshot(Environment.GetFolderPath(Environment.SpecialFolder.Desktop));

        internal string TakeSnapshot(string directory)
        {
            ValidateUiThread();
            var source = _displayFrame;
            if (source == null)
            {
                _lblStatus.Text = "No frame available";
                return null;
            }

            try
            {
                if (string.IsNullOrWhiteSpace(directory) || !Path.IsPathRooted(directory))
                { throw new ArgumentException("An absolute snapshot folder is required."); }
                string path = Path.Combine(directory, $"NOMAD_{DateTime.Now:yyyyMMdd_HHmmss_fff}_{Guid.NewGuid():N}.png");
                WriteSnapshot(source, path);
                _lblStatus.Text = $"Saved: {Path.GetFileName(path)}";
                _lblStatus.Foreground = Brushes.LimeGreen;
                return path;
            }
            catch (Exception error) when (error is IOException || error is UnauthorizedAccessException ||
                error is ExternalException || error is ArgumentException || error is NotSupportedException)
            {
                _lblStatus.Text = "Snapshot could not be saved; check the output folder";
                _lblStatus.Foreground = Brushes.Red;
                return null;
            }
        }

        internal static void WriteSnapshot(Bitmap source, string path)
        {
            bool created = false;
            try
            {
                using (var snapshot = (Bitmap)source.Clone())
                using (var output = new FileStream(path, FileMode.CreateNew, FileAccess.Write, FileShare.None))
                { created = true; snapshot.Save(output, System.Drawing.Imaging.ImageFormat.Png); }
            }
            catch
            {
                try { if (created) { File.Delete(path); } }
                catch (IOException) { }
                catch (UnauthorizedAccessException) { }
                throw;
            }
        }

        internal void UpdateInput(VideoInputProfile input)
        {
            ValidateUiThread();
            input.Validate();
            var wasPlaying = _isPlaying || _playbackRequested;
            StopStream();
            _input = input.Copy();
            _session.ConfigureTracking(_input);
            SetTrackingMode(_input.TrackingMode == "Area" ? VisualTrackingMode.Area : VisualTrackingMode.Object);
            _session.ConfigureDetection(_session.DetectionEnabled, _input.DetectorModelProfilePath);
            if (wasPlaying) { StartStream(); }
        }

        public void UpdateStreamUrl(string newUrl)
        {
            ValidateUiThread();
            var wasPlaying = _isPlaying || _playbackRequested;
            StopStream();
            _input.SourceType = "Network";
            _input.CaptureDevicePath = "";
            _input.Url = newUrl ?? "";
            if (wasPlaying)
            {
                StartStream();
            }
        }
    }
}
