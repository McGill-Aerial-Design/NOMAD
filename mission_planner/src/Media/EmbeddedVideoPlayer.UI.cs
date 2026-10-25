// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

// ============================================================
// NOMAD Embedded Video Player - UI partial
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class EmbeddedVideoPlayer
    {
        internal TableLayoutPanel Toolbar { get; private set; }

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

            _lblStatus = new Label
            {
                Text = "Ready - Click Play",
                Dock = DockStyle.Bottom,
                Height = 22,
                ForeColor = NOMADTheme.TEXT_SECONDARY,
                BackColor = NOMADTheme.CARD_BG,
                TextAlign = ContentAlignment.MiddleCenter,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
            };

            var ctrlPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill, AutoSize = true, Margin = Padding.Empty,
                AutoSizeMode = AutoSizeMode.GrowAndShrink, WrapContents = true,
                BackColor = NOMADTheme.CARD_BG, Padding = new Padding(5),
            };
            ctrlPanel.Controls.AddRange(new Control[]
            {
                CreateVideoButton("\uE768", "Play", (sender, args) => StartStream()),
                CreateVideoButton("\uE71A", "Stop", (sender, args) => StopStream()),
                CreateVideoButton("\uE740", "Full screen", (sender, args) => ToggleFullscreen()),
                CreateVideoButton("\uE722", "Take snapshot", (sender, args) => TakeSnapshot()),
            });

            Toolbar = new TableLayoutPanel
            {
                Name = "VideoToolbar", Dock = DockStyle.Top, AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink, ColumnCount = 2, RowCount = 1,
                BackColor = NOMADTheme.CARD_BG, Margin = Padding.Empty,
            };
            Toolbar.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            Toolbar.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            Toolbar.Controls.Add(ctrlPanel, 0, 0);
            Controls.Add(_videoBox);
            Controls.Add(_lblStatus);
            Controls.Add(Toolbar);
            _lblStatus.Visible = _showControls;
            Toolbar.Visible = _showControls;
        }

        private Button CreateVideoButton(string symbol, string name, EventHandler onClick)
        {
            var button = ControlFactory.Button(symbol, onClick, width: 36, height: 30);
            button.AccessibleName = name;
            button.Font = new Font("Segoe MDL2 Assets", NOMADTheme.SIZE_HEADING);
            button.Margin = new Padding(0, 0, 4, 4);
            _videoToolTips.SetToolTip(button, name);
            return button;
        }

        private void OnVideoPaint(object sender, PaintEventArgs e)
        {
            var box = (PictureBox)sender;
            if (box.Image == null)
            {
                return;
            }

            PaintDetections(box, e.Graphics);
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
            _fullscreenForm.Show();
        }

        public void TakeSnapshot()
        {
            ValidateUiThread();
            var source = _displayFrame;
            if (source == null)
            {
                _lblStatus.Text = "No frame available";
                return;
            }

            var path = System.IO.Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.Desktop),
                $"NOMAD_{DateTime.Now:yyyyMMdd_HHmmss}.png");
            using (var snapshot = (Bitmap)source.Clone())
            {
                snapshot.Save(path);
            }
            _lblStatus.Text = $"Saved: {System.IO.Path.GetFileName(path)}";
            _lblStatus.ForeColor = Color.LimeGreen;
        }

        internal void UpdateInput(VideoInputProfile input)
        {
            ValidateUiThread();
            input.Validate();
            var wasPlaying = _isPlaying;
            StopStream();
            _input = input.Copy();
            _session.ConfigureDetection(_session.DetectionEnabled, _input.DetectorModelProfilePath);
            if (wasPlaying) { StartStream(); }
        }

        public void UpdateStreamUrl(string newUrl)
        {
            ValidateUiThread();
            var wasPlaying = _isPlaying;
            StopStream();
            _input.Url = newUrl ?? "";
            if (wasPlaying)
            {
                StartStream();
            }
        }
    }
}
