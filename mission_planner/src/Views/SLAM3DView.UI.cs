// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// SLAM3DView.UI.cs - UI component initialization
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;

namespace NOMAD.MissionPlanner
{
    public partial class SLAM3DView
    {
        // ==================== UI Initialization ====================

        private void InitializeComponents()
        {
            BackColor = Color.FromArgb(30, 30, 33);
            Dock = DockStyle.Fill;

            var mainLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 2,
                BackColor = Color.FromArgb(30, 30, 33),
            };
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            mainLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 170));

            var bottomLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                BackColor = Color.FromArgb(30, 30, 33),
                Margin = new Padding(0),
                Padding = new Padding(0),
            };
            bottomLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 76));
            bottomLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 24));

            // OpenGL viewport
            try
            {
                _glControl = new GLControl(new GraphicsMode(32, 24, 0, 4))
                {
                    Dock = DockStyle.Fill,
                    VSync = false,
                };
                _glControl.Load += GlControl_Load;
                _glControl.Paint += GlControl_Paint;
                _glControl.Resize += GlControl_Resize;
                _glControl.MouseDown += GlControl_MouseDown;
                _glControl.MouseUp += GlControl_MouseUp;
                _glControl.MouseMove += GlControl_MouseMove;
                _glControl.MouseWheel += GlControl_MouseWheel;
                mainLayout.Controls.Add(_glControl, 0, 0);
            }
            catch (Exception ex)
            {
                var errorLabel = new Label
                {
                    Text = $"OpenGL init failed: {ex.Message}\nEnsure OpenTK.dll is available.",
                    Dock = DockStyle.Fill,
                    ForeColor = Color.Red,
                    Font = new Font("Consolas", 11),
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                mainLayout.Controls.Add(errorLabel, 0, 0);
            }

            // Control panel
            _controlPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(40, 40, 45),
                Padding = new Padding(10),
                AutoScroll = true,
            };

            int x = 10, y = 8;

            _btnToggleCamera = CreateButton("View: TPV", x, y, 100, 28, Color.FromArgb(0, 122, 204));
            _btnToggleCamera.Click += BtnToggleCamera_Click;
            _controlPanel.Controls.Add(_btnToggleCamera);
            x += 105;

            _btnResetView = CreateButton("Reset View", x, y, 85, 28, Color.FromArgb(60, 60, 65));
            _btnResetView.Click += BtnResetView_Click;
            _controlPanel.Controls.Add(_btnResetView);
            x += 90;

            _btnClearMesh = CreateButton("Clear Mesh", x, y, 85, 28, Color.FromArgb(180, 60, 60));
            _btnClearMesh.Click += BtnClearMesh_Click;
            _controlPanel.Controls.Add(_btnClearMesh);
            x += 95;

            _btnResetImuBiases = CreateButton("Reset IMU", x, y, 85, 28, Color.FromArgb(120, 80, 0));
            _btnResetImuBiases.Click += BtnResetImuBiases_Click;
            _controlPanel.Controls.Add(_btnResetImuBiases);
            x += 95;

            _chkShowGrid = CreateCheckBox("Grid", x, y + 4, true);
            _controlPanel.Controls.Add(_chkShowGrid);
            x += 55;

            _chkShowTrajectory = CreateCheckBox("Trail", x, y + 4, true);
            _controlPanel.Controls.Add(_chkShowTrajectory);
            x += 55;

            _chkAutoUpdate = CreateCheckBox("Auto", x, y + 4, true);
            _chkAutoUpdate.CheckedChanged += (s, e) => _autoUpdateEnabled = _chkAutoUpdate.Checked;
            _controlPanel.Controls.Add(_chkAutoUpdate);
            x += 55;

            _controlPanel.Controls.Add(CreateLabel("Mesh: Voxel", x, y + 6));

            // Second row: drone config
            y += 34;
            x = 10;

            _controlPanel.Controls.Add(CreateLabel("Drone (cm)  L:", x, y + 3));
            x += 82;
            _numLength = CreateNumericUpDown(x, y, 45, 1, 200, (decimal)_config.DroneLengthCm);
            _numLength.ValueChanged += (s, e) => { _config.DroneLengthCm = (float)_numLength.Value; _config.Save(); };
            _controlPanel.Controls.Add(_numLength);
            x += 50;

            _controlPanel.Controls.Add(CreateLabel("W:", x, y + 3));
            x += 18;
            _numWidth = CreateNumericUpDown(x, y, 45, 1, 200, (decimal)_config.DroneWidthCm);
            _numWidth.ValueChanged += (s, e) => { _config.DroneWidthCm = (float)_numWidth.Value; _config.Save(); };
            _controlPanel.Controls.Add(_numWidth);
            x += 50;

            _controlPanel.Controls.Add(CreateLabel("H:", x, y + 3));
            x += 18;
            _numHeight = CreateNumericUpDown(x, y, 45, 1, 100, (decimal)_config.DroneHeightCm);
            _numHeight.ValueChanged += (s, e) => { _config.DroneHeightCm = (float)_numHeight.Value; _config.Save(); };
            _controlPanel.Controls.Add(_numHeight);
            x += 55;

            _controlPanel.Controls.Add(CreateLabel("Type:", x, y + 3));
            x += 35;
            _combDroneType = new ComboBox
            {
                Location = new Point(x, y),
                Size = new Size(85, 20),
                DropDownStyle = ComboBoxStyle.DropDownList,
                ForeColor = Color.White,
                BackColor = Color.FromArgb(60, 60, 60),
            };
            _combDroneType.Items.AddRange(new[] { "Tricopter", "Quadcopter" });
            _combDroneType.SelectedItem = _config.DroneFrameType;
            _combDroneType.SelectedIndexChanged += (s, e) =>
            {
                _config.DroneFrameType = _combDroneType.SelectedItem?.ToString() ?? "Quadcopter";
                _config.Save();
                _glControl?.Invalidate();
            };
            _controlPanel.Controls.Add(_combDroneType);
            x += 90;

            _controlPanel.Controls.Add(CreateLabel("Hdg Offset:", x, y + 3));
            x += 68;
            _numHeadingOffset = CreateNumericUpDown(x, y, 55, -180, 180, (decimal)_config.SlamHeadingOffsetDeg);
            _numHeadingOffset.ValueChanged += (s, e) => { _config.SlamHeadingOffsetDeg = (float)_numHeadingOffset.Value; _config.Save(); };
            _controlPanel.Controls.Add(_numHeadingOffset);
            x += 63;

            _controlPanel.Controls.Add(CreateLabel("FOV:", x, y + 3));
            x += 30;
            _numFov = CreateNumericUpDown(x, y, 50, 30, 140, (decimal)GetClampedFovDegrees());
            _numFov.ValueChanged += (s, e) =>
            {
                _config.SlamCameraFovDeg = (float)_numFov.Value;
                _config.Save();
                _glControl?.Invalidate();
            };
            _controlPanel.Controls.Add(_numFov);

            x += 60;
            _controlPanel.Controls.Add(CreateLabel("Radius:", x, y + 3));
            x += 45;
            _numMapRadius = CreateNumericUpDown(x, y, 50, 1, 20, (decimal)Math.Max(1f, Math.Min(20f, _config.SlamMapRadiusM)));
            _numMapRadius.ValueChanged += (s, e) =>
            {
                _config.SlamMapRadiusM = (float)_numMapRadius.Value;
                _config.Save();
            };
            _controlPanel.Controls.Add(_numMapRadius);

            // Third row: area map controls
            y += 26;
            x = 10;

            _lblMapPath = CreateLabel("Area Map:", x, y + 3);
            _controlPanel.Controls.Add(_lblMapPath);
            x += 65;

            _txtMapPath = new TextBox
            {
                Location = new Point(x, y),
                Size = new Size(300, 22),
                Text = "~/NOMAD/data/area_maps/slam_area_map.area",
                BackColor = Color.FromArgb(50, 50, 55),
                ForeColor = Color.White,
                BorderStyle = BorderStyle.FixedSingle,
            };
            _controlPanel.Controls.Add(_txtMapPath);
            x += 310;

            _btnSaveMap = CreateButton("Save", x, y, 58, 24, Color.FromArgb(0, 122, 204));
            _btnSaveMap.Click += async (s, e) => await SaveAreaMapAsync();
            _controlPanel.Controls.Add(_btnSaveMap);
            x += 62;

            _btnLoadMap = CreateButton("Load", x, y, 58, 24, Color.FromArgb(60, 60, 65));
            _btnLoadMap.Click += async (s, e) => await LoadAreaMapAsync();
            _controlPanel.Controls.Add(_btnLoadMap);
            x += 62;

            _btnRelocalizeMap = CreateButton("Relocalize", x, y, 80, 24, Color.FromArgb(150, 90, 0));
            _btnRelocalizeMap.Click += async (s, e) => await RelocalizeAreaMapAsync();
            _controlPanel.Controls.Add(_btnRelocalizeMap);
            x += 86;

            _btnCenterOnPose = CreateButton("Center", x, y, 65, 24, Color.FromArgb(60, 60, 65));
            _btnCenterOnPose.Click += (s, e) => CenterOrbitOnCurrentPose();
            _controlPanel.Controls.Add(_btnCenterOnPose);

            // Third row: status
            y += 30;
            _lblStatus = new Label
            {
                Text = "Status: Connecting...",
                Location = new Point(10, y),
                ForeColor = Color.FromArgb(200, 200, 200),
                AutoSize = true,
                Font = new Font("Consolas", 9),
            };
            _controlPanel.Controls.Add(_lblStatus);

            _lblStats = new Label
            {
                Text = "Mesh: 0 voxels",
                Location = new Point(300, y),
                ForeColor = Color.FromArgb(150, 150, 150),
                AutoSize = true,
                Font = new Font("Consolas", 9),
            };
            _controlPanel.Controls.Add(_lblStats);

            y += 18;
            _lblPerceptionStatus = new Label
            {
                Text = "HSV: -- | Servo: --",
                Location = new Point(10, y),
                ForeColor = Color.FromArgb(150, 150, 150),
                AutoSize = true,
                Font = new Font("Consolas", 9),
            };
            _controlPanel.Controls.Add(_lblPerceptionStatus);

            _statusLogPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(34, 34, 38),
                Padding = new Padding(8),
            };

            var lblStatusLogTitle = new Label
            {
                Text = "Status Log",
                Dock = DockStyle.Top,
                Height = 20,
                ForeColor = Color.LightSteelBlue,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleLeft,
            };

            _txtStatusLog = new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ScrollBars = ScrollBars.Vertical,
                ReadOnly = true,
                BackColor = Color.FromArgb(24, 24, 26),
                ForeColor = Color.Gainsboro,
                BorderStyle = BorderStyle.FixedSingle,
                Font = new Font("Consolas", 8),
            };

            _statusLogPanel.Controls.Add(_txtStatusLog);
            _statusLogPanel.Controls.Add(lblStatusLogTitle);

            bottomLayout.Controls.Add(_controlPanel, 0, 0);
            bottomLayout.Controls.Add(_statusLogPanel, 1, 0);
            mainLayout.Controls.Add(bottomLayout, 0, 1);
            Controls.Add(mainLayout);

            UpdateCameraToggleLabel();

            AppendStatusLogSafe("SLAM status log initialized");

            // Render timer ~30fps
            _renderTimer = new System.Windows.Forms.Timer { Interval = 33 };
            _renderTimer.Tick += (s, e) => { if (_glControl != null && _glInitialized) _glControl.Invalidate(); };
            _renderTimer.Start();
        }

        private static Button CreateButton(string text, int x, int y, int w, int h, Color bg)
        {
            return new Button
            {
                Text = text,
                Location = new Point(x, y),
                Size = new Size(w, h),
                FlatStyle = FlatStyle.Flat,
                BackColor = bg,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
            };
        }

        private static CheckBox CreateCheckBox(string text, int x, int y, bool chk)
        {
            return new CheckBox
            {
                Text = text,
                Location = new Point(x, y),
                ForeColor = Color.White,
                AutoSize = true,
                Checked = chk,
                Font = new Font("Segoe UI", 9),
            };
        }

        private static Label CreateLabel(string text, int x, int y)
        {
            return new Label
            {
                Text = text,
                Location = new Point(x, y),
                ForeColor = Color.FromArgb(180, 180, 180),
                AutoSize = true,
                Font = new Font("Segoe UI", 8.5f),
            };
        }

        private static NumericUpDown CreateNumericUpDown(int x, int y, int w, int min, int max, decimal val)
        {
            return new NumericUpDown
            {
                Location = new Point(x, y),
                Size = new Size(w, 22),
                Minimum = min,
                Maximum = max,
                Value = Math.Min(max, Math.Max(min, val)),
                DecimalPlaces = 0,
                BackColor = Color.FromArgb(50, 50, 55),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 8.5f),
                BorderStyle = BorderStyle.FixedSingle,
            };
        }

        private float GetClampedFovDegrees()
        {
            float fov = Math.Max(30f, Math.Min(140f, _config.SlamCameraFovDeg));
            _cameraController.FieldOfView = fov;
            return fov;
        }

        private void ApplyProjectionMatrix(int width, int height)
        {
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();
            float aspect = (float)width / height;
            var proj = Matrix4.CreatePerspectiveFieldOfView(
                MathHelper.DegreesToRadians(GetClampedFovDegrees()), aspect, 0.05f, 500f);
            GL.LoadMatrix(ref proj);
            GL.MatrixMode(MatrixMode.Modelview);
        }
    }
}
