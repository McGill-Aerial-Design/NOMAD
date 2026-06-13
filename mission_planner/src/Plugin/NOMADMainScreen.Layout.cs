// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADMainScreen.Layout.cs - Sidebar and content-area layout
// ============================================================
// Builds the legacy (hardcoded) sidebar, sidebar buttons, section
// separators, and the header/content panels. View switching and
// the module-driven sidebar live in the other partials.
// ============================================================

using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using NOMAD.MissionPlanner.Core;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADMainScreen
    {
        // ============================================================
        // UI Initialization
        // ============================================================

        private void InitializeUI()
        {
            this.BackColor = CONTENT_BG;
            this.Dock = DockStyle.Fill;

            // IMPORTANT: In Windows Forms, docking order matters!
            // Controls are docked in REVERSE order of addition.
            // So we add content area FIRST (fills remaining), then sidebar (docks left).
            CreateContentArea();

            // If a module host with descriptors is present, build the sidebar from
            // those modules; otherwise fall back to the built-in hardcoded sidebar.
            _descriptors = _moduleHost != null ? _moduleHost.GetViewDescriptors().ToList() : null;
            _isModuleMode = _descriptors != null && _descriptors.Count > 0;
            if (_isModuleMode)
                CreateSidebarFromDescriptors();
            else
                CreateSidebar();
        }

        private void CreateSidebar()
        {
            _sidebarPanel = new Panel
            {
                Dock = DockStyle.Left,
                Width = SIDEBAR_WIDTH,
                BackColor = SIDEBAR_BG,
                Padding = new Padding(0),
            };

            // Logo/Title area at top
            var logoPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 45,
                BackColor = Color.FromArgb(8, 8, 10),
                Padding = new Padding(12, 8, 12, 5),
            };

            var logoLabel = new Label
            {
                Text = "NOMAD",
                Font = new Font("Segoe UI", 14, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(12, 10),
                AutoSize = true,
            };
            logoPanel.Controls.Add(logoLabel);

            // Active-profile indicator (set by scripts/profile.py on profile load).
            var profileText = string.IsNullOrWhiteSpace(_config?.ActiveProfile) ? "dev" : _config.ActiveProfile;
            _profileLabel = new Label
            {
                Text = "● " + profileText,
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                ForeColor = Color.FromArgb(120, 200, 120),
                Dock = DockStyle.Right,
                TextAlign = ContentAlignment.MiddleRight,
                AutoSize = false,
                Width = 110,
            };
            logoPanel.Controls.Add(_profileLabel);
            var navPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                Padding = new Padding(5),
                AutoScroll = true,
                BackColor = SIDEBAR_BG,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
            };

            // Dashboard button (primary)
            _btnDashboard = CreateSidebarButton("Dashboard");
            _btnDashboard.Click += (s, e) => ShowView("Dashboard");
            navPanel.Controls.Add(_btnDashboard);

            // Separator
            navPanel.Controls.Add(CreateSeparatorLabel("SAFETY"));

            // Boundaries button (important for competition)
            _btnBoundaries = CreateSidebarButton("Flight Boundaries");
            _btnBoundaries.Click += (s, e) => ShowView("Boundaries");
            navPanel.Controls.Add(_btnBoundaries);

            // Separator
            navPanel.Controls.Add(CreateSeparatorLabel("TOOLS"));

            // Video button
            _btnVideo = CreateSidebarButton("Video Feed");
            _btnVideo.Click += (s, e) => ShowView("Video");
            navPanel.Controls.Add(_btnVideo);

            // Terminal button
            _btnTerminal = CreateSidebarButton("Terminal");
            _btnTerminal.Click += (s, e) => ShowView("Terminal");
            navPanel.Controls.Add(_btnTerminal);

            // Health button
            _btnHealth = CreateSidebarButton("System Health");
            _btnHealth.Click += (s, e) => ShowView("Health");
            navPanel.Controls.Add(_btnHealth);

            // Links button
            _btnLinks = CreateSidebarButton("Link Status");
            _btnLinks.Click += (s, e) => ShowView("Links");
            navPanel.Controls.Add(_btnLinks);

            // ZED Calibration button
            _btnCalibration = CreateSidebarButton("ZED Calibration");
            _btnCalibration.Click += (s, e) => ShowView("Calibration");
            navPanel.Controls.Add(_btnCalibration);

            var btnGimbal = CreateSidebarButton("Caddx Gimbal");
            btnGimbal.Click += (s, e) => GimbalJoystickWindow.ShowSingleton(_config, this.FindForm());
            navPanel.Controls.Add(btnGimbal);

            // IMPORTANT: In Windows Forms, docking order is reverse of Z-order
            // Add navPanel FIRST (will be at back, fills remaining space)
            // Add logoPanel SECOND (will be in front, docked at top)
            _sidebarPanel.Controls.Add(navPanel);
            _sidebarPanel.Controls.Add(logoPanel);

            this.Controls.Add(_sidebarPanel);
        }

        private Button CreateSidebarButton(string text)
        {
            var btn = new Button
            {
                Text = text,
                Size = new Size(SIDEBAR_WIDTH - 15, 36),
                Margin = new Padding(0, 1, 0, 1),
                FlatStyle = FlatStyle.Flat,
                BackColor = SIDEBAR_BG,  // Explicit background color to prevent default green
                ForeColor = TEXT_SECONDARY,
                Font = new Font("Segoe UI", 9),
                TextAlign = ContentAlignment.MiddleLeft,
                Padding = new Padding(10, 0, 0, 0),
                Cursor = Cursors.Hand,
            };
            btn.FlatAppearance.BorderSize = 0;
            btn.FlatAppearance.MouseOverBackColor = Color.FromArgb(45, 45, 50);
            btn.FlatAppearance.MouseDownBackColor = ACCENT_COLOR;

            return btn;
        }

        private Label CreateSeparatorLabel(string text)
        {
            if (string.IsNullOrEmpty(text))
            {
                return new Label
                {
                    Size = new Size(SIDEBAR_WIDTH - 15, 8),
                    Margin = new Padding(0, 4, 0, 4),
                };
            }

            return new Label
            {
                Text = text,
                Font = new Font("Segoe UI", 8, FontStyle.Bold),
                ForeColor = TEXT_SECONDARY,
                Size = new Size(SIDEBAR_WIDTH - 15, 22),
                Margin = new Padding(0, 8, 0, 4),
                TextAlign = ContentAlignment.BottomLeft,
                Padding = new Padding(5, 0, 0, 0),
            };
        }
        private void CreateContentArea()
        {
            _contentPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CONTENT_BG,
                Padding = new Padding(0),
            };

            // No header band: the active sidebar button already names the page,
            // so the view content gets the full height. _headerPanel stays null
            // and ShowView/ShowEntry skip the title update.
            _viewContainer = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = CONTENT_BG,
                Padding = new Padding(12),
            };

            _contentPanel.Controls.Add(_viewContainer);

            this.Controls.Add(_contentPanel);
        }
    }
}
