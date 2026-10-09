// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADMainScreen.Layout.cs - Sidebar and content-area layout
// ============================================================
// Builds the default (hardcoded) sidebar, sidebar buttons, section
// separators, and the header/content panels. View switching and
// view switching lives in NOMADMainScreen.cs.
// ============================================================

using System.Drawing;
using System.Windows.Forms;

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

            navPanel.Controls.Add(CreateSeparatorLabel("ANALYSIS"));

            navPanel.Controls.Add(CreateSeparatorLabel("TOOLS"));

            // Video button
            _btnVideo = CreateSidebarButton("Video Feed");
            _btnVideo.Click += (s, e) => ShowView("Video");
            navPanel.Controls.Add(_btnVideo);


            // Links button
            _btnLinks = CreateSidebarButton("Link Status");
            _btnLinks.Click += (s, e) => ShowView("Links");
            navPanel.Controls.Add(_btnLinks);

            var btnGimbal = CreateSidebarButton("Gimbal");
            btnGimbal.Click += (s, e) => GimbalJoystickWindow.ShowSingleton(_config, this.FindForm());
            navPanel.Controls.Add(btnGimbal);

            // Append any module-contributed entries (NOMAD module SDK — see src/Core).

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
