// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADMainScreen.Modules.cs - Module-driven sidebar
// ============================================================
// Built only when a ModuleHost with view descriptors is supplied.
// The default CreateSidebar()/ShowView() path runs unless modules
// are wired in.
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using NOMAD.MissionPlanner.Core;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADMainScreen
    {
        // ============================================================
        // Module-driven Sidebar (NOMAD module SDK — see src/Core)
        // ============================================================
        // Built only when a ModuleHost with view descriptors is supplied. The
        // default CreateSidebar()/ShowView() path runs unless modules are wired in.

        private void CreateSidebarFromDescriptors()
        {
            _sidebarPanel = new Panel
            {
                Dock = DockStyle.Left,
                Width = SIDEBAR_WIDTH,
                BackColor = SIDEBAR_BG,
                Padding = new Padding(0),
            };

            var logoPanel = new Panel
            {
                Dock = DockStyle.Top,
                Height = 45,
                BackColor = Color.FromArgb(20, 20, 23),
                Padding = new Padding(12, 8, 12, 5),
            };
            logoPanel.Controls.Add(new Label
            {
                Text = "NOMAD",
                Font = new Font("Segoe UI", 14, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                Location = new Point(12, 10),
                AutoSize = true,
            });

            var navPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                Padding = new Padding(5),
                AutoScroll = true,
                BackColor = SIDEBAR_BG,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
            };

            string currentSection = null;
            foreach (var descriptor in _descriptors)
            {
                // Emit a section heading whenever the group changes.
                if (!string.IsNullOrEmpty(descriptor.Section) && descriptor.Section != currentSection)
                {
                    navPanel.Controls.Add(CreateSeparatorLabel(descriptor.Section));
                    currentSection = descriptor.Section;
                }

                var entry = descriptor; // capture for the click closure
                var btn = CreateSidebarButton(entry.ButtonText);
                btn.Click += (s, e) => ShowEntry(entry);
                navPanel.Controls.Add(btn);

                if (entry.Kind == NomadEntryKind.View)
                    _descriptorButtons[entry.Id] = btn;
            }

            // Same docking order rules as the default sidebar.
            _sidebarPanel.Controls.Add(navPanel);
            _sidebarPanel.Controls.Add(logoPanel);
            this.Controls.Add(_sidebarPanel);
        }

        private void ShowEntry(NomadViewDescriptor descriptor)
        {
            if (descriptor == null) return;

            // Action entries (e.g. floating windows) don't swap the content view.
            if (descriptor.Kind == NomadEntryKind.Action)
            {
                try { descriptor.Invoke(); }
                catch (Exception ex) { Log.Error($"module action '{descriptor.Id}' failed — {ex.Message}"); }
                return;
            }

            // Notify the outgoing view it is being hidden (kept cached, not disposed).
            if (_currentView is INomadView previousView)
                previousView.OnDeactivated();

            if (_currentView != null)
                _viewContainer.Controls.Remove(_currentView);

            // Lazily create + cache the view control.
            Control control;
            if (!_descriptorViewCache.TryGetValue(descriptor.Id, out control) || control == null)
            {
                control = descriptor.CreateView();
                _descriptorViewCache[descriptor.Id] = control;
            }

            if (control != null)
            {
                control.Dock = DockStyle.Fill;
                _viewContainer.Controls.Add(control);
                _currentView = control as UserControl; // keeps the update timer working
                _currentDescriptorId = descriptor.Id;
                if (control is INomadView activated)
                    activated.OnActivated();
            }

            UpdateDescriptorButtonState(descriptor.Id);
        }

        private void UpdateDescriptorButtonState(string activeId)
        {
            foreach (var pair in _descriptorButtons)
            {
                pair.Value.BackColor = SIDEBAR_BG;
                pair.Value.ForeColor = TEXT_SECONDARY;
            }

            Button activeBtn;
            if (!string.IsNullOrEmpty(activeId) &&
                _descriptorButtons.TryGetValue(activeId, out activeBtn) && activeBtn != null)
            {
                activeBtn.BackColor = ACCENT_COLOR;
                activeBtn.ForeColor = TEXT_PRIMARY;
            }
        }
    }
}
