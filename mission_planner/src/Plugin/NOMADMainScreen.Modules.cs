// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADMainScreen.Modules.cs - Module-contributed sidebar entries
// ============================================================
// A plugin-supplied ModuleHost (NOMAD module SDK — see src/Core) can add its
// own sidebar views and actions. CreateSidebar() appends them after the
// built-in entries, so modules extend the screen without replacing it.
// ============================================================

using System;
using System.Windows.Forms;
using NOMAD.MissionPlanner.Core;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADMainScreen
    {
        // Append each module-contributed entry to the sidebar nav panel: a
        // section heading when the group changes, then a button that either
        // shows the module's view or runs its action.
        private void AppendModuleEntries(FlowLayoutPanel navPanel)
        {
            if (_moduleHost == null) return;

            string currentSection = null;
            foreach (var descriptor in _moduleHost.GetViewDescriptors())
            {
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
                if (control is INomadView activated)
                    activated.OnActivated();
            }

            // Clear the built-in button highlight, then light up this module button.
            UpdateSidebarButtonState(null);
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
