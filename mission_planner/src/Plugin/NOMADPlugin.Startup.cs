// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADPlugin.Startup.cs - Init-time helpers
// ============================================================
// Mission Planner version check and construction of the FlightData
// right-click and main menu-bar entries. Called once from Init().
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADPlugin
    {
        /// <summary>Log a warning when running on an untested Mission Planner version.</summary>
        private static void WarnIfUntestedMissionPlannerVersion()
        {
            try
            {
                string executable = System.Reflection.Assembly.GetEntryAssembly()?.Location;
                string fileVersion = string.IsNullOrEmpty(executable) ? null :
                    System.Diagnostics.FileVersionInfo.GetVersionInfo(executable).FileVersion;
                System.Version.TryParse(fileVersion, out var mpVersion);
                string warning = MissionPlannerVersion.GetWarning(mpVersion, NomadRelease.MissionPlannerTarget);
                if (warning != null)
                {
                    Log.Warn(warning);
                }
            }
            catch (Exception ex)
            {
                Log.Warn($"Could not determine Mission Planner version — {ex.Message}");
            }
        }

        /// <summary>Add NOMAD entries to the FlightData right-click map menu (if available).</summary>
        private void AddFlightDataMenuItems()
        {
            try
            {
                if (Host?.FDMenuMap?.Items == null)
                    return;

                var openMainItem = new ToolStripMenuItem("NOMAD Full Control");
                openMainItem.Click += (s, e) => ShowMainScreen();
                Host.FDMenuMap.Items.Add(openMainItem);

                var openPopOutItem = new ToolStripMenuItem("NOMAD Pop Out Window");
                openPopOutItem.Click += (s, e) => ShowPopOutWindow();
                Host.FDMenuMap.Items.Add(openPopOutItem);

                var settingsItem = new ToolStripMenuItem("NOMAD Settings");
                settingsItem.Click += (s, e) => ShowSettings();
                Host.FDMenuMap.Items.Add(settingsItem);

                var linkStatusItem = new ToolStripMenuItem("NOMAD Link Status");
                linkStatusItem.Click += (s, e) => ShowLinkHealthPanel();
                Host.FDMenuMap.Items.Add(linkStatusItem);
            }
            catch (Exception ex)
            {
                Log.Error($"Could not add FlightData menu — {ex.Message}");
            }
        }

        /// <summary>Add the NOMAD top-level menu to the Mission Planner menu bar.</summary>
        private void AddMainMenuItems()
        {
            try
            {
                var menuStrip = Host.MainForm.MainMenuStrip;
                if (menuStrip == null)
                    return;

                // Find or create NOMAD menu
                ToolStripMenuItem nomadMenu = null;
                foreach (ToolStripItem existing in menuStrip.Items)
                {
                    if (existing is ToolStripMenuItem item && string.Equals(item.Text, "NOMAD", StringComparison.OrdinalIgnoreCase))
                    {
                        nomadMenu = item;
                        break;
                    }
                }
                if (nomadMenu == null)
                {
                    nomadMenu = new ToolStripMenuItem("NOMAD")
                    {
                        ForeColor = Color.White,
                        BackColor = NOMADTheme.ACCENT
                    };
                    // Insert before Help menu (usually last)
                    int insertIndex = menuStrip.Items.Count - 1;
                    if (insertIndex < 0) insertIndex = 0;
                    menuStrip.Items.Insert(insertIndex, nomadMenu);
                }

                // Avoid duplicate items if plugin reloads
                nomadMenu.DropDownItems.Clear();

                var openItem = new ToolStripMenuItem("Open NOMAD");
                openItem.Click += (s, e) => ShowMainScreen();
                nomadMenu.DropDownItems.Add(openItem);

                // Pop-out window option (for multi-monitor setups)
                var popOutItem = new ToolStripMenuItem("Pop Out to Window");
                popOutItem.Click += (s, e) => ShowPopOutWindow();
                nomadMenu.DropDownItems.Add(popOutItem);

                nomadMenu.DropDownItems.Add(new ToolStripSeparator());

                var tools = new ToolStripMenuItem("Tools");
                nomadMenu.DropDownItems.Add(tools);
                // Link Status (Multi-Link Failover)
                var linkStatusItem = new ToolStripMenuItem("Link Status (Failover)");
                linkStatusItem.ForeColor = _config.RouterClientEnabled ? Color.LimeGreen : Color.Gray;
                linkStatusItem.Click += (s, e) => ShowLinkHealthPanel();
                tools.DropDownItems.Add(linkStatusItem);

                // HUD Video controls
                _hudVideoMenuItem = new ToolStripMenuItem("Start HUD Video");
                _hudVideoMenuItem.Click += OnHudVideoMenuClicked;
                tools.DropDownItems.Add(_hudVideoMenuItem);
                var landSetup = new ToolStripMenuItem("Stage Copter LAND settings…");
                landSetup.Click += (sender, args) => StageCopterLandSettings();
                tools.DropDownItems.Add(landSetup);

                nomadMenu.DropDownItems.Add(new ToolStripSeparator());

                var settingsItem2 = new ToolStripMenuItem("Settings...");
                settingsItem2.Click += (s, e) => ShowSettings();
                nomadMenu.DropDownItems.Add(settingsItem2);

                var aboutItem = new ToolStripMenuItem("About NOMAD");
                aboutItem.Click += (s, e) => CustomMessageBox.Show(
                    $"NOMAD Plugin v{Version}\n" +
                    $"McGill Aerial Design\n\n" +
                    $"Choose Open NOMAD for the NOMAD screen\n" +
                    $"(dashboard, native fence setup, video,\n" +
                    $"and plugin router status).\n\n" +
                    $"Native Plan fence draft staging,\n" +
                    $"plugin-wide alerts with toast overlays, plugin router\n" +
                    $"management and status, and configurable payload controls.\n\n" +
                    $"Video: {_config.SelectedVideoInput?.Name ?? "Unconfigured"}\n" +
                    $"Multi-Link: {(_config.RouterClientEnabled ? "Enabled" : "Disabled")}\n" +
                    $"Log: %LOCALAPPDATA%\\Mission Planner\\plugins\\NOMAD\\nomad.log",
                    "About NOMAD"
                );
                nomadMenu.DropDownItems.Add(aboutItem);
            }
            catch (Exception ex)
            {
                Log.Error($"Could not add main menu — {ex.Message}");
            }
        }

        private void StageCopterLandSettings()
        {
            var session = _videoShutdown;
            if (!IsActiveSession(session)) { return; }
            try
            {
                var draft = new NativeLandParameterDraft();
                bool confirmed = MessageBox.Show(Host.MainForm, draft.Preview, "Stage native Copter LAND draft",
                    MessageBoxButtons.OKCancel, MessageBoxIcon.Warning, MessageBoxDefaultButton.Button2) == DialogResult.OK;
                if (!IsActiveSession(session)) { return; }
                draft.TryStage(confirmed, out string message);
                MessageBox.Show(Host.MainForm, message, "Native parameter draft", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception error)
            {
                if (IsActiveSession(session))
                { MessageBox.Show(Host.MainForm, error.Message, "LAND staging unavailable", MessageBoxButtons.OK, MessageBoxIcon.Warning); }
            }
        }
    }
}
