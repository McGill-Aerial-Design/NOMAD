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
        private NOMADMenuItem _nomadMenu;

        private void RemoveMainMenu()
        {
            _nomadMenu?.Owner?.Items.Remove(_nomadMenu);
            _nomadMenu?.Dispose();
            _nomadMenu = null;
        }

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

                RemoveMainMenu();
                var nomadMenu = new NOMADMenuItem(menuStrip, IsMainScreenCurrent, ShowMainScreen);
                _nomadMenu = nomadMenu;
                menuStrip.Items.Insert(Math.Max(0, menuStrip.Items.Count - 1), nomadMenu);

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

                nomadMenu.DropDownItems.Add(new ToolStripSeparator());

                var settingsItem2 = new ToolStripMenuItem("Settings...");
                settingsItem2.Click += (s, e) => ShowSettings();
                nomadMenu.DropDownItems.Add(settingsItem2);

                var aboutItem = new ToolStripMenuItem("About NOMAD");
                aboutItem.Click += (s, e) => MessageBox.Show(Host.MainForm,
                    $"NOMAD Plugin v{Version}\n" +
                    $"McGill Aerial Design\n\n" +
                    $"Video: {_config.SelectedVideoInput?.Name ?? "Unconfigured"}\n" +
                    $"Router: {(_config.RouterClientEnabled ? "Enabled" : "Disabled")}",
                    "About NOMAD"
                );
                nomadMenu.DropDownItems.Add(aboutItem);
            }
            catch (Exception ex)
            {
                Log.Error($"Could not add main menu — {ex.Message}");
            }
        }
    }
}
