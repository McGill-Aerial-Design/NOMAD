// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADPlugin.Startup.cs - Init-time helpers
// ============================================================
// Assembly resolver for HelixToolkit dependencies, the Mission
// Planner version check, and construction of the FlightData
// right-click and main menu-bar entries. Called once from Init().
// ============================================================

using System;
using System.Drawing;
using System.IO;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADPlugin
    {
        /// <summary>
        /// Registers an AppDomain resolver that loads HelixToolkit assemblies
        /// from the Mission Planner plugins folders. Idempotent.
        /// </summary>
        private static void RegisterAssemblyResolver()
        {
            if (_assemblyResolverRegistered)
                return;

            AppDomain.CurrentDomain.AssemblyResolve += (sender, args) =>
            {
                try
                {
                    var assemblyName = new System.Reflection.AssemblyName(args.Name).Name;
                    if (assemblyName.StartsWith("HelixToolkit", StringComparison.OrdinalIgnoreCase))
                    {
                        // Look in the plugins folder next to this assembly first.
                        string pluginsPath = Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location);
                        string dllPath = Path.Combine(pluginsPath, assemblyName + ".dll");
                        if (File.Exists(dllPath))
                        {
                            Log.Debug($"Loading {assemblyName} from plugins folder");
                            return System.Reflection.Assembly.LoadFrom(dllPath);
                        }

                        string userPluginsPath = Path.Combine(
                            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
                            "Mission Planner", "plugins");
                        dllPath = Path.Combine(userPluginsPath, assemblyName + ".dll");
                        if (File.Exists(dllPath))
                        {
                            Log.Debug($"Loading {assemblyName} from user plugins folder");
                            return System.Reflection.Assembly.LoadFrom(dllPath);
                        }
                    }
                }
                catch (Exception ex)
                {
                    Log.Error($"Assembly resolve error for {args.Name}: {ex.Message}");
                }
                return null;
            };
            _assemblyResolverRegistered = true;
        }

        /// <summary>Log a warning when running on an untested Mission Planner version.</summary>
        private static void WarnIfUntestedMissionPlannerVersion()
        {
            try
            {
                var mpVersion = System.Reflection.Assembly.GetEntryAssembly()?.GetName()?.Version;
                if (mpVersion != null && mpVersion < new System.Version("1.3.80"))
                {
                    Log.Warn($"Untested Mission Planner version {mpVersion}. Recommend 1.3.80+. Some features may not work correctly.");
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

                // Hovering "NOMAD" opens the dropdown so the user can reach the
                // items without clicking; a deliberate CLICK opens the NOMAD
                // screen. Previously the dropdown only opened on click, and that
                // same click navigated — yanking the user to the dashboard every
                // time they just wanted the menu.
                nomadMenu.MouseEnter += (s, e) =>
                {
                    if (!nomadMenu.DropDown.Visible)
                        nomadMenu.ShowDropDown();
                };
                nomadMenu.MouseDown += (s, e) =>
                {
                    if (e.Button == MouseButtons.Left)
                        ShowMainScreen();
                };

                // Avoid duplicate items if plugin reloads
                nomadMenu.DropDownItems.Clear();

                // Pop-out window option (for multi-monitor setups)
                var popOutItem = new ToolStripMenuItem("Pop Out to Window");
                popOutItem.Click += (s, e) => ShowPopOutWindow();
                nomadMenu.DropDownItems.Add(popOutItem);

                nomadMenu.DropDownItems.Add(new ToolStripSeparator());

                // Link Status (Dual Link Failover)
                var linkStatusItem = new ToolStripMenuItem("Link Status (Failover)");
                linkStatusItem.ForeColor = _config.DualLinkEnabled ? Color.LimeGreen : Color.Gray;
                linkStatusItem.Click += (s, e) => ShowLinkHealthPanel();
                nomadMenu.DropDownItems.Add(linkStatusItem);

                // HUD Video controls
                var hudVideoItem = new ToolStripMenuItem("Start HUD Video");
                hudVideoItem.Click += (s, e) =>
                {
                    if (_hudVideoStarted)
                    {
                        StopHudVideo();
                        hudVideoItem.Text = "Start HUD Video";
                    }
                    else
                    {
                        StartHudVideo();
                        hudVideoItem.Text = "Stop HUD Video";
                    }
                };
                nomadMenu.DropDownItems.Add(hudVideoItem);

                nomadMenu.DropDownItems.Add(new ToolStripSeparator());

                var settingsItem2 = new ToolStripMenuItem("Settings...");
                settingsItem2.Click += (s, e) => ShowSettings();
                nomadMenu.DropDownItems.Add(settingsItem2);

                var aboutItem = new ToolStripMenuItem("About NOMAD");
                aboutItem.Click += (s, e) => CustomMessageBox.Show(
                    $"NOMAD Plugin v{Version}\n" +
                    $"McGill Aerial Design — AEAC 2026\n\n" +
                    $"Hover the NOMAD menu for tools; click it to open the\n" +
                    $"NOMAD screen (dashboard, flight boundaries, video,\n" +
                    $"terminal, health, motor music, dual-link status,\n" +
                    $"ZED calibration).\n\n" +
                    $"Geofence monitoring with enforced violation actions,\n" +
                    $"plugin-wide alerts with toast overlays, MAVLink dual-link\n" +
                    $"failover routing, and configurable payload controls.\n\n" +
                    $"Jetson: {_config.EffectiveIP}:{_config.JetsonPort}\n" +
                    $"Dual Link: {(_config.DualLinkEnabled ? "Enabled" : "Disabled")}\n" +
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
    }
}
