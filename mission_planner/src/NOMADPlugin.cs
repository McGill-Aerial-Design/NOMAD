// ============================================================
// NOMAD Mission Planner Plugin
// ============================================================
// Target: Mission Planner 1.3.x
// Competition: AEAC 2026
//
// This plugin adds NOMAD-specific controls to Mission Planner
// for Task 1 (Recon) and Task 2 (Extinguish) operations.
//
// Features:
// - Dual-link communication (HTTP or MAVLink/ELRS)
// - Task 1: Capture snapshot command
// - Task 2: Reset exclusion map command
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Plugin;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Main NOMAD plugin class implementing Mission Planner Plugin interface.
    /// </summary>
    public class NOMADPlugin : Plugin
    {
        // Plugin metadata
        public override string Name => "NOMAD Control";
        public override string Version => "1.0.0";
        public override string Author => "McGill Aerial Design";

        // Plugin state
        private NOMADConfig _config;
        private DualLinkSender _sender;
        private NOMADControlPanel _controlPanelTab;
        private NOMADControlPanel _controlPanelWindow;
        private TabPage _tabPage;
        private Form _panelForm;
        private bool _tabAdded;

        // ============================================================
        // Plugin Lifecycle
        // ============================================================

        /// <summary>
        /// Called when plugin is loaded.
        /// </summary>
        public override bool Init()
        {
            try
            {
                // Load configuration
                _config = NOMADConfig.Load();
                
                // Initialize dual-link sender
                _sender = new DualLinkSender(_config);

                // Add menu items to FlightData right-click menu (if available)
                try
                {
                    if (Host?.FDMenuMap?.Items != null)
                    {
                        var openPanelItem = new ToolStripMenuItem("NOMAD Control Panel");
                        openPanelItem.Click += (s, e) => ShowControlPanel();
                        Host.FDMenuMap.Items.Add(openPanelItem);

                        var settingsItem = new ToolStripMenuItem("NOMAD Settings");
                        settingsItem.Click += (s, e) => ShowSettings();
                        Host.FDMenuMap.Items.Add(settingsItem);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"NOMAD: Could not add FlightData menu - {ex.Message}");
                }

                // Also add to main menu bar under Help (common location for plugins)
                try
                {
                    var menuStrip = Host.MainForm.MainMenuStrip;
                    if (menuStrip != null)
                    {
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
                                ForeColor = Color.White
                            };
                            // Insert before Help menu (usually last)
                            int insertIndex = menuStrip.Items.Count - 1;
                            if (insertIndex < 0) insertIndex = 0;
                            menuStrip.Items.Insert(insertIndex, nomadMenu);
                        }

                        // Avoid duplicate items if plugin reloads
                        nomadMenu.DropDownItems.Clear();

                        var openPanelItem = new ToolStripMenuItem("Open Control Panel");
                        openPanelItem.Click += (s, e) => ShowControlPanel();
                        nomadMenu.DropDownItems.Add(openPanelItem);
                        
                        var settingsItem2 = new ToolStripMenuItem("Settings...");
                        settingsItem2.Click += (s, e) => ShowSettings();
                        nomadMenu.DropDownItems.Add(settingsItem2);
                        
                        var aboutItem = new ToolStripMenuItem("About NOMAD");
                        aboutItem.Click += (s, e) => CustomMessageBox.Show(
                            $"NOMAD Plugin v{Version}\n" +
                            $"McGill Aerial Design - AEAC 2026\n\n" +
                            $"Jetson IP: {_config.JetsonIP}:{_config.JetsonPort}\n" +
                            $"Mode: {(_config.UseELRS ? "ELRS/MAVLink" : "HTTP")}",
                            "About NOMAD"
                        );
                        nomadMenu.DropDownItems.Add(aboutItem);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"NOMAD: Could not add main menu - {ex.Message}");
                }

                // Keep startup non-blocking; show popup only in DebugMode
                if (_config.DebugMode)
                {
                    Host?.MainForm?.BeginInvoke((MethodInvoker)delegate
                    {
                        CustomMessageBox.Show(
                            $"NOMAD Plugin v{Version} loaded.\n\n" +
                            $"Use the NOMAD menu → Open Control Panel\n" +
                            $"Or right-click in Flight Data → NOMAD Control Panel\n\n" +
                            $"Mode: {(_config.UseELRS ? "ELRS/MAVLink" : "HTTP")}\n" +
                            $"Jetson IP: {_config.JetsonIP}",
                            "NOMAD"
                        );
                    });
                }

                return true;
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show($"NOMAD Plugin failed to load: {ex.Message}", "Error");
                return false;
            }
        }

        /// <summary>
        /// Called when FlightData tab is first shown.
        /// Adds NOMAD control panel to the UI.
        /// </summary>
        public override bool Loaded()
        {
            try
            {
                // Ensure UI setup runs on the UI thread
                if (Host?.MainForm != null && Host.MainForm.InvokeRequired)
                {
                    Host.MainForm.BeginInvoke((MethodInvoker)delegate { Loaded(); });
                    return true;
                }

                if (_controlPanelTab == null)
                {
                    _controlPanelTab = new NOMADControlPanel(_sender, _config);
                }

                if (_tabPage == null)
                {
                    _tabPage = new TabPage("NOMAD")
                    {
                        BackColor = Color.FromArgb(45, 45, 48)
                    };
                    _tabPage.Controls.Add(_controlPanelTab);
                    _controlPanelTab.Dock = DockStyle.Fill;
                }

                // Attempt tab injection; if it fails, use the control-panel window.
                if (!_tabAdded)
                {
                    _tabAdded = AddTabToFlightData(_tabPage);
                    if (!_tabAdded)
                    {
                        Console.WriteLine("NOMAD: Could not add FlightData tab; using control panel window");
                        ShowControlPanel();
                    }
                }

                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to create UI - {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Called periodically during FlightData updates.
        /// </summary>
        public override bool Loop()
        {
            // Update status display if needed
            var connected = Host.cs.connected;
            var lat = Host.cs.lat;
            var lng = Host.cs.lng;
            var alt = Host.cs.alt;

            _controlPanelTab?.UpdateStatus(connected, lat, lng, alt);
            _controlPanelWindow?.UpdateStatus(connected, lat, lng, alt);
            return true;
        }

        /// <summary>
        /// Called when plugin is unloaded.
        /// </summary>
        public override bool Exit()
        {
            try
            {
                if (_panelForm != null)
                {
                    _panelForm.Close();
                    _panelForm = null;
                }
            }
            catch
            {
                // ignore
            }

            _controlPanelWindow = null;
            _sender?.Dispose();
            return true;
        }

        // ============================================================
        // Private Methods
        // ============================================================

        private bool AddTabToFlightData(TabPage tab)
        {
            // Access FlightData tab control
            // Note: This depends on Mission Planner internals and may
            // need adjustment for different MP versions
            try
            {
                var flightData = Host.MainForm.Controls.Find("FlightData", true);
                if (flightData.Length > 0 && flightData[0] is UserControl fd)
                {
                    var tabControl = fd.Controls.Find("tabControlactions", true);
                    if (tabControl.Length > 0 && tabControl[0] is TabControl tc)
                    {
                        if (!tc.TabPages.Contains(tab))
                        {
                            tc.TabPages.Add(tab);
                        }
                        return true;
                    }
                }
                
                return false;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Tab insertion failed - {ex.Message}");
                return false;
            }
        }

        private void ShowControlPanel()
        {
            if (Host?.MainForm != null && Host.MainForm.InvokeRequired)
            {
                Host.MainForm.BeginInvoke((MethodInvoker)delegate { ShowControlPanel(); });
                return;
            }

            // Always create a fresh instance to avoid disposed object errors
            if (_panelForm == null || _panelForm.IsDisposed)
            {
                // Create new panel instance
                _controlPanelWindow = new NOMADControlPanel(_sender, _config);
                
                _panelForm = new Form
                {
                    Text = "NOMAD Control Panel",
                    StartPosition = FormStartPosition.CenterParent,
                    Width = 420,
                    Height = 1000,  // Increased height for health tab
                    AutoScroll = true  // Enable scrolling if content exceeds form
                };
                _panelForm.FormClosed += (s, e) =>
                {
                    // Dispose control when form closes
                    _controlPanelWindow?.Dispose();
                    _controlPanelWindow = null;
                    _panelForm = null;
                };
                _panelForm.Controls.Add(_controlPanelWindow);
                _controlPanelWindow.Dock = DockStyle.Fill;
                _controlPanelWindow.AutoScroll = true;  // Enable scrolling in panel too
            }

            if (!_panelForm.Visible)
            {
                _panelForm.Show(Host?.MainForm);
            }
            else
            {
                _panelForm.BringToFront();
                _panelForm.Activate();
            }
        }

        private void ShowSettings()
        {
            using (var form = new NOMADSettingsForm(_config))
            {
                if (form.ShowDialog() == DialogResult.OK)
                {
                    _config = form.Config;
                    _config.Save();
                    _sender.UpdateConfig(_config);
                    _controlPanelTab?.UpdateConfig(_config);
                    _controlPanelWindow?.UpdateConfig(_config);
                }
            }
        }
    }
}
