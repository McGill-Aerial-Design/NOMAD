// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADPlugin.Screens.cs - NOMAD screen hosting
// ============================================================
// Registers the NOMAD screen with Mission Planner's MainSwitcher
// through the reviewed host API and shows it
// either as a top-level page or in a pop-out window.
// ============================================================

using System;
using System.Drawing;
using System.Linq;
using MissionPlanner.Controls;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADPlugin
    {
        /// <summary>
        /// Registers the NOMAD screen with Mission Planner's MainSwitcher.
        /// This makes NOMAD appear as a top-level page like FlightData, FlightPlan, etc.
        /// </summary>
        private void RegisterNomadScreen()
        {
            if (_screenRegistered || MainV2.View == null) { return; }
            NOMADMainScreen.SetStaticConfig(_config, _connectionManager);
            if (!MainV2.View.screens.Any(screen => screen.Name == "NOMAD"))
            { MainV2.View.AddScreen(new MainSwitcher.Screen("NOMAD", typeof(NOMADMainScreen), false)); }
            _screenRegistered = true;
        }

        private static bool IsMainScreenCurrent() => MainV2.View?.current?.Name == "NOMAD" &&
            MainV2.View.current.Control != null && !MainV2.View.current.Control.IsDisposed;

        /// <summary>
        /// Shows the NOMAD interface as a top-level page in Mission Planner.
        /// Similar to FlightData, FlightPlan, InitialSetup, etc.
        /// </summary>
        private void ShowMainScreen()
        {
            if (!IsActiveSession(_videoShutdown)) { return; }
            if (Host?.MainForm != null && Host.MainForm.InvokeRequired)
            {
                Host.MainForm.BeginInvoke((MethodInvoker)delegate { ShowMainScreen(); });
                return;
            }

            if (IsMainScreenCurrent()) { return; }
            try
            {
                RegisterNomadScreen();
                MainV2.View?.ShowScreen("NOMAD");
            }
            catch (Exception ex)
            { Log.Error($"Could not show NOMAD screen — {ex.Message}"); }
        }

        /// <summary>
        /// Shows the NOMAD screen in a pop-out window for multi-monitor setups.
        /// </summary>
        private void ShowPopOutWindow()
        {
            if (!IsActiveSession(_videoShutdown)) { return; }
            if (Host?.MainForm != null && Host.MainForm.InvokeRequired)
            {
                Host.MainForm.BeginInvoke((MethodInvoker)delegate { ShowPopOutWindow(); });
                return;
            }

            try
            {
                // Create or show the pop-out window
                if (_popOutForm == null || _popOutForm.IsDisposed)
                {
                    // Set static configuration for the screen
                    NOMADMainScreen.SetStaticConfig(_config, _connectionManager);

                    var nomadScreen = new NOMADMainScreen(_config, _connectionManager);

                    _popOutForm = new NOMADForm
                    {
                        Text = "NOMAD Control - Pop Out",
                        StartPosition = FormStartPosition.CenterScreen,
                        Size = new Size(1200, 800),
                        MinimumSize = new Size(900, 600),
                        BackColor = Color.FromArgb(30, 30, 33),
                        Icon = Host?.MainForm?.Icon,
                    };

                    nomadScreen.Dock = DockStyle.Fill;
                    _popOutForm.Controls.Add(nomadScreen);

                    // Activate when shown - NOMAD screen implements Activate() directly
                    _popOutForm.VisibleChanged += (s, e) =>
                    {
                        if (_popOutForm.Visible) { nomadScreen.Activate(); }
                        else { nomadScreen.Deactivate(); }
                    };

                    // Deactivate when hidden
                    _popOutForm.FormClosing += (s, e) =>
                    {
                        if (e.CloseReason == CloseReason.UserClosing)
                        {
                            e.Cancel = true;
                            _popOutForm.Hide();
                        }
                    };
                }

                if (!_popOutForm.Visible)
                {
                    _popOutForm.Show(Host?.MainForm);
                }
                else
                {
                    _popOutForm.BringToFront();
                    _popOutForm.Activate();
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Failed to show pop-out window — {ex.Message}");
                MessageBox.Show(Host.MainForm, $"Failed to open pop-out window: {ex.Message}", "Error");
            }
        }
    }
}
