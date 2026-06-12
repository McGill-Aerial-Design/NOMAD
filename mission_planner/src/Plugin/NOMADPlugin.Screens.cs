// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADPlugin.Screens.cs - NOMAD screen hosting
// ============================================================
// Registers the NOMAD screen with Mission Planner's MainSwitcher
// (via reflection, since the API is not public) and shows it
// either as a top-level page or in a pop-out window.
// ============================================================

using System;
using System.Drawing;
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
            if (_screenRegistered)
            {
                return;
            }

            try
            {
                NOMADMainScreen.SetStaticConfig(_sender, _config, _connectionManager, _jetsonConnectionManager, _geofenceConfig, _boundaryMonitor);

                object mainSwitcher = null;

                var viewField = typeof(MainV2).GetField("View",
                    System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static);
                if (viewField != null)
                {
                    mainSwitcher = viewField.GetValue(null);
                }

                if (mainSwitcher == null)
                {
                    var viewProp = Host.MainForm.GetType().GetField("View",
                        System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static);
                    if (viewProp != null)
                    {
                        mainSwitcher = viewProp.GetValue(null);
                    }
                }

                if (mainSwitcher != null)
                {
                    var screenType = mainSwitcher.GetType().GetNestedType("Screen",
                        System.Reflection.BindingFlags.Public);
                    if (screenType != null)
                    {
                        var screenCtor = screenType.GetConstructor(new Type[] { typeof(string), typeof(Type), typeof(bool) });
                        if (screenCtor != null)
                        {
                            var screen = screenCtor.Invoke(new object[] { "NOMAD", typeof(NOMADMainScreen), false });

                            var addScreenMethod = mainSwitcher.GetType().GetMethod("AddScreen",
                                System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance);
                            if (addScreenMethod != null)
                            {
                                addScreenMethod.Invoke(mainSwitcher, new object[] { screen });
                                _screenRegistered = true;
                            }
                        }
                    }
                }

                if (!_screenRegistered)
                {
                    Log.Warn("Could not register as top-level screen");
                }
            }
            catch (Exception ex)
            {
                Log.Error($"RegisterNomadScreen failed — {ex.Message}");
            }
        }

        /// <summary>
        /// Shows the NOMAD interface as a top-level page in Mission Planner.
        /// Similar to FlightData, FlightPlan, InitialSetup, etc.
        /// </summary>
        private void ShowMainScreen()
        {
            if (Host?.MainForm != null && Host.MainForm.InvokeRequired)
            {
                Host.MainForm.BeginInvoke((MethodInvoker)delegate { ShowMainScreen(); });
                return;
            }

            if (!_screenRegistered)
            {
                RegisterNomadScreen();
            }

            try
            {
                var viewField = typeof(MainV2).GetField("View",
                    System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static);
                if (viewField != null)
                {
                    var mainSwitcher = viewField.GetValue(null);
                    if (mainSwitcher != null)
                    {
                        var showScreenMethod = mainSwitcher.GetType().GetMethod("ShowScreen",
                            System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance,
                            null, new Type[] { typeof(string) }, null);
                        if (showScreenMethod != null)
                        {
                            showScreenMethod.Invoke(mainSwitcher, new object[] { "NOMAD" });
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Could not show NOMAD screen — {ex.Message}");
            }
        }

        /// <summary>
        /// Shows the NOMAD screen in a pop-out window for multi-monitor setups.
        /// </summary>
        private void ShowPopOutWindow()
        {
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
                    NOMADMainScreen.SetStaticConfig(_sender, _config, _connectionManager, _jetsonConnectionManager, _geofenceConfig, _boundaryMonitor);

                    var nomadScreen = new NOMADMainScreen(_sender, _config, _connectionManager, _jetsonConnectionManager);

                    _popOutForm = new Form
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
                    _popOutForm.Shown += (s, e) =>
                    {
                        nomadScreen.Activate();
                    };

                    // Deactivate when hidden
                    _popOutForm.FormClosing += (s, e) =>
                    {
                        if (e.CloseReason == CloseReason.UserClosing)
                        {
                            e.Cancel = true;
                            _popOutForm.Hide();
                            nomadScreen.Deactivate();
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
                CustomMessageBox.Show($"Failed to open pop-out window: {ex.Message}", "Error");
            }
        }
    }
}
