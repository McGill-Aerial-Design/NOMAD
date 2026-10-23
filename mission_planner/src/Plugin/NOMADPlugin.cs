// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Mission Planner Plugin
// ============================================================
// Target: Mission Planner 1.3.x
//
// Features:
// - Full-page NOMAD control interface with tabs
// - Embedded video streaming
// - Native Mission Planner gimbal controls
// - Plugin-owned ground router
// - Saved payload mappings and native controls with local input interlocks
// ============================================================

using System;
using System.Drawing;
using System.IO;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Plugin;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Main NOMAD plugin class implementing Mission Planner Plugin interface.
    /// </summary>
    public partial class NOMADPlugin : Plugin
    {
        // Plugin metadata
        public override string Name => "NOMAD Control";
        public override string Version => NomadRelease.Version;
        public override string Author => "McGill Aerial Design";

        // Plugin state
        private NOMADConfig _config;
        private NOMADProfiles _profiles;
        private NotificationService _notificationService;
        private FenceSetupInput _fenceSetupInput;
        private TerminationController _termination;
        private MAVLinkConnectionManager _connectionManager;  // Plugin-owned router
        private NomadJoystickService _joystickService;        // Physical joysticks → gimbal + camera tilt
        private GimbalArrowKeyFilter _gimbalArrowKeyFilter;    // Mission Planner-wide arrow key nudges
        private Form _popOutForm;                             // Pop-out window for NOMAD screen
        private HudVideoPlayer _hudVideo;
        private bool _hudVideoStarted => _hudVideo?.IsActive == true;
        private System.Threading.CancellationTokenSource _videoShutdown;
        private bool _screenRegistered = false;               // Track if NOMAD screen is registered with MainSwitcher

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
                loopratehz = 1;
                ShutdownVideo();
                _fenceSetupInput = new FenceSetupInput();
                NOMADMainScreen.SetFenceSetupInput(_fenceSetupInput);
                _videoShutdown = new System.Threading.CancellationTokenSource();
                NOMADMainScreen.SetVideoShutdown(_videoShutdown.Token, ShowSettings);
                WarnIfUntestedMissionPlannerVersion();

                // Load configuration
                _profiles = NOMADProfiles.Load();
                _config = _profiles.Selected.Config;

                GimbalController.ApplyProfile(_config);
                OutputController.Initialize(_config);
                _termination = new TerminationController(_config);
                PayloadController.ApplyProfile(_config);
                OutputController.ConnectionChanged += GimbalController.InvalidateInput;
                if (Host?.MainForm != null) { Host.MainForm.FormClosed += OnHostClosed; }

                // Notification service runs plugin-wide so battery / GPS
                // alerts (including audio + TTS) fire regardless of which NOMAD tab
                // is open — and even when the user is on a non-NOMAD MP screen.
                _notificationService = new NotificationService();
                NotificationService.Shared = _notificationService;
                _notificationService.StartMonitoring();

                // Toast overlay: warnings pop bottom-right
                // on every MP page, not just inside the NOMAD screen.
                NotificationToast.Attach(_notificationService, Host?.MainForm);
                // Apply saved audio preferences (master mute, altitude callouts)
                // before anything speaks.
                AudioAlerts.ApplyConfig(_config);

                // Startup chime + spoken welcome (fires once per process).
                AudioAlerts.PlayWelcomeOnce();

                // Initialize the plugin-owned ground router
                if (_config.RouterClientEnabled)
                {
                    InitializeConnectionManager();
                }

                // Capture arrow keys at the application message-pump level so
                // focused buttons, grids, and non-NOMAD views cannot consume them
                // while the operator has enabled gimbal keyboard control.
                _gimbalArrowKeyFilter = new GimbalArrowKeyFilter(_profiles);

                // Physical joystick service — starts only if either channel is enabled in config.
                _joystickService = new NomadJoystickService(_config);
                if (_joystickService.NeedsToRun())
                {
                    try
                    {
                        _joystickService.Start();
                    }
                    catch (Exception ex) { Log.Error($"Joystick service start failed — {ex.Message}"); }
                }

                // Menu entries (FlightData right-click + main menu bar) are
                // built in NOMADPlugin.Startup.cs.
                AddFlightDataMenuItems();
                AddMainMenuItems();

                // Keep startup non-blocking; show popup only in DebugMode
                if (_config.DebugMode)
                {
                    Host?.MainForm?.BeginInvoke((MethodInvoker)delegate
                    {
                        CustomMessageBox.Show(
                            $"NOMAD Plugin v{Version} loaded (debug mode).\n\n" +
                            $"Use NOMAD → Open NOMAD for the interface,\n" +
                            $"or choose tools and settings from the menu.\n\n" +
                            "Custom gimbal and payload controls use Mission Planner. Physical completion remains unverified.",
                            "NOMAD"
                        );
                    });
                }

                return true;
            }
            catch (Exception ex)
            {
                Exit();
                CustomMessageBox.Show($"NOMAD Plugin failed to load: {ex.Message}", "Error");
                return false;
            }
        }

        /// <summary>
        /// Called when FlightData tab is first shown.
        /// </summary>
        public override bool Loaded()
        {
            try
            {
                if (_videoShutdown == null || _videoShutdown.IsCancellationRequested)
                {
                    return false;
                }
                var videoToken = _videoShutdown.Token;
                // Ensure UI setup runs on the UI thread
                if (Host?.MainForm != null && Host.MainForm.InvokeRequired)
                {
                    Host.MainForm.BeginInvoke((MethodInvoker)delegate
                    {
                        if (!videoToken.IsCancellationRequested)
                        {
                            Loaded();
                        }
                    });
                    return true;
                }

                // Register NOMAD as a top-level screen (no quick tab - use pop-out instead)
                RegisterNomadScreen();

                // Auto-start HUD video if configured
                if (_config.AutoStartHudVideo && !_hudVideoStarted)
                {
                    StartHudVideo();
                }

                return true;
            }
            catch (Exception ex)
            {
                Log.Error($"Failed to create UI — {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Called periodically during FlightData updates.
        /// </summary>
        public override bool Loop()
        {
            OutputController.RefreshConnection();
            _termination?.UpdateBinding();
            UpdateTerminationMenu();
            return true;
        }

        /// <summary>
        /// Called when plugin is unloaded.
        /// </summary>
        private void OnHostClosed(object sender, FormClosedEventArgs args) { Exit(); }

        public override bool Exit()
        {
            if (Host?.MainForm != null) { Host.MainForm.FormClosed -= OnHostClosed; }
            StopCustomControls();
            ShutdownVideo();
            NOMADMainScreen.SetFenceSetupInput(null);
            _fenceSetupInput = null;
            try
            {
                // Unhook the toast overlay before the service goes away
                NotificationToast.Detach();

                // Stop notification service
                if (NotificationService.Shared == _notificationService) NotificationService.Shared = null;
                _notificationService?.StopMonitoring();
                _notificationService?.Dispose();
                _notificationService = null;

                // Stop connection manager monitoring
                _connectionManager?.StopMonitoring();
                _connectionManager?.Dispose();
                _connectionManager = null;



                if (_popOutForm != null && !_popOutForm.IsDisposed)
                {
                    _popOutForm.Dispose();
                    _popOutForm = null;
                }

            }
            catch
            {
                // ignore disposal errors
            }

            return true;
        }

        // ============================================================
        // ============================================================

        // Screen registration and pop-out hosting live in NOMADPlugin.Screens.cs.

        private void StopCustomControls()
        {
            RemoveTerminationMenu();
            _termination?.Dispose();
            _termination = null;
            try { _gimbalArrowKeyFilter?.Dispose(); }
            catch (Exception error) { Log.Warn("Keyboard cleanup failed: " + error.Message); }
            _gimbalArrowKeyFilter = null;
            try { GimbalJoystickWindow.CloseSingleton(); }
            catch (Exception error) { Log.Warn("Gimbal window cleanup failed: " + error.Message); }
            try { _joystickService?.Dispose(); }
            catch (Exception error) { Log.Warn("Joystick cleanup failed: " + error.Message); }
            _joystickService = null;
            OutputController.ConnectionChanged -= GimbalController.InvalidateInput;
            OutputController.Shutdown();
            PayloadController.Shutdown();
        }

        private bool IsActiveSession(System.Threading.CancellationTokenSource session)
        {
            return session != null && ReferenceEquals(session, _videoShutdown) && !session.IsCancellationRequested;
        }

        private bool _savingSettings;
        private async void ShowSettings()
        {
            var session = _videoShutdown;
            if (!IsActiveSession(session) || _savingSettings) { return; }
            _savingSettings = true;
            var previousConfig = _config;
            var previousService = _joystickService;
            var previousOwner = PayloadController.Current;
            bool inputStopped = false;
            try
            {
                using (var form = new NOMADSettingsForm(_profiles))
                {
                    if (form.ShowDialog() != DialogResult.OK || !IsActiveSession(session)) { return; }
                    bool connectionChanged;
                    try
                    {
                        form.Profiles.CheckForConnection(_profiles, MainV2.comPort?.BaseStream?.IsOpen == true);
                        _joystickService?.Stop();
                        inputStopped = true;
                        await (PayloadController.Current?.WaitForPendingAsync() ?? System.Threading.Tasks.Task.CompletedTask);
                        if (!IsActiveSession(session)) { return; }
                        connectionChanged = form.Profiles.SaveForConnection(_profiles, MainV2.comPort?.BaseStream?.IsOpen == true);
                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show(ex.Message, "Aircraft profiles were not saved", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        return;
                    }
                    StopHudVideo();
                    GimbalJoystickWindow.CloseSingleton();
                    _gimbalArrowKeyFilter?.Dispose();
                    _profiles = form.Profiles;
                    _config = _profiles.Selected.Config;
                    GimbalController.ApplyProfile(_config);
                    OutputController.Initialize(_config);
                    _termination?.Dispose();
                    _termination = new TerminationController(_config);
                    PayloadController.ApplyProfile(_config);
                    _gimbalArrowKeyFilter = new GimbalArrowKeyFilter(_profiles);
                    AudioAlerts.ApplyConfig(_config);
                    ApplyRouterClientSettings(connectionChanged);
                    try
                    {
                        _joystickService?.UpdateConfig(_config);
                    }
                    catch (Exception ex) { Log.Error($"Joystick restart failed — {ex.Message}"); }
                }
            }
            finally
            {
                if (inputStopped && IsActiveSession(session) && ReferenceEquals(previousConfig, _config) &&
                    ReferenceEquals(previousService, _joystickService) && OutputController.IsCurrentProfile(previousConfig) &&
                    ReferenceEquals(previousOwner, PayloadController.Current))
                {
                    try { previousService?.RestartWithConfig(); }
                    catch (Exception error) { Log.Error("Previous joystick restart failed: " + error.Message); }
                }
                _savingSettings = false;
            }
        }
    }
}
