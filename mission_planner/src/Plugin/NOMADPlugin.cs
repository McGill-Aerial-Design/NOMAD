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
// - Authenticated runtime command boundary
// - Standalone ground-router management
// - Configurable payload controls
// ============================================================

using System;
using System.Drawing;
using System.IO;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Plugin;
using MissionPlanner.Utilities;
using NOMAD.MissionPlanner.Core;

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
        private NotificationService _notificationService;
        private GeofenceConfig _geofenceConfig;               // Plugin-owned: survives NOMAD screen disposal
        private BoundaryMonitor _boundaryMonitor;             // Plugin-owned: alerts fire on every MP page
        private MAVLinkConnectionManager _connectionManager;  // Standalone router management/status client
        private NomadJoystickService _joystickService;        // Physical joysticks → gimbal + camera tilt
        private GimbalArrowKeyFilter _gimbalArrowKeyFilter;    // Mission Planner-wide arrow key nudges
        private Form _popOutForm;                             // Pop-out window for NOMAD screen
        private HudVideoPlayer _hudVideo;
        private bool _hudVideoStarted => _hudVideo?.IsActive == true;
        private System.Threading.CancellationTokenSource _videoShutdown;
        private bool _screenRegistered = false;               // Track if NOMAD screen is registered with MainSwitcher
        private DateTime _nextBoundaryMapBindUtc = DateTime.MinValue;

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
                ShutdownVideo();
                _videoShutdown = new System.Threading.CancellationTokenSource();
                NOMADMainScreen.SetVideoShutdown(_videoShutdown.Token);
                WarnIfUntestedMissionPlannerVersion();

                // Load configuration
                _config = NOMADConfig.Load();

                // OutputController builds the loopback runtime client from the
                // saved port and local actuation gate. GuidedGoto remains unavailable.
                OutputController.Initialize(_config);

                // Notification service runs plugin-wide so battery / GPS
                // alerts (including audio + TTS) fire regardless of which NOMAD tab
                // is open — and even when the user is on a non-NOMAD MP screen.
                _notificationService = new NotificationService();
                NotificationService.Shared = _notificationService;
                _notificationService.StartMonitoring();

                // Geofence boundary monitor lives at plugin level so the
                // "Real-time Monitor" setting persists and violation alerts
                // keep firing even when the NOMAD screen is disposed (MainSwitcher
                // recreates it on every page switch).
                _geofenceConfig = GeofenceConfig.Load();
                _boundaryMonitor = new BoundaryMonitor(_geofenceConfig, _config);
                _notificationService.SetBoundaryMonitor(_boundaryMonitor);
                if (_geofenceConfig.MonitoringEnabled)
                {
                    _boundaryMonitor.StartMonitoring();
                }

                // Toast overlay: Warning/Critical notifications pop bottom-right
                // on every MP page, not just inside the NOMAD screen.
                NotificationToast.Attach(_notificationService, Host?.MainForm);

                // Apply saved audio preferences (master mute, altitude callouts)
                // before anything speaks.
                AudioAlerts.ApplyConfig(_config);

                // Startup chime + spoken welcome (fires once per process).
                AudioAlerts.PlayWelcomeOnce();

                // Initialize the standalone ground router management client
                if (_config.DualLinkEnabled)
                {
                    InitializeConnectionManager();
                }

                // Seed centralized gimbal rate from persisted config so the floating
                // gimbal window, the settings dialog, and the physical joystick
                // service all start with the same value (single source of truth lives
                // on GimbalController.MaxRateDegSec).
                GimbalController.MaxRateDegSec = _config.JoystickGimbalMaxRateDegSec;

                // Capture arrow keys at the application message-pump level so
                // focused buttons, grids, and non-NOMAD views cannot consume them
                // while the operator has enabled gimbal keyboard control.
                _gimbalArrowKeyFilter = new GimbalArrowKeyFilter(_config);

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
                            $"Click NOMAD in the menu bar to open the interface;\n" +
                            $"hover it for tools and settings.\n\n" +
                            $"C++ runtime IPC port: 127.0.0.1:{_config.CoreRuntimePort}",
                            "NOMAD"
                        );
                    });
                }

                return true;
            }
            catch (Exception ex)
            {
                ShutdownVideo();
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

                // Boundary visualization comes from the saved plugin config and
                // does not depend on a connected vehicle or a fence upload.
                MapOverlayManager.DrawBoundaries(_geofenceConfig);

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
            if (DateTime.UtcNow >= _nextBoundaryMapBindUtc)
            {
                _nextBoundaryMapBindUtc = DateTime.UtcNow.AddSeconds(2);
                var mainForm = Host?.MainForm;
                if (mainForm != null && !mainForm.IsDisposed && mainForm.IsHandleCreated)
                {
                    UiAsync.RunSync(mainForm, () =>
                    {
                        if (MapOverlayManager.BoundaryRenderingConfigured)
                            MapOverlayManager.EnsureBoundaryMaps();
                        else
                            MapOverlayManager.DrawBoundaries(_geofenceConfig);
                    }, "boundary map binding");
                }
            }
            return true;
        }

        /// <summary>
        /// Called when plugin is unloaded.
        /// </summary>
        public override bool Exit()
        {
            ShutdownVideo();
            try
            {
                // Unhook the toast overlay before the service goes away
                NotificationToast.Detach();
                MapOverlayManager.StopBoundaryRendering();

                // Stop boundary monitor (plugin-owned)
                _boundaryMonitor?.Dispose();
                _boundaryMonitor = null;

                // Stop notification service
                if (NotificationService.Shared == _notificationService) NotificationService.Shared = null;
                _notificationService?.StopMonitoring();
                _notificationService?.Dispose();
                _notificationService = null;

                // Stop connection manager monitoring
                _connectionManager?.StopMonitoring();
                _connectionManager?.Dispose();
                _connectionManager = null;

                // Release joystick devices
                _joystickService?.Dispose();
                _joystickService = null;

                _gimbalArrowKeyFilter?.Dispose();
                _gimbalArrowKeyFilter = null;

                // Kill serial bridge subprocess

                if (_popOutForm != null && !_popOutForm.IsDisposed)
                {
                    _popOutForm.Dispose();
                    _popOutForm = null;
                }

                _moduleHost?.StopAll();
                _moduleHost = null;
            }
            catch
            {
                // ignore disposal errors
            }

            return true;
        }

        // ============================================================
        // Module Host (NOMAD module SDK — see src/Core)
        // ============================================================

        /// <summary>
        /// Shared module context so the NOMADMainScreen pop-out and any registered
        /// modules can re-use plugin-level config (theme, runtime client credential, etc.) without
        /// requiring a full NOMADConfig instance. Built and cached on first read,
        /// with <see cref="EnvFlag"/> resolving module enable flags.
        /// </summary>
        internal static NomadModuleContext SharedContext
        {
            get
            {
                if (_sharedContext == null)
                    _sharedContext = new NomadModuleContext(EnvFlag);
                return _sharedContext;
            }
        }

        private static NomadModuleContext _sharedContext;

        private static ModuleHost _moduleHost;

        /// <summary>
        /// Build the module host once and return it. Register new modules here.
        /// Modules read services (e.g. the plugin config) from the shared context
        /// and contribute their own sidebar views/actions to the NOMAD screen.
        /// </summary>
        private ModuleHost BuildModuleHost()
        {
            if (_moduleHost != null) return _moduleHost;

            SharedContext.Register(_config); // expose the plugin config to modules

            var host = new ModuleHost();
            host.Register(new Modules.ExampleModule());
            host.Configure(SharedContext); // resolve order + Configure each module
            host.StartAll();

            _moduleHost = host;
            return host;
        }

        /// <summary>Resolve a module enable flag from an environment variable (true/false/null).</summary>
        private static bool? EnvFlag(string name)
        {
            var raw = Environment.GetEnvironmentVariable(name);
            if (string.IsNullOrEmpty(raw)) return null;
            switch (raw.Trim().ToLowerInvariant())
            {
                case "1":
                case "true":
                case "yes":
                case "on":
                    return true;
                case "0":
                case "false":
                case "no":
                case "off":
                    return false;
                default:
                    return null;
            }
        }

        // Screen registration and pop-out hosting live in NOMADPlugin.Screens.cs.

        private void ShowSettings()
        {
            using (var form = new NOMADSettingsForm(_config))
            {
                // Live serial bridge status indicator on the Joystick tab.

                if (form.ShowDialog() == DialogResult.OK)
                {
                    _config = form.Config;
                    _config.Save();
                    try
                    {
                        _joystickService?.Stop();
                    }
                    catch (Exception ex)
                    {
                        Log.Error($"Joystick stop before output reconfiguration failed — {ex.Message}");
                    }
                    OutputController.Initialize(_config);
                    ApplyRouterClientSettings();
                    try
                    {
                        _joystickService?.UpdateConfig(_config);
                    }
                    catch (Exception ex) { Log.Error($"Joystick restart failed — {ex.Message}"); }
                }
            }
        }
    }
}
