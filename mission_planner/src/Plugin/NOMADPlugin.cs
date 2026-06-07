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
// - Jetson terminal access
// - System health monitoring
// - Dual-link communication (HTTP or MAVLink/ELRS)
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
        public override string Version => "3.0.0";
        public override string Author => "McGill Aerial Design";

        // Plugin state
        private NOMADConfig _config;
        private NotificationService _notificationService;
        private DualLinkSender _sender;
        private MAVLinkConnectionManager _connectionManager;  // Dual link manager
        private JetsonConnectionManager _jetsonConnectionManager;  // Jetson HTTP connectivity
        private NomadJoystickService _joystickService;        // Physical joysticks → gimbal + ZED tilt
        private SerialJoystickBridge _serialBridge;           // Python subprocess: serial → virtual Xbox 360
        private Form _popOutForm;                             // Pop-out window for NOMAD screen
        private bool _hudVideoStarted = false;
        private bool _screenRegistered = false;               // Track if NOMAD screen is registered with MainSwitcher

        // Static assembly resolver for HelixToolkit dependencies
        private static bool _assemblyResolverRegistered = false;

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
                // Register assembly resolver for HelixToolkit dependencies (once)
                if (!_assemblyResolverRegistered)
                {
                    AppDomain.CurrentDomain.AssemblyResolve += (sender, args) =>
                    {
                        try
                        {
                            var assemblyName = new System.Reflection.AssemblyName(args.Name).Name;

                            // Check if it's a HelixToolkit assembly
                            if (assemblyName.StartsWith("HelixToolkit", StringComparison.OrdinalIgnoreCase))
                            {
                                // Look in the plugins folder
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

                // Version check warning - log if running on untested Mission Planner version
                try
                {
                var mpVersion = System.Reflection.Assembly.GetEntryAssembly()?.GetName()?.Version;
                if (mpVersion != null)
                {
                    if (mpVersion < new System.Version("1.3.80"))
                    {
                        Log.Warn($"Untested Mission Planner version {mpVersion}. Recommend 1.3.80+. Some features may not work correctly.");
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Warn($"Could not determine Mission Planner version — {ex.Message}");
            }

                // Load configuration
                _config = NOMADConfig.Load();

                // Initialize centralized API service (must be before any component that uses HttpClient)
                JetsonApiService.Initialize(_config);

                // Initialize dual-link sender
                _sender = new DualLinkSender(_config);

                // Notification service runs plugin-wide so battery / GPS
                // alerts (including audio + TTS) fire regardless of which NOMAD tab
                // is open — and even when the user is on a non-NOMAD MP screen.
                _notificationService = new NotificationService(null, _sender);
                NotificationService.Shared = _notificationService;
                _notificationService.StartMonitoring();

                // Startup chime + spoken welcome (fires once per process).
                AudioAlerts.PlayWelcomeOnce();

                // Initialize Jetson connection manager for non-blocking UI
                _jetsonConnectionManager = new JetsonConnectionManager(_config);
                _jetsonConnectionManager.ConnectionStateChanged += OnJetsonConnected_PushServoConfig;
                _jetsonConnectionManager.StartPolling();

                // Initialize MAVLink dual link connection manager
                if (_config.DualLinkEnabled && _config.RouterEnabled)
                {
                    InitializeConnectionManager();
                }

                // Serial → virtual Xbox 360 bridge — must start BEFORE the joystick
                // service so the virtual device is registered with Windows by the
                // time NomadJoystickService enumerates DirectInput devices.
                _serialBridge = new SerialJoystickBridge(_config);
                if (_config.SerialJoystickEnabled)
                {
                    try { _serialBridge.Start(); }
                    catch (Exception ex) { Log.Error($"Serial bridge start failed — {ex.Message}"); }
                }

                // Seed centralized gimbal rate from persisted config so the floating
                // gimbal window, the settings dialog, and the physical joystick
                // service all start with the same value (single source of truth lives
                // on GimbalController.MaxRateDegSec).
                GimbalController.MaxRateDegSec = _config.JoystickGimbalMaxRateDegSec;

                // Physical joystick service — starts only if either channel is enabled in config.
                _joystickService = new NomadJoystickService(_config);
                if (_joystickService.NeedsToRun())
                {
                    try { _joystickService.Start(); }
                    catch (Exception ex) { Log.Error($"Joystick service start failed — {ex.Message}"); }
                }

                // Add menu items to FlightData right-click menu (if available)
                try
                {
                    if (Host?.FDMenuMap?.Items != null)
                    {
                        var openMainItem = new ToolStripMenuItem("NOMAD Full Control");
                        openMainItem.Click += (s, e) => ShowMainScreen();
                        Host.FDMenuMap.Items.Add(openMainItem);

                        var openPopOutItem = new ToolStripMenuItem("NOMAD Pop Out Window");
                        openPopOutItem.Click += (s, e) => ShowPopOutWindow();
                        Host.FDMenuMap.Items.Add(openPopOutItem);

                        var settingsItem = new ToolStripMenuItem("NOMAD Settings");
                        settingsItem.Click += (s, e) => ShowSettings();
                        Host.FDMenuMap.Items.Add(settingsItem);

                        // Add link status menu item
                        var linkStatusItem = new ToolStripMenuItem("NOMAD Link Status");
                        linkStatusItem.Click += (s, e) => ShowLinkHealthPanel();
                        Host.FDMenuMap.Items.Add(linkStatusItem);
                    }
                }
                catch (Exception ex)
                {
                Log.Error($"Could not add FlightData menu — {ex.Message}");
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
                                ForeColor = Color.White,
                                BackColor = Color.FromArgb(0, 122, 204)
                            };
                            // Insert before Help menu (usually last)
                            int insertIndex = menuStrip.Items.Count - 1;
                            if (insertIndex < 0) insertIndex = 0;
                            menuStrip.Items.Insert(insertIndex, nomadMenu);
                        }

                        // Make clicking directly on "NOMAD" open the screen
                        // Use MouseDown event which fires before dropdown opens
                        nomadMenu.MouseDown += (s, e) =>
                        {
                            if (e.Button == MouseButtons.Left)
                            {
                                Log.Debug("Menu bar item clicked directly");
                                ShowMainScreen();
                            }
                        };

                        // Avoid duplicate items if plugin reloads
                        nomadMenu.DropDownItems.Clear();

                        // Open NOMAD Screen (Primary action - also in dropdown for accessibility)
                        var openMainItem = new ToolStripMenuItem("Open NOMAD Screen");
                        openMainItem.Font = new Font(openMainItem.Font, FontStyle.Bold);
                        openMainItem.Click += (s, e) =>
                        {
                            Log.Debug("Open NOMAD Screen clicked from dropdown");
                            ShowMainScreen();
                        };
                        nomadMenu.DropDownItems.Add(openMainItem);

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
                        hudVideoItem.Click += (s, e) => {
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
                            $"McGill Aerial Design\n\n" +
                            $"Features:\n" +
                            $"- Full-page sidebar interface\n" +
                            $"- Dashboard with quick overview\n" +
                            $"- Embedded video streaming\n" +
                            $"- Jetson terminal access\n" +
                            $"- Real-time health monitoring\n" +
                            $"- MAVLink dual link failover\n" +
                            $"- Configurable payload controls\n\n" +
                            $"Jetson: {_config.EffectiveIP}:{_config.JetsonPort}\n" +
                            $"Dual Link: {(_config.DualLinkEnabled ? "Enabled" : "Disabled")}\n" +
                            $"Mode: {(_config.UseELRS ? "ELRS/MAVLink" : "HTTP")}",
                            "About NOMAD"
                        );
                        nomadMenu.DropDownItems.Add(aboutItem);
                    }
                }
                catch (Exception ex)
                {
                Log.Error($"Could not add main menu — {ex.Message}");
            }

                // Keep startup non-blocking; show popup only in DebugMode
                if (_config.DebugMode)
                {
                    Host?.MainForm?.BeginInvoke((MethodInvoker)delegate
                    {
                        CustomMessageBox.Show(
                            $"NOMAD Plugin v{Version} loaded.\n\n" +
                            $"Use the NOMAD menu → Open NOMAD Tab\n" +
                            $"for the complete NOMAD interface.\n\n" +
                            $"Mode: {(_config.UseELRS ? "ELRS/MAVLink" : "HTTP")}\n" +
                            $"Jetson IP: {_config.EffectiveIP}",
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

                // Register NOMAD as a top-level screen (no quick tab - use pop-out instead)
                RegisterNomadScreen();

                // Auto-start HUD video if configured
                if (_config.AutoStartHudVideo && !_hudVideoStarted)
                {
                    // Delay slightly to ensure FlightData is fully loaded
                    System.Threading.Tasks.Task.Run(async () =>
                    {
                        await System.Threading.Tasks.Task.Delay(2000); // 2 second delay
                        Host?.MainForm?.BeginInvoke((MethodInvoker)delegate
                        {
                            StartHudVideo();
                        });
                    });
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
            // GroundLinkRouter owns the source sockets directly and derives
            // heartbeat/loss stats from real packet flow — nothing to do here.
            return true;
        }

        /// <summary>
        /// Called when plugin is unloaded.
        /// </summary>
        public override bool Exit()
        {
            try
            {
                // Stop notification service
                if (NotificationService.Shared == _notificationService) NotificationService.Shared = null;
                _notificationService?.StopMonitoring();
                _notificationService?.Dispose();
                _notificationService = null;

                // Stop Jetson connection manager
                _jetsonConnectionManager?.StopPolling();
                _jetsonConnectionManager?.Dispose();
                _jetsonConnectionManager = null;

                // Stop connection manager monitoring
                _connectionManager?.StopMonitoring();
                _connectionManager?.Dispose();
                _connectionManager = null;

                // Release joystick devices
                _joystickService?.Dispose();
                _joystickService = null;

                // Kill serial bridge subprocess
                _serialBridge?.Dispose();
                _serialBridge = null;

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

            _sender?.Dispose();
            JetsonApiService.Shutdown();
            return true;
        }

        // ============================================================
        // Module Host (NOMAD module SDK — see src/Core)
        // ============================================================

        /// <summary>
        /// Shared module context so the NOMADMainScreen pop-out and any registered
        /// modules can re-use plugin-level config (theme, API key, etc.) without
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

        // ============================================================
        // Private Methods
        // ============================================================

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
                NOMADMainScreen.SetStaticConfig(_sender, _config, _connectionManager, _jetsonConnectionManager);

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
                    NOMADMainScreen.SetStaticConfig(_sender, _config, _connectionManager, _jetsonConnectionManager);

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

        private void ShowSettings()
        {
            using (var form = new NOMADSettingsForm(_config))
            {
                // Live serial bridge status indicator on the Joystick tab.
                form.SetSerialBridgeStatusProvider(() => _serialBridge?.GetStatus() ?? "(no bridge instance)");

                if (form.ShowDialog() == DialogResult.OK)
                {
                    _config = form.Config;
                    _config.Save();
                    _sender.UpdateConfig(_config);
                    ApplyDualLinkSettings();
                    try { _serialBridge?.UpdateConfig(_config); }
                    catch (Exception ex) { Log.Error($"Serial bridge update failed — {ex.Message}"); }
                    try { _joystickService?.UpdateConfig(_config); }
                    catch (Exception ex) { Log.Error($"Joystick restart failed — {ex.Message}"); }
                }
            }
        }
    }
}
