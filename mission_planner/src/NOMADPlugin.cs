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
// - Full-page NOMAD control interface with tabs
// - Embedded video streaming
// - Jetson terminal access
// - System health monitoring
// - Dual-link communication (HTTP or MAVLink/ELRS)
// - Task 1: Capture snapshot command
// - Task 2: Reset exclusion map, WASD indoor control
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Plugin;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Main NOMAD plugin class implementing Mission Planner Plugin interface.
    /// </summary>
    public class NOMADPlugin : Plugin
    {
        // Plugin metadata
        public override string Name => "NOMAD Control";
        public override string Version => "3.0.0";
        public override string Author => "McGill Aerial Design";

        // Plugin state
        private NOMADConfig _config;
        private DualLinkSender _sender;
        private MAVLinkConnectionManager _connectionManager;  // Dual link manager
        private Form _popOutForm;                             // Pop-out window for NOMAD screen
        private bool _hudVideoStarted = false;
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
                // Load configuration
                _config = NOMADConfig.Load();
                
                // Initialize dual-link sender
                _sender = new DualLinkSender(_config);

                // Initialize MAVLink dual link connection manager
                if (_config.DualLinkEnabled)
                {
                    InitializeConnectionManager();
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
                                Console.WriteLine("NOMAD: Menu bar item clicked directly");
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
                            Console.WriteLine("NOMAD: Open NOMAD Screen clicked from dropdown");
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
                            $"McGill Aerial Design - AEAC 2026\n\n" +
                            $"Features:\n" +
                            $"- Full-page sidebar interface\n" +
                            $"- Dashboard with quick overview\n" +
                            $"- Embedded video streaming\n" +
                            $"- Jetson terminal access\n" +
                            $"- Real-time health monitoring\n" +
                            $"- MAVLink dual link failover\n" +
                            $"- Task 1 (Outdoor) & Task 2 (Indoor) support\n\n" +
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
                    Console.WriteLine($"NOMAD: Could not add main menu - {ex.Message}");
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
                Console.WriteLine($"NOMAD: Failed to create UI - {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Called periodically during FlightData updates.
        /// </summary>
        public override bool Loop()
        {
            // Feed MAVLink heartbeats to connection manager for link monitoring
            if (_connectionManager != null && MainV2.comPort?.BaseStream?.IsOpen == true)
            {
                // Determine which link this heartbeat came from based on connection
                // By convention: if using Tailscale IP endpoint, it's LTE; otherwise RadioMaster
                try
                {
                    var endpoint = MainV2.comPort.BaseStream.ToString();
                    var linkType = endpoint?.Contains(_config.TailscaleIP) == true 
                        ? LinkType.LTE 
                        : LinkType.RadioMaster;
                    _connectionManager.ProcessHeartbeat(linkType);
                }
                catch { /* ignore monitoring errors */ }
            }
            
            return true;
        }

        /// <summary>
        /// Called when plugin is unloaded.
        /// </summary>
        public override bool Exit()
        {
            try
            {
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

            _sender?.Dispose();
            return true;
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
                Console.WriteLine("NOMAD: Screen already registered");
                return;
            }
                
            try
            {
                Console.WriteLine("NOMAD: Registering screen...");
                
                // Set static configuration for the MainSwitcher-created instance
                NOMADMainScreen.SetStaticConfig(_sender, _config, _connectionManager);
                
                object mainSwitcher = null;
                
                // MainV2.View is a public static field that holds the MainSwitcher instance
                var viewField = typeof(MainV2).GetField("View", 
                    System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static);
                if (viewField != null)
                {
                    mainSwitcher = viewField.GetValue(null);
                    Console.WriteLine($"NOMAD: Got MainSwitcher from static View field: {mainSwitcher != null}");
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
                    // Get the Screen inner class type
                    var screenType = mainSwitcher.GetType().GetNestedType("Screen",
                        System.Reflection.BindingFlags.Public);
                    if (screenType != null)
                    {
                        // Create a new Screen instance: new MainSwitcher.Screen("NOMAD", typeof(NOMADMainScreen), false)
                        var screenCtor = screenType.GetConstructor(new Type[] { typeof(string), typeof(Type), typeof(bool) });
                        if (screenCtor != null)
                        {
                            var screen = screenCtor.Invoke(new object[] { "NOMAD", typeof(NOMADMainScreen), false });
                            
                            // Call AddScreen
                            var addScreenMethod = mainSwitcher.GetType().GetMethod("AddScreen",
                                System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance);
                            if (addScreenMethod != null)
                            {
                                addScreenMethod.Invoke(mainSwitcher, new object[] { screen });
                                _screenRegistered = true;
                                Console.WriteLine("NOMAD: Registered as top-level screen");
                            }
                        }
                    }
                }
                
                if (!_screenRegistered)
                {
                    Console.WriteLine("NOMAD: Could not register as top-level screen");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: RegisterNomadScreen failed - {ex.Message}");
            }
        }

        /// <summary>
        /// Shows the NOMAD interface as a top-level page in Mission Planner.
        /// Similar to FlightData, FlightPlan, InitialSetup, etc.
        /// </summary>
        private void ShowMainScreen()
        {
            Console.WriteLine("NOMAD: ShowMainScreen called");
            
            if (Host?.MainForm != null && Host.MainForm.InvokeRequired)
            {
                Console.WriteLine("NOMAD: Invoking on UI thread");
                Host.MainForm.BeginInvoke((MethodInvoker)delegate { ShowMainScreen(); });
                return;
            }

            // Ensure screen is registered
            if (!_screenRegistered)
            {
                Console.WriteLine("NOMAD: Screen not registered, registering now...");
                RegisterNomadScreen();
            }

            try
            {
                Console.WriteLine("NOMAD: Attempting to show NOMAD screen...");
                
                // Use MainV2.View (static field) to access the MainSwitcher
                var viewField = typeof(MainV2).GetField("View", 
                    System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static);
                if (viewField != null)
                {
                    var mainSwitcher = viewField.GetValue(null);
                    if (mainSwitcher != null)
                    {
                        Console.WriteLine("NOMAD: Got MainSwitcher, calling ShowScreen('NOMAD')");
                        var showScreenMethod = mainSwitcher.GetType().GetMethod("ShowScreen", 
                            System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance,
                            null, new Type[] { typeof(string) }, null);
                        if (showScreenMethod != null)
                        {
                            showScreenMethod.Invoke(mainSwitcher, new object[] { "NOMAD" });
                            Console.WriteLine("NOMAD: ShowScreen invoked successfully");
                        }
                        else
                        {
                            Console.WriteLine("NOMAD: ShowScreen method not found");
                        }
                    }
                    else
                    {
                        Console.WriteLine("NOMAD: MainSwitcher is null");
                    }
                }
                else
                {
                    Console.WriteLine("NOMAD: View field not found");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Could not show NOMAD screen - {ex.Message}");
                Console.WriteLine($"NOMAD: Stack trace - {ex.StackTrace}");
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
                    NOMADMainScreen.SetStaticConfig(_sender, _config, _connectionManager);
                    
                    var nomadScreen = new NOMADMainScreen(_sender, _config, _connectionManager);
                    
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
                Console.WriteLine($"NOMAD: Failed to show pop-out window - {ex.Message}");
                CustomMessageBox.Show($"Failed to open pop-out window: {ex.Message}", "Error");
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
                }
            }
        }
        
        // ============================================================
        // HUD Video Streaming
        // ============================================================
        
        /// <summary>
        /// Starts the ZED camera video stream on Mission Planner's HUD overlay.
        /// Uses the same GStreamer pipeline format as built-in HereLink support.
        /// </summary>
        public void StartHudVideo()
        {
            try
            {
                // Build the GStreamer pipeline using Mission Planner's expected format
                var streamUrl = _config.VideoUrl;
                if (string.IsNullOrWhiteSpace(streamUrl))
                {
                    Console.WriteLine("NOMAD HUD: No video URL configured");
                    return;
                }
                
                // Ensure GStreamer is available
                GStreamer.GstLaunch = GStreamer.LookForGstreamer();
                if (!GStreamer.GstLaunchExists)
                {
                    Console.WriteLine("NOMAD HUD: GStreamer not found, cannot start HUD video");
                    CustomMessageBox.Show(
                        "GStreamer is not installed. The HUD video requires GStreamer.\n\n" +
                        "You can install it via Tools > GStreamer in Mission Planner.",
                        "GStreamer Required"
                    );
                    return;
                }
                
                // Build pipeline - matches Mission Planner's proven format for RTSP streams
                // Key elements: decodebin3 for auto codec detection, queue with leaky for low latency
                string pipeline;
                int latency = 50; // Low latency for real-time video
                
                if (streamUrl.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
                {
                    // UDP RTP stream
                    var port = ExtractUdpPort(streamUrl);
                    pipeline = $"udpsrc port={port} buffer-size=90000 ! application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264 ! decodebin3 ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false";
                }
                else
                {
                    // RTSP stream - crop to left camera only (left half of 2560x720)
                    // Add videocrop after decoding to extract left 1280 pixels
                    pipeline = $"rtspsrc location={streamUrl} latency={latency} udp-reconnect=1 timeout=0 do-retransmission=false ! application/x-rtp ! decodebin3 ! queue max-size-buffers=1 leaky=2 ! videocrop right=1280 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false";
                }
                
                Console.WriteLine($"NOMAD HUD: Starting video with pipeline: {pipeline}");
                
                // Start video on the HUD using Mission Planner's static hudGStreamer
                // Use global:: to avoid namespace ambiguity with NOMAD.MissionPlanner
                global::MissionPlanner.GCSViews.FlightData.hudGStreamer.Start(pipeline);
                _hudVideoStarted = true;
                
                Console.WriteLine("NOMAD HUD: Video started successfully");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD HUD: Failed to start video - {ex.Message}");
                _hudVideoStarted = false;
            }
        }
        
        /// <summary>
        /// Stops the HUD video stream.
        /// </summary>
        public void StopHudVideo()
        {
            try
            {
                global::MissionPlanner.GCSViews.FlightData.hudGStreamer.Stop();
                _hudVideoStarted = false;
                Console.WriteLine("NOMAD HUD: Video stopped");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD HUD: Failed to stop video - {ex.Message}");
            }
        }
        
        /// <summary>
        /// Toggles HUD video on/off.
        /// </summary>
        public void ToggleHudVideo()
        {
            if (_hudVideoStarted)
            {
                StopHudVideo();
            }
            else
            {
                StartHudVideo();
            }
        }
        
        /// <summary>
        /// Extract UDP port from URL (udp://@:5600, udp://5600, etc.)
        /// </summary>
        private int ExtractUdpPort(string url)
        {
            try
            {
                var cleaned = url.Replace("udp://", "").Replace("@", "").TrimStart(':');
                if (int.TryParse(cleaned, out int port))
                {
                    return port;
                }
            }
            catch { }
            return 5600; // Default port
        }

        // ============================================================
        // MAVLink Dual Link Management
        // ============================================================

        /// <summary>
        /// Initialize the MAVLink connection manager for dual link failover.
        /// </summary>
        private void InitializeConnectionManager()
        {
            try
            {
                var linkConfig = new MAVLinkConnectionManager.ConnectionConfig
                {
                    JetsonTailscaleIP = _config.TailscaleIP,
                    LtePort = _config.LteMavlinkPort,
                    RadioMasterPort = _config.RadioMasterPort,
                    AutoFailoverEnabled = _config.AutoFailoverEnabled,
                    PreferredLink = _config.PreferredMavlinkLink switch
                    {
                        "LTE" => LinkType.LTE,
                        "RadioMaster" => LinkType.RadioMaster,
                        _ => LinkType.None
                    },
                    AutoReconnectPreferred = _config.AutoReconnectToPreferred,
                    PreferredLinkReconnectDelaySec = _config.PreferredLinkReconnectDelay,
                    MonitorIntervalMs = _config.LinkMonitorInterval,
                    RadioMasterConnectionType = _config.RadioMasterConnectionType,
                    RadioMasterComPort = _config.RadioMasterComPort,
                    RadioMasterBaudRate = _config.RadioMasterBaudRate
                };

                _connectionManager = new MAVLinkConnectionManager(linkConfig);
                
                // Subscribe to failover events for logging/notification
                _connectionManager.FailoverOccurred += (s, e) =>
                {
                    Console.WriteLine($"NOMAD: Link failover {e.FromLink} → {e.ToLink}: {e.Reason}");
                    
                    // Inject notification into Mission Planner's message system
                    try
                    {
                        MainV2.comPort?.MAV?.cs?.messages?.Add((DateTime.Now, 
                            $"NOMAD: Failover to {e.ToLink} - {e.Reason}"));
                    }
                    catch { }
                };

                _connectionManager.ActiveLinkChanged += (s, newLink) =>
                {
                    Console.WriteLine($"NOMAD: Active link changed to {newLink}");
                };

                // Start monitoring
                _connectionManager.StartMonitoring();
                Console.WriteLine("NOMAD: Dual link connection manager initialized");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to initialize connection manager - {ex.Message}");
            }
        }

        /// <summary>
        /// Shows the Link Health Panel in a separate window.
        /// </summary>
        private void ShowLinkHealthPanel()
        {
            if (Host?.MainForm != null && Host.MainForm.InvokeRequired)
            {
                Host.MainForm.BeginInvoke((MethodInvoker)delegate { ShowLinkHealthPanel(); });
                return;
            }

            try
            {
                if (_connectionManager == null)
                {
                    CustomMessageBox.Show(
                        "Dual link management is not enabled.\n\n" +
                        "Enable it in NOMAD Settings → Connection → Enable Dual Link.",
                        "Link Manager Not Available"
                    );
                    return;
                }

                var form = new Form
                {
                    Text = "NOMAD MAVLink Link Status",
                    Size = new Size(550, 650),
                    StartPosition = FormStartPosition.CenterParent,
                    BackColor = Color.FromArgb(30, 30, 30),
                    MinimumSize = new Size(500, 500),
                };

                var linkPanel = new LinkHealthPanel(_connectionManager, _config);
                linkPanel.Dock = DockStyle.Fill;
                form.Controls.Add(linkPanel);

                form.Show(Host.MainForm);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"NOMAD: Failed to show link health panel - {ex.Message}");
            }
        }

        /// <summary>
        /// Get the connection manager instance (for external access).
        /// </summary>
        public MAVLinkConnectionManager ConnectionManager => _connectionManager;
    }
}
