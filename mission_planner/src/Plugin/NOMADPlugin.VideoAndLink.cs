// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Mission Planner Plugin — Video & Dual Link
// ============================================================
// Target: Mission Planner 1.3.x
//
// Video streaming (HUD overlay) and MAVLink dual-link
// management for NOMAD.
// ============================================================

using System;
using System.Drawing;
using System.Net.Http;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;
using NOMAD.MissionPlanner.Core;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Video streaming and MAVLink dual-link management for NOMAD.
    /// </summary>
    public partial class NOMADPlugin
    {
        // ============================================================
        // Dual Link Settings
        // ============================================================

        /// <summary>
        /// Bring the live router in sync with the (now-saved) config. Handles
        /// all three transitions: enabled→disabled, disabled→enabled, and
        /// changes while still enabled (rebind sockets to new ports/bindings).
        /// </summary>
        private void ApplyDualLinkSettings()
        {
            try
            {
                if (!_config.DualLinkEnabled || !_config.RouterEnabled)
                {
                    if (_connectionManager != null)
                    {
                        _connectionManager.StopMonitoring();
                        _connectionManager.Dispose();
                        _connectionManager = null;
                        Log.Info("Dual link/router disabled — MAVLink sockets released for direct Mission Planner connection");
                    }
                    return;
                }

                if (_connectionManager == null)
                {
                    InitializeConnectionManager();
                    Log.Info("Dual link enabled — router started");
                    return;
                }

                _connectionManager.UpdateConfig(BuildLinkConfig());
                _connectionManager.RestartRouter();
                Log.Info("Router restarted with new config");
            }
            catch (Exception ex)
            {
                Log.Error($"Failed to apply dual link settings — {ex.Message}");
            }
            finally
            {
                // _connectionManager may have been recreated or nulled above;
                // refresh the statics so the next MainSwitcher-created NOMAD
                // screen (and its Links view) sees the live instance instead of
                // the stale one captured at plugin load.
                NOMADMainScreen.SetStaticConfig(_sender, _config, _connectionManager, _jetsonConnectionManager, _geofenceConfig, _boundaryMonitor);
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
                    Log.Info("HUD video: no video URL configured");
                    return;
                }

                // Ensure GStreamer is available
                GStreamer.GstLaunch = GStreamer.LookForGstreamer();
                if (!GStreamer.GstLaunchExists)
                {
                    Log.Warn("GStreamer not found, cannot start HUD video");
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

                Log.Debug($"Starting HUD video with pipeline: {pipeline}");

                global::MissionPlanner.GCSViews.FlightData.hudGStreamer.Start(pipeline);
                _hudVideoStarted = true;
            }
            catch (Exception ex)
            {
                Log.Error($"HUD video failed to start — {ex.Message}");
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
            }
            catch (Exception ex)
            {
                Log.Error($"HUD video failed to stop — {ex.Message}");
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
        /// Build the ConnectionConfig the router needs from the current NOMADConfig.
        /// Pulled out so both first-init and settings-save paths produce identical configs.
        /// </summary>
        private MAVLinkConnectionManager.ConnectionConfig BuildLinkConfig()
        {
            return new MAVLinkConnectionManager.ConnectionConfig
            {
                JetsonTailscaleIP = _config.TailscaleIP,
                LtePort = _config.LteMavlinkPort,
                LteRemoteHost = _config.LteRemoteHost,
                LteRemotePort = _config.LteRemotePort,
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
                RadioMasterBaudRate = _config.RadioMasterBaudRate,
                RadioMasterTcpHost = _config.RadioMasterTcpHost,
                RouterBindAddress = _config.RouterBindAddress,
                RouterLocalPort = _config.RouterLocalPort,
                RouterDedupEnabled = _config.RouterDedupEnabled,
                HeartbeatTimeoutSec = _config.MavlinkHeartbeatTimeout,
            };
        }

        private void InitializeConnectionManager()
        {
            try
            {
                var linkConfig = BuildLinkConfig();

                _connectionManager = new MAVLinkConnectionManager(linkConfig);

                _connectionManager.FailoverOccurred += (s, e) =>
                {
                    Log.Info($"Link failover {e.FromLink} → {e.ToLink}: {e.Reason}");

                    try
                    {
                        MainV2.comPort?.MAV?.cs?.messages?.Add((DateTime.Now,
                            $"NOMAD: Failover to {e.ToLink} - {e.Reason}"));
                    }
                    catch { }
                };

                _connectionManager.ActiveLinkChanged += (s, newLink) =>
                {
                    Log.Info($"Active link changed to {newLink}");
                };

                _connectionManager.StartMonitoring();
            }
            catch (Exception ex)
            {
                Log.Error($"Failed to initialize connection manager — {ex.Message}");
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
                Log.Error($"Failed to show link health panel — {ex.Message}");
            }
        }

        /// <summary>
        /// Get the connection manager instance (for external access).
        /// </summary>
        public MAVLinkConnectionManager ConnectionManager => _connectionManager;

        /// <summary>
        /// Push servo/spray calibration config to Jetson on every new connection so
        /// the Jetson always uses the settings configured in the NOMAD Settings UI
        /// rather than its own defaults (e.g. after a NOMAD restart on the Jetson).
        /// </summary>
        private void OnJetsonConnected_PushServoConfig(object sender, JetsonConnectionStateChangedEventArgs e)
        {
            if (e.NewState != JetsonConnectionState.Connected) return;

            var cfg = _config;
            _ = System.Threading.Tasks.Task.Run(async () =>
            {
                try
                {
                    var body = new System.Net.Http.StringContent(
                        Newtonsoft.Json.JsonConvert.SerializeObject(new
                        {
                            channel         = cfg.CameraTiltChannel,
                            pwm_down        = cfg.CameraTiltPwmMin,
                            pwm_neutral     = cfg.CameraTiltPwmNeutral,
                            pwm_up          = cfg.CameraTiltPwmMax,
                            angle_range_deg = cfg.CameraTiltAngleRange,
                        }),
                        System.Text.Encoding.UTF8,
                        "application/json");

                    var response = await JetsonApiService.PostAsync("/api/servo/camera/config", body);
                    Log.Debug($"Pushed servo config to Jetson — HTTP {(int)response.StatusCode}");

                    // The server consumes only the relay number (extra fields
                    // are rejected); all other spray settings are GCS-local.
                    var sprayBody = new System.Net.Http.StringContent(
                        Newtonsoft.Json.JsonConvert.SerializeObject(new
                        {
                            water_pump_relay_number = cfg.WaterPump()?.Channel ?? 0,
                        }),
                        System.Text.Encoding.UTF8,
                        "application/json");

                    var sprayResponse = await JetsonApiService.PostAsync("/api/spray/calibration", sprayBody);
                    Log.Debug($"Pushed spray calibration to Jetson — HTTP {(int)sprayResponse.StatusCode}");
                }
                catch (Exception ex)
                {
                    Log.Debug($"Failed to push Jetson calibration — {ex.Message}");
                }
            });
        }
    }
}
