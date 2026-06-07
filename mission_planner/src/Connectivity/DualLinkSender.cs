// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Dual Link Sender - HTTP and MAVLink Communication
// ============================================================
// Provides dual-path communication to NOMAD Edge Core:
// 1. HTTP: Direct API calls via Tailscale
// 2. ELRS/MAVLink: Custom MAVLink commands through telemetry
// ============================================================

using System;
using System.Net.Http;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using MissionPlanner;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Holds Jetson health status data from the Edge Core API.
    /// </summary>
    public class JetsonHealthStatus
    {
        public float CpuUsage { get; set; }
        public float GpuUsage { get; set; }
        public float CpuTemp { get; set; }
        public float GpuTemp { get; set; }
        public float MemoryUsed { get; set; }
        public float MemoryTotal { get; set; }
        public float DiskUsed { get; set; }
        public DateTime Timestamp { get; set; }
    }

    /// <summary>
    /// Handles dual-path communication to NOMAD Edge Core.
    /// Supports both HTTP (via Tailscale) and MAVLink (via ELRS).
    /// </summary>
    public partial class DualLinkSender : IDisposable
    {
        // ============================================================
        // Custom MAVLink Command IDs for NOMAD
        // ============================================================
        // Using user-defined command range: MAV_CMD_USER_1 to MAV_CMD_USER_5
        // Reference: https://mavlink.io/en/services/command.html

        /// <summary>MAV_CMD_SET_EKF_SOURCE_SET (42007) - Switch EKF source</summary>
        public const ushort CMD_SET_EKF_SOURCE = 42007;

        // Security: Whitelist of allowed service names (defense-in-depth).
        // Each name maps to a *_<name> command_name in Edge Core's
        // COMMAND_WHITELIST (see edge_core/api.py). The mapping happens in the
        // per-action switch statements below.
        private static readonly System.Collections.Generic.HashSet<string> ALLOWED_SERVICES = new System.Collections.Generic.HashSet<string>(StringComparer.OrdinalIgnoreCase)
        {
            "nomad", "edge_core",
            "mediamtx",
            "mavlink-router",
            "isaac",            // = whole Isaac ROS stack (container + zed + ros_http_bridge)
            "zed",              // = nomad-zed-wrapper.service
            "ros_bridge",       // = nomad-ros-http-bridge.service
            "video_bridge",     // = nomad-video-bridge.service
            "nvblox",           // = nomad-nvblox.service (opt-in)
            "all",              // = nomad.target (autostart set)
            "novnc"
        };

        // ============================================================
        // Fields
        // ============================================================

        private NOMADConfig _config;
        private bool _disposed;

        // ============================================================
        // Constructor
        // ============================================================

        public DualLinkSender(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
        }

        // ============================================================
        // Public Properties
        // ============================================================

        /// <summary>
        /// Gets whether ELRS/MAVLink mode is enabled.
        /// </summary>
        public bool UseELRS => _config.UseELRS;

        /// <summary>
        /// Gets the configured Jetson IP address.
        /// </summary>
        public string JetsonIP => _config.EffectiveIP;

        /// <summary>
        /// Gets whether the Jetson is currently connected/reachable.
        /// This is set by the last health check result.
        /// </summary>
        public bool IsJetsonConnected { get; private set; }

        /// <summary>
        /// Gets the last known Jetson health status.
        /// Updated by GetHealthAsync() calls.
        /// </summary>
        public JetsonHealthStatus LastHealthStatus { get; private set; }

        // ============================================================
        // Public Methods
        // ============================================================

        /// <summary>
        /// Update configuration at runtime.
        /// </summary>
        public void UpdateConfig(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));

            // Reconfigure centralized API service with updated config
            JetsonApiService.Reconfigure(_config);
        }

        // ============================================================
        // EKF Source Switching
        // ============================================================

        /// <summary>
        /// EKF source options for ArduPilot EK3_SRC parameters.
        /// </summary>
        public enum EkfSource
        {
            /// <summary>SRC1: GPS (outdoor)</summary>
            GPS = 1,
            /// <summary>SRC2: External Navigation / Vision (indoor)</summary>
            ExternalNav = 2,
            /// <summary>SRC3: Optical Flow</summary>
            OpticalFlow = 3
        }

        /// <summary>
        /// Switch ArduPilot EKF source using MAV_CMD_SET_EKF_SOURCE_SET (42007).
        /// RC9 is configured as source selector, but this allows GCS override.
        /// </summary>
        /// <param name="source">EKF source to switch to (1=GPS, 2=Vision, 3=OptFlow)</param>
        /// <returns>Command result</returns>
        public async Task<CommandResult> SetEkfSource(EkfSource source)
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = "MAVLink not connected",
                    Method = "MAVLink"
                };
            }

            // Serialize MAVLink writes via the process-wide s_mavlinkLock so
            // a concurrent joystick write cannot corrupt the serial stream.
            bool acquired = false;
            try
            {
                acquired = await CubeOutputController.MavlinkLock
                    .WaitAsync(5000).ConfigureAwait(false);
                if (!acquired)
                {
                    return new CommandResult
                    {
                        Success = false,
                        Message = "MAVLink port busy (lock timeout)",
                        Method = "MAVLink"
                    };
                }

                // Send MAV_CMD_SET_EKF_SOURCE_SET (42007)
                // param1 = source set number (1, 2, or 3)
                MainV2.comPort.doCommand(
                    MainV2.comPort.MAV.sysid,
                    MainV2.comPort.MAV.compid,
                    (MAVLink.MAV_CMD)CMD_SET_EKF_SOURCE,
                    (float)source,  // Source set number
                    0, 0, 0, 0, 0, 0
                );

                string sourceName = source switch
                {
                    EkfSource.GPS => "GPS (SRC1)",
                    EkfSource.ExternalNav => "Vision/VIO (SRC2)",
                    EkfSource.OpticalFlow => "Optical Flow (SRC3)",
                    _ => $"Unknown ({(int)source})"
                };

                return new CommandResult
                {
                    Success = true,
                    Message = $"EKF source switched to {sourceName}",
                    Method = "MAVLink"
                };
            }
            catch (Exception ex)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = $"EKF source switch failed: {ex.Message}",
                    Method = "MAVLink"
                };
            }
            finally
            {
                if (acquired) CubeOutputController.MavlinkLock.Release();
            }
        }

        /// <summary>
        /// Switch to GPS source (SRC1) - for outdoor flight.
        /// </summary>
        public Task<CommandResult> SetEkfSourceGPS() => SetEkfSource(EkfSource.GPS);

        /// <summary>
        /// Switch to External Navigation / Vision source (SRC2) - for indoor flight.
        /// </summary>
        public Task<CommandResult> SetEkfSourceVision() => SetEkfSource(EkfSource.ExternalNav);

        /// <summary>
        /// Switch to Optical Flow source (SRC3).
        /// </summary>
        public Task<CommandResult> SetEkfSourceOptFlow() => SetEkfSource(EkfSource.OpticalFlow);

        // ============================================================
        // MAVLink Communication
        // ============================================================

        private async Task<CommandResult> SendMAVLinkCommand(
            ushort commandId,
            float param1 = 0,
            float param2 = 0,
            float param3 = 0,
            float param4 = 0,
            float param5 = 0,
            float param6 = 0,
            float param7 = 0)
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = "MAVLink not connected",
                    Method = "MAVLink"
                };
            }

            // Serialize MAVLink writes via the shared s_mavlinkLock so this
            // call cannot interleave with GimbalJoystickWindow writes to the same serial port.
            bool acquired = false;
            try
            {
                acquired = await CubeOutputController.MavlinkLock
                    .WaitAsync(5000).ConfigureAwait(false);
                if (!acquired)
                {
                    return new CommandResult
                    {
                        Success = false,
                        Message = "MAVLink port busy (lock timeout)",
                        Method = "MAVLink"
                    };
                }

                var command = new MAVLink.mavlink_command_long_t
                {
                    target_system = MainV2.comPort.MAV.sysid,
                    target_component = MainV2.comPort.MAV.compid,
                    command = commandId,
                    confirmation = 0,
                    param1 = param1,
                    param2 = param2,
                    param3 = param3,
                    param4 = param4,
                    param5 = param5,
                    param6 = param6,
                    param7 = param7
                };

                MainV2.comPort.sendPacket(command, MainV2.comPort.MAV.sysid, MainV2.comPort.MAV.compid);

                return new CommandResult
                {
                    Success = true,
                    Message = $"MAVLink command {commandId} sent",
                    Method = "MAVLink"
                };
            }
            catch (Exception ex)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = $"MAVLink error: {ex.Message}",
                    Method = "MAVLink"
                };
            }
            finally
            {
                if (acquired) CubeOutputController.MavlinkLock.Release();
            }
        }

        // ============================================================
        // IDisposable
        // ============================================================

        public void Dispose()
        {
            if (!_disposed)
            {
                _disposed = true;
            }
        }
    }

    /// <summary>
    /// Result of a command sent via DualLinkSender.
    /// </summary>
    public class CommandResult
    {
        /// <summary>Whether the command was successful.</summary>
        public bool Success { get; set; }

        /// <summary>Human-readable status message.</summary>
        public string Message { get; set; }

        /// <summary>Raw response data (JSON for HTTP).</summary>
        public string Data { get; set; }

        /// <summary>Communication method used (HTTP or MAVLink).</summary>
        public string Method { get; set; }
    }
}
