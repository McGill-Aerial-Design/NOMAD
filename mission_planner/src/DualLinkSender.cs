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
using System.Threading.Tasks;
using MissionPlanner;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Handles dual-path communication to NOMAD Edge Core.
    /// Supports both HTTP (via Tailscale) and MAVLink (via ELRS).
    /// </summary>
    public class DualLinkSender : IDisposable
    {
        // ============================================================
        // Custom MAVLink Command IDs for NOMAD
        // ============================================================
        // Using user-defined command range: MAV_CMD_USER_1 to MAV_CMD_USER_5
        // Reference: https://mavlink.io/en/services/command.html
        
        /// <summary>Task 1: Capture snapshot (MAV_CMD_USER_1 = 31010)</summary>
        public const ushort CMD_NOMAD_TASK1_CAPTURE = 31010;
        
        /// <summary>Task 2: Reset exclusion map (MAV_CMD_USER_2 = 31011)</summary>
        public const ushort CMD_NOMAD_TASK2_RESET = 31011;
        
        /// <summary>Task 2: Register target hit (MAV_CMD_USER_3 = 31012)</summary>
        public const ushort CMD_NOMAD_TASK2_HIT = 31012;

        // ============================================================
        // Fields
        // ============================================================

        private NOMADConfig _config;
        private readonly HttpClient _httpClient;
        private bool _disposed;

        // ============================================================
        // Constructor
        // ============================================================

        public DualLinkSender(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            
            _httpClient = new HttpClient
            {
                Timeout = TimeSpan.FromSeconds(Math.Max(1, _config.HttpTimeoutSeconds))
            };
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
        public string JetsonIP => _config.JetsonIP;
        
        /// <summary>
        /// Gets whether the Jetson is currently connected/reachable.
        /// This is set by the last health check result.
        /// </summary>
        public bool IsJetsonConnected { get; private set; }

        // ============================================================
        // Public Methods
        // ============================================================

        /// <summary>
        /// Update configuration at runtime.
        /// </summary>
        public void UpdateConfig(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));

            // Keep the HTTP client consistent with updated config
            _httpClient.Timeout = TimeSpan.FromSeconds(Math.Max(1, _config.HttpTimeoutSeconds));
        }

        /// <summary>
        /// Send Task 1 Capture command.
        /// </summary>
        /// <param name="headingOverride">Optional heading override</param>
        /// <param name="gimbalPitchOverride">Optional gimbal pitch override</param>
        /// <param name="lidarDistanceOverride">Optional LiDAR distance override</param>
        /// <returns>Response from Edge Core or MAVLink ACK</returns>
        public async Task<CommandResult> SendTask1Capture(
            float? headingOverride = null,
            float? gimbalPitchOverride = null,
            float? lidarDistanceOverride = null)
        {
            if (_config.UseELRS)
            {
                return await SendMAVLinkCommand(
                    CMD_NOMAD_TASK1_CAPTURE,
                    headingOverride ?? float.NaN,
                    gimbalPitchOverride ?? float.NaN,
                    lidarDistanceOverride ?? float.NaN
                );
            }
            else
            {
                var body = new
                {
                    heading_deg = headingOverride,
                    gimbal_pitch_deg = gimbalPitchOverride,
                    lidar_distance_m = lidarDistanceOverride
                };
                return await SendHttpPost("/api/task/1/capture", body);
            }
        }

        /// <summary>
        /// Send Task 2 Reset Map command.
        /// </summary>
        /// <returns>Response from Edge Core or MAVLink ACK</returns>
        public async Task<CommandResult> SendTask2ResetMap()
        {
            if (_config.UseELRS)
            {
                return await SendMAVLinkCommand(CMD_NOMAD_TASK2_RESET);
            }
            else
            {
                return await SendHttpPost("/api/task/2/reset_map", null);
            }
        }

        /// <summary>
        /// Send Task 2 Target Hit command.
        /// </summary>
        /// <param name="x">X coordinate</param>
        /// <param name="y">Y coordinate</param>
        /// <param name="z">Z coordinate</param>
        /// <returns>Response from Edge Core or MAVLink ACK</returns>
        public async Task<CommandResult> SendTask2TargetHit(float x, float y, float z)
        {
            if (_config.UseELRS)
            {
                return await SendMAVLinkCommand(CMD_NOMAD_TASK2_HIT, x, y, z);
            }
            else
            {
                var body = new { x, y, z };
                return await SendHttpPost("/api/task/2/target_hit", body);
            }
        }

        /// <summary>
        /// Simplified async wrapper for Task 1 capture (no overrides).
        /// </summary>
        public async Task<CommandResult> SendTask1CaptureAsync()
        {
            return await SendTask1Capture();
        }

        /// <summary>
        /// Simplified async wrapper for Task 2 reset map.
        /// </summary>
        public async Task<CommandResult> SendTask2ResetAsync()
        {
            return await SendTask2ResetMap();
        }

        /// <summary>
        /// Get health status from Jetson.
        /// Also updates IsJetsonConnected property.
        /// </summary>
        public async Task<CommandResult> GetHealthAsync()
        {
            try
            {
                var url = $"http://{_config.JetsonIP}:{_config.JetsonPort}/health";
                var response = await _httpClient.GetAsync(url);
                var responseBody = await response.Content.ReadAsStringAsync();

                // Update connection status
                IsJetsonConnected = response.IsSuccessStatusCode;
                
                return new CommandResult
                {
                    Success = response.IsSuccessStatusCode,
                    Message = response.IsSuccessStatusCode ? "Health check OK" : "Health check failed",
                    Data = responseBody,
                    Method = "HTTP"
                };
            }
            catch (Exception ex)
            {
                // Update connection status - failed to reach Jetson
                IsJetsonConnected = false;
                
                return new CommandResult
                {
                    Success = false,
                    Message = $"Health check error: {ex.Message}",
                    Method = "HTTP"
                };
            }
        }

        /// <summary>
        /// Reset VIO origin on Jetson.
        /// </summary>
        public async Task<CommandResult> ResetVioOriginAsync()
        {
            return await SendHttpPost("/api/vio/reset_origin", null);
        }

        /// <summary>
        /// Get VIO status from Jetson.
        /// </summary>
        public async Task<CommandResult> GetVioStatusAsync()
        {
            return await SendHttpGet("/api/vio/status");
        }

        /// <summary>
        /// Get VIO trajectory for visualization.
        /// </summary>
        public async Task<CommandResult> GetVioTrajectoryAsync(int maxPoints = 100)
        {
            return await SendHttpGet($"/api/vio/trajectory?max_points={maxPoints}");
        }

        /// <summary>
        /// Get Isaac ROS status.
        /// </summary>
        public async Task<CommandResult> GetIsaacStatusAsync()
        {
            return await SendHttpGet("/api/isaac/status");
        }

        /// <summary>
        /// Get all services status.
        /// </summary>
        public async Task<CommandResult> GetServicesStatusAsync()
        {
            return await SendHttpGet("/api/services/status");
        }

        /// <summary>
        /// Execute a terminal command on the Jetson.
        /// </summary>
        public async Task<CommandResult> ExecuteTerminalCommandAsync(string command, int timeout = 10)
        {
            return await SendHttpPost("/api/terminal/exec", new { command, timeout });
        }

        /// <summary>
        /// Start a service on the Jetson.
        /// </summary>
        public async Task<CommandResult> StartServiceAsync(string serviceName)
        {
            return await ExecuteTerminalCommandAsync($"sudo systemctl start {serviceName}", 15);
        }

        /// <summary>
        /// Stop a service on the Jetson.
        /// </summary>
        public async Task<CommandResult> StopServiceAsync(string serviceName)
        {
            return await ExecuteTerminalCommandAsync($"sudo systemctl stop {serviceName}", 15);
        }

        /// <summary>
        /// Restart a service on the Jetson.
        /// </summary>
        public async Task<CommandResult> RestartServiceAsync(string serviceName)
        {
            return await ExecuteTerminalCommandAsync($"sudo systemctl restart {serviceName}", 15);
        }

        /// <summary>
        /// Get service status on the Jetson.
        /// </summary>
        public async Task<CommandResult> GetServiceStatusAsync(string serviceName)
        {
            return await ExecuteTerminalCommandAsync($"systemctl is-active {serviceName}", 5);
        }

        /// <summary>
        /// Check if Isaac ROS container is running.
        /// </summary>
        public async Task<CommandResult> GetIsaacRosContainerStatusAsync()
        {
            return await ExecuteTerminalCommandAsync("docker ps --filter name=nomad_isaac_ros --format '{{.Status}}'", 5);
        }

        /// <summary>
        /// Start Isaac ROS container and services.
        /// </summary>
        public async Task<CommandResult> StartIsaacRosAsync()
        {
            return await SendHttpPost("/api/isaac/start", null);
        }

        /// <summary>
        /// Stop Isaac ROS container and services.
        /// </summary>
        public async Task<CommandResult> StopIsaacRosAsync()
        {
            return await SendHttpPost("/api/isaac/stop", null);
        }

        /// <summary>
        /// Get Isaac ROS logs.
        /// </summary>
        public async Task<CommandResult> GetIsaacRosLogsAsync(string logType = "all")
        {
            return await SendHttpGet($"/api/isaac/logs?log_type={logType}");
        }

        // ============================================================
        // HTTP Communication
        // ============================================================

        private async Task<CommandResult> SendHttpGet(string endpoint)
        {
            try
            {
                var url = $"http://{_config.JetsonIP}:{_config.JetsonPort}{endpoint}";
                var response = await _httpClient.GetAsync(url);
                var responseBody = await response.Content.ReadAsStringAsync();

                if (response.IsSuccessStatusCode)
                {
                    return new CommandResult
                    {
                        Success = true,
                        Message = "HTTP GET successful",
                        Data = responseBody,
                        Method = "HTTP"
                    };
                }
                else
                {
                    return new CommandResult
                    {
                        Success = false,
                        Message = $"HTTP {(int)response.StatusCode}: {response.ReasonPhrase}",
                        Data = responseBody,
                        Method = "HTTP"
                    };
                }
            }
            catch (Exception ex)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = $"HTTP error: {ex.Message}",
                    Method = "HTTP"
                };
            }
        }

        private async Task<CommandResult> SendHttpPost(string endpoint, object body)
        {
            try
            {
                var url = $"http://{_config.JetsonIP}:{_config.JetsonPort}{endpoint}";
                
                var content = body != null
                    ? new StringContent(
                        JsonConvert.SerializeObject(body),
                        Encoding.UTF8,
                        "application/json")
                    : new StringContent("{}", Encoding.UTF8, "application/json");

                var response = await _httpClient.PostAsync(url, content);
                var responseBody = await response.Content.ReadAsStringAsync();

                if (response.IsSuccessStatusCode)
                {
                    return new CommandResult
                    {
                        Success = true,
                        Message = "HTTP request successful",
                        Data = responseBody,
                        Method = "HTTP"
                    };
                }
                else
                {
                    return new CommandResult
                    {
                        Success = false,
                        Message = $"HTTP {(int)response.StatusCode}: {response.ReasonPhrase}",
                        Data = responseBody,
                        Method = "HTTP"
                    };
                }
            }
            catch (HttpRequestException ex)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = $"HTTP connection failed: {ex.Message}",
                    Method = "HTTP"
                };
            }
            catch (TaskCanceledException)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = "HTTP request timed out",
                    Method = "HTTP"
                };
            }
            catch (Exception ex)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = $"HTTP error: {ex.Message}",
                    Method = "HTTP"
                };
            }
        }

        // ============================================================
        // MAVLink Communication
        // ============================================================

        private Task<CommandResult> SendMAVLinkCommand(
            ushort commandId,
            float param1 = 0,
            float param2 = 0,
            float param3 = 0,
            float param4 = 0,
            float param5 = 0,
            float param6 = 0,
            float param7 = 0)
        {
            try
            {
                // Check if connected
                if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
                {
                    return Task.FromResult(new CommandResult
                    {
                        Success = false,
                        Message = "MAVLink not connected",
                        Method = "MAVLink"
                    });
                }

                // Build COMMAND_LONG message
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

                // Send the packet
                MainV2.comPort.sendPacket(command, MainV2.comPort.MAV.sysid, MainV2.comPort.MAV.compid);

                // Note: For proper ACK handling, would need to wait for
                // COMMAND_ACK message. Simplified implementation here.
                return Task.FromResult(new CommandResult
                {
                    Success = true,
                    Message = $"MAVLink command {commandId} sent",
                    Method = "MAVLink"
                });
            }
            catch (Exception ex)
            {
                return Task.FromResult(new CommandResult
                {
                    Success = false,
                    Message = $"MAVLink error: {ex.Message}",
                    Method = "MAVLink"
                });
            }
        }

        // ============================================================
        // IDisposable
        // ============================================================

        public void Dispose()
        {
            if (!_disposed)
            {
                _httpClient?.Dispose();
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
