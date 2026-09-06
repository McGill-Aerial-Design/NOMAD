// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Net.Http;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// HTTP-based communication methods for NOMAD Edge Core.
    /// </summary>
    public partial class DualLinkSender
    {
        private sealed class EdgeHealthResponse
        {
            [JsonProperty("cpu_load")]
            public float CpuLoad { get; set; }

            [JsonProperty("gpu_load")]
            public float GpuLoad { get; set; }

            [JsonProperty("cpu_temp")]
            public float CpuTemp { get; set; }

            [JsonProperty("gpu_temp")]
            public float GpuTemp { get; set; }

            [JsonProperty("memory_used_pct")]
            public float MemoryUsedPct { get; set; }

            [JsonProperty("disk_free_gb")]
            public float DiskFreeGb { get; set; }
        }

        /// <summary>
        /// Get health status from Jetson.
        /// Also updates IsJetsonConnected and LastHealthStatus properties.
        /// </summary>
        public async Task<CommandResult> GetHealthAsync()
        {
            var result = await SendAsync(HttpMethod.Get, "/health").ConfigureAwait(false);
            IsJetsonConnected = result.Success;

            if (!result.Success)
            {
                LastHealthStatus = null;
                result.Message = $"Health check error: {result.Message}";
                return result;
            }

            var data = HttpJson.Deserialize<EdgeHealthResponse>(result.Data);
            if (data != null)
            {
                LastHealthStatus = new JetsonHealthStatus
                {
                    CpuUsage = data.CpuLoad,
                    GpuUsage = data.GpuLoad,
                    CpuTemp = data.CpuTemp,
                    GpuTemp = data.GpuTemp,
                    MemoryUsed = data.MemoryUsedPct,
                    DiskFreeGb = data.DiskFreeGb,
                    Timestamp = DateTime.Now
                };
            }

            result.Message = "Health check OK";
            return result;
        }

        /// <summary>
        /// Get VIO status from Jetson.
        /// </summary>
        public async Task<CommandResult> GetVioStatusAsync()
        {
            return await SendAsync(HttpMethod.Get, "/api/vio/status", timeoutSeconds: 12);
        }

        /// <summary>
        /// Get VIO trajectory for visualization.
        /// </summary>
        public async Task<CommandResult> GetVioTrajectoryAsync(int maxPoints = 100)
        {
            return await SendAsync(HttpMethod.Get, $"/api/vio/trajectory?max_points={maxPoints}", timeoutSeconds: 12);
        }

        /// <summary>
        /// Clear VIO trajectory data on the Jetson.
        /// </summary>
        public async Task<CommandResult> ClearVioTrajectoryAsync()
        {
            return await SendAsync(HttpMethod.Delete, "/api/vio/trajectory");
        }

        /// <summary>
        /// Get Isaac ROS status.
        /// </summary>
        public async Task<CommandResult> GetIsaacStatusAsync()
        {
            return await SendAsync(HttpMethod.Get, "/api/isaac/status", timeoutSeconds: 15);
        }

        /// <summary>
        /// Get all services status.
        /// </summary>
        public async Task<CommandResult> GetServicesStatusAsync()
        {
            return await SendAsync(HttpMethod.Get, "/api/services/status", timeoutSeconds: 15);
        }

        /// <summary>
        /// Execute a terminal command on the Jetson.
        /// </summary>
        public async Task<CommandResult> ExecuteTerminalCommandAsync(string command, int timeout = 10)
        {
            // API expects command_name (whitelist key), not full command string
            return await SendAsync(HttpMethod.Post, "/api/terminal/run", new { command_name = command, timeout });
        }

        /// <summary>
        /// Execute a terminal command and parse the JSON response to extract stdout.
        /// The raw /api/terminal/run response is JSON: {success, stdout, stderr, return_code}.
        /// This helper extracts stdout into Data and maps command-level success.
        /// </summary>
        private async Task<CommandResult> ExecuteTerminalCommandParsedAsync(string commandName, int timeout = 10)
        {
            var result = await ExecuteTerminalCommandAsync(commandName, timeout);

            if (result.Success && !string.IsNullOrEmpty(result.Data))
            {
                try
                {
                    var json = JObject.Parse(result.Data);
                    var stdout = json["stdout"]?.Value<string>()?.Trim() ?? "";
                    var cmdSuccess = json["success"]?.Value<bool>() ?? false;

                    return new CommandResult
                    {
                        Success = cmdSuccess,
                        Message = stdout,
                        Data = stdout,
                        Method = result.Method
                    };
                }
                catch { /* fall through to raw result */ }
            }

            return result;
        }

        /// <summary>
        /// Start a service on the Jetson.
        /// </summary>
        public async Task<CommandResult> StartServiceAsync(string serviceName)
        {
            return await RunServiceCommandAsync("start", serviceName, 15);
        }

        /// <summary>
        /// Stop a service on the Jetson.
        /// </summary>
        public async Task<CommandResult> StopServiceAsync(string serviceName)
        {
            return await RunServiceCommandAsync("stop", serviceName, 15);
        }

        /// <summary>
        /// Restart a service on the Jetson.
        /// </summary>
        public async Task<CommandResult> RestartServiceAsync(string serviceName)
        {
            return await RunServiceCommandAsync("restart", serviceName, 15);
        }

        /// <summary>
        /// Get service status on the Jetson.
        /// </summary>
        public async Task<CommandResult> GetServiceStatusAsync(string serviceName)
        {
            return await RunServiceCommandAsync("status", serviceName, 5);
        }

        private static readonly Dictionary<string, string> ServiceCommandExceptions =
            new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase)
            {
                ["start:edge_core"] = "start_nomad",
                ["stop:edge_core"] = "stop_nomad",
                ["restart:edge_core"] = "restart_edge_core",
                ["restart:nomad"] = "restart_edge_core",
                ["restart:mediamtx"] = "restart_video",
                ["status:edge_core"] = "status_nomad",
                ["status:video_bridge"] = "status_video",
            };

        private static readonly HashSet<string> TerminalCommandNames =
            new HashSet<string>(StringComparer.OrdinalIgnoreCase)
            {
                "status_nomad",
                "status_mediamtx",
                "status_mavlink",
                "status_video",
                "status_isaac",
                "status_zed",
                "status_ros_vehicle",
                "status_nvblox",
                "status_novnc",
                "start_nomad",
                "stop_nomad",
                "restart_edge_core",
                "start_mediamtx",
                "stop_mediamtx",
                "restart_video",
                "start_mavlink",
                "stop_mavlink",
                "restart_mavlink",
                "start_video_bridge",
                "stop_video_bridge",
                "restart_video_bridge",
                "start_isaac",
                "stop_isaac",
                "restart_isaac",
                "restart_all",
            };

        private async Task<CommandResult> RunServiceCommandAsync(string action, string serviceName, int timeout)
        {
            string commandName = ResolveServiceCommand(action, serviceName);
            if (commandName == null)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = $"Service action '{action}:{serviceName}' not in whitelist",
                    Method = "HTTP"
                };
            }

            return await ExecuteTerminalCommandParsedAsync(commandName, timeout);
        }

        private static string ResolveServiceCommand(string action, string serviceName)
        {
            if (string.IsNullOrWhiteSpace(action) || string.IsNullOrWhiteSpace(serviceName)) return null;

            string service = serviceName.Trim().Replace("-", "_");
            string key = $"{action}:{service}";
            string commandName = ServiceCommandExceptions.TryGetValue(key, out var mapped)
                ? mapped
                : $"{action}_{service}";
            return TerminalCommandNames.Contains(commandName) ? commandName : null;
        }

        /// <summary>
        /// Start Isaac ROS container and services.
        /// </summary>
        public async Task<CommandResult> StartIsaacRosAsync()
        {
            return await SendAsync(HttpMethod.Post, "/api/isaac/start", timeoutSeconds: 60);
        }

        /// <summary>
        /// Stop Isaac ROS container and services.
        /// </summary>
        public async Task<CommandResult> StopIsaacRosAsync()
        {
            return await SendAsync(HttpMethod.Post, "/api/isaac/stop", timeoutSeconds: 60);
        }

        /// <summary>
        /// Start the C++ ROS adapter node inside the Isaac ROS container (idempotent).
        /// </summary>
        public async Task<CommandResult> StartRosBridgeAsync()
        {
            return await SendAsync(HttpMethod.Post, "/api/isaac/bridge/start", timeoutSeconds: 60);
        }

        /// <summary>
        /// Stop the C++ ROS adapter node inside the Isaac ROS container.
        /// </summary>
        public async Task<CommandResult> StopRosBridgeAsync()
        {
            return await SendAsync(HttpMethod.Post, "/api/isaac/bridge/stop");
        }

        // ============================================================
        // Video Bridges Control
        // ============================================================

        /// <summary>
        /// Get status of the single video stream from the Jetson.
        /// </summary>
        public async Task<CommandResult> GetVideoBridgesStatusAsync()
        {
            return await SendAsync(HttpMethod.Get, "/api/video/bridges", timeoutSeconds: 15);
        }

        /// <summary>
        /// Start the persistent video bridge instances.
        /// Uses the long-running HTTP client (30s timeout) because the video bridge
        /// startup involves docker cp, process launch, and a readiness check loop.
        /// </summary>
        public async Task<CommandResult> StartVideoBridgesAsync()
        {
            return await SendAsync(HttpMethod.Post, "/api/video/bridges/start", timeoutSeconds: 60);
        }

        // ============================================================
        // HTTP Communication
        // ============================================================

        private async Task<CommandResult> SendAsync(
            HttpMethod method,
            string endpoint,
            object body = null,
            int? timeoutSeconds = null)
        {
            CancellationTokenSource cts = null;
            try
            {
                var client = timeoutSeconds.HasValue ? JetsonApiService.LongRunClient : JetsonApiService.ApiClient;
                var request = new HttpRequestMessage(method, $"{JetsonApiService.BaseUrl}{endpoint}");
                if (body != null || method == HttpMethod.Post)
                {
                    request.Content = new StringContent(
                        JsonConvert.SerializeObject(body ?? new { }),
                        Encoding.UTF8,
                        "application/json");
                }

                var token = CancellationToken.None;
                if (timeoutSeconds.HasValue)
                {
                    cts = new CancellationTokenSource(TimeSpan.FromSeconds(Math.Max(1, timeoutSeconds.Value)));
                    token = cts.Token;
                }

                var response = await client.SendAsync(request, token).ConfigureAwait(false);
                var responseBody = await response.Content.ReadAsStringAsync().ConfigureAwait(false);
                return new CommandResult
                {
                    Success = response.IsSuccessStatusCode,
                    Message = response.IsSuccessStatusCode
                        ? $"HTTP {method.Method} successful"
                        : $"HTTP {(int)response.StatusCode}: {response.ReasonPhrase}",
                    Data = responseBody,
                    Method = "HTTP"
                };
            }
            catch (TaskCanceledException)
            {
                string timeout = timeoutSeconds.HasValue ? $" ({Math.Max(1, timeoutSeconds.Value)}s)" : "";
                return new CommandResult
                {
                    Success = false,
                    Message = $"HTTP request timed out{timeout}",
                    Method = "HTTP"
                };
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
            catch (Exception ex)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = $"HTTP error: {ex.Message}",
                    Method = "HTTP"
                };
            }
            finally
            {
                cts?.Dispose();
            }
        }
    }
}
