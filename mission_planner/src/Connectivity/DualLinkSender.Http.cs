// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

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
    /// HTTP-based communication methods for NOMAD Edge Core.
    /// </summary>
    public partial class DualLinkSender
    {
        /// <summary>
        /// Get health status from Jetson.
        /// Also updates IsJetsonConnected and LastHealthStatus properties.
        /// </summary>
        public async Task<CommandResult> GetHealthAsync()
        {
            try
            {
                var response = await JetsonApiService.GetAsync("/health");
                var responseBody = await response.Content.ReadAsStringAsync();

                // Update connection status
                IsJetsonConnected = response.IsSuccessStatusCode;

                // Parse and store health data
                if (response.IsSuccessStatusCode && !string.IsNullOrEmpty(responseBody))
                {
                    try
                    {
                        var data = JsonConvert.DeserializeObject<dynamic>(responseBody);
                        LastHealthStatus = new JetsonHealthStatus
                        {
                            // Match actual API field names from /health endpoint
                            CpuUsage = data.cpu_load ?? data.cpu_usage_pct ?? 0,
                            GpuUsage = data.gpu_load ?? data.gpu_usage_pct ?? 0,
                            CpuTemp = data.cpu_temp ?? data.cpu_temp_c ?? 0,
                            GpuTemp = data.gpu_temp ?? data.gpu_temp_c ?? 0,
                            MemoryUsed = data.memory_used_pct ?? data.memory_used_mb ?? 0,
                            MemoryTotal = 100, // memory_used_pct is already a percentage
                            DiskUsed = 100 - ((float)(data.disk_free_gb ?? 800) / 1000 * 100), // Approx: 1TB disk
                            Timestamp = DateTime.Now
                        };
                    }
                    catch (Exception parseEx)
                    {
                        System.Diagnostics.Debug.WriteLine($"Health parse error: {parseEx.Message}");
                    }
                }

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
                LastHealthStatus = null;

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
            return await SendHttpGetLongRun("/api/vio/status", 12);
        }

        /// <summary>
        /// Get VIO trajectory for visualization.
        /// </summary>
        public async Task<CommandResult> GetVioTrajectoryAsync(int maxPoints = 100)
        {
            return await SendHttpGetLongRun($"/api/vio/trajectory?max_points={maxPoints}", 12);
        }

        /// <summary>
        /// Clear VIO trajectory data on the Jetson.
        /// </summary>
        public async Task<CommandResult> ClearVioTrajectoryAsync()
        {
            return await SendHttpDelete("/api/vio/trajectory");
        }

        /// <summary>
        /// Get Isaac ROS status.
        /// </summary>
        public async Task<CommandResult> GetIsaacStatusAsync()
        {
            return await SendHttpGetLongRun("/api/isaac/status", 15);
        }

        /// <summary>
        /// Get all services status.
        /// </summary>
        public async Task<CommandResult> GetServicesStatusAsync()
        {
            return await SendHttpGetLongRun("/api/services/status", 15);
        }

        /// <summary>
        /// Execute a terminal command on the Jetson.
        /// </summary>
        public async Task<CommandResult> ExecuteTerminalCommandAsync(string command, int timeout = 10)
        {
            // API expects command_name (whitelist key), not full command string
            return await SendHttpPost("/api/terminal/run", new { command_name = command, timeout });
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
            if (!ALLOWED_SERVICES.Contains(serviceName))
                return new CommandResult { Success = false, Message = $"Service '{serviceName}' not in whitelist" };

            // Map service names to whitelisted command names
            string commandName = serviceName switch
            {
                "mediamtx" => "start_mediamtx",
                "mavlink-router" => "start_mavlink",
                "nomad" or "edge_core" => "start_nomad",
                "isaac" => "start_isaac",
                "zed" => "start_zed",
                "ros_bridge" => "start_ros_bridge",
                "video_bridge" => "start_video_bridge",
                "nvblox" => "start_nvblox",
                "all" => "start_all",
                "novnc" => "start_novnc",
                _ => $"start_{serviceName}"
            };

            return await ExecuteTerminalCommandParsedAsync(commandName, 15);
        }

        /// <summary>
        /// Stop a service on the Jetson.
        /// </summary>
        public async Task<CommandResult> StopServiceAsync(string serviceName)
        {
            if (!ALLOWED_SERVICES.Contains(serviceName))
                return new CommandResult { Success = false, Message = $"Service '{serviceName}' not in whitelist" };

            // Map service names to whitelisted command names
            string commandName = serviceName switch
            {
                "mediamtx" => "stop_mediamtx",
                "mavlink-router" => "stop_mavlink",
                "nomad" or "edge_core" => "stop_nomad",
                "isaac" => "stop_isaac",
                "zed" => "stop_zed",
                "ros_bridge" => "stop_ros_bridge",
                "video_bridge" => "stop_video_bridge",
                "nvblox" => "stop_nvblox",
                "all" => "stop_all",
                "novnc" => "stop_novnc",
                _ => $"stop_{serviceName}"
            };

            return await ExecuteTerminalCommandParsedAsync(commandName, 15);
        }

        /// <summary>
        /// Restart a service on the Jetson.
        /// </summary>
        public async Task<CommandResult> RestartServiceAsync(string serviceName)
        {
            if (!ALLOWED_SERVICES.Contains(serviceName))
                return new CommandResult { Success = false, Message = $"Service '{serviceName}' not in whitelist" };

            // Map service names to whitelisted command names
            string commandName = serviceName switch
            {
                "mediamtx" => "restart_video",            // restarts mediamtx + video_bridge
                "mavlink-router" => "restart_mavlink",
                "nomad" or "edge_core" => "restart_edge_core",
                "isaac" => "restart_isaac",
                "zed" => "restart_zed",
                "ros_bridge" => "restart_ros_bridge",
                "video_bridge" => "restart_video_bridge",
                "nvblox" => "restart_nvblox",
                "all" => "restart_all",
                _ => $"restart_{serviceName}" // Fallback
            };

            return await ExecuteTerminalCommandParsedAsync(commandName, 15);
        }

        /// <summary>
        /// Get service status on the Jetson.
        /// </summary>
        public async Task<CommandResult> GetServiceStatusAsync(string serviceName)
        {
            if (!ALLOWED_SERVICES.Contains(serviceName))
                return new CommandResult { Success = false, Message = $"Service '{serviceName}' not in whitelist" };

            // Map service names to whitelisted command names
            string commandName = serviceName switch
            {
                "mediamtx" => "status_mediamtx",
                "mavlink-router" => "status_mavlink",
                "nomad" or "edge_core" => "status_nomad",
                "isaac" => "status_isaac",
                "zed" => "status_zed",
                "ros_bridge" => "status_ros_bridge",
                "video_bridge" => "status_video",
                "nvblox" => "status_nvblox",
                "novnc" => "status_novnc",
                _ => $"status_{serviceName}" // Fallback
            };

            return await ExecuteTerminalCommandParsedAsync(commandName, 5);
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
            return await SendHttpPostLongRun("/api/isaac/start", null);
        }

        /// <summary>
        /// Stop Isaac ROS container and services.
        /// </summary>
        public async Task<CommandResult> StopIsaacRosAsync()
        {
            return await SendHttpPostLongRun("/api/isaac/stop", null);
        }

        /// <summary>
        /// Launch nvblox + ROS-HTTP bridge inside a running container.
        /// Lightweight: does not install deps or rebuild.
        /// </summary>
        public async Task<CommandResult> LaunchNvbloxAsync()
        {
            return await SendHttpPostLongRun("/api/isaac/launch-nvblox", null);
        }

        /// <summary>
        /// Start the ROS HTTP bridge inside the Isaac ROS container (idempotent).
        /// </summary>
        public async Task<CommandResult> StartRosBridgeAsync()
        {
            return await SendHttpPostLongRun("/api/isaac/bridge/start", null);
        }

        /// <summary>
        /// Stop the ROS HTTP bridge inside the Isaac ROS container.
        /// </summary>
        public async Task<CommandResult> StopRosBridgeAsync()
        {
            return await SendHttpPost("/api/isaac/bridge/stop", null);
        }

        /// <summary>
        /// Stop nvblox without stopping the container.
        /// </summary>
        public async Task<CommandResult> StopNvbloxAsync()
        {
            return await SendHttpPostLongRun("/api/isaac/stop-nvblox", null);
        }

        /// <summary>
        /// Stop SLAM resources by stopping nvblox.
        /// </summary>
        public async Task<CommandResult> StopSlamAsync()
        {
            return await StopNvbloxAsync();
        }

        /// <summary>
        /// Get Isaac ROS logs.
        /// </summary>
        public async Task<CommandResult> GetIsaacRosLogsAsync(string logType = "all")
        {
            return await SendHttpGetLongRun($"/api/isaac/logs?log_type={logType}", 15);
        }

        // ============================================================
        // Video Bridges Control
        // ============================================================

        /// <summary>
        /// Get status of all video bridge instances (primary/secondary).
        /// </summary>
        public async Task<CommandResult> GetVideoBridgesStatusAsync()
        {
            return await SendHttpGetLongRun("/api/video/bridges", 15);
        }

        /// <summary>
        /// Start the persistent video bridge instances.
        /// Uses the long-running HTTP client (30s timeout) because the video bridge
        /// startup involves docker cp, process launch, and a readiness check loop.
        /// </summary>
        public async Task<CommandResult> StartVideoBridgesAsync()
        {
            return await SendHttpPostLongRun("/api/video/bridges/start", null);
        }

        // ============================================================
        // SLAM Control
        // ============================================================

        /// <summary>
        /// Get SLAM/nvblox mapping status.
        /// </summary>
        public async Task<CommandResult> GetSlamStatusAsync()
        {
            return await SendHttpGetLongRun("/api/slam/status", 15);
        }

        /// <summary>
        /// Clear SLAM mesh data.
        /// </summary>
        public async Task<CommandResult> ClearSlamAsync()
        {
            return await SendHttpPostLongRun("/api/slam/clear?prefer_load_map=true&auto_create_empty_map_if_missing=true", null);
        }

        /// <summary>
        /// Save the ZED positional tracking area map to a Jetson SSD path.
        /// </summary>
        public async Task<CommandResult> SaveAreaMapAsync(string filePath)
        {
            var body = new
            {
                file_path = filePath,
                wait_for_completion = true,
                timeout_s = 30.0,
            };
            return await SendHttpPostLongRun("/api/vio/area/save", body);
        }

        /// <summary>
        /// Load a previously saved area map for relocalization.
        /// </summary>
        public async Task<CommandResult> LoadAreaMapAsync(string filePath)
        {
            var body = new
            {
                file_path = filePath,
            };
            return await SendHttpPostLongRun("/api/vio/area/load", body);
        }

        /// <summary>
        /// Load an area map and immediately attempt relocalization.
        /// </summary>
        public async Task<CommandResult> RelocalizeAreaMapAsync(string filePath)
        {
            var body = new
            {
                file_path = filePath,
            };
            return await SendHttpPostLongRun("/api/vio/area/relocalize", body);
        }

        // ============================================================
        // HTTP Communication
        // ============================================================

        private static CommandResult ToCommandResult(
            bool success, string body, string error, string successMessage)
        {
            return new CommandResult
            {
                Success = success,
                Message = success ? successMessage : error,
                Data = body,
                Method = "HTTP"
            };
        }

        private async Task<CommandResult> SendHttpGet(string endpoint)
        {
            var (success, body, error) = await HttpJson.TryGetAsync(
                JetsonApiService.ApiClient, $"{JetsonApiService.BaseUrl}{endpoint}");
            return ToCommandResult(success, body, error, "HTTP GET successful");
        }

        private async Task<CommandResult> SendHttpGetLongRun(string endpoint, int timeoutSeconds = 15)
        {
            CancellationTokenSource cts = null;
            try
            {
                cts = new CancellationTokenSource(TimeSpan.FromSeconds(Math.Max(1, timeoutSeconds)));
                var response = await JetsonApiService.LongRunClient.GetAsync(
                    $"{JetsonApiService.BaseUrl}{endpoint}",
                    cts.Token
                );
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

                return new CommandResult
                {
                    Success = false,
                    Message = $"HTTP {(int)response.StatusCode}: {response.ReasonPhrase}",
                    Data = responseBody,
                    Method = "HTTP"
                };
            }
            catch (TaskCanceledException)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = $"HTTP request timed out ({Math.Max(1, timeoutSeconds)}s)",
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

        private async Task<CommandResult> SendHttpDelete(string endpoint)
        {
            try
            {
                var response = await JetsonApiService.DeleteAsync(endpoint);
                var body = await response.Content.ReadAsStringAsync();
                return ToCommandResult(
                    response.IsSuccessStatusCode, body,
                    $"HTTP {(int)response.StatusCode}: {response.ReasonPhrase}",
                    "HTTP DELETE successful");
            }
            catch (Exception ex)
            {
                return ToCommandResult(false, null, $"HTTP error: {ex.Message}", null);
            }
        }

        private async Task<CommandResult> SendHttpPost(string endpoint, object body)
        {
            var (success, responseBody, error) = await HttpJson.TryPostAsync(
                JetsonApiService.ApiClient, $"{JetsonApiService.BaseUrl}{endpoint}", body);
            return ToCommandResult(success, responseBody, error, "HTTP request successful");
        }

        /// <summary>
        /// Send an HTTP POST using the long-running client (60s timeout).
        /// Use for operations that take longer than the standard timeout,
        /// such as starting video bridges, Isaac ROS, etc.
        /// </summary>
        private async Task<CommandResult> SendHttpPostLongRun(string endpoint, object body)
        {
            try
            {
                var content = body != null
                    ? new StringContent(
                        JsonConvert.SerializeObject(body),
                        Encoding.UTF8,
                        "application/json")
                    : new StringContent("{}", Encoding.UTF8, "application/json");

                var response = await JetsonApiService.PostLongRunAsync(endpoint, content);
                var responseBody = await response.Content.ReadAsStringAsync();

                if (response.IsSuccessStatusCode)
                {
                    // Check for application-level failure: server returns HTTP 200
                    // with {"success": false, "error": "..."} for long-running ops.
                    string appError = null;
                    try
                    {
                        var json = JObject.Parse(responseBody);
                        if (json["success"]?.Value<bool>() == false)
                            appError = json["error"]?.Value<string>() ?? json["message"]?.Value<string>() ?? "Server reported failure";
                    }
                    catch { }

                    if (appError != null)
                    {
                        return new CommandResult
                        {
                            Success = false,
                            Message = appError,
                            Data = responseBody,
                            Method = "HTTP"
                        };
                    }

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
    }
}
