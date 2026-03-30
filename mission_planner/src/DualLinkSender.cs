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
        
        /// <summary>MAV_CMD_SET_EKF_SOURCE_SET (42007) - Switch EKF source</summary>
        public const ushort CMD_SET_EKF_SOURCE = 42007;

        // Security: Whitelist of allowed service names (defense-in-depth)
        private static readonly System.Collections.Generic.HashSet<string> ALLOWED_SERVICES = new System.Collections.Generic.HashSet<string>(StringComparer.OrdinalIgnoreCase)
        {
            "nomad", "edge_core", "mediamtx", "mavlink-router", "isaac-ros", "zed-service"
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
                // Use long-run client (30s) -- RTSP capture can take 5-10s
                return await SendHttpPostLongRun("/api/task/1/capture", body);
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
        public Task<CommandResult> SetEkfSource(EkfSource source)
        {
            try
            {
                if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
                {
                    return Task.FromResult(new CommandResult
                    {
                        Success = false,
                        Message = "MAVLink not connected",
                        Method = "MAVLink"
                    });
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

                return Task.FromResult(new CommandResult
                {
                    Success = true,
                    Message = $"EKF source switched to {sourceName}",
                    Method = "MAVLink"
                });
            }
            catch (Exception ex)
            {
                return Task.FromResult(new CommandResult
                {
                    Success = false,
                    Message = $"EKF source switch failed: {ex.Message}",
                    Method = "MAVLink"
                });
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
        /// Execute a command via SSH directly (bypasses HTTP API).
        /// Use this for operations that kill the HTTP API (like service restarts).
        /// Works with SSH key authentication or password authentication.
        /// Shows console window for password entry if keys aren't configured.
        /// </summary>
        public async Task<CommandResult> ExecuteSSHCommandAsync(string command, int timeoutSeconds = 30)
        {
            try
            {
                // Detect OS and find SSH executable
                string sshPath;
                bool isWindows = Environment.OSVersion.Platform == PlatformID.Win32NT;
                
                if (isWindows)
                {
                    // Windows: Use built-in OpenSSH (Windows 10/11)
                    sshPath = @"C:\Windows\System32\OpenSSH\ssh.exe";
                    if (!System.IO.File.Exists(sshPath))
                    {
                        sshPath = "ssh.exe"; // Fallback to PATH
                    }
                }
                else
                {
                    // Linux/Mac: Use standard ssh location
                    sshPath = "/usr/bin/ssh";
                    if (!System.IO.File.Exists(sshPath))
                    {
                        sshPath = "ssh"; // Fallback to PATH
                    }
                }
                
                // SSH arguments: allow password prompts, set timeout
                // Note: Removed BatchMode=yes to allow password authentication
                // ConnectTimeout prevents hanging if host is unreachable
                var sshArgs = $"-o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new mad@{_config.EffectiveIP} \"{command}\"";
                
                var startInfo = new System.Diagnostics.ProcessStartInfo
                {
                    FileName = sshPath,
                    Arguments = sshArgs,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    RedirectStandardInput = true,  // Required for password prompts
                    UseShellExecute = false,
                    CreateNoWindow = false,  // Show console window for password entry
                    WindowStyle = System.Diagnostics.ProcessWindowStyle.Normal
                };

                using (var process = System.Diagnostics.Process.Start(startInfo))
                {
                    // Use tasks for async I/O
                    var outputTask = process.StandardOutput.ReadToEndAsync();
                    var errorTask = process.StandardError.ReadToEndAsync();
                    
                    // Wait for process to complete with timeout
                    var exited = await Task.Run(() => process.WaitForExit(timeoutSeconds * 1000));

                    if (!exited)
                    {
                        process.Kill();
                        return new CommandResult 
                        { 
                            Success = false, 
                            Message = $"SSH command timed out after {timeoutSeconds}s (host unreachable or command hung)" 
                        };
                    }

                    var output = await outputTask;
                    var error = await errorTask;

                    // SSH returns 0 on success
                    bool success = process.ExitCode == 0;
                    
                    return new CommandResult
                    {
                        Success = success,
                        Data = output,
                        Message = success ? "SSH command executed successfully" : $"SSH failed (exit code {process.ExitCode}): {error}"
                    };
                }
            }
            catch (Exception ex)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = $"SSH execution exception: {ex.Message}"
                };
            }
        }

        /// <summary>
        /// Restart all NOMAD services via SSH (doesn't rely on HTTP API).
        /// Stops existing services and runs start_nomad_full.sh script.
        /// </summary>
        public async Task<CommandResult> RestartAllServicesViaSSHAsync()
        {
            // Use dedicated restart script that properly kills all processes (including ZED)
            // and restarts NOMAD services in background
            var command = "cd ~/NOMAD && bash scripts/run/restart_nomad.sh";
            
            return await ExecuteSSHCommandAsync(command, 30);
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
                "mediamtx" => "restart_video",
                "mavlink-router" => "restart_mavlink",
                "nomad" => "restart_edge_core",
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
                "nomad" => "status_nomad",
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
        /// Stop nvblox and ROS-HTTP bridge without stopping the container.
        /// </summary>
        public async Task<CommandResult> StopNvbloxAsync()
        {
            return await SendHttpPostLongRun("/api/isaac/stop-nvblox", null);
        }

        /// <summary>
        /// Get Isaac ROS logs.
        /// </summary>
        public async Task<CommandResult> GetIsaacRosLogsAsync(string logType = "all")
        {
            return await SendHttpGet($"/api/isaac/logs?log_type={logType}");
        }

        // ============================================================
        // Video Bridges Control
        // ============================================================

        /// <summary>
        /// Get status of all video bridge instances (primary/secondary).
        /// </summary>
        public async Task<CommandResult> GetVideoBridgesStatusAsync()
        {
            return await SendHttpGet("/api/video/bridges");
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
            return await SendHttpGet("/api/task/2/slam/status");
        }

        /// <summary>
        /// Clear SLAM mesh data.
        /// </summary>
        public async Task<CommandResult> ClearSlamAsync()
        {
            return await SendHttpPost("/api/task/2/slam/clear", null);
        }

        // ============================================================
        // HTTP Communication
        // ============================================================

        private async Task<CommandResult> SendHttpGet(string endpoint)
        {
            try
            {
                var response = await JetsonApiService.GetAsync(endpoint);
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

        private async Task<CommandResult> SendHttpDelete(string endpoint)
        {
            try
            {
                var response = await JetsonApiService.DeleteAsync(endpoint);
                var responseBody = await response.Content.ReadAsStringAsync();

                if (response.IsSuccessStatusCode)
                {
                    return new CommandResult
                    {
                        Success = true,
                        Message = "HTTP DELETE successful",
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
                var content = body != null
                    ? new StringContent(
                        JsonConvert.SerializeObject(body),
                        Encoding.UTF8,
                        "application/json")
                    : new StringContent("{}", Encoding.UTF8, "application/json");

                var response = await JetsonApiService.PostAsync(endpoint, content);
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

        /// <summary>
        /// Send an HTTP POST using the long-running client (30s timeout).
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
