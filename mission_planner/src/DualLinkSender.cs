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
                // Use the target-localizer capture endpoint with the long-run client.
                return await SendHttpPostLongRun("/api/task/1/target/capture", body);
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
            // a concurrent PayloadControlPanel/Task2PayloadPanel/joystick
            // write cannot corrupt the serial stream.
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
        /// Get target-localization runtime status.
        /// </summary>
        public async Task<CommandResult> GetTargetLocalizationStatusAsync()
        {
            return await SendHttpGetLongRun("/api/detections/status", 15);
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
                    RedirectStandardInput = false, // Let interactive password prompts pass through
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
                        // Race: the process may exit between the timeout check and
                        // Kill(), which raises InvalidOperationException. Swallow
                        // that specific case -- the process is already gone.
                        try { process.Kill(); }
                        catch (InvalidOperationException) { }
                        // Observe the abandoned stdout/stderr reads so they don't surface
                        // as TaskScheduler.UnobservedTaskException once the killed process
                        // closes the pipes. Result is intentionally discarded.
                        _ = outputTask.ContinueWith(t => { var _ = t.Exception; }, TaskContinuationOptions.OnlyOnFaulted);
                        _ = errorTask.ContinueWith(t => { var _ = t.Exception; }, TaskContinuationOptions.OnlyOnFaulted);
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
        /// Uses the Jetson-side `nomad restart all` hard-reset path, which
        /// stops the autostart set, kills stale launch children, then starts
        /// the autostart set again.
        /// </summary>
        public async Task<CommandResult> RestartAllServicesViaSSHAsync()
        {
            // Run the CLI hard reset in the background so the SSH call can
            // return before Edge Core is stopped.
            var command = "nohup bash -c 'sleep 2 && /home/mad/NOMAD/scripts/nomad restart all' > /dev/null 2>&1 & echo 'restart scheduled (~2s)'";
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
        /// Start target localization without stopping the Isaac ROS container.
        /// </summary>
        public async Task<CommandResult> StartTargetLocalizationAsync()
        {
            return await SendHttpPostLongRun("/api/detections/start", null);
        }

        /// <summary>
        /// Stop target localization without stopping the Isaac ROS container.
        /// </summary>
        public async Task<CommandResult> StopTargetLocalizationAsync()
        {
            return await SendHttpPostLongRun("/api/detections/stop", null);
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
            return await SendHttpGetLongRun("/api/task/2/slam/status", 15);
        }

        /// <summary>
        /// Clear SLAM mesh data.
        /// </summary>
        public async Task<CommandResult> ClearSlamAsync()
        {
            return await SendHttpPostLongRun("/api/task/2/slam/clear?prefer_load_map=true&auto_create_empty_map_if_missing=true", null);
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
            catch (TaskCanceledException)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = "HTTP request timed out",
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
            // call cannot interleave with PayloadControlPanel, Task2PayloadPanel,
            // or GimbalJoystickWindow writes to the same serial port.
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
