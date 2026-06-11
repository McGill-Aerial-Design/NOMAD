// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public partial class ServiceControlPanel
    {
        private static bool ShouldLogStreak(int streak)
        {
            // Ignore one-off timeouts/cancels. Log only when persistent.
            return streak >= 5 && (streak == 5 || streak % 10 == 0);
        }

        private async void PollServicesAsync()
        {
            // Prevent overlapping async polls (Timer can fire again before prior await chain completes).
            if (Interlocked.Exchange(ref _isPolling, 1) == 1)
                return;

            try
            {
                int cycle = Interlocked.Increment(ref _pollCycle);
                bool pollIsaac = (cycle == 1) || (cycle % 2 == 0);
                bool pollVio = (cycle == 1) || (cycle % 2 == 0);
                bool pollVideoAndSlam = (cycle == 1) || (cycle % 3 == 0);
                bool servicesFresh = false;
                bool isaacRunningFromServices = false;

                // Primary source of truth for service states.
                var servicesResult = await _sender.GetServicesStatusAsync();
                if (servicesResult.Success)
                {
                    try
                    {
                        var services = JObject.Parse(servicesResult.Data);

                        bool edgeRunning = services["edge_core"]?["running"]?.Value<bool>() ?? false;
                        UpdateStatusLabel(_lblEdgeCoreStatus, edgeRunning, edgeRunning ? "Running" : "Stopped");

                        bool mavRunning = services["mavlink_router"]?["running"]?.Value<bool>() ?? false;
                        string mavRaw = services["mavlink_router"]?["status"]?.Value<string>() ?? string.Empty;
                        string mavText;
                        if (mavRunning)
                        {
                            mavText = "Running";
                        }
                        else if (mavRaw.Equals("no_cubepilot", StringComparison.OrdinalIgnoreCase))
                        {
                            mavText = "No CubePilot";
                        }
                        else
                        {
                            mavText = !string.IsNullOrWhiteSpace(mavRaw) ? $"Stopped ({mavRaw})" : "Stopped";
                        }
                        UpdateStatusLabel(_lblMavlinkStatus, mavRunning, mavText);

                        bool mediamtxRunning = services["mediamtx"]?["running"]?.Value<bool>() ?? false;
                        UpdateStatusLabel(_lblMediamtxStatus, mediamtxRunning, mediamtxRunning ? "Running" : "Stopped");

                        var novncToken = services["novnc"];
                        if (novncToken != null)
                        {
                            bool noVncRunning = novncToken["running"]?.Value<bool>() ?? false;
                            string noVncStatus = novncToken["status"]?.Value<string>() ?? string.Empty;
                            string noVncText = noVncRunning
                                ? "Running (port 6080)"
                                : (!string.IsNullOrWhiteSpace(noVncStatus) ? $"Stopped ({noVncStatus})" : "Stopped");
                            UpdateStatusLabel(_lblNoVncStatus, noVncRunning, noVncText);
                        }
                        else
                        {
                            UpdateStatusLabel(_lblNoVncStatus, false, "Unavailable");
                        }

                        bool isaacRunning = services["isaac_ros"]?["running"]?.Value<bool>() ?? false;
                        string isaacMessage = services["isaac_ros"]?["message"]?.Value<string>();
                        string isaacText = isaacRunning
                            ? "Running"
                            : (string.IsNullOrWhiteSpace(isaacMessage) ? "Not Running" : isaacMessage);
                        UpdateStatusLabel(_lblIsaacRosStatus, isaacRunning, isaacText);
                        isaacRunningFromServices = isaacRunning;

                        servicesFresh = true;
                        _servicesFailStreak = 0;
                    }
                    catch (Exception parseEx)
                    {
                        _servicesFailStreak++;
                        if (ShouldLogStreak(_servicesFailStreak))
                            LogMessage($"Services parse warning (streak {_servicesFailStreak}): {parseEx.Message}");
                    }
                }
                else
                {
                    _servicesFailStreak++;
                    if (ShouldLogStreak(_servicesFailStreak))
                        LogMessage($"Services poll warning (streak {_servicesFailStreak}): {servicesResult.Message}");
                }

                // If the primary snapshot is unavailable, keep last-known labels and
                // skip the rest of endpoint-specific probes this cycle.
                if (!servicesFresh)
                {
                    UpdateStatusPendingIfChecking(_lblNoVncStatus, "Waiting...");
                    UpdateStatusPendingIfChecking(_lblIsaacRosStatus, "Waiting...");
                    UpdateStatusPendingIfChecking(_lblNvbloxStatus, "Waiting...");
                    UpdateStatusPendingIfChecking(_lblVioStatus, "Waiting...");
                    UpdateStatusPendingIfChecking(_lblVideoBridgesStatus, "Waiting...");
                    UpdateStatusPendingIfChecking(_lblSlamStatus, "Waiting...");
                    UpdateLabel(_lblLastUpdate, $"Last update: {DateTime.Now:HH:mm:ss} (partial/stale)");
                    return;
                }

                // Check Isaac ROS status (container + nvblox + bridge)
                if (pollIsaac)
                {
                    var isaacResult = await _sender.GetIsaacStatusAsync();
                    if (isaacResult.Success)
                    {
                        try
                        {
                            var isaacData = JObject.Parse(isaacResult.Data);
                            var containerRunning = isaacData["container_running"]?.Value<bool>() ?? false;
                            var nvbloxRunning = isaacData["nvblox_running"]?.Value<bool>() ?? false;
                            var bridgeRunning = isaacData["bridge_running"]?.Value<bool>() ?? false;

                            UpdateStatusLabel(_lblIsaacRosStatus, containerRunning, containerRunning ? "Running" : "Not Running");

                            UpdateStatusLabel(_lblRosBridgeStatus, bridgeRunning,
                                bridgeRunning ? "Running" : (containerRunning ? "Stopped" : "No Container"));

                            if (nvbloxRunning)
                                UpdateStatusLabel(_lblNvbloxStatus, true, "Running");
                            else if (containerRunning)
                                UpdateStatusLabel(_lblNvbloxStatus, false, "Stopped");
                            else
                                UpdateStatusLabel(_lblNvbloxStatus, false, "No Container");

                            _isaacFailStreak = 0;
                        }
                        catch (Exception parseEx)
                        {
                            _isaacFailStreak++;
                            if (ShouldLogStreak(_isaacFailStreak))
                                LogMessage($"Isaac status parse warning (streak {_isaacFailStreak}): {parseEx.Message}");
                        }
                    }
                    else
                    {
                        _isaacFailStreak++;
                        if (ShouldLogStreak(_isaacFailStreak))
                            LogMessage($"Isaac status warning (streak {_isaacFailStreak}): {isaacResult.Message}");
                    }
                }

                // Check VIO status
                if (pollVio)
                {
                    var vioResult = await _sender.GetVioStatusAsync();
                    if (vioResult.Success)
                    {
                        try
                        {
                            var vioData = JObject.Parse(vioResult.Data);
                            var health = vioData["health"]?.Value<string>() ?? "unknown";
                            var source = vioData["source"]?.Value<string>() ?? "none";
                            var confidence = vioData["tracking_confidence"]?.Value<double>() ?? 0;

                            bool healthy = health == "healthy";
                            string statusText = $"{health} ({source})";
                            if (health.Equals("unknown", StringComparison.OrdinalIgnoreCase) && isaacRunningFromServices)
                                statusText = "warming up (isaac_ros)";
                            if (confidence > 0)
                                statusText += $" {confidence:P0}";

                            UpdateStatusLabel(_lblVioStatus, healthy, statusText);
                            _vioFailStreak = 0;
                        }
                        catch (Exception parseEx)
                        {
                            _vioFailStreak++;
                            if (ShouldLogStreak(_vioFailStreak))
                                LogMessage($"VIO parse warning (streak {_vioFailStreak}): {parseEx.Message}");
                        }
                    }
                    else
                    {
                        _vioFailStreak++;
                        if (ShouldLogStreak(_vioFailStreak))
                            LogMessage($"VIO status warning (streak {_vioFailStreak}): {vioResult.Message}");
                    }

                    // Get trajectory points less aggressively (same cadence as VIO poll)
                    var trajResult = await _sender.GetVioTrajectoryAsync(10);
                    if (trajResult.Success)
                    {
                        try
                        {
                            var trajData = JObject.Parse(trajResult.Data);
                            var totalPoints = trajData["total_points"]?.Value<int>() ?? 0;
                            UpdateLabel(_lblVioTrajectoryPoints, $"{totalPoints} points");
                        }
                        catch { }
                    }
                }

                // Video bridge status (single bridge configuration)
                if (pollVideoAndSlam)
                {
                    var bridgesResult = await _sender.GetVideoBridgesStatusAsync();
                    if (bridgesResult.Success)
                    {
                        try
                        {
                            var data = JObject.Parse(bridgesResult.Data);
                            var primary = data["bridges"]?["primary"]?["state"]?.ToString() ?? "stopped";
                            // Only check primary bridge (we simplified to single bridge)
                            bool isStreaming = primary == "playing";
                            var fps = data["bridges"]?["primary"]?["fps"]?.Value<float>() ?? 0;
                            string statusText = isStreaming ? $"Streaming ({fps:F1} fps)" : "Stopped";
                            UpdateStatusLabel(_lblVideoBridgesStatus, isStreaming, statusText);
                            _videoFailStreak = 0;
                        }
                        catch (Exception parseEx)
                        {
                            _videoFailStreak++;
                            if (ShouldLogStreak(_videoFailStreak))
                                LogMessage($"Video status parse warning (streak {_videoFailStreak}): {parseEx.Message}");
                        }
                    }
                    else
                    {
                        _videoFailStreak++;
                        if (ShouldLogStreak(_videoFailStreak))
                            LogMessage($"Video status warning (streak {_videoFailStreak}): {bridgesResult.Message}");
                    }

                    // SLAM status
                    var slamResult = await _sender.GetSlamStatusAsync();
                    if (slamResult.Success)
                    {
                        try
                        {
                            var data = JObject.Parse(slamResult.Data);
                            var available = data["available"]?.Value<bool>() ?? false;
                            var running = data["running"]?.Value<bool>() ?? false;
                            if (running)
                            {
                                var blocks = data["block_count"]?.Value<int>() ?? 0;
                                UpdateStatusLabel(_lblSlamStatus, true, $"Active ({blocks} blocks)");
                            }
                            else if (available)
                            {
                                UpdateStatusLabel(_lblSlamStatus, false, "Available (no data)");
                            }
                            else
                            {
                                var error = (string)data["error"];
                                if (isaacRunningFromServices &&
                                    string.Equals(error, "No mesh data available", StringComparison.OrdinalIgnoreCase))
                                {
                                    UpdateStatusLabel(_lblSlamStatus, false, "Waiting for mesh");
                                }
                                else
                                {
                                    UpdateStatusLabel(_lblSlamStatus, false, error ?? "Inactive");
                                }
                            }

                            _slamFailStreak = 0;
                        }
                        catch (Exception parseEx)
                        {
                            _slamFailStreak++;
                            if (ShouldLogStreak(_slamFailStreak))
                                LogMessage($"SLAM status parse warning (streak {_slamFailStreak}): {parseEx.Message}");
                        }
                    }
                    else
                    {
                        _slamFailStreak++;
                        if (ShouldLogStreak(_slamFailStreak))
                            LogMessage($"SLAM status warning (streak {_slamFailStreak}): {slamResult.Message}");
                    }
                }

                // Update timestamp
                var suffix = servicesFresh ? string.Empty : " (partial/stale)";
                UpdateLabel(_lblLastUpdate, $"Last update: {DateTime.Now:HH:mm:ss}{suffix}");
            }
            catch (Exception ex)
            {
                LogMessage($"Poll error: {ex.Message}");
            }
            finally
            {
                Interlocked.Exchange(ref _isPolling, 0);
            }
        }

        private void UpdateStatusLabel(Label label, bool isOk, string customText = null)
        {
            if (label == null || label.IsDisposed) return;

            UiAsync.RunSync(label, () =>
            {
                label.Text = customText ?? (isOk ? "Running" : "Stopped");
                label.ForeColor = isOk ? Color.LimeGreen : Color.OrangeRed;
            }, "UpdateStatusLabel");
        }

        private void UpdateStatusPendingIfChecking(Label label, string text)
        {
            if (label == null || label.IsDisposed) return;

            UiAsync.RunSync(label, () =>
            {
                if (string.Equals(label.Text, "Checking...", StringComparison.OrdinalIgnoreCase))
                {
                    label.Text = text;
                    label.ForeColor = Color.Goldenrod;
                }
            }, "UpdateStatusPendingIfChecking");
        }

        private void UpdateLabel(Label label, string text)
        {
            if (label == null || label.IsDisposed) return;

            UiAsync.RunSync(label, () =>
            {
                label.Text = text;
            }, "UpdateLabel");
        }

        private void LogMessage(string message)
        {
            if (_txtLog == null || _txtLog.IsDisposed) return;

            UiAsync.RunSync(_txtLog, () =>
            {
                var timestamp = DateTime.Now.ToString("HH:mm:ss");
                _txtLog.AppendText($"[{timestamp}] {message}\r\n");
            }, "LogMessage");
        }

        private async Task RestartServiceAsync(string serviceName, Label statusLabel)
        {
            LogMessage($"Restarting {serviceName}...");
            UpdateStatusLabel(statusLabel, false, "Restarting...");

            var result = await _sender.RestartServiceAsync(serviceName);

            if (result.Success)
            {
                LogMessage($"{serviceName} restart command sent");
                await Task.Delay(2000);
                // Will be updated by next poll
            }
            else
            {
                LogMessage($"Failed to restart {serviceName}: {result.Message}");
            }
        }

        private async Task RestartAllServicesAsync()
        {
            LogMessage("Restarting all NOMAD services via SSH...");
            UpdateStatusLabel(_lblEdgeCoreStatus, false, "Restarting...");
            UpdateStatusLabel(_lblMavlinkStatus, false, "Restarting...");
            UpdateStatusLabel(_lblMediamtxStatus, false, "Restarting...");
            UpdateStatusLabel(_lblIsaacRosStatus, false, "Restarting...");
            UpdateStatusLabel(_lblVideoBridgesStatus, false, "Restarting...");

            // Use SSH instead of HTTP API (since we're killing edge_core)
            LogMessage("Using SSH for restart (HTTP API will be unavailable during restart)");
            var result = await _sender.RestartAllServicesViaSSHAsync();

            if (result.Success || result.Message.Contains("executed") || string.IsNullOrEmpty(result.Message))
            {
                LogMessage("NOMAD services restart command sent successfully");
                LogMessage("All services starting (MAVLink, MediaMTX, Edge Core, Isaac ROS, Video Bridge)...");
                LogMessage("Full startup takes 30-45 seconds. Status will update automatically.");

                // Show info message on UI thread
                UiAsync.RunSync(this, () =>
                {
                    MessageBox.Show(
                        this,
                        "NOMAD Services Restart Initiated!\n\n" +
                        "Restarting all services via SSH:\n" +
                        "1. MAVLink Router\n" +
                        "2. MediaMTX (RTSP)\n" +
                        "3. Edge Core API\n" +
                        "4. Isaac ROS + ZED Camera\n" +
                        "5. Video Bridge\n\n" +
                        "Full startup: 30-45 seconds.\n" +
                        "Status will update automatically.",
                        "NOMAD Restarting",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information
                    );
                }, "RestartAllServicesMessage");
            }
            else
            {
                LogMessage($"SSH restart command may have issues: {result.Message}");
                LogMessage("Note: This is normal if SSH keys aren't configured. Services may still be restarting.");
            }
        }

        private async Task StartIsaacRosAsync()
        {
            LogMessage("Starting Isaac ROS container and services...");
            UpdateStatusLabel(_lblIsaacRosStatus, false, "Starting...");

            // Use the dedicated Isaac ROS API endpoint instead of raw shell command
            var result = await _sender.StartIsaacRosAsync();

            if (result.Success)
            {
                LogMessage("Isaac ROS startup initiated");
                LogMessage("Container starting with ROS2 environment...");
                LogMessage("Full startup takes 30-60 seconds");

                // Show non-blocking message on UI thread
                UiAsync.RunSync(this, () =>
                {
                    MessageBox.Show(
                        this,
                        "Isaac ROS startup initiated!\n\n" +
                        "The following will start automatically:\n" +
                        "1. Docker container\n" +
                        "2. ROS2 dependencies installation\n\n" +
                        "Note: Nvblox VSLAM must be launched separately\n" +
                        "using the Nvblox controls in this panel.\n\n" +
                        "Full startup takes 30-60 seconds.\n" +
                        "Status will update automatically.",
                        "Isaac ROS Starting",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information
                    );
                }, "StartIsaacRosMessage");
            }
            else
            {
                LogMessage($"Failed to start Isaac ROS: {result.Message}");
                UpdateStatusLabel(_lblIsaacRosStatus, false, "Start Failed");
            }
        }

        private async Task StartNoVncAsync()
        {
            LogMessage("Starting noVNC...");
            UpdateStatusLabel(_lblNoVncStatus, false, "Starting...");

            var result = await _sender.StartServiceAsync("novnc");

            if (result.Success)
            {
                LogMessage("noVNC start command sent");
            }
            else
            {
                LogMessage($"Failed to start noVNC: {result.Message}");
                UpdateStatusLabel(_lblNoVncStatus, false, "Start Failed");
            }
        }

        private async Task StopNoVncAsync()
        {
            LogMessage("Stopping noVNC...");
            UpdateStatusLabel(_lblNoVncStatus, false, "Stopping...");

            var result = await _sender.StopServiceAsync("novnc");

            if (result.Success)
            {
                LogMessage("noVNC stop command sent");
            }
            else
            {
                LogMessage($"Failed to stop noVNC: {result.Message}");
            }
        }

        private async Task StopIsaacRosAsync()
        {
            LogMessage("Stopping Isaac ROS...");
            UpdateStatusLabel(_lblIsaacRosStatus, false, "Stopping...");

            var result = await _sender.StopIsaacRosAsync();

            if (result.Success)
            {
                LogMessage("Isaac ROS stopped");
                UpdateStatusLabel(_lblIsaacRosStatus, false, "Stopped");
            }
            else
            {
                LogMessage($"Failed to stop Isaac ROS: {result.Message}");
            }
        }

        private async Task ClearTrajectoryAsync()
        {
            LogMessage("Clearing VIO trajectory...");
            // Use the dedicated HTTP DELETE endpoint for VIO trajectory
            var result = await _sender.ClearVioTrajectoryAsync();

            if (result.Success)
            {
                LogMessage("Trajectory cleared");
                UpdateLabel(_lblVioTrajectoryPoints, "0 points");
            }
            else
            {
                LogMessage($"Clear failed: {result.Message}");
            }
        }

        private async Task StartRosBridgeAsync()
        {
            LogMessage("Starting ROS HTTP bridge...");
            UpdateStatusLabel(_lblRosBridgeStatus, false, "Starting...");
            var result = await _sender.StartRosBridgeAsync();
            if (result.Success)
            {
                LogMessage("ROS HTTP bridge started");
                UpdateStatusLabel(_lblRosBridgeStatus, true, "Running");
            }
            else
            {
                LogMessage($"Failed to start ROS bridge: {result.Message}");
                UpdateStatusLabel(_lblRosBridgeStatus, false, "Start Failed");
            }
        }

        private async Task StopRosBridgeAsync()
        {
            LogMessage("Stopping ROS HTTP bridge...");
            UpdateStatusLabel(_lblRosBridgeStatus, false, "Stopping...");
            var result = await _sender.StopRosBridgeAsync();
            if (result.Success)
            {
                LogMessage("ROS HTTP bridge stopped");
                UpdateStatusLabel(_lblRosBridgeStatus, false, "Stopped");
            }
            else
            {
                LogMessage($"Failed to stop ROS bridge: {result.Message}");
            }
        }

        private async Task LaunchNvbloxAsync()
        {
            LogMessage("Launching nvblox...");
            UpdateStatusLabel(_lblNvbloxStatus, false, "Launching...");

            var result = await _sender.LaunchNvbloxAsync();
            if (result.Success)
            {
                LogMessage("nvblox launch initiated (~15s for ZED init)");
            }
            else
            {
                LogMessage($"Failed to launch nvblox: {result.Message}");
                UpdateStatusLabel(_lblNvbloxStatus, false, "Launch Failed");
            }
        }

        private async Task StopNvbloxAsync()
        {
            LogMessage("Stopping nvblox...");
            UpdateStatusLabel(_lblNvbloxStatus, false, "Stopping...");

            var result = await _sender.StopNvbloxAsync();
            if (result.Success)
            {
                LogMessage("nvblox stopped");
                UpdateStatusLabel(_lblNvbloxStatus, false, "Stopped");
            }
            else
            {
                LogMessage($"Failed to stop nvblox: {result.Message}");
            }
        }

        private async Task StartVideoBridgesAsync()
        {
            LogMessage("Starting video bridges...");
            UpdateStatusLabel(_lblVideoBridgesStatus, false, "Starting...");
            var result = await _sender.StartVideoBridgesAsync();
            if (result.Success)
                LogMessage("Video bridges start command sent");
            else
                LogMessage($"Failed to start bridges: {result.Message}");
        }

        private async Task StopSlamAsync()
        {
            LogMessage("Stopping SLAM / nvblox...");
            UpdateStatusLabel(_lblSlamStatus, false, "Stopping...");
            var result = await _sender.StopSlamAsync();
            if (result.Success)
            {
                LogMessage("SLAM / nvblox stopped");
                UpdateStatusLabel(_lblSlamStatus, false, "Stopped");
            }
            else
            {
                LogMessage($"Stop failed: {result.Message}");
                UpdateStatusLabel(_lblSlamStatus, false, "Stop Failed");
            }
        }
    }
}
