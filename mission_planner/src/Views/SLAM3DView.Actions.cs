// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// SLAM3DView.Actions.cs - Event handlers, helpers, area map ops
// ============================================================

using System;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;
using NOMAD.MissionPlanner.SLAM3D.Camera;
using NOMAD.MissionPlanner.SLAM3D.Data;
using NOMAD.MissionPlanner.SLAM3D.Models;
using NOMAD.MissionPlanner.SLAM3D.Rendering;

namespace NOMAD.MissionPlanner
{
    public partial class SLAM3DView
    {
        // ==================== Event Handlers ====================

        private void BtnToggleCamera_Click(object sender, EventArgs e)
        {
            var nextMode = _cameraController.CycleViewMode();
            if (nextMode == CameraViewMode.FreeOrbit)
            {
                var (glX, glY, glZ) = CoordinateConverter.RosToOpenGL(_renderPosX, _renderPosY, _renderPosZ);
                _cameraController.CenterOn(glX, glY, glZ);
            }

            UpdateCameraToggleLabel();
        }

        private void BtnResetView_Click(object sender, EventArgs e)
        {
            _cameraController.Reset();
        }

        private void CenterOrbitOnCurrentPose()
        {
            var (glX, glY, glZ) = CoordinateConverter.RosToOpenGL(_renderPosX, _renderPosY, _renderPosZ);
            _cameraController.CenterOn(glX, glY, glZ);
            _cameraController.ViewMode = CameraViewMode.FreeOrbit;
            UpdateCameraToggleLabel();
        }

        private void UpdateCameraToggleLabel()
        {
            if (_btnToggleCamera == null)
                return;

            string modeName = _cameraController.ViewMode switch
            {
                CameraViewMode.FirstPerson => "FPV",
                CameraViewMode.ThirdPerson => "TPV",
                CameraViewMode.FreeOrbit => "Orbit",
                _ => "?"
            };
            _btnToggleCamera.Text = $"View: {modeName}";
        }

        private async void BtnClearMesh_Click(object sender, EventArgs e)
        {
            try
            {
                var response = await JetsonApiService.PostLongRunAsync(
                    "/api/slam/clear?prefer_load_map=true&auto_create_empty_map_if_missing=true"
                );
                var responseBody = await response.Content.ReadAsStringAsync();

                if (!response.IsSuccessStatusCode)
                {
                    string failureMessage = $"Mesh clear failed ({(int)response.StatusCode})";
                    try
                    {
                        if (!string.IsNullOrWhiteSpace(responseBody))
                        {
                            var payload = JObject.Parse(responseBody);
                            var apiMessage = payload["message"]?.Value<string>();
                            if (!string.IsNullOrWhiteSpace(apiMessage))
                                failureMessage = apiMessage;
                        }
                    }
                    catch
                    {
                    }

                    UpdateStatusSafe(failureMessage);
                    AppendStatusLogSafe(failureMessage);
                    return;
                }

                _voxelMeshBuilder.Clear();
                _trajectoryRenderer.Clear();
                _detectionRenderer.Clear();
                _totalBlocks = 0;

                string successMessage = "Mesh cleared";
                try
                {
                    if (!string.IsNullOrWhiteSpace(responseBody))
                    {
                        var payload = JObject.Parse(responseBody);
                        var clearStrategy = payload["clear_strategy"]?.Value<string>();
                        if (!string.IsNullOrWhiteSpace(clearStrategy))
                            successMessage = $"Mesh cleared ({clearStrategy})";
                    }
                }
                catch
                {
                }

                UpdateStatusSafe(successMessage);
                AppendStatusLogSafe(successMessage);
            }
            catch (TaskCanceledException)
            {
                const string timeoutMessage = "Mesh clear timed out before server confirmation";
                UpdateStatusSafe(timeoutMessage);
                AppendStatusLogSafe(timeoutMessage);
            }
            catch (Exception ex)
            {
                UpdateStatusSafe($"Mesh clear failed ({ex.Message})");
                AppendStatusLogSafe($"Mesh clear failed ({ex.Message})");
            }
        }

        private async void BtnResetImuBiases_Click(object sender, EventArgs e)
        {
            var result = MessageBox.Show(
                "Reset IMU bias values stored in the ZED camera's internal EEPROM?\n\n" +
                "This is equivalent to running 'ZED Sensor Calibration.exe --cimu'.\n" +
                "Use this if camera orientation continues to drift after warmup.\n\n" +
                "The camera must be stationary during this operation.",
                "Reset IMU Biases",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning
            );

            if (result != DialogResult.Yes) return;

            try
            {
                var isaacResponse = await JetsonApiService.GetAsync("/api/isaac/status");
                if (isaacResponse.IsSuccessStatusCode)
                {
                    var isaacBody = await isaacResponse.Content.ReadAsStringAsync();
                    var isaacData = JObject.Parse(isaacBody);
                    bool isaacRunning = isaacData["running"]?.Value<bool>() ?? false;
                    if (isaacRunning)
                    {
                        UpdateStatusSafe("Stop Isaac ROS before IMU reset");
                        AppendStatusLogSafe("IMU reset blocked: Isaac ROS/nvblox is running.");
                        MessageBox.Show(
                            "IMU reset requires exclusive camera access, but Isaac ROS/nvblox is currently using the ZED camera.\n\n" +
                            "Stop Isaac ROS first, then retry IMU reset.",
                            "IMU Reset Blocked",
                            MessageBoxButtons.OK,
                            MessageBoxIcon.Warning
                        );
                        return;
                    }
                }
            }
            catch
            {
            }

            _btnResetImuBiases.Enabled = false;
            _btnResetImuBiases.Text = "Resetting...";
            UpdateStatusSafe("Resetting IMU biases...");

            try
            {
                var response = await JetsonApiService.PostLongRunAsync("/api/calibration/imu/reset_biases");
                var body = await response.Content.ReadAsStringAsync();
                var data = JObject.Parse(body);

                if (response.IsSuccessStatusCode)
                {
                    UpdateStatusSafe("IMU biases reset successfully");
                    AppendStatusLogSafe("IMU biases reset. Camera EEPROM cleared.");
                    MessageBox.Show(
                        "IMU biases reset successfully.\n\n" +
                        "The camera's internal bias values have been cleared.\n" +
                        "Allow the camera to warm up for a few minutes before use.",
                        "IMU Reset Complete",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information
                    );
                }
                else
                {
                    var detail = data["detail"]?.ToString() ?? "Unknown error";
                    UpdateStatusSafe($"IMU reset failed: {detail}");
                    AppendStatusLogSafe($"IMU reset failed: {detail}");
                    MessageBox.Show(
                        $"Failed to reset IMU biases:\n\n{detail}",
                        "IMU Reset Failed",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Error
                    );
                }
            }
            catch (Exception ex)
            {
                UpdateStatusSafe($"IMU reset error: {ex.Message}");
                AppendStatusLogSafe($"IMU reset error: {ex.Message}");
                MessageBox.Show(
                    $"Error resetting IMU biases:\n\n{ex.Message}",
                    "IMU Reset Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error
                );
            }
            finally
            {
                _btnResetImuBiases.Enabled = true;
                _btnResetImuBiases.Text = "Reset IMU";
            }
        }

        // ==================== Helpers ====================

        private void UpdateStatusSafe(string text)
        {
            if (_lblStatus == null) return;
            UiAsync.RunSync(_lblStatus, () => { if (_lblStatus != null) _lblStatus.Text = text; }, "UpdateStatusSafe");
        }

        private void UpdateStatsSafe(string text)
        {
            if (_lblStats == null) return;
            UiAsync.RunSync(_lblStats, () => { if (_lblStats != null) _lblStats.Text = text; }, "UpdateStatsSafe");
        }

        private void UpdatePerceptionStatusSafe(string text)
        {
            if (_lblPerceptionStatus == null) return;
            UiAsync.RunSync(_lblPerceptionStatus, () => { if (_lblPerceptionStatus != null) _lblPerceptionStatus.Text = text; }, "UpdatePerceptionStatusSafe");
        }

        private void AppendStatusLogSafe(string text)
        {
            if (string.IsNullOrWhiteSpace(text) || _txtStatusLog == null || _txtStatusLog.IsDisposed)
                return;

            void Append()
            {
                if (_txtStatusLog == null || _txtStatusLog.IsDisposed)
                    return;

                string timestamp = DateTime.Now.ToString("HH:mm:ss");
                _txtStatusLog.AppendText($"[{timestamp}] {text}\r\n");

                var lines = _txtStatusLog.Lines;
                if (lines.Length > MaxStatusLogLines)
                {
                    _txtStatusLog.Lines = lines.Skip(lines.Length - MaxStatusLogLines).ToArray();
                    _txtStatusLog.SelectionStart = _txtStatusLog.TextLength;
                    _txtStatusLog.ScrollToCaret();
                }
            }

            UiAsync.RunSync(_txtStatusLog, Append, "AppendStatusLogSafe");
        }

        private static string SummarizeCommandResult(CommandResult result)
        {
            if (result == null)
                return "Unknown result";

            if (!string.IsNullOrWhiteSpace(result.Data))
            {
                try
                {
                    var json = JObject.Parse(result.Data);
                    var message = json["message"]?.ToString()
                        ?? json["detail"]?.ToString()
                        ?? json["output"]?.ToString();
                    if (!string.IsNullOrWhiteSpace(message))
                        return message;
                }
                catch
                {
                }
            }

            return string.IsNullOrWhiteSpace(result.Message) ? "Completed" : result.Message;
        }

        private static bool IsServiceUnavailableMessage(string message)
        {
            string text = (message ?? string.Empty).ToLowerInvariant();
            return text.Contains("waiting for service")
                || text.Contains("service to be available")
                || text.Contains("service is not available")
                || text.Contains("service unavailable");
        }

        private static string BuildAreaMapFailureText(string actionName, string summary)
        {
            if (IsServiceUnavailableMessage(summary))
            {
                return $"{actionName} failed: map service not ready yet (start Isaac ROS + nvblox, then retry)";
            }

            if (!string.IsNullOrWhiteSpace(summary)
                && summary.IndexOf("FilePath_Response(success=False)", StringComparison.OrdinalIgnoreCase) >= 0)
            {
                return $"{actionName} failed: nvblox rejected the request (map not ready yet or file path invalid)";
            }

            return $"{actionName} failed ({summary})";
        }

        private async Task<CommandResult> ExecuteAreaMapCommandWithRetryAsync(
            Func<Task<CommandResult>> command,
            string actionName)
        {
            CommandResult lastResult = null;
            const int maxServiceReadyAttempts = 6;

            for (int attempt = 1; attempt <= maxServiceReadyAttempts; attempt++)
            {
                lastResult = await command();
                if (lastResult.Success)
                    return lastResult;

                string summary = SummarizeCommandResult(lastResult);
                if (!IsServiceUnavailableMessage(summary) || attempt >= maxServiceReadyAttempts)
                    return lastResult;

                AppendStatusLogSafe($"{actionName}: service not ready (attempt {attempt}/{maxServiceReadyAttempts}), retrying in 2s...");
                await Task.Delay(2000);
            }

            return lastResult ?? new CommandResult
            {
                Success = false,
                Message = $"{actionName} failed",
                Method = "HTTP",
            };
        }

        private async Task SaveAreaMapAsync()
        {
            var path = (_txtMapPath?.Text ?? string.Empty).Trim();
            if (string.IsNullOrWhiteSpace(path))
            {
                UpdateStatusSafe("Status: Area map path is empty");
                AppendStatusLogSafe("Save area map failed: path is empty");
                return;
            }

            try
            {
                UpdateStatusSafe("Status: Saving area map...");
                AppendStatusLogSafe($"Save area map requested: {path}");

                var result = await ExecuteAreaMapCommandWithRetryAsync(
                    () => _sender.SaveAreaMapAsync(path),
                    "Save area map");
                var message = SummarizeCommandResult(result);

                if (result.Success)
                {
                    UpdateStatusSafe($"Status: {message}");
                    AppendStatusLogSafe($"Save area map succeeded: {message}");
                }
                else
                {
                    string failureText = BuildAreaMapFailureText("Save area map", message);
                    UpdateStatusSafe($"Status: {failureText}");
                    AppendStatusLogSafe(failureText);
                }
            }
            catch (Exception ex)
            {
                string failureText = $"Save area map failed ({ex.Message})";
                UpdateStatusSafe($"Status: {failureText}");
                AppendStatusLogSafe(failureText);
            }
        }

        private async Task LoadAreaMapAsync()
        {
            var path = (_txtMapPath?.Text ?? string.Empty).Trim();
            if (string.IsNullOrWhiteSpace(path))
            {
                UpdateStatusSafe("Status: Area map path is empty");
                AppendStatusLogSafe("Load area map failed: path is empty");
                return;
            }

            try
            {
                UpdateStatusSafe("Status: Loading area map...");
                AppendStatusLogSafe($"Load area map requested: {path}");

                var result = await ExecuteAreaMapCommandWithRetryAsync(
                    () => _sender.LoadAreaMapAsync(path),
                    "Load area map");
                if (result.Success)
                    CenterOrbitOnCurrentPose();

                var message = SummarizeCommandResult(result);
                if (result.Success)
                {
                    UpdateStatusSafe($"Status: {message}");
                    AppendStatusLogSafe($"Load area map succeeded: {message}");
                }
                else
                {
                    string failureText = BuildAreaMapFailureText("Load area map", message);
                    UpdateStatusSafe($"Status: {failureText}");
                    AppendStatusLogSafe(failureText);
                }
            }
            catch (Exception ex)
            {
                string failureText = $"Load area map failed ({ex.Message})";
                UpdateStatusSafe($"Status: {failureText}");
                AppendStatusLogSafe(failureText);
            }
        }

        private async Task RelocalizeAreaMapAsync()
        {
            var path = (_txtMapPath?.Text ?? string.Empty).Trim();
            if (string.IsNullOrWhiteSpace(path))
            {
                UpdateStatusSafe("Status: Area map path is empty");
                AppendStatusLogSafe("Relocalize area map failed: path is empty");
                return;
            }

            try
            {
                UpdateStatusSafe("Status: Relocalizing...");
                AppendStatusLogSafe($"Relocalize area map requested: {path}");

                var result = await ExecuteAreaMapCommandWithRetryAsync(
                    () => _sender.RelocalizeAreaMapAsync(path),
                    "Relocalize area map");
                if (result.Success)
                    CenterOrbitOnCurrentPose();

                var message = SummarizeCommandResult(result);
                if (result.Success)
                {
                    UpdateStatusSafe($"Status: {message}");
                    AppendStatusLogSafe($"Relocalize area map succeeded: {message}");
                }
                else
                {
                    string failureText = BuildAreaMapFailureText("Relocalize area map", message);
                    UpdateStatusSafe($"Status: {failureText}");
                    AppendStatusLogSafe(failureText);
                }
            }
            catch (Exception ex)
            {
                string failureText = $"Relocalize area map failed ({ex.Message})";
                UpdateStatusSafe($"Status: {failureText}");
                AppendStatusLogSafe(failureText);
            }
        }
    }
}
