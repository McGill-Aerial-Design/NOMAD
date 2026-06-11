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

        // Local-only: the server-side /api/slam/clear route was gutted from this
        // baseline. Clearing the local renderers is still useful to the operator.
        private void BtnClearMesh_Click(object sender, EventArgs e)
        {
            _voxelMeshBuilder.Clear();
            _trajectoryRenderer.Clear();
            _detectionRenderer.Clear();
            _totalBlocks = 0;
            UpdateStatusSafe("Mesh cleared (local view)");
            AppendStatusLogSafe("Mesh cleared (local view)");
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

    }
}
