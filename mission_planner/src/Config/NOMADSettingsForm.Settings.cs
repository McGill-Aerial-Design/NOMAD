// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADSettingsForm.Settings.cs - Config <-> control sync
// ============================================================

using System;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private void LoadSettings()
        {
            _numCoreRuntimePort.Value = ClampValue(_numCoreRuntimePort, Config.CoreRuntimePort);
            _txtCoreApiKey.Text = Config.CoreApiKey ?? "";

            _txtVideoUrl.Text = Config.VideoUrl;
            _numVideoCaching.Value = ClampValue(_numVideoCaching, Config.VideoNetworkCaching);
            SetComboBoxValue(_cmbVideoPlayer, Config.PreferredVideoPlayer);
            _chkVideoAutoStart.Checked = Config.VideoAutoStart;
            _chkAutoStartHudVideo.Checked = Config.AutoStartHudVideo;

            _chkDualLinkEnabled.Checked = Config.DualLinkEnabled;
            _numRouterLocalPort.Value = ClampValue(_numRouterLocalPort, Config.RouterLocalPort);
            _numManagementPort.Value = ClampValue(_numManagementPort, Config.ManagementPort);

            _chkDarkMode.Checked = Config.DarkMode;
            _chkShowNotifications.Checked = Config.ShowNotifications;
            SetComboBoxValue(_cmbDefaultTab, Config.DefaultTab);
            _chkDebugMode.Checked = Config.DebugMode;
            _numSlamFov.Value = ClampValue(_numSlamFov, Config.SlamCameraFovDeg);
            _numSlamMapRadius.Value = ClampValue(_numSlamMapRadius, Config.SlamMapRadiusM);

            _numTempWarning.Value = ClampValue(_numTempWarning, Config.TempWarningC);
            _numTempCritical.Value = ClampValue(_numTempCritical, Config.TempCriticalC);
            _chkAudioAlerts.Checked = Config.AudioAlerts;
            _chkAltitudeCallouts.Checked = Config.AltitudeCallouts;

            _txtDefaultLogDirectory.Text = Config.DefaultLogDirectory ?? "";
            _numLogVibrationWarning.Value = ClampValue(_numLogVibrationWarning, Config.LogVibrationWarning);
            _numLogVibrationCritical.Value = ClampValue(_numLogVibrationCritical, Config.LogVibrationCritical);
            _numLogHdopWarning.Value = ClampValue(_numLogHdopWarning, Config.LogHdopWarning);
            _numLogHdopCritical.Value = ClampValue(_numLogHdopCritical, Config.LogHdopCritical);
            _numLogMinimumSatellites.Value = ClampValue(_numLogMinimumSatellites, Config.LogMinimumSatellites);
            _numLogTuneWarning.Value = ClampValue(_numLogTuneWarning, Config.LogTuneRmsWarning);
            _numLogTuneCritical.Value = ClampValue(_numLogTuneCritical, Config.LogTuneRmsCritical);
            _numLogEkfWarning.Value = ClampValue(_numLogEkfWarning, Config.LogEkfVarianceWarning);
            _numLogEkfCritical.Value = ClampValue(_numLogEkfCritical, Config.LogEkfVarianceCritical);
            _numLogLiveBufferPoints.Value = ClampValue(_numLogLiveBufferPoints, Config.LogLiveBufferPoints);
            _chkLogInjectHud.Checked = Config.LogInjectAlertsToHud;

            LoadPayloads();
            _numSprayRange.Value = (decimal)Config.SprayTargetCameraRangeM;
            _numSprayRangeTol.Value = (decimal)Config.SprayRangeToleranceM;
            _numSprayTriggerMax.Value = (decimal)Config.SprayTriggerMaxDistanceM;
            _numSprayAimX.Value = Config.SprayAimPixelX;
            _numSprayAimY.Value = Config.SprayAimPixelY;
            _numSprayAimTol.Value = Config.SprayAimTolerancePx;
            _numSprayServoAngle.Value = (decimal)Config.SprayServoFireAngleDeg;
            _numSprayForwardGain.Value = (decimal)Config.SprayForwardGain;
            _numSprayLateralGain.Value = (decimal)Config.SprayLateralGain;
            _numSprayAltitudeGain.Value = (decimal)Config.SprayAltitudeGain;
            _numSprayYawGain.Value = (decimal)Config.SprayYawGain;
            _chkSprayUseYaw.Checked = Config.SprayUseYawAlignment;
            _numSprayMaxForward.Value = (decimal)Config.SprayMaxForwardSpeedMps;
            _numSprayMaxLateral.Value = (decimal)Config.SprayMaxLateralSpeedMps;
            _numSprayMaxAltitude.Value = (decimal)Config.SprayMaxAltitudeSpeedMps;
            _numSprayMaxYaw.Value = (decimal)Config.SprayMaxYawRateRadps;
            _numSprayLockMs.Value = Config.SprayLockHoldMs;
            _numSprayTimeout.Value = (decimal)Config.SprayAlignTimeoutS;

            LoadJoystickSettings();
        }

        private void LoadJoystickSettings()
        {
            _chkJoyGimbalEnabled.Checked = Config.JoystickGimbalEnabled;
            SetComboBoxValue(
                _cmbJoyGimbalDevice,
                string.IsNullOrEmpty(Config.JoystickGimbalDevice) ? "(none)" : Config.JoystickGimbalDevice);
            SetComboBoxValue(_cmbJoyGimbalPitchAxis, Config.JoystickGimbalPitchAxis);
            _chkJoyGimbalPitchInvert.Checked = Config.JoystickGimbalPitchInvert;
            SetComboBoxValue(_cmbJoyGimbalRollAxis, Config.JoystickGimbalRollAxis);
            _chkJoyGimbalRollInvert.Checked = Config.JoystickGimbalRollInvert;
            _numJoyGimbalDeadzone.Value = ClampValue(_numJoyGimbalDeadzone, Config.JoystickGimbalDeadzone);
            _numJoyGimbalMaxRate.Value = ClampValue(_numJoyGimbalMaxRate, Config.JoystickGimbalMaxRateDegSec);

            _chkJoyCameraTiltEnabled.Checked = Config.JoystickCameraTiltEnabled;
            SetComboBoxValue(
                _cmbJoyCameraTiltDevice,
                string.IsNullOrEmpty(Config.JoystickCameraTiltDevice) ? "(none)" : Config.JoystickCameraTiltDevice);
            SetComboBoxValue(_cmbJoyCameraTiltAxis, Config.JoystickCameraTiltAxis);
            _chkJoyCameraTiltInvert.Checked = Config.JoystickCameraTiltInvert;
            _numJoyCameraTiltDeadzone.Value = ClampValue(_numJoyCameraTiltDeadzone, Config.JoystickCameraTiltDeadzone);
            _numJoyCameraTiltMaxRate.Value =
                ClampValue(_numJoyCameraTiltMaxRate, Config.JoystickCameraTiltMaxRateUsPerSec);

            SetComboBoxValue(
                _cmbSwitchDevice,
                string.IsNullOrEmpty(Config.JoystickSwitchDevice) ? "(none)" : Config.JoystickSwitchDevice);
            SetComboBoxValue(_cmbSw1Up, LabelForActionId(Config.JoystickSw1UpAction));
            SetComboBoxValue(_cmbSw1Down, LabelForActionId(Config.JoystickSw1DownAction));
            SetComboBoxValue(_cmbSw2Up, LabelForActionId(Config.JoystickSw2UpAction));
            SetComboBoxValue(_cmbSw2Down, LabelForActionId(Config.JoystickSw2DownAction));
            SetComboBoxValue(_cmbSw3Up, LabelForActionId(Config.JoystickSw3UpAction));
            SetComboBoxValue(_cmbSw3Down, LabelForActionId(Config.JoystickSw3DownAction));
            _chkJoyAutoSelect.Checked = Config.JoystickAutoSelectDevice;
            _chkKillSwitchEnabled.Checked = Config.JoystickKillSwitchEnabled;
            _chkSerialBridgeEnabled.Checked = Config.SerialJoystickEnabled;
            _cmbSerialBridgePort.Text = Config.SerialJoystickPort ?? "";
            _numSerialBridgeBaud.Value = ClampValue(_numSerialBridgeBaud, Config.SerialJoystickBaud);
            _txtSerialBridgePython.Text = Config.SerialJoystickPython ?? "python";
            _txtSerialBridgeScript.Text = Config.SerialJoystickScriptPath ?? "";
        }

        private void SaveSettings()
        {
            Config.CoreRuntimePort = (int)_numCoreRuntimePort.Value;
            Config.CoreApiKey = _txtCoreApiKey.Text.Trim();

            Config.VideoUrl = _txtVideoUrl.Text.Trim();
            Config.VideoNetworkCaching = (int)_numVideoCaching.Value;
            Config.PreferredVideoPlayer = _cmbVideoPlayer.SelectedItem?.ToString() ?? "Embedded";
            Config.VideoAutoStart = _chkVideoAutoStart.Checked;
            Config.AutoStartHudVideo = _chkAutoStartHudVideo.Checked;

            Config.DualLinkEnabled = _chkDualLinkEnabled.Checked;
            Config.RouterLocalPort = (int)_numRouterLocalPort.Value;
            Config.ManagementPort = (int)_numManagementPort.Value;

            Config.DarkMode = _chkDarkMode.Checked;
            Config.ShowNotifications = _chkShowNotifications.Checked;
            Config.DefaultTab = _cmbDefaultTab.SelectedItem?.ToString() ?? "Dashboard";
            Config.DebugMode = _chkDebugMode.Checked;
            Config.SlamCameraFovDeg = (float)_numSlamFov.Value;
            Config.SlamMapRadiusM = (float)_numSlamMapRadius.Value;
            Config.TempWarningC = (float)_numTempWarning.Value;
            Config.TempCriticalC = (float)_numTempCritical.Value;
            Config.AudioAlerts = _chkAudioAlerts.Checked;
            Config.AltitudeCallouts = _chkAltitudeCallouts.Checked;
            AudioAlerts.ApplyConfig(Config);

            Config.DefaultLogDirectory = _txtDefaultLogDirectory.Text.Trim();
            Config.LogVibrationWarning = (double)_numLogVibrationWarning.Value;
            Config.LogVibrationCritical = Math.Max(Config.LogVibrationWarning, (double)_numLogVibrationCritical.Value);
            Config.LogHdopWarning = (double)_numLogHdopWarning.Value;
            Config.LogHdopCritical = Math.Max(Config.LogHdopWarning, (double)_numLogHdopCritical.Value);
            Config.LogMinimumSatellites = (int)_numLogMinimumSatellites.Value;
            Config.LogTuneRmsWarning = (double)_numLogTuneWarning.Value;
            Config.LogTuneRmsCritical = Math.Max(Config.LogTuneRmsWarning, (double)_numLogTuneCritical.Value);
            Config.LogEkfVarianceWarning = (double)_numLogEkfWarning.Value;
            Config.LogEkfVarianceCritical = Math.Max(Config.LogEkfVarianceWarning, (double)_numLogEkfCritical.Value);
            Config.LogLiveBufferPoints = (int)_numLogLiveBufferPoints.Value;
            Config.LogInjectAlertsToHud = _chkLogInjectHud.Checked;

            SavePayloads();
            Config.SprayTargetCameraRangeM = (float)_numSprayRange.Value;
            Config.SprayRangeToleranceM = (float)_numSprayRangeTol.Value;
            Config.SprayTriggerMaxDistanceM = (float)_numSprayTriggerMax.Value;
            Config.SprayAimPixelX = (int)_numSprayAimX.Value;
            Config.SprayAimPixelY = (int)_numSprayAimY.Value;
            Config.SprayAimTolerancePx = (int)_numSprayAimTol.Value;
            Config.SprayServoFireAngleDeg = (float)_numSprayServoAngle.Value;
            Config.SprayForwardGain = (float)_numSprayForwardGain.Value;
            Config.SprayLateralGain = (float)_numSprayLateralGain.Value;
            Config.SprayAltitudeGain = (float)_numSprayAltitudeGain.Value;
            Config.SprayYawGain = (float)_numSprayYawGain.Value;
            Config.SprayUseYawAlignment = _chkSprayUseYaw.Checked;
            Config.SprayMaxForwardSpeedMps = (float)_numSprayMaxForward.Value;
            Config.SprayMaxLateralSpeedMps = (float)_numSprayMaxLateral.Value;
            Config.SprayMaxAltitudeSpeedMps = (float)_numSprayMaxAltitude.Value;
            Config.SprayMaxYawRateRadps = (float)_numSprayMaxYaw.Value;
            Config.SprayLockHoldMs = (int)_numSprayLockMs.Value;
            Config.SprayAlignTimeoutS = (float)_numSprayTimeout.Value;

            SaveJoystickSettings();
        }

        private void SaveJoystickSettings()
        {
            Config.JoystickGimbalEnabled = _chkJoyGimbalEnabled.Checked;
            Config.JoystickGimbalDevice = NormalizeDevice(_cmbJoyGimbalDevice.SelectedItem?.ToString());
            Config.JoystickGimbalPitchAxis = _cmbJoyGimbalPitchAxis.SelectedItem?.ToString() ?? "Y";
            Config.JoystickGimbalPitchInvert = _chkJoyGimbalPitchInvert.Checked;
            Config.JoystickGimbalRollAxis = _cmbJoyGimbalRollAxis.SelectedItem?.ToString() ?? "X";
            Config.JoystickGimbalRollInvert = _chkJoyGimbalRollInvert.Checked;
            Config.JoystickGimbalDeadzone = (float)_numJoyGimbalDeadzone.Value;
            Config.JoystickGimbalMaxRateDegSec = (float)_numJoyGimbalMaxRate.Value;
            GimbalController.MaxRateDegSec = Config.JoystickGimbalMaxRateDegSec;
            Config.JoystickCameraTiltEnabled = _chkJoyCameraTiltEnabled.Checked;
            Config.JoystickCameraTiltDevice = NormalizeDevice(_cmbJoyCameraTiltDevice.SelectedItem?.ToString());
            Config.JoystickCameraTiltAxis = _cmbJoyCameraTiltAxis.SelectedItem?.ToString() ?? "Y";
            Config.JoystickCameraTiltInvert = _chkJoyCameraTiltInvert.Checked;
            Config.JoystickCameraTiltDeadzone = (float)_numJoyCameraTiltDeadzone.Value;
            Config.JoystickCameraTiltMaxRateUsPerSec = (float)_numJoyCameraTiltMaxRate.Value;
            Config.JoystickSwitchDevice = NormalizeDevice(_cmbSwitchDevice?.SelectedItem?.ToString());
            Config.JoystickSw1UpAction = ActionIdForLabel(_cmbSw1Up?.SelectedItem?.ToString());
            Config.JoystickSw1DownAction = ActionIdForLabel(_cmbSw1Down?.SelectedItem?.ToString());
            Config.JoystickSw2UpAction = ActionIdForLabel(_cmbSw2Up?.SelectedItem?.ToString());
            Config.JoystickSw2DownAction = ActionIdForLabel(_cmbSw2Down?.SelectedItem?.ToString());
            Config.JoystickSw3UpAction = ActionIdForLabel(_cmbSw3Up?.SelectedItem?.ToString());
            Config.JoystickSw3DownAction = ActionIdForLabel(_cmbSw3Down?.SelectedItem?.ToString());
            Config.JoystickAutoSelectDevice = _chkJoyAutoSelect.Checked;
            Config.JoystickKillSwitchEnabled = _chkKillSwitchEnabled.Checked;
            Config.SerialJoystickEnabled = _chkSerialBridgeEnabled.Checked;
            Config.SerialJoystickPort = _cmbSerialBridgePort.Text.Trim();
            Config.SerialJoystickBaud = (int)_numSerialBridgeBaud.Value;
            Config.SerialJoystickPython = _txtSerialBridgePython.Text.Trim();
            Config.SerialJoystickScriptPath = _txtSerialBridgeScript.Text.Trim();
        }

        private static string NormalizeDevice(string value)
            => string.IsNullOrEmpty(value) || value == "(none)" ? "" : value;

        private void SetComboBoxValue(ComboBox combo, string value)
        {
            int index = combo.Items.IndexOf(value);
            combo.SelectedIndex = index >= 0 ? index : (combo.Items.Count > 0 ? 0 : -1);
        }

        private static decimal ClampValue(NumericUpDown control, double value)
            => Math.Max(control.Minimum, Math.Min(control.Maximum, (decimal)value));
    }
}
