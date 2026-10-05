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
            _txtCoreClientCredential.Text = Config.CoreClientCredential ?? "";

            _txtVideoUrl.Text = Config.VideoUrl;
            _numVideoCaching.Value = ClampValue(_numVideoCaching, Config.VideoNetworkCaching);
            SetComboBoxValue(_cmbVideoPlayer, Config.PreferredVideoPlayer);
            _chkVideoAutoStart.Checked = Config.VideoAutoStart;
            _chkAutoStartHudVideo.Checked = Config.AutoStartHudVideo;

            _chkRouterClientEnabled.Checked = Config.DualLinkEnabled;
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



            LoadJoystickSettings();
        }

        private void LoadJoystickSettings()
        {
            _chkJoyGimbalEnabled.Checked = Config.JoystickGimbalEnabled;
            SetDeviceComboValue(
                _cmbJoyGimbalDevice,
                string.IsNullOrEmpty(Config.JoystickGimbalDevice) ? "(none)" : Config.JoystickGimbalDevice);
            SetComboBoxValue(_cmbJoyGimbalPitchAxis, Config.JoystickGimbalPitchAxis);
            _chkJoyGimbalPitchInvert.Checked = Config.JoystickGimbalPitchInvert;
            SetComboBoxValue(_cmbJoyGimbalRollAxis, Config.JoystickGimbalRollAxis);
            _chkJoyGimbalRollInvert.Checked = Config.JoystickGimbalRollInvert;
            _numJoyGimbalDeadzone.Value = ClampValue(_numJoyGimbalDeadzone, Config.JoystickGimbalDeadzone);
            _numJoyGimbalMaxRate.Value = ClampValue(_numJoyGimbalMaxRate, Config.JoystickGimbalMaxRateDegSec);

            _chkJoyPositionEnabled.Checked = Config.JoystickPositionEnabled;
            SetDeviceComboValue(
                _cmbJoyPositionDevice,
                string.IsNullOrEmpty(Config.JoystickPositionDevice) ? "(none)" : Config.JoystickPositionDevice);
            SetComboBoxValue(_cmbJoyPositionAxis, Config.JoystickPositionAxis);
            _chkJoyPositionInvert.Checked = Config.JoystickPositionInvert;
            _numJoyPositionDeadzone.Value = ClampValue(_numJoyPositionDeadzone, Config.JoystickPositionDeadzone);
            _txtJoyPositionActuatorId.Text = Config.JoystickPositionActuatorId;
            SetDeviceComboValue(_cmbSwitchDevice, string.IsNullOrEmpty(Config.JoystickSwitchDevice) ? "(none)" : Config.JoystickSwitchDevice);
            var actionCombos = new[] { _cmbSw1Up, _cmbSw1Down, _cmbSw2Up, _cmbSw2Down, _cmbSw3Up, _cmbSw3Down };
            var actionIds = new[] { Config.JoystickSw1UpAction, Config.JoystickSw1DownAction, Config.JoystickSw2UpAction,
                Config.JoystickSw2DownAction, Config.JoystickSw3UpAction, Config.JoystickSw3DownAction };
            for (int index = 0; index < actionCombos.Length; index++)
            {
                actionCombos[index].Text = actionIds[index];
                _switchButtons[index].Value = Config.JoystickButtonIndices[index];
            }
            _chkKillSwitchEnabled.Checked = Config.JoystickKillSwitchEnabled;

        }

        private void SaveSettings()
        {
            Config.CoreRuntimePort = (int)_numCoreRuntimePort.Value;
            Config.CoreClientCredential = _txtCoreClientCredential.Text.Trim();

            Config.VideoUrl = _txtVideoUrl.Text.Trim();
            Config.VideoNetworkCaching = (int)_numVideoCaching.Value;
            Config.PreferredVideoPlayer = _cmbVideoPlayer.SelectedItem?.ToString() ?? "Embedded";
            Config.VideoAutoStart = _chkVideoAutoStart.Checked;
            Config.AutoStartHudVideo = _chkAutoStartHudVideo.Checked;

            Config.DualLinkEnabled = _chkRouterClientEnabled.Checked;
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
            Config.JoystickPositionEnabled = _chkJoyPositionEnabled.Checked;
            Config.JoystickPositionDevice = NormalizeDevice(_cmbJoyPositionDevice.SelectedItem?.ToString());
            Config.JoystickPositionAxis = _cmbJoyPositionAxis.SelectedItem?.ToString() ?? "Y";
            Config.JoystickPositionInvert = _chkJoyPositionInvert.Checked;
            Config.JoystickPositionDeadzone = (float)_numJoyPositionDeadzone.Value;
            Config.JoystickPositionActuatorId = _txtJoyPositionActuatorId.Text.Trim();
            Config.JoystickButtonIndices = System.Array.ConvertAll(_switchButtons, control => (int)control.Value);
            Config.JoystickSwitchDevice = NormalizeDevice(_cmbSwitchDevice?.SelectedItem?.ToString());
            Config.JoystickSw1UpAction = ActionIdForLabel(_cmbSw1Up?.Text);
            Config.JoystickSw1DownAction = ActionIdForLabel(_cmbSw1Down?.Text);
            Config.JoystickSw2UpAction = ActionIdForLabel(_cmbSw2Up?.Text);
            Config.JoystickSw2DownAction = ActionIdForLabel(_cmbSw2Down?.Text);
            Config.JoystickSw3UpAction = ActionIdForLabel(_cmbSw3Up?.Text);
            Config.JoystickSw3DownAction = ActionIdForLabel(_cmbSw3Down?.Text);
            Config.JoystickKillSwitchEnabled = _chkKillSwitchEnabled.Checked;
            Config.ValidateInputBindings();

        }

        private static string NormalizeDevice(string value)
            => string.IsNullOrEmpty(value) || value == "(none)" ? "" : value;

        private static void SetDeviceComboValue(ComboBox combo, string value)
        {
            string selected = string.IsNullOrEmpty(value) ? "(none)" : value;
            if (!combo.Items.Contains(selected))
            {
                combo.Items.Add(selected);
            }
            combo.SelectedItem = selected;
        }

        private void SetComboBoxValue(ComboBox combo, string value)
        {
            int index = combo.Items.IndexOf(value);
            combo.SelectedIndex = index >= 0 ? index : (combo.Items.Count > 0 ? 0 : -1);
        }

        private static decimal ClampValue(NumericUpDown control, double value)
            => Math.Max(control.Minimum, Math.Min(control.Maximum, (decimal)value));
    }
}
