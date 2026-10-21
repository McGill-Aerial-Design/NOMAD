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
            _loadingAircraft = true;
            try
            {
                LoadProfileSettings();
            }
            finally
            {
                _loadingAircraft = false;
            }
        }

        private void LoadProfileSettings()
        {
            LoadAircraftSettings();

            _txtVideoUrl.Text = Config.VideoUrl;
            _chkAutoStartHudVideo.Checked = Config.AutoStartHudVideo;

            _chkRouterClientEnabled.Checked = Config.RouterClientEnabled;
            _numRouterLocalPort.Value = ClampValue(_numRouterLocalPort, Config.RouterLocalPort);
            _txtRouterLinks.Text = Newtonsoft.Json.JsonConvert.SerializeObject(Config.RouterLinks, Newtonsoft.Json.Formatting.Indented);
            _txtRouterPreferredLink.Text = Config.RouterPreferredLink;

            _chkDebugMode.Checked = Config.DebugMode;

            _chkAudioAlerts.Checked = Config.AudioAlerts;
            _chkAltitudeCallouts.Checked = Config.AltitudeCallouts;




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
            UpdatePositionEligibility();

        }

        private void SaveSettings()
        {
            SaveAircraftSettings();

            Config.VideoUrl = _txtVideoUrl.Text.Trim();
            Config.AutoStartHudVideo = _chkAutoStartHudVideo.Checked;

            Config.RouterClientEnabled = _chkRouterClientEnabled.Checked;
            Config.RouterLocalPort = (int)_numRouterLocalPort.Value;
            Config.RouterLinks = NOMADConfig.ParseRouterLinks(_txtRouterLinks.Text);
            Config.RouterPreferredLink = _txtRouterPreferredLink.Text.Trim();

            Config.DebugMode = _chkDebugMode.Checked;
            Config.AudioAlerts = _chkAudioAlerts.Checked;
            Config.AltitudeCallouts = _chkAltitudeCallouts.Checked;




            SaveJoystickSettings();
            Profiles.Validate();
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
            Config.JoystickPositionEnabled = _chkJoyPositionEnabled.Checked;
            Config.JoystickPositionDevice = NormalizeDevice(_cmbJoyPositionDevice.SelectedItem?.ToString());
            Config.JoystickPositionAxis = _cmbJoyPositionAxis.SelectedItem?.ToString() ?? "Y";
            Config.JoystickPositionInvert = _chkJoyPositionInvert.Checked;
            Config.JoystickPositionDeadzone = (float)_numJoyPositionDeadzone.Value;
            Config.JoystickPositionActuatorId = _txtJoyPositionActuatorId.Text.Trim();
            Config.JoystickButtonIndices = System.Array.ConvertAll(_switchButtons, control => (int)control.Value);
            Config.JoystickSwitchDevice = NormalizeDevice(_cmbSwitchDevice?.SelectedItem?.ToString());
            Config.JoystickSw1UpAction = (_cmbSw1Up?.Text ?? "None");
            Config.JoystickSw1DownAction = (_cmbSw1Down?.Text ?? "None");
            Config.JoystickSw2UpAction = (_cmbSw2Up?.Text ?? "None");
            Config.JoystickSw2DownAction = (_cmbSw2Down?.Text ?? "None");
            Config.JoystickSw3UpAction = (_cmbSw3Up?.Text ?? "None");
            Config.JoystickSw3DownAction = (_cmbSw3Down?.Text ?? "None");
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
