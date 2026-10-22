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

            LoadVideoSettings();
            _chkAutoStartHudVideo.Checked = Config.AutoStartHudVideo;

            _chkRouterClientEnabled.Checked = Config.RouterClientEnabled;
            _numRouterLocalPort.Value = ClampValue(_numRouterLocalPort, Config.RouterLocalPort);
            LoadRouterLinks();

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
            RefreshPayloadBindings(false);
            SetDeviceComboValue(_cmbSwitchDevice, string.IsNullOrEmpty(Config.JoystickSwitchDevice) ? "(none)" : Config.JoystickSwitchDevice);
            for (int index = 0; index < SwitchActions.Length; index++)
            {
                _switchButtons[index].Value = Config.JoystickButtonIndices[index];
            }
            UpdatePositionEligibility();

        }

        private void SaveSettings()
        {
            SaveAircraftSettings();

            SaveVideoInput();
            Config.AutoStartHudVideo = _chkAutoStartHudVideo.Checked;

            Config.RouterClientEnabled = _chkRouterClientEnabled.Checked;
            Config.RouterLocalPort = (int)_numRouterLocalPort.Value;
            Config.RouterPreferredLink = BindingValue(_cmbRouterPreferred);

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
            Config.JoystickPositionActuatorId = BindingValue(_cmbJoyPositionPayload);
            Config.JoystickButtonIndices = System.Array.ConvertAll(_switchButtons, control => (int)control.Value);
            Config.JoystickSwitchDevice = NormalizeDevice(_cmbSwitchDevice?.SelectedItem?.ToString());
            Config.JoystickSw1UpAction = BindingValue(_cmbSw1Up);
            Config.JoystickSw1DownAction = BindingValue(_cmbSw1Down);
            Config.JoystickSw2UpAction = BindingValue(_cmbSw2Up);
            Config.JoystickSw2DownAction = BindingValue(_cmbSw2Down);
            Config.JoystickSw3UpAction = BindingValue(_cmbSw3Up);
            Config.JoystickSw3DownAction = BindingValue(_cmbSw3Down);
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
