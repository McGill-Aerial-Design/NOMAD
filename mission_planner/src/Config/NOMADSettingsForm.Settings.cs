// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMADSettingsForm.Settings.cs - Config <-> control sync
// ============================================================

using System;
using ComboBox = System.Windows.Controls.ComboBox;
using NumericUpDown = NOMAD.MissionPlanner.WpfNumberBox;

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
            _chkAutoStartHudVideo.IsChecked = Config.AutoStartHudVideo;

            _chkRouterClientEnabled.IsChecked = Config.RouterClientEnabled;
            _numRouterLocalPort.Value = ClampValue(_numRouterLocalPort, Config.RouterLocalPort);
            LoadRouterLinks();

            _chkDebugMode.IsChecked = Config.DebugMode;

            _chkAudioAlerts.IsChecked = Config.AudioAlerts;
            _chkAltitudeCallouts.IsChecked = Config.AltitudeCallouts;




            LoadJoystickSettings();
        }

        private void LoadJoystickSettings()
        {
            _numJoyGimbalMaxRate.Value = ClampValue(_numJoyGimbalMaxRate, Config.JoystickGimbalMaxRateDegSec);
            _cmbGimbalHorizontal.SelectedItem = Config.JoystickGimbalHorizontalAxis;
            _cmbGimbalVertical.SelectedItem = Config.JoystickGimbalVerticalAxis;
            _chkGimbalHorizontalInverted.IsChecked = Config.JoystickGimbalHorizontalInverted;
            _chkGimbalVerticalInverted.IsChecked = Config.JoystickGimbalVerticalInverted;
            LoadInputList();
        }

        private void SaveSettings()
        {
            var numericFields = new[] { _numRouterLocalPort, _numJoyGimbalMaxRate };
            foreach (var field in numericFields) { _ = field.Value; }
            if (_editingVideoInput != null)
            {
                var videoFields = new[] { _numVideoLatency, _numVideoQueue, _numGroundPlaneHeight, _numCameraDelay,
                    _numVisionFov, _numVisionPitchMin, _numVisionPitchMax, _numVisionYawMin, _numVisionYawMax, _numVisionRate };
                foreach (var field in videoFields) { _ = field.Value; }
            }
            SaveAircraftSettings();

            SaveVideoInput();
            Config.AutoStartHudVideo = _chkAutoStartHudVideo.IsChecked == true;

            Config.RouterClientEnabled = _chkRouterClientEnabled.IsChecked == true;
            Config.RouterLocalPort = (int)_numRouterLocalPort.Value;
            Config.RouterPreferredLink = BindingValue(_cmbRouterPreferred);

            Config.DebugMode = _chkDebugMode.IsChecked == true;
            Config.AudioAlerts = _chkAudioAlerts.IsChecked == true;
            Config.AltitudeCallouts = _chkAltitudeCallouts.IsChecked == true;




            SaveJoystickSettings();
            Profiles.Validate();
        }

        private void SaveJoystickSettings()
        {
            Config.JoystickGimbalMaxRateDegSec = (float)_numJoyGimbalMaxRate.Value;
            Config.JoystickGimbalHorizontalAxis = _cmbGimbalHorizontal.SelectedItem as string;
            Config.JoystickGimbalVerticalAxis = _cmbGimbalVertical.SelectedItem as string;
            Config.JoystickGimbalHorizontalInverted = _chkGimbalHorizontalInverted.IsChecked == true;
            Config.JoystickGimbalVerticalInverted = _chkGimbalVerticalInverted.IsChecked == true;
            Config.ValidateInputBindings();
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
