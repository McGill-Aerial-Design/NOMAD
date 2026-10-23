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
            _numJoyGimbalMaxRate.Value = ClampValue(_numJoyGimbalMaxRate, Config.JoystickGimbalMaxRateDegSec);
            LoadInputList();
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
            Config.JoystickGimbalMaxRateDegSec = (float)_numJoyGimbalMaxRate.Value;
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
