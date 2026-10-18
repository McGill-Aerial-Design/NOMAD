// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using NOMAD.MissionPlanner.Connectivity;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private ComboBox _cmbAircraft, _cmbArmingChannel, _cmbFlightModeChannel, _cmbGimbalMountMode;
        private TextBox _txtAircraftName, _txtFlightController, _txtReceiver, _txtAircraftFirmware, _txtAirframe;
        private Button _btnRemoveAircraft;
        private bool _loadingAircraft;
        private volatile int _profileEditRevision;
        private volatile bool _runtimeConnectionUnchanged;
        private readonly string _initialProfileId, _initialCredential;
        private readonly int _initialRuntimePort;
        private readonly NomadCoreClient _initialCoreClient;

        private TabPage CreateAircraftTab()
        {
            var tab = CreateTabPage("Aircraft");
            int y = 15;
            AddSectionLabel(tab, "Aircraft profiles", ref y);
            _cmbAircraft = AddComboBox(tab, 20, y, 330, Array.Empty<string>());
            _cmbAircraft.DisplayMember = nameof(AircraftProfile.Name);
            _cmbAircraft.SelectedIndexChanged += AircraftSelectionChanged;
            var create = new Button { Text = "New", Location = new Point(365, y), Size = new Size(85, 27) };
            create.Click += (sender, args) => AddAircraft(false);
            tab.Controls.Add(create);
            var copy = new Button { Text = "Copy", Location = new Point(460, y), Size = new Size(85, 27) };
            copy.Click += (sender, args) => AddAircraft(true);
            tab.Controls.Add(copy);
            _btnRemoveAircraft = new Button { Text = "Remove", Location = new Point(555, y), Size = new Size(85, 27) };
            _btnRemoveAircraft.Click += (sender, args) => RemoveAircraft();
            tab.Controls.Add(_btnRemoveAircraft);
            y += 45;
            AddLabel(tab, "Aircraft name:", 20, y);
            _txtAircraftName = AddTextBox(tab, 180, y, 440);
            y += 32;
            AddLabel(tab, "Flight controller:", 20, y);
            _txtFlightController = AddTextBox(tab, 180, y, 440);
            y += 32;
            AddLabel(tab, "Receiver / RC link:", 20, y);
            _txtReceiver = AddTextBox(tab, 180, y, 440);
            y += 32;
            AddLabel(tab, "Firmware:", 20, y);
            _txtAircraftFirmware = AddTextBox(tab, 180, y, 440);
            y += 32;
            AddLabel(tab, "Airframe:", 20, y);
            _txtAirframe = AddTextBox(tab, 180, y, 440);
            y += 35;
            var channels = new[] { "Unconfigured" }.Concat(
                Enumerable.Range(1, 18).Select(channel => channel.ToString())).ToArray();
            AddLabel(tab, "Arming RC input:", 20, y);
            _cmbArmingChannel = AddComboBox(tab, 180, y, 140, channels);
            y += 32;
            AddLabel(tab, "Flight-mode RC input:", 20, y);
            _cmbFlightModeChannel = AddComboBox(tab, 180, y, 140, channels);
            y += 32;
            AddLabel(tab, "Gimbal mode preset:", 20, y);
            _cmbGimbalMountMode = AddComboBox(tab, 180, y, 260,
                new[] { "Retract", "Neutral", "MAVLink targeting", "RC targeting" });
            y += 45;
            tab.Controls.Add(new Label
            {
                Text = "All configuration tabs belong to the selected aircraft. " +
                    "Save makes it active and restores it on restart. " +
                    "Export omits its client credential. Hardware values still require FC readback and calibration.",
                Location = new Point(20, y), Size = new Size(650, 60), ForeColor = Color.Silver,
            });
            return tab;
        }

        private void LoadAircraftSettings()
        {
            _cmbAircraft.Items.Clear();
            _cmbAircraft.Items.AddRange(Profiles.Aircraft.Cast<object>().ToArray());
            _cmbAircraft.SelectedItem = Profiles.Selected;
            _btnRemoveAircraft.Enabled = Profiles.Aircraft.Count > 1;
            _txtAircraftName.Text = Config.AircraftName;
            _txtFlightController.Text = Config.FlightControllerModel;
            _txtReceiver.Text = Config.ReceiverType;
            _txtAircraftFirmware.Text = Config.AircraftFirmware;
            _txtAirframe.Text = Config.Airframe;
            _cmbArmingChannel.SelectedIndex = Config.RcArmingChannel ?? 0;
            _cmbFlightModeChannel.SelectedIndex = Config.RcFlightModeChannel ?? 0;
            _cmbGimbalMountMode.SelectedIndex = Config.GimbalMountMode;
            ApplyActuatorActions(Array.Empty<NomadActuator>());
            _actuatorJson.Text = Config.ActuatorDefinitions?.ToString(Formatting.Indented) ?? "";
            _actuatorConfigStatus.Text = Config.ActuatorDefinitions == null ?
                "No stored definitions. Load them from this aircraft's runtime or enter intended definitions." :
                "Stored definitions; runtime validation and explicit submission are separate from saving this profile.";
        }

        private void SaveAircraftSettings()
        {
            Config.AircraftName = _txtAircraftName.Text.Trim();
            Config.FlightControllerModel = _txtFlightController.Text.Trim();
            Config.ReceiverType = _txtReceiver.Text.Trim();
            Config.AircraftFirmware = _txtAircraftFirmware.Text.Trim();
            Config.Airframe = _txtAirframe.Text.Trim();
            Config.RcArmingChannel = _cmbArmingChannel.SelectedIndex > 0 ? (int?)_cmbArmingChannel.SelectedIndex : null;
            Config.RcFlightModeChannel = _cmbFlightModeChannel.SelectedIndex > 0 ?
                (int?)_cmbFlightModeChannel.SelectedIndex : null;
            Config.GimbalMountMode = _cmbGimbalMountMode.SelectedIndex;
            Config.ActuatorDefinitions = string.IsNullOrWhiteSpace(_actuatorJson.Text) ? null :
                NOMADConfig.ReadToken(_actuatorJson.Text) as JArray ??
                throw new JsonSerializationException("Actuator definitions must be an array.");
        }

        private void AircraftSelectionChanged(object sender, EventArgs args)
        {
            if (_loadingAircraft || !(_cmbAircraft.SelectedItem is AircraftProfile next) ||
                next.Id == Profiles.SelectedId)
            {
                return;
            }
            try
            {
                SaveSettings();
                Profiles.SelectedId = next.Id;
                LoadSettings();
            }
            catch (Exception error)
            {
                _loadingAircraft = true;
                _cmbAircraft.SelectedItem = Profiles.Selected;
                _loadingAircraft = false;
                MessageBox.Show(this, error.Message, "Aircraft profile requires review",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }

        private void AddAircraft(bool copy)
        {
            try
            {
                SaveSettings();
                var config = copy ? Config.Clone() : new NOMADConfig { ActuatorDefinitions = new JArray() };
                string prefix = copy ? Config.AircraftName + " copy" : "Aircraft";
                config.AircraftName = AvailableAircraftName(prefix);
                Profiles.Add(config);
                LoadSettings();
            }
            catch (Exception error)
            {
                MessageBox.Show(this, error.Message, "Aircraft profile requires review",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }

        private string AvailableAircraftName(string prefix)
        {
            for (int index = 1; ; index++)
            {
                string name = index == 1 ? prefix : prefix + " " + index;
                if (!Profiles.Aircraft.Any(profile =>
                    string.Equals(profile.Name, name, StringComparison.OrdinalIgnoreCase)))
                {
                    return name;
                }
            }
        }

        private void RemoveAircraft()
        {
            if (Profiles.Aircraft.Count <= 1)
            {
                return;
            }
            Profiles.RemoveSelected();
            LoadSettings();
        }

        private bool SettingsRuntimeIsCurrent()
        {
            return string.Equals(Profiles.SelectedId, _initialProfileId, StringComparison.OrdinalIgnoreCase) &&
                _runtimeConnectionUnchanged &&
                _initialCoreClient != null && _initialCoreClient == OutputController.CreateCoreClient();
        }

        private void RuntimeConnectionEdited()
        {
            _profileEditRevision++;
            _runtimeConnectionUnchanged = (int)_numCoreRuntimePort.Value == _initialRuntimePort &&
                _txtCoreClientCredential.Text.Trim() == _initialCredential;
        }
    }
}
