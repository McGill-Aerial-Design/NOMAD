// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Linq;
using System.Windows.Forms;
using ComboBox = System.Windows.Controls.ComboBox;
using TextBox = System.Windows.Controls.TextBox;
using Button = System.Windows.Controls.Button;
using TabPage = System.Windows.Controls.TabItem;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private ComboBox _cmbAircraft, _cmbGimbalMountMode;
        private TextBox _txtAircraftName, _txtFlightController, _txtReceiver, _txtAircraftFirmware, _txtAirframe;
        private Button _btnRemoveAircraft;
        private bool _loadingAircraft;

        private TabPage CreateAircraftTab()
        {
            var tab = CreateTabPage("Aircraft");
            int y = 15;
            AddSectionLabel(tab, "Aircraft profiles", ref y);
            _cmbAircraft = AddComboBox(tab, 20, y, 330, Array.Empty<string>());
            _cmbAircraft.DisplayMemberPath = nameof(AircraftProfile.Name);
            _cmbAircraft.SelectionChanged += AircraftSelectionChanged;
            var create = WpfUi.Action("New", () => AddAircraft(false));
            AddAt(tab, create, 365, y, 85, 27);
            var copy = WpfUi.Action("Copy", () => AddAircraft(true));
            AddAt(tab, copy, 460, y, 85, 27);
            _btnRemoveAircraft = WpfUi.Action("Remove", RemoveAircraft);
            AddAt(tab, _btnRemoveAircraft, 555, y, 85, 27);
            y += 45;
            AddLabel(tab, "Aircraft name:", 20, y);
            _txtAircraftName = AddTextBox(tab, 180, y, 440);
            _txtAircraftName.TextChanged += (sender, args) => UpdateProfileContext();
            y += 32;
            AddSectionLabel(tab, "Optional aircraft notes", ref y);
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
            AddSectionLabel(tab, "Custom controls", ref y);
            AddLabel(tab, "Gimbal mode preset:", 20, y);
            _cmbGimbalMountMode = AddComboBox(tab, 180, y, 260,
                new[] { "Retract", "Neutral", "MAVLink targeting", "RC targeting" });
            return tab;
        }

        private void LoadAircraftSettings()
        {
            _cmbAircraft.Items.Clear();
            foreach (var profile in Profiles.Aircraft) _cmbAircraft.Items.Add(profile);
            _cmbAircraft.SelectedItem = Profiles.Selected;
            _btnRemoveAircraft.IsEnabled = Profiles.Aircraft.Count > 1;
            _txtAircraftName.Text = Config.AircraftName;
            _txtFlightController.Text = Config.FlightControllerModel;
            _txtReceiver.Text = Config.ReceiverType;
            _txtAircraftFirmware.Text = Config.AircraftFirmware;
            _txtAirframe.Text = Config.Airframe;
            _cmbGimbalMountMode.SelectedIndex = Config.GimbalMountMode;
            LoadPayloadList();

        }

        private void SaveAircraftSettings()
        {
            Config.AircraftName = _txtAircraftName.Text.Trim();
            Config.FlightControllerModel = _txtFlightController.Text.Trim();
            Config.ReceiverType = _txtReceiver.Text.Trim();
            Config.AircraftFirmware = _txtAircraftFirmware.Text.Trim();
            Config.Airframe = _txtAirframe.Text.Trim();
            Config.GimbalMountMode = _cmbGimbalMountMode.SelectedIndex;
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

    }
}
