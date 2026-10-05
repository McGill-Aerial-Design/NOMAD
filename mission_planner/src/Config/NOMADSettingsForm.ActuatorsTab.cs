// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System.Drawing;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json;
using NOMAD.MissionPlanner.Connectivity;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private TextBox _actuatorJson;
        private Label _actuatorConfigStatus;
        private TabPage CreateActuatorsTab()
        {
            var tab = CreateTabPage("Actuators");
            AddLabel(tab, "Actuator definitions are validated and stored by nomad-runtime.", 10, 10, Color.Silver);
            AddLabel(tab, "Load the current definitions, edit JSON data, then submit explicitly while disarmed.", 10, 30, Color.Silver);
            _actuatorJson = new TextBox { Location = new Point(10, 58), Size = new Size(710, 280),
                Multiline = true, ScrollBars = ScrollBars.Both, WordWrap = false, Text = "[]" };
            tab.Controls.Add(_actuatorJson);
            var load = new Button { Text = "Load from runtime", Location = new Point(10, 345), Size = new Size(150, 28) };
            load.Click += async (s, e) => await LoadActuatorConfigurationAsync();
            tab.Controls.Add(load);
            var submit = new Button { Text = "Submit to runtime", Location = new Point(170, 345), Size = new Size(150, 28) };
            submit.Click += async (s, e) =>
            {
                NomadCoreRequestResult result = null;
                try
                {
                    result = await OutputController.ConfigureActuatorsAsync(_actuatorJson.Text);
                    if (IsDisposed)
                    {
                        return;
                    }
                    _actuatorConfigStatus.Text = OutputController.DescribeConfigurationResult(result);
                    if (!result.PresentationCurrent)
                    { _actuatorConfigStatus.Text += " Response belongs to an earlier runtime display; refresh the current configuration."; }
                    else if (result.Succeeded)
                    {
                        ApplyActuatorActions(result.Actuators);
                    }
                }
                catch (System.Exception ex)
                {
                    if (IsDisposed)
                    {
                        return;
                    }
                    _actuatorConfigStatus.Text = (result == null ?
                        "Configuration request did not complete; its disposition is unverified. Inspect runtime configuration before submitting again. " :
                        OutputController.DescribeConfigurationResult(result) + " Display update failed. ") + ex.Message;
                }
            };
            tab.Controls.Add(submit);
            _actuatorConfigStatus = new Label { Location = new Point(10, 382), Size = new Size(710, 70), ForeColor = Color.Silver };
            tab.Controls.Add(_actuatorConfigStatus);
            return tab;
        }

        private async Task LoadActuatorConfigurationAsync()
        {
            var result = await OutputController.GetActuatorsAsync();
            if (IsDisposed || !result.PresentationCurrent)
            {
                return;
            }
            _actuatorConfigStatus.Text = result.Message;
            if (!result.Succeeded)
            {
                return;
            }
            _actuatorJson.Text = JsonConvert.SerializeObject(result.Actuators.Select(a => a.Config), Formatting.Indented);
            ApplyActuatorActions(result.Actuators);
        }
    }
}
