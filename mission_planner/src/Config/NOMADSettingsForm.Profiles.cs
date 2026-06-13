// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.IO;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private void BtnLoadConfig_Click(object sender, EventArgs e)
        {
            using (var dialog = new OpenFileDialog
            {
                Filter = "NOMAD configuration (*.json)|*.json|All files (*.*)|*.*",
                Title = "Load NOMAD Configuration"
            })
            {
                if (dialog.ShowDialog(this) != DialogResult.OK) return;

                try
                {
                    Config = NOMADConfig.LoadFromFile(dialog.FileName);
                    LoadSettings();
                    MessageBox.Show(
                        "Configuration loaded. Review the settings, then click Save to make this profile active.",
                        "NOMAD Configuration",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information
                    );
                }
                catch (Exception ex)
                {
                    MessageBox.Show(
                        $"Could not load the configuration:\n{ex.Message}",
                        "Load Configuration",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Error
                    );
                }
            }
        }

        private void BtnExportConfig_Click(object sender, EventArgs e)
        {
            var warning = MessageBox.Show(
                "The exported configuration may contain API keys and connection details. Store it securely.",
                "Export NOMAD Configuration",
                MessageBoxButtons.OKCancel,
                MessageBoxIcon.Warning
            );
            if (warning != DialogResult.OK) return;

            using (var dialog = new SaveFileDialog
            {
                AddExtension = true,
                DefaultExt = "json",
                FileName = $"nomad-config-{DateTime.Now:yyyyMMdd-HHmm}.json",
                Filter = "NOMAD configuration (*.json)|*.json|All files (*.*)|*.*",
                InitialDirectory = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
                OverwritePrompt = true,
                Title = "Export NOMAD Configuration"
            })
            {
                if (dialog.ShowDialog(this) != DialogResult.OK) return;

                try
                {
                    SaveSettings();
                    Config.ExportToFile(dialog.FileName);
                    MessageBox.Show(
                        $"Configuration exported to:\n{Path.GetFullPath(dialog.FileName)}",
                        "NOMAD Configuration",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information
                    );
                }
                catch (Exception ex)
                {
                    MessageBox.Show(
                        $"Could not export the configuration:\n{ex.Message}",
                        "Export Configuration",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Error
                    );
                }
            }
        }
    }
}
