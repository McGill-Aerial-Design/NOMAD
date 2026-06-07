// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.IO;
using System.Net.Http;
using System.Text;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private TabPage CreateUploadsTab()
        {
            var tab = CreateTabPage("Uploads");
            int y = 15;

            AddSectionLabel(tab, "Google Drive (Spray Photo Upload)", ref y);
            AddLabel(tab, "Upload OAuth2 token to Jetson (from gdrive_upload --setup):", 20, y, Color.LightGray);
            y += 25;

            _btnUploadGDrive = new Button
            {
                Text = "Upload GDrive Token...",
                Location = new Point(20, y),
                Size = new Size(200, 30),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            _btnUploadGDrive.FlatAppearance.BorderSize = 0;
            _btnUploadGDrive.Click += async (s, e) => await UploadGDriveCredentials();
            tab.Controls.Add(_btnUploadGDrive);

            _lblGDriveStatus = new Label
            {
                Text = "",
                Location = new Point(230, y + 5),
                AutoSize = true,
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 9),
            };
            tab.Controls.Add(_lblGDriveStatus);

            return tab;
        }

        private async System.Threading.Tasks.Task UploadGDriveCredentials()
        {
            using (var dlg = new OpenFileDialog())
            {
                dlg.Title = "Select Google Drive OAuth2 Token JSON";
                dlg.Filter = "JSON files (*.json)|*.json";
                var nomadDir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".nomad");
                if (Directory.Exists(nomadDir))
                    dlg.InitialDirectory = nomadDir;
                else
                    dlg.InitialDirectory = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);

                if (dlg.ShowDialog() != DialogResult.OK)
                    return;

                try
                {
                    _btnUploadGDrive.Enabled = false;
                    _lblGDriveStatus.Text = "Uploading...";
                    _lblGDriveStatus.ForeColor = Color.Yellow;

                    string jsonContent = File.ReadAllText(dlg.FileName);

                    var parsed = JObject.Parse(jsonContent);
                    if (parsed["token"] == null && parsed["refresh_token"] == null)
                    {
                        _lblGDriveStatus.Text = "Invalid token JSON (no token/refresh_token)";
                        _lblGDriveStatus.ForeColor = Color.Red;
                        return;
                    }

                    var content = new StringContent(jsonContent, Encoding.UTF8, "application/json");
                    var response = await JetsonApiService.PostAsync("/api/admin/upload-gdrive-token", content);

                    if (response.IsSuccessStatusCode)
                    {
                        _lblGDriveStatus.Text = "Token uploaded to Jetson";
                        _lblGDriveStatus.ForeColor = Color.LimeGreen;
                    }
                    else
                    {
                        var body = await response.Content.ReadAsStringAsync();
                        string detail = body;
                        try
                        {
                            var err = JObject.Parse(body);
                            detail = err["detail"]?.ToString() ?? body;
                        }
                        catch { }
                        _lblGDriveStatus.Text = $"Failed: {detail}";
                        _lblGDriveStatus.ForeColor = Color.Red;
                    }
                }
                catch (Exception ex)
                {
                    _lblGDriveStatus.Text = $"Error: {ex.Message}";
                    _lblGDriveStatus.ForeColor = Color.Red;
                }
                finally
                {
                    _btnUploadGDrive.Enabled = true;
                }
            }
        }
    }
}
