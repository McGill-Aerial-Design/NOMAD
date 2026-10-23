// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private ComboBox _cmbVideoInputs, _cmbVideoTransport;
        private TextBox _txtVideoName;
        private NumericUpDown _numVideoLatency, _numVideoQueue;
        private CheckBox _chkVideoDrop;
        private Button _btnRemoveVideo;
        private VideoInputProfile _editingVideoInput;
        private bool _loadingVideo;

        private TabPage CreateVideoTab()
        {
            var tab = CreateTabPage("Video");
            int y = 15;

            AddSectionLabel(tab, "Named video inputs", ref y);
            _cmbVideoInputs = AddComboBox(tab, 20, y, 330, Array.Empty<string>());
            _cmbVideoInputs.DisplayMember = nameof(VideoInputProfile.Name);
            _cmbVideoInputs.SelectedIndexChanged += VideoInputSelectionChanged;
            var add = ControlFactory.Button("Add", width: 85, height: 27);
            add.Location = new Point(365, y);
            add.Click += (sender, args) => AddVideoInput();
            tab.Controls.Add(add);
            _btnRemoveVideo = ControlFactory.Button("Remove", width: 85, height: 27);
            _btnRemoveVideo.Location = new Point(460, y);
            _btnRemoveVideo.Click += (sender, args) => RemoveVideoInput();
            tab.Controls.Add(_btnRemoveVideo);
            y += 40;
            AddSectionLabel(tab, "Source", ref y);
            AddLabel(tab, "Input name:", 20, y);
            _txtVideoName = AddTextBox(tab, 180, y, 440);
            y += 32;
            AddLabel(tab, "Source URL:", 20, y);
            _txtVideoUrl = AddTextBox(tab, 180, y, 440);
            y += 32;
            AddSectionLabel(tab, "Receiver tuning", ref y);
            AddLabel(tab, "RTSP transport:", 20, y);
            _cmbVideoTransport = AddComboBox(tab, 180, y, 140, new[] { "TCP", "UDP" });
            y += 32;
            AddLabel(tab, "RTSP jitter buffer (ms):", 20, y);
            _numVideoLatency = AddNumericUpDown(tab, 180, y, 100, 0, 5000, 100);
            y += 32;
            AddLabel(tab, "Decode queue (frames):", 20, y);
            _numVideoQueue = AddNumericUpDown(tab, 180, y, 100, 1, 120, 1);
            y += 32;
            _chkVideoDrop = AddCheckBox(tab, "Drop old frames when receiver buffers are full", 20, y);
            y += 32;

            _chkAutoStartHudVideo = AddCheckBox(tab, "Auto-start video on HUD", 20, y);
            y += 35;
            AddLabel(tab, "Choose the active input at the top-right of the video player.", 20, y);
            y += 32;
            var info = ControlFactory.Button("Receiver help ⓘ", width: 150, height: 28);
            info.Location = new Point(20, y);
            info.AccessibleName = "Video receiver help";
            string help = "RTSP negotiates its video codec. UDP listens for RTP/H.264 on the explicit port.\n" +
                "RTSP transport and jitter settings do not affect UDP inputs.\n" +
                "The display always shows the latest frame; queue backpressure does not preserve every frame.\n" +
                "Bitrate, resolution and encoded quality are set at the camera or encoder.";
            _toolTips.SetToolTip(info, help);
            _toolTips.SetToolTip(_txtVideoUrl, "Absolute RTSP URL or UDP listening port, for example udp://@:5600.");
            _toolTips.SetToolTip(_numVideoLatency, "RTSP jitter buffering, not end-to-end video delay. Range: 0–5000 ms.");
            _toolTips.SetToolTip(_numVideoQueue, "Decoded-frame queue: 1–120 frames. The display still shows only the latest frame.");
            _toolTips.SetToolTip(_chkVideoDrop, "Drop older decode-queue frames and bound RTSP jitter buffering when full.");
            info.Click += (sender, args) => _toolTips.Show(help, info, 0, info.Height, 15000);
            tab.Controls.Add(info);

            return tab;
        }

        private void LoadVideoSettings(VideoInputProfile editing = null)
        {
            _loadingVideo = true;
            try
            {
                _cmbVideoInputs.Items.Clear();
                _cmbVideoInputs.Items.AddRange(Config.VideoInputs.Cast<object>().ToArray());
                editing = editing ?? Config.SelectedVideoInput;
                _cmbVideoInputs.SelectedItem = editing;
                LoadVideoInput(editing);
            }
            finally { _loadingVideo = false; }
        }

        private void LoadVideoInput(VideoInputProfile input)
        {
            _editingVideoInput = input;
            _txtVideoName.Text = input?.Name ?? "";
            _txtVideoUrl.Text = input?.Url ?? "";
            _cmbVideoTransport.SelectedItem = input?.RtspTransport ?? "TCP";
            _numVideoLatency.Value = input?.LatencyMs ?? 100;
            _numVideoQueue.Value = input?.QueueMaxBuffers ?? 1;
            _chkVideoDrop.Checked = input?.DropOldFrames ?? true;
            foreach (var control in new Control[] { _txtVideoName, _txtVideoUrl, _cmbVideoTransport,
                _numVideoLatency, _numVideoQueue, _chkVideoDrop, _btnRemoveVideo })
            { control.Enabled = input != null; }
        }

        private void SaveVideoInput()
        {
            if (_editingVideoInput == null) { return; }
            var input = new VideoInputProfile
            {
                Name = _txtVideoName.Text.Trim(), Url = _txtVideoUrl.Text.Trim(),
                RtspTransport = (string)_cmbVideoTransport.SelectedItem,
                LatencyMs = (int)_numVideoLatency.Value, QueueMaxBuffers = (int)_numVideoQueue.Value,
                DropOldFrames = _chkVideoDrop.Checked,
            };
            input.Validate();
            if (Config.VideoInputs.Any(other => !ReferenceEquals(other, _editingVideoInput) &&
                string.Equals(other.Name, input.Name, StringComparison.OrdinalIgnoreCase)))
            { throw new Newtonsoft.Json.JsonSerializationException("Video input names must be unique."); }
            int index = Config.VideoInputs.IndexOf(_editingVideoInput);
            bool wasActive = string.Equals(Config.SelectedVideoInputName, _editingVideoInput.Name, StringComparison.OrdinalIgnoreCase);
            Config.VideoInputs[index] = input;
            _editingVideoInput = input;
            if (wasActive) { Config.SelectedVideoInputName = input.Name; }
            _loadingVideo = true;
            try { _cmbVideoInputs.Items[index] = input; }
            finally { _loadingVideo = false; }
        }

        private void VideoInputSelectionChanged(object sender, EventArgs args)
        {
            if (_loadingVideo || _loadingAircraft) { return; }
            var next = _cmbVideoInputs.SelectedItem as VideoInputProfile;
            if (next == null || ReferenceEquals(next, _editingVideoInput)) { return; }
            try
            {
                SaveVideoInput();
                LoadVideoInput(next);
            }
            catch (Exception error)
            {
                _loadingVideo = true;
                _cmbVideoInputs.SelectedItem = _editingVideoInput;
                _loadingVideo = false;
                MessageBox.Show(this, error.Message, "Video input requires review", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }

        private void AddVideoInput()
        {
            try
            {
                SaveVideoInput();
                int number = 1;
                while (Config.VideoInputs.Any(input => input.Name.Equals("Input " + number, StringComparison.OrdinalIgnoreCase)))
                { number++; }
                var added = new VideoInputProfile { Name = "Input " + number };
                Config.VideoInputs.Add(added);
                if (Config.VideoInputs.Count == 1) { Config.SelectedVideoInputName = added.Name; }
                LoadVideoSettings(added);
            }
            catch (Exception error)
            { MessageBox.Show(this, error.Message, "Video input requires review", MessageBoxButtons.OK, MessageBoxIcon.Warning); }
        }

        private void RemoveVideoInput()
        {
            if (_editingVideoInput == null) { return; }
            bool wasActive = string.Equals(Config.SelectedVideoInputName, _editingVideoInput.Name, StringComparison.OrdinalIgnoreCase);
            Config.VideoInputs.Remove(_editingVideoInput);
            if (wasActive) { Config.SelectedVideoInputName = Config.VideoInputs.FirstOrDefault()?.Name ?? ""; }
            LoadVideoSettings();
        }

        private TabPage CreatePreferencesTab()
        {
            var tab = CreateTabPage("Preferences");
            int y = 15;
            AddSectionLabel(tab, "Startup", ref y);
            _chkDebugMode = AddCheckBox(tab, "Show startup diagnostics popup", 20, y);
            y += 40;
            AddSectionLabel(tab, "Alerts", ref y);
            _chkAudioAlerts = AddCheckBox(tab, "Enable audio alerts for critical warnings", 20, y);
            y += 30;
            _chkAltitudeCallouts = AddCheckBox(tab, "Speak altitude callouts in flight (Airbus-style)", 20, y);

            return tab;
        }
    }
}
