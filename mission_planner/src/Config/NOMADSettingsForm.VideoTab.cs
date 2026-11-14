// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Linq;
using System.Windows.Forms;
using System.Windows;
using ComboBox = System.Windows.Controls.ComboBox;
using TextBox = System.Windows.Controls.TextBox;
using CheckBox = System.Windows.Controls.CheckBox;
using Label = System.Windows.Controls.TextBlock;
using Button = System.Windows.Controls.Button;
using TabPage = System.Windows.Controls.TabItem;
using NumericUpDown = NOMAD.MissionPlanner.WpfNumberBox;
using MessageBox = System.Windows.Forms.MessageBox;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private ComboBox _cmbVideoInputs, _cmbVideoTransport, _cmbVideoSourceType, _cmbVideoDevice;
        private Button _btnVideoDevices;
        private Label _lblVideoDevices;
        private Func<VideoCaptureDevice[]> _getVideoDevices = VideoCaptureDevice.Enumerate;
        private TextBox _txtVideoName;
        private NumericUpDown _numVideoLatency, _numVideoQueue;
        private CheckBox _chkVideoDrop, _chkRtspSkipOptions;
        private Button _btnRemoveVideo;
        private VideoInputProfile _editingVideoInput;
        private bool _loadingVideo;
        private ComboBox _cmbVisionModel;
        private CheckBox _chkTrackingCalibrated;
        private NumericUpDown _numVisionFov, _numVisionPitchMin, _numVisionPitchMax;
        private NumericUpDown _numVisionYawMin, _numVisionYawMax, _numVisionRate;
        private ComboBox _cmbVisionPitchSign, _cmbVisionYawSign;
        private Button _btnVisionModelBrowse;
        private ComboBox _cmbTrackingMode, _cmbObjectTracker;
        private CheckBox _chkGroundPrediction;
        private NumericUpDown _numGroundPlaneHeight, _numCameraDelay;

        private TabPage CreateVideoTab()
        {
            var tab = CreateTabPage("Video");
            int y = 15;

            AddSectionLabel(tab, "Named video inputs", ref y);
            _cmbVideoInputs = AddComboBox(tab, 20, y, 330, Array.Empty<string>());
            _cmbVideoInputs.DisplayMemberPath = nameof(VideoInputProfile.Name);
            _cmbVideoInputs.SelectionChanged += VideoInputSelectionChanged;
            var add = WpfUi.Action("Add", AddVideoInput);
            AddAt(tab, add, 365, y, 85, 27);
            _btnRemoveVideo = WpfUi.Action("Remove", RemoveVideoInput);
            AddAt(tab, _btnRemoveVideo, 460, y, 85, 27);
            y += 40;
            AddSectionLabel(tab, "Source", ref y);
            AddLabel(tab, "Input name:", 20, y);
            _txtVideoName = AddTextBox(tab, 180, y, 440);
            y += 32;
            AddLabel(tab, "Source type:", 20, y);
            _cmbVideoSourceType = AddComboBox(tab, 180, y, 250, new[] { "Network stream", "Camera / capture card" });
            System.Windows.Automation.AutomationProperties.SetName(_cmbVideoSourceType, "Video source type");
            _cmbVideoSourceType.SelectionChanged += (sender, args) =>
            {
                if (!_loadingVideo) { _chkTrackingCalibrated.IsChecked = false; }
                UpdateVideoSourceFields();
            };
            y += 32;
            AddLabel(tab, "Source URL:", 20, y);
            _txtVideoUrl = AddTextBox(tab, 180, y, 440);
            _txtVideoUrl.TextChanged += (sender, args) =>
            {
                if (!_loadingVideo) { _chkTrackingCalibrated.IsChecked = false; }
            };
            y += 32;
            AddLabel(tab, "Capture device:", 20, y);
            _cmbVideoDevice = AddComboBox(tab, 180, y, 330, Array.Empty<string>());
            _cmbVideoDevice.DisplayMemberPath = nameof(VideoCaptureDevice.Name);
            System.Windows.Automation.AutomationProperties.SetName(_cmbVideoDevice, "Local camera or capture card");
            _cmbVideoDevice.SelectionChanged += (sender, args) =>
            {
                if (!_loadingVideo) { _chkTrackingCalibrated.IsChecked = false; }
            };
            _btnVideoDevices = WpfUi.Action("Refresh", RefreshVideoDevices);
            AddAt(tab, _btnVideoDevices, 520, y, 100, 27);
            y += 30;
            _lblVideoDevices = AddLabel(tab, "Refresh to list connected devices.", 180, y);
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
            _chkVideoDrop = AddCheckBox(tab, "Drop old frames", 20, y);
            y += 32;
            _chkRtspSkipOptions = AddCheckBox(tab, "Skip RTSP OPTIONS and keep-alives", 20, y);
            y += 32;

            _chkAutoStartHudVideo = AddCheckBox(tab, "Start video on HUD", 20, y);
            y += 35;
            var info = new Button { Content = "Input help ⓘ" };
            System.Windows.Automation.AutomationProperties.SetName(info, "Video receiver help");
            string help = "Select a network stream or a Windows camera / USB capture card.\n" +
                "Refresh lists supported devices without opening a camera. Playback and HUD autostart begin capture.\n" +
                "Windows capture negotiates a supported format up to 1920×1080 at 30 FPS.\n" +
                "The capture driver may briefly probe other cameras during startup; only the selected device streams.\n" +
                "HUD, Dashboard and Video share one local capture; each view displays the latest frame.\n" +
                "RTSP negotiates its video codec. UDP listens for RTP/H.264 on the explicit port.\n" +
                "Receiver queue tuning applies to network sources; local capture uses a one-frame dropping queue.\n" +
                "RTSP transport and jitter settings do not affect UDP inputs.\n" +
                "The display always shows the latest frame; queue backpressure does not preserve every frame.\n" +
                "Bitrate, resolution and encoded quality are set at the camera or encoder.";
            info.ToolTip = help;
            _txtVideoUrl.ToolTip = "Absolute RTSP URL or UDP listening port, for example udp://@:5600.";
            _numVideoLatency.ToolTip = "RTSP jitter buffering, not end-to-end video delay. Range: 0–5000 ms.";
            _numVideoQueue.ToolTip = "Decoded-frame queue: 1–120 frames. The display still shows only the latest frame.";
            _chkVideoDrop.ToolTip = "Drop older decode-queue frames and bound RTSP jitter buffering when full.";
            _chkRtspSkipOptions.ToolTip = "For receivers such as EDGE T3 that reset OPTIONS requests. Disables RTSP keep-alives; " +
                "leave off for servers that require them.";
            info.Click += (sender, args) => MessageBox.Show(this, help, "Video receiver help");
            AddAt(tab, info, 20, y, 150, 28);
            y += 40;
            AddVisionSettings(tab, ref y);

            return tab;
        }

        private void AddVisionSettings(TabPage tab, ref int y)
        {
            AddServoGimbalSettings(tab, ref y);
            AddSectionLabel(tab, "Vision and gimbal calibration", ref y);
            AddLabel(tab, "Detection model:", 20, y);
            _cmbVisionModel = AddComboBox(tab, 180, y, 340, Array.Empty<string>());
            System.Windows.Automation.AutomationProperties.SetName(_cmbVisionModel, "Detection model");
            _btnVisionModelBrowse = new Button { Content = "Manage" };
            _btnVisionModelBrowse.Click += (sender, args) =>
            {
                string current = (_cmbVisionModel.SelectedItem as VisionModelChoice)?.Path ?? "";
                try { using (var dialog = new VisionModelsForm(current)) { dialog.ShowDialog(this); } }
                catch (Exception failure) when (failure is System.IO.IOException ||
                    failure is UnauthorizedAccessException || failure is Newtonsoft.Json.JsonException ||
                    failure is FormatException || failure is ArgumentException)
                { _cmbVisionModel.ToolTip = "Model catalog unavailable; current selection is preserved."; }
                LoadModelChoices(current);
            };
            AddAt(tab, _btnVisionModelBrowse, 530, y, 85, 27);
            _cmbVisionModel.ToolTip = "Models are shared by this groundstation. Manage local YOLOX profiles; weights stay in place.";
            y += 32;
            _chkTrackingCalibrated = AddCheckBox(tab, "Camera and mount calibrated", 20, y);
            _chkTrackingCalibrated.ToolTip = "Confirm the camera's field of view, mount travel limits and pitch/yaw directions are calibrated " +
                "before enabling automatic gimbal tracking.";
            y += 32;
            AddLabel(tab, "Horizontal FOV (deg):", 20, y);
            _numVisionFov = AddNumericUpDown(tab, 180, y, 85, 5, 170, 90);
            _numVisionFov.ToolTip = "Rectilinear camera with square pixels and fixed zoom. Vertical FOV is derived from the image aspect ratio. " +
                "Verify pitch/yaw signs and physical limits before enabling tracking.";
            y += 32;
            AddLabel(tab, "Pitch limits (deg):", 20, y);
            _numVisionPitchMin = AddNumericUpDown(tab, 180, y, 75, -90, 90, -90);
            _numVisionPitchMax = AddNumericUpDown(tab, 270, y, 75, -90, 90, 90);
            System.Windows.Automation.AutomationProperties.SetName(_numVisionPitchMin, "Minimum pitch in degrees");
            System.Windows.Automation.AutomationProperties.SetName(_numVisionPitchMax, "Maximum pitch in degrees");
            _cmbVisionPitchSign = AddComboBox(tab, 365, y, 140, new[] { "Normal (+1)", "Reverse (-1)" });
            y += 32;
            AddLabel(tab, "Yaw limits (deg):", 20, y);
            _numVisionYawMin = AddNumericUpDown(tab, 180, y, 75,
                (decimal)GimbalCommand.YAW_MIN_DEG, (decimal)GimbalCommand.YAW_MAX_DEG, -90);
            _numVisionYawMax = AddNumericUpDown(tab, 270, y, 75,
                (decimal)GimbalCommand.YAW_MIN_DEG, (decimal)GimbalCommand.YAW_MAX_DEG, 90);
            System.Windows.Automation.AutomationProperties.SetName(_numVisionYawMin, "Minimum yaw in degrees");
            System.Windows.Automation.AutomationProperties.SetName(_numVisionYawMax, "Maximum yaw in degrees");
            _cmbVisionYawSign = AddComboBox(tab, 365, y, 140, new[] { "Normal (+1)", "Reverse (-1)" });
            y += 32;
            AddLabel(tab, "Tracking rate (deg/s):", 20, y);
            _numVisionRate = AddNumericUpDown(tab, 180, y, 85, 1, 60, 10);
            y += 32;
            AddLabel(tab, "Default tracking:", 20, y);
            _cmbTrackingMode = AddComboBox(tab, 180, y, 140, new[] { "Object", "Area" }); y += 32;
            AddLabel(tab, "Object tracker:", 20, y);
            _cmbObjectTracker = AddComboBox(tab, 180, y, 240, new[] { "Standard (CPU)", "GPU (NVIDIA, EfficientTAM)" });
            System.Windows.Automation.AutomationProperties.SetName(_cmbObjectTracker, "Object tracker");
            _cmbObjectTracker.ToolTip = "GPU follows the whole selected object with EfficientTAM video memory on an NVIDIA DirectX 12 GPU. " +
                "If that GPU or model is unavailable, tracking reports it; it never switches tracker silently.";
            y += 32;
            _chkGroundPrediction = AddCheckBox(tab, "Use estimated ground-plane search hint", 20, y); y += 32;
            AddLabel(tab, "Plane above HOME (m):", 20, y);
            _numGroundPlaneHeight = AddNumericUpDown(tab, 180, y, 100, -10000, 10000, 0);
            _numGroundPlaneHeight.DecimalPlaces = 1; y += 32;
            AddLabel(tab, "Camera delay (ms):", 20, y);
            _numCameraDelay = AddNumericUpDown(tab, 180, y, 100, 0, 2000, 0); y += 32;
            _chkGroundPrediction.ToolTip = "Area search only: assumes a horizontal ground plane at the entered height relative to HOME. " +
                "Leave off for walls, uneven terrain or unknown height. GPS/pose estimates never authorize steering.";
            _numGroundPlaneHeight.ToolTip = "Explicit horizontal surface height relative to aircraft HOME, not height above the camera.";
            _numCameraDelay.ToolTip = "Estimated camera/encoder delay, in addition to receiver jitter buffering. " +
                "Servo tracking reduces feedback gain as this delay increases.";
        }

        private void LoadVideoSettings(VideoInputProfile editing = null)
        {
            _loadingVideo = true;
            try
            {
                _cmbVideoInputs.Items.Clear();
                foreach (var input in Config.VideoInputs) _cmbVideoInputs.Items.Add(input);
                editing = editing ?? Config.SelectedVideoInput;
                _cmbVideoInputs.SelectedItem = editing;
                LoadVideoInput(editing);
            }
            finally { _loadingVideo = false; }
        }

        private void LoadVideoInput(VideoInputProfile input)
        {
            LoadServoGimbalSettings(input);
            _editingVideoInput = input;
            _txtVideoName.Text = input?.Name ?? "";
            _cmbVideoSourceType.SelectedIndex = input?.SourceType == "Camera" ? 1 : 0;
            _txtVideoUrl.Text = input?.Url ?? "";
            LoadVideoDevices(Array.Empty<VideoCaptureDevice>(), input?.CaptureDevicePath ?? "", false);
            _lblVideoDevices.Text = "Refresh to list connected devices.";
            _cmbVideoTransport.SelectedItem = input?.RtspTransport ?? "TCP";
            _numVideoLatency.Value = input?.LatencyMs ?? 100;
            _numVideoQueue.Value = input?.QueueMaxBuffers ?? 1;
            _chkVideoDrop.IsChecked = input?.DropOldFrames ?? true;
            _chkRtspSkipOptions.IsChecked = input?.RtspSkipOptions ?? false;
            LoadModelChoices(input?.DetectorModelProfilePath ?? "");
            _chkTrackingCalibrated.IsChecked = input?.TrackingCalibrated ?? false;
            _cmbTrackingMode.SelectedItem = input?.TrackingMode ?? "Object";
            _cmbObjectTracker.SelectedIndex = input?.ObjectTracker == "GPU" ? 1 : 0;
            _chkGroundPrediction.IsChecked = input?.GroundPredictionEnabled ?? false;
            _numGroundPlaneHeight.Value = (decimal)(input?.GroundPlaneHeightMeters ?? 0);
            _numCameraDelay.Value = input?.CameraDelayEstimateMs ?? 0;
            _numVisionFov.Value = input?.HorizontalFovDeg ?? 90;
            _numVisionPitchMin.Value = input?.PitchMinDeg ?? -90;
            _numVisionPitchMax.Value = input?.PitchMaxDeg ?? 90;
            _numVisionYawMin.Value = input?.YawMinDeg ?? -90;
            _numVisionYawMax.Value = input?.YawMaxDeg ?? 90;
            _numVisionRate.Value = input?.TrackingMaxRateDegSec ?? 10;
            _cmbVisionPitchSign.SelectedIndex = input?.PitchSign == -1 ? 1 : 0;
            _cmbVisionYawSign.SelectedIndex = input?.YawSign == -1 ? 1 : 0;
            foreach (var control in new FrameworkElement[] { _txtVideoName, _txtVideoUrl, _cmbVideoTransport,
                _numVideoLatency, _numVideoQueue, _chkVideoDrop, _chkRtspSkipOptions, _btnRemoveVideo, _cmbVisionModel,
                _chkTrackingCalibrated, _numVisionFov, _numVisionPitchMin, _numVisionPitchMax,
                _numVisionYawMin, _numVisionYawMax, _numVisionRate, _cmbVisionPitchSign, _cmbVisionYawSign,
                _btnVisionModelBrowse, _cmbTrackingMode, _cmbObjectTracker, _chkGroundPrediction, _numGroundPlaneHeight, _numCameraDelay })
            { control.IsEnabled = input != null; }
            _cmbVideoSourceType.IsEnabled = input != null;
            UpdateVideoSourceFields();
        }

        private void UpdateVideoSourceFields()
        {
            bool selected = _editingVideoInput != null;
            bool camera = _cmbVideoSourceType.SelectedIndex == 1;
            _txtVideoUrl.IsEnabled = selected && !camera;
            _cmbVideoTransport.IsEnabled = selected && !camera;
            _numVideoLatency.IsEnabled = selected && !camera;
            _numVideoQueue.IsEnabled = selected && !camera;
            _chkVideoDrop.IsEnabled = selected && !camera;
            _chkRtspSkipOptions.IsEnabled = selected && !camera;
            _cmbVideoDevice.IsEnabled = selected && camera;
            _btnVideoDevices.IsEnabled = selected && camera;
            _lblVideoDevices.Visibility = selected && camera ? Visibility.Visible : Visibility.Collapsed;
        }

        private void LoadVideoDevices(VideoCaptureDevice[] devices, string selectedPath, bool refreshed)
        {
            bool loading = _loadingVideo;
            _loadingVideo = true;
            try
            {
                _cmbVideoDevice.Items.Clear();
                var selected = devices.FirstOrDefault(device =>
                    string.Equals(device.Path, selectedPath, StringComparison.OrdinalIgnoreCase));
                if (selected == null)
                {
                    selected = new VideoCaptureDevice(selectedPath == "" ? "Choose a device" :
                        refreshed ? "Unavailable saved device" : "Saved device (Refresh to check)", selectedPath);
                    _cmbVideoDevice.Items.Add(selected);
                }
                foreach (var device in devices)
                {
                    int count = devices.Count(other => other.Name == device.Name);
                    int index = _cmbVideoDevice.Items.Cast<VideoCaptureDevice>().Count(other =>
                        other.Name == device.Name || other.Name.StartsWith(device.Name + " (", StringComparison.Ordinal));
                    var choice = count > 1 ?
                        new VideoCaptureDevice(device.Name + " (" + (index + 1) + ")", device.Path) : device;
                    _cmbVideoDevice.Items.Add(choice);
                    if (string.Equals(device.Path, selectedPath, StringComparison.OrdinalIgnoreCase)) { selected = choice; }
                }
                _cmbVideoDevice.SelectedItem = selected;
            }
            finally { _loadingVideo = loading; }
        }

        private void RefreshVideoDevices()
        {
            string path = (_cmbVideoDevice.SelectedItem as VideoCaptureDevice)?.Path ?? "";
            try
            {
                var devices = _getVideoDevices();
                LoadVideoDevices(devices, path, true);
                _lblVideoDevices.Text = devices.Length == 0 ? "No supported capture devices found." :
                    path != "" && !devices.Any(device =>
                        string.Equals(device.Path, path, StringComparison.OrdinalIgnoreCase)) ? "Saved device is unavailable." :
                    "Supported modes up to 1080p / 30 FPS.";
            }
            catch (Exception error) when (error is System.ComponentModel.Win32Exception ||
                error is InvalidOperationException || error is FormatException || error is DllNotFoundException ||
                error is EntryPointNotFoundException)
            {
                _lblVideoDevices.Text = "Device list unavailable. Check the driver and Refresh.";
                _btnVideoDevices.ToolTip = error.Message;
            }
        }

        private void LoadModelChoices(string path)
        {
            _cmbVisionModel.Items.Clear();
            var choices = new VisionModelCatalog().SelectionChoices(path, out string error);
            if (error != null) { _cmbVisionModel.ToolTip = error; }
            foreach (var choice in choices) _cmbVisionModel.Items.Add(choice);
            _cmbVisionModel.SelectedItem = choices.Single(item => VisionModelCatalog.SamePath(item.Path, path));
        }

        private void SaveVideoInput()
        {
            if (_editingVideoInput == null) { return; }
            var input = new VideoInputProfile
            {
                Name = _txtVideoName.Text.Trim(), Url = _txtVideoUrl.Text.Trim(),
                SourceType = _cmbVideoSourceType.SelectedIndex == 1 ? "Camera" : "Network",
                CaptureDevicePath = (_cmbVideoDevice.SelectedItem as VideoCaptureDevice)?.Path ?? "",
                RtspTransport = (string)_cmbVideoTransport.SelectedItem,
                LatencyMs = (int)_numVideoLatency.Value, QueueMaxBuffers = (int)_numVideoQueue.Value,
                DropOldFrames = _chkVideoDrop.IsChecked == true,
                RtspSkipOptions = _chkRtspSkipOptions.IsChecked == true,
                DetectorModelProfilePath = (_cmbVisionModel.SelectedItem as VisionModelChoice)?.Path ?? "",
                TrackingCalibrated = _chkTrackingCalibrated.IsChecked == true,
                TrackingMode = (string)_cmbTrackingMode.SelectedItem,
                ObjectTracker = _cmbObjectTracker.SelectedIndex == 1 ? "GPU" : "CPU",
                GroundPredictionEnabled = _chkGroundPrediction.IsChecked == true,
                GroundPlaneHeightMeters = (double)_numGroundPlaneHeight.Value,
                CameraDelayEstimateMs = (int)_numCameraDelay.Value,
                HorizontalFovDeg = (int)_numVisionFov.Value,
                PitchMinDeg = (int)_numVisionPitchMin.Value, PitchMaxDeg = (int)_numVisionPitchMax.Value,
                YawMinDeg = (int)_numVisionYawMin.Value, YawMaxDeg = (int)_numVisionYawMax.Value,
                PitchSign = _cmbVisionPitchSign.SelectedIndex == 1 ? -1 : 1,
                YawSign = _cmbVisionYawSign.SelectedIndex == 1 ? -1 : 1,
                TrackingMaxRateDegSec = (int)_numVisionRate.Value,
                ServoGimbalAxes = _editingGimbalAxes.Select(axis => axis.Copy()).ToList(),
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
