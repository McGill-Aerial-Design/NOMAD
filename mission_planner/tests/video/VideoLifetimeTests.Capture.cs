// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Reflection;
using System.Threading;
using System.Linq;
using System.Windows.Forms;
using Newtonsoft.Json;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static VideoInputProfile CaptureInput(string path) => JsonConvert.DeserializeObject<VideoInputProfile>(
        JsonConvert.SerializeObject(new { Name = "USB capture", SourceType = "Camera", CaptureDevicePath = path }));

    private static void LocalCaptureInputs()
    {
        const string path = @"\\?\usb#synthetic#{6994ad05-93ef-11d0-a3cc-00a0c9223196}";
        var input = CaptureInput(path);
        string pipeline = VideoPipeline.Build(input);
        string hud = (string)typeof(NOMADPlugin).GetMethod("BuildHudVideoPipeline",
            BindingFlags.Static | BindingFlags.NonPublic).Invoke(null, new object[] { input });
        Check(pipeline.StartsWith("ksvideosrc device-path=\"" + path.Replace("\\", "\\\\") + "\" ! ") &&
            pipeline == hud && pipeline.Contains("decodebin3 ! queue") &&
            !pipeline.Contains("rtspsrc") && !pipeline.Contains("udpsrc"),
            "Local capture did not use one exact device and the common HUD/embedded playback path");
        var first = new FakeVideoPipeline();
        var second = new FakeVideoPipeline();
        int created = 0;
        using (var player = new EmbeddedVideoPlayer(input, true, CancellationToken.None,
            () => ++created == 1 ? first : second))
        {
            Check(created == 0, "Configuring a local source opened a pipeline before Play");
            player.StartStream();
            Wait(first.ReadEntered, "explicit local capture Play");
            player.UpdateInput(CaptureInput(path.Replace("synthetic", "replacement")));
            WaitUi(() => second.ReadEntered.IsSet, "local capture device replacement");
            CheckClean(first);
            Check(second.Pipeline.Contains("replacement") && !second.Pipeline.Contains("synthetic"),
                "Changing a capture device retained the previous device source");
            player.StopStream();
            player.UpdateInput(input);
            Check(created == 2, "Editing a stopped local source activated capture");
        }
        CheckClean(second);
    }

    private static void LocalCaptureProfiles()
    {
        const string path = @"\\?\usb#synthetic#video";
        foreach (var invalid in new[] { "", "USB camera", "0", path + "\n", path + "\0", new string('x', 1025), null })
        {
            bool rejected = false;
            try { VideoPipeline.Build(CaptureInput(invalid)); }
            catch (JsonSerializationException) { rejected = true; }
            Check(rejected, "An invalid capture device silently selected a default device");
        }
        var config = new NOMADConfig();
        config.VideoInputs.Add(CaptureInput(path));
        config.SelectedVideoInputName = "USB capture";
        var copy = config.Clone();
        Check(copy.SelectedVideoInput.SourceType == "Camera" && copy.SelectedVideoInput.CaptureDevicePath == path,
            "Capture source identity did not survive strict configuration round trip");
        Check(JsonConvert.DeserializeObject<VideoInputProfile>("{}").SourceType == "Network",
            "Existing video profiles changed their source type");
        copy.SelectedVideoInput.SourceType = "Unknown";
        bool invalidType = false;
        try { copy.SelectedVideoInput.Validate(); } catch (JsonSerializationException) { invalidType = true; }
        Check(invalidType, "An unknown source type fell back to a network or default camera");
        var input = CaptureInput(path + "\" ! fakesink name=unexpected");
        Check(VideoPipeline.Build(input).Contains("\\\" ! fakesink name=unexpected\" ! "),
            "Capture device path escaped its quoted GStreamer property");
        Check(VideoPipeline.Build(input).Contains("width=(int)[1,1920]") &&
            VideoPipeline.Build(input).Contains("framerate=(fraction)[1/1,30/1]"),
            "Automatic capture negotiation was not bounded to 1080p / 30 FPS");
    }

    private static void LocalCaptureSettings()
    {
        const string first = @"\\?\usb#first#video";
        const string second = @"\\?\usb#second#video";
        var profiles = new NOMADProfiles(VideoConfig());
        int queries = 0;
        using (var form = new NOMADSettingsForm(profiles))
        {
            var flags = BindingFlags.Instance | BindingFlags.NonPublic;
            var devices = new[] { new VideoCaptureDevice("USB camera", first), new VideoCaptureDevice("USB camera", second) };
            typeof(NOMADSettingsForm).GetField("_getVideoDevices", flags).SetValue(form,
                new Func<VideoCaptureDevice[]>(() => { queries++; return devices; }));
            var source = GetField<System.Windows.Controls.ComboBox>(form, "_cmbVideoSourceType");
            var picker = GetField<System.Windows.Controls.ComboBox>(form, "_cmbVideoDevice");
            var calibrated = GetField<System.Windows.Controls.CheckBox>(form, "_chkTrackingCalibrated");
            calibrated.IsChecked = true;
            source.SelectedIndex = 1;
            Check(queries == 0 && picker.IsEnabled && !GetField<System.Windows.Controls.TextBox>(form, "_txtVideoUrl").IsEnabled && calibrated.IsChecked != true,
                "Changing source type queried a camera, enabled URL input or retained old calibration");
            Action refresh = () => typeof(NOMADSettingsForm).GetMethod("RefreshVideoDevices", flags).Invoke(form, null);
            refresh();
            Check(queries == 1 && ((VideoCaptureDevice)picker.SelectedItem).Path == "" &&
                picker.Items.Cast<VideoCaptureDevice>().Select(device => device.Name).Distinct().Count() == 3,
                "Refreshing auto-selected a device or left duplicate names indistinguishable");
            picker.SelectedItem = picker.Items.Cast<VideoCaptureDevice>().Single(device => device.Path == second);
            calibrated.IsChecked = true;
            devices = new[] { new VideoCaptureDevice("USB camera", first), new VideoCaptureDevice("USB camera", second.ToUpperInvariant()) };
            refresh();
            Check(string.Equals(((VideoCaptureDevice)picker.SelectedItem).Path, second, StringComparison.OrdinalIgnoreCase) &&
                calibrated.IsChecked == true, "A case-only device path change lost the selected camera or its calibration");
            typeof(NOMADSettingsForm).GetMethod("SaveSettings", flags).Invoke(form, null);
            Check(form.Config.Clone().SelectedVideoInput.SourceType == "Camera" &&
                string.Equals(form.Config.SelectedVideoInput.CaptureDevicePath, second, StringComparison.OrdinalIgnoreCase),
                "Saving the camera definition lost its exact device identity");
            devices = new[] { new VideoCaptureDevice("USB camera", first) };
            refresh();
            Check(((VideoCaptureDevice)picker.SelectedItem).Path == second.ToUpperInvariant() &&
                ((VideoCaptureDevice)picker.SelectedItem).Name.Contains("Unavailable"),
                "An unplugged saved camera was replaced by the first remaining device");
            typeof(NOMADSettingsForm).GetField("_getVideoDevices", flags).SetValue(form,
                new Func<VideoCaptureDevice[]>(() => { throw new System.ComponentModel.Win32Exception(5); }));
            refresh();
            Check(((VideoCaptureDevice)picker.SelectedItem).Path == second.ToUpperInvariant() &&
                GetField<System.Windows.Controls.TextBlock>(form, "_lblVideoDevices").IsEnabled &&
                GetField<System.Windows.Controls.TextBlock>(form, "_lblVideoDevices").Text.Contains("unavailable"),
                "A device enumeration error discarded the saved camera or failed silently");
        }
        Check(profiles.Selected.Config.SelectedVideoInput.SourceType == "Network" &&
            profiles.Selected.Config.SelectedVideoInput.CaptureDevicePath == "",
            "Closing an unaccepted camera settings draft changed the operator profile");
    }
}
