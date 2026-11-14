// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Reflection;
using System.Threading;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static NOMADConfig VideoConfig() => new NOMADConfig
    {
        VideoInputs = new List<VideoInputProfile>
            { new VideoInputProfile { Name = "Synthetic", Url = "rtsp://video.invalid/stream" } },
        SelectedVideoInputName = "Synthetic",
    };

    private static void VideoProfileOptions()
    {
        var input = new VideoInputProfile
        {
            Name = "Alternate", Url = "rtsp://video.invalid/alternate", RtspTransport = "UDP",
            LatencyMs = 375, QueueMaxBuffers = 7, DropOldFrames = false,
        };
        string hud = (string)typeof(NOMADPlugin).GetMethod("BuildHudVideoPipeline",
            BindingFlags.Static | BindingFlags.NonPublic).Invoke(null, new object[] { input });
        var fake = new FakeVideoPipeline();
        using (var player = new EmbeddedVideoPlayer(input, true, CancellationToken.None, () => fake))
        {
            player.StartStream();
            Wait(fake.ReadEntered, "alternate receiver profile");
            Check(fake.Pipeline == hud && hud.Contains("protocols=udp latency=375 drop-on-latency=false") &&
                hud.Contains("max-size-buffers=7 max-size-bytes=0 max-size-time=0 leaky=no") &&
                hud.Contains("application/x-rtp,media=video ! decodebin3") && !hud.Contains("encoding-name=H264"),
                "HUD and embedded receiver did not honor profile options or RTSP video codec negotiation.");
        }
        CheckClean(fake);
        input.DropOldFrames = true;
        input.LatencyMs = 0;
        input.QueueMaxBuffers = 120;
        string pipeline = VideoPipeline.Build(input);
        Check(pipeline.Contains("latency=0 drop-on-latency=true") && pipeline.Contains("max-size-buffers=120") &&
            pipeline.Contains("leaky=downstream"), "Receiver option boundaries or old-frame drop mode were lost.");
        Check(new VideoInputProfile().ObjectTracker == "CPU", "Existing profiles did not default to the CPU Object tracker.");
        input.ObjectTracker = "GPU";
        input.Validate();
        input.ObjectTracker = "Auto";
        try { input.Validate(); Check(false, "An unknown Object tracker setting was accepted."); }
        catch (Newtonsoft.Json.JsonSerializationException) { }
    }

    private static void EmptyVideoDoesNotStart()
    {
        int created = 0;
        using (var shutdown = new CancellationTokenSource())
        using (var service = new NotificationService())
        {
            var previous = NotificationService.Shared;
            NotificationService.Shared = service;
            try
            {
                using (var video = new NOMADVideoView(new NOMADConfig(), shutdown.Token,
                    () => { created++; return new FakeVideoPipeline(); }))
                using (var dashboard = new NOMADDashboardView(new NOMADConfig(), null, shutdown.Token,
                    () => { created++; return new FakeVideoPipeline(); }))
                {
                    Check(GetField<VideoInputPlayer>(video, "_videoHost").Player == null &&
                        GetField<VideoInputPlayer>(dashboard, "_videoHost").Player == null && created == 0,
                        "An empty video configuration invented a source or created a pipeline.");
                    var payloads = GetField<ActuatorControlPanel>(video, "_actuatorPanel");
                    payloads.Size = new Size(200, 250);
                    payloads.PerformLayout();
                    var status = GetField<System.Windows.Controls.TextBlock>(payloads, "_status");
                    Check(status.Text == "" && status.Visibility == System.Windows.Visibility.Collapsed,
                        "Idle payload controls displayed a permanent warning paragraph.");
                    status.Text = "Safe not confirmed. Check payload wiring and the connection before retrying.";
                    status.Visibility = System.Windows.Visibility.Visible;
                    status.Measure(new System.Windows.Size(200, double.PositiveInfinity));
                    Check(status.DesiredSize.Width <= 200 && status.DesiredSize.Height > status.FontSize,
                        "Payload action status did not wrap inside a narrow video-side panel.");
                }
            }
            finally { NotificationService.Shared = previous; }
        }
    }

    private static void VideoSettingsDrafts()
    {
        var config = VideoConfig();
        config.AircraftName = "Alpha";
        config.SelectedVideoInput.TrackingCalibrated = true;
        config.VideoInputs.Add(new VideoInputProfile { Name = "Second", Url = "rtsp://second.invalid/live" });
        var profiles = new NOMADProfiles(config);
        string firstId = profiles.SelectedId;
        profiles.Add(new NOMADConfig { AircraftName = "Beta" });
        profiles.SelectedId = firstId;
        using (var form = new NOMADSettingsForm(profiles))
        {
            var tabs = GetField<System.Windows.Controls.TabControl>(form, "_tabControl");
            Check(string.Join(",", System.Linq.Enumerable.Select(
                System.Linq.Enumerable.Cast<System.Windows.Controls.TabItem>(tabs.Items), tab => tab.Header)) ==
                "Aircraft,Links,Video,Payloads,Inputs,Preferences", "Settings lost its six focused sections.");
            Check(GetField<System.Windows.Controls.TextBlock>(form, "_profileContext").Text == "Editing: Alpha",
                "Settings did not identify the aircraft being edited.");
            var latency = GetField<WpfNumberBox>(form, "_numVideoLatency");
            latency.Text = "invalid";
            GetField<System.Windows.Controls.TextBox>(form, "_txtAircraftName").Text = "Unsaved invalid draft";
            bool rejected = false;
            try { typeof(NOMADSettingsForm).GetMethod("SaveSettings", BindingFlags.Instance | BindingFlags.NonPublic).Invoke(form, null); }
            catch (TargetInvocationException error) when (error.InnerException is FormatException) { rejected = true; }
            Check(rejected && form.Config.AircraftName == "Alpha", "Invalid numeric input published part of the settings draft.");
            latency.Value = 100;
            var height = GetField<WpfNumberBox>(form, "_numGroundPlaneHeight");
            height.Value = 12.55m;
            Check(height.Value == 12.55m, "Loading a valid ground height silently lost precision.");
            height.Text = "12.56"; rejected = false;
            try { _ = height.Value; } catch (FormatException) { rejected = true; }
            Check(rejected, "Typed ground height bypassed its decimal precision limit.");
            height.Value = 0;
            GetField<System.Windows.Controls.TextBox>(form, "_txtAircraftName").Text = "Renamed Alpha";
            Check(GetField<System.Windows.Controls.TextBlock>(form, "_profileContext").Text == "Editing: Renamed Alpha",
                "The persistent Settings context did not track an aircraft rename.");
            var selector = GetField<System.Windows.Controls.ComboBox>(form, "_cmbVideoInputs");
            var calibrated = GetField<System.Windows.Controls.CheckBox>(form, "_chkTrackingCalibrated");
            Check(calibrated.IsChecked == true, "Loading a saved video input lost its calibration confirmation.");
            GetField<System.Windows.Controls.TextBox>(form, "_txtVideoName").Text = "Renamed";
            GetField<System.Windows.Controls.TextBox>(form, "_txtVideoUrl").Text = "rtsp://edited.invalid/live";
            Check(calibrated.IsChecked != true, "A replacement network camera inherited the previous calibration confirmation.");
            typeof(NOMADSettingsForm).GetMethod("SaveVideoInput", BindingFlags.Instance | BindingFlags.NonPublic)
                .Invoke(form, null);
            Check(!form.Config.SelectedVideoInput.TrackingCalibrated && profiles.Selected.Config.SelectedVideoInput.TrackingCalibrated,
                "Saving the changed camera retained its calibration or changed the active profile before acceptance.");
            GetField<NOMAD.MissionPlanner.WpfNumberBox>(form, "_numVideoLatency").Value = 700;
            GetField<System.Windows.Controls.CheckBox>(form, "_chkRtspSkipOptions").IsChecked = true;
            GetField<NOMAD.MissionPlanner.WpfNumberBox>(form, "_numVisionFov").Value = 110;
            GetField<NOMAD.MissionPlanner.WpfNumberBox>(form, "_numVisionRate").Value = 60;
            GetField<NOMAD.MissionPlanner.WpfNumberBox>(form, "_numVisionYawMin").Value = -170;
            GetField<NOMAD.MissionPlanner.WpfNumberBox>(form, "_numVisionYawMax").Value = 170;
            GetField<System.Windows.Controls.CheckBox>(form, "_chkTrackingCalibrated").IsChecked = true;
            GetField<System.Windows.Controls.ComboBox>(form, "_cmbTrackingMode").SelectedItem = "Area";
            GetField<System.Windows.Controls.ComboBox>(form, "_cmbObjectTracker").SelectedIndex = 1;
            GetField<System.Windows.Controls.CheckBox>(form, "_chkGroundPrediction").IsChecked = true;
            GetField<NOMAD.MissionPlanner.WpfNumberBox>(form, "_numGroundPlaneHeight").Value = 12.5m;
            GetField<NOMAD.MissionPlanner.WpfNumberBox>(form, "_numCameraDelay").Value = 180;
            selector.SelectedIndex = 1;
            Check(GetField<System.Windows.Controls.TextBox>(form, "_txtVideoUrl").Text == "rtsp://second.invalid/live",
                "Switching named video input did not load its independent fields.");
            Check(GetField<System.Windows.Controls.CheckBox>(form, "_chkRtspSkipOptions").IsChecked != true,
                "RTSP compatibility leaked to another input.");
            GetField<NOMAD.MissionPlanner.WpfNumberBox>(form, "_numVideoQueue").Value = 9;
            selector.SelectedIndex = 0;
            Check(GetField<System.Windows.Controls.TextBox>(form, "_txtVideoName").Text == "Renamed" &&
                GetField<System.Windows.Controls.CheckBox>(form, "_chkRtspSkipOptions").IsChecked == true &&
                GetField<NOMAD.MissionPlanner.WpfNumberBox>(form, "_numVisionFov").Value == 110 &&
                GetField<NOMAD.MissionPlanner.WpfNumberBox>(form, "_numVisionRate").Value == 60 &&
                GetField<System.Windows.Controls.CheckBox>(form, "_chkTrackingCalibrated").IsChecked == true &&
                GetField<System.Windows.Controls.TextBox>(form, "_txtVideoUrl").Text == "rtsp://edited.invalid/live" &&
                GetField<NOMAD.MissionPlanner.WpfNumberBox>(form, "_numVideoLatency").Value == 700 &&
                form.Config.SelectedVideoInputName == "Renamed" && form.Config.VideoInputs[1].QueueMaxBuffers == 9,
                "Switching video input lost a draft, rename, selection or receiver settings.");
            var aircraft = GetField<System.Windows.Controls.ComboBox>(form, "_cmbAircraft");
            aircraft.SelectedIndex = 1;
            Check(GetField<System.Windows.Controls.TextBlock>(form, "_profileContext").Text == "Editing: Beta",
                "The persistent Settings context retained another aircraft's name.");
            Check(selector.Items.Count == 0 && !GetField<System.Windows.Controls.TextBox>(form, "_txtVideoUrl").IsEnabled,
                "Another aircraft inherited the previous aircraft's video inputs.");
            aircraft.SelectedIndex = 0;
            Check(GetField<System.Windows.Controls.TextBlock>(form, "_profileContext").Text == "Editing: Renamed Alpha",
                "Returning to an aircraft lost its edited name in the persistent context.");
            Check(selector.Items.Count == 2 && form.Config.SelectedVideoInputName == "Renamed",
                "Returning to an aircraft lost its video input list or selection.");
            var flags = BindingFlags.Instance | BindingFlags.NonPublic;
            typeof(NOMADSettingsForm).GetMethod("SaveSettings", flags).Invoke(form, null);
            var saved = form.Config.Clone();
            Check(saved.SelectedVideoInput.Name == "Renamed" && saved.SelectedVideoInput.LatencyMs == 700 &&
                saved.SelectedVideoInput.RtspSkipOptions &&
                saved.SelectedVideoInput.TrackingCalibrated && saved.SelectedVideoInput.HorizontalFovDeg == 110 &&
                saved.SelectedVideoInput.TrackingMaxRateDegSec == 60 &&
                saved.SelectedVideoInput.TrackingMode == "Area" && saved.SelectedVideoInput.ObjectTracker == "GPU" &&
                saved.SelectedVideoInput.GroundPredictionEnabled &&
                saved.SelectedVideoInput.GroundPlaneHeightMeters == 12.5 && saved.SelectedVideoInput.CameraDelayEstimateMs == 180 &&
                saved.SelectedVideoInput.YawMinDeg == -170 && saved.SelectedVideoInput.YawMaxDeg == 170,
                "Saved/clone video input selection or settings were lost.");
            typeof(NOMADSettingsForm).GetMethod("RemoveVideoInput", flags).Invoke(form, null);
            typeof(NOMADSettingsForm).GetMethod("RemoveVideoInput", flags).Invoke(form, null);
            Check(form.Config.VideoInputs.Count == 0 && form.Config.SelectedVideoInputName == "",
                "Removing the final video input did not clear the selection.");
            typeof(NOMADSettingsForm).GetMethod("AddVideoInput", flags).Invoke(form, null);
            GetField<System.Windows.Controls.TextBox>(form, "_txtVideoUrl").Text = "rtsp://new.invalid/live";
            typeof(NOMADSettingsForm).GetMethod("SaveSettings", flags).Invoke(form, null);
            Check(form.Config.VideoInputs.Count == 1 && form.Config.SelectedVideoInput != null,
                "Adding and saving a video input did not select it.");
        }
        Check(profiles.Selected.Config.SelectedVideoInputName == "Synthetic" &&
            profiles.Selected.Config.VideoInputs[0].Url == "rtsp://video.invalid/stream" &&
            profiles.Aircraft[1].Config.VideoInputs.Count == 0,
            "Closing unaccepted Settings changed active aircraft inputs.");
    }
}
