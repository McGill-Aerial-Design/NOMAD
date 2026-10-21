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
        using (var player = new EmbeddedVideoPlayer(input.Name, input, true, CancellationToken.None, () => fake))
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
                    Check(GetField<EmbeddedVideoPlayer>(video, "_videoPlayer") == null &&
                        GetField<EmbeddedVideoPlayer>(dashboard, "_videoPlayer") == null && created == 0,
                        "An empty video configuration invented a source or created a pipeline.");
                }
            }
            finally { NotificationService.Shared = previous; }
        }
    }

    private static void VideoSettingsDrafts()
    {
        var config = VideoConfig();
        config.AircraftName = "Alpha";
        config.VideoInputs.Add(new VideoInputProfile { Name = "Second", Url = "rtsp://second.invalid/live" });
        var profiles = new NOMADProfiles(config);
        string firstId = profiles.SelectedId;
        profiles.Add(new NOMADConfig { AircraftName = "Beta" });
        profiles.SelectedId = firstId;
        using (var form = new NOMADSettingsForm(profiles))
        {
            var selector = GetField<ComboBox>(form, "_cmbVideoInputs");
            GetField<TextBox>(form, "_txtVideoName").Text = "Renamed";
            GetField<TextBox>(form, "_txtVideoUrl").Text = "rtsp://edited.invalid/live";
            GetField<NumericUpDown>(form, "_numVideoLatency").Value = 700;
            selector.SelectedIndex = 1;
            Check(GetField<TextBox>(form, "_txtVideoUrl").Text == "rtsp://second.invalid/live",
                "Switching named video input did not load its independent fields.");
            GetField<NumericUpDown>(form, "_numVideoQueue").Value = 9;
            selector.SelectedIndex = 0;
            Check(GetField<TextBox>(form, "_txtVideoName").Text == "Renamed" &&
                GetField<TextBox>(form, "_txtVideoUrl").Text == "rtsp://edited.invalid/live" &&
                GetField<NumericUpDown>(form, "_numVideoLatency").Value == 700 &&
                form.Config.SelectedVideoInputName == "Renamed" && form.Config.VideoInputs[1].QueueMaxBuffers == 9,
                "Switching video input lost a draft, rename, selection or receiver settings.");
            var aircraft = GetField<ComboBox>(form, "_cmbAircraft");
            aircraft.SelectedIndex = 1;
            Check(selector.Items.Count == 0 && !GetField<TextBox>(form, "_txtVideoUrl").Enabled,
                "Another aircraft inherited the previous aircraft's video inputs.");
            aircraft.SelectedIndex = 0;
            Check(selector.Items.Count == 2 && form.Config.SelectedVideoInputName == "Renamed",
                "Returning to an aircraft lost its video input list or selection.");
            var flags = BindingFlags.Instance | BindingFlags.NonPublic;
            typeof(NOMADSettingsForm).GetMethod("SaveSettings", flags).Invoke(form, null);
            var saved = form.Config.Clone();
            Check(saved.SelectedVideoInput.Name == "Renamed" && saved.SelectedVideoInput.LatencyMs == 700,
                "Saved/clone video input selection or settings were lost.");
            typeof(NOMADSettingsForm).GetMethod("RemoveVideoInput", flags).Invoke(form, null);
            typeof(NOMADSettingsForm).GetMethod("RemoveVideoInput", flags).Invoke(form, null);
            Check(form.Config.VideoInputs.Count == 0 && form.Config.SelectedVideoInputName == "",
                "Removing the final video input did not clear the selection.");
            typeof(NOMADSettingsForm).GetMethod("AddVideoInput", flags).Invoke(form, null);
            GetField<TextBox>(form, "_txtVideoUrl").Text = "rtsp://new.invalid/live";
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
