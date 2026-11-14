// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Threading;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void PendingPlaybackRestart()
    {
        foreach (bool cancel in new[] { false, true })
        {
            var first = new FakeVideoPipeline { HoldRead = true, LateFrame = new Bitmap(1, 1) };
            var next = new FakeVideoPipeline();
            int created = 0;
            using (var player = new EmbeddedVideoPlayer(new VideoInputProfile { Name = "First", Url = "udp://@:5600" },
                true, CancellationToken.None, () => ++created == 1 ? first : next))
            {
                player.StartStream(); Wait(first.ReadEntered, "pending restart original input");
                var clock = System.Diagnostics.Stopwatch.StartNew();
                player.UpdateInput(new VideoInputProfile { Name = "Intermediate", Url = "rtsp://intermediate.invalid/live" });
                Check(clock.ElapsedMilliseconds < 100 && next.Starts == 0,
                    "Input change waited for held native work or opened a replacement too early");
                if (cancel) { player.StopStream(); }
                else { player.UpdateInput(new VideoInputProfile { Name = "Latest", Url = "rtsp://latest.invalid/live" }); }
                first.ReleaseRead.Set();
                if (cancel)
                {
                    WaitUi(GetField<VideoSession>(player, "_session").Completion, "canceled restart cleanup");
                    Application.DoEvents();
                    Check(next.Starts == 0, "A queued restart survived Stop");
                    player.StartStream();
                }
                WaitUi(() => next.ReadEntered.IsSet, "latest replacement startup");
                Check(next.Starts == 1 && next.Pipeline.Contains(cancel ? "intermediate.invalid" : "latest.invalid"),
                    "A retired input restart replaced the latest requested source");
                CheckClean(first); CheckDisposed(first.LateFrame);
            }
            CheckClean(next);
        }
    }

    private static NOMADConfig TwoInputs()
    {
        var config = VideoConfig();
        config.VideoInputs.Add(new VideoInputProfile
        {
            Name = "Alternate", Url = "rtsp://alternate.invalid/live", RtspTransport = "UDP",
            LatencyMs = 333, QueueMaxBuffers = 7, DropOldFrames = false,
        });
        return config;
    }

    private static void ActiveVideoInput()
    {
        var config = TwoInputs();
        var first = new FakeVideoPipeline();
        var second = new FakeVideoPipeline();
        var fullFirst = new FakeVideoPipeline();
        var fullSecond = new FakeVideoPipeline();
        var previewSources = new Queue<FakeVideoPipeline>(new[] { first, second });
        var fullSources = new Queue<FakeVideoPipeline>(new[] { fullFirst, fullSecond });
        bool save = true;
        Func<NOMADConfig, string, bool> select = (owner, name) =>
        {
            Check(ReferenceEquals(owner, config), "Selector changed another aircraft's config.");
            if (save) { owner.SelectedVideoInputName = name; }
            return save;
        };
        using var preview = new VideoInputPlayer(config, false, CancellationToken.None,
            () => previewSources.Dequeue(), select);
        using var full = new VideoInputPlayer(config, true, CancellationToken.None,
            () => fullSources.Dequeue(), select, createTracker: (token, mode) => new FakeVisualTracker());
        var player = preview.Player;
        var handle = player.Handle;
        Wait(first.ReadEntered, "selected preview source");
        var oldSession = GetField<VideoSession>(player, "_session");
        long oldGeneration = oldSession.Generation;
        var frame = new Bitmap(1, 1);
        oldSession.PublishFrame(oldGeneration, frame);
        Tick(player);
        // Views of one input share its receiver: the full view joins the preview's instead of opening another.
        full.Player.StartStream();
        AwaitCapture(() => CaptureSubscribers(VideoPipeline.ShareKey(config.SelectedVideoInput)) == 2 &&
            GetField<VideoSession>(full.Player, "_session").State == VideoState.Streaming, "full view joins preview receiver");
        Check(fullFirst.Starts == 0, "The full view opened a second receiver for the same input.");
        full.Player.StopStream();
        Check(oldSession.State == VideoState.Streaming && first.Disposals == 0,
            "Stopping the full view closed a receiver the preview still uses.");
        GetField<System.Windows.Controls.ComboBox>(preview, "_selector").SelectedItem = "Alternate";
        WaitUi(() => second.ReadEntered.IsSet, "replacement preview");
        full.RefreshInput();
        Check(ReferenceEquals(preview.Player, player) && player.Handle == handle,
            "Input selection recreated the player/control handle.");
        CheckClean(first);
        CheckDisposed(frame);
        Check(second.Pipeline == VideoPipeline.Build(config.VideoInputs[1]),
            "Replacement lost transport, latency, queue or drop settings.");
        Check(fullSecond.Starts == 0 && GetField<Bitmap>(full.Player, "_displayFrame") == null,
            "A stopped cached player restarted or retained a frame during input selection.");
        var stale = new Bitmap(1, 1);
        oldSession.PublishFrame(oldGeneration, stale);
        CheckDisposed(stale);
        full.Player.StartStream();
        AwaitCapture(() => CaptureSubscribers(VideoPipeline.ShareKey(config.VideoInputs[1])) == 2 &&
            GetField<VideoSession>(full.Player, "_session").State == VideoState.Streaming,
            "restarted full view joins the replacement receiver");
        Check(fullSecond.Starts == 0, "Cached full video retained the old input or opened a second receiver.");
        save = false;
        GetField<System.Windows.Controls.ComboBox>(preview, "_selector").SelectedItem = "Synthetic";
        Check(config.SelectedVideoInputName == "Alternate" &&
            (string)GetField<System.Windows.Controls.ComboBox>(preview, "_selector").SelectedItem == "Alternate" && second.Starts == 1,
            "Failed selection did not restore the selector/current playback.");
        var failure = GetField<System.Windows.Controls.TextBlock>(preview, "_selectionError");
        Check(failure.Visibility == System.Windows.Visibility.Visible && failure.Text.Contains("not changed or saved"),
            "Failed source save silently restored the selection without an immediately visible explanation.");
    }

    private static void ActiveVideoInputSave()
    {
        string directory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "selection-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(directory);
        string path = Path.Combine(directory, "profiles.json");
        typeof(NOMADConfig).Assembly.GetType("NOMAD.MissionPlanner.Log", true)
            .GetField("_logPath", BindingFlags.Static | BindingFlags.NonPublic)
            .SetValue(null, Path.Combine(directory, "nomad.log"));
        var profiles = new NOMADProfiles(TwoInputs());
        var config = profiles.Selected.Config;
        SetField(profiles, "_path", path);
        var plugin = new NOMADPlugin();
        using var shutdown = new CancellationTokenSource();
        SetField(plugin, "_profiles", profiles);
        SetField(plugin, "_config", config);
        SetField(plugin, "_videoShutdown", shutdown);
        var select = typeof(NOMADPlugin).GetMethod("SelectActiveVideoInput", BindingFlags.Instance | BindingFlags.NonPublic);
        var output = typeof(NOMADConfig).Assembly.GetType("NOMAD.MissionPlanner.OutputController", true);
        var generationField = output.GetField("_generation", BindingFlags.Static | BindingFlags.NonPublic);
        long generation = (long)generationField.GetValue(null);
        var first = new FakeVideoPipeline();
        var second = new FakeVideoPipeline();
        var sources = new Queue<FakeVideoPipeline>(new[] { first, second });
        using var hud = new PictureBox();
        using var video = new HudVideoPlayer(hud, frame => hud.Image = frame, () => hud.Image, () => sources.Dequeue());
        SetField(plugin, "_hudVideo", video);
        video.Start(VideoPipeline.Build(config.SelectedVideoInput));
        Wait(first.ReadEntered, "HUD original input");
        Check((bool)select.Invoke(plugin, new object[] { config, "Alternate" }), "Owned input selection was rejected.");
        WaitUi(() => second.ReadEntered.IsSet, "HUD replacement input");
        CheckClean(first);
        Check(config.SelectedVideoInputName == "Alternate" && File.ReadAllText(path).Contains("Alternate") &&
            second.Pipeline == VideoPipeline.Build(config.VideoInputs[1]), "Selected input was not saved/applied to HUD.");
        Check((long)generationField.GetValue(null) == generation && ReferenceEquals(GetField<NOMADConfig>(plugin, "_config"), config),
            "Camera selection replaced the aircraft/output owner.");
        var selectModel = typeof(NOMADPlugin).GetMethod("SelectActiveVideoModel", BindingFlags.Instance | BindingFlags.NonPublic);
        Func<NOMADConfig, string, string, bool> saveModel = (owner, name, model) =>
            WaitUi((System.Threading.Tasks.Task<bool>)selectModel.Invoke(plugin, new object[] { owner, name, model }),
                "model selection persistence");
        string modelPath = Path.Combine(directory, "external-model.json");
        Check(saveModel(config, "Alternate", modelPath) &&
            config.SelectedVideoInput.DetectorModelProfilePath == modelPath &&
            File.ReadAllText(path).Contains("external-model.json") && second.Starts == 1 &&
            (long)generationField.GetValue(null) == generation,
            "Model default was not persisted or restarted aircraft/video ownership");
        Check(!saveModel(config, "Synthetic", "") && !saveModel(TwoInputs(), "Alternate", ""),
            "Stale aircraft/input selected a model");
        string blocked = Path.Combine(directory, "blocked");
        File.WriteAllText(blocked, "owned test blocker");
        SetField(profiles, "_path", Path.Combine(blocked, "profiles.json"));
        Check(!saveModel(config, "Alternate", "") &&
            config.SelectedVideoInput.DetectorModelProfilePath == modelPath && second.Starts == 1,
            "Failed model persistence changed the selected default or source");
        Check(!(bool)select.Invoke(plugin, new object[] { config, "Synthetic" }) &&
            config.SelectedVideoInputName == "Alternate" && second.Starts == 1,
            "Failed profile persistence changed the active source/HUD.");
        Check(!(bool)select.Invoke(plugin, new object[] { TwoInputs(), "Synthetic" }), "A stale profile selected video.");
        SetField(plugin, "_savingSettings", true);
        Check(!saveModel(config, "Alternate", ""),
            "Settings draft allowed a concurrent model selection");
        Check(!(bool)select.Invoke(plugin, new object[] { config, "Synthetic" }), "Settings draft allowed a concurrent selection.");
        SetField(plugin, "_savingSettings", false);
        shutdown.Cancel();
        Check(!saveModel(config, "Alternate", ""),
            "Retired session selected a model");
        Check(!(bool)select.Invoke(plugin, new object[] { config, "Synthetic" }), "A retired session selected video.");
    }

    private static void VideoDefinitionSelection()
    {
        using var form = new NOMADSettingsForm(new NOMADProfiles(TwoInputs()));
        var selector = GetField<System.Windows.Controls.ComboBox>(form, "_cmbVideoInputs");
        var save = typeof(NOMADSettingsForm).GetMethod("SaveSettings", BindingFlags.Instance | BindingFlags.NonPublic);
        var remove = typeof(NOMADSettingsForm).GetMethod("RemoveVideoInput", BindingFlags.Instance | BindingFlags.NonPublic);
        selector.SelectedIndex = 1;
        GetField<System.Windows.Controls.TextBox>(form, "_txtVideoName").Text = "Renamed inactive";
        save.Invoke(form, null);
        Check(form.Config.SelectedVideoInputName == "Synthetic", "Editing an inactive definition selected it.");
        remove.Invoke(form, null);
        Check(form.Config.SelectedVideoInputName == "Synthetic", "Removing an inactive definition selected another input.");
        GetField<System.Windows.Controls.TextBox>(form, "_txtVideoName").Text = "Renamed active";
        save.Invoke(form, null);
        Check(form.Config.SelectedVideoInputName == "Renamed active", "Renaming the active definition lost its selection.");
        typeof(NOMADSettingsForm).GetMethod("AddVideoInput", BindingFlags.Instance | BindingFlags.NonPublic).Invoke(form, null);
        GetField<System.Windows.Controls.TextBox>(form, "_txtVideoUrl").Text = "rtsp://new.invalid/live";
        save.Invoke(form, null);
        Check(form.Config.SelectedVideoInputName == "Renamed active", "Adding another definition changed the active input.");
        selector.SelectedIndex = 0;
        remove.Invoke(form, null);
        Check(form.Config.SelectedVideoInputName == form.Config.VideoInputs[0].Name,
            "Removing the active definition did not select the first remaining input.");
    }
}
