// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static VideoFrame DetectionFrame(VideoSession session, FakeVideoPipeline pipeline, int width, int height,
        Func<VideoFrame, bool> ready)
    {
        var deadline = Stopwatch.StartNew();
        while (deadline.Elapsed < TimeSpan.FromSeconds(5))
        {
            pipeline.Frames.Add(new Bitmap(width, height));
            var frame = NextVideoFrame(session);
            if (ready(frame)) { return frame; }
            frame.Dispose(); Thread.Sleep(5);
        }
        throw new InvalidOperationException("Timed out waiting for current detection: " + session.VisionError);
    }

    private static void DetectionModelSwap()
    {
        CheckDetectionModelSwap(false);
        CheckDetectionModelSwap(true);
    }

    private static void CheckDetectionModelSwap(bool obsoleteFault)
    {
        var pipeline = new FakeVideoPipeline();
        var entered = new ManualResetEventSlim(); var release = new ManualResetEventSlim();
        var created = new List<string>();
        var first = new FakeDetector(); var last = new FakeDetector();
        int activeLoads = 0, maximumLoads = 0;
        using (var session = new VideoSession(() => pipeline, (path, token) =>
        {
            int concurrent = Interlocked.Increment(ref activeLoads);
            maximumLoads = Math.Max(maximumLoads, concurrent);
            try
            {
                lock (created) { created.Add(path); }
                if (path == "A")
                {
                    entered.Set(); Wait(release, "obsolete model constructor release");
                    if (obsoleteFault) { throw new InvalidOperationException("obsolete-private-model-error"); }
                    return first;
                }
                if (path == "fail") { throw new InvalidOperationException("private-model-path"); }
                return last;
            }
            finally { Interlocked.Decrement(ref activeLoads); }
        }))
        {
            session.ConfigureDetection(true, "A"); session.Start("owned source");
            pipeline.Frames.Add(new Bitmap(20, 10));
            using (var frame = NextVideoFrame(session))
            { Check(frame.Detections.Count == 0 && frame.DetectorIdentity == "", "Loading retained old model boxes"); }
            Wait(entered, "model A constructor");
            long generation = session.Generation;
            session.ConfigureDetection(true, "B"); session.ConfigureDetection(true, "C");
            pipeline.Frames.Add(new Bitmap(30, 15));
            using (var frame = NextVideoFrame(session))
            { Check(frame.Image.Width == 30 && frame.Detections.Count == 0, "Loading blocked plain video"); }
            Check(activeLoads == 1, "A second model constructor ran before the obsolete constructor completed");
            release.Set();
            using (var frame = DetectionFrame(session, pipeline, 40, 20, item => item.Detections.Count != 0))
            { Check(frame.DetectionRevision == session.DetectionRevision, "Obsolete revision escaped to playback"); }
            Check(first.Disposals == (obsoleteFault ? 0 : 1) && session.VisionError == null &&
                maximumLoads == 1 && !created.Contains("B") &&
                created.SequenceEqual(new[] { "A", "C" }) && pipeline.Starts == 1 && session.Generation == generation,
                "Hot switch leaked/duplicated a detector, loaded an intermediate request or restarted the decoder");
            session.ConfigureDetection(true, "fail");
            var previous = last;
            using (var frame = DetectionFrame(session, pipeline, 20, 10, item => session.VisionError != null))
            { Check(frame.Detections.Count == 0 && !session.VisionError.Contains("private-model-path"), "Model failure exposed old boxes or private error"); }
            Check(!session.DetectionEnabled && session.State == VideoState.Streaming && previous.Disposals == 1,
                "Model load failure stopped video or retained the previous model");
            var recovered = new FakeDetector();
            // A fresh detector instance is required after the previous one has been retired.
            last = recovered;
            session.ConfigureDetection(true, "C");
            using (var frame = DetectionFrame(session, pipeline, 20, 10, item => item.Detections.Count != 0))
            { Check(session.VisionError == null, "Explicit retry kept an obsolete model error"); }
            session.Stop(); Wait(session.Completion, "recovered model cleanup");
            Check(recovered.Disposals == 1, "Shutdown retained the recovered model");
        }
        CheckClean(pipeline);
    }

    private static void ModelCatalogAndEditor()
    {
        string directory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "models-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(directory);
        try
        {
            Check(new VisionModelCatalog().DirectoryPath == Path.Combine(NOMADStorage.PluginsDirectory, "NOMAD", "models"),
                "Catalog did not follow isolated host storage");
            var catalog = new VisionModelCatalog(Path.Combine(directory, "catalog"));
            string weights = Path.Combine(directory, "weights.onnx"); File.WriteAllText(weights, "owned fixture");
            var profile = VisionModelProfile.Builtin(); profile.Name = "Owned model"; profile.Model = weights;
            profile.Sha256 = VisionModelEditorForm.HashModel(weights, CancellationToken.None);
            string path = catalog.Save(profile);
            Check(VisionModelProfile.Load(path).Model == weights && catalog.Choices().Count == 2 &&
                !Directory.EnumerateFiles(catalog.DirectoryPath, "*.onnx").Any(), "Catalog copied weights or lost canonical paths");
            File.WriteAllText(Path.Combine(catalog.DirectoryPath, "corrupt.json"), "{}");
            var additional = VisionModelProfile.Load(path); additional.Name = "Another good model";
            string additionalPath = catalog.Save(additional);
            Check(VisionModelProfile.Load(additionalPath).Name == additional.Name,
                "An unrelated corrupt model profile blocked a good catalog save");
            var duplicate = VisionModelProfile.Load(path); duplicate.Name = "owned MODEL";
            bool rejected = false;
            try { catalog.Save(duplicate); } catch (FormatException) { rejected = true; }
            Check(rejected, "Duplicate catalog model name was accepted");
            CheckModelClassEditor(VisionModelProfile.Load(path));
            CheckModelDuplicates(catalog, path);
            CheckModelLabelImport(profile, directory);
            using (var manager = new VisionModelsForm(path, catalog))
            {
                manager.ShowInTaskbar = false; manager.StartPosition = FormStartPosition.Manual;
                manager.Location = new Point(-2000, -2000);
                manager.Show();
                var models = GetField<System.Windows.Controls.ListBox>(manager, "_models");
                WaitUi(() => models.SelectedItem != null && !GetField<bool>(manager, "_preparing"),
                    "asynchronous model manager catalog");
                Check(((VisionModelChoice)models.SelectedItem).Path == path &&
                    GetField<System.Windows.Controls.Button>(manager, "_edit").IsEnabled && GetField<System.Windows.Controls.Button>(manager, "_remove").IsEnabled,
                    "Model manager lost the current managed selection or disabled its actions");
                WaitUi((System.Threading.Tasks.Task)typeof(VisionModelsForm)
                    .GetMethod("RefreshModels", BindingFlags.Instance | BindingFlags.NonPublic)
                    .Invoke(manager, new object[] { null }), "model manager refresh");
                Check(((VisionModelChoice)models.SelectedItem).Path == path, "Catalog refresh reset the selection");
                models.SelectedIndex = 0;
                Check(!GetField<System.Windows.Controls.Button>(manager, "_edit").IsEnabled && !GetField<System.Windows.Controls.Button>(manager, "_export").IsEnabled &&
                    !GetField<System.Windows.Controls.Button>(manager, "_remove").IsEnabled && GetField<System.Windows.Controls.Button>(manager, "_duplicate").IsEnabled,
                    "Built-in model allowed an invalid management action or could not be duplicated");
            }
            string before = File.ReadAllText(path);
            using (var editor = new VisionModelEditorForm(VisionModelProfile.Load(path)))
            {
                GetField<System.Windows.Controls.TextBox>(editor, "_name").Text = "Unsaved draft";
                editor.Close();
                Check(editor.Draft == null && File.ReadAllText(path) == before, "Canceled editor changed a saved model");
            }
            using (var editor = new VisionModelEditorForm(VisionModelProfile.Load(path)))
            {
                GetField<System.Windows.Controls.TextBox>(editor, "_name").Text = "Accepted draft";
                typeof(VisionModelEditorForm).GetMethod("AcceptDraft", BindingFlags.Instance | BindingFlags.NonPublic)
                    .Invoke(editor, null);
                Check(editor.Draft?.Name == "Accepted draft" && File.ReadAllText(path) == before,
                    "Editor acceptance wrote before the catalog save or lost its draft");
            }
            profile.Name = "Renamed model"; catalog.Save(profile, path);
            Check(File.Exists(path + ".bak") && VisionModelProfile.Load(path).Name == "Renamed model",
                "Model edit changed its stable path or lost atomic backup");
            string external = Path.Combine(directory, "external.json");
            File.WriteAllText(external, JsonConvert.SerializeObject(profile));
            Check(catalog.Choices(external).Any(item => item.Path == external), "External current profile was replaced by built-in");
            string externalBefore = File.ReadAllText(external);
            var externalCopy = VisionModelsForm.PrepareDuplicate(new VisionModelChoice(external, profile.Name), CancellationToken.None);
            string externalCopyPath = catalog.Save(externalCopy);
            Check(externalCopyPath != external && externalCopy.Model == weights && File.ReadAllText(external) == externalBefore,
                "Duplicating an external model overwrote its source or changed its weights");
            catalog.Remove(path);
            Check(File.Exists(weights) && catalog.Choices(path).Any(item => item.Path == path),
                "Removing a profile removed weights or silently selected another model");
            var bounded = new VisionModelCatalog(Path.Combine(directory, "bounded"));
            Directory.CreateDirectory(bounded.DirectoryPath);
            for (int index = 0; index < 257; index++)
            { File.WriteAllText(Path.Combine(bounded.DirectoryPath, index + ".json"), "{}"); }
            rejected = false;
            try { bounded.Choices(); } catch (FormatException) { rejected = true; }
            Check(rejected, "An oversized model catalog was scanned without its document bound");
            ReparseModelCatalog(directory, profile);
        }
        finally { Directory.Delete(directory, true); }
    }

    private static void CheckModelDuplicates(VisionModelCatalog catalog, string original)
    {
        string before = File.ReadAllText(original);
        var copy = VisionModelsForm.PrepareDuplicate(new VisionModelChoice(original, "Owned model"), CancellationToken.None);
        string saved = catalog.Save(copy);
        var source = VisionModelProfile.Load(original);
        Check(saved != original && copy.Model == source.Model && copy.Sha256 == source.Sha256 &&
            copy.Labels.SequenceEqual(source.Labels) && copy.AllowedClasses.SequenceEqual(source.AllowedClasses) &&
            File.ReadAllText(original) == before && !Directory.EnumerateFiles(catalog.DirectoryPath, "*.onnx").Any(),
            "Duplicating a managed model changed its definition, overwrote its source or copied weights");
    }

    private static void NativeBuiltinModelDuplication()
    {
        string directory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "builtin-model-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(directory);
        try { CheckBuiltinModelDuplication(new VisionModelCatalog(directory)); }
        finally { Directory.Delete(directory, true); }
    }

    private static void CheckBuiltinModelDuplication(VisionModelCatalog catalog)
    {
        var builtin = new VisionModelChoice("", "Built-in");
        using (var canceled = new CancellationTokenSource())
        {
            canceled.Cancel(); int count = catalog.Choices().Count; bool rejected = false;
            try { VisionModelsForm.PrepareDuplicate(builtin, canceled.Token); }
            catch (OperationCanceledException) { rejected = true; }
            Check(rejected && catalog.Choices().Count == count, "Canceled duplicate preparation created a preset");
        }
        var people = VisionModelsForm.PrepareDuplicate(builtin, CancellationToken.None);
        Check(people.Model == VisionRuntime.BuiltinModelPath && File.Exists(people.Model) &&
            people.Sha256 == VisionModelProfile.BuiltinHash, "Built-in duplicate did not reuse verified cached weights");
        using (var editor = new VisionModelEditorForm(people))
        {
            GetField<System.Windows.Controls.TextBox>(editor, "_name").Text = "People";
            var classes = GetField<System.Windows.Controls.DataGrid>(editor, "_classes");
            for (int index = 0; index < classes.Items.Count; index++)
            { classes.Items[index].GetType().GetProperty("Detect").SetValue(classes.Items[index], index == 0); }
            typeof(VisionModelEditorForm).GetMethod("AcceptDraft", BindingFlags.Instance | BindingFlags.NonPublic)
                .Invoke(editor, null);
            people = editor.Draft;
        }
        var reloaded = VisionModelProfile.Load(catalog.Save(people));
        Check(reloaded.Name == "People" && reloaded.Labels[0] == "person" &&
            reloaded.AllowedClasses.SequenceEqual(new[] { 0 }) && reloaded.Model == VisionRuntime.BuiltinModelPath &&
            VisionModelProfile.Builtin().AllowedClasses.SequenceEqual(new[] { 1, 2, 3, 5, 7 }) &&
            !Directory.EnumerateFiles(catalog.DirectoryPath, "*.onnx").Any(),
            "People preset lost person ID 0, changed built-in vehicle filters or copied weights");
    }

    private static void CheckModelLabelImport(VisionModelProfile profile, string directory)
    {
        string path = Path.Combine(directory, "labels.txt");
        using (var editor = new VisionModelEditorForm(profile))
        {
            var read = typeof(VisionModelEditorForm).GetMethod("ReadDraft", BindingFlags.Instance | BindingFlags.NonPublic);
            File.WriteAllText(path, " person \r\nvehicle\r\n"); editor.ImportLabels(path);
            var imported = (VisionModelProfile)read.Invoke(editor, null);
            Check(imported.Labels.SequenceEqual(new[] { "person", "vehicle" }) &&
                imported.AllowedClasses.SequenceEqual(new[] { 0, 1 }), "Label import changed line order or filter defaults");
            foreach (string invalid in new[] { "", "first\n\nlast", string.Join("\n", Enumerable.Repeat("class", 257)),
                new string('a', 81), "class\tname", new string('a', 65537) })
            {
                File.WriteAllText(path, invalid); bool rejected = false;
                try { editor.ImportLabels(path); } catch (FormatException) { rejected = true; }
                var unchanged = (VisionModelProfile)read.Invoke(editor, null);
                Check(rejected && unchanged.Labels.SequenceEqual(imported.Labels) &&
                    unchanged.AllowedClasses.SequenceEqual(imported.AllowedClasses),
                    "Invalid label import changed the existing canonical class table");
            }
        }
    }

    private static void OversizedCatalogKeepsPlainVideo()
    {
        var catalog = new VisionModelCatalog();
        Directory.CreateDirectory(catalog.DirectoryPath);
        var created = new List<string>();
        string current = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "owned-current-model.json");
        try
        {
            for (int index = 0; index < 257; index++)
            {
                string path = Path.Combine(catalog.DirectoryPath, "owned-overflow-" + index + ".json");
                Check(!File.Exists(path), "Catalog fixture would replace an existing file");
                File.WriteAllText(path, "{}"); created.Add(path);
            }
            var choices = catalog.SelectionChoices(current, out string error);
            Check(error != null && choices.Count == 2 && choices.Any(choice => choice.Path == current),
                "Unavailable catalog silently substituted a different current model");
            var config = VideoConfig(); config.SelectedVideoInput.DetectorModelProfilePath = current;
            var pipeline = new FakeVideoPipeline();
            using (var player = new VideoInputPlayer(config, true, CancellationToken.None, () => pipeline, null,
                createTracker: (token, mode) => new FakeVisualTracker()))
            {
                var models = GetField<System.Windows.Controls.ComboBox>(player, "_models");
                WaitUi(() => models.SelectedItem != null, "unavailable optional model catalog");
                Check(((VisionModelChoice)models.SelectedItem).Path == current,
                    "Oversized catalog tore down the Video selector or changed its default");
                player.Player.StartStream(); Wait(pipeline.ReadEntered, "overflow plain video");
                PlayerVisionFrame(player.Player, pipeline, Color.Blue);
                Check(player.Player.DisplayedVideoFrame.Image.Width == 100 &&
                    player.Player.DisplayedVideoFrame.Detections.Count == 0 &&
                    config.SelectedVideoInput.DetectorModelProfilePath == current && pipeline.Starts == 1,
                    "Optional catalog failure stopped plain frames or replaced the saved model");
                ClickVideoButton(player.Player.VideoControls.ManageModelsButton);
                Check(pipeline.Starts == 1 && config.SelectedVideoInput.DetectorModelProfilePath == current,
                    "Unavailable Manage dialog escaped its optional-feature boundary");
            }
            using (var settings = new NOMADSettingsForm(new NOMADProfiles(config)))
            { Check(((VisionModelChoice)GetField<System.Windows.Controls.ComboBox>(settings, "_cmbVisionModel").SelectedItem).Path == current,
                "Unavailable catalog tore down Settings or replaced its current model path"); }
        }
        finally { foreach (string path in created) { File.Delete(path); } }
    }

    private static void ObservationSelectionModelProvenance()
    {
        string directory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "owned-model-observation-" + Guid.NewGuid().ToString("N"));
        var pipeline = new FakeVideoPipeline();
        string modelA = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "owned-model-A.json");
        string modelB = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "owned-model-B.json");
        var input = new VideoInputProfile { Name = "Provenance", Url = "udp://@:5600", DetectorModelProfilePath = modelA };
        try
        {
            using (var player = new EmbeddedVideoPlayer(input, true, CancellationToken.None, () => pipeline,
                createDetector: (path, token) => new FakeDetector { DetectorName = path == modelA ? "A" : "B" },
                createTracker: (token, mode) => new FakeVisualTracker()))
            {
                player.ObservationDirectory = directory; player.StartStream(); Wait(pipeline.ReadEntered, "provenance source");
                player.EnableDetection(true);
                var timeout = Stopwatch.StartNew();
                while (player.DisplayedVideoFrame?.DetectorIdentity != "A" && timeout.Elapsed < TimeSpan.FromSeconds(5))
                { PlayerVisionFrame(player, pipeline, Color.Red); }
                Check(player.SelectTargetAt(new Point(50, 40), new Size(100, 80)), "Model-A selection failed");
                PlayerVisionFrame(player, pipeline, Color.Red);
                long selectionRevision = GetField<long>(player, "_regionRevision");
                player.UpdateDetectionModel(modelB); timeout.Restart();
                while ((player.DisplayedVideoFrame?.DetectorIdentity != "B" ||
                    player.DisplayedVideoFrame?.Region?.Revision != selectionRevision ||
                    player.DisplayedVideoFrame?.Region?.Found != true) && timeout.Elapsed < TimeSpan.FromSeconds(5))
                { PlayerVisionFrame(player, pipeline, Color.Red); }
                Check(player.SelectedRegion.HasValue && player.DisplayedVideoFrame?.Region?.Revision == selectionRevision,
                    "Model swap lost the selected reference: " + player.TrackingStatus);
                string path = WaitUi(player.SaveObservationAsync(), "model-provenance observation write");
                var metadata = Newtonsoft.Json.Linq.JObject.Parse(File.ReadAllText(Path.ChangeExtension(path, ".json")));
                Check((string)metadata["DetectorIdentity"] == "B" &&
                    (string)metadata["SelectedTarget"]["SelectionDetectorIdentity"] == "A" &&
                    (string)metadata["SelectedTarget"]["OptionalDetectorLabel"] == "car",
                    "A model swap relabeled the frozen selection as recognized by the new detector");
            }
        }
        finally
        {
            CheckClean(pipeline);
            if (Directory.Exists(directory)) { Directory.Delete(directory, true); }
        }
    }

    private static void CheckModelClassEditor(VisionModelProfile profile)
    {
        using (var editor = new VisionModelEditorForm(profile))
        {
            var classes = GetField<System.Windows.Controls.DataGrid>(editor, "_classes");
            Check(classes.Columns[0].IsReadOnly && !classes.CanUserSortColumns,
                "Class IDs were editable or tensor class order could be sorted");
            var read = typeof(VisionModelEditorForm).GetMethod("ReadDraft", BindingFlags.Instance | BindingFlags.NonPublic);
            var unchanged = (VisionModelProfile)read.Invoke(editor, null);
            Check(unchanged.Labels.SequenceEqual(profile.Labels) &&
                unchanged.AllowedClasses.SequenceEqual(profile.AllowedClasses), "Class table lost profile labels or filters");
            string labels = Path.GetTempFileName();
            try { File.WriteAllText(labels, "first\nmiddle\nlast\n"); editor.ImportLabels(labels); }
            finally { File.Delete(labels); }
            classes.Items[1].GetType().GetProperty("Detect").SetValue(classes.Items[1], false);
            classes.SelectedIndex = 1;
            typeof(VisionModelEditorForm).GetMethod("RemoveClass", BindingFlags.Instance | BindingFlags.NonPublic)
                .Invoke(editor, null);
            editor.ShowInTaskbar = false; editor.StartPosition = FormStartPosition.Manual;
            editor.Location = new Point(-2000, -2000); editor.Show(); Application.DoEvents();
            classes.ScrollIntoView(classes.Items[1]); classes.UpdateLayout(); classes.Focus();
            classes.CurrentCell = new System.Windows.Controls.DataGridCellInfo(classes.Items[1], classes.Columns[1]);
            Check(classes.BeginEdit(), "Could not begin a pending class label edit");
            Application.DoEvents();
            var editing = FindVisual<System.Windows.Controls.TextBox>(classes);
            Check(editing != null, "Class label editor was not created"); editing.Text = "renamed";
            var edited = (VisionModelProfile)read.Invoke(editor, null);
            Check(edited.Labels.SequenceEqual(new[] { "first", "renamed" }) &&
                edited.AllowedClasses.SequenceEqual(new[] { 0, 1 }) &&
                (int)classes.Items[1].GetType().GetProperty("Id").GetValue(classes.Items[1]) == 1,
                "Class removal or pending label edit changed the tensor class mapping");
            classes.CurrentCell = new System.Windows.Controls.DataGridCellInfo(classes.Items[1], classes.Columns[2]);
            Check(classes.BeginEdit(), "Could not begin a pending Detect edit");
            Application.DoEvents();
            var detect = FindVisual<System.Windows.Controls.CheckBox>(classes);
            Check(detect != null, "Class Detect editor was not created"); detect.IsChecked = false;
            var filtered = (VisionModelProfile)read.Invoke(editor, null);
            Check(filtered.AllowedClasses.SequenceEqual(new[] { 0 }) && filtered.Labels.Length == 2,
                "Pending Detect edit was lost or removed an unchecked output label");
            var footer = GetField<System.Windows.Controls.Button>(editor, "_save").Parent as System.Windows.Controls.WrapPanel;
            Check(footer != null && System.Windows.Controls.DockPanel.GetDock(footer) == System.Windows.Controls.Dock.Bottom && editor.MinimumSize.Width > 0,
                "Model actions scroll with fields or the editor has no minimum size");
        }
    }

    private static T FindVisual<T>(System.Windows.DependencyObject parent) where T : System.Windows.DependencyObject
    {
        for (int i = 0; i < System.Windows.Media.VisualTreeHelper.GetChildrenCount(parent); i++)
        {
            var child = System.Windows.Media.VisualTreeHelper.GetChild(parent, i);
            if (child is T match && child is System.Windows.FrameworkElement element && element.IsKeyboardFocusWithin) { return match; }
            var nested = FindVisual<T>(child); if (nested != null) { return nested; }
        }
        return null;
    }

    private static void ReparseModelCatalog(string directory, VisionModelProfile profile)
    {
        string target = Path.Combine(directory, "owned-target");
        string junction = Path.Combine(directory, "redirected-catalog");
        Directory.CreateDirectory(target);
        using (var create = Process.Start(new ProcessStartInfo("cmd.exe",
            "/d /c mklink /J \"" + junction + "\" \"" + target + "\"")
            { UseShellExecute = false, CreateNoWindow = true, RedirectStandardOutput = true }))
        { Check(create.WaitForExit(5000) && create.ExitCode == 0, "Could not create owned model junction fixture"); }
        try
        {
            bool rejected = false;
            try { new VisionModelCatalog(junction).Save(profile); } catch (IOException) { rejected = true; }
            Check(rejected && Directory.GetFileSystemEntries(target).Length == 0,
                "Managed model save wrote through a junction");
        }
        finally { Directory.Delete(junction); }
    }

    private static void ModelSwitchKeepsRegionAndSource()
    {
        var pipeline = new FakeVideoPipeline();
        using (var player = new EmbeddedVideoPlayer(new VideoInputProfile { Name = "Owned", Url = "udp://@:5600" },
            true, CancellationToken.None, () => pipeline, createTracker: (token, mode) => new FakeVisualTracker()))
        {
            player.StartStream(); Wait(pipeline.ReadEntered, "model-only source");
            PlayerVisionFrame(player, pipeline, Color.Blue);
            Check(player.SelectRegion(new RectangleF(20, 20, 20, 20)), "Model fixture selection failed");
            PlayerVisionFrame(player, pipeline, Color.Blue);
            var selected = player.SelectedRegion;
            var session = GetField<VideoSession>(player, "_session");
            long generation = session.Generation;
            player.UpdateDetectionModel(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "missing-model.json"));
            Check(player.RegionReady && player.SelectedRegion == selected && pipeline.Starts == 1 &&
                session.Generation == generation, "Model switch retired a generic region or restarted video");
            player.EnableDetection(true);
            var oldFrame = new VideoFrame(new Bitmap(100, 80), new[] { Car() }, 900, generation,
                Stopwatch.GetTimestamp(), DateTime.UtcNow, "old model", detectionRevision: session.DetectionRevision - 1);
            var current = typeof(EmbeddedVideoPlayer).GetMethod("CurrentDetections", BindingFlags.Instance | BindingFlags.NonPublic);
            using (oldFrame)
            { Check(!(bool)current.Invoke(player, new object[] { oldFrame }), "Old model boxes remained selectable/displayable"); }
        }
        CheckClean(pipeline);

        var config = TwoInputs(); bool save = false;
        pipeline = new FakeVideoPipeline();
        string model = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "external-model.json");
        using (var host = new VideoInputPlayer(config, true, CancellationToken.None, () => pipeline, null,
            (owner, name, path) =>
            {
                if (save) { owner.SelectedVideoInput.DetectorModelProfilePath = path; }
                return System.Threading.Tasks.Task.FromResult(save);
            }))
        {
            host.Player.StartStream(); Wait(pipeline.ReadEntered, "model dropdown source");
            var selector = GetField<System.Windows.Controls.ComboBox>(host, "_models");
            WaitUi(() => selector.SelectedItem != null, "initial model dropdown catalog");
            var choice = new VisionModelChoice(model, "External model"); selector.Items.Add(choice);
            selector.SelectedItem = choice;
            Check(((VisionModelChoice)selector.SelectedItem).Path == "" && config.SelectedVideoInput.DetectorModelProfilePath == "",
                "Failed save changed the model dropdown/default");
            save = true; selector.SelectedItem = choice;
            Check(config.SelectedVideoInput.DetectorModelProfilePath == model && pipeline.Starts == 1,
                "Model dropdown lost its canonical path or restarted source");
            var session = GetField<VideoSession>(host.Player, "_session");
            long revision = session.DetectionRevision;
            SetField(host, "_modelPath", null);
            host.RefreshInput();
            Check(session.DetectionRevision > revision && pipeline.Starts == 1,
                "Closing Manage after an edit could not reload the same model path without restarting video");
        }
        CheckClean(pipeline);
    }

    private static void ModelCatalogSharedPath()
    {
        var config = TwoInputs();
        using var host = new VideoInputPlayer(config, true, CancellationToken.None, () => new FakeVideoPipeline(),
            null, (owner, name, path) => Task.FromResult(true));
        var selector = GetField<System.Windows.Controls.ComboBox>(host, "_models");
        WaitUi(() => selector.SelectedItem != null, "initial shared model catalog");
        config.SelectedVideoInputName = "Alternate";
        host.RefreshInput();
        selector.Items.Clear(); selector.IsEnabled = false;
        var load = typeof(VideoInputPlayer).GetMethod("LoadModelChoicesAsync", BindingFlags.Instance | BindingFlags.NonPublic);
        var completion = (Task)load.Invoke(host, new object[] { "", GetField<long>(host, "_modelRevision") });
        WaitUi(completion, "shared-path catalog publication after camera switch");
        Check(selector.IsEnabled && selector.SelectedItem is VisionModelChoice choice && choice.Path == "",
            "Switching cameras with the same model path stranded the pending catalog or disabled its selector.");
    }

    private static void ModelLoadShutdown()
    {
        var entered = new ManualResetEventSlim();
        var pipeline = new FakeVideoPipeline(); var detector = new FakeDetector();
        using (var session = new VideoSession(() => pipeline, (path, token) =>
        {
            entered.Set();
            Check(token.WaitHandle.WaitOne(TimeSpan.FromSeconds(5)), "Shutdown did not cancel model construction");
            return detector;
        }))
        {
            session.ConfigureDetection(true, "held"); session.Start("fake");
            pipeline.Frames.Add(new Bitmap(20, 10));
            Wait(entered, "shutdown model constructor");
            Wait(System.Threading.Tasks.Task.Run(() => session.Stop()), "model constructor shutdown join");
            Wait(session.Completion, "model constructor cleanup completion");
            Check(session.Completion.IsCompleted && detector.Disposals == 1 && session.TakeFrame() == null,
                "Shutdown retained or published the obsolete constructed detector");
        }
        CheckClean(pipeline);
        pipeline = new FakeVideoPipeline();
        using (var session = new VideoSession(() => pipeline, (path, token) => null))
        {
            session.ConfigureDetection(true, "null model"); session.Start("fake");
            using (var frame = DetectionFrame(session, pipeline, 20, 10, item => session.VisionError != null))
            {
                Check(!session.DetectionEnabled && !session.DetectionLoading && frame.Detections.Count == 0 &&
                    session.State == VideoState.Streaming, "Null factory result left loading stuck or stopped plain video");
            }
        }
        CheckClean(pipeline);
    }
}
