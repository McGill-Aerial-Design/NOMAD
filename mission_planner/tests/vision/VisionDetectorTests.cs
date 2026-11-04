// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;
using Newtonsoft.Json;
using NOMAD.MissionPlanner;

namespace NOMAD.MissionPlanner
{
    internal static class NOMADStorage
    {
        internal static string PluginsDirectory => Path.Combine(
            Environment.GetEnvironmentVariable("NOMAD_VISION_TEST_DATA"), "plugins");
    }
}

internal static class VisionDetectorTests
{
    private static int _checks;
    private static int Main(string[] args)
    {
        if (args.Length == 1 && args[0] == "--gdal-c-runtime") { return GdalCRuntimeFirst(); }
        try
        {
            VisualTrackerTests.Run();
            // The first factory call must work with only embedded ORT resources beside this executable.
            using (var detector = VisionDetectorFactory.Create("", CancellationToken.None))
            { Run(detector, args); }
            Console.WriteLine("PASS: " + _checks + " vision checks.");
            return 0;
        }
        catch (Exception error) { Console.Error.WriteLine(error); return 1; }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Run(IVideoDetector detector, string[] args)
    {
        var profile = VisionModelProfile.Builtin();
        profile.Validate();
        Check(profile.Labels.Length == 80 && profile.Labels[2] == "car" && profile.Rows == 3549,
            "Builtin model has exact COCO labels and output grid.");
        using (var frame = new Bitmap(4, 2))
        {
            using (var graphics = Graphics.FromImage(frame)) { graphics.Clear(Color.FromArgb(30, 20, 10)); }
            float ratio;
            var pixels = YoloXVideoDetector.Preprocess(frame, 4, CancellationToken.None, out ratio);
            Check(ratio == 1 && pixels[0] == 10 && pixels[16] == 20 && pixels[32] == 30,
                "Preprocess preserves BGR planar 0..255 input.");
            Check(pixels[12] == 114 && pixels[28] == 114 && pixels[44] == 114,
                "Letterbox padding remains at bottom with value 114.");
            using (var canceled = new CancellationTokenSource())
            {
                canceled.Cancel();
                Throws<OperationCanceledException>(() => detector.Detect(frame, canceled.Token),
                    "Pre-canceled inference must stop.");
                Throws<OperationCanceledException>(() => VisionDetectorFactory.Create("", canceled.Token),
                    "Pre-canceled model creation must stop.");
            }
            Check(detector.Detect(frame, CancellationToken.None).Count == 0,
                "Actual model rejects a blank color frame.");
        }
        var values = new float[profile.Rows * 85];
        SetBox(values, 0, -1, -1, 2, 2, .9f);
        SetBox(values, 1, -2, -1, 2, 2, .8f);
        var boxes = YoloXVideoDetector.Decode(values, profile, new Size(100, 100), 1, CancellationToken.None);
        Check(boxes.Count == 1 && boxes[0].ClassId == 2 && boxes[0].Confidence > .89f &&
            boxes[0].Bounds.Left == 0 && boxes[0].Bounds.Top == 0 && boxes[0].Bounds.Right < 100,
            "Raw grid decode clips geometry and suppresses overlapping same-class boxes.");
        values[2] = float.PositiveInfinity;
        values[85 + 4] = float.NaN;
        Check(YoloXVideoDetector.Decode(values, profile, new Size(100, 100), 1, CancellationToken.None).Count == 0,
            "Nonfinite output is rejected.");
        Throws<FormatException>(() => YoloXVideoDetector.Decode(new float[1], profile,
            new Size(100, 100), 1, CancellationToken.None), "Wrong output dimensions fail.");
        profile.ColorOrder = "RGB";
        Throws<FormatException>(() => profile.Validate(), "Unsupported input color order fails.");
        profile = VisionModelProfile.Builtin();
        profile.AllowedClasses = new[] { 80 };
        Throws<FormatException>(() => profile.Validate(), "Invalid class filtering fails.");
        profile = VisionModelProfile.Builtin();
        profile.ScoreThreshold = float.NaN;
        Throws<FormatException>(() => profile.Validate(), "Nonfinite profile scores fail.");
        CheckProfiles();
        CheckNativeCancellation();
        if (args.Length == 1)
        {
            using (var frame = new Bitmap(args[0]))
            {
                var timer = System.Diagnostics.Stopwatch.StartNew();
                var detections = detector.Detect(frame, CancellationToken.None);
                Check(detections.Any(detection => detection.ClassId == 2 && detection.Confidence > .5f),
                    "Actual pinned YOLOX model must detect a car in the owned simulator fixture.");
                Check(detections.All(detection => detection.Bounds.Right <= frame.Width &&
                    detection.Bounds.Bottom <= frame.Height), "Actual detector output stays inside original frame.");
                Console.WriteLine("Owned image: " + detections.Count + " vehicles in " + timer.ElapsedMilliseconds + " ms.");
            }
        }
        else { Console.WriteLine("SKIPPED: owned simulator image check (supply -FixtureImage)."); }
        detector.Dispose();
        using (var frame = new Bitmap(4, 4))
        { Throws<ObjectDisposedException>(() => detector.Detect(frame, CancellationToken.None), "Disposed detector refuses work."); }
    }

    private static void CheckProfiles()
    {
        string directory = Path.Combine(Path.GetTempPath(), "NOMAD-vision-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(directory);
        try
        {
            string path = Path.Combine(directory, "model.json");
            var profile = VisionModelProfile.Builtin();
            string validJson = JsonConvert.SerializeObject(profile);
            File.WriteAllText(path, validJson);
            var loaded = VisionModelProfile.Load(path);
            Check(loaded.Model == Path.Combine(directory, "yolox_nano.onnx"), "Model paths resolve beside profile.");
            File.WriteAllText(loaded.Model, "invalid model");
            Throws<FormatException>(() => loaded.ReadModel(CancellationToken.None), "Model hash is checked before runtime parsing.");
            loaded.Sha256 = VisionModelProfile.Hash(File.ReadAllBytes(loaded.Model));
            try
            {
                VisionDetectorFactory.ValidateModel(loaded, CancellationToken.None);
                throw new Exception("Invalid ONNX bytes passed the model editor's real load test.");
            }
            catch (InvalidOperationException error)
            {
                Check(error.Message.Contains("ONNX weights could not load") && !error.Message.Contains(directory),
                    "Model test hid its actionable failure or exposed an operator path.");
            }
            foreach (string invalidJson in new[]
            {
                validJson.Replace("\"SchemaVersion\":1", "\"SchemaVersion\":1,\"Unexpected\":true"),
                validJson.Replace("\"SchemaVersion\":1", "\"SchemaVersion\":2,\"SchemaVersion\":1"),
                validJson.Replace("\"SchemaVersion\":1", "\"SchemaVersion\":\"1\""),
                validJson.Replace("\"SchemaVersion\":1", "\"schemaVersion\":1"),
                validJson.Replace("\"Strides\":[8,16,32]", "\"Strides\":[\"8\",16,32]"),
                validJson.Replace("\"person\"", "2"),
                validJson + "{}"
            })
            {
                File.WriteAllText(path, invalidJson);
                Throws<JsonException>(() => VisionModelProfile.Load(path),
                    "Unknown, duplicate, wrongly typed, mis-cased or trailing profile content must fail.");
            }
            profile.Model = VisionRuntime.BuiltinModelPath;
            profile.InputSize = 320;
            Throws<FormatException>(() => { using (new YoloXVideoDetector(profile, CancellationToken.None)) { } },
                "Actual tensor metadata must match the selected profile.");
            try
            {
                VisionDetectorFactory.ValidateModel(profile, CancellationToken.None);
                throw new Exception("Wrong tensor profile passed the editor model test.");
            }
            catch (InvalidOperationException error)
            { Check(error.Message.Contains("[1,3,320,320]"), "Model test did not identify the expected tensor shape."); }
        }
        finally { Directory.Delete(directory, true); }
    }

    private static void CheckNativeCancellation()
    {
        var model = File.ReadAllBytes(VisionRuntime.BuiltinModelPath);
        using (var options = new SessionOptions { IntraOpNumThreads = 1, InterOpNumThreads = 1 })
        using (var canceled = new CancellationTokenSource())
        using (canceled.Token.Register(() => options.SetLoadCancellationFlag(true)))
        {
            CheckConcurrentNativeCancellation("model load", () =>
            {
                using (new InferenceSession(model, options)) { }
            }, canceled);
        }
        using (var options = new SessionOptions
            { IntraOpNumThreads = 1, InterOpNumThreads = 1, GraphOptimizationLevel = GraphOptimizationLevel.ORT_DISABLE_ALL })
        using (var session = new InferenceSession(model, options))
        using (var runOptions = new RunOptions())
        using (var canceled = new CancellationTokenSource())
        using (canceled.Token.Register(() => runOptions.Terminate = true))
        {
            var tensor = new DenseTensor<float>(new float[3 * 416 * 416], new[] { 1, 3, 416, 416 });
            var inputs = new[] { NamedOnnxValue.CreateFromTensor(session.InputMetadata.Single().Key, tensor) };
            var outputs = new[] { session.OutputMetadata.Single().Key };
            // Warm the actual native call before measuring cancellation, excluding first-call JIT time.
            using (session.Run(inputs)) { }
            CheckConcurrentNativeCancellation("inference", () =>
            {
                using (session.Run(inputs, outputs, runOptions)) { }
            }, canceled);
            using (var recovered = session.Run(inputs))
            { Check(recovered.Single().AsTensor<float>().Length == 3549 * 85, "Native inference recovers after cancellation."); }
        }
    }

    private static void CheckConcurrentNativeCancellation(string operation, Action action, CancellationTokenSource canceled)
    {
        using (var started = new ManualResetEventSlim())
        {
            var elapsed = System.Diagnostics.Stopwatch.StartNew();
            var task = Task.Factory.StartNew(() =>
            {
                started.Set();
                try { action(); return (Exception)null; }
                catch (Exception error) { return error; }
            }, CancellationToken.None, TaskCreationOptions.LongRunning, TaskScheduler.Default);
            Check(started.Wait(5000), "Native " + operation + " worker must start.");
            bool completedBeforeCancel = task.Wait(2);
            long cancelAt = elapsed.ElapsedMilliseconds;
            canceled.Cancel();
            Check(task.Wait(5000), "Native " + operation + " must complete within five seconds after cancellation.");
            Check(!completedBeforeCancel && task.Result is OnnxRuntimeException &&
                (task.Result.Message.IndexOf("cancel", StringComparison.OrdinalIgnoreCase) >= 0 ||
                 task.Result.Message.IndexOf("terminate", StringComparison.OrdinalIgnoreCase) >= 0),
                "Native " + operation + " must be incomplete at cancellation and report native cancellation; " +
                (task.Result?.Message ?? "completed normally"));
            Console.WriteLine("Native " + operation + " canceled at " + cancelAt + " ms; completed at " +
                elapsed.ElapsedMilliseconds + " ms: " + task.Result.Message);
        }
    }

    private static void SetBox(float[] output, int row, float x, float y, float width, float height, float score)
    {
        int offset = row * 85;
        output[offset] = x; output[offset + 1] = y;
        output[offset + 2] = width; output[offset + 3] = height;
        output[offset + 4] = score; output[offset + 7] = 1;
    }

    [System.Runtime.InteropServices.DllImport("kernel32.dll", CharSet = System.Runtime.InteropServices.CharSet.Unicode)]
    private static extern IntPtr LoadLibraryEx(string path, IntPtr file, uint flags);

    // Fresh process: Mission Planner's GDAL loads a 2017 Visual C++ runtime before plugins start. With that copy
    // already loaded by name, ONNX Runtime must still load (in its own activation context) and run inference.
    private static int GdalCRuntimeFirst()
    {
        string gdal = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86),
            "Mission Planner", "gdal", "x64");
        if (!File.Exists(Path.Combine(gdal, "msvcp140.dll")))
        { Console.WriteLine("SKIP: Mission Planner GDAL C runtime not installed."); return 0; }
        foreach (string name in new[] { "vcruntime140.dll", "msvcp140.dll" })
        {
            // LOAD_WITH_ALTERED_SEARCH_PATH, as GDAL resolves its own directory.
            if (LoadLibraryEx(Path.Combine(gdal, name), IntPtr.Zero, 8) == IntPtr.Zero)
            { Console.WriteLine("FAIL: could not load GDAL's " + name); return 1; }
        }
        try
        {
            using (var detector = VisionDetectorFactory.Create("", CancellationToken.None))
            using (var frame = new Bitmap(320, 240))
            { detector.Detect(frame, CancellationToken.None); }
            if (!string.IsNullOrEmpty(Environment.GetEnvironmentVariable("NOMAD_TEST_TAM_MODEL")))
            {
                using (var tracker = VisualTrackerFactory.Create(CancellationToken.None, VisualTrackingMode.Object, true))
                using (var frame = new Bitmap(640, 480))
                using (var graphics = Graphics.FromImage(frame))
                {
                    graphics.Clear(Color.DarkOliveGreen);
                    graphics.FillEllipse(Brushes.SaddleBrown, 280, 200, 80, 60);
                    tracker.Initialize(frame, new RectangleF(270, 190, 100, 80), CancellationToken.None);
                    tracker.Update(frame, CancellationToken.None);
                }
                Console.WriteLine("PASS: GPU Object tracker runs with GDAL's 2017 C runtime already loaded.");
            }
            Console.WriteLine("PASS: ONNX Runtime loads and infers with GDAL's 2017 C runtime already loaded.");
            return 0;
        }
        catch (Exception error)
        {
            Console.WriteLine("FAIL: ONNX Runtime beside GDAL's C runtime: " + error.Message);
            return 1;
        }
    }

    private static void Check(bool condition, string message)
    {
        if (!condition) { throw new Exception(message); }
        _checks++;
    }

    private static void Throws<T>(Action action, string message) where T : Exception
    {
        try { action(); } catch (T) { _checks++; return; }
        throw new Exception(message);
    }
}
