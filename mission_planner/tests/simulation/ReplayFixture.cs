// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Threading;
using System.Web.Script.Serialization;
using NOMAD.MissionPlanner;
using Newtonsoft.Json.Linq;
using System.Windows.Forms;

namespace NOMAD.Simulation
{
    internal static class ReplayFixture
    {
        internal static void Run(string run)
        {
            var json = new JavaScriptSerializer();
            var options = JObject.Parse(File.ReadAllText(Path.Combine(run, "replay.json")));
            string dataset = Path.GetFullPath((string)options["dataset"]);
            string marker = Path.GetFileName(dataset).Substring(4);
            Guid id;
            if (!Guid.TryParseExact(marker, "N", out id) ||
                File.ReadAllText(Path.Combine(dataset, "run.id")).Trim() != marker)
            { throw new InvalidDataException("Replay dataset ownership mismatch."); }
            string folder = (string)options["folder"];
            if (folder != "frames" && folder != "decoded") { throw new InvalidDataException("Unknown replay pixel source."); }
            int first = (int)options["first"];
            string modeName = (string)options["mode"] ?? "Area";
            if (!Enum.TryParse(modeName, out VisualTrackingMode mode) || (modeName != "Area" && modeName != "Object"))
            { throw new InvalidDataException("Replay mode must be Area or Object."); }
            string truthPath = Path.Combine(dataset, "replay-truth.json");
            var truth = File.Exists(truthPath) ? JObject.Parse(File.ReadAllText(truthPath)) : null;
            if (truth != null && first != 0) { throw new InvalidDataException("Public replay must start at its original frame."); }
            var receipts = new Dictionary<string, JObject>(StringComparer.Ordinal);
            // Complete decoded sets replay their original receipt clocks so gaps fence the tracker as they did live.
            bool clocked = folder == "decoded" && first == 0 && truth == null;
            if (clocked)
            {
                foreach (string line in File.ReadLines(Path.Combine(dataset, "video-frames.jsonl")))
                {
                    var row = JObject.Parse(line);
                    receipts.Add("decoded_" + ((long)row["sequence"]).ToString("D8") + ".png", row);
                }
            }
            var files = Directory.GetFiles(Path.Combine(dataset, folder), "*.png")
                .Where(path => !path.EndsWith(".mask.png", StringComparison.OrdinalIgnoreCase))
                .OrderBy(path => path, StringComparer.Ordinal).Skip(first).ToArray();
            if (files.Length > 2000) { throw new InvalidDataException("Replay exceeds its bounded frame count."); }
            if (files.Length < 6) { throw new InvalidDataException("Replay needs initialization and five later frames."); }
            if (truth != null && files.Length != ((JObject)truth["frames"]).Count)
            { throw new InvalidDataException("Public replay frame/annotation count mismatch."); }
            if (truth != null)
            {
                foreach (var path in files)
                {
                    var expected = truth["frames"][Path.GetFileName(path)];
                    if (expected == null || Hash(path) != (string)expected["sha256"])
                    { throw new InvalidDataException("Public frame name/checksum changed."); }
                }
            }
            int eligible = 0, verified = 0, wrong = 0;
            var selection = truth == null ? new RectangleF(360, 180, 144, 144) :
                new RectangleF((float)truth["selection"][0], (float)truth["selection"][1],
                    (float)truth["selection"][2], (float)truth["selection"][3]);
            if (options["selection"] is JArray selected)
            {
                if (selected.Count != 4) { throw new InvalidDataException("Replay selection needs x,y,width,height."); }
                selection = new RectangleF((float)selected[0], (float)selected[1], (float)selected[2], (float)selected[3]);
            }
            var errors = new List<double>();
            var durations = new List<double>();
            string templateHash = Hash(files[0]);
            bool visible = Environment.GetEnvironmentVariable("NOMAD_SIMULATION_VISIBLE") == "1";
            string trackerName = null;
            using (var form = new Form { Text = "NOMAD actual " + mode + " tracker - recorded pixels, no aircraft commands",
                ClientSize = new Size(800, 630), StartPosition = FormStartPosition.CenterScreen })
            using (var picture = new PictureBox { Dock = DockStyle.Fill, SizeMode = PictureBoxSizeMode.Zoom })
            using (var status = new Label { Dock = DockStyle.Bottom, Height = 30 })
            using (var tracker = VisualTrackerFactory.Create(CancellationToken.None, mode,
                Environment.GetEnvironmentVariable("NOMAD_SIMULATION_OBJECT_TRACKER") == "GPU"))
            using (var reference = new Bitmap(files[0]))
            using (var output = new StreamWriter(Path.Combine(run, "replay-frames.jsonl")))
            {
                if (visible) { form.Controls.Add(picture); form.Controls.Add(status); form.Show(); }
                trackerName = tracker.GetType().Name;
                tracker.Initialize(reference, selection, CancellationToken.None);
                long previousReceipt = clocked ? (long)receipts[Path.GetFileName(files[0])]["decoded_received_ticks"] : 0;
                try
                {
                for (int index = 1; index < files.Length; index++)
                {
                    using (var image = new Bitmap(files[index]))
                    {
                        var source = clocked ? receipts[Path.GetFileName(files[index])] : null;
                        long received = source == null ? 0 : (long)source["decoded_received_ticks"];
                        long frequency = source == null ? 0 : (long)source["frequency"];
                        if (source != null && frequency <= 0) { throw new InvalidDataException("Invalid source clock."); }
                        bool gap = source != null && (received <= previousReceipt ||
                            (received - previousReceipt) / (double)frequency >= .5);
                        previousReceipt = received;
                        var elapsed = System.Diagnostics.Stopwatch.StartNew();
                        var result = tracker.Update(image, CancellationToken.None, new VisualFrameContext(null, gap));
                        elapsed.Stop();
                        durations.Add(elapsed.Elapsed.TotalMilliseconds);
                        if (elapsed.Elapsed.TotalSeconds >= 1) { throw new TimeoutException("Replay update exceeded one second."); }
                        double? error = null;
                        var expected = truth?["frames"]?[Path.GetFileName(files[index])];
                        if (expected != null)
                        {
                            if (result.Found)
                            {
                                error = Math.Sqrt(Math.Pow(result.AimPoint.Value.X - (double)expected["aim"][0], 2) +
                                    Math.Pow(result.AimPoint.Value.Y - (double)expected["aim"][1], 2));
                                errors.Add(error.Value); if (error > 5) { wrong++; }
                            }
                        }
                        if (index >= 3) { eligible++; if (result.Found) { verified++; } }
                        if (visible)
                        {
                            if (form.IsDisposed) { throw new OperationCanceledException("Replay window closed before completion."); }
                            var overlay = (Bitmap)image.Clone();
                            using (var graphics = Graphics.FromImage(overlay))
                            {
                                if (result.Bounds.HasValue)
                                {
                                    var box = result.Bounds.Value;
                                    graphics.DrawRectangle(Pens.Lime, box.X, box.Y, box.Width, box.Height);
                                }
                                if (expected != null)
                                { graphics.DrawEllipse(Pens.Gold, (float)expected["aim"][0] - 4,
                                    (float)expected["aim"][1] - 4, 8, 8); }
                                if (result.AimPoint.HasValue)
                                { graphics.FillEllipse(Brushes.Lime, result.AimPoint.Value.X - 2,
                                    result.AimPoint.Value.Y - 2, 4, 4); }
                            }
                            var previous = picture.Image; picture.Image = overlay; previous?.Dispose();
                            status.Text = index + "/" + (files.Length - 1) + "  " + result.State +
                                "  green=tracker, gold=independent annotated point  " +
                                (error.HasValue ? error.Value.ToString("F2") + " px" : "");
                            Application.DoEvents();
                        }
                        var area = tracker as AreaVisualTracker;
                        var target = tracker as CsrtVisualTracker;
                        output.WriteLine(json.Serialize(new { frame = Path.GetFileName(files[index]),
                            sha256 = Hash(files[index]), source_received_ticks = received, frequency, discontinuous = gap,
                            state = result.State.ToString(), result.Status, aim_x = result.AimPoint?.X,
                            aim_y = result.AimPoint?.Y, fits = area?.LastHomographyFits,
                            bounds = result.Bounds.HasValue ? new[] { result.Bounds.Value.X, result.Bounds.Value.Y,
                                result.Bounds.Value.Width, result.Bounds.Value.Height } : null,
                            learned_views = area?.LearnedViews ?? target?.LearnedViews, error_px = error,
                            candidate = area?.LastCandidateSource,
                            update_ms = elapsed.Elapsed.TotalMilliseconds,
                            diagnostic = (object)area?.LastMatchDiagnostic ?? target?.LastObjectDiagnostic ??
                                (tracker as EfficientTamVisualTracker)?.LastDiagnostic }));
                    }
                }
                }
                finally { picture.Image?.Dispose(); picture.Image = null; }
            }
            bool passed = verified >= eligible * .95 && wrong == 0;
            File.WriteAllText(Path.Combine(run, "replay-result.json"), json.Serialize(new {
                passed, eligible, verified, wrong, source = folder, mode = modeName, no_reseeding = true,
                tracker = trackerName,
                template_frame = Path.GetFileName(files[0]), template_sha256 = templateHash,
                pixel_accuracy = "rendered replay needs separate semantic scoring after this process exits",
                dataset_manifest_sha256 = truth == null ? null : Hash(truthPath),
                error_p95_px = errors.Count == 0 ? (double?)null : Percentile(errors),
                update_p95_ms = Percentile(durations),
                scope = truth == null ? "positive coverage diagnostic; independent pixel accuracy not evaluated here" :
                    "real-video planar canonical point against independent four-corner annotations; 5 px tolerance" }));
            if (!passed) { throw new InvalidOperationException("Positive replay coverage failed; see replay-result.json."); }
            Console.WriteLine("PASS real " + mode + " replay positive coverage");
        }

        private static double Percentile(List<double> values) =>
            values.OrderBy(value => value).ToArray()[(int)Math.Ceiling((values.Count - 1) * .95)];

        private static string Hash(string path)
        {
            using (var hash = System.Security.Cryptography.SHA256.Create())
            using (var input = File.OpenRead(path))
            { return BitConverter.ToString(hash.ComputeHash(input)).Replace("-", "").ToLowerInvariant(); }
        }
    }
}
