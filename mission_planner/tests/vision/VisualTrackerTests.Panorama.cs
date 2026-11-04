// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Threading;
using System.Security.Cryptography;
using Newtonsoft.Json.Linq;
using NOMAD.MissionPlanner;

internal static partial class VisualTrackerTests
{
    private static void PanoramaReturn()
    {
        var assembly = System.Reflection.Assembly.GetExecutingAssembly();
        JObject fixture;
        using (var input = assembly.GetManifestResourceStream("NOMAD.Tests.panorama.json"))
        using (var reader = new StreamReader(input)) { fixture = JObject.Parse(reader.ReadToEnd()); }
        var frames = (JArray)fixture["frames"];
        var selected = (JArray)fixture["selection"];
        var selection = new RectangleF((float)selected[0], (float)selected[1], (float)selected[2], (float)selected[3]);
        foreach (var mode in new[] { VisualTrackingMode.Object, VisualTrackingMode.Area })
        using (var tracker = VisualTrackerFactory.Create(CancellationToken.None, mode))
        using (var reference = PanoramaImage(frames[0]))
        using (var trace = new StreamWriter(Path.Combine(AppDomain.CurrentDomain.BaseDirectory,
            "panorama-" + mode + "-" + Guid.NewGuid().ToString("N") + ".jsonl")))
        {
            tracker.Initialize(reference, selection, CancellationToken.None);
            int returned = 0, positives = 0, verified = 0, outside = 0, firstRecovery = 0;
            for (int index = 1; index < frames.Count; index++)
            using (var frame = PanoramaImage(frames[index]))
            {
                var data = frames[index];
                var elapsed = System.Diagnostics.Stopwatch.StartNew();
                var result = tracker.Update(frame, CancellationToken.None);
                bool visible = (bool)data["visible"];
                bool returning = (bool)data["returning"];
                if (returning && visible) { returned++; }
                double? error = null;
                if (result.Found)
                {
                    Check(data["aim"].Type != JTokenType.Null, mode + " accepted the target behind the camera");
                    var aim = new PointF((float)data["aim"][0], (float)data["aim"][1]);
                    if (mode == VisualTrackingMode.Object)
                    {
                        Check(data["object_outline"].All(p => p.Type != JTokenType.Null),
                            "Object accepted the watering can behind the camera");
                        var corners = data["object_outline"].Select(p => new PointF((float)p[0], (float)p[1])).ToArray();
                        aim = new PointF((corners.Min(p => p.X) + corners.Max(p => p.X)) / 2,
                            (corners.Min(p => p.Y) + corners.Max(p => p.Y)) / 2);
                    }
                    error = PointDistance(result.AimPoint.Value, aim);
                }
                else if (!visible) { outside++; }
                if (returning && visible && result.Found && firstRecovery == 0) { firstRecovery = returned; }
                if (returning && visible && returned >= 6)
                { positives++; if (result.Found) { verified++; } }
                trace.WriteLine(new JObject { ["frame"] = index, ["mode"] = mode.ToString(),
                    ["state"] = result.State.ToString(), ["status"] = result.Status,
                    ["visible"] = visible, ["returning"] = returning, ["error_px"] = error,
                    ["aim_x"] = result.AimPoint?.X, ["aim_y"] = result.AimPoint?.Y,
                    ["box"] = result.Bounds.HasValue ? new JArray(result.Bounds.Value.X, result.Bounds.Value.Y,
                        result.Bounds.Value.Width, result.Bounds.Value.Height) : null,
                    ["update_ms"] = elapsed.Elapsed.TotalMilliseconds }.ToString(Newtonsoft.Json.Formatting.None));
                trace.Flush();
                // Whole silhouette extent is estimated; Area retains its tighter immutable-point check.
                double tolerance = mode == VisualTrackingMode.Object ? 8 : 5;
                Check(!error.HasValue || error <= tolerance,
                    mode + " panorama aim exceeded its centre/point tolerance at frame " + index + ": " + error);
                Check(elapsed.ElapsedMilliseconds < 1000, "Panorama update exceeded one second");
            }
            Check(outside >= 10, "Panorama never exercised sustained out-of-frame loss");
            Check(firstRecovery >= 3 && firstRecovery <= 6 && positives >= 3 && verified >= positives * .95,
                mode + " failed 360-degree return: first=" + firstRecovery + ", positive=" + verified + "/" + positives);
            Console.WriteLine(mode + " photographed 360-degree return: first=" + firstRecovery +
                ", positive=" + verified + "/" + positives + ", no reselection.");
        }
    }

    private static Bitmap PanoramaImage(JToken frame)
    {
        using (var input = System.Reflection.Assembly.GetExecutingAssembly().GetManifestResourceStream(
            "NOMAD.Tests." + (string)frame["file"]))
        using (var algorithm = SHA256.Create())
        {
            string hash = BitConverter.ToString(algorithm.ComputeHash(input)).Replace("-", "").ToLowerInvariant();
            Check(hash == (string)frame["sha256"], "Frozen panorama view changed: " + frame["file"]);
            input.Position = 0;
            using (var image = new Bitmap(input)) { return new Bitmap(image); }
        }
    }
}
