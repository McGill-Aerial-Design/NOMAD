// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Threading;
using System.Reflection;
using System.Drawing;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void VideoAddressRegressions()
    {
        var failures = new List<string>();
        var udp = new FakeVideoPipeline();
        using (var player = new EmbeddedVideoPlayer("Synthetic UDP", "udp://127.0.0.1:6000", true,
            CancellationToken.None, () => udp))
        {
            player.StartStream();
            Wait(udp.ReadEntered, "synthetic UDP startup");
            if (!udp.Pipeline.StartsWith("udpsrc port=6000 ", StringComparison.Ordinal))
            { failures.Add("Normal UDP URL discarded its configured port."); }
        }
        CheckClean(udp);
        const string url = "rtsp://synthetic:credential@video.invalid/live?note=\"left ! fakesink\"&view=main";
        var rtsp = new FakeVideoPipeline();
        using (var player = new EmbeddedVideoPlayer("Synthetic RTSP", url, true,
            CancellationToken.None, () => rtsp))
        {
            player.StartStream();
            Wait(rtsp.ReadEntered, "synthetic RTSP startup");
            string expected = "location=\"" + new Uri(url).AbsoluteUri + "\" ";
            if (!rtsp.Pipeline.Contains(expected))
            { failures.Add("RTSP URL was not contained in one quoted, URI-encoded source property."); }
        }
        CheckClean(rtsp);
        Check(failures.Count == 0, string.Join(" ", failures));
    }
    private static string HudPipeline(string url) => (string)typeof(NOMADPlugin).GetMethod(
        "BuildHudVideoPipeline", BindingFlags.Static | BindingFlags.NonPublic).Invoke(null, new object[]
            { new VideoInputProfile { Name = "Synthetic", Url = url } });

    private static string EmbeddedPipeline(string url)
    {
        using (var player = new EmbeddedVideoPlayer("Synthetic source", url, true,
            CancellationToken.None, () => throw new InvalidOperationException("No source may start in this check.")))
        {
            return (string)typeof(EmbeddedVideoPlayer).GetMethod("BuildGStreamerPipeline",
                BindingFlags.Instance | BindingFlags.NonPublic).Invoke(player, null);
        }
    }

    private static void UdpVideoAddresses()
    {
        var valid = new Dictionary<string, int>
        {
            { "udp://1", 1 }, { "udp://65535", 65535 }, { "udp://:6000", 6000 }, { "UDP://@:6500", 6500 },
            { "udp://127.0.0.1:6001", 6001 }, { "udp://video.invalid:6002", 6002 }, { "udp://[::1]:6003", 6003 }
        };
        foreach (var item in valid)
        {
            string prefix = "udpsrc port=" + item.Value + " ";
            Check(VideoStreamAddress.UdpPort(item.Key) == item.Value &&
                HudPipeline(item.Key).StartsWith(prefix, StringComparison.Ordinal) &&
                EmbeddedPipeline(item.Key).StartsWith(prefix, StringComparison.Ordinal),
                "UDP port was not honored consistently by both video paths.");
        }
        foreach (string invalid in new[] { "udp://", "udp://0", "udp://65536", "udp://-1", "udp://@:bad",
            "udp://video.invalid", "udp://video.invalid:0", "udp://video.invalid:6000/path",
            "udp://video.invalid:6000?option=ignored", "udp://video.invalid:6000#ignored", "udp://6000\n" })
        {
            bool rejected = false;
            try { VideoStreamAddress.UdpPort(invalid); } catch (FormatException) { rejected = true; }
            Check(rejected, "Invalid UDP address silently selected a default port.");
        }
    }

    private static void RtspVideoAddresses()
    {
        foreach (string url in new[] { "rtsp://video.invalid/live", "rtsps://video.invalid/live?view=main",
            "rtsp://video.invalid:1/live", "rtsp://video.invalid:65535/live", "rtsp://video.invalid:80/live",
            "rtsp://synthetic:credential@video.invalid:8554/live stream?note=\"left ! fakesink\"",
            "rtsp://video.invalid/live?note=left\\right&mode=main!aux" })
        {
            string property = "location=\"" + new Uri(url).AbsoluteUri.Replace("\\", "\\\\") + "\" ";
            string hud = HudPipeline(url);
            Check(hud.Contains(property) && EmbeddedPipeline(url).Contains(property),
                "RTSP query/credentials were not one quoted, escaped property in both video paths.");
            Check(!hud.Contains("videocrop"), "HUD imposed a stereo crop on a general video source.");
        }
        foreach (string invalid in new[] { "", "not a URL", "http://video.invalid/live", "rtsp:///live",
            "rtsp://video.invalid:0/live", "rtsp://video.invalid:65536/live", "rtsp://video.invalid/live\n" })
        {
            bool rejected = false;
            try { VideoStreamAddress.QuotedRtspUrl(invalid); } catch (FormatException) { rejected = true; }
            Check(rejected, "Malformed or unsupported synthetic RTSP source was admitted: " + invalid);
        }
    }

    private static void InvalidVideoDoesNotStart()
    {
        int created = 0;
        using (var player = new EmbeddedVideoPlayer("Invalid synthetic source", "udp://bad", true,
            CancellationToken.None, () => { created++; return new FakeVideoPipeline(); }))
        {
            player.StartStream();
            var status = GetField<Label>(player, "_lblStatus");
            Check(created == 0 && status.ForeColor == Color.Red && status.Text.Contains("UDP listening port"),
                "Invalid video source started native resources or escaped its UI error state.");
        }
    }

}
