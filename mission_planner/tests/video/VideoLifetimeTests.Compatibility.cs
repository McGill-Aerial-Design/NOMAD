// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Threading;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void VideoCompatibility()
    {
        Check(typeof(NOMADVideoView).GetConstructor(new[] { typeof(NOMADConfig) }) != null,
            "Original public Video view constructor is missing");
        Check(typeof(NOMADDashboardView).GetConstructor(new[]
            { typeof(NOMADConfig), typeof(MAVLinkConnectionManager) }) != null,
            "Original public dashboard constructor is missing");
        var fake = new FakeVideoPipeline();
        using (var player = new EmbeddedVideoPlayer("fake", "UDP://@:6500", true,
            CancellationToken.None, () => fake))
        {
            player.StartStream();
            Wait(fake.ReadEntered, "UDP stream startup");
            Check(fake.Pipeline.StartsWith("udpsrc port=6500 ", StringComparison.Ordinal),
                "Configured UDP port was changed: " + fake.Pipeline);
        }
        CheckClean(fake);
    }
}
