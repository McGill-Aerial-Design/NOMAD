// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Diagnostics;
using System.IO;
using System.Reflection;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static string GetSdpPath(string arguments)
    {
        int firstQuote = arguments.IndexOf('"');
        return arguments.Substring(firstQuote + 1).TrimEnd('"');
    }

    private static void ExternalProcessCleanup()
    {
        int launches = 0;
        string sdpPath = null;
        Process observer = null;
        using (var owner = new OwnedVideoProcess(arguments =>
        {
            ++launches;
            sdpPath = GetSdpPath(arguments);
            var process = Process.Start(new ProcessStartInfo(Assembly.GetExecutingAssembly().Location, "--child")
            {
                UseShellExecute = false,
                CreateNoWindow = true,
            });
            observer = Process.GetProcessById(process.Id);
            return process;
        }))
        {
            owner.Start("udp://5600", 100, 5600);
            Check(File.Exists(sdpPath), "SDP resource was not created");
            owner.Start("udp://5600", 100, 5600);
            Check(launches == 1, "External start duplicated process");
            owner.Stop();
            owner.Stop();
            Check(observer.HasExited, "Stop left external process running");
            Check(!File.Exists(sdpPath), "Stop leaked SDP file");
            observer.Dispose();
            owner.Start("udp://5600", 100, 5600);
            owner.Dispose();
            Check(observer.HasExited, "Dispose left external process running");
            Check(!File.Exists(sdpPath), "Dispose leaked SDP file");
            observer.Dispose();
        }
    }

    private static void ExternalStartupFailure()
    {
        string sdpPath = null;
        using (var owner = new OwnedVideoProcess(arguments =>
        {
            sdpPath = GetSdpPath(arguments);
            Check(File.Exists(sdpPath), "Failure was not injected after resource creation");
            throw new InvalidOperationException("injected process failure");
        }))
        {
            try
            {
                owner.Start("udp://5600", 100, 5600);
                throw new Exception("External startup unexpectedly succeeded");
            }
            catch (InvalidOperationException ex)
            {
                Check(ex.Message == "injected process failure", "Unexpected external startup error");
            }
            Check(!File.Exists(sdpPath), "Failed startup leaked SDP resource");
            owner.Stop();
        }
    }
}
