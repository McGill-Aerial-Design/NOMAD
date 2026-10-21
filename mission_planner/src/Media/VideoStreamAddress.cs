// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Globalization;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    internal static class VideoStreamAddress
    {
        internal static int UdpPort(string url)
        {
            const string error = "Use a UDP video URL with an explicit port from 1 to 65535.";
            if (string.IsNullOrWhiteSpace(url) || url.Any(char.IsControl) ||
                !url.StartsWith("udp://", StringComparison.OrdinalIgnoreCase)) { throw new FormatException(error); }
            string shortPort = url.Substring(6);
            if (shortPort.StartsWith("@:", StringComparison.Ordinal)) { shortPort = shortPort.Substring(2); }
            else if (shortPort.StartsWith(":", StringComparison.Ordinal)) { shortPort = shortPort.Substring(1); }
            if (!int.TryParse(shortPort, NumberStyles.None, CultureInfo.InvariantCulture, out int port))
            {
                if (!Uri.TryCreate(url, UriKind.Absolute, out var uri) || string.IsNullOrEmpty(uri.Host) ||
                    uri.UserInfo != "" || uri.Query != "" || uri.Fragment != "" ||
                    (uri.AbsolutePath != "" && uri.AbsolutePath != "/")) { throw new FormatException(error); }
                port = uri.Port;
            }
            if (port < 1 || port > 65535) { throw new FormatException(error); }
            return port;
        }

        internal static string QuotedRtspUrl(string url)
        {
            if (string.IsNullOrWhiteSpace(url) || url.Any(char.IsControl) ||
                !Uri.TryCreate(url, UriKind.Absolute, out var uri) ||
                (uri.Scheme != "rtsp" && uri.Scheme != "rtsps"))
            {
                throw new FormatException("Use an absolute RTSP video URL with a host.");
            }
            // Framework RTSP parsing can hide the port in Host; validate the same authority with HTTP parsing.
            if (!Uri.TryCreate("http" + url.Substring(url.IndexOf(':')), UriKind.Absolute, out var authority) ||
                string.IsNullOrEmpty(authority.Host) || authority.Port < 1)
            {
                throw new FormatException("RTSP video requires a host and a valid port.");
            }
            // A URL is one GStreamer property, even when its query contains pipeline delimiters.
            return "\"" + uri.AbsoluteUri.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";
        }
    }
}
