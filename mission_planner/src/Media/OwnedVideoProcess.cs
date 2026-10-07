// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Diagnostics;
using System.IO;

namespace NOMAD.MissionPlanner
{
    internal sealed class OwnedVideoProcess : IDisposable
    {
        private readonly object _gate = new object();
        private readonly Func<string, Process> _launch;
        private Process _process;
        private string _sdpPath;
        private bool _disposed;

        public OwnedVideoProcess() : this(LaunchVlc) { }

        internal OwnedVideoProcess(Func<string, Process> launch)
        {
            _launch = launch;
        }

        public void Start(string url, int latency, int udpPort)
        {
            lock (_gate)
            {
                if (_disposed || (_process != null && !_process.HasExited))
                {
                    return;
                }
                StopProcess();
                try
                {
                    var arguments = BuildArguments(url, latency, udpPort);
                    _process = _launch(arguments);
                }
                catch
                {
                    StopProcess();
                    throw;
                }
            }
        }

        private string BuildArguments(string url, int latency, int port)
        {
            string source = url;
            if (url.StartsWith("udp://", StringComparison.OrdinalIgnoreCase))
            {
                _sdpPath = Path.Combine(Path.GetTempPath(), "nomad-video-" + Guid.NewGuid().ToString("N") + ".sdp");
                File.WriteAllText(_sdpPath,
                    "v=0\no=- 0 0 IN IP4 127.0.0.1\ns=Stream\nc=IN IP4 127.0.0.1\nt=0 0\n" +
                    $"m=video {port} RTP/AVP 96\na=rtpmap:96 H264/90000");
                source = _sdpPath;
            }
            return $"--no-one-instance --network-caching={latency} --rtsp-tcp \"{source}\"";
        }

        private static Process LaunchVlc(string arguments)
        {
            var paths = new[]
            {
                "vlc",
                Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles),
                    "VideoLAN", "VLC", "vlc.exe"),
                Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86),
                    "VideoLAN", "VLC", "vlc.exe"),
            };
            foreach (var path in paths)
            {
                try
                {
                    return Process.Start(new ProcessStartInfo(path, arguments) { UseShellExecute = false });
                }
                catch (System.ComponentModel.Win32Exception) { }
            }
            throw new InvalidOperationException("VLC not found");
        }

        public void Stop()
        {
            lock (_gate) { StopProcess(); }
        }

        public void Dispose()
        {
            lock (_gate)
            {
                _disposed = true;
                StopProcess();
            }
        }

        private void StopProcess()
        {
            try
            {
                if (_process != null && !_process.HasExited)
                {
                    try
                    {
                        _process.Kill();
                    }
                    catch (InvalidOperationException) when (_process.HasExited) { }
                    _process.WaitForExit();
                }
            }
            finally
            {
                _process?.Dispose();
                _process = null;
                if (_sdpPath != null)
                {
                    File.Delete(_sdpPath);
                    _sdpPath = null;
                }
            }
        }
    }
}
