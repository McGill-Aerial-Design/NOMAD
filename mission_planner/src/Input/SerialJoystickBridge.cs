// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Serial Joystick Bridge launcher
// ============================================================
// Spawns the joystick.py serial → vgamepad/ViGEmBus process so a
// RadioMaster (or similar) microcontroller flow shows up to Windows
// as an Xbox 360 controller, which NomadJoystickService can then
// consume as a DirectInput device just like any other gamepad.
// ============================================================

using System;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using Microsoft.Win32.SafeHandles;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Owns the lifecycle of the Python serial-bridge subprocess.
    /// Idempotent Start/Stop. Logs stdout/stderr to the MP console.
    /// </summary>
    public sealed class SerialJoystickBridge : IDisposable
    {
        private NOMADConfig _config;
        private Process _proc;
        private readonly object _gate = new object();
        private string _lastError;
        private string _lastOutput;
        private string _lastLoggedOutput;
        private string _lastResolvedScript;
        private string _lastResolvedPython;
        private DateTime _lastStartUtc;
        private DateTime _lastOutputLogUtc;
        private bool _stopping;

        public SerialJoystickBridge(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
        }

        public bool IsRunning
        {
            get { lock (_gate) return _proc != null && !_proc.HasExited; }
        }

        /// <summary>
        /// Human-readable status string suitable for a settings UI indicator.
        /// </summary>
        public string GetStatus()
        {
            lock (_gate)
            {
                if (!_config.SerialJoystickEnabled) return "Disabled";
                if (_proc == null) return string.IsNullOrEmpty(_lastError) ? "Not started" : $"Failed: {_lastError}";
                if (_proc.HasExited)
                {
                    string code; try { code = _proc.ExitCode.ToString(); } catch { code = "?"; }
                    return string.IsNullOrWhiteSpace(_lastError)
                        ? $"Exited (code {code})"
                        : $"Exited (code {code}): {_lastError}";
                }
                var uptime = DateTime.UtcNow - _lastStartUtc;
                string detail = string.IsNullOrWhiteSpace(_lastOutput) ? "" : $" - {_lastOutput}";
                return $"Running (PID {SafePid(_proc)}, up {(int)uptime.TotalSeconds}s) - "
                    + $"{Path.GetFileName(_lastResolvedPython ?? "")} "
                    + $"{Path.GetFileName(_lastResolvedScript ?? "")} @ {_config.SerialJoystickPort}{detail}";
            }
        }

        public void Start()
        {
            lock (_gate)
            {
                if (_proc != null && !_proc.HasExited) return;

                if (!_config.SerialJoystickEnabled)
                {
                    Log.Debug("bridge: not enabled, skipping launch.");
                    return;
                }

                string script = ResolveScriptPath(_config.SerialJoystickScriptPath);
                if (script == null)
                {
                    _lastError = "script not found (set Settings → Joystick → Script Path)";
                    Log.Error($"bridge: {_lastError}");
                    return;
                }
                _lastResolvedScript = script;

                string configuredPython = string.IsNullOrWhiteSpace(_config.SerialJoystickPython)
                    ? "python"
                    : _config.SerialJoystickPython;
                string python = ResolveExecutablePath(configuredPython);
                if (python == null)
                {
                    _lastError = $"Python executable not found: {configuredPython}";
                    Log.Error($"bridge: {_lastError}");
                    return;
                }
                _lastResolvedPython = python;

                string port = (_config.SerialJoystickPort ?? "").Trim();
                if (string.IsNullOrEmpty(port))
                {
                    _lastError = "serial port is blank";
                    Log.Error($"bridge: {_lastError}");
                    return;
                }

                var psi = new ProcessStartInfo
                {
                    FileName = python,
                    Arguments = $"-u {QuoteArgument(script)} --port {QuoteArgument(port)} "
                        + $"--baud {_config.SerialJoystickBaud}",
                    UseShellExecute = false,
                    CreateNoWindow = true,
                    WindowStyle = ProcessWindowStyle.Hidden,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    StandardOutputEncoding = Encoding.UTF8,
                    StandardErrorEncoding = Encoding.UTF8,
                    WorkingDirectory = Path.GetDirectoryName(script) ?? Environment.CurrentDirectory,
                };
                psi.EnvironmentVariables["PYTHONUNBUFFERED"] = "1";

                try
                {
                    _proc = new Process { StartInfo = psi, EnableRaisingEvents = true };
                    _proc.OutputDataReceived += BridgeOutputReceived;
                    _proc.ErrorDataReceived += BridgeErrorReceived;
                    _proc.Exited += BridgeExited;
                    if (!_proc.Start())
                        throw new InvalidOperationException("Python process did not start");

                    _proc.BeginOutputReadLine();
                    _proc.BeginErrorReadLine();
                    _lastStartUtc = DateTime.UtcNow;
                    _lastError = null;
                    _lastOutput = "starting";
                    Log.Info($"bridge: started headless PID {SafePid(_proc)} with "
                        + $"{Path.GetFileName(python)} on {port} @ {_config.SerialJoystickBaud}");
                }
                catch (Exception ex)
                {
                    _lastError = ex.Message;
                    Log.Error($"failed to launch — {ex.Message}");
                    try { _proc?.Dispose(); } catch { }
                    _proc = null;
                }
            }
        }

        public void Stop()
        {
            lock (_gate)
            {
                if (_proc == null) return;
                _stopping = true;
                try
                {
                    _proc.Exited -= BridgeExited;
                    if (!_proc.HasExited)
                    {
                        try { _proc.Kill(); } catch { }
                        try { _proc.WaitForExit(1500); } catch { }
                    }
                }
                catch { }
                try { _proc.Dispose(); } catch { }
                _proc = null;
                _stopping = false;
            }
        }

        public void UpdateConfig(NOMADConfig config)
        {
            if (config == null) return;
            _config = config;
            Log.Info($"bridge: applying saved serial setting "
                + $"{_config.SerialJoystickPort} @ {_config.SerialJoystickBaud}");
            Stop();
            if (_config.SerialJoystickEnabled) Start();
        }

        public void Dispose() => Stop();

        // ============================================================
        // Helpers
        // ============================================================

        private void BridgeOutputReceived(object sender, DataReceivedEventArgs e)
        {
            if (string.IsNullOrWhiteSpace(e.Data))
                return;

            if (e.Data.StartsWith("NOMAD bridge:", StringComparison.OrdinalIgnoreCase))
            {
                _lastOutput = e.Data.Substring("NOMAD bridge:".Length).Trim();
                var now = DateTime.UtcNow;
                bool repeated = string.Equals(_lastLoggedOutput, _lastOutput, StringComparison.Ordinal)
                    && now - _lastOutputLogUtc < TimeSpan.FromSeconds(30);
                if (repeated)
                    return;

                _lastLoggedOutput = _lastOutput;
                _lastOutputLogUtc = now;
                if (_lastOutput.IndexOf("failed", StringComparison.OrdinalIgnoreCase) >= 0
                    || _lastOutput.IndexOf("error", StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    Log.Warn($"bridge: {_lastOutput}");
                }
                else
                {
                    Log.Info($"bridge: {_lastOutput}");
                }
            }
        }

        private void BridgeErrorReceived(object sender, DataReceivedEventArgs e)
        {
            if (string.IsNullOrWhiteSpace(e.Data))
                return;

            _lastError = e.Data;
            Log.Error($"bridge stderr: {e.Data}");
        }

        private void BridgeExited(object sender, EventArgs e)
        {
            var process = sender as Process;
            int exitCode = SafeExitCodeValue(process);
            if (_stopping)
            {
                Log.Info("bridge: stopped");
                return;
            }

            if (string.IsNullOrWhiteSpace(_lastError))
                _lastError = $"Python exited with code {exitCode}";
            Log.Error($"bridge: exited with code {exitCode}"
                + (string.IsNullOrWhiteSpace(_lastError) ? "" : $" - {_lastError}"));
        }

        // Tries the configured path verbatim, then a small set of conventional
        // locations relative to the running DLL so a default install Just Works.
        private static string ResolveScriptPath(string configured)
        {
            if (!string.IsNullOrWhiteSpace(configured) && File.Exists(configured))
                return configured;

            if (!string.IsNullOrWhiteSpace(configured))
            {
                string configuredDir;
                try { configuredDir = Path.GetDirectoryName(configured); }
                catch { configuredDir = null; }

                foreach (var candidate in EnumerateCandidates(configuredDir))
                {
                    if (!string.IsNullOrWhiteSpace(candidate) && File.Exists(candidate))
                        return candidate;
                }
            }

            string dllDir;
            try { dllDir = Path.GetDirectoryName(typeof(SerialJoystickBridge).Assembly.Location); }
            catch { dllDir = null; }

            foreach (var candidate in EnumerateCandidates(dllDir))
            {
                if (!string.IsNullOrWhiteSpace(candidate) && File.Exists(candidate))
                    return candidate;
            }
            return null;
        }

        private static string ResolveExecutablePath(string configured)
        {
            string command = Environment.ExpandEnvironmentVariables(configured ?? "").Trim().Trim('"');
            if (string.IsNullOrWhiteSpace(command))
                return null;

            if (Path.IsPathRooted(command) || command.IndexOf(Path.DirectorySeparatorChar) >= 0)
                return File.Exists(command) ? ResolveFinalPath(command) : null;

            string fileName = Path.HasExtension(command) ? command : command + ".exe";
            foreach (string directory in (Environment.GetEnvironmentVariable("PATH") ?? "").Split(';'))
            {
                if (string.IsNullOrWhiteSpace(directory))
                    continue;

                string candidate;
                try { candidate = Path.Combine(directory.Trim().Trim('"'), fileName); }
                catch { continue; }

                if (File.Exists(candidate))
                    return ResolveFinalPath(candidate);
            }
            return null;
        }

        private static string ResolveFinalPath(string path)
        {
            try
            {
                using (var stream = new FileStream(
                    path,
                    FileMode.Open,
                    FileAccess.Read,
                    FileShare.ReadWrite | FileShare.Delete))
                {
                    var resolved = new StringBuilder(1024);
                    uint length = GetFinalPathNameByHandle(
                        stream.SafeFileHandle,
                        resolved,
                        (uint)resolved.Capacity,
                        0);
                    if (length > 0 && length < resolved.Capacity)
                    {
                        string finalPath = resolved.ToString();
                        if (finalPath.StartsWith(@"\\?\", StringComparison.Ordinal))
                            finalPath = finalPath.Substring(4);
                        if (File.Exists(finalPath))
                            return finalPath;
                    }
                }
            }
            catch { }
            return path;
        }

        private static string QuoteArgument(string value)
        {
            return "\"" + (value ?? "").Replace("\"", "\\\"") + "\"";
        }

        private static System.Collections.Generic.IEnumerable<string> EnumerateCandidates(string dllDir)
        {
            // Historical name was jotystick.py; current name is joystick.py. Probe both.
            string[] names = { "joystick.py", "jotystick.py" };
            // joystick.py lives at scripts/hardware/ in the repo; also probe the
            // bare filename for older checkouts.
            string[] subdirs = { "", Path.Combine("scripts", "hardware") };
            if (!string.IsNullOrEmpty(dllDir))
            {
                var d = dllDir;
                for (int i = 0; i < 6 && d != null; i++)
                {
                    foreach (var sub in subdirs)
                        foreach (var name in names)
                            yield return Path.Combine(d, sub, name);
                    d = Path.GetDirectoryName(d);
                }
            }
        }

        private static int SafeExitCodeValue(Process p)
        {
            try { return p?.ExitCode ?? -1; } catch { return -1; }
        }

        private static string SafePid(Process p)
        {
            try { return p.Id.ToString(); } catch { return "?"; }
        }

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern uint GetFinalPathNameByHandle(
            SafeFileHandle file,
            StringBuilder path,
            uint pathLength,
            uint flags);
    }
}
