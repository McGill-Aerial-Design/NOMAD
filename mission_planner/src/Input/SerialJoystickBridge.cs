// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Serial Joystick Bridge launcher
// ============================================================
// Spawns the jotystick.py serial → vgamepad/ViGEmBus process so a
// RadioMaster (or similar) microcontroller flow shows up to Windows
// as an Xbox 360 controller, which NomadJoystickService can then
// consume as a DirectInput device just like any other gamepad.
// ============================================================

using System;
using System.Diagnostics;
using System.IO;

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
        private string _lastResolvedScript;
        private DateTime _lastStartUtc;

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
                    return $"Exited (code {code})";
                }
                var uptime = DateTime.UtcNow - _lastStartUtc;
                return $"Running (PID {SafePid(_proc)}, up {(int)uptime.TotalSeconds}s) — {Path.GetFileName(_lastResolvedScript ?? "")} @ {_config.SerialJoystickPort}";
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

                string python = string.IsNullOrWhiteSpace(_config.SerialJoystickPython)
                    ? "python"
                    : _config.SerialJoystickPython;

                // Spawn python in its own visible console window. Two reasons:
                //   1. The previous headless launch (UseShellExecute=false +
                //      CreateNoWindow=true) silently failed for some users —
                //      vgamepad/ViGEmBus appears to rely on a real console
                //      session, and redirected stdout fights Python's default
                //      block-buffering so we couldn't see why.
                //   2. A visible window means the user immediately sees pyserial
                //      / vgamepad errors and live telemetry frames, matching
                //      what they get from running the script in a terminal.
                // We still own the spawned cmd.exe handle so Stop() can kill it.
                // Using cmd /k keeps the window open if python exits with an
                // error so the message stays on screen.
                string argString = $"/k title NOMAD Joystick Bridge && \"{python}\" \"{script}\" --port {_config.SerialJoystickPort} --baud {_config.SerialJoystickBaud}";
                var psi = new ProcessStartInfo
                {
                    FileName = "cmd.exe",
                    Arguments = argString,
                    UseShellExecute = true,         // required for a real console window
                    CreateNoWindow = false,
                    WindowStyle = ProcessWindowStyle.Normal,
                    WorkingDirectory = Path.GetDirectoryName(script) ?? Environment.CurrentDirectory,
                };

                try
                {
                    _proc = new Process { StartInfo = psi, EnableRaisingEvents = true };
                    _proc.Exited += (s, e) =>
                    {
                        // Surface unexpected exits so the user notices the bridge died.
                        Log.Error($"cmd window closed (code {SafeExitCode(_proc)}).");
                    };
                    _proc.Start();
                    _lastStartUtc = DateTime.UtcNow;
                    _lastError = null;
                    Log.Error($"launched in console window ({python} {script} --port {_config.SerialJoystickPort})");
                }
                catch (Exception ex)
                {
                    _lastError = ex.Message;
                    Log.Error($"failed to launch — {ex.Message}");
                    _proc = null;
                }
            }
        }

        public void Stop()
        {
            lock (_gate)
            {
                if (_proc == null) return;
                try
                {
                    if (!_proc.HasExited)
                    {
                        // cmd.exe's python child won't die from Process.Kill() on
                        // the parent — taskkill /T walks the tree.
                        int pid = -1;
                        try { pid = _proc.Id; } catch { }
                        if (pid > 0)
                        {
                            try
                            {
                                var killer = new ProcessStartInfo
                                {
                                    FileName = "taskkill",
                                    Arguments = $"/PID {pid} /T /F",
                                    UseShellExecute = false,
                                    CreateNoWindow = true,
                                    RedirectStandardOutput = true,
                                    RedirectStandardError = true,
                                };
                                using (var p = Process.Start(killer)) { p?.WaitForExit(1500); }
                            }
                            catch { }
                        }
                        try { if (!_proc.HasExited) _proc.Kill(); } catch { }
                        try { _proc.WaitForExit(1500); } catch { }
                    }
                }
                catch { }
                try { _proc.Dispose(); } catch { }
                _proc = null;
            }
        }

        public void UpdateConfig(NOMADConfig config)
        {
            if (config == null) return;
            _config = config;
            // Bridge args are baked at process start, so a config change always
            // means a full relaunch — cheaper and more predictable than IPC.
            Stop();
            if (_config.SerialJoystickEnabled) Start();
        }

        public void Dispose() => Stop();

        // ============================================================
        // Helpers
        // ============================================================

        // Tries the configured path verbatim, then a small set of conventional
        // locations relative to the running DLL so a default install Just Works.
        private static string ResolveScriptPath(string configured)
        {
            if (!string.IsNullOrWhiteSpace(configured) && File.Exists(configured))
                return configured;

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

        private static string SafeExitCode(Process p)
        {
            try { return p.ExitCode.ToString(); } catch { return "?"; }
        }

        private static string SafePid(Process p)
        {
            try { return p.Id.ToString(); } catch { return "?"; }
        }
    }
}
