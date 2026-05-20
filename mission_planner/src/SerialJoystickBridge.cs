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
                    Console.WriteLine("NOMAD bridge: not enabled, skipping launch.");
                    return;
                }

                string script = ResolveScriptPath(_config.SerialJoystickScriptPath);
                if (script == null)
                {
                    _lastError = "script not found (set Settings → Joystick → Script Path)";
                    Console.WriteLine("NOMAD bridge: " + _lastError);
                    return;
                }
                _lastResolvedScript = script;

                string python = string.IsNullOrWhiteSpace(_config.SerialJoystickPython)
                    ? "python"
                    : _config.SerialJoystickPython;

                var psi = new ProcessStartInfo
                {
                    FileName = python,
                    Arguments = $"\"{script}\" --port {_config.SerialJoystickPort} --baud {_config.SerialJoystickBaud}",
                    UseShellExecute = false,
                    CreateNoWindow = true,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    WorkingDirectory = Path.GetDirectoryName(script) ?? Environment.CurrentDirectory,
                };

                try
                {
                    _proc = new Process { StartInfo = psi, EnableRaisingEvents = true };
                    _proc.OutputDataReceived += (s, e) => { if (e.Data != null) Console.WriteLine("[NOMAD bridge] " + e.Data); };
                    _proc.ErrorDataReceived  += (s, e) => { if (e.Data != null) Console.WriteLine("[NOMAD bridge ERR] " + e.Data); };
                    _proc.Exited += (s, e) =>
                    {
                        // Surface unexpected exits so the user notices the bridge died.
                        Console.WriteLine($"[NOMAD bridge] process exited (code {SafeExitCode(_proc)}).");
                    };
                    _proc.Start();
                    _proc.BeginOutputReadLine();
                    _proc.BeginErrorReadLine();
                    _lastStartUtc = DateTime.UtcNow;
                    _lastError = null;
                    Console.WriteLine($"NOMAD bridge: launched ({python} {script} --port {_config.SerialJoystickPort})");
                }
                catch (Exception ex)
                {
                    _lastError = ex.Message;
                    Console.WriteLine($"NOMAD bridge: failed to launch — {ex.Message}");
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
                        try { _proc.Kill(); } catch { }
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
            if (!string.IsNullOrEmpty(dllDir))
            {
                foreach (var name in names) yield return Path.Combine(dllDir, name);
                var d = dllDir;
                for (int i = 0; i < 4 && d != null; i++)
                {
                    d = Path.GetDirectoryName(d);
                    if (d == null) break;
                    foreach (var name in names) yield return Path.Combine(d, name);
                }
            }
            // Common repo location on the development machine.
            yield return @"C:\Users\Youssef\Documents\Code\MAD\NOMAD\joystick.py";
            yield return @"C:\Users\Youssef\Documents\Code\MAD\NOMAD\jotystick.py";
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
