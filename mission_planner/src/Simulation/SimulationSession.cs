// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;
using NOMAD.Simulation;

namespace NOMAD.MissionPlanner
{
    internal sealed class SimulationEndpoint
    {
        internal IPAddress Host;
        internal string ScenarioHash;
        internal int MavlinkPort, SystemId, VideoTcpPort, VideoPort, CameraFov;
        internal string VideoUrl => VideoPort == 0 ? "" : "udp://127.0.0.1:" + VideoPort;

        internal static SimulationEndpoint Parse(string line)
        {
            if (line == null || line.Length > 32768) { throw new InvalidDataException("No simulator readiness record."); }
            var data = JObject.Parse(line);
            if ((string)data["event"] != "ready" || (int?)data["schema"] != 1 ||
                !IPAddress.TryParse((string)data["host"], out IPAddress host) || host.AddressFamily != AddressFamily.InterNetwork)
            { throw new InvalidDataException("Invalid simulator readiness record."); }
            int port = (int?)data["mavlink_port"] ?? 0, system = (int?)data["system_id"] ?? 0;
            int video = (int?)data["video_tcp_port"] ?? 0;
            if (port < 1 || port > 65535 || system < 1 || system > 254 || video < 0 || video > 65535)
            { throw new InvalidDataException("Invalid simulator endpoint."); }
            return new SimulationEndpoint { Host = host, MavlinkPort = port, SystemId = system,
                ScenarioHash = (string)data["scenario_sha256"], VideoTcpPort = video,
                CameraFov = Math.Max(5, Math.Min(170, (int)Math.Round((double?)data["camera_hfov"] ?? 90))) };
        }
    }

    internal sealed class SimulationStatus
    {
        internal string Phase, Message, Directory, Name, VideoUrl;
        internal bool Active, CanConnect;
        internal int CameraFov;
    }

    internal sealed class SimulationSession : IDisposable
    {
        private sealed class Run
        {
            internal readonly CancellationTokenSource Cancellation = new CancellationTokenSource();
            internal readonly TaskCompletionSource<bool> Stopped = new TaskCompletionSource<bool>(
                TaskCreationOptions.RunContinuationsAsynchronously);
            internal readonly TaskCompletionSource<bool> Started = new TaskCompletionSource<bool>(
                TaskCreationOptions.RunContinuationsAsynchronously);
            internal Process Process;
            internal readonly OwnedProcessJob Job = new OwnedProcessJob();
            internal Task<string> Errors;
            internal Task NativeConnect = Task.CompletedTask;
            internal SimulationEndpoint Endpoint;
            internal RtpBridge Video;
            internal string Directory, Name, Failure;
            internal bool Stopping, Connecting;
        }

        private readonly object _gate = new object();
        private readonly ISimulationConnection _native;
        private readonly Func<SimulationPreferences, SimulationScenario, string, Task<ProcessStartInfo>> _command;
        private Run _run;
        private string _phase = "Stopped", _message = "Review WSL, the prepared simulator folder and scenario, then Start.";
        private string _lastDirectory, _lastName;
        private bool _disposed;
        internal event Action Changed;

        internal SimulationSession() : this(new NativeSimulationConnection(), BuildCommandAsync) { }
        internal SimulationSession(ISimulationConnection native,
            Func<SimulationPreferences, SimulationScenario, string, Task<ProcessStartInfo>> command)
        { _native = native; _command = command; }

        internal SimulationStatus Status
        {
            get
            {
                lock (_gate)
                {
                    bool connected = _native.IsConnected;
                    return new SimulationStatus { Phase = _phase == "Connected" && !connected ? "Running" : _phase,
                    Message = _phase == "Connected" && !connected ? "Simulator active; its native link is not selected or connected." : _message,
                    Active = _run != null, Directory = _run?.Directory ?? _lastDirectory, Name = _run?.Name ?? _lastName,
                    CanConnect = _run?.Endpoint != null && _run.Started.Task.IsCompleted && !_run.Stopping &&
                        !_run.Connecting && !connected,
                    VideoUrl = _run?.Stopping != false ? "" : _run.Endpoint?.VideoUrl ?? "",
                    CameraFov = _run?.Endpoint?.CameraFov ?? 90 }; }
            }
        }

        private void Update(Run run, string phase, string message)
        {
            bool notify;
            lock (_gate)
            { if (!ReferenceEquals(_run, run)) { return; } _phase = phase; _message = message; notify = !_disposed; }
            if (notify) { Changed?.Invoke(); }
        }

        internal async Task StartAsync(SimulationPreferences options, SimulationScenario scenario)
        {
            options = options.Copy();
            options.Validate();
            if (SimulationScenario.Load(scenario.Path).Hash != scenario.Hash)
            { throw new InvalidOperationException("Scenario changed; reload it before starting."); }
            if (options.WorldPath.Length > 0 && scenario.Backend != "gazebo")
            { throw new InvalidOperationException("World files require a Gazebo scenario; last_letter is a separate headless backend."); }
            if (!_native.CanStart(out string reason)) { throw new InvalidOperationException(reason); }
            Run run;
            lock (_gate)
            {
                if (_disposed || _run != null || _phase == "Cleanup failed")
                { throw new InvalidOperationException("The simulation owner is busy, retired, or has an unresolved cleanup failure."); }
                run = new Run { Name = scenario.Name, Directory = Path.Combine(NOMADStorage.PluginsDirectory,
                    "simulation", "run-" + Guid.NewGuid().ToString("N")) };
                _run = run;
                _lastDirectory = run.Directory; _lastName = run.Name;
            }
            Update(run, "Starting", "Checking configured simulator builds and starting the scenario…");
            try
            {
                Directory.CreateDirectory(run.Directory);
                File.WriteAllText(Path.Combine(run.Directory, "run.id"), Path.GetFileName(run.Directory));
                var command = await _command(options, scenario, run.Directory).ConfigureAwait(false);
                run.Cancellation.Token.ThrowIfCancellationRequested();
                run.Process = Process.Start(command);
                run.Job.Attach(run.Process);
                run.Errors = run.Process.StandardError.ReadToEndAsync();
                var read = run.Process.StandardOutput.ReadLineAsync();
                if (await Task.WhenAny(read, Task.Delay(100000, run.Cancellation.Token)).ConfigureAwait(false) != read)
                { run.Cancellation.Token.ThrowIfCancellationRequested(); throw new TimeoutException("Simulator startup timed out."); }
                string ready = await read.ConfigureAwait(false);
                if (ready == null)
                {
                    string detail = "Open the session logs for details.";
                    if (await Task.WhenAny(run.Errors, Task.Delay(1000)).ConfigureAwait(false) == run.Errors)
                    {
                        string error = (await run.Errors.ConfigureAwait(false)).Trim();
                        if (error.Length > 0)
                        { detail = error.Substring(error.LastIndexOf('\n') + 1).Trim(); }
                    }
                    throw new IOException("Simulator launcher exited before it was ready. " + detail);
                }
                run.Endpoint = SimulationEndpoint.Parse(ready);
                if (!string.Equals(run.Endpoint.ScenarioHash, scenario.Hash, StringComparison.OrdinalIgnoreCase) ||
                    run.Endpoint.SystemId != scenario.SystemId)
                { throw new InvalidDataException("The launched scenario changed; reload it before connecting."); }
                run.Cancellation.Token.ThrowIfCancellationRequested();
                if (run.Endpoint.VideoTcpPort != 0)
                {
                    using (var available = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0)))
                    { run.Endpoint.VideoPort = ((IPEndPoint)available.Client.LocalEndPoint).Port; }
                    run.Video = new RtpBridge(run.Endpoint.Host, run.Endpoint.VideoTcpPort, run.Endpoint.VideoPort);
                }
                Update(run, "Running", "Simulator ready. Flight and setup use Mission Planner's normal pages.");
                _ = Task.Run(async () =>
                {
                    try
                    {
                        while (!run.Process.WaitForExit(250))
                        {
                            if (run.Stopping) { return; }
                            if (run.Video?.Error != null)
                            { lock (_gate) { if (!run.Stopping) { run.Failure = "Simulator video relay failed: " + run.Video.Error.Message; } } break; }
                        }
                        if (!run.Stopping)
                        {
                            if (run.Process.HasExited && run.Process.ExitCode != 0) { run.Failure = "Simulator failed; inspect the session logs."; }
                            Update(run, "Ended", run.Failure ?? "Simulator ended; closing its owned connection.");
                            await StopAsync(run).ConfigureAwait(false);
                        }
                    }
                    catch (Exception error) when (error is ObjectDisposedException || error is InvalidOperationException)
                    { if (!run.Stopping) { run.Failure = error.Message; await StopAsync(run).ConfigureAwait(false); } }
                });
                run.Started.TrySetResult(true);
                if (options.AutoConnect) { await ConnectAsync().ConfigureAwait(false); }
            }
            catch (Exception error)
            {
                bool canceled = run.Stopping || error is OperationCanceledException;
                if (!canceled)
                {
                    run.Failure = error.Message;
                    try { File.WriteAllText(Path.Combine(run.Directory, "startup-error.log"), error.ToString()); }
                    catch (Exception logError) when (logError is IOException || logError is UnauthorizedAccessException) { }
                    Update(run, "Failed", error.Message + " Open the session logs for details.");
                }
                run.Started.TrySetResult(true);
                await StopAsync(run).ConfigureAwait(false);
                if (!canceled) { throw; }
            }
        }

        internal async Task ConnectAsync()
        {
            Run run;
            lock (_gate)
            {
                run = _run;
                if (run?.Endpoint == null || run.Stopping || run.Connecting || _disposed || _native.IsConnected) { return; }
                run.Connecting = true;
                run.NativeConnect = _native.ConnectAsync(run.Endpoint, run.Directory, run.Cancellation.Token);
            }
            Update(run, "Connecting", "Waiting for Mission Planner's native simulator handshake…");
            try
            {
                await run.NativeConnect.ConfigureAwait(false);
                if (!run.Stopping) { Update(run, "Connected", "Connected to the owned simulator. Parameters remain available through MP setup."); }
            }
            catch (Exception error)
            { if (!run.Stopping) { Update(run, "Running", "Simulator ready; native connection: " + error.Message); } }
            finally { lock (_gate) { run.Connecting = false; } Changed?.Invoke(); }
        }

        internal Task StopAsync() => StopAsync(null);

        private Task StopAsync(Run expected)
        {
            Run run;
            lock (_gate)
            {
                run = _run;
                if (run == null) { return Task.CompletedTask; }
                if (expected != null && !ReferenceEquals(run, expected)) { return Task.CompletedTask; }
                if (run.Stopping) { return run.Stopped.Task; }
                run.Stopping = true; run.Cancellation.Cancel();
            }
            Update(run, "Stopping", "Stopping owned processes; a dispatched native helper may still finish.");
            _ = FinishStopAsync(run);
            return run.Stopped.Task;
        }

        private async Task FinishStopAsync(Run run)
        {
            string failure = null;
            try
            {
                try { _native.CloseTransport(); } catch (Exception error) { failure = error.Message; }
                SignalStop(run.Process);
                // A stop during preparation must fence a process that has not yet been assigned.
                await run.Started.Task.ConfigureAwait(false);
                if (run.Process != null)
                {
                    SignalStop(run.Process);
                    if (!await Task.Run(() => run.Process.WaitForExit(30000)).ConfigureAwait(false))
                    {
                        run.Job.Dispose();
                        if (!await Task.Run(() => run.Process.WaitForExit(3000)).ConfigureAwait(false))
                        { throw new TimeoutException("Owned simulator launcher did not exit."); }
                    }
                    string record = Path.Combine(run.Directory, "session.json");
                    if (File.Exists(record))
                    {
                        var data = JObject.Parse(File.ReadAllText(record));
                        if (!(data["cleanup_errors"] is JArray errors) || errors.Count != 0)
                        { throw new IOException("Linux reported incomplete simulator cleanup. Inspect session.json."); }
                    }
                    else if (run.Endpoint != null) { throw new IOException("Simulator cleanup acknowledgement is missing."); }
                }
            }
            catch (Exception error) { failure = error.Message; }
            finally
            {
                try { _native.CloseTransport(); } catch (Exception error) { failure = failure ?? error.Message; }
                // Even failed launcher cleanup must drain a native helper already dispatched by MP.
                try { await run.NativeConnect.ConfigureAwait(false); } catch (Exception) { }
                foreach (Action cleanup in new Action[] { () => _native.CloseOwned(), () => run.Video?.Dispose(),
                    () => run.Job.Dispose(), () => run.Process?.Dispose() })
                { try { cleanup(); } catch (Exception error) { failure = failure ?? error.Message; } }
                if (run.Errors != null)
                {
                    try { File.WriteAllText(Path.Combine(run.Directory, "launcher-errors.log"), await run.Errors.ConfigureAwait(false)); }
                    catch (Exception error) { failure = failure ?? error.Message; }
                }
                lock (_gate)
                {
                    if (ReferenceEquals(_run, run))
                    { _run = null; _phase = failure != null ? "Cleanup failed" : run.Failure != null ? "Failed" : "Stopped";
                        _message = failure ?? run.Failure ?? "Owned simulator stopped. Session logs are retained."; }
                }
                run.Cancellation.Dispose(); run.Stopped.TrySetResult(true); Changed?.Invoke();
            }
        }

        private static void SignalStop(Process process)
        {
            if (process == null) { return; }
            try { process.StandardInput.WriteLine("STOP"); process.StandardInput.Close(); }
            catch (Exception error) when (error is IOException || error is ObjectDisposedException || error is InvalidOperationException) { }
        }

        internal static Task<string> QueryAsync(string distribution, params string[] arguments)
        {
            string prefix = string.IsNullOrWhiteSpace(distribution) ? "" : "-d " + SimulationPreferences.Argument(distribution) + " ";
            return WslQueryAsync(prefix + "--exec " + string.Join(" ", arguments.Select(SimulationPreferences.Argument)));
        }

        internal static async Task<string> WslQueryAsync(string arguments)
        {
            var start = new ProcessStartInfo("wsl.exe", arguments)
            { UseShellExecute = false, CreateNoWindow = true, RedirectStandardOutput = true, RedirectStandardError = true };
            using (var process = Process.Start(start))
            {
                var output = ReadQueryAsync(process.StandardOutput.BaseStream);
                var errors = ReadQueryAsync(process.StandardError.BaseStream);
                if (!await Task.Run(() => process.WaitForExit(10000)).ConfigureAwait(false))
                { process.Kill(); throw new TimeoutException("WSL path lookup timed out."); }
                string stdout = await output.ConfigureAwait(false), stderr = await errors.ConfigureAwait(false);
                if (process.ExitCode != 0)
                { throw new IOException("WSL exited " + process.ExitCode + ": " + (stderr + " " + stdout).Trim()); }
                return stdout;
            }
        }

        private static async Task<string> ReadQueryAsync(Stream stream)
        {
            using (var bytes = new MemoryStream())
            {
                await stream.CopyToAsync(bytes).ConfigureAwait(false);
                byte[] data = bytes.ToArray();
                // WSL writes Windows-side errors as UTF-16LE, but Linux command output as UTF-8.
                bool unicode = data.Length >= 2 && ((data[0] == 255 && data[1] == 254) ||
                    (data.Length % 2 == 0 && Array.IndexOf(data, (byte)0) >= 0));
                return (unicode ? Encoding.Unicode : Encoding.UTF8).GetString(data).Trim('\uFEFF', '\0', ' ', '\r', '\n');
            }
        }

        private static async Task<ProcessStartInfo> BuildCommandAsync(SimulationPreferences options,
            SimulationScenario scenario, string directory)
        {
            string script = await QueryAsync(options.Distribution, "wslpath", "-a", Path.Combine(options.ToolsDirectory, "interactive.py"));
            string run = await QueryAsync(options.Distribution, "wslpath", "-a", directory);
            string file = await QueryAsync(options.Distribution, "wslpath", "-a", scenario.Path);
            var arguments = new System.Collections.Generic.List<string> { options.PythonExecutable, script, "--run", run,
                "--scenario", file, "--duration-seconds", (options.DurationMinutes * 60).ToString() };
            if (options.Workspace.Length > 0) { arguments.AddRange(new[] { "--workspace", options.Workspace }); }
            if (!options.Visible) { arguments.Add("--no-gui"); }
            if (options.WorldPath.Length > 0)
            { arguments.Add("--world"); arguments.Add(await QueryAsync(options.Distribution, "wslpath", "-a", options.WorldPath)); }
            string prefix = options.Distribution.Length == 0 ? "" : "-d " + SimulationPreferences.Argument(options.Distribution) + " ";
            return new ProcessStartInfo("wsl.exe", prefix + "--exec " + string.Join(" ", arguments.Select(SimulationPreferences.Argument)))
            { UseShellExecute = false, CreateNoWindow = true, RedirectStandardInput = true,
                RedirectStandardOutput = true, RedirectStandardError = true, WorkingDirectory = directory };
        }

        public void Dispose()
        {
            lock (_gate) { _disposed = true; }
            _ = StopAsync();
        }
    }
}
