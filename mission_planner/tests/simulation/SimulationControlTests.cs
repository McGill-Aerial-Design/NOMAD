// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Comms;
using MissionPlanner.Utilities;
using NOMAD.MissionPlanner;

internal static class SimulationControlTests
{
    private static int _checks;
    private static string _exe;
    [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
    private static extern bool SetDllDirectory(string path);

    [STAThread]
    private static int Main(string[] args)
    {
        if (args[0] == "--echo")
        { foreach (string value in args.Skip(1)) { Console.WriteLine(Convert.ToBase64String(Encoding.UTF8.GetBytes(value))); } return 0; }
        if (args[0] == "--child")
        {
            if (args[1] == "early-exit")
            {
                Console.Error.WriteLine("Traceback:\nRuntimeError: gazebo exited during startup; inspect gazebo.log");
                File.WriteAllText("session.json", "{\"cleanup_errors\":[]}"); return 5;
            }
            if (args[1] == "bad") { Console.WriteLine("not a readiness record"); return 3; }
            Console.WriteLine("{\"event\":\"ready\",\"schema\":1,\"host\":\"127.0.0.1\",\"mavlink_port\":45000," +
                "\"system_id\":201,\"video_tcp_port\":0,\"scenario_sha256\":\"" + args[2] + "\"}");
            if (args[1] == "fail")
            { File.WriteAllText("session.json", "{\"cleanup_errors\":[]}"); Thread.Sleep(100); return 7; }
            Console.ReadLine(); File.WriteAllText("session.json", args[1] == "cleanup-fail"
                ? "{\"cleanup_errors\":[\"injected cleanup fault\"]}" : "{\"cleanup_errors\":[]}"); return 0;
        }
        AppDomain.CurrentDomain.AssemblyResolve += (sender, request) =>
        {
            string name = new AssemblyName(request.Name).Name;
            string path = Path.Combine(args[0], name == "MissionPlanner" ? name + ".exe" : name + ".dll");
            return File.Exists(path) ? Assembly.LoadFrom(path) : null;
        };
        try { Run(args); return 0; } catch (Exception error) { Console.Error.WriteLine(error); return 1; }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Run(string[] args)
    {
        Settings.CustomUserDataDirectory = args[1];
        Settings.AppConfigName = "NOMAD-Simulation-Tests-" + Guid.NewGuid().ToString("N");
        Settings.FileName = Path.Combine(args[1], "host.xml");
        _exe = Assembly.GetExecutingAssembly().Location;
        if (args.Length > 2 && args[2] == "native")
        {
            Native(args[3], int.Parse(args[4]), args[1], args.Length > 7 ? int.Parse(args[7]) : -1).GetAwaiter().GetResult();
            if (args.Length > 5 && int.Parse(args[5]) > 0) { Camera(args[3], int.Parse(args[5]), args[1], args[6]); }
        }
        else if (args.Length > 2 && args[2] == "wsl")
        { Wsl(args[3], args[4], args[5], args[1], args.Length > 6 ? args[6] : "",
            args.Length > 7 ? args[7] : "").GetAwaiter().GetResult(); }
        else
        {
            Arguments(); Preferences(args[1]);
            SetupDefaults();
            QueryEncoding().GetAwaiter().GetResult();
            Lifecycle(args[1]).GetAwaiter().GetResult();
            NativeGuards();
        }
        Console.WriteLine("PASS: " + _checks + " simulation UI/ownership/lifecycle checks.");
    }

    private static void Check(bool condition, string message)
    { if (!condition) { throw new InvalidOperationException(message); } _checks++; }
    private static async Task Until(Func<bool> condition, string message)
    {
        var clock = Stopwatch.StartNew();
        while (!condition() && clock.ElapsedMilliseconds < 5000) { await Task.Delay(10).ConfigureAwait(false); }
        Check(condition(), message);
    }
    private static async Task Reject<T>(Func<Task> action, string message) where T : Exception
    { try { await action().ConfigureAwait(false); } catch (T) { _checks++; return; } throw new Exception(message); }

    private static void Arguments()
    {
        Check(SimulationPreferences.Argument("Ubuntu-24.04") == "Ubuntu-24.04",
            "WSL distribution names received unnecessary quotes");
        var values = new[] { "C:\\tools with spaces\\", "a\"b", "", "literal $(value) `ticks`", "trailing\\\\" };
        var start = new ProcessStartInfo(_exe, "--echo " + string.Join(" ", values.Select(SimulationPreferences.Argument)))
        { UseShellExecute = false, CreateNoWindow = true, RedirectStandardOutput = true };
        using (var process = Process.Start(start))
        {
            string[] echoed = process.StandardOutput.ReadToEnd().Split(new[] { '\r', '\n' }, StringSplitOptions.None);
            Check(process.WaitForExit(5000) && process.ExitCode == 0, "Argument fixture exceeded deadline");
            string output = string.Join("\n", echoed.Where((value, index) => index % 2 == 0).Take(values.Length));
            string expected = string.Join("\n", values.Select(value => Convert.ToBase64String(Encoding.UTF8.GetBytes(value))));
            Check(output == expected, "Spaces, quotes, empty values or trailing slashes changed across process launch");
        }
        try { SimulationPreferences.Argument("unsafe\nargument"); throw new Exception("Control character accepted"); }
        catch (ArgumentException) { _checks++; }
    }

    private static async Task QueryEncoding()
    {
        var read = typeof(SimulationSession).GetMethod("ReadQueryAsync", BindingFlags.Static | BindingFlags.NonPublic);
        foreach (var encoding in new[] { Encoding.UTF8, Encoding.Unicode })
        using (var input = new MemoryStream(encoding.GetBytes("WSL test: café 日本語\r\n")))
        {
            string decoded = await (Task<string>)read.Invoke(null, new object[] { input });
            Check(decoded == "WSL test: café 日本語", "WSL UTF-8/Linux or UTF-16/Windows output was lost");
        }
    }

    private static void SetupDefaults()
    {
        var setup = SimulationSetup.Parse("Custom-Linux\r\nOther-Linux\r\n",
            "Custom-Linux\n/home/another-operator/nomad-simulation\n/usr/bin/python3\nprepared\n");
        Check(setup.Distributions.Length == 2 && setup.Distribution == "Custom-Linux" && setup.Prepared &&
            setup.Workspace == "/home/another-operator/nomad-simulation" && setup.Python == "/usr/bin/python3",
            "WSL discovery assumed a distribution or operator-specific home");
        setup = SimulationSetup.Parse("My-Linux", "My-Linux\n/mnt/data/my simulator\n\n");
        Check(setup.Workspace == "/mnt/data/my simulator" && setup.Python == "" && !setup.Prepared,
            "Detection replaced a custom workspace or claimed missing prerequisites");
        try { SimulationSetup.Parse("My-Linux", "Unknown\nrelative\n"); throw new Exception("Bad WSL metadata accepted"); }
        catch (InvalidDataException) { _checks++; }
    }

    private static SimulationPreferences Options(string directory)
    {
        string tools = Path.Combine(directory, "tools with spaces"); Directory.CreateDirectory(tools);
        File.WriteAllText(Path.Combine(tools, "interactive.py"), "# owned test marker");
        string scenario = Path.Combine(tools, "custom scenario.json");
        File.WriteAllText(scenario, "{\"schema\":1,\"name\":\"Custom test\",\"backend\":\"sitl\",\"frame\":\"quad\",\"system_id\":201}");
        return new SimulationPreferences { ToolsDirectory = tools, Distribution = "User chosen distro",
            Workspace = "/home/different operator/custom simulator", PythonExecutable = "/custom venv/bin/python3",
            ScenarioPath = scenario, AutoConnect = false, DurationMinutes = 1 };
    }

    private static void Preferences(string directory)
    {
        var options = Options(directory); options.Validate(); options.Save();
        var loaded = SimulationPreferences.Load();
        Check(loaded.Workspace == options.Workspace && loaded.Distribution == options.Distribution &&
            loaded.PythonExecutable == options.PythonExecutable && loaded.ToolsDirectory == options.ToolsDirectory,
            "User-selected paths, distribution or Python command were replaced");
        File.WriteAllText(SimulationPreferences.StoragePath, "{\"ToolsDirectory\":null}");
        using (var view = new NOMADSimulationView(null, CancellationToken.None, detectOnOpen: false))
        { Check(view.ForeColor == NOMADTheme.TEXT_PRIMARY, "Simulation labels inherit black on dark background"); }
        options.Save();
    }

    private sealed class NativeStub : ISimulationConnection
    {
        internal bool Connected, Foreign;
        internal int Opens, Closes;
        internal TaskCompletionSource<bool> Hold;
        public bool CanStart(out string reason) { reason = "foreign connection"; return !Foreign; }
        public bool IsConnected => Connected;
        public async Task ConnectAsync(SimulationEndpoint endpoint, string directory, CancellationToken cancellation)
        {
            Opens++;
            if (Foreign) { throw new InvalidOperationException("foreign connection"); }
            if (Hold != null) { await Hold.Task.ConfigureAwait(false); }
            Connected = true;
        }
        public void CloseTransport() { }
        public void CloseOwned() { Connected = false; Closes++; }
    }

    private static ProcessStartInfo Child(string mode, SimulationScenario scenario, string directory = null) => new ProcessStartInfo(_exe,
        "--child " + mode + " " + scenario.Hash)
    { UseShellExecute = false, CreateNoWindow = true, RedirectStandardInput = true,
        RedirectStandardOutput = true, RedirectStandardError = true, WorkingDirectory = directory ?? Environment.CurrentDirectory };

    private static async Task Lifecycle(string directory)
    {
        var options = Options(directory); var scenario = SimulationScenario.Load(options.ScenarioPath);
        using (var early = new SimulationSession(new NativeStub(), (a, b, c) => Task.FromResult(Child("early-exit", b, c))))
        {
            await Reject<IOException>(() => early.StartAsync(options, scenario), "A launcher exiting before readiness succeeded")
                .ConfigureAwait(false);
            Check(early.Status.Phase == "Failed" && early.Status.Message.Contains("gazebo exited") &&
                File.ReadAllText(Path.Combine(early.Status.Directory, "launcher-errors.log")).Contains("RuntimeError"),
                "Early launcher exit lost its reason or stderr artifact");
        }
        using (var failedQuery = new SimulationSession(new NativeStub(), (a, b, c) =>
            Task.FromException<ProcessStartInfo>(new IOException("injected WSL query failure"))))
        {
            await Reject<IOException>(() => failedQuery.StartAsync(options, scenario), "A failed WSL lookup succeeded")
                .ConfigureAwait(false);
            Check(File.ReadAllText(Path.Combine(failedQuery.Status.Directory, "startup-error.log"))
                .Contains("injected WSL query failure"), "A startup failure before launcher creation had no diagnostic artifact");
        }
        int blockedLaunches = 0;
        var blockedNative = new NativeStub { Foreign = true };
        using (var blocked = new SimulationSession(blockedNative, (a, b, c) =>
            { blockedLaunches++; return Task.FromResult(Child("wait", b, c)); }))
        {
            await Reject<InvalidOperationException>(() => blocked.StartAsync(options, scenario), "An open foreign link was accepted");
            Check(blockedLaunches == 0 && blockedNative.Foreign, "Refused startup touched a foreign connection");
        }
        var native = new NativeStub(); int launches = 0;
        using (var session = new SimulationSession(native, (a, b, c) => { launches++; return Task.FromResult(Child("wait", b, c)); }))
        {
            Check(launches == 0 && !session.Status.Active, "Creating a simulation owner launched a dependency");
            await session.StartAsync(options, scenario).ConfigureAwait(false);
            Check(session.Status.Phase == "Running" && native.Opens == 0 && session.Status.CanConnect,
                "Readiness was equated with a native connection");
            Exception uiFailure = null;
            var ui = new Thread(() =>
            {
                try { using (var view = new NOMADSimulationView(session, CancellationToken.None, detectOnOpen: false))
                    { Check(session.Status.Active, "Borrowing a view changed simulator lifetime"); } }
                catch (Exception error) { uiFailure = error; }
            });
            ui.SetApartmentState(ApartmentState.STA); ui.Start();
            Check(ui.Join(5000), "Borrowed view lifecycle exceeded deadline");
            if (uiFailure != null) { throw uiFailure; }
            Check(session.Status.Active, "Disposing a borrowed page stopped its simulator");
            await session.ConnectAsync().ConfigureAwait(false);
            Check(session.Status.Phase == "Connected" && native.Opens == 1, "Explicit native connection did not complete");
            await session.ConnectAsync().ConfigureAwait(false);
            Check(native.Opens == 1, "Duplicate Connect began a second handshake");
            object old = typeof(SimulationSession).GetField("_run", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(session);
            await session.StopAsync().ConfigureAwait(false);
            Check(!session.Status.Active && !native.Connected && Directory.Exists(session.Status.Directory),
                "Stop lost ownership cleanup or its retained diagnostics");
            await session.StartAsync(options, scenario).ConfigureAwait(false);
            var late = typeof(SimulationSession).GetMethods(BindingFlags.Instance | BindingFlags.NonPublic)
                .Single(method => method.Name == "StopAsync" && method.GetParameters().Length == 1);
            await ((Task)late.Invoke(session, new[] { old })).ConfigureAwait(false);
            Check(session.Status.Active && session.Status.Phase == "Running", "An old callback stopped a newer session");
            await session.StopAsync().ConfigureAwait(false);
        }
        var pending = new TaskCompletionSource<ProcessStartInfo>(TaskCreationOptions.RunContinuationsAsynchronously);
        using (var session = new SimulationSession(new NativeStub(), (a, b, c) => pending.Task))
        {
            var starting = session.StartAsync(options, scenario);
            var stopping = session.StopAsync();
            Check(!stopping.IsCompleted && session.Status.Active, "Stop admitted another run before startup drained");
            pending.SetResult(Child("wait", scenario));
            await Task.WhenAll(starting, stopping).ConfigureAwait(false);
            Check(!session.Status.Active && session.Status.Phase == "Stopped", "Canceled preparation launched a late process");
        }
        native = new NativeStub { Hold = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously) };
        options.AutoConnect = true;
        using (var session = new SimulationSession(native, (a, b, c) => Task.FromResult(Child("wait", b, c))))
        {
            var starting = session.StartAsync(options, scenario);
            await Until(() => native.Opens == 1, "Native handshake did not start").ConfigureAwait(false);
            var stopping = session.StopAsync();
            Check(!stopping.IsCompleted && session.Status.Phase == "Stopping", "A dispatched native helper was falsely canceled");
            native.Hold.SetResult(true);
            await Task.WhenAll(starting, stopping).ConfigureAwait(false);
            Check(!native.Connected && session.Status.Phase == "Stopped", "Late native success reconnected after Stop");
        }
        options.AutoConnect = false;
        native = new NativeStub { Hold = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously) };
        options.AutoConnect = true;
        using (var session = new SimulationSession(native, (a, b, c) => Task.FromResult(Child("cleanup-fail", b, c))))
        {
            var starting = session.StartAsync(options, scenario);
            await Until(() => native.Opens == 1, "Fault cleanup native handshake did not start").ConfigureAwait(false);
            var stopping = session.StopAsync();
            await Task.Delay(300).ConfigureAwait(false);
            Check(!stopping.IsCompleted && session.Status.Active, "Cleanup fault skipped the dispatched native helper drain");
            native.Hold.SetResult(true);
            await Task.WhenAll(starting, stopping).ConfigureAwait(false);
            Check(!native.Connected && session.Status.Phase == "Cleanup failed", "Cleanup fault was hidden or reconnected");
        }
        options.AutoConnect = false;
        foreach (string mode in new[] { "bad", "fail" })
        using (var session = new SimulationSession(new NativeStub(), (a, b, c) => Task.FromResult(Child(mode, b, c))))
        {
            if (mode == "bad")
            { await Reject<Newtonsoft.Json.JsonReaderException>(() => session.StartAsync(options, scenario), "Bad readiness passed"); }
            else { await session.StartAsync(options, scenario).ConfigureAwait(false); }
            await Until(() => !session.Status.Active, "Failed backend left an active session").ConfigureAwait(false);
            Check(session.Status.Phase == "Failed", "A failed backend/cleanup was reported as a successful stop");
        }
        using (var session = new SimulationSession(new NativeStub(), (a, b, c) => Task.FromResult(Child("wait", b, c))))
        {
            await session.StartAsync(options, scenario).ConfigureAwait(false);
            session.Dispose();
            await Until(() => !session.Status.Active, "Plugin unload did not clean its owned session").ConfigureAwait(false);
            await Reject<InvalidOperationException>(() => session.StartAsync(options, scenario), "Retired owner started again");
        }
    }

    private static void NativeGuards()
    {
        using (var link = new MAVLinkInterface { BaseStream = new TcpSerial { autoReconnect = false } })
        {
            var list = new List<MAVLinkInterface> { link };
            var connection = new NativeSimulationConnection(() => link, () => list, value => { }, action => Task.CompletedTask);
            link.ReadOnly = true;
            Check(!connection.CanStart(out _), "Read-only target accepted a simulation start");
            link.ReadOnly = false; link.logreadmode = true;
            Check(!connection.CanStart(out _), "Replay target accepted a simulation start");
        }
    }

    private static async Task Wsl(string distro, string workspace, string scenarioPath, string directory, string decoder,
        string toolsDirectory)
    {
        var detected = await SimulationSetup.DetectAsync(distro, workspace);
        Check(detected.Distribution == distro && detected.Workspace == workspace && detected.Prepared &&
            detected.Distributions.Contains(distro) && detected.Python.StartsWith("/"),
            "Actual WSL detection failed or replaced the configured workspace");
        var defaults = await SimulationSetup.DetectAsync("", "");
        Check(defaults.Distributions.Contains(defaults.Distribution) && defaults.Workspace.EndsWith("/nomad-simulation"),
            "Default WSL distribution or home-relative folder was not discovered");
        var query = typeof(SimulationSession).GetMethod("QueryAsync", BindingFlags.Static | BindingFlags.NonPublic);
        string path = Path.Combine(directory, "path with spaces.txt"); File.WriteAllText(path, "owned fixture");
        string converted = await (Task<string>)query.Invoke(null, new object[] { distro, new[] { "wslpath", "-a", path } });
        Check(converted.StartsWith("/") && converted.EndsWith("path with spaces.txt"), "Real WSL path query lost spaces");
        try
        {
            await (Task<string>)query.Invoke(null, new object[] { "nomad-test-missing-" + Guid.NewGuid().ToString("N"),
                new[] { "/bin/echo", "never" } });
            throw new Exception("Missing distribution succeeded");
        }
        catch (IOException error)
        { Check(error.Message.Contains("WSL_E_DISTRO_NOT_FOUND") && !error.Message.Contains("\0"),
            "Real Windows-side WSL failure lost its stdout explanation"); }
        var build = typeof(SimulationSession).GetMethod("BuildCommandAsync", BindingFlags.Static | BindingFlags.NonPublic);
        var command = (Func<SimulationPreferences, SimulationScenario, string, Task<ProcessStartInfo>>)
            Delegate.CreateDelegate(typeof(Func<SimulationPreferences, SimulationScenario, string, Task<ProcessStartInfo>>), build);
        string tools = toolsDirectory.Length > 0 ? toolsDirectory :
            Directory.GetParent(Path.GetDirectoryName(scenarioPath)).FullName;
        var options = new SimulationPreferences { ToolsDirectory = tools, Distribution = distro, Workspace = workspace,
            PythonExecutable = "/usr/bin/python3", ScenarioPath = scenarioPath, Visible = true, AutoConnect = true,
            DurationMinutes = 3 };
        options.Save();
        Exception uiFailure = null;
        var ui = new Thread(() =>
        {
            try
            {
                using (var owner = new SimulationSession(new NativeStub(), (a, b, c) =>
                    throw new InvalidOperationException("Detection must never launch a simulator")))
                using (var view = new NOMADSimulationView(owner, CancellationToken.None))
                {
                    view.CreateControl();
                    var list = (ComboBox)view.Controls.Find("SimulationDistribution", true).Single();
                    var clock = Stopwatch.StartNew();
                    while (list.Items.Count == 0 && clock.ElapsedMilliseconds < 15000)
                    { Application.DoEvents(); Thread.Sleep(10); }
                    Check(list.Items.Contains(distro) && list.Text == distro &&
                        view.Controls.Find("SimulationWorkspace", true).Single().Text == workspace && !owner.Status.Active,
                        "Opening the page did not auto-detect WSL/defaults without launching a simulator");
                }
            }
            catch (Exception error) { uiFailure = error; }
        });
        ui.SetApartmentState(ApartmentState.STA); ui.Start();
        Check(ui.Join(20000), "Automatic page discovery exceeded its UI deadline");
        if (uiFailure != null) { throw uiFailure; }
        MAVLinkInterface current = new MAVLinkInterface { BaseStream = new TcpSerial { autoReconnect = false } };
        var links = new List<MAVLinkInterface> { current };
        var native = new NativeSimulationConnection(() => current, () => links, value => current = value,
            action => { action(); return Task.CompletedTask; });
        using (var session = new SimulationSession(native, command))
        {
            try
            {
                await session.StartAsync(options, SimulationScenario.Load(scenarioPath));
                Check(session.Status.Phase == "Connected" && native.IsConnected,
                    "Production WSL launch/native connection failed: " + session.Status.Phase + ": " + session.Status.Message);
                Check(session.Status.VideoUrl.StartsWith("udp://127.0.0.1:"), "Production renderer did not expose its owned video URL");
                if (decoder.Length > 0) { Decode(session.Status.VideoUrl, directory, decoder); }
                await Timing(current, directory).ConfigureAwait(false);
                File.WriteAllText(Path.Combine(directory, "session-directory.txt"), session.Status.Directory);
            }
            finally { await session.StopAsync(); }
            Check(session.Status.Phase == "Stopped", "Production launch did not stop cleanly: " + session.Status.Message);
        }
        foreach (var link in links) { link.Dispose(); }
    }

    private static async Task Native(string host, int port, string directory, int expectedAhrs)
    {
        MAVLinkInterface current = new MAVLinkInterface { BaseStream = new TcpSerial { autoReconnect = false } };
        var list = new List<MAVLinkInterface> { current };
        var connection = new NativeSimulationConnection(() => current, () => list, value => current = value,
            action => { action(); return Task.CompletedTask; });
        var endpoint = new SimulationEndpoint { Host = IPAddress.Parse(host), MavlinkPort = port, SystemId = 201 };
        await connection.ConnectAsync(endpoint, directory, CancellationToken.None).ConfigureAwait(false);
        Check(connection.IsConnected && list.Contains(current) && current.sysidcurrent == 201 && current.compidcurrent == 1,
            "Actual native handshake did not publish the owned simulator");
        if (expectedAhrs >= 0)
        {
            var read = current.GetParamAsync(201, 1, "AHRS_EKF_TYPE");
            Check(await Task.WhenAny(read, Task.Delay(6000)).ConfigureAwait(false) == read,
                "Native estimator parameter readback exceeded deadline");
            float actual = await read.ConfigureAwait(false);
            Check(actual == expectedAhrs, "Unexpected estimator mode: " + actual);
            File.WriteAllText(Path.Combine(directory, "estimator-readback.txt"), "AHRS_EKF_TYPE=" + actual);
        }
        await Timing(current, directory).ConfigureAwait(false);
        var old = current;
        current.ReadOnly = true;
        connection.CloseOwned();
        Check(!connection.IsConnected && !ReferenceEquals(current, old) && current.BaseStream != null &&
            !current.BaseStream.IsOpen && current.ReadOnly, "Stop restored a disposed placeholder or cleared read-only mode");
        current.ReadOnly = false;
        await connection.ConnectAsync(endpoint, directory, CancellationToken.None).ConfigureAwait(false);
        var owned = current;
        var listener = new TcpListener(IPAddress.Loopback, 0); listener.Start();
        var client = new TcpClient(); var accepted = listener.AcceptTcpClientAsync();
        await client.ConnectAsync(IPAddress.Loopback, ((IPEndPoint)listener.LocalEndpoint).Port);
        using (var peer = await accepted)
        using (var foreign = new TcpSerial { client = client, autoReconnect = false })
        using (var log = new BufferedStream(new MemoryStream()))
        using (var raw = new BufferedStream(new MemoryStream()))
        {
            current.BaseStream = foreign; current.logfile = log; current.rawlogfile = raw;
            connection.CloseOwned();
            Check(ReferenceEquals(current, owned) && foreign.IsOpen && list.Contains(owned) && log.CanWrite && raw.CanWrite,
                "Simulation cleanup closed a replacement native link or its logs");
        }
        listener.Stop();
        foreach (var link in list) { link.Dispose(); }
    }

    private static async Task Timing(MAVLinkInterface link, string directory)
    {
        string enabled = Environment.GetEnvironmentVariable("NOMAD_SIMULATION_TIMING_PROBE");
        if (string.IsNullOrEmpty(enabled)) { return; }
        Check(enabled == "1", "Timing probe must be explicitly enabled with 1");
        var gate = new object();
        var clock = Stopwatch.StartNew();
        var observations = new List<object>();
        var failures = new List<string>();
        const uint required = 1U | (1U << 28); // SYS_STATUS 3D gyro and pre-arm checks.
        bool stopping = false, armed = false;
        double heartbeatAt = -1, statusAt = -1;
        double measurementStart = double.PositiveInfinity;
        uint present = 0, active = 0, healthy = 0;
        int measured = 0;
        Exception readerError = null;
        EventHandler<MAVLink.MAVLinkMessage> received = (sender, packet) =>
        {
            if (packet.sysid != 201 || packet.compid != 1) { return; }
            lock (gate)
            {
                double at = clock.Elapsed.TotalSeconds;
                if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.HEARTBEAT)
                {
                    armed = (packet.ToStructure<MAVLink.mavlink_heartbeat_t>().base_mode & 128) != 0;
                    heartbeatAt = at;
                    observations.Add(new { host_seconds = at, armed });
                    if (armed) { failures.Add("Simulator became armed"); }
                }
                else if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.SYS_STATUS)
                {
                    var status = packet.ToStructure<MAVLink.mavlink_sys_status_t>();
                    present = status.onboard_control_sensors_present; active = status.onboard_control_sensors_enabled;
                    healthy = status.onboard_control_sensors_health; statusAt = at;
                    observations.Add(new { host_seconds = at, present, active, healthy, status.load });
                    if (at >= measurementStart)
                    {
                        measured++;
                        if ((present & active & healthy & required) != required)
                        { failures.Add("Unhealthy gyro/pre-arm status at " + at.ToString("F2")); }
                    }
                }
                else if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.SYSTEM_TIME)
                {
                    var time = packet.ToStructure<MAVLink.mavlink_system_time_t>();
                    observations.Add(new { host_seconds = at, simulation_boot_ms = time.time_boot_ms });
                }
                else if (packet.msgid == (uint)MAVLink.MAVLINK_MSG_ID.STATUSTEXT)
                {
                    var status = packet.ToStructure<MAVLink.mavlink_statustext_t>();
                    string text = Encoding.UTF8.GetString(status.text).TrimEnd('\0');
                    observations.Add(new { host_seconds = at, text });
                    if (at >= measurementStart && (text.Contains("Main loop slow") || text.Contains("rate*1.8")))
                    { failures.Add(text); }
                }
            }
        };
        var reader = new Thread(() =>
        {
            try
            {
                while (!Volatile.Read(ref stopping))
                {
                    if (!link.giveComport && link.BaseStream.BytesToRead > 0) { link.readPacket(); }
                    else { Thread.Sleep(2); }
                }
            }
            catch (Exception error) { readerError = error; }
        }) { IsBackground = true, Name = "Owned native timing probe reader" };
        link.OnPacketReceived += received; reader.Start();
        var parameters = new Dictionary<string, float>();
        try
        {
            foreach (string name in new[] { "SCHED_LOOP_RATE", "ARMING_SKIPCHK" })
            {
                var read = link.GetParamAsync(201, 1, name);
                Check(await Task.WhenAny(read, Task.Delay(6000)).ConfigureAwait(false) == read,
                    "Timing parameter read exceeded deadline: " + name);
                parameters[name] = await read.ConfigureAwait(false);
            }
            Check(parameters["SCHED_LOOP_RATE"] == 400 && parameters["ARMING_SKIPCHK"] == 0,
                "Timing check requires the normal 400 Hz loop with no skipped arming checks");
            foreach (int message in new[] { 1, 2 })
            {
                var stream = link.doCommandAsync(201, 1, MAVLink.MAV_CMD.SET_MESSAGE_INTERVAL,
                    message, 200000, 0, 0, 0, 0, 0, true);
                Check(await Task.WhenAny(stream, Task.Delay(6000)).ConfigureAwait(false) == stream &&
                    await stream.ConfigureAwait(false),
                    "Native timing stream request failed: " + message);
            }
            await Task.Delay(Math.Max(0, 30000 - (int)clock.ElapsedMilliseconds)).ConfigureAwait(false);
            // RUN_PREARM_CHECKS accepts the request even when checks fail; SYS_STATUS is the independent result.
            var check = link.doCommandAsync(201, 1, MAVLink.MAV_CMD.RUN_PREARM_CHECKS, 0, 0, 0, 0, 0, 0, 0, true);
            Check(await Task.WhenAny(check, Task.Delay(6000)).ConfigureAwait(false) == check &&
                await check.ConfigureAwait(false),
                "Native pre-arm check request failed");
            lock (gate) { measurementStart = clock.Elapsed.TotalSeconds; }
            while (clock.Elapsed.TotalSeconds < measurementStart + 30)
            {
                await Task.Delay(200).ConfigureAwait(false);
                lock (gate)
                {
                    double at = clock.Elapsed.TotalSeconds;
                    if (armed || at - heartbeatAt > 3 || at - statusAt > 2)
                    { failures.Add("Unhealthy or stale disarmed gyro/pre-arm status at " + at.ToString("F2")); }
                }
            }
        }
        finally
        {
            Volatile.Write(ref stopping, true);
            bool joined = reader.Join(5000);
            link.OnPacketReceived -= received;
            lock (gate)
            {
                File.WriteAllText(Path.Combine(directory, "timing-health.json"),
                    Newtonsoft.Json.JsonConvert.SerializeObject(
                    new { parameters, warmup_seconds = 30, measurement_seconds = 30, measured, failures, observations,
                        measurement_start_host_seconds = double.IsInfinity(measurementStart)
                            ? (double?)null : measurementStart,
                        reader_error = readerError?.ToString(), reader_exited = joined,
                        clock = "host monotonic receipt",
                        passed = joined && readerError == null && measured > 0 && failures.Count == 0 },
                    Newtonsoft.Json.Formatting.Indented));
            }
            Check(joined, "Timing probe reader did not stop");
        }
        Check(readerError == null && measured > 0 && failures.Count == 0,
            "ArduPilot timing/pre-arm health failed; inspect timing-health.json");
    }

    private static void Camera(string host, int port, string directory, string runtime)
    {
        int videoPort;
        using (var socket = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0)))
        { videoPort = ((IPEndPoint)socket.Client.LocalEndPoint).Port; }
        using (var bridge = new NOMAD.Simulation.RtpBridge(IPAddress.Parse(host), port, videoPort))
        {
            Decode("udp://127.0.0.1:" + videoPort, directory, runtime);
            Check(bridge.Error == null && bridge.Packets > 0, "Encoded simulator bridge failed");
        }
    }

    private static void Decode(string url, string directory, string runtime)
    {
        int seconds = VideoProbeSetting("SECONDS", 30);
        int expectedWidth = VideoProbeSetting("WIDTH", 1920), expectedHeight = VideoProbeSetting("HEIGHT", 1080);
        Check((expectedWidth == 0) == (expectedHeight == 0), "Video probe dimensions must be supplied together");
        string bin = Path.Combine(runtime, "bin");
        Check(File.Exists(Path.Combine(bin, "libgstreamer-1.0-0.dll")), "Existing GStreamer runtime is required");
        GStreamer.BundledPath = runtime;
        Check(string.Equals(GStreamer.LookForGstreamer(), Path.Combine(bin, "libgstreamer-1.0-0.dll"),
            StringComparison.OrdinalIgnoreCase), "Decoder runtime provenance differs");
        Environment.SetEnvironmentVariable("GST_REGISTRY_1_0", Path.Combine(directory, "gst-registry.bin"));
        Environment.SetEnvironmentVariable("GST_PLUGIN_SYSTEM_PATH_1_0", "");
        Environment.SetEnvironmentVariable("GST_PLUGIN_PATH_1_0", Path.Combine(runtime, "lib/gstreamer-1.0"));
        Environment.SetEnvironmentVariable("GST_PLUGIN_SCANNER_1_0", Path.Combine(runtime, "libexec/gstreamer-1.0/gst-plugin-scanner.exe"));
        Environment.SetEnvironmentVariable("PATH", bin + ";" + Environment.GetEnvironmentVariable("PATH"));
        Check(SetDllDirectory(bin), "Cannot select the existing decoder runtime");
        using (var cancellation = new CancellationTokenSource(seconds > 0 ? 45000 : 15000))
        using (IVideoPipeline pipeline = new GStreamerVideoPipeline())
        {
            pipeline.Start(VideoPipeline.Build(new VideoInputProfile { Name = "Owned video probe", Url = url }), cancellation.Token);
            var clock = new Stopwatch();
            var receipts = new List<long>();
            int width = 0, height = 0;
            for (int i = 0; seconds > 0 ? !clock.IsRunning || clock.Elapsed.TotalSeconds < seconds + 2 : i < 3; i++)
            using (var image = pipeline.ReadFrame(cancellation.Token, out long ticks, out DateTime utc))
            {
                Check(image != null && image.Width > 0 && image.Height > 0 && ticks > 0 && utc.Kind == DateTimeKind.Utc,
                    "Rendered RTP did not reach the real NOMAD decoder with receipt clocks");
                if (expectedWidth > 0)
                { Check(image.Width == expectedWidth && image.Height == expectedHeight, "Decoded video dimensions differ"); }
                if (i == 0)
                { width = image.Width; height = image.Height; image.Save(Path.Combine(directory, "video.png")); clock.Start(); }
                if (seconds > 0 && clock.Elapsed.TotalSeconds >= 2) { receipts.Add(ticks); }
            }
            if (seconds > 0)
            {
                double span = receipts.Count > 1 ? (receipts.Last() - receipts.First()) / (double)Stopwatch.Frequency : 0;
                Check(span >= seconds * .8, "Video measurement did not cover its requested interval");
                double fps = (receipts.Count - 1) / span;
                File.WriteAllText(Path.Combine(directory, "video-rate.json"), Newtonsoft.Json.JsonConvert.SerializeObject(
                    new { width, height, frames = receipts.Count, span_seconds = span, decoded_receipt_fps = fps,
                        warmup_seconds = 2, clock = "host monotonic decoded receipt", capture_latency = "not measured" },
                    Newtonsoft.Json.Formatting.Indented));
                Console.WriteLine("Measured decoded video: " + width + "x" + height + " at " + fps.ToString("F2") + " FPS");
            }
        }
    }

    private static int VideoProbeSetting(string suffix, int maximum)
    {
        string name = "NOMAD_SIMULATION_VIDEO_PROBE_" + suffix;
        string raw = Environment.GetEnvironmentVariable(name);
        if (string.IsNullOrEmpty(raw)) { return 0; }
        if (!int.TryParse(raw, out int value) || value < 1 || value > maximum)
        { throw new ArgumentException("Invalid " + name); }
        return value;
    }
}
