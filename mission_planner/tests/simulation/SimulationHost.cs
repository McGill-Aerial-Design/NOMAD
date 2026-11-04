// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Web.Script.Serialization;
using System.Xml;

namespace NOMAD.Simulation
{
    internal static class SimulationHost
    {
        [STAThread]
        private static int Main(string[] args)
        {
            try
            {
                if (args.Length != 2 || Array.IndexOf(new[] { "--probe", "--native", "--rendered", "--replay",
                    "--job-child", "--detect", "--occlusion" }, args[0]) < 0)
                { throw new ArgumentException("Expected a supported test mode and an owned run directory."); }
                string run = RequireRun(args[1]);
                string host = Path.Combine(run, "host");
                RequireConfig(Path.Combine(host, "MissionPlanner.exe.config"), run);
                RequireConfig(Path.Combine(host, "VideoLifetimeTests.exe.config"), run);
                if (args[0] == "--job-child") { System.Threading.Thread.Sleep(30000); return 1; }
                var setup = new AppDomainSetup { ApplicationBase = host,
                    ConfigurationFile = Path.Combine(host, "MissionPlanner.exe.config") };
                var domain = AppDomain.CreateDomain("NOMAD simulation probe", null, setup);
                try
                {
                    var probe = (HostProbe)domain.CreateInstanceFromAndUnwrap(
                        Assembly.GetExecutingAssembly().Location, typeof(HostProbe).FullName);
                    probe.Run(run, args[0]);
                }
                finally { AppDomain.Unload(domain); }
                return 0;
            }
            catch (Exception error)
            {
                Console.Error.WriteLine(error.GetType().Name + ": " + error.Message);
                return 1;
            }
        }

        internal static string RequireRun(string value)
        {
            if (!Path.IsPathRooted(value)) { throw new ArgumentException("Run directory must be absolute."); }
            string run = Path.GetFullPath(value).TrimEnd(Path.DirectorySeparatorChar);
            string name = Path.GetFileName(run);
            Guid id;
            if (!name.StartsWith("run-", StringComparison.Ordinal) ||
                !Guid.TryParseExact(name.Substring(4), "N", out id) ||
                !string.Equals(Directory.GetParent(AppDomain.CurrentDomain.BaseDirectory.TrimEnd(
                    Path.DirectorySeparatorChar)).FullName, run, StringComparison.OrdinalIgnoreCase))
            { throw new ArgumentException("Executable must be inside its owned run/host directory."); }
            for (var parent = new DirectoryInfo(run); parent != null; parent = parent.Parent)
            {
                if ((parent.Attributes & FileAttributes.ReparsePoint) != 0)
                { throw new IOException("Run directory cannot use a reparse path."); }
            }
            if (File.ReadAllText(Path.Combine(run, "run.id")).Trim() != id.ToString("N"))
            { throw new InvalidDataException("Run ownership marker does not match."); }
            return run;
        }

        private static void RequireConfig(string path, string run)
        {
            var config = new XmlDocument { XmlResolver = null };
            config.Load(path);
            foreach (XmlElement sink in config.SelectNodes(
                "//log4net/appender/file[@value] | //system.diagnostics//*[@initializeData]"))
            {
                string value = sink.HasAttribute("initializeData") ? sink.GetAttribute("initializeData") :
                    sink.GetAttribute("value");
                if (!Path.IsPathRooted(value) || !Path.GetFullPath(value).StartsWith(run + "\\",
                    StringComparison.OrdinalIgnoreCase))
                { throw new InvalidDataException("Host file logging escaped the owned run."); }
            }
        }
    }

    public sealed class HostProbe : MarshalByRefObject
    {
        public override object InitializeLifetimeService() => null;

        public void Run(string run, string mode)
        {
            string host = Path.Combine(run, "host");
            AppDomain.CurrentDomain.AssemblyResolve += (sender, request) =>
            {
                string name = new AssemblyName(request.Name).Name;
                string path = Path.Combine(host, name == "NOMADPlugin" ? "plugins/NOMADPlugin.dll" :
                    name == "MissionPlanner" ? name + ".exe" : name + ".dll");
                return File.Exists(path) ? Assembly.LoadFrom(path) : null;
            };
            Prepare(run);
            if (mode == "--replay") { ReplayFixture.Run(run); }
            else if (mode != "--probe")
            { NativeFixture.Run(run, mode != "--native", mode == "--detect", mode == "--occlusion"); }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static void Prepare(string run)
        {
            string id = Path.GetFileName(run).Substring(4);
            string name = "NOMAD-Simulation-" + id;
            var settings = typeof(global::MissionPlanner.Utilities.Settings);
            global::MissionPlanner.Utilities.Settings.CustomUserDataDirectory = Path.Combine(run, "user");
            global::MissionPlanner.Utilities.Settings.AppConfigName = name;
            global::MissionPlanner.Utilities.Settings.FileName = Path.Combine(run, "mp.config.xml");
            string data = global::MissionPlanner.Utilities.Settings.GetDataDirectory();
            string user = global::MissionPlanner.Utilities.Settings.GetUserDataDirectory();
            string expectedData = Path.Combine(Environment.GetFolderPath(
                Environment.SpecialFolder.CommonApplicationData), name);
            if (Path.GetFullPath(data).TrimEnd('\\') != Path.GetFullPath(expectedData) ||
                !Path.GetFullPath(user).StartsWith(Path.Combine(run, "user") + "\\",
                    StringComparison.OrdinalIgnoreCase) ||
                !settings.Assembly.Location.StartsWith(Path.Combine(run, "host") + "\\",
                    StringComparison.OrdinalIgnoreCase))
            { throw new InvalidOperationException("Reviewed host paths escaped isolation."); }

            // Load settings only after FileName is absolute and owned; never start MP here.
            if (global::MissionPlanner.Utilities.Settings.Instance["AutoConnect"] != "[]")
            { throw new InvalidDataException("Automatic connections must be disabled."); }
            var plugin = Assembly.LoadFrom(Path.Combine(run, "host", "plugins", "NOMADPlugin.dll"));
            var manifest = new JavaScriptSerializer().Deserialize<Dictionary<string, object>>(
                File.ReadAllText(Path.Combine(run, "manifest.json")));
            var identity = plugin.GetType("NOMAD.MissionPlanner.NomadRelease", true);
            if ((string)identity.GetField("SourceSha").GetRawConstantValue() != (string)manifest["source"] ||
                (string)identity.GetField("MissionPlannerTarget").GetRawConstantValue() != "1.3.83")
            { throw new InvalidDataException("Build the current reviewed source before running the fixture."); }
            var storage = plugin.GetType("NOMAD.MissionPlanner.NOMADStorage", true);
            var flags = BindingFlags.Static | BindingFlags.NonPublic;
            string profiles = (string)storage.GetProperty("PluginsDirectory", flags).GetValue(null);
            if (!Path.GetFullPath(profiles).StartsWith(Path.Combine(run, "user") + "\\",
                StringComparison.OrdinalIgnoreCase))
            { throw new InvalidOperationException("Plugin storage escaped isolation."); }
            var result = new Dictionary<string, object> {
                { "schema", 1 }, { "passed", true }, { "case", "early-storage-probe" },
                { "host_started", false }, { "native_connection_opened", false },
                { "metadata", data }, { "user_data", user }, { "plugin_data", profiles },
                { "host_assembly", settings.Assembly.Location },
                { "full_host_lifecycle", "unavailable: unconditional HTTP/mDNS startup requires isolation" }
            };
            File.WriteAllText(Path.Combine(run, "probe.json"), new JavaScriptSerializer().Serialize(result));
            Console.WriteLine("PASS owned early host settings; MP startup not invoked");
        }
    }
}
