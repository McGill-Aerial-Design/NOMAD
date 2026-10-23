// SPDX-License-Identifier: Apache-2.0
using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Forms;

namespace NOMAD.Conference
{
    public static class Screenshot
    {
        // Bind the framework drawing types here; MP also loads a drawing shim into IronPython's namespace.
        public static void Save(Control control, Bitmap frame, string path)
        {
            frame.Save(path + "-camera.png", ImageFormat.Png);
            using (var image = new Bitmap(control.ClientSize.Width, control.ClientSize.Height))
            {
                control.DrawToBitmap(image, new Rectangle(Point.Empty, control.ClientSize));
                image.Save(path + ".png", ImageFormat.Png);
            }
        }
    }

    internal static class Program
    {
        [STAThread]
        private static int Main(string[] args)
        {
            try
            {
                if (args.Length < 2) { throw new ArgumentException("Usage: NOMADHost.exe MP_EXECUTABLE RUN_DIRECTORY [MP_ARGUMENTS]"); }
                string host = Path.GetFullPath(args[0]);
                string run = Path.GetFullPath(args[1]);
                if (!File.Exists(host) || !string.Equals(Path.GetFileName(host), "MissionPlanner.exe", StringComparison.OrdinalIgnoreCase))
                { throw new ArgumentException("The owned MissionPlanner.exe is required."); }
                Directory.CreateDirectory(run);
                var setup = new AppDomainSetup { ApplicationBase = Path.GetDirectoryName(host),
                    ConfigurationFile = host + ".config" };
                var domain = AppDomain.CreateDomain("NOMAD owned Mission Planner", null, setup);
                try
                {
                    var runner = (HostRunner)domain.CreateInstanceFromAndUnwrap(
                        Assembly.GetExecutingAssembly().Location, typeof(HostRunner).FullName);
                    return runner.Run(host, run, args.Skip(2).ToArray());
                }
                finally { AppDomain.Unload(domain); }
            }
            catch (Exception error) { Console.Error.WriteLine(error); return 1; }
        }
    }

    public sealed class HostRunner : MarshalByRefObject
    {
        public override object InitializeLifetimeService() => null;

        public int Run(string executable, string directory, string[] arguments)
        {
            string host = Path.GetDirectoryName(executable);
            AppDomain.CurrentDomain.AssemblyResolve += (sender, request) =>
            {
                string name = new AssemblyName(request.Name).Name;
                string path = Path.Combine(host, name == "MissionPlanner" ? name + ".exe" : name + ".dll");
                return File.Exists(path) ? Assembly.LoadFrom(path) : null;
            };
            PrepareSettings(directory);
            Directory.SetCurrentDirectory(host);
            return AppDomain.CurrentDomain.ExecuteAssembly(executable, arguments);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static void PrepareSettings(string directory)
        {
            global::MissionPlanner.Utilities.Settings.CustomUserDataDirectory = directory;
            global::MissionPlanner.Utilities.Settings.AppConfigName = "NOMADConference_" +
                Path.GetFileName(directory.TrimEnd(Path.DirectorySeparatorChar));
            global::MissionPlanner.Utilities.Settings.FileName = Path.Combine(directory, "mp.config.xml");
            global::MissionPlanner.Utilities.GStreamer.BundledPath = Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.CommonApplicationData), "Mission Planner", "gstreamer");
        }
    }
}
