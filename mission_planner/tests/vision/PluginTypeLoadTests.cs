// SPDX-License-Identifier: Apache-2.0
using System;
using System.IO;
using System.Linq;
using System.Reflection;

internal static class PluginTypeLoadTests
{
    private static int Main(string[] args)
    {
        try
        {
            string host = args[1];
            AppDomain.CurrentDomain.AssemblyResolve += (sender, request) =>
            {
                string name = new AssemblyName(request.Name).Name;
                foreach (string extension in new[] { ".dll", ".exe" })
                {
                    string path = Path.Combine(host, name + extension);
                    if (File.Exists(path)) { return Assembly.LoadFrom(path); }
                }
                return null;
            };
            var types = Assembly.LoadFrom(args[0]).GetTypes();
            if (!types.Any(type => type.Name == "NOMADPlugin") ||
                !types.Any(type => type.Name == "CsrtVisualTracker"))
            { throw new Exception("Expected plugin and visual tracker types are missing."); }
            if (AppDomain.CurrentDomain.GetAssemblies().Any(assembly =>
                assembly.GetName().Name == "OpenCvSharp" || assembly.GetName().Name == "Microsoft.ML.OnnxRuntime"))
            { throw new Exception("Plugin type discovery must not load either lazy vision runtime."); }
            Console.WriteLine("PASS: fresh CLR discovered " + types.Length +
                " plugin types with only the main DLL and host references; both vision runtimes remain unloaded.");
            return 0;
        }
        catch (ReflectionTypeLoadException error)
        {
            foreach (var loaderError in error.LoaderExceptions) { Console.Error.WriteLine(loaderError.Message); }
            return 1;
        }
        catch (Exception error) { Console.Error.WriteLine(error); return 1; }
    }
}
