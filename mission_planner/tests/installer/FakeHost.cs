// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System.IO;
using System.Reflection;
using System.Threading;
#if WRONG_HOST
[assembly: AssemblyFileVersion("1.3.82.0")]
#else
[assembly: AssemblyFileVersion("1.3.83.0")]
#endif
public static class FakeHost
{
    public static string Probe() { return "Plugin assembly loaded"; }

    public static void Main(string[] args)
    {
        if (args.Length == 0) { return; }
        File.WriteAllText(args[0] + ".ready", "ready");
        for (int count = 0; count < 200 && !File.Exists(args[0]); count++)
        {
            Thread.Sleep(50);
        }
    }
}
