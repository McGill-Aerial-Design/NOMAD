// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Diagnostics;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using Microsoft.Win32.SafeHandles;
using NOMAD.Installer;

internal static class InstallerTests
{
    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    private static extern SafeFileHandle CreateFileW(string name, uint access, uint share, IntPtr security,
        uint creation, uint attributes, IntPtr template);
    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    private static extern uint GetFileAttributesW(string name);
    private static string fixtures;
    private static string workspace;
    private static byte[] payload;
    private static string digest;
    private static int cases;

    private static int Main(string[] args)
    {
        if (args.Length == 2 && args[0] == "load")
        {
            Assembly loaded = Assembly.LoadFrom(args[1]);
            Require((string)loaded.GetType("FakeHost").GetMethod("Probe").Invoke(null, null) ==
                "Plugin assembly loaded", "Installed assembly did not execute in fresh process");
            return 0;
        }
        fixtures = AppDomain.CurrentDomain.BaseDirectory;
        workspace = Path.Combine(Path.GetTempPath(), "NOMAD-installer-tests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(workspace);
        payload = File.ReadAllBytes(Path.Combine(fixtures, "MissionPlanner.exe"));
        using (SHA256 algorithm = SHA256.Create())
        {
            digest = BitConverter.ToString(algorithm.ComputeHash(payload)).Replace("-", "");
        }
        try
        {
            Fresh();
            Update();
            InvalidInputs();
            RunningHost();
            LockedPlugin();
            ReparsePaths();
            Console.WriteLine("Installer filesystem tests passed: " + cases);
            return 0;
        }
        finally
        {
            Directory.Delete(workspace, true);
        }
    }

    private static string Host(string name)
    {
        string root = Path.Combine(workspace, name);
        Directory.CreateDirectory(root);
        File.Copy(Path.Combine(fixtures, "MissionPlanner.exe"), Path.Combine(root, "MissionPlanner.exe"));
        File.WriteAllText(Path.Combine(root, "config.xml"), "operator settings stay exactly here");
        return root;
    }

    private static string Destination(string root) { return Path.Combine(root, "plugins", "NOMADPlugin.dll"); }
    private static string Install(string root) { return InstallEngine.Install(root, payload, digest, "1.3.83"); }

    private static void Fresh()
    {
        string root = Host("fresh");
        Require(Install(root) == null, "Fresh install unexpectedly has a backup");
        Require(Convert.ToBase64String(File.ReadAllBytes(Destination(root))) == Convert.ToBase64String(payload),
            "Fresh installed bytes differ");
        VerifyUntouched(root);
        cases++;
    }

    private static void Update()
    {
        string root = Host("update");
        Directory.CreateDirectory(Path.Combine(root, "plugins"));
        string target = Destination(root);
        File.WriteAllText(target, "previous plugin bytes");
        Block(target);
        string backup = Install(root);
        Require(File.ReadAllText(Path.Combine(backup, "NOMADPlugin.dll")) == "previous plugin bytes",
            "Backup did not preserve previous bytes");
        Require(!backup.StartsWith(Path.Combine(root, "plugins"), StringComparison.OrdinalIgnoreCase),
            "Backup is inside active plugin directory");
        Require(GetFileAttributesW(target + ":Zone.Identifier") == uint.MaxValue,
            "Installed plugin retains download block");
        using (Process probe = Process.Start(new ProcessStartInfo(Assembly.GetExecutingAssembly().Location,
            "load \"" + target + "\"") { UseShellExecute = false, CreateNoWindow = true }))
        {
            Require(probe.WaitForExit(10000) && probe.ExitCode == 0, "Fresh process failed to load installed assembly");
        }
        Block(target);
        string identicalBackup = Install(root);
        Require(identicalBackup != backup && File.Exists(Path.Combine(identicalBackup, "NOMADPlugin.dll")),
            "Identical install did not preserve a separate backup");
        Require(GetFileAttributesW(target + ":Zone.Identifier") == uint.MaxValue,
            "Identical payload remained blocked");
        VerifyUntouched(root);
        cases += 2;
    }

    private static void InvalidInputs()
    {
        string root = Host("invalid");
        ExpectFailure(() => InstallEngine.Install(root, payload, "bad", "1.3.83"), "integrity");
        Require(!Directory.Exists(Path.Combine(root, "plugins")), "Bad payload mutated host folder");
        File.Copy(Path.Combine(fixtures, "WrongHost.exe"), Path.Combine(root, "MissionPlanner.exe"), true);
        ExpectFailure(() => Install(root), "Untested Mission Planner version");
        File.Delete(Path.Combine(root, "MissionPlanner.exe"));
        ExpectFailure(() => Install(root), "does not contain");
        VerifyUntouched(root);
        cases += 3;
    }

    private static void RunningHost()
    {
        string root = Host("running");
        string stop = Path.Combine(root, "stop");
        using (Process host = Process.Start(new ProcessStartInfo(Path.Combine(root, "MissionPlanner.exe"),
            "\"" + stop + "\"") { UseShellExecute = false, CreateNoWindow = true }))
        {
            try
            {
                var deadline = Stopwatch.StartNew();
                while (!File.Exists(stop + ".ready") && deadline.ElapsedMilliseconds < 5000)
                {
                    Thread.Sleep(20);
                }
                Require(File.Exists(stop + ".ready"), "Owned fake host did not start");
                ExpectFailure(() => Install(root), "Close Mission Planner from this installation");
                Require(!Directory.Exists(Path.Combine(root, "plugins")), "Running-host rejection wrote plugin");
                Require(!host.HasExited, "Setup terminated the host");
            }
            finally
            {
                File.WriteAllText(stop, "stop");
                Require(host.WaitForExit(10000), "Owned fake host did not exit cooperatively");
            }
        }
        cases++;
    }

    private static void LockedPlugin()
    {
        string root = Host("locked");
        Directory.CreateDirectory(Path.Combine(root, "plugins"));
        File.WriteAllText(Destination(root), "locked old plugin");
        using (var locked = new FileStream(Destination(root), FileMode.Open, FileAccess.Read, FileShare.Read))
        {
            ExpectFailure(() => Install(root), "Backup folder:");
            Require(File.ReadAllText(Destination(root)) == "locked old plugin", "Locked plugin changed");
        }
        Require(Directory.GetFiles(Path.Combine(root, "plugins")).Length == 1, "Setup left staging files");
        VerifyUntouched(root);
        cases++;
    }

    private static void ReparsePaths()
    {
        string outside = Path.Combine(workspace, "outside");
        Directory.CreateDirectory(outside);
        foreach (string redirected in new[] { "plugins", "NOMAD-backups", "plugins/NOMADPlugin.dll", "selected-folder" })
        {
            string root = Host("junction-" + redirected.Replace('/', '-'));
            string junction = Path.Combine(root, redirected);
            Directory.CreateDirectory(Path.GetDirectoryName(junction));
            using (Process create = Process.Start(new ProcessStartInfo("cmd.exe",
                "/d /c mklink /J \"" + junction + "\" \"" + outside + "\"")
                { UseShellExecute = false, CreateNoWindow = true, RedirectStandardOutput = true }))
            {
                Require(create.WaitForExit(5000) && create.ExitCode == 0, "Could not create owned junction fixture");
            }
            try
            {
                string selected = redirected == "selected-folder" ? junction : root;
                ExpectFailure(() => Install(selected), "symbolic link or junction");
                Require(Directory.GetFileSystemEntries(outside).Length == 0, "Setup wrote through junction");
            }
            finally
            {
                Directory.Delete(junction);
            }
            cases++;
        }
    }

    private static void VerifyUntouched(string root)
    {
        Require(File.ReadAllText(Path.Combine(root, "config.xml")) == "operator settings stay exactly here",
            "Setup changed operator settings");
        Require(Directory.GetFiles(root, "*.tmp", SearchOption.AllDirectories).Length == 0, "Staging file leaked");
    }

    private static void Block(string path)
    {
        using (SafeFileHandle handle = CreateFileW(path + ":Zone.Identifier", 0x40000000, 0, IntPtr.Zero,
            2, 0x80, IntPtr.Zero))
        {
            Require(!handle.IsInvalid, "Could not create Zone.Identifier fixture");
            using (var stream = new FileStream(handle, FileAccess.Write))
            {
                byte[] bytes = Encoding.ASCII.GetBytes("[ZoneTransfer]\r\nZoneId=3\r\n");
                stream.Write(bytes, 0, bytes.Length);
            }
        }
        Require(GetFileAttributesW(path + ":Zone.Identifier") != uint.MaxValue,
            "Zone.Identifier fixture is missing");
    }

    private static void ExpectFailure(Action action, string message)
    {
        try { action(); }
        catch (IOException error)
        {
            Require(error.Message.Contains(message), "Unexpected failure: " + error.Message);
            return;
        }
        throw new Exception("Expected failure containing: " + message);
    }

    private static void Require(bool condition, string message)
    {
        if (!condition) { throw new Exception(message); }
    }
}
