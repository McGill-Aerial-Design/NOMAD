// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using NOMAD.MissionPlanner;

namespace NOMAD.Installer
{
    internal static class InstallEngine
    {
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool DeleteFileW(string name);

        internal static string ValidateDirectory(string directory, string reviewedVersion)
        {
            if (string.IsNullOrWhiteSpace(directory))
            {
                throw new IOException("Select the folder containing MissionPlanner.exe.");
            }
            string fullPath = Path.GetFullPath(directory);
            GuardPath(fullPath);
            if (!Directory.Exists(fullPath))
            {
                throw new IOException("The Mission Planner folder does not exist.");
            }
            string host = Path.Combine(fullPath, "MissionPlanner.exe");
            GuardPath(host);
            if (!File.Exists(host))
            {
                throw new IOException("This folder does not contain MissionPlanner.exe.");
            }
            FileVersionInfo info = FileVersionInfo.GetVersionInfo(host);
            var actual = new Version(info.FileMajorPart, info.FileMinorPart, info.FileBuildPart, info.FilePrivatePart);
            string warning = MissionPlannerVersion.GetWarning(actual, reviewedVersion);
            if (warning != null)
            {
                throw new IOException(warning);
            }
            GuardPath(Path.Combine(fullPath, "plugins", "NOMADPlugin.dll"));
            GuardPath(Path.Combine(fullPath, "NOMAD-backups"));
            RequireClosed(host);
            return fullPath;
        }

        internal static string Install(string directory, byte[] payload, string expectedSha256, string reviewedVersion)
        {
            if (payload == null || payload.Length == 0 ||
                !string.Equals(Hash(payload), expectedSha256, StringComparison.OrdinalIgnoreCase))
            {
                throw new IOException("The embedded NOMAD plugin failed its SHA-256 integrity check.");
            }
            string root = ValidateDirectory(directory, reviewedVersion);
            string plugins = Path.Combine(root, "plugins");
            string destination = Path.Combine(plugins, "NOMADPlugin.dll");
            string temporary = Path.Combine(plugins, "NOMADPlugin." + Guid.NewGuid().ToString("N") + ".tmp");
            string backupDirectory = null;
            bool changed = false;
            bool staged = false;
            try
            {
                Directory.CreateDirectory(plugins);
                GuardPath(destination);
                string oldHash = ExistingHash(destination);
                if (oldHash != null)
                {
                    backupDirectory = Path.Combine(root, "NOMAD-backups", DateTime.UtcNow.ToString("yyyyMMdd-HHmmss") +
                        "-" + Guid.NewGuid().ToString("N"));
                    Backup(backupDirectory, destination, oldHash);
                }
                using (var stream = new FileStream(temporary, FileMode.CreateNew, FileAccess.Write, FileShare.None))
                {
                    staged = true;
                    stream.Write(payload, 0, payload.Length);
                    stream.Flush(true);
                }
                Unblock(temporary);
                RequireHash(temporary, expectedSha256);
                ValidateDirectory(root, reviewedVersion);
                GuardPath(temporary);
                if (!string.Equals(oldHash, ExistingHash(destination), StringComparison.OrdinalIgnoreCase))
                {
                    throw new IOException("The existing plugin changed during setup. Run setup again after closing Mission Planner.");
                }
                // Publish verified bytes atomically, then clear any download stream inherited during replacement.
                changed = true;
                if (oldHash == null)
                {
                    File.Move(temporary, destination);
                }
                else
                {
                    File.Replace(temporary, destination, null);
                }
                Unblock(destination);
                RequireHash(destination, expectedSha256);
                return backupDirectory;
            }
            catch (Exception error)
            {
                string state = changed ? "The installed plugin may have changed. " : "Setup did not replace the plugin. ";
                string backup = backupDirectory == null ? "" : "Backup folder: " + backupDirectory + ". ";
                throw new IOException(state + backup + error.Message, error);
            }
            finally
            {
                // Only this invocation's uniquely named staging file belongs to setup.
                if (staged && File.Exists(temporary))
                {
                    try
                    {
                        GuardPath(temporary);
                        File.Delete(temporary);
                    }
                    catch (IOException) { }
                    catch (UnauthorizedAccessException) { }
                }
            }
        }

        private static void Backup(string folder, string source, string oldHash)
        {
            GuardPath(folder);
            Directory.CreateDirectory(folder);
            string backup = Path.Combine(folder, "NOMADPlugin.dll");
            GuardPath(backup);
            File.Copy(source, backup, false);
            RequireHash(backup, oldHash);
        }

        private static void RequireClosed(string host)
        {
            Process[] processes = Process.GetProcessesByName("MissionPlanner");
            try
            {
                foreach (Process process in processes)
                {
                    try
                    {
                        if (string.Equals(process.MainModule.FileName, host, StringComparison.OrdinalIgnoreCase))
                        {
                            throw new IOException("Close Mission Planner from this installation, then run setup again.");
                        }
                    }
                    catch (InvalidOperationException) { }
                    catch (Win32Exception error)
                    {
                        throw new IOException("Cannot check whether Mission Planner is closed. Close it and retry.", error);
                    }
                }
            }
            finally
            {
                foreach (Process process in processes)
                {
                    process.Dispose();
                }
            }
        }

        private static void GuardPath(string path)
        {
            string current = Path.GetFullPath(path);
            while (!string.IsNullOrEmpty(current))
            {
                try
                {
                    if ((File.GetAttributes(current) & FileAttributes.ReparsePoint) != 0)
                    {
                        throw new IOException("Setup cannot use a symbolic link or junction: " + current);
                    }
                }
                catch (FileNotFoundException) { }
                catch (DirectoryNotFoundException) { }
                current = Path.GetDirectoryName(current);
            }
        }

        private static string ExistingHash(string path)
        {
            if (Directory.Exists(path))
            {
                throw new IOException("The plugin destination is a directory: " + path);
            }
            return File.Exists(path) ? Hash(File.ReadAllBytes(path)) : null;
        }

        private static void RequireHash(string path, string expected)
        {
            if (!string.Equals(ExistingHash(path), expected, StringComparison.OrdinalIgnoreCase))
            {
                throw new IOException("File integrity verification failed: " + path);
            }
        }

        private static string Hash(byte[] bytes)
        {
            using (SHA256 algorithm = SHA256.Create())
            {
                return BitConverter.ToString(algorithm.ComputeHash(bytes)).Replace("-", "");
            }
        }

        private static void Unblock(string path)
        {
            if (DeleteFileW(path + ":Zone.Identifier"))
            {
                return;
            }
            int error = Marshal.GetLastWin32Error();
            if (error != 2 && error != 3)
            {
                throw new IOException("Could not remove the downloaded-file block: " + path,
                    new Win32Exception(error));
            }
        }
    }
}
