// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Diagnostics;
using System.IO;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// SSH-based communication methods for NOMAD Edge Core.
    /// Bypasses the HTTP API for operations that might kill it (e.g. service restarts).
    /// </summary>
    public partial class DualLinkSender
    {
        /// <summary>
        /// Execute a command via SSH directly (bypasses HTTP API).
        /// Use this for operations that kill the HTTP API (like service restarts).
        /// Works with SSH key authentication or password authentication.
        /// Shows console window for password entry if keys aren't configured.
        /// </summary>
        public async Task<CommandResult> ExecuteSSHCommandAsync(string command, int timeoutSeconds = 30)
        {
            try
            {
                // Detect OS and find SSH executable
                string sshPath;
                bool isWindows = Environment.OSVersion.Platform == PlatformID.Win32NT;

                if (isWindows)
                {
                    // Windows: Use built-in OpenSSH (Windows 10/11)
                    sshPath = @"C:\Windows\System32\OpenSSH\ssh.exe";
                    if (!System.IO.File.Exists(sshPath))
                    {
                        sshPath = "ssh.exe"; // Fallback to PATH
                    }
                }
                else
                {
                    // Linux/Mac: Use standard ssh location
                    sshPath = "/usr/bin/ssh";
                    if (!System.IO.File.Exists(sshPath))
                    {
                        sshPath = "ssh"; // Fallback to PATH
                    }
                }

                // SSH arguments: allow password prompts, set timeout
                // Note: Removed BatchMode=yes to allow password authentication
                // ConnectTimeout prevents hanging if host is unreachable
                var sshUser = string.IsNullOrWhiteSpace(_config.JetsonSshUser) ? "nomad" : _config.JetsonSshUser;
                var sshArgs = $"-o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new {sshUser}@{_config.EffectiveIP} \"{command}\"";

                var startInfo = new System.Diagnostics.ProcessStartInfo
                {
                    FileName = sshPath,
                    Arguments = sshArgs,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    RedirectStandardInput = false, // Let interactive password prompts pass through
                    UseShellExecute = false,
                    CreateNoWindow = false,  // Show console window for password entry
                    WindowStyle = System.Diagnostics.ProcessWindowStyle.Normal
                };

                using (var process = System.Diagnostics.Process.Start(startInfo))
                {
                    // Use tasks for async I/O
                    var outputTask = process.StandardOutput.ReadToEndAsync();
                    var errorTask = process.StandardError.ReadToEndAsync();

                    // Wait for process to complete with timeout
                    var exited = await Task.Run(() => process.WaitForExit(timeoutSeconds * 1000));

                    if (!exited)
                    {
                        // Race: the process may exit between the timeout check and
                        // Kill(), which raises InvalidOperationException. Swallow
                        // that specific case -- the process is already gone.
                        try { process.Kill(); }
                        catch (InvalidOperationException) { }
                        // Observe the abandoned stdout/stderr reads so they don't surface
                        // as TaskScheduler.UnobservedTaskException once the killed process
                        // closes the pipes. Result is intentionally discarded.
                        _ = outputTask.ContinueWith(t => { var _ = t.Exception; }, TaskContinuationOptions.OnlyOnFaulted);
                        _ = errorTask.ContinueWith(t => { var _ = t.Exception; }, TaskContinuationOptions.OnlyOnFaulted);
                        return new CommandResult
                        {
                            Success = false,
                            Message = $"SSH command timed out after {timeoutSeconds}s (host unreachable or command hung)"
                        };
                    }

                    var output = await outputTask;
                    var error = await errorTask;

                    // SSH returns 0 on success
                    bool success = process.ExitCode == 0;

                    return new CommandResult
                    {
                        Success = success,
                        Data = output,
                        Message = success ? "SSH command executed successfully" : $"SSH failed (exit code {process.ExitCode}): {error}"
                    };
                }
            }
            catch (Exception ex)
            {
                return new CommandResult
                {
                    Success = false,
                    Message = $"SSH execution exception: {ex.Message}"
                };
            }
        }

        /// <summary>
        /// Download one file from the configured Jetson with the platform OpenSSH
        /// scp client. The remote path comes from an SSH listing, not user-entered
        /// command text, and is rejected if it contains shell-control characters.
        /// </summary>
        public async Task<CommandResult> DownloadFileViaScpAsync(
            string remotePath,
            string localPath,
            int timeoutSeconds = 120)
        {
            if (string.IsNullOrWhiteSpace(remotePath) || string.IsNullOrWhiteSpace(localPath))
                return new CommandResult { Success = false, Message = "Remote and local paths are required." };
            if (remotePath.IndexOfAny(new[] { '\r', '\n', '\0', '\'', '"' }) >= 0)
                return new CommandResult { Success = false, Message = "The remote log path contains unsupported characters." };

            try
            {
                string directory = Path.GetDirectoryName(localPath);
                if (!string.IsNullOrWhiteSpace(directory)) Directory.CreateDirectory(directory);

                bool isWindows = Environment.OSVersion.Platform == PlatformID.Win32NT;
                string scpPath = isWindows
                    ? @"C:\Windows\System32\OpenSSH\scp.exe"
                    : "/usr/bin/scp";
                if (!File.Exists(scpPath)) scpPath = isWindows ? "scp.exe" : "scp";

                string sshUser = string.IsNullOrWhiteSpace(_config.JetsonSshUser)
                    ? "nomad"
                    : _config.JetsonSshUser;
                string remote = $"{sshUser}@{_config.EffectiveIP}:'{remotePath}'";
                var startInfo = new ProcessStartInfo
                {
                    FileName = scpPath,
                    Arguments = $"-o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new " +
                        $"{QuoteProcessArgument(remote)} {QuoteProcessArgument(localPath)}",
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    UseShellExecute = false,
                    CreateNoWindow = false,
                    WindowStyle = ProcessWindowStyle.Normal,
                };

                using (var process = Process.Start(startInfo))
                {
                    if (process == null)
                        return new CommandResult { Success = false, Message = "Could not start scp." };
                    var outputTask = process.StandardOutput.ReadToEndAsync();
                    var errorTask = process.StandardError.ReadToEndAsync();
                    bool exited = await Task.Run(() => process.WaitForExit(timeoutSeconds * 1000));
                    if (!exited)
                    {
                        try { process.Kill(); } catch (InvalidOperationException) { }
                        _ = outputTask.ContinueWith(
                            task => { _ = task.Exception; },
                            TaskContinuationOptions.OnlyOnFaulted);
                        _ = errorTask.ContinueWith(
                            task => { _ = task.Exception; },
                            TaskContinuationOptions.OnlyOnFaulted);
                        try { File.Delete(localPath); } catch { }
                        return new CommandResult
                        {
                            Success = false,
                            Message = $"SCP download timed out after {timeoutSeconds}s.",
                        };
                    }

                    string output = await outputTask;
                    string error = await errorTask;
                    bool success = process.ExitCode == 0 && File.Exists(localPath);
                    if (!success)
                    {
                        try { File.Delete(localPath); } catch { }
                    }
                    return new CommandResult
                    {
                        Success = success,
                        Data = output,
                        Message = success ? "Flight log downloaded." : $"SCP failed: {error}",
                    };
                }
            }
            catch (Exception ex)
            {
                return new CommandResult { Success = false, Message = $"SCP download failed: {ex.Message}" };
            }
        }

        /// <summary>
        /// Restart all NOMAD services via SSH (doesn't rely on HTTP API).
        /// Uses the Jetson-side `nomad restart all` hard-reset path, which
        /// stops the autostart set, kills stale launch children, then starts
        /// the autostart set again.
        /// </summary>
        public async Task<CommandResult> RestartAllServicesViaSSHAsync()
        {
            // Run the CLI hard reset in the background so the SSH call can
            // return before Edge Core is stopped.
            var command = "nohup bash -c 'sleep 2 && ${NOMAD_REPO_ROOT:-$HOME/NOMAD}/scripts/nomad restart all' > /dev/null 2>&1 & echo 'restart scheduled (~2s)'";
            return await ExecuteSSHCommandAsync(command, 30);
        }

        private static string QuoteProcessArgument(string value)
            => "\"" + (value ?? "").Replace("\"", "\\\"") + "\"";
    }
}
