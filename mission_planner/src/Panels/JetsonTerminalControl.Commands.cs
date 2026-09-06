// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// JetsonTerminalControl.Commands.cs - Command presets + execution
// ============================================================
// Quick-command presets, the client-side safe-command check, and
// HTTP execution against the Edge Core terminal API. Layout and
// output formatting live in JetsonTerminalControl.cs.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Net.Http;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    public partial class JetsonTerminalControl
    {
        // Quick commands
        private readonly Dictionary<string, string> _quickCommands = new Dictionary<string, string>
        {
            { "System Status", "uptime && free -m && df -h /" },
            { "Tailscale Status", "tailscale status" },
            { "Network Info", "ip addr show | grep -E 'inet |state'" },
            { "Tegrastats (1 sample)", "timeout 2 tegrastats --interval 500 2>&1 | head -3" },
            { "Temperature", "cat /sys/devices/virtual/thermal/thermal_zone*/temp 2>/dev/null | awk '{printf \"Zone %d: %.1fC\\n\", NR-1, $1/1000}'" },
            { "Edge Core Status", "pgrep -f edge_core.main && echo 'Edge Core: Running' || echo 'Edge Core: Not running'" },
            { "Edge Core Logs", "tail -50 ~/nomad_logs/edge_core.log 2>/dev/null || journalctl -u nomad -n 50 --no-pager 2>/dev/null || echo 'No logs found'" },
            { "List Processes", "ps aux --sort=-%cpu | head -15" },
            { "Disk Usage", "df -h / && du -sh ~/NOMAD 2>/dev/null" },
            { "ZED Camera Check", "lsusb | grep -i stereolabs && echo 'ZED Camera: Connected' || echo 'ZED Camera: Not found'" },
            { "Ping Test", "ping -c 3 8.8.8.8" },
            { "Video Stream Check", "pgrep -f gst-launch && echo 'Video Stream: Running' || echo 'Video Stream: Not running'" },
            { "Isaac ROS Status", "docker inspect -f '{{.State.Status}}' nomad_isaac_ros 2>/dev/null || echo 'Container not found'" },
            { "Git Status", "cd ~/NOMAD && git log --oneline -5 && echo '' && git status -s" },
            // Drive systemd directly so unit state stays accurate. Sudoers
            // grants the `mad` user NOPASSWD on these specific commands
            // (infra/systemd/install.sh). Restart/Stop use nohup so the SSH
            // call returns before Edge Core is taken down.
            { "Start NOMAD", "sudo -n systemctl start nomad.target && echo started || echo failed" },
            { "Stop NOMAD", "nohup bash -c 'sleep 2 && sudo -n systemctl stop nomad.target' > /dev/null 2>&1 & echo 'stop scheduled (~2s)'" },
            { "Restart NOMAD", "nohup bash -c 'sleep 2 && ~/NOMAD/scripts/nomad restart all' > /dev/null 2>&1 & echo 'restart scheduled (~2s)'" },
            { "NOMAD Status", "systemctl is-active nomad-edge-core nomad-mavlink-router nomad-mediamtx nomad-isaac-ros-container nomad-zed-wrapper nomad-ros-vehicle nomad-video-bridge nomad-nvblox | paste -d' ' <(echo -e 'edge_core\\nmavlink_router\\nmediamtx\\nisaac_container\\nzed_wrapper\\nros_vehicle\\nvideo_bridge\\nnvblox') -" },
            { "Pull Latest Code", "cd ~/NOMAD && git pull origin main" },
        };
        // ============================================================
        // Command Execution
        // ============================================================

        // SECURITY NOTE: Arbitrary command execution on the Jetson presents a
        // security risk. The Edge Core API should enforce a server-side whitelist
        // of allowed commands. The client-side safe-command list below is for UX
        // convenience only and does NOT replace server-side validation.

        /// <summary>
        /// Commands considered safe and executed without extra confirmation.
        /// Any command not in this set triggers a user confirmation dialog.
        /// </summary>
        private static readonly HashSet<string> _safeCommands = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
        {
            "uptime", "free -m", "df -h /", "df -h", "top -bn1 | head -20",
            "tailscale status", "ip addr show", "ip addr",
            "cat /sys/devices/virtual/thermal/thermal_zone*/temp",
            "pgrep -f edge_core.main", "ps aux --sort=-%cpu | head -15",
            "lsusb", "ping -c 3 8.8.8.8", "whoami", "hostname", "date",
            "cat /proc/uptime", "uname -a", "ls ~/NOMAD", "git -C ~/NOMAD log --oneline -5",
            "ls", "ls -la", "pwd", "cat", "head", "tail", "grep",
        };

        /// <summary>
        /// Check whether a command is considered safe (no confirmation needed).
        /// </summary>
        private bool IsCommandSafe(string command)
        {
            if (string.IsNullOrWhiteSpace(command)) return false;

            // Quick-command entries are always considered safe
            if (_quickCommands.ContainsValue(command)) return true;

            var trimmed = command.Trim();

            // cd commands are always safe (just changing directory)
            if (trimmed == "cd" || trimmed.StartsWith("cd ")) return true;

            // Read-only commands starting with these are safe
            if (trimmed.StartsWith("ls") || trimmed.StartsWith("pwd") ||
                trimmed.StartsWith("cat ") || trimmed.StartsWith("head ") ||
                trimmed.StartsWith("tail ") || trimmed.StartsWith("grep "))
                return true;

            // Exact match against known safe commands
            return _safeCommands.Contains(trimmed);
        }

        public async Task ExecuteCommand(string command)
        {
            if (string.IsNullOrWhiteSpace(command))
                return;

            // SEC3: Warn user before running non-whitelisted commands
            if (!IsCommandSafe(command))
            {
                var confirmResult = MessageBox.Show(
                    $"You are about to execute a command on the Jetson:\n\n  {command}\n\n" +
                    "This command is not in the safe-commands list.\n" +
                    "Only proceed if you trust this command.\n\nContinue?",
                    "Confirm Command Execution",
                    MessageBoxButtons.YesNo,
                    MessageBoxIcon.Warning);

                if (confirmResult != DialogResult.Yes)
                {
                    AppendOutput("\n[Command cancelled by user]\n", Color.Yellow);
                    return;
                }
            }

            // Add to history
            _commandHistory.Add(command);
            _historyIndex = _commandHistory.Count;

            // Show command in output
            AppendOutput($"\n$ {command}\n", Color.LimeGreen);

            // Clear input
            _txtInput.Clear();

            // Update status
            UpdateStatus("Executing...", Color.Yellow);
            _btnExecute.Enabled = false;

            try
            {
                var result = await SendCommand(command);

                if (result.Success)
                {
                    if (!string.IsNullOrEmpty(result.StdOut))
                    {
                        AppendOutput(result.StdOut, Color.LightGray);
                    }
                    if (!string.IsNullOrEmpty(result.StdErr))
                    {
                        AppendOutput(result.StdErr, Color.Orange);
                    }

                    UpdateStatus($"Exit: {result.ReturnCode}", result.ReturnCode == 0 ? Color.LimeGreen : Color.Yellow);
                }
                else
                {
                    // Show stderr if available (API-level error), otherwise show Error (HTTP error)
                    if (!string.IsNullOrEmpty(result.StdErr))
                    {
                        AppendOutput($"Error: {result.StdErr}\n", Color.Red);
                    }
                    else if (!string.IsNullOrEmpty(result.Error))
                    {
                        AppendOutput($"Error: {result.Error}\n", Color.Red);
                    }
                    else
                    {
                        AppendOutput($"Command failed with exit code {result.ReturnCode}\n", Color.Red);
                    }
                    UpdateStatus("Error", Color.Red);
                }
            }
            catch (Exception ex)
            {
                AppendOutput($"Connection error: {ex.Message}\n", Color.Red);
                UpdateStatus("Connection failed", Color.Red);
            }
            finally
            {
                _btnExecute.Enabled = true;
                _txtInput.Focus();
            }
        }

        private async Task<CommandResult> SendCommand(string command)
        {
            var url = $"{_config.EffectiveBaseUrl}/api/terminal/exec";

            var payload = new
            {
                command = command,
                timeout = 30,
                cwd = _currentCwd
            };

            var content = new StringContent(
                JsonConvert.SerializeObject(payload),
                Encoding.UTF8,
                "application/json"
            );

            var response = await JetsonApiService.LongRunClient.PostAsync(url, content);
            var responseBody = await response.Content.ReadAsStringAsync();

            if (response.IsSuccessStatusCode)
            {
                var result = JsonConvert.DeserializeObject<dynamic>(responseBody);

                // Update tracked cwd from response
                string newCwd = result.cwd;
                if (!string.IsNullOrEmpty(newCwd))
                {
                    _currentCwd = newCwd;
                    UpdatePrompt();
                }

                return new CommandResult
                {
                    Success = result.success,
                    StdOut = result.stdout,
                    StdErr = result.stderr,
                    ReturnCode = result.return_code,
                };
            }
            else
            {
                return new CommandResult
                {
                    Success = false,
                    Error = $"HTTP {(int)response.StatusCode}: {response.ReasonPhrase}",
                };
            }
        }

        // ============================================================
        // Command Result
        // ============================================================

        private class CommandResult
        {
            public bool Success { get; set; }
            public string StdOut { get; set; }
            public string StdErr { get; set; }
            public int ReturnCode { get; set; }
            public string Error { get; set; }
        }
    }
}
