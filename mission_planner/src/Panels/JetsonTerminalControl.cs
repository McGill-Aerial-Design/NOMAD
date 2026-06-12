// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Jetson Terminal Control
// ============================================================
// Provides an embedded terminal interface for executing commands
// on the Jetson Orin Nano from within Mission Planner.
// Communicates via HTTP API for security and simplicity.
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
    /// <summary>
    /// Terminal control for executing commands on the Jetson.
    /// Uses the NOMAD Edge Core API for command execution.
    /// </summary>
    public partial class JetsonTerminalControl : UserControl
    {
        // ============================================================
        // Fields
        // ============================================================

        private NOMADConfig _config;
        private readonly List<string> _commandHistory;
        private int _historyIndex;
        private string _currentCwd;  // Tracked working directory on Jetson

        // UI Controls
        private RichTextBox _txtOutput;
        private TextBox _txtInput;
        private Button _btnExecute;
        private Button _btnClear;
        private ComboBox _cmbQuickCommands;
        private Label _lblStatus;
        private Panel _toolbar;
        private Label _promptLabel;

        // Quick-command presets and execution live in JetsonTerminalControl.Commands.cs.

        // ============================================================
        // Constructor
        // ============================================================

        public JetsonTerminalControl(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            _commandHistory = new List<string>();
            _historyIndex = -1;

            InitializeUI();
            PrintWelcome();
        }

        // ============================================================
        // UI Initialization
        // ============================================================

        private void InitializeUI()
        {
            this.BackColor = Color.FromArgb(20, 20, 20);
            this.Dock = DockStyle.Fill;
            this.Padding = new Padding(5);

            var mainPanel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3,
                BackColor = Color.Transparent,
            };

            mainPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 40));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            mainPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 35));

            // Toolbar
            _toolbar = CreateToolbar();
            mainPanel.Controls.Add(_toolbar, 0, 0);

            // Output Area
            _txtOutput = new RichTextBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(12, 12, 12),
                ForeColor = Color.LightGray,
                Font = new Font("Consolas", 10),
                ReadOnly = true,
                BorderStyle = BorderStyle.None,
                ScrollBars = RichTextBoxScrollBars.Both,
                WordWrap = false,
                DetectUrls = false,
            };
            mainPanel.Controls.Add(_txtOutput, 0, 1);

            // Input Area
            var inputPanel = CreateInputPanel();
            mainPanel.Controls.Add(inputPanel, 0, 2);

            this.Controls.Add(mainPanel);
        }

        private Panel CreateToolbar()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(45, 45, 48),
                Padding = new Padding(5, 5, 5, 5),
            };

            // Quick Commands Dropdown
            var lblQuick = new Label
            {
                Text = "Quick:",
                Location = new Point(10, 10),
                ForeColor = Color.LightGray,
                AutoSize = true,
            };
            panel.Controls.Add(lblQuick);

            _cmbQuickCommands = new ComboBox
            {
                Location = new Point(55, 7),
                Size = new Size(180, 25),
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(30, 30, 30),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
            };

            _cmbQuickCommands.Items.Add("-- Quick Commands --");
            foreach (var cmd in _quickCommands.Keys)
            {
                _cmbQuickCommands.Items.Add(cmd);
            }
            _cmbQuickCommands.SelectedIndex = 0;
            _cmbQuickCommands.SelectedIndexChanged += CmbQuickCommands_SelectedIndexChanged;
            panel.Controls.Add(_cmbQuickCommands);

            // Clear Button
            _btnClear = new Button
            {
                Text = "[CLR] Clear",
                Location = new Point(250, 5),
                Size = new Size(70, 28),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(60, 60, 65),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
            };
            _btnClear.Click += (s, e) => ClearOutput();
            panel.Controls.Add(_btnClear);

            // SSH Button (opens external SSH)
            var btnSsh = new Button
            {
                Text = "[SSH]",
                Location = new Point(325, 5),
                Size = new Size(60, 28),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(80, 60, 100),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9),
            };
            btnSsh.Click += (s, e) => OpenSSH();
            panel.Controls.Add(btnSsh);

            // Status Label
            _lblStatus = new Label
            {
                Text = "Ready",
                Location = new Point(400, 10),
                ForeColor = Color.Gray,
                AutoSize = true,
                Font = new Font("Segoe UI", 9),
            };
            panel.Controls.Add(_lblStatus);

            return panel;
        }

        private Panel CreateInputPanel()
        {
            var panel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(30, 30, 30),
                Padding = new Padding(5, 3, 5, 3),
            };

            var promptLabel = new Label
            {
                Text = $"{_config.SshUsername}@jetson:~$",
                Location = new Point(5, 8),
                ForeColor = Color.LimeGreen,
                Font = new Font("Consolas", 10, FontStyle.Bold),
                AutoSize = true,
            };
            _promptLabel = promptLabel;
            panel.Controls.Add(promptLabel);

            _txtInput = new TextBox
            {
                Location = new Point(125, 5),
                Size = new Size(400, 25),
                BackColor = Color.FromArgb(20, 20, 20),
                ForeColor = Color.White,
                Font = new Font("Consolas", 10),
                BorderStyle = BorderStyle.FixedSingle,
            };
            _txtInput.KeyDown += TxtInput_KeyDown;
            panel.Controls.Add(_txtInput);

            _btnExecute = new Button
            {
                Text = "Execute",
                Location = new Point(535, 3),
                Size = new Size(75, 27),
                FlatStyle = FlatStyle.Flat,
                BackColor = NOMADTheme.ACCENT,
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 9, FontStyle.Bold),
            };
            _btnExecute.Click += async (s, e) => await ExecuteCommand(_txtInput.Text);
            panel.Controls.Add(_btnExecute);

            // Resize handling
            panel.Resize += (s, e) =>
            {
                _txtInput.Width = panel.Width - 220;
                _btnExecute.Left = _txtInput.Right + 10;
            };

            return panel;
        }

        // ============================================================
        // Event Handlers
        // ============================================================

        private async void CmbQuickCommands_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (_cmbQuickCommands.SelectedIndex > 0)
            {
                var cmdName = _cmbQuickCommands.SelectedItem.ToString();
                if (_quickCommands.TryGetValue(cmdName, out string command))
                {
                    _txtInput.Text = command;
                    await ExecuteCommand(command);
                }
                _cmbQuickCommands.SelectedIndex = 0;
            }
        }

        private async void TxtInput_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Enter)
            {
                e.SuppressKeyPress = true;
                await ExecuteCommand(_txtInput.Text);
            }
            else if (e.KeyCode == Keys.Up)
            {
                e.SuppressKeyPress = true;
                NavigateHistory(-1);
            }
            else if (e.KeyCode == Keys.Down)
            {
                e.SuppressKeyPress = true;
                NavigateHistory(1);
            }
        }


        // ============================================================
        // Helper Methods
        // ============================================================

        private void PrintWelcome()
        {
            AppendOutput(@"
+===========================================================+
|             NOMAD Jetson Terminal Interface               |
|===========================================================|
|  Target: Jetson Orin Nano                                 |
|  Connection: HTTP API (secure command execution)          |
|                                                           |
|  Use quick commands dropdown for common operations.       |
|  Type commands below and press Enter to execute.          |
|                                                           |
|  [!] Commands run with API user permissions.              |
|  [!] All commands are now enabled in production mode.     |
+===========================================================+

", Color.Cyan);

            AppendOutput($"Jetson endpoint: {_config.EffectiveBaseUrl}\n", Color.Gray);
            AppendOutput("Type 'help' or select a quick command to get started.\n\n", Color.Gray);
        }

        private void AppendOutput(string text, Color color)
        {
            UiAsync.RunSync(this, () =>
            {
                _txtOutput.SelectionStart = _txtOutput.TextLength;
                _txtOutput.SelectionLength = 0;
                _txtOutput.SelectionColor = color;
                _txtOutput.AppendText(text);
                _txtOutput.SelectionColor = _txtOutput.ForeColor;

                _txtOutput.SelectionStart = _txtOutput.TextLength;
                _txtOutput.ScrollToCaret();
            }, "AppendOutput");
        }

        private void ClearOutput()
        {
            _txtOutput.Clear();
            PrintWelcome();
        }

        private void UpdatePrompt()
        {
            if (_promptLabel == null) return;
            UiAsync.RunSync(this, () =>
            {
                string displayPath = _currentCwd ?? "~";
                string homePrefix = $"/home/{_config.SshUsername}";
                if (displayPath.StartsWith(homePrefix))
                    displayPath = "~" + displayPath.Substring(homePrefix.Length);

                _promptLabel.Text = $"{_config.SshUsername}@jetson:{displayPath}$";
            }, "UpdatePrompt");
        }

        private void UpdateStatus(string status, Color color)
        {
            UiAsync.RunSync(this, () =>
            {
                _lblStatus.Text = status;
                _lblStatus.ForeColor = color;
            }, "UpdateStatus");
        }

        private void NavigateHistory(int direction)
        {
            if (_commandHistory.Count == 0)
                return;

            _historyIndex += direction;

            if (_historyIndex < 0)
                _historyIndex = 0;
            else if (_historyIndex >= _commandHistory.Count)
                _historyIndex = _commandHistory.Count - 1;

            if (_historyIndex >= 0 && _historyIndex < _commandHistory.Count)
            {
                _txtInput.Text = _commandHistory[_historyIndex];
                _txtInput.SelectionStart = _txtInput.Text.Length;
            }
        }

        private void OpenSSH()
        {
            try
            {
                var jetsonIp = _config.UseTailscale ? _config.TailscaleIP : _config.JetsonIP;
                var sshCommand = $"ssh {_config.SshUsername}@{jetsonIp}";

                // Try Windows Terminal first
                try
                {
                    System.Diagnostics.Process.Start("wt.exe", sshCommand);
                    return;
                }
                catch { }

                // Fall back to cmd with ssh
                System.Diagnostics.Process.Start("cmd.exe", $"/c start cmd /k {sshCommand}");
            }
            catch (Exception ex)
            {
                var jetsonIp = _config.UseTailscale ? _config.TailscaleIP : _config.JetsonIP;
                MessageBox.Show(
                    $"Could not open SSH session.\n\n" +
                    $"IP: {jetsonIp}\n" +
                    $"Error: {ex.Message}\n\n" +
                    $"Try running manually:\n  ssh {_config.SshUsername}@{jetsonIp}",
                    "SSH Error",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Warning
                );
            }
        }

        public void UpdateConfig(NOMADConfig config)
        {
            _config = config;
            AppendOutput($"\n[Config updated: {_config.EffectiveBaseUrl}]\n", Color.Yellow);
        }


        // ============================================================
        // Cleanup
        // ============================================================

        protected override void Dispose(bool disposing)
        {
            base.Dispose(disposing);
        }
    }
}
