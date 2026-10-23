// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Windows.Forms;

namespace NOMAD.Installer
{
    internal static class Program
    {
        [STAThread]
        private static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new InstallerForm());
        }

        internal static byte[] Resource(string name)
        {
            using (Stream stream = Assembly.GetExecutingAssembly().GetManifestResourceStream(name))
            using (var bytes = new MemoryStream())
            {
                if (stream == null) { throw new IOException("Setup is incomplete. Download it again."); }
                stream.CopyTo(bytes);
                return bytes.ToArray();
            }
        }
    }

    internal sealed class InstallerForm : Form
    {
        private readonly TextBox folder = new TextBox { Dock = DockStyle.Fill };
        private readonly TextBox status = new TextBox { Dock = DockStyle.Fill, Multiline = true,
            ReadOnly = true, BorderStyle = BorderStyle.None, BackColor = SystemColors.Control,
            ScrollBars = ScrollBars.Vertical, TabStop = false };
        private readonly Button install = new Button { Text = "Install", AutoSize = true };
        private readonly Button browse = new Button { Text = "Browse...", AutoSize = true };
        private readonly Button close = new Button { Text = "Cancel", AutoSize = true };

        public InstallerForm()
        {
            Text = "NOMAD Setup";
            Font = new Font("Segoe UI", 9F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(590, 285);
            FormBorderStyle = FormBorderStyle.FixedDialog;
            MaximizeBox = false;
            MinimizeBox = false;
            StartPosition = FormStartPosition.CenterScreen;
            var layout = new TableLayoutPanel { Dock = DockStyle.Fill, Padding = new Padding(20), ColumnCount = 3 };
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            AddRow(layout, new Label { Text = "Install NOMAD " + SetupInfo.Version, AutoSize = true,
                Font = new Font(Font.FontFamily, 17F, FontStyle.Bold), Margin = new Padding(0, 0, 0, 8) });
            AddRow(layout, new Label { Text = "For Mission Planner " + SetupInfo.ReviewedVersion,
                AutoSize = true, Margin = new Padding(0, 0, 0, 18) });
            AddRow(layout, new Label { Text = "Mission Planner folder", AutoSize = true });
            int pathRow = layout.RowCount++;
            layout.Controls.Add(folder, 0, pathRow);
            layout.SetColumnSpan(folder, 2);
            layout.Controls.Add(browse, 2, pathRow);
            status.Text = "Close Mission Planner before installing. Your aircraft profiles are preserved.";
            status.Margin = new Padding(0, 14, 0, 14);
            AddRow(layout, status);
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            var actions = new FlowLayoutPanel { FlowDirection = FlowDirection.RightToLeft,
                AutoSize = true, Dock = DockStyle.Fill, WrapContents = false };
            actions.Controls.Add(close);
            actions.Controls.Add(install);
            var license = new LinkLabel { Text = "License", AutoSize = true, Margin = new Padding(0, 7, 20, 0) };
            license.LinkClicked += delegate { ShowLicense(); };
            actions.Controls.Add(license);
            AddRow(layout, actions);
            Controls.Add(layout);
            folder.Text = FindInstallation();
            browse.Click += delegate { Browse(); };
            install.Click += delegate { Install(); };
            close.Click += delegate { Close(); };
            AcceptButton = install;
            CancelButton = close;
        }

        private static void AddRow(TableLayoutPanel layout, Control control)
        {
            int row = layout.RowCount++;
            layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            layout.Controls.Add(control, 0, row);
            layout.SetColumnSpan(control, 3);
        }

        private static string FindInstallation()
        {
            foreach (var special in new[] { Environment.SpecialFolder.ProgramFilesX86, Environment.SpecialFolder.ProgramFiles })
            {
                string root = Environment.GetFolderPath(special);
                if (string.IsNullOrEmpty(root)) { continue; }
                string candidate = Path.Combine(root, "Mission Planner");
                if (File.Exists(Path.Combine(candidate, "MissionPlanner.exe"))) { return candidate; }
            }
            return "";
        }

        private void Browse()
        {
            using (var dialog = new FolderBrowserDialog { Description = "Select the folder containing MissionPlanner.exe",
                SelectedPath = folder.Text, ShowNewFolderButton = false })
            {
                if (dialog.ShowDialog(this) == DialogResult.OK) { folder.Text = dialog.SelectedPath; }
            }
        }

        private void Install()
        {
            install.Enabled = browse.Enabled = close.Enabled = folder.Enabled = false;
            UseWaitCursor = true;
            try
            {
                string backup = InstallEngine.Install(folder.Text.Trim(), Program.Resource("NOMAD.Payload.dll"),
                    SetupInfo.PayloadSha256, SetupInfo.ReviewedVersion);
                status.Text = "Installed. You can now open Mission Planner." +
                    (backup == null ? "" : "\nPrevious DLL backup: " + backup);
                close.Text = "Close";
            }
            catch (Exception error)
            {
                status.Text = error.Message;
                install.Enabled = browse.Enabled = folder.Enabled = true;
            }
            finally
            {
                close.Enabled = true;
                UseWaitCursor = false;
            }
        }

        private void ShowLicense()
        {
            using (var dialog = new Form { Text = "NOMAD License", Size = new Size(650, 500),
                StartPosition = FormStartPosition.CenterParent })
            using (var text = new TextBox { Dock = DockStyle.Fill, ReadOnly = true, Multiline = true,
                ScrollBars = ScrollBars.Both, WordWrap = false, Font = new Font("Consolas", 9F),
                Text = System.Text.Encoding.UTF8.GetString(Program.Resource("NOMAD.LICENSE")) + "\r\n\r\n" +
                    System.Text.Encoding.UTF8.GetString(Program.Resource("NOMAD.NOTICE")) })
            {
                dialog.Controls.Add(text);
                dialog.ShowDialog(this);
            }
        }
    }
}
