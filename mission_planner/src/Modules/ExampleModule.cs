// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// ExampleModule - a minimal, working NOMAD plugin module
// ============================================================
// Demonstrates the C# module SDK (src/Core): metadata with an enable flag,
// Configure() pulling the plugin config out of the shared context, and
// GetViews() contributing a sidebar view plus an action. NOMADMainScreen
// appends these to its sidebar automatically. Turn it off by setting the
// environment variable NOMAD_PLUGIN_EXAMPLE_MODULE=0.
// ============================================================

using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;
using NOMAD.MissionPlanner.Core;

namespace NOMAD.MissionPlanner.Modules
{
    /// <summary>Adds an "Example Module" page and a sample action to the NOMAD screen.</summary>
    public sealed class ExampleModule : NomadModuleBase
    {
        private NOMADConfig _config;

        public override NomadModuleMetadata Metadata => new NomadModuleMetadata
        {
            Name = "example",
            Version = "1.0.0",
            Description = "Example module demonstrating the NOMAD plugin module SDK.",
            EnableFlag = "NOMAD_PLUGIN_EXAMPLE_MODULE",
            EnabledByDefault = true,
        };

        public override void Configure(NomadModuleContext context)
        {
            // Pull whatever services the module needs out of the shared context.
            _config = context.Get<NOMADConfig>();
        }

        public override IEnumerable<NomadViewDescriptor> GetViews()
        {
            // A sidebar entry that swaps a view into the content area.
            yield return NomadViewDescriptor.View(
                id: "example",
                buttonText: "Example Module",
                title: "Example Module",
                section: "MODULES",
                viewFactory: () => new ExampleView(_config));

            // A sidebar entry that just runs an action (no view swap).
            yield return NomadViewDescriptor.ActionEntry(
                id: "example_ping",
                buttonText: "Example Action",
                section: "MODULES",
                action: () => MessageBox.Show(
                    "Hello from ExampleModule, contributed through the NOMAD module SDK.",
                    "NOMAD Module SDK"));
        }
    }

    /// <summary>The page contributed by <see cref="ExampleModule"/>.</summary>
    public sealed class ExampleView : NOMADViewBase
    {
        public ExampleView(NOMADConfig config)
        {
            var profile = string.IsNullOrWhiteSpace(config?.ActiveProfile) ? "dev" : config.ActiveProfile;

            var layout = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = true,
                BackColor = NOMADTheme.BG_DARK,
            };

            layout.Controls.Add(new Label
            {
                Text = "Example Module",
                Font = new Font("Segoe UI", 16, FontStyle.Bold),
                ForeColor = ACCENT_COLOR,
                AutoSize = true,
                Margin = new Padding(4, 4, 4, 12),
            });

            layout.Controls.Add(new Label
            {
                Text =
                    "This page is contributed by ExampleModule through the NOMAD plugin\n" +
                    "module SDK (mission_planner/src/Core). A module declares its metadata,\n" +
                    "reads services from the shared context, and returns sidebar views and\n" +
                    "actions, without editing NOMADMainScreen.\n\n" +
                    "Active config profile (read from the module context): " + profile,
                Font = new Font("Segoe UI", 10),
                ForeColor = TEXT_PRIMARY,
                AutoSize = true,
                Margin = new Padding(4),
            });

            Controls.Add(layout);
        }
    }
}
