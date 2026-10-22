// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Threading;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.GCSViews;

namespace NOMAD.MissionPlanner
{
    // The plugin owns this unsaved input while MP disposes/recreates its NOMAD screen.
    internal sealed class FenceSetupInput
    {
        internal string Coordinates = "";
        internal bool LongitudeFirst;
    }

    public sealed class NativeFenceSetupView : UserControl
    {
        private readonly CancellationToken _shutdown;
        private readonly TextBox _coordinates;
        private readonly ComboBox _order;
        private readonly Label _status;
        private readonly ToolTip _tips = new ToolTip();

        internal NativeFenceSetupView(CancellationToken shutdown, FenceSetupInput input)
        {
            _shutdown = shutdown;
            BackColor = NOMADTheme.BG_DARK;
            ForeColor = NOMADTheme.TEXT_PRIMARY;
            Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL);
            AutoScroll = true;
            var layout = new TableLayoutPanel
            {
                Dock = DockStyle.Top, AutoSize = true, ColumnCount = 1, Padding = new Padding(16),
            };
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            Add(layout, new Label
            {
                Text = "NATIVE FENCE SETUP", AutoSize = true,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_TITLE, FontStyle.Bold),
            });
            Add(layout, new Label
            {
                Text = "Prepare polygon → stage in native Plan → review → native Write/Read. " +
                    "Mission Planner owns fence transfers and failsafe settings.",
                AutoSize = true, MaximumSize = new Size(680, 0),
            });
            var preset = Button("Use AEAC 2027 published C1", (s, e) =>
            {
                if (!Active) { return; }
                _order.SelectedIndex = 0;
                _coordinates.Text = FenceCoordinates.PublishedC1;
                _status.Text = "Published C1 loaded into this draft. Review the site and coordinates before staging.";
            });
            Add(layout, preset);
            _tips.SetToolTip(preset, "AEAC CONOPS v1.0 · 2026-09-08 · Appendix C, p31. " +
                "One published polygon; source labels remain ambiguous. No second polygon is inferred.");
            var source = new LinkLabel
            {
                Text = "Published CONOPS source", AutoSize = true, LinkColor = NOMADTheme.ACCENT,
            };
            source.LinkClicked += (s, e) =>
            {
                if (Active) { System.Diagnostics.Process.Start(FenceCoordinates.SourceUrl); }
            };
            Add(layout, source);
            _order = new ComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList, Width = 240,
                BackColor = NOMADTheme.CONTROL_BG, ForeColor = NOMADTheme.TEXT_PRIMARY,
            };
            _order.Items.AddRange(new object[] { "Latitude, longitude", "Longitude, latitude" });
            _order.SelectedIndex = input.LongitudeFirst ? 1 : 0;
            _order.SelectedIndexChanged += (s, e) =>
            {
                if (Active) { input.LongitudeFirst = _order.SelectedIndex == 1; }
            };
            Add(layout, _order);
            Add(layout, new Label
            {
                Text = "Paste one pair per line; comma, space or tab. Use . for decimals. The complete polygon is validated.",
                AutoSize = true, MaximumSize = new Size(680, 0),
            });
            _coordinates = new TextBox
            {
                Name = "FenceCoordinates", Multiline = true, AcceptsReturn = true,
                ScrollBars = ScrollBars.Vertical, Height = 200, Dock = DockStyle.Fill,
                Font = NOMADTheme.Mono(NOMADTheme.SIZE_SMALL),
                BackColor = NOMADTheme.CONTROL_BG, ForeColor = NOMADTheme.TEXT_PRIMARY,
                Text = input.Coordinates,
            };
            _coordinates.TextChanged += (s, e) =>
            {
                if (Active) { input.Coordinates = _coordinates.Text; }
            };
            Add(layout, _coordinates);
            Add(layout, Button("Open native Plan (select Fence)", (s, e) => OpenPlan()));
            Add(layout, Button("Stage polygon in native Plan", Stage));
            Add(layout, Button("Open native configuration (failsafes)", (s, e) =>
            {
                if (Active) { MainV2.instance?.MenuConfigTune?.PerformClick(); }
            }));
            _status = new Label
            {
                Name = "FenceSetupStatus", AutoSize = true, MaximumSize = new Size(680, 0),
                Text = "Choose Fence in native Plan first. Staging replaces only its editor draft; it does not upload.",
            };
            Add(layout, _status);
            Controls.Add(layout);
        }

        private bool Active => !IsDisposed && !Disposing && !_shutdown.IsCancellationRequested;

        private void OpenPlan()
        {
            if (Active) { MainV2.instance?.MenuFlightPlanner?.PerformClick(); }
        }

        private void Stage(object sender, EventArgs args)
        {
            if (!Active) { return; }
            if (!FenceCoordinates.TryParse(_coordinates.Text, _order.SelectedIndex == 1, out var points, out var error))
            {
                _status.Text = error;
                return;
            }
            var planner = FlightPlanner.instance;
            if (!NativeFenceDraft.IsFence(planner))
            {
                _status.Text = "Select Fence in native Plan, then return here to stage. Existing mission drafts are not switched.";
                OpenPlan();
                return;
            }
            try
            {
                var draft = new NativeFenceDraft(planner, points);
                string original = _coordinates.Text;
                int order = _order.SelectedIndex;
                var handle = Handle;
                var choice = MessageBox.Show(this,
                    $"Replace the current native Fence editor draft with {points.Count} vertices?\n" +
                    "No aircraft upload is performed. Review the draft in Plan before native Write/Read.",
                    "Stage native fence draft", MessageBoxButtons.OKCancel,
                    MessageBoxIcon.Question, MessageBoxDefaultButton.Button2);
                if (!Active || !IsHandleCreated || Handle != handle || choice != DialogResult.OK) { return; }
                if (_coordinates.Text != original || _order.SelectedIndex != order)
                {
                    _status.Text = "Coordinates changed; staging cancelled. Review and try again.";
                    return;
                }
                if (draft.TryStage(out var message))
                {
                    _status.Text = message;
                    try { planner.writeKML(); OpenPlan(); }
                    catch (Exception)
                    {
                        _status.Text = "Draft staged; Plan refresh/navigation failed. " +
                            "Open native Plan manually and review before Write/Read.";
                    }
                    return;
                }
                _status.Text = message;
            }
            catch (Exception ex)
            {
                if (Active) { _status.Text = "Fence staging unavailable: " + ex.Message; }
            }
        }

        private static Button Button(string text, EventHandler click)
        {
            var button = ControlFactory.Button(text, click);
            button.AutoSize = true;
            button.Padding = new Padding(8, 4, 8, 4);
            return button;
        }

        private static void Add(TableLayoutPanel layout, Control control)
        {
            int row = layout.RowCount++;
            layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            control.Margin = new Padding(0, 0, 0, 10);
            layout.Controls.Add(control, 0, row);
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing) { _tips.Dispose(); }
            base.Dispose(disposing);
        }
    }
}
