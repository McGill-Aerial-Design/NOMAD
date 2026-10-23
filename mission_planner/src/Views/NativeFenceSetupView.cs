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
        internal NativePlanDrafts Drafts;
    }

    public sealed partial class NativeFenceSetupView : UserControl
    {
        private readonly CancellationToken _shutdown;
        private readonly TextBox _coordinates;
        private readonly ComboBox _order;
        private readonly Label _status;
        private readonly ToolTip _tips = new ToolTip();
        private readonly FenceSetupInput _input;

        internal NativeFenceSetupView(CancellationToken shutdown, FenceSetupInput input, NOMADConfig config)
        {
            _shutdown = shutdown;
            _input = input;
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
                Text = "Prepare a polygon, stage it in Plan, then review and use native Write/Read.",
                AutoSize = true, MaximumSize = new Size(680, 0),
            });
            var polygon = Group(layout, "Polygon draft");
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
            Add(polygon, _order);
            Add(polygon, new Label
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
            Add(polygon, _coordinates);
            var actions = new FlowLayoutPanel { AutoSize = true, Dock = DockStyle.Fill, WrapContents = true };
            actions.Controls.Add(Button("Open native Plan · Fence", (s, e) => OpenPlan()));
            actions.Controls.Add(Button("Stage polygon in native Plan", Stage));
            Add(polygon, actions);
            _status = new Label
            {
                Name = "FenceSetupStatus", AutoSize = true, MaximumSize = new Size(680, 0),
                Text = "Open Plan selects Fence. Staging replaces its editor draft after confirmation; it does not upload.",
            };
            Add(polygon, _status);
            AddParameters(layout, config);
            Controls.Add(layout);
        }

        private bool Active => !IsDisposed && !Disposing && !_shutdown.IsCancellationRequested;

        private void OpenPlan()
        {
            if (!Active) { return; }
            try
            {
                MainV2.instance?.MenuFlightPlanner?.PerformClick();
                SelectFence();
            }
            catch (Exception error)
            {
                if (Active) { _status.Text = "Native Plan unavailable: " + error.Message; }
                else { MessageBox.Show(MainV2.instance, error.Message, "Native Fence selection failed",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning); }
            }
        }

        private FlightPlanner SelectFence()
        {
            var planner = FlightPlanner.instance;
            if (planner == null || planner.IsDisposed) { throw new InvalidOperationException("Open native Plan first."); }
            if (_input.Drafts?.IsFor(planner) != true)
            {
                _input.Drafts?.Dispose();
                _input.Drafts = new NativePlanDrafts(planner);
            }
            if (!_input.Drafts.SelectFence()) { throw new InvalidOperationException(_input.Drafts.Error); }
            return planner;
        }

        private void Stage(object sender, EventArgs args)
        {
            if (!Active) { return; }
            if (!FenceCoordinates.TryParse(_coordinates.Text, _order.SelectedIndex == 1, out var points, out var error))
            {
                _status.Text = error;
                return;
            }
            try
            {
                var planner = SelectFence();
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

        private static TableLayoutPanel Group(TableLayoutPanel parent, string title)
        {
            var panel = new TableLayoutPanel
            {
                AutoSize = true, Dock = DockStyle.Fill, ColumnCount = 1, Padding = new Padding(14),
                BackColor = NOMADTheme.CARD_BG,
            };
            panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            Add(panel, new Label { Text = title, AutoSize = true,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_HEADING, FontStyle.Bold) });
            Add(parent, panel);
            return panel;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _parameterShutdown.Dispose();
                _parameters?.Dispose();
                _tips.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
