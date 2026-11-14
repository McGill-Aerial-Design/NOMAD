// SPDX-License-Identifier: Apache-2.0
using System;
using System.Threading;
using System.Windows.Forms;
using System.Windows;
using Wpf = System.Windows.Controls;
using MessageBox = System.Windows.Forms.MessageBox;
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
        private readonly Wpf.TextBox _coordinates;
        private readonly Wpf.ComboBox _order;
        private readonly Wpf.TextBlock _status;
        private readonly FenceSetupInput _input;

        internal NativeFenceSetupView(CancellationToken shutdown, FenceSetupInput input, NOMADConfig config)
        {
            _shutdown = shutdown;
            _input = input;
            BackColor = NOMADTheme.BG_DARK;
            ForeColor = NOMADTheme.TEXT_PRIMARY;
            Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL);
            var layout = new Wpf.StackPanel { Margin = new Thickness(16) };
            Add(layout, WpfUi.Section("FENCE SETUP"));
            var polygon = Group(layout, "Polygon draft");
            _order = new Wpf.ComboBox { Width = 240, HorizontalAlignment = System.Windows.HorizontalAlignment.Left };
            _order.Items.Add("Latitude, longitude"); _order.Items.Add("Longitude, latitude");
            _order.SelectedIndex = input.LongitudeFirst ? 1 : 0;
            _order.SelectionChanged += (s, e) =>
            {
                if (Active) { input.LongitudeFirst = _order.SelectedIndex == 1; }
            };
            Add(polygon, _order);
            _coordinates = new Wpf.TextBox
            {
                Name = "FenceCoordinates", AcceptsReturn = true,
                VerticalScrollBarVisibility = Wpf.ScrollBarVisibility.Auto, Height = 140,
                Text = input.Coordinates,
                ToolTip = "One coordinate pair per line. Separate values with a comma, space or tab.",
            };
            _coordinates.TextChanged += (s, e) =>
            {
                if (Active) { input.Coordinates = _coordinates.Text; }
            };
            Add(polygon, _coordinates);
            var actions = new Wpf.WrapPanel();
            actions.Children.Add(Button("Open Fence in Plan", (s, e) => OpenPlan()));
            actions.Children.Add(Button("Stage in Plan…", Stage));
            Add(polygon, actions);
            _status = WpfUi.Text(""); _status.Name = "FenceSetupStatus";
            Add(polygon, _status);
            AddParameters(layout, config);
            Controls.Add(WpfUi.Host(new Wpf.ScrollViewer { Content = layout, VerticalScrollBarVisibility = Wpf.ScrollBarVisibility.Auto }));
        }

        private bool Active => !IsDisposed && !Disposing && !_shutdown.IsCancellationRequested;

        private void OpenPlan()
        {
            if (!Active) { return; }
            if (MainV2.comPort?.giveComport == true)
            { _status.Text = "Wait for the current request before opening Fence."; return; }
            try
            {
                MainV2.instance?.MenuFlightPlanner?.PerformClick();
                SelectFence();
            }
            catch (Exception error)
            {
                if (Active) { _status.Text = "Plan unavailable: " + error.Message; }
                else { MessageBox.Show(MainV2.instance, error.Message, "Fence selection failed",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning); }
            }
        }

        private FlightPlanner SelectFence()
        {
            var planner = FlightPlanner.instance;
            if (planner == null || planner.IsDisposed) { throw new InvalidOperationException("Open Plan first."); }
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
                    $"Replace the current Fence draft with {points.Count} vertices?\n" +
                    "No aircraft upload is performed. Review the draft in Plan before Write/Read.",
                    "Stage fence draft", MessageBoxButtons.OKCancel,
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
                            "Open Plan and review before Write/Read.";
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

        private static Wpf.Button Button(string text, EventHandler click)
        {
            return WpfUi.Action(text, () => click(null, EventArgs.Empty));
        }

        private static void Add(Wpf.StackPanel layout, FrameworkElement control)
        {
            control.Margin = new Thickness(0, 0, 0, 10);
            layout.Children.Add(control);
        }

        private static Wpf.StackPanel Group(Wpf.StackPanel parent, string title)
        {
            var panel = new Wpf.StackPanel { Background = WpfTheme.Card, Margin = new Thickness(0, 0, 0, 10) };
            Add(panel, WpfUi.Section(title));
            Add(parent, panel);
            return panel;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _parameterShutdown.Dispose();
                _parameters?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
