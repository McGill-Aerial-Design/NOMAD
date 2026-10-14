// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using Newtonsoft.Json;
using NOMAD.MissionPlanner;

internal static class CoordinatePasteTests
{
    [DllImport("user32.dll")]
    private static extern bool PostMessage(IntPtr window, uint message, IntPtr wparam, IntPtr lparam);

    private static void Require(bool condition, string message)
    {
        if (!condition) { throw new Exception("Coordinate paste: " + message); }
    }

    private static T Control<T>(Form dialog, string name) where T : System.Windows.Forms.Control
    {
        return (T)dialog.Controls.Find(name, true).Single();
    }

    private static GeofenceConfig Config(bool derive = false)
    {
        GeofenceConfig.Writes = 0; GeofenceConfig.SaveResult = true; GeofenceConfig.BeforeSave = null;
        return new GeofenceConfig
        {
            SoftBoundaryFromHard = derive,
            SoftBoundary = new FlightBoundary { Vertices = new List<GpsPoint> { new GpsPoint(1, 2) } },
            HardBoundary = new FlightBoundary { Vertices = new List<GpsPoint> { new GpsPoint(3, 4) } },
        };
    }

    private static DialogResult Show(Form dialog, Action<Form> action)
    {
        Exception failure = null;
        using (var deadline = new Timer { Interval = 3000 })
        {
            deadline.Tick += (s, e) => { failure = new Exception("Modal exceeded its deadline."); dialog.Close(); };
            dialog.Shown += (s, e) => dialog.BeginInvoke(new Action(() =>
            {
                try { action(dialog); }
                catch (Exception error) { failure = error; dialog.Close(); }
            }));
            deadline.Start();
            var result = dialog.ShowDialog();
            if (failure != null) { throw failure; }
            return result;
        }
    }

    internal static void Run()
    {
        ParseOrdersAndFailures();
        ModalCorrectionAndSaveFailure();
        Cancellation();
        CommitAndContext();
        DialogLayout();
        Console.WriteLine("PASS coordinate paste: invariant orders, whole-input rejection, real modal/cancel, pending grids and context");
    }

    private static void ParseOrdersAndFailures()
    {
        var previous = CultureInfo.CurrentCulture;
        try
        {
            foreach (string culture in new[] { "en-CA", "fr-CA" })
            {
                CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo(culture);
                foreach (bool longitudeFirst in new[] { false, true })
                foreach (var pair in new[] { new[] { -33.9, 18.4 }, new[] { 75.0, 45.0 }, new[] { 0.0, 0.0 } })
                {
                    var input = pair[0].ToString("R", CultureInfo.InvariantCulture) + "," +
                        pair[1].ToString("R", CultureInfo.InvariantCulture);
                    Require(NOMADBoundaryView.ParseCoordinatesForTest(input, longitudeFirst, out var points, out var error),
                        "valid ambiguous pair rejected under " + culture + ": " + error);
                    Require(points.Count == 1 && points[0].Lat == pair[longitudeFirst ? 1 : 0] &&
                        points[0].Lon == pair[longitudeFirst ? 0 : 1], "explicit coordinate order changed");
                }
                Require(NOMADBoundaryView.ParseCoordinatesForTest("\r\n -90, -180\r\n90\t180\n0 0\n1e1,2e1", false,
                    out var boundaries, out _), "legal endpoints/zero/separators/exponents rejected");
                Require(boundaries.Count == 4 && boundaries[0].Lat == -90 && boundaries[1].Lon == 180 &&
                    boundaries[3].Lat == 10, "valid line values changed");
                Require(NOMADBoundaryView.ParseCoordinatesForTest("180,90\n-180,-90", true, out var two, out _) &&
                    two.Count == 2 && two[0].Lat == 90 && two[1].Lon == -180, "two-point ordered endpoint edit lost");
                foreach (string invalid in new[] { "", " \r\n", "1", "1,,2", "1,2,", ",2", "1,",
                    "1 2 3", "1,2,3", "NaN,2", "1,Infinity", "1e999,2", "90.0001,0", "0,-180.0001", "45,5 -75,5" })
                {
                    Require(!NOMADBoundaryView.ParseCoordinatesForTest(invalid, false, out var points, out var error) &&
                        points == null && !string.IsNullOrEmpty(error), "invalid paste partly accepted: " + invalid);
                }
                Require(!NOMADBoundaryView.ParseCoordinatesForTest("\r\n1,2\r\nbad\r\n3,4", false, out var rejected,
                    out var lineError) && rejected == null && lineError.StartsWith("Line 3:"),
                    "whole-input rejection lost original blank/CRLF line number");
            }
        }
        finally { CultureInfo.CurrentCulture = previous; }
    }

    private static void ModalCorrectionAndSaveFailure()
    {
        foreach (bool saveFailure in new[] { false, true })
        {
            var config = Config();
            var original = JsonConvert.SerializeObject(config);
            using (var view = new NOMADBoundaryView(config))
            {
                view.PendingEdit(false);
                string gridBefore = view.GridValues();
                using (var dialog = view.CoordinateDialog(false))
                {
                var result = Show(dialog, form =>
                {
                    var text = Control<TextBox>(form, "coordinateText");
                    var order = Control<ComboBox>(form, "coordinateOrder");
                    var replace = Control<CheckBox>(form, "replaceCoordinates");
                    var import = Control<Button>(form, "importCoordinates");
                    Require(order.SelectedIndex == 0 && text.AcceptsReturn && import.DialogResult == DialogResult.None,
                        "default order/newline/error behavior changed");
                    order.SelectedIndex = 1; replace.Checked = false;
                    Require(Control<Label>(form, "coordinateHint").Text.Contains("-75.5, 45.5"), "order example not updated");
                    text.Text = saveFailure ? "18.4,-33.9" : "1,2\r\nbad\r\n3,4";
                    string pasted = text.Text;
                    GeofenceConfig.SaveResult = !saveFailure;
                    import.PerformClick();
                    Require(form.Visible && form.Modal && form.DialogResult == DialogResult.None && text.Text == pasted &&
                        order.SelectedIndex == 1 && !replace.Checked, "failed import closed or cleared user input/options");
                    Require(JsonConvert.SerializeObject(config) == original &&
                        view.GridValues() == gridBefore && view.EditingText(false) == "7.25" &&
                        !string.IsNullOrEmpty(Control<Label>(form, "coordinateError").Text), "failed import mutated config or pending grid");
                    import.Focus();
                    Require((bool)typeof(Form).GetMethod("ProcessDialogKey", BindingFlags.Instance | BindingFlags.NonPublic)
                        .Invoke(form, new object[] { Keys.Enter }) && form.Visible && text.Text == pasted,
                        "Import Enter closed or cleared failed input");
                    int writes = config.Saves;
                    GeofenceConfig.SaveResult = true;
                    text.Text = "18.4,-33.9";
                    import.PerformClick();
                    Require(config.Saves == writes + 1 && config.HardBoundary.Vertices.Count == 2 &&
                        config.HardBoundary.Vertices[1].Lat == -33.9, "correction did not commit once using retained options");
                });
                Require(result == DialogResult.OK, "corrected modal did not return success");
                }
            }
        }
    }

    private static void Cancellation()
    {
        foreach (string action in new[] { "cancel", "escape", "close" })
        {
            var config = Config(); var before = JsonConvert.SerializeObject(config);
            using (var view = new NOMADBoundaryView(config))
            using (var dialog = view.CoordinateDialog(false))
            {
                string gridBefore = view.GridValues();
                var result = Show(dialog, form =>
                {
                    Control<TextBox>(form, "coordinateText").Text = "1,2";
                    if (action == "cancel") { Control<Button>(form, "cancelCoordinates").PerformClick(); }
                    else if (action == "close") { form.Close(); }
                    else
                    {
                        Require(PostMessage(form.Handle, 0x100, new IntPtr(27), IntPtr.Zero), "Escape key dispatch failed");
                        PostMessage(form.Handle, 0x101, new IntPtr(27), IntPtr.Zero);
                    }
                });
                Require(result == DialogResult.Cancel && config.Saves == 0 && before == JsonConvert.SerializeObject(config) &&
                    view.GridValues() == gridBefore, action + " changed config or editor");
            }
        }
    }

    private static void CommitAndContext()
    {
        foreach (bool inner in new[] { false, true })
        foreach (bool append in new[] { false, true })
        {
            var config = Config(); MapOverlayManager.Draws = 0;
            var untouched = inner ? config.HardBoundary.Vertices : config.SoftBoundary.Vertices;
            using (var view = new NOMADBoundaryView(config))
            {
                view.PendingEdit(inner);
                using (var dialog = view.CoordinateDialog(inner))
                {
                    Show(dialog, form =>
                    {
                        Control<TextBox>(form, "coordinateText").Text = "-20,30";
                        Control<CheckBox>(form, "replaceCoordinates").Checked = !append;
                        Control<Button>(form, "importCoordinates").PerformClick();
                    });
                }
                var target = inner ? config.SoftBoundary.Vertices : config.HardBoundary.Vertices;
                Require(target.Count == (append ? 2 : 1) && target.Last().Lat == -20 && target.Last().Lon == 30 &&
                    ReferenceEquals(untouched, inner ? config.HardBoundary.Vertices : config.SoftBoundary.Vertices) &&
                    config.Saves == 1 && MapOverlayManager.Draws == 1, "pending-grid append/replace changed geometry or repeated save/draw");
            }
        }
        foreach (int mutation in Enumerable.Range(0, 8))
        {
            var config = Config(); MapOverlayManager.Draws = 0;
            using (var view = new NOMADBoundaryView(config))
            using (var dialog = view.CoordinateDialog(false))
            {
                switch (mutation)
                {
                    case 0: view.RecreateForTest(); break;
                    case 1: view.Dispose(); break;
                    case 2: config.HardBoundary = new FlightBoundary(); break;
                    case 3: config.HardBoundary.Vertices = config.HardBoundary.Vertices.ToList(); break;
                    case 4: config.HardBoundary.Vertices[0].Lat++; break;
                    case 5: config.AdvisoryAudioAlertsEnabled = false; break;
                    case 6: config.SoftBoundaryFromHard = true; break;
                    case 7: typeof(NOMADBoundaryView).GetField("_missionConfig", BindingFlags.Instance | BindingFlags.NonPublic)
                        .SetValue(view, Config()); break;
                }
                Show(dialog, form =>
                {
                    Control<TextBox>(form, "coordinateText").Text = "1,2";
                    Control<Button>(form, "importCoordinates").PerformClick();
                    Require(form.DialogResult == DialogResult.None && config.Saves == 0 && MapOverlayManager.Draws == 0 &&
                        Control<Label>(form, "coordinateError").Text.Contains("changed"), "stale context accepted: " + mutation);
                    Control<Button>(form, "cancelCoordinates").PerformClick();
                });
            }
        }
        var derived = Config(true);
        using (var view = new NOMADBoundaryView(derived))
        using (var dialog = view.CoordinateDialog(true))
        {
            Show(dialog, form =>
            {
                Control<TextBox>(form, "coordinateText").Text = "1,2";
                Control<Button>(form, "importCoordinates").PerformClick();
                Require(derived.Saves == 0 && Control<Label>(form, "coordinateError").Text.Contains("derived"),
                    "direct modal bypassed derived-inner guard");
                Control<Button>(form, "cancelCoordinates").PerformClick();
            });
        }
        using (var view = new NOMADBoundaryView(derived))
        using (var dialog = view.CoordinateDialog(false))
        {
            Show(dialog, form =>
            {
                Control<TextBox>(form, "coordinateText").Text = "0,0\n1,1";
                Control<Button>(form, "importCoordinates").PerformClick();
                Require(derived.HardBoundary.Vertices.Count == 2 && derived.SoftBoundary.Vertices.Count == 0 &&
                    derived.Saves == 1, "two-point derived outer edit invented a complete inner outline");
            });
        }
    }

    private static void DialogLayout()
    {
        foreach (bool narrow in new[] { false, true })
        foreach (string failure in new[] { "invalid", "range", "save", "context" })
        {
            var config = Config();
            using (var view = new NOMADBoundaryView(config))
            using (var dialog = view.CoordinateDialog(false))
            {
                if (narrow) { dialog.Font = new Font("Segoe UI", 12); dialog.Size = dialog.MinimumSize; }
                Show(dialog, form =>
                {
                    Control<TextBox>(form, "coordinateText").Text = failure == "invalid" ? "1,2,3" :
                        failure == "range" ? "90.001,0" : "1,2";
                    if (failure == "save") { GeofenceConfig.SaveResult = false; }
                    if (failure == "context") { config.SoftBoundaryInsetMeters++; }
                    Control<Button>(form, "importCoordinates").PerformClick();
                    form.PerformLayout();
                    foreach (string name in new[] { "coordinateOrder", "coordinateHint", "coordinateText",
                        "replaceCoordinates", "coordinateError", "importCoordinates", "cancelCoordinates" })
                    {
                        var control = Control<System.Windows.Forms.Control>(form, name);
                        var bounds = form.RectangleToClient(control.RectangleToScreen(control.ClientRectangle));
                        Require(control.Visible && bounds.Width > 0 && bounds.Height > 0 &&
                            form.ClientRectangle.Contains(bounds), "dialog control clipped: " + name + "/narrow=" + narrow);
                        if (control is CheckBox)
                        {
                            Require(control.PreferredSize.Width <= control.Width, "replace/append choice text clipped");
                        }
                        if (control is Label)
                        {
                            Require(control.GetPreferredSize(new Size(control.Width, int.MaxValue)).Height <= control.Height,
                                "dialog message text clipped: " + failure + "/" + name);
                        }
                    }
                    string capture = Environment.GetEnvironmentVariable("NOMAD_COORDINATE_RENDER_DIR");
                    if (!string.IsNullOrEmpty(capture))
                    {
                        using (var image = new Bitmap(form.Width, form.Height))
                        {
                            form.DrawToBitmap(image, new Rectangle(Point.Empty, image.Size));
                            image.Save(Path.Combine(capture, (narrow ? "paste-narrow-" : "paste-normal-") + failure + ".png"));
                        }
                    }
                    Control<Button>(form, "cancelCoordinates").PerformClick();
                });
                Require(config.Saves == (failure == "save" ? 1 : 0), "layout validation dispatched an unexpected save");
            }
        }
    }
}
