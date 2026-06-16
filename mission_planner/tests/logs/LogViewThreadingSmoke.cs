// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

internal static class LogViewThreadingSmoke
{
    [STAThread]
    private static int Main(string[] args)
    {
        if (args.Length != 3)
        {
            Console.Error.WriteLine("usage: LogViewThreadingSmoke <plugin> <fixture> <mission-planner-dir>");
            return 2;
        }

        string pluginPath = Path.GetFullPath(args[0]);
        string fixturePath = Path.GetFullPath(args[1]);
        string missionPlannerDirectory = Path.GetFullPath(args[2]);
        string pluginDirectory = Path.GetDirectoryName(pluginPath);
        AppDomain.CurrentDomain.AssemblyResolve += (sender, eventArgs) =>
            ResolveAssembly(eventArgs.Name, pluginDirectory, missionPlannerDirectory);

        Application.EnableVisualStyles();
        Assembly plugin = Assembly.LoadFrom(pluginPath);
        Type configType = plugin.GetType("NOMAD.MissionPlanner.NOMADConfig", true);
        Type viewType = plugin.GetType("NOMAD.MissionPlanner.NOMADLogView", true);
        object config = Activator.CreateInstance(configType);
        ConstructorInfo constructor = viewType.GetConstructor(new[]
        {
            configType,
            plugin.GetType("NOMAD.MissionPlanner.DualLinkSender", true),
        });

        using (var host = new Form
        {
            Location = new Point(-32000, -32000),
            ShowInTaskbar = false,
            Size = new Size(1200, 800),
        })
        using (var view = (Control)constructor.Invoke(new[] { config, null }))
        {
            view.Dock = DockStyle.Fill;
            host.Controls.Add(view);
            host.Show();

            MethodInfo loadMethod = viewType.GetMethod(
                "LoadFileAsync",
                BindingFlags.Instance | BindingFlags.NonPublic);
            var loadTask = (Task)loadMethod.Invoke(view, new object[] { fixturePath });
            var timeout = Stopwatch.StartNew();
            while (!loadTask.IsCompleted && timeout.Elapsed < TimeSpan.FromSeconds(20))
            {
                Application.DoEvents();
                Thread.Sleep(10);
            }
            if (!loadTask.IsCompleted)
                throw new TimeoutException("The async log load did not complete on the WinForms UI thread.");

            loadTask.GetAwaiter().GetResult();
            FieldInfo summaryField = viewType.GetField("_summary", BindingFlags.Instance | BindingFlags.NonPublic);
            object summary = summaryField.GetValue(view);
            if (summary == null)
                throw new InvalidOperationException("The log view did not retain the loaded summary.");

            var metrics = (ICollection)summary.GetType().GetProperty("Metrics").GetValue(summary);
            if (metrics.Count < 6)
                throw new InvalidOperationException($"Expected at least 6 summary metrics, got {metrics.Count}.");

            AssertModeVisuals(plugin, view, viewType);
            ExerciseResponsiveLayouts(view, viewType, host);
            host.Close();
        }

        Console.WriteLine("Flight log WinForms threading and responsive layout smoke test passed.");
        return 0;
    }

    private static void AssertModeVisuals(Assembly plugin, Control view, Type viewType)
    {
        Type visuals = plugin.GetType("NOMAD.MissionPlanner.FlightModeVisuals", true);
        MethodInfo colorFor = visuals.GetMethod(
            "ColorFor",
            BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
        var stabilize = (Color)colorFor.Invoke(null, new object[] { "STABILIZE" });
        var loiter = (Color)colorFor.Invoke(null, new object[] { "LOITER" });
        var land = (Color)colorFor.Invoke(null, new object[] { "LAND" });
        if (stabilize == loiter || stabilize == land || loiter == land)
            throw new InvalidOperationException("Common flight modes do not have distinct timeline colors.");

        Control plot = (Control)GetField(viewType, view, "_offlinePlot");
        FieldInfo modesField = plot.GetType().GetField("_modes", BindingFlags.Instance | BindingFlags.NonPublic);
        var modes = (ICollection)modesField.GetValue(plot);
        if (modes.Count < 2)
            throw new InvalidOperationException("Flight modes were not attached to the time-series plot.");
    }

    private static void ExerciseResponsiveLayouts(Control view, Type viewType, Form host)
    {
        var sizes = new[]
        {
            new Size(480, 800),
            new Size(640, 480),
            new Size(1280, 540),
            new Size(1920, 1080),
        };
        var tabs = (TabControl)GetField(viewType, view, "_tabs");
        var summaryBody = (TableLayoutPanel)GetField(viewType, view, "_summaryBody");
        var plotSplit = (SplitContainer)GetField(viewType, view, "_plotSplit");
        var liveLayout = (TableLayoutPanel)GetField(viewType, view, "_liveLayout");
        var liveMetrics = (FlowLayoutPanel)GetField(viewType, view, "_liveMetrics");
        MethodInfo applyLayout = viewType.GetMethod(
            "ApplyResponsiveLayout",
            BindingFlags.Instance | BindingFlags.NonPublic);

        if (tabs.DrawMode != TabDrawMode.OwnerDrawFixed)
            throw new InvalidOperationException("Flight log tabs are not using the NOMAD owner-drawn theme.");
        if (!liveMetrics.WrapContents)
            throw new InvalidOperationException("Live metric cards are not configured to wrap.");

        foreach (Size size in sizes)
        {
            host.ClientSize = size;
            host.PerformLayout();

            tabs.SelectedIndex = 0;
            applyLayout.Invoke(view, null);
            Application.DoEvents();
            bool compact = tabs.SelectedTab.ClientSize.Width < 900;
            bool compactPlot = tabs.SelectedTab.ClientSize.Width < 820;
            if (summaryBody.ColumnCount != (compact ? 1 : 2))
                throw new InvalidOperationException($"Summary layout did not adapt at {size.Width}x{size.Height}.");
            Orientation expectedOrientation = compactPlot ? Orientation.Horizontal : Orientation.Vertical;
            if (plotSplit.Orientation != expectedOrientation)
                throw new InvalidOperationException($"Plot layout did not adapt at {size.Width}x{size.Height}.");
            AssertUsable(summaryBody, "summary", size);
            AssertUsable(plotSplit.Panel2, "post-flight plot", size);
            Render(view);

            tabs.SelectedIndex = 1;
            applyLayout.Invoke(view, null);
            Application.DoEvents();
            AssertUsable(liveLayout, "live layout", size);
            AssertUsable((Control)GetField(viewType, view, "_livePlot"), "live plot", size);
            if (liveLayout.RowStyles[1].Height <= 0)
                throw new InvalidOperationException($"Live metric row collapsed at {size.Width}x{size.Height}.");
            Render(view);
        }
    }

    private static object GetField(Type type, object instance, string name)
    {
        FieldInfo field = type.GetField(name, BindingFlags.Instance | BindingFlags.NonPublic);
        if (field == null)
            throw new MissingFieldException(type.FullName, name);
        return field.GetValue(instance);
    }

    private static void AssertUsable(Control control, string name, Size hostSize)
    {
        if (control.ClientSize.Width < 40 || control.ClientSize.Height < 30)
        {
            throw new InvalidOperationException(
                $"{name} collapsed to {control.ClientSize.Width}x{control.ClientSize.Height} " +
                $"at {hostSize.Width}x{hostSize.Height}.");
        }
    }

    private static void Render(Control control)
    {
        using (var bitmap = new Bitmap(
            Math.Max(1, control.ClientSize.Width),
            Math.Max(1, control.ClientSize.Height)))
        {
            control.DrawToBitmap(bitmap, control.ClientRectangle);
        }
    }

    private static Assembly ResolveAssembly(string assemblyName, params string[] directories)
    {
        string simpleName = new AssemblyName(assemblyName).Name;
        foreach (string directory in directories)
        {
            foreach (string extension in new[] { ".dll", ".exe" })
            {
                string candidate = Path.Combine(directory, simpleName + extension);
                if (File.Exists(candidate))
                    return Assembly.LoadFrom(candidate);
            }
        }
        return null;
    }
}
