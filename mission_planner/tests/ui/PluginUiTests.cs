// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Controls;
using MissionPlanner.Utilities;
using Newtonsoft.Json.Linq;
using NOMAD.MissionPlanner;

internal static class PluginUiTests
{
    private const BindingFlags Hidden = BindingFlags.Instance | BindingFlags.NonPublic;
    [DllImport("user32.dll")]
    private static extern IntPtr GetForegroundWindow();

    [STAThread]
    private static int Main(string[] args)
    {
        AppDomain.CurrentDomain.AssemblyResolve += (sender, request) =>
        {
            string name = new AssemblyName(request.Name).Name;
            string path = Path.Combine(args[0], name == "MissionPlanner" ? name + ".exe" : name + ".dll");
            return File.Exists(path) ? Assembly.LoadFrom(path) : null;
        };
        try { Run(args[1]); return 0; }
        catch (Exception error) { Console.Error.WriteLine(error); return 1; }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Run(string directory)
    {
        Settings.CustomUserDataDirectory = directory;
        Settings.AppConfigName = "NOMAD-UI-Tests";
        Settings.FileName = Path.Combine(directory, "host.xml");
        int selections = 0;
        NOMADMainScreen.SetVideoInputSelection((config, name) => { selections++; return true; });
        var cachedSelection = NOMADMainScreen.SelectVideoInput;
        NOMADMainScreen.SetVideoInputSelection(null);
        Check(!cachedSelection(new NOMADConfig(), "retired") && selections == 0,
            "A cached video selector invoked the retired plugin callback.");
        Gesture();
        Menu();
        SameScreen();
        Theme(directory);
        SettingsInputPause();
        InputEditorInterpretation(directory);
        using (var form = new NOMADForm())
        using (var widget = new TerminationWidget(null, CancellationToken.None))
        {
            form.Controls.Add(widget);
            Check(!widget.IsForeground(), "An inactive owned form was eligible for termination keys.");
            widget.SampleState(false, true, true, 0);
            Check(widget.PendingReview.IsCompleted, "An ineligible widget began a review.");
        }
        Console.WriteLine("PASS native menu/navigation, actual host theme, gesture and inactive-widget checks");
    }

    private static void Check(bool condition, string message)
    { if (!condition) { throw new InvalidOperationException(message); } }

    private static void InputEditorInterpretation(string directory)
    {
        foreach (string source in InputBinding.Sources)
        {
            var config = new NOMADConfig { ActuatorDefinitions = JArray.Parse("[{\"id\":\"payload\",\"name\":\"Example payload\",\"behavior\":\"ServoToggle\",\"channel\":1,\"safe_pwm\":1000}]") };
            config.Inputs.Add(JObject.FromObject(new InputBinding { Id = "stable", Name = "Named control", Device = "missing device",
                Source = source, Action = source == "Axis" ? "gimbal:pitch" : "payload:safe" }));
            using var editor = new InputBindingEditorForm(config, 0);
            var sourceChoice = (ComboBox)typeof(InputBindingEditorForm).GetField("_source", Hidden).GetValue(editor);
            var selected = (KeyValuePair<string, string>)sourceChoice.SelectedItem;
            Check(selected.Key == source && selected.Value == InputBindingEditorForm.SourceLabel(source), "Friendly source label changed persisted source identity.");
            var candidate = (JArray)typeof(InputBindingEditorForm).GetMethod("BuildCandidate", Hidden).Invoke(editor, null);
            Check(candidate[0]["Id"].Value<string>() == "stable" && candidate[0]["Source"].Value<string>() == source,
                "Input editor roundtrip changed source or stable identity.");
            var device = (ComboBox)typeof(InputBindingEditorForm).GetField("_device", Hidden).GetValue(editor);
            Check(((KeyValuePair<string, string>)device.SelectedItem).Value.StartsWith("Unavailable:"), "Missing device was not visibly unavailable.");
            var axisDetection = new InputListenState(DateTime.UtcNow);
            typeof(InputListenState).GetProperty("Axis", Hidden).SetValue(axisDetection, "Y");
            typeof(InputBindingEditorForm).GetMethod("ApplyDetection", Hidden).Invoke(editor, new object[] { axisDetection, source == "ButtonPair" });
            string interpreted = ((KeyValuePair<string, string>)sourceChoice.SelectedItem).Key;
            Check(interpreted == (source == "Button" ? "Axis" : source), "Listen changed an explicit switch interpretation or a paired endpoint.");
            var listenField = typeof(InputBindingEditorForm).GetField("_listen", Hidden);
            var pending = new InputListenState(DateTime.UtcNow); listenField.SetValue(editor, pending);
            sourceChoice.SelectedItem = InputBindingEditorForm.SourceChoices[interpreted == "Button" ? 2 : 0];
            Check(!pending.Active && listenField.GetValue(editor) == null &&
                !((Button)typeof(InputBindingEditorForm).GetField("_stopListen", Hidden).GetValue(editor)).Enabled,
                "Changing input type did not cancel and clean up Listen.");
            pending = new InputListenState(DateTime.UtcNow); listenField.SetValue(editor, pending);
            device.Items.Add(new KeyValuePair<string, string>("another missing", "Unavailable: another missing"));
            device.SelectedIndex = device.Items.Count - 1;
            Check(!pending.Active && listenField.GetValue(editor) == null, "Changing device did not retire Listen.");
            typeof(InputBindingEditorForm).GetMethod("SetSource", Hidden).Invoke(editor, new object[] { source });
            typeof(InputBindingEditorForm).GetMethod("UpdateSource", Hidden).Invoke(editor,
                new object[] { source == "Axis" ? "gimbal:pitch" : "payload:safe", "" });
            Capture(editor, directory, "input-" + source + ".png", editor.Size);
            editor.Scale(new SizeF(1.5f, 1.5f));
            Capture(editor, directory, "input-" + source + "-150.png", editor.Size);
        }
    }

    private static void SettingsInputPause()
    {
        var assembly = typeof(NOMADConfig).Assembly;
        var output = assembly.GetType("NOMAD.MissionPlanner.OutputController", true);
        var config = new NOMADConfig { AircraftName = "Pause fixture", GimbalArrowKeysEnabled = true };
        var profiles = new NOMADProfiles(config);
        config = profiles.Selected.Config;
        output.GetMethod("Initialize", BindingFlags.Static | BindingFlags.NonPublic).Invoke(null, new object[] { config });
        var filterType = assembly.GetType("NOMAD.MissionPlanner.GimbalArrowKeyFilter", true);
        var filter = (IDisposable)Activator.CreateInstance(filterType, new object[] { profiles });
        var pluginType = assembly.GetType("NOMAD.MissionPlanner.NOMADPlugin", true);
        var plugin = System.Runtime.Serialization.FormatterServices.GetUninitializedObject(pluginType);
        pluginType.GetField("_gimbalArrowKeyFilter", Hidden).SetValue(plugin, filter);
        pluginType.GetField("_joystickService", Hidden).SetValue(plugin, new NomadJoystickService(config));
        GimbalController.SetTargetAngles(0, 0);
        pluginType.GetMethod("PauseCustomInputs", Hidden).Invoke(plugin, null);
        using var list = new ComboBox(); list.Items.AddRange(new object[] { "First", "Second" }); list.SelectedIndex = 0;
        var down = Message.Create(list.Handle, 0x0100, (IntPtr)Keys.Down, IntPtr.Zero);
        Check(!Application.FilterMessage(ref down), "Settings pause still intercepted list navigation.");
        list.SelectedIndex = 1;
        Check(GimbalController.TargetPitchDeg == 0 && GimbalController.TargetRollDeg == 0 && config.GimbalArrowKeysEnabled,
            "Settings list navigation moved gimbal or changed saved arrow preference.");
        output.GetMethod("Shutdown", BindingFlags.Static | BindingFlags.NonPublic).Invoke(null, null);
    }

    private static void Gesture()
    {
        var gesture = new TerminationGesture();
        Check(!gesture.Update(true, true, true, 0), "Held Shift on entry activated review.");
        Check(!gesture.Update(true, false, true, 10), "A partial release rearmed review.");
        Check(!gesture.Update(true, false, false, 20), "Release activated review.");
        Check(!gesture.Update(true, true, false, 30), "One Shift activated review.");
        Check(gesture.Update(true, true, true, 780), "Inclusive 750 ms chord boundary was rejected.");
        Check(!gesture.Update(true, true, true, 790), "Held chord repeated review.");
        gesture.Update(true, false, false, 800);
        gesture.Update(true, false, true, 810);
        Check(!gesture.Update(true, true, true, 1561), "Expired chord activated review.");
        Check(!gesture.Update(true, true, true, 1570), "Expired held chord rearmed review.");
        gesture.Update(true, false, false, 1600);
        gesture.Update(true, true, false, 1610);
        Check(!gesture.Update(false, true, false, 1620), "Ineligible state activated review.");
        Check(!gesture.Update(true, true, true, 1630), "Focus reentry inherited pressed keys.");
        gesture.Update(true, false, false, 1640);
        Check(gesture.Update(true, true, true, 1650), "Fresh simultaneous edges did not activate review.");
    }

    private static void Mouse(NOMADMenuItem item, string method, MouseButtons button)
    {
        typeof(NOMADMenuItem).GetMethod(method, Hidden).Invoke(item,
            new object[] { new MouseEventArgs(button, 1, 2, 2, 0) });
    }

    private static void Menu()
    {
        using var form = new Form();
        using var strip = new MenuStrip();
        using var context = new ContextMenuStrip();
        form.Controls.Add(strip);
        strip.ContextMenuStrip = context;
        bool current = false;
        int opened = 0, dropdowns = 0;
        using var item = new NOMADMenuItem(strip, () => current, () => { current = true; opened++; });
        strip.Items.Add(item);
        item.DropDownItems.Add("Settings");
        item.DropDown.Opening += (sender, args) => { dropdowns++; args.Cancel = true; };
        var handle = strip.Handle;
        Mouse(item, "OnMouseDown", MouseButtons.Left);
        Mouse(item, "OnMouseUp", MouseButtons.Left);
        Check(opened == 1 && dropdowns == 0, "Outside left-click did not navigate without a dropdown.");
        Mouse(item, "OnMouseDown", MouseButtons.Left);
        Mouse(item, "OnMouseUp", MouseButtons.Left);
        Check(opened == 1 && dropdowns == 1, "Current-screen left-click navigated or lost its dropdown.");
        current = false;
        Mouse(item, "OnMouseDown", MouseButtons.Right);
        Mouse(item, "OnMouseUp", MouseButtons.Right);
        Check(opened == 1 && dropdowns == 2 && !current, "Right-click navigated or lost its dropdown.");
        Point over = strip.PointToScreen(new Point(item.Bounds.Left + 2, item.Bounds.Top + 2));
        Check(item.SuppressHostContext(strip, over), "Native host context was not suppressed over NOMAD.");
        Check(!item.SuppressHostContext(form, over) &&
            !item.SuppressHostContext(strip, strip.PointToScreen(new Point(strip.Width + 2, 2))),
            "Context suppression escaped NOMAD's bounds/source control.");
        WeakReference disposed = DisposedMenu(strip);
        GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
        Check(!disposed.IsAlive, "Host menu subscriptions retained a disposed NOMAD menu.");
        Check(ReferenceEquals(strip.ContextMenuStrip, context), "NOMAD replaced the host context menu.");
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static WeakReference DisposedMenu(MenuStrip strip)
    {
        var item = new NOMADMenuItem(strip, () => false, () => { });
        var weak = new WeakReference(item);
        strip.Items.Add(item);
        strip.Items.Remove(item);
        item.Dispose();
        return weak;
    }

    private static void SameScreen()
    {
        var previous = MainV2.View;
        using var root = new UserControl();
        using var current = new System.Windows.Forms.MyUserControl();
        using var session = new CancellationTokenSource();
        var switcher = new MainSwitcher(root);
        var screen = new MainSwitcher.Screen("NOMAD", typeof(NOMADMainScreen), false) { Control = current };
        switcher.screens.Add(screen);
        switcher.current = screen;
        MainV2.View = switcher;
        try
        {
            var plugin = new NOMADPlugin();
            typeof(NOMADPlugin).GetField("_videoShutdown", Hidden).SetValue(plugin, session);
            typeof(NOMADPlugin).GetMethod("ShowMainScreen", Hidden).Invoke(plugin, null);
            Check(ReferenceEquals(switcher.current.Control, current) && !current.IsDisposed,
                "Central navigation guard recreated the current nonpersistent screen.");
        }
        finally { MainV2.View = previous; }
    }

    private static IEnumerable<Control> Tree(Control root)
    {
        yield return root;
        foreach (Control child in root.Controls)
        { foreach (Control nested in Tree(child)) { yield return nested; } }
    }

    private static void RetainsTheme(Control root)
    {
        var colors = new Dictionary<Control, Tuple<Color, Color>>();
        foreach (var control in Tree(root))
        { colors[control] = Tuple.Create(control.BackColor, control.ForeColor); }
        ThemeManager.ApplyThemeTo(root);
        foreach (var entry in colors)
        { Check(entry.Key.BackColor == entry.Value.Item1 && entry.Key.ForeColor == entry.Value.Item2,
            root.GetType().Name + " lost its NOMAD palette under actual host theming."); }
    }

    private sealed class InactiveRenderForm : Form
    {
        protected override bool ShowWithoutActivation => true;
        protected override CreateParams CreateParams
        {
            get { var value = base.CreateParams; value.ExStyle |= 0x08000000; return value; }
        }
    }

    private static void Capture(Control control, string directory, string name, Size size)
    {
        using var host = new InactiveRenderForm
        {
            ClientSize = size, StartPosition = FormStartPosition.Manual,
            Location = new Point(-20000, -20000), ShowInTaskbar = false,
        };
        if (control is Form dialog) { dialog.TopLevel = false; dialog.FormBorderStyle = FormBorderStyle.None; }
        control.Dock = DockStyle.Fill;
        host.Controls.Add(control);
        try
        {
            IntPtr foreground = GetForegroundWindow();
            host.Show();
            control.Show();
            host.PerformLayout();
            Application.DoEvents();
            Check(GetForegroundWindow() == foreground, "Owned render fixture changed foreground focus.");
            using var image = new Bitmap(control.Width, control.Height);
            control.DrawToBitmap(image, new Rectangle(Point.Empty, control.Size));
            image.Save(Path.Combine(directory, name));
        }
        finally { control.Hide(); host.Controls.Remove(control); }
    }

    private static void Theme(string directory)
    {
        var config = new NOMADConfig { AircraftName = "Synthetic UI" };
        using var service = new NotificationService();
        var previous = NotificationService.Shared;
        NotificationService.Shared = service;
        try
        {
            using var screen = new NOMADMainScreen(config);
            using var settings = new NOMADSettingsForm(new NOMADProfiles(config));
            using var payload = new PayloadEditorForm(new JArray(), -1, "synthetic");
            using var router = new RouterLinkEditorForm(config, -1, "synthetic");
            using var input = new InputBindingEditorForm(config, -1);
            using var form = new NOMADForm { BackColor = NOMADTheme.BG_DARK };
            form.Controls.Add(ControlFactory.Button("Save", backColor: NOMADTheme.BTN_PRIMARY));
            foreach (var root in new Control[] { screen, settings, payload, router, input, form }) { RetainsTheme(root); }
            typeof(NOMADMainScreen).GetMethod("ShowView", Hidden).Invoke(screen, new object[] { "Dashboard" });
            Capture(screen, directory, "dashboard.png", new Size(1180, 760));
            Capture(settings, directory, "settings.png", settings.Size);
            Capture(input, directory, "input-editor.png", input.Size);
            config.VideoInputs.Add(new VideoInputProfile { Name = "Example receiver", Url = "rtsp://camera.invalid/live" });
            config.VideoInputs.Add(new VideoInputProfile { Name = "Alternate receiver", Url = "rtsp://alternate.invalid/live" });
            config.SelectedVideoInputName = "Example receiver";
            using var player = new VideoInputPlayer(config, true, CancellationToken.None,
                () => throw new InvalidOperationException("A stopped render must not start a pipeline."), (owner, name) => false);
            Capture(player, directory, "video-selector.png", new Size(800, 500));
            foreach (var name in new[] { "GimbalJoystickWindow", "NotificationToast+ToastForm" })
            {
                var type = typeof(NOMADConfig).Assembly.GetType("NOMAD.MissionPlanner." + name, true);
                Check(Attribute.IsDefined(type, typeof(PreventThemingAttribute)), name + " has no host theme opt-out.");
            }
            Check(((CheckBox)typeof(NOMADSettingsForm).GetField("_chkRouterClientEnabled", Hidden)
                .GetValue(settings)).Text == "Enable", "Router checkbox is not labelled Enable.");
            using var host = new Form { BackColor = Color.Magenta };
            ThemeManager.ApplyThemeTo(host);
            Check(host.BackColor != Color.Magenta, "NOMAD disabled host theming globally.");
        }
        finally { NotificationService.Shared = previous; }
    }
}
