// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
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
        NotificationRows();
        NotificationClockAndLayout(directory);
        ToastAttachmentLifetime();
        QueuedUiUpdates();
        FlightDataMenuLifetime();
        SameScreen();
        SameViewNavigation(directory);
        VideoSourceCalibration(directory);
        ModelCatalogNavigation(directory);
        var padded = new System.Windows.Controls.Grid { Margin = new System.Windows.Thickness(12) };
        using (var bridge = WpfUi.Host(padded))
        { Check(padded.Margin == new System.Windows.Thickness(12), "The native bridge removed the WPF page padding."); }
        Theme(directory);
        SettingsInputPause();
        GimbalScreenAxes();
        GimbalPreferenceFailure(directory);
        InputEditorInterpretation(directory);
        using (var form = new NOMADForm())
        using (var widget = new TerminationWidget(null, CancellationToken.None))
        {
            form.Controls.Add(widget);
            Check(!widget.IsForeground(), "An inactive owned form was eligible for termination keys.");
            widget.SampleState(false, true, true, 0);
            Check(widget.PendingReview.IsCompleted, "An ineligible widget began a review.");
        }
        int interactionSamples = 0;
        using (var widget = new TerminationWidget(null, CancellationToken.None,
            canInteract: () => { interactionSamples++; return false; }))
        {
            typeof(TerminationWidget).GetMethod("Sample", Hidden).Invoke(widget, new object[] { null, EventArgs.Empty });
            Check(interactionSamples == 1 && widget.PendingReview.IsCompleted,
                "Native timer bypassed the widget interaction predicate");
        }
        Console.WriteLine("PASS native menu/navigation, actual host theme, gesture and inactive-widget checks");
    }

    private static void Check(bool condition, string message)
    { if (!condition) { throw new InvalidOperationException(message); } }

    private static void QueuedUiUpdates()
    {
        using var owner = new Control();
        var run = typeof(NOMADConfig).Assembly.GetType("NOMAD.MissionPlanner.UiAsync", true).GetMethod("RunSync");
        bool premature = false;
        var earlyWorker = new Thread(() => run.Invoke(null,
            new object[] { owner, (Action)(() => premature = true), "before handle" }));
        earlyWorker.IsBackground = true;
        earlyWorker.Start();
        Check(earlyWorker.Join(2000), "Pre-handle UI callback blocked its worker.");
        Check(!premature && !owner.IsHandleCreated, "UI callback ran on a worker before its owner had a handle.");
        var handle = owner.Handle;
        int unhandled = 0;
        bool completed = false;
        Exception workerError = null;
        ThreadExceptionEventHandler onError = (sender, args) => unhandled++;
        Application.ThreadException += onError;
        try
        {
            var worker = new Thread(() =>
            {
                try
                {
                    run.Invoke(null, new object[] { owner, (Action)(() => throw new Exception("fixture")), "throw" });
                    run.Invoke(null, new object[] { owner, (Action)(() => completed = true), "complete" });
                }
                catch (Exception error) { workerError = error; }
            });
            worker.IsBackground = true;
            worker.Start();
            Check(worker.Join(2000), "UI worker blocked while queueing updates.");
            Check(workerError == null, "Queueing a UI update failed: " + workerError);
            var deadline = System.Diagnostics.Stopwatch.StartNew();
            while (!completed && deadline.ElapsedMilliseconds < 2000) { Application.DoEvents(); Thread.Sleep(1); }
            Check(completed && unhandled == 0, "Queued UI error escaped the guard or prevented the next update.");
            run.Invoke(null, new object[] { owner, (Action)(() => completed = false), "retire" });
            owner.Dispose();
            run.Invoke(null, new object[] { owner, (Action)(() => completed = true), "disposed" });
            Check(!completed, "Disposed owner accepted a UI update.");
        }
        finally { Application.ThreadException -= onError; }
    }

    private static void NotificationRows()
    {
        using var service = new NotificationService();
        using var panel = new NotificationPanel(service);
        service.AddNotification(NotificationSeverity.Info, NotificationCategory.System, "Before handle", "Fixture");
        var handle = panel.Handle;
        var list = (System.Windows.Controls.StackPanel)typeof(NotificationPanel).GetField("_notificationListPanel", Hidden).GetValue(panel);
        Check(list.Children.Count == 1 && ReferenceEquals(((System.Windows.FrameworkElement)list.Children[0]).Tag, service.Notifications[0]),
            "Handle creation did not load the current notification snapshot.");
        var old = (System.Windows.FrameworkElement)list.Children[0];
        panel.RefreshNotificationList();
        Check(old.Parent == null && list.Children.Count == 1, "Notification refresh retained an attached obsolete row.");
        var older = service.Notifications[0];
        older.Timestamp = DateTime.Now.AddMinutes(-1);
        service.AddNotification(NotificationSeverity.Critical, NotificationCategory.System, "Newest", "Fixture");
        var newest = service.Notifications[0];
        newest.Timestamp = older.Timestamp.AddHours(-1);
        panel.Size = new Size(700, 400);
        panel.RefreshNotificationList();
        panel.PerformLayout(); list.UpdateLayout();
        var newestRow = list.Children.Cast<System.Windows.FrameworkElement>().Single(row => ReferenceEquals(row.Tag, newest));
        var olderRow = list.Children.Cast<System.Windows.FrameworkElement>().Single(row => ReferenceEquals(row.Tag, older));
        Check(list.Children.IndexOf(newestRow) < list.Children.IndexOf(olderRow),
            "Newest notification was below the older row.");
        for (int index = 0; index < 55; index++)
        { service.AddNotification(NotificationSeverity.Info, NotificationCategory.System, "Row " + index, "Fixture"); }
        Check(list.Children.Count == 50, "Notification rows exceeded their visible bound.");
        var rows = list.Children.Cast<System.Windows.FrameworkElement>().ToArray();
        var worker = new Thread(() =>
            service.AddNotification(NotificationSeverity.Warning, NotificationCategory.System, "Queued", "Fixture"));
        worker.IsBackground = true;
        worker.Start();
        Check(worker.Join(2000), "Notification publisher blocked the UI.");
        service.ClearAll();
        Application.DoEvents();
        Check(list.Children.Count == 0 && rows.All(row => row.Parent == null),
            "A queued notification returned after Clear or removed rows stayed attached.");
    }

    private static void NotificationClockAndLayout(string directory)
    {
        using var service = new NotificationService();
        service.AddNotification(NotificationSeverity.Warning, NotificationCategory.System, "Clock", "First warning");
        var cooldowns = (System.Collections.IDictionary)typeof(NotificationService)
            .GetField("_lastNotificationTime", Hidden).GetValue(service);
        Check(cooldowns["System_Clock"] is long, "Warning cooldown does not use a monotonic clock");
        cooldowns["System_Clock"] = System.Diagnostics.Stopwatch.GetTimestamp() - 29 * System.Diagnostics.Stopwatch.Frequency;
        service.AddNotification(NotificationSeverity.Warning, NotificationCategory.System, "Clock", "Too soon");
        Check(service.Notifications.Count == 1, "Warning cooldown admitted an early repeat");
        cooldowns["System_Clock"] = System.Diagnostics.Stopwatch.GetTimestamp() - 31 * System.Diagnostics.Stopwatch.Frequency;
        service.AddNotification(NotificationSeverity.Warning, NotificationCategory.System, "Clock", "Elapsed repeat");
        Check(service.Notifications.Count == 2, "Elapsed warning cooldown still suppressed a repeat");
        service.ClearAll();
        const string title = "Battery warning: review the aircraft before continuing";
        const string message = "Battery voltage is below the configured warning level. Check the aircraft's voltage " +
            "and remaining capacity, then follow the operator procedure. Keep the full warning visible while resizing this window.";
        service.AddNotification(NotificationSeverity.Warning, NotificationCategory.Battery, title, message);
        using var panel = new NotificationPanel(service);
        Capture(panel, directory, "notifications-narrow.png", new Size(380, 520));
        var list = (System.Windows.Controls.StackPanel)typeof(NotificationPanel).GetField("_notificationListPanel", Hidden).GetValue(panel);
        var row = (System.Windows.FrameworkElement)list.Children[0];
        var body = WpfTree(row).OfType<System.Windows.Controls.TextBlock>().SingleOrDefault(label => label.Text == message);
        Check(body != null, "Notification permanently truncated its message");
        Check(body.ActualHeight >= body.DesiredSize.Height &&
            body.TransformToAncestor(row).TransformBounds(new System.Windows.Rect(body.RenderSize)).Bottom <= row.ActualHeight,
            "Narrow notification clipped its wrapped message");
        double narrowHeight = row.ActualHeight;
        Capture(panel, directory, "notifications-wide.png", new Size(760, 460));
        Check(row.ActualHeight < narrowHeight && body.ActualHeight >= body.DesiredSize.Height,
            "Notification did not reflow after widening the window");
    }

    private static void ToastAttachmentLifetime()
    {
        using var owner = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-20000, -20000) };
        using var first = new NotificationService();
        using var second = new NotificationService();
        var handle = owner.Handle;
        var open = (System.Collections.IList)typeof(NotificationToast)
            .GetField("_open", BindingFlags.Static | BindingFlags.NonPublic).GetValue(null);
        try
        {
            foreach (bool sameService in new[] { false, true })
            {
                NotificationToast.Attach(first, owner);
                var worker = new Thread(() => first.AddNotification(NotificationSeverity.Warning,
                    NotificationCategory.System, "Retired " + sameService, "Fixture")) { IsBackground = true };
                worker.Start();
                Check(worker.Join(2000), "Toast publisher blocked while queueing a callback");
                var current = sameService ? first : second;
                NotificationToast.Attach(current, owner);
                Application.DoEvents();
                Check(open.Count == 0, "A queued warning crossed notification attachments");
                current.AddNotification(NotificationSeverity.Warning, NotificationCategory.System,
                    "Current " + sameService, "Fixture");
                Check(open.Count == 1, "Current attachment stopped displaying warnings");
                var toast = (Form)open[0];
                NotificationToast.Detach();
                Check(open.Count == 0 && toast.IsDisposed, "Detaching retained its toast or form resources");
            }
        }
        finally { NotificationToast.Detach(); }
    }

    private static void FlightDataMenuLifetime()
    {
        using var menu = new ContextMenuStrip();
        var hostItem = menu.Items.Add("Host action");
        var plugin = new NOMADPlugin();
        var owned = new[] { new ToolStripMenuItem("NOMAD fixture"), new ToolStripMenuItem("NOMAD fixture 2") };
        int disposed = 0;
        foreach (var item in owned) { item.Disposed += (sender, args) => disposed++; }
        menu.Items.AddRange(owned);
        typeof(NOMADPlugin).GetField("_flightDataMenuItems", Hidden).SetValue(plugin, owned);
        var stop = typeof(NOMADPlugin).GetMethod("StopCustomControls", Hidden);
        stop.Invoke(plugin, null);
        stop.Invoke(plugin, null);
        Check(menu.Items.Count == 1 && ReferenceEquals(menu.Items[0], hostItem) &&
            disposed == 2 && !hostItem.IsDisposed,
            $"Plugin cleanup retained menus or removed a host item: count={menu.Items.Count}, " +
            $"owned disposed={disposed}, host disposed={hostItem.IsDisposed}.");
    }

    private static void InputEditorInterpretation(string directory)
    {
        foreach (string source in InputBinding.Sources)
        {
            var config = new NOMADConfig { ActuatorDefinitions = JArray.Parse("[{\"id\":\"payload\",\"name\":\"Example payload\",\"behavior\":\"ServoToggle\",\"channel\":1,\"safe_pwm\":1000}]") };
            config.Inputs.Add(JObject.FromObject(new InputBinding { Id = "stable", Name = "Named control", Device = "missing device",
                Source = source, Action = source == "Axis" ? "gimbal:pitch" : "payload:safe" }));
            using var editor = new InputBindingEditorForm(config, 0);
            var sourceChoice = (System.Windows.Controls.ComboBox)typeof(InputBindingEditorForm).GetField("_source", Hidden).GetValue(editor);
            var selected = (KeyValuePair<string, string>)sourceChoice.SelectedItem;
            Check(selected.Key == source && selected.Value == InputBindingEditorForm.SourceLabel(source), "Friendly source label changed persisted source identity.");
            var candidate = (JArray)typeof(InputBindingEditorForm).GetMethod("BuildCandidate", Hidden).Invoke(editor, null);
            Check(candidate[0]["Id"].Value<string>() == "stable" && candidate[0]["Source"].Value<string>() == source,
                "Input editor roundtrip changed source or stable identity.");
            var device = (System.Windows.Controls.ComboBox)typeof(InputBindingEditorForm).GetField("_device", Hidden).GetValue(editor);
            Check(((KeyValuePair<string, string>)device.SelectedItem).Value.StartsWith("Unavailable:"), "Missing device was not visibly unavailable.");
            var axisDetection = new InputListenState(System.Diagnostics.Stopwatch.GetTimestamp());
            typeof(InputListenState).GetProperty("Axis", Hidden).SetValue(axisDetection, "Y");
            typeof(InputBindingEditorForm).GetMethod("ApplyDetection", Hidden).Invoke(editor, new object[] { axisDetection, source == "ButtonPair" });
            string interpreted = ((KeyValuePair<string, string>)sourceChoice.SelectedItem).Key;
            Check(interpreted == (source == "Button" ? "Axis" : source), "Listen changed an explicit switch interpretation or a paired endpoint.");
            var listenField = typeof(InputBindingEditorForm).GetField("_listen", Hidden);
            var pending = new InputListenState(System.Diagnostics.Stopwatch.GetTimestamp()); listenField.SetValue(editor, pending);
            sourceChoice.SelectedItem = InputBindingEditorForm.SourceChoices[interpreted == "Button" ? 2 : 0];
            Check(!pending.Active && listenField.GetValue(editor) == null &&
                !((System.Windows.Controls.Button)typeof(InputBindingEditorForm).GetField("_stopListen", Hidden).GetValue(editor)).IsEnabled,
                "Changing input type did not cancel and clean up Listen.");
            pending = new InputListenState(System.Diagnostics.Stopwatch.GetTimestamp()); listenField.SetValue(editor, pending);
            device.Items.Add(new KeyValuePair<string, string>("another missing", "Unavailable: another missing"));
            device.SelectedIndex = device.Items.Count - 1;
            Check(!pending.Active && listenField.GetValue(editor) == null, "Changing device did not retire Listen.");
            typeof(InputBindingEditorForm).GetMethod("SetSource", Hidden).Invoke(editor, new object[] { source });
            typeof(InputBindingEditorForm).GetMethod("UpdateSource", Hidden).Invoke(editor,
                new object[] { source == "Axis" ? "gimbal:pitch" : "payload:safe", "" });
            Capture(editor, directory, "input-" + source + ".png", editor.Size);
            Check(WpfVisualTree(sourceChoice).OfType<System.Windows.Controls.TextBlock>()
                .Any(text => text.Text == InputBindingEditorForm.SourceLabel(source)),
                "Input selector displayed its persisted object instead of the friendly source label.");
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

    private static void GimbalScreenAxes()
    {
        var config = new NOMADConfig { AircraftName = "Screen-axis fixture", GimbalMountMode = (int)MountMode.RcTargeting,
            JoystickGimbalHorizontalAxis = "Roll", JoystickGimbalVerticalAxis = "Yaw",
            JoystickGimbalHorizontalInverted = true };
        OutputController.Initialize(config); GimbalController.ApplyProfile(config);
        var type = typeof(GimbalJoystickWindow);
        using var window = (Form)type.GetConstructor(Hidden, null, new[] { typeof(NOMADConfig) }, null)
            .Invoke(new object[] { config });
        try
        {
            window.Location = new Point(-20000, -20000); window.Show(); Application.DoEvents();
            ((System.Windows.Forms.Timer)type.GetField("_streamTimer", Hidden).GetValue(window)).Stop();
            var pad = (System.Windows.FrameworkElement)type.GetField("_pad", Hidden).GetValue(window);
            Check((string)pad.GetType().GetProperty("HorizontalLabel", Hidden).GetValue(pad) == "Roll" &&
                (string)pad.GetType().GetProperty("VerticalLabel", Hidden).GetValue(pad) == "Yaw" &&
                (bool)pad.GetType().GetProperty("HorizontalInverted", Hidden).GetValue(pad),
                "Gimbal screen pad labels did not reflect the aircraft axis mapping.");
            var tick = type.GetMethod("OnStreamTick", Hidden);
            tick.Invoke(window, new object[] { null, EventArgs.Empty });
            type.GetField("_stickX", Hidden).SetValue(window, .5f);
            type.GetField("_stickY", Hidden).SetValue(window, .25f);
            tick.Invoke(window, new object[] { null, EventArgs.Empty });
            Check(Math.Abs(GimbalController.TargetRollDeg + 1.5f) < .001f &&
                Math.Abs(GimbalController.TargetYawDeg - .75f) < .001f && GimbalController.TargetPitchDeg == 0,
                "Gimbal screen pad did not apply the selected axes/inversion or moved its unselected axis.");
            Check(((System.Windows.Controls.TextBlock)type.GetField("_lblYaw", Hidden).GetValue(window)).Text.Contains(.75f.ToString("0.0")),
                "Gimbal screen omitted its updated yaw target readout.");
        }
        finally { window.Close(); OutputController.Shutdown(); }
    }

    private static void GimbalPreferenceFailure(string directory)
    {
        string blocker = Path.Combine(directory, "profile-parent-blocker");
        File.WriteAllText(blocker, "Owned save-failure fixture");
        var profiles = new NOMADProfiles(new NOMADConfig { AircraftName = "Preference failure fixture" });
        typeof(NOMADProfiles).GetField("_path", Hidden).SetValue(profiles, Path.Combine(blocker, "profiles.json"));
        OutputController.Initialize(profiles.Selected.Config);
        GimbalController.ApplyProfile(profiles.Selected.Config);
        GimbalController.SetTargetAngles(0, 0);
        using var filter = new GimbalArrowKeyFilter(profiles);
        using var service = new NotificationService();
        var previous = NotificationService.Shared;
        NotificationService.Shared = service;
        using var window = (Form)typeof(GimbalJoystickWindow).GetConstructor(Hidden, null,
            new[] { typeof(NOMADConfig) }, null).Invoke(new object[] { profiles.Selected.Config });
        try
        {
            var checkbox = (System.Windows.Controls.CheckBox)typeof(GimbalJoystickWindow).GetField("_chkArrowKeys", Hidden).GetValue(window);
            checkbox.IsChecked = true;
            Check(checkbox.IsChecked != true && !GimbalArrowKeyFilter.Enabled && !profiles.Selected.Config.GimbalArrowKeysEnabled &&
                service.Notifications.Count == 1 && service.Notifications[0].Severity == NotificationSeverity.Warning,
                "Failed keyboard-preference save escaped the UI, retained a false checkbox, or hid its warning.");
            Capture(window, directory, "gimbal-save-failure.png", window.Size);
            using var notifications = new NotificationPanel(service);
            Capture(notifications, directory, "gimbal-save-warning.png", new Size(450, 260));
            string validPath = Path.Combine(directory, "keyboard-profile.json");
            typeof(NOMADProfiles).GetField("_path", Hidden).SetValue(profiles, validPath);
            bool saving = true;
            filter.CanSave = () => !saving;
            checkbox.IsChecked = true;
            Check(checkbox.IsChecked != true && !GimbalArrowKeyFilter.Enabled && !profiles.Selected.Config.GimbalArrowKeysEnabled &&
                !File.Exists(validPath), "A concurrent save admitted a keyboard preference or changed its UI state.");
            saving = false;
            checkbox.IsChecked = true;
            Check(checkbox.IsChecked == true && GimbalArrowKeyFilter.Enabled &&
                NOMADProfiles.LoadFromPaths(validPath, validPath + ".bak").Selected.Config.GimbalArrowKeysEnabled,
                "Keyboard preference could not be retried after the other save completed.");
            Capture(window, directory, "gimbal-save-recovered.png", window.Size);
        }
        finally { window.Close(); OutputController.Shutdown(); NotificationService.Shared = previous; }
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

    private static IEnumerable<System.Windows.DependencyObject> WpfTree(System.Windows.DependencyObject root)
    {
        yield return root;
        foreach (object item in System.Windows.LogicalTreeHelper.GetChildren(root))
        {
            if (item is System.Windows.DependencyObject child)
            { foreach (var nested in WpfTree(child)) { yield return nested; } }
        }
    }

    private static IEnumerable<System.Windows.DependencyObject> WpfVisualTree(System.Windows.DependencyObject root)
    {
        yield return root;
        for (int index = 0; index < System.Windows.Media.VisualTreeHelper.GetChildrenCount(root); index++)
        {
            foreach (var nested in WpfVisualTree(System.Windows.Media.VisualTreeHelper.GetChild(root, index)))
            { yield return nested; }
        }
    }

    private static void SameViewNavigation(string directory)
    {
        var config = new NOMADConfig { AircraftName = "Navigation fixture" };
        config.VideoInputs.Add(new VideoInputProfile { Name = "Example camera", Url = "rtsp://camera.invalid/live" });
        config.SelectedVideoInputName = "Example camera";
        using var screen = new NOMADMainScreen(config);
        var show = typeof(NOMADMainScreen).GetMethod("ShowView", Hidden);
        show.Invoke(screen, new object[] { "Video" });
        var view = (NOMADVideoView)typeof(NOMADMainScreen).GetField("_videoView", Hidden).GetValue(screen);
        var player = ((VideoInputPlayer)typeof(NOMADVideoView).GetField("_videoHost", Hidden).GetValue(view)).Player;
        string status = player.TrackingStatus;
        int removals = 0;
        view.Parent.ControlRemoved += (sender, args) => { if (ReferenceEquals(args.Control, view)) { removals++; } };
        show.Invoke(screen, new object[] { "Video" });
        Check(removals == 0 && player.TrackingStatus == status,
            "Selecting the current Video page detached it or interrupted its tracking state.");
        Capture(screen, directory, "video-navigation.png", new Size(1180, 760));
        screen.Scale(new SizeF(1.5f, 1.5f));
        Capture(screen, directory, "video-navigation-150.png", new Size(1380, 980));
        show.Invoke(screen, new object[] { "Links" });
        Check(removals == 1 && player.TrackingStatus != status,
            "Leaving Video did not retire its tracking state.");
    }

    private static void VideoSourceCalibration(string directory)
    {
        var config = new NOMADConfig { AircraftName = "Example Copter" };
        config.VideoInputs.Add(new VideoInputProfile { Name = "Example camera", Url = "rtsp://camera.invalid/live",
            TrackingCalibrated = true });
        config.SelectedVideoInputName = "Example camera";
        using var settings = new NOMADSettingsForm(new NOMADProfiles(config));
        var tabs = (System.Windows.Controls.TabControl)typeof(NOMADSettingsForm).GetField("_tabControl", Hidden).GetValue(settings);
        var video = tabs.Items.Cast<System.Windows.Controls.TabItem>().Single(page => (string)page.Header == "Video");
        tabs.SelectedItem = video;
        ((System.Windows.Controls.TextBox)typeof(NOMADSettingsForm).GetField("_txtVideoUrl", Hidden).GetValue(settings)).Text =
            "rtsp://replacement-camera.invalid/live";
        var calibrated = (System.Windows.Controls.CheckBox)typeof(NOMADSettingsForm).GetField("_chkTrackingCalibrated", Hidden).GetValue(settings);
        Check(calibrated.IsChecked != true, "Changed network source still displays a calibration confirmation.");
        Capture(settings, directory, "camera-source-changed.png", settings.Size);
        calibrated.BringIntoView();
        Capture(settings, directory, "camera-calibration-review.png", settings.Size);
        settings.Scale(new SizeF(1.5f, 1.5f));
        calibrated.BringIntoView();
        Capture(settings, directory, "camera-calibration-review-150.png", settings.Size);
    }

    private static void ModelCatalogNavigation(string directory)
    {
        var config = new NOMADConfig();
        config.VideoInputs.Add(new VideoInputProfile { Name = "Camera A", Url = "rtsp://camera-a.invalid/live" });
        config.VideoInputs.Add(new VideoInputProfile { Name = "Camera B", Url = "rtsp://camera-b.invalid/live" });
        config.SelectedVideoInputName = "Camera A";
        using var host = new VideoInputPlayer(config, true, CancellationToken.None,
            () => throw new InvalidOperationException("A stopped render must not start capture."),
            (owner, name) => { owner.SelectedVideoInputName = name; return true; },
            (owner, name, path) => System.Threading.Tasks.Task.FromResult(false));
        config.SelectedVideoInputName = "Camera B";
        host.RefreshInput();
        var models = (System.Windows.Controls.ComboBox)typeof(VideoInputPlayer).GetField("_models", Hidden).GetValue(host);
        var deadline = System.Diagnostics.Stopwatch.StartNew();
        while (models.SelectedItem == null && deadline.ElapsedMilliseconds < 2000)
        { Application.DoEvents(); Thread.Sleep(1); }
        Check(models.IsEnabled && models.SelectedItem != null, "Camera switch left model selection unavailable.");
        host.Player.VideoControls.MoreButton.RaiseEvent(new System.Windows.RoutedEventArgs(System.Windows.Controls.Button.ClickEvent));
        Capture(host, directory, "model-catalog-camera-switch.png", new Size(900, 600));
        Capture(host, directory, "model-catalog-camera-switch-narrow.png", new Size(380, 600));
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

    private static void UsesWpfBody(Control root)
    {
        foreach (var control in Tree(root))
        {
            Check(!(control is ButtonBase || control is Label || control is TextBoxBase ||
                control is ListControl || control is NumericUpDown || control is TabControl || control is DataGridView),
                root.GetType().Name + " retained a WinForms widget: " + control.GetType().Name);
        }
        Check(Tree(root).OfType<System.Windows.Forms.Integration.ElementHost>().Any(),
            root.GetType().Name + " does not host its WPF body");
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
            using var image = NOMAD.Testing.ControlRender.Capture(control);
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
            int simulationOwners = 0;
            NOMADMainScreen.SetSimulationFactory(() => { simulationOwners++; return null; });
            using var screen = new NOMADMainScreen(config);
            Check(simulationOwners == 0, "Constructing the normal screen created a simulation owner.");
            var simulationButton = (System.Windows.Controls.Button)typeof(NOMADMainScreen).GetField("_btnSimulation", Hidden).GetValue(screen);
            Check((string)simulationButton.Content == "Testing & Simulation",
                "Simulation navigation lost its label or treated the ampersand as a shortcut.");
            using var settings = new NOMADSettingsForm(new NOMADProfiles(config));
            using var payload = new PayloadEditorForm(new JArray(), -1, "synthetic");
            using var router = new RouterLinkEditorForm(config, -1, "synthetic");
            using var input = new InputBindingEditorForm(config, -1);
            using var payloadControls = new ActuatorControlPanel(config);
            var payloadStatus = (System.Windows.Controls.TextBlock)typeof(ActuatorControlPanel).GetField("_status", Hidden).GetValue(payloadControls);
            Check(string.IsNullOrEmpty(payloadStatus.Text) && payloadStatus.Visibility == System.Windows.Visibility.Collapsed,
                "Idle payload controls displayed boilerplate instead of action status.");
            using var form = new NOMADForm { BackColor = NOMADTheme.BG_DARK };
            using var simulationSession = new SimulationSession();
            using var simulation = new NOMADSimulationView(simulationSession, CancellationToken.None, detectOnOpen: false);
            form.Controls.Add(WpfUi.Host(WpfUi.Action("Save", () => { })));
            foreach (var root in new Control[] { screen, settings, payload, router, input, form, simulation })
            { RetainsTheme(root); UsesWpfBody(root); }
            typeof(NOMADMainScreen).GetMethod("ShowView", Hidden).Invoke(screen, new object[] { "Dashboard" });
            Capture(screen, directory, "dashboard.png", new Size(1180, 760));
            Capture(settings, directory, "settings.png", settings.Size);
            Capture(input, directory, "input-editor.png", input.Size);
            Capture(simulation, directory, "simulation.png", new Size(1180, 760));
            Check(!simulationSession.Status.Active, "An inactive simulation render launched a simulator.");
            config.VideoInputs.Add(new VideoInputProfile { Name = "Example receiver", Url = "rtsp://camera.invalid/live" });
            config.VideoInputs.Add(new VideoInputProfile { Name = "Alternate receiver", Url = "rtsp://alternate.invalid/live" });
            config.SelectedVideoInputName = "Example receiver";
            using var player = new VideoInputPlayer(config, true, CancellationToken.None,
                () => throw new InvalidOperationException("A stopped render must not start a pipeline."), (owner, name) => false);
            Capture(player, directory, "video-selector.png", new Size(800, 500));
            var videoSession = (VideoSession)typeof(EmbeddedVideoPlayer).GetField("_session", Hidden).GetValue(player.Player);
            var visionStatus = (System.Windows.Controls.TextBlock)typeof(EmbeddedVideoPlayer).GetField("_lblVisionStatus", Hidden).GetValue(player.Player);
            foreach (var prerequisite in new[] { ("gpu", MediaWire.NvidiaUnavailable), ("runtime", MediaWire.VisualCppRequired) })
            {
                typeof(VideoSession).GetField("_trackerError", Hidden).SetValue(videoSession, prerequisite.Item2);
                typeof(EmbeddedVideoPlayer).GetMethod("UpdateVisionStatus", Hidden).Invoke(player.Player, null);
                Check(visionStatus.Text == prerequisite.Item2, "Vision status hid its actionable prerequisite");
                Capture(player, directory, "vision-" + prerequisite.Item1 + ".png", new Size(800, 500));
                Capture(player, directory, "vision-" + prerequisite.Item1 + "-narrow.png", new Size(380, 500));
                Check(visionStatus.ActualHeight >= visionStatus.DesiredSize.Height,
                    "Vision prerequisite was clipped in a narrow viewport");
            }
            foreach (var name in new[] { "GimbalJoystickWindow", "NotificationToast+ToastForm" })
            {
                var type = typeof(NOMADConfig).Assembly.GetType("NOMAD.MissionPlanner." + name, true);
                Check(Attribute.IsDefined(type, typeof(PreventThemingAttribute)), name + " has no host theme opt-out.");
            }
            Check((string)((System.Windows.Controls.CheckBox)typeof(NOMADSettingsForm).GetField("_chkRouterClientEnabled", Hidden)
                .GetValue(settings)).Content == "Enable", "Router checkbox is not labelled Enable.");
            using var host = new Form { BackColor = Color.Magenta };
            ThemeManager.ApplyThemeTo(host);
            Check(host.BackColor != Color.Magenta, "NOMAD disabled host theming globally.");
        }
        finally { NOMADMainScreen.SetSimulationFactory(null); NotificationService.Shared = previous; }
    }
}
