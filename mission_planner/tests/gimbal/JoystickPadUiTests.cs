// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Windows.Forms;

internal static class JoystickPadUiTests
{
    private const uint WM_LBUTTONDOWN = 0x0201;
    private const uint WM_LBUTTONUP = 0x0202;
    private const uint WM_MOUSEMOVE = 0x0200;
    private const uint WM_MOUSELEAVE = 0x02A3;
    private const int PAD_CENTER_X = 160;
    private const int PAD_CENTER_Y = 150;
    // The input vector extends 2 px past the control's 120 px drawing cap to test clamping.
    private const int PAD_TEST_RADIUS = 122;

    private static int _failures;

    [STAThread]
    private static int Main(string[] args)
    {
        if (args.Length != 1)
        {
            Console.Error.WriteLine("Pass the built NOMADPlugin.dll path.");
            return 2;
        }

        Application.EnableVisualStyles();
        Assembly plugin = Assembly.LoadFrom(args[0]);
        Type padType = FindPadType(plugin);
        RunInteractionChecks(plugin, padType);
        return _failures == 0 ? 0 : 1;
    }

    private static Type FindPadType(Assembly plugin)
    {
        Type padType = plugin.GetType("NOMAD.MissionPlanner.GimbalJoystickWindow+JoystickPad")
            ?? plugin.GetType("NOMAD.MissionPlanner.JoystickPad");
        if (padType == null)
        {
            throw new InvalidOperationException("JoystickPad was not found in the plugin assembly.");
        }
        return padType;
    }

    private static void RunInteractionChecks(Assembly plugin, Type padType)
    {
        Color card = GetThemeColor(plugin, "CARD_BG");
        Color accent = GetThemeColor(plugin, "ACCENT");
        using (Form host = CreateOffscreenHost())
        using (Control pad = CreatePad(plugin, padType))
        {
            List<PointF> values = AttachStickListener(pad);
            pad.BackColor = card;
            pad.SetBounds(0, 0, 320, 300);
            host.Controls.Add(pad);
            host.Show();
            Application.DoEvents();

            IntPtr handle = pad.Handle;
            CheckMouseDrag(handle, pad, values);
            CheckMouseLeave(handle, values);
            CheckRendering(pad, card, accent);
            ObserveDisableDuringDrag(handle, pad, values);
        }
    }

    private static Form CreateOffscreenHost()
    {
        return new Form
        {
            AutoScaleMode = AutoScaleMode.None,
            FormBorderStyle = FormBorderStyle.None,
            ShowInTaskbar = false,
            StartPosition = FormStartPosition.Manual,
            Location = new Point(-10000, -10000),
            ClientSize = new Size(320, 300),
        };
    }

    private static Control CreatePad(Assembly plugin, Type padType)
    {
        if (padType.IsNested)
        {
            return (Control)Activator.CreateInstance(padType, nonPublic: true);
        }

        Type window = plugin.GetType("NOMAD.MissionPlanner.GimbalJoystickWindow");
        FieldInfo deadzoneField = window.GetField("STICK_DEADZONE", BindingFlags.NonPublic | BindingFlags.Static);
        float deadzone = (float)deadzoneField.GetRawConstantValue();
        ConstructorInfo constructor = padType.GetConstructor(
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            binder: null,
            types: new[] { typeof(float) },
            modifiers: null);
        return (Control)constructor.Invoke(new object[] { deadzone });
    }

    private static List<PointF> AttachStickListener(Control pad)
    {
        var values = new List<PointF>();
        EventInfo stickChanged = pad.GetType().GetEvent("StickChanged");
        if (stickChanged == null)
        {
            throw new InvalidOperationException("JoystickPad has no StickChanged event.");
        }

        Action<float, float> onStickChanged = (x, y) => values.Add(new PointF(x, y));
        stickChanged.AddEventHandler(pad, onStickChanged);
        return values;
    }

    private static void CheckMouseDrag(IntPtr handle, Control pad, List<PointF> values)
    {
        SendMouse(handle, WM_LBUTTONDOWN, 1, PAD_CENTER_X + PAD_TEST_RADIUS, PAD_CENTER_Y);
        Check(values.Count == 1, "left press emits one stick value");
        CheckNear(1f, values[0].X, "right edge is +roll");
        CheckNear(0f, values[0].Y, "right edge has no pitch");
        Check(pad.Capture, "left press captures the pointer");

        SendMouse(handle, WM_MOUSEMOVE, 1, PAD_CENTER_X + PAD_TEST_RADIUS, PAD_CENTER_Y - PAD_TEST_RADIUS);
        Check(values.Count == 2, "drag emits one new stick value");
        CheckNear(0.7071f, values[1].X, "diagonal drag is bounded on roll");
        CheckNear(0.7071f, values[1].Y, "upward diagonal drag is positive pitch");

        SendMouse(handle, WM_MOUSEMOVE, 1, 319, 28);
        Check(values.Count == 3, "second drag emits one new stick value");
        CheckNear(0.7934f, values[2].X, "outward drag clamps to the circular boundary");
        CheckNear(0.6087f, values[2].Y, "outward drag keeps the pitch axis orientation");

        SendMouse(handle, WM_LBUTTONUP, 0, 319, 28);
        Check(values.Count == 4, "release emits one center value");
        CheckNear(0f, values[3].X, "release centers roll");
        CheckNear(0f, values[3].Y, "release centers pitch");
        Check(!pad.Capture, "release drops pointer capture");

        SendMouse(handle, WM_MOUSEMOVE, 0, PAD_CENTER_X, PAD_CENTER_Y - PAD_TEST_RADIUS);
        Check(values.Count == 4, "mouse movement after release emits no value");
    }

    private static void CheckMouseLeave(IntPtr handle, List<PointF> values)
    {
        SendMouse(handle, WM_LBUTTONDOWN, 1, PAD_CENTER_X, PAD_CENTER_Y - PAD_TEST_RADIUS);
        Check(values.Count == 5, "second press emits one stick value");
        CheckNear(1f, values[4].Y, "top edge is positive pitch");

        SendMouse(handle, WM_MOUSELEAVE, 0, 0, 0);
        Check(values.Count == 6, "leaving during a drag emits one center value");
        CheckNear(0f, values[5].X, "mouse leave centers roll");
        CheckNear(0f, values[5].Y, "mouse leave centers pitch");
    }

    private static void CheckRendering(Control pad, Color card, Color accent)
    {
        using (var bitmap = new Bitmap(pad.Width, pad.Height))
        {
            pad.DrawToBitmap(bitmap, new Rectangle(Point.Empty, pad.Size));
            Check(bitmap.GetPixel(PAD_CENTER_X, PAD_CENTER_Y).ToArgb() == accent.ToArgb(),
                "rendered puck uses the theme accent at center");
            Check(bitmap.GetPixel(2, 2).ToArgb() == card.ToArgb(),
                "rendered pad keeps its themed background");
            bitmap.Save(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "JoystickPadRender.png"));
        }
    }

    private static void ObserveDisableDuringDrag(IntPtr handle, Control pad, List<PointF> values)
    {
        pad.Enabled = true;
        SendMouse(handle, WM_LBUTTONDOWN, 1, PAD_CENTER_X + PAD_TEST_RADIUS / 2, PAD_CENTER_Y);
        int beforeDisable = values.Count;
        PointF heldValue = values[values.Count - 1];
        pad.Enabled = false;
        Application.DoEvents();

        Console.WriteLine(
            $"OBSERVED disable during drag: events {beforeDisable}->{values.Count}, " +
            $"before=({heldValue.X:0.0000},{heldValue.Y:0.0000}), " +
            $"after=({values[values.Count - 1].X:0.0000},{values[values.Count - 1].Y:0.0000}), " +
            $"capture={pad.Capture}");
        Check(values.Count == beforeDisable + 1, "disabling during drag emits one center value");
        CheckNear(0f, values[values.Count - 1].X, "disable centers roll");
        CheckNear(0f, values[values.Count - 1].Y, "disable centers pitch");
        Check(!pad.Capture, "disabling during drag releases pointer capture");
        SendMouse(handle, WM_LBUTTONUP, 0, PAD_CENTER_X + PAD_TEST_RADIUS / 2, PAD_CENTER_Y);
        pad.Enabled = true;
    }

    private static Color GetThemeColor(Assembly plugin, string name)
    {
        Type theme = plugin.GetType("NOMAD.MissionPlanner.NOMADTheme");
        return (Color)theme.GetField(name, BindingFlags.Public | BindingFlags.Static).GetValue(null);
    }

    private static void SendMouse(IntPtr handle, uint message, int buttons, int x, int y)
    {
        int coordinates = (y << 16) | (x & 0xffff);
        SendMessage(handle, message, new IntPtr(buttons), new IntPtr(coordinates));
    }

    private static void CheckNear(float expected, float actual, string name)
    {
        Check(Math.Abs(expected - actual) <= 0.01f,
            $"{name} (expected {expected:0.0000}, got {actual:0.0000})");
    }

    private static void Check(bool passed, string name)
    {
        Console.WriteLine($"  {(passed ? "PASS" : "FAIL")}  {name}");
        if (!passed)
        {
            _failures++;
        }
    }

    [DllImport("user32.dll", CharSet = CharSet.Auto)]
    private static extern IntPtr SendMessage(IntPtr handle, uint message, IntPtr wParam, IntPtr lParam);
}
