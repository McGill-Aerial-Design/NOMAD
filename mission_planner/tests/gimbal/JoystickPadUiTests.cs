// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Forms;
using System.Windows.Forms.Integration;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Application = System.Windows.Forms.Application;
using MouseEventArgs = System.Windows.Input.MouseEventArgs;
using Color = System.Drawing.Color;

internal static class JoystickPadUiTests
{
    private const int CX = 160, CY = 150, RADIUS = 122;
    private static int failures;

    [STAThread]
    private static int Main(string[] args)
    {
        if (args.Length != 1) { Console.Error.WriteLine("Pass the built NOMADPlugin.dll path."); return 2; }
        var plugin = Assembly.LoadFrom(args[0]);
        var type = plugin.GetType("NOMAD.MissionPlanner.JoystickPad", true);
        var position = new System.Windows.Point();
        bool captured = false, allowCapture = true;
        Func<MouseEventArgs, System.Windows.Point> positionProvider = mouseArgs => position;
        Func<bool> capture = () => captured = allowCapture;
        Action release = () => captured = false;
        var pad = (FrameworkElement)Activator.CreateInstance(type, BindingFlags.Instance | BindingFlags.Public |
            BindingFlags.NonPublic, null, new object[] { 0.06f, positionProvider, capture, release }, null);
        var values = new List<PointF>();
        type.GetEvent("StickChanged").AddEventHandler(pad,
            new Action<float, float>((x, y) => values.Add(new PointF(x, y))));
        using (var form = new Form { AutoScaleMode = AutoScaleMode.None, FormBorderStyle = FormBorderStyle.None,
            ShowInTaskbar = false, StartPosition = FormStartPosition.Manual, Location = new System.Drawing.Point(-10000, -10000),
            ClientSize = new System.Drawing.Size(320, 300) })
        using (var host = new ElementHost { Dock = DockStyle.Fill, Child = pad })
        {
            form.Controls.Add(host); form.Show(); Application.DoEvents();
            position = new System.Windows.Point(CX + RADIUS, CY);
            SendButton(pad, UIElement.MouseLeftButtonDownEvent);
            Check(values.Count == 1, "left press emits one stick value");
            CheckValue(values, 0, 1, 0, "right edge is +horizontal");
            Check(captured, "left press captures pointer");
            position = new System.Windows.Point(CX + RADIUS, CY - RADIUS); SendMove(pad, UIElement.MouseMoveEvent);
            Check(values.Count == 2, "drag emits one new stick value");
            CheckValue(values, 1, 0.7071f, 0.7071f, "diagonal drag is bounded and upward is positive");
            position = new System.Windows.Point(319, 28); SendMove(pad, UIElement.MouseMoveEvent);
            Check(values.Count == 3, "second drag emits one new stick value");
            CheckValue(values, 2, 0.7934f, 0.6087f, "outward drag clamps to circular boundary");
            SendButton(pad, UIElement.MouseLeftButtonUpEvent);
            Check(values.Count == 4, "release emits one center value");
            CheckValue(values, 3, 0, 0, "release centers both axes");
            Check(!captured, "release drops pointer capture");
            position = new System.Windows.Point(CX, CY - RADIUS); SendMove(pad, UIElement.MouseMoveEvent);
            Check(values.Count == 4, "movement after release emits no value");
            SendButton(pad, UIElement.MouseLeftButtonDownEvent);
            Check(values.Count == 5, "second press emits one stick value");
            CheckValue(values, 4, 0, 1, "top edge is positive vertical");
            SendMove(pad, UIElement.MouseLeaveEvent);
            Check(values.Count == 6, "leaving during drag emits one center value");
            CheckValue(values, 5, 0, 0, "leave centers both axes");
            CheckRendering(pad, plugin);
            position = new System.Windows.Point(CX + RADIUS / 2, CY); SendButton(pad, UIElement.MouseLeftButtonDownEvent);
            int before = values.Count;
            pad.IsEnabled = false; Application.DoEvents();
            Check(values.Count == before + 1, "disabling during drag emits one center value");
            CheckValue(values, before, 0, 0, "disable centers both axes");
            Check(!captured, "disabling releases pointer capture");
            pad.IsEnabled = true;
            position = new System.Windows.Point(CX + RADIUS / 2, CY); SendButton(pad, UIElement.MouseLeftButtonDownEvent);
            before = values.Count;
            captured = false; SendMove(pad, Mouse.LostMouseCaptureEvent);
            Check(values.Count == before + 1, "capture loss emits one center value");
            CheckValue(values, before, 0, 0, "capture loss centers both axes");
            position = new System.Windows.Point(CX + RADIUS / 2, CY); SendButton(pad, UIElement.MouseLeftButtonDownEvent);
            before = values.Count;
            host.Child = null; Application.DoEvents();
            Check(values.Count == before + 1, "unloading during drag emits one center value");
            CheckValue(values, before, 0, 0, "unloading centers both axes");
            Check(!captured, "unloading releases pointer capture");
            host.Child = pad; Application.DoEvents();
            allowCapture = false;
            before = values.Count;
            SendButton(pad, UIElement.MouseLeftButtonDownEvent);
            Check(values.Count == before && !captured, "failed capture emits no stick value");
            SendMove(pad, UIElement.MouseMoveEvent);
            Check(values.Count == before, "movement after failed capture emits no stick value");
        }
        return failures == 0 ? 0 : 1;
    }
    private static void CheckRendering(FrameworkElement pad, Assembly plugin)
    {
        var bitmap = new RenderTargetBitmap(320, 300, 96, 96, PixelFormats.Pbgra32);
        bitmap.Render(pad);
        var pixels = new byte[320 * 300 * 4]; bitmap.CopyPixels(pixels, 320 * 4, 0);
        var theme = plugin.GetType("NOMAD.MissionPlanner.NOMADTheme", true);
        Check(PixelMatches(pixels, CX, CY, (Color)theme.GetField("ACCENT").GetValue(null)), "puck uses theme accent");
        Check(PixelMatches(pixels, 2, 2, (Color)theme.GetField("CARD_BG").GetValue(null)), "pad uses themed background");
        var png = new PngBitmapEncoder(); png.Frames.Add(BitmapFrame.Create(bitmap));
        using (var output = File.Create(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "JoystickPadRender.png")))
        { png.Save(output); }
    }
    private static bool PixelMatches(byte[] data, int x, int y, Color expected)
    {
        int index = (y * 320 + x) * 4;
        return data[index] == expected.B && data[index + 1] == expected.G && data[index + 2] == expected.R &&
            data[index + 3] == expected.A;
    }
    private static void CheckValue(List<PointF> values, int index, float x, float y, string name) =>
        Check(values.Count > index && Math.Abs(values[index].X - x) < 0.01f && Math.Abs(values[index].Y - y) < 0.01f, name);
    private static void Check(bool passed, string name)
    {
        Console.WriteLine($"  {(passed ? "PASS" : "FAIL")}  {name}");
        if (!passed) failures++;
    }
    private static void SendButton(FrameworkElement pad, RoutedEvent routedEvent)
    {
        pad.RaiseEvent(new MouseButtonEventArgs(Mouse.PrimaryDevice, Environment.TickCount, MouseButton.Left)
            { RoutedEvent = routedEvent, Source = pad });
        Application.DoEvents();
    }
    private static void SendMove(FrameworkElement pad, RoutedEvent routedEvent)
    {
        pad.RaiseEvent(new MouseEventArgs(Mouse.PrimaryDevice, Environment.TickCount)
            { RoutedEvent = routedEvent, Source = pad });
        Application.DoEvents();
    }
}
