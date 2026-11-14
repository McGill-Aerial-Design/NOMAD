// SPDX-License-Identifier: Apache-2.0
using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Forms.Integration;
using System.Windows.Media;

namespace NOMAD.MissionPlanner
{
    internal static class WpfUi
    {
        internal static ElementHost Host(FrameworkElement content)
        {
            var margin = content.Margin;
            content.Resources.MergedDictionaries.Add(WpfTheme.Resources());
            TextElement.SetFontFamily(content, new FontFamily(NOMADTheme.FONT_FAMILY));
            TextElement.SetFontSize(content, NOMADTheme.SIZE_BODY * 96.0 / 72);
            TextElement.SetForeground(content, WpfTheme.Text);
            var host = new ElementHost
            {
                Dock = System.Windows.Forms.DockStyle.Fill, Child = content,
                BackColor = NOMADTheme.BG_DARK, Font = NOMADTheme.Font(),
            };
            // ElementHost clears the root margin when assigning Child.
            content.Margin = margin;
            return host;
        }

        internal static Button Action(string text, Action action)
        {
            var button = new Button { Content = text };
            System.Windows.Automation.AutomationProperties.SetName(button, text);
            button.Click += (sender, args) => action();
            return button;
        }

        internal static TextBlock Text(string text) => new TextBlock
        { Text = text, TextWrapping = TextWrapping.Wrap, Margin = new Thickness(0, 0, 0, NOMADTheme.GAP) };

        internal static TextBlock Section(string text) => new TextBlock
        {
            Text = text, TextWrapping = TextWrapping.Wrap, FontWeight = FontWeights.SemiBold,
            FontSize = NOMADTheme.SIZE_HEADING * 96.0 / 72, Foreground = WpfTheme.Accent,
            Margin = new Thickness(0, NOMADTheme.PAD, 0, NOMADTheme.GAP),
        };
    }
}
