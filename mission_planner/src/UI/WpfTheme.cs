// SPDX-License-Identifier: Apache-2.0
using System;
using System.Windows;
using System.Windows.Media;

namespace NOMAD.MissionPlanner
{
    public static class WpfTheme
    {
        public static Brush Background { get; } = Brush(NOMADTheme.BG_DARK);
        public static Brush Card { get; } = Brush(NOMADTheme.CARD_BG);
        public static Brush Input { get; } = Brush(NOMADTheme.INPUT_BG);
        public static Brush Button { get; } = Brush(NOMADTheme.BUTTON_BG);
        public static Brush Outline { get; } = Brush(NOMADTheme.CARD_BORDER);
        public static Brush Accent { get; } = Brush(NOMADTheme.ACCENT);
        public static Brush Warning { get; } = Brush(NOMADTheme.WARNING);
        public static Brush Text { get; } = Brush(NOMADTheme.TEXT_PRIMARY);
        public static Brush SecondaryText { get; } = Brush(NOMADTheme.TEXT_SECONDARY);

        private static Brush Brush(System.Drawing.Color color)
        {
            var brush = new SolidColorBrush(Color.FromArgb(color.A, color.R, color.G, color.B));
            brush.Freeze();
            return brush;
        }

        internal static ResourceDictionary Resources() => (ResourceDictionary)Application.LoadComponent(
            new Uri("/NOMADPlugin;component/UI/WpfTheme.xaml", UriKind.Relative));
    }
}
