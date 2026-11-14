// SPDX-License-Identifier: Apache-2.0
using System;
using System.Globalization;
using System.Windows.Controls;

namespace NOMAD.MissionPlanner
{
    internal sealed class WpfNumberBox : TextBox
    {
        private string _loadedText;
        internal WpfNumberBox() { SetResourceReference(StyleProperty, typeof(TextBox)); }
        internal decimal Minimum { get; set; }
        internal decimal Maximum { get; set; }
        internal int DecimalPlaces { get; set; }
        internal decimal Value
        {
            get
            {
                if (!decimal.TryParse(Text, NumberStyles.Number, CultureInfo.CurrentCulture, out decimal value) ||
                    value < Minimum || value > Maximum || (Text != _loadedText && decimal.Round(value, DecimalPlaces) != value))
                { throw new FormatException("Enter a value from " + Minimum + " to " + Maximum + "."); }
                return value;
            }
            set { Text = value.ToString(CultureInfo.CurrentCulture); _loadedText = Text; }
        }
    }
}
