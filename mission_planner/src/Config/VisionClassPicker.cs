// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using System.Windows.Data;
using Wpf = System.Windows.Controls;

namespace NOMAD.MissionPlanner
{
    // One model output class. Id is the tensor index and never changes with sorting or filtering.
    internal sealed class VisionClassRow : INotifyPropertyChanged
    {
        private int _id;
        private string _label;
        private bool _detect;
        public event PropertyChangedEventHandler PropertyChanged;
        public int Id { get => _id; set => Set(ref _id, value, nameof(Id)); }
        public string Label { get => _label; set => Set(ref _label, value, nameof(Label)); }
        public bool Detect { get => _detect; set => Set(ref _detect, value, nameof(Detect)); }

        private void Set<T>(ref T field, T value, string name)
        {
            if (EqualityComparer<T>.Default.Equals(field, value)) { return; }
            field = value;
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }
    }

    // Searchable checklist over the shared class rows, for choosing what the detector reports.
    internal sealed class VisionClassPicker : Wpf.DockPanel
    {
        private readonly ObservableCollection<VisionClassRow> _rows;
        private readonly int[] _defaults;
        private readonly ListCollectionView _view;
        private readonly Wpf.TextBox _search = new Wpf.TextBox();
        private readonly Wpf.TextBlock _count = new Wpf.TextBlock
        { Foreground = WpfTheme.SecondaryText, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(8, 0, 0, 0) };

        internal VisionClassPicker(ObservableCollection<VisionClassRow> rows, int[] defaults)
        {
            _rows = rows; _defaults = defaults;
            _view = new ListCollectionView(rows) { Filter = Matches };
            System.Windows.Automation.AutomationProperties.SetName(_search, "Search classes");
            _search.ToolTip = "Type to filter the class list.";
            _search.TextChanged += (sender, args) => _view.Refresh();

            var tools = new Wpf.DockPanel { Margin = new Thickness(0, 0, 0, 6) };
            var buttons = new Wpf.StackPanel { Orientation = Wpf.Orientation.Horizontal };
            buttons.Children.Add(Tool("All", "Detect every class shown.", () => SetShown(true)));
            buttons.Children.Add(Tool("None", "Detect none of the classes shown.", () => SetShown(false)));
            if (defaults != null) { buttons.Children.Add(Tool("Defaults", "Restore the model's default classes.", Restore)); }
            buttons.Children.Add(_count);
            Wpf.DockPanel.SetDock(buttons, Wpf.Dock.Right);
            tools.Children.Add(buttons); tools.Children.Add(_search);
            Wpf.DockPanel.SetDock(tools, Wpf.Dock.Top);
            Children.Add(tools);

            var list = new Wpf.ListBox { ItemsSource = _view, Height = 220, Background = WpfTheme.Input, Foreground = WpfTheme.Text,
                ItemTemplate = CheckTemplate() };
            System.Windows.Automation.AutomationProperties.SetName(list, "Classes to detect");
            Children.Add(list);

            foreach (var row in rows) { row.PropertyChanged += OnRowChanged; }
            rows.CollectionChanged += (sender, args) =>
            {
                foreach (VisionClassRow row in args.NewItems ?? new VisionClassRow[0]) { row.PropertyChanged += OnRowChanged; }
                UpdateCount();
            };
            UpdateCount();
        }

        private static Wpf.Button Tool(string text, string tip, Action action)
        {
            var button = WpfUi.Action(text, action);
            button.ToolTip = tip;
            return button;
        }

        private static DataTemplate CheckTemplate()
        {
            var check = new FrameworkElementFactory(typeof(Wpf.CheckBox));
            check.SetBinding(Wpf.Primitives.ToggleButton.IsCheckedProperty, new Binding(nameof(VisionClassRow.Detect)));
            check.SetBinding(Wpf.ContentControl.ContentProperty, new Binding(nameof(VisionClassRow.Label)));
            check.SetValue(Wpf.Control.ForegroundProperty, WpfTheme.Text);
            return new DataTemplate { VisualTree = check };
        }

        private bool Matches(object item)
        {
            string text = _search.Text.Trim();
            return text.Length == 0 || ((VisionClassRow)item).Label?.IndexOf(text, StringComparison.OrdinalIgnoreCase) >= 0;
        }

        private void SetShown(bool detect) { foreach (VisionClassRow row in _view) { row.Detect = detect; } }

        private void Restore() { foreach (var row in _rows) { row.Detect = _defaults.Contains(row.Id); } }

        private void OnRowChanged(object sender, PropertyChangedEventArgs args)
        {
            if (args.PropertyName == nameof(VisionClassRow.Detect)) { UpdateCount(); }
            else if (args.PropertyName == nameof(VisionClassRow.Label)) { _view.Refresh(); }
        }

        private void UpdateCount() => _count.Text = _rows.Count(row => row.Detect) + " of " + _rows.Count + " selected";
    }
}
