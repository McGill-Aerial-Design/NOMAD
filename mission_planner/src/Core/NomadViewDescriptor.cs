// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD View Descriptor
// ============================================================
// Declarative description of a single sidebar entry contributed by a module.
// A descriptor is either a lazily-created in-place View, or an Action that runs
// something else (e.g. opens a floating window) without swapping the content.
// The host (NOMADMainScreen) turns descriptors into sidebar buttons + views.
// ============================================================

using System;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner.Core
{
    /// <summary>Whether a descriptor renders an in-place view or fires an action.</summary>
    public enum NomadEntryKind
    {
        View,
        Action,
    }

    /// <summary>
    /// Immutable description of one navigation entry. Use the <see cref="View"/>
    /// and <see cref="ActionEntry"/> factories rather than the constructor.
    /// </summary>
    public sealed class NomadViewDescriptor
    {
        /// <summary>Stable identifier used for button state and view caching.</summary>
        public string Id { get; private set; }

        /// <summary>Sidebar button label.</summary>
        public string ButtonText { get; private set; }

        /// <summary>Header text shown above the content area when this view is active.</summary>
        public string Title { get; private set; }

        /// <summary>Optional group heading (e.g. "MISSIONS"); empty means ungrouped.</summary>
        public string Section { get; private set; }

        public NomadEntryKind Kind { get; private set; }

        private readonly Func<Control> _viewFactory;
        private readonly Action _action;

        private NomadViewDescriptor(string id, string buttonText, string title, string section,
                                    NomadEntryKind kind, Func<Control> viewFactory, Action action)
        {
            if (string.IsNullOrEmpty(id)) throw new ArgumentNullException(nameof(id));
            Id = id;
            ButtonText = string.IsNullOrEmpty(buttonText) ? id : buttonText;
            Title = string.IsNullOrEmpty(title) ? ButtonText : title;
            Section = section ?? string.Empty;
            Kind = kind;
            _viewFactory = viewFactory;
            _action = action;
        }

        /// <summary>An in-place view, created lazily the first time it is shown.</summary>
        public static NomadViewDescriptor View(string id, string buttonText, string title,
                                               string section, Func<Control> viewFactory)
        {
            if (viewFactory == null) throw new ArgumentNullException(nameof(viewFactory));
            return new NomadViewDescriptor(id, buttonText, title, section, NomadEntryKind.View, viewFactory, null);
        }

        /// <summary>An action entry (e.g. opens a floating window); does not swap the content view.</summary>
        public static NomadViewDescriptor ActionEntry(string id, string buttonText, string section, Action action)
        {
            if (action == null) throw new ArgumentNullException(nameof(action));
            return new NomadViewDescriptor(id, buttonText, buttonText, section, NomadEntryKind.Action, null, action);
        }

        /// <summary>Instantiate the view control (for <see cref="NomadEntryKind.View"/> entries).</summary>
        public Control CreateView()
        {
            return _viewFactory != null ? _viewFactory() : null;
        }

        /// <summary>Run the action (for <see cref="NomadEntryKind.Action"/> entries).</summary>
        public void Invoke()
        {
            if (_action != null) _action();
        }
    }
}
