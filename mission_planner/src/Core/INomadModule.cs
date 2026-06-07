// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Module Contract
// ============================================================
// A NOMAD module is a self-contained feature set that contributes one or more
// sidebar views (and/or actions) to the NOMAD screen. This is the Mission
// Planner counterpart of the Jetson-side edge_core.core.NomadModule protocol:
// same metadata (name, version, requires, enable flag) and lifecycle
// (configure -> start -> stop), expressed in C#.
// ============================================================

using System;
using System.Collections.Generic;

namespace NOMAD.MissionPlanner.Core
{
    /// <summary>Static description of a module: identity, dependencies, and gating.</summary>
    public sealed class NomadModuleMetadata
    {
        /// <summary>Unique module name (used for dependency wiring and de-duplication).</summary>
        public string Name { get; set; }

        public string Version { get; set; } = "0.1.0";

        public string Description { get; set; } = "";

        /// <summary>Names of other modules that must load (and be enabled) first.</summary>
        public string[] Requires { get; set; } = new string[0];

        /// <summary>Optional enable-flag name; null means the module is always on.</summary>
        public string EnableFlag { get; set; }

        /// <summary>Value used when <see cref="EnableFlag"/> is set but unresolved.</summary>
        public bool EnabledByDefault { get; set; } = true;
    }

    /// <summary>
    /// A pluggable NOMAD feature module. Implement this (or derive from
    /// <see cref="NomadModuleBase"/>) and register it with a <see cref="ModuleHost"/>.
    /// </summary>
    public interface INomadModule
    {
        NomadModuleMetadata Metadata { get; }

        /// <summary>Resolve dependencies from the context. Called once, in dependency order.</summary>
        void Configure(NomadModuleContext context);

        /// <summary>Sidebar entries this module contributes. Called after Configure.</summary>
        IEnumerable<NomadViewDescriptor> GetViews();

        /// <summary>Optional background work to begin (called in dependency order).</summary>
        void Start();

        /// <summary>Optional cleanup (called in reverse dependency order).</summary>
        void Stop();
    }

    /// <summary>Convenience base with no-op lifecycle and no views.</summary>
    public abstract class NomadModuleBase : INomadModule
    {
        public abstract NomadModuleMetadata Metadata { get; }

        public virtual void Configure(NomadModuleContext context) { }

        public virtual IEnumerable<NomadViewDescriptor> GetViews()
        {
            yield break;
        }

        public virtual void Start() { }

        public virtual void Stop() { }
    }

    /// <summary>Raised on duplicate names, missing/disabled dependencies, or cycles.</summary>
    public sealed class ModuleException : Exception
    {
        public ModuleException(string message) : base(message) { }
    }
}
