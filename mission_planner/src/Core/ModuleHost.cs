// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Module Host
// ============================================================
// Registers modules, filters them by allow-list + enable flags, resolves a
// dependency order (topological sort with cycle/missing detection), and drives
// the configure/start/stop lifecycle. Mirrors edge_core.core.ModuleRegistry on
// the Jetson side. Aggregated view descriptors are consumed by NOMADMainScreen
// to build the sidebar.
// ============================================================

using System;
using System.Collections.Generic;
using System.Linq;

namespace NOMAD.MissionPlanner.Core
{
    /// <summary>Holds and orchestrates the registered NOMAD modules.</summary>
    public sealed class ModuleHost
    {
        private readonly Dictionary<string, INomadModule> _modules =
            new Dictionary<string, INomadModule>(StringComparer.Ordinal);
        private readonly List<string> _registrationOrder = new List<string>();
        private readonly HashSet<string> _allow; // null => allow all
        private List<string> _resolved;          // enabled modules, in dependency order

        /// <param name="allowList">Optional whitelist of module names; null allows all.</param>
        public ModuleHost(IEnumerable<string> allowList = null)
        {
            _allow = allowList != null ? new HashSet<string>(allowList, StringComparer.Ordinal) : null;
        }

        public int Count
        {
            get { return _modules.Count; }
        }

        /// <summary>Register a module. Throws on a missing name or duplicate.</summary>
        public string Register(INomadModule module)
        {
            if (module == null) throw new ArgumentNullException(nameof(module));
            var meta = module.Metadata;
            if (meta == null || string.IsNullOrEmpty(meta.Name))
                throw new ModuleException("Module is missing metadata or a name.");
            if (_modules.ContainsKey(meta.Name))
                throw new ModuleException("Duplicate module name: " + meta.Name);
            _modules[meta.Name] = module;
            _registrationOrder.Add(meta.Name);
            return meta.Name;
        }

        private bool IsModuleEnabled(INomadModule module, NomadModuleContext context)
        {
            var meta = module.Metadata;
            if (_allow != null && !_allow.Contains(meta.Name)) return false;
            if (string.IsNullOrEmpty(meta.EnableFlag)) return true;
            return context.IsEnabled(meta.EnableFlag, meta.EnabledByDefault);
        }

        /// <summary>
        /// Compute the enabled modules in dependency order. Throws on a missing or
        /// disabled dependency, or a dependency cycle.
        /// </summary>
        public IList<string> ResolveOrder(NomadModuleContext context)
        {
            if (context == null) throw new ArgumentNullException(nameof(context));

            // 1. Enabled set (allow-list + enable flags).
            var enabled = new HashSet<string>(
                _registrationOrder.Where(n => IsModuleEnabled(_modules[n], context)),
                StringComparer.Ordinal);

            // 2. Validate dependencies exist and are themselves enabled.
            foreach (var name in enabled)
            {
                foreach (var dep in _modules[name].Metadata.Requires ?? new string[0])
                {
                    if (!_modules.ContainsKey(dep))
                        throw new ModuleException(
                            "Module '" + name + "' requires '" + dep + "', which is not registered.");
                    if (!enabled.Contains(dep))
                        throw new ModuleException(
                            "Module '" + name + "' requires '" + dep + "', which is disabled.");
                }
            }

            // 3. Topological sort (depth-first), preserving registration order on ties.
            var ordered = new List<string>();
            var state = new Dictionary<string, int>(); // 0 = visiting, 1 = done
            foreach (var name in _registrationOrder.Where(enabled.Contains))
                Visit(name, enabled, ordered, state);
            return ordered;
        }

        private void Visit(string name, HashSet<string> enabled, List<string> ordered, Dictionary<string, int> state)
        {
            int s;
            if (state.TryGetValue(name, out s))
            {
                if (s == 0)
                    throw new ModuleException("Dependency cycle detected at module: " + name);
                return;
            }
            state[name] = 0;
            foreach (var dep in _modules[name].Metadata.Requires ?? new string[0])
            {
                if (enabled.Contains(dep)) Visit(dep, enabled, ordered, state);
            }
            state[name] = 1;
            ordered.Add(name);
        }

        /// <summary>Resolve order and call Configure on each enabled module.</summary>
        public void Configure(NomadModuleContext context)
        {
            _resolved = ResolveOrder(context).ToList();
            foreach (var name in _resolved)
                _modules[name].Configure(context);
        }

        /// <summary>Aggregate the view descriptors of all enabled modules, in order.</summary>
        public IEnumerable<NomadViewDescriptor> GetViewDescriptors()
        {
            var names = _resolved ?? new List<string>();
            foreach (var name in names)
            {
                var views = _modules[name].GetViews();
                if (views == null) continue;
                foreach (var descriptor in views)
                    if (descriptor != null) yield return descriptor;
            }
        }

        /// <summary>Start enabled modules in dependency order.</summary>
        public void StartAll()
        {
            if (_resolved == null) return;
            foreach (var name in _resolved)
                _modules[name].Start();
        }

        /// <summary>Stop enabled modules in reverse dependency order.</summary>
        public void StopAll()
        {
            if (_resolved == null) return;
            for (int i = _resolved.Count - 1; i >= 0; i--)
                _modules[_resolved[i]].Stop();
        }
    }
}
