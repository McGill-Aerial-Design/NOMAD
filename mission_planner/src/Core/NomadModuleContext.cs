// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Module Context
// ============================================================
// Shared service registry + feature-flag accessor handed to every module at
// Configure() time. This is the C# counterpart of edge_core.core.AppContext on
// the Jetson side: modules pull the dependencies they need (config, senders,
// connection managers, ...) from here instead of taking a fixed constructor.
// ============================================================

using System;
using System.Collections.Generic;

namespace NOMAD.MissionPlanner.Core
{
    /// <summary>
    /// Dependency container and feature-flag lookup passed to a module when it is
    /// configured. Services are resolved by type (the common case) or by name.
    /// </summary>
    public sealed class NomadModuleContext
    {
        private readonly Dictionary<Type, object> _typed = new Dictionary<Type, object>();
        private readonly Dictionary<string, object> _named =
            new Dictionary<string, object>(StringComparer.OrdinalIgnoreCase);
        private readonly Func<string, bool?> _flagLookup;

        /// <param name="flagLookup">
        /// Optional callback resolving an enable-flag name to true/false, or null
        /// to fall back to the module's <c>EnabledByDefault</c>. The host typically
        /// backs this with environment variables or NOMAD settings.
        /// </param>
        public NomadModuleContext(Func<string, bool?> flagLookup = null)
        {
            _flagLookup = flagLookup;
        }

        /// <summary>Register a service under its concrete type.</summary>
        public NomadModuleContext Register<T>(T service) where T : class
        {
            if (service == null) throw new ArgumentNullException(nameof(service));
            _typed[typeof(T)] = service;
            return this;
        }

        /// <summary>Register a service under an explicit name.</summary>
        public NomadModuleContext Register(string name, object service)
        {
            if (string.IsNullOrEmpty(name)) throw new ArgumentNullException(nameof(name));
            _named[name] = service;
            return this;
        }

        /// <summary>Resolve a service by type, or null when absent.</summary>
        public T Get<T>() where T : class
        {
            object svc;
            return _typed.TryGetValue(typeof(T), out svc) ? (T)svc : null;
        }

        /// <summary>Resolve a service by type, throwing when absent.</summary>
        public T Require<T>() where T : class
        {
            var svc = Get<T>();
            if (svc == null)
                throw new KeyNotFoundException("Required service not registered: " + typeof(T).FullName);
            return svc;
        }

        /// <summary>Resolve a named service, or null when absent.</summary>
        public object Get(string name)
        {
            object svc;
            return _named.TryGetValue(name, out svc) ? svc : null;
        }

        /// <summary>
        /// Evaluate an enable flag. Empty/absent flags return <paramref name="defaultValue"/>,
        /// matching the opt-in/opt-out semantics of the Jetson module SDK.
        /// </summary>
        public bool IsEnabled(string flag, bool defaultValue = true)
        {
            if (string.IsNullOrEmpty(flag)) return defaultValue;
            var result = _flagLookup != null ? _flagLookup(flag) : null;
            return result ?? defaultValue;
        }
    }
}
