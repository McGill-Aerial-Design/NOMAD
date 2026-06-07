// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD View Contract (optional)
// ============================================================
// Optional interface a module-contributed view may implement to receive
// activation notifications when the host swaps it in/out of the content area.
// Views that only need periodic refreshes can implement the existing
// IUpdatableView instead (the host's update timer already drives it).
// ============================================================

namespace NOMAD.MissionPlanner.Core
{
    /// <summary>
    /// Lifecycle hooks for a module view. Both calls happen on the UI thread.
    /// </summary>
    public interface INomadView
    {
        /// <summary>Called immediately after the view is shown in the content area.</summary>
        void OnActivated();

        /// <summary>Called just before the view is swapped out (kept cached, not disposed).</summary>
        void OnDeactivated();
    }
}
