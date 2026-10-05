// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System.Collections.Generic;

namespace NOMAD.MissionPlanner.Connectivity
{
    public sealed class NomadActuatorAction
    {
        public string Operation { get; internal set; }
        public string Label { get; internal set; }
        public string Control { get; internal set; }
        public bool ContinuousAxisAllowed { get; internal set; }
        public string ContinuousAxisBlockedReason { get; internal set; } = "";
        public string ReleaseOperation { get; internal set; } = "";
    }
    public sealed class NomadActuatorState
    {
        public string Id { get; internal set; }
        public ulong Revision { get; internal set; }
        public bool RecoveryRequired { get; internal set; }
        public bool Pending { get; internal set; }
        public int ConfirmationRemaining { get; internal set; }
        public double? CommandedPosition { get; internal set; }
        public bool SoftwareCommandSuccess { get; internal set; }
        public bool PulseOnSucceeded { get; internal set; }
        public bool ActivationCommanded { get; internal set; }
        public string ActivationOutcome { get; internal set; }
        public string SafeOutcome { get; internal set; }
    }
    public sealed class NomadActuator
    {
        public string Id { get; internal set; }
        public string Name { get; internal set; }
        public IReadOnlyList<NomadActuatorAction> Actions { get; internal set; }
        public NomadActuatorState State { get; internal set; }
        public Dictionary<string, object> Config { get; internal set; }
    }
}
