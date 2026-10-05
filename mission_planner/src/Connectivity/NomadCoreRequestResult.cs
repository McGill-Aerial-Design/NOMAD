// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System.Collections.Generic;

namespace NOMAD.MissionPlanner.Connectivity
{
    public enum NomadCoreRequestOutcome
    {
        NotAttempted,
        Succeeded,
        Rejected,
        FailedBeforeSend,
        UnknownOutcome,
        Failed,
        Interrupted
    }

    /// <summary>One request's software result. Physical effects remain unverified.</summary>
    public sealed class NomadCoreRequestResult
    {
        public NomadCoreRequestOutcome Outcome { get; }
        public string ErrorCode { get; }
        public string Message { get; }
        public bool? Acknowledged { get; }
        public IReadOnlyList<NomadActuator> Actuators { get; internal set; } = new List<NomadActuator>();
        public NomadActuatorState ActuatorState { get; internal set; }
        public ulong RequestSequence { get; internal set; }
        public string RuntimeIncarnation { get; internal set; } = "";
        public bool? ExecutionAttempted { get; internal set; }
        public bool? SoftwareCommandSuccess { get; internal set; }
        public bool PresentationCurrent { get; internal set; } = true;
        public bool ConfigurationChanged { get; internal set; }
        public bool ConfigurationRecoveryRequired { get; internal set; }
        public bool Succeeded => Outcome == NomadCoreRequestOutcome.Succeeded;

        public NomadCoreRequestResult(NomadCoreRequestOutcome outcome, string errorCode = "",
                                      string message = "", bool? acknowledged = null)
        {
            Outcome = outcome;
            ErrorCode = errorCode;
            Message = message;
            Acknowledged = acknowledged;
        }
    }
}
