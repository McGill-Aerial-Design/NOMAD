// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

namespace NOMAD.MissionPlanner.Connectivity
{
    /// <summary>One request's software result. Physical effects remain unverified.</summary>
    public sealed class NomadCoreRequestResult
    {
        public NomadCoreRequestOutcome Outcome { get; }
        public string ErrorCode { get; }
        public string Message { get; }
        public bool? Acknowledged { get; }
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
