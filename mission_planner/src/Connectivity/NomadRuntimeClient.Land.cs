// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System.Collections.Generic;

namespace NOMAD.MissionPlanner.Connectivity
{
    internal sealed partial class NomadRuntimeClient
    {
        private static bool HasLandCapability(Dictionary<string, object> hello)
        {
            if (!hello.TryGetValue("capabilities", out var raw) || raw is not object[] capabilities)
            {
                return false;
            }
            var supportsLand = false;
            foreach (var capability in capabilities)
            {
                if (capability is not string name)
                {
                    return false;
                }
                supportsLand |= name == "land";
            }
            return supportsLand;
        }

        private static bool HasValidLandResult(Dictionary<string, object> response)
        {
            if (!response.TryGetValue("version", out var rawVersion) || rawVersion is not int version || version != 1 ||
                !response.TryGetValue("ok", out var rawOk) || rawOk is not bool ok ||
                !response.TryGetValue("outcome", out var rawOutcome) || rawOutcome is not string outcome ||
                (outcome != "success" && outcome != "failed" && outcome != "rejected" &&
                 outcome != "unknown" && outcome != "interrupted"))
            {
                return false;
            }
            if (!ok)
            {
                return (outcome == "rejected" || outcome == "unknown" || outcome == "interrupted") &&
                    response.TryGetValue("error", out var rawError) &&
                    rawError is Dictionary<string, object> error &&
                    error.TryGetValue("code", out var code) && code is string &&
                    error.TryGetValue("message", out var message) && message is string &&
                    (!response.TryGetValue("command_result", out var rawEvidence) ||
                        rawEvidence is Dictionary<string, object> evidence &&
                        HasValidLandEvidence(evidence, outcome, false));
            }
            if (GetString(response, "type") != "command_response" ||
                !response.TryGetValue("command_result", out var rawResult) ||
                rawResult is not Dictionary<string, object> result)
            {
                return false;
            }
            return result.TryGetValue("message", out var rawMessage) && rawMessage is string &&
                HasValidLandEvidence(result, outcome, true);
        }

        private static bool HasValidLandEvidence(Dictionary<string, object> result, string outcome,
            bool requiresMatchingSuccess)
        {
            if (!result.TryGetValue("success", out var rawSuccess) || rawSuccess is not bool success ||
                !result.TryGetValue("acknowledged", out var rawAck) || rawAck is not bool acknowledged)
            {
                return false;
            }
            return (!requiresMatchingSuccess || success == (outcome == "success")) && (!success || acknowledged) &&
                (outcome != "failed" || acknowledged) && (outcome != "rejected" || !acknowledged);
        }
    }
}
