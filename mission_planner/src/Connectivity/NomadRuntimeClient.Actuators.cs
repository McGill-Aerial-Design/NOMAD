// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Web.Script.Serialization;

namespace NOMAD.MissionPlanner.Connectivity
{
    internal sealed partial class NomadRuntimeClient
    {
        private static bool IsActuatorVerb(string verb) => verb == "get-actuators" ||
            verb == "configure-actuators" || verb == "actuator-action";
        private static bool IsActuatorType(string type) => type == "get_actuators" ||
            type == "configure_actuators" || type == "actuator_action";

        private Dictionary<string, object> BuildActuatorRequest(string verb, string[] values)
        {
            var type = verb == "get-actuators" ? "get_actuators" : verb == "configure-actuators" ?
                "configure_actuators" : "actuator_action";
            var request = BaseRequest("", type);
            if (verb == "get-actuators" && values.Length == 0)
            {
                return request;
            }
            if (verb == "configure-actuators" && values.Length == 1)
            {
                request["actuator_configs"] = new JavaScriptSerializer { MaxJsonLength = 65536, RecursionLimit = 16 }
                    .DeserializeObject(values[0]);
                return request;
            }
            if (verb != "actuator-action" || values.Length != 5)
            {
                return null;
            }
            request["actuator_id"] = values[0];
            request["operation"] = values[1];
            request["input_source"] = values[2];
            if (values[3] != "")
            {
                request["input_slot"] = int.Parse(values[3], CultureInfo.InvariantCulture);
            }
            if (values[4] != "")
            {
                request["value"] = ParseProtocolNumber(values[4]);
            }
            return request;
        }

        private NomadCoreRequestResult ReadActuatorResult(Dictionary<string, object> response,
            string verb, string incarnation)
        {
            if (TryReadError(response, out var code, out var message))
            {
                return new NomadCoreRequestResult(ReadOutcome(response), code, message, ReadAcknowledgement(response))
                { ConfigurationChanged = GetBool(response, "configuration_changed"),
                    ConfigurationRecoveryRequired = GetBool(response, "configuration_recovery_required") };
            }
            try
            {
                if (GetString(response, "runtime_incarnation") != incarnation ||
                    GetString(response, "type") != (verb == "get-actuators" ? "actuators_response" : "actuator_response"))
                { throw new FormatException("Actuator response type or incarnation did not match."); }
                var outcome = verb == "get-actuators" ? NomadCoreRequestOutcome.Succeeded : ReadOutcome(response);
                var request = verb == "get-actuators" ? null : RequiredObject(response, "request_result");
                if (request != null && outcome == NomadCoreRequestOutcome.Succeeded && !RequiredBool(request, "success"))
                { throw new FormatException("Semantic success contradicted the request disposition."); }
                var result = new NomadCoreRequestResult(outcome, "", request == null ? "Actuator configuration received." :
                    GetString(request, "message"), ReadAcknowledgement(response)) { RuntimeIncarnation = incarnation };
                if (response.ContainsKey("actuators"))
                {
                    result.HasActuatorDefinitions = true;
                    if (response.TryGetValue("actuator_configuration_revision", out var revision) &&
                        ulong.TryParse(Convert.ToString(revision, CultureInfo.InvariantCulture), NumberStyles.None,
                            CultureInfo.InvariantCulture, out var catalogRevision))
                    { result.ActuatorConfigurationRevision = catalogRevision; }
                    result.Actuators = ReadActuators(response["actuators"]);
                }
                if (response.ContainsKey("actuator_state"))
                {
                    result.ActuatorState = ReadState(RequiredObject(response, "actuator_state"));
                }
                if (request != null)
                {
                    result.ExecutionAttempted = RequiredBool(response, "execution_attempted");
                    result.SoftwareCommandSuccess = RequiredBool(RequiredObject(response, "command_result"), "success");
                }
                result.ConfigurationChanged = GetBool(response, "configuration_changed");
                result.ConfigurationRecoveryRequired = GetBool(response, "configuration_recovery_required");
                return result;
            }
            catch (Exception ex)
            {
                return new NomadCoreRequestResult(NomadCoreRequestOutcome.UnknownOutcome,
                    "invalid_actuator_response", ex.Message + " Do not retry a mutation blindly.");
            }
        }

        private static List<NomadActuator> ReadActuators(object raw)
        {
            if (!(raw is IEnumerable list) || raw is string || raw is IDictionary)
            {
                throw new FormatException("Actuator list missing.");
            }
            var result = new List<NomadActuator>();
            foreach (var item in list)
            {
                var data = item as Dictionary<string, object> ?? throw new FormatException("Actuator entry invalid.");
                var actions = new List<NomadActuatorAction>();
                foreach (var entry in (IEnumerable)data["actions"])
                {
                    var action = entry as Dictionary<string, object> ?? throw new FormatException("Action entry invalid.");
                    string control = GetString(action, "control");
                    if (control != "button" && control != "position")
                    {
                        throw new FormatException("Unknown display control.");
                    }
                    actions.Add(new NomadActuatorAction { Operation = GetString(action, "operation"),
                        Label = GetString(action, "label"), Control = control, ReleaseOperation = GetString(action, "release_operation"),
                        ContinuousAxisAllowed = GetBool(action, "continuous_axis_allowed"),
                        ContinuousAxisBlockedReason = GetString(action, "continuous_axis_blocked_reason") });
                }
                result.Add(new NomadActuator { Id = GetString(data, "id"), Name = GetString(data, "name"),
                    Actions = actions, State = ReadState(RequiredObject(data, "state")), Config = RequiredObject(data, "config") });
            }
            return result;
        }

        private static NomadActuatorState ReadState(Dictionary<string, object> data)
        {
            var state = new NomadActuatorState { Id = GetString(data, "id"),
                Revision = ulong.Parse(Convert.ToString(data["state_revision"], CultureInfo.InvariantCulture)),
                RecoveryRequired = RequiredBool(data, "recovery_required"), Pending = RequiredBool(data, "pending"),
                ConfirmationRemaining = Convert.ToInt32(data["confirmation_remaining"], CultureInfo.InvariantCulture),
                SoftwareCommandSuccess = RequiredBool(data, "software_command_success"),
                PulseOnSucceeded = RequiredBool(data, "pulse_on_succeeded"), ActivationCommanded = RequiredBool(data, "activation_commanded"),
                ActivationOutcome = GetString(data, "activation_outcome"), SafeOutcome = GetString(data, "safe_outcome") };
            if (data.TryGetValue("commanded_position", out var position) && position != null)
            { state.CommandedPosition = Convert.ToDouble(position, CultureInfo.InvariantCulture); }
            if (string.IsNullOrWhiteSpace(state.Id) || state.ConfirmationRemaining < 0 ||
                (state.CommandedPosition.HasValue && (state.CommandedPosition < 0 || state.CommandedPosition > 1 ||
                    double.IsNaN(state.CommandedPosition.Value)))) { throw new FormatException("Actuator display state invalid."); }
            return state;
        }

        private static Dictionary<string, object> RequiredObject(Dictionary<string, object> data, string key) =>
            data.TryGetValue(key, out var value) && value is Dictionary<string, object> result ? result :
            throw new FormatException("Missing object: " + key);
        private static bool RequiredBool(Dictionary<string, object> data, string key) =>
            data.TryGetValue(key, out var value) && value is bool result ? result :
            throw new FormatException("Missing boolean: " + key);
    }
}
