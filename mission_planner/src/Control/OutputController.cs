// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using NOMAD.MissionPlanner.Connectivity;

namespace NOMAD.MissionPlanner
{
    internal static class OutputController
    {
        private static NomadCoreClient _coreClient;
        private static readonly SemaphoreSlim GimbalRequests = new SemaphoreSlim(1, 1);
        private static readonly object GimbalFailureLock = new object();
        private static string _lastGimbalFailure = "";
        private static DateTime _lastGimbalFailureAt = DateTime.MinValue;

        internal static void Initialize(NOMADConfig config)
        {
            lock (ProjectionGate)
            {
                DisplayStates.Clear();
                ReleaseOperations.Clear();
                ContinuousAxisActions.Clear();
                _catalogRevision = null;
                _displayIncarnation = "";
                _displaySequence = 0;
                RetiredIncarnations.Clear();
            }
            _coreClient = config == null ? null :
                new NomadCoreClient(config.CoreClientCredential, config.CoreRuntimePort);
        }

        internal static NomadCoreClient CreateCoreClient() => _coreClient;

        // Nonwaiting gates reject overlapping input instead of collecting stale commands.
        private static NomadCoreRequestResult NotSent(string code, string message)
        {
            return new NomadCoreRequestResult(NomadCoreRequestOutcome.NotAttempted, code, message);
        }

        private static readonly object ProjectionGate = new object();
        private static readonly Dictionary<string, NomadActuatorState> DisplayStates = new Dictionary<string, NomadActuatorState>();
        private static readonly Dictionary<string, string> ReleaseOperations = new Dictionary<string, string>();
        private static readonly Dictionary<string, NomadActuatorAction> ContinuousAxisActions = new Dictionary<string, NomadActuatorAction>();
        private static ulong? _catalogRevision;
        private static readonly HashSet<string> RetiredIncarnations = new HashSet<string>();
        private static ulong _displaySequence;
        private static string _displayIncarnation = "";
        internal static event Action<NomadActuatorState> ActuatorStateChanged;

        internal static Task<NomadCoreRequestResult> GetActuatorsAsync() =>
            SendSemanticAsync(client => client.GetActuatorsAsync());
        internal static Task<NomadCoreRequestResult> ConfigureActuatorsAsync(string json) =>
            SendSemanticAsync(client => client.ConfigureActuatorsAsync(json), configuration: true);
        internal static Task<NomadCoreRequestResult> ActuatorActionAsync(string id, string operation,
            string source = "ui", int? slot = null, double? value = null, Func<bool> inputStillCurrent = null) =>
            SendSemanticAsync(client => client.ActuatorActionAsync(id, operation, source, slot, value,
                inputStillCurrent: inputStillCurrent));

        private static async Task<NomadCoreRequestResult> SendSemanticAsync(
            Func<NomadCoreClient, Task<NomadCoreRequestResult>> send, bool configuration = false)
        {
            var client = CreateCoreClient();
            var result = client == null ? NotSent("runtime_not_configured", "NOMAD runtime is not configured.") :
                await send(client).ConfigureAwait(false);
            if (client != CreateCoreClient())
            {
                result.PresentationCurrent = false; return result;
            }
            if (result.RuntimeIncarnation != "" && !ObserveIncarnation(result.RuntimeIncarnation, result.RequestSequence))
            { result.PresentationCurrent = false; return result; }
            if (!AcceptCatalog(result)) { result.PresentationCurrent = false; return result; }
            foreach (var actuator in result.Actuators)
            {
                if (!PublishState(result.RuntimeIncarnation, actuator.State, result.RequestSequence))
                {
                    continue;
                }
            }
            if (result.ActuatorState != null)
            {
                PublishState(result.RuntimeIncarnation, result.ActuatorState, result.RequestSequence);
            }
            if (!result.Succeeded)
            {
                Log.Warn(configuration ? DescribeConfigurationResult(result) : DescribeFailure("Actuator request", result));
            }
            return result;
        }

        private static bool AcceptCatalog(NomadCoreRequestResult result)
        {
            if (!result.HasActuatorDefinitions) { return true; }
            lock (ProjectionGate)
            {
                if (!result.ActuatorConfigurationRevision.HasValue)
                { ContinuousAxisActions.Clear(); return false; }
                ulong revision = result.ActuatorConfigurationRevision.Value;
                if (_catalogRevision.HasValue && revision < _catalogRevision.Value) { return false; }
                _catalogRevision = revision;
                ReleaseOperations.Clear();
                ContinuousAxisActions.Clear();
                foreach (var actuator in result.Actuators)
                {
                    foreach (var action in actuator.Actions)
                    {
                        ReleaseOperations[actuator.Id + ":" + action.Operation] = action.ReleaseOperation;
                        if (action.Control == "position") { ContinuousAxisActions[actuator.Id] = action; }
                    }
                }
                return true;
            }
        }
        internal static NomadActuatorAction GetContinuousAxisMetadata(string id)
        {
            if (string.IsNullOrWhiteSpace(id)) { return null; }
            lock (ProjectionGate) { return ContinuousAxisActions.TryGetValue(id, out var action) ? action : null; }
        }

        internal static string GetReleaseOperation(string binding)
        {
            lock (ProjectionGate)
            {
                return ReleaseOperations.TryGetValue(binding, out var release) ? release : "";
            }
        }
        internal static NomadActuatorState GetDisplayState(string id)
        {
            lock (ProjectionGate)
            {
                return DisplayStates.TryGetValue(id, out var state) ? state : null;
            }
        }
        internal static bool ObserveIncarnation(string incarnation, ulong sequence)
        {
            lock (ProjectionGate)
            {
                if (_displayIncarnation != incarnation)
                {
                    if (sequence < _displaySequence || RetiredIncarnations.Contains(incarnation))
                    {
                        return false;
                    }
                    if (_displayIncarnation != "")
                    {
                        RetiredIncarnations.Add(_displayIncarnation);
                    }
                    DisplayStates.Clear();
                    ReleaseOperations.Clear();
                    ContinuousAxisActions.Clear();
                    _catalogRevision = null;
                    _displayIncarnation = incarnation;
                }
                _displaySequence = Math.Max(_displaySequence, sequence);
                return true;
            }
        }
        internal static bool PublishState(string incarnation, NomadActuatorState state, ulong sequence = 0)
        {
            lock (ProjectionGate)
            {
                if (!ObserveIncarnation(incarnation, sequence))
                {
                    return false;
                }
                if (DisplayStates.TryGetValue(state.Id, out var old) && old.Revision > state.Revision)
                {
                    return false;
                }
                DisplayStates[state.Id] = state;
            }
            var handlers = ActuatorStateChanged;
            if (handlers == null)
            {
                return true;
            }
            foreach (Action<NomadActuatorState> handler in handlers.GetInvocationList())
            {
                try
                {
                    handler(state);
                }
                catch (Exception ex)
                {
                    Log.Warn("Actuator display update failed: " + ex.Message);
                }
            }
            return true;
        }

        internal static async Task<NomadCoreRequestResult> SendGimbalTargetAsync(
            double pitchDeg, double rollDeg, Func<bool> inputStillCurrent = null)
        {
            var result = await SendGimbalAsync(client =>
                client.GimbalTargetAsync(pitchDeg, rollDeg, inputStillCurrent: inputStillCurrent))
                .ConfigureAwait(false);
            if (result.Succeeded)
            {
                ClearGimbalFailure();
            }
            else
            {
                ReportGimbalFailure(DescribeFailure("Gimbal target", result));
            }
            return result;
        }

        internal static async Task<NomadCoreRequestResult> ConfigureGimbalAndSendTargetAsync(
            int mountMode, double pitchDeg, double rollDeg, Func<bool> inputStillCurrent = null)
        {
            var result = await SendGimbalAsync(client =>
                client.GimbalConfigureAndTargetAsync(mountMode, pitchDeg, rollDeg,
                    inputStillCurrent: inputStillCurrent)).ConfigureAwait(false);
            if (result.Succeeded)
            {
                ClearGimbalFailure();
            }
            else
            {
                ReportGimbalFailure(DescribeFailure("Gimbal mode and target", result));
            }
            return result;
        }

        internal static async Task<NomadCoreRequestResult> ConfigureGimbalAsync(
            int mountMode, Func<bool> inputStillCurrent = null)
        {
            var result = await SendGimbalAsync(client => client.GimbalConfigureAsync(mountMode,
                inputStillCurrent: inputStillCurrent)).ConfigureAwait(false);
            if (!result.Succeeded)
            {
                Log.Warn(DescribeFailure("Gimbal configure", result));
            }
            return result;
        }

        private static async Task<NomadCoreRequestResult> SendGimbalAsync(
            Func<NomadCoreClient, Task<NomadCoreRequestResult>> send)
        {
            if (!await GimbalRequests.WaitAsync(0).ConfigureAwait(false))
            {
                return NotSent("request_in_progress", "Another gimbal request is in progress; no request was sent.");
            }
            try
            {
                var client = CreateCoreClient();
                return client == null ? NotSent("core_not_configured", "NOMAD core is not configured.")
                    : await send(client).ConfigureAwait(false);
            }
            finally
            {
                GimbalRequests.Release();
            }
        }

        private static void ReportGimbalFailure(string detail)
        {
            lock (GimbalFailureLock)
            {
                var now = DateTime.UtcNow;
                if (detail == _lastGimbalFailure && now - _lastGimbalFailureAt < TimeSpan.FromSeconds(5))
                {
                    return;
                }
                _lastGimbalFailure = detail;
                _lastGimbalFailureAt = now;
            }
            Log.Warn($"Gimbal target failed: {detail}");
        }

        private static void ClearGimbalFailure()
        {
            lock (GimbalFailureLock)
            {
                _lastGimbalFailure = "";
                _lastGimbalFailureAt = DateTime.MinValue;
            }
        }

        internal static string DescribeConfigurationResult(NomadCoreRequestResult result)
        {
            if (result.ConfigurationRecoveryRequired || (result.ConfigurationChanged && !result.Succeeded))
            {
                return "Runtime configuration changed or may have changed. Restart and review the persisted configuration before further operation. " + result.Message;
            }
            if (result.Succeeded)
            {
                return result.Message;
            }
            string evidence = result.Outcome == NomadCoreRequestOutcome.UnknownOutcome || result.Outcome == NomadCoreRequestOutcome.Interrupted ?
                "Configuration request disposition is uncertain; inspect runtime configuration before submitting again. " :
                result.Outcome == NomadCoreRequestOutcome.NotAttempted || result.Outcome == NomadCoreRequestOutcome.FailedBeforeSend ?
                "Configuration request was not sent. " : "Runtime configuration request failed. ";
            return evidence + result.ErrorCode + ": " + result.Message;
        }

        internal static string DescribeFailure(string action, NomadCoreRequestResult result)
        {
            var evidence = result.Outcome switch
            {
                NomadCoreRequestOutcome.Rejected => "Command was not sent to the vehicle; runtime rejected it.",
                NomadCoreRequestOutcome.Failed => "Operation was attempted and NOMAD obtained a definite failure.",
                NomadCoreRequestOutcome.Interrupted => "Authority or session changed during execution; "
                    + "final vehicle state is unknown. Do not retry blindly.",
                NomadCoreRequestOutcome.UnknownOutcome => "Request may have been transmitted; "
                    + "final vehicle state is unknown. Do not retry blindly.",
                _ => "No runtime mutation request was sent.",
            };
            return $"{action}: {evidence} {result.ErrorCode}: {result.Message}";
        }

    }
}
