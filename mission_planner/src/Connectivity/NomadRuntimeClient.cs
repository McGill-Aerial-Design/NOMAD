// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Web.Script.Serialization;

namespace NOMAD.MissionPlanner.Connectivity
{
    internal sealed partial class NomadRuntimeClient
    {
        private sealed class IdentityRequestState
        {
            internal ulong Last;
            internal readonly SemaphoreSlim MutationGate = new SemaphoreSlim(1, 1);
        }

        private static readonly ConcurrentDictionary<string, IdentityRequestState> RequestStates =
            new ConcurrentDictionary<string, IdentityRequestState>();
        private readonly int _runtimePort;
        private readonly string _credential;
        private readonly string _clientId;
        private readonly int _responseTimeoutMilliseconds;

        internal NomadRuntimeClient(int runtimePort, string credential, string clientId,
            int responseTimeoutMilliseconds = 120000)
        {
            _runtimePort = runtimePort;
            _credential = credential;
            _clientId = clientId;
            _responseTimeoutMilliseconds = responseTimeoutMilliseconds;
        }

        internal async Task<NomadCoreRequestResult> RunAsync(string verb, string[] values,
                                                             CancellationToken cancellationToken, Func<bool> inputStillCurrent = null)
        {
            var command = BuildRuntimeRequest(verb, values);
            if (command == null)
            {
                return new NomadCoreRequestResult(NomadCoreRequestOutcome.NotAttempted,
                    "unsupported_request", "This operation is not supported by protocol v1.");
            }
            if (string.IsNullOrWhiteSpace(_credential))
            {
                return new NomadCoreRequestResult(NomadCoreRequestOutcome.NotAttempted,
                    "missing_credential", "The NOMAD client credential setting is empty.");
            }

            if (IsAuthorityVerb(verb) || IsActuatorVerb(verb))
            {
                return await SendRequestAsync(command, verb, cancellationToken, inputStillCurrent).ConfigureAwait(false);
            }

            var state = GetRequestState();
            if (!await state.MutationGate.WaitAsync(0).ConfigureAwait(false))
            {
                return new NomadCoreRequestResult(NomadCoreRequestOutcome.NotAttempted,
                    "request_in_progress", "Another vehicle mutation for this runtime identity is in progress; "
                        + "no request was sent.");
            }
            try
            {
                return await SendRequestAsync(command, verb, cancellationToken, inputStillCurrent).ConfigureAwait(false);
            }
            finally
            {
                state.MutationGate.Release();
            }
        }

        private async Task<NomadCoreRequestResult> SendRequestAsync(Dictionary<string, object> command,
            string verb, CancellationToken cancellationToken, Func<bool> inputStillCurrent)
        {
            var commandWriteStarted = false;
            try
            {
                using var client = await ConnectToRuntimeAsync(cancellationToken).ConfigureAwait(false);
                using var cancellation = cancellationToken.Register(client.Close);
                using var stream = client.GetStream();
                var serializer = new JavaScriptSerializer { MaxJsonLength = 65536, RecursionLimit = 16 };
                var reader = new ResponseBuffer();
                var hello = BaseRequest(Guid.NewGuid().ToString("N"), "hello");
                var helloResponse = await GetHelloAsync(client, stream, reader, serializer, hello,
                    cancellationToken).ConfigureAwait(false);
                var failure = ValidateHello(helloResponse, hello);
                if (failure != null)
                {
                    return failure;
                }
                if (verb == "land" && !HasLandCapability(helloResponse))
                {
                    return new NomadCoreRequestResult(NomadCoreRequestOutcome.FailedBeforeSend,
                        "unsupported_request", "The authenticated runtime does not advertise LAND engagement. "
                            + "No mutation request was sent.");
                }
                if (inputStillCurrent != null && !inputStillCurrent())
                {
                    return StaleInput();
                }
                if (!BindAuthority(command, helloResponse))
                {
                    return new NomadCoreRequestResult(NomadCoreRequestOutcome.FailedBeforeSend,
                        "invalid_response", "Runtime did not provide an authority context.");
                }

                command["id"] = Guid.NewGuid().ToString("N");
                command["client_id"] = _clientId;
                command["protocol"] = "nomad-core";
                command["version"] = 1;
                var payload = serializer.Serialize(command);
                command["auth_payload"] = payload;
                command["auth_proof"] = MakeProof(_credential, "nomad-core:request:v1:" + payload);
                var bytes = EncodeMessage(serializer.Serialize(command));
                await WriteMessageAsync(client, stream, bytes, cancellationToken,
                    () =>
                    {
                        if (inputStillCurrent != null && !inputStillCurrent())
                        {
                            throw new StaleInputException();
                        }
                        commandWriteStarted = true;
                    }).ConfigureAwait(false);
                var response = await ReadResponseAsync(client, stream, reader, serializer,
                    cancellationToken).ConfigureAwait(false);
                var result = ReadCommandResult(response, command["id"].ToString(), verb,
                    Convert.ToString(command["runtime_incarnation"]));
                result.RequestSequence = Convert.ToUInt64(command["sequence"], CultureInfo.InvariantCulture);
                return result;
            }
            catch (StaleInputException)
            {
                return StaleInput();
            }
            catch (Exception ex)
            {
                return new NomadCoreRequestResult(commandWriteStarted
                    ? NomadCoreRequestOutcome.UnknownOutcome : NomadCoreRequestOutcome.FailedBeforeSend,
                    commandWriteStarted ? "unknown_outcome" : "runtime_unavailable",
                    commandWriteStarted
                        ? "The runtime connection ended after request transmission began; "
                            + "the outcome is unknown. Do not retry blindly."
                        : ex.Message);
            }
        }

        private sealed class StaleInputException : Exception { }
        private static NomadCoreRequestResult StaleInput() => new NomadCoreRequestResult(
            NomadCoreRequestOutcome.NotAttempted, "stale_input", "Physical input changed before transmission; no mutation request was sent.");

        private async Task<Dictionary<string, object>> GetHelloAsync(TcpClient client, Stream stream,
            ResponseBuffer reader, JavaScriptSerializer serializer, Dictionary<string, object> hello,
            CancellationToken cancellationToken)
        {
            hello["auth_nonce"] = MakeNonce();
            await WriteMessageAsync(client, stream, EncodeMessage(serializer.Serialize(hello)),
                cancellationToken).ConfigureAwait(false);
            return await ReadResponseAsync(client, stream, reader, serializer, cancellationToken).ConfigureAwait(false);
        }

        private NomadCoreRequestResult ValidateHello(Dictionary<string, object> response,
                                                     Dictionary<string, object> hello)
        {
            if (!HasAcceptedHello(response, hello["id"].ToString()))
            {
                return ReadProtocolFailure(response);
            }
            var payload = "nomad-core:server:v1:" + _clientId + ":" + hello["auth_nonce"] + ":"
                + GetString(response, "runtime_incarnation");
            if (GetString(response, "client_authentication") != "hmac-sha256-v1" ||
                !EqualProof(GetString(response, "server_proof"), MakeProof(_credential, payload)))
            {
                return new NomadCoreRequestResult(NomadCoreRequestOutcome.FailedBeforeSend,
                    "authentication_required", "Runtime does not support authenticated clients.");
            }
            return null;
        }

        private static string MakeNonce()
        {
            using var random = RandomNumberGenerator.Create();
            var bytes = new byte[32];
            random.GetBytes(bytes);
            return BitConverter.ToString(bytes).Replace("-", "").ToLowerInvariant();
        }

        private static string MakeProof(string secret, string payload)
        {
            using var hmac = new HMACSHA256(Encoding.UTF8.GetBytes(secret));
            return BitConverter.ToString(hmac.ComputeHash(Encoding.UTF8.GetBytes(payload)))
                .Replace("-", "").ToLowerInvariant();
        }

        private static bool EqualProof(string left, string right)
        {
            if (left.Length != 64 || right.Length != 64)
            {
                return false;
            }
            int difference = 0;
            for (int index = 0; index < 64; index++)
            {
                difference |= left[index] ^ right[index];
            }
            return difference == 0;
        }

        private Dictionary<string, object> BaseRequest(string id, string type)
        {
            return new Dictionary<string, object>
            {
                ["protocol"] = "nomad-core",
                ["version"] = 1,
                ["id"] = id,
                ["client_id"] = _clientId,
                ["type"] = type,
            };
        }

        private bool BindAuthority(Dictionary<string, object> command, Dictionary<string, object> hello)
        {
            if (!hello.TryGetValue("runtime_incarnation", out var incarnation) ||
                !hello.TryGetValue("authority", out var rawAuthority) ||
                !(rawAuthority is Dictionary<string, object> authority) ||
                !authority.TryGetValue("vehicle_session", out var session) ||
                !authority.TryGetValue("generation", out var generation) ||
                !authority.TryGetValue("next_sequence", out var rawSequence) ||
                !ulong.TryParse(Convert.ToString(rawSequence, CultureInfo.InvariantCulture),
                    NumberStyles.None, CultureInfo.InvariantCulture, out var lowerBound) || lowerBound == 0)
            {
                return false;
            }
            command["runtime_incarnation"] = incarnation;
            command["vehicle_session"] = session;
            command["authority_generation"] = generation;
            command["command_source"] = _clientId;
            command["sequence"] = AllocateSequence(lowerBound);
            command["expires_at_ms"] = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() + (IsActuatorType(GetString(command, "type")) ? 5000 : 3000);
            return true;
        }

        private IdentityRequestState GetRequestState()
        {
            return RequestStates.GetOrAdd(_runtimePort + ":" + _clientId, _ => new IdentityRequestState());
        }

        private ulong AllocateSequence(ulong lowerBound)
        {
            var counter = GetRequestState();
            lock (counter)
            {
                if (counter.Last == ulong.MaxValue)
                {
                    throw new InvalidDataException("Runtime request sequence is exhausted.");
                }
                counter.Last = Math.Max(lowerBound, counter.Last + 1);
                return counter.Last;
            }
        }

        private static bool HasAcceptedHello(Dictionary<string, object> response, string requestId)
        {
            return GetBool(response, "ok") && GetString(response, "protocol") == "nomad-core" &&
                   GetInt(response, "version") == 1 && GetString(response, "id") == requestId &&
                   GetString(response, "type") == "hello_response";
        }

        private NomadCoreRequestResult ReadCommandResult(Dictionary<string, object> response,
            string requestId, string verb, string incarnation)
        {
            var outcome = NomadCoreRequestOutcome.UnknownOutcome;
            var errorCode = "";
            var messageText = "";
            if (GetString(response, "id") != requestId || GetString(response, "protocol") != "nomad-core" ||
                GetInt(response, "version") != 1)
            {
                outcome = NomadCoreRequestOutcome.UnknownOutcome;
                errorCode = "unknown_outcome";
                messageText = "The runtime response did not match its protocol and ID; the outcome is unknown.";
                return new NomadCoreRequestResult(outcome, errorCode, messageText);
            }

            if (IsActuatorVerb(verb))
            {
                return ReadActuatorResult(response, verb, incarnation);
            }
            if (verb == "land" && !HasValidLandResult(response))
            {
                return new NomadCoreRequestResult(NomadCoreRequestOutcome.UnknownOutcome, "unknown_outcome",
                    "The runtime returned an incomplete or contradictory LAND result. "
                        + "LAND engagement and touchdown are unverified. Do not retry blindly.",
                    ReadAcknowledgement(response));
            }

            if (TryReadError(response, out var code, out var message))
            {
                outcome = IsAuthorityVerb(verb) && !response.ContainsKey("outcome")
                    ? NomadCoreRequestOutcome.Rejected : ReadOutcome(response);
                if (outcome == NomadCoreRequestOutcome.Succeeded)
                {
                    outcome = NomadCoreRequestOutcome.UnknownOutcome;
                }
                errorCode = code;
                messageText = message;
                return new NomadCoreRequestResult(outcome, errorCode, messageText, ReadAcknowledgement(response));
            }
            return IsAuthorityVerb(verb) ? ReadAuthorityResult(response, verb) : ReadVehicleResult(response);
        }

        private NomadCoreRequestResult ReadAuthorityResult(Dictionary<string, object> response, string verb)
        {
            var outcome = NomadCoreRequestOutcome.UnknownOutcome;
            var errorCode = "";
            var messageText = "";
            if (GetString(response, "type") == "authority_response" &&
                response.ContainsKey("authority_generation") &&
                (verb == "revoke" ? GetString(response, "authority_owner") == "" :
                 GetString(response, "authority_owner") == _clientId))
            {
                outcome = NomadCoreRequestOutcome.Succeeded;
                messageText = "Runtime authority changed explicitly.";
                return new NomadCoreRequestResult(outcome, errorCode, messageText, ReadAcknowledgement(response));
            }
            outcome = NomadCoreRequestOutcome.UnknownOutcome;
            errorCode = "unknown_outcome";
            messageText = "The runtime returned no valid authority result; the outcome is unknown.";
            return new NomadCoreRequestResult(outcome, errorCode, messageText, ReadAcknowledgement(response));
        }

        private NomadCoreRequestResult ReadVehicleResult(Dictionary<string, object> response)
        {
            var outcome = NomadCoreRequestOutcome.UnknownOutcome;
            var errorCode = "";
            var messageText = "";
            if (GetString(response, "type") != "command_response" ||
                !response.TryGetValue("command_result", out var resultObject) ||
                resultObject is not Dictionary<string, object> result)
            {
                outcome = NomadCoreRequestOutcome.UnknownOutcome;
                errorCode = "unknown_outcome";
                messageText = "The runtime returned no command result; the vehicle outcome is unknown.";
                return new NomadCoreRequestResult(outcome, errorCode, messageText, ReadAcknowledgement(response));
            }
            messageText = GetString(result, "message");
            outcome = ReadOutcome(response);
            var success = outcome == NomadCoreRequestOutcome.Succeeded && GetBool(result, "success");
            if (outcome == NomadCoreRequestOutcome.Succeeded && !success)
            {
                outcome = NomadCoreRequestOutcome.UnknownOutcome;
            }
            errorCode = success ? "" : outcome == NomadCoreRequestOutcome.UnknownOutcome
                ? "unknown_outcome" : "vehicle_" + GetString(response, "outcome");
            return new NomadCoreRequestResult(outcome, errorCode, messageText, ReadAcknowledgement(response));
        }

        private static bool? ReadAcknowledgement(Dictionary<string, object> response)
        {
            if (response.TryGetValue("command_result", out var rawResult) &&
                rawResult is Dictionary<string, object> result &&
                result.TryGetValue("acknowledged", out var value) && value is bool acknowledged)
            {
                return acknowledged;
            }
            return null;
        }

        private static NomadCoreRequestOutcome ReadOutcome(Dictionary<string, object> response)
        {
            if (GetString(response, "outcome") == "rejected" &&
                response.TryGetValue("command_result", out var rawResult) &&
                rawResult is Dictionary<string, object> result &&
                (GetBool(result, "acknowledged") || GetBool(result, "success")))
            {
                return NomadCoreRequestOutcome.UnknownOutcome;
            }
            return GetString(response, "outcome") switch
            {
                "success" => NomadCoreRequestOutcome.Succeeded,
                "rejected" => NomadCoreRequestOutcome.Rejected,
                "failed" => NomadCoreRequestOutcome.Failed,
                "interrupted" => NomadCoreRequestOutcome.Interrupted,
                _ => NomadCoreRequestOutcome.UnknownOutcome,
            };
        }

        private static NomadCoreRequestResult ReadProtocolFailure(Dictionary<string, object> response)
        {
            var errorCode = "";
            var messageText = "";
            if (TryReadError(response, out var code, out var message))
            {
                errorCode = string.IsNullOrEmpty(code) ? "incompatible_protocol" : code;
                messageText = string.IsNullOrEmpty(message) ? "Runtime protocol negotiation failed." : message;
                return new NomadCoreRequestResult(NomadCoreRequestOutcome.FailedBeforeSend, errorCode, messageText);
            }
            if (GetString(response, "protocol") != "nomad-core")
            {
                errorCode = "incompatible_protocol";
                messageText = "The runtime uses an incompatible protocol name.";
                return new NomadCoreRequestResult(NomadCoreRequestOutcome.FailedBeforeSend, errorCode, messageText);
            }
            errorCode = "incompatible_version";
            messageText = "The runtime protocol version is not supported.";
            return new NomadCoreRequestResult(NomadCoreRequestOutcome.FailedBeforeSend, errorCode, messageText);
        }

        private static bool TryReadError(Dictionary<string, object> response, out string code, out string message)
        {
            code = "";
            message = "";
            if (GetBool(response, "ok"))
            {
                return false;
            }
            if (response.TryGetValue("error", out var errorObject) &&
                errorObject is Dictionary<string, object> error)
            {
                code = GetString(error, "code");
                message = GetString(error, "message");
            }
            return true;
        }

        private static string GetString(Dictionary<string, object> values, string key)
        {
            if (!values.TryGetValue(key, out var value))
            {
                return "";
            }
            return Convert.ToString(value, CultureInfo.InvariantCulture) ?? "";
        }

        private static bool GetBool(Dictionary<string, object> values, string key)
        {
            return values.TryGetValue(key, out var value) && value is bool boolean && boolean;
        }

        private static int GetInt(Dictionary<string, object> values, string key)
        {
            return values.TryGetValue(key, out var value)
                ? Convert.ToInt32(value, CultureInfo.InvariantCulture)
                : 0;
        }
    }
}
