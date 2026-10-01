// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Security.Cryptography;
using System.Threading;
using System.Web.Script.Serialization;

namespace NOMAD.MissionPlanner.Connectivity
{
    internal sealed class NomadRuntimeClient
    {
        private static long _nextSequence;
        private readonly int _runtimePort;
        private readonly string _credential;
        private readonly string _clientId;

        internal NomadCoreRequestOutcome LastOutcome { get; private set; }
        internal string LastErrorCode { get; private set; } = "";
        internal string LastMessage { get; private set; } = "";

        internal NomadRuntimeClient(int runtimePort, string credential, string clientId)
        {
            _runtimePort = runtimePort;
            _credential = credential;
            _clientId = clientId;
        }

        internal int Run(string verb, string[] values)
        {
            LastOutcome = NomadCoreRequestOutcome.NotAttempted;
            LastErrorCode = "";
            LastMessage = "";
            return RunRuntime(verb, values);
        }

        private int RunRuntime(string verb, string[] values)
        {
            var command = BuildRuntimeRequest(verb, values);
            if (command == null)
            {
                LastOutcome = NomadCoreRequestOutcome.Rejected;
                LastErrorCode = "unsupported_request";
                LastMessage = "This operation is not supported by protocol v1.";
                return -1;
            }
            if (string.IsNullOrWhiteSpace(_credential))
            {
                LastOutcome = NomadCoreRequestOutcome.Rejected;
                LastErrorCode = "missing_credential";
                LastMessage = "The NOMAD client credential setting is empty.";
                return -1;
            }

            var commandWriteStarted = false;
            try
            {
                using var client = ConnectToRuntime();
                using var stream = client.GetStream();
                stream.ReadTimeout = 120000;
                stream.WriteTimeout = 3000;
                var serializer = new JavaScriptSerializer { MaxJsonLength = 65536, RecursionLimit = 16 };
                var hello = BaseRequest(Guid.NewGuid().ToString("N"), "hello");
                hello["auth_nonce"] = MakeNonce();
                WriteMessage(stream, serializer.Serialize(hello));
                var helloResponse = ReadResponse(stream, serializer);
                if (!HasAcceptedHello(helloResponse, hello["id"].ToString()))
                {
                    SetProtocolFailure(helloResponse);
                    LastOutcome = NomadCoreRequestOutcome.FailedBeforeSend;
                    return -1;
                }
                var serverPayload = "nomad-core:server:v1:" + _clientId + ":" + hello["auth_nonce"] + ":"
                    + GetString(helloResponse, "runtime_incarnation");
                if (GetString(helloResponse, "client_authentication") != "hmac-sha256-v1" ||
                    !EqualProof(GetString(helloResponse, "server_proof"), MakeProof(_credential, serverPayload)))
                {
                    LastOutcome = NomadCoreRequestOutcome.FailedBeforeSend;
                    LastErrorCode = "authentication_required";
                    LastMessage = "Runtime does not support authenticated clients.";
                    return -1;
                }

                if (!BindAuthority(command, helloResponse))
                {
                    LastOutcome = NomadCoreRequestOutcome.FailedBeforeSend;
                    LastErrorCode = "invalid_response";
                    LastMessage = "Runtime did not provide an authority context.";
                    return -1;
                }

                command["id"] = Guid.NewGuid().ToString("N");
                command["client_id"] = _clientId;
                command["protocol"] = "nomad-core";
                command["version"] = 1;
                var payload = serializer.Serialize(command);
                command["auth_payload"] = payload;
                command["auth_proof"] = MakeProof(_credential, "nomad-core:request:v1:" + payload);
                commandWriteStarted = true;
                WriteMessage(stream, serializer.Serialize(command));
                var response = ReadResponse(stream, serializer);
                return ReadCommandResult(response, command["id"].ToString(), verb);
            }
            catch (Exception ex)
            {
                LastOutcome = commandWriteStarted
                    ? NomadCoreRequestOutcome.UnknownOutcome
                    : NomadCoreRequestOutcome.FailedBeforeSend;
                LastErrorCode = commandWriteStarted ? "unknown_outcome" : "runtime_unavailable";
                LastMessage = commandWriteStarted
                    ? "The runtime connection ended after the request was sent; the vehicle outcome is unknown."
                    : ex.Message;
                return -1;
            }
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
            return BitConverter.ToString(hmac.ComputeHash(Encoding.UTF8.GetBytes(payload))).Replace("-", "").ToLowerInvariant();
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

        private TcpClient ConnectToRuntime()
        {
            var client = new TcpClient(AddressFamily.InterNetwork);
            var pending = client.BeginConnect(IPAddress.Loopback, _runtimePort, null, null);
            try
            {
                if (!pending.AsyncWaitHandle.WaitOne(1500))
                {
                    throw new TimeoutException("Timed out connecting to the NOMAD runtime.");
                }
                client.EndConnect(pending);
                client.SendTimeout = 3000;
                client.ReceiveTimeout = 120000;
                client.NoDelay = true;
                return client;
            }
            catch
            {
                client.Close();
                throw;
            }
            finally
            {
                pending.AsyncWaitHandle.Close();
            }
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
                !authority.TryGetValue("generation", out var generation))
            {
                return false;
            }
            command["runtime_incarnation"] = incarnation;
            command["vehicle_session"] = session;
            command["authority_generation"] = generation;
            command["command_source"] = _clientId;
            command["sequence"] = Interlocked.Increment(ref _nextSequence);
            command["expires_at_ms"] = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() + 3000;
            return true;
        }

        private Dictionary<string, object> BuildRuntimeRequest(string verb, string[] values)
        {
            if (IsAuthorityVerb(verb) && values.Length == 0)
            {
                var authorityType = verb == "admit" ? "admit_authority" :
                                    verb == "revoke" ? "revoke_authority" : "handback_authority";
                return BaseRequest("", authorityType);
            }
            var type = verb switch
            {
                "servo" => "set_servo",
                "relay" => "set_relay",
                "motor-test" => "motor_test",
                "gimbal-config" => "configure_gimbal",
                "gimbal-target" => "set_gimbal_target",
                _ => null,
            };
            if (type == null)
            {
                return null;
            }
            var request = BaseRequest("", type);
            if (verb == "servo" && values.Length == 2)
            {
                request["channel"] = int.Parse(values[0], CultureInfo.InvariantCulture);
                request["pwm_microseconds"] = int.Parse(values[1], CultureInfo.InvariantCulture);
            }
            else if (verb == "relay" && values.Length == 2)
            {
                request["relay_number"] = int.Parse(values[0], CultureInfo.InvariantCulture);
                request["on"] = values[1] == "1";
            }
            else if (verb == "motor-test" && values.Length == 3)
            {
                request["motor_instance"] = int.Parse(values[0], CultureInfo.InvariantCulture);
                request["pwm_microseconds"] = int.Parse(values[1], CultureInfo.InvariantCulture);
                request["timeout_seconds"] = ParseProtocolNumber(values[2]);
            }
            else if (verb == "gimbal-config" && values.Length == 1)
            {
                request["mount_mode"] = int.Parse(values[0], CultureInfo.InvariantCulture);
            }
            else if (verb == "gimbal-target" && values.Length == 2)
            {
                request["pitch_deg"] = ParseProtocolNumber(values[0]);
                request["roll_deg"] = ParseProtocolNumber(values[1]);
            }
            else
            {
                return null;
            }
            return request;
        }

        private static bool IsAuthorityVerb(string verb)
        {
            return verb == "admit" || verb == "revoke" || verb == "handback";
        }

        private static double ParseProtocolNumber(string value)
        {
            return double.Parse(value, NumberStyles.Float, CultureInfo.InvariantCulture);
        }

        private void WriteMessage(Stream stream, string message)
        {
            var bytes = Encoding.UTF8.GetBytes(message + "\n");
            if (bytes.Length > 65537)
            {
                throw new InvalidDataException("Runtime request exceeds the 65536-byte limit.");
            }
            stream.Write(bytes, 0, bytes.Length);
            stream.Flush();
        }

        private static Dictionary<string, object> ReadResponse(Stream stream, JavaScriptSerializer serializer)
        {
            using var bytes = new MemoryStream();
            while (bytes.Length <= 65536)
            {
                var value = stream.ReadByte();
                if (value < 0)
                {
                    throw new EndOfStreamException("Runtime closed the connection before its response.");
                }
                if (value == '\n')
                {
                    var parsed = serializer.DeserializeObject(Encoding.UTF8.GetString(bytes.ToArray()));
                    return parsed as Dictionary<string, object>
                        ?? throw new InvalidDataException("Runtime response must be a JSON object.");
                }
                bytes.WriteByte((byte)value);
            }
            throw new InvalidDataException("Runtime response exceeds the 65536-byte limit.");
        }

        private static bool HasAcceptedHello(Dictionary<string, object> response, string requestId)
        {
            return GetBool(response, "ok") && GetString(response, "protocol") == "nomad-core" &&
                   GetInt(response, "version") == 1 && GetString(response, "id") == requestId &&
                   GetString(response, "type") == "hello_response";
        }

        private int ReadCommandResult(Dictionary<string, object> response, string requestId, string verb)
        {
            if (GetString(response, "id") != requestId || GetString(response, "protocol") != "nomad-core" ||
                GetInt(response, "version") != 1)
            {
                LastOutcome = NomadCoreRequestOutcome.UnknownOutcome;
                LastErrorCode = "unknown_outcome";
                LastMessage = "The runtime response did not match its protocol and ID; "
                    + "the vehicle outcome is unknown.";
                return -1;
            }
            if (TryReadError(response, out var code, out var message))
            {
                LastOutcome = code == "audit_failure" && GetString(response, "outcome") == "unknown"
                    ? NomadCoreRequestOutcome.UnknownOutcome : NomadCoreRequestOutcome.Rejected;
                LastErrorCode = code;
                LastMessage = message;
                return -1;
            }
            if (IsAuthorityVerb(verb))
            {
                if (GetString(response, "type") == "authority_response" &&
                    response.ContainsKey("authority_generation") &&
                    (verb == "revoke" ? GetString(response, "authority_owner") == "" :
                     GetString(response, "authority_owner") == _clientId))
                {
                    LastOutcome = NomadCoreRequestOutcome.Succeeded;
                    LastMessage = "Runtime authority changed explicitly.";
                    return 0;
                }
                LastOutcome = NomadCoreRequestOutcome.UnknownOutcome;
                LastErrorCode = "unknown_outcome";
                LastMessage = "The runtime returned no valid authority result; the outcome is unknown.";
                return -1;
            }
            if (GetString(response, "type") != "command_response" ||
                !response.TryGetValue("command_result", out var resultObject) ||
                resultObject is not Dictionary<string, object> result)
            {
                LastOutcome = NomadCoreRequestOutcome.UnknownOutcome;
                LastErrorCode = "unknown_outcome";
                LastMessage = "The runtime returned no command result; the vehicle outcome is unknown.";
                return -1;
            }
            LastMessage = GetString(result, "message");
            var success = GetBool(result, "success");
            LastOutcome = success ? NomadCoreRequestOutcome.Succeeded : GetString(response, "outcome") == "unknown"
                ? NomadCoreRequestOutcome.UnknownOutcome : NomadCoreRequestOutcome.Rejected;
            LastErrorCode = success ? "" : "vehicle_rejected";
            return success ? 0 : -1;
        }

        private void SetProtocolFailure(Dictionary<string, object> response)
        {
            if (TryReadError(response, out var code, out var message))
            {
                LastErrorCode = string.IsNullOrEmpty(code) ? "incompatible_protocol" : code;
                LastMessage = string.IsNullOrEmpty(message) ? "Runtime protocol negotiation failed." : message;
                return;
            }
            if (GetString(response, "protocol") != "nomad-core")
            {
                LastErrorCode = "incompatible_protocol";
                LastMessage = "The runtime uses an incompatible protocol name.";
                return;
            }
            LastErrorCode = "incompatible_version";
            LastMessage = "The runtime protocol version is not supported.";
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
