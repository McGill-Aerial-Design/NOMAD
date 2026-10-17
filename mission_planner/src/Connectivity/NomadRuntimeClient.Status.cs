// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Web.Script.Serialization;

namespace NOMAD.MissionPlanner.Connectivity
{
    internal sealed partial class NomadRuntimeClient
    {
        internal async Task<NomadStatusResult> GetStatusAsync(CancellationToken cancellationToken)
        {
            if (!ValidRuntimePort)
            {
                return new NomadStatusResult(null, "invalid_configuration", "The runtime IPC port is invalid.");
            }
            if (string.IsNullOrWhiteSpace(_credential))
            {
                return new NomadStatusResult(null, "missing_credential",
                    "The NOMAD client credential setting is empty.");
            }
            using var timeout = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
            // This bounds the entire read, including a peer that continually renews the read-idle timer.
            timeout.CancelAfter(5000);
            var started = Stopwatch.GetTimestamp();
            try
            {
                using var client = await ConnectToRuntimeAsync(timeout.Token).ConfigureAwait(false);
                using var cancellation = timeout.Token.Register(client.Close);
                using var stream = client.GetStream();
                var serializer = new JavaScriptSerializer { MaxJsonLength = 65536, RecursionLimit = 16 };
                var reader = new ResponseBuffer();
                var hello = BaseRequest(Guid.NewGuid().ToString("N"), "hello");
                var helloResponse = await GetHelloAsync(client, stream, reader, serializer, hello, timeout.Token)
                    .ConfigureAwait(false);
                var failure = ValidateHello(helloResponse, hello);
                if (failure != null)
                {
                    return new NomadStatusResult(null, failure.ErrorCode,
                        "Runtime authentication or protocol negotiation failed.");
                }
                if (!ReadEnvelopeMatches(helloResponse, hello["id"].ToString(), "hello_response"))
                {
                    throw new InvalidDataException("Runtime HELLO envelope is invalid.");
                }
                var request = BaseRequest(Guid.NewGuid().ToString("N"), "status");
                await WriteMessageAsync(client, stream, EncodeMessage(serializer.Serialize(request)), timeout.Token)
                    .ConfigureAwait(false);
                var response = await ReadResponseAsync(client, stream, reader, serializer, timeout.Token)
                    .ConfigureAwait(false);
                timeout.Token.ThrowIfCancellationRequested();
                if (!ReadEnvelopeMatches(response, request["id"].ToString(), "status_response"))
                {
                    throw new InvalidDataException("Runtime did not return a matching STATUS response.");
                }
                var snapshot = NomadRuntimeStatus.Parse(helloResponse, response, started);
                timeout.Token.ThrowIfCancellationRequested();
                return new NomadStatusResult(snapshot);
            }
            catch (Exception exception)
            {
                if (timeout.IsCancellationRequested)
                {
                    return cancellationToken.IsCancellationRequested
                        ? new NomadStatusResult(null, "cancelled", "Runtime status request was cancelled.")
                        : new NomadStatusResult(null, "status_timeout",
                            "Runtime status exceeded its five-second deadline.");
                }
                if (exception is InvalidDataException || exception is ArgumentException
                    || exception is FormatException || exception is OverflowException
                    || exception is InvalidCastException)
                {
                    return new NomadStatusResult(null, "invalid_response",
                        "Runtime returned an invalid STATUS response.");
                }
                return new NomadStatusResult(null, "runtime_unavailable", "Runtime status is unavailable.");
            }
        }

        private static bool ReadEnvelopeMatches(Dictionary<string, object> response, string id, string type) =>
            GetBool(response, "ok")
            && response.TryGetValue("protocol", out var protocol) && protocol is string name && name == "nomad-core"
            && response.TryGetValue("version", out var version) && version is int number && number == 1
            && response.TryGetValue("id", out var responseId) && responseId is string text && text == id
            && response.TryGetValue("type", out var responseType) && responseType is string kind && kind == type;
    }
}
