// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Web.Script.Serialization;

namespace NOMAD.MissionPlanner.Connectivity
{
    internal sealed partial class NomadRuntimeClient
    {
        private async Task<TcpClient> ConnectToRuntimeAsync(CancellationToken cancellationToken)
        {
            var client = new TcpClient(AddressFamily.InterNetwork);
            try
            {
                cancellationToken.ThrowIfCancellationRequested();
                using var timeout = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
                timeout.CancelAfter(1500);
                using var cancellation = timeout.Token.Register(client.Close);
                await client.ConnectAsync(IPAddress.Loopback, _runtimePort).ConfigureAwait(false);
                timeout.Token.ThrowIfCancellationRequested();
                client.NoDelay = true;
                return client;
            }
            catch
            {
                client.Close();
                throw;
            }
        }

        private static byte[] EncodeMessage(string message)
        {
            var bytes = Encoding.UTF8.GetBytes(message + "\n");
            if (bytes.Length > 65537)
            {
                throw new InvalidDataException("Runtime request exceeds the 65536-byte limit.");
            }
            return bytes;
        }

        private static async Task WriteMessageAsync(TcpClient client, Stream stream, byte[] bytes,
            CancellationToken cancellationToken, Action writeStarted = null)
        {
            using var timeout = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
            timeout.CancelAfter(3000);
            using var cancellation = timeout.Token.Register(client.Close);
            timeout.Token.ThrowIfCancellationRequested();
            writeStarted?.Invoke();
            await stream.WriteAsync(bytes, 0, bytes.Length, timeout.Token).ConfigureAwait(false);
            timeout.Token.ThrowIfCancellationRequested();
        }

        private async Task<Dictionary<string, object>> ReadResponseAsync(TcpClient client, Stream stream,
            JavaScriptSerializer serializer, CancellationToken cancellationToken)
        {
            using var bytes = new MemoryStream();
            var buffer = new byte[1];
            while (bytes.Length <= 65536)
            {
                using var timeout = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
                timeout.CancelAfter(_responseTimeoutMilliseconds);
                using var cancellation = timeout.Token.Register(client.Close);
                var count = await stream.ReadAsync(buffer, 0, 1, timeout.Token).ConfigureAwait(false);
                timeout.Token.ThrowIfCancellationRequested();
                if (count == 0)
                {
                    throw new EndOfStreamException("Runtime closed the connection before its response.");
                }
                if (buffer[0] == '\n')
                {
                    var parsed = serializer.DeserializeObject(Encoding.UTF8.GetString(bytes.ToArray()));
                    return parsed as Dictionary<string, object>
                        ?? throw new InvalidDataException("Runtime response must be a JSON object.");
                }
                bytes.WriteByte(buffer[0]);
            }
            throw new InvalidDataException("Runtime response exceeds the 65536-byte limit.");
        }
    }
}
