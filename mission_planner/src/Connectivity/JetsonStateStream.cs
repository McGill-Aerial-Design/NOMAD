// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Jetson State Stream
// ============================================================
// Shared /ws/state client used as the primary source for high-rate status.
// HTTP polling remains available for command responses and low-rate detail.
// ============================================================

using System;
using System.IO;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public sealed class JetsonStateStream : IDisposable
    {
        private static readonly object SharedLock = new object();
        private static JetsonStateStream _shared;

        public static JetsonStateStream Shared
        {
            get
            {
                lock (SharedLock)
                {
                    return _shared ?? (_shared = new JetsonStateStream());
                }
            }
        }

        private readonly object _lock = new object();
        private NOMADConfig _config;
        private ClientWebSocket _webSocket;
        private CancellationTokenSource _cts;
        private Task _loopTask;
        private JObject _latestState;
        private DateTime _lastMessageUtc = DateTime.MinValue;
        private bool _disposed;

        public event Action<JObject> StateUpdated;
        public event Action<bool, string> ConnectionChanged;

        public JObject LatestState
        {
            get { lock (_lock) return _latestState == null ? null : (JObject)_latestState.DeepClone(); }
        }

        public DateTime LastMessageUtc
        {
            get { lock (_lock) return _lastMessageUtc; }
        }

        public bool IsConnected => _webSocket != null && _webSocket.State == WebSocketState.Open;

        public bool HasFreshState
        {
            get
            {
                lock (_lock)
                {
                    return _latestState != null &&
                           (DateTime.UtcNow - _lastMessageUtc) < TimeSpan.FromSeconds(5);
                }
            }
        }

        public void Configure(NOMADConfig config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));
            lock (_lock)
            {
                _config = config;
            }
        }

        public void Start()
        {
            if (_disposed) return;
            lock (_lock)
            {
                if (_loopTask != null) return;
                _cts = new CancellationTokenSource();
                _loopTask = Task.Run(() => ConnectionLoopAsync(_cts.Token));
            }
        }

        public void Stop()
        {
            CancellationTokenSource cts;
            ClientWebSocket ws;
            lock (_lock)
            {
                cts = _cts;
                ws = _webSocket;
                _cts = null;
                _webSocket = null;
                _loopTask = null;
            }

            try { cts?.Cancel(); } catch { }
            try { ws?.Dispose(); } catch { }
        }

        private async Task ConnectionLoopAsync(CancellationToken ct)
        {
            var reconnectDelayMs = 1000;

            while (!ct.IsCancellationRequested)
            {
                try
                {
                    using (var ws = new ClientWebSocket())
                    {
                        ws.Options.KeepAliveInterval = TimeSpan.FromSeconds(5);
                        var apiKey = JetsonApiService.ApiKey;
                        if (!string.IsNullOrEmpty(apiKey))
                        {
                            ws.Options.SetRequestHeader("X-API-Key", apiKey);
                        }

                        lock (_lock) _webSocket = ws;
                        var url = BuildWebSocketUrl();
                        await ws.ConnectAsync(new Uri(url), ct);
                        reconnectDelayMs = 1000;
                        ConnectionChanged?.Invoke(true, "Connected to /ws/state");

                        await ReceiveLoopAsync(ws, ct);
                    }
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    ConnectionChanged?.Invoke(false, ex.Message);
                }
                finally
                {
                    lock (_lock) _webSocket = null;
                }

                if (ct.IsCancellationRequested) break;

                try
                {
                    await Task.Delay(reconnectDelayMs, ct);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                reconnectDelayMs = Math.Min(reconnectDelayMs * 2, 30000);
            }

            ConnectionChanged?.Invoke(false, "Disconnected from /ws/state");
        }

        private async Task ReceiveLoopAsync(ClientWebSocket ws, CancellationToken ct)
        {
            var buffer = new byte[64 * 1024];
            using (var message = new MemoryStream())
            {
                while (ws.State == WebSocketState.Open && !ct.IsCancellationRequested)
                {
                    message.SetLength(0);
                    WebSocketReceiveResult result;
                    do
                    {
                        result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), ct);
                        if (result.MessageType == WebSocketMessageType.Close)
                            return;
                        message.Write(buffer, 0, result.Count);
                    } while (!result.EndOfMessage);

                    var json = Encoding.UTF8.GetString(message.GetBuffer(), 0, (int)message.Length);
                    var state = JObject.Parse(json);
                    lock (_lock)
                    {
                        _latestState = state;
                        _lastMessageUtc = DateTime.UtcNow;
                    }
                    try
                    {
                        StateUpdated?.Invoke(state);
                    }
                    catch (Exception ex)
                    {
                        System.Diagnostics.Debug.WriteLine($"NOMAD state stream handler error: {ex.Message}");
                    }
                }
            }
        }

        private string BuildWebSocketUrl()
        {
            NOMADConfig config;
            lock (_lock) config = _config;
            var baseUrl = (config?.EffectiveBaseUrl ?? JetsonApiService.BaseUrl).TrimEnd('/');
            return baseUrl.Replace("https://", "wss://").Replace("http://", "ws://") + "/ws/state";
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            Stop();
        }
    }
}
