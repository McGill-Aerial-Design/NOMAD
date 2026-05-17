// ============================================================
// WebSocketClient.cs - SLAM WebSocket connection management
// ============================================================
// Handles WebSocket connection to Edge Core with auto-reconnect.
// ============================================================

using System;
using System.IO;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner.SLAM3D.Network
{
    /// <summary>
    /// WebSocket frame type from Edge Core.
    /// </summary>
    public enum SlamFrameType
    {
        Pose,
        Mesh,
        Unknown
    }
    
    /// <summary>
    /// Parsed frame from WebSocket.
    /// </summary>
    public class SlamFrame
    {
        public SlamFrameType Type { get; set; }
        public JObject RawJson { get; set; }
        
        // Position (always present)
        public float X { get; set; }
        public float Y { get; set; }
        public float Z { get; set; }
        
        // Attitude
        public float Roll { get; set; }
        public float Pitch { get; set; }
        public float Yaw { get; set; }
        public float BodyRoll { get; set; }
        public float BodyPitch { get; set; }
        public float BodyYaw { get; set; }
        public bool HasBodyAttitude { get; set; }
        public bool AttitudeValid { get; set; }
        
        // Velocity (optional)
        public float VelocityX { get; set; }
        public float VelocityY { get; set; }
        public float VelocityZ { get; set; }
        
        // Mesh data (only for mesh frames)
        public JToken MeshToken { get; set; }
        
        // Frame metadata
        public string FrameId { get; set; }
        public int FrameNumber { get; set; }
    }
    
    /// <summary>
    /// Manages WebSocket connection to Edge Core /ws/slam endpoint.
    /// </summary>
    public class WebSocketClient : IDisposable
    {
        // ==================== Configuration ====================
        
        /// <summary>Base URL of Edge Core (e.g., http://100.85.121.98:8000).</summary>
        public string BaseUrl { get; set; } = "http://100.85.121.98:8000";
        
        /// <summary>API key for authentication (optional).</summary>
        public string ApiKey { get; set; }
        
        /// <summary>Initial reconnect delay in milliseconds.</summary>
        public int ReconnectDelayMs { get; set; } = 1000;
        
        /// <summary>Maximum reconnect delay in milliseconds.</summary>
        public int MaxReconnectDelayMs { get; set; } = 30000;
        
        /// <summary>Maximum message size in bytes (64 MB — raised so dense nvblox mesh updates are not silently dropped).</summary>
        public int MaxMessageSize { get; set; } = 64 * 1024 * 1024;
        
        /// <summary>Receive timeout in seconds.</summary>
        public int ReceiveTimeoutSec { get; set; } = 30;
        
        // ==================== State ====================
        
        private ClientWebSocket _webSocket;
        private CancellationTokenSource _cts;
        private int _currentReconnectDelay;
        private bool _disposed;
        
        // Statistics
        private int _framesReceived;
        private int _meshFramesReceived;
        private int _reconnectCount;
        private int _oversizeDrops;
        private long _lastOversizeBytes;
        private DateTime _lastFrameTime = DateTime.MinValue;
        
        // ==================== Events ====================
        
        /// <summary>Fired when a frame is received.</summary>
        public event Action<SlamFrame> OnFrameReceived;
        
        /// <summary>Fired when connection status changes.</summary>
        public event Action<string> OnStatusChanged;
        
        /// <summary>Fired on connection error.</summary>
        public event Action<string> OnError;
        
        // ==================== Properties ====================
        
        /// <summary>Whether currently connected.</summary>
        public bool IsConnected => _webSocket?.State == WebSocketState.Open;
        
        /// <summary>Total frames received.</summary>
        public int FramesReceived => _framesReceived;
        
        /// <summary>Mesh frames received.</summary>
        public int MeshFramesReceived => _meshFramesReceived;
        
        /// <summary>Number of reconnection attempts.</summary>
        public int ReconnectCount => _reconnectCount;

        /// <summary>Number of frames dropped because they exceeded MaxMessageSize.</summary>
        public int OversizeDrops => _oversizeDrops;

        /// <summary>Size in bytes of the most recent oversize-dropped frame.</summary>
        public long LastOversizeBytes => _lastOversizeBytes;
        
        /// <summary>Time since last frame (for health monitoring).</summary>
        public TimeSpan TimeSinceLastFrame => DateTime.UtcNow - _lastFrameTime;
        
        // ==================== Public Methods ====================
        
        /// <summary>
        /// Start the WebSocket connection loop.
        /// </summary>
        public void Start()
        {
            if (_cts != null) return;
            
            _cts = new CancellationTokenSource();
            _currentReconnectDelay = ReconnectDelayMs;
            Task.Run(() => ConnectionLoop(_cts.Token));
        }
        
        /// <summary>
        /// Stop the WebSocket connection.
        /// </summary>
        public void Stop()
        {
            _cts?.Cancel();
            try { _webSocket?.Dispose(); } catch { }
            _webSocket = null;
            _cts = null;
        }
        
        /// <summary>
        /// Get connection statistics.
        /// </summary>
        public (int frames, int meshFrames, int reconnects, bool connected) GetStats()
        {
            return (_framesReceived, _meshFramesReceived, _reconnectCount, IsConnected);
        }
        
        // ==================== Private Methods ====================
        
        private async Task ConnectionLoop(CancellationToken ct)
        {
            while (!ct.IsCancellationRequested)
            {
                try
                {
                    _webSocket = new ClientWebSocket();
                    _webSocket.Options.KeepAliveInterval = TimeSpan.FromSeconds(5);
                    if (!string.IsNullOrEmpty(ApiKey))
                    {
                        _webSocket.Options.SetRequestHeader("X-API-Key", ApiKey);
                    }
                    
                    string wsUrl = BuildWebSocketUrl();
                    OnStatusChanged?.Invoke("Connecting...");
                    
                    await _webSocket.ConnectAsync(new Uri(wsUrl), ct);
                    OnStatusChanged?.Invoke("Connected (30Hz)");
                    _currentReconnectDelay = ReconnectDelayMs; // Reset on successful connect
                    
                    await ReceiveLoop(ct);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (WebSocketException ex)
                {
                    OnError?.Invoke($"WebSocket error: {ex.Message}");
                }
                catch (Exception ex)
                {
                    OnError?.Invoke($"Connection error: {ex.Message}");
                }
                finally
                {
                    try { _webSocket?.Dispose(); } catch { }
                    _webSocket = null;
                }
                
                if (ct.IsCancellationRequested) break;
                
                // Reconnect with exponential backoff
                _reconnectCount++;
                OnStatusChanged?.Invoke($"Reconnecting in {_currentReconnectDelay / 1000}s...");
                
                try
                {
                    await Task.Delay(_currentReconnectDelay, ct);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                
                _currentReconnectDelay = Math.Min(_currentReconnectDelay * 2, MaxReconnectDelayMs);
            }
            
            OnStatusChanged?.Invoke("Disconnected");
        }
        
        private async Task ReceiveLoop(CancellationToken ct)
        {
            var buffer = new byte[64 * 1024];
            var messageBuffer = new MemoryStream();
            
            while (_webSocket.State == WebSocketState.Open && !ct.IsCancellationRequested)
            {
                messageBuffer.SetLength(0);
                WebSocketReceiveResult result = null;
                bool timedOut = false;
                bool oversized = false;
                
                do
                {
                    using var timeoutCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
                    timeoutCts.CancelAfter(TimeSpan.FromSeconds(ReceiveTimeoutSec));
                    
                    try
                    {
                        result = await _webSocket.ReceiveAsync(new ArraySegment<byte>(buffer), timeoutCts.Token);
                    }
                    catch (OperationCanceledException)
                    {
                        if (ct.IsCancellationRequested) throw;
                        timedOut = true;
                        break;
                    }
                    
                    if (result.MessageType == WebSocketMessageType.Close) break;
                    
                    if (!oversized)
                    {
                        messageBuffer.Write(buffer, 0, result.Count);
                        if (messageBuffer.Length > MaxMessageSize) oversized = true;
                    }
                } while (result != null && !result.EndOfMessage);
                
                if (timedOut) break;
                if (result == null || result.MessageType == WebSocketMessageType.Close) break;
                if (oversized)
                {
                    // Track oversized drops visibly instead of silently swallowing
                    // them -- UI stats reads these counters so operators can see
                    // whether mesh visualization is stale because of transport
                    // limits rather than because the bridge stopped publishing.
                    _oversizeDrops++;
                    _lastOversizeBytes = messageBuffer.Length;
                    OnError?.Invoke($"Dropped oversized frame ({_lastOversizeBytes} bytes > {MaxMessageSize} limit)");
                    continue;
                }
                
                // Parse the message
                try
                {
                    string json = Encoding.UTF8.GetString(messageBuffer.GetBuffer(), 0, (int)messageBuffer.Length);
                    var frame = ParseFrame(json);
                    
                    _framesReceived++;
                    _lastFrameTime = DateTime.UtcNow;
                    
                    if (frame.Type == SlamFrameType.Mesh)
                    {
                        _meshFramesReceived++;
                    }
                    
                    OnFrameReceived?.Invoke(frame);
                }
                catch (Exception ex)
                {
                    OnError?.Invoke($"Parse error: {ex.Message}");
                }
            }
        }
        
        private string BuildWebSocketUrl()
        {
            string wsUrl = BaseUrl
                .Replace("https://", "wss://")
                .Replace("http://", "ws://")
                .TrimEnd('/') + "/ws/slam";
            return wsUrl;
        }
        
        private static SlamFrame ParseFrame(string json)
        {
            var jobj = JObject.Parse(json);
            var frame = new SlamFrame
            {
                RawJson = jobj,
                // Canonical SLAM frame identifier end-to-end: "map" (REP-103 axes).
                FrameId = jobj["frame_id"]?.ToString() ?? "map",
                FrameNumber = jobj["ts"]?.Value<int>() ?? 0,
            };
            
            // Determine frame type
            string typeStr = jobj["type"]?.ToString() ?? "pose";
            frame.Type = typeStr.Equals("mesh", StringComparison.OrdinalIgnoreCase) 
                ? SlamFrameType.Mesh 
                : SlamFrameType.Pose;
            
            // Parse position
            if (TryGetFloat(jobj["x"], out float x)) frame.X = x;
            if (TryGetFloat(jobj["y"], out float y)) frame.Y = y;
            if (TryGetFloat(jobj["z"], out float z)) frame.Z = z;
            
            // Parse attitude
            frame.AttitudeValid = jobj["attitude_valid"]?.Value<bool>() ?? true;
            if (TryGetFloat(jobj["roll"], out float roll)) frame.Roll = roll;
            if (TryGetFloat(jobj["pitch"], out float pitch)) frame.Pitch = pitch;
            if (TryGetFloat(jobj["yaw"], out float yaw)) frame.Yaw = yaw;
            
            // Parse body attitude (with gimbal compensation)
            bool hasBodyRoll = TryGetFloat(jobj["body_roll"], out float bodyRoll);
            bool hasBodyPitch = TryGetFloat(jobj["body_pitch"], out float bodyPitch);
            bool hasBodyYaw = TryGetFloat(jobj["body_yaw"], out float bodyYaw);
            if (hasBodyRoll) frame.BodyRoll = bodyRoll;
            if (hasBodyPitch) frame.BodyPitch = bodyPitch;
            if (hasBodyYaw) frame.BodyYaw = bodyYaw;
            frame.HasBodyAttitude = hasBodyRoll && hasBodyPitch && hasBodyYaw;
            
            // Parse velocity
            if (TryGetFloat(jobj["vx"], out float vx)) frame.VelocityX = vx;
            if (TryGetFloat(jobj["vy"], out float vy)) frame.VelocityY = vy;
            if (TryGetFloat(jobj["vz"], out float vz)) frame.VelocityZ = vz;
            
            // Get mesh token for mesh frames
            if (frame.Type == SlamFrameType.Mesh)
            {
                frame.MeshToken = jobj["mesh"];
            }
            
            return frame;
        }
        
        private static bool TryGetFloat(JToken token, out float value)
        {
            value = 0;
            if (token == null) return false;
            
            try
            {
                value = token.Value<float>();
                return true;
            }
            catch
            {
                return false;
            }
        }
        
        // ==================== IDisposable ====================
        
        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            Stop();
        }
    }
}
