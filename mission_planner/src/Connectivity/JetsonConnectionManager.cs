// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Jetson Connection Manager
// ============================================================
// Central singleton tracking Jetson HTTP reachability.
// Components subscribe to state changes to enable/disable controls.
// SAFETY: Flight controls must be disabled when disconnected.
// ============================================================

using System;
using System.Net.Http;
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Jetson connection state for safety-critical UI decisions.
    /// </summary>
    public enum JetsonConnectionState
    {
        Unknown,
        Connected,
        Disconnected
    }

    /// <summary>
    /// Event args for connection state changes.
    /// </summary>
    public class JetsonConnectionStateChangedEventArgs : EventArgs
    {
        public JetsonConnectionState OldState { get; }
        public JetsonConnectionState NewState { get; }
        public string Message { get; }

        public JetsonConnectionStateChangedEventArgs(JetsonConnectionState oldState, JetsonConnectionState newState, string message = null)
        {
            OldState = oldState;
            NewState = newState;
            Message = message;
        }
    }

    /// <summary>
    /// Central manager for Jetson connection state.
    /// SAFETY: Flight controls subscribe to this to disable when disconnected.
    /// </summary>
    public class JetsonConnectionManager : IDisposable
    {
        // ============================================================
        // Constants
        // ============================================================

        private const int HTTP_TIMEOUT_MS = 2500;        // Fail fast timeout
        private const int CONSECUTIVE_FAILURES_THRESHOLD = 2;  // Require 2 failures before disconnected

        // ============================================================
        // Fields
        // ============================================================

        private readonly object _lock = new object();
        private readonly NOMADConfig _config;
        private Timer _pollTimer;
        private JetsonConnectionState _state = JetsonConnectionState.Unknown;
        private int _consecutiveFailures = 0;
        private DateTime _lastSuccessTime = DateTime.MinValue;
        private string _lastError = null;
        private bool _disposed = false;
        private bool _isPolling = false;
        private JetsonStateStream _stateStream;
        // Reentrancy guard: prevents the timer from stacking up CheckConnectionAsync
        // calls if HTTP responses lag behind the poll interval (mirrors the pattern
        // used in EnhancedHealthDashboard).
        private int _checkInFlight = 0;

        // ============================================================
        // Events
        // ============================================================

        /// <summary>
        /// Fired when connection state changes. Subscribe to disable flight controls on disconnect.
        /// </summary>
        public event EventHandler<JetsonConnectionStateChangedEventArgs> ConnectionStateChanged;

        // ============================================================
        // Properties
        // ============================================================

        /// <summary>
        /// Current connection state.
        /// </summary>
        public JetsonConnectionState State
        {
            get { lock (_lock) return _state; }
        }

        /// <summary>
        /// Whether Jetson is currently connected.
        /// SAFETY: Use this to guard flight control operations.
        /// </summary>
        public bool IsConnected
        {
            get { lock (_lock) return _state == JetsonConnectionState.Connected; }
        }

        /// <summary>
        /// Last successful connection time.
        /// </summary>
        public DateTime LastSuccessTime
        {
            get { lock (_lock) return _lastSuccessTime; }
        }

        /// <summary>
        /// Last error message if disconnected.
        /// </summary>
        public string LastError
        {
            get { lock (_lock) return _lastError; }
        }

        // ============================================================
        // Constructor
        // ============================================================

        public JetsonConnectionManager(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
        }

        // ============================================================
        // Public Methods
        // ============================================================

        /// <summary>
        /// Start polling for Jetson connectivity.
        /// </summary>
        public void StartPolling()
        {
            if (_disposed) return;

            lock (_lock)
            {
                if (_isPolling) return;
                _isPolling = true;
            }

            _stateStream = JetsonStateStream.Shared;
            _stateStream.Configure(_config);
            _stateStream.StateUpdated += OnStateStreamUpdated;
            _stateStream.Start();

            // Initial check immediately
            _ = CheckConnectionAsync();

            // Start periodic polling using config interval.
            // Reentrancy-guarded so a slow HTTP roundtrip cannot stack thread-pool work.
            _pollTimer = new Timer(
                async _ =>
                {
                    if (Interlocked.Exchange(ref _checkInFlight, 1) == 1) return;
                    try { await CheckConnectionAsync(); }
                    finally { Interlocked.Exchange(ref _checkInFlight, 0); }
                },
                null,
                _config.HealthPollInterval,
                _config.HealthPollInterval
            );
        }

        /// <summary>
        /// Stop polling.
        /// </summary>
        public void StopPolling()
        {
            lock (_lock)
            {
                _isPolling = false;
            }

            _pollTimer?.Change(Timeout.Infinite, Timeout.Infinite);
            _pollTimer?.Dispose();
            _pollTimer = null;

            if (_stateStream != null)
            {
                _stateStream.StateUpdated -= OnStateStreamUpdated;
                _stateStream = null;
            }
        }

        /// <summary>
        /// Force an immediate connection check.
        /// </summary>
        public async Task<bool> CheckNowAsync()
        {
            return await CheckConnectionAsync();
        }

        /// <summary>
        /// Execute an action only if connected. Returns false if disconnected.
        /// SAFETY: Use this to guard all flight control operations.
        /// </summary>
        public async Task<bool> ExecuteIfConnectedAsync(Func<Task> action, string operationName = "operation")
        {
            if (!IsConnected)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD Safety: Blocked {operationName} - Jetson disconnected");
                return false;
            }

            try
            {
                await action();
                return true;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD: {operationName} failed - {ex.Message}");
                return false;
            }
        }

        // ============================================================
        // Private Methods
        // ============================================================

        private async Task<bool> CheckConnectionAsync()
        {
            if (_disposed) return false;

            bool wasConnected;
            lock (_lock)
            {
                wasConnected = _state == JetsonConnectionState.Connected;
            }

            try
            {
                if (_stateStream != null && _stateStream.HasFreshState)
                {
                    SetConnected();
                    return true;
                }

                var response = await JetsonApiService.GetAsync("/health");

                if (response.IsSuccessStatusCode)
                {
                    SetConnected();
                    return true;
                }
                else
                {
                    SetDisconnected($"HTTP {(int)response.StatusCode}");
                    return false;
                }
            }
            catch (TaskCanceledException)
            {
                SetDisconnected("Connection timeout");
                return false;
            }
            catch (HttpRequestException ex)
            {
                SetDisconnected(ex.InnerException?.Message ?? ex.Message);
                return false;
            }
            catch (Exception ex)
            {
                SetDisconnected(ex.Message);
                return false;
            }
        }

        private void SetConnected()
        {
            JetsonConnectionState oldState;
            bool changed = false;

            lock (_lock)
            {
                oldState = _state;
                _consecutiveFailures = 0;
                _lastSuccessTime = DateTime.Now;
                _lastError = null;

                if (_state != JetsonConnectionState.Connected)
                {
                    _state = JetsonConnectionState.Connected;
                    changed = true;
                }
            }

            if (changed)
            {
                OnConnectionStateChanged(oldState, JetsonConnectionState.Connected, "Connected to Jetson");
            }
        }

        private void OnStateStreamUpdated(Newtonsoft.Json.Linq.JObject state)
        {
            SetConnected();
        }

        private void SetDisconnected(string error)
        {
            JetsonConnectionState oldState;
            bool changed = false;

            lock (_lock)
            {
                oldState = _state;
                _lastError = error;
                _consecutiveFailures++;

                // Only transition to disconnected after consecutive failures (debounce)
                if (_consecutiveFailures >= CONSECUTIVE_FAILURES_THRESHOLD &&
                    _state != JetsonConnectionState.Disconnected)
                {
                    _state = JetsonConnectionState.Disconnected;
                    changed = true;
                }
            }

            if (changed)
            {
                OnConnectionStateChanged(oldState, JetsonConnectionState.Disconnected, error);
            }
        }

        private void OnConnectionStateChanged(JetsonConnectionState oldState, JetsonConnectionState newState, string message)
        {
            System.Diagnostics.Debug.WriteLine($"NOMAD: Jetson connection state: {oldState} -> {newState} ({message})");

            try
            {
                ConnectionStateChanged?.Invoke(this, new JetsonConnectionStateChangedEventArgs(oldState, newState, message));
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"NOMAD: Error in connection state handler - {ex.Message}");
            }
        }

        // ============================================================
        // IDisposable
        // ============================================================

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            StopPolling();
        }
    }
}
