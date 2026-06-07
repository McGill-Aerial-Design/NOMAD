// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Jetson API Service
// ============================================================
// Centralized HttpClient management for Jetson API calls.
// Prevents socket exhaustion from per-control HttpClient
// instances and provides consistent timeout/base URL handling.
// ============================================================

using System;
using System.Net;
using System.Net.Http;
using System.Text;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Centralized HTTP client service for Jetson API communication.
    /// All components should use this instead of creating their own HttpClient instances.
    /// </summary>
    public static class JetsonApiService
    {
        // ============================================================
        // Shared HttpClient Instances
        // ============================================================

        /// <summary>
        /// Standard API client (short timeouts for health/status checks).
        /// Timeout: configured via NOMADConfig.HttpTimeoutSeconds, default 5s.
        /// </summary>
        private static HttpClient _apiClient;

        /// <summary>
        /// Long-running operations client (terminal commands, file transfers).
        /// Timeout: 60s.
        /// </summary>
        private static HttpClient _longRunClient;

        /// <summary>
        /// Configuration reference for base URL resolution.
        /// </summary>
        private static NOMADConfig _config;

        private static readonly object _lock = new object();
        private static bool _initialized;

        // ============================================================
        // Initialization
        // ============================================================

        /// <summary>
        /// Initialize the API service with the plugin configuration.
        /// Must be called once during plugin startup.
        /// </summary>
        public static void Initialize(NOMADConfig config)
        {
            lock (_lock)
            {
                _config = config ?? throw new ArgumentNullException(nameof(config));

                // Mission Planner opens several status panels at once.  The
                // .NET Framework default per-host connection limit is too low
                // for parallel health/video/service polls and causes healthy
                // requests to queue behind slow diagnostic endpoints.
                ServicePointManager.DefaultConnectionLimit = Math.Max(
                    ServicePointManager.DefaultConnectionLimit, 16);

                // Build the new clients first, then atomically swap.  Do NOT
                // Dispose() the previous clients here: a background poll
                // (PollJetsonHealth, PollSprayStatus, ...) may still be awaiting
                // a SendAsync call on them and ObjectDisposedException would
                // surface as an unobserved task exception.  Letting the old
                // instances drop out of scope means the GC reclaims them after
                // in-flight requests finish.
                var newApiClient = new HttpClient
                {
                    Timeout = TimeSpan.FromSeconds(Math.Max(1, config.HttpTimeoutSeconds))
                };

                var newLongRunClient = new HttpClient
                {
                    Timeout = TimeSpan.FromSeconds(60)
                };

                // Set API key header if configured
                if (!string.IsNullOrEmpty(config.JetsonApiKey))
                {
                    newApiClient.DefaultRequestHeaders.Add("X-API-Key", config.JetsonApiKey);
                    newLongRunClient.DefaultRequestHeaders.Add("X-API-Key", config.JetsonApiKey);
                }

                _apiClient = newApiClient;
                _longRunClient = newLongRunClient;

                _initialized = true;
            }
        }

        /// <summary>
        /// Re-initialize clients if configuration changes (e.g., timeout updated).
        /// </summary>
        public static void Reconfigure(NOMADConfig config)
        {
            Initialize(config);
        }

        // ============================================================
        // Client Accessors
        // ============================================================

        /// <summary>
        /// Gets the standard API HttpClient for typical Jetson API calls.
        /// Uses the configured HTTP timeout (default 5s).
        /// </summary>
        public static HttpClient ApiClient
        {
            get
            {
                EnsureInitialized();
                return _apiClient;
            }
        }

        /// <summary>
        /// Gets the long-running HttpClient for terminal commands, file transfers, etc.
        /// Uses a 30-second timeout.
        /// </summary>
        public static HttpClient LongRunClient
        {
            get
            {
                EnsureInitialized();
                return _longRunClient;
            }
        }

        // ============================================================
        // Convenience Methods
        // ============================================================

        /// <summary>
        /// Gets the effective base URL from the current configuration.
        /// </summary>
        public static string BaseUrl
        {
            get
            {
                EnsureInitialized();
                return _config.EffectiveBaseUrl;
            }
        }

        /// <summary>
        /// Gets the configured Jetson API key, or null if not set.
        /// </summary>
        public static string ApiKey
        {
            get
            {
                EnsureInitialized();
                var key = _config.JetsonApiKey;
                return string.IsNullOrEmpty(key) ? null : key;
            }
        }

        /// <summary>
        /// Perform a GET request to a Jetson API endpoint.
        /// </summary>
        /// <param name="path">Relative path (e.g., "/health/detailed").</param>
        /// <returns>HTTP response.</returns>
        public static Task<HttpResponseMessage> GetAsync(string path)
        {
            return ApiClient.GetAsync($"{BaseUrl}{path}");
        }

        /// <summary>
        /// Perform a GET request and return the response body as a string.
        /// </summary>
        /// <param name="path">Relative path (e.g., "/health").</param>
        /// <returns>Response body string.</returns>
        public static Task<string> GetStringAsync(string path)
        {
            return ApiClient.GetStringAsync($"{BaseUrl}{path}");
        }

        /// <summary>
        /// Perform a GET request using the long-running client.
        /// Use for diagnostic endpoints that may shell out or inspect Docker.
        /// </summary>
        /// <param name="path">Relative path.</param>
        /// <returns>HTTP response.</returns>
        public static Task<HttpResponseMessage> GetLongRunAsync(string path)
        {
            return LongRunClient.GetAsync($"{BaseUrl}{path}");
        }

        /// <summary>
        /// Perform a POST request to a Jetson API endpoint.
        /// </summary>
        /// <param name="path">Relative path (e.g., "/api/nav/stop").</param>
        /// <param name="content">Optional request body.</param>
        /// <returns>HTTP response.</returns>
        public static Task<HttpResponseMessage> PostAsync(string path, HttpContent content = null)
        {
            return ApiClient.PostAsync($"{BaseUrl}{path}", content);
        }

        /// <summary>
        /// Perform a POST request using the long-running client (30s timeout).
        /// Use for operations like starting video bridges, Isaac ROS, etc.
        /// </summary>
        /// <param name="path">Relative path.</param>
        /// <param name="content">Optional request body.</param>
        /// <returns>HTTP response.</returns>
        public static Task<HttpResponseMessage> PostLongRunAsync(string path, HttpContent content = null)
        {
            return LongRunClient.PostAsync($"{BaseUrl}{path}", content);
        }

        /// <summary>
        /// Perform a POST request with JSON body.
        /// </summary>
        /// <param name="path">Relative path.</param>
        /// <param name="json">JSON string body.</param>
        /// <returns>HTTP response.</returns>
        public static Task<HttpResponseMessage> PostJsonAsync(string path, string json)
        {
            var content = new StringContent(json, Encoding.UTF8, "application/json");
            return ApiClient.PostAsync($"{BaseUrl}{path}", content);
        }

        /// <summary>
        /// Perform a DELETE request to a Jetson API endpoint.
        /// </summary>
        /// <param name="path">Relative path (e.g., "/api/vio/trajectory").</param>
        /// <returns>HTTP response.</returns>
        public static Task<HttpResponseMessage> DeleteAsync(string path)
        {
            return ApiClient.DeleteAsync($"{BaseUrl}{path}");
        }

        /// <summary>
        /// Download binary data (images, files) from a Jetson endpoint.
        /// </summary>
        /// <param name="url">Full URL to download from.</param>
        /// <returns>Byte array of the response body.</returns>
        public static Task<byte[]> GetByteArrayAsync(string url)
        {
            return LongRunClient.GetByteArrayAsync(url);
        }

        // ============================================================
        // Internal Helpers
        // ============================================================

        private static void EnsureInitialized()
        {
            if (!_initialized)
            {
                throw new InvalidOperationException(
                    "JetsonApiService has not been initialized. Call JetsonApiService.Initialize(config) during plugin startup.");
            }
        }

        /// <summary>
        /// Dispose all shared HttpClient instances.
        /// Call during plugin shutdown.
        /// </summary>
        public static void Shutdown()
        {
            lock (_lock)
            {
                _apiClient?.Dispose();
                _longRunClient?.Dispose();
                _apiClient = null;
                _longRunClient = null;
                _initialized = false;
            }
        }
    }
}
