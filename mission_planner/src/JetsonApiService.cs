// ============================================================
// NOMAD Jetson API Service
// ============================================================
// Centralized HttpClient management for Jetson API calls.
// Prevents socket exhaustion from per-control HttpClient
// instances and provides consistent timeout/base URL handling.
// ============================================================

using System;
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
        /// Timeout: 30s.
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

                // Dispose old clients if re-initializing
                _apiClient?.Dispose();
                _longRunClient?.Dispose();

                _apiClient = new HttpClient
                {
                    Timeout = TimeSpan.FromSeconds(Math.Max(1, config.HttpTimeoutSeconds))
                };

                _longRunClient = new HttpClient
                {
                    Timeout = TimeSpan.FromSeconds(30)
                };

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
            return ApiClient.GetByteArrayAsync(url);
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
