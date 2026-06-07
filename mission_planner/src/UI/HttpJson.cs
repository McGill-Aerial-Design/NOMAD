// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD HTTP+JSON Helper
// ============================================================
// Shared HttpClient patterns used by DualLinkSender and other
// components.  Consolidates timeout/connection/generic error
// handling that was previously duplicated in every helper.
// ============================================================

using System;
using System.Net.Http;
using System.Text;
using System.Threading.Tasks;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    internal static class HttpJson
    {
        /// <summary>
        /// Attempt a GET and return the raw body string.
        /// On error, success is false and error is a human-readable message.
        /// </summary>
        public static async Task<(bool Success, string Body, string Error)> TryGetAsync(
            HttpClient client, string url)
        {
            try
            {
                var response = await client.GetAsync(url).ConfigureAwait(false);
                var body = await response.Content.ReadAsStringAsync().ConfigureAwait(false);
                if (response.IsSuccessStatusCode)
                    return (true, body, null);
                return (false, body, $"HTTP {(int)response.StatusCode}: {response.ReasonPhrase}");
            }
            catch (TaskCanceledException)
            {
                return (false, null, "Request timed out");
            }
            catch (HttpRequestException ex)
            {
                return (false, null, $"Connection failed: {ex.Message}");
            }
            catch (Exception ex)
            {
                return (false, null, ex.Message);
            }
        }

        /// <summary>
        /// Attempt a POST with a JSON-serialized body and return the raw body string.
        /// </summary>
        public static async Task<(bool Success, string Body, string Error)> TryPostAsync(
            HttpClient client, string url, object body)
        {
            try
            {
                var json = JsonConvert.SerializeObject(body ?? new { });
                var content = new StringContent(json, Encoding.UTF8, "application/json");
                var response = await client.PostAsync(url, content).ConfigureAwait(false);
                var responseBody = await response.Content.ReadAsStringAsync().ConfigureAwait(false);
                if (response.IsSuccessStatusCode)
                    return (true, responseBody, null);
                return (false, responseBody, $"HTTP {(int)response.StatusCode}: {response.ReasonPhrase}");
            }
            catch (TaskCanceledException)
            {
                return (false, null, "Request timed out");
            }
            catch (HttpRequestException ex)
            {
                return (false, null, $"Connection failed: {ex.Message}");
            }
            catch (Exception ex)
            {
                return (false, null, ex.Message);
            }
        }

        /// <summary>
        /// Deserialize a JSON body string, returning default(T) on failure.
        /// </summary>
        public static T Deserialize<T>(string body) where T : class
        {
            try
            {
                return JsonConvert.DeserializeObject<T>(body);
            }
            catch (JsonException)
            {
                return null;
            }
        }
    }
}
