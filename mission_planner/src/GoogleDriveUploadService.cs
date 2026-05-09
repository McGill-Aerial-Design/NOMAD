// ============================================================
// NOMAD Google Drive Upload Service
// ============================================================
// Direct Google Drive uploads from the GCS using OAuth2 token.
// Uses raw REST API calls - no Google NuGet packages required.
// ============================================================

using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Http;
using System.Text;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Uploads files to Google Drive using OAuth2 user credentials and the Drive REST API.
    /// </summary>
    public class GoogleDriveUploadService
    {
        private static readonly string TOKEN_DIR = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".nomad");
        private static readonly string TOKEN_PATH = Path.Combine(TOKEN_DIR, "gdrive_token.json");
        private static readonly string FOLDER_ID_ENV = "GDRIVE_FOLDER_ID";

        private readonly string _tokenPath;
        private readonly string _folderId;

        public GoogleDriveUploadService(string tokenPath = null, string folderId = null)
        {
            _tokenPath = tokenPath ?? TOKEN_PATH;
            _folderId = folderId ?? Environment.GetEnvironmentVariable(FOLDER_ID_ENV);
        }

        /// <summary>
        /// Check if a token file exists (does NOT validate the token content).
        /// </summary>
        public bool HasToken()
        {
            return File.Exists(_tokenPath);
        }

        /// <summary>
        /// Return a human-readable description of the token file state, or null if everything looks ok.
        /// </summary>
        public string DiagnoseToken()
        {
            if (!File.Exists(_tokenPath))
                return $"Token file not found: {_tokenPath}";
            try
            {
                var json = File.ReadAllText(_tokenPath);
                var tokenData = JObject.Parse(json);
                var missing = new List<string>();
                if (string.IsNullOrEmpty(tokenData["access_token"]?.ToString()))   missing.Add("access_token");
                if (string.IsNullOrEmpty(tokenData["refresh_token"]?.ToString()))  missing.Add("refresh_token");
                if (string.IsNullOrEmpty(tokenData["client_id"]?.ToString()))      missing.Add("client_id");
                if (string.IsNullOrEmpty(tokenData["client_secret"]?.ToString()))  missing.Add("client_secret");
                if (missing.Count > 0)
                    return $"Token file is missing fields: {string.Join(", ", missing)}\nFile: {_tokenPath}";
                return null; // looks ok
            }
            catch (Exception ex)
            {
                return $"Token file is not valid JSON: {ex.Message}\nFile: {_tokenPath}";
            }
        }

        /// <summary>
        /// Upload a file to Google Drive.
        /// </summary>
        /// <param name="localPath">Path to the local file to upload.</param>
        /// <param name="filename">Name for the file in Google Drive.</param>
        /// <param name="folderId">Optional Drive folder ID (overrides constructor value).</param>
        /// <returns>Google Drive file ID on success.</returns>
        /// <exception cref="Exception">Thrown with the HTTP status and response body on any failure.</exception>
        public async Task<string> UploadFileAsync(string localPath, string filename, string folderId = null)
        {
            if (!File.Exists(localPath))
                throw new FileNotFoundException($"Local file not found: {localPath}");

            string token = GetAccessToken();
            if (string.IsNullOrEmpty(token))
            {
                if (!File.Exists(_tokenPath))
                    throw new Exception($"Token file missing: {_tokenPath}\nPlace gdrive_token.json in ~/.nomad/");
                throw new Exception($"Token file exists but contains no access_token: {_tokenPath}");
            }

            string targetFolderId = folderId ?? _folderId;

            using (var client = new HttpClient { Timeout = TimeSpan.FromMinutes(2) })
            {
                client.DefaultRequestHeaders.Authorization =
                    new System.Net.Http.Headers.AuthenticationHeaderValue("Bearer", token);

                var (response, responseBody) = await DoUploadAsync(client, localPath, filename, targetFolderId);

                if (response.IsSuccessStatusCode)
                {
                    var result = JObject.Parse(responseBody);
                    return result["id"]?.ToString() ?? "";
                }

                // Token expired — try to refresh and retry once
                if (response.StatusCode == System.Net.HttpStatusCode.Unauthorized)
                {
                    string newToken;
                    try { newToken = await RefreshTokenAsync(); }
                    catch (Exception refreshEx)
                    {
                        throw new Exception($"Token expired and refresh failed: {refreshEx.Message}");
                    }

                    if (string.IsNullOrEmpty(newToken))
                        throw new Exception(
                            "Token expired (HTTP 401) and refresh_token / client_id / client_secret are missing " +
                            "from the token file. Re-run the OAuth flow to get a fresh token.\n\n" +
                            $"Drive response: {responseBody}");

                    client.DefaultRequestHeaders.Authorization =
                        new System.Net.Http.Headers.AuthenticationHeaderValue("Bearer", newToken);

                    var (response2, responseBody2) = await DoUploadAsync(client, localPath, filename, targetFolderId);

                    if (response2.IsSuccessStatusCode)
                    {
                        var result2 = JObject.Parse(responseBody2);
                        return result2["id"]?.ToString() ?? "";
                    }

                    throw new Exception(
                        $"Upload failed after token refresh: HTTP {(int)response2.StatusCode} {response2.ReasonPhrase}\n{responseBody2}");
                }

                throw new Exception(
                    $"Upload failed: HTTP {(int)response.StatusCode} {response.ReasonPhrase}\n{responseBody}");
            }
        }

        private async Task<(HttpResponseMessage response, string body)> DoUploadAsync(
            HttpClient client, string localPath, string filename, string targetFolderId)
        {
            var boundary = "----NOMADUploadBoundary" + DateTime.Now.Ticks.ToString("x");
            var content = new MultipartContent("related", boundary);

            var metadata = new JObject { { "name", filename } };
            if (!string.IsNullOrEmpty(targetFolderId))
                metadata["parents"] = new JArray { targetFolderId };

            var metadataContent = new StringContent(
                metadata.ToString(Formatting.None), Encoding.UTF8, "application/json");
            metadataContent.Headers.Add("Content-ID", "<metadata>");
            content.Add(metadataContent);

            var fileStream = new FileStream(localPath, FileMode.Open, FileAccess.Read);
            var streamContent = new StreamContent(fileStream);
            streamContent.Headers.Add("Content-Type", GetMimeType(localPath));
            streamContent.Headers.Add("Content-ID", "<file-data>");
            content.Add(streamContent);

            var response = await client.PostAsync(
                "https://www.googleapis.com/upload/drive/v3/files?uploadType=multipart&fields=id",
                content);
            var body = await response.Content.ReadAsStringAsync();
            return (response, body);
        }

        /// <summary>
        /// Get the current access token from the token file.
        /// </summary>
        private string GetAccessToken()
        {
            try
            {
                if (!File.Exists(_tokenPath))
                    return null;

                var json = File.ReadAllText(_tokenPath);
                var tokenData = JObject.Parse(json);
                return tokenData["access_token"]?.ToString();
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Refresh the OAuth2 token using the refresh_token.
        /// Returns null if refresh_token / client_id / client_secret are missing (caller decides what to do).
        /// Throws if the OAuth2 endpoint rejects the request.
        /// </summary>
        private async Task<string> RefreshTokenAsync()
        {
            if (!File.Exists(_tokenPath))
                return null;

            var json = File.ReadAllText(_tokenPath);
            var tokenData = JObject.Parse(json);
            var refreshToken = tokenData["refresh_token"]?.ToString();
            var clientId = tokenData["client_id"]?.ToString();
            var clientSecret = tokenData["client_secret"]?.ToString();

            if (string.IsNullOrEmpty(refreshToken) || string.IsNullOrEmpty(clientId) || string.IsNullOrEmpty(clientSecret))
                return null;

            using (var client = new HttpClient())
            {
                var postData = new FormUrlEncodedContent(new[]
                {
                    new KeyValuePair<string, string>("client_id", clientId),
                    new KeyValuePair<string, string>("client_secret", clientSecret),
                    new KeyValuePair<string, string>("refresh_token", refreshToken),
                    new KeyValuePair<string, string>("grant_type", "refresh_token"),
                });

                var response = await client.PostAsync("https://oauth2.googleapis.com/token", postData);
                var responseBody = await response.Content.ReadAsStringAsync();

                if (!response.IsSuccessStatusCode)
                    throw new Exception(
                        $"OAuth2 token refresh rejected: HTTP {(int)response.StatusCode} {response.ReasonPhrase}\n{responseBody}");

                var result = JObject.Parse(responseBody);
                var newAccessToken = result["access_token"]?.ToString();

                if (!string.IsNullOrEmpty(newAccessToken))
                {
                    tokenData["access_token"] = newAccessToken;
                    if (result["expires_in"] != null)
                        tokenData["expires_in"] = result["expires_in"];
                    if (result["scope"] != null)
                        tokenData["scope"] = result["scope"];
                    tokenData["token_type"] = result["token_type"]?.ToString() ?? "Bearer";

                    Directory.CreateDirectory(Path.GetDirectoryName(_tokenPath));
                    File.WriteAllText(_tokenPath, tokenData.ToString(Formatting.Indented));
                }

                return newAccessToken;
            }
        }

        private string GetMimeType(string filePath)
        {
            var ext = Path.GetExtension(filePath).ToLowerInvariant();
            return ext switch
            {
                ".jpg" or ".jpeg" => "image/jpeg",
                ".png" => "image/png",
                ".bmp" => "image/bmp",
                ".txt" => "text/plain",
                ".json" => "application/json",
                _ => "application/octet-stream",
            };
        }
    }
}
