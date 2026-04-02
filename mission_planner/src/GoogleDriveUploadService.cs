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
        /// Check if a valid token exists.
        /// </summary>
        public bool HasToken()
        {
            return File.Exists(_tokenPath);
        }

        /// <summary>
        /// Upload a file to Google Drive.
        /// </summary>
        /// <param name="localPath">Path to the local file to upload.</param>
        /// <param name="filename">Name for the file in Google Drive.</param>
        /// <param name="folderId">Optional Drive folder ID (overrides constructor value).</param>
        /// <returns>Google Drive file ID on success, empty string on failure.</returns>
        public async Task<string> UploadFileAsync(string localPath, string filename, string folderId = null)
        {
            if (!File.Exists(localPath))
                return "";

            string token = GetAccessToken();
            if (string.IsNullOrEmpty(token))
                return "";

            string targetFolderId = folderId ?? _folderId;

            try
            {
                using (var client = new HttpClient { Timeout = TimeSpan.FromMinutes(2) })
                {
                    client.DefaultRequestHeaders.Authorization =
                        new System.Net.Http.Headers.AuthenticationHeaderValue("Bearer", token);

                    // Build multipart upload request
                    var boundary = "----NOMADUploadBoundary" + DateTime.Now.Ticks.ToString("x");
                    var content = new MultipartContent("related", boundary);

                    // Part 1: metadata JSON
                    var metadata = new JObject { { "name", filename } };
                    if (!string.IsNullOrEmpty(targetFolderId))
                        metadata["parents"] = new JArray { targetFolderId };

                    var metadataContent = new StringContent(
                        metadata.ToString(Formatting.None),
                        Encoding.UTF8,
                        "application/json");
                    metadataContent.Headers.Add("Content-ID", "<metadata>");
                    content.Add(metadataContent);

                    // Part 2: file data
                    var fileStream = new FileStream(localPath, FileMode.Open, FileAccess.Read);
                    var streamContent = new StreamContent(fileStream);
                    streamContent.Headers.Add("Content-Type", GetMimeType(localPath));
                    streamContent.Headers.Add("Content-ID", "<file-data>");
                    content.Add(streamContent);

                    var response = await client.PostAsync(
                        "https://www.googleapis.com/upload/drive/v3/files?uploadType=multipart&fields=id",
                        content);

                    var responseBody = await response.Content.ReadAsStringAsync();

                    if (response.IsSuccessStatusCode)
                    {
                        var result = JObject.Parse(responseBody);
                        return result["id"]?.ToString() ?? "";
                    }

                    // If token expired, try to refresh and retry once
                    if (response.StatusCode == System.Net.HttpStatusCode.Unauthorized)
                    {
                        token = await RefreshTokenAsync();
                        if (!string.IsNullOrEmpty(token))
                        {
                            client.DefaultRequestHeaders.Authorization =
                                new System.Net.Http.Headers.AuthenticationHeaderValue("Bearer", token);

                            // Need to recreate content since StreamContent was consumed
                            var boundary2 = "----NOMADUploadBoundary" + DateTime.Now.Ticks.ToString("x");
                            var content2 = new MultipartContent("related", boundary2);

                            var metadata2 = new JObject { { "name", filename } };
                            if (!string.IsNullOrEmpty(targetFolderId))
                                metadata2["parents"] = new JArray { targetFolderId };

                            var metadataContent2 = new StringContent(
                                metadata2.ToString(Formatting.None),
                                Encoding.UTF8,
                                "application/json");
                            metadataContent2.Headers.Add("Content-ID", "<metadata>");
                            content2.Add(metadataContent2);

                            var fileStream2 = new FileStream(localPath, FileMode.Open, FileAccess.Read);
                            var streamContent2 = new StreamContent(fileStream2);
                            streamContent2.Headers.Add("Content-Type", GetMimeType(localPath));
                            streamContent2.Headers.Add("Content-ID", "<file-data>");
                            content2.Add(streamContent2);

                            var response2 = await client.PostAsync(
                                "https://www.googleapis.com/upload/drive/v3/files?uploadType=multipart&fields=id",
                                content2);

                            var responseBody2 = await response2.Content.ReadAsStringAsync();

                            if (response2.IsSuccessStatusCode)
                            {
                                var result2 = JObject.Parse(responseBody2);
                                return result2["id"]?.ToString() ?? "";
                            }
                        }
                    }

                    return "";
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Google Drive upload failed: {ex.Message}");
                return "";
            }
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
        /// </summary>
        private async Task<string> RefreshTokenAsync()
        {
            try
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

                    if (response.IsSuccessStatusCode)
                    {
                        var result = JObject.Parse(responseBody);
                        var newAccessToken = result["access_token"]?.ToString();

                        // Update token file with new access token
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
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Token refresh failed: {ex.Message}");
            }

            return null;
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
