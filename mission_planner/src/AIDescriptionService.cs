// ============================================================
// AI Description Service
// ============================================================
// Provides AI-powered image description generation for Task 1.
// Supports multiple providers: OpenRouter, Gemini, Ollama.
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
    /// AI provider types.
    /// </summary>
    public enum AIProvider
    {
        OpenRouter,
        Gemini,
        Ollama
    }

    /// <summary>
    /// Result from AI description generation.
    /// </summary>
    public class AIDescriptionResult
    {
        public bool Success { get; set; }
        public string Description { get; set; }
        public string ErrorMessage { get; set; }
        public AIProvider Provider { get; set; }
        public string Model { get; set; }
        public DateTime GeneratedAt { get; set; }
    }

    /// <summary>
    /// Service for generating AI descriptions of aerial images.
    /// </summary>
    public class AIDescriptionService
    {
        private static readonly HttpClient _httpClient = new HttpClient
        {
            Timeout = TimeSpan.FromSeconds(60)
        };

        private readonly NOMADConfig _config;

        public AIDescriptionService(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
        }

        /// <summary>
        /// Generate description for an image with metadata.
        /// </summary>
        public async Task<AIDescriptionResult> GenerateDescriptionAsync(
            string imagePath,
            SnapshotMetadata metadata)
        {
            try
            {
                switch (_config.AiProvider)
                {
                    case AIProvider.OpenRouter:
                        return await GenerateWithOpenRouterAsync(imagePath, metadata);

                    case AIProvider.Gemini:
                        return await GenerateWithGeminiAsync(imagePath, metadata);

                    case AIProvider.Ollama:
                        return await GenerateWithOllamaAsync(imagePath, metadata);

                    default:
                        return new AIDescriptionResult
                        {
                            Success = false,
                            ErrorMessage = $"Unsupported AI provider: {_config.AiProvider}"
                        };
                }
            }
            catch (Exception ex)
            {
                return new AIDescriptionResult
                {
                    Success = false,
                    ErrorMessage = $"AI description failed: {ex.Message}"
                };
            }
        }

        /// <summary>
        /// Generate description using OpenRouter API.
        /// </summary>
        private async Task<AIDescriptionResult> GenerateWithOpenRouterAsync(
            string imagePath,
            SnapshotMetadata metadata)
        {
            if (string.IsNullOrEmpty(_config.OpenRouterApiKey))
            {
                return new AIDescriptionResult
                {
                    Success = false,
                    ErrorMessage = "OpenRouter API key not configured"
                };
            }

            // Read and encode image
            byte[] imageBytes = File.ReadAllBytes(imagePath);
            string base64Image = Convert.ToBase64String(imageBytes);

            // Build prompt
            string prompt = BuildPrompt(metadata);

            // Build request
            var requestBody = new
            {
                model = _config.AiModel,
                messages = new[]
                {
                    new
                    {
                        role = "user",
                        content = new object[]
                        {
                            new { type = "text", text = prompt },
                            new
                            {
                                type = "image_url",
                                image_url = new
                                {
                                    url = $"data:image/jpeg;base64,{base64Image}"
                                }
                            }
                        }
                    }
                },
                max_tokens = 1000
            };

            var request = new HttpRequestMessage(HttpMethod.Post, "https://openrouter.ai/api/v1/chat/completions")
            {
                Content = new StringContent(
                    JsonConvert.SerializeObject(requestBody),
                    Encoding.UTF8,
                    "application/json")
            };

            request.Headers.Add("Authorization", $"Bearer {_config.OpenRouterApiKey}");
            request.Headers.Add("HTTP-Referer", "https://github.com/McGill-Aerial-Design/NOMAD");
            request.Headers.Add("X-Title", "NOMAD Task 1 Image Analysis");

            // Send request with retry
            for (int attempt = 0; attempt < 3; attempt++)
            {
                try
                {
                    var response = await _httpClient.SendAsync(request);

                    if (response.IsSuccessStatusCode)
                    {
                        var responseJson = await response.Content.ReadAsStringAsync();
                        var result = JObject.Parse(responseJson);
                        var description = result["choices"]?[0]?["message"]?["content"]?.ToString();

                        if (!string.IsNullOrEmpty(description))
                        {
                            return new AIDescriptionResult
                            {
                                Success = true,
                                Description = description,
                                Provider = AIProvider.OpenRouter,
                                Model = _config.AiModel,
                                GeneratedAt = DateTime.UtcNow
                            };
                        }
                    }
                    else if (response.StatusCode == System.Net.HttpStatusCode.Unauthorized)
                    {
                        return new AIDescriptionResult
                        {
                            Success = false,
                            ErrorMessage = "Invalid OpenRouter API key"
                        };
                    }
                    else if (response.StatusCode == (System.Net.HttpStatusCode)429)
                    {
                        // Rate limited - retry with backoff
                        if (attempt < 2)
                        {
                            await Task.Delay((int)Math.Pow(2, attempt) * 1000);
                            continue;
                        }
                        return new AIDescriptionResult
                        {
                            Success = false,
                            ErrorMessage = "Rate limit exceeded"
                        };
                    }
                }
                catch (HttpRequestException ex)
                {
                    if (attempt < 2)
                    {
                        await Task.Delay((int)Math.Pow(2, attempt) * 1000);
                        continue;
                    }
                    return new AIDescriptionResult
                    {
                        Success = false,
                        ErrorMessage = $"Network error: {ex.Message}"
                    };
                }
            }

            return new AIDescriptionResult
            {
                Success = false,
                ErrorMessage = "Max retries exceeded"
            };
        }

        /// <summary>
        /// Generate description using Google Gemini API.
        /// </summary>
        private async Task<AIDescriptionResult> GenerateWithGeminiAsync(
            string imagePath,
            SnapshotMetadata metadata)
        {
            if (string.IsNullOrEmpty(_config.GeminiApiKey))
            {
                return new AIDescriptionResult
                {
                    Success = false,
                    ErrorMessage = "Gemini API key not configured"
                };
            }

            // Read and encode image
            byte[] imageBytes = File.ReadAllBytes(imagePath);
            string base64Image = Convert.ToBase64String(imageBytes);

            // Build prompt
            string prompt = BuildPrompt(metadata);

            // Build request (Gemini uses slightly different format)
            var requestBody = new
            {
                contents = new[]
                {
                    new
                    {
                        parts = new object[]
                        {
                            new { text = prompt },
                            new
                            {
                                inline_data = new
                                {
                                    mime_type = "image/jpeg",
                                    data = base64Image
                                }
                            }
                        }
                    }
                }
            };

            string url = $"https://generativelanguage.googleapis.com/v1/models/{_config.AiModel}:generateContent?key={_config.GeminiApiKey}";

            var request = new HttpRequestMessage(HttpMethod.Post, url)
            {
                Content = new StringContent(
                    JsonConvert.SerializeObject(requestBody),
                    Encoding.UTF8,
                    "application/json")
            };

            try
            {
                var response = await _httpClient.SendAsync(request);

                if (response.IsSuccessStatusCode)
                {
                    var responseJson = await response.Content.ReadAsStringAsync();
                    var result = JObject.Parse(responseJson);
                    var description = result["candidates"]?[0]?["content"]?["parts"]?[0]?["text"]?.ToString();

                    if (!string.IsNullOrEmpty(description))
                    {
                        return new AIDescriptionResult
                        {
                            Success = true,
                            Description = description,
                            Provider = AIProvider.Gemini,
                            Model = _config.AiModel,
                            GeneratedAt = DateTime.UtcNow
                        };
                    }
                }
                else if (response.StatusCode == System.Net.HttpStatusCode.Unauthorized)
                {
                    return new AIDescriptionResult
                    {
                        Success = false,
                        ErrorMessage = "Invalid Gemini API key"
                    };
                }
            }
            catch (Exception ex)
            {
                return new AIDescriptionResult
                {
                    Success = false,
                    ErrorMessage = $"Gemini API error: {ex.Message}"
                };
            }

            return new AIDescriptionResult
            {
                Success = false,
                ErrorMessage = "Failed to generate description with Gemini"
            };
        }

        /// <summary>
        /// Generate description using local Ollama instance.
        /// </summary>
        private async Task<AIDescriptionResult> GenerateWithOllamaAsync(
            string imagePath,
            SnapshotMetadata metadata)
        {
            // Read and encode image
            byte[] imageBytes = File.ReadAllBytes(imagePath);
            string base64Image = Convert.ToBase64String(imageBytes);

            // Build prompt
            string prompt = BuildPrompt(metadata);

            // Build request
            var requestBody = new
            {
                model = _config.AiModel,
                prompt = prompt,
                images = new[] { base64Image },
                stream = false
            };

            var request = new HttpRequestMessage(HttpMethod.Post, $"{_config.OllamaHost}/api/generate")
            {
                Content = new StringContent(
                    JsonConvert.SerializeObject(requestBody),
                    Encoding.UTF8,
                    "application/json")
            };

            try
            {
                var response = await _httpClient.SendAsync(request);

                if (response.IsSuccessStatusCode)
                {
                    var responseJson = await response.Content.ReadAsStringAsync();
                    var result = JObject.Parse(responseJson);
                    var description = result["response"]?.ToString();

                    if (!string.IsNullOrEmpty(description))
                    {
                        return new AIDescriptionResult
                        {
                            Success = true,
                            Description = description,
                            Provider = AIProvider.Ollama,
                            Model = _config.AiModel,
                            GeneratedAt = DateTime.UtcNow
                        };
                    }
                }
            }
            catch (HttpRequestException)
            {
                return new AIDescriptionResult
                {
                    Success = false,
                    ErrorMessage = $"Cannot connect to Ollama at {_config.OllamaHost}. Is Ollama running?"
                };
            }
            catch (Exception ex)
            {
                return new AIDescriptionResult
                {
                    Success = false,
                    ErrorMessage = $"Ollama error: {ex.Message}"
                };
            }

            return new AIDescriptionResult
            {
                Success = false,
                ErrorMessage = "Failed to generate description with Ollama"
            };
        }

        /// <summary>
        /// Build prompt for AI image analysis.
        /// </summary>
        private string BuildPrompt(SnapshotMetadata metadata)
        {
            var sb = new StringBuilder();

            sb.AppendLine("You are an AI assistant analyzing aerial reconnaissance photos for a drone competition.");
            sb.AppendLine();
            sb.AppendLine("MISSION CONTEXT:");
            sb.AppendLine("- Task: Outdoor reconnaissance and localization");
            sb.AppendLine("- Goal: Identify landmarks and provide relative positioning to target building");
            sb.AppendLine("- Competition: AEAC 2026");
            sb.AppendLine();

            if (metadata?.Position != null)
            {
                sb.AppendLine("CAMERA METADATA:");
                sb.AppendLine($"- GPS Position: {metadata.Position.Lat:F6}°, {metadata.Position.Lon:F6}°");
                sb.AppendLine($"- Altitude: {metadata.Position.Alt:F1}m MSL");

                if (metadata.HeadingDeg.HasValue)
                    sb.AppendLine($"- Aircraft Heading: {metadata.HeadingDeg:F1}° (0=North, 90=East, 180=South, 270=West)");

                if (metadata.PitchDeg.HasValue)
                    sb.AppendLine($"- Aircraft Pitch: {metadata.PitchDeg:F1}°");

                if (metadata.RollDeg.HasValue)
                    sb.AppendLine($"- Aircraft Roll: {metadata.RollDeg:F1}°");

                if (metadata.GimbalPitchDeg.HasValue)
                    sb.AppendLine($"- Gimbal Pitch: {metadata.GimbalPitchDeg:F1}°");

                if (!string.IsNullOrEmpty(metadata.BuildingLocation))
                    sb.AppendLine($"- Target Building: {metadata.BuildingLocation}");

                sb.AppendLine();
            }

            sb.AppendLine("ANALYSIS REQUIRED:");
            sb.AppendLine();
            sb.AppendLine("1. SCENE DESCRIPTION:");
            sb.AppendLine("   - Describe visible landmarks, terrain, structures");
            sb.AppendLine("   - Note weather conditions and visibility");
            sb.AppendLine("   - Identify any text, signs, or unique features");
            sb.AppendLine();
            sb.AppendLine("2. LOCALIZATION FEATURES:");
            sb.AppendLine("   - List unique identifiers (building names, addresses, street signs)");
            sb.AppendLine("   - Notable architectural features");
            sb.AppendLine("   - Permanent landmarks (water towers, cell towers, monuments)");
            sb.AppendLine();
            sb.AppendLine("3. RELATIVE POSITION TO BUILDING:");
            sb.AppendLine("   - Estimate distance and direction to target building");
            sb.AppendLine("   - Note line-of-sight obstructions");
            sb.AppendLine("   - Suggest optimal viewing angles");
            sb.AppendLine();
            sb.AppendLine("4. COMPETITION RELEVANCE:");
            sb.AppendLine("   - Assess image quality and suitability for evidence submission");
            sb.AppendLine("   - Note any potential target areas or points of interest");
            sb.AppendLine("   - Suggest improvements for future captures");
            sb.AppendLine();
            sb.AppendLine("Be precise, factual, and concise. Focus on information useful for autonomous navigation and competition scoring.");

            return sb.ToString();
        }
    }
}
