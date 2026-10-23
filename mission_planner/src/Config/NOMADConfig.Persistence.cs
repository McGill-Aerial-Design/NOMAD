// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.IO;
using System.Globalization;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADConfig
    {
        /// <summary>Load and validate a configuration profile from an arbitrary JSON file.</summary>
        public static NOMADConfig LoadFromFile(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
                throw new ArgumentException("A configuration file path is required.", nameof(path));

            var json = File.ReadAllText(path);
            if (string.IsNullOrWhiteSpace(json))
                throw new InvalidDataException("The configuration file is empty.");

            return Deserialize(json);
        }

        internal static void WriteAtomically(string path, string json)
        {
            var temporary = path + ".tmp";
            try
            {
                var dir = Path.GetDirectoryName(path);
                if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                {
                    Directory.CreateDirectory(dir);
                }
                File.WriteAllText(temporary, json);

                if (File.Exists(path))
                {
                    // Atomic rename + backup. Replace() requires the destination
                    // to exist; otherwise fall through to a plain Move().
                    File.Replace(temporary, path, path + ".bak", ignoreMetadataErrors: true);
                }
                else
                {
                    File.Move(temporary, path);
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Failed to save config - {ex.Message}");
                try
                {
                    File.Delete(temporary);
                }
                catch
                {
                    // Cleanup must not hide the original save failure.
                }
                throw new IOException("Configuration was not saved.", ex);
            }
        }

        /// <summary>Export this configuration as a portable JSON profile.</summary>
        public void ExportToFile(string path)
        {
            Validate();
            if (string.IsNullOrWhiteSpace(path))
                throw new ArgumentException("A configuration file path is required.", nameof(path));

            var directory = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
                Directory.CreateDirectory(directory);

            var profile = JObject.FromObject(this);
            WriteAtomically(path, profile.ToString(Formatting.Indented));
        }

        private static NOMADConfig Deserialize(string json)
        {
            var document = ReadToken(json) as JObject ??
                throw new JsonSerializationException("Expected a NOMAD configuration object.");
            ValidateInputDocument(document);
            var serializer = JsonSerializer.Create(new JsonSerializerSettings
            {
                MissingMemberHandling = MissingMemberHandling.Error,
            });
            var config = document.ToObject<NOMADConfig>(serializer);
            if (config == null)
            {
                throw new JsonSerializationException("Expected a NOMAD configuration object.");
            }
            config.Validate();
            return config;
        }

        internal static JToken ReadToken(string json)
        {
            using (var reader = new JsonTextReader(new StringReader(json)))
            {
                reader.DateParseHandling = DateParseHandling.None;
                var document = JToken.Load(reader, new JsonLoadSettings
                { DuplicatePropertyNameHandling = DuplicatePropertyNameHandling.Error });
                while (reader.Read())
                {
                    if (reader.TokenType != JsonToken.Comment)
                    {
                        throw new JsonSerializationException("Unexpected content after configuration object.");
                    }
                }
                return document;
            }
        }

        internal static void ValidateInputDocument(JObject document)
        {
            ValidateVideoDocument(document);
            foreach (var property in document.Properties())
            {
                // Json.NET's case-insensitive coercion must not bypass raw-token validation below.
                if (typeof(NOMADConfig).GetProperty(property.Name)?.CanWrite != true)
                {
                    throw new JsonSerializationException("Unknown configuration field: " + property.Name);
                }
            }
            ValidateAircraftDocument(document);
            if (document[nameof(RouterLinks)] != null) { ValidateRouterLinks(document[nameof(RouterLinks)]); }
            if (document[nameof(RouterPreferredLink)] != null && document[nameof(RouterPreferredLink)].Type != JTokenType.String)
            { throw new JsonSerializationException("RouterPreferredLink must be text."); }
            if (document[nameof(RouterClientEnabled)] != null && document[nameof(RouterClientEnabled)].Type != JTokenType.Boolean)
            { throw new JsonSerializationException("RouterClientEnabled must be boolean."); }
            if (document[nameof(RouterLocalPort)] != null && document[nameof(RouterLocalPort)].Type != JTokenType.Integer)
            { throw new JsonSerializationException("RouterLocalPort must be an integer."); }
            if (document["JoystickButtonIndices"] == null) { return; }
            if (!(document["JoystickButtonIndices"] is JArray indices))
            { throw new JsonSerializationException("HID button indices must be an integer array."); }
            foreach (var index in indices)
            {
                if (index.Type != JTokenType.Integer)
                { throw new JsonSerializationException("HID button indices must be integers."); }
                ValidatePhysicalIndex(index, "HID button indices");
            }
        }

        private static void ValidateRouterLinks(JToken token)
        {
            if (!(token is JArray links)) { throw new JsonSerializationException("RouterLinks must be an array."); }
            foreach (var item in links)
            {
                if (!(item is JObject link)) { throw new JsonSerializationException("Each router link must be an object."); }
                foreach (var field in link.Properties())
                {
                    var property = typeof(LinkConfig).GetProperty(field.Name);
                    var type = property?.PropertyType;
                    var value = field.Value.Type;
                    bool valid = type == typeof(string) ? value == JTokenType.String :
                        type == typeof(int) ? value == JTokenType.Integer :
                        type == typeof(bool) ? value == JTokenType.Boolean :
                        type == typeof(double) && (value == JTokenType.Integer || value == JTokenType.Float);
                    if (!valid) { throw new JsonSerializationException("Invalid router link field: " + field.Name); }
                }
            }
        }

        private static void ValidatePhysicalIndex(JToken token, string name)
        {
            if (!int.TryParse(token.ToString(Formatting.None), NumberStyles.Integer, CultureInfo.InvariantCulture,
                out int value) || value < 0 || value > 127)
            { throw new JsonSerializationException(name + " must be an integer between 0 and 127."); }
        }

        internal bool HasSameRouterSettings(NOMADConfig other)
        {
            return other != null && RouterClientEnabled == other.RouterClientEnabled &&
                RouterLocalPort == other.RouterLocalPort && RouterPreferredLink == other.RouterPreferredLink &&
                JToken.DeepEquals(JArray.FromObject(RouterLinks), JArray.FromObject(other.RouterLinks));
        }

        internal void Validate()
        {
            ValidateVideoSettings();
            ValidateAircraftSettings();
            ValidateInputBindings();
            if (RouterLocalPort < 1 || RouterLocalPort > 65535)
            {
                throw new JsonSerializationException("Router ports must be between 1 and 65535.");
            }
            if (RouterLinks == null) { throw new JsonSerializationException("RouterLinks cannot be null."); }
            if (RouterLinks.Count == 0 && !string.IsNullOrEmpty(RouterPreferredLink))
            { throw new JsonSerializationException("Preferred link requires configured RouterLinks."); }
            if (RouterLinks.Count > 0)
            {
                using (var router = new GroundLinkRouter(MAVLinkConnectionManager.BuildRouterConfig(
                    new MAVLinkConnectionManager.ConnectionConfig { RouterLocalPort = RouterLocalPort,
                        Links = RouterLinks, PreferredLink = RouterPreferredLink }))) { }
            }

        }

        internal void ValidateInputBindings()
        {
            string error = GetInputMappingError();
            if (error != null) { throw new JsonSerializationException(error); }
        }

        /// <summary>
        /// Create a copy of the configuration.
        /// </summary>
        public NOMADConfig Clone()
        {
            var json = JsonConvert.SerializeObject(this);
            return Deserialize(json);
        }

    }
}
