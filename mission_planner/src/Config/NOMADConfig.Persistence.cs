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
        internal static NOMADConfig LoadFromPaths(string primary, string backup)
        {
            if (File.Exists(primary))
            {
                return LoadFromFile(primary);
            }
            if (File.Exists(backup))
            {
                return LoadFromFile(backup);
            }
            return new NOMADConfig();
        }

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
            profile.Remove(nameof(CoreClientCredential));
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
            foreach (var property in document.Properties())
            {
                // Json.NET's case-insensitive coercion must not bypass raw-token validation below.
                if (typeof(NOMADConfig).GetProperty(property.Name) == null)
                {
                    throw new JsonSerializationException("Unknown configuration field: " + property.Name);
                }
            }
            ValidateAircraftDocument(document);
            var termination = document["JoystickTerminationButtonIndex"];
            var monitor = document["JoystickKillSwitchEnabled"];
            if ((termination != null && termination.Type != JTokenType.Integer) ||
                (monitor != null && monitor.Type != JTokenType.Boolean))
            {
                throw new JsonSerializationException(
                    "HID termination index must be an integer and its enabled flag must be boolean.");
            }
            if (termination != null) { ValidatePhysicalIndex(termination, "HID termination index"); }
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

        private static void ValidatePhysicalIndex(JToken token, string name)
        {
            if (!int.TryParse(token.ToString(Formatting.None), NumberStyles.Integer, CultureInfo.InvariantCulture,
                out int value) || value < 0 || value > 127)
            { throw new JsonSerializationException(name + " must be an integer between 0 and 127."); }
        }

        internal void Validate()
        {
            ValidateAircraftSettings();
            ValidateInputBindings();
            if (CoreRuntimePort < 1 || CoreRuntimePort > 65535 || RouterLocalPort < 1 || RouterLocalPort > 65535 ||
                ManagementPort < 1 || ManagementPort > 65535)
            {
                throw new JsonSerializationException("Runtime and router ports must be between 1 and 65535.");
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
