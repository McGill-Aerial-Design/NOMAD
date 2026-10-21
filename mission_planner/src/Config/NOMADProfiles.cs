// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public sealed class AircraftProfile
    {
        public string Id { get; set; } = "";
        public NOMADConfig Config { get; set; } = new NOMADConfig();
        [JsonIgnore] public string Name => Config?.AircraftName ?? "";
    }

    public sealed class NOMADProfiles
    {
        private static readonly string DirectoryPath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "Mission Planner", "plugins");
        private string _path = Path.Combine(DirectoryPath, "nomad_aircraft_profiles.json");

        public List<AircraftProfile> Aircraft { get; set; } = new List<AircraftProfile>();
        public string SelectedId { get; set; } = "";
        [JsonIgnore] public AircraftProfile Selected => Aircraft.Single(profile =>
            string.Equals(profile.Id, SelectedId, StringComparison.OrdinalIgnoreCase));

        public NOMADProfiles() { }

        public NOMADProfiles(NOMADConfig current)
        {
            var config = (current ?? new NOMADConfig()).Clone();
            if (string.IsNullOrWhiteSpace(config.AircraftName))
            {
                config.AircraftName = "Current aircraft";
            }
            Add(config);
        }

        public static NOMADProfiles Load()
        {
            var path = Path.Combine(DirectoryPath, "nomad_aircraft_profiles.json");
            var legacy = Path.Combine(DirectoryPath, "nomad_config.json");
            return LoadFromPaths(path, path + ".bak", legacy, legacy + ".bak");
        }

        internal static NOMADProfiles LoadFromPaths(string primary, string backup, string legacy, string legacyBackup)
        {
            NOMADProfiles profiles;
            if (File.Exists(primary))
            {
                profiles = Deserialize(File.ReadAllText(primary));
            }
            else if (File.Exists(backup))
            {
                profiles = Deserialize(File.ReadAllText(backup));
            }
            else
            {
                profiles = new NOMADProfiles(NOMADConfig.LoadFromPaths(legacy, legacyBackup));
            }
            profiles._path = primary;
            return profiles;
        }

        public NOMADProfiles Clone()
        {
            var clone = Deserialize(JsonConvert.SerializeObject(this));
            clone._path = _path;
            return clone;
        }

        public AircraftProfile Add(NOMADConfig config)
        {
            var copy = config.Clone();
            if (string.IsNullOrWhiteSpace(copy.AircraftName) || copy.AircraftName != copy.AircraftName.Trim())
            {
                throw new JsonSerializationException("An aircraft profile name is required.");
            }
            if (Aircraft.Any(profile =>
                string.Equals(profile.Name, copy.AircraftName, StringComparison.OrdinalIgnoreCase)))
            {
                throw new JsonSerializationException("Aircraft profile names must be unique.");
            }
            var added = new AircraftProfile { Id = Guid.NewGuid().ToString("N"), Config = copy };
            Aircraft.Add(added);
            SelectedId = added.Id;
            return added;
        }

        public void RemoveSelected()
        {
            if (Aircraft.Count <= 1)
            {
                throw new InvalidOperationException("Keep at least one aircraft profile.");
            }
            Aircraft.Remove(Selected);
            SelectedId = Aircraft[0].Id;
        }

        internal void Validate()
        {
            if (Aircraft == null || Aircraft.Count == 0)
            {
                throw new JsonSerializationException("At least one aircraft profile is required.");
            }
            var ids = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            var names = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var profile in Aircraft)
            {
                if (profile == null || !Guid.TryParseExact(profile.Id, "N", out _) || !ids.Add(profile.Id))
                {
                    throw new JsonSerializationException("Aircraft profile IDs must be unique valid IDs.");
                }
                if (profile.Config == null || string.IsNullOrWhiteSpace(profile.Name) ||
                    profile.Name != profile.Name.Trim() || !names.Add(profile.Name))
                {
                    throw new JsonSerializationException("Profile names must be nonempty, trimmed and unique.");
                }
                profile.Config.Validate();
            }
            if (!ids.Contains(SelectedId))
            {
                throw new JsonSerializationException("The selected aircraft profile does not exist.");
            }
        }

        private static NOMADProfiles Deserialize(string json)
        {
            var document = NOMADConfig.ReadToken(json) as JObject ??
                throw new JsonSerializationException("Expected an aircraft profile collection.");
            if (document.Properties().Any(property =>
                    property.Name != nameof(Aircraft) && property.Name != nameof(SelectedId)) ||
                document[nameof(SelectedId)]?.Type != JTokenType.String ||
                !(document[nameof(Aircraft)] is JArray aircraft))
            {
                throw new JsonSerializationException("Profiles require an aircraft array and selected profile ID.");
            }
            foreach (var item in aircraft)
            {
                if (!(item is JObject entry) || entry[nameof(AircraftProfile.Id)]?.Type != JTokenType.String ||
                    entry.Properties().Any(property => property.Name != nameof(AircraftProfile.Id) &&
                        property.Name != nameof(AircraftProfile.Config)) ||
                    !(entry[nameof(AircraftProfile.Config)] is JObject config))
                {
                    throw new JsonSerializationException("An aircraft profile requires an ID and config object.");
                }
                NOMADConfig.ValidateInputDocument(config);
            }
            var profiles = document.ToObject<NOMADProfiles>(JsonSerializer.Create(new JsonSerializerSettings
            {
                MissingMemberHandling = MissingMemberHandling.Error,
            }));
            if (profiles == null)
            {
                throw new JsonSerializationException("Expected aircraft profiles.");
            }
            profiles.Validate();
            return profiles;
        }

        internal bool CheckForConnection(NOMADProfiles active, bool connectionOpen)
        {
            if (PayloadController.HasPendingWork)
            { throw new InvalidOperationException("Wait for the payload command/repeat to finish, or send Safe, before saving settings."); }
            bool connectionChanged = !string.Equals(SelectedId, active.SelectedId, StringComparison.OrdinalIgnoreCase) ||
                !Selected.Config.HasSameRouterSettings(active.Selected.Config);
            bool outputChanged = !Selected.Config.HasSameOutputSettings(active.Selected.Config);
            if ((connectionChanged || outputChanged) && connectionOpen)
            {
                throw new InvalidOperationException("Disconnect Mission Planner before changing aircraft, physical links or output/input calibration.");
            }
            return connectionChanged;
        }

        internal bool SaveForConnection(NOMADProfiles active, bool connectionOpen)
        {
            bool connectionChanged = CheckForConnection(active, connectionOpen);
            Save();
            return connectionChanged;
        }

        public void Save()
        {
            Validate();
            NOMADConfig.WriteAtomically(_path, JsonConvert.SerializeObject(this, Formatting.Indented));
        }
    }
}
