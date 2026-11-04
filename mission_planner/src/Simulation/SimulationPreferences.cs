// SPDX-License-Identifier: Apache-2.0
using System;
using System.IO;
using System.Linq;
using System.Text;
using System.Security.Cryptography;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    internal sealed class SimulationPreferences
    {
        public string ToolsDirectory { get; set; } = "";
        public string Distribution { get; set; } = "";
        public string Workspace { get; set; } = "";
        public string PythonExecutable { get; set; } = "python3";
        public string ScenarioPath { get; set; } = "";
        public string WorldPath { get; set; } = "";
        public bool Visible { get; set; } = true;
        public bool AutoConnect { get; set; } = true;
        public int DurationMinutes { get; set; } = 60;
        internal SimulationPreferences Copy() => (SimulationPreferences)MemberwiseClone();

        internal static string StoragePath => Path.Combine(NOMADStorage.PluginsDirectory, "nomad_simulation.json");

        internal static SimulationPreferences Load()
        {
            try { return File.Exists(StoragePath)
                ? JsonConvert.DeserializeObject<SimulationPreferences>(File.ReadAllText(StoragePath)) ?? new SimulationPreferences()
                : new SimulationPreferences(); }
            catch (Exception error) when (error is JsonException || error is IOException || error is UnauthorizedAccessException)
            { return new SimulationPreferences(); }
        }

        internal void Save() => NOMADConfig.WriteAtomically(StoragePath, JsonConvert.SerializeObject(this, Formatting.Indented));

        internal void Validate()
        {
            foreach (string value in new[] { ToolsDirectory, Distribution, Workspace, PythonExecutable, ScenarioPath, WorldPath })
            { if (value == null || value.Length > 4096 || value.Any(char.IsControl))
                { throw new ArgumentException("Review the simulation paths and WSL distribution."); } }
            if (!Directory.Exists(ToolsDirectory) || !File.Exists(Path.Combine(ToolsDirectory, "interactive.py")))
            { throw new ArgumentException("Choose the simulation tools folder containing interactive.py."); }
            if (!File.Exists(ScenarioPath)) { throw new ArgumentException("Choose a scenario configuration."); }
            if (WorldPath.Length > 0 && !File.Exists(WorldPath)) { throw new ArgumentException("World file is unavailable."); }
            if (Workspace.Length > 0 && !Workspace.StartsWith("/", StringComparison.Ordinal))
            { throw new ArgumentException("The simulator workspace must be an absolute Linux path, or blank for its default."); }
            if (DurationMinutes < 1 || DurationMinutes > 1440) { throw new ArgumentException("Choose 1..1440 minutes."); }
            if (string.IsNullOrWhiteSpace(PythonExecutable)) { throw new ArgumentException("Choose a Python command or Linux path."); }
        }

        internal static string Argument(string value)
        {
            if (value == null || value.Any(char.IsControl)) { throw new ArgumentException("Invalid launch argument."); }
            // WSL's distribution parser retains unnecessary quotes around a simple name.
            if (value.Length > 0 && !value.Any(c => char.IsWhiteSpace(c) || c == '"')) { return value; }
            var text = new StringBuilder("\"");
            int slashes = 0;
            foreach (char character in value)
            {
                if (character == '\\') { slashes++; continue; }
                text.Append('\\', character == '"' ? slashes * 2 + 1 : slashes);
                text.Append(character); slashes = 0;
            }
            text.Append('\\', slashes * 2); text.Append('"');
            return text.ToString();
        }
    }

    internal sealed class SimulationScenario
    {
        internal string Path { get; private set; }
        internal string Name { get; private set; }
        internal string Backend { get; private set; }
        internal string Hash { get; private set; }
        internal int SystemId { get; private set; }
        internal static SimulationScenario Load(string path)
        {
            var file = new FileInfo(path);
            if (!file.Exists || file.Length > 65536) { throw new ArgumentException("Scenario must be a local JSON file under 64 KiB."); }
            byte[] bytes = File.ReadAllBytes(path);
            string text;
            using (var reader = new StreamReader(new MemoryStream(bytes), Encoding.UTF8, true)) { text = reader.ReadToEnd(); }
            var data = JObject.Parse(text);
            string name = (string)data["name"], backend = (string)data["backend"];
            if ((int?)data["schema"] != 1 || string.IsNullOrWhiteSpace(name) || name.Length > 120 || name.Any(char.IsControl) ||
                (backend != "sitl" && backend != "gazebo" && backend != "last-letter"))
            { throw new ArgumentException("Scenario needs schema 1, a name and a supported backend."); }
            int system = (int?)data["system_id"] ?? 201;
            if (system < 1 || system > 254) { throw new ArgumentException("Choose a simulator system ID from 1 to 254."); }
            using (var hash = SHA256.Create())
            { return new SimulationScenario { Path = System.IO.Path.GetFullPath(path), Name = name, Backend = backend,
                SystemId = system, Hash = BitConverter.ToString(hash.ComputeHash(bytes)).Replace("-", "") }; }
        }
        public override string ToString() => Name;
    }
}
