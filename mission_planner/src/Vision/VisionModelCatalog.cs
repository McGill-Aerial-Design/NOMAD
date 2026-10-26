// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    internal sealed class VisionModelChoice
    {
        internal string Path { get; }
        internal string Name { get; }
        internal VisionModelChoice(string path, string name) { Path = path; Name = name; }
        public override string ToString() => Name;
    }

    // Profiles are shared by groundstation users of the same MP data directory; weights stay in place.
    internal sealed class VisionModelCatalog
    {
        internal string DirectoryPath { get; }
        internal VisionModelCatalog(string directory = null)
        { DirectoryPath = Path.GetFullPath(directory ?? Path.Combine(NOMADStorage.PluginsDirectory, "NOMAD", "models")); }

        internal List<VisionModelChoice> Choices(string current = "")
        {
            var choices = new List<VisionModelChoice>
            { new VisionModelChoice("", "Built-in YOLOX Nano vehicles") };
            if (Directory.Exists(DirectoryPath))
            {
                string[] files = Directory.EnumerateFiles(DirectoryPath, "*.json").Take(257).ToArray();
                if (files.Length > 256) { throw new FormatException("The model catalog supports at most 256 profiles."); }
                foreach (string path in files.OrderBy(path => path))
                {
                    try { choices.Add(new VisionModelChoice(path, VisionModelProfile.Load(path).Name)); }
                    catch (Exception error) when (error is IOException || error is UnauthorizedAccessException ||
                        error is JsonException || error is FormatException || error is ArgumentException)
                    { choices.Add(new VisionModelChoice(path, "Unavailable profile: " + Path.GetFileNameWithoutExtension(path))); }
                }
            }
            if (!string.IsNullOrEmpty(current) && !choices.Any(item => SamePath(item.Path, current)))
            {
                string name;
                try { name = VisionModelProfile.Load(current).Name + " (external)"; }
                catch (Exception error) when (error is IOException || error is UnauthorizedAccessException ||
                    error is JsonException || error is FormatException || error is ArgumentException)
                { name = "Current external profile (unavailable)"; }
                choices.Add(new VisionModelChoice(current, name));
            }
            return choices;
        }

        internal List<VisionModelChoice> SelectionChoices(string current, out string error)
        {
            try { error = null; return Choices(current); }
            catch (Exception failure) when (failure is IOException || failure is UnauthorizedAccessException ||
                failure is JsonException || failure is FormatException || failure is ArgumentException)
            {
                error = "Model catalog unavailable; current model selection is preserved. Review Manage models.";
                var choices = new List<VisionModelChoice>
                { new VisionModelChoice("", "Built-in YOLOX Nano vehicles (catalog unavailable)") };
                if (!string.IsNullOrEmpty(current))
                { choices.Add(new VisionModelChoice(current, "Current profile (catalog unavailable)")); }
                return choices;
            }
        }

        internal string Save(VisionModelProfile profile, string path = null)
        {
            profile.Validate();
            if (!Path.IsPathRooted(profile.Model) || Path.GetPathRoot(profile.Model).Length < 3)
            { throw new FormatException("Choose an absolute ONNX model path."); }
            profile.Model = Path.GetFullPath(profile.Model);
            string target = path ?? Path.Combine(DirectoryPath, Guid.NewGuid().ToString("N") + ".json");
            if (!SamePath(Path.GetDirectoryName(Path.GetFullPath(target)), Path.GetFullPath(DirectoryPath)))
            { throw new InvalidOperationException("Only managed model profiles can be edited."); }
            var choices = Choices();
            if (!File.Exists(target) && choices.Count > 256)
            { throw new FormatException("The model catalog supports at most 256 profiles."); }
            foreach (var choice in choices.Where(item => item.Path != "" && !SamePath(item.Path, target)))
            {
                VisionModelProfile other;
                try { other = VisionModelProfile.Load(choice.Path); }
                catch (Exception error) when (error is IOException || error is UnauthorizedAccessException ||
                    error is JsonException || error is FormatException || error is ArgumentException) { continue; }
                if (string.Equals(other.Name, profile.Name, StringComparison.OrdinalIgnoreCase))
                { throw new FormatException("Model profile names must be unique."); }
            }
            CheckWritePath(target);
            Directory.CreateDirectory(DirectoryPath);
            NOMADConfig.WriteAtomically(target, JsonConvert.SerializeObject(profile, Formatting.Indented));
            return target;
        }

        internal void Remove(string path)
        {
            if (string.IsNullOrEmpty(path) || !SamePath(Path.GetDirectoryName(Path.GetFullPath(path)),
                Path.GetFullPath(DirectoryPath)))
            { throw new InvalidOperationException("Only managed model profiles can be removed."); }
            CheckWritePath(path);
            File.Delete(path);
            File.Delete(path + ".bak");
        }

        private void CheckWritePath(string target)
        {
            for (var directory = new DirectoryInfo(DirectoryPath); directory != null; directory = directory.Parent)
            {
                if (IsReparsePath(directory.FullName))
                { throw new IOException("Managed model profiles cannot be written through a reparse directory."); }
            }
            foreach (string path in new[] { target, target + ".tmp", target + ".bak" })
            {
                if (IsReparsePath(path))
                { throw new IOException("Managed model profiles cannot replace a reparse file."); }
            }
        }

        private static bool IsReparsePath(string path)
        {
            try { return (File.GetAttributes(path) & FileAttributes.ReparsePoint) != 0; }
            catch (FileNotFoundException) { return false; }
            catch (DirectoryNotFoundException) { return false; }
        }

        internal static bool SamePath(string left, string right) =>
            string.Equals(left, right, StringComparison.OrdinalIgnoreCase);
    }
}
