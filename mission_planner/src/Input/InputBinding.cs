// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public sealed class InputBinding
    {
        public string Id { get; set; } = "";
        public string Name { get; set; } = "New control";
        public bool Enabled { get; set; } = false;
        public string Device { get; set; } = "";
        public string Source { get; set; } = "Button";
        public int Button { get; set; }
        public int OtherButton { get; set; } = 1;
        public string Axis { get; set; } = "X";
        public bool Invert { get; set; }
        public float Deadzone { get; set; } = 0.08f;
        public string Action { get; set; } = "";
        public string OtherAction { get; set; } = "";
        internal bool Paired => Source == "ButtonPair" || Source == "AxisSwitch3";
        internal bool Continuous => Source == "Axis";
        internal IEnumerable<string> Actions => Paired ? new[] { Action, OtherAction } : new[] { Action };
        internal static readonly string[] Sources = { "Button", "ButtonPair", "Axis", "AxisSwitch2", "AxisSwitch3" };
        internal static readonly string[] Axes = { "X", "Y", "Z", "Rx", "Ry", "Rz", "Slider1", "Slider2" };
        internal static bool Gimbal(string action) => action == "gimbal:pitch" || action == "gimbal:roll";
        internal static bool Split(string action, out string id, out string operation)
        {
            int separator = action?.LastIndexOf(':') ?? -1;
            id = separator > 0 ? action.Substring(0, separator) : "";
            operation = separator > 0 ? action.Substring(separator + 1) : "";
            return id != "" && operation != "" && !Gimbal(action);
        }
        internal bool Authorizes(string id, string operation)
        {
            return Enabled && Actions.Any(action => Split(action, out var target, out var mapped) && target == id &&
                (operation == mapped || operation == "neutral" || operation == "release_input" || operation == "safe" || operation == "stop"));
        }
        internal static List<InputBinding> Parse(JToken token)
        {
            if (!(token is JArray array) || array.Count > 32) { throw Invalid("Inputs must be an array of at most 32 controls."); }
            var schema = JObject.FromObject(new InputBinding());
            var result = new List<InputBinding>();
            var ids = new HashSet<string>(StringComparer.Ordinal);
            var physical = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            var gimbalTargets = new HashSet<string>();
            foreach (var item in array)
            {
                if (!(item is JObject document)) { throw Invalid("Each input control must be an object."); }
                foreach (var field in document.Properties())
                {
                    var expected = schema[field.Name];
                    if (expected == null || (field.Value.Type != expected.Type &&
                        !(expected.Type == JTokenType.Float && field.Value.Type == JTokenType.Integer)))
                    { throw Invalid("Unknown or incorrectly typed input field: " + field.Name); }
                }
                foreach (string name in new[] { "Id", "Name", "Device", "Source", "Action" })
                { if (document[name] == null) { throw Invalid("Input field required: " + name); } }
                var value = document.ToObject<InputBinding>();
                value.Validate();
                if (!ids.Add(value.Id)) { throw Invalid("Input identities must be unique."); }
                if (value.Enabled)
                {
                    if (Gimbal(value.Action) && !gimbalTargets.Add(value.Action)) { throw Invalid("Use one enabled axis per gimbal direction."); }
                    var controls = value.Source == "ButtonPair" ? new[] { "button:" + value.Button, "button:" + value.OtherButton } :
                        new[] { value.Source == "Button" ? "button:" + value.Button : "axis:" + value.Axis };
                    foreach (string control in controls)
                    { if (!physical.Add(value.Device + ":" + control)) { throw Invalid("Enabled controls must use unique physical inputs."); } }
                }
                result.Add(value);
            }
            return result;
        }
        private void Validate()
        {
            if (!Regex.IsMatch(Id ?? "", @"\A[a-zA-Z0-9_.-]{1,64}\z") || !Readable(Name) ||
                !Readable(Device) || !Sources.Contains(Source) || !Axes.Contains(Axis) ||
                Button < 0 || Button > 127 || OtherButton < 0 || OtherButton > 127 ||
                (Source == "ButtonPair" && Button == OtherButton) || float.IsNaN(Deadzone) ||
                float.IsInfinity(Deadzone) || Deadzone < 0 || Deadzone > 0.5f)
            { throw Invalid("Input identity, device, source, indices or deadzone is invalid."); }
            if (!Actions.Any(action => !string.IsNullOrEmpty(action))) { throw Invalid("Choose at least one input action."); }
            foreach (string action in Actions)
            {
                if (string.IsNullOrEmpty(action)) { continue; }
                if (Gimbal(action))
                {
                    if (!Continuous) { throw Invalid("Gimbal movement requires a continuous axis."); }
                    continue;
                }
                if (!Split(action, out var id, out var operation) || !Regex.IsMatch(id, @"\A[a-zA-Z0-9_.-]{1,64}\z") ||
                    !(Continuous ? operation == "position" : new[] { "activate", "negative", "positive", "safe" }.Contains(operation)))
                { throw Invalid("Input actions must select a payload operation or a continuous gimbal axis."); }
            }
        }
        private static bool Readable(string text) => !string.IsNullOrWhiteSpace(text) && text.Length <= 256 && !text.Any(char.IsControl);
        private static JsonSerializationException Invalid(string text) => new JsonSerializationException(text);
    }
}
