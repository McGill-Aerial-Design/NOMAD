// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    internal sealed class PayloadDefinition
    {
        [JsonProperty("id")] public string Id { get; set; }
        [JsonProperty("name")] public string Name { get; set; }
        [JsonProperty("behavior")] public string Behavior { get; set; }
        [JsonProperty("channel")] public int Channel { get; set; }
        [JsonProperty("pwm_min")] public int Minimum { get; set; } = 1000;
        [JsonProperty("pwm_max")] public int Maximum { get; set; } = 2000;
        [JsonProperty("pwm_neutral")] public int Neutral { get; set; } = 1500;
        [JsonProperty("safe_pwm")] public int Safe { get; set; } = 1000;
        [JsonProperty("reversed")] public bool Reversed { get; set; }
        [JsonProperty("pulse_ms")] public int PulseMs { get; set; } = 500;
        [JsonProperty("hold_ms")] public int HoldMs { get; set; } = 1000;
        [JsonProperty("primary_label")] public string PrimaryLabel { get; set; } = "Activate";
        [JsonProperty("secondary_label")] public string SafeLabel { get; set; } = "Safe";
        [JsonProperty("negative_label")] public string NegativeLabel { get; set; } = "Negative";
        [JsonProperty("neutral_label")] public string NeutralLabel { get; set; } = "Stop";
        [JsonProperty("positive_label")] public string PositiveLabel { get; set; } = "Positive";
        [JsonProperty("hazardous")] public bool Hazardous { get; set; } = true;
        [JsonProperty("confirmation_count")] public int Confirmations { get; set; } = 3;
        [JsonProperty("confirmation_window_ms")] public int WindowMs { get; set; } = 3000;
        [JsonProperty("require_neutral")] public bool RequireNeutral { get; set; } = true;
        [JsonIgnore] internal bool Relay => Behavior == "RelayToggle" || Behavior == "RelayPulse";
        [JsonIgnore] internal bool Timed => Behavior == "ServoBidirectional" || Behavior == "RelayPulse";
        [JsonIgnore] internal bool Continuous => Behavior == "ServoPosition" && !Hazardous && Confirmations == 0;
        internal string[] Operations => Behavior == "ServoBidirectional" ? new[] { "negative", "positive" } :
            Behavior == "ServoPosition" ? new[] { "position" } : new[] { "activate" };

        internal static List<PayloadDefinition> Parse(JToken token)
        {
            if (token == null || token.Type == JTokenType.Null) { return new List<PayloadDefinition>(); }
            if (!(token is JArray array) || array.Count > 8) { throw Invalid("Use an array of at most eight payload outputs."); }
            var schema = JObject.FromObject(new PayloadDefinition { Id = "", Name = "", Behavior = "" });
            var result = new List<PayloadDefinition>();
            var ids = new HashSet<string>(StringComparer.Ordinal);
            var outputs = new HashSet<string>();
            foreach (var item in array)
            {
                if (!(item is JObject document)) { throw Invalid("Each payload output must be an object."); }
                foreach (var field in document.Properties())
                {
                    var expected = schema[field.Name];
                    if (expected == null || field.Value.Type != expected.Type)
                    { throw Invalid("Unknown or incorrectly typed payload field: " + field.Name); }
                }
                foreach (string required in new[] { "id", "name", "behavior", "channel", "safe_pwm" })
                { if (document[required] == null) { throw Invalid("Payload field required: " + required); } }
                var definition = document.ToObject<PayloadDefinition>();
                definition.Validate();
                if (!ids.Add(definition.Id) || !outputs.Add((definition.Relay ? "relay:" : "servo:") + definition.Channel))
                { throw Invalid("Payload IDs and physical outputs must be unique."); }
                result.Add(definition);
            }
            return result;
        }

        private void Validate()
        {
            if (Id == null || !Regex.IsMatch(Id, @"\A[a-zA-Z0-9_.-]{1,64}\z") || !Readable(Name) ||
                !new[] { "ServoToggle", "ServoPosition", "ServoBidirectional", "RelayToggle", "RelayPulse" }.Contains(Behavior))
            { throw Invalid("Payload ID, name or behavior is invalid."); }
            if (Channel < (Relay ? 0 : 1) || Channel > (Relay ? 15 : 16) || Minimum < 500 || Maximum > 2500 ||
                Minimum >= Maximum || Neutral < Minimum || Neutral > Maximum || Safe < Minimum || Safe > Maximum ||
                PulseMs < 50 || PulseMs > 1500 || HoldMs < 50 || HoldMs > 1500 ||
                Confirmations < 0 || Confirmations > 3 || WindowMs < 500 || WindowMs > 5000 ||
                (Hazardous && (Confirmations < 2 || !RequireNeutral)))
            { throw Invalid("Payload calibration, timing or confirmation bounds are invalid."); }
            if ((Behavior == "ServoToggle" && Safe != (Reversed ? Maximum : Minimum)) ||
                (Behavior == "ServoBidirectional" && Safe != Neutral))
            { throw Invalid("Payload safe PWM must match the inactive endpoint or bidirectional neutral."); }
            if (new[] { PrimaryLabel, SafeLabel, NegativeLabel, NeutralLabel, PositiveLabel }.Any(label => !Readable(label)))
            { throw Invalid("Payload action labels must be readable text."); }
        }
        private static bool Readable(string text) => !string.IsNullOrWhiteSpace(text) && text.Length <= 80 &&
            !text.Any(character => char.IsControl(character));
        private static JsonSerializationException Invalid(string message) => new JsonSerializationException(message);
    }
}
