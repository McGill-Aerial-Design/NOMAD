// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADConfig
    {
        private static void ValidateVideoDocument(JObject document)
        {
            if (document[nameof(SelectedVideoInputName)] != null &&
                document[nameof(SelectedVideoInputName)].Type != JTokenType.String)
            { throw new JsonSerializationException("Selected video input name must be text."); }
            var token = document[nameof(VideoInputs)];
            if (token == null) { return; }
            if (!(token is JArray inputs)) { throw new JsonSerializationException("VideoInputs must be an array."); }
            foreach (var item in inputs)
            {
                if (!(item is JObject input)) { throw new JsonSerializationException("Video inputs must be objects."); }
                foreach (var field in input.Properties())
                {
                    if (field.Name == nameof(VideoInputProfile.ServoGimbalAxes))
                    { ServoGimbalAxis.Parse(field.Value); continue; }
                    var type = typeof(VideoInputProfile).GetProperty(field.Name)?.PropertyType;
                    bool valid = type == typeof(string) ? field.Value.Type == JTokenType.String :
                        type == typeof(int) ? field.Value.Type == JTokenType.Integer :
                        type == typeof(double) ? (field.Value.Type == JTokenType.Integer ||
                            field.Value.Type == JTokenType.Float) :
                        type == typeof(bool) && field.Value.Type == JTokenType.Boolean;
                    if (!valid) { throw new JsonSerializationException("Invalid video input field or type."); }
                }
            }
        }

        private void ValidateVideoSettings()
        {
            if (VideoInputs == null || SelectedVideoInputName == null)
            { throw new JsonSerializationException("Video inputs and selection cannot be null."); }
            var names = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            var payloads = PayloadDefinition.Parse(ActuatorDefinitions);
            foreach (var input in VideoInputs)
            {
                if (input == null) { throw new JsonSerializationException("Video inputs cannot contain null."); }
                input.Validate();
                foreach (var axis in input.ServoGimbalAxes)
                {
                    foreach (var payload in payloads)
                    {
                        if (!payload.Relay && payload.Channel == axis.Channel && (payload.Timed ||
                            payload.Minimum < axis.MinimumPwm || payload.Maximum > axis.MaximumPwm))
                        { throw new JsonSerializationException("A payload sharing a gimbal output must use direct servo commands within its calibrated PWM limits."); }
                    }
                }
                if (!names.Add(input.Name)) { throw new JsonSerializationException("Video input names must be unique."); }
            }
            if (VideoInputs.Count == 0 ? SelectedVideoInputName != "" : !names.Contains(SelectedVideoInputName))
            { throw new JsonSerializationException("Select an existing video input, or leave an empty list unselected."); }
        }
    }
}
