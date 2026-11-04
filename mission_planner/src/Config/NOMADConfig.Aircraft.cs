// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System.Linq;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADConfig
    {
        public string AircraftName { get; set; } = "";
        public string FlightControllerModel { get; set; } = "";
        public string ReceiverType { get; set; } = "";
        public string AircraftFirmware { get; set; } = "";
        public string Airframe { get; set; } = "";
        public int GimbalMountMode { get; set; } = (int)MountMode.MavlinkTargeting;

        // Calibrated payload definitions belong to this saved aircraft.
        public JArray ActuatorDefinitions { get; set; }

        private static void ValidateAircraftDocument(JObject document)
        {
            foreach (var name in new[] { nameof(AircraftName), nameof(FlightControllerModel),
                nameof(ReceiverType), nameof(AircraftFirmware), nameof(Airframe) })
            {
                var value = document[name];
                if (value != null && value.Type != JTokenType.String && value.Type != JTokenType.Null)
                {
                    throw new JsonSerializationException(name + " must be text.");
                }
            }
            if (document[nameof(GimbalMountMode)] != null && document[nameof(GimbalMountMode)].Type != JTokenType.Integer)
            {
                throw new JsonSerializationException("GimbalMountMode must be an integer.");
            }
            foreach (string name in new[] { nameof(JoystickGimbalHorizontalAxis), nameof(JoystickGimbalVerticalAxis) })
            {
                if (document[name] != null && document[name].Type != JTokenType.String)
                { throw new JsonSerializationException(name + " must be text."); }
            }
            foreach (string name in new[] { nameof(JoystickGimbalHorizontalInverted), nameof(JoystickGimbalVerticalInverted) })
            {
                if (document[name] != null && document[name].Type != JTokenType.Boolean)
                { throw new JsonSerializationException(name + " must be boolean."); }
            }
            ValidateActuatorDefinitions(document[nameof(ActuatorDefinitions)]);
            if (document[nameof(Inputs)] != null) { InputBinding.Parse(document[nameof(Inputs)]); }
        }

        private static void ValidateActuatorDefinitions(JToken definitions)
        {
            PayloadDefinition.Parse(definitions);
        }

        internal bool HasSameOutputSettings(NOMADConfig other)
        {
            var candidate = JObject.FromObject(this);
            var active = JObject.FromObject(other);
            return candidate.Properties().Where(property => property.Name.StartsWith("Joystick", System.StringComparison.Ordinal) ||
                property.Name == nameof(Inputs) || property.Name == nameof(ActuatorDefinitions) || property.Name == nameof(GimbalMountMode))
                .All(property => JToken.DeepEquals(property.Value, active[property.Name]));
        }

        private void ValidateAircraftSettings()
        {
            ValidateActuatorDefinitions(ActuatorDefinitions);
            if (!GimbalCommand.ValidScreenAxes(JoystickGimbalHorizontalAxis, JoystickGimbalVerticalAxis))
            { throw new JsonSerializationException("Choose distinct gimbal screen axes: None, Pitch, Roll or Yaw."); }
            if (float.IsNaN(JoystickGimbalMaxRateDegSec) || float.IsInfinity(JoystickGimbalMaxRateDegSec) ||
                JoystickGimbalMaxRateDegSec < GimbalCommand.MIN_MAX_RATE_DEG_SEC ||
                JoystickGimbalMaxRateDegSec > GimbalCommand.MAX_MAX_RATE_DEG_SEC)
            {
                throw new JsonSerializationException("Gimbal rate is outside the supported input range.");
            }
            if (GimbalMountMode < (int)MountMode.Retract || GimbalMountMode > (int)MountMode.RcTargeting)
            {
                throw new JsonSerializationException("Gimbal mount mode must be between 0 and 3.");
            }
        }
    }
}
