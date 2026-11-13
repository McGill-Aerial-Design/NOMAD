// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public sealed class ServoGimbalAxis
    {
        public string Axis { get; set; } = "Pitch";
        public int Channel { get; set; } = 1;
        public int MinimumPwm { get; set; } = 1000;
        public int MaximumPwm { get; set; } = 2000;
        public double MinimumAngleDeg { get; set; } = -90;
        public double MaximumAngleDeg { get; set; } = 90;
        public bool Reversed { get; set; }

        internal ServoGimbalAxis Copy() => (ServoGimbalAxis)MemberwiseClone();
        private bool Calibrated => (Axis == "Pitch" || Axis == "Yaw" || Axis == "Roll") &&
            Channel >= 1 && Channel <= 16 && MinimumPwm >= 500 && MaximumPwm <= 2500 && MinimumPwm < MaximumPwm &&
            Finite(MinimumAngleDeg) && Finite(MaximumAngleDeg) && MinimumAngleDeg >= -180 && MaximumAngleDeg <= 180 &&
            MinimumAngleDeg < MaximumAngleDeg && MinimumAngleDeg <= 0 && MaximumAngleDeg >= 0;
        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        internal bool TryPwm(double angle, out int pwm)
        {
            pwm = 0;
            if (!Calibrated || !Finite(angle)) { return false; }
            // Controller targets and PWM feedback carry the float representation of these double limits.
            if (angle == (double)(float)MinimumAngleDeg) { angle = MinimumAngleDeg; }
            else if (angle == (double)(float)MaximumAngleDeg) { angle = MaximumAngleDeg; }
            if (angle < MinimumAngleDeg || angle > MaximumAngleDeg) { return false; }
            double fraction = (angle - MinimumAngleDeg) / (MaximumAngleDeg - MinimumAngleDeg);
            if (Reversed) { fraction = 1 - fraction; }
            pwm = (int)Math.Round(MinimumPwm + fraction * (MaximumPwm - MinimumPwm), MidpointRounding.AwayFromZero);
            return true;
        }
        internal bool TryAngle(int pwm, out float angle)
        {
            angle = 0;
            if (!Calibrated || pwm < MinimumPwm || pwm > MaximumPwm) { return false; }
            double fraction = (pwm - MinimumPwm) / (double)(MaximumPwm - MinimumPwm);
            if (Reversed) { fraction = 1 - fraction; }
            angle = (float)(MinimumAngleDeg + fraction * (MaximumAngleDeg - MinimumAngleDeg));
            return true;
        }
        internal static void Validate(IReadOnlyList<ServoGimbalAxis> axes)
        {
            if (axes == null || axes.Count > 3) { throw Invalid("Use at most three servo gimbal axes."); }
            var names = new HashSet<string>();
            var channels = new HashSet<int>();
            foreach (var axis in axes)
            {
                if (axis == null || !axis.Calibrated)
                { throw Invalid("Gimbal axes need channels 1–16, PWM 500–2500, and increasing angle limits within ±180° containing zero."); }
                if (!names.Add(axis.Axis) || !channels.Add(axis.Channel))
                { throw Invalid("Gimbal axes and servo channels must be distinct within a video input."); }
            }
        }
        internal static List<ServoGimbalAxis> Parse(JToken token)
        {
            if (!(token is JArray array) || array.Count > 3) { throw Invalid("ServoGimbalAxes must be an array of up to three axes."); }
            var schema = JObject.FromObject(new ServoGimbalAxis());
            var result = new List<ServoGimbalAxis>();
            foreach (var item in array)
            {
                if (!(item is JObject axis)) { throw Invalid("Each servo gimbal axis must be an object."); }
                foreach (var field in axis.Properties())
                {
                    var expected = schema[field.Name];
                    if (expected == null || (field.Value.Type != expected.Type &&
                        !(expected.Type == JTokenType.Float && field.Value.Type == JTokenType.Integer)))
                    { throw Invalid("Unknown or incorrectly typed gimbal axis field: " + field.Name); }
                }
                foreach (string required in new[] { "Axis", "Channel", "MinimumPwm", "MaximumPwm",
                    "MinimumAngleDeg", "MaximumAngleDeg" })
                { if (axis[required] == null) { throw Invalid("Gimbal calibration field required: " + required); } }
                result.Add(axis.ToObject<ServoGimbalAxis>());
            }
            Validate(result);
            return result;
        }
        internal static bool Same(IReadOnlyList<ServoGimbalAxis> a, IReadOnlyList<ServoGimbalAxis> b)
        {
            if ((a?.Count ?? 0) != (b?.Count ?? 0)) { return false; }
            for (int index = 0; index < (a?.Count ?? 0); index++)
            {
                var x = a[index]; var y = b[index];
                if (x == null || y == null || x.Axis != y.Axis || x.Channel != y.Channel ||
                    x.MinimumPwm != y.MinimumPwm || x.MaximumPwm != y.MaximumPwm ||
                    x.MinimumAngleDeg != y.MinimumAngleDeg || x.MaximumAngleDeg != y.MaximumAngleDeg ||
                    x.Reversed != y.Reversed) { return false; }
            }
            return true;
        }
        private static JsonSerializationException Invalid(string message) => new JsonSerializationException(message);
    }
}
