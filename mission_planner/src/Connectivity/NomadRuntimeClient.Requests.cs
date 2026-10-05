// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Globalization;

namespace NOMAD.MissionPlanner.Connectivity
{
    internal sealed partial class NomadRuntimeClient
    {
        private Dictionary<string, object> BuildRuntimeRequest(string verb, string[] values)
        {
            if (IsAuthorityVerb(verb) && values.Length == 0)
            {
                var authorityType = verb == "admit" ? "admit_authority" :
                                    verb == "revoke" ? "revoke_authority" : "handback_authority";
                return BaseRequest("", authorityType);
            }
            if (IsActuatorVerb(verb))
            {
                return BuildActuatorRequest(verb, values);
            }
            var type = verb switch
            {
                "servo" => "set_servo",
                "relay" => "set_relay",
                "motor-test" => "motor_test",
                "gimbal-config" => "configure_gimbal",
                "gimbal-target" => "set_gimbal_target",
                _ => null,
            };
            if (type == null)
            {
                return null;
            }
            var request = BaseRequest("", type);
            if (verb == "servo" && values.Length == 2)
            {
                request["channel"] = int.Parse(values[0], CultureInfo.InvariantCulture);
                request["pwm_microseconds"] = int.Parse(values[1], CultureInfo.InvariantCulture);
            }
            else if (verb == "relay" && values.Length == 2)
            {
                request["relay_number"] = int.Parse(values[0], CultureInfo.InvariantCulture);
                request["on"] = values[1] == "1";
            }
            else if (verb == "motor-test" && values.Length == 3)
            {
                request["motor_instance"] = int.Parse(values[0], CultureInfo.InvariantCulture);
                request["pwm_microseconds"] = int.Parse(values[1], CultureInfo.InvariantCulture);
                request["timeout_seconds"] = ParseProtocolNumber(values[2]);
            }
            else if (verb == "gimbal-config" && values.Length == 1)
            {
                request["mount_mode"] = int.Parse(values[0], CultureInfo.InvariantCulture);
            }
            else if (verb == "gimbal-target" && values.Length == 2)
            {
                request["pitch_deg"] = ParseProtocolNumber(values[0]);
                request["roll_deg"] = ParseProtocolNumber(values[1]);
            }
            else
            {
                return null;
            }
            return request;
        }

        private static bool IsAuthorityVerb(string verb)
        {
            return verb == "admit" || verb == "revoke" || verb == "handback";
        }

        private static double ParseProtocolNumber(string value)
        {
            return double.Parse(value, NumberStyles.Float, CultureInfo.InvariantCulture);
        }

    }
}
