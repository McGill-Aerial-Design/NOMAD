// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Core Client
// ============================================================
// MP-free client for the C++ core CLI boundary. The core owns vehicle
// behavior, command validation, and safety decisions; this class only spawns
// the local `nomad` binary with a verb and reports its result. It must stay
// free of Mission Planner references so the standalone csc test harness can
// compile it (see scripts/build/test_plugin_core_client.ps1).
//
// The core CLI refuses actuation verbs without NOMAD_API_KEY and emits an
// audit line for every attempt, so an empty key here fails closed before any
// socket work — the same gate the Python contract tests pin.
// ============================================================

using System;
using System.Diagnostics;
using System.Globalization;

namespace NOMAD.MissionPlanner.Connectivity
{
    /// <summary>
    /// Invokes the C++ core CLI for the vehicle operations the plugin needs.
    /// </summary>
    public sealed class NomadCoreClient
    {
        /// <summary>
        /// The core binds this endpoint in listen mode and learns the vehicle
        /// peer from the first datagram (same as the SITL runners).
        /// </summary>
        public const string DefaultEndpoint = "udpin:0.0.0.0:14550";

        public string ExecutablePath { get; }
        public string Endpoint { get; }
        public string ApiKey { get; }

        public NomadCoreClient(string executablePath, string endpoint = DefaultEndpoint, string apiKey = "")
        {
            ExecutablePath = string.IsNullOrWhiteSpace(executablePath) ? "nomad" : executablePath;
            Endpoint = string.IsNullOrWhiteSpace(endpoint) ? DefaultEndpoint : endpoint;
            ApiKey = apiKey ?? "";
        }

        /// <summary>
        /// Fly to a position in GUIDED mode at the given relative altitude.
        /// The core sends MAV_CMD_DO_REPOSITION with the change-mode flag, so
        /// ArduPilot switches to GUIDED and the destination is verified
        /// against telemetry. Returns true only when the core verified the
        /// arrival position.
        /// </summary>
        public bool Goto(double latitudeDeg, double longitudeDeg, double altitudeRelM)
        {
            if (!IsValidLatitude(latitudeDeg) || !IsValidLongitude(longitudeDeg) || !IsFinite(altitudeRelM))
            {
                return false;
            }
            return RunCore("goto", BuildGotoValues(latitudeDeg, longitudeDeg, altitudeRelM)) == 0;
        }

        /// <summary>
        /// Drive an ArduPilot servo channel to a PWM value through the core
        /// (MAV_CMD_DO_SET_SERVO). Fails closed on out-of-range input.
        /// </summary>
        public bool Servo(int channel, int pwmUs)
        {
            if (channel < 1 || pwmUs < 500 || pwmUs > 2500)
            {
                return false;
            }
            return RunCore("servo", channel.ToString(CultureInfo.InvariantCulture),
                           pwmUs.ToString(CultureInfo.InvariantCulture)) == 0;
        }

        /// <summary>
        /// Toggle an ArduPilot relay through the core (MAV_CMD_DO_SET_RELAY).
        /// </summary>
        public bool SetRelay(int relayNumber, bool on)
        {
            if (relayNumber < 0 || relayNumber > 15)
            {
                return false;
            }
            return RunCore("relay", relayNumber.ToString(CultureInfo.InvariantCulture), on ? "1" : "0") == 0;
        }

        /// <summary>
        /// Run a motor test spin through the core (MAV_CMD_DO_MOTOR_TEST).
        /// PWM is a channel value 500..2500 us, or 0 to stop; the timeout is
        /// clamped to 0.05..3.0 s (the same bounds the plugin enforced).
        /// </summary>
        public bool MotorTest(int motorInstance, int pwmUs, double timeoutSeconds)
        {
            if (motorInstance < 1)
            {
                return false;
            }
            if (pwmUs != 0 && (pwmUs < 500 || pwmUs > 2500))
            {
                return false;
            }
            if (!IsFinite(timeoutSeconds))
            {
                return false;
            }
            var clamped = Math.Max(0.05, Math.Min(timeoutSeconds, 3.0));
            return RunCore(
                "motor-test",
                motorInstance.ToString(CultureInfo.InvariantCulture),
                pwmUs.ToString(CultureInfo.InvariantCulture),
                clamped.ToString("F2", CultureInfo.InvariantCulture)) == 0;
        }

        /// <summary>
        /// Select the gimbal mount mode through the core
        /// (MAV_CMD_DO_MOUNT_CONFIGURE; MAV_MOUNT_MODE values 0..4).
        /// </summary>
        public bool GimbalConfigure(int mountMode)
        {
            if (mountMode < 0 || mountMode > 4)
            {
                return false;
            }
            return RunCore("gimbal-config", mountMode.ToString(CultureInfo.InvariantCulture)) == 0;
        }

        /// <summary>
        /// Send a MAV_CMD_USER_1 command (the NOMAD motor-music Lua opcode
        /// protocol) through the core. Exactly seven finite values are
        /// required; the semantic layout is the adapter's concern.
        /// </summary>
        public bool SendUserCommand(params double[] parameters)
        {
            if (parameters == null || parameters.Length != 7)
            {
                return false;
            }
            var values = new string[7];
            for (var index = 0; index < 7; index++)
            {
                if (!IsFinite(parameters[index]))
                {
                    return false;
                }
                values[index] = parameters[index].ToString("F3", CultureInfo.InvariantCulture);
            }
            return RunCore("user-command", values) == 0;
        }

        /// <summary>
        /// Invariant-culture value vector for the goto verb; pinned by the
        /// standalone tests so a wrapper can trust the exact bytes sent to the
        /// core CLI (the CLI parses with strtof, so F7 lat/lon and F1 altitude
        /// round-trip without comma-decimal ambiguity).
        /// </summary>
        internal static string[] BuildGotoValues(double latitudeDeg, double longitudeDeg, double altitudeRelM)
        {
            return new[]
            {
                latitudeDeg.ToString("F7", CultureInfo.InvariantCulture),
                longitudeDeg.ToString("F7", CultureInfo.InvariantCulture),
                altitudeRelM.ToString("F1", CultureInfo.InvariantCulture),
            };
        }

        /// <summary>
        /// Joins an argument vector into the command line the process receives.
        /// Boundary values are space-free by construction (verb, formatted
        /// numbers, the udpin endpoint), but quoting is applied anyway so a
        /// configured executable path containing spaces still round-trips.
        /// </summary>
        private static string JoinArguments(string[] arguments)
        {
            return string.Join(" ", Array.ConvertAll(arguments, QuoteArgument));
        }

        private static string QuoteArgument(string argument)
        {
            if (argument.Length > 0 && argument.IndexOf(' ') < 0 && argument.IndexOf('"') < 0)
            {
                return argument;
            }
            return "\"" + argument.Replace("\"", "\\\"") + "\"";
        }

        /// <summary>
        /// Exact argument vector for a verb; pinned by the contract tests so a
        /// wrapper can trust the boundary surface.
        /// </summary>
        public static string[] BuildArguments(string endpoint, string verb, params string[] values)
        {
            var arguments = new string[values.Length + 3];
            arguments[0] = verb;
            for (var index = 0; index < values.Length; index++)
            {
                arguments[index + 1] = values[index];
            }
            arguments[arguments.Length - 2] = "--endpoint";
            arguments[arguments.Length - 1] = endpoint;
            return arguments;
        }

        internal static bool IsValidLatitude(double value)
        {
            return IsFinite(value) && value >= -90.0 && value <= 90.0;
        }

        internal static bool IsValidLongitude(double value)
        {
            return IsFinite(value) && value >= -180.0 && value <= 180.0;
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        private int RunCore(string verb, params string[] values)
        {
            var start = new ProcessStartInfo
            {
                FileName = ExecutablePath,
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true,
            };
            start.Arguments = JoinArguments(BuildArguments(Endpoint, verb, values));
            if (!string.IsNullOrEmpty(ApiKey))
            {
                start.Environment["NOMAD_API_KEY"] = ApiKey;
            }
            using var process = new Process { StartInfo = start };
            try
            {
                if (!process.Start())
                {
                    return -1;
                }
            }
            catch (Exception)
            {
                return -1;
            }
            process.WaitForExit(60000);
            if (!process.HasExited)
            {
                process.Kill();
                return -1;
            }
            return process.ExitCode;
        }
    }
}
