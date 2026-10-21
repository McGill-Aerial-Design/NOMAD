// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

namespace NOMAD.MissionPlanner.Connectivity
{
    public sealed class NomadStatusResult
    {
        public NomadRuntimeStatus Snapshot { get; }
        public string ErrorCode { get; }
        public string Message { get; }
        public bool Succeeded => Snapshot != null;

        internal NomadStatusResult(NomadRuntimeStatus snapshot, string errorCode = "", string message = "")
        {
            Snapshot = snapshot;
            ErrorCode = errorCode;
            Message = message;
        }
    }

    public sealed class NomadFlightRequestStatus
    {
        public string RequestId { get; }
        public string Operation { get; }
        public ulong VehicleSession { get; }
        public ulong AuthorityGeneration { get; }
        public long AgeMilliseconds { get; }
        public string Phase { get; }
        public bool Acknowledged { get; }
        public string Outcome { get; }

        internal NomadFlightRequestStatus(string id, string operation, ulong session, ulong generation,
            long age, string phase, bool acknowledged, string outcome)
        {
            RequestId = id;
            Operation = operation;
            VehicleSession = session;
            AuthorityGeneration = generation;
            AgeMilliseconds = age;
            Phase = phase;
            Acknowledged = acknowledged;
            Outcome = outcome;
        }
    }

    /// <summary>One authenticated observation, never a grant of aircraft authority.</summary>
    public sealed class NomadRuntimeStatus
    {
        public DateTimeOffset ReceivedAtUtc { get; } = DateTimeOffset.UtcNow;
        private readonly long _receivedTicks = Stopwatch.GetTimestamp();
        private readonly long _requestStartedTicks;
        public double RequestDurationMilliseconds => Math.Max(0,
            (_receivedTicks - _requestStartedTicks) * 1000.0 / Stopwatch.Frequency);
        public double AgeUpperBoundMilliseconds => Math.Max(0,
            (Stopwatch.GetTimestamp() - _requestStartedTicks) * 1000.0 / Stopwatch.Frequency);
        public string RuntimeIncarnation { get; private set; }
        public string RuntimeVersion { get; private set; }
        public IReadOnlyList<string> ProtocolCapabilities { get; private set; }
        public IReadOnlyList<string> SupportedFlightOperations { get; private set; }
        public NomadFlightRequestStatus LastFlightRequest { get; private set; }
        public bool? RuntimeReady { get; private set; }
        public string Lifecycle { get; private set; }
        public bool? AuditHealthy { get; private set; }
        public bool? ActuationEnabled { get; private set; }
        public ulong? VehicleSession { get; private set; }
        public ulong? AuthorityGeneration { get; private set; }
        public bool AuthorityOwnerKnown { get; private set; }
        public string AuthorityOwner { get; private set; }
        public bool? ConnectionOpen { get; private set; }
        public bool? VehicleTransportConnected { get; private set; }
        public bool? VehicleSessionEstablished { get; private set; }
        public bool? VehicleConnected { get; private set; }
        public bool? IdentityResolved { get; private set; }
        public string AircraftClass { get; private set; }
        public bool? Armed { get; private set; }
        public uint? CustomMode { get; private set; }
        public bool? HeartbeatFresh { get; private set; }
        public long? HeartbeatAgeMs { get; private set; }
        public string VtolState { get; private set; }
        public bool? VtolStateValid { get; private set; }
        public long? VtolStateAgeMs { get; private set; }
        public string LandedState { get; private set; }
        public bool? LandedStateValid { get; private set; }
        public long? LandedStateAgeMs { get; private set; }
        public bool? PositionValid { get; private set; }
        public long? PositionAgeMs { get; private set; }
        public double? LatitudeDeg { get; private set; }
        public double? LongitudeDeg { get; private set; }
        public double? AbsoluteAltitudeM { get; private set; }
        public double? RelativeAltitudeM { get; private set; }
        public bool? GpsValid { get; private set; }
        public long? GpsAgeMs { get; private set; }
        public bool? AttitudeValid { get; private set; }
        public long? AttitudeAgeMs { get; private set; }
        public bool? BatteryValid { get; private set; }
        public long? BatteryAgeMs { get; private set; }

        private NomadRuntimeStatus(long requestStartedTicks)
        {
            _requestStartedTicks = requestStartedTicks;
        }

        internal static NomadRuntimeStatus Parse(Dictionary<string, object> hello, Dictionary<string, object> response,
            long requestStartedTicks)
        {
            var status = Object(response, "status");
            var incarnation = Text(hello, "runtime_incarnation");
            if (status == null || string.IsNullOrEmpty(incarnation)
                || Text(status, "runtime_incarnation") != incarnation)
            {
                throw new InvalidDataException("STATUS does not match the authenticated runtime incarnation.");
            }
            var result = new NomadRuntimeStatus(requestStartedTicks)
            {
                RuntimeIncarnation = incarnation,
                RuntimeVersion = Text(hello, "runtime_version"),
                ProtocolCapabilities = Strings(hello, "capabilities"),
                SupportedFlightOperations = Strings(status, "supported_flight_operations"),
                LastFlightRequest = ReadFlightRequest(Object(status, "last_flight_request")),
                RuntimeReady = Boolean(status, "runtime_ready"),
                Lifecycle = Choice(status, "lifecycle", "ready", "degraded", "stopping"),
                AuditHealthy = Boolean(status, "audit_healthy"),
                ActuationEnabled = Boolean(status, "actuation_enabled"),
                VehicleSession = Unsigned(status, "vehicle_session"),
                AuthorityGeneration = Unsigned(status, "authority_generation"),
                ConnectionOpen = Boolean(status, "mavsdk_connection_open"),
                VehicleTransportConnected = Boolean(status, "vehicle_transport_connected"),
                VehicleSessionEstablished = Boolean(status, "vehicle_session_established"),
                VehicleConnected = Boolean(status, "vehicle_connected"),
                IdentityResolved = Boolean(status, "identity_resolved"),
                AircraftClass = Choice(status, "aircraft_class", "Unknown", "Copter", "Plane", "QuadPlane"),
                Armed = Boolean(status, "armed"),
            };
            var mode = Unsigned(status, "custom_mode");
            result.CustomMode = mode <= uint.MaxValue ? (uint?)mode : null;
            result.AuthorityOwner = Text(status, "authority_owner");
            result.AuthorityOwnerKnown = status.TryGetValue("authority_owner", out var owner)
                && (owner == null || !string.IsNullOrEmpty(result.AuthorityOwner));
            result.ReadTelemetry(Object(status, "telemetry"));
            return result;
        }

        private static NomadFlightRequestStatus ReadFlightRequest(Dictionary<string, object> request)
        {
            var id = Text(request, "request_id");
            var operation = Choice(request, "operation", "land");
            var session = Unsigned(request, "vehicle_session");
            var generation = Unsigned(request, "authority_generation");
            var age = Age(request, "age_ms");
            var phase = Choice(request, "phase", "sending", "observing", "completed");
            var acknowledged = Boolean(request, "acknowledged");
            if (string.IsNullOrEmpty(id) || operation == null || session == null || session == 0
                || generation == null || generation == 0 || age == null || phase == null || acknowledged == null)
            {
                return null;
            }
            var outcome = Choice(request, "outcome", "success", "failed", "rejected", "unknown", "interrupted");
            if (phase == "completed")
            {
                if (outcome == null || (outcome == "success" || outcome == "failed") && !acknowledged.Value
                    || outcome == "rejected" && acknowledged.Value)
                {
                    return null;
                }
            }
            else if (!request.TryGetValue("outcome", out var rawOutcome) || rawOutcome != null
                || phase == "observing" && !acknowledged.Value)
            {
                return null;
            }
            return new NomadFlightRequestStatus(id, operation, session.Value, generation.Value, age.Value,
                phase, acknowledged.Value, outcome);
        }

        private void ReadTelemetry(Dictionary<string, object> telemetry)
        {
            HeartbeatFresh = Boolean(telemetry, "heartbeat_fresh");
            HeartbeatAgeMs = Age(telemetry, "heartbeat_age_ms");
            VtolState = Choice(telemetry, "vtol_state", "undefined", "transition_to_fixed_wing",
                "transition_to_multicopter", "multicopter", "fixed_wing");
            VtolStateValid = Boolean(telemetry, "vtol_state_valid");
            VtolStateAgeMs = Age(telemetry, "vtol_state_age_ms");
            LandedState = Choice(telemetry, "landed_state", "unknown", "on_ground", "in_air", "taking_off", "landing");
            LandedStateValid = Boolean(telemetry, "landed_state_valid");
            LandedStateAgeMs = Age(telemetry, "landed_state_age_ms");
            PositionValid = Boolean(telemetry, "position_valid");
            PositionAgeMs = Age(telemetry, "position_age_ms");
            var position = Object(telemetry, "position");
            LatitudeDeg = Number(position, "latitude_deg");
            LongitudeDeg = Number(position, "longitude_deg");
            if (!LatitudeDeg.HasValue || !LongitudeDeg.HasValue || LatitudeDeg < -90 || LatitudeDeg > 90
                || LongitudeDeg < -180 || LongitudeDeg > 180)
            {
                LatitudeDeg = null;
                LongitudeDeg = null;
            }
            AbsoluteAltitudeM = Number(position, "absolute_altitude_m");
            RelativeAltitudeM = Number(position, "relative_altitude_m");
            GpsValid = Boolean(telemetry, "gps_valid");
            GpsAgeMs = Age(telemetry, "gps_age_ms");
            AttitudeValid = Boolean(telemetry, "attitude_valid");
            AttitudeAgeMs = Age(telemetry, "attitude_age_ms");
            BatteryValid = Boolean(telemetry, "battery_valid");
            BatteryAgeMs = Age(telemetry, "battery_age_ms");
        }

        private static Dictionary<string, object> Object(Dictionary<string, object> values, string key) =>
            values != null && values.TryGetValue(key, out var value) ? value as Dictionary<string, object> : null;

        private static bool? Boolean(Dictionary<string, object> values, string key) =>
            values != null && values.TryGetValue(key, out var value) && value is bool boolean ? boolean : (bool?)null;

        private static ulong? Unsigned(Dictionary<string, object> values, string key)
        {
            if (values == null || !values.TryGetValue(key, out var value))
            {
                return null;
            }
            if (value is int integer && integer >= 0)
            {
                return (ulong)integer;
            }
            if (value is long wide && wide >= 0)
            {
                return (ulong)wide;
            }
            if (value is decimal number && number >= 0 && number <= ulong.MaxValue
                && decimal.Truncate(number) == number)
            {
                return (ulong)number;
            }
            return null;
        }

        private static long? Age(Dictionary<string, object> values, string key)
        {
            var value = Unsigned(values, key);
            return value <= long.MaxValue ? (long?)value : null;
        }

        private static double? Number(Dictionary<string, object> values, string key)
        {
            if (values == null || !values.TryGetValue(key, out var value)
                || !(value is int || value is long || value is decimal || value is double))
            {
                return null;
            }
            var number = Convert.ToDouble(value, System.Globalization.CultureInfo.InvariantCulture);
            return double.IsNaN(number) || double.IsInfinity(number) ? (double?)null : number;
        }

        private static string Text(Dictionary<string, object> values, string key)
        {
            if (values == null || !values.TryGetValue(key, out var value)
                || !(value is string text) || text.Length > 512)
            {
                return null;
            }
            foreach (var character in text)
            {
                if (char.IsControl(character))
                {
                    return null;
                }
            }
            return text;
        }

        private static string Choice(Dictionary<string, object> values, string key, params string[] choices)
        {
            var value = Text(values, key);
            return Array.IndexOf(choices, value) >= 0 ? value : null;
        }

        private static IReadOnlyList<string> Strings(Dictionary<string, object> values, string key)
        {
            if (values == null || !values.TryGetValue(key, out var value)
                || !(value is object[] entries) || entries.Length > 64)
            {
                return null;
            }
            var result = new List<string>();
            foreach (var entry in entries)
            {
                if (!(entry is string text) || string.IsNullOrEmpty(text) || text.Length > 64)
                {
                    return null;
                }
                foreach (var character in text)
                {
                    if (!(character >= 'a' && character <= 'z') && !(character >= 'A' && character <= 'Z')
                        && !(character >= '0' && character <= '9') && character != '_' && character != '-')
                    {
                        return null;
                    }
                }
                if (result.Contains(text))
                {
                    return null;
                }
                result.Add(text);
            }
            return result.AsReadOnly();
        }
    }
}
