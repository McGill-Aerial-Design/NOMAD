// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    /// <summary>Present NOMAD notifications in Mission Planner's local message list.</summary>
    public class TelemetryInjector
    {
        /// <summary>Append a local notification; this does not transmit MAVLink.</summary>
        public void InjectStatusText(string message, MAVLink.MAV_SEVERITY severity = MAVLink.MAV_SEVERITY.INFO)
        {
            if (string.IsNullOrWhiteSpace(message))
            {
                return;
            }
            var notification = $"NOMAD [{severity}]: {message}";
            UiAsync.RunSync(MainV2.instance, () =>
            {
                try
                {
                    MainV2.comPort.MAV.cs.messages.Add((DateTime.Now, notification));
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"{notification} (local display unavailable: {ex.Message})");
                }
            }, "InjectStatusText");
        }

        /// <summary>
        /// Inject vision status update.
        /// </summary>
        public void SendVisionStatus(bool isOk, string details = "")
        {
            var severity = isOk ? MAVLink.MAV_SEVERITY.INFO : MAVLink.MAV_SEVERITY.WARNING;
            var message = isOk ? $"Vision: OK {details}" : $"Vision: FAIL {details}";
            InjectStatusText(message, severity);
        }

        /// <summary>
        /// Inject target lock status.
        /// </summary>
        public void SendTargetStatus(bool isLocked, string targetType = "")
        {
            var message = isLocked
                ? $"Target: Locked {targetType}"
                : "Target: Searching";
            InjectStatusText(message);
        }

        /// <summary>
        /// Inject a labeled status message (e.g. "Payload: Released").
        /// </summary>
        public void SendStatus(string label, string status)
        {
            InjectStatusText($"{label}: {status}");
        }

        /// <summary>
        /// Inject custom NOMAD status.
        /// </summary>
        public void SendCustomStatus(string status)
        {
            InjectStatusText(status);
        }
    }
}
