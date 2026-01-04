using System;
using System.Threading.Tasks;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Telemetry Injection for NOMAD Mission Planner Plugin.
    /// 
    /// Injects status updates from the Jetson into the Mission Planner HUD
    /// using MAVLink STATUSTEXT messages.
    /// 
    /// Example messages:
    /// - "Vision: OK"
    /// - "Target: Locked"
    /// - "Task 1: Snapshot Captured"
    /// </summary>
    public class TelemetryInjector
    {
        private readonly MAVLink.MavlinkParse _mavlink;
        
        public TelemetryInjector(MAVLink.MavlinkParse mavlink)
        {
            _mavlink = mavlink ?? throw new ArgumentNullException(nameof(mavlink));
        }
        
        /// <summary>
        /// Inject a status text message into the Mission Planner HUD.
        /// </summary>
        /// <param name="message">Message text (max 50 chars)</param>
        /// <param name="severity">Severity level (INFO, WARNING, ERROR, CRITICAL)</param>
        public void InjectStatusText(string message, MAVLink.MAV_SEVERITY severity = MAVLink.MAV_SEVERITY.INFO)
        {
            if (string.IsNullOrEmpty(message))
                return;
            
            // Truncate message to MAVLink limit
            if (message.Length > 50)
                message = message.Substring(0, 50);
            
            try
            {
                // Create STATUSTEXT message
                var statusText = new MAVLink.mavlink_statustext_t
                {
                    severity = (byte)severity,
                    text = System.Text.Encoding.ASCII.GetBytes(message.PadRight(50, '\0'))
                };
                
                // Send to Mission Planner for HUD display
                // Note: This is a simplified version. In production, you would send
                // this through the MAVLink connection to ensure proper routing.
                
                // For now, log to Mission Planner console
                MainV2.instance.BeginInvoke((Action)(() =>
                {
                    try
                    {
                        MainV2.comPort.MAV.cs.messages.Add((DateTime.Now, $"NOMAD: {message}"));
                    }
                    catch
                    {
                        // Fallback: try alternate method
                        System.Diagnostics.Debug.WriteLine($"NOMAD: {message}");
                    }
                }));
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"TelemetryInjector error: {ex.Message}");
            }
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
        /// Inject task completion status.
        /// </summary>
        public void SendTaskStatus(int taskId, string status)
        {
            InjectStatusText($"Task {taskId}: {status}");
        }
        
        /// <summary>
        /// Inject custom NOMAD status.
        /// </summary>
        public void SendCustomStatus(string status)
        {
            InjectStatusText($"NOMAD: {status}");
        }
    }
}
