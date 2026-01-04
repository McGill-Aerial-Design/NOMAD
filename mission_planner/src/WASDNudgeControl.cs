using System;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Indoor WASD Nudge Control for NOMAD Mission Planner Plugin.
    /// 
    /// Maps keyboard keys to SET_POSITION_TARGET_LOCAL_NED messages:
    /// - W: Forward (+Velocity X)
    /// - S: Backward (-Velocity X)
    /// - A: Left (-Velocity Y)
    /// - D: Right (+Velocity Y)
    /// - Q: Up (-Velocity Z)
    /// - E: Down (+Velocity Z)
    /// 
    /// Allows manual nudging during Task 2 (indoor) when GPS is unavailable.
    /// Works over ELRS transparent serial link even if 4G is down.
    /// </summary>
    public class WASDNudgeControl
    {
        private readonly MAVLink.MavlinkParse _mavlink;
        private bool _enabled = false;
        private float _nudgeSpeed = 0.5f; // m/s
        
        // Current velocity command
        private float _vx = 0f;
        private float _vy = 0f;
        private float _vz = 0f;
        
        // Timer for sending commands
        private System.Threading.Timer _commandTimer;
        
        public WASDNudgeControl(MAVLink.MavlinkParse mavlink)
        {
            _mavlink = mavlink ?? throw new ArgumentNullException(nameof(mavlink));
        }
        
        /// <summary>
        /// Enable or disable WASD nudge control.
        /// </summary>
        public bool Enabled
        {
            get => _enabled;
            set
            {
                if (_enabled == value)
                    return;
                
                _enabled = value;
                
                if (_enabled)
                {
                    StartCommandTimer();
                }
                else
                {
                    StopCommandTimer();
                    ResetVelocities();
                }
            }
        }
        
        /// <summary>
        /// Nudge speed in m/s.
        /// </summary>
        public float NudgeSpeed
        {
            get => _nudgeSpeed;
            set => _nudgeSpeed = Math.Max(0.1f, Math.Min(2.0f, value)); // Clamp 0.1-2.0 m/s
        }
        
        /// <summary>
        /// Handle keyboard input for WASD nudge.
        /// Call this from Form KeyDown event.
        /// </summary>
        public void HandleKeyDown(Keys keyCode)
        {
            if (!_enabled)
                return;
            
            switch (keyCode)
            {
                case Keys.W:
                    _vx = _nudgeSpeed;  // Forward
                    break;
                case Keys.S:
                    _vx = -_nudgeSpeed; // Backward
                    break;
                case Keys.A:
                    _vy = -_nudgeSpeed; // Left
                    break;
                case Keys.D:
                    _vy = _nudgeSpeed;  // Right
                    break;
                case Keys.Q:
                    _vz = -_nudgeSpeed; // Up
                    break;
                case Keys.E:
                    _vz = _nudgeSpeed;  // Down
                    break;
                default:
                    return; // Ignore other keys
            }
            
            SendVelocityCommand();
        }
        
        /// <summary>
        /// Handle keyboard release for WASD nudge.
        /// Call this from Form KeyUp event.
        /// </summary>
        public void HandleKeyUp(Keys keyCode)
        {
            if (!_enabled)
                return;
            
            switch (keyCode)
            {
                case Keys.W:
                case Keys.S:
                    _vx = 0f;
                    break;
                case Keys.A:
                case Keys.D:
                    _vy = 0f;
                    break;
                case Keys.Q:
                case Keys.E:
                    _vz = 0f;
                    break;
                default:
                    return;
            }
            
            SendVelocityCommand();
        }
        
        /// <summary>
        /// Send SET_POSITION_TARGET_LOCAL_NED message with velocity control.
        /// </summary>
        private void SendVelocityCommand()
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
                return;
            
            try
            {
                // Create SET_POSITION_TARGET_LOCAL_NED message
                var msg = new MAVLink.mavlink_set_position_target_local_ned_t
                {
                    time_boot_ms = (uint)Environment.TickCount,
                    target_system = MainV2.comPort.MAV.sysid,
                    target_component = MainV2.comPort.MAV.compid,
                    coordinate_frame = (byte)MAVLink.MAV_FRAME.LOCAL_NED,
                    
                    // Type mask: Only velocity components are used
                    // Bit 0-2: Position, Bit 3-5: Velocity, Bit 6-8: Acceleration, Bit 9: Force, Bit 10: Yaw, Bit 11: Yaw rate
                    // We want velocity only, so mask position (0b0000_0111 = 0x07)
                    type_mask = 0b0000_0111_1111_1000, // Use velocity only
                    
                    // Position (not used, but set to 0)
                    x = 0,
                    y = 0,
                    z = 0,
                    
                    // Velocity (NED frame)
                    vx = _vx,  // North (forward)
                    vy = _vy,  // East (right)
                    vz = _vz,  // Down (positive down)
                    
                    // Acceleration (not used)
                    afx = 0,
                    afy = 0,
                    afz = 0,
                    
                    // Yaw (not used)
                    yaw = 0,
                    yaw_rate = 0
                };
                
                // Send packet
                MainV2.comPort.sendPacket(msg, MainV2.comPort.MAV.sysid, MainV2.comPort.MAV.compid);
                
                System.Diagnostics.Debug.WriteLine($"WASD Nudge: vx={_vx:F2}, vy={_vy:F2}, vz={_vz:F2}");
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"WASDNudgeControl error: {ex.Message}");
            }
        }
        
        /// <summary>
        /// Start periodic command sending.
        /// </summary>
        private void StartCommandTimer()
        {
            // Send commands at 10 Hz
            _commandTimer = new System.Threading.Timer(
                _ => SendVelocityCommand(),
                null,
                TimeSpan.Zero,
                TimeSpan.FromMilliseconds(100)
            );
        }
        
        /// <summary>
        /// Stop periodic command sending.
        /// </summary>
        private void StopCommandTimer()
        {
            _commandTimer?.Dispose();
            _commandTimer = null;
        }
        
        /// <summary>
        /// Reset all velocities to zero.
        /// </summary>
        private void ResetVelocities()
        {
            _vx = 0f;
            _vy = 0f;
            _vz = 0f;
            SendVelocityCommand();
        }
    }
}
