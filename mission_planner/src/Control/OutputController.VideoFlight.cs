// SPDX-License-Identifier: Apache-2.0
using System;

namespace NOMAD.MissionPlanner
{
    internal static partial class OutputController
    {
        internal static bool SendVideoVelocity(NativeTarget target, VideoFlightVelocity velocity, Func<bool> current)
        {
            if (target == null || !velocity.Valid) { return false; }
            lock (Gate)
            {
                if (!target.IsCurrent || _payloadBusy || target.Connection.giveComport || current?.Invoke() != true)
                { return false; }
                target.Connection.sendPacket(new MAVLink.mavlink_set_position_target_local_ned_t
                {
                    target_system = target.System, target_component = target.Component,
                    coordinate_frame = (byte)MAVLink.MAV_FRAME.BODY_NED,
                    // All velocity axes and yaw rate are active; position, acceleration and yaw angle are ignored.
                    type_mask = 1479, vx = velocity.Forward, vy = velocity.Right,
                    vz = velocity.Down, yaw_rate = velocity.YawRate,
                }, target.System, target.Component);
                return true;
            }
        }
    }
}
