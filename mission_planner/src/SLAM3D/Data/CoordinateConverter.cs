// ============================================================
// CoordinateConverter.cs - Frame conversion utilities
// ============================================================
// Handles conversions between ROS frames and OpenGL rendering.
// ============================================================

using System;

namespace NOMAD.MissionPlanner.SLAM3D.Data
{
    /// <summary>
    /// Converts between coordinate frames.
    /// </summary>
    /// <remarks>
    /// Frame conventions:
    /// - ROS REP-103 (map/odom): X-forward, Y-left, Z-up
    /// - OpenGL: X-right, Y-up, Z-out (toward viewer)
    /// - NED: X-north, Y-east, Z-down
    /// </remarks>
    public static class CoordinateConverter
    {
        /// <summary>
        /// Convert ROS map/odom frame position to OpenGL frame.
        /// </summary>
        /// <remarks>
        /// ROS: X-forward, Y-left, Z-up
        /// OpenGL: X-right, Y-up, Z-out (toward viewer)
        /// Mapping: GL_X = -ROS_Y, GL_Y = ROS_Z, GL_Z = -ROS_X
        /// </remarks>
        public static (float x, float y, float z) RosToOpenGL(float rosX, float rosY, float rosZ)
        {
            return (-rosY, rosZ, -rosX);
        }
        
        /// <summary>
        /// Convert OpenGL frame position to ROS map/odom frame.
        /// </summary>
        public static (float x, float y, float z) OpenGLToRos(float glX, float glY, float glZ)
        {
            return (-glZ, -glX, glY);
        }
        
        /// <summary>
        /// Convert ROS attitude (radians) to OpenGL rotation.
        /// </summary>
        /// <remarks>
        /// ROS: roll about X (forward), pitch about Y (left), yaw about Z (up)
        /// OpenGL: We use Euler angles with Y-up convention.
        /// Returns (rotX, rotY, rotZ) in degrees for OpenGL rotation.
        /// </remarks>
        public static (float rotX, float rotY, float rotZ) RosAttitudeToOpenGL(float rosRoll, float rosPitch, float rosYaw)
        {
            // Convert radians to degrees
            float rollDeg = rosRoll * 180f / MathHelper.PI;
            float pitchDeg = rosPitch * 180f / MathHelper.PI;
            float yawDeg = rosYaw * 180f / MathHelper.PI;
            
            // In OpenGL with Y-up:
            // - ROS roll (about forward/X) -> OpenGL Z rotation
            // - ROS pitch (about left/Y) -> OpenGL X rotation (negated)
            // - ROS yaw (about up/Z) -> OpenGL Y rotation (negated)
            return (pitchDeg, -yawDeg, -rollDeg);
        }
        
        /// <summary>
        /// Convert ROS yaw (radians, CCW from X-forward) to compass heading.
        /// </summary>
        /// <remarks>
        /// Returns heading in degrees, 0=North, 90=East, 180=South, 270=West.
        /// </remarks>
        public static float RosYawToHeading(float rosYaw)
        {
            // ROS yaw: 0 = forward (X), CCW positive
            // Heading: 0 = north, CW positive
            // Assuming ROS X-forward is pointing North when yaw=0
            float heading = -rosYaw * 180f / MathHelper.PI;
            while (heading < 0) heading += 360;
            while (heading >= 360) heading -= 360;
            return heading;
        }
        
        /// <summary>
        /// Convert compass heading to ROS yaw.
        /// </summary>
        public static float HeadingToRosYaw(float heading)
        {
            return -heading * MathHelper.PI / 180f;
        }
        
        /// <summary>
        /// Normalize angle to [-PI, PI].
        /// </summary>
        public static float NormalizeAngleRad(float angle)
        {
            while (angle > MathHelper.PI) angle -= 2 * MathHelper.PI;
            while (angle < -MathHelper.PI) angle += 2 * MathHelper.PI;
            return angle;
        }
        
        /// <summary>
        /// Normalize angle to [-180, 180].
        /// </summary>
        public static float NormalizeAngleDeg(float angle)
        {
            while (angle > 180) angle -= 360;
            while (angle < -180) angle += 360;
            return angle;
        }
        
        /// <summary>
        /// Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
        /// </summary>
        public static (float roll, float pitch, float yaw) QuaternionToEuler(float qx, float qy, float qz, float qw)
        {
            // Roll (X-axis rotation)
            float sinr_cosp = 2 * (qw * qx + qy * qz);
            float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
            float roll = MathHelper.Atan2(sinr_cosp, cosr_cosp);
            
            // Pitch (Y-axis rotation)
            float sinp = 2 * (qw * qy - qz * qx);
            float pitch;
            if (MathHelper.Abs(sinp) >= 1)
                pitch = MathHelper.CopySign(MathHelper.PI / 2, sinp); // Gimbal lock
            else
                pitch = MathHelper.Asin(sinp);
            
            // Yaw (Z-axis rotation)
            float siny_cosp = 2 * (qw * qz + qx * qy);
            float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
            float yaw = MathHelper.Atan2(siny_cosp, cosy_cosp);
            
            return (roll, pitch, yaw);
        }
    }
}
