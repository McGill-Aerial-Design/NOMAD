#!/usr/bin/env python3
"""
Test magnetometer heading from ZED camera.

Run inside Isaac ROS container or on host with ZED ROS2 wrapper running:
    python3 test_magnetometer.py

Displays:
- Raw magnetometer X, Y, Z values (uT)
- Computed heading (degrees, 0=North, 90=East)
- IMU roll/pitch/yaw for comparison
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, Imu
import math

def quat_to_euler(x, y, z, w):
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

class MagTester(Node):
    def __init__(self):
        super().__init__('mag_tester')
        
        # State
        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0
        self.mag_heading = 0.0
        self.mag_count = 0
        
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        self.imu_count = 0
        
        # Subscribe to magnetometer
        self.create_subscription(
            MagneticField,
            '/zed/zed_node/imu/mag',
            self.mag_callback,
            10
        )
        
        # Subscribe to IMU for comparison
        self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data',
            self.imu_callback,
            10
        )
        
        # Timer to print
        self.timer = self.create_timer(1.0, self.print_status)
        
    def mag_callback(self, msg):
        self.mag_x = msg.magnetic_field.x
        self.mag_y = msg.magnetic_field.y
        self.mag_z = msg.magnetic_field.z
        
        # Compute heading (0=North, positive=East/CW)
        # For ZED camera frame: X=forward, Y=left
        # atan2(-Y, X) gives heading where forward=North
        self.mag_heading = math.atan2(-self.mag_y, self.mag_x)
        self.mag_count += 1
    
    def imu_callback(self, msg):
        q = msg.orientation
        self.imu_roll, self.imu_pitch, self.imu_yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        self.imu_count += 1
    
    def print_status(self):
        mag_h_deg = math.degrees(self.mag_heading)
        imu_r_deg = math.degrees(self.imu_roll)
        imu_p_deg = math.degrees(self.imu_pitch)
        imu_y_deg = math.degrees(self.imu_yaw)
        
        print(f"\n{'='*70}")
        print(f"Magnetometer ({self.mag_count} samples):")
        print(f"  Raw: X={self.mag_x:8.2f} Y={self.mag_y:8.2f} Z={self.mag_z:8.2f} uT")
        print(f"  Heading: {mag_h_deg:7.1f}° (0=North, 90=East, 180=South, -90=West)")
        print(f"\nIMU ({self.imu_count} samples):")
        print(f"  Roll:  {imu_r_deg:7.1f}°  (right wing down = positive)")
        print(f"  Pitch: {imu_p_deg:7.1f}°  (nose up = positive)")
        print(f"  Yaw:   {imu_y_deg:7.1f}°  (gyro-integrated, drifts)")
        print(f"\nRotate camera and watch heading update")

def main():
    rclpy.init()
    node = MagTester()
    print("Testing magnetometer from /zed/zed_node/imu/mag")
    print("Point camera North (forward) to see heading ~0°")
    print("Rotate clockwise: East ~90°, South ~180°, West ~-90°")
    print("Press Ctrl+C to exit")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
