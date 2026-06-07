# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""CLI entry point and execution loop for the ROS-HTTP Bridge."""

from __future__ import annotations

import argparse
import logging

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .node import ROSHTTPBridge

logger = logging.getLogger("ros_http_bridge.main")


def main():
    """Main CLI entry point for the ROS2-HTTP bridge."""
    parser = argparse.ArgumentParser(description="NOMAD ROS2-HTTP Bridge")
    parser.add_argument("--host", default=None, help="Edge Core host IP")
    parser.add_argument("--port", type=int, default=8000, help="Edge Core port")
    parser.add_argument("--vio-topic", default="/zed/zed_node/odom", help="ROS VIO odom topic")
    parser.add_argument("--imu-topic", default="/zed/zed_node/imu/data", help="ROS IMU topic")
    parser.add_argument("--mag-topic", default="/zed/zed_node/imu/mag", help="ROS Magnetometer topic")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel", help="ROS cmd_vel topic")
    parser.add_argument("--mesh-topic", default="/nvblox_node/color_layer_marker", help="ROS Mesh marker topic")
    parser.add_argument("--rate", type=float, default=30.0, help="Bridge publish rate (Hz)")
    parser.add_argument("--disable-nav", action="store_true", help="Disable navigation control")
    parser.add_argument("--disable-mesh", action="store_true", help="Disable mesh forwarding")
    parser.add_argument("--servo-topic", default="/nomad/servo/nozzle_angle", help="Nozzle angle topic")
    parser.add_argument("--disable-servo", action="store_true", help="Disable servo forwarding")
    parser.add_argument("--disable-imu-attitude", action="store_true", help="Disable IMU-based attitude")
    parser.add_argument("--disable-mag-heading", action="store_true", help="Disable magnetometer-based heading")

    args = parser.parse_args()
    if args.rate <= 0:
        parser.error(f"--rate must be positive, got {args.rate}")

    rclpy.init()

    # Create the multi-threaded bridge node
    bridge = ROSHTTPBridge(
        host=args.host or "127.0.0.1",
        port=args.port,
        vio_topic=args.vio_topic,
        imu_topic=args.imu_topic,
        mag_topic=args.mag_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        mesh_topic=args.mesh_topic,
        servo_topic=args.servo_topic,
        send_rate_hz=args.rate,
        enable_nav_control=not args.disable_nav,
        enable_mesh=not args.disable_mesh,
        enable_servo=not args.disable_servo,
        use_imu_attitude=not args.disable_imu_attitude,
        use_mag_heading=not args.disable_mag_heading,
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)

    logger.info("Starting ROS HTTP bridge spin loop...")
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Bridge thread execution crashed: {e}")
    finally:
        try:
            stats = bridge.get_stats()
            logger.info(f"Final bridge statistics: {stats}")
        except Exception:
            pass

        try:
            bridge.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
