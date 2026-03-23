#!/usr/bin/env python3
"""
Nav2 Goal Bridge for NOMAD.

Bridges Edge Core HTTP API navigation commands to ROS2 Nav2 action servers.
Runs INSIDE the Isaac ROS Docker container alongside ros_http_bridge.

Polls Edge Core for pending navigation goals and sends them to Nav2.
Reports nav2 feedback/result back to Edge Core.

Architecture:
    Mission Planner -> Edge Core API -> (HTTP poll) -> nav2_goal_bridge
    -> Nav2 NavigateToPose/NavigateThroughPoses action
    -> Nav2 plans path using nvblox costmap (obstacle avoidance)
    -> /cmd_vel -> ros_http_bridge -> Edge Core -> NavController -> ArduPlane GUIDED

Usage (inside Isaac ROS container):
    python3 nav2_goal_bridge.py --host 172.17.0.1 --port 8000
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import threading
import time
from typing import Optional
from urllib.request import Request, urlopen
from urllib.error import URLError

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, FollowWaypoints
from action_msgs.msg import GoalStatus

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("nav2_goal_bridge")


class Nav2GoalBridge(Node):
    """
    Bridges Edge Core navigation commands to Nav2 action servers.

    Polls Edge Core /api/nav2/pending for navigation goals.
    Sends goals to Nav2 via action clients.
    Reports status back to Edge Core /api/nav2/feedback.
    """

    def __init__(
        self,
        host: str = "172.17.0.1",
        port: int = 8000,
        poll_rate_hz: float = 2.0,
    ):
        super().__init__("nomad_nav2_goal_bridge")

        self._host = host
        self._port = port
        self._base_url = f"http://{host}:{port}"

        # Nav2 action clients
        self._nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self._nav_through_poses_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses'
        )
        self._follow_waypoints_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints'
        )

        # Current goal state
        self._current_goal_handle = None
        self._current_goal_id: Optional[str] = None
        self._nav_status = "idle"  # idle, navigating, succeeded, failed, cancelled
        self._nav_feedback = {}

        # Poll timer
        poll_period = 1.0 / poll_rate_hz
        self.create_timer(poll_period, self._poll_and_dispatch)

        # Feedback report timer (5 Hz)
        self.create_timer(0.2, self._report_feedback)

        self.get_logger().info(
            f"Nav2 goal bridge started: {poll_rate_hz} Hz poll -> {self._base_url}"
        )

    def _http_get(self, path: str, timeout: float = 2.0) -> Optional[dict]:
        """GET request to Edge Core."""
        try:
            resp = urlopen(f"{self._base_url}{path}", timeout=timeout)
            return json.loads(resp.read())
        except Exception:
            return None

    def _http_post(self, path: str, data: dict, timeout: float = 2.0) -> bool:
        """POST JSON to Edge Core."""
        try:
            req = Request(
                f"{self._base_url}{path}",
                data=json.dumps(data).encode(),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            urlopen(req, timeout=timeout)
            return True
        except Exception as e:
            self.get_logger().warning(f"POST {path} failed: {e}", throttle_duration_sec=5.0)
            return False

    def _poll_and_dispatch(self) -> None:
        """Poll Edge Core for pending navigation goals."""
        result = self._http_get("/api/nav2/pending")
        if not result or not result.get("goal"):
            return

        goal = result["goal"]
        goal_id = goal.get("id", "unknown")
        goal_type = goal.get("type", "navigate_to_pose")

        # Don't re-send the same goal
        if goal_id == self._current_goal_id and self._nav_status == "navigating":
            return

        self.get_logger().info(f"New nav2 goal: {goal_type} id={goal_id}")

        if goal_type == "navigate_to_pose":
            self._send_navigate_to_pose(goal, goal_id)
        elif goal_type == "navigate_through_poses":
            self._send_navigate_through_poses(goal, goal_id)
        elif goal_type == "follow_waypoints":
            self._send_follow_waypoints(goal, goal_id)
        elif goal_type == "cancel":
            self._cancel_current_goal()

    def _make_pose_stamped(self, pose_dict: dict) -> PoseStamped:
        """Convert dict {x, y, z, yaw} to PoseStamped in odom frame."""
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(pose_dict.get("x", 0.0))
        ps.pose.position.y = float(pose_dict.get("y", 0.0))
        ps.pose.position.z = float(pose_dict.get("z", 0.0))

        # Yaw to quaternion (rotation around Z)
        yaw = float(pose_dict.get("yaw", 0.0))
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        return ps

    def _send_navigate_to_pose(self, goal: dict, goal_id: str) -> None:
        """Send a single pose goal to Nav2."""
        if not self._nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 navigate_to_pose server not available")
            self._nav_status = "failed"
            self._report_result(goal_id, "failed", "Nav2 server not available")
            return

        # Cancel any existing goal
        if self._current_goal_handle is not None:
            self._cancel_current_goal()
            time.sleep(0.2)

        msg = NavigateToPose.Goal()
        msg.pose = self._make_pose_stamped(goal.get("pose", {}))

        self._current_goal_id = goal_id
        self._nav_status = "navigating"
        self._nav_feedback = {}

        future = self._nav_to_pose_client.send_goal_async(
            msg, feedback_callback=self._nav_feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _send_navigate_through_poses(self, goal: dict, goal_id: str) -> None:
        """Send multiple poses to Nav2 (single path through all)."""
        if not self._nav_through_poses_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 navigate_through_poses server not available")
            self._nav_status = "failed"
            self._report_result(goal_id, "failed", "Nav2 server not available")
            return

        if self._current_goal_handle is not None:
            self._cancel_current_goal()
            time.sleep(0.2)

        msg = NavigateThroughPoses.Goal()
        poses = goal.get("poses", [])
        msg.poses = [self._make_pose_stamped(p) for p in poses]

        self._current_goal_id = goal_id
        self._nav_status = "navigating"
        self._nav_feedback = {"total_poses": len(poses)}

        future = self._nav_through_poses_client.send_goal_async(
            msg, feedback_callback=self._nav_feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _send_follow_waypoints(self, goal: dict, goal_id: str) -> None:
        """Send waypoints to Nav2 waypoint follower (stops at each)."""
        if not self._follow_waypoints_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 follow_waypoints server not available")
            self._nav_status = "failed"
            self._report_result(goal_id, "failed", "Nav2 server not available")
            return

        if self._current_goal_handle is not None:
            self._cancel_current_goal()
            time.sleep(0.2)

        msg = FollowWaypoints.Goal()
        waypoints = goal.get("waypoints", [])
        msg.poses = [self._make_pose_stamped(w) for w in waypoints]

        self._current_goal_id = goal_id
        self._nav_status = "navigating"
        self._nav_feedback = {"total_waypoints": len(waypoints)}

        future = self._follow_waypoints_client.send_goal_async(
            msg, feedback_callback=self._nav_feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _cancel_current_goal(self) -> None:
        """Cancel the current Nav2 goal."""
        if self._current_goal_handle is not None:
            self.get_logger().info(f"Cancelling goal {self._current_goal_id}")
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warning(f"Cancel failed: {e}")
            self._current_goal_handle = None
            self._nav_status = "cancelled"
            if self._current_goal_id:
                self._report_result(self._current_goal_id, "cancelled", "Goal cancelled")
            self._current_goal_id = None

    def _goal_response_cb(self, future) -> None:
        """Called when Nav2 accepts/rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(f"Goal {self._current_goal_id} rejected by Nav2")
            self._nav_status = "failed"
            self._report_result(self._current_goal_id, "failed", "Goal rejected by Nav2")
            self._current_goal_id = None
            return

        self.get_logger().info(f"Goal {self._current_goal_id} accepted by Nav2")
        self._current_goal_handle = goal_handle

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future) -> None:
        """Called when Nav2 completes/fails the goal."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal {self._current_goal_id} succeeded")
            self._nav_status = "succeeded"
            self._report_result(self._current_goal_id, "succeeded", "Goal reached")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"Goal {self._current_goal_id} cancelled")
            self._nav_status = "cancelled"
            self._report_result(self._current_goal_id, "cancelled", "Goal cancelled")
        else:
            self.get_logger().warning(f"Goal {self._current_goal_id} failed (status={status})")
            self._nav_status = "failed"
            self._report_result(self._current_goal_id, "failed", f"Nav2 status {status}")

        self._current_goal_handle = None
        self._current_goal_id = None

    def _nav_feedback_cb(self, feedback_msg) -> None:
        """Called periodically by Nav2 with progress updates."""
        fb = feedback_msg.feedback
        if hasattr(fb, 'current_pose'):
            p = fb.current_pose.pose.position
            self._nav_feedback["current_x"] = p.x
            self._nav_feedback["current_y"] = p.y
            self._nav_feedback["current_z"] = p.z
        if hasattr(fb, 'distance_remaining'):
            self._nav_feedback["distance_remaining"] = fb.distance_remaining
        if hasattr(fb, 'estimated_time_remaining'):
            self._nav_feedback["eta_sec"] = fb.estimated_time_remaining.sec
        if hasattr(fb, 'number_of_poses_remaining'):
            self._nav_feedback["poses_remaining"] = fb.number_of_poses_remaining

    def _report_feedback(self) -> None:
        """Report current nav2 status to Edge Core."""
        if self._nav_status == "idle":
            return

        data = {
            "goal_id": self._current_goal_id,
            "status": self._nav_status,
            "feedback": self._nav_feedback,
            "timestamp": time.time(),
        }
        self._http_post("/api/nav2/feedback", data)

    def _report_result(self, goal_id: str, status: str, message: str) -> None:
        """Report final result to Edge Core."""
        data = {
            "goal_id": goal_id,
            "status": status,
            "message": message,
            "timestamp": time.time(),
        }
        self._http_post("/api/nav2/result", data)


def main():
    parser = argparse.ArgumentParser(description="NOMAD Nav2 Goal Bridge")
    parser.add_argument("--host", default="172.17.0.1", help="Edge Core host")
    parser.add_argument("--port", type=int, default=8000, help="Edge Core port")
    parser.add_argument("--rate", type=float, default=2.0,
                        help="Goal poll rate in Hz (default: 2)")
    args = parser.parse_args()

    rclpy.init()
    node = Nav2GoalBridge(host=args.host, port=args.port, poll_rate_hz=args.rate)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
