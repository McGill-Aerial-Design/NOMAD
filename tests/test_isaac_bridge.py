# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
import sys
import unittest
from unittest.mock import MagicMock

# Mock rclpy before importing the bridge
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
sys.modules["rclpy.qos"] = MagicMock()
sys.modules["nav_msgs.msg"] = MagicMock()
sys.modules["vision_msgs.msg"] = MagicMock()

# Import the module under test
from edge_core.modules.slam.isaac import IsaacROSBridge


class TestIsaacROSBridge(unittest.TestCase):
    def setUp(self):
        self.bridge = IsaacROSBridge(
            vio_topic="/test/vio",
            detection_topic="/test/det",
        )

    def test_initial_state(self):
        status = self.bridge.get_status()
        self.assertFalse(status["running"])
        self.assertEqual(status["vio_count"], 0)
        self.assertEqual(status["detection_count"], 0)


if __name__ == "__main__":
    unittest.main()
