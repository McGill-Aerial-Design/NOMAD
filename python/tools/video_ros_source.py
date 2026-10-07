# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""ROS 2 image subscription owned by the video bridge.

Everything that touches rclpy lives here, so the GStreamer pipeline can be read
and tested without ROS and the subscription can be restarted per topic switch.
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable
from typing import Any

__all__ = ["RosImageSource"]

logger = logging.getLogger("nomad.video_bridge.ros")

_NODE_NAME = "nomad_simple_video_bridge"
_EXECUTOR_JOIN_TIMEOUT_S = 2.0


def _load_ros():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Image

    return rclpy, Image, SingleThreadedExecutor


def _image_qos_profile():
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class RosImageSource:
    """Owns the node, subscription and executor for one image topic.

    The executor is owned rather than using rclpy.spin() so stop() can break the
    spin loop before the node is destroyed; rclpy.spin() would otherwise keep
    spinning a destroyed node on a topic switch.
    """

    def __init__(self, topic: str, on_image: Callable[[Any], None]) -> None:
        self._topic = topic
        self._on_image = on_image
        self._node: Any = None
        self._subscription: Any = None
        self._executor: Any = None
        self._thread: threading.Thread | None = None
        self._initialized_rclpy = False

    def start(self) -> bool:
        try:
            self._subscribe()
            return True
        except Exception as error:
            logger.error("Failed to start ROS image subscription: %s", error)
            return False

    def stop(self) -> None:
        self._join_executor()
        self._destroy_entities()
        self._shutdown_rclpy()

    def _subscribe(self) -> None:
        rclpy, message_type, executor_class = _load_ros()
        if not rclpy.ok():
            rclpy.init(args=None)
            self._initialized_rclpy = True
        self._node = rclpy.create_node(_NODE_NAME)
        self._subscription = self._node.create_subscription(
            message_type, self._topic, self._on_image, _image_qos_profile()
        )
        self._executor = executor_class()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        logger.info("Subscribed to ROS image topic: %s", self._topic)

    def _join_executor(self) -> None:
        if self._executor is not None:
            try:
                self._executor.shutdown()
            except Exception as error:
                logger.debug("Executor shutdown failed: %s", error)
        thread = self._thread
        self._thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=_EXECUTOR_JOIN_TIMEOUT_S)

    def _destroy_entities(self) -> None:
        self._remove_node()
        self._destroy_subscription()
        self._destroy_node()

    def _remove_node(self) -> None:
        try:
            if self._executor is not None and self._node is not None:
                self._executor.remove_node(self._node)
        except Exception as error:
            logger.debug("Executor remove_node failed: %s", error)
        self._executor = None

    def _destroy_subscription(self) -> None:
        try:
            if self._node is not None and self._subscription is not None:
                self._node.destroy_subscription(self._subscription)
        except Exception as error:
            logger.debug("Subscription destroy failed: %s", error)
        self._subscription = None

    def _destroy_node(self) -> None:
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception as error:
            logger.debug("Node destroy failed: %s", error)
        self._node = None

    def _shutdown_rclpy(self) -> None:
        if not self._initialized_rclpy:
            return
        try:
            rclpy, _message_type, _executor_class = _load_ros()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as error:
            logger.debug("rclpy shutdown failed: %s", error)
        self._initialized_rclpy = False
