import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from fastapi import HTTPException

from edge_core.api_routes.task2_ops import normalize_vio_area_file_path_or_raise
from edge_core.api_routes.video_slam import build_mesh_websocket_delta
from edge_core.nav_controller import NavController
from edge_core.servo_controller import ServoController
from edge_core.spray_controller import SprayController


class TestVIOAreaPathValidation(unittest.TestCase):
    def test_relative_path_resolves_under_root(self):
        with tempfile.TemporaryDirectory() as root:
            path = normalize_vio_area_file_path_or_raise("maps/test.nvblx", root=root)
            self.assertTrue(path.startswith(root))
            self.assertEqual(Path(path).name, "test.nvblx")
            self.assertEqual(Path(path).parent.name, "maps")

    def test_path_escape_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaises(HTTPException) as ctx:
                normalize_vio_area_file_path_or_raise("../outside.nvblx", root=root)
            self.assertEqual(ctx.exception.status_code, 403)


class TestMeshDelta(unittest.TestCase):
    def test_second_voxel_frame_emits_changed_and_removed_only(self):
        first = {
            "mode": "voxel",
            "voxel_size": 0.5,
            "voxels": [
                {"p": [0.0, 0.0, 0.0], "c": [1, 2, 3]},
                {"p": [0.5, 0.0, 0.0], "c": [4, 5, 6]},
            ],
            "total_voxels": 2,
        }
        full, index, meta = build_mesh_websocket_delta(first)
        self.assertTrue(full["clear"])
        self.assertEqual(len(full["voxels"]), 2)

        second = {
            "mode": "voxel",
            "voxel_size": 0.5,
            "voxels": [
                {"p": [0.0, 0.0, 0.0], "c": [1, 2, 3]},
                {"p": [1.0, 0.0, 0.0], "c": [7, 8, 9]},
            ],
            "total_voxels": 2,
        }
        delta, _, _ = build_mesh_websocket_delta(second, index, meta)
        self.assertFalse(delta["clear"])
        self.assertEqual(delta["voxels"], [{"p": [1.0, 0.0, 0.0], "c": [7, 8, 9]}])
        self.assertEqual(delta["removed"], [{"x": 1, "y": 0, "z": 0}])


class TestSprayFailures(unittest.TestCase):
    def test_missing_servo_is_spray_failure(self):
        controller = SprayController(servo_controller=None)
        self.assertFalse(controller._spray_target())

    def test_failed_relay_trigger_is_spray_failure(self):
        class FakeServo:
            def configure_water_pump_relay(self, relay_number):
                return True

            def trigger_water_shooter(self, duration_ms):
                return False

        controller = SprayController(servo_controller=FakeServo())
        self.assertFalse(controller._spray_target())


class TestServoRelaySafety(unittest.TestCase):
    def test_failed_relay_on_attempts_best_effort_off(self):
        class FakeMavlink:
            def __init__(self):
                self.calls = []

            def set_relay(self, relay, enabled):
                self.calls.append((relay, enabled))
                return False

        mavlink = FakeMavlink()
        controller = ServoController()
        controller._mavlink_service = mavlink
        self.assertFalse(controller.trigger_water_shooter(duration_ms=50))
        self.assertEqual(mavlink.calls, [(0, True), (0, False)])


class TestNavPositionBounds(unittest.TestCase):
    def test_position_target_outside_bound_is_rejected(self):
        class FakeMavlink:
            def send_position_target(self, x, y, z, yaw):
                return True

        class FakeState:
            def get_state(self):
                return SimpleNamespace(armed=True, flight_mode="GUIDED")

        controller = NavController(FakeMavlink(), FakeState())
        controller.set_vio_state(confidence=1.0, healthy=True)
        self.assertFalse(controller.send_position(100.0, 0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
