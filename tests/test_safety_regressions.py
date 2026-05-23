import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from fastapi import HTTPException

from edge_core.api_routes.task2_ops import normalize_vio_area_file_path_or_raise
from edge_core.api_routes.video_slam import build_mesh_websocket_delta
from edge_core.nav_controller import NavController, NavMode
from edge_core.servo_controller import ServoController
from edge_core.spray_controller import SprayController
from edge_core.state import StateManager


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
        controller._update_status(mode=NavMode.STANDBY)
        controller.set_vio_state(confidence=1.0, healthy=True)
        self.assertFalse(controller.send_position(100.0, 0.0, 0.0, 0.0))

    def test_stale_vio_rejects_velocity(self):
        class FakeMavlink:
            def __init__(self):
                self.velocity_calls = []

            def send_velocity_command(self, *args):
                self.velocity_calls.append(args)
                return True

        class FakeState:
            def get_state(self):
                return SimpleNamespace(armed=True, flight_mode="GUIDED")

        mavlink = FakeMavlink()
        controller = NavController(mavlink, FakeState())
        controller._update_status(mode=NavMode.STANDBY)
        controller._vio_max_age_s = 0.1
        controller.set_vio_state(
            confidence=1.0,
            healthy=True,
            received_monotonic=time.monotonic() - 1.0,
        )

        self.assertFalse(controller.send_velocity(0.1, 0.0, 0.0, 0.0))
        self.assertEqual(mavlink.velocity_calls, [])


class TestGuidedRthLanding(unittest.TestCase):
    class FakeSystemState(SimpleNamespace):
        def has_valid_gps(self):
            return self.gps_fix and self.gps_lat is not None and self.gps_lon is not None

    class FakeStateManager:
        def __init__(self):
            self.state = TestGuidedRthLanding.FakeSystemState(
                connected=True,
                armed=True,
                flight_mode="GUIDED",
                gps_fix=True,
                gps_lat=43.0,
                gps_lon=-79.0,
                gps_alt=110.0,
                alt_agl_m=10.0,
                home_lat=43.0001,
                home_lon=-79.0001,
                home_alt=100.0,
            )

        def get_state(self):
            return self.state

    class FakeMavlink:
        def __init__(self):
            self.global_targets = []
            self.land_calls = 0

        def request_home_position(self):
            return True

        def send_global_position_target(self, lat, lon, alt_msl, yaw=None):
            self.global_targets.append((lat, lon, alt_msl, yaw))
            return True

        def send_velocity_command(self, *args):
            return True

        def land(self):
            self.land_calls += 1
            return True

    def test_rth_plan_waits_for_operator_approval(self):
        mavlink = self.FakeMavlink()
        controller = NavController(mavlink, self.FakeStateManager())

        result = controller.start_rth_landing(climb_alt_m=30.0)

        self.assertTrue(result["success"])
        self.assertTrue(result["awaiting_approval"])
        self.assertEqual(result["next_phase"], "climb")
        self.assertEqual(mavlink.global_targets, [])
        self.assertEqual(mavlink.land_calls, 0)

    def test_rth_climb_uses_altitude_above_home_not_raw_msl(self):
        mavlink = self.FakeMavlink()
        controller = NavController(mavlink, self.FakeStateManager())
        controller.start_rth_landing(climb_alt_m=30.0)

        result = controller.approve_rth_landing_phase()

        self.assertTrue(result["success"])
        self.assertEqual(result["phase"], "climb")
        self.assertEqual(mavlink.global_targets[-1][2], 130.0)
        self.assertEqual(mavlink.land_calls, 0)


class TestStateManagerBatching(unittest.TestCase):
    def test_get_state_flushes_pending_update_after_interval(self):
        original_interval = StateManager.MODEL_UPDATE_INTERVAL
        StateManager.MODEL_UPDATE_INTERVAL = 0.01
        try:
            manager = StateManager()
            manager.update_state(battery_voltage=11.1)
            self.assertEqual(manager.get_state().battery_voltage, 11.1)

            manager.update_state(battery_voltage=12.2)
            time.sleep(0.02)

            self.assertEqual(manager.get_state().battery_voltage, 12.2)
        finally:
            StateManager.MODEL_UPDATE_INTERVAL = original_interval


if __name__ == "__main__":
    unittest.main()
