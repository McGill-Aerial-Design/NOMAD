# SPDX-License-Identifier: Apache-2.0
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SIMULATION = ROOT / "scripts/simulation"
MODEL = SIMULATION / "models/tiltrotor_tricopter"
SCENARIOS = SIMULATION / "scenarios"


@pytest.fixture
def recipe(monkeypatch):
    monkeypatch.syspath_prepend(str(SIMULATION))
    import interactive

    return interactive.recipe


def load_model():
    return ET.parse(MODEL / "model.sdf").getroot().find("model")


def parameters(path):
    values = {}
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            name, value = line.split()
            values[name] = value
    return values


def test_model_is_a_complete_free_aircraft():
    model = load_model()
    assert model.get("name") == "tiltrotor_tricopter"
    assert model.findtext("static", "false") == "false"
    links = {link.get("name") for link in model.findall("link")}
    joints = {joint.get("name") for joint in model.findall("joint")}
    assert len(links) == 10 and len(joints) == 9
    assert all(
        joint.findtext("parent") != "world" and joint.findtext("child") != "world" for joint in model.findall("joint")
    )
    assert sum(float(link.findtext("inertial/mass")) for link in model.findall("link")) == pytest.approx(1.801)
    lift = [plugin for plugin in model.findall("plugin") if plugin.get("name") == "gz::sim::systems::LiftDrag"]
    assert len(lift) == 11
    assert all(plugin.findtext("link_name") in links for plugin in lift)
    assert all(
        plugin.findtext("control_joint_name") in joints for plugin in lift if plugin.findtext("control_joint_name")
    )
    sensor_pose = model.find("link[@name='imu_link']/sensor/pose")
    assert sensor_pose.get("degrees") == "true"
    assert sensor_pose.text == "0 0 0 180 0 0", "Plugin forwards IMU vectors directly in ArduPilot FRD axes"
    assert model.findtext("plugin[@name='ArduPilotPlugin']/imuName") == "imu_link::imu_sensor"


def test_propeller_blades_follow_each_spin_direction():
    model = load_model()
    joints = {joint.get("name"): joint for joint in model.findall("joint")}
    controls = {
        control.findtext("jointName"): float(control.findtext("multiplier"))
        for control in model.findall(".//plugin[@name='ArduPilotPlugin']/control")
    }
    lift = [plugin for plugin in model.findall("plugin") if plugin.get("name") == "gz::sim::systems::LiftDrag"]
    for joint_name, multiplier in controls.items():
        if "prop_joint" not in joint_name:
            continue
        sign = 1 if multiplier > 0 else -1
        link = joints[joint_name].findtext("child")
        blades = {
            float(plugin.findtext("cp").split()[0]): plugin.findtext("forward")
            for plugin in lift
            if plugin.findtext("link_name") == link
        }
        assert blades[-0.115] == f"0 {-sign} 0"
        assert blades[0.115] == f"0 {sign} 0"


def test_native_force_controllers_have_feedback_without_test_instrumentation():
    model = load_model()
    publisher = model.find("plugin[@name='gz::sim::systems::JointStatePublisher']")
    assert publisher is not None, "Native force PIDs require joint position/velocity feedback components"
    assert publisher.get("filename") == "gz-sim-joint-state-publisher-system"
    assert not publisher.findall("joint_name"), "Create feedback for every controlled joint"
    assert float(publisher.findtext("update_rate")) == 30, "Bound transport publication without throttling physics"


def test_wing_tail_and_tilt_controls_match_the_airframe():
    model = load_model()
    base = [
        plugin
        for plugin in model.findall("plugin")
        if plugin.get("name") == "gz::sim::systems::LiftDrag" and plugin.findtext("link_name") == "base_link"
    ]
    assert [plugin.findtext("cp") for plugin in base] == ["0.015 0 0.02", "-0.64 0 0.05"]
    assert model.findtext("link[@name='elevator']/pose") == "-0.655 0 0.005 0 0 0"
    controls = {
        control.findtext("jointName"): control
        for control in model.findall(".//plugin[@name='ArduPilotPlugin']/control")
    }
    left, right = controls["left_tilt_joint"], controls["right_tilt_joint"]
    assert (left.get("channel"), left.findtext("multiplier"), left.findtext("offset")) == (
        "5",
        "1.832596",
        "-0.142857",
    )
    assert (left.findtext("servo_min"), left.findtext("servo_max")) == ("920", "2060")
    assert (right.get("channel"), right.findtext("multiplier"), right.findtext("offset")) == (
        "7",
        "-1.832596",
        "-0.857143",
    )
    assert (right.findtext("servo_min"), right.findtext("servo_max")) == ("850", "2050")


def test_hover_and_transition_tilt_endpoints_are_neutral():
    model = load_model()
    controls = {
        control.findtext("jointName"): control
        for control in model.findall(".//plugin[@name='ArduPilotPlugin']/control")
    }
    for name, reversed_horn in (("left", False), ("right", True)):
        control = controls[name + "_tilt_joint"]
        low, high = float(control.findtext("servo_min")), float(control.findtext("servo_max"))
        multiplier, offset = float(control.findtext("multiplier")), float(control.findtext("offset"))
        for output, expected in (
            (15 / 105, 0.0),
            (0.0, math.radians(-15)),
            (1.0, math.radians(90)),
        ):
            pwm = high - output * (high - low) if reversed_horn else low + output * (high - low)
            raw = (pwm - low) / (high - low)
            assert multiplier * (raw + offset) == pytest.approx(expected, abs=1e-6)


@pytest.mark.parametrize("side", ["left", "right"])
def test_motor_housing_and_hub_follow_the_thrust_axis(side):
    model = load_model()
    nacelle = model.find(f"link[@name='{side}_nacelle']")
    propeller = model.find(f"link[@name='{side}_propeller']")
    pivot = list(map(float, nacelle.findtext("pose").split()))
    hub = list(map(float, propeller.findtext("pose").split()))
    offset = [hub[i] - pivot[i] for i in range(3)]
    assert offset == pytest.approx([0, 0, 0.05]), "Hub must sit on the nacelle's upward shaft"
    for kind, name in (("visual", "body"), ("collision", "body_collision")):
        body = nacelle.find(f"{kind}[@name='{name}']")
        pose = list(map(float, body.findtext("pose").split()))
        size = list(map(float, body.findtext("geometry/box/size").split()))
        assert pose[0:2] == [0, 0] and pose[3:] == [0, 0, 0]
        assert size[2] > max(size[:2]), "Motor housing must share the propeller shaft axis"
    joint = model.find(f"joint[@name='{side}_tilt_joint']")
    assert joint.findtext("axis/xyz") == "0 1 0"
    assert propeller.findtext("visual[@name='disc']/pose", "0 0 0 0 0 0") == "0 0 0 0 0 0"
    for angle, direction in ((0, [0, 0, 1]), (math.pi / 2, [1, 0, 0])):
        # Positive rotation about +Y must carry the shaft and hub from up to forward.
        thrust = [math.sin(angle), 0, math.cos(angle)]
        rotated_hub = [
            math.cos(angle) * offset[0] + math.sin(angle) * offset[2],
            offset[1],
            -math.sin(angle) * offset[0] + math.cos(angle) * offset[2],
        ]
        assert thrust == pytest.approx(direction, abs=1e-9)
        assert rotated_hub == pytest.approx([0.05 * v for v in direction], abs=1e-9)


def test_parm_selects_vectored_yaw_tricopter_and_servo_map():
    parm = parameters(SCENARIOS / "tiltrotor.parm")
    expected = {
        "Q_ENABLE": "1",
        "Q_FRAME_CLASS": "7",
        "Q_TILT_ENABLE": "1",
        "Q_TILT_MASK": "3",
        "Q_TILT_TYPE": "2",
        "Q_TILT_YAW_ANGLE": "15",
        "Q_ASSIST_SPEED": "8",
        "Q_GUIDED_MODE": "1",
        "Q_A_ANG_YAW_P": "2",
        "Q_A_RAT_YAW_P": "0.5",
        "Q_TRANS_DECEL": "0.7",
        "Q_BACKTRANS_MS": "10000",
        "WP_LOITER_RAD": "180",
        "SERVO1_FUNCTION": "33",
        "SERVO3_FUNCTION": "34",
        "SERVO4_FUNCTION": "36",
        "SERVO6_FUNCTION": "75",
        "SERVO8_FUNCTION": "76",
        "SERVO7_FUNCTION": "4",
        "SERVO9_FUNCTION": "4",
        "SERVO10_FUNCTION": "19",
    }
    assert {name: parm[name] for name in expected} == expected


def test_aileron_and_elevon_trims_match_the_model_neutral():
    model = load_model()
    parm = parameters(SCENARIOS / "tiltrotor.parm")
    controls = {
        control.findtext("jointName"): control
        for control in model.findall(".//plugin[@name='ArduPilotPlugin']/control")
    }
    for joint, servo in (
        ("left_aileron_joint", "SERVO7"),
        ("right_aileron_joint", "SERVO9"),
        ("elevator_joint", "SERVO10"),
    ):
        control = controls[joint]
        low, high = float(control.findtext("servo_min")), float(control.findtext("servo_max"))
        raw = (float(parm[servo + "_TRIM"]) - low) / (high - low)
        target = float(control.findtext("multiplier")) * (raw + float(control.findtext("offset")))
        assert abs(target) < 0.01, (joint, target)


@pytest.mark.parametrize("direction", [-1, 1])
@pytest.mark.parametrize(
    "surface, servo, lift_sign",
    [("left_aileron", "SERVO7", 1), ("right_aileron", "SERVO9", -1), ("elevator", "SERVO10", -1)],
)
def test_positive_roll_and_pitch_outputs_create_the_correct_lift(surface, servo, lift_sign, direction):
    model = load_model()
    control = model.find(f"plugin[@name='ArduPilotPlugin']/control[jointName='{surface}_joint']")
    aero = model.find(f"plugin[link_name='{surface}']")
    axis_y = float(model.findtext(f"joint[@name='{surface}_joint']/axis/xyz").split()[1])
    span = float(control.findtext("servo_max")) - float(control.findtext("servo_min"))
    angle_change = direction * 100 / span * float(control.findtext("multiplier"))
    # Rotation changes incidence as well as the explicit flap contribution in LiftDrag.
    lift_change = (
        -float(aero.findtext("cla")) * axis_y + float(aero.findtext("control_joint_rad_to_cl"))
    ) * angle_change
    assert lift_change * lift_sign * direction > 0, f"{servo} reverses the required {surface} lift response"


@pytest.mark.parametrize("surface", ["left_aileron", "right_aileron", "elevator"])
def test_surface_servo_damping_is_stable_at_the_flight_physics_step(surface):
    model = load_model()
    inertia = float(model.findtext(f"link[@name='{surface}']/inertial/inertia/iyy"))
    control = model.find(f"plugin[@name='ArduPilotPlugin']/control[jointName='{surface}_joint']")
    damping = float(control.findtext("d_gain")) + float(
        model.findtext(f"joint[@name='{surface}_joint']/axis/dynamics/damping")
    )
    step = 0.001
    # Semi-implicit hinge dynamics must keep both discrete poles inside the unit circle.
    stability = (step * step * float(control.findtext("p_gain")) + 2 * step * damping) / inertia
    assert stability < 4, f"{surface} derivative gain destabilizes the 1000 Hz hinge: {stability} >= 4"


@pytest.mark.parametrize(
    "name, wind",
    [("tiltrotor.json", [0, 0, 0]), ("tiltrotor-wind.json", [4, 0, 0])],
)
def test_tiltrotor_scenarios_select_plane_and_local_model(recipe, name, wind):
    path = SCENARIOS / name
    scenario = recipe(path)
    assert scenario["vehicle"] == "ArduPlane"
    assert scenario["wsl_graphics_adapter"] == "NVIDIA"
    assert scenario["wind"] == wind
    assert scenario["defaults_local"] == ["tiltrotor.parm"]
    assert (path.parent / scenario["aircraft_model_local"]).resolve(strict=True) == MODEL
    assert (path.parent / scenario["defaults_local"][0]).resolve(strict=True) == SCENARIOS / "tiltrotor.parm"
