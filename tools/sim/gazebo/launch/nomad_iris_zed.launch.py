# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""NOMAD integrated ArduPilot SITL + Gazebo Harmonic ZED-2i launch.

This launch starts the official ArduPilot Gazebo runway world, spawns an Iris
carrying a ZED-2i camera on a single pitch servo (the upstream 3-axis gimbal is
removed), starts ArduPilot SITL + MAVProxy + micro-ROS, and bridges Gazebo sensor
topics onto the ZED topic names consumed by ``edge_core.ros_http_bridge``.

The camera looks forward (+X) at pitch 0 and tilts up/down about the body Y axis;
the pitch joint is driven by ArduPilot SERVO14 (see ZED_PITCH_* below).

The model is spawned from an SDF file instead of via ``robot_description``.
``robot_state_publisher`` cannot parse the upstream nested gimbal model as URDF,
and a failed state publisher prevents the spawn action from receiving a model.
"""

from __future__ import annotations

import math
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NOMAD_IRIS_BASE_MODEL = "nomad_iris_with_standoffs"
NOMAD_IRIS_MODEL = "nomad_iris_zed"


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except ValueError:
        return default


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


def _is_truthy(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


ZED2I_HORIZONTAL_FOV_RAD = math.radians(110.0)
# 360p (640x360, 16:9) per side. The real ZED 2i is a stereo pair, so the model
# carries two cameras (left + right) separated by the 120 mm baseline.
ZED2I_IMAGE_WIDTH = _env_int("NOMAD_GAZEBO_CAMERA_WIDTH", 640)
ZED2I_IMAGE_HEIGHT = _env_int("NOMAD_GAZEBO_CAMERA_HEIGHT", 360)
ZED2I_UPDATE_RATE_HZ = _env_int("NOMAD_GAZEBO_CAMERA_FPS", 10)
ZED2I_BASELINE_M = _env_float("NOMAD_GAZEBO_CAMERA_BASELINE_M", 0.12)
# ZED 2i depth range (min ~0.3 m, usable to ~20 m). The depth camera is
# co-located with the LEFT lens so depth registers to the rgb/left frame, like
# the real ZED's depth_registered output.
ZED2I_DEPTH_NEAR_M = _env_float("NOMAD_GAZEBO_DEPTH_NEAR_M", 0.3)
ZED2I_DEPTH_FAR_M = _env_float("NOMAD_GAZEBO_DEPTH_FAR_M", 20.0)
GAZEBO_SPAWN_Z_M = _env_float("NOMAD_GAZEBO_SPAWN_Z", 0.45)
GAZEBO_CONTACT_TUNING = _is_truthy(os.environ.get("NOMAD_GAZEBO_APPLY_CONTACT_TUNING", "0"))
SITL_RATE_HZ = os.environ.get("NOMAD_SITL_RATE_HZ", "400").strip()
SITL_SPEEDUP = os.environ.get("NOMAD_SITL_SPEEDUP", "1").strip() or "1"
SITL_SYNTHETIC_CLOCK = os.environ.get("NOMAD_SITL_SYNTHETIC_CLOCK", "False").strip() or "False"

# ZED-2i camera on a single pitch servo (look up/down) — replaces the upstream
# 3-axis gimbal. The pitch joint is driven by ArduPilot SERVO14. The plugin's
# `channel` attribute is 0-indexed (channel 0 == SERVO1), so SERVO14 == channel
# 13. ArduPilot writes the servo PWM, the ArduPilotPlugin control (type=COMMAND)
# republishes it to the cmd topic, and a JointPositionController moves the joint —
# the same path the upstream gimbal used, reduced to one axis.
ZED_CAMERA_LINK = "zed_camera_link"
ZED_LEFT_SENSOR = "zed_left"
ZED_RIGHT_SENSOR = "zed_right"
ZED_DEPTH_SENSOR = "zed_depth"
ZED_PITCH_JOINT = "zed_pitch_joint"
ZED_PITCH_CMD_TOPIC = os.environ.get("NOMAD_GAZEBO_CAMERA_CMD_TOPIC", "/nomad/camera/cmd_pitch").strip()
# ArduPilot SERVO14 -> 0-indexed plugin channel 13.
ZED_PITCH_SERVO_CHANNEL = _env_int("NOMAD_GAZEBO_CAMERA_SERVO_CHANNEL", 13)
ZED_PITCH_LIMIT_RAD = math.radians(_env_float("NOMAD_GAZEBO_CAMERA_PITCH_LIMIT_DEG", 70.0))
# Mount pose relative to the iris base_link: forward of the body, level (the
# camera looks down +X — forward — at pitch 0, i.e. aligned with the airframe).
ZED_MOUNT_POSE = os.environ.get("NOMAD_GAZEBO_CAMERA_MOUNT_POSE", "0.12 0 0.0 0 0 0").strip()

_PHYSICS_DEFAULTS = {
    "max_step_size": os.environ.get("NOMAD_GAZEBO_MAX_STEP_SIZE", "0.001"),
    "real_time_update_rate": os.environ.get("NOMAD_GAZEBO_REAL_TIME_UPDATE_RATE", "1000"),
    "contact_max_correcting_vel": os.environ.get("NOMAD_GAZEBO_CONTACT_MAX_CORRECTING_VEL", "5"),
    "contact_surface_layer": os.environ.get("NOMAD_GAZEBO_CONTACT_SURFACE_LAYER", "0.001"),
}
_CONTACT_DEFAULTS = {
    "restitution_coefficient": os.environ.get("NOMAD_GAZEBO_RESTITUTION_COEFFICIENT", "0"),
    "max_vel": os.environ.get("NOMAD_GAZEBO_COLLISION_MAX_VEL", "5"),
    "min_depth": os.environ.get("NOMAD_GAZEBO_COLLISION_MIN_DEPTH", "0.001"),
    "kp": os.environ.get("NOMAD_GAZEBO_COLLISION_KP", "1e8"),
    "kd": os.environ.get("NOMAD_GAZEBO_COLLISION_KD", "1e4"),
}


def _set_text(root: ET.Element, path: str, value: str) -> None:
    element = root.find(path)
    if element is None:
        raise RuntimeError(f"Expected SDF element missing: {path}")
    element.text = value


def _child(parent: ET.Element, name: str) -> ET.Element:
    element = parent.find(name)
    if element is None:
        element = ET.SubElement(parent, name)
    return element


def _set_nested_text(parent: ET.Element, path: tuple[str, ...], value: str) -> None:
    element = parent
    for name in path:
        element = _child(element, name)
    element.text = value


def _apply_contact_defaults(root: ET.Element) -> None:
    if not GAZEBO_CONTACT_TUNING:
        return
    for collision in root.findall(".//collision"):
        _set_nested_text(
            collision,
            ("surface", "bounce", "restitution_coefficient"),
            _CONTACT_DEFAULTS["restitution_coefficient"],
        )
        for tag in ("max_vel", "min_depth", "kp", "kd"):
            _set_nested_text(collision, ("surface", "contact", "ode", tag), _CONTACT_DEFAULTS[tag])


def _apply_world_physics(world: ET.Element) -> None:
    physics = world.find("physics")
    if physics is None:
        physics = ET.SubElement(world, "physics", {"name": "nomad_stable_physics"})
    elif not physics.get("name"):
        physics.set("name", "nomad_stable_physics")

    _set_nested_text(physics, ("max_step_size",), _PHYSICS_DEFAULTS["max_step_size"])
    _set_nested_text(physics, ("real_time_update_rate",), _PHYSICS_DEFAULTS["real_time_update_rate"])
    if GAZEBO_CONTACT_TUNING:
        for tag in ("contact_max_correcting_vel", "contact_surface_layer"):
            _set_nested_text(physics, ("ode", "constraints", tag), _PHYSICS_DEFAULTS[tag])


def _prepend_env_path(name: str, path: Path) -> None:
    current = os.environ.get(name, "")
    parts = [str(path)]
    if current:
        parts.append(current)
    os.environ[name] = ":".join(parts)


def _write_model_config(model_dir: Path, model_name: str, description: str) -> None:
    model_dir.mkdir(parents=True, exist_ok=True)
    (model_dir / "model.config").write_text(
        "\n".join(
            [
                '<?xml version="1.0"?>',
                "<model>",
                f"  <name>{model_name}</name>",
                "  <version>1.0</version>",
                '  <sdf version="1.9">model.sdf</sdf>',
                "  <author>",
                "    <name>The NOMAD Authors</name>",
                "  </author>",
                f"  <description>{description}</description>",
                "</model>",
                "",
            ]
        ),
        encoding="utf-8",
    )


def _write_tree(tree: ET.ElementTree, path: Path) -> None:
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def _rewrite_ardupilot_package_uris(root: ET.Element, pkg_ardupilot_gazebo: Path) -> None:
    package_prefix = "package://ardupilot_gazebo/"
    for uri in root.findall(".//uri"):
        text = uri.text or ""
        if text.startswith(package_prefix):
            uri.text = (pkg_ardupilot_gazebo / text.removeprefix(package_prefix)).as_uri()


def _build_nomad_iris_base_model(pkg_ardupilot_gazebo: Path, temp_root: Path) -> None:
    source = pkg_ardupilot_gazebo / "models" / "iris_with_standoffs" / "model.sdf"
    model_dir = temp_root / NOMAD_IRIS_BASE_MODEL
    _write_model_config(model_dir, NOMAD_IRIS_BASE_MODEL, "ArduPilot Iris base with file URI mesh references.")

    tree = ET.parse(source)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise RuntimeError(f"Expected model element missing in {source}")
    model.set("name", NOMAD_IRIS_BASE_MODEL)
    _rewrite_ardupilot_package_uris(root, pkg_ardupilot_gazebo)
    _apply_contact_defaults(root)
    _write_tree(tree, model_dir / "model.sdf")


def _zed_stereo_sensor_xml(name: str, y_offset: float) -> str:
    """One ZED stereo camera (forward-looking, +X optical axis at pitch 0)."""
    return f"""
      <sensor name="{name}" type="camera">
        <pose>0 {y_offset:.5f} 0 0 0 0</pose>
        <always_on>1</always_on>
        <update_rate>{ZED2I_UPDATE_RATE_HZ}</update_rate>
        <visualize>0</visualize>
        <camera>
          <horizontal_fov>{ZED2I_HORIZONTAL_FOV_RAD:.12f}</horizontal_fov>
          <image>
            <width>{ZED2I_IMAGE_WIDTH}</width>
            <height>{ZED2I_IMAGE_HEIGHT}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.1</near><far>50</far></clip>
        </camera>
      </sensor>"""


def _zed_depth_sensor_xml(name: str, y_offset: float) -> str:
    """A gz depth camera (ground-truth depth + point cloud, registered to LEFT)."""
    return f"""
      <sensor name="{name}" type="depth_camera">
        <pose>0 {y_offset:.5f} 0 0 0 0</pose>
        <always_on>1</always_on>
        <update_rate>{ZED2I_UPDATE_RATE_HZ}</update_rate>
        <visualize>0</visualize>
        <camera>
          <horizontal_fov>{ZED2I_HORIZONTAL_FOV_RAD:.12f}</horizontal_fov>
          <image>
            <width>{ZED2I_IMAGE_WIDTH}</width>
            <height>{ZED2I_IMAGE_HEIGHT}</height>
            <format>R_FLOAT32</format>
          </image>
          <clip><near>{ZED2I_DEPTH_NEAR_M}</near><far>{ZED2I_DEPTH_FAR_M}</far></clip>
        </camera>
      </sensor>"""


def _zed_camera_link_element() -> ET.Element:
    """ZED-2i stereo link: left + right RGB (120 mm baseline) + a depth camera.

    The ZED SDK cannot consume Gazebo frames, so depth is produced by a gz depth
    camera (ground truth) co-located with the LEFT lens, mirroring the ZED's
    left-registered ``depth_registered`` output for the dev environment.
    """
    half_baseline = ZED2I_BASELINE_M / 2.0
    depth_xml = _zed_depth_sensor_xml(ZED_DEPTH_SENSOR, half_baseline)
    xml = f"""
    <link name="{ZED_CAMERA_LINK}">
      <pose relative_to="{ZED_PITCH_JOINT}">0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>1e-05</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-05</iyy><iyz>0</iyz><izz>1e-05</izz>
        </inertia>
      </inertial>
      <visual name="zed_visual">
        <!-- ZED 2i enclosure: 175 mm (Y) x 30 mm (X) x 33 mm (Z). -->
        <geometry><box><size>0.030 0.175 0.033</size></box></geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
      {_zed_stereo_sensor_xml(ZED_LEFT_SENSOR, half_baseline)}
      {_zed_stereo_sensor_xml(ZED_RIGHT_SENSOR, -half_baseline)}
      {depth_xml}
    </link>
    """
    return ET.fromstring(xml)


def _zed_pitch_joint_element() -> ET.Element:
    """A single revolute pitch joint (axis Y) so the ZED can look up/down."""
    limit = ZED_PITCH_LIMIT_RAD
    xml = f"""
    <joint name="{ZED_PITCH_JOINT}" type="revolute">
      <pose relative_to="base_link">{ZED_MOUNT_POSE}</pose>
      <parent>base_link</parent>
      <child>{ZED_CAMERA_LINK}</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>{-limit:.6f}</lower><upper>{limit:.6f}</upper></limit>
        <dynamics><damping>0.2</damping></dynamics>
      </axis>
    </joint>
    """
    return ET.fromstring(xml)


def _build_nomad_iris_model(pkg_ardupilot_gazebo: Path, temp_root: Path) -> Path:
    source = pkg_ardupilot_gazebo / "models" / "iris_with_gimbal" / "model.sdf"
    model_dir = temp_root / NOMAD_IRIS_MODEL
    _write_model_config(
        model_dir, NOMAD_IRIS_MODEL, "ArduPilot Iris with a NOMAD ZED-2i camera on a pitch servo (no gimbal)."
    )

    tree = ET.parse(source)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise RuntimeError(f"Expected model element missing in {source}")
    model.set("name", NOMAD_IRIS_MODEL)

    # Flight body -> our file-URI base model; fully drop the gimbal include.
    for include in list(model.findall("include")):
        include_name = include.findtext("name")
        if include_name == "iris":
            _set_text(include, "uri", f"model://{NOMAD_IRIS_BASE_MODEL}")
        elif include_name == "gimbal":
            model.remove(include)

    # Drop the joint that attached the gimbal to the airframe.
    for joint in list(model.findall("joint")):
        if joint.get("name") == "gimbal_joint":
            model.remove(joint)

    # Reduce the ArduPilot gimbal controls to a single pitch servo (SERVO14).
    ardupilot = model.find(".//plugin[@name='ArduPilotPlugin']")
    if ardupilot is None:
        raise RuntimeError(f"ArduPilotPlugin missing in {source}")
    for control in list(ardupilot.findall("control")):
        joint_name = control.findtext("jointName")
        if joint_name in {"roll_joint", "yaw_joint"}:
            ardupilot.remove(control)
        elif joint_name == "pitch_joint":
            control.set("channel", str(ZED_PITCH_SERVO_CHANNEL))
            _set_text(control, "jointName", ZED_PITCH_JOINT)
            _set_text(control, "cmd_topic", ZED_PITCH_CMD_TOPIC)
            # angle = (servo_input + offset) * multiplier; input in [0, 1], so
            # offset -0.5 + multiplier 2*limit maps PWM mid (1500) -> 0 (level).
            _set_text(control, "multiplier", f"{2.0 * ZED_PITCH_LIMIT_RAD:.6f}")
            _set_text(control, "offset", "-0.5")
    _set_text(ardupilot, "fdm_addr", "127.0.0.1")

    # Keep only the pitch JointPositionController, rewired to the ZED joint.
    for plugin in list(model.findall("plugin")):
        if plugin.get("filename") == "gz-sim-joint-position-controller-system":
            joint_name = plugin.findtext("joint_name")
            if joint_name in {"roll_joint", "yaw_joint"}:
                model.remove(plugin)
            elif joint_name == "pitch_joint":
                _set_text(plugin, "joint_name", ZED_PITCH_JOINT)
                _set_text(plugin, "topic", ZED_PITCH_CMD_TOPIC)

    # Insert the ZED camera link + pitch joint ahead of the plugins.
    first_plugin = model.find("plugin")
    insert_at = list(model).index(first_plugin) if first_plugin is not None else len(model)
    model.insert(insert_at, _zed_pitch_joint_element())
    model.insert(insert_at, _zed_camera_link_element())

    _rewrite_ardupilot_package_uris(root, pkg_ardupilot_gazebo)
    _apply_contact_defaults(root)
    _write_tree(tree, model_dir / "model.sdf")
    return model_dir / "model.sdf"


def _prepare_world(source: Path) -> Path:
    temp_root = Path(tempfile.mkdtemp(prefix="nomad_gazebo_world_"))
    world_dir = temp_root / "worlds"
    world_dir.mkdir(parents=True, exist_ok=True)
    output = world_dir / source.name

    tree = ET.parse(source)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError(f"Expected world element missing in {source}")
    _apply_world_physics(world)
    _apply_contact_defaults(world)
    _write_tree(tree, output)
    return output


def _prepare_nomad_model(pkg_ardupilot_gazebo: Path) -> Path:
    temp_root = Path(tempfile.mkdtemp(prefix="nomad_gazebo_models_"))
    _build_nomad_iris_base_model(pkg_ardupilot_gazebo, temp_root)
    model_sdf = _build_nomad_iris_model(pkg_ardupilot_gazebo, temp_root)

    _prepend_env_path("GZ_SIM_RESOURCE_PATH", temp_root)
    _prepend_env_path("GZ_SIM_RESOURCE_PATH", pkg_ardupilot_gazebo)
    _prepend_env_path("GZ_SIM_RESOURCE_PATH", pkg_ardupilot_gazebo / "models")
    _prepend_env_path("SDF_PATH", temp_root)
    _prepend_env_path("SDF_PATH", pkg_ardupilot_gazebo)
    _prepend_env_path("SDF_PATH", pkg_ardupilot_gazebo / "models")
    return model_sdf


def _default_param_files(pkg_ardupilot_sitl: Path, pkg_ardupilot_gazebo: Path) -> str:
    params = [
        pkg_ardupilot_sitl / "config" / "default_params" / "copter.parm",
        pkg_ardupilot_gazebo / "config" / "gazebo-iris-gimbal.parm",
        pkg_ardupilot_sitl / "config" / "default_params" / "dds_udp.parm",
        pkg_ardupilot_sitl / "config" / "default_params" / "dds_use_ns.parm",
    ]
    return ",".join(str(path) for path in params)


def _gz_sim_actions(context, pkg_ros_gz_sim: Path, world_path: Path) -> list[IncludeLaunchDescription]:
    headless = _is_truthy(context.perform_substitution(LaunchConfiguration("headless")))
    render_args = "-s -r --headless-rendering" if headless else "-r"
    verbosity = os.environ.get("NOMAD_GAZEBO_VERBOSITY", "2").strip()
    if not verbosity.isdigit():
        verbosity = "2"
    gz_args = f"-v{verbosity} {render_args} {world_path}"

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg_ros_gz_sim / "launch" / "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args}.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_ardupilot_sitl = Path(get_package_share_directory("ardupilot_sitl"))
    pkg_gazebo = Path(get_package_share_directory("ardupilot_gazebo"))
    pkg_gz_worlds = Path(get_package_share_directory("ardupilot_gz_gazebo"))
    pkg_ros_gz_sim = Path(get_package_share_directory("ros_gz_sim"))

    model_sdf = _prepare_nomad_model(pkg_gazebo)
    world_path = _prepare_world(pkg_gz_worlds / "worlds" / "runway.sdf")
    launch_dir = Path(__file__).resolve().parent
    bridge_config = launch_dir.parent / "config" / "nomad_zed_bridge.yaml"

    gz_sim_server = OpaqueFunction(function=_gz_sim_actions, args=[pkg_ros_gz_sim, world_path])

    spawn_iris = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            "runway",
            "-file",
            str(model_sdf),
            "-name",
            "iris",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            f"{GAZEBO_SPAWN_Z_M:.3f}",
        ],
        output="screen",
    )

    sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_ardupilot_sitl / "launch" / "sitl_dds_udp.launch.py")),
        launch_arguments={
            "command": "arducopter",
            "model": "json",
            "speedup": SITL_SPEEDUP,
            "slave": "0",
            "instance": "0",
            "sysid": "1",
            "wipe": "False",
            "synthetic_clock": SITL_SYNTHETIC_CLOCK,
            "rate": SITL_RATE_HZ,
            "sim_address": "127.0.0.1",
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
            "out": LaunchConfiguration("mavlink_out"),
            "defaults": _default_param_files(pkg_ardupilot_sitl, pkg_gazebo),
            "use_instance_dir": "False",
            "micro_ros_agent_ns": "iris",
            "transport": "udp4",
            "port": "2019",
        }.items(),
    )

    nomad_zed_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="nomad_zed_bridge",
        parameters=[{"config_file": str(bridge_config)}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mavlink_out",
                default_value=os.environ.get("NOMAD_GAZEBO_MAVLINK_OUT", "127.0.0.1:14550"),
                description="MAVProxy --out target. May include extra --out entries for local dev compose.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value=os.environ.get("NOMAD_GAZEBO_HEADLESS", "0"),
                description="Run gz sim with --headless-rendering instead of the GUI.",
            ),
            gz_sim_server,
            TimerAction(period=3.0, actions=[spawn_iris]),
            sitl_dds,
            nomad_zed_bridge,
        ]
    )
