# SPDX-License-Identifier: Apache-2.0
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


@pytest.fixture
def launcher(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts/simulation"))
    import interactive
    import interactive_world

    return interactive, interactive_world


def test_custom_recipe_and_frozen_identity(launcher, tmp_path):
    interactive, _ = launcher
    path = tmp_path / "my scenario.json"
    data = {
        "schema": 1,
        "name": "My own aircraft",
        "backend": "sitl",
        "frame": "quad",
        "home": [44, -75, 123, 37],
        "system_id": 87,
        "defaults": ["my parameters.parm"],
    }
    path.write_text(json.dumps(data))
    loaded = interactive.recipe(path)
    assert loaded["name"] == data["name"] and loaded["home"] == data["home"] and loaded["system_id"] == 87
    assert loaded["_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    path.write_text('{"schema":1,"schema":1}')
    with pytest.raises(ValueError, match="Duplicate"):
        interactive.recipe(path)


@pytest.mark.parametrize(
    "home", [[91, 0, 0, 0], [0, 181, 0, 0], [0, 0, math.nan, 0], [True, 0, 0, 0], [0, 0, 0, 361], [0, 0, 0], None]
)
def test_bad_home_cannot_launch(launcher, home):
    with pytest.raises(ValueError):
        launcher[0].check_home(home)


@pytest.mark.parametrize(
    "extra",
    [
        {"defaults_local": "file.parm"},
        {"defaults_local": [None]},
        {"seed": True},
        {"seed": -1},
        {"wsl_graphics_adapter": None},
        {"wsl_graphics_adapter": "GPU\nunsafe"},
        {"wsl_graphics_adapter": "GPU"},
        {"physics_hz": 400},
    ],
)
def test_bad_launch_fields(launcher, tmp_path, extra):
    path = tmp_path / "scenario.json"
    path.write_text(json.dumps({"schema": 1, "name": "Custom", "backend": "sitl", "frame": "quad", **extra}))
    with pytest.raises(ValueError):
        launcher[0].recipe(path)


def test_renderer_adapter_is_optional_and_configurable(launcher, tmp_path):
    path = tmp_path / "custom.json"
    data = {"schema": 1, "name": "Custom GPU", "backend": "gazebo", "frame": "JSON:127.0.0.1"}
    path.write_text(json.dumps(data))
    assert "wsl_graphics_adapter" not in launcher[0].recipe(path)
    data["wsl_graphics_adapter"] = "User selected GPU"
    path.write_text(json.dumps(data))
    assert launcher[0].recipe(path)["wsl_graphics_adapter"] == "User selected GPU"


@pytest.mark.parametrize("rate", [None, True, 0, 500, 400.0, "400"])
def test_bad_physics_rate_cannot_launch(launcher, tmp_path, rate):
    path = tmp_path / "scenario.json"
    path.write_text(
        json.dumps({"schema": 1, "name": "Custom", "backend": "gazebo", "frame": "JSON", "physics_hz": rate})
    )
    with pytest.raises(ValueError, match="physics_hz"):
        launcher[0].recipe(path)


@pytest.mark.parametrize(
    "camera",
    [
        None,
        {},
        {"width": 1920, "height": 1080, "fps": 30, "typo": 1},
        {"width": 1921, "height": 1080, "fps": 30},
        {"width": 1920, "height": 1079, "fps": 30},
        {"width": True, "height": 1080, "fps": 30},
        {"width": 1920, "height": 1080, "fps": 0},
        {"width": 1920, "height": 1080, "fps": 31},
        {"width": 1920, "height": 1080, "fps": 30.0},
        {"width": 3840, "height": 1080, "fps": 30},
        {"width": 1920, "height": 2160, "fps": 30},
        {"width": 1920, "height": 1080, "fps": 30, "use_cuda": "true"},
        {"width": 1920, "height": 1080, "fps": 30, "bitrate_kbps": 99},
        {"width": 1920, "height": 1080, "fps": 30, "bitrate_kbps": 50001},
        {"width": 1920, "height": 1080, "fps": 30, "bitrate_kbps": 8000.0},
        {"width": 1920, "height": 1080, "fps": 30, "bitrate_kbps": True},
    ],
)
def test_bad_camera_override_cannot_launch(launcher, tmp_path, camera):
    path = tmp_path / "scenario.json"
    path.write_text(
        json.dumps(
            {
                "schema": 1,
                "name": "Custom",
                "backend": "gazebo",
                "frame": "JSON",
                "camera_model": "my-camera",
                "camera": camera,
            }
        )
    )
    with pytest.raises(ValueError, match="camera"):
        launcher[0].recipe(path)


def test_camera_override_requires_rendered_camera_model(launcher, tmp_path):
    path = tmp_path / "scenario.json"
    data = {
        "schema": 1,
        "name": "Custom",
        "backend": "gazebo",
        "frame": "JSON",
        "camera": {"width": 1920, "height": 1080, "fps": 30},
    }
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="camera"):
        launcher[0].recipe(path)
    data.update(backend="sitl", camera_model="my-camera")
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="camera"):
        launcher[0].recipe(path)


def test_last_letter_home_copy_does_not_edit_upstream(launcher, tmp_path):
    interactive, _ = launcher
    source = tmp_path / "upstream models"
    model = source / "aircraft/custom_quad"
    model.mkdir(parents=True)
    initial = model / "init.yaml"
    initial.write_text("# source\ncoordinates: [1, 2, 3] # origin\norientation: [0, 0, 0, 1]\nposition: [0, 0, -.17]\n")
    before = initial.read_bytes()
    copied = interactive.last_letter_models(source, tmp_path / "owned copy", "custom_quad", [44, -75, 999, 90])
    text = (copied / "aircraft/custom_quad/init.yaml").read_text()
    assert "coordinates: [44, -75, 999] # origin" in text
    orientation = json.loads(
        next(line for line in text.splitlines() if line.startswith("orientation:")).split(":", 1)[1]
    )
    assert orientation == pytest.approx([0, 0, math.sqrt(0.5), math.sqrt(0.5)])
    assert "position: [0, 0, -.17]" in text and initial.read_bytes() == before
    with pytest.raises(ValueError, match="model name"):
        interactive.last_letter_models(source, tmp_path / "escape", "../other", [0, 0, 0, 0])


@pytest.mark.parametrize("policy", [None, "true", "false"])
@pytest.mark.parametrize("physics_hz", [None, 400])
@pytest.mark.parametrize(
    "camera_override",
    [
        None,
        {"width": 1920, "height": 1080, "fps": 30},
        {"width": 1920, "height": 1080, "fps": 30, "use_cuda": True, "bitrate_kbps": 8000},
    ],
)
def test_world_is_scenery_plus_one_free_configured_aircraft(
    launcher, tmp_path, monkeypatch, policy, physics_hz, camera_override
):
    _, builder = launcher
    workspace = tmp_path / "workspace"
    vehicle = workspace / "sources/models/custom_aircraft"
    camera = workspace / "sources/models/custom_camera"
    vehicle.mkdir(parents=True)
    camera.mkdir()
    vehicle_file = vehicle / "model.sdf"
    vehicle_file.write_text("""<sdf version="1.9"><model name="custom_aircraft"><link name="body"/>
      <plugin name="ArduPilotPlugin"><fdm_port_in>9002</fdm_port_in></plugin>
      <plugin name="gz::sim::systems::JointPositionController"/>
      <plugin name="gz::sim::systems::JointPositionController"/>
      <plugin name="gz::sim::systems::JointPositionController"/>
    </model></sdf>""")
    camera_file = camera / "model.sdf"
    camera_file.write_text("""<sdf version="1.9"><model name="custom_camera"><link name="camera">
      <sensor name="rgb" type="camera"><camera><horizontal_fov>1.1</horizontal_fov></camera>
      <plugin name="GstCameraPlugin"><udp_port>5600</udp_port></plugin></sensor>
    </link></model></sdf>""")
    source = tmp_path / "my field.sdf"
    source.write_text("""<sdf version="1.9"><world name="example"><physics name="old" type="ignore">
      <max_step_size>.01</max_step_size></physics><spherical_coordinates><latitude_deg>44</latitude_deg>
      <longitude_deg>-75</longitude_deg><elevation>123</elevation></spherical_coordinates>
      <include><uri>model://custom_aircraft</uri><name>another_name</name><pose>5 6 .3 0 0 .2</pose></include>
    </world></sdf>""")
    if policy is not None:
        source.write_text(
            source.read_text()
            .replace('<sdf version="1.9">', '<sdf version="1.9" xmlns:gz="urn:custom-simulator">')
            .replace(
                "</world>",
                "<gz:policies><parallel_postupdates>" + policy + "</parallel_postupdates></gz:policies></world>",
            )
        )
    before = [p.read_bytes() for p in (vehicle_file, camera_file, source)]
    monkeypatch.setattr(builder, "component", lambda *_: {"binary": str(workspace / "build/libArduPilotPlugin.so")})
    run = tmp_path / "run"
    run.mkdir()
    data = {"aircraft_model": str(vehicle), "camera_model": str(camera), "world": str(source), "home": [1, 2, 3, 0]}
    if physics_hz is not None:
        data["physics_hz"] = physics_hz
    if camera_override is not None:
        data["camera"] = camera_override
        path = tmp_path / "checked-recipe.json"
        path.write_text(json.dumps({"schema": 1, "name": "Camera", "backend": "gazebo", "frame": "JSON", **data}))
        data = launcher[0].recipe(path)
    info = builder.create(workspace, run, data, tmp_path / "config.json", 12345, 23456)
    world = ET.parse(info["world"]).getroot().find("world")
    assert info["home"] == [44, -75, 123, 0] and not info["body_fixed"]
    assert len(world.findall("include")) == 1 and world.findtext("include/pose") == "5.0 6.0 0.3 0.0 0.0 0.2"
    assert float(world.findtext("physics/max_step_size")) == 1 / (physics_hz or 1000)
    assert info["physics_hz"] == (physics_hz or 1000)
    assert world.findtext("{urn:custom-simulator}policies/parallel_postupdates") == policy
    if policy is not None:
        assert "<gz:policies>" in Path(info["world"]).read_text()
    cloned = ET.parse(run / "models/custom_aircraft/model.sdf")
    assert cloned.findtext(".//fdm_port_in") == "12345"
    assert len(cloned.findall(".//plugin[@name='gz::sim::systems::JointPositionController']")) == 3
    assert not cloned.findall(".//plugin[@name='NomadTestSystem']")
    assert ET.parse(run / "models/custom_camera/model.sdf").findtext(".//udp_port") == "23456"
    if camera_override is not None:
        sensor = ET.parse(run / "models/custom_camera/model.sdf").find(".//sensor")
        assert sensor.findtext("camera/image/width") == "1920"
        assert sensor.findtext("camera/image/height") == "1080"
        assert sensor.findtext("update_rate") == "30"
        assert sensor.findtext("camera/horizontal_fov") == "1.1"
        assert info["camera"]["width"] == 1920 and info["camera"]["height"] == 1080
        assert info["camera"]["fps"] == 30
        assert info["camera"]["use_cuda"] == camera_override.get("use_cuda", False)
        assert info["camera"]["bitrate_kbps"] == camera_override.get("bitrate_kbps", 800)
        assert sensor.findtext("plugin/bitrate") == (
            str(camera_override["bitrate_kbps"]) if "bitrate_kbps" in camera_override else None
        )
    assert [p.read_bytes() for p in (vehicle_file, camera_file, source)] == before
    vehicle_file.write_text(
        vehicle_file.read_text().replace('<link name="body"/>', '<static>true</static><link name="body"/>')
    )
    invalid = tmp_path / "invalid"
    invalid.mkdir()
    with pytest.raises(ValueError, match="free aircraft"):
        builder.create(workspace, invalid, data, tmp_path / "config.json", 12345, 23456)


def test_port_allocation_checks_both_protocols(launcher):
    import socket

    for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
        port = launcher[0].port(kind)
        with socket.socket(socket.AF_INET, kind) as listener:
            listener.bind(("127.0.0.1", port))


def test_generated_natural_scene_uses_paths_with_spaces(launcher, tmp_path, monkeypatch):
    _, builder = launcher
    aircraft = tmp_path / "aircraft models" / "custom"
    aircraft.mkdir(parents=True)
    (aircraft / "model.sdf").write_text(
        '<sdf version="1.9"><model name="custom"><link name="body"/>'
        '<plugin name="ArduPilotPlugin"><fdm_port_in>9002</fdm_port_in></plugin></model></sdf>'
    )
    texture = tmp_path / "my texture # café.png"
    texture.write_bytes(b"fixture")
    run = tmp_path / "Mission Planner"
    run.mkdir()
    monkeypatch.setattr(builder, "component", lambda *_: {"binary": str(tmp_path / "libArduPilotPlugin.so")})
    info = builder.create(
        tmp_path,
        run,
        {
            "aircraft_model": str(aircraft),
            "texture": str(texture),
            "home": [44, -75, 123, 0],
            "spawn": [0, 0, 0.3, 0, 0, 0],
        },
        tmp_path / "config.json",
        12345,
        23456,
    )
    tree = ET.parse(info["world"])
    assert tree.findtext("world/{http://gazebosim.org/schema}policies/parallel_postupdates") == "false"
    assert "<gz:policies>" in Path(info["world"]).read_text()
    assert tree.findtext(".//visual/geometry/mesh/uri") == (run / "ground.obj").resolve().as_posix()
    assert tree.findtext(".//albedo_map") == texture.resolve().as_posix()


def test_gui_readiness_requires_render_signal_and_live_process(launcher):
    import threading
    import time
    import types

    wait = launcher[0].wait_for_gui
    frame = threading.Event()
    alive = types.SimpleNamespace(poll=lambda: None)
    with pytest.raises(TimeoutError, match="did not render"):
        wait(alive, frame, time.monotonic() - 1, lambda: True)
    with pytest.raises(RuntimeError, match="code -6"):
        wait(types.SimpleNamespace(poll=lambda: -6), frame, time.monotonic() + 1, lambda: True)
    assert not wait(alive, frame, time.monotonic() + 1, lambda: False)
    frame.set()
    assert wait(alive, frame, time.monotonic() + 1, lambda: True)
