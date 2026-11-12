# SPDX-License-Identifier: Apache-2.0
"""Create the small owned renderer fixture from pinned upstream models and a seeded texture."""

import hashlib
import json
import math
import random
import shutil
import struct
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path

from build_manifest import verify


def texture(path, seed=173):
    size = 1024
    pixels = bytearray([78, 100, 68] * size * size)
    rng = random.Random(seed)
    for _ in range(7000):
        x, y = rng.randrange(size - 20), rng.randrange(size - 20)
        width, height = rng.randrange(3, 20), rng.randrange(3, 20)
        color = bytes(rng.randrange(256) for _ in range(3))
        for row in range(y, y + height):
            start = (row * size + x) * 3
            pixels[start : start + width * 3] = color * width

    def chunk(name, data):
        return struct.pack(">I", len(data)) + name + data + struct.pack(">I", zlib.crc32(name + data))

    raw = b"".join(b"\0" + pixels[row * size * 3 : (row + 1) * size * 3] for row in range(size))
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw))
        + chunk(b"IEND", b"")
    )


def create(
    workspace: Path,
    run: Path,
    rtp_host: str,
    scene="ground",
    visible=False,
    occlusion=False,
    endurance=False,
    physics_hz=1000,
    object_motion=None,
    encoder_kbps=800,
):
    if type(physics_hz) is not int or physics_hz not in (400, 1000):
        raise ValueError("physics_hz must be 400 or 1000")
    if type(encoder_kbps) is not int or not 100 <= encoder_kbps <= 50000:
        raise ValueError("encoder_kbps must be 100..50000")
    ET.register_namespace("gz", "http://gazebosim.org/schema")
    upstream = workspace / "sources/ardupilot_gazebo"
    renderer_build = verify(
        workspace,
        "ardupilot_gazebo",
        "80dc9664dda7efd6c5d1a71a79198d1d6b708c0e",
        workspace / "build/gazebo-plugin/libArduPilotPlugin.so",
    )
    fixture_source = Path(__file__).resolve().parents[2] / "mission_planner/tests/simulation/gazebo"
    fixture_build = json.loads((workspace / "logs/nomad-test-system-build.json").read_text())
    for path, expected in fixture_build["files"].items():
        source = workspace / path if path.startswith("build/") else fixture_source / path
        if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
            raise ValueError("Test gimbal/camera source or binary changed after build")
    models = run / "models"
    models.mkdir()
    for name in ("iris_with_gimbal", "iris_with_standoffs", "gimbal_small_3d"):
        shutil.copytree(upstream / "models" / name, models / name)
    shutil.copy2(upstream / "LICENSE.md", models / "upstream-LICENSE.md")
    iris_path = models / "iris_with_gimbal/model.sdf"
    iris = ET.parse(iris_path)
    model = iris.getroot().find("model")
    for plugin in list(model.findall("plugin")):
        if plugin.get("name") == "gz::sim::systems::JointPositionController":
            model.remove(plugin)
    fixed = ET.SubElement(model, "joint", name="body_fixed_to_world", type="fixed")
    ET.SubElement(fixed, "parent").text = "world"
    ET.SubElement(fixed, "child").text = "iris_with_standoffs::base_link"
    plugin = ET.SubElement(model, "plugin", name="NomadTestSystem", filename="NomadTestSystem")
    plugin.set("filename", str(workspace / "build/nomad-test-system/libNomadTestSystem.so"))
    ET.SubElement(plugin, "output").text = str(run)
    if occlusion:
        ET.SubElement(plugin, "occlusion").text = "true"
    if endurance:
        ET.SubElement(plugin, "bounded_pngs").text = "true"
    ap = next(value for value in model.findall("plugin") if "ArduPilotPlugin" in value.get("name", ""))
    ap.set("filename", str(workspace / "build/gazebo-plugin/libArduPilotPlugin.so"))
    ap.find("fdm_port_in").text = "19202"
    iris.write(iris_path, encoding="utf-8", xml_declaration=True)
    camera_path = models / "gimbal_small_3d/model.sdf"
    camera = ET.parse(camera_path)
    sensor = camera.getroot().find(".//sensor[@name='camera']")
    ET.SubElement(sensor, "topic").text = "/nomad/rgb"
    sensor.find("camera/horizontal_fov").text = str(math.pi / 2)
    ET.SubElement(sensor.find("camera/image"), "format").text = "R8G8B8"
    for plugin in list(sensor.findall("plugin")):
        if plugin.get("name") == "CameraZoomPlugin":
            sensor.remove(plugin)
    stream = sensor.find("plugin[@name='GstCameraPlugin']")
    stream.set("filename", str(workspace / "build/gazebo-plugin/libGstCameraPlugin.so"))
    stream.find("udp_host").text = rtp_host
    stream.find("udp_port").text = "15601"
    ET.SubElement(stream, "image_topic").text = "/nomad/rgb"
    ET.SubElement(stream, "enable_topic").text = "/nomad/rgb/enable_streaming"
    ET.SubElement(stream, "bitrate").text = str(encoder_kbps)
    camera.write(camera_path, encoding="utf-8", xml_declaration=True)
    texture(run / "ground.png")
    world = ET.fromstring("""<sdf version="1.9" xmlns:gz="http://gazebosim.org/schema"><world name="nomad">
      <gz:policies><parallel_postupdates>false</parallel_postupdates></gz:policies>
      <physics name="fixed_step" type="ignore"><max_step_size>0.001</max_step_size>
        <real_time_factor>1</real_time_factor></physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
        <render_engine>ogre2</render_engine></plugin>
      <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
      <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
      <scene><ambient>1 1 1 1</ambient><background>0.5 0.6 0.7 1</background></scene>
      <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>-35.362938</latitude_deg>
        <longitude_deg>149.165085</longitude_deg><elevation>584</elevation><heading_deg>0</heading_deg></spherical_coordinates>
      <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0 0 0 1</specular><direction>-0.5 0.1 -0.9</direction><cast_shadows>false</cast_shadows></light>
      <model name="ground"><static>true</static><link name="plane">
        <collision name="floor"><geometry><plane><normal>0 0 1</normal><size>64 64</size></plane></geometry></collision>
        <visual name="texture"><geometry><plane><normal>0 0 1</normal><size>64 64</size></plane></geometry>
          <material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map/>
            <metalness>0</metalness><roughness>1</roughness></metal></pbr></material></visual>
      </link></model>
      <include><uri>model://iris_with_gimbal</uri><name>iris</name><pose>0 0 12 0 0 0</pose></include>
    </world></sdf>""")
    world.find(".//albedo_map").text = (run / "ground.png").resolve().as_posix()
    world.find("world/physics/max_step_size").text = str(1 / physics_hz)
    if visible:
        for filename, name in (
            ("gz-sim-scene-broadcaster-system", "SceneBroadcaster"),
            ("gz-sim-user-commands-system", "UserCommands"),
        ):
            plugin = ET.SubElement(world.find("world"), "plugin", filename=filename, name="gz::sim::systems::" + name)
            if name == "SceneBroadcaster":
                ET.SubElement(plugin, "max_publish_period").text = "0.033"
    plane_normal, plane_point = (0, 0, 1), (0, 0, 0)
    if occlusion:
        wall = ET.fromstring("""<model name="test_occluder"><static>true</static><pose>6 9 6 0 0 0</pose>
          <link name="body"><collision name="box"><geometry><box><size>4 8 8</size></box></geometry></collision>
          <visual name="opaque"><geometry><box><size>4 8 8</size></box></geometry>
            <material><ambient>0.12 0.12 0.12 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
          </visual></link></model>""")
        world.find("world").append(wall)
    scene_assets = None
    if scene == "natural":
        assets = Path(__file__).resolve().parents[2] / "local/simulation/assets"
        scene_assets = json.loads((assets / "prepared-scene.json").read_text())
        for relative, expected in scene_assets["files"].items():
            path = (assets / relative).resolve(strict=True)
            path.relative_to(assets.resolve())
            if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                raise ValueError("Prepared natural-scene asset changed")
        shutil.copytree(assets / "models/hatchback", models / "hatchback")
        shutil.copytree(assets / "models/construction_barrel", models / "construction_barrel")
        material = models / "hatchback/meshes/hatchback.mtl"
        material.write_text(
            material.read_text()
            .replace("map_Ka hatchback.png", "map_Ka model://hatchback/materials/textures/hatchback.png")
            .replace("map_Kd hatchback.png", "map_Kd model://hatchback/materials/textures/hatchback.png")
        )
        barrel_path = models / "construction_barrel/model.sdf"
        barrel = ET.parse(barrel_path)
        visual = barrel.find(".//visual")
        paint = ET.SubElement(visual, "material")
        ET.SubElement(paint, "diffuse").text = "1 1 1 1"
        metal = ET.SubElement(ET.SubElement(paint, "pbr"), "metal")
        ET.SubElement(metal, "albedo_map").text = (
            (models / "construction_barrel/materials/textures/Construction_Barrel_Diffuse.png").resolve().as_posix()
        )
        ET.SubElement(metal, "metalness").text = "0"
        ET.SubElement(metal, "roughness").text = "0.8"
        barrel.write(barrel_path, encoding="utf-8", xml_declaration=True)
        shutil.copy2(assets / "aerial_grass_rock_diff_1k.jpg", run / "natural-ground.jpg")
        ground = world.find(".//model[@name='ground']/link")
        template = ground.find("visual")
        ground.remove(template)
        for x in range(-2, 3):
            for y in range(-2, 3):
                tile = ET.fromstring(ET.tostring(template))
                tile.set("name", f"natural_{x}_{y}")
                ET.SubElement(tile, "pose").text = f"{x * 15} {y * 15} 0 0 0 0"
                tile.find("geometry/plane/size").text = "15 15"
                tile.find(".//albedo_map").text = (run / "natural-ground.jpg").resolve().as_posix()
                ground.append(tile)
        for name, model, pose in (
            ("car", "hatchback", "14 3 0 0 0 0.4"),
            ("barrel_a", "construction_barrel", "9 1 0 0 0 0"),
            ("barrel_b", "construction_barrel", "10 1.5 0 0 0 0"),
        ):
            include = ET.SubElement(world.find("world"), "include")
            ET.SubElement(include, "uri").text = "model://" + model
            ET.SubElement(include, "name").text = name
            ET.SubElement(include, "pose").text = pose
            ET.SubElement(include, "static").text = "true"
        world.find(".//scene/ambient").text = "0.45 0.45 0.45 1"
        world.find(".//light/cast_shadows").text = "true"
    if scene == "photo-board":
        # This is a calibration display, not terrain; the old floor cut through the lower image.
        ground_model = world.find("world/model[@name='ground']")
        world.find("world").remove(ground_model)
        fixture = Path(__file__).resolve().parents[2] / "mission_planner/tests/vision/vehicle.png"
        shutil.copy2(fixture, run / "board.png")
        board = ET.fromstring("""<model name="photo_board"><static>true</static>
          <pose>8.485281 0 3.514719 0.785398163397 0 -1.570796326795</pose>
          <link name="surface"><visual name="photo"><geometry><plane><normal>0 0 1</normal>
            <size>24 13.5</size></plane></geometry>
            <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><emissive>1 1 1 1</emissive>
              <pbr><metal><emissive_map/><metalness>0</metalness>
              <roughness>1</roughness></metal></pbr></material>
          </visual></link></model>""")
        board.find(".//emissive_map").text = (run / "board.png").resolve().as_posix()
        world.find("world").append(board)
        plane_normal, plane_point = (-math.sqrt(0.5), 0, math.sqrt(0.5)), (8.485281, 0, 3.514719)
    motion_info = None
    if object_motion:
        from object_motion import configure

        motion_info = configure(world.find("world"), iris.getroot().find("model"), camera.getroot(), run, object_motion)
        iris.write(iris_path, encoding="utf-8", xml_declaration=True)
        camera.write(camera_path, encoding="utf-8", xml_declaration=True)
    path = run / "world.sdf"
    ET.ElementTree(world).write(path, encoding="utf-8", xml_declaration=True)
    files = [path, *sorted(file for file in models.rglob("*") if file.is_file()), run / "ground.png"]
    if motion_info:
        files.append(run / "tag-A1.png")
    if scene == "natural":
        files.append(run / "natural-ground.jpg")
    if scene == "photo-board":
        files.append(run / "board.png")
    (run / "world-manifest.json").write_text(
        json.dumps(
            {
                "seed": 173,
                "physics_hz": physics_hz,
                "scene": scene,
                "scene_assets": scene_assets,
                "renderer_build": renderer_build,
                "fixture_build": fixture_build,
                "visible_preview": visible,
                "occlusion": occlusion,
                "endurance": endurance,
                "object_motion": motion_info,
                "detector_chart": json.loads(
                    (
                        Path(__file__).resolve().parents[2] / "mission_planner/tests/vision/fixtures/vehicle-chart.json"
                    ).read_text()
                )
                if scene == "photo-board"
                else None,
                "occluder_trajectory": "Owned trigger; clear1s,enter1s,covered1.5s,exit1s; world-level opaque4x8x8m box"
                if occlusion
                else None,
                "plane_normal": plane_normal,
                "plane_point": plane_point,
                "camera": {
                    "width": int(sensor.findtext("camera/image/width")),
                    "height": int(sensor.findtext("camera/image/height")),
                    "hfov_radians": float(sensor.findtext("camera/horizontal_fov")),
                    "hz": float(sensor.findtext("update_rate")),
                    "encoder_kbps": encoder_kbps,
                    "use_cuda": bool(object_motion),
                },
                "body_fixed": not bool(object_motion),
                "plant_delay_ms": 100,
                "plant_lag_ms": 80,
                "files": {str(file.relative_to(run)): hashlib.sha256(file.read_bytes()).hexdigest() for file in files},
            },
            indent=2,
        )
        + "\n"
    )
    return path
