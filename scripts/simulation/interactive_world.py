# SPDX-License-Identifier: Apache-2.0
"""Compose scenery and a configured free aircraft; no qualification constraints or camera steering."""

import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from build_manifest import verify

SITL_EXECUTABLES = {"ArduCopter": "arducopter", "ArduPlane": "arduplane"}


def component(workspace, name, executable=None):
    recorded = json.loads((workspace / "logs" / f"{name}-build.json").read_text())
    binary = Path(recorded["binary"])
    if executable is not None:
        binary = binary.with_name(executable)
    return verify(workspace, name, recorded["source"], binary)


def asset_path(value, root):
    path = Path(value)
    return (path if path.is_absolute() else root / path).resolve(strict=True)


def create(workspace, run, recipe, scenario_path, fdm_port, video_port, world_override=None):
    ET.register_namespace("gz", "http://gazebosim.org/schema")
    renderer = component(workspace, "ardupilot_gazebo")
    plugins = Path(renderer["binary"]).parent
    original = world_override or recipe.get("world")
    home, spawn = recipe.get("home"), recipe.get("spawn")
    if original:
        source = asset_path(original, scenario_path.parent)
        for _, (prefix, uri) in ET.iterparse(source, events=["start-ns"]):
            if prefix == "gz":
                ET.register_namespace(prefix, uri)
        tree = ET.parse(source)
        resource_paths = [str(source.parent)]
    elif recipe.get("scene") == "task1":
        from task1_world import create as site_world

        tools = Path(__file__).parent
        site = json.loads((tools / "task1-site.json").read_text())
        inputs = tools.parents[1] / "local/simulation/task1/inputs"
        info = site_world(site, inputs, run / "site")
        tree = ET.parse(run / "site/task1.sdf")
        world = tree.getroot().find("world")
        for camera in list(world.findall("model")):
            if camera.get("name") in ("inspection_camera", "overview_camera"):
                world.remove(camera)
        home = home or [*site["origin_lat_lon"], info["terrain"]["baseline_cgvd28_m"], 0]
        spawn = spawn or [*info["camera_pose_enu_rpy"][:2], info["camera_pose_enu_rpy"][2] - 8 + 0.3, 0, 0, 0]
        resource_paths = [str(inputs)]
    else:
        tree = ET.ElementTree(
            ET.fromstring("""<sdf version="1.9" xmlns:gz="http://gazebosim.org/schema"><world name="nomad_sim">
          <gz:policies><parallel_postupdates>false</parallel_postupdates></gz:policies>
          <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.5 0.65 0.8 1</background></scene>
          <light name="sun" type="directional"><pose>0 0 100 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.5 0.1 -0.9</direction>
            <cast_shadows>true</cast_shadows></light>
          <model name="ground"><static>true</static><link name="surface">
            <collision name="floor"><geometry><plane><normal>0 0 1</normal>
              <size>2000 2000</size></plane></geometry></collision>
            <visual name="grass"><geometry><mesh><uri/></mesh></geometry><material>
              <diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map/><roughness>1</roughness></metal></pbr>
            </material></visual></link></model></world></sdf>""")
        )
        texture = asset_path(recipe["texture"], scenario_path.parent)
        if (
            recipe.get("texture_sha256")
            and hashlib.sha256(texture.read_bytes()).hexdigest() != recipe["texture_sha256"]
        ):
            raise ValueError("Configured terrain texture checksum differs")
        plane = run / "ground.obj"
        plane.write_text(
            "v -150 -150 0\nv 150 -150 0\nv 150 150 0\nv -150 150 0\n"
            "vt 0 0\nvt 20 0\nvt 20 20\nvt 0 20\nvn 0 0 1\n"
            "f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n"
        )
        tree.find(".//visual/geometry/mesh/uri").text = plane.resolve().as_posix()
        tree.find(".//albedo_map").text = texture.resolve().as_posix()
        resource_paths = [str(texture.parent)]
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("Scenario needs one SDF world")
    if any((uri.text or "").strip().startswith(("http:", "https:")) for uri in world.iter("uri")):
        raise ValueError("Prepare remote world assets locally before launching")
    world.set("name", "nomad_sim")
    for filename, name in (
        ("physics", "Physics"),
        ("sensors", "Sensors"),
        ("imu", "Imu"),
        ("navsat", "NavSat"),
        ("scene-broadcaster", "SceneBroadcaster"),
        ("user-commands", "UserCommands"),
    ):
        if not any(p.get("name") == "gz::sim::systems::" + name for p in world.findall("plugin")):
            plugin = ET.SubElement(
                world, "plugin", filename=f"gz-sim-{filename}-system", name="gz::sim::systems::" + name
            )
            if name == "Sensors":
                ET.SubElement(plugin, "render_engine").text = "ogre2"
    for plugin in world.findall("plugin"):
        if plugin.get("name") == "gz::sim::systems::SceneBroadcaster":
            period = plugin.find("max_publish_period")
            if period is None:
                period = ET.SubElement(plugin, "max_publish_period")
            period.text = "0.033"
    physics = world.find("physics")
    if physics is None:
        physics = ET.SubElement(world, "physics", name="fixed", type="ignore")
    if physics.find("max_step_size") is None:
        ET.SubElement(physics, "max_step_size")
    physics.find("max_step_size").text = str(1 / recipe.get("physics_hz", 1000))
    coordinates = world.find("spherical_coordinates")
    if coordinates is not None and (original or home is None):
        # A loaded site's georeference takes precedence over the example scenario's origin.
        heading = home[3] if home else 0
        home = [float(coordinates.findtext(name)) for name in ("latitude_deg", "longitude_deg", "elevation")] + [
            heading
        ]
        if float(coordinates.findtext("heading_deg", "0")) != 0:
            raise ValueError("ArduPilot scenarios require an ENU world (spherical heading_deg 0)")
    if home is None:
        raise ValueError("Provide a home coordinate in the scenario or SDF")
    if coordinates is None:
        coordinates = ET.SubElement(world, "spherical_coordinates")
    for key, value in {
        "surface_model": "EARTH_WGS84",
        "latitude_deg": home[0],
        "longitude_deg": home[1],
        "elevation": home[2],
        "heading_deg": 0,
    }.items():
        if coordinates.find(key) is None:
            ET.SubElement(coordinates, key)
        coordinates.find(key).text = str(value)
    wind = recipe.get("wind")
    if wind is not None:
        for existing in list(world.findall("wind")):
            world.remove(existing)
        if any(wind):
            element = ET.Element("wind")
            ET.SubElement(element, "linear_velocity").text = " ".join(str(value) for value in wind)
            world.append(element)
    if recipe.get("aircraft_model_local"):
        model_source = asset_path(recipe["aircraft_model_local"], scenario_path.parent)
    elif recipe.get("aircraft_model"):
        model_source = asset_path(recipe["aircraft_model"], workspace)
    else:
        raise ValueError("Provide aircraft_model or aircraft_model_local")
    models = run / "models"
    models.mkdir()
    target = models / model_source.name
    shutil.copytree(model_source, target)
    aircraft = ET.parse(target / "model.sdf")
    if aircraft.findtext("model/static", "false").lower() in ("true", "1") or any(
        joint.findtext("parent") == "world" or joint.findtext("child") == "world"
        for joint in aircraft.findall(".//joint")
    ):
        raise ValueError("Choose a free aircraft model without static or world-fixed joints")
    ap = [p for p in aircraft.findall(".//plugin") if "ArduPilotPlugin" in p.get("name", "")]
    if len(ap) != 1:
        raise ValueError("Configured aircraft must contain one native ArduPilotPlugin")
    ap[0].set("filename", str(plugins / "libArduPilotPlugin.so"))
    ap[0].find("fdm_port_in").text = str(fdm_port)
    aircraft.write(target / "model.sdf", encoding="utf-8", xml_declaration=True)
    camera_info = None
    if recipe.get("camera_model"):
        camera_source = asset_path(recipe["camera_model"], workspace)
        camera_target = models / camera_source.name
        shutil.copytree(camera_source, camera_target)
        camera_tree = ET.parse(camera_target / "model.sdf")
        sensor = camera_tree.find(".//sensor[@type='camera']")
        if sensor is None:
            raise ValueError("Configured camera model has no camera sensor")
        if recipe.get("camera"):
            image = sensor.find("camera/image")
            if image is None:
                image = ET.SubElement(sensor.find("camera"), "image")
            for key in ("width", "height"):
                if image.find(key) is None:
                    ET.SubElement(image, key)
                image.find(key).text = str(recipe["camera"][key])
            if sensor.find("update_rate") is None:
                ET.SubElement(sensor, "update_rate")
            sensor.find("update_rate").text = str(recipe["camera"]["fps"])
        if sensor.find("topic") is None:
            ET.SubElement(sensor, "topic")
        sensor.find("topic").text = "/nomad/simulation/rgb"
        for plugin in list(sensor.findall("plugin")):
            if "CameraZoomPlugin" in plugin.get("name", ""):
                sensor.remove(plugin)
        stream = sensor.find("plugin[@name='GstCameraPlugin']")
        if stream is None:
            stream = ET.SubElement(sensor, "plugin", name="GstCameraPlugin")
        stream.set("filename", str(plugins / "libGstCameraPlugin.so"))
        if "use_cuda" in recipe.get("camera", {}):
            if stream.find("use_cuda") is None:
                ET.SubElement(stream, "use_cuda")
            stream.find("use_cuda").text = str(recipe["camera"]["use_cuda"]).lower()
        if "bitrate_kbps" in recipe.get("camera", {}):
            if stream.find("bitrate") is None:
                ET.SubElement(stream, "bitrate")
            stream.find("bitrate").text = str(recipe["camera"]["bitrate_kbps"])
        for key, value in {
            "udp_host": "127.0.0.1",
            "udp_port": video_port,
            "use_basic_pipeline": "true",
            "image_topic": "/nomad/simulation/rgb",
            "enable_topic": "/nomad/simulation/rgb/enable_streaming",
        }.items():
            if stream.find(key) is None:
                ET.SubElement(stream, key)
            stream.find(key).text = str(value)
        camera_tree.write(camera_target / "model.sdf", encoding="utf-8", xml_declaration=True)
        camera_info = {
            "hfov_degrees": math.degrees(float(sensor.findtext("camera/horizontal_fov"))),
            "width": int(sensor.findtext("camera/image/width", "0")),
            "height": int(sensor.findtext("camera/image/height", "0")),
            "fps": float(sensor.findtext("update_rate", "0")),
            "use_cuda": stream.findtext("use_cuda", "false").lower() in ("true", "1"),
            "bitrate_kbps": int(stream.findtext("bitrate", "800")),
            "topic": "/nomad/simulation/rgb",
        }
        resource_paths.append(str(camera_source.parent))
    aircraft_name = recipe.get("aircraft_name", "vehicle")
    for existing in list(world):
        if (existing.tag == "model" and existing.get("name") == aircraft_name) or (
            existing.tag == "include"
            and (
                existing.findtext("name") == aircraft_name
                or (existing.findtext("uri", "").rstrip("/").split("/")[-1] == model_source.name)
            )
        ):
            spawn = spawn or [float(v) for v in existing.findtext("pose", "0 0 .3 0 0 0").split()]
            world.remove(existing)
    if any("ArduPilotPlugin" in p.get("name", "") for p in world.findall(".//plugin")):
        raise ValueError("Select the scenery and configure its aircraft model separately")
    if (
        not isinstance(spawn, list)
        or len(spawn) != 6
        or not all(type(v) in (int, float) and math.isfinite(v) for v in spawn)
    ):
        raise ValueError("Provide a finite six-value aircraft spawn pose")
    include = ET.SubElement(world, "include")
    ET.SubElement(include, "uri").text = "model://" + target.name
    ET.SubElement(include, "name").text = aircraft_name
    ET.SubElement(include, "pose").text = " ".join(map(str, spawn))
    destination = run / "interactive.sdf"
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return {
        "world": str(destination),
        "physics_hz": recipe.get("physics_hz", 1000),
        "wind": wind,
        "home": home,
        "renderer_build": renderer,
        "camera": camera_info,
        "resources": [str(models), str(model_source.parent), *resource_paths],
        "plugins": str(plugins),
        "body_fixed": False,
        "gimbal": "Native ArduPilotPlugin commands and native joint controllers",
    }
