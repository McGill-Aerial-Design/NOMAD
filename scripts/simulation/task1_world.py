# SPDX-License-Identifier: Apache-2.0
"""Build a georeferenced Task 1 perception world; optional owned, visible Gazebo preview."""

import argparse
import base64
import datetime
import json
import math
import os
import random
import shutil
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

from owned_processes import stop as stop_owned
from prepare_assets import digest, ground_assets, prepare_ground
from task1_assets import prepare, verify


def local_xy(lat, lon, origin):
    def ecef(latitude, longitude):
        latitude, longitude = map(math.radians, (latitude, longitude))
        radius = 6378137 / math.sqrt(1 - 0.00669437999014 * math.sin(latitude) ** 2)
        return (
            radius * math.cos(latitude) * math.cos(longitude),
            radius * math.cos(latitude) * math.sin(longitude),
            radius * (1 - 0.00669437999014) * math.sin(latitude),
        )

    delta = [a - b for a, b in zip(ecef(lat, lon), ecef(*origin), strict=True)]
    latitude, longitude = map(math.radians, origin)
    return (
        -math.sin(longitude) * delta[0] + math.cos(longitude) * delta[1],
        -math.sin(latitude) * math.cos(longitude) * delta[0]
        - math.sin(latitude) * math.sin(longitude) * delta[1]
        + math.cos(latitude) * delta[2],
    )


def inside(point, polygon):
    x, y = point
    result = False
    for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1], strict=True):
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            result = not result
    return result


def ground_materials(inputs, output, cache):
    import cv2

    assets = ground_assets(cache)
    site_colour = cv2.imread(str(inputs / "site-rgb.png"))
    if site_colour is None or site_colour.shape != (160, 128, 3):
        raise ValueError("Prepared site colour must be the original 128x160 RGB crop")
    colours = cv2.cvtColor(cv2.resize(site_colour, (64, 64), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
    quantized = [[tuple(int(value) // 24 * 24 + 12 for value in pixel) for pixel in row] for row in colours]
    counts = Counter(colour for row in quantized for colour in row)
    palette = sorted(counts, key=lambda colour: (-counts[colour], colour))[:24]
    cells = [
        [
            min(range(len(palette)), key=lambda i: sum((a - b) ** 2 for a, b in zip(colour, palette[i])))
            for colour in row
        ]
        for row in quantized
    ]
    means = {}
    for asset in assets:
        source = cache / asset["name"]
        image = cv2.imread(str(source))
        if image is None:
            raise ValueError("Ground texture cannot be decoded: " + asset["name"])
        means[asset["asset_id"]] = image.mean(axis=(0, 1))[::-1].tolist()
        shutil.copy2(source, output / asset["name"])
    by_id = {asset["asset_id"]: asset for asset in assets}
    materials = []
    with (output / "terrain.mtl").open("w", encoding="utf-8") as mtl:
        for index, (red, green, blue) in enumerate(palette):
            # Synthetic surface choice follows coarse colour, not a surveyed vegetation/soil classification.
            surface = "sparse_grass" if green > red and green > blue else "dirt" if red < 120 else "withered_grass"
            asset = by_id[surface]
            tint = [min(1, value / max(1, average)) for value, average in zip((red, green, blue), means[surface])]
            colour = " ".join(f"{value:.8f}" for value in tint)
            mtl.write(
                f"newmtl ground_{index:02d}\nKa {colour}\nKd {colour}\nKs 0 0 0\nNs 0\nd 1\n"
                f"illum 1\nPr 1\nPm 0\nmap_Kd {asset['name']}\n\n"
            )
            materials.append(
                {
                    "name": f"ground_{index:02d}",
                    "source_rgb": [red, green, blue],
                    "surface": surface,
                    "diffuse_rgb": tint,
                }
            )
    return {
        "tile_m": 2,
        "palette": materials,
        "cells": cells,
        "assets": assets,
        "source_colour_sha256": digest(inputs / "site-rgb.png"),
        "recipe": "RGB bin width24/255; at most24 dominant colours; nearest-colour cells; photo-mean-normalized tint",
        "limit": "Synthetic grass/dry-grass/dirt detail; coarse site colours, not a measured ground-cover map",
    }


def terrain(site, heights, output, materials=None):
    west, south, east, north = site["imagery_bbox_w_s_e_n"]
    origin = site["origin_lat_lon"]
    baseline = heights[32][32]
    vertices = []
    with output.open("w") as mesh:
        if materials:
            mesh.write("mtllib terrain.mtl\n")
        for y in range(65):
            for x in range(65):
                ex, ny = local_xy(north - (north - south) * y / 64, west + (east - west) * x / 64, origin)
                vertices.append((ex, ny, heights[y][x] - baseline))
                mesh.write(f"v {ex:.6f} {ny:.6f} {heights[y][x] - baseline:.6f}\n")
        for y in range(65):
            for x in range(65):
                uv = (
                    (vertices[y * 65 + x][0] / materials["tile_m"], vertices[y * 65 + x][1] / materials["tile_m"])
                    if materials
                    else (x / 64, 1 - y / 64)
                )
                mesh.write(f"vt {uv[0]:.8f} {uv[1]:.8f}\n")
        for y in range(65):
            for x in range(65):
                left, right = vertices[y * 65 + max(0, x - 1)], vertices[y * 65 + min(64, x + 1)]
                south_point, north_point = vertices[min(64, y + 1) * 65 + x], vertices[max(0, y - 1) * 65 + x]
                a = [after - before for after, before in zip(right, left, strict=True)]
                b = [n - s for n, s in zip(north_point, south_point, strict=True)]
                normal = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
                length = math.sqrt(sum(value * value for value in normal))
                mesh.write("vn " + " ".join(f"{value / length:.8f}" for value in normal) + "\n")
        for index in range(len(materials["palette"]) if materials else 1):
            if materials:
                mesh.write(f"o ground_{index:02d}\nusemtl ground_{index:02d}\n")
            for y in range(64):
                for x in range(64):
                    if materials and materials["cells"][y][x] != index:
                        continue
                    a, b, c, d = y * 65 + x + 1, y * 65 + x + 2, (y + 1) * 65 + x + 1, (y + 1) * 65 + x + 2
                    mesh.write(f"f {a}/{a}/{a} {c}/{c}/{c} {b}/{b}/{b}\nf {b}/{b}/{b} {c}/{c}/{c} {d}/{d}/{d}\n")

    def height_at(ex, ny):
        # Local inversion only selects a coarse CDEM interpolation cell; the mesh uses exact WGS84 ENU vertices.
        lat = origin[0] + ny / 111230
        lon = origin[1] + ex / (111320 * math.cos(math.radians(origin[0])))
        fx, fy = (lon - west) / (east - west) * 64, (north - lat) / (north - south) * 64
        if not 0 <= fx < 64 or not 0 <= fy < 64:
            raise ValueError("Object lies outside terrain")
        x, y = int(fx), int(fy)
        dx, dy = fx - x, fy - y
        # Match the actual OBJ triangle, rather than hovering on a different bilinear surface.
        if dx + dy <= 1:
            value = heights[y][x] + dx * (heights[y][x + 1] - heights[y][x]) + dy * (heights[y + 1][x] - heights[y][x])
        else:
            value = (
                heights[y + 1][x + 1]
                + (1 - dx) * (heights[y + 1][x] - heights[y + 1][x + 1])
                + (1 - dy) * (heights[y][x + 1] - heights[y + 1][x + 1])
            )
        return value - baseline

    return height_at, {
        "baseline_cgvd28_m": baseline,
        "minimum_relative_m": min(v[2] for v in vertices),
        "maximum_relative_m": max(v[2] for v in vertices),
        "triangles": 8192,
        "absolute_ellipsoid_height_available": False,
    }


def mesh_visual(link, name, uri, scale, pose="0 0 0 0 0 0"):
    visual = ET.SubElement(link, "visual", name=name)
    ET.SubElement(visual, "pose").text = pose
    mesh = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
    ET.SubElement(mesh, "uri").text = uri
    ET.SubElement(mesh, "scale").text = " ".join([str(scale)] * 3)
    return visual


def tag(link, code, scale, output):
    import cv2
    import numpy as np

    image = np.full((256, 384, 3), 245, dtype=np.uint8)
    font_scale = 96 / 21  # 96/256 * 0.08 m = 3 cm glyph height.
    (width, height), _ = cv2.getTextSize(code, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 7)
    cv2.putText(
        image,
        code,
        ((384 - width) // 2, (256 + height) // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (10, 10, 10),
        7,
        cv2.LINE_AA,
    )
    path = output / f"tag-{code}.png"
    if not cv2.imwrite(str(path), image):
        raise OSError("Cannot write tag texture")
    for face in range(2):
        panel = ET.SubElement(link, "visual", name=f"tag_{code}_{face}")
        ET.SubElement(
            panel, "pose"
        ).text = (
            f"{0.5685 * scale} {-0.263 * scale + face * 0.001} {1.5925 * scale - 0.04} {math.pi / 2} 0 {face * math.pi}"
        )
        plane = ET.SubElement(ET.SubElement(panel, "geometry"), "plane")
        ET.SubElement(plane, "normal").text = "0 0 1"
        ET.SubElement(plane, "size").text = "0.12 0.08"
        material = ET.SubElement(panel, "material")
        ET.SubElement(material, "diffuse").text = "1 1 1 1"
        metal = ET.SubElement(ET.SubElement(material, "pbr"), "metal")
        ET.SubElement(metal, "albedo_map").text = path.resolve().as_posix()
        ET.SubElement(metal, "roughness").text = "0.85"


def create(site, inputs, output, ground_cache=None):
    ET.register_namespace("gz", "http://gazebosim.org/schema")
    manifest = verify(inputs)
    if manifest["site"] != site:
        raise ValueError("World definition differs from frozen inputs; prepare a fresh input directory")
    if output.exists():
        raise ValueError("A world output must be a fresh owned directory")
    output.mkdir(parents=True)
    run_id = output.name
    (output / "run.id").write_text(run_id + "\n")
    heights = json.loads((inputs / "elevation.json").read_text())
    if len(heights) != 65 or any(len(row) != 65 or any(not math.isfinite(v) for v in row) for row in heights):
        raise ValueError("Invalid terrain grid")
    ground_cache = ground_cache or Path(__file__).resolve().parents[2] / "local/simulation/assets/ground"
    materials = ground_materials(inputs, output, ground_cache)
    height_at, terrain_info = terrain(site, heights, output / "terrain.obj", materials)
    sdf = ET.fromstring("""<sdf version="1.9" xmlns:gz="http://gazebosim.org/schema"><world name="nomad_task1">
      <gz:policies><parallel_postupdates>false</parallel_postupdates></gz:policies>
      <physics name="fixed" type="ignore"><max_step_size>0.01</max_step_size>
        <real_time_factor>1</real_time_factor></physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
        <render_engine>ogre2</render_engine></plugin>
      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <scene><ambient>0.55 0.55 0.55 1</ambient><background>0.54 0.70 0.85 1</background><shadows>true</shadows></scene>
      <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
        <latitude_deg/><longitude_deg/><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates>
      <light name="sun" type="directional"><pose>0 0 1000 0 0 0</pose><diffuse>0.85 0.85 0.8 1</diffuse>
        <specular>0.1 0.1 0.1 1</specular><direction>0.25 0.5 -0.83</direction><cast_shadows>true</cast_shadows></light>
      <model name="site_terrain"><static>true</static><link name="ground"/></model>
    </world></sdf>""")
    world = sdf.find("world")
    world.find("spherical_coordinates/latitude_deg").text = str(site["origin_lat_lon"][0])
    world.find("spherical_coordinates/longitude_deg").text = str(site["origin_lat_lon"][1])
    ground = world.find("model/link")
    mesh_visual(ground, "site_colour_with_synthetic_detail", (output / "terrain.obj").resolve().as_posix(), 1)
    collision = ET.SubElement(ground, "collision", name="terrain")
    geometry = ET.SubElement(ET.SubElement(collision, "geometry"), "mesh")
    ET.SubElement(geometry, "uri").text = (output / "terrain.obj").resolve().as_posix()
    boundary = [local_xy(*point, site["origin_lat_lon"]) for point in site["boundary_lat_lon"]]
    scale = site["deer"]["proxy_length_m"] / site["deer"]["source_length_m"]
    rng, deer = random.Random(site["seed"]), []
    for group_index, group in enumerate(site["deer_groups_xy_m"]):
        for x, y in group:
            if not inside((x, y), boundary):
                raise ValueError("A decoy escaped the published C1 outline")
            index, yaw = len(deer), rng.uniform(-math.pi, math.pi)
            z = height_at(x, y) + 0.015
            antlers = index % 2 == 0
            code = f"{chr(65 + index // 3)}{1 + index // 3}" if index % 3 == 0 else None
            model = ET.SubElement(world, "model", name=f"deer_{index:02d}")
            ET.SubElement(model, "static").text = "true"
            ET.SubElement(model, "pose").text = f"{x} {y} {z:.6f} 0 0 {yaw:.8f}"
            link = ET.SubElement(model, "link", name="decoy")
            collision = ET.SubElement(link, "collision", name="body_proxy")
            ET.SubElement(collision, "pose").text = "0 0 .575 0 0 0"
            box = ET.SubElement(ET.SubElement(collision, "geometry"), "box")
            ET.SubElement(box, "size").text = "1.524 .41 1.15"
            asset = site["deer"]["name"] if antlers else "antlerless.glb"
            mesh_visual(link, "deer", (inputs / asset).resolve().as_posix(), scale, f"0 0 0 {math.pi / 2} 0 0")
            if code:
                tag(link, code, scale, output)
            deer.append(
                {
                    "id": index,
                    "group_fixture": group_index,
                    "antlers": antlers,
                    "tag": code,
                    "position_enu_m": [x, y, z],
                    "yaw_radians": yaw,
                    "length_m": site["deer"]["proxy_length_m"],
                }
            )
    x, y = 127, -137
    camera_pose = [
        x,
        y,
        height_at(x, y) + 8,
        0,
        math.atan2(8, math.hypot(120 - x, -120 - y)),
        math.atan2(-120 - y, 120 - x),
    ]
    rig = ET.SubElement(world, "model", name="inspection_camera")
    ET.SubElement(rig, "static").text = "true"
    ET.SubElement(rig, "pose").text = " ".join(map(str, camera_pose))
    sensor = ET.SubElement(ET.SubElement(rig, "link", name="camera"), "sensor", name="rgb", type="camera")
    ET.SubElement(sensor, "topic").text = "/nomad/task1/rgb"
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "10"
    camera = ET.SubElement(sensor, "camera")
    ET.SubElement(camera, "horizontal_fov").text = str(math.pi / 3)
    image = ET.SubElement(camera, "image")
    for key, value in {"width": "1280", "height": "720", "format": "R8G8B8"}.items():
        ET.SubElement(image, key).text = value
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = ".05"
    ET.SubElement(clip, "far").text = "2500"
    overview = ET.fromstring(ET.tostring(rig))
    overview.set("name", "overview_camera")
    overview.find("pose").text = f"0 0 650 0 {math.pi / 2} {math.pi / 2}"
    overview.find("link/sensor/topic").text = "/nomad/task1/overview"
    overview.find("link/sensor/update_rate").text = "1"
    overview.find(".//camera/horizontal_fov").text = str(math.radians(75))
    overview.find(".//image/width").text = "1024"
    overview.find(".//image/height").text = "1280"
    world.append(overview)
    ET.ElementTree(sdf).write(output / "task1.sdf", encoding="utf-8", xml_declaration=True)
    ring = [[lon, lat] for lat, lon in site["boundary_lat_lon"]]
    (output / "boundary.geojson").write_text(json.dumps({"type": "Polygon", "coordinates": [ring + ring[:1]]}) + "\n")
    truth = {
        "site": site,
        "inputs_manifest_sha256": digest(inputs / "inputs.json"),
        "terrain": terrain_info,
        "ground_materials": materials,
        "deer": deer,
        "seed": site["seed"],
        "camera_pose_enu_rpy": camera_pose,
        "camera": {"width": 1280, "height": 720, "hfov_radians": math.pi / 3, "hz": 10},
        "overview": {
            "pose_enu_rpy": [0, 0, 650, 0, math.pi / 2, math.pi / 2],
            "width": 1024,
            "height": 1280,
            "hfov_radians": math.radians(75),
            "hz": 1,
        },
        "tag_panel_m": [0.12, 0.08],
        "tag_glyph_m": 0.03,
        "truth": "Configured static independent scene oracle; never detector/tracker input",
        "limits": [
            "No exact 2027 search polygon or actual decoy model",
            "10 m imagery and coarse CDEM; photographic ground detail is an approximation, not a site survey",
            "Inspection camera only; no flight/gimbal command-path qualification",
            "No deer weights in built-in detector",
        ],
        "generator_sources": {
            name: digest(Path(__file__).with_name(name))
            for name in ("task1_world.py", "task1_assets.py", "prepare_assets.py", "assets.json", "owned_processes.py")
        },
        "files": {p.name: digest(p) for p in output.iterdir() if p.is_file()},
    }
    (output / "scene.json").write_text(json.dumps(truth, indent=2) + "\n")
    return truth


def capture(output, name, topic, size, environment):
    captured = subprocess.run(
        ["gz", "topic", "-e", "--json-output", "-n", "1", "-t", topic],
        env=environment,
        capture_output=True,
        timeout=45,
        check=True,
    )
    received_ns = time.monotonic_ns()
    if len(captured.stdout) > 8 * 1024 * 1024:
        raise ValueError("Camera response exceeds expected RGB frame bound")
    frame = json.loads(captured.stdout)
    if (frame["width"], frame["height"], frame["step"]) != (*size, size[0] * 3) or frame[
        "pixelFormatType"
    ] != "RGB_INT8":
        raise ValueError("Rendered camera geometry differs")
    pixels = base64.b64decode(frame.pop("data"), validate=True)
    if len(pixels) != size[0] * size[1] * 3:
        raise ValueError("Rendered pixel count differs")
    from PIL import Image

    Image.frombytes("RGB", size, pixels).save(output / f"{name}.png")
    frame["host_receipt_monotonic_ns"] = received_ns
    frame["host_receipt_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    (output / f"{name}-capture.json").write_text(json.dumps(frame, indent=2) + "\n")


def preview(output, seconds):
    run_id = (output / "run.id").read_text().strip()
    environment = dict(
        os.environ,
        NOMAD_SIMULATION_RUN_ID=run_id,
        GZ_PARTITION=run_id,
        GALLIUM_DRIVER="d3d12",
        MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA",
        QT_QPA_PLATFORM="xcb",
        QT_XCB_GL_INTEGRATION="xcb_glx",
    )
    # OpenCV's wheel sets its own incompatible Qt path while generating tag bitmaps.
    environment["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"
    environment.pop("QT_QPA_FONTDIR", None)
    for key, directory in {
        "HOME": "gazebo-home",
        "GZ_FUEL_CACHE_PATH": "fuel-cache",
        "GZ_LOG_PATH": "gz-logs",
        "XDG_CACHE_HOME": "cache",
    }.items():
        (output / directory).mkdir()
        environment[key] = str(output / directory)
    server = gui = None
    try:
        with (output / "server.log").open("w") as log:
            server = subprocess.Popen(
                ["gz", "sim", "-s", "-r", "-v3", "--seed", "173", str(output / "task1.sdf")],
                env=environment,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        capture(output, "camera", "/nomad/task1/rgb", (1280, 720), environment)
        capture(output, "overview", "/nomad/task1/overview", (1024, 1280), environment)
        config = Path("/usr/share/gz/gz-sim8/gui/gui.config").read_text()
        config = config.replace(
            '<dialog name="quick_start" show_again="true"/>', '<dialog name="quick_start" show_again="false"/>'
        )
        z = json.loads((output / "scene.json").read_text())["camera_pose_enu_rpy"][2] + 5
        config = config.replace(
            "<camera_pose>-6 0 6 0 0.5 0</camera_pose>", f"<camera_pose>142 -142 {z} 0 0.4 2.35619449</camera_pose>"
        )
        (output / "gui.config").write_text(config)
        with (output / "gui.log").open("w") as log:
            gui = subprocess.Popen(
                ["gz", "sim", "-g", "-v3", "--gui-config", str(output / "gui.config")],
                env=environment,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        print("Visible Task 1 preview: " + str(output), flush=True)
        end = time.monotonic() + seconds
        while gui.poll() is None and time.monotonic() < end:
            if server.poll() is not None:
                raise RuntimeError("Owned world server exited")
            time.sleep(0.2)
        if gui.poll() not in (None, 0):
            raise RuntimeError("Visible Gazebo GUI failed; inspect gui.log")
    finally:
        errors = []
        for process in (gui, server):
            if process is not None:
                try:
                    stop_owned(process, run_id)
                except Exception as error:
                    errors.append(str(error))
        (output / "cleanup.json").write_text(
            json.dumps({"owned_sessions_stopped": not errors, "errors": errors, "physical_devices_opened": False})
            + "\n"
        )
        if errors:
            raise RuntimeError("Owned preview cleanup failed: " + "; ".join(errors))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--seconds", type=int, default=120)
    args = parser.parse_args()
    if not 10 <= args.seconds <= 600:
        raise ValueError("Preview must be bounded to 10â€“600 seconds")
    repo = Path(__file__).resolve().parents[2]
    site = json.loads(Path(__file__).with_name("task1-site.json").read_text())
    inputs = repo / "local/simulation/task1/inputs"
    if args.prepare:
        prepare(site, inputs)
        prepare_ground(repo / "local/simulation/assets/ground")
    output = repo / "build/tests/task1-world" / ("run-" + uuid.uuid4().hex)
    create(site, inputs, output)
    print("Created " + str(output / "task1.sdf"), flush=True)
    if args.preview:
        preview(output, args.seconds)


if __name__ == "__main__":
    main()
