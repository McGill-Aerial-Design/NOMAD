# SPDX-License-Identifier: Apache-2.0
import json
import math
import random
import struct
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


@pytest.fixture
def task1(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts/simulation"))
    import task1_assets
    import task1_world

    site = json.loads(Path(task1_world.__file__).with_name("task1-site.json").read_text())
    return task1_world, task1_assets, site


@pytest.fixture
def ground_fixture(task1, tmp_path, monkeypatch):
    cv2 = pytest.importorskip("cv2", reason="Photographic ground preparation uses existing OpenCV on Linux")
    import numpy as np
    import prepare_assets

    inputs = tmp_path / "input assets café #1"
    cache = tmp_path / "ground photos café #1"
    inputs.mkdir()
    cache.mkdir()
    image = np.empty((160, 128, 3), dtype=np.uint8)
    image[:, :42] = (50, 120, 80)
    image[:, 42:84] = (60, 72, 96)
    image[:, 84:] = (108, 144, 180)
    assert cv2.imwrite(str(inputs / "site-rgb.png"), image)
    pinned = []
    for index, asset in enumerate(prepare_assets.assets_for("ground")):
        path = cache / asset["name"]
        assert cv2.imwrite(str(path), np.full((32, 32, 3), (90 + index * 20, 130, 160), dtype=np.uint8))
        pinned.append(dict(asset, bytes=path.stat().st_size, sha256=prepare_assets.digest(path)))
    monkeypatch.setattr(prepare_assets, "assets_for", lambda group: pinned if group == "ground" else [])
    return inputs, cache, pinned


def test_site_cardinals_and_published_boundary(task1):
    world, _, site = task1
    origin = site["origin_lat_lon"]
    assert math.dist(world.local_xy(*origin, origin), (0, 0)) < 1e-8
    east = world.local_xy(origin[0], origin[1] + 0.001, origin)
    north = world.local_xy(origin[0] + 0.001, origin[1], origin)
    assert 71 < east[0] < 72 and abs(east[1]) < 0.01
    assert 111 < north[1] < 112 and abs(north[0]) < 0.01
    polygon = [world.local_xy(*point, origin) for point in site["boundary_lat_lon"]]
    assert world.inside((120, -120), polygon)
    assert not world.inside((-300, -400), polygon)
    assert sum(map(len, site["deer_groups_xy_m"])) == 12
    assert all(world.inside(point, polygon) for group in site["deer_groups_xy_m"] for point in group)
    assert site["boundary_lat_lon"][0] == [50.0970884, -110.7328077]
    assert site["imagery"]["native_resolution_m"] == 10


def test_terrain_north_uv_winding_and_finite_placement(task1, tmp_path):
    world, _, site = task1
    heights = [[715 + 0.2 * x + 0.1 * y for x in range(65)] for y in range(65)]
    mesh = tmp_path / "terrain.obj"
    height, info = world.terrain(site, heights, mesh)
    rows = mesh.read_text().splitlines()
    vertices = [list(map(float, line.split()[1:])) for line in rows if line.startswith("v ")]
    assert vertices[0][0] < 0 < vertices[64][0]
    assert vertices[0][1] > 0 > vertices[-1][1]
    uv = [line for line in rows if line.startswith("vt ")]
    assert uv[0] == "vt 0.00000000 1.00000000" and uv[-1] == "vt 1.00000000 0.00000000"
    normals = [list(map(float, line.split()[1:])) for line in rows if line.startswith("vn ")]
    assert len(normals) == 65 * 65 and all(normal[2] > 0.9 for normal in normals)
    assert next(line for line in rows if line.startswith("f ")) == "f 1/1/1 66/66/66 2/2/2"
    assert info["triangles"] == 8192 and not info["absolute_ellipsoid_height_available"]
    assert math.isfinite(height(120, -120))
    with pytest.raises(ValueError, match="outside terrain"):
        height(10000, 10000)


def test_generated_scene_keeps_literal_local_asset_paths(task1, ground_fixture, tmp_path, monkeypatch):
    world, _, site = task1
    inputs, cache, _ = ground_fixture
    for name in (site["deer"]["name"], "antlerless.glb", "inputs.json"):
        (inputs / name).write_bytes(b"fixture")
    before_colour = (inputs / "site-rgb.png").read_bytes()
    (inputs / "elevation.json").write_text(json.dumps([[720] * 65 for _ in range(65)]))
    monkeypatch.setattr(world, "verify", lambda _: {"site": site})
    output = tmp_path / "Mission Planner" / "run" / "site"
    info = world.create(site, inputs, output, cache)
    tree = ET.parse(output / "task1.sdf")
    assert tree.findtext("world/{http://gazebosim.org/schema}policies/parallel_postupdates") == "false"
    assert "<gz:policies>" in (output / "task1.sdf").read_text()
    paths = [node.text for node in tree.iter() if node.tag in ("uri", "albedo_map")]
    assert len(paths) == 22 and all(Path(path).is_file() for path in paths)
    assert (output / "terrain.obj").resolve().as_posix() in paths
    assert all("%20" not in path and not path.startswith("file:") for path in paths)
    assert tree.find("world/model[@name='site_terrain']/link/visual/material") is None
    assert {entry["surface"] for entry in info["ground_materials"]["palette"]} == {
        "sparse_grass",
        "withered_grass",
        "dirt",
    }
    assert (inputs / "site-rgb.png").read_bytes() == before_colour
    rng = random.Random(site["seed"])
    assert [deer["yaw_radians"] for deer in info["deer"]] == [rng.uniform(-math.pi, math.pi) for _ in range(12)]


def test_ground_cache_is_read_only_and_preparation_preserves_changed_files(task1, tmp_path, monkeypatch):
    import prepare_assets

    cache = tmp_path / "photos"
    asset = {"name": "photo.jpg", "bytes": 3, "sha256": "wrong", "group": "ground"}
    monkeypatch.setattr(prepare_assets, "assets_for", lambda _: [asset])
    monkeypatch.setattr(
        prepare_assets.urllib.request, "urlopen", lambda *_args, **_kwargs: pytest.fail("Unexpected runtime download")
    )
    with pytest.raises(ValueError, match="Ground texture missing.*prepare_assets.py ground"):
        prepare_assets.ground_assets(cache)
    assert not cache.exists()
    cache.mkdir()
    path = cache / asset["name"]
    path.write_bytes(b"old")
    asset["sha256"] = prepare_assets.digest(path)
    assert prepare_assets.ground_assets(cache) == [asset]
    manifest = prepare_assets.prepare_ground(cache)
    assert manifest["files"] == {asset["name"]: asset["sha256"]}
    path.write_bytes(b"changed photo")
    with pytest.raises(ValueError, match="ground texture changed; preserved"):
        prepare_assets.ground_assets(cache)
    with pytest.raises(ValueError, match="preserved"):
        prepare_assets.prepare_ground(cache)
    assert path.read_bytes() == b"changed photo"


def test_ground_detail_keeps_geometry_and_is_bounded_and_repeatable(task1, ground_fixture, tmp_path):
    import cv2
    import numpy as np

    world, _, site = task1
    inputs, cache, assets = ground_fixture
    before = {
        path: path.read_bytes() for path in (inputs / "site-rgb.png", *(cache / asset["name"] for asset in assets))
    }
    yy, xx = np.indices((160, 128))
    gradient = np.stack((xx * 2 % 256, yy * 3 % 256, (xx + yy) % 256), axis=2).astype(np.uint8)
    assert cv2.imwrite(str(inputs / "site-rgb.png"), gradient)
    before[inputs / "site-rgb.png"] = (inputs / "site-rgb.png").read_bytes()
    outputs = [tmp_path / name for name in ("detail first café #1", "detail second")]
    materials = []
    heights = [[715 + 0.2 * x + 0.1 * y for x in range(65)] for y in range(65)]
    plain = tmp_path / "original.obj"
    world.terrain(site, heights, plain)
    for output in outputs:
        output.mkdir()
        data = world.ground_materials(inputs, output, cache)
        world.terrain(site, heights, output / "terrain.obj", data)
        materials.append(data)
        assert len(data["palette"]) == 24 and data["tile_m"] == 2
        assert all(
            0 <= channel <= 1 and math.isfinite(channel) for item in data["palette"] for channel in item["diffuse_rgb"]
        )
        mtl = (output / "terrain.mtl").read_text()
        assert mtl.count("newmtl ") == 24 and mtl.count("Pr 1\nPm 0\n") == 24
        for line in mtl.splitlines():
            if line.startswith("map_Kd "):
                name = line.split(" ", 1)[1]
                assert not Path(name).is_absolute() and (output / name).is_file()
        rows = (output / "terrain.obj").read_text().splitlines()
        original = plain.read_text().splitlines()
        for prefix in ("v ", "vn "):
            assert [row for row in rows if row.startswith(prefix)] == [
                row for row in original if row.startswith(prefix)
            ]
        faces = [row for row in rows if row.startswith("f ")]
        assert len(faces) == 8192 and sorted(faces) == sorted(row for row in original if row.startswith("f "))
        groups = [row for row in rows if row.startswith("o ")]
        assert len(groups) == len(set(groups)) == 24
        vertices = [list(map(float, row.split()[1:])) for row in rows if row.startswith("v ")]
        uv = [list(map(float, row.split()[1:])) for row in rows if row.startswith("vt ")]
        assert all(math.dist(point, (vertex[0] / 2, vertex[1] / 2)) < 1e-6 for point, vertex in zip(uv, vertices))
    assert materials[0] == materials[1]
    assert (outputs[0] / "terrain.obj").read_bytes() == (outputs[1] / "terrain.obj").read_bytes()
    assert (outputs[0] / "terrain.mtl").read_bytes() == (outputs[1] / "terrain.mtl").read_bytes()
    assert all(path.read_bytes() == data for path, data in before.items())


def test_antlerless_preserves_binary_and_rejects_changed_hierarchy(task1):
    _, assets, site = task1
    nodes = [{"name": "part"} for _ in range(43)]
    for index in site["deer"]["antler_nodes"]:
        nodes[index]["name"] = "antler tine"
    nodes[42]["children"] = list(range(42))
    data = {"scene": 0, "scenes": [{"nodes": [42]}], "nodes": nodes}

    def bundle(document):
        encoded = json.dumps(document).encode()
        encoded += b" " * (-len(encoded) % 4)
        return (
            struct.pack("<III", 0x46546C67, 2, 28 + len(encoded))
            + struct.pack("<II", len(encoded), 0x4E4F534A)
            + encoded
            + struct.pack("<II", 0, 0x004E4942)
        )

    original = bundle(data)
    result = assets.antlerless(original, site["deer"]["antler_nodes"])
    assert struct.unpack_from("<I", result, 8)[0] == len(result)
    count = struct.unpack_from("<I", result, 12)[0]
    changed = json.loads(result[20 : 20 + count])
    assert len(changed["nodes"][42]["children"]) == 30 and result[-8:] == original[-8:]
    assert changed["nodes"][20] == nodes[20]  # Preserve the ear that owns the laminated tag.
    nodes[7]["name"] = "ear"
    with pytest.raises(ValueError, match="non-antler"):
        assets.antlerless(bundle(data), site["deer"]["antler_nodes"])


def test_cache_rejects_changed_definition_and_pixels(task1, tmp_path):
    _, assets, site = task1
    (tmp_path / "image").write_bytes(b"old pixels")
    manifest = {"site": site, "files": {"image": assets.digest(tmp_path / "image")}}
    (tmp_path / "inputs.json").write_text(json.dumps(manifest))
    assert assets.prepare(site, tmp_path) == manifest
    changed = dict(site, seed=174)
    with pytest.raises(ValueError, match="definition changed"):
        assets.prepare(changed, tmp_path)
    (tmp_path / "image").write_bytes(b"new pixels")
    with pytest.raises(ValueError, match="input changed"):
        assets.verify(tmp_path)


def test_capture_failure_stops_owned_server(task1, tmp_path, monkeypatch):
    world, _, _ = task1
    (tmp_path / "run.id").write_text("run-test\n")
    process = object()
    stopped = []
    monkeypatch.setattr(world.subprocess, "Popen", lambda *args, **kwargs: process)

    def failed_capture(*args, **kwargs):
        raise TimeoutError("camera unavailable")

    monkeypatch.setattr(world, "capture", failed_capture)
    monkeypatch.setattr(world, "stop_owned", lambda child, run: stopped.append((child, run)))
    with pytest.raises(TimeoutError, match="camera unavailable"):
        world.preview(tmp_path, 10)
    assert stopped == [(process, "run-test")]
    assert json.loads((tmp_path / "cleanup.json").read_text())["owned_sessions_stopped"]


@pytest.mark.parametrize("cleanup_fault", [False, True])
def test_gui_crash_fails_and_attempts_both_cleanups(task1, tmp_path, monkeypatch, cleanup_fault):
    world, _, _ = task1
    (tmp_path / "run.id").write_text("run-test\n")
    (tmp_path / "scene.json").write_text(json.dumps({"camera_pose_enu_rpy": [0, 0, 8, 0, 0, 0]}))
    server = type("Server", (), {"poll": lambda self: None})()
    gui = type("Gui", (), {"poll": lambda self: -6})()
    processes, stopped = iter([server, gui]), []
    monkeypatch.setattr(world.subprocess, "Popen", lambda *args, **kwargs: next(processes))
    monkeypatch.setattr(world, "capture", lambda *args, **kwargs: None)
    read = Path.read_text
    monkeypatch.setattr(
        Path,
        "read_text",
        lambda path: (
            "<camera_pose>-6 0 6 0 0.5 0</camera_pose>"
            if str(path).replace("\\", "/").startswith("/usr/share/")
            else read(path)
        ),
    )

    def stop(child, run):
        stopped.append(child)
        if cleanup_fault and child is gui:
            raise RuntimeError("injected cleanup fault")

    monkeypatch.setattr(world, "stop_owned", stop)
    message = "cleanup failed" if cleanup_fault else "GUI failed"
    with pytest.raises(RuntimeError, match=message):
        world.preview(tmp_path, 10)
    assert stopped == [gui, server]
    assert json.loads(read(tmp_path / "cleanup.json"))["owned_sessions_stopped"] != cleanup_fault


@pytest.mark.parametrize("width,format_type", [(640, "RGB_INT8"), (1280, "BGR_INT8"), (1280, "RGB_INT8")])
def test_camera_rejects_wrong_geometry_format_or_count(task1, tmp_path, monkeypatch, width, format_type):
    world, _, _ = task1
    response = {"width": width, "height": 720, "step": 3840, "pixelFormatType": format_type, "data": "AAAA"}
    monkeypatch.setattr(
        world.subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess([], 0, stdout=json.dumps(response).encode()),
    )
    with pytest.raises(ValueError, match="differs"):
        world.capture(tmp_path, "camera", "/test", (1280, 720), {})
    assert not (tmp_path / "camera.png").exists()
