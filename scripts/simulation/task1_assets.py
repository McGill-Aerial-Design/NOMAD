# SPDX-License-Identifier: Apache-2.0
"""Freeze small public site inputs; never download the full satellite COG or install tools."""

import ctypes as ct
import ctypes.util
import hashlib
import json
import math
import struct
import urllib.parse
import urllib.request

from prepare_assets import digest


def fetch(url, limit=2 * 1024 * 1024):
    request = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(request, timeout=30) as response:
        data = response.read(limit + 1)
    if len(data) > limit:
        raise ValueError("Site response exceeds its bounded input size")
    return data


def antlerless(data, removed):
    if data[:4] != b"glTF" or struct.unpack_from("<I", data, 4)[0] != 2:
        raise ValueError("Expected pinned GLB 2")
    size, kind = struct.unpack_from("<II", data, 12)
    if kind != 0x4E4F534A:
        raise ValueError("Expected GLB JSON first")
    model = json.loads(data[20 : 20 + size])
    root = model["nodes"][42]
    if model["scenes"][model["scene"]]["nodes"] != [42] or root["children"] != list(range(42)):
        raise ValueError("Pinned deer hierarchy changed")
    if any(
        "antler" not in model["nodes"][index]["name"].lower() and "tine" not in model["nodes"][index]["name"].lower()
        for index in removed
    ):
        raise ValueError("Refusing to remove a non-antler node")
    root["children"] = [index for index in root["children"] if index not in removed]
    encoded = json.dumps(model, separators=(",", ":")).encode()
    encoded += b" " * (-len(encoded) % 4)
    tail = data[20 + size :]
    return (
        struct.pack("<III", 0x46546C67, 2, 20 + len(encoded) + len(tail))
        + struct.pack("<II", len(encoded), kind)
        + encoded
        + tail
    )


def satellite_crop(site, destination):
    """Use the already-installed GDAL C API for bounded COG window reads and CRS reprojection."""
    library = ctypes.util.find_library("gdal")
    if not library:
        raise RuntimeError("Existing WSL GDAL library required; no automatic installation")
    gdal = ct.CDLL(library)
    signatures = {
        "GDALAllRegister": (None, []),
        "CPLSetConfigOption": (None, [ct.c_char_p, ct.c_char_p]),
        "GDALOpen": (ct.c_void_p, [ct.c_char_p, ct.c_int]),
        "GDALClose": (None, [ct.c_void_p]),
        "GDALWarpAppOptionsNew": (ct.c_void_p, [ct.POINTER(ct.c_char_p), ct.c_void_p]),
        "GDALWarpAppOptionsFree": (None, [ct.c_void_p]),
        "GDALWarp": (
            ct.c_void_p,
            [ct.c_char_p, ct.c_void_p, ct.c_int, ct.POINTER(ct.c_void_p), ct.c_void_p, ct.POINTER(ct.c_int)],
        ),
    }
    for name, (result, arguments) in signatures.items():
        function = getattr(gdal, name)
        function.restype, function.argtypes = result, arguments
    gdal.GDALAllRegister()
    for key, value in {
        "GDAL_HTTP_TIMEOUT": "25",
        "GDAL_HTTP_MAX_RETRY": "0",
        "GDAL_DISABLE_READDIR_ON_OPEN": "EMPTY_DIR",
        "GDAL_CACHEMAX": "64",
    }.items():
        gdal.CPLSetConfigOption(key.encode(), value.encode())
    source = gdal.GDALOpen(("/vsicurl/" + site["imagery"]["url"]).encode(), 0)
    if not source:
        raise RuntimeError("Cannot open the public Sentinel COG")
    arguments = [
        "-of",
        "GTiff",
        "-t_srs",
        "EPSG:4326",
        "-te",
        *map(str, site["imagery_bbox_w_s_e_n"]),
        "-ts",
        "128",
        "160",
        "-r",
        "bilinear",
        "-wm",
        "32",
    ]
    options = gdal.GDALWarpAppOptionsNew(
        (ct.c_char_p * (len(arguments) + 1))(*[value.encode() for value in arguments], None), None
    )
    output = None
    try:
        error = ct.c_int()
        output = gdal.GDALWarp(str(destination).encode(), None, 1, (ct.c_void_p * 1)(source), options, ct.byref(error))
        if not output or error.value:
            raise RuntimeError("Bounded Sentinel crop failed")
    finally:
        if output:
            gdal.GDALClose(output)
        gdal.GDALWarpAppOptionsFree(options)
        gdal.GDALClose(source)


def elevation_grid(site, root):
    # 16 small, ordered requests sample one coherent CDEM datum; no HRDEM hole filling.
    west, south, east, north = site["imagery_bbox_w_s_e_n"]
    values = [[None] * 65 for _ in range(65)]
    for by in range(4):
        for bx in range(4):
            ordered = [
                (by * 16 + y, bx * 16 + x) for y in range(17) for x in (range(17) if y % 2 == 0 else range(16, -1, -1))
            ]
            points = [(west + (east - west) * x / 64, north - (north - south) * y / 64) for y, x in ordered]
            path = "LINESTRING(" + ",".join(f"{lon:.6f} {lat:.6f}" for lon, lat in points) + ")"
            url = site["terrain"]["url"] + "?" + urllib.parse.urlencode({"path": path})
            response = fetch(url)
            (root / f"cdem-{by}-{bx}.json").write_bytes(response)
            rows = json.loads(response)
            if len(rows) != len(ordered):
                raise ValueError("CDEM response count differs from requested vertices")
            for (y, x), (lon, lat), row in zip(ordered, points, rows, strict=True):
                value = row["altitude"]
                coordinates = row["geometry"]["coordinates"]
                if value is None or not math.isfinite(value) or not 600 <= value <= 850 or not row["vertex"]:
                    raise ValueError("Unavailable or implausible CDEM vertex")
                if max(abs(lon - coordinates[0]), abs(lat - coordinates[1])) > 0.00011:
                    raise ValueError("CDEM response order/coordinates differ")
                if values[y][x] is not None and abs(values[y][x] - value) > 0.02:
                    raise ValueError("Shared terrain vertices disagree")
                values[y][x] = value
    (root / "elevation.json").write_text(json.dumps(values) + "\n")


def prepare(site, root):
    if root.exists():
        manifest = verify(root)
        if manifest["site"] != site:
            raise ValueError("Site definition changed; preserve this cache and prepare a fresh input directory")
        return manifest
    from PIL import Image

    root.mkdir(parents=True)
    try:
        deer = site["deer"]
        data = fetch(deer["url"], deer["bytes"])
        if len(data) != deer["bytes"] or hashlib.sha256(data).hexdigest() != deer["sha256"]:
            raise ValueError("Downloaded deer differs from the explicit asset pin")
        (root / deer["name"]).write_bytes(data)
        (root / "antlerless.glb").write_bytes(
            antlerless((root / site["deer"]["name"]).read_bytes(), site["deer"]["antler_nodes"])
        )
        (root / "sentinel-item.json").write_bytes(fetch(site["imagery"]["item"]))
        satellite_crop(site, root / "site-rgb.tif")
        with Image.open(root / "site-rgb.tif") as image:
            if image.size != (128, 160):
                raise ValueError("Unexpected site crop dimensions")
            image.convert("RGB").save(root / "site-rgb.png")
        elevation_grid(site, root)
        manifest = {
            "site": site,
            "files": {path.name: digest(path) for path in sorted(root.iterdir()) if path.is_file()},
        }
        (root / "inputs.json").write_text(json.dumps(manifest, indent=2) + "\n")
        return manifest
    except BaseException:
        # Keep the failed acquisition visible; never silently reuse or replace partial inputs.
        (root / "INCOMPLETE.txt").write_text("Preparation failed; inspect/preserve this owned input directory.\n")
        raise


def verify(root):
    manifest = json.loads((root / "inputs.json").read_text())
    for relative, expected in manifest["files"].items():
        path = (root / relative).resolve(strict=True)
        path.relative_to(root.resolve())
        if digest(path) != expected:
            raise ValueError("Prepared site input changed: " + relative)
    return manifest
