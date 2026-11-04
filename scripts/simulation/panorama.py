# SPDX-License-Identifier: Apache-2.0
"""Project fixed, ordinary camera views from the pinned photographed 360-degree source."""

import argparse
import hashlib
import json
import math
from pathlib import Path

SOURCE_SHA256 = "b11be77cdeca832d44d5067521444feac410d062c476ba4a95666afb4dfa758e"


# Independently annotated watering-can outline in panorama-00, including handle and spout.
OBJECT_OUTLINE = [
    (295, 241),
    (305, 235),
    (326, 235),
    (343, 241),
    (345, 252),
    (379, 250),
    (352, 266),
    (343, 270),
    (339, 276),
    (320, 278),
    (307, 273),
    (302, 259),
    (295, 252),
]


def rotation(yaw, pitch, roll):
    y, p, r = map(math.radians, (yaw, pitch, roll))
    cy, sy, cp, sp, cr, sr = math.cos(y), math.sin(y), math.cos(p), math.sin(p), math.cos(r), math.sin(r)
    return (
        (cy * cp, -cy * sp * sr - sy * cr, -cy * sp * cr + sy * sr),
        (sy * cp, -sy * sp * sr + cy * cr, -sy * sp * cr - cy * sr),
        (sp, cp * sr, cp * cr),
    )


def camera_view(source, pose):
    import cv2
    import numpy as np

    yy, xx = np.indices((480, 640), dtype=np.float64)
    directions = np.stack((np.ones_like(xx), -(xx - 320) / 320, -(yy - 240) / 320), axis=-1)
    rays = directions @ np.asarray(rotation(*pose)).T
    longitude = np.arctan2(rays[:, :, 1], rays[:, :, 0])
    latitude = np.arctan2(rays[:, :, 2], np.hypot(rays[:, :, 0], rays[:, :, 1]))
    u = ((longitude / (2 * math.pi) + 0.5) * source.shape[1]).astype(np.float32)
    v = ((0.5 - latitude / math.pi) * source.shape[0]).astype(np.float32)
    return cv2.remap(source, u, v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)


def world_ray(pixel, pose):
    x, y = pixel
    ray = (1, -(x - 320) / 320, -(y - 240) / 320)
    return tuple(sum(a * b for a, b in zip(row, ray)) for row in rotation(*pose))


def project(ray, pose):
    matrix = rotation(*pose)
    x, y, z = (sum(matrix[row][column] * ray[row] for row in range(3)) for column in range(3))
    if x <= 1e-6:
        return None
    return [320 - 320 * y / x, 240 - 320 * z / x]


def object_projection(pose):
    return [project(world_ray(point, (75, -12, 0)), pose) for point in OBJECT_OUTLINE]


def generate(source_path, output, preview=False):
    import cv2

    if hashlib.sha256(source_path.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError("Panorama source checksum changed")
    source = cv2.imread(str(source_path))
    if source is None or source.shape[:2] != (4096, 8192):
        raise ValueError("Expected the pinned 8192x4096 panorama")
    output.mkdir(parents=True, exist_ok=True)
    initial = (75, -12, 0)
    if preview:
        cv2.imwrite(str(output / "courtyard-preview.png"), camera_view(source, initial))
        return
    selection = [285, 238, 70, 64]
    x, y, width, height = selection
    initial_pixels = [(x, y), (x + width, y), (x + width, y + height), (x, y + height)]
    rays = [world_ray(pixel, initial) for pixel in initial_pixels]
    aim = world_ray((x + width / 2, y + height / 2), initial)
    offsets = [
        0,
        0,
        0,
        5,
        10,
        20,
        30,
        45,
        60,
        90,
        120,
        150,
        179,
        180,
        210,
        240,
        270,
        300,
        315,
        330,
        338,
        344,
        350,
        352,
        354,
        356,
        358,
        360,
        360,
        360,
    ]
    frames = []
    for index, offset in enumerate(offsets):
        returning = index >= 19
        pose = (
            initial[0] + offset,
            initial[1] + (3 if returning and offset < 360 else 0),
            3 if returning and offset < 360 else 0,
        )
        name = f"panorama-{index:02}.jpg"
        image = camera_view(source, pose)
        if not cv2.imwrite(str(output / name), image, [cv2.IMWRITE_JPEG_QUALITY, 95]):
            raise OSError("Panorama frame write failed")
        points = [project(ray, pose) for ray in rays]
        visible = all(point is not None and 4 <= point[0] < 636 and 4 <= point[1] < 476 for point in points)
        frames.append(
            {
                "file": name,
                "pose_deg": pose,
                "returning": returning,
                "visible": visible,
                "corners": points,
                "object_outline": object_projection(pose),
                "aim": project(aim, pose),
                "sha256": hashlib.sha256((output / name).read_bytes()).hexdigest(),
            }
        )
    manifest = {
        "source_sha256": SOURCE_SHA256,
        "source": "https://polyhaven.com/a/courtyard",
        "author": "Greg Zaal / Poly Haven",
        "license": "CC0",
        "selection": selection,
        "object_annotation": (
            "Manual watering-can silhouette in initial view, independent of tracker; camera rotation projects its rays"
        ),
        "initial_object_outline": OBJECT_OUTLINE,
        "camera": {"width": 640, "height": 480, "fx": 320, "fy": 320, "cx": 320, "cy": 240},
        "truth": "Forward projection of immutable world rays; generator uses inverse equirectangular sampling",
        "scope": "Photographed stationary panorama; camera rotation only, no motion/parallax/capture timing",
        "seed": 173,
        "jpeg_quality": 95,
        "frames": frames,
    }
    (output / "panorama.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()
    generate(args.source, args.output, args.preview)
