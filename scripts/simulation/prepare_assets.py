# SPDX-License-Identifier: Apache-2.0
"""Fetch small pinned fixtures only. No simulator/toolchain installation or product startup."""

import argparse
import hashlib
import json
import urllib.request
import uuid
import zipfile
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def assets_for(group):
    return [
        asset
        for asset in json.loads(Path(__file__).with_name("assets.json").read_text(encoding="utf-8"))["assets"]
        if asset["group"] == group
    ]


def ground_assets(root):
    assets = assets_for("ground")
    for asset in assets:
        path = root / asset["name"]
        if not path.is_file():
            raise ValueError(
                "Ground texture missing: " + asset["name"] + "; run python3 scripts/simulation/prepare_assets.py ground"
            )
        if path.stat().st_size != asset["bytes"] or digest(path) != asset["sha256"]:
            raise ValueError("Prepared ground texture changed; preserved: " + str(path))
    return assets


def prepare_ground(root):
    root.mkdir(parents=True, exist_ok=True)
    assets = assets_for("ground")
    for asset in assets:
        acquire(root, asset)
    manifest = {
        "schema": 1,
        "assets": assets,
        "files": {asset["name"]: digest(root / asset["name"]) for asset in assets},
    }
    (root / "prepared-ground.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def acquire(root, asset):
    path = root / asset["name"]
    if not path.exists():
        with urllib.request.urlopen(asset["url"], timeout=30) as response:
            data = response.read(asset["bytes"] + 1)
        if len(data) != asset["bytes"] or hashlib.sha256(data).hexdigest() != asset["sha256"]:
            raise ValueError("Downloaded fixture differs from its explicit pin: " + asset["name"])
        path.write_bytes(data)
    if path.stat().st_size != asset["bytes"] or digest(path) != asset["sha256"]:
        raise ValueError("Existing fixture differs from its pin; preserved: " + str(path))
    return path


def extract_model(archive, target):
    target.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive) as bundle:
        if sum(item.file_size for item in bundle.infolist()) > 32 * 1024 * 1024:
            raise ValueError("Scene model exceeds 32 MiB unpacked")
        for item in bundle.infolist():
            destination = (target / item.filename).resolve()
            destination.relative_to(target.resolve())
            if item.is_dir():
                destination.mkdir(parents=True, exist_ok=True)
                continue
            if (item.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError("Scene model contains a symbolic link")
            data = bundle.read(item)
            if destination.exists() and destination.read_bytes() != data:
                raise ValueError("Existing scene asset changed; preserved: " + str(destination))
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)


def prepare_video(root, assets, output):
    import cv2
    import numpy as np

    if output.exists():
        raise ValueError("A public replay dataset must be a fresh directory")
    output.mkdir(parents=True)
    (output / "run.id").write_text(output.name.removeprefix("run-") + "\n")
    frames = output / "frames"
    frames.mkdir()
    rows = [line.split() for line in (root / "nl_bookI_s1.txt").read_text().splitlines()[1:]]
    corners = [np.array([float(value) for value in row[1:]], dtype=np.float32).reshape(4, 2) for row in rows]
    # A textured interior of the book; context remains on the same plane. Chosen before the first run.
    selection = [345, 345, 55, 70]
    point = np.array([[[372.5, 380]]], dtype=np.float32)
    camera = cv2.VideoCapture(str(root / "nl_bookI_s1.avi"))
    truth = {}
    try:
        for index, quad in enumerate(corners):
            ok, image = camera.read()
            if not ok or image.shape[:2] != (600, 800):
                raise ValueError("Public sequence decode/count/geometry mismatch at frame " + str(index + 1))
            name = f"capture_{index + 1:08d}.png"
            if not cv2.imwrite(str(frames / name), image):
                raise OSError("Public replay PNG write failed")
            transform = cv2.getPerspectiveTransform(corners[0], quad)
            aim = cv2.perspectiveTransform(point, transform)[0, 0]
            truth[name] = {"aim": [float(value) for value in aim], "sha256": digest(frames / name)}
        if camera.read()[0]:
            raise ValueError("Public footage has frames absent from its annotation file")
    finally:
        camera.release()
    manifest = {
        "schema": 1,
        "dataset": "TMT nl_bookI_s1",
        "sources": assets,
        "decoder": "OpenCV " + cv2.__version__,
        "selection": selection,
        "frames": truth,
        "truth": "independent four-corner annotations; project initial canonical point, never bbox center",
        "time": "source 30 Hz frame index; offline replay is not a video-latency measurement",
        "no_reseeding": True,
    }
    (output / "replay-truth.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("group", choices=("scene", "video", "panorama", "ground"))
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    root = repo / "local/simulation/assets"
    if args.group == "ground":
        root /= "ground"
        prepare_ground(root)
        print(root)
        return
    root.mkdir(parents=True, exist_ok=True)
    assets = assets_for(args.group)
    for asset in assets:
        path = acquire(root, asset)
        if "model" in asset:
            extract_model(path, root / "models" / asset["model"])
    if args.group == "scene":
        files = [root / "aerial_grass_rock_diff_1k.jpg", *sorted((root / "models").rglob("*"))]
        manifest = {
            "schema": 1,
            "assets": assets,
            "files": {str(path.relative_to(root)): digest(path) for path in files if path.is_file()},
        }
        (root / "prepared-scene.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(root)
    elif args.group == "video":
        print(prepare_video(root, assets, repo / "local/simulation/datasets" / ("run-" + uuid.uuid4().hex)))
    else:
        from panorama import generate

        output = repo / "local/simulation/panorama"
        generate(root / "courtyard.jpg", output)
        print(output)


if __name__ == "__main__":
    main()
