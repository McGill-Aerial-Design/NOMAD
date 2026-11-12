# SPDX-License-Identifier: Apache-2.0
"""Optional GUI-launched development session. Owns simulators, never reads MAVLink."""

import argparse
import hashlib
import ipaddress
import json
import math
import os
import re
import select
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

from interactive_world import SITL_EXECUTABLES, asset_path, component
from interactive_world import create as create_world
from owned_processes import stop as stop_owned
from rtp_relay import RtpRelay


def recipe(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("Duplicate scenario field: " + key)
            result[key] = value
        return result

    if path.stat().st_size > 65536:
        raise ValueError("Scenario exceeds 64 KiB")
    raw = path.read_bytes()
    value = json.loads(raw, object_pairs_hook=unique)
    if (
        type(value.get("schema")) is not int
        or value["schema"] != 1
        or value.get("backend") not in ("sitl", "gazebo", "last-letter")
    ):
        raise ValueError("Unsupported scenario schema/backend")
    for name in ("name", "frame"):
        if not isinstance(value.get(name), str) or not value[name].strip() or any(ord(c) < 32 for c in value[name]):
            raise ValueError("Invalid scenario " + name)
    vehicle = value.get("vehicle", "ArduCopter")
    if not isinstance(vehicle, str) or vehicle not in SITL_EXECUTABLES:
        raise ValueError("Scenario vehicle must be ArduCopter or ArduPlane")
    if type(value.get("system_id", 201)) is not int or not 1 <= value.get("system_id", 201) <= 254:
        raise ValueError("Simulator system ID must be 1..254")
    if "home" in value:
        check_home(value["home"])
    for key in ("defaults", "defaults_local"):
        if not isinstance(value.get(key, []), list) or any(
            not isinstance(p, str) or not p.strip() or any(ord(c) < 32 for c in p) for p in value.get(key, [])
        ):
            raise ValueError("Defaults must be file paths")
    if type(value.get("seed", 0)) is not int or not 0 <= value.get("seed", 0) <= 4294967295:
        raise ValueError("Seed must be an unsigned 32-bit integer")
    adapter = value.get("wsl_graphics_adapter", "")
    if not isinstance(adapter, str) or len(adapter) > 256 or any(ord(c) < 32 for c in adapter):
        raise ValueError("WSLg graphics adapter must be a device-name substring")
    if adapter and value["backend"] != "gazebo":
        raise ValueError("A graphics adapter requires a Gazebo scenario")
    if "physics_hz" in value and (
        value["backend"] != "gazebo" or type(value["physics_hz"]) is not int or value["physics_hz"] not in (400, 1000)
    ):
        raise ValueError("physics_hz must be 400 or 1000 for a Gazebo scenario")
    if "camera" in value:
        camera = value["camera"]
        if (
            value["backend"] != "gazebo"
            or not isinstance(value.get("camera_model"), str)
            or not value.get("camera_model")
            or not isinstance(camera, dict)
            or not {"width", "height", "fps"} <= set(camera) <= {"width", "height", "fps", "use_cuda", "bitrate_kbps"}
            or any(type(camera[key]) is not int for key in ("width", "height", "fps"))
            or ("use_cuda" in camera and type(camera["use_cuda"]) is not bool)
            or (
                "bitrate_kbps" in camera
                and (type(camera["bitrate_kbps"]) is not int or not 100 <= camera["bitrate_kbps"] <= 50000)
            )
            or not 16 <= camera["width"] <= 1920
            or not 16 <= camera["height"] <= 1080
            or camera["width"] % 2
            or camera["height"] % 2
            or not 1 <= camera["fps"] <= 30
        ):
            raise ValueError("camera requires a Gazebo camera_model, even dimensions up to 1920x1080 and fps 1..30")
    if "wind" in value:
        wind = value["wind"]
        if (
            value["backend"] != "gazebo"
            or not isinstance(wind, list)
            or len(wind) != 3
            or not all(type(v) in (int, float) and math.isfinite(v) for v in wind)
            or any(abs(v) > 30 for v in wind)
        ):
            raise ValueError("Wind requires a Gazebo scenario and three finite ENU values up to 30 m/s")
    value["_sha256"] = hashlib.sha256(raw).hexdigest()
    return value


def last_letter_models(source, destination, model, home):
    if not re.fullmatch(r"[A-Za-z0-9_-]+", model):
        raise ValueError("Invalid last_letter model name")
    shutil.copytree(source, destination)
    initial = (destination / "aircraft" / model / "init.yaml").resolve(strict=True)
    initial.relative_to(destination.resolve())
    text, count = re.subn(
        r"(?m)^(\s*coordinates:\s*)\[[^\n]*?\]", lambda match: match[1] + json.dumps(home[:3]), initial.read_text()
    )
    if count != 1:
        raise ValueError("The selected last_letter model needs one coordinates entry in init.yaml")
    yaw = math.radians(home[3]) / 2
    text, count = re.subn(
        r"(?m)^(\s*orientation:\s*)\[[^\n]*?\]",
        lambda match: match[1] + json.dumps([0, 0, math.sin(yaw), math.cos(yaw)]),
        text,
    )
    if count != 1:
        raise ValueError("The selected last_letter model needs one initial orientation entry")
    initial.write_text(text)
    return destination


def check_home(home):
    if (
        not isinstance(home, list)
        or len(home) != 4
        or not all(type(v) in (int, float) and math.isfinite(v) for v in home)
    ):
        raise ValueError("Home needs finite latitude, longitude, altitude and heading")
    if (
        not -90 <= home[0] <= 90
        or not -180 <= home[1] <= 180
        or not -500 <= home[2] <= 10000
        or not -360 <= home[3] <= 360
    ):
        raise ValueError("Home coordinate is outside supported bounds")


def port(kind):
    with socket.socket(socket.AF_INET, kind) as listener:
        listener.bind(("127.0.0.1", 0))
        return listener.getsockname()[1]


def wait_for_gui(gui, rendered, deadline, parent_alive):
    while parent_alive():
        code = gui.poll()
        if code is not None:
            raise RuntimeError(f"GUI exited before its first rendered frame (code {code}); inspect gui.log")
        if time.monotonic() >= deadline:
            raise TimeoutError("GUI did not render a camera before the startup deadline; inspect gui.log")
        if rendered.wait(0.1):
            return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, default=Path.home() / "nomad-simulation")
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--world", type=Path)
    parser.add_argument("--duration-seconds", type=int, default=3600)
    parser.add_argument("--no-gui", action="store_true")
    args = parser.parse_args()
    if not 60 <= args.duration_seconds <= 86400:
        raise ValueError("Session duration must be 1 minute..24 hours")
    run = args.run.resolve(strict=True)
    run_id = run.name
    if not run_id.startswith("run-") or (run / "run.id").read_text().strip() != run_id:
        raise ValueError("Invalid session ownership marker")
    workspace = args.workspace.resolve(strict=True)
    if (workspace / ".nomad-simulation-owned").read_text().strip() != "NOMAD simulation sources v1":
        raise ValueError("Choose a prepared, owned simulator workspace")
    scenario_path = args.scenario.resolve(strict=True)
    scenario = recipe(scenario_path)
    if args.world and scenario["backend"] != "gazebo":
        raise ValueError("A world override requires a Gazebo scenario; last_letter is a separate headless backend")
    ap_build = component(workspace, "ardupilot", SITL_EXECUTABLES[scenario.get("vehicle", "ArduCopter")])
    binary = Path(ap_build["binary"])
    home = scenario.get("home")
    processes, relay, stopping = [], None, False
    environment = dict(os.environ, NOMAD_SIMULATION_RUN_ID=run_id, GZ_PARTITION=run_id)
    result = {
        "scenario": scenario,
        "scenario_file": str(scenario_path),
        "vehicle": scenario.get("vehicle", "ArduCopter"),
        "ardupilot_build": ap_build,
        "ready": False,
        "cleanup_errors": [],
        "started_monotonic_ns": time.monotonic_ns(),
    }

    def stop_requested(_signal=None, _frame=None):
        nonlocal stopping
        stopping = True

    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(signum, stop_requested)

    def parent_alive():
        nonlocal stopping
        if select.select([sys.stdin], [], [], 0)[0]:
            if sys.stdin.readline().strip() in ("", "STOP"):
                stopping = True
        return not stopping

    def launch(command, name, env=environment):
        if not parent_alive():
            raise InterruptedError("Session was stopped before process launch")
        with (run / (name + ".log")).open("w") as log:
            child = subprocess.Popen(
                command, cwd=run, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
        processes.append((name, child))
        return child

    try:
        tcp_port, fdm_port = port(socket.SOCK_STREAM), port(socket.SOCK_DGRAM)
        dynamics = scenario["backend"]
        world = None
        if dynamics == "gazebo":
            relay = RtpRelay(udp_port=0, tcp_port=0, tcp_host="0.0.0.0")
            world = create_world(
                workspace, run, scenario, scenario_path, fdm_port, relay.udp.getsockname()[1], args.world
            )
            home = world["home"]
            for key, name in {
                "HOME": "gazebo-home",
                "GZ_FUEL_CACHE_PATH": "fuel-cache",
                "XDG_CACHE_HOME": "renderer-cache",
                "GZ_LOG_PATH": "gz-logs",
            }.items():
                (run / name).mkdir()
                environment[key] = str(run / name)
            environment.update(
                {
                    "GZ_SIM_SYSTEM_PLUGIN_PATH": world["plugins"],
                    "GZ_SIM_RESOURCE_PATH": os.pathsep.join(world["resources"]),
                    "QT_QPA_PLATFORM": os.environ.get("QT_QPA_PLATFORM", "xcb"),
                }
            )
            graphics = dict(environment)
            adapter = scenario.get("wsl_graphics_adapter", "")
            if Path("/mnt/wslg").exists():
                graphics.setdefault("GALLIUM_DRIVER", "d3d12")
            if adapter:
                if not Path("/mnt/wslg").exists() or graphics.get("GALLIUM_DRIVER") != "d3d12":
                    raise ValueError("wsl_graphics_adapter requires the WSLg D3D12 renderer")
                graphics["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = adapter
            # OpenCV's bitmap generation must not redirect the native Gazebo Qt loader.
            for key in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR"):
                if "cv2" in graphics.get(key, ""):
                    graphics.pop(key)
            result["renderer_environment"] = {
                key: graphics.get(key, "") for key in ("GALLIUM_DRIVER", "MESA_D3D12_DEFAULT_ADAPTER_NAME")
            }
            launch(
                ["gz", "sim", "-s", "-r", "-v3", "--seed", str(scenario.get("seed", 0)), str(world["world"])],
                "gazebo",
                graphics,
            )
        elif dynamics == "last-letter":
            check_home(home)
            ll_build = component(workspace, "last_letter")
            result["last_letter_build"] = ll_build
            model = scenario.get("last_letter_model", "ardupilot_copter")
            models = last_letter_models(
                workspace / "sources/last_letter/models", run / "last-letter-models", model, home
            )
            launch(
                [
                    ll_build["binary"],
                    "--model",
                    model,
                    "--address",
                    "127.0.0.1",
                    "--port",
                    str(fdm_port),
                    "--models-dir",
                    str(models),
                    "--log",
                    str(run / "flight.mcap"),
                ],
                "last-letter",
            )
        check_home(home)
        defaults = [asset_path(name, workspace) for name in scenario.get("defaults", [])]
        defaults += [asset_path(name, scenario_path.parent) for name in scenario.get("defaults_local", [])]
        override = run / "session.parm"
        override.write_text(f"MAV_SYSID {scenario.get('system_id', 201)}\nLOG_BACKEND_TYPE 0\n")
        defaults.append(override)
        if any("," in str(p) for p in defaults):
            raise ValueError("ArduPilot defaults paths cannot contain commas")
        command = [
            str(binary),
            "--model",
            scenario["frame"],
            "--speedup",
            "1",
            "--wipe",
            "--home",
            ",".join(map(str, home)),
            "--defaults",
            ",".join(map(str, defaults)),
            "--serial0",
            f"tcp:{tcp_port}",
            "--rc-in-port",
            str(port(socket.SOCK_DGRAM)),
        ]
        for serial in (1, 2, 5, 6, 7, 8):
            command += [f"--serial{serial}", "none"]
        if dynamics in ("gazebo", "last-letter"):
            command += ["--sim-address", "127.0.0.1", "--sim-port-out", str(fdm_port)]
        ap = launch(command, "sitl")
        result["command"] = command
        publisher = node = None
        if world:
            from gz.msgs10.boolean_pb2 import Boolean
            from gz.transport13 import Node, NodeOptions

            transport_options = NodeOptions()
            transport_options.partition = run_id
            node = Node(transport_options)
            if world["camera"]:
                publisher = node.advertise("/nomad/simulation/rgb/enable_streaming", Boolean)
        deadline, enabled = time.monotonic() + 90, False
        while time.monotonic() < deadline and parent_alive():
            for name, child in processes:
                if child.poll() is not None:
                    raise RuntimeError(f"{name} exited during startup (code {child.returncode}); inspect {name}.log")
            listeners = subprocess.check_output(["ss", "-ltnp", "sport", "=", f":{tcp_port}"], text=True)
            if f"pid={ap.pid}," in listeners:
                if world and world["camera"]:
                    # Upstream StartStreaming is not idempotent. Wait for its subscriber, then enable once.
                    if not enabled and publisher.has_connections():
                        if not publisher.publish(Boolean(data=True)):
                            raise RuntimeError("Gazebo rejected camera enable")
                        enabled = True
                    if relay.stats["received"] == 0:
                        time.sleep(0.1)
                        continue
                break
            time.sleep(0.1)
        else:
            if stopping:
                return
            raise TimeoutError("Simulator startup deadline; inspect logs and prerequisite build manifests")
        if world and not args.no_gui:
            gui_home = run / "gui-home"
            gui_home.mkdir()
            gui_env = dict(graphics, HOME=str(gui_home))
            if Path("/mnt/wslg").exists():
                gui_env.setdefault("QT_XCB_GL_INTEGRATION", "xcb_glx")
            gui_env["QT_LOGGING_RULES"] = (
                gui_env.get("QT_LOGGING_RULES", "") + ";qt.qpa.gl=true;qt.scenegraph.general=true"
            )
            result["gui_environment"] = {
                key: gui_env.get(key, "")
                for key in ("QT_QPA_PLATFORM", "QT_XCB_GL_INTEGRATION", "QSG_RENDER_LOOP", "QT_LOGGING_RULES")
            }
            from gz.msgs10.pose_pb2 import Pose

            rendered = threading.Event()
            if not node.subscribe(Pose, "/gui/camera/pose", lambda _: rendered.set()):
                raise RuntimeError("Cannot observe the GUI render-readiness topic")
            config = Path("/usr/share/gz/gz-sim8/gui/gui.config").read_text()
            config = config.replace(
                '<dialog name="quick_start" show_again="true"/>', '<dialog name="quick_start" show_again="false"/>'
            )
            (run / "gui.config").write_text(config)
            gui = launch(["gz", "sim", "-g", "-v3", "--gui-config", str(run / "gui.config")], "gui", gui_env)
            if not wait_for_gui(gui, rendered, deadline, parent_alive):
                return
            result["gui_rendered_monotonic_ns"] = time.monotonic_ns()
        addresses = subprocess.check_output(["hostname", "-I"], text=True).split()
        address = next((str(ipaddress.IPv4Address(value)) for value in addresses if ":" not in value), None)
        if address is None:
            raise RuntimeError("No WSL IPv4 endpoint is available")
        ready = {
            "event": "ready",
            "schema": 1,
            "name": scenario["name"],
            "backend": dynamics,
            "vehicle": scenario.get("vehicle", "ArduCopter"),
            "host": address,
            "mavlink_port": tcp_port,
            "system_id": scenario.get("system_id", 201),
            "video_tcp_port": relay.server.getsockname()[1] if relay and world["camera"] else 0,
            "camera_hfov": world["camera"]["hfov_degrees"] if world and world["camera"] else 90,
            "body_fixed": False,
            "scenario_sha256": scenario["_sha256"],
        }
        result["ready"] = ready
        result["world"] = world
        print(json.dumps(ready), flush=True)
        deadline = time.monotonic() + args.duration_seconds
        next_size_check = 0
        while time.monotonic() < deadline and parent_alive():
            for name, child in processes:
                if child.poll() is not None:
                    if name == "gui" and child.returncode == 0:
                        stopping = True
                    else:
                        raise RuntimeError(name + " exited; inspect the session logs")
            if relay and relay.stats["error"]:
                raise RuntimeError("Video relay: " + str(relay.stats["error"]))
            if time.monotonic() >= next_size_check:
                if sum(p.stat().st_size for p in run.rglob("*") if p.is_file()) > 1024**3:
                    raise RuntimeError("Session artifacts exceeded 1 GiB")
                next_size_check = time.monotonic() + 5
            time.sleep(0.1)
    finally:
        for name, child in reversed(processes):
            try:
                stop_owned(child, run_id)
            except Exception as error:
                result["cleanup_errors"].append(name + ": " + str(error))
        if relay:
            try:
                relay.close()
            except Exception as error:
                result["cleanup_errors"].append("video: " + str(error))
            result["video_relay"] = relay.stats
        result["stopped_monotonic_ns"] = time.monotonic_ns()
        (run / "session.json").write_text(json.dumps(result, indent=2) + "\n")
        if result["cleanup_errors"]:
            raise RuntimeError("Owned simulator cleanup failed")


if __name__ == "__main__":
    main()
