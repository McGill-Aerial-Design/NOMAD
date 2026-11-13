# SPDX-License-Identifier: Apache-2.0
"""Own one isolated SITL process. Closing the controlling stdin stops only its process group."""

import argparse
import hashlib
import ipaddress
import json
import os
import select
import signal
import socket
import subprocess
import sys
import time
import uuid
from pathlib import Path

from build_manifest import verify
from owned_processes import stop as stop_owned
from resources import append as record_resources
from rtp_relay import RtpRelay
from world import create as create_world

AP_REVISION = "fb773567896014da007bca58358704d4a0d8516f"


def owned_run(path):
    path = Path(path).resolve(strict=True)
    run_id = uuid.UUID(path.name.removeprefix("run-"))
    if path.name != f"run-{run_id.hex}" or (path / "run.id").read_text().strip() != run_id.hex:
        raise ValueError("Invalid run ownership")
    return path


def available(port):
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("0.0.0.0", port))


def check_renderer(renderer, run):
    if (run / "capture-error.txt").exists():
        raise RuntimeError("Renderer capture/pose recording failed; inspect capture-error.txt")
    if renderer.poll() is not None:
        raise RuntimeError("Owned renderer exited during the case")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True, type=Path)
    parser.add_argument("--run", required=True, type=owned_run)
    dynamics = parser.add_mutually_exclusive_group()
    dynamics.add_argument("--rendered", action="store_true")
    dynamics.add_argument("--last-letter", action="store_true")
    parser.add_argument("--scene", choices=("ground", "photo-board", "natural"), default="ground")
    parser.add_argument("--visible", action="store_true")
    parser.add_argument("--occlusion", action="store_true")
    parser.add_argument("--endurance-seconds", type=int, default=0)
    parser.add_argument("--physics-hz", type=int, choices=(400, 1000), default=1000)
    parser.add_argument("--object-motion", choices=("orbit", "flyby", "direct", "approach"))
    parser.add_argument("--object-altitude", type=int, choices=(5, 15), default=5)
    parser.add_argument("--object-speed", type=int, choices=(5, 10), default=5)
    parser.add_argument("--encoder-kbps", type=int, default=800)
    parser.add_argument("--cpu-only", action="store_true")
    parser.add_argument("--target-case", choices=("static", "moving", "occlusion", "lookalike"), default="static")
    args = parser.parse_args()
    if args.object_motion and (
        not args.rendered or args.occlusion or args.endurance_seconds or args.scene == "photo-board"
    ):
        raise ValueError("Object motion requires rendered ground/natural scenery without another video case")
    if args.object_motion and args.physics_hz != 1000:
        raise ValueError("Object motion requires the 1000 Hz physics baseline")
    if not args.rendered and args.physics_hz != 1000:
        raise ValueError("A nondefault physics rate requires rendered Gazebo")
    if args.occlusion and not args.rendered:
        raise ValueError("Occlusion needs the rendered case")
    if args.endurance_seconds and (not args.rendered or args.occlusion or not 600 <= args.endurance_seconds <= 1800):
        raise ValueError("Invalid bounded endurance duration")
    workspace = args.workspace.expanduser().resolve(strict=True)
    if (workspace / ".nomad-simulation-owned").read_text().strip() != "NOMAD simulation sources v1":
        raise ValueError("Invalid source ownership")
    source = workspace / "sources/ardupilot"
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if revision != AP_REVISION:
        raise ValueError("ArduPilot revision mismatch")
    binary = source / "build/sitl/bin/arducopter"
    build = verify(workspace, "ardupilot", revision, binary)
    binary_hash = build["sha256"]
    available(15760)
    state = args.run / "sitl"
    state.mkdir()
    fixture = args.run / "mount.parm"
    fixture.write_text(
        f"MAV_SYSID 201\nFRAME_CLASS 1\nFRAME_TYPE {0 if args.last_letter else 1}\nLOG_BACKEND_TYPE 0\n"
        "MNT1_TYPE 1\nMNT1_DEFLT_MODE 2\nMNT1_ROLL_MIN -30\nMNT1_ROLL_MAX 30\n"
        "MNT1_PITCH_MIN -135\nMNT1_PITCH_MAX 45\nMNT1_YAW_MIN -160\nMNT1_YAW_MAX 160\n"
        "SERVO9_FUNCTION 8\nSERVO10_FUNCTION 7\nSERVO11_FUNCTION 6\n"
        + "".join(
            f"SERVO{channel}_MIN 1100\nSERVO{channel}_MAX 1900\nSERVO{channel}_TRIM 1500\n" for channel in (9, 10, 11)
        )
    )
    command = [
        str(binary),
        "-I",
        "21",
        "--model",
        "JSON:127.0.0.1" if args.rendered or args.last_letter else "quad",
        "--speedup",
        "1",
        "--home=-35.362938,149.165085,584,0",
        "--defaults",
        str(fixture),
        "--serial0",
        "tcp:15760",
    ]
    if args.rendered or args.last_letter:
        command.extend(["--sim-address", "127.0.0.1", "--sim-port-out", "19202"])
    result = {
        "schema": 1,
        "source": revision,
        "binary_sha256": binary_hash,
        "passed": False,
        "defaults_sha256": hashlib.sha256(fixture.read_bytes()).hexdigest(),
        "command": command,
        "monotonic_start_ns": time.monotonic_ns(),
        "tcp_bind": "0.0.0.0:15760 inside WSL NAT",
        "build": build,
    }
    process = None
    renderer = None
    gui = None
    last_letter = None
    relay = None
    environment = dict(os.environ, NOMAD_SIMULATION_RUN_ID=args.run.name)
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGHUP, stop)
    try:
        record_resources(args.run / "linux-resources.jsonl", set())
        with (args.run / "sitl-console.log").open("w") as log:
            if args.last_letter:
                ll_binary = workspace / "build/last-letter/last_letter_ardupilot"
                ll_build = verify(workspace, "last_letter", "13baa815e971abbcc568bea1e265026b44686273", ll_binary)
                with (args.run / "last-letter-console.log").open("w") as ll_log:
                    last_letter = subprocess.Popen(
                        [
                            str(ll_binary),
                            "--model",
                            "ardupilot_copter",
                            "--address",
                            "127.0.0.1",
                            "--port",
                            "19202",
                            "--models-dir",
                            str(workspace / "sources/last_letter/models"),
                            "--log",
                            str(args.run / "flight.mcap"),
                        ],
                        cwd=state,
                        env=environment,
                        stdout=ll_log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                result["last_letter_build"] = ll_build
            if args.rendered:
                host = str(ipaddress.IPv4Address("127.0.0.1"))
                relay = RtpRelay()
                # Above ~2 Mbit/s, keyframe bursts overflow the default-capped socket and the decoder stays grey.
                if args.encoder_kbps > 2000 and relay.stats["udp_receive_buffer"] < 4 << 20:
                    raise RuntimeError(
                        "Relay UDP buffer is capped by net.core.rmem_max; raise it first: "
                        "wsl -d Ubuntu-24.04 -u root sysctl -w net.core.rmem_max=8388608"
                    )
                world = create_world(
                    workspace,
                    args.run,
                    host,
                    args.scene,
                    args.visible,
                    args.occlusion,
                    bool(args.endurance_seconds),
                    args.physics_hz,
                    dict(
                        kind=args.object_motion,
                        altitude_m=args.object_altitude,
                        speed_m_s=args.object_speed,
                        cpu_only=args.cpu_only,
                        target_case=args.target_case,
                    )
                    if args.object_motion
                    else None,
                    args.encoder_kbps,
                    args.cpu_only,
                )
                result["physics_hz"] = args.physics_hz
                environment.update(
                    {
                        "GALLIUM_DRIVER": "d3d12",
                        "MESA_D3D12_DEFAULT_ADAPTER_NAME": "NVIDIA",
                        "GZ_PARTITION": args.run.name,
                        "GZ_LOG_PATH": str(args.run / "gz-logs"),
                        "GZ_FUEL_CACHE_PATH": str(args.run / "fuel-cache"),
                        "GZ_SIM_SYSTEM_PLUGIN_PATH": (
                            f"{workspace}/build/gazebo-plugin:{workspace}/build/nomad-test-system"
                        ),
                        "GZ_SIM_RESOURCE_PATH": str(args.run / "models"),
                    }
                )
                if args.cpu_only:
                    environment.update(LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe", LP_NUM_THREADS="4")
                    environment.pop("MESA_D3D12_DEFAULT_ADAPTER_NAME", None)
                renderer_environment = dict(environment)
                renderer_environment["QT_QPA_PLATFORM"] = "xcb"
                renderer_environment["QT_XCB_GL_INTEGRATION"] = "xcb_glx"
                renderer_environment["QT_LOGGING_RULES"] = "qt.qpa.gl=true"
                result["graphics"] = {
                    key: renderer_environment.get(key)
                    for key in (
                        "GALLIUM_DRIVER",
                        "MESA_D3D12_DEFAULT_ADAPTER_NAME",
                        "LIBGL_ALWAYS_SOFTWARE",
                        "LP_NUM_THREADS",
                        "QT_QPA_PLATFORM",
                        "QT_XCB_GL_INTEGRATION",
                        "QT_LOGGING_RULES",
                    )
                }
                private_home = args.run / "gazebo-home"
                private_home.mkdir()
                (args.run / "fuel-cache").mkdir()
                # OGRE2 hardcodes its log under its process home.
                renderer_environment["HOME"] = str(private_home)
                renderer_environment["XDG_CACHE_HOME"] = str(args.run / "renderer-cache")
                renderer_environment["MESA_SHADER_CACHE_DIR"] = str(args.run / "mesa-cache")
                renderer_environment["GST_REGISTRY"] = str(args.run / "linux-gst-registry.bin")
                render_command = ["gz", "sim", "-s", "-v4", "-r", "--seed", "173"]
                if args.visible:
                    gui_config = Path("/usr/share/gz/gz-sim8/gui/gui.config").read_text()
                    gui_config = gui_config.replace(
                        '<dialog name="quick_start" show_again="true"/>',
                        '<dialog name="quick_start" show_again="false"/>',
                    )
                    gui_config = gui_config.replace(
                        "<camera_pose>-6 0 6 0 0.5 0</camera_pose>",
                        "<camera_pose>3 -3 14 0 0.441 2.35619449</camera_pose>",
                    )
                    gui_path = args.run / "gui.config"
                    gui_path.write_text(gui_config)
                render_command.append(str(world))
                with (args.run / "gazebo-console.log").open("w") as render_log:
                    renderer = subprocess.Popen(
                        render_command,
                        cwd=args.run,
                        env=renderer_environment,
                        stdout=render_log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                result["renderer_pid"] = renderer.pid
                result["rtp_host"] = host
            process = subprocess.Popen(
                command, cwd=state, env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
            result["pid"] = process.pid
            end = time.monotonic() + (90 if args.rendered else 20)
            while process.poll() is None and time.monotonic() < end and not stopping:
                # Inspect /proc rather than consuming the first MAVLink connection for readiness.
                listeners = subprocess.check_output(["ss", "-ltnp", "sport = :15760"], text=True)
                if f"pid={process.pid}," in listeners:
                    if args.rendered:
                        while (
                            not list((args.run / "frames").glob("*.json")) and time.monotonic() < end and not stopping
                        ):
                            check_renderer(renderer, args.run)
                            time.sleep(0.1)
                        if not list((args.run / "frames").glob("*.json")):
                            raise TimeoutError("Rendered camera capture not ready")
                        if args.visible:
                            gui_environment = dict(renderer_environment)
                            gui_home = args.run / "gui-home"
                            gui_home.mkdir()
                            gui_environment["HOME"] = str(gui_home)
                            # Initialize the GUI only after the camera has a working OpenGL context.
                            with (args.run / "gui-console.log").open("w") as gui_log:
                                gui = subprocess.Popen(
                                    ["gz", "sim", "-g", "-v4", "--gui-config", str(gui_path)],
                                    cwd=args.run,
                                    env=gui_environment,
                                    stdout=gui_log,
                                    stderr=subprocess.STDOUT,
                                    start_new_session=True,
                                )
                            result["gui_pid"] = gui.pid
                    print("READY", flush=True)
                    break
                time.sleep(0.1)
            else:
                raise TimeoutError("Owned SITL listener not ready")
            end = time.monotonic() + (
                args.endurance_seconds + 180
                if args.endurance_seconds
                else 660
                if args.cpu_only
                else 300
                if args.visible
                else 120
            )
            last_sample = 0
            while process.poll() is None and time.monotonic() < end and not stopping:
                if renderer is not None:
                    check_renderer(renderer, args.run)
                if gui is not None and gui.poll() is not None:
                    raise RuntimeError("Owned Gazebo GUI exited during the case")
                if last_letter is not None and last_letter.poll() is not None:
                    raise RuntimeError("Owned last_letter bridge exited during the case")
                if time.monotonic() - last_sample >= 1:
                    groups = {process.pid}
                    if renderer is not None:
                        groups.add(renderer.pid)
                    if gui is not None:
                        groups.add(gui.pid)
                    if last_letter is not None:
                        groups.add(last_letter.pid)
                    record_resources(args.run / "linux-resources.jsonl", groups)
                    last_sample = time.monotonic()
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready and sys.stdin.readline().strip() in ("", "STOP"):
                    stopping = True
            result["passed"] = stopping and process.poll() is None
    finally:
        result["cleanup_errors"] = []
        for owner, key in (
            (process, "owned_process_exited"),
            (gui, "gui_exited"),
            (renderer, "renderer_exited"),
            (last_letter, "last_letter_exited"),
        ):
            if owner is None:
                continue
            try:
                stop_owned(owner, args.run.name)
                result[key] = True
            except (OSError, RuntimeError, TimeoutError) as error:
                result[key] = False
                result["passed"] = False
                result["cleanup_errors"].append(f"{key}: {error}")
        if process is not None:
            result["exit_code"] = process.returncode
        if relay is not None:
            try:
                relay.close()
            except TimeoutError as error:
                result["passed"] = False
                result["cleanup_errors"].append(str(error))
            result["rtp_relay"] = relay.stats
        if (args.run / "capture-error.txt").exists():
            result["passed"] = False
            result["capture_error"] = "Renderer capture/pose recording failed; inspect capture-error.txt"
        (args.run / "linux.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
