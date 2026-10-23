# SPDX-License-Identifier: Apache-2.0
"""Actual ProjectAirSim cameras -> local RTSP; observed SITL PWM -> camera pose.

The launcher owns the renderer, SITL and MediaMTX. This process owns only its
two GStreamer children. Source 253 requests servo telemetry once; it sends no
flight or actuator commands.
"""

import argparse
import json
import logging
import math
import queue
import signal
import socket
import subprocess
import threading
import time
from pathlib import Path

import commentjson
from projectairsim import Drone, ProjectAirSimClient, World
from projectairsim.types import Pose
from projectairsim.utils import rpy_to_quaternion, unpack_image
from pymavlink import mavutil


def pwm_angle(pwm, limit):
    if not 1000 <= pwm <= 2000:
        raise ValueError("Observed servo PWM outside the owned calibration")
    return (pwm - 1500) * limit / 500


class CameraStream:
    def __init__(self, name, gst, output):
        self.name, self.frames, self.written, self.dropped = name, 0, 0, 0
        self.error, self.last_frame = "", None
        self.pending = queue.Queue(maxsize=1)
        self.stopped = threading.Event()
        self.peer = None
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.listener.settimeout(30)
        self.log = (output / (name + "-encoder.log")).open("wb")
        # The Python-owned listener reserves the port until this child connects.
        command = [
            str(gst),
            "-q",
            "tcpclientsrc",
            "host=127.0.0.1",
            "port=" + str(self.listener.getsockname()[1]),
            "!",
            "rawvideoparse",
            "format=bgr",
            "width=640",
            "height=360",
            "framerate=10/1",
            "!",
            "videoconvert",
            "!",
            "video/x-raw,format=I420",
            "!",
            "x264enc",
            "tune=zerolatency",
            "speed-preset=ultrafast",
            "key-int-max=10",
            "bitrate=1500",
            "!",
            "h264parse",
            "!",
            "rtspclientsink",
            "protocols=tcp",
            "location=rtsp://127.0.0.1:8554/" + name,
        ]
        try:
            self.process = subprocess.Popen(
                command, stdout=self.log, stderr=self.log, creationflags=subprocess.CREATE_NO_WINDOW
            )
        except OSError:
            self.listener.close()
            self.log.close()
            raise
        self.worker = threading.Thread(target=self.send, daemon=True)
        self.worker.start()

    def received(self, topic, message):
        if self.stopped.is_set():
            return
        try:
            image = unpack_image(message)
            if image.shape != (360, 640, 3) or image.dtype.name != "uint8":
                raise ValueError("Expected a real 640x360 BGR camera frame")
            frame = image.tobytes()
            self.frames += 1
            self.last_frame = time.monotonic()
            try:
                self.pending.get_nowait()
                self.dropped += 1
            except queue.Empty:
                pass
            self.pending.put_nowait(frame)
        except Exception as error:
            self.error = type(error).__name__ + ": " + str(error)

    def send(self):
        try:
            self.peer, _ = self.listener.accept()
            self.peer.settimeout(1)
            while not self.stopped.is_set():
                try:
                    frame = self.pending.get(timeout=0.2)
                except queue.Empty:
                    continue
                self.peer.sendall(frame)
                self.written += 1
        except (OSError, TimeoutError) as error:
            if not self.stopped.is_set():
                self.error = "Encoder transport: " + str(error)

    def status(self):
        return {
            "frames_received": self.frames,
            "frames_submitted": self.written,
            "frames_replaced": self.dropped,
            "encoder_running": self.process.poll() is None,
            "encoder_pid": self.process.pid,
            "frame_age_seconds": None if self.last_frame is None else time.monotonic() - self.last_frame,
            "error": self.error,
        }

    def close(self):
        self.stopped.set()
        if self.peer:
            self.peer.close()
        self.listener.close()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        self.worker.join(timeout=2)
        self.log.close()


def check_calibration():
    assert [pwm_angle(x, 30) for x in (1000, 1500, 2000)] == [-30, 0, 30]
    assert [pwm_angle(x, 90) for x in (1000, 1500, 2000)] == [-90, 0, 90]
    for value in (0, 999, 2001, 65535):
        try:
            pwm_angle(value, 90)
        except ValueError:
            continue
        raise AssertionError("Invalid PWM accepted")


def run(args):
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    config = args.config_dir.resolve()
    robot = commentjson.loads((config / "robot_ardu_quadrotor.jsonc").read_text())
    scene = commentjson.loads((config / "scene_ardu_quadrotor.jsonc").read_text())
    if scene["clock"]["step-ns"] != 1000000:
        raise ValueError("The qualified scene requires the 1 ms physics step")
    controller = robot["controller"]["ardupilot-settings"]
    if any(controller[x] != "127.0.0.1" for x in ("local-host-ip", "ardupilot-ip")):
        raise ValueError("Only the owned loopback SITL configuration is allowed")
    inspection = next(x for x in robot["sensors"] if x["id"] == "DownCamera")
    xyz = [float(x) for x in inspection["origin"]["xyz"].split()]
    if len(xyz) != 3 or not all(math.isfinite(x) for x in xyz):
        raise ValueError("Invalid inspection-camera translation")
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    logging.getLogger("projectairsim").addHandler(logging.NullHandler())
    client, mav, streams = None, None, {}
    status = {
        "passed": False,
        "mode": "real-rendered-simulation",
        "stage": "starting",
        "mavlink_packets_sent": 0,
        "servo_packets_observed": 0,
        "pose_updates": 0,
        "message_counts": {},
    }
    latest_servo, previous_pwm = None, None
    started = time.monotonic()
    try:
        client = ProjectAirSimClient(address="127.0.0.1")
        client.connect()
        client.socket_services.recv_timeout = 15000
        world = World(client, "scene_ardu_quadrotor.jsonc", 2, sim_config_path=str(config))
        drone = Drone(client, world, "Drone1")
        for name, sensor in (("inspection", "DownCamera"), ("chase", "Chase")):
            streams[name] = CameraStream(name, args.gst, output)
            client.subscribe(drone.sensors[sensor]["scene_camera"], streams[name].received)
        mav = mavutil.mavlink_connection("tcp:127.0.0.1:" + str(args.sitl_port), source_system=253)
        heartbeat = mav.wait_heartbeat(timeout=15)
        if heartbeat is None or heartbeat.get_srcSystem() != 201:
            raise RuntimeError("Owned SITL system 201 heartbeat missing")
        mav.mav.command_long_send(
            201,
            heartbeat.get_srcComponent(),
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
            100000,
            0,
            0,
            0,
            0,
            0,
        )
        status["mavlink_packets_sent"] = 1
        status["stage"] = "streaming"
        while not stop.wait(0.05):
            if (output / "camera.stop").exists():
                break
            for _ in range(100):
                packet = mav.recv_match(blocking=False)
                if packet is None:
                    break
                if packet.get_type() == "BAD_DATA":
                    continue
                if packet.get_srcSystem() != 201:
                    # ArduPilot routes GCS messages between its MAVLink ports as well.
                    continue
                kind = packet.get_type()
                status["message_counts"][kind] = status["message_counts"].get(kind, 0) + 1
                if kind == "COMMAND_ACK" and packet.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                    status["stream_request_ack"] = packet.result
                    if packet.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        raise RuntimeError("SITL rejected the one servo telemetry subscription")
                if packet.get_type() != "SERVO_OUTPUT_RAW" or packet.port != 0:
                    continue
                pwm = (packet.servo7_raw, packet.servo8_raw)
                # SERVO7_FUNCTION=7 is mount tilt; SERVO8_FUNCTION=8 is mount roll.
                pitch, roll = pwm_angle(pwm[0], 90), pwm_angle(pwm[1], 30)
                latest_servo = time.monotonic()
                status.update(
                    servo_packets_observed=status["servo_packets_observed"] + 1,
                    servo7_pwm=pwm[0],
                    servo8_pwm=pwm[1],
                    roll_degrees=roll,
                    pitch_degrees=pitch,
                )
                if pwm != previous_pwm:
                    quat = rpy_to_quaternion(math.radians(roll), math.radians(pitch), 0)
                    pose = Pose(
                        {
                            "translation": dict(zip(("x", "y", "z"), xyz)),
                            "rotation": dict(zip(("w", "x", "y", "z"), quat)),
                        }
                    )
                    if not drone.set_camera_pose("DownCamera", pose):
                        raise RuntimeError("Renderer rejected observed servo-derived camera pose")
                    previous_pwm = pwm
                    status["pose_updates"] += 1
            status["servo_age_seconds"] = None if latest_servo is None else time.monotonic() - latest_servo
            status["streams"] = {name: stream.status() for name, stream in streams.items()}
            if any(x["error"] or not x["encoder_running"] for x in status["streams"].values()):
                raise RuntimeError("Camera/encoder failed; see the owned status and encoder logs")
            ready = (
                status.get("stream_request_ack") == 0
                and latest_servo is not None
                and status["servo_age_seconds"] < 3
                and all(x["frames_submitted"] >= 10 and x["frame_age_seconds"] < 3 for x in status["streams"].values())
            )
            status["passed"] = ready
            temporary = output / "camera-status.tmp"
            temporary.write_text(json.dumps(status, indent=2))
            try:
                temporary.replace(output / "camera-status.json")
            except PermissionError:
                pass  # Windows readers can briefly hold the previous snapshot open.
            if not ready and time.monotonic() - started > 30:
                raise RuntimeError("Real camera/servo startup exceeded 30 seconds")
            if args.seconds and time.monotonic() - started >= args.seconds:
                if not ready:
                    raise RuntimeError("Timed run ended before real camera/servo readiness")
                break
    except Exception as error:
        status.update(passed=False, stage="failed", error=type(error).__name__ + ": " + str(error))
        raise
    finally:
        if mav:
            mav.close()
        for stream in streams.values():
            stream.close()
        if client:
            client.disconnect()
        status["stopped"] = True
        (output / "camera-status.json").write_text(json.dumps(status, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--sitl-port", type=int, default=6313)
    parser.add_argument("--gst", type=Path, default=Path("C:/gstreamer/1.0/msvc_x86_64/bin/gst-launch-1.0.exe"))
    parser.add_argument("--seconds", type=float, default=0, help="Optional bounded integration run")
    parser.add_argument("--check", action="store_true", help="Check calibration without processes or network")
    options = parser.parse_args()
    check_calibration()
    if options.check:
        print("PASS PWM calibration and invalid boundaries")
    else:
        if (
            options.config_dir is None
            or options.output_dir is None
            or not 1024 <= options.sitl_port <= 65535
            or not math.isfinite(options.seconds)
            or options.seconds < 0
        ):
            parser.error("Owned --config-dir, --output-dir and valid unprivileged --sitl-port are required")
        run(options)
