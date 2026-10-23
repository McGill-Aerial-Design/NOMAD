# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: E402,F821
"""Native Mission Planner script: the real NOMAD plugin and owned external SITL."""

import json
import os

import clr

clr.AddReference("System.Drawing")
clr.AddReference("System.Windows.Forms")
from MissionPlanner import MainV2 as MainType
from MissionPlanner.Comms import UdpSerialConnect
from MissionPlanner.Plugin import PluginLoader
from MissionPlanner.Utilities import Locationwp
from System import Action, Array, Byte, EventHandler, Object, Single, UInt16
from System.IO import File, Path
from System.Net import IPAddress, IPEndPoint
from System.Net.Sockets import UdpClient
from System.Reflection import BindingFlags
from System.Threading import Monitor, Thread
from System.Windows.Forms import ToolStripMenuItem

from MAVLink import (
    MAV_CMD,
    MAV_FRAME,
    MAVLinkMessage,
    mavlink_extended_sys_state_t,
    mavlink_global_position_int_t,
    mavlink_heartbeat_t,
    mavlink_mission_current_t,
    mavlink_servo_output_raw_t,
)

HOST = MainV2
RUN = os.environ["NOMAD_CONFERENCE_RUN"]
CHECK = os.environ.get("NOMAD_CONFERENCE_CHECK") == "1"
STATIC = BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic
INSTANCE = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic
state = {"passed": False, "stage": "startup"}
observed = {"altitude": 0, "boot_ms": 0, "mode": -1, "armed": False, "landed": -1}
gate = Object()
connection = MainType.comPort
stream = None
plugin = None
assembly = None


def ui(function):
    HOST.Invoke(Action(function))


def require(condition, diagnostic):
    if not condition:
        raise Exception(diagnostic)


def stage(name):
    state["stage"] = name
    File.WriteAllText(Path.Combine(RUN, "result.json"), json.dumps(state, indent=2))
    print("NOMAD conference: " + name)


def wait_for(predicate, seconds, diagnostic):
    from System import DateTime

    deadline = DateTime.UtcNow.AddSeconds(seconds)
    while DateTime.UtcNow < deadline:
        if predicate():
            return
        Thread.Sleep(50)
    raise Exception(diagnostic)


def current():
    require(
        MainType.comPort == connection and connection.BaseStream == stream,
        "The conference connection changed; no new command sent",
    )
    require(
        connection.sysidcurrent == 201 and connection.compidcurrent == 1 and stream.IsOpen,
        "The selected endpoint is not the owned SITL",
    )


def native_idle():
    from System.Diagnostics import Stopwatch

    timer = Stopwatch.StartNew()
    idle_since = None
    while timer.ElapsedMilliseconds < 10000:
        current()
        if connection.giveComport:
            idle_since = None
        elif idle_since is None:
            idle_since = timer.ElapsedMilliseconds
        elif timer.ElapsedMilliseconds - idle_since >= 300:
            return
        Thread.Sleep(25)
    raise Exception("Native Mission Planner request remained busy; no new command sent")


def packet_received(sender, packet):
    if packet.sysid != 201 or packet.compid != 1:
        return
    Monitor.Enter(gate)
    try:
        message = int(packet.msgid)
        if message == 0:
            item = packet.ToStructure[mavlink_heartbeat_t]()
            observed.update(mode=int(item.custom_mode), armed=bool(int(item.base_mode) & 128))
        elif message == 33:
            item = packet.ToStructure[mavlink_global_position_int_t]()
            observed.update(
                altitude=item.relative_alt / 1000.0,
                lat=item.lat / 1e7,
                lon=item.lon / 1e7,
                boot_ms=int(item.time_boot_ms),
            )
        elif message == 36:
            item = packet.ToStructure[mavlink_servo_output_raw_t]()
            if item.port == 0:
                observed.update(
                    pitch_pwm=int(item.servo7_raw), roll_pwm=int(item.servo8_raw), marker_pwm=int(item.servo9_raw)
                )
        elif message == 245:
            item = packet.ToStructure[mavlink_extended_sys_state_t]()
            observed["landed"] = int(item.landed_state)
        elif message == 42:
            observed["mission_item"] = int(packet.ToStructure[mavlink_mission_current_t]().seq)
    finally:
        Monitor.Exit(gate)


def command(identifier, values):
    native_idle()
    args = [Byte(201), Byte(1), identifier] + [Single(value) for value in values] + [True]
    accepted = connection.doCommandAsync(*args).GetAwaiter().GetResult()
    require(accepted, "Native command rejected: " + str(identifier))


def call(name, method, values):
    subject = assembly.GetType("NOMAD.MissionPlanner." + name)
    return subject.GetMethod(method, STATIC).Invoke(None, Array[Object](values))


def set_links(radio, network):
    File.WriteAllText(Path.Combine(RUN, "links.json"), json.dumps({"radio": radio, "network": network}))


def menu():
    item = ToolStripMenuItem("Conference")
    item.ForeColor = next(entry.ForeColor for entry in HOST.MainMenuStrip.Items if entry.Text == "NOMAD")
    actions = [
        ("Cut telemetry radio", lambda: set_links(False, True)),
        ("Cut telemetry network", lambda: set_links(True, False)),
        ("Cut both telemetry routes", lambda: set_links(False, False)),
        ("Restore routes", lambda: set_links(True, True)),
    ]
    for text, action in actions:
        entry = ToolStripMenuItem(text)
        entry.Click += EventHandler(lambda sender, args, callback=action: callback())
        item.DropDownItems.Add(entry)
    HOST.MainMenuStrip.Items.Add(item)


def upload_survey():
    native_idle()
    # ArduPilot executes this mission; the script only enters the operator's plan.
    definitions = [
        (16, 47.641468, -122.140165, 122, 0),
        (22, 47.641468, -122.140165, 25, 0),
        (16, 47.641648, -122.140165, 25, 2),
        (19, 47.641648, -122.139925, 25, 90),
        (20, 0, 0, 0, 0),
    ]
    connection.setWPTotal(UInt16(len(definitions)))
    for index, definition in enumerate(definitions):
        # IronPython property assignments mutate a struct copy; use its native initializer.
        point = Locationwp().Set(definition[1], definition[2], definition[3], UInt16(definition[0]))
        clr.GetClrType(Locationwp).GetField("p1").SetValue(point, Single(definition[4]))
        require(int(point.id) == definition[0], "Native mission item initialization failed")
        frame = MAV_FRAME.GLOBAL if index == 0 else MAV_FRAME.GLOBAL_RELATIVE_ALT
        native_idle()
        result = connection.setWP(Byte(201), Byte(1), point, UInt16(index), frame, Byte(0), Byte(1), True)
        require(int(result) == 0, "Native mission item rejected: " + str(index) + ": " + str(result))
    connection.setWPACK()
    require(connection.setWPCurrent(Byte(201), Byte(1), UInt16(1)), "Mission start item not confirmed")
    state["mission_items"] = len(definitions)


def open_video():
    plugin.GetType().GetMethod("ShowMainScreen", INSTANCE).Invoke(plugin, None)
    screen = MainType.View.current.Control
    screen.GetType().GetMethod("ShowView", INSTANCE).Invoke(screen, Array[Object](["Video"]))
    video = screen.GetType().GetField("_videoView", INSTANCE).GetValue(screen)
    player = video.GetType().GetField("_videoHost", INSTANCE).GetValue(video)
    embedded = player.GetType().GetProperty("Player", INSTANCE).GetValue(player, None)
    embedded.GetType().GetMethod("StartStream", INSTANCE).Invoke(embedded, None)
    return embedded


def camera_frame(embedded):
    return embedded.GetType().GetField("_displayFrame", INSTANCE).GetValue(embedded)


def select_camera(embedded, name):
    decoded = {"last": camera_frame(embedded), "frames": 0}

    def select():
        screen = MainType.View.current.Control
        video = screen.GetType().GetField("_videoView", INSTANCE).GetValue(screen)
        host = video.GetType().GetField("_videoHost", INSTANCE).GetValue(video)
        selector = host.GetType().GetField("_selector", INSTANCE).GetValue(host)
        selector.SelectedItem = name
        require(selector.SelectedItem == name, "Native video input selection was rejected")

    ui(select)

    def receiving():
        frame = camera_frame(embedded)
        if frame is not None and not Object.ReferenceEquals(decoded["last"], frame):
            decoded.update(last=frame, frames=decoded["frames"] + 1)
        # Join a running H.264 source through its next one-second keyframe interval.
        return decoded["frames"] >= 15

    wait_for(
        receiving,
        15,
        "NOMAD did not decode the selected live input: " + name,
    )


def capture(embedded, name):
    clr.AddReferenceToFileAndPath(Path.Combine(Path.GetDirectoryName(RUN), "prepared", "NOMADHost.exe"))
    from NOMAD.Conference import Screenshot

    def save():
        frame = camera_frame(embedded)
        require(frame is not None, "NOMAD has no decoded renderer frame")
        Screenshot.Save(HOST, frame, Path.Combine(RUN, name))

    ui(save)


handler = EventHandler[MAVLinkMessage](packet_received)
try:
    wait_for(
        lambda: any(p.GetType().FullName == "NOMAD.MissionPlanner.NOMADPlugin" for p in PluginLoader.Plugins),
        20,
        "NOMAD did not load",
    )
    plugin = next(p for p in PluginLoader.Plugins if p.GetType().FullName == "NOMAD.MissionPlanner.NOMADPlugin")
    assembly = plugin.GetType().Assembly

    def size_host():
        HOST.Width = 1440
        HOST.Height = 900

    ui(size_host)
    require(
        not connection.BaseStream.IsOpen and not any(p.BaseStream.IsOpen for p in MainType.Comports),
        "An unexpected connection was already open",
    )
    stream = UdpSerialConnect()
    stream.client.Dispose()
    stream.client = UdpClient(IPEndPoint(IPAddress.Loopback, 0))
    stream.hostEndPoint = IPEndPoint(IPAddress.Loopback, 14600)
    stream.Port = "14600"
    stream.IsOpen = True
    connection.BaseStream = stream
    connection.OnPacketReceived += handler
    stage("connect_native_router")
    ui(lambda: connection.Open(False, True, False))
    current()
    for message in (33, 36, 245):
        command(MAV_CMD.SET_MESSAGE_INTERVAL, [message, 100000, 0, 0, 0, 0, 0])
    upload_survey()
    ui(menu)
    player_box = []
    ui(lambda: player_box.append(open_video()))
    player = player_box[0]
    wait_for(lambda: camera_frame(player) is not None, 20, "NOMAD did not decode the actual RTSP feed")
    state["native_video_observed"] = True
    stage("ready")
    if CHECK:
        wait_for(lambda: observed["boot_ms"] >= 45000 and observed.get("lat", 0) != 0, 60, "GPS/EKF settling deadline")
        native_idle()
        connection.setMode(Byte(201), Byte(1), "GUIDED")
        wait_for(lambda: observed["mode"] == 4, 5, "GUIDED mode not observed")
        native_idle()
        require(connection.doARM(Byte(201), Byte(1), True, False), "Normal arming rejected")
        wait_for(lambda: observed["armed"] and connection.MAV.cs.armed, 5, "Native armed state not observed")
        stage("native_takeoff")
        command(MAV_CMD.TAKEOFF, [0, 0, 0, 0, 0, 0, 25])
        wait_for(lambda: observed["altitude"] >= 24, 45, "Actual aircraft did not reach the requested takeoff height")
        state["takeoff_altitude_m"] = observed["altitude"]
        wait_for(lambda: observed["landed"] == 2, 5, "Airborne state not observed")
        native_idle()
        require(connection.setWPCurrent(Byte(201), Byte(1), UInt16(2)), "Survey start item not confirmed")
        native_idle()
        connection.setMode(Byte(201), Byte(1), "AUTO")
        wait_for(lambda: observed["mode"] == 3, 5, "AUTO mode not observed")
        wait_for(
            lambda: observed.get("mission_item", 0) == 3 and observed.get("lat", 0) > 47.64158,
            40,
            "ArduPilot did not execute the survey leg",
        )
        state["auto_mission_item"] = observed["mission_item"]
        state["survey_position"] = [observed["lat"], observed["lon"], observed["altitude"]]
        select_camera(player, "Chase camera")
        capture(player, "nomad-flight")
        select_camera(player, "Inspection gimbal")
        state["video_inputs_observed"] = ["Chase camera", "Inspection gimbal"]
        old_frame = camera_frame(player)
        native_idle()
        result = call("GimbalController", "RequestPitchRollTargetAsync", [Single(-35), Single(12)])
        result.GetAwaiter().GetResult()
        wait_for(
            lambda: abs(observed.get("pitch_pwm", 0) - 1306) < 20 and abs(observed.get("roll_pwm", 0) - 1698) < 20,
            8,
            "NOMAD gimbal command did not change real SITL outputs",
        )
        state["gimbal_pwm"] = [observed["pitch_pwm"], observed["roll_pwm"]]

        def camera_followed():
            try:
                status = json.loads(File.ReadAllText(Path.Combine(RUN, "camera-status.json")))
                return (
                    status["passed"]
                    and status["pose_updates"] >= 2
                    and status["servo_age_seconds"] < 3
                    and abs(status["pitch_degrees"] + 35) < 4
                    and abs(status["roll_degrees"] - 12) < 4
                    and not Object.ReferenceEquals(old_frame, camera_frame(player))
                )
            except Exception:
                return False

        wait_for(
            camera_followed, 10, "Real camera pose or fresh NOMAD decoder frame did not follow observed servo output"
        )
        state["camera_followed_observed_output"] = True
        manager = plugin.GetType().GetField("_connectionManager", INSTANCE).GetValue(plugin)
        wait_for(lambda: manager.ActiveLink == "radio", 10, "Preferred route missing")
        set_links(False, True)
        wait_for(lambda: manager.ActiveLink == "network", 15, "Actual router did not fail over")
        state["failover_observed"] = True
        # ArduPilot message intervals belong to each MAVLink serial port.
        for message in (33, 36, 245):
            command(MAV_CMD.SET_MESSAGE_INTERVAL, [message, 100000, 0, 0, 0, 0, 0])
        owner = (
            assembly.GetType("NOMAD.MissionPlanner.PayloadController")
            .GetProperty("Current", STATIC)
            .GetValue(None, None)
        )
        method = owner.GetType().GetMethod("ActAsync", INSTANCE)
        native_idle()
        safe = method.Invoke(owner, Array[Object](["marker", "safe", None, "ui", None, None])).GetAwaiter().GetResult()
        require(str(safe) == "Safe accepted.", "Marker Safe rejected: " + str(safe))
        wait_for(lambda: observed.get("marker_pwm") == 1000, 6, "Initial Safe output not observed")
        first = (
            method.Invoke(owner, Array[Object](["marker", "activate", None, "ui", None, None])).GetAwaiter().GetResult()
        )
        state["first_confirmation"] = str(first)
        require(
            str(first) == "Confirmation incomplete or neutral needed." and observed.get("marker_pwm") == 1000,
            "First marker confirmation changed output: " + str(first),
        )
        method.Invoke(owner, Array[Object](["marker", "activate", None, "ui", None, None])).GetAwaiter().GetResult()
        wait_for(lambda: observed.get("marker_pwm") == 2000, 6, "Marker output not observed after failover")
        state["marker_pwm"] = observed["marker_pwm"]
        capture(player, "nomad-inspection")
        method.Invoke(owner, Array[Object](["marker", "safe", None, "ui", None, None])).GetAwaiter().GetResult()
        wait_for(lambda: observed.get("marker_pwm") == 1000, 6, "Safe output not observed")
        set_links(True, True)
        stage("native_return_and_land")
        native_idle()
        connection.setMode(Byte(201), Byte(1), "RTL")
        wait_for(lambda: observed["mode"] == 6, 6, "RTL mode not observed")
        wait_for(
            lambda: (
                observed["landed"] == 1
                and abs(observed["lat"] - 47.641468) < 0.00005
                and abs(observed["lon"] + 122.140165) < 0.00007
                and abs(observed["altitude"]) < 2
            ),
            90,
            "Return to launch and landed state not observed",
        )
        state["landing_position"] = [observed["lat"], observed["lon"], observed["altitude"]]
        native_idle()
        require(connection.doARM(Byte(201), Byte(1), False, False), "Normal disarming rejected after landing")
        wait_for(lambda: not observed["armed"], 6, "Disarmed state not observed")
        state.update(passed=True, landed_state=observed["landed"], disarmed=True)
        stage("passed")
except Exception as error:
    state.update(passed=False, error=str(error), last_observed=dict(observed))
    stage("failed")
finally:
    if CHECK or state["stage"] == "failed":
        HOST.BeginInvoke(Action(HOST.Close))
