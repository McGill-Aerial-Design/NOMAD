# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: E402,F821
"""Native Mission Planner script: the real NOMAD plugin and owned external SITL."""

import json
import os
import sys
import traceback

import clr

clr.AddReference("System.Drawing")
clr.AddReference("System.Windows.Forms")
from MissionPlanner import MainV2 as MainType
from MissionPlanner.Comms import UdpSerialConnect
from MissionPlanner.Plugin import PluginLoader
from MissionPlanner.Utilities import Locationwp
from System import (
    Action,
    Activator,
    Array,
    Byte,
    DateTime,
    DateTimeKind,
    EventHandler,
    Int32,
    Int64,
    Object,
    Single,
    UInt16,
    UInt32,
)
from System.Diagnostics import Stopwatch
from System.IO import File, Path
from System.Net import IPAddress, IPEndPoint
from System.Net.Sockets import UdpClient
from System.Reflection import BindingFlags
from System.Threading import Monitor, Thread
from System.Windows.Forms import ToolStripMenuItem

# The host imports above establish this CLR namespace before it is used.
# isort: split
from MAVLink import (
    MAV_CMD,
    MAV_FRAME,
    MAVLinkMessage,
    mavlink_command_ack_t,
    mavlink_extended_sys_state_t,
    mavlink_global_position_int_t,
    mavlink_heartbeat_t,
    mavlink_mission_current_t,
    mavlink_rc_channels_t,
    mavlink_servo_output_raw_t,
    mavlink_statustext_t,
)

HOST = MainV2
RUN = os.environ["NOMAD_CONFERENCE_RUN"]
CHECK = os.environ.get("NOMAD_CONFERENCE_CHECK") == "1"
sys.path.insert(0, RUN)
from descent import controller_time_advanced, measure_descent

STATIC = BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic
INSTANCE = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic
state = {"passed": False, "stage": "startup"}
observed = {"altitude": 0, "boot_ms": 0, "mode": -1, "armed": False, "landed": -1}
clock = Stopwatch.StartNew()
trace_mode = None
descent_rows = []
servo_rows = []
ack_rows = []
ack_serial = 0
callback_error = ""
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
    deadline = clock.ElapsedMilliseconds + seconds * 1000
    while clock.ElapsedMilliseconds < deadline:
        require(not callback_error, callback_error)
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
    global callback_error, ack_serial
    if packet.sysid != 201 or packet.compid != 1:
        return
    age = (DateTime.UtcNow - packet.rxtime).TotalSeconds
    if packet.rxtime.Kind != DateTimeKind.Utc or age < 0 or age > 3.5:
        return
    received = clock.ElapsedMilliseconds - age * 1000
    Monitor.Enter(gate)
    try:
        message = int(packet.msgid)
        if message == 0:
            item = packet.ToStructure[mavlink_heartbeat_t]()
            observed.update(mode=int(item.custom_mode), armed=bool(int(item.base_mode) & 128), heartbeat_rx=received)
        elif message == 33:
            item = packet.ToStructure[mavlink_global_position_int_t]()
            boot_ms = int(item.time_boot_ms)
            if not controller_time_advanced(observed["boot_ms"], boot_ms):
                observed["stale_position_samples"] = observed.get("stale_position_samples", 0) + 1
                return
            observed.update(
                altitude=item.relative_alt / 1000.0,
                lat=item.lat / 1e7,
                lon=item.lon / 1e7,
                boot_ms=boot_ms,
                vz=item.vz * 0.01,
                position_rx=received,
            )
            if (
                trace_mode is not None
                and observed["mode"] == trace_mode
                and fresh("heartbeat_rx")
                and (not descent_rows or boot_ms > descent_rows[-1][0])
            ):
                if len(descent_rows) >= 2000:
                    callback_error = "Bounded descent trace overflowed"
                else:
                    descent_rows.append((boot_ms, item.relative_alt * 0.001, item.vz * 0.01))
        elif message == 36:
            item = packet.ToStructure[mavlink_servo_output_raw_t]()
            if item.port == 0:
                observed.update(
                    pitch_pwm=int(item.servo7_raw),
                    roll_pwm=int(item.servo8_raw),
                    marker_pwm=int(item.servo9_raw),
                    yaw_pwm=int(item.servo12_raw),
                    servo_rx=received,
                    servo_serial=observed.get("servo_serial", 0) + 1,
                )
                servo_rows.append((observed["servo_serial"], received, int(item.servo9_raw)))
                if len(servo_rows) > 32:
                    servo_rows.pop(0)
        elif message == 245:
            item = packet.ToStructure[mavlink_extended_sys_state_t]()
            observed.update(landed=int(item.landed_state), landed_rx=received)
        elif message == 42:
            observed["mission_item"] = int(packet.ToStructure[mavlink_mission_current_t]().seq)
        elif message == 65:
            item = packet.ToStructure[mavlink_rc_channels_t]()
            observed.update(
                rc_channels=int(item.chancount),
                rc7_pwm=int(item.chan7_raw),
                rc3_pwm=int(item.chan3_raw),
                rc5_pwm=int(item.chan5_raw),
                rc_rx=received,
            )
        elif message == 77:
            item = packet.ToStructure[mavlink_command_ack_t]()
            ack_serial += 1
            ack_rows.append((ack_serial, int(item.command), int(item.result), received))
            if len(ack_rows) > 32:
                ack_rows.pop(0)
            if int(item.command) in (176, 400):
                prefix = "mode_ack" if int(item.command) == 176 else "arm_ack"
                observed.update({prefix: int(item.result), prefix + "_rx": received})
        elif message == 253:
            item = packet.ToStructure[mavlink_statustext_t]()
            text = "".join(chr(int(value)) for value in item.text).rstrip("\0")
            if "Termination latched: reboot required" in text:
                observed["latch_notice_rx"] = received
    except Exception as error:
        callback_error = "Native packet observation failed: " + str(error)
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


def fresh(*keys):
    now = clock.ElapsedMilliseconds
    return all(key in observed and 0 <= now - observed[key] <= 3500 for key in keys)


def finish(task):
    wait_for(lambda: task.IsCompleted, 12, "Native helper deadline; outcome requires inspection, no retry")
    return task.GetAwaiter().GetResult()


def read_parameter(name):
    native_idle()
    return float(finish(connection.GetParamAsync(Byte(201), Byte(1), name)))


def check_termination(controller):
    subject = controller.GetType()
    # MP periodically owns the port for camera discovery; repeat completed read-only checks only.
    for attempt in range(3):
        native_idle()
        supported = bool(finish(subject.GetMethod("CheckAsync", INSTANCE).Invoke(controller, None)))
        state.setdefault("termination_check_attempts", {})[state["stage"]] = attempt + 1
        state["termination_check_status"] = str(subject.GetProperty("Status", INSTANCE).GetValue(controller, None))
        if supported:
            return
        Thread.Sleep(250)
    raise Exception("Actual NOMAD did not verify TERMINATE: " + state["termination_check_status"])


def rejected_ack(command_id, after):
    Monitor.Enter(gate)
    try:
        matching = [
            row
            for row in ack_rows
            if row[0] > after
            and row[1] == command_id
            and row[2] in (2, 4)
            and 0 <= clock.ElapsedMilliseconds - row[3] <= 3500
        ]
        return matching[-1][2] if matching else None
    finally:
        Monitor.Exit(gate)


def marker_reports(after, started, expected, reports):
    Monitor.Enter(gate)
    try:
        rows = [
            row
            for row in servo_rows
            if row[0] > after and row[1] >= started and 0 <= clock.ElapsedMilliseconds - row[1] <= 3500
        ]
        if reports == 2:
            require(all(row[2] == expected for row in rows), "First confirmation changed reported marker output")
        return fresh("servo_rx") and len([row for row in rows if row[2] == expected]) >= reports
    finally:
        Monitor.Exit(gate)


def marker_action(owner, method, operation, expected, reports=1):
    native_idle()
    Monitor.Enter(gate)
    try:
        started, after = clock.ElapsedMilliseconds, observed.get("servo_serial", 0)
    finally:
        Monitor.Exit(gate)
    outcome = str(finish(method.Invoke(owner, Array[Object](["marker", operation, None, "ui", None, None]))))
    wait_for(
        lambda: marker_reports(after, started, expected, reports),
        6,
        "Fresh post-action marker output was not observed: " + operation,
    )
    return outcome


def verify_commissioning():
    values = {}
    for line in File.ReadAllLines(Path.Combine(RUN, "termination.parm")):
        fields = line.split("#", 1)[0].split()
        if not fields:
            continue
        require(len(fields) == 2 and fields[0] not in values, "Invalid commissioning parameter row")
        name, expected = fields[0], float(fields[1])
        actual = read_parameter(name)
        require(abs(actual - expected) <= 0.0001, "Commissioning readback mismatch: " + name)
        values[name] = actual
    require(len(values) > 0, "Commissioning parameters are missing")
    wait_for(
        lambda: (
            fresh("rc_rx", "heartbeat_rx")
            and not observed["armed"]
            and observed.get("rc_channels", 0) >= 7
            and abs(observed.get("rc7_pwm", 0) - 1000) <= 25
            and abs(observed.get("rc3_pwm", 0) - 1000) <= 25
            and abs(observed.get("rc5_pwm", 0) - 1800) <= 25
        ),
        8,
        "Healthy native simulated receiver defaults were not observed",
    )
    state["commissioning"] = values
    state["receiver"] = {key: observed[key] for key in ("rc_channels", "rc3_pwm", "rc5_pwm", "rc7_pwm")}
    controller = plugin.GetType().GetField("_termination", INSTANCE).GetValue(plugin)
    check_termination(controller)
    state["termination_supported_disarmed"] = True


def takeoff(name):
    stage(name)
    native_idle()
    connection.setMode(Byte(201), Byte(1), "GUIDED")
    wait_for(lambda: fresh("heartbeat_rx") and observed["mode"] == 4, 5, "Fresh GUIDED mode not observed")
    native_idle()
    require(connection.doARM(Byte(201), Byte(1), True, False), "Normal arming rejected")
    # Native camera discovery can briefly own MP's reader after the arming ACK.
    wait_for(
        lambda: fresh("heartbeat_rx") and observed["armed"] and connection.MAV.cs.armed,
        15,
        "Fresh native armed state not observed",
    )
    command(MAV_CMD.TAKEOFF, [0, 0, 0, 0, 0, 0, 25])
    wait_for(
        lambda: fresh("position_rx") and observed["altitude"] >= 24,
        45,
        "Actual aircraft did not reach the requested takeoff height",
    )
    wait_for(lambda: fresh("landed_rx") and observed["landed"] == 2, 5, "Fresh airborne state not observed")


def begin_descent(mode):
    global trace_mode
    Monitor.Enter(gate)
    try:
        descent_rows[:] = []
        trace_mode = mode
    finally:
        Monitor.Exit(gate)


def descent_metrics(minimum, maximum):
    global trace_mode
    Monitor.Enter(gate)
    try:
        mode = trace_mode
        trace_mode = None
        rows = list(descent_rows)
    finally:
        Monitor.Exit(gate)
    require(not callback_error, callback_error)
    File.WriteAllText(Path.Combine(RUN, "descent-" + str(mode) + ".json"), json.dumps(rows))
    return measure_descent(rows, minimum, maximum)


def at_home():
    return (
        fresh("position_rx")
        and abs(observed["lat"] - 47.641468) < 0.00005
        and abs(observed["lon"] + 122.140165) < 0.00007
    )


def baseline_landing():
    stage("ordinary_land_descent")
    begin_descent(9)
    native_idle()
    connection.setMode(Byte(201), Byte(1), "LAND")
    wait_for(lambda: fresh("heartbeat_rx") and observed["mode"] == 9, 6, "Fresh LAND mode not observed")
    state["ordinary_land_mode_observed"] = True
    wait_for(
        lambda: (
            fresh("landed_rx", "position_rx")
            and observed["landed"] == 1
            and at_home()
            and abs(observed["altitude"]) < 2
        ),
        90,
        "Ordinary LAND did not reach the clear launch site",
    )
    state["land_descent"] = descent_metrics(0.2, 1.0)
    native_idle()
    require(connection.doARM(Byte(201), Byte(1), False, False), "Normal disarming rejected after ordinary LAND")
    wait_for(lambda: fresh("heartbeat_rx") and not observed["armed"], 6, "Fresh baseline disarm not observed")
    state["ordinary_land_disarmed"] = True
    state["baseline_end_boot_ms"] = observed["boot_ms"]


def return_above_home():
    stage("native_return_above_home")
    native_idle()
    connection.setMode(Byte(201), Byte(1), "RTL")
    wait_for(lambda: fresh("heartbeat_rx") and observed["mode"] == 6, 6, "Fresh RTL mode not observed")
    wait_for(lambda: at_home() and observed["altitude"] >= 20, 45, "RTL did not reach the clear launch site above 20 m")
    native_idle()
    connection.setMode(Byte(201), Byte(1), "GUIDED")
    wait_for(
        lambda: (
            fresh("heartbeat_rx", "position_rx")
            and observed["mode"] == 4
            and at_home()
            and observed["altitude"] >= 20
            and abs(observed["vz"]) < 0.3
        ),
        10,
        "GUIDED hold above the clear launch site was not observed",
    )


def terminate_and_measure():
    stage("native_terminate_request")
    require(
        fresh("position_rx") and observed["boot_ms"] > state["baseline_end_boot_ms"],
        "Controller time did not advance from the ordinary LAND flight",
    )
    dashboard_players = []
    ui(lambda: dashboard_players.append(open_video("Dashboard")))
    dashboard_player = dashboard_players[0]
    wait_for_video(dashboard_player)
    controller = plugin.GetType().GetField("_termination", INSTANCE).GetValue(plugin)
    subject = controller.GetType()
    check_termination(controller)
    capture_args = Array[Object]([Int64(0), UInt32(0)])
    require(
        bool(subject.GetMethod("TryCaptureRequest", INSTANCE).Invoke(controller, capture_args)),
        "NOMAD could not capture the fresh armed termination target",
    )
    require(int(capture_args[1]) == 29, "Unexpected advertised termination mode")
    begin_descent(29)
    outcome = str(finish(subject.GetMethod("RequestAsync", INSTANCE).Invoke(controller, capture_args)))
    state["termination_command_accepted"] = outcome.startswith("Command accepted")
    require(state["termination_command_accepted"], "Native termination request was not confirmed; no retry")
    wait_for(
        lambda: (
            fresh("heartbeat_rx")
            and observed["mode"] == 29
            and bool(subject.GetProperty("Observed", INSTANCE).GetValue(controller, None))
        ),
        6,
        "Fresh TERMINATE heartbeat and actual NOMAD observation were not established",
    )
    state["termination_mode_observed"] = True
    state["termination_entry_boot_ms"] = observed["boot_ms"]
    ui(
        lambda: (
            MainType.View.current.Control.GetType()
            .GetField("_dashboardView", INSTANCE)
            .GetValue(MainType.View.current.Control)
            .UpdateData()
        )
    )
    capture(dashboard_player, "nomad-terminate")
    native_idle()
    started = clock.ElapsedMilliseconds
    previous_ack = ack_serial
    escaped = finish(
        connection.doCommandAsync(
            Byte(201),
            Byte(1),
            MAV_CMD.DO_SET_MODE,
            Single(1),
            Single(4),
            Single(0),
            Single(0),
            Single(0),
            Single(0),
            Single(0),
            True,
        )
    )
    rejection = rejected_ack(176, previous_ack)
    require(not escaped and rejection is not None, "The ordinary GUIDED mode request was not explicitly rejected")
    wait_for(
        lambda: fresh("heartbeat_rx") and observed["heartbeat_rx"] > started and observed["mode"] == 29,
        5,
        "Fresh TERMINATE mode did not persist after the rejected escape",
    )
    state["ordinary_mode_change_rejected"] = True
    state["ordinary_mode_rejection_ack"] = rejection
    stage("native_terminate_descent")
    wait_for(
        lambda: (
            fresh("landed_rx", "heartbeat_rx", "position_rx")
            and observed["landed"] == 1
            and not observed["armed"]
            and at_home()
            and abs(observed["altitude"]) < 2
        ),
        45,
        "Native TERMINATE contact and disarming were not observed at the launch site",
    )
    state["terminate_descent"] = descent_metrics(2.0, 2.7)
    state["termination_landed_disarmed"] = True


def verify_latch_and_ordinary_rates():
    stage("termination_latch_refusal")
    native_idle()
    started = clock.ElapsedMilliseconds
    previous_ack = ack_serial
    require(not connection.doARM(Byte(201), Byte(1), True, False), "Normal arming escaped the native latch")
    rejections = []

    def latch_refused():
        rejection = rejected_ack(400, previous_ack)
        if rejection is not None and not rejections:
            rejections.append(rejection)
        return (
            bool(rejections)
            and observed.get("latch_notice_rx", -1) >= started
            and fresh("heartbeat_rx")
            and observed["heartbeat_rx"] > started
            and not observed["armed"]
            and observed["mode"] == 29
        )

    wait_for(
        latch_refused, 6, "Native rearming refusal, latch reason and fresh disarmed TERMINATE state were not observed"
    )
    state["latch_refused"] = True
    state["rearm_rejection_ack"] = rejections[0]
    state["reboot_required"] = True
    state["termination_latch_parameter"] = read_parameter("AFS_TERMINATE")
    require(state["termination_latch_parameter"] == 1, "Native termination latch parameter was not observed")
    rates = {name: read_parameter(name) for name in ("LAND_SPD_MS", "LAND_SPD_HIGH_MS")}
    require(all(abs(value - 0.5) <= 0.0001 for value in rates.values()), "Termination changed ordinary LAND rates")
    state["ordinary_land_rates_after_termination"] = rates
    state["controller_boot_count_after"] = read_parameter("STAT_BOOTCNT")
    require(
        not callback_error and state["controller_boot_count_after"] == state["controller_boot_count_before"],
        "Controller boot count changed during the two-flight check",
    )
    state["same_controller_boot"] = True
    state["stale_position_samples"] = observed.get("stale_position_samples", 0)


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


def open_video(view="Video"):
    plugin.GetType().GetMethod("ShowMainScreen", INSTANCE).Invoke(plugin, None)
    screen = MainType.View.current.Control
    screen.GetType().GetMethod("ShowView", INSTANCE).Invoke(screen, Array[Object]([view]))
    field = "_videoView" if view == "Video" else "_dashboardView"
    video = screen.GetType().GetField(field, INSTANCE).GetValue(screen)
    player = video.GetType().GetField("_videoHost", INSTANCE).GetValue(video)
    embedded = player.GetType().GetProperty("Player", INSTANCE).GetValue(player, None)
    embedded.GetType().GetMethod("StartStream", INSTANCE).Invoke(embedded, None)
    return embedded


def camera_frame(embedded):
    return embedded.GetType().GetField("_displayFrame", INSTANCE).GetValue(embedded)


def select_camera(embedded, name):
    def select():
        screen = MainType.View.current.Control
        video = screen.GetType().GetField("_videoView", INSTANCE).GetValue(screen)
        host = video.GetType().GetField("_videoHost", INSTANCE).GetValue(video)
        selector = host.GetType().GetField("_selector", INSTANCE).GetValue(host)
        selector.SelectedItem = name
        require(selector.SelectedItem == name, "Native video input selection was rejected")

    ui(select)
    wait_for_video(embedded)


def wait_for_video(embedded):
    decoded = {"last": camera_frame(embedded), "frames": 0}

    def receiving():
        frame = camera_frame(embedded)
        if frame is not None and not Object.ReferenceEquals(decoded["last"], frame):
            decoded.update(last=frame, frames=decoded["frames"] + 1)
        # Join a running H.264 source through its next one-second keyframe interval.
        return decoded["frames"] >= 15

    wait_for(
        receiving,
        15,
        "NOMAD did not decode fresh frames from the selected live input",
    )


def capture(embedded, name):
    clr.AddReferenceToFileAndPath(Path.Combine(Path.GetDirectoryName(RUN), "prepared", "NOMADHost.exe"))
    from NOMAD.Conference import Screenshot

    def save():
        frame = camera_frame(embedded)
        require(frame is not None, "NOMAD has no decoded renderer frame")
        Screenshot.Save(HOST, frame, Path.Combine(RUN, name))

    ui(save)


def video_property(subject, name):
    return subject.GetType().GetProperty(name, INSTANCE).GetValue(subject, None)


def video_action(player, name, arguments=None):
    return (
        player.GetType()
        .GetMethod(name, INSTANCE)
        .Invoke(player, None if arguments is None else Array[Object](arguments))
    )


def vision_sample(player):
    sample = {}

    def read():
        frame = video_property(player, "DisplayedVideoFrame")
        target = video_property(player, "SelectedRegion")
        session = player.GetType().GetField("_session", INSTANCE).GetValue(player)
        sample.update(
            tracking=bool(video_property(player, "IsTracking")),
            ready=bool(video_property(player, "RegionReady")),
            detecting=bool(video_property(session, "DetectionEnabled")),
            status=str(video_property(player, "TrackingStatus")),
            requests=int(video_property(player, "SubmittedTrackingCommands")),
        )
        if frame is None or not video_property(frame, "Fresh"):
            return
        image = video_property(frame, "Image")
        sample.update(sequence=int(video_property(frame, "Sequence")), fresh=True)
        sample["cars"] = sum(1 for box in video_property(frame, "Detections") if video_property(box, "ClassId") == 2)
        sample["frame_age_seconds"] = float(video_property(frame, "AgeSeconds"))
        sample["boxes"] = []
        for detection in video_property(frame, "Detections"):
            bounds = video_property(detection, "Bounds")
            sample["boxes"].append(
                [
                    int(video_property(detection, "ClassId")),
                    float(video_property(detection, "Confidence")),
                    float(bounds.X),
                    float(bounds.Y),
                    float(bounds.Width),
                    float(bounds.Height),
                ]
            )
        if target is not None:
            box = target
            sample.update(
                x=float((box.X + box.Width / 2.0 - image.Width / 2.0) / image.Width),
                y=float((box.Y + box.Height / 2.0 - image.Height / 2.0) / image.Height),
                region=[float(box.X), float(box.Y), float(box.Width), float(box.Height)],
            )

    ui(read)
    return sample


def select_detected_car(player):
    selected = []

    def select():
        frame = video_property(player, "DisplayedVideoFrame")
        if frame is None or not video_property(frame, "Fresh"):
            return
        cars = [box for box in video_property(frame, "Detections") if video_property(box, "ClassId") == 2]
        if not cars:
            return
        target = max(cars, key=lambda box: video_property(box, "Confidence"))
        box = video_property(target, "Bounds")
        image = video_property(frame, "Image")
        viewport = player.GetType().GetField("_videoBox", INSTANCE).GetValue(player).ClientSize
        scale = min(viewport.Width / float(image.Width), viewport.Height / float(image.Height))
        x = (viewport.Width - image.Width * scale) / 2 + (box.X + box.Width / 2) * scale
        y = (viewport.Height - image.Height * scale) / 2 + (box.Y + box.Height / 2) * scale
        method = player.GetType().GetMethod("SelectTargetAt", INSTANCE)
        point = Activator.CreateInstance(
            method.GetParameters()[0].ParameterType, Array[Object]([Int32(int(round(x))), Int32(int(round(y)))])
        )
        if bool(method.Invoke(player, Array[Object]([point, viewport]))):
            selected.append(True)

    def ready():
        ui(select)
        return bool(selected)

    wait_for(ready, 10, "No fresh pixel-detected car accepted selection")
    wait_for(lambda: vision_sample(player)["ready"], 8, "Visual tracker did not lock the selected car")


def start_visual_follow(player):
    started = []

    def ready():
        def start():
            if bool(video_action(player, "StartTracking")):
                started.append(True)

        ui(start)
        return bool(started)

    wait_for(ready, 5, "Gimbal following did not accept the fresh selected region")


def center_visual_region(player, evidence):
    samples = []
    evidence["samples"] = samples

    def centered():
        sample = vision_sample(player)
        if not sample["tracking"]:
            evidence["retired"] = sample
        require(sample["tracking"], "Tracking retired before centering: " + sample["status"])
        require(not sample["detecting"], "Region following unexpectedly depends on detection")
        if sample.get("fresh") and "x" in sample and (not samples or samples[-1]["sequence"] != sample["sequence"]):
            samples.append(sample)
        return len(samples) >= 8 and all(max(abs(row["x"]), abs(row["y"])) < 0.035 for row in samples[-8:])

    wait_for(centered, 30, "Image region did not remain centered in eight fresh frames")
    return samples[-1]


def reset_inspection_gimbal():
    task = call("GimbalController", "RequestPitchRollTargetAsync", [Single(-32), Single(0)])
    finish(task)
    # IronPython binds the public Task awaiter when its generic result type is internal.
    result = video_property(task, "Result")
    require(video_property(result, "Submitted"), "Manual gimbal request was not submitted")


def check_vision(player):
    stage("vision_tracking")
    state["vision"] = {}
    native_idle()
    connection.setMode(Byte(201), Byte(1), "GUIDED")
    wait_for(lambda: fresh("heartbeat_rx") and observed["mode"] == 4, 8, "GUIDED inspection hold missing")
    reset_inspection_gimbal()
    wait_for_video(player)
    ui(lambda: video_action(player, "EnableDetection", [True]))
    wait_for(lambda: vision_sample(player).get("cars", 0) > 0, 20, "Pinned model found no car in rendered video")
    select_detected_car(player)
    ui(lambda: video_action(player, "EnableDetection", [False]))
    wait_for_video(player)
    before = vision_sample(player)
    capture(player, "nomad-tracking-before")
    require(max(abs(before["x"]), abs(before["y"])) > 0.05, "Selected target was already centered")
    initial_pwm = [observed["pitch_pwm"], observed["yaw_pwm"]]

    state["vision"]["before"] = before
    start_visual_follow(player)
    after = center_visual_region(player, state["vision"])
    require(after["requests"] >= 3, "Too few actual submitted tracking requests")
    require(abs(observed["yaw_pwm"] - initial_pwm[1]) >= 8, "Native pan output did not change")
    require(abs(observed["pitch_pwm"] - initial_pwm[0]) >= 5, "Native pitch output did not change")
    camera = json.loads(File.ReadAllText(Path.Combine(RUN, "camera-status.json")))
    require(camera["passed"] and camera["servo_age_seconds"] < 3, "Rendered camera telemetry is stale")
    require(abs(camera["servo12_pwm"] - observed["yaw_pwm"]) < 15, "Renderer did not follow observed pan output")
    state["vision"].update(
        after=after,
        initial_pitch_yaw_pwm=initial_pwm,
        final_pitch_yaw_pwm=[observed["pitch_pwm"], observed["yaw_pwm"]],
        renderer_pitch_yaw_deg=[camera["pitch_degrees"], camera["yaw_degrees"]],
        detector_disabled_during_follow=True,
    )
    capture(player, "nomad-tracking")
    saved = []

    def save():
        player.GetType().GetProperty("ObservationDirectory", INSTANCE).SetValue(player, RUN, None)
        saved.append(video_action(player, "SaveObservation"))

    ui(save)
    require(bool(saved[0]) and File.Exists(saved[0]), "Observation PNG was not saved")
    metadata = json.loads(File.ReadAllText(Path.ChangeExtension(saved[0], ".json")))
    require(
        metadata["SelectedTarget"]["Found"]
        and metadata["SelectedTarget"]["Tracker"] == "OpenCV CSRT"
        and metadata["AircraftPosition"]["Available"],
        "Observation metadata incomplete",
    )
    require(
        not metadata["TargetPosition"]["Available"] and metadata["LastSubmittedTrackingRequest"]["Available"],
        "Observation confused aircraft/target or command identity",
    )
    state["vision"]["observation"] = str(Path.GetFileName(saved[0]))
    native_idle()
    reset_inspection_gimbal()
    wait_for(lambda: not vision_sample(player)["tracking"], 2, "Manual input did not stop tracking")
    requests = vision_sample(player)["requests"]
    wait_for_video(player)
    require(vision_sample(player)["requests"] == requests, "Automatic tracking resumed after manual input")
    state["vision"]["manual_override"] = True

    # An ordinary scene detail, selected directly from image pixels with detection off.
    texture_selected = []

    def select_texture():
        method = player.GetType().GetMethod("SelectRegion", INSTANCE)
        region = Activator.CreateInstance(
            method.GetParameters()[0].ParameterType, Array[Object]([Single(160), Single(170), Single(70), Single(70)])
        )
        if bool(method.Invoke(player, Array[Object]([region]))):
            texture_selected.append(True)

    def texture_ready():
        ui(select_texture)
        return bool(texture_selected)

    wait_for(texture_ready, 5, "Fresh texture-region selection failed")
    wait_for(lambda: vision_sample(player)["ready"], 8, "Unclassified texture region did not lock")
    state["vision"]["texture"] = {"before": vision_sample(player)}
    start_visual_follow(player)
    state["vision"]["texture"]["after"] = center_visual_region(player, state["vision"]["texture"])
    capture(player, "nomad-texture-tracking")
    select_camera(player, "Chase camera")
    require(not vision_sample(player)["tracking"], "Changing video source did not retire tracking")
    requests = vision_sample(player)["requests"]
    wait_for_video(player)
    require(vision_sample(player)["requests"] == requests, "Old-source tracking continued issuing requests")
    state["vision"]["source_change_retired"] = True
    ui(lambda: video_action(player, "EnableDetection", [False]))
    select_camera(player, "Inspection gimbal")


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
    # Align MP automatic stream requests with the descent and output sampling.
    connection.MAV.cs.rateposition = 10
    connection.MAV.cs.raterc = 10
    for message in (33, 36, 65, 245):
        command(MAV_CMD.SET_MESSAGE_INTERVAL, [message, 100000, 0, 0, 0, 0, 0])
    ui(menu)
    player_box = []
    ui(lambda: player_box.append(open_video()))
    player = player_box[0]
    wait_for(lambda: camera_frame(player) is not None, 20, "NOMAD did not decode the actual RTSP feed")
    state["native_video_observed"] = True
    stage("wait_native_startup")
    wait_for(lambda: observed["boot_ms"] >= 45000 and observed.get("lat", 0) != 0, 60, "GPS/EKF settling deadline")
    upload_survey()
    stage("ready")
    if CHECK:
        stage("termination_commissioning")
        verify_commissioning()
        state["controller_boot_count_before"] = read_parameter("STAT_BOOTCNT")
        require(
            state["controller_boot_count_before"] >= 0
            and state["controller_boot_count_before"] == int(state["controller_boot_count_before"]),
            "Invalid controller boot counter",
        )
        state["descent_evidence"] = "Sampled MAVLink rates with an altitude/time cross-check"
        takeoff("ordinary_land_takeoff")
        baseline_landing()
        takeoff("inspection_takeoff")
        state["takeoff_altitude_m"] = observed["altitude"]
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
        gimbal_started, gimbal_serial = clock.ElapsedMilliseconds, observed.get("servo_serial", 0)
        result = call("GimbalController", "RequestPitchRollTargetAsync", [Single(-35), Single(12)])
        result.GetAwaiter().GetResult()
        wait_for(
            lambda: (
                fresh("servo_rx")
                and observed.get("servo_serial", 0) > gimbal_serial
                and observed["servo_rx"] >= gimbal_started
                and abs(observed.get("pitch_pwm", 0) - 1306) < 20
                and abs(observed.get("roll_pwm", 0) - 1698) < 20
            ),
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
        for message in (33, 36, 65, 245):
            command(MAV_CMD.SET_MESSAGE_INTERVAL, [message, 100000, 0, 0, 0, 0, 0])
        owner = (
            assembly.GetType("NOMAD.MissionPlanner.PayloadController")
            .GetProperty("Current", STATIC)
            .GetValue(None, None)
        )
        method = owner.GetType().GetMethod("ActAsync", INSTANCE)
        safe = marker_action(owner, method, "safe", 1000)
        require(str(safe) == "Safe accepted.", "Marker Safe rejected: " + str(safe))
        first = marker_action(owner, method, "activate", 1000, reports=2)
        state["first_confirmation"] = str(first)
        require(
            str(first) == "Confirmation incomplete or neutral needed." and observed.get("marker_pwm") == 1000,
            "First marker confirmation changed output: " + str(first),
        )
        marker_action(owner, method, "activate", 2000)
        state["marker_pwm"] = observed["marker_pwm"]
        capture(player, "nomad-inspection")
        marker_action(owner, method, "safe", 1000)
        set_links(True, True)
        check_vision(player)
        return_above_home()
        terminate_and_measure()
        verify_latch_and_ordinary_rates()
        state["landing_position"] = [observed["lat"], observed["lon"], observed["altitude"]]
        state.update(passed=True, landed_state=observed["landed"], disarmed=True)
        stage("aircraft_checks_passed")
        wait_for(
            lambda: File.Exists(Path.Combine(RUN, "postflight-verified")),
            45,
            "Launcher did not verify the live controller and camera lifecycle",
        )
        stage("passed")
except Exception as error:
    state.update(
        passed=False,
        error=str(error),
        failed_stage=state["stage"],
        last_observed=dict(observed),
        failure_locations=[{"function": row[2], "line": row[1]} for row in traceback.extract_tb(sys.exc_info()[2])],
    )
    if state["failed_stage"] == "vision_tracking":
        try:
            capture(player, "nomad-vision-failure")
        except Exception:
            pass
    stage("failed")
finally:
    if CHECK or state["stage"] == "failed":
        HOST.BeginInvoke(Action(HOST.Close))
