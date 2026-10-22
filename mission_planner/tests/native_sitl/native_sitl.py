# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: E402
# Native MP integration fixture: simulator TCP only; normal host owns Init/Loop/Exit.
import sys
import traceback

import clr

clr.AddReference("System.Web.Extensions")
# Native CLR namespaces must retain execution order across Windows/Linux lint environments.
# isort: off
from MissionPlanner import MainV2 as MainType
from MissionPlanner.Comms import TcpSerial
from MissionPlanner.Plugin import PluginLoader
from Newtonsoft.Json.Linq import JArray
from System import Action, Activator, Array, Byte, Double, Environment, EventHandler, Int64, Object, Single
from System.Collections.Generic import Dictionary, List
from System.Diagnostics import Stopwatch
from System.IO import File, Path
from System.Net.Sockets import TcpClient
from System.Reflection import BindingFlags
from System.Threading import Monitor, Thread, Timeout, Timer
from System.Web.Script.Serialization import JavaScriptSerializer

from MAVLink import (
    MAV_CMD,
    MAVLinkMessage,
    mavlink_command_ack_t,
    mavlink_command_int_t,
    mavlink_command_long_t,
    mavlink_relay_status_t,
    mavlink_servo_output_raw_t,
)
# isort: on

HOST = MainV2  # noqa: F821 - injected by the native Mission Planner script host
STATIC = BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic
INSTANCE = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic
summary = Dictionary[str, Object]()
summary["passed"] = False
summary["stage"] = "preflight"
summary["watchdog"] = False
cases = List[Object]()
summary["cases"] = cases
connection = MainType.comPort
plugin = None
assembly = None
owner = None
handler = None
sent_handler = None
outgoing = []
frames = []
relays = []
acks = []
gate = Object()
clock = Stopwatch.StartNew()
callback_error = ""


def now():
    return clock.ElapsedMilliseconds / 1000.0


def ui(action):
    HOST.Invoke(Action(action))


def call_static(name, method, values):
    subject = assembly.GetType("NOMAD.MissionPlanner." + name)
    return subject.GetMethod(method, STATIC).Invoke(None, Array[Object](values))


def on_packet(sender, packet):
    global callback_error
    if packet.sysid != connection.sysidcurrent or packet.compid != connection.compidcurrent:
        return
    try:
        stamp = now()
        Monitor.Enter(gate)
        try:
            if int(packet.msgid) == 36:
                item = packet.ToStructure[mavlink_servo_output_raw_t]()
                frames.append(
                    (
                        stamp,
                        float(item.servo9_raw),
                        float(item.servo10_raw),
                        float(item.servo11_raw),
                        float(item.servo7_raw),
                        float(item.servo8_raw),
                    )
                )
            elif int(packet.msgid) == 376:
                item = packet.ToStructure[mavlink_relay_status_t]()
                relays.append((stamp, float(item.on), float(item.present)))
            elif int(packet.msgid) == 77:
                item = packet.ToStructure[mavlink_command_ack_t]()
                acks.append((stamp, float(item.command), float(item.result)))
        finally:
            Monitor.Exit(gate)
    except Exception as error:
        callback_error = type(error).__name__


def on_sent(sender, packet):
    if int(packet.msgid) in [75, 76, 157]:
        command = 0
        if int(packet.msgid) == 75:
            command = int(packet.ToStructure[mavlink_command_int_t]().command)
        elif int(packet.msgid) == 76:
            command = int(packet.ToStructure[mavlink_command_long_t]().command)
        Monitor.Enter(gate)
        try:
            outgoing.append((now(), int(packet.msgid), command))
        finally:
            Monitor.Exit(gate)


def snapshot(rows):
    Monitor.Enter(gate)
    try:
        return list(rows)
    finally:
        Monitor.Exit(gate)


def wait_for(rows, start, predicate, label, timeout=5.0):
    end = now() + timeout
    while now() < end:
        if callback_error:
            raise Exception("packet_decode_failed")
        for row in snapshot(rows):
            if row[0] > start and predicate(row):
                return row
        Thread.Sleep(20)
    raise Exception(label)


def record(label, row, value):
    result = Dictionary[str, Object]()
    result["case"] = label
    result["observed"] = float(value)
    result["at_seconds"] = float(row[0])
    cases.Add(result)


def action_category(text):
    statuses = {
        "Safe command ACK accepted. Physical state is unverified.": "safe_accepted",
        "Mission Planner received an accepted command ACK. Physical completion is unverified.": "activation_accepted",
        "Safe command is already pending.": "safe_pending",
        "Aircraft or input is unavailable; no command sent.": "unavailable",
        "Aircraft changed during Safe. Current aircraft safe state is unknown.": "safe_target_changed",
        "Mission Planner did not confirm Safe acceptance; physical state is unverified.": "safe_not_confirmed",
        (
            "Safe outcome is uncertain; inspect aircraft before retrying. "
            "Mission Planner is busy with a native request."
        ): "safe_host_busy",
        (
            "Safe outcome is uncertain; inspect aircraft before retrying. "
            "Aircraft or input changed; no new command was sent."
        ): "safe_target_retired",
    }
    if text in statuses:
        return statuses[text]
    if text.startswith("Safe outcome is uncertain; inspect aircraft before retrying. "):
        return "safe_uncertain_other"
    return "other_status"


def action_state():
    output = assembly.GetType("NOMAD.MissionPlanner.OutputController")
    target = call_static("OutputController", "CaptureTarget", [config])
    state = Dictionary[str, Object]()
    state["give_comport"] = bool(connection.giveComport)
    state["generation"] = float(output.GetProperty("Generation", STATIC).GetValue(None))
    state["payload_busy"] = bool(output.GetField("_payloadBusy", STATIC).GetValue(None))
    state["target_valid"] = target is not None
    state["target_current"] = target is not None and bool(
        target.GetType().GetProperty("IsCurrent", INSTANCE).GetValue(target, None)
    )
    return state


def action(identifier, operation, value=None):
    before = action_state()
    ack_start = len(snapshot(acks))
    outgoing_start = len(snapshot(outgoing))
    start = now()
    task = (
        owner.GetType()
        .GetMethod("ActAsync", INSTANCE)
        .Invoke(owner, Array[Object]([identifier, operation, value, "ui", -1, None]))
    )
    text = str(task.GetAwaiter().GetResult())
    diagnostic = Dictionary[str, Object]()
    diagnostic["category"] = action_category(text)
    diagnostic["before"] = before
    diagnostic["after"] = action_state()
    diagnostic["acks"] = Array[Object]([Array[Object](row) for row in snapshot(acks)[ack_start:]][-8:])
    diagnostic["outgoing"] = Array[Object]([Array[Object](row) for row in snapshot(outgoing)[outgoing_start:]][-8:])
    if identifier in ["toggle", "position", "motion", "relay", "pulse"] and operation in [
        "safe",
        "activate",
        "position",
        "positive",
        "negative",
    ]:
        diagnostic["action"] = identifier + "_" + operation
    if not summary.ContainsKey("action_diagnostics"):
        summary["action_diagnostics"] = List[Object]()
    if summary["action_diagnostics"].Count < 32:
        summary["action_diagnostics"].Add(diagnostic)
    if "accepted" not in text.lower() or "uncertain" in text.lower() or "not confirm" in text.lower():
        raise Exception(identifier + "_" + operation + "_not_accepted")
    wait_for(
        acks,
        start,
        lambda row: (
            row[1]
            == (
                181
                if identifier == "relay" or (identifier == "pulse" and operation == "safe")
                else 182
                if identifier == "pulse"
                else 184
                if identifier == "motion" and operation in ["positive", "negative"]
                else 183
            )
            and row[2] == 0
        ),
        "accepted_ack_missing",
    )
    return start


def pwm(identifier, operation, index, expected, label, value=None):
    start = action(identifier, operation, value)
    row = wait_for(frames, start, lambda item: abs(item[index] - expected) <= 2, label)
    record(label, row, row[index])
    return row


def relay(identifier, operation, bit, expected, label):
    start = action(identifier, operation)
    row = wait_for(
        relays, start, lambda item: (int(item[2]) & (1 << bit)) != 0 and ((int(item[1]) >> bit) & 1) == expected, label
    )
    record(label, row, expected)
    return row


def close_watchdog(state):
    summary["watchdog"] = True
    HOST.BeginInvoke(Action(HOST.Close))


def closed(sender, event):
    try:
        summary["disconnected"] = not connection.BaseStream.IsOpen
        output = assembly.GetType("NOMAD.MissionPlanner.OutputController") if assembly is not None else None
        summary["exit_retired_owner"] = output is not None and output.GetField("_config", STATIC).GetValue(None) is None
        summary["passed"] = bool(
            summary["passed"] and summary["disconnected"] and summary["exit_retired_owner"] and not summary["watchdog"]
        )
    except Exception as error:
        summary["cleanup_error"] = type(error).__name__
        summary["passed"] = False
    finally:
        watchdog.Dispose()
        if handler is not None:
            connection.OnPacketReceived -= handler
        if sent_handler is not None:
            connection.OnPacketSent -= sent_handler
        summary["pwm_sample_count"] = float(len(frames))
        summary["relay_sample_count"] = float(len(relays))
        summary["ack_count"] = float(len(acks))
        File.WriteAllText(
            Path.Combine(Path.GetDirectoryName(__file__), "summary.json"), JavaScriptSerializer().Serialize(summary)
        )


watchdog = Timer(close_watchdog, None, 90000, Timeout.Infinite)
HOST.FormClosed += closed
try:
    candidates = [
        item for item in PluginLoader.Plugins if item.GetType().FullName == "NOMAD.MissionPlanner.NOMADPlugin"
    ]
    if len(candidates) != 1:
        raise Exception("native_plugin_missing")
    plugin = candidates[0]
    assembly = plugin.GetType().Assembly
    loaded_config = plugin.GetType().GetField("_config", INSTANCE).GetValue(plugin)
    service = plugin.GetType().GetField("_joystickService", INSTANCE).GetValue(plugin)
    if loaded_config.RouterClientEnabled or service.NeedsToRun() or loaded_config.GimbalArrowKeysEnabled:
        raise Exception("disable_saved_router_and_inputs_before_test")
    summary["loaded_screen_registered"] = bool(
        plugin.GetType().GetField("_screenRegistered", INSTANCE).GetValue(plugin)
    )
    if not summary["loaded_screen_registered"]:
        raise Exception("native_loaded_missing")
    identity = assembly.GetType("NOMAD.MissionPlanner.NomadRelease")
    source = identity.GetField("SourceSha", STATIC).GetRawConstantValue()
    if source != Environment.GetEnvironmentVariable("NOMAD_SITL_SOURCE"):
        raise Exception("unexpected_product_source")
    summary["source_sha"] = source
    if connection.BaseStream.IsOpen or any(item.BaseStream.IsOpen for item in MainType.Comports):
        raise Exception("unexpected_open_connection")
    next_run = plugin.NextRun
    bind_time = plugin.GetType().GetField("_nextBoundaryMapBindUtc", INSTANCE).GetValue(plugin)
    tcp = TcpSerial()
    tcp.Host = "127.0.0.1"
    tcp.Port = Environment.GetEnvironmentVariable("NOMAD_SITL_PORT")
    tcp.autoReconnect = False
    tcp.client = TcpClient("127.0.0.1", int(tcp.Port))
    connection.ReadOnly = False
    connection.CONNECT_TIMEOUT_SECONDS = 10
    connection.BaseStream = tcp
    handler = EventHandler[MAVLinkMessage](on_packet)
    if handler.Target is None:
        raise Exception("native_delegate_target_missing")
    connection.OnPacketReceived += handler
    sent_handler = EventHandler[MAVLinkMessage](on_sent)
    connection.OnPacketSent += sent_handler
    summary["stage"] = "native_connect"
    ui(lambda: connection.Open(False, True, False))
    summary["simulator_sysid"] = float(connection.sysidcurrent)
    if not connection.BaseStream.IsOpen or connection.sysidcurrent != 201:
        raise Exception("not_expected_simulator")
    if connection.MAV.cs.armed:
        raise Exception("simulator_unexpectedly_armed")
    # Request only the output/status streams needed for independent observation.
    for message in [36, 376]:
        request_start = now()
        accepted = (
            connection.doCommandAsync(
                Byte(connection.sysidcurrent),
                Byte(connection.compidcurrent),
                MAV_CMD.SET_MESSAGE_INTERVAL,
                Single(message),
                Single(100000),
                Single(0),
                Single(0),
                Single(0),
                Single(0),
                Single(0),
                True,
            )
            .GetAwaiter()
            .GetResult()
        )
        if not accepted:
            summary["rejected_stream_message_id"] = float(message)
            rejected = [row for row in snapshot(acks) if row[0] > request_start and row[1] == 511]
            if rejected:
                summary["rejected_stream_ack_result"] = rejected[-1][2]
            raise Exception("output_stream_request_rejected")
    wait_for(frames, 0, lambda row: True, "servo_stream_absent")
    wait_for(relays, 0, lambda row: int(row[2]) & 3 == 3, "relay_stream_absent")
    summary["stage"] = "payload"
    config = Activator.CreateInstance(assembly.GetType("NOMAD.MissionPlanner.NOMADConfig"))
    config.AircraftName = "Synthetic native SITL"
    config.ActuatorDefinitions = JArray.Parse(
        File.ReadAllText(Path.Combine(Path.GetDirectoryName(__file__), "payloads.json"))
    )
    call_static("OutputController", "Initialize", [config])
    call_static("PayloadController", "ApplyProfile", [config])
    call_static("GimbalController", "ApplyProfile", [config])
    owner = assembly.GetType("NOMAD.MissionPlanner.PayloadController").GetProperty("Current", STATIC).GetValue(None)
    pwm("toggle", "safe", 1, 1000, "toggle_safe")
    pwm("toggle", "activate", 1, 2000, "toggle_active")
    pwm("toggle", "safe", 1, 1000, "toggle_return")
    pwm("position", "position", 2, 1250, "position_quarter", 0.25)
    pwm("position", "safe", 2, 1500, "position_safe")
    pwm("motion", "safe", 3, 1500, "motion_safe")
    active = pwm("motion", "positive", 3, 2000, "motion_positive")
    returned = wait_for(frames, active[0], lambda row: abs(row[3] - 1500) <= 2, "motion_native_return_absent")
    record("motion_native_return", returned, returned[3])
    Thread.Sleep(100)
    pwm("motion", "safe", 3, 1500, "motion_explicit_safe")
    active = pwm("motion", "negative", 3, 1000, "motion_negative")
    returned = wait_for(frames, active[0], lambda row: abs(row[3] - 1500) <= 2, "negative_native_return_absent")
    record("motion_negative_native_return", returned, returned[3])
    Thread.Sleep(100)
    pwm("motion", "safe", 3, 1500, "motion_negative_explicit_safe")
    relay("relay", "safe", 0, 0, "relay_safe")
    relay("relay", "activate", 0, 1, "relay_active")
    relay("relay", "safe", 0, 0, "relay_return")
    active = relay("pulse", "activate", 1, 1, "pulse_active")
    returned = wait_for(
        relays, active[0], lambda row: int(row[2]) & 2 and int(row[1]) & 2 == 0, "pulse_native_return_absent"
    )
    record("pulse_native_return", returned, 0)
    Thread.Sleep(100)
    relay("pulse", "safe", 1, 0, "pulse_explicit_safe")
    summary["stage"] = "gimbal"
    start = now()
    result = call_static("GimbalController", "RequestPitchRollTargetAsync", [Single(-30), Single(15)])
    result.GetAwaiter().GetResult()
    result = result.GetType().GetProperty("Result").GetValue(result, None)
    if str(result.GetType().GetProperty("Outcome", INSTANCE).GetGetMethod(True).Invoke(result, None)) != "Submitted":
        raise Exception("gimbal_target_not_submitted")
    summary["gimbal_submitted"] = True
    row = wait_for(frames, start, lambda item: item[4] < 1450 and item[5] > 1550, "gimbal_pwm_response_absent")
    record("gimbal_pitch_pwm", row, row[4])
    record("gimbal_roll_pwm", row, row[5])
    result = call_static("GimbalController", "RequestPitchRollTargetAsync", [Single(0), Single(0)])
    result.GetAwaiter().GetResult()
    result = result.GetType().GetProperty("Result").GetValue(result, None)
    if str(result.GetType().GetProperty("Outcome", INSTANCE).GetGetMethod(True).Invoke(result, None)) != "Submitted":
        raise Exception("gimbal_neutral_not_submitted")
    row = wait_for(
        frames, now(), lambda item: abs(item[4] - 1500) <= 25 and abs(item[5] - 1500) <= 25, "gimbal_neutral_absent"
    )
    record("gimbal_neutral", row, row[4])
    summary["stage"] = "admission"
    count = len(outgoing)
    generation = (
        assembly.GetType("NOMAD.MissionPlanner.OutputController").GetProperty("Generation", STATIC).GetValue(None)
    )
    call_static("OutputController", "Initialize", [config])
    summary["stage"] = "stale_dispatch"
    stale = call_static("OutputController", "SendGimbalAsync", [2, Double(-20), Double(10), Int64(generation), None])
    stale.GetAwaiter().GetResult()
    stale = stale.GetType().GetProperty("Result").GetValue(stale, None)
    if str(stale.GetType().GetProperty("Outcome", INSTANCE).GetGetMethod(True).Invoke(stale, None)) != "NotSent":
        raise Exception("stale_generation_accepted")
    summary["stage"] = "readonly_dispatch"
    connection.ReadOnly = True
    blocked = call_static("GimbalController", "RequestPitchRollTargetAsync", [Single(-20), Single(10)])
    blocked.GetAwaiter().GetResult()
    blocked = blocked.GetType().GetProperty("Result").GetValue(blocked, None)
    if str(blocked.GetType().GetProperty("Outcome", INSTANCE).GetGetMethod(True).Invoke(blocked, None)) != "NotSent":
        raise Exception("readonly_gimbal_accepted")
    denied = (
        owner.GetType()
        .GetMethod("ActAsync", INSTANCE)
        .Invoke(owner, Array[Object](["toggle", "activate", None, "ui", -1, None]))
        .GetAwaiter()
        .GetResult()
    )
    if "no command" not in str(denied).lower():
        raise Exception("readonly_payload_accepted")
    Thread.Sleep(150)
    if len(outgoing) != count:
        raise Exception("rejected_input_emitted_command")
    summary["stale_generation_no_command"] = True
    summary["readonly_no_command"] = True
    connection.ReadOnly = False
    summary["scheduler_advanced"] = plugin.NextRun > next_run
    summary["loop_body_advanced"] = (
        plugin.GetType().GetField("_nextBoundaryMapBindUtc", INSTANCE).GetValue(plugin) > bind_time
    )
    summary["no_raw_telemetry_logs"] = connection.logfile is None and connection.rawlogfile is None
    summary["passed"] = bool(
        summary["scheduler_advanced"] and summary["loop_body_advanced"] and summary["no_raw_telemetry_logs"]
    )
except Exception as error:
    summary["error_type"] = type(error).__name__
    summary["error_line"] = float(traceback.extract_tb(sys.exc_info()[2])[-1][1])
    code = str(error)
    summary["failure_code"] = (
        code if code and all(character.isalnum() or character == "_" for character in code) else "native_exception"
    )
finally:
    try:
        if connection.BaseStream.IsOpen:
            ui(lambda: HOST.doDisconnect(connection))
    except Exception as error:
        summary["disconnect_error"] = type(error).__name__
        summary["passed"] = False
    HOST.BeginInvoke(Action(HOST.Close))
