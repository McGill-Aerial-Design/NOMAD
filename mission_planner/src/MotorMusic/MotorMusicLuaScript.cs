// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

namespace NOMAD.MissionPlanner
{
    internal static class MotorMusicLuaScript
    {
        public const string Text =
@"-- SPDX-License-Identifier: Apache-2.0
-- NOMAD motor music speed bridge.
--
-- Install to APM/scripts/nomad_motor_music.lua and reboot or restart scripting.
-- Mission Planner sends MAV_CMD_USER_1:
--   p1 opcode: 1 note, 2 stop, 3 ping
--   p2 motor slot: 1..N, 0 means all discovered motors
--   p3 MIDI note, p4 velocity, p5 duration ms
--   p6/p7 are max/min ArduPilot motor outputs in PWM-equivalent microseconds.

local mavlink_msgs = require(""MAVLink/mavlink_msgs"")
local COMMAND_LONG_ID = mavlink_msgs.get_msgid(""COMMAND_LONG"")
local COMMAND_ACK_ID = mavlink_msgs.get_msgid(""COMMAND_ACK"")
local MAV_CMD_USER_1 = 31010

local MAV_RESULT_ACCEPTED = 0
local MAV_RESULT_TEMPORARILY_REJECTED = 1
local MAV_RESULT_DENIED = 2
local MAV_RESULT_FAILED = 4

local OP_NOTE = 1
local OP_STOP = 2
local OP_PING = 3

local STOP_OUTPUT = 1000
local DEFAULT_MIN_OUTPUT = 1100
local DEFAULT_MAX_OUTPUT = 1800
local MAX_DURATION_MS = 3000
local OUTPUT_TIMEOUT_MS = 120
local REFRESH_MS = 50
local UPDATE_MS = 20

local msg_map = {}
msg_map[COMMAND_LONG_ID] = ""COMMAND_LONG""

local motors = {}
local voices = {}

local function now_ms()
    return millis():toint()
end

local function clamp(value, min_value, max_value)
    if value < min_value then return min_value end
    if value > max_value then return max_value end
    return value
end

local function discover_motors()
    motors = {}
    for fn = 33, 44 do
        local chan = SRV_Channels:find_channel(fn)
        if chan ~= nil then
            motors[#motors + 1] = chan
        end
    end
    for i = 1, #motors do
        voices[i] = nil
    end
    gcs:send_text(6, string.format(""NOMAD motor music: %d motor outputs"", #motors))
end

local function clear_outputs()
    for i = 1, #motors do
        SRV_Channels:set_output_pwm_chan_timeout(motors[i], STOP_OUTPUT, 100)
        voices[i] = nil
    end
end

local function note_to_output(note, velocity, min_output, max_output)
    local note_norm = (note - 24.0) / 72.0
    note_norm = clamp(note_norm, 0.0, 1.0)

    local output = min_output + (max_output - min_output) * note_norm
    -- Velocity should not retune the note, but a very small trim keeps quiet
    -- passages from sitting at exactly the same motor speed.
    local trim = ((velocity / 127.0) - 0.5) * 0.08 * (max_output - min_output)
    return clamp(math.floor(output + trim + 0.5), min_output, max_output)
end

local function start_voice(slot, note, velocity, duration_ms, max_output, min_output)
    if #motors == 0 then return MAV_RESULT_FAILED end
    if arming:is_armed() then
        clear_outputs()
        return MAV_RESULT_DENIED
    end

    local first_slot = slot
    local last_slot = slot
    if slot <= 0 then
        first_slot = 1
        last_slot = #motors
    end
    if first_slot < 1 or first_slot > #motors then return MAV_RESULT_FAILED end
    if last_slot > #motors then last_slot = #motors end

    note = clamp(math.floor(note + 0.5), 24, 96)
    velocity = clamp(math.floor(velocity + 0.5), 1, 127)
    duration_ms = clamp(math.floor(duration_ms + 0.5), 20, MAX_DURATION_MS)
    if min_output == 0 then min_output = DEFAULT_MIN_OUTPUT end
    if max_output == 0 then max_output = DEFAULT_MAX_OUTPUT end
    min_output = clamp(math.floor(min_output + 0.5), STOP_OUTPUT, 2000)
    max_output = clamp(math.floor(max_output + 0.5), min_output, 2000)

    local output = note_to_output(note, velocity, min_output, max_output)

    local start = now_ms()
    for i = first_slot, last_slot do
        voices[i] = {
            note = note,
            output = output,
            end_ms = start + duration_ms,
            next_refresh_ms = start
        }
    end
    return MAV_RESULT_ACCEPTED
end

local function ack(chan, cmd, result)
    local reply = {}
    reply.command = cmd.command
    reply.result = result
    reply.progress = 0
    reply.result_param2 = 0
    reply.target_system = cmd.sysid
    reply.target_component = cmd.compid
    mavlink:send_chan(chan, mavlink_msgs.encode(""COMMAND_ACK"", reply))
end

local function handle_command(cmd)
    if cmd.command ~= MAV_CMD_USER_1 then return nil end
    local opcode = math.floor((cmd.param1 or 0) + 0.5)
    if opcode == OP_STOP then
        clear_outputs()
        return MAV_RESULT_ACCEPTED
    end
    if opcode == OP_PING then
        gcs:send_text(6, ""NOMAD motor music script alive"")
        return MAV_RESULT_ACCEPTED
    end
    if opcode == OP_NOTE then
        return start_voice(
            math.floor((cmd.param2 or 0) + 0.5),
            cmd.param3 or 60,
            cmd.param4 or 80,
            cmd.param5 or 120,
            cmd.param6 or DEFAULT_MAX_OUTPUT,
            cmd.param7 or DEFAULT_MIN_OUTPUT)
    end
    return MAV_RESULT_TEMPORARILY_REJECTED
end

local function run_voices()
    if arming:is_armed() then
        clear_outputs()
        return
    end

    local t = now_ms()
    for i = 1, #motors do
        local v = voices[i]
        if v ~= nil then
            if t >= v.end_ms then
                SRV_Channels:set_output_pwm_chan_timeout(motors[i], STOP_OUTPUT, 100)
                voices[i] = nil
            elseif t >= v.next_refresh_ms then
                SRV_Channels:set_output_pwm_chan_timeout(motors[i], v.output, OUTPUT_TIMEOUT_MS)
                v.next_refresh_ms = t + REFRESH_MS
            end
        end
    end
end

local function update()
    local msg, chan = mavlink:receive_chan()
    while msg ~= nil do
        local parsed = mavlink_msgs.decode(msg, msg_map)
        if parsed ~= nil and parsed.msgid == COMMAND_LONG_ID then
            local result = handle_command(parsed)
            if result ~= nil then ack(chan, parsed, result) end
        end
        msg, chan = mavlink:receive_chan()
    end

    run_voices()
    return update, UPDATE_MS
end

mavlink:init(20, 1)
mavlink:register_rx_msgid(COMMAND_LONG_ID)
mavlink:block_command(MAV_CMD_USER_1)
discover_motors()

return update, 100
";
    }
}
