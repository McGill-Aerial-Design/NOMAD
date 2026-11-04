--[[
 Auto Filter Tune (AFT): guided gyro harmonic-notch setup for multicopters.

 Workflow (the script never commands the vehicle and never changes filters while armed):
   1. Measure: fly a normal, gentle hover in any mode. The script samples ESC RPM, the
      filtered gyro, AHRS vibration, throttle, yaw output and the native FFT hover estimate.
   2. Calculate: after disarm it checks for faults, then prints a recommended notch set.
   3. Apply: on the ground set AFT_ACTION=1. Previous values are written to aft_session.txt
      first. Reboot.
   4. Verify: fly the same hover. After disarm the residual gyro noise is compared with the
      measurement flight. A worse or faulty flight restores the previous values automatically.
   5. Accept with AFT_ACTION=2, or restore at any time on the ground with AFT_ACTION=3.
--]]
-- luacheck: only 0

local TABLE_KEY = 94
local PREFIX = "AFT_"
local SESSION_FILE = "aft_session.txt"
local MAV_ERR, MAV_WARN, MAV_NOTICE, MAV_INFO = 3, 4, 5, 6
local RAD2DEG = 57.29578

assert(param:add_table(TABLE_KEY, PREFIX, 16), "AFT: could not add param table")
local function bind(name, idx, default)
    assert(param:add_param(TABLE_KEY, idx, name, default), "AFT: could not add " .. name)
    return Parameter(PREFIX .. name)
end

--[[
  // @Param: AFT_ENABLE
  // @DisplayName: Auto Filter Tune enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local AFT_ENABLE = bind("ENABLE", 1, 0)
--[[
  // @Param: AFT_ACTION
  // @DisplayName: Auto Filter Tune ground action
  // @Description: Set while disarmed. Returns to 0 once handled.
  // @Values: 0:None,1:Apply recommendation,2:Accept verified settings,3:Restore previous settings,4:Reset session
  // @User: Standard
--]]
local AFT_ACTION = bind("ACTION", 2, 0)
--[[
  // @Param: AFT_HOVER_S
  // @DisplayName: Required steady hover time
  // @Units: s
  // @Range: 10 300
--]]
local AFT_HOVER_S = bind("HOVER_S", 3, 30)
--[[
  // @Param: AFT_IMB_PCT
  // @DisplayName: Maximum motor RPM imbalance
  // @Description: A motor whose mean hover RPM differs from the motor average by more than this refuses the tune.
  // @Units: %
  // @Range: 5 50
--]]
local AFT_IMB_PCT = bind("IMB_PCT", 4, 15)
--[[
  // @Param: AFT_NOTCH_MAX
  // @DisplayName: Notch budget
  // @Description: Maximum notch filters (sources x harmonics x notch multiplicity x active IMUs). Lua cannot read CPU load, so this is the CPU guard. Lower it on F4/F7 boards.
  // @Range: 4 64
--]]
local AFT_NOTCH_MAX = bind("NOTCH_MAX", 5, 24)
--[[
  // @Param: AFT_VERIFY_RAT
  // @DisplayName: Verification noise ratio
  // @Description: Verification passes when residual gyro noise is at most this multiple of the measurement flight.
  // @Range: 0.5 1.5
--]]
local AFT_VERIFY_RAT = bind("VERIFY_RAT", 6, 1.05)
--[[
  // @Param: AFT_VIBE_MAX
  // @DisplayName: Maximum AHRS vibration
  // @Units: m/s/s
  // @Range: 10 60
--]]
local AFT_VIBE_MAX = bind("VIBE_MAX", 7, 30)
--[[
  // @Param: AFT_RPM_OK
  // @DisplayName: Required ESC RPM availability
  // @Description: Fraction of hover samples in which every motor must report fresh RPM for RPM tracking.
  // @Range: 0.5 1
--]]
local AFT_RPM_OK = bind("RPM_OK", 8, 0.95)
--[[
  // @Param: AFT_YAW_MAX
  // @DisplayName: Maximum mean yaw output
  // @Description: A persistent yaw output bias above this in hover indicates motor tilt, direction or thrust imbalance.
  // @Range: 0.05 0.5
--]]
local AFT_YAW_MAX = bind("YAW_MAX", 9, 0.15)
--[[
  // @Param: AFT_OSC_DPS
  // @DisplayName: Maximum hover body rate
  // @Description: Mean low-passed roll/pitch rate in steady hover above this is treated as oscillation.
  // @Units: deg/s
  // @Range: 2 30
--]]
local AFT_OSC_DPS = bind("OSC_DPS", 10, 8)
--[[
  // @Param: AFT_BLADES
  // @DisplayName: Propeller blade count
  // @Description: Selects the starting harmonics: 2 blades load the 2nd harmonic, 3 blades the 3rd. 0 keeps an enabled bank's harmonics, else uses 1st-3rd.
  // @Values: 0:Unknown,2:Two,3:Three
--]]
local AFT_BLADES = bind("BLADES", 11, 0)
--[[
  // @Param: AFT_LPF_MAX
  // @DisplayName: Gyro low-pass ceiling
  // @Description: Once the notches match the recommendation, raise INS_GYRO_FILTER in verified 25% steps up to this. 0 disables the low-pass stage.
  // @Units: Hz
  // @Range: 0 300
--]]
local AFT_LPF_MAX = bind("LPF_MAX", 12, 0)
--[[
  // @Param: AFT_LPF_RAT
  // @DisplayName: Low-pass step noise ratio
  // @Description: A low-pass step passes when residual gyro noise is at most this multiple of the flight before it.
  // @Range: 1 2
--]]
local AFT_LPF_RAT = bind("LPF_RAT", 13, 1.3)
--[[
  // @Param: AFT_TEMP_RISE
  // @DisplayName: Maximum motor temperature rise
  // @Description: Verification fails when the hottest ESC/motor temperature exceeds the previous flight's by more than this. Needs ESC temperature telemetry.
  // @Units: degC
  // @Range: 2 40
--]]
local AFT_TEMP_RISE = bind("TEMP_RISE", 14, 10)
--[[
  // @Param: AFT_AUTO
  // @DisplayName: Automation level
  // @Description: 0: every change waits for AFT_ACTION. 1: apply, accept and restore automatically after each flight; arming stays blocked until you reboot. 2: as 1, and reboot automatically after 5 s disarmed on the ground.
  // @Values: 0:Manual,1:Auto without reboot,2:Fully automatic
--]]
local AFT_AUTO = bind("AUTO", 15, 2)

local STEADY_RAD = 0.6      -- low-passed roll/pitch rate gate for a "steady" sample
local STOP_FRACTION = 0.1   -- motor at <10% of the others' mean RPM counts as stopped
local STOP_MS = 300

-- STATUSTEXT carries 50 characters: wrap at spaces, continuation lines start with "AFT+".
-- Lines are queued and sent one per 100 ms: an end-of-flight burst would overflow the GCS text queue.
local outbox = {}
local last_sent_ms = 0

local function say(sev, fmt, ...)
    local text = string.format(fmt, ...)
    local prefix = "AFT: "
    while #text > 45 do
        local space = text:sub(1, 46):match("^.*() ")
        local cut = space and space - 1 or 45
        outbox[#outbox + 1] = { sev, prefix .. text:sub(1, cut) }
        text = text:sub(cut + (space and 2 or 1))
        prefix = "AFT+ "
    end
    outbox[#outbox + 1] = { sev, prefix .. text }
end

local function flush_outbox(now)
    if #outbox > 0 and now - last_sent_ms >= 100 then
        local line = table.remove(outbox, 1)
        gcs:send_text(line[1], line[2])
        last_sent_ms = now
    end
end

local function bits(value)
    return math.floor((value or 0) + 0.5)
end

local function popcount(mask)
    local n = 0
    while mask > 0 do
        n = n + (mask & 1)
        mask = mask >> 1
    end
    return n
end

local function finite(x)
    return x == x and x ~= math.huge and x ~= -math.huge
end

-- ESC telemetry index == output channel - 1 for motor functions 33..40 (Motor1..Motor8)
local function motor_outputs()
    local outs = {}
    for ch = 1, 16 do
        local fn = param:get(string.format("SERVO%d_FUNCTION", ch))
        if fn and fn >= 33 and fn <= 40 then
            outs[#outs + 1] = ch - 1
        end
    end
    return outs
end

local function healthy_imus()
    local n = 0
    for i = 0, 2 do
        if ins:get_gyro_health(i) then
            n = n + 1
        end
    end
    return n
end

local function bank_of(name)
    return name:match("^(INS_HNTC[H2])_")
end

-- A disabled bank's sub-parameter defaults can change once it is enabled (see ArduPilot PR #33879),
-- so its values are never trusted: enable, reboot, then write every value.
local function bank_disabled(name)
    local bank = bank_of(name)
    return bank ~= nil and (param:get(bank .. "_ENABLE") or 0) < 1
end

----------------------------------------------------------------------------------------------------
-- Session persistence: phase, baseline metrics, recommendation and backup survive reboots.
----------------------------------------------------------------------------------------------------
local session = { phase = "IDLE", base = {}, rec = {}, bak = {} }

local function save_session()
    local f = io.open(SESSION_FILE, "w")
    if not f then
        say(MAV_ERR, "cannot write %s", SESSION_FILE)
        return false
    end
    f:write("phase=" .. session.phase .. "\n")
    for k, v in pairs(session.base) do
        f:write(string.format("base.%s=%.7g\n", k, v))
    end
    for _, e in ipairs(session.rec) do
        f:write(string.format("rec.%s=%.7g\n", e[1], e[2]))
    end
    for _, e in ipairs(session.bak) do
        f:write(string.format("bak.%s=%.7g\n", e[1], e[2]))
    end
    f:close()
    return true
end

local function load_session()
    local f = io.open(SESSION_FILE, "r")
    if not f then
        return
    end
    for line in f:lines() do
        local phase = line:match("^phase=(%u+)")
        local kind, key, value = line:match("^(%a+)%.([%w_]+)=(.+)$")
        value = tonumber(value)
        if phase then
            session.phase = phase
        elseif kind == "base" and value then
            session.base[key] = value
        elseif kind == "rec" and value then
            session.rec[#session.rec + 1] = { key, value }
        elseif kind == "bak" and value then
            session.bak[#session.bak + 1] = { key, value }
        end
    end
    f:close()
end

local function has_backup(name)
    for _, e in ipairs(session.bak) do
        if e[1] == name then
            return true
        end
    end
    return false
end

----------------------------------------------------------------------------------------------------
-- In-flight measurement
----------------------------------------------------------------------------------------------------
local motors_out = {}
local m = nil

local function new_stats()
    return {
        steady_s = 0, n = 0, noise_sq = 0, rate_sum = 0, yaw_sum = 0, thr_sum = 0,
        vibe_sum = 0, vibe_max = 0, rpm_n = 0, rpm_fresh = 0, rpm_sum = {}, pwm_sum = {}, hz_min = math.huge,
        stop_ms = {}, spikes = {}, prev_hz = nil, temp_max = nil, unhealthy_ms = 0, last_gyro = nil, last_ms = 0, rate_lp = 0, fault = nil,
        fft_start = param:get("FFT_FREQ_HOVER"),
    }
end

local function sample(now, dt)
    -- spool state 3 = THROTTLE_UNLIMITED: excludes spool-up, touchdown and spool-down
    if not vehicle:get_likely_flying() or motors:get_spool_state() ~= 3 then
        m.last_gyro = nil
        return
    end
    local g = ins:get_gyro(0)
    local thr = motors:get_throttle()
    local rate = math.sqrt(g:x() ^ 2 + g:y() ^ 2)
    m.rate_lp = m.rate_lp + math.min(dt / 0.2, 1) * (rate - m.rate_lp)
    -- a touchdown jolt can flag one sample; a persistent loss of health is a fault
    m.unhealthy_ms = ins:get_gyro_health(0) and 0 or m.unhealthy_ms + dt * 1000
    if m.unhealthy_ms > 200 then
        m.fault = "primary gyro unhealthy"
    end
    local vibe = ahrs:get_vibration():length()
    m.vibe_max = math.max(m.vibe_max, vibe)

    -- per-motor RPM: freshness, stoppage and imbalance
    local hz, fresh, total = {}, true, 0
    for i, out in ipairs(motors_out) do
        local rpm = esc_telem:get_rpm(out)
        if rpm then
            hz[i] = rpm / 60
            total = total + hz[i]
        else
            fresh = false
        end
    end
    -- touchdown can briefly stop a motor while the landing detector settles: only judge lifting flight
    if fresh and #motors_out > 1 and thr > 0.1 then
        for i = 1, #motors_out do
            local others = (total - hz[i]) / (#motors_out - 1)
            if others > 5 and hz[i] < STOP_FRACTION * others then
                m.stop_ms[i] = (m.stop_ms[i] or 0) + dt * 1000
                if m.stop_ms[i] > STOP_MS then
                    m.fault = string.format("motor %d stopped %.0f/%.0fHz thr %.2f", i, hz[i], others, thr)
                end
            else
                m.stop_ms[i] = 0
            end
        end
    end

    -- climbs and descents are not hover: their RPM dips would drag FREQ down
    local vel = ahrs:get_velocity_NED()
    local level = vel ~= nil and math.abs(vel:z()) < 0.5
    local steady = level and thr > 0.1 and thr < 0.8 and m.rate_lp < STEADY_RAD
    if not steady then
        m.last_gyro = nil
        return
    end
    m.steady_s = m.steady_s + dt
    m.n = m.n + 1
    m.rate_sum = m.rate_sum + m.rate_lp
    m.yaw_sum = m.yaw_sum + motors:get_yaw()
    m.thr_sum = m.thr_sum + thr
    m.vibe_sum = m.vibe_sum + vibe
    m.rpm_n = m.rpm_n + 1
    -- The rate loop makes a weak motor reach normal RPM by commanding it harder:
    -- output spread, not RPM spread, is what shows it.
    for i, out in ipairs(motors_out) do
        local pwm = SRV_Channels:get_output_pwm_chan(out)
        m.pwm_sum[i] = (m.pwm_sum[i] or 0) + (pwm and pwm - 1000 or 0)
    end
    if fresh and #motors_out > 0 then
        m.rpm_fresh = m.rpm_fresh + 1
        for i = 1, #motors_out do
            m.rpm_sum[i] = (m.rpm_sum[i] or 0) + hz[i]
            m.hz_min = math.min(m.hz_min, hz[i])
        end
        -- One motor's RPM jumping >30% between ~10 ms samples while the others hold is a telemetry
        -- glitch (typically a timer/DMA conflict on that output), not a real speed change.
        local prev = m.prev_hz
        if prev and #motors_out > 1 then
            for i = 1, #motors_out do
                local others_now = (total - hz[i]) / (#motors_out - 1)
                local others_prev = (prev.total - prev[i]) / (#motors_out - 1)
                local others_steady = math.abs(others_now - others_prev) < 0.05 * others_prev
                if prev[i] > 20 and math.abs(hz[i] - prev[i]) > 0.3 * prev[i] and others_steady then
                    m.spikes[i] = (m.spikes[i] or 0) + 1
                end
            end
        end
        hz.total = total
        m.prev_hz = hz
    else
        m.prev_hz = nil
    end
    -- hottest motor (or ESC) temperature, for the "motors come down hot" check
    for _, out in ipairs(motors_out) do
        local cdeg = esc_telem:get_motor_temperature(out) or esc_telem:get_temperature(out)
        if cdeg and cdeg > 0 then
            m.temp_max = math.max(m.temp_max or -100, cdeg / 100)
        end
    end
    -- Residual (post-filter) gyro noise from first differences: var(diff) = 2 var(noise) per axis.
    -- ponytail: the sample interval is jittered so tones locked to the script rate do not alias to DC.
    if m.last_gyro and now - m.last_ms < 30 then
        local dx, dy = g:x() - m.last_gyro:x(), g:y() - m.last_gyro:y()
        m.noise_sq = m.noise_sq + (dx * dx + dy * dy) / 4
    end
    m.last_gyro = g
    m.last_ms = now
end

local function summarize()
    local s = { steady_s = m.steady_s, fault = m.fault, vibe_max = m.vibe_max, temp_max = m.temp_max }
    if m.n < 2 then
        return s
    end
    s.spike_rate, s.spike_motor = 0, 0
    for i = 1, #motors_out do
        local rate = (m.spikes[i] or 0) / math.max(1, m.rpm_fresh)
        if rate > s.spike_rate then
            s.spike_rate, s.spike_motor = rate, i
        end
    end
    s.noise = math.sqrt(m.noise_sq / (m.n - 1)) * RAD2DEG
    s.osc = m.rate_sum / m.n * RAD2DEG
    s.yaw = m.yaw_sum / m.n
    s.thr = m.thr_sum / m.n
    s.vibe = m.vibe_sum / m.n
    s.fresh = m.rpm_n > 0 and m.rpm_fresh / m.rpm_n or 0
    s.out_imbalance, s.out_motor = 0, 0
    if #motors_out > 1 then
        local sum = 0
        for i = 1, #motors_out do
            sum = sum + m.pwm_sum[i]
        end
        local mean = sum / #motors_out
        for i = 1, #motors_out do
            local dev = mean > 0 and math.abs(m.pwm_sum[i] / mean - 1) * 100 or 0
            if dev > s.out_imbalance then
                s.out_imbalance, s.out_motor = dev, i
            end
        end
    end
    if m.rpm_fresh > 0 then
        local sum, worst, worst_i = 0, 0, 0
        s.motor_hz = {}
        for i = 1, #motors_out do
            s.motor_hz[i] = m.rpm_sum[i] / m.rpm_fresh
            sum = sum + s.motor_hz[i]
        end
        s.hz_mean = sum / #motors_out
        for i = 1, #motors_out do
            local dev = math.abs(s.motor_hz[i] / s.hz_mean - 1) * 100
            if dev > worst then
                worst, worst_i = dev, i
            end
        end
        s.imbalance, s.imbalance_motor = worst, worst_i
        s.hz_min = m.hz_min
    end
    if (param:get("FFT_ENABLE") or 0) > 0 then
        local fft = param:get("FFT_FREQ_HOVER")
        if fft and fft ~= m.fft_start then
            s.fft_hz = fft
        else
            say(MAV_WARN, "FFT hover frequency did not update; FFT data not used")
        end
    end
    return s
end

-- Returns nil when the data is usable, otherwise the reason for refusing.
local function refusal(s)
    if s.fault then
        return s.fault
    end
    if s.steady_s < AFT_HOVER_S:get() then
        return string.format("need %.0fs steady hover, got %.0fs", AFT_HOVER_S:get(), s.steady_s)
    end
    if s.vibe_max > AFT_VIBE_MAX:get() then
        return string.format("vibration %.0f m/s/s, fix mounting/props first", s.vibe_max)
    end
    if math.abs(s.yaw) > AFT_YAW_MAX:get() then
        return string.format("yaw output bias %.2f, check motor tilt/direction/thrust", s.yaw)
    end
    if s.osc > AFT_OSC_DPS:get() then
        return string.format("hover rate %.1f deg/s, oscillation: tune PIDs first", s.osc)
    end
    if s.out_imbalance > AFT_IMB_PCT:get() then
        return string.format("motor %d output %.0f%% off mean, check motor/ESC/prop", s.out_motor, s.out_imbalance)
    end
    if s.spike_rate > 0.005 then
        return string.format("motor %d RPM spikes (%.1f%% of samples): output timer/DMA conflict? move ESC "
            .. "to another output", s.spike_motor, s.spike_rate * 100)
    end
    if s.fresh >= AFT_RPM_OK:get() and s.imbalance > AFT_IMB_PCT:get() then
        return string.format("motor %d RPM %.0f%% off mean, check motor/prop", s.imbalance_motor, s.imbalance)
    end
    return nil
end

----------------------------------------------------------------------------------------------------
-- Recommendation
----------------------------------------------------------------------------------------------------
local function notch_multiplicity(opts)
    if opts & 1 ~= 0 then return 2 end
    if opts & 16 ~= 0 then return 3 end
    if opts & 64 ~= 0 then return 5 end
    return 1
end

local function recommend(s)
    local rec = {}
    local function put(name, value)
        rec[#rec + 1] = { name, value }
    end
    local gyro_hz = 1000 * 2 ^ bits(param:get("INS_GYRO_RATE"))
    local limit_hz = 0.45 * gyro_hz
    local imus = math.max(1, healthy_imus())
    -- Starting harmonics for a fresh bank: 1st-3rd covers 2- and 3-blade props (2 blades load the
    -- 2nd, 3 blades the 3rd). An enabled bank keeps its harmonics; the pruning stage trims them.
    local opts, hmncs = 0, bits(AFT_BLADES:get()) == 2 and 3 or 7
    if not bank_disabled("INS_HNTCH_HMNCS") then
        opts = bits(param:get("INS_HNTCH_OPTS"))
        local current = bits(param:get("INS_HNTCH_HMNCS"))
        hmncs = current > 0 and current or hmncs
    end
    local mult = notch_multiplicity(opts)
    local budget = AFT_NOTCH_MAX:get()
    local rpm_ok = s.hz_mean and s.fresh >= AFT_RPM_OK:get()
    local fft_on = (param:get("FFT_ENABLE") or 0) > 0
    local resonance = nil

    -- The FFT sees the raw spectrum. Match its dominant hover peak to a motor harmonic.
    if s.fft_hz and s.hz_mean then
        local ratio = s.fft_hz / s.hz_mean
        local k = math.floor(ratio + 0.5)
        if k >= 1 and k <= 4 and math.abs(ratio - k) / k < 0.15 then
            say(MAV_INFO, "FFT peak %.0fHz is motor harmonic %d", s.fft_hz, k)
            hmncs = hmncs | (1 << (k - 1))
        else
            resonance = s.fft_hz
            say(MAV_INFO, "FFT peak %.0fHz is %.2fx motor freq: separate resonance", s.fft_hz, ratio)
        end
    end

    local base_hz
    if rpm_ok then
        -- ESC FREQ is the lowest tracked motor speed: below hover, above INS_GYRO_FILTER (ArduPilot docs)
        base_hz = math.max(param:get("INS_GYRO_FILTER") or 20, math.floor(0.8 * s.hz_min + 0.5))
        put("INS_HNTCH_ENABLE", 1)
        put("INS_HNTCH_MODE", 3)
        put("INS_HNTCH_REF", 1)
        say(MAV_INFO, "RPM fresh %.0f%% on %d motors: ESC tracking", s.fresh * 100, #motors_out)
    elseif fft_on then
        -- No usable RPM: FFT-driven notch. Throttle/static notches are avoided as brittle (props,
        -- altitude and payload change the throttle-to-RPM relation).
        if s.hz_mean then
            say(MAV_WARN, "RPM only %.0f%% fresh: fix ESC wiring/DMA; FFT for now", s.fresh * 100)
        end
        base_hz = bits(param:get("FFT_MINHZ"))
        put("INS_HNTCH_ENABLE", 1)
        put("INS_HNTCH_MODE", 4)
        put("INS_HNTCH_REF", 1)
        put("INS_HNTCH_FM_RAT", 1)
        put("FFT_NUM_FRAMES", 0)  -- motor noise moves with RPM: averaging frames only adds lag
        put("FFT_OPTIONS", bits(param:get("FFT_OPTIONS")) | 1)  -- window after the low-pass
        if s.fft_hz and s.thr and s.thr > 0 then
            -- full-throttle fundamental ~ hover * sqrt(1/thrust), plus 10% margin
            put("FFT_MAXHZ", math.min(math.floor(limit_hz), math.ceil(1.1 * s.fft_hz / math.sqrt(s.thr))))
        end
        resonance = nil  -- one FFT cannot track motors and a frame resonance at once
    else
        say(MAV_WARN, "no RPM: set SERVO_BLH_BDMASK for bidirectional DShot, or FFT_ENABLE=1")
        return nil
    end

    -- Harmonics the gyro sample rate cannot represent at hover would only cost CPU.
    local hover_hz = s.hz_mean or s.fft_hz or base_hz
    for k = 1, 16 do
        if hmncs & (1 << (k - 1)) ~= 0 and k * hover_hz > limit_hz then
            hmncs = hmncs & ~(1 << (k - 1))
            say(MAV_INFO, "dropping harmonic %d: above %.0fHz gyro limit", k, limit_hz)
        end
    end

    -- CPU budget: prefer one notch per motor, then fewer harmonics.
    -- Per-motor notches are narrow, so they also update at loop rate (OPTS bit 2) to keep up.
    local sources = 1
    if rpm_ok and #motors_out > 1 and #motors_out * popcount(hmncs) * mult * imus <= budget then
        sources = #motors_out
        opts = opts | 2 | 4
    else
        opts = opts & ~2
    end
    while sources * popcount(hmncs) * mult * imus > budget and popcount(hmncs) > 1 do
        local top = 15
        while hmncs & (1 << top) == 0 do
            top = top - 1
        end
        hmncs = hmncs & ~(1 << top)
        say(MAV_INFO, "notch budget: dropping harmonic %d", top + 1)
    end
    local used = sources * popcount(hmncs) * mult * imus
    if used > budget then
        say(MAV_WARN, "even one notch needs %d filters > AFT_NOTCH_MAX %d", used, budget)
        return nil
    end

    put("INS_HNTCH_FREQ", base_hz)
    -- per-motor notches add phase lag: ArduPilot docs start them at FREQ/BW = 4 instead of 2
    put("INS_HNTCH_BW", math.floor(base_hz / (sources > 1 and 4 or 2) + 0.5))
    put("INS_HNTCH_HMNCS", hmncs)
    put("INS_HNTCH_OPTS", opts)

    -- A persistent peak that is not a motor harmonic (frame resonance) earns a second bank: one
    -- FFT-driven notch, with the FFT bracketed around it and windowed after the RPM notches/low-pass.
    if resonance and param:get("INS_HNTC2_ENABLE") and used + imus <= budget then
        local low = math.floor(0.8 * resonance)
        put("FFT_MINHZ", low)
        put("FFT_MAXHZ", math.ceil(1.25 * resonance))
        put("FFT_OPTIONS", bits(param:get("FFT_OPTIONS")) | 1)
        put("INS_HNTC2_ENABLE", 1)
        put("INS_HNTC2_MODE", 4)
        put("INS_HNTC2_FREQ", low)
        put("INS_HNTC2_BW", math.floor(low / 2 + 0.5))
        put("INS_HNTC2_REF", 1)
        put("INS_HNTC2_FM_RAT", 1)
        put("INS_HNTC2_HMNCS", 1)
        put("INS_HNTC2_OPTS", 0)
        used = used + imus
    end
    say(MAV_INFO, "plan uses %d/%d notch filters on %d IMU(s)", used, budget, imus)
    return rec
end

local function valid_value(name, value)
    if not finite(value) then
        return false
    end
    if name:match("_FREQ$") or name:match("HZ$") then return value >= 10 and value <= 495 end
    if name == "INS_GYRO_FILTER" then return value >= 10 and value <= 300 end
    if name:match("_BW$") then return value >= 5 and value <= 250 end
    if name:match("_REF$") or name:match("_FM_RAT$") then return value > 0 and value <= 1 end
    return value >= 0
end

local function changed_entries(rec)
    local out = {}
    for _, e in ipairs(rec) do
        local current = param:get(e[1])
        if current == nil or bank_disabled(e[1]) or math.abs(current - e[2]) > 1e-3 then
            out[#out + 1] = e
        end
    end
    return out
end

----------------------------------------------------------------------------------------------------
-- Ground actions and stages
--
-- Each verified flight is the baseline for the next step, so the tune advances one step per hover:
--   1 notch:  tracking source, FREQ, BW and harmonics from the measurement
--   2 prune:  drop the highest harmonic while residual noise does not rise (unused notches only add lag)
--   3 narrow: cut notch BW by 20% while residual noise does not rise (narrower notches add less lag)
--   4 lpf:    raise INS_GYRO_FILTER by 25% toward its ceiling while noise rises at most AFT_LPF_RAT
----------------------------------------------------------------------------------------------------
local STAGE_NAME = { "notch", "prune", "narrow", "lpf" }
local STAGE_FLAG = { "notch_done", "prune_done", "bw_done", "lpf_done" }
local applied_this_boot = false
local reboot_at = nil

local function changed_this_boot()
    applied_this_boot = true
    reboot_at = nil
end

local function restore()
    -- reverse order so ENABLE parameters (backed up first) are restored last
    local ok = true
    for i = #session.bak, 1, -1 do
        local e = session.bak[i]
        if param:get(e[1]) ~= nil then
            ok = param:set_and_save(e[1], e[2]) and ok
        end
    end
    say(ok and MAV_NOTICE or MAV_ERR, "restored %d previous values%s", #session.bak, ok and "" or " (SOME FAILED)")
    session.bak, session.rec = {}, {}
    changed_this_boot()
end

local function apply()
    if (session.phase ~= "PROPOSED" and session.phase ~= "PENDING") or #session.rec == 0 then
        say(MAV_WARN, "nothing to apply (phase %s)", session.phase)
        return
    end
    for _, e in ipairs(session.rec) do
        if not valid_value(e[1], e[2]) then
            say(MAV_ERR, "refusing invalid %s=%.3f", e[1], e[2])
            return
        end
    end
    -- A disabled notch bank is enabled alone first; its other values are written after a reboot.
    local pending = false
    local to_set = {}
    for _, e in ipairs(session.rec) do
        local bank = bank_of(e[1])
        local hidden = bank and bank_disabled(e[1])
        pending = pending or hidden
        if not hidden or e[1] == bank .. "_ENABLE" then
            to_set[#to_set + 1] = e
        end
    end
    for _, e in ipairs(to_set) do
        local current = param:get(e[1])
        if current == nil then
            say(MAV_ERR, "parameter %s missing, nothing changed", e[1])
            return
        end
        if not has_backup(e[1]) then
            session.bak[#session.bak + 1] = { e[1], current }
        end
    end
    if not save_session() then
        say(MAV_ERR, "backup not saved, nothing changed")
        return
    end
    for _, e in ipairs(to_set) do
        if not param:set_and_save(e[1], e[2]) then
            say(MAV_ERR, "failed to set %s, restoring", e[1])
            restore()
            session.phase = "DONE"
            save_session()
            return
        end
    end
    changed_this_boot()
    session.phase = pending and "PENDING" or "VERIFY"
    save_session()
    if pending then
        say(MAV_NOTICE, "notch bank enabled: reboot to expose its settings")
    else
        say(MAV_NOTICE, "applied %d values: reboot, then hover again to verify", #to_set)
    end
end

-- Ceiling for the low-pass stage. Auto (0) keeps the cutoff at or below the lowest notched
-- motor frequency; the notches handle everything above it.
local function lpf_ceiling()
    local cap = AFT_LPF_MAX:get()
    if cap < 0 then
        return 0
    end
    if cap == 0 then
        return (param:get("INS_HNTCH_ENABLE") or 0) > 0 and (param:get("INS_HNTCH_FREQ") or 0) or 0
    end
    return cap
end

-- Next step after a clean flight: returns stage, changes; or nil when the tune is complete.
local function plan_next(s)
    local b = session.base
    if (b.notch_done or 0) < 1 then
        local rec = recommend(s)
        if not rec then
            return nil
        end
        local diff = changed_entries(rec)
        if #diff > 0 then
            return 1, diff
        end
        b.notch_done = 1
        say(MAV_INFO, "notches already match the recommendation")
    end
    local notch_on = (param:get("INS_HNTCH_ENABLE") or 0) > 0
    if notch_on and (b.prune_done or 0) < 1 then
        local h = bits(param:get("INS_HNTCH_HMNCS"))
        if popcount(h) > 1 then
            local top = 15
            while h & (1 << top) == 0 do
                top = top - 1
            end
            return 2, { { "INS_HNTCH_HMNCS", h & ~(1 << top) } }
        end
        b.prune_done = 1
    end
    if notch_on and (b.bw_done or 0) < 1 then
        local bw, freq = param:get("INS_HNTCH_BW") or 0, param:get("INS_HNTCH_FREQ") or 0
        local narrower = math.floor(bw * 0.8)
        -- floor at FREQ/8: narrower than that needs tracking precision the RPM/FFT data rarely has
        if narrower >= math.max(5, freq / 8) then
            return 3, { { "INS_HNTCH_BW", narrower } }
        end
        b.bw_done = 1
    end
    if (b.lpf_done or 0) < 1 then
        local cur = param:get("INS_GYRO_FILTER") or 0
        local higher = math.min(lpf_ceiling(), math.floor(cur * 1.25 + 0.5))
        if higher > cur then
            return 4, { { "INS_GYRO_FILTER", higher } }
        end
        b.lpf_done = 1
    end
    return nil
end

local function plan_and_start(s)
    local stage, rec = plan_next(s)
    if not stage then
        session.phase = "DONE"
        save_session()
        say(MAV_NOTICE, "DONE: notch mode %d freq %.0f bw %.0f hmncs %d opts %d, gyro LPF %.0f",
            bits(param:get("INS_HNTCH_MODE")), param:get("INS_HNTCH_FREQ") or 0, param:get("INS_HNTCH_BW") or 0,
            bits(param:get("INS_HNTCH_HMNCS")), bits(param:get("INS_HNTCH_OPTS")), param:get("INS_GYRO_FILTER") or 0)
        return
    end
    session.rec = rec
    session.base.stage = stage
    session.base.noise = s.noise
    session.base.temp = s.temp_max or -1000
    session.phase = "PROPOSED"
    if not save_session() then
        return
    end
    -- one wrapped message: a burst of separate lines overflows the GCS text queue
    local parts = {}
    for _, e in ipairs(rec) do
        parts[#parts + 1] = string.format("%s=%.3g", e[1]:gsub("^INS_", ""), e[2])
    end
    say(MAV_NOTICE, "%s step: %s", STAGE_NAME[stage], table.concat(parts, " "))
    if AFT_AUTO:get() >= 1 then
        apply()
    else
        say(MAV_NOTICE, "set AFT_ACTION=1 on the ground to apply")
    end
end

local function reset_session()
    session = { phase = "IDLE", base = {}, rec = {}, bak = {} }
    save_session()
end

local function handle_action()
    local action = bits(AFT_ACTION:get())
    if action == 0 then
        return
    end
    -- the GCS write was saved: clear the saved copy too or the action repeats after reboot
    AFT_ACTION:set_and_save(0)
    if arming:is_armed() then
        say(MAV_WARN, "actions are only accepted while disarmed")
        return
    end
    if action == 1 then
        apply()
    elseif action == 2 then
        if session.phase ~= "VERIFIED" then
            say(MAV_WARN, "nothing verified to accept (phase %s)", session.phase)
            return
        end
        session.bak, session.rec, session.phase = {}, {}, "IDLE"
        save_session()
        say(MAV_NOTICE, "accepted; the next hover plans the next step")
    elseif action == 3 then
        if #session.bak == 0 then
            say(MAV_WARN, "no backup to restore")
            return
        end
        restore()
        session.base[STAGE_FLAG[session.base.stage or 1]] = 1
        session.phase = "IDLE"
        save_session()
    elseif action == 4 then
        reset_session()
        say(MAV_NOTICE, "session reset")
    end
end

----------------------------------------------------------------------------------------------------
-- Flight end
----------------------------------------------------------------------------------------------------
local function report(s)
    say(MAV_INFO, "hover %.0fs noise %.2fdps vibe %.1f thr %.2f yaw %.2f", s.steady_s, s.noise or -1,
        s.vibe or -1, s.thr or -1, s.yaw or 0)
    if s.hz_mean then
        say(MAV_INFO, "motor %.0fHz (min %.0f) fresh %.0f%% imbalance %.0f%%", s.hz_mean, s.hz_min,
            s.fresh * 100, s.imbalance)
    end
    say(MAV_INFO, "output imbalance %.0f%% (motor %d)", s.out_imbalance or 0, s.out_motor or 0)
end

-- nil when the verification flight passes, otherwise why it failed
local function verify_failure(s, why)
    if why then
        return why
    end
    local stage = session.base.stage or 1
    local ratio = s.noise / session.base.noise
    local limit = stage == 4 and AFT_LPF_RAT:get() or AFT_VERIFY_RAT:get()
    if ratio > limit then
        return string.format("noise %.2f -> %.2f dps (x%.2f > %.2f)", session.base.noise, s.noise, ratio, limit)
    end
    local before = session.base.temp or -1000
    if before > -1000 and s.temp_max and s.temp_max > before + AFT_TEMP_RISE:get() then
        return string.format("motors %.0fC vs %.0fC before", s.temp_max, before)
    end
    return nil
end

local function finish_flight(verify_counts)
    local s = summarize()
    m = nil
    report(s)
    local why = refusal(s)
    local short = s.steady_s < AFT_HOVER_S:get() and not s.fault
    local phase = session.phase

    if phase == "DONE" then
        say(MAV_INFO, "tune complete; AFT_ACTION=4 starts a new one")
        return
    elseif phase == "VERIFIED" then
        say(MAV_INFO, "verified step pending: AFT_ACTION=2 accept, 3 restore")
        return
    elseif phase == "PROPOSED" or phase == "PENDING" then
        say(MAV_INFO, "previous step not applied yet")
        return
    elseif phase == "VERIFY" then
        if not verify_counts then
            say(MAV_WARN, "settings changed this boot: reboot before the verification flight")
            return
        end
        if short then
            say(MAV_WARN, "not verified: %s; hover again", why)
            return
        end
        local stage = session.base.stage or 1
        local failed = verify_failure(s, why)
        if failed then
            say(MAV_ERR, "%s step failed: %s", STAGE_NAME[stage], failed)
            restore()
            session.base[STAGE_FLAG[stage]] = 1
            -- a failed notch stage means the measurement cannot be trusted: stop rather than tune on top
            session.phase = stage == 1 and "DONE" or "IDLE"
            save_session()
            return
        end
        say(MAV_NOTICE, "%s step VERIFIED: noise %.2f -> %.2f dps", STAGE_NAME[stage], session.base.noise, s.noise)
        if stage == 1 then
            session.base.notch_done = 1
        end
        if AFT_AUTO:get() < 1 then
            session.phase = "VERIFIED"
            save_session()
            say(MAV_NOTICE, "AFT_ACTION=2 accept, 3 restore")
            return
        end
        session.bak = {}
        plan_and_start(s)  -- this flight is the next step's baseline
        return
    end
    if why then
        say(MAV_ERR, "REFUSED: %s", why)
        return
    end
    plan_and_start(s)
end

----------------------------------------------------------------------------------------------------
-- Main loop
----------------------------------------------------------------------------------------------------
local function motor_mask()
    local mask = 0
    for _, out in ipairs(motors_out) do
        mask = mask | (1 << out)
    end
    return mask
end

-- Setup checks from the usual filter-tuning workflow. Warnings only: logging and ESC wiring are the
-- operator's, not the tuner's.
local function preflight_report()
    motors_out = motor_outputs()
    local loop = param:get("SCHED_LOOP_RATE") or 0
    local gyro_hz = 1000 * 2 ^ bits(param:get("INS_GYRO_RATE"))
    say(MAV_INFO, "IMUs %d gyro %.0fHz loop %.0fHz motors %d phase %s", healthy_imus(), gyro_hz, loop,
        #motors_out, session.phase)
    local fft = param:get("FFT_ENABLE")
    say(MAV_INFO, "notch en %d mode %d freq %.0f hmncs %d opts %d, LPF %.0f, FFT %s",
        bits(param:get("INS_HNTCH_ENABLE")), bits(param:get("INS_HNTCH_MODE")), param:get("INS_HNTCH_FREQ") or 0,
        bits(param:get("INS_HNTCH_HMNCS")), bits(param:get("INS_HNTCH_OPTS")), param:get("INS_GYRO_FILTER") or 0,
        fft == nil and "not built" or (fft > 0 and "on" or "off"))
    if #motors_out == 0 then
        say(MAV_WARN, "no MotorN outputs found; RPM checks disabled")
    end
    local bd = param:get("SERVO_BLH_BDMASK")
    if bd and bd > 0 and (bits(bd) & motor_mask()) ~= motor_mask() then
        say(MAV_WARN, "SERVO_BLH_BDMASK %d misses motor outputs (%d)", bits(bd), motor_mask())
    end
    local poles = param:get("SERVO_BLH_POLES")
    if bd and bd > 0 and poles then
        say(MAV_INFO, "motor poles %d: count rotor magnets if RPM looks wrong", bits(poles))
    end
    if fft and fft > 0 then
        local window = param:get("FFT_WINDOW_SIZE") or 0
        if window > 0 and gyro_hz / window > 4 then
            say(MAV_WARN, "FFT resolution %.0fHz is coarse: raise FFT_WINDOW_SIZE", gyro_hz / window)
        end
        if bits(param:get("LOG_BITMASK")) & (1 << 19) ~= 0 then
            say(MAV_WARN, "raw IMU logging is on: it stops FFT sampling")
        end
    end
    if (param:get("INS_LOG_BAT_MASK") or 0) == 0 then
        say(MAV_INFO, "INS_LOG_BAT_MASK=1 INS_LOG_BAT_OPT=5 lets Filter Review check the result")
    end
end

local auth_id = arming:get_aux_auth_id()
local auth_blocked = nil

-- Arming stays blocked while new filters are saved but not loaded, so no flight runs on stale ones.
local function update_arming_block(block)
    if auth_id == nil or block == auth_blocked then
        return
    end
    auth_blocked = block
    if block then
        arming:set_aux_auth_failed(auth_id, "AFT: reboot to load new filters")
    else
        arming:set_aux_auth_passed(auth_id)
    end
end

-- Reboot only while disarmed and on the ground, after a 5 s warning. With AFT_AUTO=1, ask once.
local function auto_reboot(now)
    if applied_this_boot and AFT_AUTO:get() == 1 then
        if reboot_at == nil then
            reboot_at = math.huge
            say(MAV_NOTICE, "reboot to load the new filters (arming blocked until then)")
        end
        return
    end
    if AFT_AUTO:get() < 2 or not applied_this_boot or vehicle:get_likely_flying() then
        reboot_at = nil
        return
    end
    if reboot_at == nil then
        reboot_at = now + 5000
        say(MAV_NOTICE, "rebooting in 5 s to load the new filters (AFT_AUTO=2)")
    elseif now >= reboot_at and #outbox == 0 then  -- let queued messages reach the GCS first
        vehicle:reboot(false)
    end
end

local was_enabled = false
local was_armed = false
local verify_counts = true
local last_ms = millis():tofloat()

local function update()
    local now = millis():tofloat()
    local dt = (now - last_ms) / 1000
    last_ms = now
    flush_outbox(now)
    local enabled = AFT_ENABLE:get() > 0
    if enabled and not was_enabled then
        preflight_report()
    end
    was_enabled = enabled
    update_arming_block(enabled and applied_this_boot)
    if not enabled then
        m = nil
        return update, #outbox > 0 and 100 or 500
    end
    local armed = arming:is_armed()
    if armed and not was_armed then
        m = new_stats()
        verify_counts = not applied_this_boot
    elseif not armed and was_armed and m then
        finish_flight(verify_counts)
    end
    was_armed = armed
    if armed and m then
        sample(now, dt)
    else
        handle_action()
        -- A notch bank enabled before the last reboot now exposes its settings: finish applying,
        -- 5 s after boot so the GCS link is up to see the messages.
        if session.phase == "PENDING" and not applied_this_boot and AFT_AUTO:get() >= 1 and now > 5000 then
            apply()
        end
        auto_reboot(now)
    end
    return update, 7 + math.random(0, 6)
end

load_session()
return update, 1000
