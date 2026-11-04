# Auto Filter Tune (ArduPilot Lua prototype)

Standalone ArduPilot Copter script, not part of NOMAD. It tunes the gyro filters from what the
aircraft reports during normal hovers. It sets the harmonic-notch source, frequency and harmonics,
then trims unneeded harmonics, narrows the notches and raises the gyro low-pass. Each step is
verified on the next hover and restored if it makes things worse.

The script runs alongside any flight mode. It never commands the vehicle and never changes
filters while armed. Every change is made on the ground, after the previous values are backed up.

## Use

1. Copy `auto_filter_tune.lua` to `APM/scripts/`, set `SCR_ENABLE=1` and reboot.
2. Set `AFT_ENABLE=1`. Optionally set `AFT_BLADES` to 2 or 3.
3. Hover gently for at least 30 s in any mode, then land and disarm. Repeat until the GCS shows `DONE`.

The default is `AFT_AUTO=2`. After each flight the script checks the data, applies the next
step and reboots once the vehicle has been disarmed on the ground for 5 s. Until that reboot,
arming is blocked, so no flight runs on half-loaded filters.

- `AFT_AUTO=1` is the same, but you do the reboot.
- `AFT_AUTO=0` waits for `AFT_ACTION` after every flight: 1 applies, 2 accepts, 3 restores.
- `AFT_ACTION=3` restores the last step, and `AFT_ACTION=4` starts over, in any mode.

The session is stored in `aft_session.txt`: the phase, stage flags, baseline, pending change and backup.

## Stages (one step per hover)

Each verified flight is also the baseline for the next step.

| Stage | Step | Kept when |
|---|---|---|
| 1 notch | Choose the tracking source, FREQ, BW, harmonics and second bank from the measurement | Residual noise ≤ `AFT_VERIFY_RAT` × baseline |
| 2 prune | Drop the highest harmonic (unused notches only add delay) | Noise does not rise; a failure restores that harmonic and ends the stage |
| 3 narrow | Cut BW by 20 %, down to FREQ/8 (narrower notches add less delay) | Noise does not rise |
| 4 lpf | Raise `INS_GYRO_FILTER` by 25 % up to `AFT_LPF_MAX` | Noise ≤ `AFT_LPF_RAT` × baseline |

In every stage, verification also fails on any refusal gate below, or on a motor temperature rise
above `AFT_TEMP_RISE`. A failed notch stage stops the tune, because its measurement cannot be trusted.
`AFT_LPF_MAX=0` (auto) caps the low-pass at the notch FREQ; set a value in Hz to go higher, or -1 to skip the stage.

## Notch rules

These follow the ArduPilot ESC-notch documentation and Chris Rosser's filter tuning guide.

- **RPM on every motor** (≥ `AFT_RPM_OK` of samples): `MODE=3`, `REF=1`.
  `FREQ` is 0.8 × the lowest hover motor frequency, but never below `INS_GYRO_FILTER`.
  If the notch budget allows, it uses one notch per motor with loop-rate updates (`OPTS` 2+4) and `BW = FREQ/4`;
  otherwise `BW = FREQ/2`.
- **No usable RPM, FFT on:** FFT-driven `MODE=4` with `FREQ=FFT_MINHZ`, `REF=1`, `FM_RAT=1`, `FFT_NUM_FRAMES=0`,
  a post-filter window (`FFT_OPTIONS` bit 0) and `FFT_MAXHZ` from the hover peak. Throttle and static notches
  are never proposed, because prop, altitude or payload changes break the throttle-to-RPM relation.
  Intermittent RPM is reported as an ESC wiring or DMA problem.
- **Harmonics:** a fresh bank starts on the 1st–3rd harmonics (1st+2nd with `AFT_BLADES=2`). The FFT's dominant
  hover peak adds its harmonic. Harmonics above 0.45 × the gyro rate at hover are dropped, and the prune stage
  removes harmonics that carry no noise.
- **Frame resonance** (FFT peak that is not a motor harmonic, with RPM tracking on bank 1): bank 2 is FFT-driven
  (`MODE=4`, one harmonic, `OPTS=0`). The FFT is bracketed at 0.8–1.25 × the peak and windowed after the filters.
- **CPU:** the filter count is sources × harmonics × multiplicity × IMUs, and it must stay within `AFT_NOTCH_MAX`.
  The script drops per-motor notches first, then the highest harmonics.

## Refusal gates (no change is made)

- Less than `AFT_HOVER_S` of steady, level hover. Climbs and descents faster than 0.5 m/s do not count.
- A motor stopped for more than 300 ms in lifting flight, or a persistent gyro health failure.
- Vibration above `AFT_VIBE_MAX`.
- Yaw output bias above `AFT_YAW_MAX` (motor tilt, direction or thrust).
- Mean hover body rate above `AFT_OSC_DPS` (oscillation: tune PIDs first).
- One motor's output command `AFT_IMB_PCT` off the mean (weak motor, ESC or prop). The rate loop hides this
  in RPM, so the output command is checked instead.
- RPM imbalance above `AFT_IMB_PCT`.
- RPM spikes on one ESC in more than 0.5 % of samples (typically a timer/DMA conflict: move that ESC to another output).

The preflight messages also warn about:

- `SERVO_BLH_BDMASK` missing motor outputs, and they show the motor pole count.
- RAW IMU logging, which stops FFT sampling.
- Coarse FFT resolution (more than 4 Hz per bin: raise `FFT_WINDOW_SIZE`).
- Batch logging being off (`INS_LOG_BAT_MASK=1`, `INS_LOG_BAT_OPT=5` are needed to review the result in Filter Review).

## What it measures (existing Lua bindings only)

| Data | Source |
|---|---|
| Per-motor RPM, freshness, spikes, temperature | `esc_telem` |
| Residual post-filter gyro noise | `ins:get_gyro` first differences at a jittered ~100 Hz (a variance estimate, so no Nyquist limit; jitter stops tones locked to the script rate from aliasing to DC) |
| Dominant raw-spectrum peak | Native FFT's live `FFT_FREQ_HOVER` |
| Output command per motor | `SRV_Channels:get_output_pwm_chan` |
| Yaw output, throttle, spool state, climb rate, vibration, gyro health | `motors`, `ahrs`, `ins` |

## SITL check

`sitl_test.py` runs the real `arducopter` SITL binary. The SITL models RPM-scaled per-motor vibration
with harmonics, ESC RPM telemetry, motor faults and fixed resonances.

```bash
python3 tools/ardupilot/auto_filter_tune/sitl_test.py --ardupilot ~/ardupilot
```

| Scenario | Expectation |
|---|---|
| `auto_full` | Hover only until DONE: ESC per-motor notches, pruning keeps exactly SITL's 1st+2nd harmonics, low-pass raised |
| `manual_mode_waits` | `AFT_AUTO=0`: proposal only, filters unchanged |
| `esc_fft_harmonic` | FFT peak classified as motor harmonic 1 |
| `frame_resonance` | 120 Hz tone: FFT-driven second bank proposed |
| `motor_imbalance` | Motor 1 at 80 % PWM: refused, filters unchanged |
| `verify_fail_restores` | 45 Hz resonance added before verification: notch step fails, backup restored |
| `no_esc_uses_fft` | No ESC telemetry: FFT-driven notch |
| `short_hover_refused` | Too little steady hover: refused |

The scenarios run with `AFT_AUTO=1`. SITL hangs when a script calls `vehicle:reboot()`, because its
reboot is an `execv` from the calling thread. So the harness performs the reboot the script requests over MAVLink.
The one automatic-reboot call (`AFT_AUTO=2`) is therefore not exercised in SITL.

The harness force-disarms only when SITL's ground contact tips the airframe on touchdown and the land
detector never completes, and it prints when it does. Run directories stay in `~/aft-sitl-runs/`.
SITL is hardware-free and does not qualify behaviour on a real aircraft.

## Limits

- Lua cannot read the gyro spectrum, the FFT peak list, scheduler load or loop overruns.
  Steps are judged by the total residual noise, not by per-peak dB. The guide's target of post-filter peaks
  below about −50 dB still needs Filter Review. CPU is guarded only by the `AFT_NOTCH_MAX` filter count.
- Total residual noise can hide a small peak that grows while the rest shrinks. Review the final log in Filter Review.
- The residual-noise estimate repeats only to within about ±5 % between identical hovers. Steps judged
  against `AFT_VERIFY_RAT=1.05` that sit at the margin can pass on one flight and fail on the next.
  In SITL, the last BW narrowing did both. Raise `AFT_HOVER_S` for steadier numbers.
- A frame resonance is found only when it is the FFT's dominant hover peak. Lua sees one weighted
  frequency (`FFT_FREQ_HOVER`), not the FFT's peak list, so a resonance under a stronger motor peak is invisible.
- `param:set_and_save` does not store a value equal to its default, so such a value follows later default
  changes (ArduPilot PR #33879).
- Pole count, prop size and motor condition cannot be detected. A wrong pole count makes the RPM frequency wrong;
  the FFT/RPM ratio message is the hint.
