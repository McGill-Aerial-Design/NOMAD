# Auto Filter Tune (ArduPilot Lua)

Standalone ArduPilot script, not part of NOMAD. It tunes the gyro filters from what the
aircraft reports during normal hovers. It sets the harmonic-notch source, frequency and harmonics,
raises the gyro low-pass and then trims unneeded harmonics. Each step is verified on the next hover
and restored if it makes things worse. It runs on ArduCopter and on ArduPlane quadplanes (VTOL hover).

The script runs alongside any flight mode. It never commands the vehicle and never changes
filters while armed. Every change is made on the ground, after the previous values are backed up.

## Use

1. Copy `auto_filter_tune.lua` to `APM/scripts/`, set `SCR_ENABLE=1` and reboot.
2. Set `AFT_ENABLE=1`. Optionally set `AFT_BLADES` to 2 or 3.
3. Hover gently for at least 30 s in any mode (Q modes on a quadplane), then land and disarm. Repeat until
   the GCS shows `DONE`, typically 3–5 hovers.

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
| 2 lpf | Raise `INS_GYRO_FILTER` straight to its ceiling; after one failure, try the geometric midpoint once | Noise ≤ `AFT_LPF_RAT` × the untuned first flight (a fixed anchor, so small steps cannot ratchet noise up) |
| 3 prune | Drop each harmonic, top down; the last one by disabling the bank (unused notches cost CPU and delay) | Noise does not rise; a failure restores that harmonic and tries the next |

In every stage, verification also fails on any refusal gate below, or on a motor temperature rise
above `AFT_TEMP_RISE`. A failed notch stage stops the tune, because its measurement cannot be trusted.
After any other failure the next step is planned at once against the same baseline, without another hover.
`AFT_LPF_MAX=0` (auto) caps the low-pass at the notch FREQ; set a value in Hz to go higher, or -1 to skip the stage.

The order matters. The low-pass is most of the delay the gyro filters add (40 → 150 Hz is about 15° less lag
at 10 Hz; halving a notch's width saves about 1–2°), and pruning before it is raised judges harmonics that the
low cutoff is still hiding. Notch width is therefore not stepped: narrowing changes the residual less than the
measurement repeats.

## Notch rules

These follow the ArduPilot ESC-notch documentation and Chris Rosser's filter tuning guide.

- **RPM on every motor** (≥ `AFT_RPM_OK` of samples): `MODE=3`, `REF=1`.
  `FREQ` is 0.8 × the slowest motor's mean hover frequency, but never below `INS_GYRO_FILTER`.
  If the notch budget allows, it uses one notch per motor (`OPTS` 2) and `BW = FREQ/4`; otherwise `BW = FREQ/2`.
  Loop-rate updates (`OPTS` 4) are left to the operator: ArduPilot recommends them only on H7-class CPUs.
- **No usable RPM, FFT on:** FFT-driven `MODE=4` with `FREQ=FFT_MINHZ`, `REF=1`, `FM_RAT=1`, `FFT_NUM_FRAMES=0`,
  a post-filter window (`FFT_OPTIONS` bit 0) and `FFT_MAXHZ` from the hover peak. Throttle and static notches
  are never proposed, because prop, altitude or payload changes break the throttle-to-RPM relation.
  Intermittent RPM is reported as an ESC wiring or DMA problem.
- **Harmonics:** a fresh bank starts on the 1st–3rd harmonics (1st+2nd with `AFT_BLADES=2`). The FFT's dominant
  hover peak adds its harmonic. Harmonics above 0.45 × the gyro rate at hover are dropped, and the prune stage
  removes harmonics that carry no noise.
- **Frame resonance** (FFT peak below the motor fundamental or above its 4th harmonic, with RPM tracking on
  bank 1): bank 2 is FFT-driven (`MODE=4`, one harmonic, `OPTS=0`). The FFT is bracketed at 0.8–1.25 × the peak
  and windowed after the filters. `FFT_FREQ_HOVER` averages the roll and pitch peaks by energy, so a peak between
  two motor harmonics is usually the two axes peaking on different harmonics: both harmonics are notched instead.
- **CPU:** the filter count is sources × harmonics × multiplicity × notched IMUs, and it must stay within
  `AFT_NOTCH_MAX` (default 16; about 8 on F4). Notches run on the primary IMU only unless `OPTS` bit 3
  (EnableOnAllIMUs) is set. The script drops per-motor notches first, then the highest harmonics.
- **Fast gyro rate:** each notch runs once per gyro sample, so `AFT_NOTCH_MAX` counts filters at 1 kHz and the
  budget shrinks in proportion at a faster `INS_GYRO_RATE` (16 at 1 kHz is 4 at 4 kHz). The rate counts only when
  IMU1 is in `INS_FAST_SAMPLE`. On an H7 running 2–4 kHz, raise `AFT_NOTCH_MAX` (e.g. 48) to keep per-motor notches.
  The fast rate thread (`FSTRATE_ENABLE`) does not change the measurement: the script reads the same filtered gyro
  the rate loop uses. Faster rates also keep more harmonics, since the limit is 0.45 × the gyro rate.
- **Only what is needed:** ArduPilot's guidance is to "enable only the number of harmonic notch filters actually
  required". The notch exists to let the low-pass sit higher with motor noise removed, so its value is judged at
  the raised low-pass: the prune stage removes every harmonic, and finally the whole bank, whose removal does not
  raise the residual. A frame resonance gets the second bank only when the FFT shows it.

## Refusal gates (no change is made)

- Less than `AFT_HOVER_S` of steady, level hover. Climbs and descents faster than 0.5 m/s do not count, nor does
  a quadplane's forward flight or transition (only Q modes are sampled).
- A motor stopped for more than 300 ms in lifting flight, or a persistent gyro health failure.
- Vibration above `AFT_VIBE_MAX`.
- Yaw output bias above `AFT_YAW_MAX` (motor tilt, direction or thrust).
- Mean hover body rate above `AFT_OSC_DPS` (oscillation: tune PIDs first). The rate comes from the attitude,
  not the gyro, so motor vibration is not mistaken for oscillation.
- One motor's output command `AFT_IMB_PCT` off the mean (weak motor, ESC or prop). The rate loop hides this
  in RPM, so the output command is checked instead.
- RPM imbalance above `AFT_IMB_PCT`.
- RPM spikes on one ESC in more than 0.5 % of samples (typically a timer/DMA conflict: move that ESC to another output).

The yaw-bias and both imbalance gates are skipped on tricopters (`FRAME_CLASS` or `Q_FRAME_CLASS` 7): the rear
motor carries a different load and yaw is vectored, so neither indicates a fault there.

The preflight messages also warn about:

- `SERVO_BLH_BDMASK` missing motor outputs, and they show the motor pole count.
- RAW IMU logging, which stops FFT sampling.
- Coarse FFT resolution (more than 4 Hz per bin: raise `FFT_WINDOW_SIZE`).
- Batch logging being off (`INS_LOG_BAT_MASK=1`, `INS_LOG_BAT_OPT=5` are needed to review the result in Filter Review).

## What it measures (existing Lua bindings only)

| Data | Source |
|---|---|
| Per-motor RPM, freshness, spikes, temperature | `esc_telem` |
| Residual post-filter gyro noise | `ins:get_gyro` first differences at a jittered ~100 Hz (a variance estimate, so no Nyquist limit; jitter stops tones locked to the script rate from aliasing to DC), as the median of ~5 s windows |
| Dominant raw-spectrum peak | Native FFT's live `FFT_FREQ_HOVER` |
| Output command per motor | `SRV_Channels:get_output_pwm_chan` |
| Yaw output, throttle, spool state, climb rate, vibration, gyro health | `motors`, `ahrs`, `ins` |

## First tests on a real aircraft

The script has been checked only in SITL, on ArduCopter master (4.7-dev) and ArduPlane 4.7.2; older firmware is
untested. Before the first real hover:

- The aircraft must already fly well on its current filters and PIDs. The script refuses oscillating hovers;
  it does not tune PIDs.
- Use bidirectional DShot (`SERVO_BLH_BDMASK` on every motor, correct `SERVO_BLH_POLES`) for RPM tracking,
  or `FFT_ENABLE=1` without it. The board needs Lua scripting; raise `SCR_HEAP_SIZE` if the script fails to load.
- Turn on batch logging (`INS_LOG_BAT_MASK=1`, `INS_LOG_BAT_OPT=5`) so each step can be checked in Filter Review.
- Hover in AltHold or Loiter (QHover or QLoiter on a quadplane) a few metres up, in calm air and an open area.
  Use `AFT_AUTO=1` for the first aircraft: you choose when each step loads by rebooting. The default
  `AFT_AUTO=2` reboots by itself, and that call cannot run in SITL, so it is untested.
- After each flight, feel the motors. Land at once if the aircraft sounds or flies worse, then set
  `AFT_ACTION=3` on the ground to restore the previous filters: a hover shorter than `AFT_HOVER_S` is not judged,
  so it does not restore them by itself.
- After `DONE`, review the last log in Filter Review before trusting the result for normal flying.

## SITL check

`sitl_test.py` runs the real `arducopter` SITL binary. The SITL models RPM-scaled per-motor vibration
with harmonics, ESC RPM telemetry, motor faults and fixed resonances.

```bash
python3 tools/ardupilot/auto_filter_tune/sitl_test.py --ardupilot ~/ardupilot
```

| Scenario | Expectation |
|---|---|
| `auto_full` | Hover only until DONE (≤ 8 flights): ESC per-motor notches, FREQ from mean hover RPM, low-pass raised to FREQ, pruning keeps exactly SITL's 1st+2nd harmonics |
| `manual_mode_waits` | `AFT_AUTO=0`: proposal only, filters unchanged |
| `esc_fft_harmonic` | FFT peak matched to the motor harmonics (SITL's equal harmonics can put it between two), no second bank |
| `frame_resonance` | 120 Hz tone: FFT-driven second bank proposed |
| `motor_imbalance` | Motor 1 at 80 % PWM: refused, filters unchanged |
| `verify_fail_restores` | 45 Hz resonance added before verification: notch step fails, backup restored |
| `no_esc_uses_fft` | No ESC telemetry: FFT-driven notch |
| `short_hover_refused` | Too little steady hover: refused |
| `fast_gyro_rate` | `INS_GYRO_RATE=2` with the fast rate thread: notch budget scaled to 4 at 4 kHz, no per-motor notches (plan only: stock SITL samples at 1 kHz) |

The scenarios run with `AFT_AUTO=1`. SITL hangs when a script calls `vehicle:reboot()`, because its
reboot is an `execv` from the calling thread. So the harness performs the reboot the script requests over MAVLink.
The one automatic-reboot call (`AFT_AUTO=2`) is therefore not exercised in SITL.

The harness force-disarms only when SITL's ground contact tips the airframe on touchdown and the land
detector never completes, and it prints when it does. Run directories stay in `~/aft-sitl-runs/`.
SITL is hardware-free and does not qualify behaviour on a real aircraft.

### Airframe sweep

`sweep.py` runs the tune hands-off on several airframes in parallel SITL instances and grades the result
independently of the script's own noise estimate. It compares batch-logged raw and filtered gyro spectra
(residual above 0.6 × the motor fundamental) from the first flight against a flight on the final filters,
and computes the filter chain's phase lag at 10 Hz from the final parameters.

```bash
python3 tools/ardupilot/auto_filter_tune/sweep.py --ardupilot ~/ardupilot \n    --plane-ardupilot ~/nomad-simulation/sources/ardupilot --out local/aft-sweep/run.json
```

| Case | Airframe | Hover fundamental |
|---|---|---|
| `fpv5_3blade` | SITL `freestyle` 5-inch X, 3-blade harmonics | ~225 Hz |
| `quad10_2blade` | SITL default quad | ~122 Hz |
| `hexa13_2blade` | SITL hexa | ~106 Hz |
| `octa_large` | SITL octa at large-prop motor speeds | ~48 Hz |
| `tilttri_plane` | ArduPlane `quadplane-tilttrivec` (vectored-yaw tilt-tricopter), FFT path | ~100 Hz |

Each runs calm and in gusts (6 m/s, turbulence 4). The tilt-tricopter uses ArduPlane's built-in physics:
the NOMAD Gazebo fixture reaches ArduPilot through the JSON backend, which carries no motor RPM, so it gets
neither RPM-scaled vibration nor ESC telemetry. SITL also numbers ESC telemetry by motor slot rather than
output, so the tri's rear motor never reports RPM there; that case exercises the FFT path.

The `quad10_2blade_4khz*` cases set `INS_GYRO_RATE=2`, `FSTRATE_ENABLE=3` and `SIM_RATE_HZ=4000` at 2× speed
(`_h7` also raises `AFT_NOTCH_MAX` to 48). Stock SITL
registers its IMU at 1 kHz whatever `INS_GYRO_RATE` says, so they need a build of ArduPilot PR #34642
(the SITL gyro rate follows `INS_GYRO_RATE`), passed with `--ardupilot`.

## Limits

- Lua cannot read the gyro spectrum, the FFT peak list, scheduler load or loop overruns.
  Steps are judged by the total residual noise, not by per-peak dB. The guide's target of post-filter peaks
  below about −50 dB still needs Filter Review. CPU is guarded only by the `AFT_NOTCH_MAX` filter count.
- Total residual noise can hide a small peak that grows while the rest shrinks. Review the final log in Filter Review.
- The residual-noise estimate does not repeat exactly between identical hovers (in SITL, single flights of the
  same filters differed by up to ±25 % before the median-of-windows estimate). A prune step at the
  `AFT_VERIFY_RAT=1.05` margin can therefore keep a harmonic it could have dropped; that errs towards more
  filtering, never less. Raise `AFT_HOVER_S` for steadier numbers.
- A frame resonance is found only when it is the FFT's dominant hover peak and lies outside the motor harmonics.
  Lua sees one weighted frequency (`FFT_FREQ_HOVER`), not the FFT's peak list, so a resonance under a stronger
  motor peak, or between two motor harmonics, is not given its own notch.
- `param:set_and_save` does not store a value equal to its default, so such a value follows later default
  changes (ArduPilot PR #33879).
- Pole count, prop size and motor condition cannot be detected. A wrong pole count makes the RPM frequency wrong;
  the FFT/RPM ratio message is the hint.
