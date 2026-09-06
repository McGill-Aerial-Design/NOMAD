# NOMAD Todo List

> Working ledger for in-flight migration work. Canonical product, safety, and
> migration decisions belong in `docs/` (`safety.md`, `migration.md`).

## Status key

- `[ ]` Open
- `[~]` In progress
- `[x]` Complete
- `[N/A]` Not applicable

## Current focus: GCS heartbeat for relay-gated UDP links (this session)

- [x] Root cause: ArduPilot/MAVProxy only streams a UDP leg after it sees a
  GCS heartbeat from that endpoint. The C++ core never sent one, so on
  heartbeat-gated relays (Docker Desktop's UDP relay; real mavlink-router
  setups behave the same) the leg stays mostly silent and `wait_for_heartbeat`
  times out.
- [x] `encode_gcs_heartbeat()` added to `src/mavlink/protocol.cpp` — standard
  MAVLink2 GCS heartbeat (sysid 255, compid 190, `MAV_TYPE_GCS`,
  `MAV_AUTOPILOT_INVALID`); a status frame, not a command.
- [x] `UdpMavlinkConnection` emits it at 1 Hz inside `wait_for_heartbeat` and
  `wait_for_state`, and latches the peer address from any valid decoded frame
  (not just heartbeats). Before the peer latches it announces to the configured
  endpoint (loopback fallback for a wildcard `0.0.0.0` host) or, when set, to
  `NOMAD_RELAY_ADDRESS` — the configurable announcement address for relays
  behind a separate gateway IP (Docker Desktop's UDP proxy, LTE routers).
  A malformed value fails `connect()` closed; the override stops applying
  once a vehicle latches as the peer (docs/operations.md).
- [x] Golden-frame unit test added: `tests/core_test.cpp::
  test_gcs_heartbeat_encoder_matches_mavlink_reference` pins the exact 21-byte
  wire frame (produced with pymavlink 2.4.49, ardupilotmega, v2 framing);
  `test_gcs_heartbeat_frame_round_trips_through_decoder` proves the decoder
  reads it back.
- [x] Loopback emission tests added (`tests/udp_connection_test.cpp`):
  `test_unlatched_connection_sends_gcs_heartbeats` proves an unlatched
  connection announces sysid 255 / compid 190 / MAV_TYPE_GCS heartbeats to its
  configured endpoint at ~1 Hz (and never faster);
  `test_relay_address_override_targets_prelatch_announcements` proves the
  `NOMAD_RELAY_ADDRESS` override carries the announcements to the relay and
  stops once the peer latches; `test_invalid_relay_address_fails_closed`
  proves connect() refuses a malformed override.
- [x] Test binaries converted to exception-based failure reporting so a failing
  check cannot open a Windows abort dialog that hangs unattended CI.
- [~] SITL evidence: scenario rewritten (`scripts/dev/core_sitl_gcs_heartbeat.py`,
  pixi task `core-sitl-gcs-heartbeat`, wired into CI sitl.yml) as a real
  heartbeat-gated relay with a negative control (announcements dropped →
  status must fail closed). `NOMAD_RELAY_ADDRESS` removed the topology blocker:
  a local Windows run now proves the mechanism end to end — announcements
  land on the relay at ~1 Hz with correct GCS bytes, the gate opens, the
  vehicle heartbeat (msgid 0, sysid 1) is forwarded and consumed, and the CLI
  progresses to the telemetry stage. The run still fails closed with an
  accurate diagnostic because the degraded Docker Desktop UDP proxy trickles
  (~2 datagrams/5s, confirmed also for the canonical `core-sitl-status`
  baseline, and unchanged after a SITL restart) and cannot sustain the
  position/GPS streams. Full-telemetry evidence must be recorded on Linux/CI;
  the mechanism itself is pinned by the unit/loopback tests.

## Flight-controller-generic naming (2026-09-05)

- [x] `CubeOutputController` renamed to `OutputController` (file moved to
  `mission_planner/src/Control/OutputController.cs`); all call sites and docs
  updated. The class drives generic ArduPilot DO_SET_SERVO/DO_SET_RELAY
  outputs on any board. Closed the class's `debt:` marker.
- [x] Direct-MAVLink fallbacks removed from the plugin's discrete output
  paths: servo, relay, relay fire, and motor-music user commands now go
  through the C++ core only and fail closed. `MavlinkSerialLock` extracted to
  `mission_planner/src/Connectivity/MavlinkSerialLock.cs` for the remaining
  high-rate direct streams (gimbal mount control, motor-music script install,
  EKF source switch). `tryOnly` flag removed everywhere.
- [x] Services-status wire key generalized: `no_cubepilot`/`cubepilot_present`
  -> `no_flight_controller`/`flight_controller_present` (edge_core route,
  plugin panel, tests).
- [x] Blocking service probes extracted to `edge_core/services/service_probes.py`;
  the status route passes the 40-line gate; per-app TTL cache on `app.state`.
- [x] Cube-specific wording generalized in docs, infra configs,
  `config/profiles/drone.env`, and the profile scripts. Deliberately kept:
  real USB by-id vendor strings in `scripts/services/mavlink_router.sh`
  (must literally match `/dev/serial/by-id` for any-board discovery),
  historical CHANGELOG entries, and third_party.

## C++ core link reliability (earlier sessions)

- [x] UDP receive path decodes every MAVLink frame per datagram (MAVProxy
  coalesces frames; the old decoder dropped ACKs in the second-or-later
  frame). `decode_datagram` in `src/mavlink/protocol.cpp`; pinned by loopback
  regression tests in `tests/udp_connection_test.cpp`.
- [x] Command path no longer requests telemetry streams (the burst made the
  Docker relay drop the ACK); `status` keeps full telemetry from the
  MAVProxy default stream.
- [x] SITL scenario scripts hardened: bounded CLI retries with authoritative
  status polling, no post-landing altitude check, velocity-watchdog arms
  itself, `run_cli` deduplicated.

## SITL evidence (Docker Desktop relay, udpin:0.0.0.0:14570; Copter 4.7.1 image)

All recorded 2026-09-04/05 against `nomad-sitl:copter-4.7.1`; re-verified
after the generated-codec switch:

- [x] `pixi run core-sitl-status`
- [x] `pixi run core-sitl-command-flow` (mode, arm, takeoff 5 m, goto
  verified, RTL, land, disarm — authoritatively verified)
- [x] `pixi run core-sitl-mission` (`completed_steps == 6`)
- [x] `pixi run core-sitl-velocity-watchdog` (stops with `command_timeout`)
- [x] `pixi run core-sitl-geofence` (mission-fence upload + readback verified;
  SR-FEN-01; codec pinned by pymavlink golden-frame tests)
- [x] `pixi run core-sitl-payload` (relay-on, bounded pulse, relay-off)
- [x] `pixi run core-sitl-link-loss` (runner stops safely; transport-level
  zero-delivery proof remains a separate gate)
- [x] `pixi run core-sitl-link-recovery` (works without special stack env; the
  stack always forwards a MAVLink copy to host UDP 14572)

## MAVLink codec generated from the pinned submodule (2026-09-05)

- [x] `third_party/ardupilot-mavlink` submodule pinned to the exact commit
  Copter-4.7.1 compiles against; `scripts/dev/generate_mavlink.py` runs the
  submodule's own mavgen into gitignored `build/generated/mavlink`; wired
  into CMake, `pixi run generate-mavlink`, CI submodules checkout, and
  `Dockerfile.sim-ros`.
- [x] `protocol.cpp` and `fence.cpp` ported onto the generated headers;
  golden wire frames byte-identical; latent `FENCE_STATUS` decode-layout fix.
- [x] Full SITL battery re-passed on the generated codec.

## Cutover progress (transitional Python vehicle-control path)

- [x] `edge_core/ros_http_bridge/` deleted (gate 1); C# REST callers moved to
  the core client (gate 2).
- [x] `edge_core/safety/` Python package deleted; requirements owned by
  `src/safety/*` with the `cpp_traceability` block in `docs/safety.md` and
  C++ SITL coverage; Python-only gates removed from pixi/CI/mypy.
- [x] `edge_core/modules/payload/servo.py` and
  `edge_core/services/payload_module.py` deleted (inventory callers gone).
- [x] Confirmed no remaining Python MAVLink vehicle-control callers (2026-09-05):
  `edge_core` has no MAVLink command/telemetry code left (services probes only);
  the C# plugin's remaining `/api/*` calls are video, VIO, isaac, terminal,
  and ZED camera tools (device-side, not vehicle control).
- [~] Remaining deletions are blocked on open client gates (see PLAN.md):
  camera-tilt/spray routes await the Jetson camera-stack decision; the module
  registry still wires camera/VIO/isaac/services routes used by the plugin;
  deleting them requires moving those status/health surfaces first.
- [x] `sitl-scenario` CI task retargeted: the scenario drives the C++ CLI
  (`velocity` verb) for motion and the Python link is observer-only
  (docs/migration.md Phase 7 ledger); task kept as loop-closure evidence.
- [x] New `core-sitl-gcs-heartbeat` pixi task added for the relay-gate
  scenario (see GCS heartbeat section above).

## Validation (latest full battery, 2026-09-05)

- [x] `pixi run build-core` / `pixi run test-core` — 7/7 pass (new
  `nomad_codec_golden_tests` suite; GCS-heartbeat golden-frame, decoder
  round-trip, and pre-latch emission tests; exception-based failure reporting)
- [x] Pre-commit unblocked for all-files runs: `line_report.py` now excludes
  vendored `third_party` submodules and baselines legacy oversized Python
  functions in `config/function_size_baseline.txt` (same mechanism as the
  file-size baseline); the new oversized duallink test partials were split
  instead (`.Harness.cs`/`.Stress.cs`, build script updated) and
  `test-plugin-duallink` passes
- [x] Pre-commit unblocked on commit shells without `python` on PATH:
  the `complexity-check` hook is `language: system`, so its `python` entry
  failed with "Executable `python` not found" under pixi-managed
  environments; converted to `language: python` like every other hook so
  pre-commit runs it in its own venv
- [x] `pixi run pytest` — 451 passed, 2 skipped
- [x] `pixi run lint` / `pixi run format-check` / `pixi run complexity-check`
- [x] `pixi run lint-plugin`; `test-plugin-core-client/-interlock/-duallink`
- [x] `pixi run docs-build` — passed strict (rerun after SR-LNK-04 doc update)
- [ ] SITL battery (`core-sitl-*` incl. the new `core-sitl-gcs-heartbeat`) —
  full-telemetry re-record on Linux/CI; local Windows runs are degraded by the
  Docker Desktop UDP proxy (see GCS heartbeat section above; other scenarios
  were green in the 2026-09-04/05 battery and are unaffected by this change
  set)

## Remaining migration work

- [~] Safety evidence: remaining items are transport-level zero-delivery
  proof, hardware payload/VIO evidence, and the GCS-heartbeat SITL evidence
  above.
- [x] ROS 2 adapter package (`ros2/nomad_ros`): compiles and passes
  translation + integration tests in the Humble image; live-vehicle
  validated (2026-09-04).
- [ ] Mission Planner: remaining vehicle operations behind the core client
  boundary; authenticated remote command handling; audit logging.
- [ ] Cutover completion (see deletion progress above).
- [ ] Hardware readiness review before any real flight.
