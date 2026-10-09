# Development

Install Git, Pixi, CMake >=3.22.1 and a C++20 compiler. Linux requires OpenSSL Crypto
development files; Windows uses BCrypt. Mission Planner/router builds require Windows,
Visual Studio/MSBuild, .NET Framework 4.8 and the reviewed Mission Planner assemblies.

```sh
git submodule update --init --recursive
pixi install
pixi run build-core
pixi run test-core
pixi run build-qualification-cli
pixi run test-python
pixi run test-runtime-ipc
pixi run test-runtime-land
pixi run test-runtime-lifecycle
pixi run lint
pixi run format-check
pixi run docs-build
pixi run precommit
```

`pixi.toml` is the task source of truth. `build-core` builds the two production
executables; the direct qualification driver is explicitly selected and never installed.
Fake peers exercise real processes and transports without hardware. Run focused tests
while changing code, then the full checks. Engineering rules live only in [AGENTS](../AGENTS.md).

## MAVSDK transport and authority checks

```sh
pixi run verify-mavsdk-provenance
pixi run test-mavsdk-connectivity
pixi run test-mavsdk-transport-qualification
pixi run test-mavsdk-authority-wire
pixi run test-authority-sitl-harness
```

These cover selection/telemetry, command framing, freshness, cancellation at the wire,
connection lifetime and no replay. They do not establish physical control. Deterministic
fixtures may select a Release build via `NOMAD_QUALIFICATION_BUILD_DIR`.

## Mission Planner and standalone router

```sh
pixi run build-plugin-only
pixi run lint-plugin
pixi run test-plugin-build-only
pixi run test-plugin-config-validation
pixi run test-plugin-core-client
pixi run test-plugin-geometry
pixi run test-plugin-gimbal
pixi run test-plugin-gimbal-ui
pixi run test-plugin-audio
pixi run test-plugin-video
pixi run test-plugin-version
pixi run test-ground-router
```

Build/test commands do not deploy to Mission Planner. The plugin uses current JSON only;
unknown fields and invalid saved data fail explicitly. Log analysis uses Mission Planner's
native tools. The [router README](../infra/transport/ground_router/README.md) defines its
local protocol; [plugin packaging](../mission_planner/packaging/README.md) defines deployment.

## SITL qualification

Docker is required. The Copter image is `nomad-sitl:copter-4.7.1`; build it using the
instructions in [Compose](../docker/docker-compose.dev.yml). QuadPlane builds from
[Dockerfile.sitl-plane](../docker/Dockerfile.sitl-plane), pinned to ArduPlane 4.7.1.

```sh
pixi run build-sitl-tools
pixi run dev-up
pixi run core-sitl-status
pixi run quadplane-sitl-up
```

Run the full aircraft scenarios listed in [SITL notes](../tests/sitl/README.md).
[sitl.yml](../.github/workflows/sitl.yml) retains full Copter and QuadPlane chains for
safety-sensitive changes, scheduled and manual runs. Its scope gate fails closed for
unknown outputs. The separate MAVSDK smoke proves connectivity only. Do not run simulator
actuation against live endpoints. Neither simulator success nor an ACK qualifies hardware.

## Package and release checks

```sh
pixi run build-core-release
pixi run package-core
pixi run verify-core-package
pixi run verify-core-staged-install
pixi run test-runtime-service-artifacts
pixi run test-release-lifecycle
```

Builds write the build tree. Staged installation is restricted to `build/package/stage`;
`install-core` requires an explicit prefix. Source archives/fixture identities cannot
produce a production package. Release fixture A/B activation/rollback checks are in
[test.yml](../.github/workflows/test.yml). [Operations](operations.md) covers protected
configuration and explicit deployment.

Main protection is specified in [github-main-protection.json](../config/github-main-protection.json).
Do not weaken required safety checks or use admin bypass to clear a failing gate.
