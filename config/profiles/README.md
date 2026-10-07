# Deployment profile templates

Product profiles: onboard_companion, groundstation_gpu and groundstation_minimal.
These files are deployment templates. Deterministic tests check configuration
shape; they do not prove runtime startup, hardware readiness, or flight capability.

Use profile-list to inspect names. profile-load writes ignored config/nomad.env
and synchronizes Mission Planner configuration; it is a state-changing action.
Do not load a profile merely to inspect it.

Profiles own identity, compute placement, service autostarts, runtime MAVLink and
video endpoints, simulation/inhibition flags, fences and velocity limits.
Loading retains reviewed deployment-local keys from the active environment,
including credentials, runtime paths, IPC ports, GCS addresses and device/service
configuration. Missing local keys use `nomad.env.example` defaults; required
credentials and audit paths still need provisioning before runtime startup.
Unknown and retired keys are discarded. Saving writes only profile-owned keys.

The installed CLI/runtime IPC v1 exposes no mission, navigation, geofence, or
payload request path. Perception/VIO/autostart flags do not prove that a workload
runs. `NOMAD_AUTOSTART_MAVLINK_ROUTER` refers to the optional aircraft-side
router service, not the standalone ground router or `nomad-runtime`.
Ground-station profiles leave that aircraft service disabled; the onboard
profile enables it deliberately. The ground router uses its own JSON configuration.
See the
canonical [operations](../../docs/operations.md),
[qualification status](../../docs/qualification.md), and
[development workflow](../../docs/development.md).
