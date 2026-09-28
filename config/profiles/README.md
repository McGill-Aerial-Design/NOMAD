# Deployment profile templates

Product profiles: onboard_companion, groundstation_gpu and groundstation_minimal.
The profile loader and deterministic configuration tests exist; runtime capability
and hardware qualification remain open.

Use profile-list to inspect names. profile-load writes ignored config/nomad.env
and synchronizes Mission Planner configuration; it is a state-changing action.
Do not load a profile merely to inspect it.

The three product templates now validate canonical endpoints and remove retired
profile-owned settings; optional workloads remain disabled until qualified.
Perception/VIO/autostart flags are not proof that a workload runs;
the minimal flag does not bypass the C++ VIO gate. See the canonical
[operations](../../docs/operations.md) and [migration](../../docs/migration.md)
for G1/G3 repairs and acceptance. Do not create another setup plan here.
