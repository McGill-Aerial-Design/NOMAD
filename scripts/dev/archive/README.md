# Archived dev scripts

These are **one-off, unsupported** hardware bring-up and debugging scripts kept
for reference only. They were written against a specific Jetson + ZED + Isaac ROS
setup during development and are **not** part of the NOMAD baseline:

- `check_*` / `diag_zed_*` — ZED camera, USB, and ROS topic probes.
- `test_zed_*` / `test_usb*` / `test_egl*` / `test_*` — ad-hoc hardware tests.
- `spy_*` / `probe_mesh_*` / `inspect_mesh.py` / `analyze_voxels.py` /
  `diagnose_mesh.py` — nvblox/voxel mesh inspection.
- `compare_*` / `odom_diag.py` / `zed_pose_stability_check.py` — VIO/odometry
  comparisons.
- `patch_depth_conf*.py` / `find_mounts.py` — environment patches.

They are not linted to baseline standards, may reference paths/services that no
longer exist, and are not covered by CI. Use them only as starting points for
on-hardware debugging. The supported dev entry points live one level up
(`run_dev.sh`/`run_dev.ps1`, `test_api_endpoints.py`, `gdrive_auth.py`).
