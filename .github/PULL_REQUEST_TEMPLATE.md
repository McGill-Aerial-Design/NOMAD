# Description

Please provide a brief description of your merge request and describe the
changes made. Remove any sections that are not applicable.

## Safety-critical checklist

<!-- Required ONLY if this PR touches an SC path (edge_core/safety/,
     mavlink_velocity.py, services/mavlink/, modules/payload/, or the C#
     Control/, Payload/, Geofence/ dirs — see docs/safety/partition.md).
     Delete this section otherwise. -->

- [ ] Requirement: SR-___ (docs/safety/requirements.md; new ones append, never renumber)
- [ ] Test proving it: `tests/...::test_...` (fault inputs included, not just happy path)
- [ ] `pixi run cov-safety` still 100% and traceability.md updated if symbols moved
- [ ] SITL evidence if the velocity/fence/payload command path changed (`pixi run sitl-scenario` / `sitl-fence`), or stated why not run
- [ ] No gate/clamp/watchdog weakened, reordered, or removed

**Category:** <!-- one of: edge, plugin, docker, infra, ci, docs, refactor, chore -->

# Changelog

## Added
- [required_label] [optional_label] Mainlevel description that requires details given in Sublevel 1.
  - Sublevel 1 description detailing the change mentioned in the Mainlevel, and that requires even more detail given in Sublevel 2.
    - Sublevel 2 description detailing further the change mentioned in Sublevel 2.

## Changed
- [folder] For changes in existing functionality.

## Deprecated
- [folder] For soon-to-be removed features.

## Fixed
- [folder] [bug] For any bug fixes.

## Removed
- [folder] For now removed features.
