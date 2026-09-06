# Description

Please provide a brief description of your merge request and describe the
changes made. Remove any sections that are not applicable.

## Safety-critical checklist

<!-- Required if this PR touches an SC path (transitional Python safety,
     MAVLink, payload, geofence, or future C++ core code). See docs/safety.md. -->

- [ ] Requirement: SR-___ (docs/safety.md; new IDs append and are never reused)
- [ ] Test proving it: `tests/...::test_...` (fault inputs included, not just happy path)
- [ ] Safety coverage passes and the traceability block in `docs/safety.md` is updated
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
