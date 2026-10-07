# Infrastructure

Deployment support lives outside the core: systemd templates, MAVLink routing,
optional network monitors and MediaMTX/video support. Retained routing/video
support is useful in onboard and ground GPU profiles; it is not removed merely
because Edge Core is deleted.

The target has one active C++ command owner per aircraft and only selected
adapters. Runtime wiring still needs G1/G3 qualification. See
[operations](../docs/operations.md) and [migration](../docs/migration.md).
