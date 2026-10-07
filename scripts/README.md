# Scripts

These are build, test, simulation, setup and deployment utilities, not the
vehicle core. scripts/dev contains C++ SITL runners and quality checks;
scripts/build contains plugin/image helpers; scripts/services manages retained
adapters. scripts/nomad is a deployment dispatcher, distinct from the C++ CLI.

Setup/provisioning targets the C++ core and retained service inventory; live host
qualification remains in G1 in [migration](../docs/migration.md). Retained Python
may configure or observe tests, but production vehicle decisions belong in C++.
Use [development](../docs/development.md) and [operations](../docs/operations.md)
for the canonical workflow.
