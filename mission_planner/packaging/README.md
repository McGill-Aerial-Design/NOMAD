# Plugin artifact

The product artifact is `mission_planner/src/bin/Release/NOMADPlugin.dll`.
Build with `scripts/build/build_plugin_windows.ps1`; it never installs anything.
Use the explicit [closed-Mission-Planner manual procedure](../../docs/operations.md#manual-plugin-installation).
No host assemblies, runtime/service executables or dependency binaries are bundled.
The former multi-component installer/automatic rollback capability is removed.
