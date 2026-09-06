# Transitional configuration profiles

The profile files are complete environment templates for the current Python edge
runtime. They are useful for simulation and deployment tests while the C++ core
is being built.

```bash
pixi run profile-list
pixi run profile-load dev
pixi run profile-show
```

The profile loader writes the ignored `config/nomad.env`. Do not add new product
settings here unless the migration plan assigns them to a real deployment
boundary. See [operations](../../docs/operations.md).
