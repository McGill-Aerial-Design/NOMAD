# NOMAD documentation

Welcome to the NOMAD documentation. NOMAD is a **reusable drone edge + ground-control
framework** pairing a Python FastAPI service (onboard companion computer) with a C#
Mission Planner plugin (ground station).

## Quick links

| If you want to… | Start here |
|---|---|
| Run the sim without hardware | [Getting Started](getting_started.md) |
| Understand the architecture | [Architecture](architecture.md) |
| Write your own module | [Writing a Module](writing_a_module.md) |
| Deploy to a Jetson | [Deployment](deployment.md) |
| Configure the system | [Configuration](configuration.md) |
| Browse the API | [API Reference](api_reference.md) |
| Understand the safety case | [Safety Case](safety/README.md) |

## Documentation structure

| Path | Content |
|------|---------|
| `getting_started.md` | Pixi dev env, sim Docker, first run |
| `architecture.md` | System design, domains, data flow |
| `writing_a_module.md` | Python plugin SDK + C# module pattern |
| `deployment.md` | Jetson all-in-one image, bare-metal systemd |
| `configuration.md` | `config/nomad.env` reference |
| `api_reference.md` | Consolidated Edge Core API endpoints |
| `safety/` | Safety case: SC/SR/NC partition, FHA-lite hazards, requirements, traceability |
