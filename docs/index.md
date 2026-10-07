# NOMAD documentation

Start with the root [README](../README.md), then use the subject owners below.
Current system behavior belongs in the first group. Dated implementation
reports are preserved as history and do not override the current guides.

## Current system and workflow

| Subject | Canonical document |
|---|---|
| Current component ownership and command path | [Architecture](architecture.md) |
| What is proven and what remains unqualified | [Safety and qualification status](qualification.md) |
| Safety requirements and evidence mappings | [Safety case](safety.md) |
| Build, tests, CI, SITL, ROS and packaging workflow | [Development](development.md) |
| Runtime, router, profiles and deployment operation | [Operations](operations.md) |
| Persistent runtime and local IPC contract | [Runtime IPC](runtime-ipc.md) |

## Product requirements and component detail

| Subject | Canonical document |
|---|---|
| Product requirements and decisions | [PRD](prd.md) |
| Official rules, scoring and interpretations | [CONOPS requirement inventory](conops-requirements.md) |
| AEAC 2027 external contract and future integration scope | [AEAC 2027 integration](aeac-2027.md) |
| MAVSDK transport decision | [MAVSDK adoption](mavsdk-adoption.md) |
| Reviewed MAVSDK pin, license and dependency provenance | [MAVSDK dependencies](mavsdk-dependencies.md) |
| Engineering and agent practices | [Agent guidance](agent-guidance.md) |

The [migration archive](migration.md) preserves dated source reviews, decisions
and run evidence. [TODO](../TODO.md) is the branch-tip actionable ledger.
Component READMEs link back here and describe only their local interfaces.
