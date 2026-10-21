# Contributing

Read [AGENTS.md](AGENTS.md), [architecture](docs/architecture.md) and
[development checks](docs/development.md). Keep changes small and preserve real
operator profiles, ignored evidence, unrelated worktrees and local edits.

Use the reviewed Mission Planner API and existing plugin patterns. Run the focused
check, relevant product gates and final diff review. Explain the concrete behavior,
actual validation and unresolved limits in the PR. Never claim physical safety from
host doubles, loopback routing or an ACK. Installation and aircraft testing require
separate authorization; build scripts must not perform them.

Report security issues through [SECURITY.md](SECURITY.md). Source and legal notices
remain under the repository [LICENSE](LICENSE); retained third-party license texts
are documented in [NOTICE](NOTICE).
