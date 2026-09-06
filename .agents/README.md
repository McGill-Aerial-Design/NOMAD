# Repository-local agents

These narrow roles support the NOMAD workflow. Project rules in `AGENTS.md` and
any deeper instruction files always take precedence.

- `finder.agent.md`: read-only codebase research.
- `coder.agent.md`: focused implementation and validation.
- `reviewer.agent.md`: read-only correctness and risk review.

Use the finder before editing when ownership or behavior is unclear. Use the coder
only after the scope and success criteria are clear. Use the reviewer for a separate
read-only pass over correctness, safety, concurrency, compatibility, and test gaps.
