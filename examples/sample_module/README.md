# Sample NOMAD module

A minimal, runnable example of the NOMAD Python module SDK (`edge_core.core`).
It registers a single route — `GET /api/sample/ping` — and tracks its
`start()`/`stop()` lifecycle. See [`docs/writing_a_module.md`](../../docs/writing_a_module.md)
for the full guide.

## Try it

Loaded explicitly by spec (no install required — this is how the tests do it):

```python
from fastapi import FastAPI
from edge_core.core import wire_modules

app = FastAPI()
wire_modules(app, specs=["sample_module.sample_module:SampleModule"])
# GET /api/sample/ping -> {"module": "sample", "started": true, "reply": "pong"}
```

For a real third-party module you would ship it as its own package and register
it on the `nomad.modules` entry-point group so NOMAD discovers it automatically:

```toml
[project.entry-points."nomad.modules"]
sample = "sample_module.sample_module:SampleModule"
```

The behavior is pinned by `tests/test_module_registry.py`.
