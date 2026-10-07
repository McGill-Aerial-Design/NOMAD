# Configuration

nomad.env.example and profiles are deployment templates. Real values belong in
ignored local configuration; never commit credentials, actual endpoints or
machine-specific paths. Only onboard_companion, groundstation_gpu and
groundstation_minimal are supported product profiles.

Reviewed limits and validated settings feed core policy explicitly. Compute
placement does not silently choose navigation safety or command authority.
See [operations](../docs/operations.md) and [migration](../docs/migration.md).

Managed runtime uses the packaged lifecycle `runtime.example.json`, copied into
protected external configuration with a separate identity/token file and external
persistent journals. Profile loading never provisions, enables or starts that
service or admits authority. Follow Operations for Linux/Windows procedures.
