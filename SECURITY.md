# Security Policy

## Supported versions

| Version | Supported |
|---------|-----------|
| main (latest) | ✅ |
| Older releases | ❌ |

## Reporting a vulnerability

NOMAD is an open-source drone framework designed to operate in a private
Tailscale VPN. The security model assumes:

1. The companion computer and ground station are connected via a secure VPN
   (Tailscale).
2. API access is authenticated via a shared API key.
3. The radio control link (ELRS) is independent of the companion computer
   and provides a physical failsafe.

### What to report

- Authentication bypass or API key leakage
- Remote code execution via the API or terminal
- MAVLink command injection
- Hardcoded secrets or credentials in the repository

### How to report

Please report security vulnerabilities by opening a private issue on GitHub
or contacting the maintainers directly. Do not disclose vulnerabilities in
public issues.

We will acknowledge receipt within 48 hours and provide an estimated timeline
for a fix.

## Security best practices for operators

1. **Always use a strong API key.** Generate one with `python -c "import secrets; print(secrets.token_hex(32))"` and set `NOMAD_API_KEY` in `config/nomad.env`.
2. **Keep `NOMAD_ALLOW_INSECURE_REMOTE=false`** in production.
3. **Disable terminal execution** in production (`NOMAD_ENABLE_TERMINAL_EXEC=false`).
4. **Use Tailscale ACLs** to restrict access to the companion computer's ports.
5. **Keep the RC kill switch** armed at all times — the ELRS link is the
   ultimate physical failsafe and is independent of the companion computer.
6. **Never commit secrets** to the repository. The `.gitignore` excludes
   `config/nomad.env`, but be mindful of API keys and tokens in code.
