# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Authentication settings and middleware for the Edge Core API."""

from __future__ import annotations

import hmac
import ipaddress
import logging
from collections.abc import Callable
from dataclasses import dataclass, field

from fastapi import HTTPException, Request
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

from .env import env_bool, env_secret

AUTH_EXEMPT_PATHS = frozenset({"/", "/health", "/docs", "/redoc", "/openapi.json"})
COMMAND_PATH_PREFIXES = ("/api/servo/", "/api/spray/")
INTERNAL_BRIDGE_TOKEN_HEADER = "X-NOMAD-Internal-Token"
INTERNAL_BRIDGE_MIN_TOKEN_LEN = 32
INTERNAL_BRIDGE_ALLOWED_ROUTES = frozenset(
    {
        ("POST", "/api/vio/update"),
        ("POST", "/api/slam/mesh/update"),
        ("POST", "/api/servo/camera/tilt"),
        ("GET", "/api/servo/camera/tilt"),
    }
)


@dataclass(frozen=True)
class AuthSettings:
    """Authentication settings read once during app construction."""

    api_key: str | None
    allow_insecure_remote: bool
    internal_bridge_token: str | None
    auth_exempt_paths: frozenset[str] = AUTH_EXEMPT_PATHS
    command_path_prefixes: tuple[str, ...] = COMMAND_PATH_PREFIXES
    internal_bridge_header: str = INTERNAL_BRIDGE_TOKEN_HEADER
    internal_bridge_allowed_routes: frozenset[tuple[str, str]] = field(
        default_factory=lambda: INTERNAL_BRIDGE_ALLOWED_ROUTES
    )

    @classmethod
    def from_env(cls, logger: logging.Logger) -> AuthSettings:
        api_key = env_secret("NOMAD_API_KEY")
        internal_bridge_token = env_secret("NOMAD_INTERNAL_TOKEN")

        if internal_bridge_token is not None and len(internal_bridge_token) < INTERNAL_BRIDGE_MIN_TOKEN_LEN:
            logger.warning(
                "NOMAD_INTERNAL_TOKEN is shorter than %d chars; disabling internal bridge bypass",
                INTERNAL_BRIDGE_MIN_TOKEN_LEN,
            )
            internal_bridge_token = None

        settings = cls(
            api_key=api_key,
            allow_insecure_remote=env_bool("NOMAD_ALLOW_INSECURE_REMOTE"),
            internal_bridge_token=internal_bridge_token,
        )
        settings.log_startup_warnings(logger)
        return settings

    def log_startup_warnings(self, logger: logging.Logger) -> None:
        if self.api_key is None:
            logger.warning(
                "NOMAD_API_KEY is not configured; only loopback clients are allowed unless "
                "NOMAD_ALLOW_INSECURE_REMOTE=true"
            )
        if self.api_key is not None and self.internal_bridge_token is None:
            logger.warning("NOMAD_INTERNAL_TOKEN is not configured; internal bridge bypass disabled")

    def require_admin_api_key(self) -> None:
        """Require a configured API key for high-risk terminal/admin routes."""
        if self.api_key is None:
            raise HTTPException(
                status_code=403,
                detail="Admin/terminal routes require NOMAD_API_KEY to be configured",
            )

    def is_command_path(self, request_path: str) -> bool:
        return request_path.startswith(self.command_path_prefixes)


def is_loopback_client(request: Request) -> bool:
    client_host = ""
    if request.client is not None and request.client.host is not None:
        client_host = request.client.host.strip().lower()
    if client_host in {"127.0.0.1", "::1", "localhost"}:
        return True
    if client_host.startswith("::ffff:"):
        client_host = client_host.split("::ffff:", 1)[1]
    try:
        return ipaddress.ip_address(client_host).is_loopback
    except ValueError:
        return False


def client_host(request: Request) -> str:
    if request.client is not None and request.client.host is not None:
        return request.client.host
    return "unknown"


def has_valid_api_key(request: Request, api_key: str) -> bool:
    provided_key = request.headers.get("X-API-Key") or ""
    return bool(provided_key) and hmac.compare_digest(provided_key, api_key)


def is_internal_bridge_request(request: Request, request_path: str, settings: AuthSettings) -> bool:
    if settings.internal_bridge_token is None:
        return False
    if (request.method.upper(), request_path) not in settings.internal_bridge_allowed_routes:
        return False
    if not is_loopback_client(request):
        return False
    provided_token = request.headers.get(settings.internal_bridge_header) or ""
    return bool(provided_token) and hmac.compare_digest(provided_token, settings.internal_bridge_token)


class APIKeyMiddleware(BaseHTTPMiddleware):
    """Enforce API keys while preserving loopback-only development access."""

    def __init__(
        self,
        app,
        settings: AuthSettings,
        logger: logging.Logger,
        audit_command: Callable[[Request, str, str], None] | None = None,
    ) -> None:
        super().__init__(app)
        self.settings = settings
        self.logger = logger
        self.audit_command = audit_command or self._audit_command

    async def dispatch(self, request: Request, call_next):
        request_path = request.url.path.rstrip("/") or "/"
        if self.settings.api_key is None:
            return await self._dispatch_without_api_key(request, request_path, call_next)
        return await self._dispatch_with_api_key(request, request_path, call_next)

    async def _dispatch_without_api_key(self, request: Request, request_path: str, call_next):
        if self.settings.is_command_path(request_path):
            return await self._dispatch_unauthenticated_command(request, request_path, call_next)
        if self.settings.allow_insecure_remote or is_loopback_client(request):
            return await call_next(request)
        return JSONResponse(
            status_code=401,
            content={"detail": "NOMAD_API_KEY is not configured; remote API access is disabled"},
        )

    async def _dispatch_unauthenticated_command(self, request: Request, request_path: str, call_next):
        if is_internal_bridge_request(request, request_path, self.settings):
            self.audit_command(request, request_path, "internal-token")
            return await call_next(request)
        self.logger.warning(
            "SC command refused (no API key configured): %s %s from %s",
            request.method,
            request_path,
            client_host(request),
        )
        return JSONResponse(
            status_code=403,
            content={"detail": "Command endpoints require NOMAD_API_KEY to be configured"},
        )

    async def _dispatch_with_api_key(self, request: Request, request_path: str, call_next):
        if request_path in self.settings.auth_exempt_paths:
            return await call_next(request)
        if is_internal_bridge_request(request, request_path, self.settings):
            if self.settings.is_command_path(request_path):
                self.audit_command(request, request_path, "internal-token")
            return await call_next(request)
        # dispatch() only routes here when api_key is configured; the fallback
        # keeps mypy satisfied without an assert in the hot path.
        if not has_valid_api_key(request, self.settings.api_key or ""):
            if self.settings.is_command_path(request_path):
                self.logger.warning(
                    "SC command refused (bad/missing key): %s %s from %s",
                    request.method,
                    request_path,
                    client_host(request),
                )
            return JSONResponse(
                status_code=401,
                content={"detail": "Invalid or missing API key"},
            )
        if self.settings.is_command_path(request_path):
            self.audit_command(request, request_path, "api-key")
        return await call_next(request)

    def _audit_command(self, request: Request, request_path: str, auth_mode: str) -> None:
        """Post-flight audit trail for actuation commands (SR-SEC-03)."""
        self.logger.info(
            "SC command audit: %s %s from %s auth=%s",
            request.method,
            request_path,
            client_host(request),
            auth_mode,
        )
