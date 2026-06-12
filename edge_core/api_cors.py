# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""CORS configuration for the Edge Core API."""

from __future__ import annotations

import os
from dataclasses import dataclass

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware


@dataclass(frozen=True)
class CorsSettings:
    allowed_origins: list[str]
    allow_credentials: bool

    @classmethod
    def from_env(cls) -> CorsSettings:
        gcs_origin = os.environ.get("GCS_ORIGIN")
        return cls(
            allowed_origins=[gcs_origin] if gcs_origin else ["*"],
            allow_credentials=bool(gcs_origin and gcs_origin != "*"),
        )


def configure_cors(app: FastAPI, settings: CorsSettings) -> None:
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.allowed_origins,
        allow_credentials=settings.allow_credentials,
        allow_methods=["*"],
        allow_headers=["*"],
    )
