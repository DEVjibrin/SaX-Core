"""Shared-secret token and Origin checks for the REST API and WebSockets."""
from __future__ import annotations

import secrets
from collections.abc import Mapping


def token_ok(expected: str | None, provided: str | None) -> bool:
    """Constant-time token comparison. An unset ``expected`` token disables the check."""
    if not expected:
        return True
    return provided is not None and secrets.compare_digest(expected.encode("utf-8"), provided.encode("utf-8"))


def bearer_or_query(headers: Mapping[str, str], query: Mapping[str, str]) -> str | None:
    """Extract a token from ``Authorization: Bearer``, ``X-API-Key``, or the ``token`` query parameter."""
    authorization = headers.get("authorization", "")
    if authorization.lower().startswith("bearer "):
        return authorization[7:].strip()
    return headers.get("x-api-key") or query.get("token")


def origin_ok(origin: str | None, allowed: tuple[str, ...]) -> bool:
    """Reject cross-site WebSocket handshakes from browsers; non-browser clients send no Origin."""
    return origin is None or "*" in allowed or origin in allowed
