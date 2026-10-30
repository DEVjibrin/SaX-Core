"""Runtime configuration, read once from ``SAX_*`` environment variables."""
from __future__ import annotations

import os
from dataclasses import dataclass

_DEFAULT_ORIGINS = ("http://localhost:5173", "http://127.0.0.1:5173")


def _text(name: str) -> str | None:
    value = os.environ.get(name, "").strip()
    return value or None


def _number(name: str, default: float, *, low: float, high: float) -> float:
    try:
        value = float(os.environ.get(name, default))
    except ValueError:
        return default
    return min(high, max(low, value))


@dataclass(frozen=True, slots=True)
class Settings:
    """Immutable service settings. Tokens left unset disable authentication (development only)."""

    cors_origins: tuple[str, ...] = _DEFAULT_ORIGINS
    agent_token: str | None = None
    operator_token: str | None = None
    lease_ttl_s: float = 30.0
    broadcast_hz: float = 10.0
    audit_path: str = "sax_audit.json"
    audit_max_events: int = 5000
    max_message_bytes: int = 262_144
    max_robots: int = 256
    max_trajectory_points: int = 256
    agent_stale_s: float = 2.0
    lidar_zero_frames: int = 3
    raw_points_per_frame: int = 30_000
    raw_bytes_per_point: int = 16


def load_settings() -> Settings:
    """Build :class:`Settings` from the environment, clamping numeric values to safe ranges."""
    origins = _text("SAX_CORS_ORIGINS")
    return Settings(
        cors_origins=tuple(item.strip() for item in origins.split(",") if item.strip()) if origins else _DEFAULT_ORIGINS,
        agent_token=_text("SAX_AGENT_TOKEN"),
        operator_token=_text("SAX_OPERATOR_TOKEN"),
        lease_ttl_s=_number("SAX_LEASE_TTL_S", 30.0, low=5.0, high=600.0),
        broadcast_hz=_number("SAX_BROADCAST_HZ", 10.0, low=1.0, high=30.0),
        audit_path=_text("SAX_AUDIT_PATH") or "sax_audit.json",
        audit_max_events=int(_number("SAX_AUDIT_MAX_EVENTS", 5000, low=100, high=100_000)),
        max_message_bytes=int(_number("SAX_MAX_MESSAGE_BYTES", 262_144, low=1024, high=8_388_608)),
        max_robots=int(_number("SAX_MAX_ROBOTS", 256, low=1, high=10_000)),
        raw_points_per_frame=int(_number("SAX_RAW_POINTS_PER_FRAME", 30_000, low=1, high=10_000_000)),
        raw_bytes_per_point=int(_number("SAX_RAW_BYTES_PER_POINT", 16, low=1, high=64)),
    )
