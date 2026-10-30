"""Exclusive Control Lease manager.

One operator at a time may send recovery splines to a robot. A lease expires after ``ttl_s`` seconds
without a heartbeat or an authorised command. E-Stop is never gated by a lease.
"""
from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any


@dataclass(slots=True)
class Lease:
    """An active lease. Deadlines use a monotonic clock so wall-clock jumps cannot extend them."""

    robot_id: str
    operator_id: str
    acquired_at: float
    expires_at: float


@dataclass(frozen=True, slots=True)
class LeaseDecision:
    """Outcome of an acquire/authorise call. ``created`` is true only for a brand-new lease."""

    granted: bool
    robot_id: str
    holder: str | None
    expires_in_s: float
    created: bool = False
    reason: str | None = None


class LeaseManager:
    """Thread-safe in-memory store of per-robot leases."""

    def __init__(
        self,
        ttl_s: float = 30.0,
        *,
        clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        if ttl_s <= 0:
            raise ValueError("ttl_s must be positive")
        self._ttl = ttl_s
        self._clock = clock
        self._wall_clock = wall_clock
        self._lock = threading.RLock()
        self._leases: dict[str, Lease] = {}
        self._expired: list[Lease] = []

    def _sweep_locked(self, now: float) -> None:
        for robot_id in [rid for rid, lease in self._leases.items() if lease.expires_at <= now]:
            self._expired.append(self._leases.pop(robot_id))

    def acquire(self, robot_id: str, operator_id: str) -> LeaseDecision:
        """Grant the lease if the robot is free or already held by ``operator_id`` (which renews it)."""
        with self._lock:
            now = self._clock()
            self._sweep_locked(now)
            current = self._leases.get(robot_id)
            if current is None:
                self._leases[robot_id] = Lease(robot_id, operator_id, now, now + self._ttl)
                return LeaseDecision(True, robot_id, operator_id, self._ttl, created=True)
            if current.operator_id == operator_id:
                current.expires_at = now + self._ttl
                return LeaseDecision(True, robot_id, operator_id, self._ttl)
            return LeaseDecision(False, robot_id, current.operator_id, current.expires_at - now, reason="held_by_other")

    def authorize(self, robot_id: str, operator_id: str) -> LeaseDecision:
        """Check that ``operator_id`` holds the lease; success also renews it (activity counts as a heartbeat)."""
        with self._lock:
            now = self._clock()
            self._sweep_locked(now)
            current = self._leases.get(robot_id)
            if current is None:
                return LeaseDecision(False, robot_id, None, 0.0, reason="no_lease")
            if current.operator_id != operator_id:
                return LeaseDecision(False, robot_id, current.operator_id, current.expires_at - now, reason="held_by_other")
            current.expires_at = now + self._ttl
            return LeaseDecision(True, robot_id, operator_id, self._ttl)

    def authorize_estop(self, robot_id: str, operator_id: str) -> LeaseDecision:
        """E-Stop bypasses every lease: always granted. ``reason`` records when another operator held the lock."""
        with self._lock:
            now = self._clock()
            self._sweep_locked(now)
            current = self._leases.get(robot_id)
            if current is None:
                return LeaseDecision(True, robot_id, None, 0.0)
            bypassed = current.operator_id != operator_id
            return LeaseDecision(
                True,
                robot_id,
                current.operator_id,
                current.expires_at - now,
                reason="bypassed_lease" if bypassed else None,
            )

    def renew(self, robot_id: str, operator_id: str) -> bool:
        """Heartbeat for one robot. Returns false if ``operator_id`` does not hold a live lease."""
        return self.authorize(robot_id, operator_id).granted

    def renew_all(self, operator_id: str) -> list[str]:
        """Heartbeat for every lease held by ``operator_id``; returns the robots that were renewed."""
        with self._lock:
            now = self._clock()
            self._sweep_locked(now)
            renewed = [lease.robot_id for lease in self._leases.values() if lease.operator_id == operator_id]
            for robot_id in renewed:
                self._leases[robot_id].expires_at = now + self._ttl
            return renewed

    def release(self, robot_id: str, operator_id: str) -> bool:
        """Release a lease held by ``operator_id``. Returns false if it is not the holder."""
        with self._lock:
            self._sweep_locked(self._clock())
            current = self._leases.get(robot_id)
            if current is None or current.operator_id != operator_id:
                return False
            del self._leases[robot_id]
            return True

    def expire(self) -> list[Lease]:
        """Sweep and return every lease that expired since the last call (for audit and broadcast)."""
        with self._lock:
            self._sweep_locked(self._clock())
            expired, self._expired = self._expired, []
            return expired

    def holder(self, robot_id: str) -> str | None:
        with self._lock:
            self._sweep_locked(self._clock())
            current = self._leases.get(robot_id)
            return current.operator_id if current else None

    def get(self, robot_id: str) -> dict[str, Any]:
        """Lease state as ``{robot_id, operator_id | None, expires_at | None}``."""
        with self._lock:
            now = self._clock()
            self._sweep_locked(now)
            current = self._leases.get(robot_id)
            if current is None:
                return {"robot_id": robot_id, "operator_id": None, "expires_at": None}
            return {
                "robot_id": robot_id,
                "operator_id": current.operator_id,
                "expires_at": round(self._wall_clock() + (current.expires_at - now), 3),
            }

    def snapshot(self) -> list[dict[str, Any]]:
        """All active leases, with wall-clock expiry timestamps and seconds remaining."""
        with self._lock:
            now = self._clock()
            wall = self._wall_clock()
            self._sweep_locked(now)
            return [
                {
                    "robot_id": lease.robot_id,
                    "operator_id": lease.operator_id,
                    "expires_at": round(wall + (lease.expires_at - now), 3),
                    "expires_in_s": round(lease.expires_at - now, 1),
                }
                for lease in self._leases.values()
            ]
