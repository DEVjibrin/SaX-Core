"""WebSocket connection registry and fan-out primitives."""
from __future__ import annotations

import asyncio
import contextlib
import logging
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from starlette.websockets import WebSocket

from .telemetry import RobotFrame

log = logging.getLogger("sax.hub")


class Client:
    """A WebSocket plus a send lock, so concurrent tasks never interleave frames on one connection."""

    def __init__(self, websocket: WebSocket) -> None:
        self.websocket = websocket
        self.closed = False
        self._lock = asyncio.Lock()

    async def send_text(self, text: str, timeout: float = 2.0) -> bool:
        """Send with a timeout. Any failure marks the client closed and returns False."""
        if self.closed:
            return False
        try:
            async with self._lock:
                await asyncio.wait_for(self.websocket.send_text(text), timeout)
        except Exception as exc:
            self.closed = True
            log.debug("send failed: %r", exc)
            return False
        return True

    async def close(self, code: int = 1000, reason: str = "") -> None:
        self.closed = True
        with contextlib.suppress(Exception):
            await self.websocket.close(code=code, reason=reason)


@dataclass(slots=True, eq=False)
class AgentSession:
    """A connected edge robot."""

    robot_id: str
    client: Client
    connected_at: float
    last_frame_at: float = 0.0
    last_seq: int = -1
    frames: int = 0
    bytes_in: int = 0
    parse_errors: int = 0
    stale_dropped: int = 0
    estop_sent_at: float = 0.0


@dataclass(slots=True, eq=False)
class ViewerSession:
    """A connected dashboard. ``voxel_robots`` limits which robots stream voxels (``None`` means all)."""

    conn_id: str
    operator_id: str
    client: Client
    connected_at: float
    voxel_robots: set[str] | None = None


class Hub:
    """Registry of agents, viewers, the latest frame per robot, and E-stops awaiting confirmation."""

    def __init__(self, max_robots: int = 256) -> None:
        self.agents: dict[str, AgentSession] = {}
        self.viewers: dict[str, ViewerSession] = {}
        self.frames: dict[str, RobotFrame] = {}
        self.pending_estops: dict[str, dict[str, Any]] = {}
        self._max_robots = max_robots

    def known_robots(self) -> set[str]:
        return set(self.agents) | set(self.frames)

    def can_admit(self, robot_id: str) -> bool:
        return robot_id in self.agents or robot_id in self.frames or len(self.known_robots()) < self._max_robots

    def attach_agent(self, session: AgentSession) -> AgentSession | None:
        """Register ``session``; a reconnecting robot supersedes its previous (possibly half-open) socket."""
        previous = self.agents.get(session.robot_id)
        self.agents[session.robot_id] = session
        return previous

    def detach_agent(self, session: AgentSession) -> bool:
        if self.agents.get(session.robot_id) is session:
            del self.agents[session.robot_id]
            return True
        return False

    def attach_viewer(self, session: ViewerSession) -> None:
        self.viewers[session.conn_id] = session

    def detach_viewer(self, session: ViewerSession) -> None:
        self.viewers.pop(session.conn_id, None)

    async def send_to_agent(self, robot_id: str, text: str) -> bool:
        session = self.agents.get(robot_id)
        return session is not None and await session.client.send_text(text)

    async def broadcast(self, render: Callable[[ViewerSession], str]) -> None:
        """Send ``render(viewer)`` to every viewer concurrently, dropping any that fail or time out."""
        viewers = list(self.viewers.values())
        if not viewers:
            return
        results = await asyncio.gather(*(viewer.client.send_text(render(viewer)) for viewer in viewers))
        for viewer, delivered in zip(viewers, results):
            if not delivered:
                self.detach_viewer(viewer)
                await viewer.client.close(1011, "send failed")
