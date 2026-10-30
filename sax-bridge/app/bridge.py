"""Orchestration: agent ingest, operator commands, E-stop propagation, and the 10 Hz dashboard fan-out."""
from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import math
import time
import uuid
from collections.abc import Awaitable, Callable
from typing import Any

from starlette.websockets import WebSocket

from . import __version__
from .audit_store import AuditStore
from .config import Settings
from .hub import AgentSession, Client, Hub, ViewerSession
from .ids import ROBOT_ID_RE
from .lease_manager import LeaseManager
from .telemetry import (
    BandwidthTracker,
    LidarMonitor,
    RobotFrame,
    RobotState,
    SX1ParseError,
    encode_spline,
    parse_sx1,
    path_length,
)

log = logging.getLogger("sax.bridge")

STALE_SEQ_WINDOW = 10_000
ESTOP_RESEND_S = 1.0
ESTOP_PENDING_TTL_S = 600.0
MAX_PENDING_ESTOPS = 1024
MAX_COORDINATE_M = 10_000.0

Handler = Callable[[ViewerSession, dict[str, Any]], Awaitable[dict[str, Any]]]


def _dumps(value: Any) -> str:
    return json.dumps(value, separators=(",", ":"))


class CommandError(Exception):
    """A rejected operator command, reported back to the sender as an ``error`` message."""

    def __init__(self, code: str, message: str, **extra: Any) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.extra = extra


class Bridge:
    """Owns all service state. One instance per process (state is in memory, so run a single worker)."""

    def __init__(self, settings: Settings, audit: AuditStore | None = None) -> None:
        self.settings = settings
        self.hub = Hub(settings.max_robots)
        self.leases = LeaseManager(settings.lease_ttl_s)
        self.audit = audit or AuditStore(settings.audit_path, max_events=settings.audit_max_events)
        self.bandwidth = BandwidthTracker(
            raw_points_per_frame=settings.raw_points_per_frame,
            raw_bytes_per_point=settings.raw_bytes_per_point,
        )
        self.lidar = LidarMonitor(settings.lidar_zero_frames)
        self.started_at = time.time()
        self._tick_task: asyncio.Task[None] | None = None
        self._tick_count = 0
        self._blobs: dict[str, tuple[tuple[int, bool, bool], str, str]] = {}
        self._handlers: dict[str, Handler] = {
            "lease.acquire": self._lease_acquire,
            "lease.release": self._lease_release,
            "lease.heartbeat": self._lease_heartbeat,
            "trajectory": self._trajectory,
            "estop": self._estop,
            "subscribe": self._subscribe,
            "ping": self._ping,
        }

    # ------------------------------------------------------------------ lifecycle

    async def start(self) -> None:
        if self._tick_task is None:
            self._tick_task = asyncio.create_task(self._tick_loop(), name="sax-bridge-tick")

    async def stop(self) -> None:
        if self._tick_task is not None:
            self._tick_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._tick_task
            self._tick_task = None
        sessions = [*self.hub.agents.values(), *self.hub.viewers.values()]
        await asyncio.gather(*(s.client.close(1001, "server shutdown") for s in sessions), return_exceptions=True)
        await asyncio.to_thread(self.audit.close)

    # ------------------------------------------------------------------ helpers

    async def _audit(
        self,
        event_type: str,
        *,
        operator_id: str | None = None,
        robot_id: str | None = None,
        detail: dict[str, Any] | None = None,
        critical: bool = False,
    ) -> None:
        self.audit.record(event_type, operator_id=operator_id, robot_id=robot_id, detail=detail)
        if critical:
            await asyncio.to_thread(self.audit.flush)

    async def _emit(self, event: str, **fields: Any) -> None:
        text = _dumps({"type": "event", "event": event, "ts": round(time.time(), 3), **fields})
        await self.hub.broadcast(lambda _viewer: text)

    @staticmethod
    async def _send(viewer: ViewerSession, payload: dict[str, Any]) -> None:
        await viewer.client.send_text(_dumps(payload))

    async def send_error(self, viewer: ViewerSession, request_id: Any, code: str, message: str, **extra: Any) -> None:
        await self._send(viewer, {"type": "error", "id": request_id, "code": code, "message": message, **extra})

    # ------------------------------------------------------------------ agents

    async def register_agent(self, robot_id: str, websocket: WebSocket) -> AgentSession | None:
        """Attach a robot connection, superseding any previous one. Returns ``None`` at capacity."""
        if not self.hub.can_admit(robot_id):
            return None
        session = AgentSession(robot_id, Client(websocket), time.monotonic())
        previous = self.hub.attach_agent(session)
        if previous is not None:
            await previous.client.close(4000, "replaced by a newer connection")
        await self._audit("agent_connected", robot_id=robot_id, detail={"replaced": previous is not None})
        await self._emit("robot_online", robot_id=robot_id)
        pending = self.hub.pending_estops.get(robot_id)
        if pending is not None:
            await self._deliver_estop(session, pending["operator_id"])
            await self._audit(
                "e_stop_redelivered",
                operator_id=pending["operator_id"],
                robot_id=robot_id,
                detail={"waited_s": round(time.time() - pending["at"], 1)},
                critical=True,
            )
        return session

    async def unregister_agent(self, session: AgentSession, reason: str = "disconnected") -> None:
        if not self.hub.detach_agent(session):
            return
        self.lidar.reset(session.robot_id)
        await self._audit(
            "agent_disconnected",
            robot_id=session.robot_id,
            detail={"reason": reason, "frames": session.frames, "parse_errors": session.parse_errors},
        )
        await self._emit("robot_offline", robot_id=session.robot_id)

    async def ingest(self, session: AgentSession, line: str) -> bool:
        """Process one SX1 line. Returns False for a stale duplicate; raises ``SX1ParseError`` for bad input."""
        frame = parse_sx1(line)
        if frame.robot_id != session.robot_id:
            raise SX1ParseError(f"robot_id {frame.robot_id!r} does not match this connection")
        if session.last_seq >= 0 and 0 <= session.last_seq - frame.seq < STALE_SEQ_WINDOW:
            session.stale_dropped += 1
            return False

        now = time.monotonic()
        session.last_seq = frame.seq
        session.last_frame_at = now
        session.frames += 1
        session.bytes_in += frame.wire_bytes
        previous = self.hub.frames.get(frame.robot_id)
        self.hub.frames[frame.robot_id] = frame
        self.bandwidth.record(frame.robot_id, frame.wire_bytes)

        transition = self.lidar.observe(frame)
        if transition is not None:
            event = f"lidar_blockage_{transition}"
            await self._audit(
                event,
                robot_id=frame.robot_id,
                detail={"voxel_count": frame.voxel_count, "state": frame.state.value},
                critical=transition == "detected",
            )
            await self._emit(event, robot_id=frame.robot_id)
        if previous is None or previous.state is not frame.state:
            await self._emit("state_changed", robot_id=frame.robot_id, state=frame.state.value, reason=frame.reason)
        await self._reconcile_estop(session, frame, now)
        return True

    async def _reconcile_estop(self, session: AgentSession, frame: RobotFrame, now: float) -> None:
        pending = self.hub.pending_estops.get(frame.robot_id)
        if pending is None:
            return
        if frame.state is RobotState.HALTED:
            del self.hub.pending_estops[frame.robot_id]
            await self._audit(
                "e_stop_confirmed",
                operator_id=pending["operator_id"],
                robot_id=frame.robot_id,
                detail={"latency_s": round(time.time() - pending["at"], 3)},
            )
        elif now - session.estop_sent_at >= ESTOP_RESEND_S:
            await self._deliver_estop(session, pending["operator_id"])

    async def _deliver_estop(self, session: AgentSession, operator_id: str) -> bool:
        session.estop_sent_at = time.monotonic()
        return await session.client.send_text(f"SXE|{session.robot_id}|{operator_id}")

    # ------------------------------------------------------------------ operators

    def register_viewer(self, operator_id: str, websocket: WebSocket) -> ViewerSession:
        viewer = ViewerSession(uuid.uuid4().hex[:12], operator_id, Client(websocket), time.monotonic())
        self.hub.attach_viewer(viewer)
        return viewer

    def unregister_viewer(self, viewer: ViewerSession) -> None:
        self.hub.detach_viewer(viewer)

    async def send_hello(self, viewer: ViewerSession) -> None:
        await self._send(
            viewer,
            {
                "type": "hello",
                "operator_id": viewer.operator_id,
                "conn_id": viewer.conn_id,
                "broadcast_hz": self.settings.broadcast_hz,
                "lease_ttl_s": self.settings.lease_ttl_s,
                "leases": self.leases.snapshot(),
                "server_time": round(time.time(), 3),
            },
        )

    async def handle_operator_message(self, viewer: ViewerSession, message: dict[str, Any]) -> None:
        """Dispatch one JSON command from a dashboard and reply with ``ack`` or ``error``."""
        kind = message.get("type")
        request_id = message.get("id")
        if isinstance(request_id, bool) or not isinstance(request_id, (str, int, type(None))):
            request_id = None
        handler = self._handlers.get(kind) if isinstance(kind, str) else None
        if handler is None:
            await self.send_error(viewer, request_id, "unknown_type", f"unsupported message type {kind!r}")
            return
        try:
            result = await handler(viewer, message)
        except CommandError as exc:
            await self.send_error(viewer, request_id, exc.code, exc.message, **exc.extra)
            return
        await self._send(viewer, {"type": "ack", "id": request_id, "action": kind, "ok": True, **result})

    def _robot_id(self, message: dict[str, Any], *, must_exist: bool = True) -> str:
        robot_id = message.get("robot_id")
        if not isinstance(robot_id, str) or not ROBOT_ID_RE.fullmatch(robot_id):
            raise CommandError("bad_robot_id", "robot_id is missing or malformed")
        if must_exist and robot_id not in self.hub.known_robots():
            raise CommandError("unknown_robot", f"robot {robot_id} has not connected", robot_id=robot_id)
        return robot_id

    def _points(self, raw: Any) -> list[tuple[float, float]]:
        limit = self.settings.max_trajectory_points
        if not isinstance(raw, list) or not 2 <= len(raw) <= limit:
            raise CommandError("bad_points", f"points must be a list of 2 to {limit} entries")
        points: list[tuple[float, float]] = []
        for entry in raw:
            if isinstance(entry, dict):
                x, y = entry.get("x"), entry.get("y")
            elif isinstance(entry, (list, tuple)) and len(entry) == 2:
                x, y = entry
            else:
                raise CommandError("bad_points", "each point must be {x, y} or [x, y]")
            for value in (x, y):
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise CommandError("bad_points", "coordinates must be numbers in metres")
                if not math.isfinite(value) or abs(value) > MAX_COORDINATE_M:
                    raise CommandError("bad_points", "coordinates must be finite and within 10 km")
            points.append((float(x), float(y)))
        return points

    async def _lease_acquire(self, viewer: ViewerSession, message: dict[str, Any]) -> dict[str, Any]:
        robot_id = self._robot_id(message)
        decision = self.leases.acquire(robot_id, viewer.operator_id)
        if not decision.granted:
            await self._audit(
                "lease_denied", operator_id=viewer.operator_id, robot_id=robot_id, detail={"held_by": decision.holder}
            )
            raise CommandError(
                "lease_denied",
                f"robot {robot_id} is controlled by {decision.holder}",
                robot_id=robot_id,
                holder=decision.holder,
                expires_in_s=round(decision.expires_in_s, 1),
            )
        if decision.created:
            await self._audit(
                "lease_acquired",
                operator_id=viewer.operator_id,
                robot_id=robot_id,
                detail={"ttl_s": self.settings.lease_ttl_s},
            )
            await self._emit("lease_acquired", robot_id=robot_id, operator_id=viewer.operator_id)
        return {"robot_id": robot_id, "lease": self.leases.get(robot_id)}

    async def _lease_release(self, viewer: ViewerSession, message: dict[str, Any]) -> dict[str, Any]:
        robot_id = self._robot_id(message, must_exist=False)
        if not self.leases.release(robot_id, viewer.operator_id):
            raise CommandError("not_holder", "you do not hold this lease", robot_id=robot_id)
        await self._audit("lease_released", operator_id=viewer.operator_id, robot_id=robot_id, detail={"reason": "operator"})
        await self._emit("lease_released", robot_id=robot_id, operator_id=viewer.operator_id)
        return {"robot_id": robot_id}

    async def _lease_heartbeat(self, viewer: ViewerSession, message: dict[str, Any]) -> dict[str, Any]:
        robot_id = message.get("robot_id")
        if isinstance(robot_id, str):
            renewed = [robot_id] if self.leases.renew(robot_id, viewer.operator_id) else []
        else:
            renewed = self.leases.renew_all(viewer.operator_id)
        return {"renewed": renewed}

    async def _trajectory(self, viewer: ViewerSession, message: dict[str, Any]) -> dict[str, Any]:
        robot_id = self._robot_id(message)
        decision = self.leases.authorize(robot_id, viewer.operator_id)
        if not decision.granted:
            await self._audit(
                "trajectory_denied",
                operator_id=viewer.operator_id,
                robot_id=robot_id,
                detail={"holder": decision.holder, "reason": decision.reason},
            )
            raise CommandError(
                "lease_required", "only the lease holder can send trajectories", robot_id=robot_id, holder=decision.holder
            )
        points = self._points(message.get("points"))
        payload = encode_spline(robot_id, points)
        if not await self.hub.send_to_agent(robot_id, payload):
            raise CommandError("robot_offline", f"robot {robot_id} is not connected", robot_id=robot_id)
        await self._audit(
            "trajectory_submitted",
            operator_id=viewer.operator_id,
            robot_id=robot_id,
            detail={"points": len(points), "length_m": round(path_length(points), 2), "payload": payload, "bytes": len(payload)},
            critical=True,
        )
        await self._emit("trajectory_submitted", robot_id=robot_id, operator_id=viewer.operator_id, points=len(points))
        return {"robot_id": robot_id, "points": len(points), "bytes": len(payload)}

    async def _estop(self, viewer: ViewerSession, message: dict[str, Any]) -> dict[str, Any]:
        robot_id = None if message.get("robot_id") is None else self._robot_id(message, must_exist=False)
        reason = str(message.get("reason", "operator e-stop"))[:200]
        return await self.trigger_estop(operator_id=viewer.operator_id, robot_id=robot_id, reason=reason, source="ws")

    async def _subscribe(self, viewer: ViewerSession, message: dict[str, Any]) -> dict[str, Any]:
        robots = message.get("robots")
        if robots is None:
            viewer.voxel_robots = None
        elif isinstance(robots, list) and all(isinstance(r, str) and ROBOT_ID_RE.fullmatch(r) for r in robots):
            viewer.voxel_robots = set(robots)
        else:
            raise CommandError("bad_subscription", "robots must be a list of robot ids, or null for all")
        return {"robots": None if viewer.voxel_robots is None else sorted(viewer.voxel_robots)}

    async def _ping(self, viewer: ViewerSession, message: dict[str, Any]) -> dict[str, Any]:
        return {"server_time": round(time.time(), 3)}

    async def trigger_estop(
        self, *, operator_id: str, robot_id: str | None, reason: str, source: str
    ) -> dict[str, Any]:
        """Send an E-stop to one robot or the whole fleet and notify every dashboard.

        Leases are deliberately ignored: any operator can stop any robot at any time. Commands go out
        before auditing, and robots that are offline receive the stop as soon as they reconnect.
        """
        scope = "robot" if robot_id else "fleet"
        targets = [robot_id] if robot_id else sorted(self.hub.known_robots())
        delivered: list[str] = []
        queued: list[str] = []
        decisions = {}
        for target in targets:
            decisions[target] = self.leases.authorize_estop(target, operator_id)
            if target in self.hub.pending_estops or len(self.hub.pending_estops) < MAX_PENDING_ESTOPS:
                self.hub.pending_estops[target] = {"operator_id": operator_id, "at": time.time(), "reason": reason}
            agent = self.hub.agents.get(target)
            sent = agent is not None and await self._deliver_estop(agent, operator_id)
            (delivered if sent else queued).append(target)

        for target in targets:
            decision = decisions[target]
            await self._audit(
                "e_stop",
                operator_id=operator_id,
                robot_id=target,
                detail={
                    "source": source,
                    "scope": scope,
                    "reason": reason,
                    "delivered": target in delivered,
                    "lease_holder": decision.holder,
                    "bypassed_lease": decision.reason == "bypassed_lease",
                },
                critical=True,
            )
        if not targets:
            await self._audit(
                "e_stop",
                operator_id=operator_id,
                detail={"source": source, "scope": scope, "reason": reason, "note": "no robots known"},
                critical=True,
            )
        await self._emit(
            "estop", operator_id=operator_id, scope=scope, targets=targets, delivered=delivered, queued=queued, reason=reason
        )
        return {"status": "broadcast", "scope": scope, "targets": targets, "delivered": delivered, "queued": queued}

    # ------------------------------------------------------------------ 10 Hz fan-out

    async def _tick_loop(self) -> None:
        interval = 1.0 / self.settings.broadcast_hz
        next_tick = time.monotonic()
        while True:
            next_tick += interval
            try:
                await self._tick()
            except asyncio.CancelledError:
                raise
            except Exception:
                log.exception("broadcast tick failed")
            delay = next_tick - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                next_tick = time.monotonic()

    async def _tick(self) -> None:
        self._tick_count += 1
        once_a_second = self._tick_count % max(1, round(self.settings.broadcast_hz)) == 0
        if once_a_second:
            await self._housekeeping()
        if self.hub.viewers:
            await self._broadcast_fleet(include_bandwidth=once_a_second)

    async def _housekeeping(self) -> None:
        for lease in self.leases.expire():
            await self._audit(
                "lease_expired",
                operator_id=lease.operator_id,
                robot_id=lease.robot_id,
                detail={"ttl_s": self.settings.lease_ttl_s},
            )
            await self._emit("lease_expired", robot_id=lease.robot_id, operator_id=lease.operator_id)
        cutoff = time.time() - ESTOP_PENDING_TTL_S
        for robot_id, pending in list(self.hub.pending_estops.items()):
            if pending["at"] < cutoff:
                del self.hub.pending_estops[robot_id]
                await self._audit("e_stop_expired", operator_id=pending["operator_id"], robot_id=robot_id)

    async def _broadcast_fleet(self, *, include_bandwidth: bool) -> None:
        now = time.monotonic()
        blobs: dict[str, tuple[str, str]] = {}
        for robot_id, frame in self.hub.frames.items():
            agent = self.hub.agents.get(robot_id)
            online = agent is not None and now - agent.last_frame_at <= self.settings.agent_stale_s
            blocked = self.lidar.is_blocked(robot_id)
            key = (frame.seq, online, blocked)
            cached = self._blobs.get(robot_id)
            if cached is None or cached[0] != key:
                full = frame.to_dict()
                full.update(online=online, lidar_blocked=blocked)
                light = {name: value for name, value in full.items() if name != "voxels"}
                cached = (key, _dumps(full), _dumps(light))
                self._blobs[robot_id] = cached
            blobs[robot_id] = (cached[1], cached[2])

        leases_json = _dumps(
            {
                lease["robot_id"]: {"operator_id": lease["operator_id"], "expires_in_s": lease["expires_in_s"]}
                for lease in self.leases.snapshot()
            }
        )
        bandwidth_json = _dumps(self.bandwidth.snapshot()) if include_bandwidth else None
        ts = f"{time.time():.3f}"
        rendered: dict[frozenset[str] | None, str] = {}

        def render(viewer: ViewerSession) -> str:
            key = None if viewer.voxel_robots is None else frozenset(viewer.voxel_robots)
            if key not in rendered:
                rendered[key] = self._compose(ts, blobs, leases_json, bandwidth_json, key)
            return rendered[key]

        await self.hub.broadcast(render)

    @staticmethod
    def _compose(
        ts: str,
        blobs: dict[str, tuple[str, str]],
        leases_json: str,
        bandwidth_json: str | None,
        voxel_robots: frozenset[str] | None,
    ) -> str:
        robots = ",".join(
            f"{json.dumps(robot_id)}:{full if voxel_robots is None or robot_id in voxel_robots else light}"
            for robot_id, (full, light) in blobs.items()
        )
        message = '{"type":"fleet","ts":' + ts + ',"robots":{' + robots + '},"leases":' + leases_json
        if bandwidth_json is not None:
            message += ',"bandwidth":' + bandwidth_json
        return message + "}"

    # ------------------------------------------------------------------ status

    def health(self) -> dict[str, Any]:
        """Engine status for ``GET /health``."""
        leases = self.leases.snapshot()
        connected = sorted(self.hub.agents)
        return {
            "status": "ok",
            "service": "sax-bridge",
            "version": __version__,
            "uptime_s": round(time.time() - self.started_at, 1),
            "broadcast_hz": self.settings.broadcast_hz,
            "robots": {"connected": len(connected), "known": len(self.hub.known_robots()), "connected_ids": connected},
            "operators": {
                "connected": len({viewer.operator_id for viewer in self.hub.viewers.values()}),
                "connections": len(self.hub.viewers),
            },
            "leases": {"active": len(leases), "items": leases},
            "pending_estops": sorted(self.hub.pending_estops),
            "bandwidth": self.bandwidth.snapshot(),
            "audit": {"events": len(self.audit), "path": str(self.audit.path)},
        }
