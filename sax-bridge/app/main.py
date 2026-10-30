"""FastAPI entry point: REST endpoints plus the agent and dashboard WebSockets.

Run with ``uvicorn app.main:app --host 0.0.0.0 --port 8000`` (one worker: state lives in memory).
"""
from __future__ import annotations

import json
import logging
import os
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from fastapi import Depends, FastAPI, HTTPException, Query, Request, WebSocket, WebSocketDisconnect, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from . import __version__
from .bridge import Bridge
from .config import Settings, load_settings
from .ids import OPERATOR_ID_FIELD, OPERATOR_ID_RE, ROBOT_ID_FIELD, ROBOT_ID_RE
from .security import bearer_or_query, origin_ok, token_ok
from .telemetry import SX1ParseError

log = logging.getLogger("sax.api")

MAX_CONSECUTIVE_BAD_FRAMES = 50


class EstopRequest(BaseModel):
    """Body for ``POST /api/v1/estop``. Omit ``robot_id`` to stop the whole fleet."""

    operator_id: str = Field(pattern=OPERATOR_ID_FIELD)
    robot_id: str | None = Field(default=None, pattern=ROBOT_ID_FIELD)
    reason: str = Field(default="manual e-stop", max_length=200)


async def read_frame(websocket: WebSocket) -> str | None:
    """Receive one text or binary frame as text. Returns ``None`` once the peer has disconnected."""
    message = await websocket.receive()
    if message["type"] == "websocket.disconnect":
        return None
    if message.get("text") is not None:
        return str(message["text"])
    return bytes(message.get("bytes") or b"").decode("utf-8", errors="replace")


def create_app(settings: Settings | None = None) -> FastAPI:
    """Build the application. Tests can pass custom :class:`Settings`."""
    cfg = settings or load_settings()
    logging.basicConfig(
        level=os.environ.get("SAX_LOG_LEVEL", "INFO").upper(),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    if cfg.agent_token is None or cfg.operator_token is None:
        log.warning("SAX_AGENT_TOKEN / SAX_OPERATOR_TOKEN not set: authentication is disabled (development only)")
    bridge = Bridge(cfg)

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        await bridge.start()
        try:
            yield
        finally:
            await bridge.stop()

    app = FastAPI(title="sax-bridge", version=__version__, lifespan=lifespan)
    app.state.bridge = bridge
    app.add_middleware(
        CORSMiddleware,
        allow_origins=list(cfg.cors_origins),
        allow_credentials=False,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["Authorization", "Content-Type", "X-API-Key"],
    )

    def require_api_key(request: Request) -> None:
        if not token_ok(cfg.operator_token, bearer_or_query(request.headers, request.query_params)):
            raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="invalid or missing token")

    @app.get("/health")
    async def health() -> dict:
        """Engine status, connected robots, and active operator leases."""
        return bridge.health()

    @app.get("/api/v1/audit", dependencies=[Depends(require_api_key)])
    async def export_audit(
        limit: int = Query(1000, ge=1, le=100_000),
        event_type: str | None = Query(None, max_length=64),
        robot_id: str | None = Query(None, pattern=ROBOT_ID_FIELD),
        since: float | None = Query(None, description="Unix timestamp; only newer events are returned"),
        download: bool = False,
    ) -> JSONResponse:
        """Export the black-box audit log as JSON, with hash-chain verification."""
        events = bridge.audit.snapshot(limit=limit, event_type=event_type, robot_id=robot_id, since=since)
        headers = {"Content-Disposition": 'attachment; filename="sax_audit_export.json"'} if download else None
        body = {"count": len(events), "chain_valid": bridge.audit.verify(), "events": events}
        return JSONResponse(body, headers=headers)

    @app.post("/api/v1/estop", dependencies=[Depends(require_api_key)])
    async def estop(payload: EstopRequest) -> dict:
        """Hardware E-Stop override: bypasses every lease and is broadcast to agents and dashboards."""
        return await bridge.trigger_estop(
            operator_id=payload.operator_id, robot_id=payload.robot_id, reason=payload.reason, source="rest"
        )

    @app.websocket("/ws/agent/{robot_id}")
    async def agent_socket(websocket: WebSocket, robot_id: str) -> None:
        """Edge robot channel: receives SX1 frames, sends back SXE (E-stop) and SXP (trajectory) commands."""
        token = bearer_or_query(websocket.headers, websocket.query_params)
        if not ROBOT_ID_RE.fullmatch(robot_id) or not token_ok(cfg.agent_token, token):
            await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
            return
        await websocket.accept()
        session = await bridge.register_agent(robot_id, websocket)
        if session is None:
            await websocket.close(code=status.WS_1013_TRY_AGAIN_LATER, reason="robot capacity reached")
            return

        bad_frames = 0
        reason = "disconnected"
        try:
            while True:
                raw = await read_frame(websocket)
                if raw is None:
                    break
                if len(raw) > cfg.max_message_bytes:
                    reason = "message too large"
                    await websocket.close(code=status.WS_1009_MESSAGE_TOO_BIG)
                    break
                for line in raw.splitlines():
                    if not line.strip():
                        continue
                    try:
                        await bridge.ingest(session, line)
                        bad_frames = 0
                    except SX1ParseError as exc:
                        bad_frames += 1
                        session.parse_errors += 1
                        if session.parse_errors % 100 == 1:
                            log.warning("agent %s sent a malformed frame (%d so far): %s", robot_id, session.parse_errors, exc)
                        if bad_frames >= MAX_CONSECUTIVE_BAD_FRAMES:
                            reason = "too many malformed frames"
                            await websocket.close(code=status.WS_1003_UNSUPPORTED_DATA, reason=reason)
                            return
        except WebSocketDisconnect:
            pass
        except Exception:
            reason = "internal error"
            log.exception("agent %s socket failed", robot_id)
        finally:
            await bridge.unregister_agent(session, reason)

    @app.websocket("/ws/view/{operator_id}")
    async def view_socket(websocket: WebSocket, operator_id: str) -> None:
        """Dashboard channel: 10 Hz fleet frames out, lease/trajectory/E-stop commands in."""
        token = bearer_or_query(websocket.headers, websocket.query_params)
        allowed = (
            OPERATOR_ID_RE.fullmatch(operator_id)
            and origin_ok(websocket.headers.get("origin"), cfg.cors_origins)
            and token_ok(cfg.operator_token, token)
        )
        if not allowed:
            await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
            return
        await websocket.accept()
        viewer = bridge.register_viewer(operator_id, websocket)
        try:
            await bridge.send_hello(viewer)
            while True:
                raw = await read_frame(websocket)
                if raw is None:
                    break
                if len(raw) > cfg.max_message_bytes:
                    await websocket.close(code=status.WS_1009_MESSAGE_TOO_BIG)
                    break
                try:
                    message = json.loads(raw)
                except json.JSONDecodeError:
                    await bridge.send_error(viewer, None, "bad_json", "message is not valid JSON")
                    continue
                if not isinstance(message, dict):
                    await bridge.send_error(viewer, None, "bad_message", "message must be a JSON object")
                    continue
                await bridge.handle_operator_message(viewer, message)
        except WebSocketDisconnect:
            pass
        except Exception:
            log.exception("viewer %s socket failed", operator_id)
        finally:
            bridge.unregister_viewer(viewer)

    return app


app = create_app()
