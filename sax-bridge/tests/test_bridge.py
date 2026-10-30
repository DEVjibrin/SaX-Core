"""Orchestration tests with fake sockets: no network and no ASGI server needed."""
import asyncio
import json
import tempfile
from pathlib import Path
from typing import Any

from app.bridge import Bridge
from app.config import Settings
from app.lease_manager import LeaseManager
from app.telemetry import SX1ParseError


class FakeSocket:
    """Stand-in for a Starlette WebSocket that records everything sent to it."""

    def __init__(self) -> None:
        self.sent: list[str] = []
        self.closed_with: int | None = None

    async def send_text(self, text: str) -> None:
        self.sent.append(text)

    async def close(self, code: int = 1000, reason: str = "") -> None:
        self.closed_with = code

    def json(self) -> list[dict[str, Any]]:
        return [json.loads(text) for text in self.sent]


class Clock:
    now = 0.0

    def __call__(self) -> float:
        return self.now


def frame(seq: int, state: str = "A", voxels: str = "1.1.1;2.2.2") -> str:
    return f"SX1|03|{seq}|{state}|100,200,0,0,0,0,1|{voxels}"


async def scenario(tmp: str) -> None:
    bridge = Bridge(Settings(audit_path=str(Path(tmp) / "audit.json")))
    agent_socket, ann_socket, bob_socket = FakeSocket(), FakeSocket(), FakeSocket()

    agent = await bridge.register_agent("03", agent_socket)
    assert agent is not None
    assert await bridge.ingest(agent, frame(1))
    assert not await bridge.ingest(agent, frame(1)), "duplicate sequence numbers are dropped"
    try:
        await bridge.ingest(agent, "SX1|07|9|A|0,0,0,0,0,0,1|")
    except SX1ParseError:
        pass
    else:
        raise AssertionError("a robot must not be able to speak for another robot")

    ann = bridge.register_viewer("ann", ann_socket)
    bob = bridge.register_viewer("bob", bob_socket)
    await bridge.handle_operator_message(ann, {"type": "lease.acquire", "robot_id": "03", "id": 1})
    assert ann_socket.json()[-1]["ok"] is True
    await bridge.handle_operator_message(bob, {"type": "lease.acquire", "robot_id": "03", "id": 2})
    assert bob_socket.json()[-1]["code"] == "lease_denied"
    await bridge.handle_operator_message(bob, {"type": "lease.acquire", "robot_id": "99", "id": 3})
    assert bob_socket.json()[-1]["code"] == "unknown_robot"

    points = [[0, 0], [1, 1]]
    await bridge.handle_operator_message(bob, {"type": "trajectory", "robot_id": "03", "points": points, "id": 4})
    assert bob_socket.json()[-1]["code"] == "lease_required"
    await bridge.handle_operator_message(ann, {"type": "trajectory", "robot_id": "03", "points": [{"x": 0, "y": 0}, {"x": 1, "y": 0.5}]})
    assert agent_socket.sent[-1] == "SXP|03|0,0;100,50"
    await bridge.handle_operator_message(ann, {"type": "trajectory", "robot_id": "03", "points": [[0, 0], [float("inf"), 1]]})
    assert ann_socket.json()[-1]["code"] == "bad_points"

    await bridge.handle_operator_message(bob, {"type": "estop", "robot_id": "03", "id": 5})
    assert agent_socket.sent[-1] == "SXE|03|bob" and "03" in bridge.hub.pending_estops
    assert bridge.audit.snapshot(event_type="e_stop")[-1]["detail"]["bypassed_lease"] is True
    assert await bridge.ingest(agent, frame(2, "H"))
    assert "03" not in bridge.hub.pending_estops and bridge.audit.snapshot(event_type="e_stop_confirmed")

    await bridge._broadcast_fleet(include_bandwidth=True)
    fleet = ann_socket.json()[-1]
    assert fleet["type"] == "fleet" and fleet["robots"]["03"]["online"] is True
    assert fleet["robots"]["03"]["voxels"] == [1, 1, 1, 2, 2, 2] and fleet["leases"]["03"]["operator_id"] == "ann"
    assert "bandwidth" in fleet
    await bridge.handle_operator_message(bob, {"type": "subscribe", "robots": []})
    await bridge._broadcast_fleet(include_bandwidth=False)
    assert "voxels" not in bob_socket.json()[-1]["robots"]["03"]
    assert "voxels" in ann_socket.json()[-1]["robots"]["03"]

    for seq in range(3, 6):
        await bridge.ingest(agent, frame(seq, "A", ""))
    assert bridge.lidar.is_blocked("03") and bridge.audit.snapshot(event_type="lidar_blockage_detected")

    clock = Clock()
    bridge.leases = LeaseManager(30, clock=clock)
    bridge.leases.acquire("03", "ann")
    clock.now = 31
    await bridge._housekeeping()
    assert bridge.audit.snapshot(event_type="lease_expired")

    replacement_socket = FakeSocket()
    replacement = await bridge.register_agent("03", replacement_socket)
    assert replacement is not None and agent_socket.closed_with == 4000
    await bridge.unregister_agent(agent)
    assert bridge.hub.agents["03"] is replacement, "a superseded socket must not evict its replacement"

    await bridge.unregister_agent(replacement)
    result = await bridge.trigger_estop(operator_id="ann", robot_id="03", reason="offline test", source="rest")
    assert result["queued"] == ["03"] and result["delivered"] == []
    late_socket = FakeSocket()
    await bridge.register_agent("03", late_socket)
    assert late_socket.sent[0] == "SXE|03|ann", "a pending E-stop is delivered when the robot reconnects"

    assert bridge.audit.verify()
    health = bridge.health()
    assert health["robots"]["connected"] == 1 and health["status"] == "ok"
    await bridge.stop()


def test_bridge_end_to_end() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        asyncio.run(scenario(tmp))
