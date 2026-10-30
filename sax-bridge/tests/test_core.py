"""Dependency-free tests for the SX1 parser, lease manager, audit store, and bandwidth math."""
import json
import math
import tempfile
from pathlib import Path

from app.audit_store import AuditStore
from app.lease_manager import LeaseManager
from app.telemetry import BandwidthTracker, LidarMonitor, RobotState, SX1ParseError, encode_spline, parse_sx1

LINE = "SX1|03|42|S1|120,-45,0,0,0,0.3827,0.9239|-60.-40.0;-59.-40.1"


class Clock:
    now = 0.0

    def __call__(self) -> float:
        return self.now


def expect_error(line: str) -> None:
    try:
        parse_sx1(line)
    except SX1ParseError:
        return
    raise AssertionError(f"expected a parse error for {line!r}")


def test_parse_valid_frame() -> None:
    frame = parse_sx1(LINE)
    assert frame.robot_id == "03" and frame.seq == 42
    assert frame.state is RobotState.STUCK and frame.reason == 1
    assert frame.voxels == [-60, -40, 0, -59, -40, 1] and frame.voxel_count == 2
    assert abs(frame.pose.x - 1.2) < 1e-9 and abs(frame.pose.y + 0.45) < 1e-9
    assert abs(frame.pose.yaw - math.radians(45)) < 1e-3
    assert frame.wire_bytes == len(LINE)


def test_parse_empty_voxels_and_states() -> None:
    frame = parse_sx1("SX1|r7|0|H|0,0,0,0,0,0,1|")
    assert frame.state is RobotState.HALTED and frame.voxels == [] and frame.reason is None
    assert parse_sx1("SX1|r7|1|A|0,0,0,0,0,0,1|").state is RobotState.ACTIVE


def test_parse_rejects_bad_input() -> None:
    for bad in (
        "",
        "SX2|03|1|A|0,0,0,0,0,0,1|",
        "SX1|03|1|A|0,0,0|",
        "SX1|03|-1|A|0,0,0,0,0,0,1|",
        "SX1|03|1|X|0,0,0,0,0,0,1|",
        "SX1|03|1|A|0,0,0,0,0,0,0|",
        "SX1|03|1|A|0,0,0,0,0,0,nan|",
        "SX1|03|1|A|0,0,0,0,0,0,1|1.2;3.4.5",
        "SX1|03|1|A|0,0,0,0,0,0,1|99999.0.0",
        "SX1|bad id|1|A|0,0,0,0,0,0,1|",
    ):
        expect_error(bad)


def test_lease_exclusive_expiry_and_estop_bypass() -> None:
    clock = Clock()
    leases = LeaseManager(30, clock=clock, wall_clock=lambda: 1000.0)
    assert leases.acquire("03", "ann").created
    denied = leases.acquire("03", "bob")
    assert not denied.granted and denied.holder == "ann"
    assert not leases.authorize("03", "bob").granted
    bypass = leases.authorize_estop("03", "bob")
    assert bypass.granted and bypass.reason == "bypassed_lease"
    clock.now = 29
    assert leases.renew("03", "ann")
    clock.now = 58
    assert leases.authorize("03", "ann").granted
    clock.now = 100
    assert [lease.robot_id for lease in leases.expire()] == ["03"]
    assert leases.acquire("03", "bob").granted
    assert leases.release("03", "bob") and leases.snapshot() == []


def test_audit_chain_cap_and_reload() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "audit.json"
        store = AuditStore(path, max_events=3, flush_interval_s=60)
        for index in range(5):
            store.record("e_stop", robot_id=str(index), detail={"n": index})
        store.flush()
        assert len(store) == 3 and store.verify()
        reloaded = AuditStore(path, max_events=3, flush_interval_s=60)
        assert [event["robot_id"] for event in reloaded.snapshot()] == ["2", "3", "4"]
        reloaded.record("lease_acquired")
        assert reloaded.verify() and reloaded.snapshot()[-1]["seq"] == 6
        store._events[0]["type"] = "tampered"
        assert not store.verify()
        store.close()
        reloaded.close()
        assert isinstance(json.loads(path.read_text()), list)


def test_bandwidth_and_lidar_monitor() -> None:
    tracker = BandwidthTracker(raw_points_per_frame=30_000, raw_bytes_per_point=16)
    for tick in range(50):
        tracker.record("03", 2700, now=tick * 0.1)
    fleet = tracker.snapshot(now=5.0)["fleet"]
    assert fleet["raw_estimate_mbps"] > 30 and fleet["sx1_kbps"] < 1000 and fleet["savings_pct"] > 99

    monitor = LidarMonitor(zero_frames=3)
    empty = parse_sx1("SX1|03|1|A|0,0,0,0,0,0,1|")
    results = [monitor.observe(empty) for _ in range(4)]
    assert results == [None, None, "detected", None] and monitor.is_blocked("03")
    assert monitor.observe(parse_sx1("SX1|03|2|A|0,0,0,0,0,0,1|1.1.1")) == "cleared"


def test_encode_spline() -> None:
    assert encode_spline("03", [(0.0, 0.0), (1.234, -0.5)]) == "SXP|03|0,0;123,-50"
