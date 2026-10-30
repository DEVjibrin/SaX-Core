"""Tests for scripts/run_agent_cli.py. Run with `python3 test/test_cli.py` or pytest; neither needs websockets or ROS."""
import asyncio
import importlib.util
import math
import sys
import time
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "run_agent_cli.py"
spec = importlib.util.spec_from_file_location("run_agent_cli", SCRIPT)
cli = importlib.util.module_from_spec(spec)
sys.modules["run_agent_cli"] = cli
spec.loader.exec_module(cli)
cli._quiet = True

# Shared with test/test_sx1_encoder.cpp: both encoders must produce this exact line.
GOLDEN = "SX1|03|42|S1|120,-45,0,0,0,0.3827,0.9239|-60.-40.0;-59.-40.1"


def test_encoder_matches_the_cpp_golden_line():
    pose = cli.Pose(1.2, -0.45, 0.0, 0.0, 0.0, 0.3827, 0.9239)
    encoded = cli.encode_frame("03", 42, "S", 1, pose, [(-60, -40, 0), (-59, -40, 1)])
    assert encoded.line == GOLDEN and encoded.voxels_encoded == 2 and encoded.voxels_dropped == 0


def test_encoder_formatting_edge_cases():
    assert cli.encode_frame("r1", 0, "A", 0, cli.Pose(0, 0, 0, 0, 0, 0, 0), []).line == "SX1|r1|0|A|0,0,0,0,0,0,1|"
    tiny = cli.Pose(-0.001, 0.004, 12.345, -0.00001, 0.0, 0.0, 1.0)
    assert cli.encode_frame("r1", 7, "H", 0, tiny, []).line == "SX1|r1|7|H|0,0,1235,0,0,0,1|"
    nan = float("nan")
    broken = cli.Pose(nan, math.inf, 0.0, nan, 0.0, 0.0, 1.0)
    assert cli.encode_frame("r1", 1, "A", 0, broken, []).line == "SX1|r1|1|A|0,0,0,0,0,0,1|"
    assert cli.encode_frame("r1", 0, "S", 0, cli.Pose(), []).line == "SX1|r1|0|S|0,0,0,0,0,0,1|"
    assert cli.encode_frame("r1", 0, "S", 150, cli.Pose(), []).line == "SX1|r1|0|S99|0,0,0,0,0,0,1|"
    assert cli.encode_frame("r1", 0, "A", 5, cli.Pose(), []).line == "SX1|r1|0|A|0,0,0,0,0,0,1|"


def test_byte_budget_truncates_whole_cells():
    voxels = [(n, -n, 3) for n in range(100)]
    header = len(cli.encode_frame("r1", 0, "A", 0, cli.Pose(), []).line)
    encoded = cli.encode_frame("r1", 0, "A", 0, cli.Pose(), voxels, header + 30)
    assert len(encoded.line) <= header + 30
    assert encoded.voxels_encoded > 0 and encoded.voxels_dropped > 0 and encoded.voxels_encoded + encoded.voxels_dropped == 100
    assert "|0.0.3;1.-1.3;2.-2.3" in encoded.line and ";;" not in encoded.line and encoded.line.endswith("3")


def test_voxelizer_matches_the_cpp_expectations():
    points = [
        (1.01, 0.01, 0.01), (1.05, 0.05, 0.05),
        (2.01, 0.00, 0.01), (2.05, 0.00, 0.05),
        (0.51, 0.00, 0.01), (0.55, 0.00, 0.05),
        (3.01, 0.01, 0.01),
        (0.10, 0.10, 0.10), (0.12, 0.12, 0.12),
        (9.00, 0.00, 0.00), (9.00, 0.00, 0.00),
        (2.01, 0.00, 5.00), (2.02, 0.00, 5.00),
        (float("nan"), 0.0, 0.0),
    ]
    assert cli.voxelize(points, (0.0, 0.0, 0.0)) == [(2, 0, 0), (5, 0, 0), (10, 0, 0)]
    capped = cli.voxelize(points, (0.0, 0.0, 0.0), cli.VoxelConfig(max_voxels=2))
    assert capped == [(2, 0, 0), (5, 0, 0)]
    assert cli.voxelize([], (0.0, 0.0, 0.0)) == []
    assert cli.voxelize([(0.6, 0.0, -0.01)] * 2, (5.0, 5.0, 0.0), cli.VoxelConfig(min_range_m=0.0)) == [(3, 0, -1)]


def test_command_parsing_matches_the_cpp_rules():
    parse = cli.parse_command
    assert parse("SXE|robot_01|jibrin", "robot_01").command == cli.EStop("jibrin")
    assert parse("SXE|*|maya", "robot_01").command == cli.EStop("maya")
    assert parse("  SXE|robot_01|ann\r\n", "robot_01").command == cli.EStop("ann")
    for line in ("SXE|robot_01", "SXE|robot_01|bad op!", "SXE|robot_01||", "SXE||ann", "SXE", "SXE|robot_01|ann|extra"):
        assert isinstance(parse(line, "robot_01").command, cli.EStop), line
    assert parse("SXE|robot_01|bad op!", "robot_01").command == cli.EStop("unknown", operator_valid=False)
    assert parse("SXE|robot_02|ann", "robot_01").command is None

    trajectory = parse("SXP|robot_01|0,0;100,50;250,-75", "robot_01").command
    assert trajectory == cli.Trajectory(((0.0, 0.0), (1.0, 0.5), (2.5, -0.75)))
    too_many = "SXP|robot_01|0,0" + ";1,1" * 256
    for line in ("SXP|robot_01", "SXP|robot_01|", "SXP|robot_01|1,2", "SXP|robot_01|1,2;3", "SXP|robot_01|1,2;a,b",
                 "SXP|robot_01|1.5,2;3,4", "SXP|robot_01|1,2;3,4;", "SXP|robot_01|0,0;2000000,0",
                 "SXP|robot_02|0,0;1,1", "SXP|robot_01|0,0;1,1|extra", "SXP|robot_01|+1,2;3,4", too_many):
        result = parse(line, "robot_01")
        assert result.command is None and result.error, line

    assert parse("SXR|robot_01|ann", "robot_01").command == cli.Release("ann")
    for line in ("SXR|robot_01", "SXR|robot_02|ann", "", "   \r\n", "SXZ|robot_01|ann", "x" * 5000):
        assert parse(line, "robot_01").command is None, line


def test_the_simulated_lidar_voxelizes_and_blockage_blinds_it():
    sim = cli.Sim("robot_01", blockage_delay=1.0, blockage_duration=2.0)
    visible = sim.frame(0.5)
    assert visible.voxels_encoded > 50 and not visible.line.endswith("|")
    assert sim.frame(1.5).line.endswith("|") and sim.frame(1.5).voxels_encoded == 0
    assert sim.frame(3.5).voxels_encoded > 50
    assert sim.frame(31.5).voxels_encoded == 0
    assert sim.frame(34.5).voxels_encoded > 50


def test_estop_halts_and_auto_releases():
    sim = cli.Sim("robot_01", estop_hold=2.0)
    sim.step(0.1, 0.1)
    start = (sim.x, sim.y)
    sim.estop("ann", 1.0)
    assert sim.state() == ("H", 0) and "|H|" in sim.frame(1.0).line
    sim.step(0.5, 1.5)
    assert (sim.x, sim.y) == start
    sim.step(0.5, 3.0)
    assert sim.state() == ("A", 0)
    assert not sim.release("ann")


def test_stuck_incident_clears_when_a_recovery_path_arrives():
    sim = cli.Sim("robot_01", stuck_after=1.0)
    sim.step(0.1, 0.5)
    assert sim.state() == ("A", 0)
    sim.step(0.1, 1.5)
    assert sim.state() == ("S", 1) and "|S1|" in sim.frame(1.5).line
    sim.follow_path([(0.0, 0.0), (1.0, 1.0)])
    assert sim.state() == ("A", 0)


def test_backoff_and_url_helpers():
    cli.BACKOFF_JITTER = 0.0
    cap = cli.BACKOFF_CAP_S
    assert [cli.backoff_delay(n) for n in range(5)] == [0.5, 1.0, 2.0, 4.0, 8.0]
    assert cli.backoff_delay(40) == cap
    cli.BACKOFF_JITTER = 0.2
    assert 0.4 <= cli.backoff_delay(0) <= 0.6
    assert cli.build_url("ws://h:8000", "r1") == "ws://h:8000/ws/agent/r1"
    assert cli.build_url("ws://h:8000/", "r1", "tok") == "ws://h:8000/ws/agent/r1?token=tok"
    assert cli.build_url("ws://h/ws/agent/zz", "r1") == "ws://h/ws/agent/zz"


class FakeClosed(Exception):
    pass


class FakeConnection:
    """Stand-in for a websockets client connection."""

    def __init__(self):
        self.sent = []
        self.times = []
        self.incoming = asyncio.Queue()
        self.closed = False

    async def send(self, text):
        if self.closed:
            raise FakeClosed("closed")
        self.sent.append(text)
        self.times.append(time.monotonic())

    def __aiter__(self):
        return self

    async def __anext__(self):
        item = await self.incoming.get()
        if item is None:
            raise StopAsyncIteration
        return item

    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        self.closed = True
        return False


async def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        await asyncio.sleep(0.005)
    return predicate()


def test_network_loop_prioritises_estop_and_reconnects():
    cli.BACKOFF_BASE_S, cli.BACKOFF_JITTER, cli.STABLE_AFTER_S = 0.05, 0.0, 60.0
    connections = []

    def fake_connect(uri, **kwargs):
        connection = FakeConnection()
        connections.append((uri, connection))
        return connection

    args = cli.parse_args(["--robot-id", "robot_01", "--bridge-url", "ws://bridge.test", "--frequency", "5",
                           "--token", "tok", "--quiet"])

    async def scenario():
        task = asyncio.create_task(cli.run(args, fake_connect, (FakeClosed, OSError)))
        assert await wait_for(lambda: connections and len(connections[0][1].sent) >= 2)
        uri, first = connections[0]
        assert uri == "ws://bridge.test/ws/agent/robot_01?token=tok"
        assert first.sent[0].startswith("SX1|robot_01|0|A|") and first.sent[1].startswith("SX1|robot_01|1|A|")

        before = len(first.sent)
        injected_at = time.monotonic()
        await first.incoming.put("SXE|robot_01|ann")
        assert await wait_for(lambda: any("|H|" in line for line in first.sent[before:]))
        halted_index = next(n for n in range(before, len(first.sent)) if "|H|" in first.sent[n])
        assert first.times[halted_index] - injected_at < 0.1, "E-stop must not wait for the next 200 ms tick"
        later = len(first.sent)
        assert await wait_for(lambda: len(first.sent) > later + 1)
        assert all("|H|" in line for line in first.sent[halted_index:])

        await first.incoming.put("SXP|robot_01|0,0;100,0")
        await first.incoming.put("garbage|||")
        await first.incoming.put(None)
        assert await wait_for(lambda: len(connections) == 2), "should reconnect after the bridge closes the link"
        assert await wait_for(lambda: len(connections[1][1].sent) >= 1)
        assert connections[1][1].sent[0].startswith("SX1|robot_01|")
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    asyncio.run(asyncio.wait_for(scenario(), 15))


def main():
    tests = sorted((name, fn) for name, fn in globals().items() if name.startswith("test_") and callable(fn))
    for name, fn in tests:
        fn()
        print(f"PASS {name}")


if __name__ == "__main__":
    main()
