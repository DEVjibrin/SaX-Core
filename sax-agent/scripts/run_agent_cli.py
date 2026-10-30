#!/usr/bin/env python3
"""sax-agent CLI: a ROS-free stand-in for the C++ agent, for testing sax-bridge and sax-view locally.

It simulates an AMR patrolling a warehouse, scans it with a 360 degree LiDAR, voxelizes the scan exactly like
the C++ daemon does, and streams SX1 frames to the bridge. It honours the bridge's commands too: SXE (E-stop,
acted on immediately), SXP (recovery trajectory) and SXR (release).

    python scripts/run_agent_cli.py --robot-id robot_01 --bridge-url ws://localhost:8000 --frequency 10
    python scripts/run_agent_cli.py --simulate-blockage 15     # LiDAR goes blind at 15 s, for 6 s, every 30 s
    python scripts/run_agent_cli.py --stuck-after 20           # report an incident until a recovery path arrives

Requires the `websockets` package (pip install websockets).
"""
from __future__ import annotations

import argparse
import asyncio
import dataclasses
import math
import os
import random
import re
import sys
import time
from collections import Counter
from typing import Any, Awaitable, Callable, NamedTuple, Sequence

ROBOT_ID_RE = re.compile(r"[A-Za-z0-9_-]{1,32}")
OPERATOR_ID_RE = re.compile(r"[A-Za-z0-9_.@-]{1,64}")
INTEGER_RE = re.compile(r"-?[0-9]+")

GRID_MIN, GRID_MAX = -32768, 32767
MAX_WAYPOINTS = 256
MAX_COORDINATE_CM = 1_000_000
BLOCKAGE_PERIOD_S = 30.0

# Reconnect policy. Module-level so tests can shorten it.
BACKOFF_BASE_S = 0.5
BACKOFF_CAP_S = 30.0
BACKOFF_JITTER = 0.2
STABLE_AFTER_S = 5.0

Voxel = tuple[int, int, int]
Point = tuple[float, float, float]

_quiet = False


def log(message: str) -> None:
    if not _quiet:
        print(message, flush=True)


# ----------------------------------------------------------------------------------------------- SX1 encoder


@dataclasses.dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


class Encoded(NamedTuple):
    line: str
    voxels_encoded: int
    voxels_dropped: int


def _round_half_away(value: float) -> int:
    """Matches C++ std::llround."""
    magnitude = abs(value)
    whole = math.floor(magnitude)
    rounded = whole + (1 if magnitude - whole >= 0.5 else 0)
    return -rounded if value < 0 else rounded


def _centimetres(metres: float) -> int:
    cm = metres * 100.0
    if not math.isfinite(cm) or abs(cm) > 9.0e15:
        return 0
    return _round_half_away(cm)


def _fraction(value: float) -> str:
    """Fixed 4 decimals with trailing zeros trimmed: 0.3827, -0.5, 1, 0."""
    if not math.isfinite(value):
        value = 0.0
    text = f"{value:.4f}"
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return "0" if text == "-0" else text


def _normalized(pose: Pose) -> tuple[float, float, float, float]:
    norm = math.sqrt(pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw)
    if not math.isfinite(norm) or norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (pose.qx / norm, pose.qy / norm, pose.qz / norm, pose.qw / norm)


def encode_frame(
    robot_id: str,
    seq: int,
    state: str,
    reason: int,
    pose: Pose,
    voxels: Sequence[Voxel],
    max_bytes: int = 0,
) -> Encoded:
    """Encode one SX1 line. Voxel cells are appended while the line stays within ``max_bytes`` (0 = unlimited)."""
    stuck_reason = str(min(reason, 99)) if state == "S" and reason > 0 else ""
    quaternion = ",".join(_fraction(c) for c in _normalized(pose))
    position = ",".join(str(_centimetres(c)) for c in (pose.x, pose.y, pose.z))
    line = f"SX1|{robot_id}|{seq}|{state}{stuck_reason}|{position},{quaternion}|"

    encoded = 0
    for i, j, k in voxels:
        cell = f"{i}.{j}.{k}"
        separator = 1 if encoded else 0
        if max_bytes and len(line) + len(cell) + separator > max_bytes:
            break
        line += (";" if encoded else "") + cell
        encoded += 1
    return Encoded(line, encoded, len(voxels) - encoded)


# ------------------------------------------------------------------------------------------------ voxelizer


@dataclasses.dataclass(frozen=True)
class VoxelConfig:
    voxel_size_m: float = 0.20
    min_range_m: float = 0.30
    max_range_m: float = 8.0
    min_z_m: float = -0.30
    max_z_m: float = 2.50
    min_points_per_voxel: int = 2
    max_voxels: int = 1500


def voxelize(points: Sequence[Point], origin: Point, config: VoxelConfig = VoxelConfig()) -> list[Voxel]:
    """World-frame points to 20 cm grid cells, nearest-first. Same rules as the C++ VoxelQuantizer."""
    inverse = 1.0 / config.voxel_size_m
    min_r2 = config.min_range_m * config.min_range_m
    max_r2 = config.max_range_m * config.max_range_m
    ox, oy, oz = origin
    counts: Counter[Voxel] = Counter()
    for x, y, z in points:
        dx, dy, dz = x - ox, y - oy, z - oz
        r2 = dx * dx + dy * dy + dz * dz
        if not (min_r2 <= r2 <= max_r2) or not (config.min_z_m <= z <= config.max_z_m):
            continue
        i, j, k = math.floor(x * inverse), math.floor(y * inverse), math.floor(z * inverse)
        if GRID_MIN <= i <= GRID_MAX and GRID_MIN <= j <= GRID_MAX and GRID_MIN <= k <= GRID_MAX:
            counts[(i, j, k)] += 1

    size = config.voxel_size_m
    cells: list[tuple[float, Voxel]] = []
    for voxel, count in counts.items():
        if count >= max(1, config.min_points_per_voxel):
            cx = (voxel[0] + 0.5) * size - ox
            cy = (voxel[1] + 0.5) * size - oy
            cz = (voxel[2] + 0.5) * size - oz
            cells.append((cx * cx + cy * cy + cz * cz, voxel))
    cells.sort()
    return [voxel for _, voxel in cells[: config.max_voxels]]


# -------------------------------------------------------------------------------------------- inbound commands


@dataclasses.dataclass(frozen=True)
class EStop:
    operator_id: str
    operator_valid: bool = True


@dataclasses.dataclass(frozen=True)
class Release:
    operator_id: str


@dataclasses.dataclass(frozen=True)
class Trajectory:
    waypoints: tuple[tuple[float, float], ...]


@dataclasses.dataclass(frozen=True)
class ParseResult:
    command: EStop | Release | Trajectory | None
    error: str = ""


def parse_command(line: str, robot_id: str) -> ParseResult:
    """Parse one bridge command. E-stop is fail-safe: any SXE line stops this robot unless it names another."""
    line = line.strip(" \t\r\n")
    if not line:
        return ParseResult(None, "empty command")
    fields = line.split("|")
    verb = fields[0]

    if verb == "SXE":
        target = fields[1] if len(fields) > 1 else ""
        if target and target != "*" and target != robot_id:
            return ParseResult(None, "E-stop is addressed to another robot")
        operator = fields[2] if len(fields) > 2 else ""
        if OPERATOR_ID_RE.fullmatch(operator):
            return ParseResult(EStop(operator))
        return ParseResult(EStop("unknown", operator_valid=False))

    if verb == "SXR":
        if len(fields) < 3 or fields[1] != robot_id:
            return ParseResult(None, "SXR must be SXR|<this robot>|<operator>")
        if not OPERATOR_ID_RE.fullmatch(fields[2]):
            return ParseResult(None, "SXR operator id is malformed")
        return ParseResult(Release(fields[2]))

    if verb == "SXP":
        if len(fields) != 3:
            return ParseResult(None, "SXP must be SXP|<robot>|<x_cm>,<y_cm>;...")
        if fields[1] != robot_id:
            return ParseResult(None, "trajectory is addressed to another robot")
        points = fields[2].split(";")
        if not 2 <= len(points) <= MAX_WAYPOINTS:
            return ParseResult(None, "trajectory needs between 2 and 256 points")
        waypoints: list[tuple[float, float]] = []
        for point in points:
            axes = point.split(",")
            if len(axes) != 2 or not all(INTEGER_RE.fullmatch(axis) for axis in axes):
                return ParseResult(None, "trajectory points must be integer x_cm,y_cm pairs")
            x_cm, y_cm = int(axes[0]), int(axes[1])
            if abs(x_cm) > MAX_COORDINATE_CM or abs(y_cm) > MAX_COORDINATE_CM:
                return ParseResult(None, "trajectory coordinate is outside +/-10 km")
            waypoints.append((x_cm / 100.0, y_cm / 100.0))
        return ParseResult(Trajectory(tuple(waypoints)))

    return ParseResult(None, "unknown command verb")


# ------------------------------------------------------------------------------------------------- simulation

WALLS = (-12.0, -8.0, 12.0, 8.0)  # x0, y0, x1, y1 of the warehouse
RACKS = ((-11.0, -6.0, -10.0, 6.0), (10.0, -6.0, 11.0, 6.0))
PILLARS = ((-4.0, 1.0, 0.5), (4.0, 1.0, 0.5), (0.0, -1.0, 0.5), (9.0, 5.0, 0.5))  # x, y, radius
PATROL = ((-3.0, -4.5), (3.0, -4.5), (3.0, 4.5), (-3.0, 4.5))
SCAN_HEIGHTS_M = (0.1, 0.5, 0.9, 1.3)
SCAN_RANGE_M = 8.0
SPEED_MPS = 0.8

Segment = tuple[float, float, float, float]


def _box_segments(x0: float, y0: float, x1: float, y1: float) -> list[Segment]:
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def _ray_segment(ox: float, oy: float, dx: float, dy: float, segment: Segment) -> float | None:
    ax, ay, bx, by = segment
    ex, ey = bx - ax, by - ay
    denominator = dx * ey - dy * ex
    if abs(denominator) < 1e-12:
        return None
    t = ((ax - ox) * ey - (ay - oy) * ex) / denominator
    u = ((ax - ox) * dy - (ay - oy) * dx) / denominator
    return t if t >= 0.0 and 0.0 <= u <= 1.0 else None


def _ray_circle(ox: float, oy: float, dx: float, dy: float, circle: tuple[float, float, float]) -> float | None:
    cx, cy, radius = circle
    fx, fy = ox - cx, oy - cy
    b = dx * fx + dy * fy
    discriminant = b * b - (fx * fx + fy * fy - radius * radius)
    if discriminant < 0.0:
        return None
    t = -b - math.sqrt(discriminant)
    return t if t >= 0.0 else None


class Sim:
    """A robot patrolling a warehouse, with a LiDAR that can be blinded and incidents that need a recovery path."""

    def __init__(
        self,
        robot_id: str,
        *,
        voxel_config: VoxelConfig = VoxelConfig(),
        max_bytes: int = 0,
        stuck_after: float = 0.0,
        estop_hold: float = 15.0,
        blockage_delay: float | None = None,
        blockage_duration: float = 6.0,
        seed: int = 7,
    ) -> None:
        self.robot_id = robot_id
        self.voxel_config = voxel_config
        self.max_bytes = max_bytes
        self.stuck_after = stuck_after
        self.estop_hold = estop_hold
        self.blockage_delay = blockage_delay
        self.blockage_duration = blockage_duration
        self.rng = random.Random(seed)
        self.seq = 0
        self.x, self.y = PATROL[0]
        self.yaw = 0.0
        self.route: list[tuple[float, float]] = list(PATROL[1:]) + [PATROL[0]]
        self.halted = False
        self.halted_at = 0.0
        self.stuck = False
        self._stuck_fired = False
        self._segments = _box_segments(*WALLS) + [s for rack in RACKS for s in _box_segments(*rack)]

    # state ---------------------------------------------------------------------------------------------

    def state(self) -> tuple[str, int]:
        if self.halted:
            return "H", 0
        if self.stuck:
            return "S", 1
        return "A", 0

    def estop(self, operator_id: str, t: float) -> None:
        if not self.halted:
            self.halted_at = t
        self.halted = True
        log(f"E-STOP engaged by {operator_id}")

    def release(self, operator_id: str) -> bool:
        if not self.halted:
            return False
        self.halted = False
        log(f"E-stop released by {operator_id}")
        return True

    def follow_path(self, waypoints: Sequence[tuple[float, float]]) -> None:
        self.route = list(waypoints) + list(PATROL)
        if self.stuck:
            log("recovery path accepted, incident cleared")
        self.stuck = False

    def blocked(self, t: float) -> bool:
        if self.blockage_delay is None or t < self.blockage_delay:
            return False
        return (t - self.blockage_delay) % BLOCKAGE_PERIOD_S < self.blockage_duration

    # physics -------------------------------------------------------------------------------------------

    def step(self, dt: float, t: float) -> None:
        if self.halted and self.estop_hold > 0 and t - self.halted_at >= self.estop_hold:
            self.release("auto-release")
        if self.stuck_after > 0 and not self._stuck_fired and t >= self.stuck_after:
            self.stuck = self._stuck_fired = True
            log("simulated incident: path blocked")
        if self.halted or self.stuck or not self.route:
            return
        tx, ty = self.route[0]
        dx, dy = tx - self.x, ty - self.y
        distance = math.hypot(dx, dy)
        stride = SPEED_MPS * dt
        if distance <= stride:
            self.x, self.y = tx, ty
            self.route.pop(0)
            if not self.route:
                self.route = list(PATROL)
            return
        self.x += dx / distance * stride
        self.y += dy / distance * stride
        wanted = math.atan2(dy, dx)
        turn = math.atan2(math.sin(wanted - self.yaw), math.cos(wanted - self.yaw))
        self.yaw += turn * min(1.0, dt * 6.0)

    # sensing -------------------------------------------------------------------------------------------

    def lidar_points(self) -> list[Point]:
        segments = list(self._segments)
        if self.stuck:  # the pallet that is blocking the robot, one metre ahead
            cx, cy = self.x + math.cos(self.yaw), self.y + math.sin(self.yaw)
            segments += _box_segments(cx - 0.5, cy - 0.6, cx + 0.5, cy + 0.6)
        points: list[Point] = []
        for degrees in range(360):
            angle = math.radians(degrees)
            dx, dy = math.cos(angle), math.sin(angle)
            best = SCAN_RANGE_M
            for segment in segments:
                t = _ray_segment(self.x, self.y, dx, dy, segment)
                if t is not None and t < best:
                    best = t
            for circle in PILLARS:
                t = _ray_circle(self.x, self.y, dx, dy, circle)
                if t is not None and t < best:
                    best = t
            if best < SCAN_RANGE_M:
                hx, hy = self.x + dx * best, self.y + dy * best
                for z in SCAN_HEIGHTS_M:
                    points.append((hx + self.rng.gauss(0, 0.01), hy + self.rng.gauss(0, 0.01), z))
        return points

    def frame(self, t: float) -> Encoded:
        state, reason = self.state()
        points = [] if self.blocked(t) else self.lidar_points()
        voxels = voxelize(points, (self.x, self.y, 0.0), self.voxel_config)
        pose = Pose(self.x, self.y, 0.0, 0.0, 0.0, math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0))
        encoded = encode_frame(self.robot_id, self.seq, state, reason, pose, voxels, self.max_bytes)
        self.seq += 1
        return encoded


# ---------------------------------------------------------------------------------------------------- network


def backoff_delay(attempt: int, rng: random.Random | None = None) -> float:
    """Exponential delay with jitter; attempt 0 is the first retry."""
    unit = (rng or random).random()
    delay = min(BACKOFF_CAP_S, BACKOFF_BASE_S * (2.0 ** min(attempt, 20)))
    return min(BACKOFF_CAP_S, delay * (1.0 + BACKOFF_JITTER * (2.0 * unit - 1.0)))


def build_url(base: str, robot_id: str, token: str = "") -> str:
    url = base if "/ws/agent/" in base else f"{base.rstrip('/')}/ws/agent/{robot_id}"
    return f"{url}{'&' if '?' in url else '?'}token={token}" if token else url


class Counters:
    def __init__(self) -> None:
        self.frames = 0
        self.bytes = 0
        self.window_started = time.monotonic()
        self.window_bytes = 0

    def record(self, encoded: Encoded) -> None:
        self.frames += 1
        self.bytes += len(encoded.line)
        self.window_bytes += len(encoded.line)

    def take_kbps(self) -> float:
        now = time.monotonic()
        kbps = self.window_bytes * 8 / max(now - self.window_started, 1e-6) / 1000.0
        self.window_started, self.window_bytes = now, 0
        return kbps


async def session(ws: Any, sim: Sim, frequency: float, clock: Callable[[], float], counters: Counters) -> None:
    """Stream frames and serve commands on one connection until it ends. Raises when the link goes down."""
    send_lock = asyncio.Lock()

    async def send_frame() -> Encoded:
        encoded = sim.frame(clock())
        async with send_lock:
            await ws.send(encoded.line)
        counters.record(encoded)
        return encoded

    async def sender() -> None:
        period = 1.0 / frequency
        last = clock()
        next_at = time.monotonic()
        next_report = time.monotonic() + 1.0
        while True:
            now = clock()
            sim.step(min(now - last, 0.2), now)
            last = now
            encoded = await send_frame()
            if time.monotonic() >= next_report:
                next_report += 1.0
                state, reason = sim.state()
                log(
                    f"[{now:7.1f}s] state={state}{reason or ''} voxels={encoded.voxels_encoded}"
                    f" frame={len(encoded.line)}B rate={counters.take_kbps():.1f} kbps"
                )
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                next_at = time.monotonic()

    async def handle(line: str) -> None:
        result = parse_command(line, sim.robot_id)
        command = result.command
        if command is None:
            log(f"rejected command: {result.error}")
        elif isinstance(command, EStop):
            sim.estop(command.operator_id, clock())  # act first,
            await send_frame()  # then report H without waiting for the next tick
        elif isinstance(command, Release):
            if sim.release(command.operator_id):
                await send_frame()
        elif sim.halted:
            log("trajectory rejected while the E-stop is latched")
        else:
            log(f"trajectory accepted: {len(command.waypoints)} points")
            sim.follow_path(command.waypoints)

    async def receiver() -> None:
        async for message in ws:
            text = message if isinstance(message, str) else message.decode("utf-8", "replace")
            for line in text.splitlines():
                await handle(line)

    tasks = [asyncio.create_task(sender()), asyncio.create_task(receiver())]
    try:
        done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
    for task in done:
        if not task.cancelled() and task.exception() is not None:
            raise task.exception()  # type: ignore[misc]
    raise ConnectionError("the bridge closed the connection")


async def run(
    args: argparse.Namespace,
    connect: Callable[..., Any],
    retryable: tuple[type[BaseException], ...],
) -> None:
    """Connect, stream, and reconnect with exponential backoff until cancelled."""
    uri = build_url(args.bridge_url, args.robot_id, args.token)
    sim = Sim(
        args.robot_id,
        max_bytes=int(args.max_kbps * 1000 / 8 / args.frequency) if args.max_kbps > 0 else 0,
        stuck_after=args.stuck_after,
        estop_hold=args.estop_hold,
        blockage_delay=args.simulate_blockage,
        blockage_duration=args.blockage_duration,
        seed=args.seed,
    )
    started = time.monotonic()
    counters = Counters()
    failures = 0
    shown = uri.split("?")[0]
    while True:
        opened_at: float | None = None
        try:
            async with connect(uri, ping_interval=10, ping_timeout=10, max_size=2**20) as ws:
                opened_at = time.monotonic()
                log(f"connected to {shown}")
                await session(ws, sim, args.frequency, lambda: time.monotonic() - started, counters)
        except retryable as error:
            log(f"link down: {error!r}")
        stable = opened_at is not None and time.monotonic() - opened_at >= STABLE_AFTER_S
        failures = 0 if stable else failures + 1
        delay = backoff_delay(max(failures - 1, 0))
        log(f"reconnecting in {delay:.1f}s")
        await asyncio.sleep(delay)


def load_websockets() -> tuple[Callable[..., Any], tuple[type[BaseException], ...]]:
    try:
        import websockets
        from websockets.exceptions import WebSocketException
    except ImportError as error:  # pragma: no cover - depends on the environment
        raise SystemExit("this tool needs the websockets package: pip install websockets") from error
    return websockets.connect, (OSError, asyncio.TimeoutError, WebSocketException)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Simulated sax-agent for testing sax-bridge without ROS.")
    parser.add_argument("--robot-id", default="robot_01", help="1-32 characters of [A-Za-z0-9_-] (default: robot_01)")
    parser.add_argument("--bridge-url", default="ws://localhost:8000", help="bridge base URL; /ws/agent/<robot-id> is appended")
    parser.add_argument("--frequency", type=float, default=10.0, help="frames per second (default: 10)")
    parser.add_argument(
        "--simulate-blockage",
        nargs="?",
        type=float,
        const=10.0,
        default=None,
        metavar="DELAY_S",
        help="blind the LiDAR after DELAY_S seconds (default 10), then every 30 s",
    )
    parser.add_argument("--blockage-duration", type=float, default=6.0, help="seconds the LiDAR stays blind (default: 6)")
    parser.add_argument("--stuck-after", type=float, default=0.0, help="report a stuck incident after N seconds (0 = never)")
    parser.add_argument("--estop-hold", type=float, default=15.0, help="seconds before a simulated operator releases an E-stop (0 = wait for SXR)")
    parser.add_argument("--max-kbps", type=float, default=900.0, help="SX1 payload budget before compression (default: 900)")
    parser.add_argument("--token", default=os.environ.get("SAX_AGENT_TOKEN", ""), help="bridge token (default: $SAX_AGENT_TOKEN)")
    parser.add_argument("--seed", type=int, default=7, help="random seed for LiDAR noise")
    parser.add_argument("--quiet", action="store_true", help="only print errors")
    args = parser.parse_args(argv)

    if not ROBOT_ID_RE.fullmatch(args.robot_id):
        parser.error("--robot-id must be 1-32 characters of [A-Za-z0-9_-]")
    if not 0.0 < args.frequency <= 50.0:
        parser.error("--frequency must be between 0 and 50 Hz")
    if args.blockage_duration <= 0.0 or args.blockage_duration >= BLOCKAGE_PERIOD_S:
        parser.error("--blockage-duration must be positive and shorter than 30 s")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    global _quiet
    args = parse_args(argv)
    _quiet = args.quiet
    connect, retryable = load_websockets()
    try:
        asyncio.run(run(args, connect, retryable))
    except KeyboardInterrupt:
        log("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
