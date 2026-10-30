"""SX1 protocol parsing, LiDAR blockage monitoring, and bandwidth accounting.

Wire format, one robot frame per line::

    SX1|<robot_id>|<seq>|<state>|<x>,<y>,<z>,<qx>,<qy>,<qz>,<qw>|<i>.<j>.<k>;<i>.<j>.<k>;...

* ``state`` is ``A`` (active), ``H`` (halted) or ``S<reason>`` (stuck, optional reason code).
* ``x,y,z`` are centimetres in the map frame. The quaternion is normalised on receipt, so integer-scaled
  and floating-point components are both accepted.
* Voxels are integer grid cells (``i.j.k``). An empty field means the point cloud is empty.
"""
from __future__ import annotations

import math
import re
import threading
import time
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from typing import Any, Final

from .ids import ROBOT_ID_RE

MAX_VOXELS: Final = 4096
POSITION_UNIT_M: Final = 0.01
_MONTH_S: Final = 30 * 24 * 3600
_STATE_RE: Final = re.compile(r"([ASH])(\d{1,2})?")
_VOXELS_RE: Final = re.compile(r"-?\d+\.-?\d+\.-?\d+(?:;-?\d+\.-?\d+\.-?\d+)*")
_MAX_VOXEL_CHARS: Final = MAX_VOXELS * 21


class SX1ParseError(ValueError):
    """Raised when a line is not a valid SX1 frame."""


class RobotState(str, Enum):
    ACTIVE = "active"
    STUCK = "stuck"
    HALTED = "halted"


_STATE_BY_CODE: Final = {"A": RobotState.ACTIVE, "S": RobotState.STUCK, "H": RobotState.HALTED}


@dataclass(frozen=True, slots=True)
class Pose:
    """Robot pose in metres and a unit quaternion, plus the derived yaw in radians."""

    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    yaw: float

    def to_dict(self) -> dict[str, float]:
        return {
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "z": round(self.z, 3),
            "qx": round(self.qx, 6),
            "qy": round(self.qy, 6),
            "qz": round(self.qz, 6),
            "qw": round(self.qw, 6),
            "yaw": round(self.yaw, 4),
        }


@dataclass(frozen=True, slots=True)
class RobotFrame:
    """One decoded SX1 frame. ``voxels`` is a flat ``[i, j, k, i, j, k, ...]`` list."""

    robot_id: str
    seq: int
    state: RobotState
    reason: int | None
    pose: Pose
    voxels: list[int]
    voxel_count: int
    wire_bytes: int
    received_at: float

    def to_dict(self, *, include_voxels: bool = True) -> dict[str, Any]:
        data: dict[str, Any] = {
            "robot_id": self.robot_id,
            "seq": self.seq,
            "state": self.state.value,
            "reason": self.reason,
            "pose": self.pose.to_dict(),
            "voxel_count": self.voxel_count,
            "wire_bytes": self.wire_bytes,
            "received_at": round(self.received_at, 3),
        }
        if include_voxels:
            data["voxels"] = self.voxels
        return data


def _parse_pose(text: str) -> Pose:
    fields = text.split(",")
    if len(fields) != 7:
        raise SX1ParseError("pose needs 7 fields: x,y,z,qx,qy,qz,qw")
    try:
        x, y, z, qx, qy, qz, qw = (float(field) for field in fields)
    except ValueError as exc:
        raise SX1ParseError("pose fields must be numbers") from exc
    if not all(math.isfinite(value) for value in (x, y, z, qx, qy, qz, qw)):
        raise SX1ParseError("pose fields must be finite")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        raise SX1ParseError("quaternion has zero length")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return Pose(x * POSITION_UNIT_M, y * POSITION_UNIT_M, z * POSITION_UNIT_M, qx, qy, qz, qw, yaw)


def _parse_voxels(text: str) -> list[int]:
    if not text:
        return []
    if len(text) > _MAX_VOXEL_CHARS:
        raise SX1ParseError(f"more than {MAX_VOXELS} voxels")
    if not _VOXELS_RE.fullmatch(text):
        raise SX1ParseError("voxels must look like i.j.k;i.j.k")
    values = [int(token) for token in text.replace(";", ".").split(".")]
    if len(values) > MAX_VOXELS * 3:
        raise SX1ParseError(f"more than {MAX_VOXELS} voxels")
    if min(values) < -32768 or max(values) > 32767:
        raise SX1ParseError("voxel indices must fit in int16")
    return values


def parse_sx1(line: str, *, received_at: float | None = None) -> RobotFrame:
    """Decode one SX1 line into a :class:`RobotFrame`, raising :class:`SX1ParseError` on bad input."""
    raw = line.strip()
    parts = raw.split("|")
    if len(parts) != 6 or parts[0] != "SX1":
        raise SX1ParseError("expected SX1|robot_id|seq|state|pose|voxels")
    _, robot_id, seq_text, state_text, pose_text, voxel_text = parts
    if not ROBOT_ID_RE.fullmatch(robot_id):
        raise SX1ParseError("malformed robot_id")
    if not (seq_text.isascii() and seq_text.isdigit() and len(seq_text) <= 12):
        raise SX1ParseError("sequence number must be a non-negative integer")
    state_match = _STATE_RE.fullmatch(state_text)
    if state_match is None:
        raise SX1ParseError("state must be A, H, or S<reason>")
    voxels = _parse_voxels(voxel_text)
    return RobotFrame(
        robot_id=robot_id,
        seq=int(seq_text),
        state=_STATE_BY_CODE[state_match.group(1)],
        reason=int(state_match.group(2)) if state_match.group(2) else None,
        pose=_parse_pose(pose_text),
        voxels=voxels,
        voxel_count=len(voxels) // 3,
        wire_bytes=len(raw.encode("utf-8")),
        received_at=time.time() if received_at is None else received_at,
    )


def encode_spline(robot_id: str, points: Sequence[tuple[float, float]]) -> str:
    """Encode recovery-path control points (metres) as an SXP command: ``SXP|robot|x_cm,y_cm;...``."""
    return f"SXP|{robot_id}|" + ";".join(f"{round(x * 100)},{round(y * 100)}" for x, y in points)


def path_length(points: Sequence[tuple[float, float]]) -> float:
    """Total polyline length in metres."""
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


class LidarMonitor:
    """Flags a LiDAR blockage when the voxel output is empty for N consecutive frames while not halted.

    Not thread-safe: call it from the event loop only.
    """

    def __init__(self, zero_frames: int = 3) -> None:
        self._limit = zero_frames
        self._zero_run: dict[str, int] = {}
        self._blocked: dict[str, bool] = {}

    def observe(self, frame: RobotFrame) -> str | None:
        """Return ``"detected"`` or ``"cleared"`` when the blockage state flips, otherwise ``None``."""
        empty = frame.state is not RobotState.HALTED and frame.voxel_count == 0
        run = self._zero_run.get(frame.robot_id, 0) + 1 if empty else 0
        self._zero_run[frame.robot_id] = run
        blocked = run >= self._limit
        if blocked == self._blocked.get(frame.robot_id, False):
            return None
        self._blocked[frame.robot_id] = blocked
        return "detected" if blocked else "cleared"

    def is_blocked(self, robot_id: str) -> bool:
        return self._blocked.get(robot_id, False)

    def reset(self, robot_id: str) -> None:
        self._zero_run.pop(robot_id, None)
        self._blocked.pop(robot_id, None)


@dataclass(slots=True)
class _Sample:
    at: float
    sx1: int
    raw: int


class BandwidthTracker:
    """Sliding-window comparison of SX1 payload bytes against an estimated raw LiDAR stream.

    The raw estimate assumes each frame replaces ``raw_points_per_frame`` points of ``raw_bytes_per_point``
    bytes (for example 30 000 points of XYZ + intensity as float32 is 480 kB per frame).
    """

    def __init__(self, *, raw_points_per_frame: int, raw_bytes_per_point: int, window_s: float = 5.0) -> None:
        self._raw_per_frame = raw_points_per_frame * raw_bytes_per_point
        self._window = window_s
        self._samples: dict[str, deque[_Sample]] = {}
        self._lock = threading.Lock()

    def record(self, robot_id: str, sx1_bytes: int, *, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        with self._lock:
            samples = self._samples.setdefault(robot_id, deque())
            samples.append(_Sample(now, sx1_bytes, self._raw_per_frame))
            self._trim(samples, now)

    def _trim(self, samples: deque[_Sample], now: float) -> None:
        while samples and now - samples[0].at > self._window:
            samples.popleft()

    @staticmethod
    def _describe(sx1_bps: float, raw_bps: float) -> dict[str, Any]:
        return {
            "sx1_kbps": round(sx1_bps / 1e3, 2),
            "raw_estimate_mbps": round(raw_bps / 1e6, 2),
            "savings_pct": round((1 - sx1_bps / raw_bps) * 100, 3) if raw_bps else None,
            "reduction_factor": round(raw_bps / sx1_bps, 1) if sx1_bps else None,
        }

    def snapshot(self, *, now: float | None = None) -> dict[str, Any]:
        """Per-robot and fleet-wide rates over the window, plus a 30-day data projection."""
        now = time.monotonic() if now is None else now
        robots: dict[str, dict[str, Any]] = {}
        total_sx1 = total_raw = 0.0
        with self._lock:
            for robot_id, samples in list(self._samples.items()):
                self._trim(samples, now)
                if not samples:
                    del self._samples[robot_id]
                    continue
                elapsed = max(1.0, min(self._window, now - samples[0].at))
                sx1_bps = sum(sample.sx1 for sample in samples) * 8 / elapsed
                raw_bps = sum(sample.raw for sample in samples) * 8 / elapsed
                total_sx1 += sx1_bps
                total_raw += raw_bps
                robots[robot_id] = self._describe(sx1_bps, raw_bps)
        fleet = self._describe(total_sx1, total_raw)
        fleet["projected_monthly_gb"] = {
            "sx1": round(total_sx1 / 8 * _MONTH_S / 1e9, 2),
            "raw_estimate": round(total_raw / 8 * _MONTH_S / 1e9, 2),
        }
        return {"window_s": self._window, "robots": robots, "fleet": fleet}
