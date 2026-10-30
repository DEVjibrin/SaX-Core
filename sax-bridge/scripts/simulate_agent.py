"""Simulated sax-agent: streams SX1 frames to a running sax-bridge and prints the commands it receives.

Usage: python scripts/simulate_agent.py --robot 03 [--token SECRET] [--block-after 20]
"""
from __future__ import annotations

import argparse
import asyncio
import math
import time

import websockets

VOXEL_CM = 20


def build_frame(robot_id: str, seq: int, elapsed: float, state: str, blocked: bool) -> str:
    """Robot driving a 4 m circle, surrounded by a ring of voxels (empty while ``blocked``)."""
    angle = elapsed * 0.25
    x, y = 400 * math.cos(angle), 400 * math.sin(angle)
    yaw = angle + math.pi / 2
    pose = f"{x:.0f},{y:.0f},0,0,0,{math.sin(yaw / 2):.4f},{math.cos(yaw / 2):.4f}"
    cells: list[str] = []
    if not blocked:
        cx, cy = round(x / VOXEL_CM), round(y / VOXEL_CM)
        for step in range(24):
            theta = step * math.tau / 24
            i, j = cx + round(15 * math.cos(theta)), cy + round(15 * math.sin(theta))
            cells.extend(f"{i}.{j}.{k}" for k in range(3))
    return f"SX1|{robot_id}|{seq}|{state}|{pose}|{';'.join(cells)}"


async def listen(ws: websockets.WebSocketClientProtocol, shared: dict[str, str]) -> None:
    async for message in ws:
        text = message if isinstance(message, str) else message.decode("utf-8", "replace")
        print(f"<- {text[:140]}")
        if text.startswith("SXE|"):
            shared["state"] = "H"


async def run(args: argparse.Namespace) -> None:
    uri = f"{args.url.rstrip('/')}/ws/agent/{args.robot}" + (f"?token={args.token}" if args.token else "")
    shared = {"state": "A"}
    async with websockets.connect(uri) as ws:
        print(f"connected to {uri}")
        listener = asyncio.create_task(listen(ws, shared))
        started = time.monotonic()
        seq = 0
        try:
            while True:
                elapsed = time.monotonic() - started
                blocked = 0 < args.block_after <= elapsed < args.block_after + 5
                await ws.send(build_frame(args.robot, seq, elapsed, shared["state"], blocked))
                seq += 1
                await asyncio.sleep(1 / args.hz)
        finally:
            listener.cancel()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://localhost:8000")
    parser.add_argument("--robot", default="03")
    parser.add_argument("--token", default="")
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--block-after", type=float, default=0.0, help="seconds before a 5 s LiDAR blockage (0 = never)")
    args = parser.parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass
    except (OSError, websockets.ConnectionClosed) as exc:
        print(f"connection ended: {exc}")


if __name__ == "__main__":
    main()
