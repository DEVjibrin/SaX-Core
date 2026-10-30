import { ROBOT_IDS, type RobotId, type StuckReason, type Vec2 } from '../types/fleet';
import { DEFAULT_REASON, VOXEL_SIZE } from './fleetConfig';

export type SimState = 'A' | 'S' | 'H';

interface SimRobot {
  x: number;
  y: number;
  heading: number;
  seq: number;
  state: SimState;
  reason: StuckReason | null;
  blocked: boolean;
  held: { state: SimState; reason: StuckReason | null } | null;
  route: Vec2[];
  patrol: Vec2[];
}

const RANGE = 6;
const SPEED = 1;

const START: Record<RobotId, Vec2> = { '03': { x: -5, y: -1 }, '07': { x: 5, y: 0 } };
const PATROL: Record<RobotId, Vec2[]> = {
  '03': [{ x: -7, y: -4 }, { x: -2, y: -4 }, { x: -2, y: 3 }, { x: -7, y: 3 }],
  '07': [{ x: 3, y: -5 }, { x: 8, y: -5 }, { x: 8, y: 2 }, { x: 3, y: 2 }],
};

function buildWorld(): Int16Array {
  const cells: number[] = [];
  const add = (i: number, j: number, layers: number) => {
    for (let k = 0; k < layers; k += 1) cells.push(i, j, k);
  };
  for (let i = -60; i <= 60; i += 1) {
    add(i, -40, 3);
    add(i, 40, 3);
  }
  for (let j = -39; j < 40; j += 1) {
    add(-60, j, 3);
    add(60, j, 3);
  }
  for (const [ci, cj] of [[-25, 15], [30, -20], [0, -30], [15, 30]]) {
    for (let step = 0; step < 16; step += 1) {
      const angle = (step / 16) * Math.PI * 2;
      add(Math.round(ci + Math.cos(angle) * 3), Math.round(cj + Math.sin(angle) * 3), 3);
    }
  }
  for (let i = -10; i <= 10; i += 1) add(i, 8, 4);
  for (let i = 22; i <= 42; i += 1) add(i, -12, 4);
  return new Int16Array(cells);
}

const WORLD = buildWorld();

function createRobots(): Record<RobotId, SimRobot> {
  const robots = {} as Record<RobotId, SimRobot>;
  for (const id of ROBOT_IDS) {
    robots[id] = {
      ...START[id],
      heading: id === '03' ? 0.4 : Math.PI,
      seq: 0,
      state: 'S',
      reason: DEFAULT_REASON[id],
      blocked: false,
      held: null,
      route: [],
      patrol: PATROL[id],
    };
  }
  return robots;
}

function obstacle(robot: SimRobot): string[] {
  if (robot.state !== 'S' || robot.reason === 3) return [];
  const ci = Math.round((robot.x + Math.cos(robot.heading)) / VOXEL_SIZE);
  const cj = Math.round((robot.y + Math.sin(robot.heading)) / VOXEL_SIZE);
  const cells: string[] = [];
  for (let a = -1; a <= 1; a += 1) {
    for (let b = -1; b <= 1; b += 1) {
      const height = robot.reason === 2 ? Math.abs(a) + 1 : 2;
      for (let k = 0; k < height; k += 1) cells.push(`${ci + a}.${cj + b}.${k}`);
    }
  }
  return cells;
}

function frameLine(id: RobotId, robot: SimRobot): string {
  const cells: string[] = [];
  if (!robot.blocked) {
    const rx = robot.x / VOXEL_SIZE;
    const ry = robot.y / VOXEL_SIZE;
    const limit = (RANGE / VOXEL_SIZE) ** 2;
    for (let n = 0; n < WORLD.length; n += 3) {
      const di = WORLD[n] - rx;
      const dj = WORLD[n + 1] - ry;
      if (di * di + dj * dj <= limit && Math.random() > 0.08) cells.push(`${WORLD[n]}.${WORLD[n + 1]}.${WORLD[n + 2]}`);
    }
    cells.push(...obstacle(robot));
  }
  const state = robot.state === 'S' ? `S${robot.reason ?? 1}` : robot.state;
  const pose = `${Math.round(robot.x * 100)},${Math.round(robot.y * 100)},${Math.round(robot.heading * 1000)}`;
  return `SX1|${id}|${robot.seq}|${state}|${pose}|${cells.join(';')}`;
}

function step(robot: SimRobot, dt: number): void {
  robot.seq += 1;
  if (robot.state !== 'A') return;
  if (robot.route.length === 0) robot.route = [...robot.patrol];
  const target = robot.route[0];
  const dx = target.x - robot.x;
  const dy = target.y - robot.y;
  const distance = Math.hypot(dx, dy);
  const stride = SPEED * dt;
  if (distance <= stride) {
    robot.x = target.x;
    robot.y = target.y;
    robot.route.shift();
    return;
  }
  robot.x += (dx / distance) * stride;
  robot.y += (dy / distance) * stride;
  const wanted = Math.atan2(dy, dx);
  const turn = Math.atan2(Math.sin(wanted - robot.heading), Math.cos(wanted - robot.heading));
  robot.heading += turn * Math.min(1, dt * 6);
}

export interface MockStreamer {
  start(): void;
  stop(): void;
  setState(id: RobotId, state: SimState): void;
  resume(id: RobotId): void;
  setBlocked(id: RobotId, blocked: boolean): void;
  followPath(id: RobotId, points: Vec2[]): void;
}

export function createMockStreamer(onLine: (line: string) => void, intervalMs = 100): MockStreamer {
  const robots = createRobots();
  const dt = intervalMs / 1000;
  let timer: number | undefined;

  return {
    start() {
      if (timer !== undefined) return;
      timer = window.setInterval(() => {
        for (const id of ROBOT_IDS) {
          step(robots[id], dt);
          onLine(frameLine(id, robots[id]));
        }
      }, intervalMs);
    },
    stop() {
      window.clearInterval(timer);
      timer = undefined;
    },
    setState(id, state) {
      const robot = robots[id];
      if (state === 'H') {
        if (robot.state !== 'H') robot.held = { state: robot.state, reason: robot.reason };
      } else {
        robot.held = null;
      }
      robot.state = state;
      robot.reason = state === 'S' ? DEFAULT_REASON[id] : null;
    },
    resume(id) {
      const robot = robots[id];
      const held = robot.held ?? { state: 'A' as SimState, reason: null };
      robot.state = held.state;
      robot.reason = held.reason;
      robot.held = null;
    },
    setBlocked(id, blocked) {
      robots[id].blocked = blocked;
    },
    followPath(id, points) {
      const robot = robots[id];
      robot.route = [...points.slice(1), ...robot.patrol];
      robot.state = 'A';
      robot.reason = null;
      robot.held = null;
    },
  };
}
