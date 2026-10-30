import { ROBOT_IDS, type Frame, type RobotId, type RobotStatus, type StuckReason, type Vec2 } from '../types/fleet';

const STATUS: Record<string, RobotStatus> = { A: 'active', S: 'stuck', H: 'halted' };

/**
 * Wire format, one line per robot per tick:
 * SX1|<robot>|<seq>|<state>|<x_cm>,<y_cm>,<heading_mrad>|<i>.<j>.<k>;<i>.<j>.<k>;...
 * State is A (active), H (halted) or S<reason> (stuck). Voxels are integer grid cells.
 */
export function parseFrame(line: string): Frame | null {
  const parts = line.split('|');
  if (parts.length !== 6 || parts[0] !== 'SX1') return null;
  const [, id, seq, state, pose, cloud] = parts;
  const status = STATUS[state[0]];
  const [x, y, heading] = pose.split(',').map(Number);
  if (!ROBOT_IDS.includes(id as RobotId) || !status || ![x, y, heading].every(Number.isFinite)) return null;

  const cells = cloud ? cloud.split(';') : [];
  const voxels = new Int16Array(cells.length * 3);
  cells.forEach((cell, index) => voxels.set(cell.split('.').slice(0, 3).map(Number), index * 3));
  const reason = state.length > 1 ? (Number(state.slice(1)) as StuckReason) : null;

  return {
    robotId: id as RobotId,
    seq: Number(seq),
    status,
    reason,
    pose: { x: x / 100, y: y / 100, heading: heading / 1000 },
    voxels,
    count: cells.length,
  };
}

export const pathLength = (points: Vec2[]): number =>
  points.reduce((sum, point, index) => (index === 0 ? 0 : sum + Math.hypot(point.x - points[index - 1].x, point.y - points[index - 1].y)), 0);

function distanceToSegment(point: Vec2, a: Vec2, b: Vec2): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const lengthSquared = dx * dx + dy * dy;
  if (lengthSquared === 0) return Math.hypot(point.x - a.x, point.y - a.y);
  const t = Math.max(0, Math.min(1, ((point.x - a.x) * dx + (point.y - a.y) * dy) / lengthSquared));
  return Math.hypot(point.x - (a.x + t * dx), point.y - (a.y + t * dy));
}

export function simplifyPath(points: Vec2[], tolerance = 0.2): Vec2[] {
  if (points.length < 3) return points;
  const first = points[0];
  const last = points[points.length - 1];
  let split = 0;
  let farthest = 0;
  for (let index = 1; index < points.length - 1; index += 1) {
    const distance = distanceToSegment(points[index], first, last);
    if (distance > farthest) {
      farthest = distance;
      split = index;
    }
  }
  if (farthest <= tolerance) return [first, last];
  return [...simplifyPath(points.slice(0, split + 1), tolerance).slice(0, -1), ...simplifyPath(points.slice(split), tolerance)];
}

export const encodePath = (robotId: RobotId, points: Vec2[]): string =>
  `SXP|${robotId}|${points.map((point) => `${Math.round(point.x * 100)},${Math.round(point.y * 100)}`).join(';')}`;
