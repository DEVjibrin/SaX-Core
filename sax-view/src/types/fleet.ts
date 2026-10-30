export const ROBOT_IDS = ['03', '07'] as const;

export type RobotId = (typeof ROBOT_IDS)[number];
export type RobotStatus = 'active' | 'stuck' | 'halted';
export type StuckReason = 1 | 2 | 3;

export interface Vec2 {
  x: number;
  y: number;
}

export interface Frame {
  robotId: RobotId;
  seq: number;
  status: RobotStatus;
  reason: StuckReason | null;
  pose: { x: number; y: number; heading: number };
  voxels: Int16Array;
  count: number;
}

export interface RobotView {
  id: RobotId;
  name: string;
  status: RobotStatus;
  reason: StuckReason | null;
}

export interface Incident {
  id: string;
  robotId: RobotId;
  reason: StuckReason;
  raisedAt: number;
  resolvedAt: number | null;
}

export type AuditEventType =
  | 'incident_raised'
  | 'incident_resolved'
  | 'incident_selected'
  | 'lease_acquired'
  | 'lease_released'
  | 'lease_denied'
  | 'lease_revoked'
  | 'trajectory_mode'
  | 'spline_submitted'
  | 'e_stop'
  | 'e_stop_released'
  | 'lidar_blockage_detected'
  | 'lidar_blockage_cleared';

export interface AuditEvent {
  id: string;
  at: string;
  type: AuditEventType;
  operator: string;
  robotId: RobotId | null;
  detail: Record<string, unknown>;
}
