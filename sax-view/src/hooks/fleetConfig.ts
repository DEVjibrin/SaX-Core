import type { RobotId, StuckReason } from '../types/fleet';

export const OPERATOR = 'Jibrin';
export const OTHER_OPERATOR = 'Maya';

export const VOXEL_SIZE = 0.2;
export const MAX_VOXELS = 1500;
export const ZERO_FRAME_LIMIT = 3;

export const REASONS: Record<StuckReason, string> = {
  1: 'Path blocked by pallet',
  2: 'Wheel slip on ramp',
  3: 'Localization lost',
};

export const DEFAULT_REASON: Record<RobotId, StuckReason> = { '03': 1, '07': 2 };

export const robotName = (id: RobotId): string => `Robot #${id}`;

export const createId = (): string =>
  typeof crypto !== 'undefined' && typeof crypto.randomUUID === 'function'
    ? crypto.randomUUID()
    : `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
