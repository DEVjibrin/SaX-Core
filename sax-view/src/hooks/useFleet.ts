import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { OPERATOR, ZERO_FRAME_LIMIT, createId, robotName } from './fleetConfig';
import { createMockStreamer, type MockStreamer } from './mockStreamer';
import { encodePath, parseFrame, pathLength, simplifyPath } from './telemetry';
import type { Frame, Incident, RobotId, RobotStatus, RobotView, Vec2 } from '../types/fleet';
import { useAuditLog } from './useAuditLog';

type Leases = Record<RobotId, string | null>;

const each = <T,>(value: T): Record<RobotId, T> => ({ '03': value, '07': value });

const initialRobots = (): Record<RobotId, RobotView> => ({
  '03': { id: '03', name: robotName('03'), status: 'active', reason: null },
  '07': { id: '07', name: robotName('07'), status: 'active', reason: null },
});

export function useFleet() {
  const audit = useAuditLog(OPERATOR);
  const [robots, setRobots] = useState(initialRobots);
  const [incidents, setIncidents] = useState<Incident[]>([]);
  const [leases, setLeases] = useState<Leases>(() => each<string | null>(null));
  const [selected, setSelected] = useState<RobotId>('03');
  const [trajectoryMode, setTrajectoryMode] = useState(false);
  const [path, setPath] = useState<Vec2[]>([]);
  const [alerts, setAlerts] = useState(() => each(false));
  const [simBlocked, setSimBlocked] = useState(() => each(false));

  const streamer = useRef<MockStreamer | null>(null);
  const frames = useRef(new Map<RobotId, Frame>());
  const statuses = useRef<Partial<Record<RobotId, RobotStatus>>>({});
  const pending = useRef(new Set<RobotId>());
  const zeroRun = useRef(each(0));
  const alertFlags = useRef(each(false));
  const leasesRef = useRef<Leases>(each<string | null>(null));
  const selectedRef = useRef<RobotId>('03');
  const record = useRef(audit.record);

  useEffect(() => {
    record.current = audit.record;
  }, [audit.record]);

  const commitLeases = useCallback((next: Leases) => {
    leasesRef.current = next;
    setLeases(next);
  }, []);

  const releaseLease = useCallback(
    (id: RobotId, reason: string) => {
      if (leasesRef.current[id] !== OPERATOR) return;
      commitLeases({ ...leasesRef.current, [id]: null });
      record.current('lease_released', id, { reason });
    },
    [commitLeases],
  );

  const acquireLease = useCallback(
    (id: RobotId): boolean => {
      const holder = leasesRef.current[id];
      if (holder === OPERATOR) return true;
      if (holder !== null) {
        record.current('lease_denied', id, { heldBy: holder });
        return false;
      }
      commitLeases({ ...leasesRef.current, [id]: OPERATOR });
      record.current('lease_acquired', id, {});
      return true;
    },
    [commitLeases],
  );

  useEffect(() => {
    const handleLine = (line: string) => {
      const frame = parseFrame(line);
      if (!frame) return;
      const id = frame.robotId;
      frames.current.set(id, frame);

      const previous = statuses.current[id];
      if (previous !== frame.status) {
        statuses.current[id] = frame.status;
        setRobots((current) => ({ ...current, [id]: { ...current[id], status: frame.status, reason: frame.reason } }));
        if (frame.status === 'stuck' && !pending.current.has(id)) {
          const reason = frame.reason ?? 1;
          pending.current.add(id);
          setIncidents((current) => [...current, { id: createId(), robotId: id, reason, raisedAt: Date.now(), resolvedAt: null }]);
          if (previous !== undefined) record.current('incident_raised', id, { reason });
        } else if (frame.status === 'active' && pending.current.has(id)) {
          pending.current.delete(id);
          setIncidents((current) =>
            current.map((incident) =>
              incident.robotId === id && incident.resolvedAt === null ? { ...incident, resolvedAt: Date.now() } : incident,
            ),
          );
          record.current('incident_resolved', id, {});
          releaseLease(id, 'incident resolved');
        }
      }

      zeroRun.current[id] = frame.status !== 'halted' && frame.count === 0 ? zeroRun.current[id] + 1 : 0;
      const blocked = zeroRun.current[id] >= ZERO_FRAME_LIMIT;
      if (blocked !== alertFlags.current[id]) {
        alertFlags.current[id] = blocked;
        setAlerts((current) => ({ ...current, [id]: blocked }));
        record.current(blocked ? 'lidar_blockage_detected' : 'lidar_blockage_cleared', id, {
          pointCount: frame.count,
          status: frame.status,
        });
      }
    };

    const mock = createMockStreamer(handleLine);
    streamer.current = mock;
    mock.start();
    return () => mock.stop();
  }, [releaseLease]);

  const getFrame = useCallback((id: RobotId) => frames.current.get(id), []);

  const selectIncident = useCallback(
    (incident: Incident) => {
      const id = incident.robotId;
      if (id !== selectedRef.current) {
        releaseLease(selectedRef.current, 'operator switched vehicle');
        selectedRef.current = id;
        setSelected(id);
        setTrajectoryMode(false);
        setPath([]);
      }
      const granted = acquireLease(id);
      record.current('incident_selected', id, { incidentId: incident.id, leaseGranted: granted });
    },
    [acquireLease, releaseLease],
  );

  const toggleTrajectory = useCallback(
    (enabled: boolean) => {
      const id = selectedRef.current;
      if (enabled && !acquireLease(id)) return;
      setTrajectoryMode(enabled);
      if (!enabled) setPath([]);
      record.current('trajectory_mode', id, { enabled });
    },
    [acquireLease],
  );

  const submitPath = useCallback(() => {
    const id = selectedRef.current;
    if (leasesRef.current[id] !== OPERATOR || path.length < 2) return;
    const waypoints = simplifyPath(path);
    const payload = encodePath(id, waypoints);
    record.current('spline_submitted', id, {
      waypoints: waypoints.length,
      lengthM: Number(pathLength(path).toFixed(2)),
      payload,
      bytes: payload.length,
    });
    streamer.current?.followPath(id, waypoints);
    setTrajectoryMode(false);
    setPath([]);
  }, [path]);

  const triggerEstop = useCallback(() => {
    const id = selectedRef.current;
    streamer.current?.setState(id, 'H');
    record.current('e_stop', id, { leaseHolder: leasesRef.current[id] });
    setTrajectoryMode(false);
    setPath([]);
  }, []);

  const releaseEstop = useCallback(() => {
    const id = selectedRef.current;
    streamer.current?.resume(id);
    record.current('e_stop_released', id, {});
  }, []);

  const sim = useMemo(
    () => ({
      trigger: (id: RobotId) => streamer.current?.setState(id, 'S'),
      resolve: (id: RobotId) => streamer.current?.setState(id, 'A'),
      setBlocked: (id: RobotId, blocked: boolean) => {
        streamer.current?.setBlocked(id, blocked);
        setSimBlocked((current) => ({ ...current, [id]: blocked }));
      },
      setLeaseHolder: (id: RobotId, holder: string | null) => {
        const current = leasesRef.current[id];
        if (current === holder) return;
        if (current === OPERATOR && holder !== null) {
          record.current('lease_revoked', id, { newHolder: holder, simulated: true });
          if (id === selectedRef.current) {
            setTrajectoryMode(false);
            setPath([]);
          }
        } else if (holder === null) {
          record.current('lease_released', id, { holder: current, simulated: true });
        } else {
          record.current('lease_acquired', id, { holder, simulated: true });
        }
        commitLeases({ ...leasesRef.current, [id]: holder });
      },
    }),
    [commitLeases],
  );

  return {
    robots,
    selected,
    incidents,
    leases,
    alerts,
    simBlocked,
    trajectoryMode,
    path,
    audit,
    getFrame,
    selectIncident,
    toggleTrajectory,
    submitPath,
    setPath,
    triggerEstop,
    releaseEstop,
    sim,
  };
}

export type Fleet = ReturnType<typeof useFleet>;
