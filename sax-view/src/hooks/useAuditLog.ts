import { useCallback, useEffect, useState } from 'react';
import { createId } from './fleetConfig';
import type { AuditEvent, AuditEventType, RobotId } from '../types/fleet';

const STORAGE_KEY = 'sax-view.audit.v1';
const MAX_EVENTS = 1000;

function load(): AuditEvent[] {
  try {
    const raw = window.localStorage.getItem(STORAGE_KEY);
    const parsed: unknown = raw ? JSON.parse(raw) : [];
    return Array.isArray(parsed) ? (parsed as AuditEvent[]) : [];
  } catch {
    return [];
  }
}

export function useAuditLog(operator: string) {
  const [events, setEvents] = useState<AuditEvent[]>(load);

  useEffect(() => {
    try {
      window.localStorage.setItem(STORAGE_KEY, JSON.stringify(events));
    } catch (error) {
      console.warn('sax-view: unable to persist the audit log', error);
    }
  }, [events]);

  const record = useCallback(
    (type: AuditEventType, robotId: RobotId | null, detail: Record<string, unknown> = {}) => {
      const event: AuditEvent = { id: createId(), at: new Date().toISOString(), type, operator, robotId, detail };
      setEvents((current) => [...current.slice(-(MAX_EVENTS - 1)), event]);
    },
    [operator],
  );

  const clear = useCallback(() => setEvents([]), []);

  const exportJson = useCallback(() => {
    const url = URL.createObjectURL(new Blob([JSON.stringify(events, null, 2)], { type: 'application/json' }));
    const link = document.createElement('a');
    link.href = url;
    link.download = `sax-view-audit-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.json`;
    link.click();
    URL.revokeObjectURL(url);
  }, [events]);

  return { events, record, clear, exportJson };
}
