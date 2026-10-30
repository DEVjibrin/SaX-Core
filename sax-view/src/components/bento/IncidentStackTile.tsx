import { Lock } from 'lucide-react';
import { OPERATOR, REASONS, robotName } from '../../hooks/fleetConfig';
import type { Incident, RobotId } from '../../types/fleet';
import { Tile } from './BentoGrid';

interface IncidentStackTileProps {
  incidents: Incident[];
  selected: RobotId;
  leases: Record<RobotId, string | null>;
  onSelect: (incident: Incident) => void;
}

export default function IncidentStackTile({ incidents, selected, leases, onSelect }: IncidentStackTileProps) {
  const queue = incidents.filter((incident) => incident.resolvedAt === null);

  return (
    <Tile>
      <p className="text-xs text-white/40">Fleet incidents</p>
      <p className="mt-1 text-2xl font-semibold">{queue.length > 0 ? `Queue: ${queue.length} Pending` : 'Queue clear'}</p>
      <ul className="-mr-1 mt-3 max-h-28 space-y-1 overflow-y-auto pr-1">
        {queue.map((incident) => {
          const holder = leases[incident.robotId];
          const active = incident.robotId === selected;
          return (
            <li key={incident.id}>
              <button
                type="button"
                aria-pressed={active}
                onClick={() => onSelect(incident)}
                className={`flex w-full items-center gap-3 rounded-2xl px-3 py-2 text-left text-sm outline-hidden transition-colors focus-visible:ring-2 focus-visible:ring-white/40 ${
                  active ? 'bg-white/10' : 'hover:bg-white/5'
                }`}
              >
                <span className="font-medium">{robotName(incident.robotId)}</span>
                <span className="min-w-0 flex-1 truncate text-white/40">{REASONS[incident.reason]}</span>
                {holder && holder !== OPERATOR && (
                  <span className="inline-flex items-center gap-1 text-xs text-amber-300/80">
                    <Lock className="h-3 w-3" strokeWidth={1.75} aria-hidden="true" />
                    {holder}
                  </span>
                )}
              </button>
            </li>
          );
        })}
      </ul>
    </Tile>
  );
}
