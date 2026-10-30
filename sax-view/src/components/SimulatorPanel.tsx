import { useEffect, useState } from 'react';
import { FlaskConical } from 'lucide-react';
import type { Fleet } from '../hooks/useFleet';
import { OTHER_OPERATOR, robotName } from '../hooks/fleetConfig';
import { ROBOT_IDS } from '../types/fleet';

const button =
  'rounded-full border border-white/10 bg-white/5 px-3 py-1.5 text-xs text-white/80 outline-hidden transition-colors hover:bg-white/10 focus-visible:ring-2 focus-visible:ring-white/40';

export default function SimulatorPanel({ fleet }: { fleet: Fleet }) {
  const [open, setOpen] = useState(false);

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.shiftKey && event.key.toLowerCase() === 's') setOpen((current) => !current);
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, []);

  return (
    <div className="absolute right-4 top-16 z-30 flex flex-col items-end gap-2 sm:right-6">
      <button
        type="button"
        aria-label="Simulator"
        aria-expanded={open}
        title="Simulator (Shift+S)"
        onClick={() => setOpen((current) => !current)}
        className="flex h-9 w-9 items-center justify-center rounded-full border border-white/10 bg-white/[0.04] text-white/50 outline-hidden backdrop-blur-xl transition-colors hover:text-white focus-visible:ring-2 focus-visible:ring-white/40"
      >
        <FlaskConical className="h-4 w-4" strokeWidth={1.5} aria-hidden="true" />
      </button>

      {open && (
        <div className="w-72 max-w-[calc(100vw-2rem)] space-y-4 rounded-3xl border border-white/10 bg-black/60 p-4 backdrop-blur-xl">
          {ROBOT_IDS.map((id) => {
            const stuck = fleet.robots[id].status === 'stuck';
            const rival = fleet.leases[id] === OTHER_OPERATOR;
            return (
              <div key={id} className="space-y-2">
                <p className="text-xs text-white/40">{robotName(id)}</p>
                <div className="flex flex-wrap gap-2">
                  <button type="button" className={button} onClick={() => (stuck ? fleet.sim.resolve(id) : fleet.sim.trigger(id))}>
                    {stuck ? 'Resolve incident' : 'Trigger incident'}
                  </button>
                  <button type="button" className={button} onClick={() => fleet.sim.setBlocked(id, !fleet.simBlocked[id])}>
                    {fleet.simBlocked[id] ? 'Restore LiDAR' : 'Block LiDAR'}
                  </button>
                  <button type="button" className={button} onClick={() => fleet.sim.setLeaseHolder(id, rival ? null : OTHER_OPERATOR)}>
                    {rival ? `Release ${OTHER_OPERATOR}` : `${OTHER_OPERATOR} takes lease`}
                  </button>
                </div>
              </div>
            );
          })}
          <div className="flex items-center justify-between border-t border-white/10 pt-3 text-xs text-white/40">
            <span>{fleet.audit.events.length} audit events</span>
            <div className="flex gap-2">
              <button type="button" className={button} onClick={fleet.audit.exportJson}>
                Export
              </button>
              <button type="button" className={button} onClick={fleet.audit.clear}>
                Clear
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
