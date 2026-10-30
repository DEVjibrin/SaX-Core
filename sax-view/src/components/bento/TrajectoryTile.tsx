import { Lock, ShieldCheck } from 'lucide-react';
import { OPERATOR } from '../../hooks/fleetConfig';
import { Tile } from './BentoGrid';

interface TrajectoryTileProps {
  enabled: boolean;
  robotName: string;
  holder: string | null;
  hasPath: boolean;
  pathMeters: number;
  onToggle: (enabled: boolean) => void;
  onClear: () => void;
  onSubmit: () => void;
}

export default function TrajectoryTile({
  enabled,
  robotName,
  holder,
  hasPath,
  pathMeters,
  onToggle,
  onClear,
  onSubmit,
}: TrajectoryTileProps) {
  const mine = holder === OPERATOR;
  const lockedOut = holder !== null && !mine;

  return (
    <Tile>
      <div className="flex items-start justify-between gap-3">
        <div>
          <p className="text-xs text-white/40">Ghost trajectory</p>
          <p className="mt-1 text-lg font-semibold">Draw recovery path</p>
        </div>
        <button
          type="button"
          role="switch"
          aria-checked={enabled}
          aria-label="Ghost trajectory mode"
          disabled={lockedOut}
          onClick={() => onToggle(!enabled)}
          className={`relative h-7 w-12 shrink-0 rounded-full outline-hidden transition-colors focus-visible:ring-2 focus-visible:ring-white/50 disabled:cursor-not-allowed disabled:opacity-40 motion-reduce:transition-none ${
            enabled ? 'bg-white' : 'bg-white/15'
          }`}
        >
          <span
            className={`absolute left-1 top-1 h-5 w-5 rounded-full transition-transform motion-reduce:transition-none ${
              enabled ? 'translate-x-5 bg-black' : 'bg-white/60'
            }`}
          />
        </button>
      </div>

      <div className="mt-auto space-y-3 pt-4">
        {mine && (
          <div>
            <span
              className={`inline-flex items-center gap-2 rounded-full border px-3 py-1.5 text-xs transition-all ${
                enabled
                  ? 'border-emerald-400/30 bg-emerald-400/10 text-emerald-200 shadow-[0_0_30px_-8px_rgba(52,211,153,0.6)]'
                  : 'border-white/10 bg-white/5 text-white/60'
              }`}
            >
              <ShieldCheck className="h-3.5 w-3.5" strokeWidth={1.75} aria-hidden="true" />
              Exclusive Control Lease
            </span>
            <p className="mt-1.5 text-xs text-white/40">Controlled by {holder}</p>
          </div>
        )}
        {lockedOut && (
          <p className="inline-flex items-center gap-2 text-xs text-amber-300/80">
            <Lock className="h-3.5 w-3.5" strokeWidth={1.75} aria-hidden="true" />
            Controlled by {holder}
          </p>
        )}
        {holder === null && <p className="text-xs text-white/40">Lease available. Selecting an incident takes it.</p>}

        {enabled && !hasPath && <p className="text-xs text-white/50">Drag on the canvas to draw from {robotName}.</p>}
        {enabled && hasPath && (
          <div className="flex items-center gap-2">
            <button
              type="button"
              onClick={onClear}
              className="rounded-full border border-white/15 px-4 py-2 text-xs text-white outline-hidden transition-colors hover:bg-white/5 focus-visible:ring-2 focus-visible:ring-white/40"
            >
              Clear
            </button>
            <button
              type="button"
              onClick={onSubmit}
              className="rounded-full bg-white px-4 py-2 text-xs font-medium text-black outline-hidden transition hover:bg-white/90 focus-visible:ring-2 focus-visible:ring-white/50"
            >
              Send path ({pathMeters.toFixed(1)} m)
            </button>
          </div>
        )}
      </div>
    </Tile>
  );
}
