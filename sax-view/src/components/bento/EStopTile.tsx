import { OctagonX } from 'lucide-react';
import { Tile } from './BentoGrid';

interface EStopTileProps {
  robotName: string;
  halted: boolean;
  onStop: () => void;
  onRelease: () => void;
}

export default function EStopTile({ robotName, halted, onStop, onRelease }: EStopTileProps) {
  return (
    <Tile className="border-red-500/25 bg-red-500/[0.07] shadow-[0_0_60px_-20px_rgba(239,68,68,0.55)]">
      <p className="text-xs text-red-200/60">Safety</p>
      <p className="mt-1 text-lg font-semibold">{halted ? `${robotName} halted` : 'Emergency stop'}</p>
      <div className="mt-auto pt-4">
        {halted ? (
          <button
            type="button"
            onClick={onRelease}
            className="w-full rounded-2xl border border-white/15 bg-white/10 py-3.5 text-sm font-medium text-white outline-hidden transition-colors hover:bg-white/15 focus-visible:ring-2 focus-visible:ring-white/50"
          >
            Release halt
          </button>
        ) : (
          <button
            type="button"
            onClick={onStop}
            className="flex w-full items-center justify-center gap-2 rounded-2xl bg-red-600 py-3.5 text-sm font-semibold text-white shadow-[0_0_40px_-6px_rgba(239,68,68,0.8)] outline-hidden transition hover:bg-red-500 focus-visible:ring-4 focus-visible:ring-red-300/50 active:scale-[0.98]"
          >
            <OctagonX className="h-5 w-5" strokeWidth={1.75} aria-hidden="true" />
            Stop {robotName}
          </button>
        )}
      </div>
    </Tile>
  );
}
