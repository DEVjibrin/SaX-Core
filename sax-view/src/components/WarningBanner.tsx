import { TriangleAlert } from 'lucide-react';

interface WarningBannerProps {
  visible: boolean;
  robotName: string;
}

export default function WarningBanner({ visible, robotName }: WarningBannerProps) {
  return (
    <div
      role={visible ? 'alert' : undefined}
      aria-hidden={!visible}
      className={`pointer-events-none absolute left-1/2 top-28 z-20 -translate-x-1/2 transition-all duration-300 motion-reduce:transition-none ${
        visible ? 'translate-y-0 opacity-100' : '-translate-y-2 opacity-0'
      }`}
    >
      <div className="flex items-center gap-2.5 whitespace-nowrap rounded-full border border-amber-400/25 bg-amber-400/10 px-4 py-2 text-sm text-amber-100 shadow-[0_0_40px_-10px_rgba(251,191,36,0.55)] backdrop-blur-xl">
        <TriangleAlert className="h-4 w-4 text-amber-300" strokeWidth={1.75} aria-hidden="true" />
        LiDAR sensor blockage detected
        <span className="hidden text-amber-200/60 sm:inline">{robotName}</span>
      </div>
    </div>
  );
}
