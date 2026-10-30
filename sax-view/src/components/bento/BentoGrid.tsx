import type { ReactNode } from 'react';

export function Tile({ children, className = '' }: { children: ReactNode; className?: string }) {
  return (
    <section
      className={`relative flex min-h-44 flex-col rounded-3xl border border-white/10 bg-white/[0.04] p-4 backdrop-blur-xl sm:p-5 ${className}`}
    >
      {children}
    </section>
  );
}

export default function BentoGrid({ children }: { children: ReactNode }) {
  return (
    <div className="absolute inset-x-0 bottom-0 z-20 p-3 pb-[max(0.75rem,env(safe-area-inset-bottom))] sm:p-4">
      <div className="mx-auto flex max-w-6xl snap-x snap-mandatory gap-3 overflow-x-auto [scrollbar-width:none] md:grid md:grid-cols-3 md:overflow-visible [&>*]:min-w-[85%] [&>*]:snap-center md:[&>*]:min-w-0">
        {children}
      </div>
    </div>
  );
}
