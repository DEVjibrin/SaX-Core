export default function Header({ operator }: { operator: string }) {
  return (
    <header className="pointer-events-none absolute inset-x-0 top-0 z-30 flex h-14 items-center justify-between bg-linear-to-b from-black/80 to-transparent px-4 sm:px-6">
      <span className="text-xl font-semibold tracking-tight">SaX</span>
      <div className="pointer-events-auto flex items-center gap-2.5 rounded-full border border-white/10 bg-white/[0.04] py-1 pl-1 pr-3.5 backdrop-blur-xl">
        <span
          aria-hidden="true"
          className="flex h-7 w-7 items-center justify-center rounded-full bg-linear-to-br from-zinc-200 to-zinc-500 text-xs font-semibold text-black"
        >
          {operator[0]}
        </span>
        <span className="text-xs">
          <span className="hidden text-white/40 sm:inline">Fleet Operator: </span>
          {operator}
        </span>
      </div>
    </header>
  );
}
