import BentoGrid from './components/bento/BentoGrid';
import EStopTile from './components/bento/EStopTile';
import IncidentStackTile from './components/bento/IncidentStackTile';
import TrajectoryTile from './components/bento/TrajectoryTile';
import Header from './components/Header';
import SimulatorPanel from './components/SimulatorPanel';
import Viewport from './components/Viewport';
import WarningBanner from './components/WarningBanner';
import { useFleet } from './hooks/useFleet';
import { OPERATOR } from './hooks/fleetConfig';
import { pathLength } from './hooks/telemetry';
import type { RobotStatus } from './types/fleet';

const STATUS: Record<RobotStatus, { label: string; dot: string }> = {
  active: { label: 'Active', dot: 'bg-sky-300' },
  stuck: { label: 'Needs assistance', dot: 'bg-amber-400' },
  halted: { label: 'Halted', dot: 'bg-red-400' },
};

export default function App() {
  const fleet = useFleet();
  const robot = fleet.robots[fleet.selected];
  const holder = fleet.leases[fleet.selected];
  const status = STATUS[robot.status];

  return (
    <div className="relative h-dvh overflow-hidden bg-black font-sans tracking-tight text-white antialiased">
      <Viewport
        robotId={fleet.selected}
        status={robot.status}
        drawMode={fleet.trajectoryMode}
        path={fleet.path}
        getFrame={fleet.getFrame}
        onPath={fleet.setPath}
      />
      <Header operator={OPERATOR} />

      <div className="pointer-events-none absolute left-4 top-16 z-20 sm:left-6">
        <p className="text-sm font-medium">{robot.name}</p>
        <p className="mt-0.5 flex items-center gap-1.5 text-xs text-white/50">
          <span className={`h-1.5 w-1.5 rounded-full ${status.dot}`} aria-hidden="true" />
          {status.label}
        </p>
        {holder && <p className="mt-0.5 text-xs text-white/40">Controlled by {holder}</p>}
      </div>

      <WarningBanner visible={fleet.alerts[fleet.selected]} robotName={robot.name} />
      <SimulatorPanel fleet={fleet} />

      <BentoGrid>
        <IncidentStackTile
          incidents={fleet.incidents}
          selected={fleet.selected}
          leases={fleet.leases}
          onSelect={fleet.selectIncident}
        />
        <TrajectoryTile
          enabled={fleet.trajectoryMode}
          robotName={robot.name}
          holder={holder}
          hasPath={fleet.path.length >= 2}
          pathMeters={pathLength(fleet.path)}
          onToggle={fleet.toggleTrajectory}
          onClear={() => fleet.setPath([])}
          onSubmit={fleet.submitPath}
        />
        <EStopTile
          robotName={robot.name}
          halted={robot.status === 'halted'}
          onStop={fleet.triggerEstop}
          onRelease={fleet.releaseEstop}
        />
      </BentoGrid>
    </div>
  );
}
