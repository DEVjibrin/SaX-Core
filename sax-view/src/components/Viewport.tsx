import { useEffect, useRef, useState } from 'react';
import { SceneController } from '../three/SceneController';
import type { Frame, RobotId, RobotStatus, Vec2 } from '../types/fleet';

interface ViewportProps {
  robotId: RobotId;
  status: RobotStatus;
  drawMode: boolean;
  path: Vec2[];
  getFrame: (id: RobotId) => Frame | undefined;
  onPath: (points: Vec2[]) => void;
}

export default function Viewport({ robotId, status, drawMode, path, getFrame, onPath }: ViewportProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const scene = useRef<SceneController | null>(null);
  const callbacks = useRef({ getFrame, onPath });
  const [unsupported, setUnsupported] = useState(false);

  useEffect(() => {
    callbacks.current = { getFrame, onPath };
  }, [getFrame, onPath]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return undefined;
    let controller: SceneController;
    try {
      controller = new SceneController({
        canvas,
        getFrame: (id) => callbacks.current.getFrame(id),
        onPath: (points) => callbacks.current.onPath(points),
      });
    } catch {
      setUnsupported(true);
      return undefined;
    }
    scene.current = controller;
    const observer = new ResizeObserver(([entry]) => controller.resize(entry.contentRect.width, entry.contentRect.height));
    observer.observe(canvas);
    return () => {
      observer.disconnect();
      controller.dispose();
      scene.current = null;
    };
  }, []);

  useEffect(() => {
    scene.current?.setRobot(robotId);
  }, [robotId]);

  useEffect(() => {
    scene.current?.setStatus(status);
  }, [status]);

  useEffect(() => {
    scene.current?.setDrawMode(drawMode);
  }, [drawMode]);

  useEffect(() => {
    scene.current?.setPath(path);
  }, [path]);

  if (unsupported) {
    return (
      <div className="absolute inset-0 flex items-center justify-center px-6 text-center text-sm text-white/40">
        WebGL isn&apos;t available in this browser, so the 3D view can&apos;t start.
      </div>
    );
  }

  return <canvas ref={canvasRef} aria-label="3D view of the selected robot" className="absolute inset-0 h-full w-full touch-none" />;
}
