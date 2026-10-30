import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { MAX_VOXELS, VOXEL_SIZE } from '../hooks/fleetConfig';
import type { Frame, RobotId, RobotStatus, Vec2 } from '../types/fleet';

const STATUS_COLOR: Record<RobotStatus, number> = { active: 0x7dd3fc, stuck: 0xfbbf24, halted: 0xf87171 };
const MIN_POINT_GAP = 0.35;
const MAX_PATH_POINTS = 160;
const GHOST_SPEED = 1.2;

export interface SceneOptions {
  canvas: HTMLCanvasElement;
  getFrame: (id: RobotId) => Frame | undefined;
  onPath: (points: Vec2[]) => void;
}

function glowTexture(): THREE.CanvasTexture {
  const size = 128;
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  const context = canvas.getContext('2d');
  if (context) {
    const gradient = context.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
    gradient.addColorStop(0, 'rgba(255,255,255,1)');
    gradient.addColorStop(1, 'rgba(255,255,255,0)');
    context.fillStyle = gradient;
    context.fillRect(0, 0, size, size);
  }
  return new THREE.CanvasTexture(canvas);
}

function buildRover(body: THREE.Material, wheel: THREE.Material, accent: THREE.Material): THREE.Group {
  const rover = new THREE.Group();
  const add = (geometry: THREE.BufferGeometry, material: THREE.Material, x: number, y: number, z: number) => {
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(x, y, z);
    rover.add(mesh);
    return mesh;
  };
  add(new THREE.BoxGeometry(0.9, 0.22, 0.58), body, 0, 0.3, 0);
  add(new THREE.CylinderGeometry(0.12, 0.12, 0.12, 20), body, -0.1, 0.5, 0);
  add(new THREE.BoxGeometry(0.04, 0.05, 0.46), accent, 0.45, 0.32, 0);
  for (const x of [-0.3, 0.3]) {
    for (const z of [-0.34, 0.34]) {
      add(new THREE.CylinderGeometry(0.14, 0.14, 0.1, 20), wheel, x, 0.14, z).rotation.x = Math.PI / 2;
    }
  }
  return rover;
}

export class SceneController {
  private readonly options: SceneOptions;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(48, 1, 0.1, 200);
  private readonly controls: OrbitControls;
  private readonly clock = new THREE.Clock();
  private readonly rover: THREE.Group;
  private readonly ghost: THREE.Group;
  private readonly glow: THREE.Mesh;
  private readonly glowMaterial: THREE.MeshBasicMaterial;
  private readonly accent = new THREE.MeshBasicMaterial({ color: STATUS_COLOR.active });
  private readonly cloud: THREE.InstancedMesh;
  private readonly pathGroup = new THREE.Group();
  private readonly pathCore = new THREE.MeshBasicMaterial({ color: 0xffffff });
  private readonly pathGlow = new THREE.MeshBasicMaterial({
    color: 0x7dd3fc,
    transparent: true,
    opacity: 0.18,
    blending: THREE.AdditiveBlending,
    depthWrite: false,
  });
  private readonly raycaster = new THREE.Raycaster();
  private readonly ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  private readonly dummy = new THREE.Object3D();
  private readonly tint = new THREE.Color();
  private readonly hit = new THREE.Vector3();
  private readonly point = new THREE.Vector3();
  private readonly tangent = new THREE.Vector3();
  private readonly focus = new THREE.Vector3();
  private readonly targetPos = new THREE.Vector3();

  private robotId: RobotId = '03';
  private heading = 0;
  private targetHeading = 0;
  private lastSeq = -1;
  private snap = true;
  private drawMode = false;
  private stroking = false;
  private path: Vec2[] = [];
  private curve: THREE.CatmullRomCurve3 | null = null;
  private curveLength = 1;
  private ghostT = 0;

  constructor(options: SceneOptions) {
    this.options = options;
    const { canvas } = options;

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0x000000, 1);

    this.scene.fog = new THREE.FogExp2(0x000000, 0.028);
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const sun = new THREE.DirectionalLight(0xffffff, 1.1);
    sun.position.set(6, 10, 4);
    this.scene.add(sun);

    const grid = new THREE.GridHelper(80, 80, 0x3f3f46, 0x1c1c1f);
    const gridMaterial = grid.material as THREE.LineBasicMaterial;
    gridMaterial.transparent = true;
    gridMaterial.opacity = 0.6;
    this.scene.add(grid);

    const body = new THREE.MeshStandardMaterial({ color: 0x18181b, metalness: 0.7, roughness: 0.35 });
    const wheel = new THREE.MeshStandardMaterial({ color: 0x0b0b0c, roughness: 0.8 });
    this.rover = buildRover(body, wheel, this.accent);

    const ghostMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true, transparent: true, opacity: 0.55 });
    this.ghost = buildRover(ghostMaterial, ghostMaterial, ghostMaterial);
    this.ghost.visible = false;

    this.glowMaterial = new THREE.MeshBasicMaterial({
      map: glowTexture(),
      color: STATUS_COLOR.active,
      transparent: true,
      opacity: 0.5,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    });
    this.glow = new THREE.Mesh(new THREE.PlaneGeometry(3.4, 3.4), this.glowMaterial);
    this.glow.rotation.x = -Math.PI / 2;
    this.glow.position.y = 0.02;

    const voxel = VOXEL_SIZE * 0.85;
    this.cloud = new THREE.InstancedMesh(
      new THREE.BoxGeometry(voxel, voxel, voxel),
      new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.9 }),
      MAX_VOXELS,
    );
    this.cloud.count = 0;
    this.cloud.frustumCulled = false;

    this.scene.add(this.rover, this.ghost, this.glow, this.cloud, this.pathGroup);

    this.camera.position.set(6, 5, 8);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.enablePan = false;
    this.controls.minDistance = 4;
    this.controls.maxDistance = 28;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.06;
    this.controls.target.set(0, 0.3, 0);
    this.controls.update();

    canvas.addEventListener('pointerdown', this.onPointerDown);
    canvas.addEventListener('pointermove', this.onPointerMove);
    canvas.addEventListener('pointerup', this.onPointerUp);
    canvas.addEventListener('pointercancel', this.onPointerUp);
    this.renderer.setAnimationLoop(this.tick);
  }

  resize(width: number, height: number): void {
    if (width === 0 || height === 0) return;
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.setViewOffset(width, height, 0, Math.round(height * 0.1), width, height);
  }

  setRobot(id: RobotId): void {
    this.robotId = id;
    this.lastSeq = -1;
    this.snap = true;
    this.cloud.count = 0;
  }

  setStatus(status: RobotStatus): void {
    this.accent.color.setHex(STATUS_COLOR[status]);
    this.glowMaterial.color.setHex(STATUS_COLOR[status]);
  }

  setDrawMode(on: boolean): void {
    this.drawMode = on;
    this.controls.enabled = !on;
    this.options.canvas.style.cursor = on ? 'crosshair' : '';
  }

  setPath(points: Vec2[]): void {
    if (points === this.path) return;
    this.path = points;
    this.rebuildPath();
  }

  dispose(): void {
    const { canvas } = this.options;
    this.renderer.setAnimationLoop(null);
    canvas.removeEventListener('pointerdown', this.onPointerDown);
    canvas.removeEventListener('pointermove', this.onPointerMove);
    canvas.removeEventListener('pointerup', this.onPointerUp);
    canvas.removeEventListener('pointercancel', this.onPointerUp);
    this.controls.dispose();
    this.scene.traverse((object) => {
      if (object instanceof THREE.Mesh || object instanceof THREE.LineSegments) {
        object.geometry.dispose();
        const materials: THREE.Material[] = Array.isArray(object.material) ? object.material : [object.material];
        materials.forEach((material) => material.dispose());
      }
    });
    this.cloud.dispose();
    this.glowMaterial.map?.dispose();
    this.renderer.dispose();
  }

  private toGround(event: PointerEvent): Vec2 | null {
    const rect = this.options.canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    this.raycaster.setFromCamera(ndc, this.camera);
    return this.raycaster.ray.intersectPlane(this.ground, this.hit) ? { x: this.hit.x, y: this.hit.z } : null;
  }

  private addPoint(event: PointerEvent): void {
    const point = this.toGround(event);
    const last = this.path[this.path.length - 1];
    if (!point || this.path.length >= MAX_PATH_POINTS || Math.hypot(point.x - last.x, point.y - last.y) < MIN_POINT_GAP) return;
    this.path.push(point);
    this.rebuildPath();
  }

  private onPointerDown = (event: PointerEvent): void => {
    if (!this.drawMode || event.button !== 0) return;
    this.path = [{ x: this.rover.position.x, y: this.rover.position.z }];
    this.stroking = true;
    this.options.canvas.setPointerCapture(event.pointerId);
    this.addPoint(event);
  };

  private onPointerMove = (event: PointerEvent): void => {
    if (this.stroking) this.addPoint(event);
  };

  private onPointerUp = (): void => {
    if (!this.stroking) return;
    this.stroking = false;
    this.path = this.path.length >= 2 ? this.path : [];
    this.rebuildPath();
    this.options.onPath(this.path);
  };

  private rebuildPath(): void {
    for (const child of [...this.pathGroup.children]) {
      (child as THREE.Mesh).geometry.dispose();
      this.pathGroup.remove(child);
    }
    this.curve = null;
    this.ghost.visible = false;
    if (this.path.length < 2) return;

    const curve = new THREE.CatmullRomCurve3(
      this.path.map((point) => new THREE.Vector3(point.x, 0.05, point.y)),
      false,
      'centripetal',
    );
    const segments = Math.max(24, this.path.length * 6);
    this.pathGroup.add(
      new THREE.Mesh(new THREE.TubeGeometry(curve, segments, 0.04, 6, false), this.pathCore),
      new THREE.Mesh(new THREE.TubeGeometry(curve, segments, 0.12, 8, false), this.pathGlow),
    );
    this.curve = curve;
    this.curveLength = curve.getLength();
    this.ghostT = 0;
    this.ghost.visible = true;
  }

  private updateCloud(frame: Frame): void {
    const count = Math.min(frame.count, MAX_VOXELS);
    for (let n = 0; n < count; n += 1) {
      const k = frame.voxels[n * 3 + 2];
      this.dummy.position.set(frame.voxels[n * 3] * VOXEL_SIZE, k * VOXEL_SIZE + VOXEL_SIZE / 2, frame.voxels[n * 3 + 1] * VOXEL_SIZE);
      this.dummy.updateMatrix();
      this.cloud.setMatrixAt(n, this.dummy.matrix);
      this.cloud.setColorAt(n, this.tint.setHSL(0.6, 0.1 + 0.08 * k, 0.38 + 0.1 * k));
    }
    this.cloud.count = count;
    this.cloud.instanceMatrix.needsUpdate = true;
    if (this.cloud.instanceColor) this.cloud.instanceColor.needsUpdate = true;
  }

  private tick = (): void => {
    const delta = Math.min(this.clock.getDelta(), 0.1);

    const frame = this.options.getFrame(this.robotId);
    if (frame && frame.seq !== this.lastSeq) {
      this.lastSeq = frame.seq;
      this.targetPos.set(frame.pose.x, 0, frame.pose.y);
      this.targetHeading = frame.pose.heading;
      this.updateCloud(frame);
      if (this.snap) {
        this.rover.position.copy(this.targetPos);
        this.heading = this.targetHeading;
        this.snap = false;
      }
    }

    const blend = 1 - Math.exp(-delta * 12);
    this.rover.position.lerp(this.targetPos, blend);
    const turn = Math.atan2(Math.sin(this.targetHeading - this.heading), Math.cos(this.targetHeading - this.heading));
    this.heading += turn * blend;
    this.rover.rotation.y = -this.heading;
    this.glow.position.x = this.rover.position.x;
    this.glow.position.z = this.rover.position.z;

    const before = this.controls.target.clone();
    this.controls.target.lerp(this.focus.set(this.rover.position.x, 0.3, this.rover.position.z), 1 - Math.exp(-delta * 3));
    this.camera.position.add(this.controls.target.clone().sub(before));

    if (this.curve && this.ghost.visible) {
      this.ghostT = (this.ghostT + (delta * GHOST_SPEED) / Math.max(this.curveLength, 0.5)) % 1;
      this.curve.getPointAt(this.ghostT, this.point);
      this.curve.getTangentAt(this.ghostT, this.tangent);
      this.ghost.position.set(this.point.x, 0, this.point.z);
      this.ghost.rotation.y = -Math.atan2(this.tangent.z, this.tangent.x);
    }

    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };
}
