import { useEffect, useRef, useState } from "react";
import type { FramePose } from "../api";
import {
  drawRobotArrowMarker,
  GO2_FOOT_LOCAL_Y,
  robotFloorSceneY,
} from "./unitreeDog";

const Z_MIN = -0.5;
const Z_MAX = 1.6;
const GROUND_Y = Z_MIN;
/** Lidar / odom ground plane in scene Y (matches typical floor points). */
const FLOOR_SCENE_Y = 0;
const VOXEL_RES = 0.05;
const VOXEL_HALF = VOXEL_RES * 0.5;
const VOXEL_OVERLAP = 1.04;
const DOG_CAM_DISTANCE = 2.6;
const DOG_CAM_LIFT = 1.0;

interface PointCloudViewProps {
  points: Float32Array | null;
  pose: FramePose | null;
  videoUrl?: string | null;
  /** Bumps when container layout changes (e.g. lidar fullscreen). */
  layoutKey?: string;
}

type ScenePoint = [number, number, number];

// Fixed height → color LUT so colors stay stable across frames (not per-scan min/max).
const HEIGHT_LUT = new Uint8Array(256 * 3);

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

function lerpRgb(
  a: [number, number, number],
  b: [number, number, number],
  t: number,
): [number, number, number] {
  return [lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t)];
}

// Unitree-style rainbow by height (green floor → purple ceiling).
const COLOR_STOPS: { h: number; rgb: [number, number, number] }[] = [
  { h: -0.5, rgb: [48, 175, 72] },
  { h: -0.1, rgb: [56, 210, 110] },
  { h: 0.2, rgb: [72, 215, 185] },
  { h: 0.45, rgb: [120, 220, 95] },
  { h: 0.65, rgb: [215, 215, 70] },
  { h: 0.85, rgb: [235, 155, 55] },
  { h: 1.05, rgb: [225, 85, 65] },
  { h: 1.35, rgb: [195, 75, 175] },
  { h: 1.6, rgb: [155, 95, 210] },
];

function heightToIndex(height: number): number {
  const t = (height - Z_MIN) / (Z_MAX - Z_MIN);
  return Math.max(0, Math.min(255, Math.floor(t * 255)));
}

function buildHeightLut() {
  for (let i = 0; i < 256; i++) {
    const h = Z_MIN + (i / 255) * (Z_MAX - Z_MIN);
    let rgb: [number, number, number] = COLOR_STOPS[COLOR_STOPS.length - 1].rgb;
    for (let s = 0; s < COLOR_STOPS.length - 1; s++) {
      const a = COLOR_STOPS[s];
      const b = COLOR_STOPS[s + 1];
      if (h >= a.h && h <= b.h) {
        const t = (h - a.h) / Math.max(0.001, b.h - a.h);
        rgb = lerpRgb(a.rgb, b.rgb, t);
        break;
      }
    }
    HEIGHT_LUT[i * 3] = rgb[0];
    HEIGHT_LUT[i * 3 + 1] = rgb[1];
    HEIGHT_LUT[i * 3 + 2] = rgb[2];
  }
}
buildHeightLut();

function heightRgb(height: number): [number, number, number] {
  const i = heightToIndex(height) * 3;
  return [HEIGHT_LUT[i], HEIGHT_LUT[i + 1], HEIGHT_LUT[i + 2]];
}

interface DogCamera {
  eye: ScenePoint;
  right: ScenePoint;
  up: ScenePoint;
  forward: ScenePoint;
  fov: number;
}

function robotForward(pose: FramePose): ScenePoint {
  if (
    pose.qx != null &&
    pose.qy != null &&
    pose.qz != null &&
    pose.qw != null
  ) {
    const odomFwd = quatRotateVector(pose.qx, pose.qy, pose.qz, pose.qw, 1, 0, 0);
    return normalize3([odomFwd[0], odomFwd[2], odomFwd[1]]);
  }
  if (pose.yaw != null) {
    return normalize3([Math.cos(pose.yaw), 0, Math.sin(pose.yaw)]);
  }
  return [1, 0, 0];
}

function buildDogCamera(pose: FramePose, fov: number): DogCamera {
  const forward = robotForward(pose);
  const worldUp: ScenePoint = [0, 1, 0];
  let right = normalize3([
    forward[1] * worldUp[2] - forward[2] * worldUp[1],
    forward[2] * worldUp[0] - forward[0] * worldUp[2],
    forward[0] * worldUp[1] - forward[1] * worldUp[0],
  ]);
  const up = normalize3([
    right[1] * forward[2] - right[2] * forward[1],
    right[2] * forward[0] - right[0] * forward[2],
    right[0] * forward[1] - right[1] * forward[0],
  ]);
  right = normalize3([
    up[1] * forward[2] - up[2] * forward[1],
    up[2] * forward[0] - up[0] * forward[2],
    up[0] * forward[1] - up[1] * forward[0],
  ]);

  const center = odomToScene(pose.x, pose.y, pose.z + 0.15);
  const eye: ScenePoint = [
    center[0] - forward[0] * DOG_CAM_DISTANCE + up[0] * DOG_CAM_LIFT,
    center[1] - forward[1] * DOG_CAM_DISTANCE + up[1] * DOG_CAM_LIFT,
    center[2] - forward[2] * DOG_CAM_DISTANCE + up[2] * DOG_CAM_LIFT,
  ];
  const lookAt: ScenePoint = [
    center[0] + forward[0] * 1.5,
    center[1] + forward[1] * 1.5 + 0.2,
    center[2] + forward[2] * 1.5,
  ];
  const viewForward = normalize3([
    lookAt[0] - eye[0],
    lookAt[1] - eye[1],
    lookAt[2] - eye[2],
  ]);
  const viewRight = normalize3([
    viewForward[1] * worldUp[2] - viewForward[2] * worldUp[1],
    viewForward[2] * worldUp[0] - viewForward[0] * worldUp[2],
    viewForward[0] * worldUp[1] - viewForward[1] * worldUp[0],
  ]);
  const viewUp = normalize3([
    viewRight[1] * viewForward[2] - viewRight[2] * viewForward[1],
    viewRight[2] * viewForward[0] - viewRight[0] * viewForward[2],
    viewRight[0] * viewForward[1] - viewRight[1] * viewForward[0],
  ]);

  return {
    eye,
    right: viewRight,
    up: viewUp,
    forward: viewForward,
    fov,
  };
}

function odomToScene(x: number, y: number, z: number): ScenePoint {
  return [x, z, y];
}

function quatRotateVector(
  qx: number,
  qy: number,
  qz: number,
  qw: number,
  vx: number,
  vy: number,
  vz: number,
): ScenePoint {
  const ix = qw * vx + qy * vz - qz * vy;
  const iy = qw * vy + qz * vx - qx * vz;
  const iz = qw * vz + qx * vy - qy * vx;
  const iw = -qx * vx - qy * vy - qz * vz;
  return [
    ix * qw + iw * -qx + iy * -qz - iz * -qy,
    iy * qw + iw * -qy + iz * -qx - ix * -qz,
    iz * qw + iw * -qz + ix * -qy - iy * -qx,
  ];
}

function normalize3(v: ScenePoint): ScenePoint {
  const l = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / l, v[1] / l, v[2] / l];
}

function dot3(a: ScenePoint, b: ScenePoint): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function projectScenePoint(
  x: number,
  y: number,
  z: number,
  cam: DogCamera,
  w: number,
  h: number,
): { sx: number; sy: number; depth: number } | null {
  const rel: ScenePoint = [x - cam.eye[0], y - cam.eye[1], z - cam.eye[2]];
  const viewZ = dot3(rel, cam.forward);
  if (viewZ <= 0.12) return null;
  const viewX = dot3(rel, cam.right);
  const viewY = dot3(rel, cam.up);
  const scale = cam.fov * 0.52;
  return {
    sx: w / 2 + (viewX / viewZ) * scale * w,
    sy: h / 2 - (viewY / viewZ) * scale * h,
    depth: viewZ,
  };
}

interface ProjectedPoint {
  sx: number;
  sy: number;
  depth: number;
  height: number;
}

function filterScenePoints(points: Float32Array): {
  positions: Float32Array;
  count: number;
  minH: number;
  maxH: number;
} {
  const zMin = Z_MIN;
  const n = Math.floor(points.length / 3);
  const positions = new Float32Array(n * 3);
  let kept = 0;
  let minH = Infinity;
  let maxH = -Infinity;

  for (let i = 0; i < n; i++) {
    const x = points[i * 3];
    const y = points[i * 3 + 1];
    const z = points[i * 3 + 2];
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
    if (z < Z_MIN || z > Z_MAX) continue;
    if (z < zMin) continue;
    positions[kept * 3] = x;
    positions[kept * 3 + 1] = z;
    positions[kept * 3 + 2] = y;
    if (z < minH) minH = z;
    if (z > maxH) maxH = z;
    kept++;
  }

  return { positions: positions.subarray(0, kept * 3), count: kept, minH, maxH };
}

function projectionScale(cam: DogCamera): number {
  return cam.fov * 0.52;
}

function voxelHalfPx(depth: number, cam: DogCamera, w: number): number {
  const ppm = (projectionScale(cam) * w) / Math.max(0.2, depth);
  return Math.max(1.5, VOXEL_HALF * ppm * VOXEL_OVERLAP);
}

function fillVoxelSquare(
  depthBuf: Float32Array,
  rBuf: Uint8Array,
  gBuf: Uint8Array,
  bBuf: Uint8Array,
  x0: number,
  y0: number,
  size: number,
  depth: number,
  r: number,
  g: number,
  b: number,
  w: number,
  h: number,
) {
  const x1 = x0 + size;
  const y1 = y0 + size;
  const ix0 = Math.max(0, Math.floor(x0));
  const iy0 = Math.max(0, Math.floor(y0));
  const ix1 = Math.min(w - 1, Math.ceil(x1));
  const iy1 = Math.min(h - 1, Math.ceil(y1));
  for (let sy = iy0; sy <= iy1; sy++) {
    for (let sx = ix0; sx <= ix1; sx++) {
      const idx = sy * w + sx;
      if (depth >= depthBuf[idx]) continue;
      depthBuf[idx] = depth;
      rBuf[idx] = r;
      gBuf[idx] = g;
      bBuf[idx] = b;
    }
  }
}

function renderVoxelCubes(
  ctx: CanvasRenderingContext2D,
  projected: ProjectedPoint[],
  cam: DogCamera,
  w: number,
  h: number,
) {
  const size = w * h;
  const depthBuf = new Float32Array(size);
  depthBuf.fill(Infinity);
  const rBuf = new Uint8Array(size);
  const gBuf = new Uint8Array(size);
  const bBuf = new Uint8Array(size);

  let minDepth = Infinity;
  let maxDepth = -Infinity;
  for (const p of projected) {
    if (p.depth < minDepth) minDepth = p.depth;
    if (p.depth > maxDepth) maxDepth = p.depth;
  }
  const depthSpan = Math.max(0.001, maxDepth - minDepth);

  const sideDx = cam.right[0] * 0.55 + 0.25;
  const sideDy = -cam.up[1] * 0.55 + 0.35;

  for (const p of projected) {
    const [r, g, b] = heightRgb(p.height);
    const depthT = (p.depth - minDepth) / depthSpan;
    const shadeTop = 0.82 + 0.18 * (1 - depthT);
    const shadeSide = shadeTop * 0.62;
    const hw = voxelHalfPx(p.depth, cam, w);
    const topX = p.sx - hw;
    const topY = p.sy - hw;
    const faceSize = hw * 2;

    fillVoxelSquare(
      depthBuf,
      rBuf,
      gBuf,
      bBuf,
      topX,
      topY,
      faceSize,
      p.depth,
      Math.min(255, Math.round(r * shadeTop)),
      Math.min(255, Math.round(g * shadeTop)),
      Math.min(255, Math.round(b * shadeTop)),
      w,
      h,
    );

    if (p.height > GROUND_Y + 0.04) {
      fillVoxelSquare(
        depthBuf,
        rBuf,
        gBuf,
        bBuf,
        topX + sideDx * hw,
        topY + sideDy * hw,
        faceSize,
        p.depth + 0.003,
        Math.min(255, Math.round(r * shadeSide)),
        Math.min(255, Math.round(g * shadeSide)),
        Math.min(255, Math.round(b * shadeSide)),
        w,
        h,
      );
    }
  }

  const img = ctx.createImageData(w, h);
  const data = img.data;
  for (let i = 0; i < size; i++) {
    if (depthBuf[i] === Infinity) continue;
    const idx = i * 4;
    data[idx] = rBuf[i];
    data[idx + 1] = gBuf[i];
    data[idx + 2] = bBuf[i];
    data[idx + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

function drawBackground(ctx: CanvasRenderingContext2D, w: number, h: number) {
  const grad = ctx.createLinearGradient(0, 0, 0, h);
  grad.addColorStop(0, "#2a2e34");
  grad.addColorStop(1, "#1e2126");
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, w, h);
}

function drawGroundGrid(
  ctx: CanvasRenderingContext2D,
  cam: DogCamera,
  w: number,
  h: number,
) {
  const span = 12;
  const step = 1;
  const y = FLOOR_SCENE_Y;

  ctx.strokeStyle = "rgba(255,255,255,0.09)";
  ctx.lineWidth = 1;

  const drawLine3d = (ax: number, ay: number, az: number, bx: number, by: number, bz: number) => {
    const a = projectScenePoint(ax, ay, az, cam, w, h);
    const b = projectScenePoint(bx, by, bz, cam, w, h);
    if (!a || !b) return;
    ctx.beginPath();
    ctx.moveTo(a.sx, a.sy);
    ctx.lineTo(b.sx, b.sy);
    ctx.stroke();
  };

  for (let i = -span; i <= span; i += step) {
    drawLine3d(i, y, -span, i, y, span);
    drawLine3d(-span, y, i, span, y, i);
  }
}

function renderPointCloud(
  ctx: CanvasRenderingContext2D,
  projected: ProjectedPoint[],
  cam: DogCamera,
  w: number,
  h: number,
) {
  renderVoxelCubes(ctx, projected, cam, w, h);
}

function drawFollowRobot(
  ctx: CanvasRenderingContext2D,
  pose: FramePose,
  cam: DogCamera,
  w: number,
  h: number,
) {
  const floorY = robotFloorSceneY(pose);
  const anchorY = floorY - GO2_FOOT_LOCAL_Y;
  const center = odomToScene(pose.x, pose.y, anchorY);
  const forward = robotForward(pose);
  const right: ScenePoint = [forward[2], 0, -forward[0]];

  const projectLocal = (lx: number, ly: number, lz: number) =>
    projectScenePoint(
      center[0] + right[0] * lx + forward[0] * lz,
      center[1] + ly,
      center[2] + right[2] * lx + forward[2] * lz,
      cam,
      w,
      h,
    );

  drawRobotArrowMarker(ctx, projectLocal);
}

function DogCameraPip({ videoUrl }: { videoUrl: string | null }) {
  const [displayUrl, setDisplayUrl] = useState<string | null>(null);
  const latestRef = useRef<string | null>(null);
  const displayRef = useRef<string | null>(null);

  useEffect(() => {
    if (!videoUrl) {
      latestRef.current = null;
      displayRef.current = null;
      setDisplayUrl(null);
      return;
    }
    latestRef.current = videoUrl;
    if (videoUrl === displayRef.current) return;

    const img = new Image();
    img.onload = () => {
      if (latestRef.current === videoUrl) {
        displayRef.current = videoUrl;
        setDisplayUrl(videoUrl);
      }
    };
    img.src = videoUrl;
  }, [videoUrl]);

  if (!displayUrl) return null;

  return (
    <div className="pointcloud-pip">
      <div className="pointcloud-pip-label">Dog camera</div>
      <img src={displayUrl} alt="Dog camera view" />
    </div>
  );
}

export function PointCloudView({
  points,
  pose,
  videoUrl = null,
  layoutKey = "inline",
}: PointCloudViewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const dogFovRef = useRef(1.55);
  const frameRef = useRef(0);
  const dirtyRef = useRef(true);
  const pointsRef = useRef(points);
  const poseRef = useRef(pose);

  pointsRef.current = points;
  poseRef.current = pose;

  const markDirty = () => {
    dirtyRef.current = true;
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      dogFovRef.current = Math.max(0.9, Math.min(2.4, dogFovRef.current + e.deltaY * 0.002));
      markDirty();
    };

    canvas.addEventListener("wheel", onWheel, { passive: false });

    const draw = () => {
      frameRef.current = requestAnimationFrame(draw);
      if (!dirtyRef.current) return;
      dirtyRef.current = false;

      const parent = canvas.parentElement;
      if (!parent) return;
      const rect = parent.getBoundingClientRect();
      const w = Math.max(1, Math.floor(rect.width));
      const h = Math.max(1, Math.floor(rect.height));
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
      }

      const ctx = canvas.getContext("2d");
      if (!ctx) return;

      drawBackground(ctx, w, h);

      const currentPoints = pointsRef.current;
      const currentPose = poseRef.current;

      if (!currentPoints || currentPoints.length < 3) {
        ctx.fillStyle = "#888";
        ctx.font = "14px Inter, system-ui, sans-serif";
        ctx.textAlign = "center";
        ctx.fillText("No lidar data", w / 2, h / 2);
        return;
      }

      if (!currentPose) {
        ctx.fillStyle = "#888";
        ctx.font = "14px Inter, system-ui, sans-serif";
        ctx.textAlign = "center";
        ctx.fillText("No robot pose for follow view", w / 2, h / 2);
        return;
      }

      const { positions, count } = filterScenePoints(currentPoints);
      if (count === 0) return;

      const cam = buildDogCamera(currentPose, dogFovRef.current);

      drawGroundGrid(ctx, cam, w, h);

      const projected: ProjectedPoint[] = [];
      projected.length = count;
      let pi = 0;
      for (let i = 0; i < count; i++) {
        const pr = projectScenePoint(
          positions[i * 3],
          positions[i * 3 + 1],
          positions[i * 3 + 2],
          cam,
          w,
          h,
        );
        if (!pr) continue;
        projected[pi++] = {
          sx: pr.sx,
          sy: pr.sy,
          depth: pr.depth,
          height: positions[i * 3 + 1],
        };
      }
      projected.length = pi;

      projected.sort((a, b) => b.depth - a.depth);
      renderPointCloud(ctx, projected, cam, w, h);

      drawFollowRobot(ctx, currentPose, cam, w, h);

      ctx.fillStyle = "rgba(255,255,255,0.4)";
      ctx.font = "11px Inter, system-ui, sans-serif";
      ctx.textAlign = "left";
      ctx.fillText("Follow · scroll to zoom", 10, h - 10);
    };

    markDirty();
    draw();
    const observer = new ResizeObserver(markDirty);
    observer.observe(canvas.parentElement ?? canvas);

    return () => {
      cancelAnimationFrame(frameRef.current);
      observer.disconnect();
      canvas.removeEventListener("wheel", onWheel);
    };
  }, []);

  useEffect(() => {
    markDirty();
  }, [points, pose, layoutKey]);

  return (
    <div className="pointcloud-view">
      <canvas ref={canvasRef} className="pointcloud-canvas" />
      <DogCameraPip videoUrl={videoUrl ?? null} />
    </div>
  );
}

export function useLidarPointsBinary(
  sessionId: string,
  seq: number | null,
  enabled: boolean,
  source: import("../api").DataSource = "recording",
) {
  const [points, setPoints] = useState<Float32Array | null>(null);
  const cache = useRef<Map<number, Float32Array>>(new Map());
  const seqRef = useRef<number | null>(null);

  useEffect(() => {
    cache.current.clear();
    seqRef.current = null;
    setPoints(null);
  }, [sessionId]);

  useEffect(() => {
    if (!enabled || seq == null) {
      if (!enabled) return;
      seqRef.current = null;
      setPoints(null);
      return;
    }
    seqRef.current = seq;
    const cached = cache.current.get(seq);
    if (cached) {
      setPoints(cached);
      return;
    }
    const requestedSeq = seq;
    import("../api").then(({ fetchLidarBinary }) =>
      fetchLidarBinary(sessionId, requestedSeq, 0, source).then((data) => {
        cache.current.set(requestedSeq, data);
        if (seqRef.current === requestedSeq) setPoints(data);
      }),
    );
  }, [sessionId, seq, enabled, source]);

  return points;
}
