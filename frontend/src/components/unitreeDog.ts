import type { FramePose } from "../api";

/** Local Y of foot contact in the 3D follow model (meters). */
export const GO2_FOOT_LOCAL_Y = 0;
/** Odom z is body origin; feet sit ~30 cm below when standing. */
const GO2_BODY_TO_FOOT = 0.3;

/** Go2 body half-width and half-length in meters (~31 cm × 70 cm). */
const BODY_HW = 0.155;
const BODY_HL = 0.35;

/** Scene Y where the robot's feet should touch the ground. */
export function robotFloorSceneY(pose: FramePose): number {
  return (pose.z ?? 0) - GO2_BODY_TO_FOOT;
}

function quatRotateVector(
  qx: number,
  qy: number,
  qz: number,
  qw: number,
  vx: number,
  vy: number,
  vz: number,
): [number, number, number] {
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

/** Heading in the odom x–y plane (radians), CCW from +X. */
export function poseHeading2D(pose: FramePose): number {
  if (
    pose.qx != null &&
    pose.qy != null &&
    pose.qz != null &&
    pose.qw != null
  ) {
    const odomFwd = quatRotateVector(pose.qx, pose.qy, pose.qz, pose.qw, 1, 0, 0);
    return Math.atan2(odomFwd[1], odomFwd[0]);
  }
  if (pose.yaw != null) return pose.yaw;
  return 0;
}

function pathTangentHeading(
  path: { x: number; y: number }[],
  x: number,
  y: number,
): number | null {
  if (path.length < 2) return null;

  let anchor = 0;
  let bestD = Infinity;
  for (let i = 0; i < path.length; i++) {
    const d = (path[i].x - x) ** 2 + (path[i].y - y) ** 2;
    if (d < bestD) {
      bestD = d;
      anchor = i;
    }
  }

  // Average direction over a short window to avoid flipping when the path grows.
  const back = Math.max(0, anchor - 3);
  const ahead = Math.min(path.length - 1, anchor + 6);
  if (ahead <= back) return null;

  const dx = path[ahead].x - path[back].x;
  const dy = path[ahead].y - path[back].y;
  if (dx * dx + dy * dy < 1e-6) return null;
  return Math.atan2(dy, dx);
}

function velocityHeadingOdom(pose: FramePose, velocity: number[]): number | null {
  const speed = Math.hypot(velocity[0], velocity[1]);
  if (speed < 0.08) return null;
  if (
    pose.qx != null &&
    pose.qy != null &&
    pose.qz != null &&
    pose.qw != null
  ) {
    const vo = quatRotateVector(
      pose.qx,
      pose.qy,
      pose.qz,
      pose.qw,
      velocity[0],
      velocity[1],
      velocity[2] ?? 0,
    );
    return Math.atan2(vo[1], vo[0]);
  }
  return Math.atan2(velocity[1], velocity[0]);
}

/** Odom heading (radians, CCW from +X) for floor-plan arrow direction. */
export function floorPlanArrowHeadingOdom(
  pose: FramePose,
  options?: {
    velocity?: number[] | null;
    path?: { x: number; y: number }[];
  },
): number {
  if (options?.velocity) {
    const velHeading = velocityHeadingOdom(pose, options.velocity);
    if (velHeading != null) return velHeading;
  }

  const pathHeading =
    options?.path && options.path.length >= 2
      ? pathTangentHeading(options.path, pose.x, pose.y)
      : null;
  if (pathHeading != null) return pathHeading;

  return poseHeading2D(pose);
}

/** Shortest-path angle interpolation (radians). */
export function smoothAngle(current: number, target: number, alpha: number): number {
  let delta = target - current;
  while (delta > Math.PI) delta -= 2 * Math.PI;
  while (delta < -Math.PI) delta += 2 * Math.PI;
  return current + delta * alpha;
}

/** Directional arrow for 2D floor-plan maps; forward is +x before rotation. */
export function drawRobotArrowTopDown(
  ctx: CanvasRenderingContext2D,
  cx: number,
  cy: number,
  yaw: number,
  scale = 1,
) {
  const s = scale;
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(yaw);

  const tip = 12 * s;
  const tail = 4.5 * s;
  const halfW = 6.5 * s;

  ctx.beginPath();
  ctx.moveTo(tip, 0);
  ctx.lineTo(-tail, -halfW);
  ctx.lineTo(-tail * 0.15, 0);
  ctx.lineTo(-tail, halfW);
  ctx.closePath();
  ctx.fillStyle = "#f97316";
  ctx.fill();
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = Math.max(1, 1.3 * s);
  ctx.lineJoin = "round";
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(0, 0, 2.4 * s, 0, Math.PI * 2);
  ctx.fillStyle = "#ffffff";
  ctx.fill();
  ctx.strokeStyle = "rgba(0, 0, 0, 0.4)";
  ctx.lineWidth = Math.max(0.75, s);
  ctx.stroke();

  ctx.restore();
}

type Project2D = (lx: number, ly: number, lz: number) => { sx: number; sy: number } | null;

function drawProjectedPolygon(
  ctx: CanvasRenderingContext2D,
  projectLocal: Project2D,
  corners: [number, number, number][],
  fill: string,
  stroke?: string,
  lineWidth = 2,
) {
  const pts = corners.map(([lx, ly, lz]) => projectLocal(lx, ly, lz));
  if (pts.some((p) => !p)) return;
  ctx.beginPath();
  ctx.moveTo(pts[0]!.sx, pts[0]!.sy);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i]!.sx, pts[i]!.sy);
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.fill();
  if (stroke) {
    ctx.strokeStyle = stroke;
    ctx.lineWidth = lineWidth;
    ctx.lineJoin = "round";
    ctx.stroke();
  }
}

/** Flat footprint + forward arrow projected into the follow-view scene. */
export function drawRobotArrowMarker(
  ctx: CanvasRenderingContext2D,
  projectLocal: Project2D,
) {
  const y = GO2_FOOT_LOCAL_Y + 0.012;

  ctx.save();

  // Body footprint (~70 × 31 cm).
  drawProjectedPolygon(
    ctx,
    projectLocal,
    [
      [-BODY_HW, y, -BODY_HL * 0.55],
      [BODY_HW, y, -BODY_HL * 0.55],
      [BODY_HW, y, BODY_HL * 0.75],
      [-BODY_HW, y, BODY_HL * 0.75],
    ],
    "rgba(255, 255, 255, 0.18)",
    "rgba(255, 255, 255, 0.85)",
    2,
  );

  // Forward arrow (+Z).
  drawProjectedPolygon(
    ctx,
    projectLocal,
    [
      [0, y, BODY_HL * 0.62],
      [-BODY_HW * 0.5, y, -BODY_HL * 0.05],
      [BODY_HW * 0.5, y, -BODY_HL * 0.05],
    ],
    "#f97316",
    "#ffffff",
    2.5,
  );

  // Center pivot dot.
  const center = projectLocal(0, y, 0);
  if (center) {
    ctx.beginPath();
    ctx.arc(center.sx, center.sy, 3.5, 0, Math.PI * 2);
    ctx.fillStyle = "#ffffff";
    ctx.fill();
    ctx.strokeStyle = "rgba(0, 0, 0, 0.45)";
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  ctx.restore();
}
