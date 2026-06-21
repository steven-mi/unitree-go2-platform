import type { FloorPlan, PathPoint } from "../api";

const INTERIOR_RGB: [number, number, number] = [255, 255, 255];
const WALL_RGB: [number, number, number] = [58, 64, 74];
const BG_RGB: [number, number, number] = [255, 255, 255];

export interface MapView {
  panX: number;
  panY: number;
  zoom: number;
}

function setPixel(data: Uint8ClampedArray, px: number, rgb: [number, number, number]) {
  data[px] = rgb[0];
  data[px + 1] = rgb[1];
  data[px + 2] = rgb[2];
  data[px + 3] = 255;
}

function wallEdgeFlags(
  walls: Uint8Array,
  zones: Uint8Array,
  width: number,
  height: number,
  col: number,
  row: number,
): { border: boolean; inner: boolean } {
  if (!walls[row * width + col]) return { border: false, inner: false };
  let border = false;
  let inner = false;
  for (let dj = -1; dj <= 1; dj++) {
    for (let di = -1; di <= 1; di++) {
      if (dj === 0 && di === 0) continue;
      const ni = col + di;
      const nj = row + dj;
      if (ni < 0 || ni >= width || nj < 0 || nj >= height) {
        border = true;
        continue;
      }
      const nidx = nj * width + ni;
      if (zones[nidx] > 0) inner = true;
      else if (!walls[nidx]) border = true;
    }
  }
  return { border, inner };
}

export function buildPlanCanvas(plan: FloorPlan): HTMLCanvasElement {
  const off = document.createElement("canvas");
  off.width = plan.width;
  off.height = plan.height;
  const offCtx = off.getContext("2d")!;
  const img = offCtx.createImageData(plan.width, plan.height);
  const data = img.data;

  for (let j = 0; j < plan.height; j++) {
    const row = plan.height - 1 - j;
    for (let i = 0; i < plan.width; i++) {
      const idx = row * plan.width + i;
      const px = (j * plan.width + i) * 4;
      const zone = plan.zones[idx];
      const wall = plan.walls[idx];

      if (zone > 0) {
        setPixel(data, px, INTERIOR_RGB);
      } else {
        setPixel(data, px, BG_RGB);
      }

      if (wall) {
        const { border, inner } = wallEdgeFlags(plan.walls, plan.zones, plan.width, plan.height, i, row);
        if (border) setPixel(data, px, [28, 32, 40]);
        else if (inner) setPixel(data, px, [78, 84, 94]);
        else setPixel(data, px, WALL_RGB);
      }
    }
  }

  offCtx.putImageData(img, 0, 0);
  return off;
}

export function worldToGrid(x: number, y: number, plan: FloorPlan): { gi: number; gj: number } {
  return {
    gi: (x - plan.origin_x) / plan.resolution,
    gj: (y - plan.origin_y) / plan.resolution,
  };
}

export function fitScale(plan: FloorPlan, w: number, h: number): number {
  return Math.min(w / plan.width, h / plan.height);
}

export function gridToScreen(
  gi: number,
  gj: number,
  plan: FloorPlan,
  w: number,
  h: number,
  view: MapView,
): { x: number; y: number } {
  const scale = fitScale(plan, w, h) * view.zoom;
  const mapW = plan.width * scale;
  const mapH = plan.height * scale;
  const ox = (w - mapW) / 2 + view.panX;
  const oy = (h - mapH) / 2 + view.panY;
  return {
    x: ox + gi * scale,
    y: oy + (plan.height - 1 - gj) * scale,
  };
}

export function drawPathOverlay(
  ctx: CanvasRenderingContext2D,
  route: PathPoint[],
  plan: FloorPlan,
  w: number,
  h: number,
  view: MapView,
  options?: {
    lineWidth?: number;
    destinations?: PathPoint[];
    markerRadius?: number;
    selectedDestinationIndex?: number;
  },
) {
  if (route.length === 0 && !(options?.destinations?.length)) return;
  const lineWidth = options?.lineWidth ?? 3;
  const markerRadius = options?.markerRadius ?? 9;
  const destinations = options?.destinations ?? [];
  const selectedIndex = options?.selectedDestinationIndex ?? 0;

  if (route.length >= 2) {
    const strokePath = () => {
      ctx.beginPath();
      for (let i = 0; i < route.length; i++) {
        const { gi, gj } = worldToGrid(route[i].x, route[i].y, plan);
        const { x: px, y: py } = gridToScreen(gi, gj, plan, w, h, view);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
    };

    ctx.setLineDash([8, 5]);
    ctx.lineWidth = lineWidth + 3;
    ctx.strokeStyle = "rgba(255, 255, 255, 0.95)";
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    strokePath();

    ctx.setLineDash([]);
    ctx.lineWidth = lineWidth;
    ctx.strokeStyle = "#16a34a";
    strokePath();
  }

  for (let i = 0; i < destinations.length; i++) {
    const pt = destinations[i];
    const { gi, gj } = worldToGrid(pt.x, pt.y, plan);
    const { x: px, y: py } = gridToScreen(gi, gj, plan, w, h, view);
    const active = destinations.length === 1 || i === selectedIndex;
    const radius = active ? markerRadius : Math.max(6, markerRadius * 0.82);
    ctx.beginPath();
    ctx.arc(px, py, radius, 0, Math.PI * 2);
    ctx.fillStyle = active ? "#ea580c" : "#fdba74";
    ctx.fill();
    ctx.strokeStyle = active ? "#fff" : "rgba(255, 255, 255, 0.9)";
    ctx.lineWidth = active ? 2 : 1.5;
    ctx.stroke();
    if (radius >= 7) {
      ctx.fillStyle = active ? "#fff" : "#9a3412";
      ctx.font = `bold ${Math.max(8, Math.round(radius))}px Inter, system-ui, sans-serif`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(String(i + 1), px, py);
      ctx.textBaseline = "alphabetic";
    }
  }
}

export function screenToGrid(
  sx: number,
  sy: number,
  plan: FloorPlan,
  w: number,
  h: number,
  view: MapView,
): { gi: number; gj: number } {
  const scale = fitScale(plan, w, h) * view.zoom;
  const mapW = plan.width * scale;
  const mapH = plan.height * scale;
  const ox = (w - mapW) / 2 + view.panX;
  const oy = (h - mapH) / 2 + view.panY;
  return {
    gi: (sx - ox) / scale,
    gj: plan.height - 1 - (sy - oy) / scale,
  };
}

export function drawScanPreview(canvas: HTMLCanvasElement, plan: FloorPlan) {
  const w = canvas.width;
  const h = canvas.height;
  const ctx = canvas.getContext("2d");
  if (!ctx) return;

  ctx.fillStyle = "#fff";
  ctx.fillRect(0, 0, w, h);

  const view = { panX: 0, panY: 0, zoom: 1 };
  const off = buildPlanCanvas(plan);
  const scale = fitScale(plan, w, h) * view.zoom;
  const mapW = plan.width * scale;
  const mapH = plan.height * scale;
  const ox = (w - mapW) / 2 + view.panX;
  const oy = (h - mapH) / 2 + view.panY;

  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(off, ox, oy, mapW, mapH);
}
