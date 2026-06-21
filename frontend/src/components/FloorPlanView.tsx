import { useEffect, useRef, useState } from "react";
import type { DataSource, FloorPlan, FramePose, PathPoint } from "../api";
import { fetchFloorPlan, fetchScanPath } from "../api";
import {
  buildPlanCanvas,
  drawPathOverlay,
  fitScale,
  gridToScreen,
  screenToGrid,
  worldToGrid,
  type MapView,
} from "./scanMapCanvas";
import { drawRobotArrowTopDown, floorPlanArrowHeadingOdom, poseHeading2D, smoothAngle } from "./unitreeDog";

interface FloorPlanViewProps {
  sessionId: string;
  pose: FramePose | null;
  velocity: number[] | null;
  t: number;
  playing: boolean;
  /** Current lidar frame seq — merged into the map on every refresh. */
  lidarSeq?: number | null;
  /** Compact overlay for lidar fullscreen PIP (no footer hint). */
  variant?: "panel" | "overlay";
  source?: DataSource;
  enabled?: boolean;
  /** Bump to force a reload of a static (scan) floor plan as the saved map updates. */
  reloadKey?: number;
  /** Planned route polyline (map coordinates). */
  pathPoints?: PathPoint[];
  /** User destinations — shown as markers only. */
  destinationPoints?: PathPoint[];
  /** Highlighted destination when multiple are saved. */
  selectedDestinationIndex?: number;
  annotateMode?: boolean;
  onPathPointAdd?: (x: number, y: number) => void;
  /** Click map to mark where the dog is (map alignment). */
  alignMode?: boolean;
  /** Load saved path.json when viewing a scan (merged with pathPoints prop). */
  loadSavedPath?: boolean;
  /** Use live pose yaw only — skip velocity / scan-trajectory heading overrides. */
  poseHeadingOnly?: boolean;
  onAlignClick?: (x: number, y: number) => void;
}

const BG_RGB: [number, number, number] = [255, 255, 255];

export function FloorPlanView({
  sessionId,
  pose,
  velocity,
  t,
  playing,
  lidarSeq = null,
  variant = "panel",
  source = "recording",
  enabled = true,
  reloadKey = 0,
  pathPoints = [],
  destinationPoints = [],
  selectedDestinationIndex = 0,
  annotateMode = false,
  onPathPointAdd,
  alignMode = false,
  onAlignClick,
  loadSavedPath = false,
  poseHeadingOnly = false,
}: FloorPlanViewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [plan, setPlan] = useState<FloorPlan | null>(null);
  const [savedPath, setSavedPath] = useState<PathPoint[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const viewRef = useRef<MapView>({ panX: 0, panY: 0, zoom: 1 });
  const dragRef = useRef(false);
  const lastPointerRef = useRef({ x: 0, y: 0 });
  const pointerDownRef = useRef({ x: 0, y: 0 });
  const frameRef = useRef(0);
  const dirtyRef = useRef(true);
  const planRef = useRef(plan);
  const poseRef = useRef(pose);
  const gridCanvasRef = useRef<HTMLCanvasElement | null>(null);

  const floorAbortRef = useRef<AbortController | null>(null);
  const floorGenRef = useRef(0);
  const lastFetchTRef = useRef(-1);
  const tRef = useRef(t);

  const velocityRef = useRef(velocity);
  const playingRef = useRef(playing);
  const poseHeadingOnlyRef = useRef(poseHeadingOnly);
  const lastPoseRef = useRef(pose);
  const arrowHeadingRef = useRef(0);
  const lastLoadedTRef = useRef(-1);

  const lastLoadedSeqRef = useRef<number | null>(null);
  const lidarSeqRef = useRef(lidarSeq);

  const pathPointsRef = useRef(pathPoints);
  const destinationPointsRef = useRef(destinationPoints);
  const selectedDestinationIndexRef = useRef(selectedDestinationIndex);
  const displayPathRef = useRef<PathPoint[]>(pathPoints);
  const displayDestinationsRef = useRef<PathPoint[]>(destinationPoints);
  const loadSavedPathRef = useRef(loadSavedPath);
  const annotateModeRef = useRef(annotateMode);
  const alignModeRef = useRef(alignMode);
  const onPathPointAddRef = useRef(onPathPointAdd);
  const onAlignClickRef = useRef(onAlignClick);

  loadSavedPathRef.current = loadSavedPath;
  pathPointsRef.current = pathPoints;
  destinationPointsRef.current = destinationPoints;
  selectedDestinationIndexRef.current = selectedDestinationIndex;
  displayPathRef.current =
    loadSavedPath && pathPoints.length === 0 ? savedPath : pathPoints;
  displayDestinationsRef.current =
    destinationPoints.length > 0 ? destinationPoints : displayPathRef.current.slice(-1);
  annotateModeRef.current = annotateMode;
  alignModeRef.current = alignMode;
  onPathPointAddRef.current = onPathPointAdd;
  onAlignClickRef.current = onAlignClick;

  const variantRef = useRef(variant);
  variantRef.current = variant;
  const isStaticSource = source === "scan";

  if (pose) lastPoseRef.current = pose;
  tRef.current = t;
  playingRef.current = playing;
  poseHeadingOnlyRef.current = poseHeadingOnly;
  planRef.current = plan;
  poseRef.current = pose;
  velocityRef.current = velocity;
  lidarSeqRef.current = lidarSeq;

  useEffect(() => {
    if (!plan) {
      gridCanvasRef.current = null;
      return;
    }
    gridCanvasRef.current = buildPlanCanvas(plan);
    markDirty();
  }, [plan]);

  const markDirty = () => {
    dirtyRef.current = true;
  };

  const loadFloorPlan = (
    time: number,
    poseForPlan: FramePose | null,
    seq: number | null,
  ) => {
    floorAbortRef.current?.abort();
    const controller = new AbortController();
    floorAbortRef.current = controller;
    const gen = ++floorGenRef.current;

    fetchFloorPlan(sessionId, time, poseForPlan, controller.signal, seq, source)
      .then((next) => {
        if (gen !== floorGenRef.current) return;
        lastLoadedTRef.current = time;
        lastLoadedSeqRef.current = seq;
        setPlan(next);
        setError(null);
      })
      .catch((err) => {
        if (gen !== floorGenRef.current || err.name === "AbortError") return;
        setError(err.message ?? "Failed to load floor plan");
      })
      .finally(() => {
        if (gen === floorGenRef.current) {
          setLoading(false);
        }
      });
  };

  useEffect(() => {
    setLoading(true);
    setError(null);
    setPlan(null);
    viewRef.current = { panX: 0, panY: 0, zoom: 1 };
    lastFetchTRef.current = -1;
    lastLoadedTRef.current = -1;
    lastLoadedSeqRef.current = null;
    arrowHeadingRef.current = 0;
    lastPoseRef.current = null;
    floorAbortRef.current?.abort();
    return () => floorAbortRef.current?.abort();
  }, [sessionId, source]);

  // Static saved scan: load floor plan once per scan id, and again when reloadKey
  // changes (the saved map is being extended by the continuous live scan).
  useEffect(() => {
    if (!enabled || !isStaticSource) return;
    loadFloorPlan(0, null, null);
    // eslint-disable-next-line react-hooks/exhaustive-deps -- load on scan id / reloadKey
  }, [sessionId, enabled, isStaticSource, reloadKey]);

  useEffect(() => {
    if (!loadSavedPath || !isStaticSource || !sessionId) {
      setSavedPath([]);
      return;
    }
    let cancelled = false;
    fetchScanPath(sessionId)
      .then((points) => {
        if (!cancelled) setSavedPath(points);
      })
      .catch(() => {
        if (!cancelled) setSavedPath([]);
      });
    return () => {
      cancelled = true;
    };
  }, [isStaticSource, loadSavedPath, sessionId]);

  // Paused / scrub: rebuild immediately when time or lidar frame changes.
  useEffect(() => {
    if (!enabled || isStaticSource) {
      if (isStaticSource) return;
      setLoading(false);
      setPlan(null);
      setError(null);
      floorAbortRef.current?.abort();
      return;
    }
    if (playing) return;
    loadFloorPlan(t, pose ?? lastPoseRef.current, lidarSeq ?? null);
  }, [playing, t, sessionId, pose, lidarSeq, enabled, isStaticSource]);

  // Play: refresh on new lidar frames only (t ticks every animation frame — not a dep).
  useEffect(() => {
    if (!enabled || isStaticSource) return;
    if (!playing) return;

    const timer = window.setTimeout(() => {
      loadFloorPlan(
        tRef.current,
        poseRef.current ?? lastPoseRef.current,
        lidarSeq ?? null,
      );
    }, 120);

    return () => {
      window.clearTimeout(timer);
    };
  }, [playing, sessionId, lidarSeq, enabled]);

  useEffect(() => {
    markDirty();
  }, [plan, pathPoints, destinationPoints, selectedDestinationIndex, savedPath]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const onDown = (e: PointerEvent) => {
      dragRef.current = true;
      pointerDownRef.current = { x: e.clientX, y: e.clientY };
      lastPointerRef.current = { x: e.clientX, y: e.clientY };
      canvas.setPointerCapture(e.pointerId);
    };
    const onMove = (e: PointerEvent) => {
      if (!dragRef.current) return;
      const dx = e.clientX - lastPointerRef.current.x;
      const dy = e.clientY - lastPointerRef.current.y;
      if (Math.abs(dx) > 3 || Math.abs(dy) > 3) {
        lastPointerRef.current = { x: e.clientX, y: e.clientY };
        viewRef.current.panX += dx;
        viewRef.current.panY += dy;
        markDirty();
      }
    };
    const onUp = (e: PointerEvent) => {
      const dx = e.clientX - pointerDownRef.current.x;
      const dy = e.clientY - pointerDownRef.current.y;
      const wasClick = Math.abs(dx) < 5 && Math.abs(dy) < 5;
      dragRef.current = false;
      canvas.releasePointerCapture(e.pointerId);

      if (wasClick && alignModeRef.current && onAlignClickRef.current) {
        const current = planRef.current;
        if (!current) return;
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const w = Math.max(1, rect.width);
        const h = Math.max(1, rect.height);
        const { gi, gj } = screenToGrid(mx, my, current, w, h, viewRef.current);
        const wx = current.origin_x + (Math.floor(gi) + 0.5) * current.resolution;
        const wy = current.origin_y + (Math.floor(gj) + 0.5) * current.resolution;
        onAlignClickRef.current(wx, wy);
      } else if (wasClick && annotateModeRef.current) {
        const current = planRef.current;
        if (!current) return;
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const w = Math.max(1, rect.width);
        const h = Math.max(1, rect.height);
        const { gi, gj } = screenToGrid(mx, my, current, w, h, viewRef.current);
        const wx = current.origin_x + (Math.floor(gi) + 0.5) * current.resolution;
        const wy = current.origin_y + (Math.floor(gj) + 0.5) * current.resolution;
        if (onPathPointAddRef.current) {
          onPathPointAddRef.current(wx, wy);
        }
      }
    };
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const rect = canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      const w = Math.max(1, rect.width);
      const h = Math.max(1, rect.height);
      const current = planRef.current;
      if (!current) return;
      const view = viewRef.current;
      const before = screenToGrid(mx, my, current, w, h, view);
      const newZoom = Math.max(0.25, Math.min(8, view.zoom * (1 - e.deltaY * 0.0012)));
      view.zoom = newZoom;
      const after = gridToScreen(before.gi, before.gj, current, w, h, view);
      view.panX += mx - after.x;
      view.panY += my - after.y;
      markDirty();
    };
    const onDblClick = () => {
      viewRef.current = { panX: 0, panY: 0, zoom: 1 };
      markDirty();
    };

    canvas.addEventListener("pointerdown", onDown);
    canvas.addEventListener("pointermove", onMove);
    canvas.addEventListener("pointerup", onUp);
    canvas.addEventListener("pointercancel", onUp);
    canvas.addEventListener("wheel", onWheel, { passive: false });
    canvas.addEventListener("dblclick", onDblClick);

    const draw = () => {
      frameRef.current = requestAnimationFrame(draw);

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

      ctx.fillStyle = `rgb(${BG_RGB[0]}, ${BG_RGB[1]}, ${BG_RGB[2]})`;
      ctx.fillRect(0, 0, w, h);

      const current = planRef.current;
      if (!current) return;

      const view = viewRef.current;
      const scale = fitScale(current, w, h) * view.zoom;
      const mapW = current.width * scale;
      const mapH = current.height * scale;
      const ox = (w - mapW) / 2 + view.panX;
      const oy = (h - mapH) / 2 + view.panY;

      const off = gridCanvasRef.current;
      if (off) {
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(off, ox, oy, mapW, mapH);
      }

      const currentPose = poseRef.current ?? lastPoseRef.current;
      if (currentPose) {
        const { gi, gj } = worldToGrid(currentPose.x, currentPose.y, current);
        const { x: px, y: py } = gridToScreen(gi, gj, current, w, h, view);
        const iconScale = Math.max(0.9, Math.min(1.6, scale * 3.2));
        const targetHeading = poseHeadingOnlyRef.current
          ? -poseHeading2D(currentPose)
          : -floorPlanArrowHeadingOdom(currentPose, {
              velocity: velocityRef.current,
              path: current.path,
            });
        const alpha = poseHeadingOnlyRef.current
          ? playingRef.current
            ? 0.55
            : 1
          : playingRef.current
            ? 0.35
            : 1;
        arrowHeadingRef.current = smoothAngle(arrowHeadingRef.current, targetHeading, alpha);
        drawRobotArrowTopDown(ctx, px, py, arrowHeadingRef.current, iconScale);
      }

      drawPathOverlay(ctx, displayPathRef.current, current, w, h, view, {
        destinations: displayDestinationsRef.current,
        selectedDestinationIndex: selectedDestinationIndexRef.current,
      });

      ctx.fillStyle = "rgba(60, 66, 76, 0.55)";
      ctx.font = "11px Inter, system-ui, sans-serif";
      ctx.textAlign = "left";
      if (variantRef.current === "panel") {
        const hint = alignModeRef.current
          ? "Click where the dog is · drag pan · scroll zoom"
          : onPathPointAddRef.current
          ? "Click to add path points · drag pan · scroll zoom"
          : annotateModeRef.current
          ? "Click map to add points · drag pan · scroll zoom"
          : "Top-down · drag pan · scroll zoom · dbl-click reset";
        ctx.fillText(hint, 10, h - 10);
      }
    };

    markDirty();
    draw();

    const observer = new ResizeObserver(markDirty);
    observer.observe(canvas.parentElement ?? canvas);

    return () => {
      cancelAnimationFrame(frameRef.current);
      observer.disconnect();
      canvas.removeEventListener("pointerdown", onDown);
      canvas.removeEventListener("pointermove", onMove);
      canvas.removeEventListener("pointerup", onUp);
      canvas.removeEventListener("pointercancel", onUp);
      canvas.removeEventListener("wheel", onWheel);
      canvas.removeEventListener("dblclick", onDblClick);
    };
  }, [loading]);

  if (!enabled) {
    return <div className="floorplan-placeholder">Not connected</div>;
  }
  if (loading && !plan) {
    return <div className="floorplan-placeholder">Building floor plan…</div>;
  }
  if (error) {
    return <div className="floorplan-placeholder">{error}</div>;
  }

  return <canvas ref={canvasRef} className={`floorplan-canvas${annotateMode || alignMode ? " annotate-mode" : ""}`} />;
}
