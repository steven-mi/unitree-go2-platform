import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  clearScanPath,
  fetchNavigationStatus,
  fetchScan,
  fetchScanPath,
  localizeScan,
  planScanRoute,
  resetLatestScan,
  saveDestinations,
  saveScanPath,
  startFollowPath,
  startLiveRecording,
  stopLiveRecording,
  stopNavigation,
  syncLatestScan,
  type FramePose,
  type PathPoint,
  type RecordingSession,
  type ScanSession,
} from "../api";
import { CockpitControls } from "../components/CockpitControls";
import { CockpitNavPanel } from "../components/CockpitNavPanel";
import { ReplayPlayer } from "../components/ReplayPlayer";
import { poseHeading2D } from "../components/unitreeDog";
import { useLiveConnection } from "../liveConnection";
import { formatPageTitle } from "../pageTitle";
import { useKeyboardDrive } from "../useKeyboardDrive";

const LATEST = "latest";
const SCAN_SYNC_DEBOUNCE_MS = 2000;
/** While navigating, replan from the live pose on the freshest map at this cadence. */
const GO_REPLAN_MS = 1500;
/** While idle with a destination set, refresh the displayed route at this cadence. */
const IDLE_REPLAN_MS = 3000;
/** Stop when the dog is within this distance (m) of the destination. */
const ARRIVE_M = 0.4;
/** Re-issue navigation only when a new route deviates from the running one by this (m). */
const ROUTE_DIVERGE_M = 0.45;
/** Alignment small enough that live odom == map frame (safe to keep extending the map). */
const ALIGN_IDENTITY_M = 0.05;
const ALIGN_IDENTITY_RAD = 0.02;

/** Rigid odom→map transform: p_map = R(dyaw)·p_odom + (tx, ty). */
interface MapAlignment {
  tx: number;
  ty: number;
  dyaw: number;
}

const ZERO_ALIGN: MapAlignment = { tx: 0, ty: 0, dyaw: 0 };

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => window.setTimeout(resolve, ms));
}

function normalizeAngle(rad: number): number {
  return ((rad + Math.PI) % (2 * Math.PI)) - Math.PI;
}

function isIdentityAlignment(a: MapAlignment): boolean {
  return (
    Math.abs(a.tx) < ALIGN_IDENTITY_M &&
    Math.abs(a.ty) < ALIGN_IDENTITY_M &&
    Math.abs(a.dyaw) < ALIGN_IDENTITY_RAD
  );
}

/** Place the live (odom-frame) pose onto the saved map via the rigid localization transform. */
function mapPoseFromLive(pose: FramePose, a: MapAlignment): FramePose {
  const c = Math.cos(a.dyaw);
  const s = Math.sin(a.dyaw);
  return {
    x: c * pose.x - s * pose.y + a.tx,
    y: s * pose.x + c * pose.y + a.ty,
    z: pose.z,
    yaw: normalizeAngle(poseHeading2D(pose) + a.dyaw),
  };
}

function pointToSegmentDist(p: PathPoint, a: PathPoint, b: PathPoint): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len2 = dx * dx + dy * dy;
  if (len2 < 1e-9) return Math.hypot(p.x - a.x, p.y - a.y);
  let t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

/**
 * True when the freshly planned route follows a different corridor than the one
 * already being driven (i.e. it had to route around a new obstacle). Robust to the
 * moving start point: a route that just advances along the old polyline reads as "same".
 */
function routesDiverge(prev: PathPoint[] | null, next: PathPoint[]): boolean {
  if (!prev || prev.length < 2 || next.length < 2) return true;
  let maxDev = 0;
  for (const p of next) {
    let best = Infinity;
    for (let i = 0; i < prev.length - 1; i++) {
      best = Math.min(best, pointToSegmentDist(p, prev[i], prev[i + 1]));
      if (best <= ROUTE_DIVERGE_M) break;
    }
    if (best > maxDev) maxDev = best;
    if (maxDev > ROUTE_DIVERGE_M) return true;
  }
  return false;
}

interface CockpitPageProps {
  onSessionsChange: () => void;
}

export function CockpitPage({ onSessionsChange }: CockpitPageProps) {
  const live = useLiveConnection(2000, { autoReconnect: true });
  const [recordingBusy, setRecordingBusy] = useState(false);
  const [driveError, setDriveError] = useState<string | null>(null);
  const [keyboardEnabled, setKeyboardEnabled] = useState(false);
  const [scanSyncing, setScanSyncing] = useState(false);
  const [scanResetting, setScanResetting] = useState(false);
  const [latestScan, setLatestScan] = useState<ScanSession | null>(null);
  const [mapReloadKey, setMapReloadKey] = useState(0);

  const [destinations, setDestinations] = useState<PathPoint[]>([]);
  const [routePoints, setRoutePoints] = useState<PathPoint[]>([]);
  const [selectedIndex, setSelectedIndex] = useState(0);
  const [planning, setPlanning] = useState(false);
  const [following, setFollowing] = useState(false);
  const [followProgress, setFollowProgress] = useState<number | null>(null);
  const [followPausedObstacle, setFollowPausedObstacle] = useState(false);
  const [alignment, setAlignment] = useState<MapAlignment>(ZERO_ALIGN);
  const [localizing, setLocalizing] = useState(false);
  const [localizeConfidence, setLocalizeConfidence] = useState<number | null>(null);

  const lastSyncedSessionLidarRef = useRef(0);
  const syncTimerRef = useRef<number | null>(null);
  const syncInFlightRef = useRef(false);
  const replanningRef = useRef(false);
  const followingRef = useRef(false);
  const autoLocalizeRef = useRef(false);

  const poseRef = useRef<FramePose | null>(null);
  poseRef.current = live.frame?.pose ?? null;
  const alignmentRef = useRef(alignment);
  alignmentRef.current = alignment;
  const destinationsRef = useRef(destinations);
  destinationsRef.current = destinations;
  const selectedIndexRef = useRef(selectedIndex);
  selectedIndexRef.current = selectedIndex;
  const connectedRef = useRef(live.liveConnected);
  connectedRef.current = live.liveConnected;

  useEffect(() => {
    document.title = live.robotIp
      ? formatPageTitle(`Cockpit - ${live.robotIp}`)
      : formatPageTitle("Cockpit");
  }, [live.robotIp]);

  const handleDriveError = useCallback((message: string) => {
    setDriveError(message);
  }, []);

  const { pressedKeys } = useKeyboardDrive(live.liveConnected, keyboardEnabled, handleDriveError);

  // Robot pose on the saved map (map frame), via the localization offset.
  const mapPose = useMemo(
    () => (live.frame?.pose ? mapPoseFromLive(live.frame.pose, alignment) : null),
    [live.frame?.pose, alignment],
  );
  /** Current robot pose in the map frame, computed from refs (for async loops). */
  const currentMapPose = useCallback((): FramePose | null => {
    const pose = poseRef.current;
    return pose ? mapPoseFromLive(pose, alignmentRef.current) : null;
  }, []);

  /** Map-frame route → robot odom frame (inverse rigid transform). */
  const toOdomRoute = useCallback((route: PathPoint[]): PathPoint[] => {
    const a = alignmentRef.current;
    const c = Math.cos(a.dyaw);
    const s = Math.sin(a.dyaw);
    return route.map((p) => {
      const x = p.x - a.tx;
      const y = p.y - a.ty;
      return { x: c * x + s * y, y: -s * x + c * y };
    });
  }, []);

  const refreshLatestMeta = useCallback(async () => {
    try {
      const meta = await fetchScan(LATEST);
      setLatestScan(meta);
      const saved = meta.map_alignment;
      setAlignment(
        saved
          ? { tx: saved.tx ?? 0, ty: saved.ty ?? 0, dyaw: saved.dyaw ?? 0 }
          : ZERO_ALIGN,
      );
    } catch {
      setLatestScan(null);
      setAlignment(ZERO_ALIGN);
    }
  }, []);

  const doSync = useCallback(async (): Promise<ScanSession | null> => {
    if (!connectedRef.current || syncInFlightRef.current) return null;
    // A localized-but-shifted map is from another odom frame; extending it with
    // live (odom-frame) lidar would corrupt it. Keep it read-only until Reset.
    if (!isIdentityAlignment(alignmentRef.current)) return null;
    const sessionLidar = live.liveStatus?.lidar_count ?? 0;
    if (sessionLidar === 0) return null;

    syncInFlightRef.current = true;
    setScanSyncing(true);
    try {
      const meta = await syncLatestScan();
      setLatestScan(meta);
      lastSyncedSessionLidarRef.current = sessionLidar;
      setMapReloadKey((k) => k + 1);
      return meta;
    } catch (err) {
      handleDriveError(err instanceof Error ? err.message : "Scan sync failed");
      return null;
    } finally {
      syncInFlightRef.current = false;
      setScanSyncing(false);
    }
  }, [live.liveStatus?.lidar_count, handleDriveError]);

  // Load any saved destinations/route on the active map.
  useEffect(() => {
    fetchScanPath(LATEST)
      .then((data) => {
        setDestinations(data.destinations);
        setRoutePoints(data.route);
      })
      .catch(() => {
        setDestinations([]);
        setRoutePoints([]);
      });
  }, []);

  useEffect(() => {
    if (!live.liveConnected) {
      lastSyncedSessionLidarRef.current = 0;
      autoLocalizeRef.current = false;
      return;
    }
    lastSyncedSessionLidarRef.current = 0;
    void refreshLatestMeta();
  }, [live.liveConnected, refreshLatestMeta]);

  // Continuously write the live scan to scans/latest as new lidar arrives.
  useEffect(() => {
    if (!live.liveConnected) return;
    const sessionLidar = live.liveStatus?.lidar_count ?? 0;
    if (sessionLidar === 0 || sessionLidar <= lastSyncedSessionLidarRef.current) return;

    if (syncTimerRef.current != null) window.clearTimeout(syncTimerRef.current);
    syncTimerRef.current = window.setTimeout(() => {
      syncTimerRef.current = null;
      void doSync();
    }, SCAN_SYNC_DEBOUNCE_MS);

    return () => {
      if (syncTimerRef.current != null) {
        window.clearTimeout(syncTimerRef.current);
        syncTimerRef.current = null;
      }
    };
  }, [live.liveConnected, live.liveStatus?.lidar_count, doSync]);

  useEffect(() => {
    setSelectedIndex((index) => {
      if (destinations.length === 0) return 0;
      return Math.min(index, destinations.length - 1);
    });
  }, [destinations]);

  /**
   * Plan a route (map frame) from `start` to `target`, then persist it alongside the
   * full destination list. Silent replans skip the spinner/error banner.
   */
  const replanRoute = useCallback(
    async (
      start: PathPoint,
      target: PathPoint,
      keepDestinations: PathPoint[],
      opts?: { silent?: boolean },
    ) => {
      if (replanningRef.current) return;
      replanningRef.current = true;
      if (!opts?.silent) {
        setPlanning(true);
        live.setError(null);
      }
      try {
        const result = await planScanRoute(LATEST, start, [target], false);
        const saved = await saveScanPath(LATEST, result.route, keepDestinations);
        setRoutePoints(saved.route);
      } catch (err) {
        if (!opts?.silent) {
          live.setError(err instanceof Error ? err.message : "Path planning failed");
        }
        setRoutePoints([]);
      } finally {
        if (!opts?.silent) setPlanning(false);
        replanningRef.current = false;
      }
    },
    [live],
  );

  // `live` (and thus replanRoute) gets a fresh identity on every live-frame poll
  // (~250ms). Route through a ref so triggerReplan stays stable and the replan
  // effects below fire on real changes only — not on every render.
  const replanRouteRef = useRef(replanRoute);
  replanRouteRef.current = replanRoute;

  const triggerReplan = useCallback(() => {
    if (!connectedRef.current || followingRef.current) return;
    const pose = currentMapPose();
    const dests = destinationsRef.current;
    const dest = dests[selectedIndexRef.current];
    if (!pose || !dest) return;
    void replanRouteRef.current({ x: pose.x, y: pose.y }, dest, dests, { silent: true });
  }, [currentMapPose]);

  const runLocalize = useCallback(
    async (apply = true) => {
      if (!connectedRef.current) return;
      setLocalizing(true);
      live.setError(null);
      try {
        const result = await localizeScan(LATEST, apply);
        // Only trust a confident match — a low-score guess would mis-shift navigation.
        if (result.ok && result.map_alignment) {
          setAlignment({
            tx: result.map_alignment.tx,
            ty: result.map_alignment.ty,
            dyaw: result.map_alignment.dyaw ?? 0,
          });
        }
        setLocalizeConfidence(result.confidence ?? null);
        if (!result.ok) {
          live.setError(
            result.reason === "low_match_score"
              ? "Could not localize on this map — move the dog a little and press Locate again"
              : result.reason === "empty_map"
                ? null
                : "Localization failed",
          );
        } else {
          triggerReplan();
        }
      } catch (err) {
        live.setError(err instanceof Error ? err.message : "Localization failed");
        setLocalizeConfidence(null);
      } finally {
        setLocalizing(false);
      }
    },
    [live, triggerReplan],
  );

  // Auto-localize once shortly after connecting (refines the offset for a reloaded map).
  useEffect(() => {
    if (!live.liveConnected || autoLocalizeRef.current) return;
    autoLocalizeRef.current = true;
    const timer = window.setTimeout(() => void runLocalize(true), 2000);
    return () => window.clearTimeout(timer);
  }, [live.liveConnected, runLocalize]);

  const destinationsKey = destinations.map((d) => `${d.x},${d.y}`).join("|");

  // Replan whenever the selected destination changes.
  useEffect(() => {
    triggerReplan();
  }, [destinationsKey, selectedIndex, triggerReplan]);

  // Keep the displayed route fresh as the dog moves / obstacles change (idle only).
  useEffect(() => {
    if (!live.liveConnected) return;
    const timer = window.setInterval(() => {
      if (followingRef.current) return;
      triggerReplan();
    }, IDLE_REPLAN_MS);
    return () => window.clearInterval(timer);
  }, [live.liveConnected, triggerReplan]);

  const handleAddDestination = useCallback(
    async (x: number, y: number) => {
      const nextDests = [...destinationsRef.current, { x, y }];
      const nextIndex = nextDests.length - 1;
      setDestinations(nextDests);
      setSelectedIndex(nextIndex);
      live.setError(null);
      try {
        const saved = await saveDestinations(LATEST, nextDests);
        setDestinations(saved.destinations);
        const pose = currentMapPose();
        if (connectedRef.current && pose && !followingRef.current) {
          await replanRoute({ x: pose.x, y: pose.y }, { x, y }, saved.destinations);
        } else {
          setRoutePoints([]);
        }
      } catch (err) {
        live.setError(err instanceof Error ? err.message : "Failed to save destination");
      }
    },
    [currentMapPose, live, replanRoute],
  );

  const handleSelectDestination = useCallback(
    (index: number) => {
      if (following || planning || index < 0 || index >= destinations.length) return;
      setSelectedIndex(index);
    },
    [destinations.length, following, planning],
  );

  const handleRemoveDestination = useCallback(
    async (index: number) => {
      if (following || planning || index < 0 || index >= destinations.length) return;
      const nextDests = destinations.filter((_, i) => i !== index);
      let nextIndex = selectedIndex;
      if (index < selectedIndex) nextIndex = selectedIndex - 1;
      else if (index === selectedIndex) {
        nextIndex = Math.min(index, Math.max(0, nextDests.length - 1));
      }
      live.setError(null);
      try {
        if (nextDests.length === 0) {
          await clearScanPath(LATEST);
          setRoutePoints([]);
          setDestinations([]);
          setSelectedIndex(0);
          return;
        }
        const saved = await saveDestinations(LATEST, nextDests);
        setDestinations(saved.destinations);
        setSelectedIndex(nextIndex);
      } catch (err) {
        live.setError(err instanceof Error ? err.message : "Failed to remove destination");
      }
    },
    [destinations, following, live, planning, selectedIndex],
  );

  const handleUndoDestination = useCallback(async () => {
    if (destinations.length === 0 || following || planning) return;
    const nextDests = destinations.slice(0, -1);
    const nextIndex = Math.max(0, Math.min(selectedIndex, nextDests.length - 1));
    live.setError(null);
    try {
      if (nextDests.length === 0) {
        await clearScanPath(LATEST);
        setRoutePoints([]);
        setDestinations([]);
        setSelectedIndex(0);
        return;
      }
      const saved = await saveDestinations(LATEST, nextDests);
      setDestinations(saved.destinations);
      setSelectedIndex(nextIndex);
    } catch (err) {
      live.setError(err instanceof Error ? err.message : "Failed to undo destination");
    }
  }, [destinations, following, live, planning, selectedIndex]);

  const handleClearPath = useCallback(async () => {
    if (destinations.length === 0 || following || planning) return;
    if (!window.confirm("Clear all destinations?")) return;
    live.setError(null);
    try {
      await clearScanPath(LATEST);
      setRoutePoints([]);
      setDestinations([]);
      setSelectedIndex(0);
    } catch (err) {
      live.setError(err instanceof Error ? err.message : "Failed to clear destinations");
    }
  }, [destinations.length, following, live, planning]);

  /**
   * Navigate to the selected destination with continuous replanning: every cycle
   * recompute the map pose, replan on the freshest map, convert the route to the
   * robot's odom frame, and re-issue navigation only when the corridor changed.
   */
  const handleGo = useCallback(async () => {
    if (!live.liveConnected) {
      live.setError("Connect to the robot first");
      return;
    }
    const dest = destinationsRef.current[selectedIndexRef.current];
    if (!dest) {
      live.setError("Click the floor plan to add a destination");
      return;
    }
    if (!currentMapPose()) {
      live.setError("Waiting for robot pose — try again in a moment");
      return;
    }

    followingRef.current = true;
    setFollowing(true);
    setFollowProgress(0);
    setFollowPausedObstacle(false);
    live.setError(null);

    let lastIssued: PathPoint[] | null = null;
    try {
      while (followingRef.current) {
        const pose = currentMapPose();
        if (!pose) {
          await sleep(200);
          continue;
        }
        const distToDest = Math.hypot(dest.x - pose.x, dest.y - pose.y);
        if (distToDest <= ARRIVE_M) break;

        // Freshest obstacles before planning (no-op for a localized, read-only map).
        await doSync();
        if (!followingRef.current) break;

        let route: PathPoint[];
        try {
          const result = await planScanRoute(LATEST, { x: pose.x, y: pose.y }, [dest], false);
          route = result.route;
        } catch {
          live.setError("Blocked — waiting for a clear path…");
          setFollowPausedObstacle(true);
          await stopNavigation().catch(() => {});
          lastIssued = null;
          await sleep(GO_REPLAN_MS);
          continue;
        }
        if (!followingRef.current) break;
        if (route.length < 2) {
          if (distToDest <= ARRIVE_M * 1.5) break;
          live.setError("Blocked — waiting for a clear path…");
          setFollowPausedObstacle(true);
          await stopNavigation().catch(() => {});
          lastIssued = null;
          await sleep(GO_REPLAN_MS);
          continue;
        }
        setFollowPausedObstacle(false);
        live.setError(null);
        setRoutePoints(route);
        void saveScanPath(LATEST, route, destinationsRef.current).catch(() => {});

        const status = lastIssued ? await fetchNavigationStatus().catch(() => null) : null;
        if (status) {
          setFollowProgress(status.completed);
          setFollowPausedObstacle(status.paused_obstacle);
          if (!status.active && status.ok === false && status.status !== "cancelled") {
            live.setError(status.error ?? `Navigation ${status.status}`);
            break;
          }
        }

        const navActive = status?.active ?? false;
        if (!navActive || routesDiverge(lastIssued, route)) {
          await startFollowPath(toOdomRoute(route), { mapFrame: false });
          lastIssued = route;
        }

        await sleep(GO_REPLAN_MS);
      }
    } catch (err) {
      live.setError(err instanceof Error ? err.message : "Path navigation failed");
    } finally {
      followingRef.current = false;
      await stopNavigation().catch(() => {});
      setFollowing(false);
      setFollowProgress(null);
      setFollowPausedObstacle(false);
    }
  }, [currentMapPose, doSync, live, toOdomRoute]);

  const handleStop = useCallback(async () => {
    followingRef.current = false;
    try {
      await stopNavigation();
      live.setError(null);
    } catch (err) {
      live.setError(err instanceof Error ? err.message : "Failed to stop");
    } finally {
      setFollowing(false);
      setFollowProgress(null);
      setFollowPausedObstacle(false);
    }
  }, [live]);

  const handleResetScan = useCallback(async () => {
    if (!live.liveConnected) return;
    if (
      !window.confirm("Save the current scan with today's date and start a fresh map?")
    ) {
      return;
    }

    if (followingRef.current) await handleStop();
    setScanResetting(true);
    live.setError(null);
    try {
      const result = await resetLatestScan();
      setLatestScan(result.latest);
      lastSyncedSessionLidarRef.current = live.liveStatus?.lidar_count ?? 0;
      setDestinations([]);
      setRoutePoints([]);
      setSelectedIndex(0);
      setAlignment(ZERO_ALIGN);
      setLocalizeConfidence(null);
      setMapReloadKey((k) => k + 1);
    } catch (err) {
      live.setError(err instanceof Error ? err.message : "Failed to reset scan");
    } finally {
      setScanResetting(false);
    }
  }, [handleStop, live]);

  const handleToggleRecording = useCallback(async () => {
    if (!live.liveStatus) return;
    setRecordingBusy(true);
    live.setError(null);
    try {
      if (live.liveStatus.recording) {
        await stopLiveRecording();
        onSessionsChange();
      } else {
        await startLiveRecording();
      }
      await live.refreshStatus();
    } catch (err) {
      live.setError(err instanceof Error ? err.message : "Recording failed");
    } finally {
      setRecordingBusy(false);
    }
  }, [live, onSessionsChange]);

  const session: RecordingSession = {
    id: live.liveStatus?.session_id ?? "live",
    duration_s: live.liveStatus?.duration_s,
    lidar_count: live.liveStatus?.lidar_count,
    video_count: live.liveStatus?.video_count,
  };

  const error = live.error ?? driveError;
  const savedLidarCount = latestScan?.lidar_count ?? 0;

  return (
    <div className="cockpit-page">
      {error && <div className="live-banner error">{error}</div>}
      {localizing && (
        <div className="live-banner info">Estimating the dog’s position on the map…</div>
      )}
      {planning && !following && (
        <div className="live-banner info">Planning route to destination {selectedIndex + 1}…</div>
      )}
      {following && (
        <div className={`live-banner ${followPausedObstacle ? "warn" : "info"}`}>
          {followPausedObstacle
            ? "Paused — obstacle ahead. Waiting for a clear path…"
            : `Going to destination ${selectedIndex + 1}${
                followProgress != null && routePoints.length > 0
                  ? ` · waypoint ${followProgress}/${routePoints.length}`
                  : ""
              }`}
        </div>
      )}

      <ReplayPlayer
        mode="live"
        session={session}
        liveStatus={live.liveStatus}
        robotIp={live.robotIp}
        cockpitTitlePrefix="Cockpit"
        onToggleConnection={() => void live.handleConnectionToggle()}
        onToggleRecording={() => void handleToggleRecording()}
        onResetScan={() => void handleResetScan()}
        scanSyncing={scanSyncing}
        scanResetting={scanResetting}
        savedScanLidarCount={savedLidarCount}
        connecting={live.connecting}
        disconnecting={live.disconnecting}
        recordingBusy={recordingBusy}
        keyboardEnabled={keyboardEnabled}
        onKeyboardEnabledChange={setKeyboardEnabled}
        pressedKeys={pressedKeys}
        navigation={{
          source: "scan",
          sessionId: LATEST,
          pose: mapPose,
          poseHeadingOnly: true,
          reloadKey: mapReloadKey,
          annotate: true,
          routePoints,
          destinations,
          selectedDestinationIndex: selectedIndex,
          onPointAdd: (x, y) => void handleAddDestination(x, y),
        }}
        panelFooter={
          <div className="cockpit-footer">
            <CockpitNavPanel
              connected={live.liveConnected}
              destinations={destinations}
              selectedIndex={selectedIndex}
              routePoints={routePoints}
              planning={planning}
              following={following}
              localizing={localizing}
              localizeConfidence={localizeConfidence}
              hasPose={Boolean(mapPose)}
              onSelect={handleSelectDestination}
              onRemove={(i) => void handleRemoveDestination(i)}
              onUndo={() => void handleUndoDestination()}
              onClear={() => void handleClearPath()}
              onLocate={() => void runLocalize(true)}
              onGo={() => void handleGo()}
              onStop={() => void handleStop()}
            />
            <CockpitControls connected={live.liveConnected} onError={handleDriveError} />
          </div>
        }
      />
    </div>
  );
}
