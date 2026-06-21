import { useCallback, useEffect, useRef, useState, type ReactNode } from "react";
import { Circle, Maximize2, Minimize2, Pause, Play, RotateCcw, Square } from "lucide-react";
import type { DataSource, FramePose, LiveStatus, PathPoint, RecordingSession, ReplayFrame, SessionDetail } from "../api";
import {
  fetchFrame,
  fetchLiveFrame,
  fetchLiveSessionDetail,
  fetchSessionDetail,
  formatDuration,
  formatSessionLabel,
  formatTime,
} from "../api";
import { FloorPlanView } from "./FloorPlanView";
import { PointCloudView, useLidarPointsBinary } from "./PointCloudView";
import { TelemetryPanel } from "./TelemetryPanel";
import { SessionInfoPanel } from "./SessionInfoPanel";
import { RecordingTags } from "./RecordingTags";
import { ConnectionButton } from "./ConnectionButton";
import { formatConnectedPageHeading } from "../pageTitle";

const PLAY_FETCH_INTERVAL_MS = 100;

interface TimelineControlsProps {
  playing: boolean;
  t: number;
  duration: number;
  onTogglePlay: () => void;
  onScrub: (value: number) => void;
}

function TimelineControls({
  playing,
  t,
  duration,
  onTogglePlay,
  onScrub,
}: TimelineControlsProps) {
  return (
    <div className="timeline-row">
      <button
        type="button"
        className="play-btn"
        onClick={onTogglePlay}
        aria-label={playing ? "Pause" : "Play"}
      >
        {playing ? <Pause size={16} fill="white" /> : <Play size={16} fill="white" style={{ marginLeft: 2 }} />}
      </button>
      <input
        type="range"
        className="timeline-slider"
        min={0}
        max={duration}
        step={0.05}
        value={t}
        onChange={(e) => onScrub(parseFloat(e.target.value))}
      />
      <span className="timeline-time">
        {formatTime(t)} / {formatTime(duration)}
      </span>
    </div>
  );
}

interface ReplayPlayerProps {
  session: RecordingSession;
  mode?: "replay" | "live";
  liveStatus?: LiveStatus | null;
  robotIp?: string;
  cockpitTitlePrefix?: string;
  onToggleRecording?: () => void;
  onToggleConnection?: () => void;
  onResetScan?: () => void;
  scanSyncing?: boolean;
  scanResetting?: boolean;
  savedScanLidarCount?: number;
  connecting?: boolean;
  disconnecting?: boolean;
  recordingBusy?: boolean;
  onTagsChange?: (tags: string[]) => void;
  panelFooter?: ReactNode;
  keyboardEnabled?: boolean;
  onKeyboardEnabledChange?: (enabled: boolean) => void;
  pressedKeys?: ReadonlySet<string>;
  /**
   * Point & Go: render the floor-plan panel from a saved map (map frame) with a
   * localized pose, click-to-place destinations, and a planned route overlay.
   * Overrides the default live floor plan when in live mode.
   */
  navigation?: CockpitNavigationFloorPlan;
}

export interface CockpitNavigationFloorPlan {
  source: DataSource;
  sessionId: string;
  pose: FramePose | null;
  poseHeadingOnly?: boolean;
  reloadKey?: number;
  annotate?: boolean;
  routePoints?: PathPoint[];
  destinations?: PathPoint[];
  selectedDestinationIndex?: number;
  onPointAdd?: (x: number, y: number) => void;
}

export function ReplayPlayer({
  session,
  mode = "replay",
  liveStatus = null,
  robotIp = "",
  cockpitTitlePrefix = "Cockpit",
  onToggleRecording,
  onToggleConnection,
  onResetScan,
  scanSyncing = false,
  scanResetting = false,
  savedScanLidarCount = 0,
  connecting = false,
  disconnecting = false,
  recordingBusy = false,
  onTagsChange,
  panelFooter,
  keyboardEnabled,
  onKeyboardEnabledChange,
  pressedKeys,
  navigation,
}: ReplayPlayerProps) {
  const isLive = mode === "live";
  const nav = isLive ? navigation : undefined;
  const dataSource = isLive ? "live" : "recording";
  const [frame, setFrame] = useState<ReplayFrame | null>(null);
  const [sessionDetail, setSessionDetail] = useState<SessionDetail | null>(null);
  const [t, setT] = useState(0);
  const [playing, setPlaying] = useState(isLive);
  const [lidarFullscreen, setLidarFullscreen] = useState(false);
  const [tags, setTags] = useState<string[]>(session.tags ?? []);
  const lidarFsRef = useRef<HTMLDivElement>(null);
  const rafRef = useRef(0);
  const lastTickRef = useRef(0);
  const tRef = useRef(0);
  const frameGenRef = useRef(0);
  const frameAbortRef = useRef<AbortController | null>(null);

  tRef.current = t;

  const liveConnected = !isLive || Boolean(liveStatus?.connected);

  const lidarPoints3d = useLidarPointsBinary(
    session.id,
    frame?.lidar?.seq ?? null,
    liveConnected,
    dataSource,
  );
  const duration = frame?.duration ?? session.duration_s ?? (isLive ? liveStatus?.duration_s : 1) ?? 1;

  useEffect(() => {
    setT(0);
    setPlaying(isLive && liveConnected);
    setLidarFullscreen(false);
    setFrame(null);
    setSessionDetail(null);
    frameAbortRef.current?.abort();
    if (isLive) {
      if (!liveConnected) return;
      fetchLiveSessionDetail().then(setSessionDetail).catch(() => setSessionDetail(null));
    } else {
      fetchSessionDetail(session.id).then(setSessionDetail).catch(() => setSessionDetail(null));
    }
  }, [session.id, isLive, liveConnected]);

  useEffect(() => {
    setTags(session.tags ?? []);
  }, [session.id, session.tags]);

  const handleTagsChange = useCallback(
    (next: string[]) => {
      setTags(next);
      onTagsChange?.(next);
    },
    [onTagsChange],
  );

  const loadFrame = useCallback((time?: number) => {
    if (isLive && !liveConnected) return;
    frameAbortRef.current?.abort();
    const controller = new AbortController();
    frameAbortRef.current = controller;
    const gen = ++frameGenRef.current;

    const request = isLive
      ? fetchLiveFrame(time, controller.signal)
      : fetchFrame(session.id, time ?? tRef.current, controller.signal);

    request
      .then((next) => {
        if (gen !== frameGenRef.current) return;
        setFrame(next);
        if (isLive && playing && time == null) {
          setT(next.t);
        }
      })
      .catch((err) => {
        if (gen !== frameGenRef.current || err.name === "AbortError") return;
      });
  }, [session.id, isLive, playing, liveConnected]);

  useEffect(() => {
    if (isLive && !liveConnected) return;
    if (playing) return;
    loadFrame(t);
    return () => frameAbortRef.current?.abort();
  }, [playing, t, loadFrame, liveConnected, isLive]);

  useEffect(() => {
    if (isLive && !liveConnected) return;
    if (!playing) return;
    if (isLive) {
      loadFrame();
      const id = window.setInterval(() => loadFrame(), PLAY_FETCH_INTERVAL_MS);
      return () => {
        window.clearInterval(id);
        frameAbortRef.current?.abort();
      };
    }
    loadFrame(tRef.current);
    const id = window.setInterval(() => loadFrame(tRef.current), PLAY_FETCH_INTERVAL_MS);
    return () => {
      window.clearInterval(id);
      frameAbortRef.current?.abort();
    };
  }, [playing, loadFrame, isLive, liveConnected]);

  useEffect(() => {
    if (!liveConnected && isLive) {
      setPlaying(false);
      return;
    }
    if (liveConnected && isLive) {
      setPlaying(true);
    }
  }, [liveConnected, isLive]);

  useEffect(() => {
    if (!playing || isLive) return;
    lastTickRef.current = performance.now();

    const loop = (now: number) => {
      const dt = (now - lastTickRef.current) / 1000;
      lastTickRef.current = now;
      setT((prev) => {
        const next = prev + dt;
        if (next >= duration) {
          setPlaying(false);
          return duration;
        }
        return next;
      });
      rafRef.current = requestAnimationFrame(loop);
    };

    rafRef.current = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(rafRef.current);
  }, [playing, duration, isLive]);

  const onScrub = useCallback((value: number) => {
    setT(value);
  }, []);

  const togglePlay = useCallback(() => {
    setPlaying((p) => !p);
  }, []);

  const toggleLidarFullscreen = useCallback(async () => {
    const el = lidarFsRef.current;
    if (!el) return;
    try {
      if (document.fullscreenElement === el) {
        await document.exitFullscreen();
      } else {
        await el.requestFullscreen();
      }
    } catch {
      // Fullscreen API unavailable or denied.
    }
  }, []);

  useEffect(() => {
    const onFullscreenChange = () => {
      setLidarFullscreen(document.fullscreenElement === lidarFsRef.current);
    };
    document.addEventListener("fullscreenchange", onFullscreenChange);
    return () => document.removeEventListener("fullscreenchange", onFullscreenChange);
  }, []);

  const pose = frame?.pose;
  const videoUrl = frame?.video?.url ?? null;
  const title = isLive
    ? formatConnectedPageHeading(
        cockpitTitlePrefix,
        robotIp,
        liveStatus?.duration_s,
        Boolean(liveStatus?.connected),
      )
    : formatSessionLabel(session);

  const meta = isLive
    ? [
        liveStatus?.recording ? "Recording" : null,
        scanSyncing ? "Saving scan…" : savedScanLidarCount > 0 ? `${savedScanLidarCount} scans saved` : null,
        `${liveStatus?.lidar_count ?? 0} lidar`,
        `${liveStatus?.video_count ?? 0} video`,
        frame?.battery?.soc != null ? `${frame.battery.soc}% battery` : null,
      ]
        .filter(Boolean)
        .join(" · ")
    : `${formatDuration(session.duration_s)} · ${session.lidar_count ?? 0} lidar · ${session.video_count ?? 0} video${session.interrupted ? " · interrupted" : ""}`;

  const lidarToolbar = (
    <>
      {frame?.lidar?.point_count != null && (
        <span className="panel-point-count">
          {frame.lidar.point_count.toLocaleString()} pts
        </span>
      )}
      <button
        type="button"
        className="panel-fs-btn"
        onClick={toggleLidarFullscreen}
        aria-label={lidarFullscreen ? "Exit fullscreen" : "Enter fullscreen"}
        title={lidarFullscreen ? "Exit fullscreen" : "Fullscreen"}
      >
        {lidarFullscreen ? <Minimize2 size={14} /> : <Maximize2 size={14} />}
      </button>
    </>
  );

  const lidarBody = (
    <PointCloudView
      points={lidarPoints3d}
      pose={pose ?? null}
      videoUrl={videoUrl}
      layoutKey={lidarFullscreen ? "fullscreen" : "inline"}
    />
  );

  return (
    <div className={`replay${isLive ? " replay-live" : " replay-recording"}`}>
      <div className="replay-header">
        <div className="replay-header-main">
          <h2>{title}</h2>
          {meta ? <div className="replay-meta">{meta}</div> : null}
          {!isLive && (
            <RecordingTags
              sessionId={session.id}
              tags={tags}
              onChange={handleTagsChange}
            />
          )}
        </div>
        {isLive && (onToggleConnection || onToggleRecording || onResetScan) && (
          <div className="replay-header-actions">
            {onToggleConnection && (
              <ConnectionButton
                liveStatus={liveStatus}
                connecting={connecting}
                disconnecting={disconnecting}
                onClick={onToggleConnection}
              />
            )}
            {onResetScan && (
              <button
                type="button"
                className="reset-scan-btn"
                onClick={onResetScan}
                disabled={scanResetting || scanSyncing || !liveStatus?.connected}
                title="Archive the current scan and start fresh"
              >
                <RotateCcw size={14} />
                {scanResetting ? "Resetting…" : "Reset scan"}
              </button>
            )}
            {onToggleRecording && (
              <button
                type="button"
                className={`record-btn${liveStatus?.recording ? " recording" : ""}`}
                onClick={onToggleRecording}
                disabled={recordingBusy || !liveStatus?.connected}
              >
                {liveStatus?.recording ? (
                  <>
                    <Square size={14} fill="currentColor" />
                    Stop recording
                  </>
                ) : (
                  <>
                    <Circle size={14} fill="currentColor" />
                    Start recording
                  </>
                )}
              </button>
            )}
          </div>
        )}
      </div>

      <div className="replay-panels">
        {!liveConnected && isLive ? (
          <>
            <div className="panel panel-floorplan">
              <div className="panel-label">Floor plan</div>
              <div className="panel-body panel-body-floorplan live-panel-empty">
                {liveStatus?.state === "connecting" ? "Connecting…" : "Waiting for connection…"}
              </div>
            </div>
            <div className="panel panel-lidar">
              <div className="panel-label">
                <span className="panel-label-title">Lidar</span>
              </div>
              <div className="panel-body live-panel-empty">
                {liveStatus?.state === "connecting" ? "Connecting…" : "Waiting for connection…"}
              </div>
            </div>
          </>
        ) : (
          <>
        <div className="panel panel-floorplan">
          <div className="panel-label">
            <span className="panel-label-title">Floor plan</span>
            {isLive && nav?.onPointAdd && (
              <span className="floorplan-hint-badge">Click to set destination</span>
            )}
          </div>
          <div className="panel-body panel-body-floorplan">
            {nav ? (
              <FloorPlanView
                sessionId={nav.sessionId}
                pose={nav.pose}
                velocity={null}
                t={t}
                playing={playing}
                source={nav.source}
                enabled={liveConnected}
                reloadKey={nav.reloadKey}
                poseHeadingOnly={nav.poseHeadingOnly}
                annotateMode={nav.annotate}
                pathPoints={nav.routePoints}
                destinationPoints={nav.destinations}
                selectedDestinationIndex={nav.selectedDestinationIndex}
                onPathPointAdd={nav.onPointAdd}
              />
            ) : (
              <FloorPlanView
                sessionId={session.id}
                pose={pose ?? null}
                velocity={frame?.velocity ?? null}
                t={t}
                playing={playing}
                lidarSeq={frame?.lidar?.seq ?? null}
                source={dataSource}
                enabled={liveConnected}
              />
            )}
          </div>
        </div>

        <div
          ref={lidarFsRef}
          className={`panel panel-lidar${lidarFullscreen ? " panel-lidar-fullscreen" : ""}`}
        >
          <div className={`panel-label${lidarFullscreen ? " panel-label-dark" : ""}`}>
            <span className="panel-label-title">Lidar</span>
            {lidarToolbar}
          </div>
          <div className="panel-body">
            {lidarBody}
            {lidarFullscreen && (
              <div className="lidar-fs-pip lidar-fs-pip--floorplan">
                <div className="lidar-fs-pip-label">Floor plan</div>
                <div className="lidar-fs-pip-body">
                  <FloorPlanView
                    sessionId={session.id}
                    pose={pose ?? null}
                    velocity={frame?.velocity ?? null}
                    t={t}
                    playing={playing}
                    lidarSeq={frame?.lidar?.seq ?? null}
                    variant="overlay"
                    source={dataSource}
                    enabled={liveConnected}
                  />
                </div>
              </div>
            )}
          </div>
          {lidarFullscreen && (
            <div className="lidar-fs-footer">
              <div className="lidar-fs-title">{title}</div>
              {!isLive && (
                <TimelineControls
                  playing={playing}
                  t={t}
                  duration={duration}
                  onTogglePlay={togglePlay}
                  onScrub={onScrub}
                />
              )}
            </div>
          )}
        </div>
          </>
        )}
      </div>

      {panelFooter}

      {!isLive && (
        <div className="timeline">
          <TimelineControls
            playing={playing}
            t={t}
            duration={duration}
            onTogglePlay={togglePlay}
            onScrub={onScrub}
          />
        </div>
      )}

      <div className="telemetry-grid telemetry-below-timeline">
        <TelemetryPanel
          frame={frame}
          pose={pose ?? null}
          section="primary"
          bare
          keyboardEnabled={keyboardEnabled}
          onKeyboardEnabledChange={onKeyboardEnabledChange}
          pressedKeys={pressedKeys}
        />
        <TelemetryPanel frame={frame} pose={pose ?? null} section="secondary" bare />
        <SessionInfoPanel detail={sessionDetail} bare />
      </div>
    </div>
  );
}
