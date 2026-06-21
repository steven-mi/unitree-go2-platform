export interface LiveStatus {
  state: string;
  connected: boolean;
  recording: boolean;
  session_id: string | null;
  error: string | null;
  robot_ip: string;
  duration_s: number;
  lidar_count: number;
  video_count: number;
}

export interface AppSettings {
  robot_ip: string;
  aes_128_key: string | null;
}

export type DataSource = "recording" | "live" | "scan";

export interface RecordingSession {
  id: string;
  note?: string;
  tags?: string[];
  duration_s?: number;
  created_at?: string;
  lidar_count?: number;
  video_count?: number;
  interrupted?: boolean;
}

export interface FramePose {
  x: number;
  y: number;
  z: number;
  yaw: number | null;
  qx?: number;
  qy?: number;
  qz?: number;
  qw?: number;
}

export interface ImuState {
  rpy: number[] | null;
  gyro: number[] | null;
  accel: number[] | null;
  temperature: number | null;
}

export interface SportState {
  mode: number;
  gait_type: number;
  body_height: number;
  yaw_rate: number;
  error_code: number;
  range_obstacle: number[] | null;
  imu: ImuState;
}

export interface MotorState {
  angle: number;
  temperature: number | null;
}

export interface BatteryState {
  voltage: number | null;
  soc: number | null;
  current_ma: number | null;
  temperature_c: number | null;
  foot_force: number[] | null;
  motors: MotorState[] | null;
}

export interface LidarSensorState {
  cloud_frequency: number | null;
  error_state: number | null;
  dirty_percentage: number | null;
  cloud_size: number | null;
}

export interface UwbState {
  distance: number | null;
  yaw: number | null;
  pitch: number | null;
  orientation: number | null;
  joystick: number[] | null;
  buttons: number | null;
  joy_mode: number | null;
  enabled_from_app: boolean;
}

export interface SystemState {
  volume: number | null;
  brightness: number | null;
  obstacles_avoid: boolean | null;
  uwb_switch: boolean | null;
}

export interface AudioState {
  play_state: string;
  is_playing: boolean;
  track: string | null;
}

export interface RobotService {
  name: string;
  status: number | null;
  version: string;
}

export interface SessionDetail {
  id: string;
  duration: number;
  rpc: Record<string, Record<string, unknown>>;
  services: RobotService[];
}

export interface ReplayFrame {
  t: number;
  duration: number;
  video: { seq: number; file: string; url: string } | null;
  lidar: { seq: number; file: string; point_count: number; url: string } | null;
  pose: FramePose | null;
  velocity: number[] | null;
  battery_v: number | null;
  sport: SportState | null;
  battery: BatteryState | null;
  lidar_state: LidarSensorState | null;
  uwb: UwbState | null;
  system: SystemState | null;
  audio: AudioState | null;
}

export interface FloorPlan {
  width: number;
  height: number;
  origin_x: number;
  origin_y: number;
  resolution: number;
  scan_count: number;
  threshold: number;
  zone_count: number;
  map_rotation: number;
  zones: Uint8Array;
  walls: Uint8Array;
  path: { x: number; y: number }[];
  upto_t?: number;
}

export interface ScanSession {
  id: string;
  name?: string;
  note?: string;
  created_at?: string;
  updated_at?: string;
  source_session_id?: string | null;
  scan_count?: number;
  lidar_count?: number;
  path_point_count?: number;
  odom_origin?: { x: number; y: number; yaw?: number };
  map_alignment?: { tx: number; ty: number; dyaw?: number };
  archived_at?: string;
  restored_from?: string;
  restored_at?: string;
}

export interface PathPoint {
  x: number;
  y: number;
}

function decodeGrid(b64: string, size: number): Uint8Array {
  const raw = atob(b64);
  const buf = new Uint8Array(size);
  for (let i = 0; i < size && i < raw.length; i++) {
    buf[i] = raw.charCodeAt(i);
  }
  return buf;
}

export async function fetchFloorPlan(
  id: string,
  t?: number,
  pose?: { x: number; y: number } | null,
  signal?: AbortSignal,
  lidarSeq?: number | null,
  source: DataSource = "recording",
): Promise<FloorPlan> {
  const params = new URLSearchParams();
  if (t != null) params.set("t", String(t));
  if (pose) {
    params.set("x", String(pose.x));
    params.set("y", String(pose.y));
  }
  if (lidarSeq != null) params.set("lidar_seq", String(lidarSeq));
  const qs = params.toString();
  const base =
    source === "live"
      ? "/api/live/floorplan"
      : source === "scan"
        ? `/api/scans/${id}/floorplan`
        : `/api/recordings/${id}/floorplan`;
  const res = await fetch(`${base}${qs ? `?${qs}` : ""}`, { signal });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Floor plan not available");
  }
  const data = await res.json();
  const size = data.width * data.height;
  return {
    width: data.width,
    height: data.height,
    origin_x: data.origin_x,
    origin_y: data.origin_y,
    resolution: data.resolution,
    scan_count: data.scan_count,
    threshold: data.threshold,
    zone_count: data.zone_count ?? 0,
    map_rotation: data.map_rotation ?? 0,
    zones: decodeGrid(data.zones_b64, size),
    walls: decodeGrid(data.walls_b64, size),
    path: data.path ?? [],
    upto_t: data.upto_t,
  };
}

export async function fetchRecordings(): Promise<RecordingSession[]> {
  const res = await fetch("/api/recordings");
  const data = await res.json();
  return data.sessions;
}

export async function updateRecordingTags(sessionId: string, tags: string[]): Promise<string[]> {
  const res = await fetch(`/api/recordings/${sessionId}/tags`, {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ tags }),
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Failed to update tags");
  }
  const data = await res.json();
  return data.tags ?? [];
}

export async function deleteRecording(sessionId: string): Promise<void> {
  const res = await fetch(`/api/recordings/${sessionId}`, { method: "DELETE" });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Failed to delete recording");
  }
}

export async function fetchSessionDetail(id: string): Promise<SessionDetail> {
  const res = await fetch(`/api/recordings/${id}`);
  if (!res.ok) throw new Error("Session not found");
  const data = await res.json();
  return {
    id: data.id,
    duration: data.duration,
    rpc: data.rpc ?? {},
    services: data.services ?? [],
  };
}

export async function fetchFrame(
  id: string,
  t: number,
  signal?: AbortSignal,
): Promise<ReplayFrame> {
  const res = await fetch(`/api/recordings/${id}/frame?t=${t}`, { signal });
  if (!res.ok) throw new Error("Frame not found");
  return res.json();
}

export async function fetchLidarBinary(
  id: string,
  seq: number,
  maxPoints = 0,
  source: DataSource = "recording",
): Promise<Float32Array> {
  const base = source === "live" ? `/api/live/lidar/${seq}` : `/api/recordings/${id}/lidar/${seq}`;
  const res = await fetch(`${base}?max_points=${maxPoints}`);
  if (!res.ok) throw new Error("Lidar not found");
  const buf = await res.arrayBuffer();
  return new Float32Array(buf);
}

export async function fetchLiveStatus(): Promise<LiveStatus> {
  const res = await fetch("/api/live/status");
  if (!res.ok) throw new Error("Live status unavailable");
  return res.json();
}

export async function fetchSettings(): Promise<AppSettings> {
  const res = await fetch("/api/settings");
  if (!res.ok) throw new Error("Settings unavailable");
  return res.json();
}

export async function saveSettings(settings: {
  robot_ip: string;
  aes_128_key?: string | null;
}): Promise<AppSettings> {
  const res = await fetch("/api/settings", {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      robot_ip: settings.robot_ip,
      aes_128_key: settings.aes_128_key ?? "",
    }),
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Failed to save settings");
  }
  return res.json();
}

export async function connectLive(ip?: string): Promise<LiveStatus> {
  const qs = ip ? `?ip=${encodeURIComponent(ip)}` : "";
  const res = await fetch(`/api/live/connect${qs}`, { method: "POST" });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Failed to connect");
  }
  return res.json();
}

export async function disconnectLive(): Promise<LiveStatus> {
  const res = await fetch("/api/live/disconnect", { method: "POST" });
  if (!res.ok) throw new Error("Failed to disconnect");
  return res.json();
}

export interface NavigationStatus {
  active: boolean;
  ok: boolean | null;
  status: string;
  completed: number;
  total: number;
  failed_at: number | null;
  mode: string;
  error: string | null;
  paused_obstacle: boolean;
}

export async function fetchNavigationStatus(): Promise<NavigationStatus> {
  const res = await fetch("/api/live/navigation");
  if (!res.ok) throw new Error("Failed to fetch navigation status");
  return res.json();
}

/**
 * Start path following and return immediately (does not poll to completion).
 * Use with `fetchNavigationStatus` for closed-loop / dynamic replanning control.
 * Re-issuing replaces any in-flight run on the server.
 */
export async function startFollowPath(
  points: PathPoint[],
  opts?: { mapFrame?: boolean },
): Promise<{ total: number }> {
  const res = await fetch("/api/live/follow-path", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ points, map_frame: opts?.mapFrame ?? false }),
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Failed to follow path");
  }
  const data = await res.json();
  return { total: Number(data.total) || points.length };
}

export async function stopNavigation(): Promise<void> {
  await fetch("/api/live/stop-navigation", { method: "POST" });
}

export async function stopDrive(): Promise<void> {
  const res = await fetch("/api/live/drive/stop", { method: "POST" });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Failed to stop drive");
  }
}

export async function sendSportCommand(
  command: string,
  parameter?: Record<string, unknown>,
): Promise<void> {
  const res = await fetch("/api/live/sport", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ command, parameter: parameter ?? null }),
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? `Sport command ${command} failed`);
  }
}

export async function startLiveRecording(name = "", note = ""): Promise<LiveStatus> {
  const params = new URLSearchParams();
  if (name) params.set("name", name);
  if (note) params.set("note", note);
  const qs = params.toString();
  const res = await fetch(`/api/live/recording/start${qs ? `?${qs}` : ""}`, { method: "POST" });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Failed to start recording");
  }
  return res.json();
}

export async function stopLiveRecording(): Promise<{ ok: boolean; session_id: string }> {
  const res = await fetch("/api/live/recording/stop", { method: "POST" });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Failed to stop recording");
  }
  return res.json();
}

export async function fetchLiveFrame(t?: number, signal?: AbortSignal): Promise<ReplayFrame> {
  const qs = t != null ? `?t=${t}` : "";
  const res = await fetch(`/api/live/frame${qs}`, { signal });
  if (!res.ok) {
    const err = new Error(
      res.status === 503 ? "Not connected to robot" : "Live frame unavailable",
    ) as Error & { status?: number };
    err.status = res.status;
    throw err;
  }
  return res.json();
}

export async function fetchLiveSessionDetail(): Promise<SessionDetail> {
  const res = await fetch("/api/live/session");
  if (!res.ok) throw new Error("Live session unavailable");
  const data = await res.json();
  return {
    id: data.id,
    duration: data.duration,
    rpc: data.rpc ?? {},
    services: data.services ?? [],
  };
}

const SESSION_ID_TIMESTAMP_RE = /^(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})/;

function formatSessionTimestampFromId(id: string): string | null {
  const match = id.match(SESSION_ID_TIMESTAMP_RE);
  if (!match) return null;
  const [, year, month, day, hour, minute, second] = match;
  return `${year}-${month}-${day} ${hour}:${minute}:${second}`;
}

function formatSessionTimestampFromIso(iso: string): string | null {
  const date = new Date(iso);
  if (Number.isNaN(date.getTime())) return null;
  const year = date.getFullYear();
  const month = String(date.getMonth() + 1).padStart(2, "0");
  const day = String(date.getDate()).padStart(2, "0");
  const hour = String(date.getHours()).padStart(2, "0");
  const minute = String(date.getMinutes()).padStart(2, "0");
  const second = String(date.getSeconds()).padStart(2, "0");
  return `${year}-${month}-${day} ${hour}:${minute}:${second}`;
}

export function formatSessionLabel(s: RecordingSession): string {
  const timestamp =
    (s.created_at && formatSessionTimestampFromIso(s.created_at)) ||
    formatSessionTimestampFromId(s.id);
  if (timestamp) {
    if (s.note) return `${timestamp} — ${s.note}`;
    return timestamp;
  }
  return s.id;
}

export function formatDuration(sec?: number): string {
  if (!sec) return "—";
  const m = Math.floor(sec / 60);
  const s = Math.round(sec % 60);
  return m > 0 ? `${m}m ${s}s` : `${s}s`;
}

export function formatTime(t: number): string {
  const m = Math.floor(t / 60);
  const s = Math.floor(t % 60);
  const d = Math.floor((t % 1) * 10);
  return m > 0 ? `${m}:${String(s).padStart(2, "0")}.${d}` : `${s}.${d}s`;
}

export async function fetchScans(): Promise<ScanSession[]> {
  const res = await fetch("/api/scans");
  if (!res.ok) throw new Error("Failed to load scans");
  const data = await res.json();
  return data.scans;
}

export async function fetchScan(scanId: string): Promise<ScanSession> {
  const res = await fetch(`/api/scans/${scanId}`);
  if (!res.ok) throw new Error("Failed to load scan");
  return res.json();
}

export async function syncLatestScan(): Promise<ScanSession> {
  const res = await fetch("/api/scans/latest/sync", { method: "PUT" });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Failed to sync scan");
  }
  return res.json();
}

export async function restoreLatestScan(scanId: string): Promise<{ latest: ScanSession; archived_id: string | null }> {
  const res = await fetch("/api/scans/latest/restore", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ scan_id: scanId }),
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Failed to restore scan");
  }
  return res.json();
}

export async function resetLatestScan(): Promise<{ latest: ScanSession; archived_id: string | null }> {
  const res = await fetch("/api/scans/latest/reset", { method: "POST" });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Failed to reset scan");
  }
  return res.json();
}

export interface LocalizeResult {
  ok: boolean;
  map_pose?: { x: number; y: number; yaw: number };
  map_alignment?: { tx: number; ty: number; dyaw?: number };
  score?: number;
  confidence?: number;
  point_count?: number;
  reason?: string | null;
}

export async function localizeScan(scanId: string, apply = true): Promise<LocalizeResult> {
  const params = new URLSearchParams({ apply: String(apply) });
  const res = await fetch(`/api/scans/${scanId}/localize?${params}`, { method: "POST" });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Localization failed");
  }
  return res.json();
}

export async function deleteScan(scanId: string): Promise<void> {
  const res = await fetch(`/api/scans/${scanId}`, { method: "DELETE" });
  if (!res.ok) throw new Error("Failed to delete scan");
}

export interface ScanPathData {
  route: PathPoint[];
  destinations: PathPoint[];
}

export async function fetchScanPath(scanId: string): Promise<ScanPathData> {
  const res = await fetch(`/api/scans/${scanId}/path`);
  if (!res.ok) throw new Error("Failed to load path");
  const data = await res.json();
  return {
    route: data.route ?? data.points ?? [],
    destinations: data.destinations ?? [],
  };
}

export async function saveScanPath(
  scanId: string,
  route: PathPoint[],
  destinations?: PathPoint[],
): Promise<ScanPathData> {
  const res = await fetch(`/api/scans/${scanId}/path`, {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      route,
      destinations: destinations ?? route,
    }),
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Failed to save path");
  }
  const data = await res.json();
  return {
    route: data.route ?? data.points ?? [],
    destinations: data.destinations ?? [],
  };
}

export async function clearScanPath(scanId: string): Promise<void> {
  const res = await fetch(`/api/scans/${scanId}/path`, { method: "DELETE" });
  if (!res.ok) throw new Error("Failed to clear path");
}

export interface PlanRouteResult {
  route: PathPoint[];
  destinations: PathPoint[];
  points: PathPoint[];
  point_count: number;
  cell_count: number;
}

export async function planScanRoute(
  scanId: string,
  start: PathPoint,
  destinations: PathPoint[],
  save = true,
): Promise<PlanRouteResult> {
  const res = await fetch(`/api/scans/${scanId}/plan-route`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      start_x: start.x,
      start_y: start.y,
      destinations,
      save,
    }),
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    throw new Error((err as { detail?: string }).detail ?? "Path planning failed");
  }
  return res.json();
}

export async function saveDestinations(scanId: string, destinations: PathPoint[]): Promise<ScanPathData> {
  return saveScanPath(scanId, [], destinations);
}

export function formatScanLabel(s: ScanSession): string {
  if (s.name && s.name !== "scan") return s.name;
  const parts = s.id.split("_");
  return parts.length >= 3 ? parts.slice(2).join("_") : s.id;
}
