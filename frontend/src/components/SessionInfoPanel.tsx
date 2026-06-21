import type { RobotService, SessionDetail } from "../api";

interface SessionInfoPanelProps {
  detail: SessionDetail | null;
  bare?: boolean;
}

function boolLabel(v: boolean | null | undefined): string {
  if (v == null) return "—";
  return v ? "on" : "off";
}

function RpcStat({ label, value }: { label: string; value: string }) {
  return (
    <div className="telemetry-stat">
      <span className="telemetry-stat-label">{label}</span>
      <span className="telemetry-stat-value">{value}</span>
    </div>
  );
}

function RpcSummary({ rpc }: { rpc: SessionDetail["rpc"] }) {
  const motion = rpc.motion_switcher_CheckMode as { name?: string; form?: string } | undefined;
  const obstacles = rpc.obstacles_avoid_SwitchGet as { enable?: boolean } | undefined;
  const speed = rpc.sport_GetSpeedLevel as { data?: number } | undefined;
  const volume = rpc.vui_GetVolume as { volume?: number } | undefined;
  const brightness = rpc.vui_GetBrightness as { brightness?: number } | undefined;
  const audioList = rpc.audiohub_GetAudioList as { audio_list?: unknown[] } | undefined;
  const photo = rpc.front_photo_response as { bytes?: number } | undefined;

  return (
    <div className="telemetry-stats-grid">
      <RpcStat label="Motion mode" value={motion?.name ?? "—"} />
      <RpcStat label="Obstacle avoid" value={obstacles?.enable != null ? boolLabel(obstacles.enable) : "—"} />
      <RpcStat label="Speed level" value={speed?.data != null ? String(speed.data) : "—"} />
      <RpcStat label="Volume" value={volume?.volume != null ? String(volume.volume) : "—"} />
      <RpcStat label="Brightness" value={brightness?.brightness != null ? String(brightness.brightness) : "—"} />
      <RpcStat
        label="Audio tracks"
        value={audioList?.audio_list ? String(audioList.audio_list.length) : "—"}
      />
      <RpcStat label="Front photo" value={photo?.bytes != null ? `${(photo.bytes / 1024).toFixed(0)} KB` : "—"} />
    </div>
  );
}

function ServiceList({ services }: { services: RobotService[] }) {
  const active = services.filter((s) => s.status === 1);
  const shown = active.length > 0 ? active : services;

  return (
    <div className="service-list">
      {shown.slice(0, 16).map((s) => (
        <span key={s.name} className={`service-chip ${s.status === 1 ? "active" : ""}`} title={s.version}>
          {s.name}
        </span>
      ))}
      {shown.length > 16 && <span className="service-chip more">+{shown.length - 16}</span>}
      {shown.length === 0 && <span className="telemetry-empty">No services</span>}
    </div>
  );
}

export function SessionInfoPanel({ detail, bare = false }: SessionInfoPanelProps) {
  if (!detail) return null;
  const hasRpc = Object.keys(detail.rpc).length > 0;
  const hasServices = detail.services.length > 0;
  if (!hasRpc && !hasServices) return null;

  const activeCount = detail.services.filter((s) => s.status === 1).length;

  const content = (
    <div className="telemetry-card session-info-card">
      <div className="telemetry-card-title">Session</div>
      <div className="telemetry-card-body">
        {hasRpc && <RpcSummary rpc={detail.rpc} />}
        {hasServices && (
          <div className={hasRpc ? "telemetry-subsection" : undefined}>
            {hasRpc && (
              <div className="telemetry-subtitle">Robot services ({activeCount} active)</div>
            )}
            <ServiceList services={detail.services} />
          </div>
        )}
      </div>
    </div>
  );

  if (bare) return content;

  return <div className="session-info">{content}</div>;
}
