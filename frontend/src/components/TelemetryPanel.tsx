import type { ReactNode } from "react";
import type {
  AudioState,
  BatteryState,
  FramePose,
  LidarSensorState,
  MotorState,
  ReplayFrame,
  SportState,
  SystemState,
  UwbState,
} from "../api";

interface TelemetryPanelProps {
  frame: ReplayFrame | null;
  pose: FramePose | null;
  section?: "primary" | "secondary";
  bare?: boolean;
  keyboardEnabled?: boolean;
  onKeyboardEnabledChange?: (enabled: boolean) => void;
  pressedKeys?: ReadonlySet<string>;
}

function fmtRadDeg(rad: number | null | undefined): string {
  return rad == null || !Number.isFinite(rad) ? "—" : `${((rad * 180) / Math.PI).toFixed(1)}°`;
}

function boolLabel(v: boolean | null | undefined): string {
  if (v == null) return "—";
  return v ? "on" : "off";
}

function Bar({ value, max, label }: { value: number; max: number; label: string }) {
  const pct = max > 0 ? Math.min(100, (Math.abs(value) / max) * 100) : 0;
  return (
    <div className="telemetry-bar">
      <span className="telemetry-bar-label">{label}</span>
      <div className="telemetry-bar-track">
        <div className="telemetry-bar-fill" style={{ width: `${pct}%` }} />
      </div>
      <span className="telemetry-bar-value">{value.toFixed(0)}</span>
    </div>
  );
}

function AxisBar({ value, max, label, unit }: { value: number | null; max: number; label: string; unit: string }) {
  if (value == null || !Number.isFinite(value)) {
    return (
      <div className="telemetry-axis">
        <span className="telemetry-axis-label">{label}</span>
        <span className="telemetry-axis-value">—</span>
      </div>
    );
  }
  const pct = Math.min(50, (Math.abs(value) / max) * 50);
  const negative = value < 0;
  return (
    <div className="telemetry-axis">
      <span className="telemetry-axis-label">{label}</span>
      <div className="telemetry-axis-track">
        <div className="telemetry-axis-center" />
        <div
          className={`telemetry-axis-fill ${negative ? "neg" : "pos"}`}
          style={negative ? { right: "50%", width: `${pct}%` } : { left: "50%", width: `${pct}%` }}
        />
      </div>
      <span className="telemetry-axis-value">
        {value.toFixed(2)}{unit}
      </span>
    </div>
  );
}

function FootForceChart({ forces, labels }: { forces: number[]; labels: string[] }) {
  const max = Math.max(1, ...forces);
  return (
    <div className="foot-force-chart">
      {forces.map((f, i) => (
        <div key={labels[i]} className="foot-force-col">
          <div className="foot-force-bar-wrap">
            <div className="foot-force-bar" style={{ height: `${(f / max) * 100}%` }} />
          </div>
          <span className="foot-force-label">{labels[i]}</span>
          <span className="foot-force-value">{f.toFixed(0)}</span>
        </div>
      ))}
    </div>
  );
}

function SocBar({ soc }: { soc: number }) {
  const color = soc > 50 ? "#2d8a4e" : soc > 20 ? "#c9a227" : "#c44";
  return (
    <div className="soc-bar">
      <div className="soc-bar-track">
        <div className="soc-bar-fill" style={{ width: `${soc}%`, background: color }} />
      </div>
      <span className="soc-bar-label">{soc}%</span>
    </div>
  );
}

function Card({ title, children, action }: { title: string; children: ReactNode; action?: ReactNode }) {
  return (
    <div className="telemetry-card">
      <div className="telemetry-card-header">
        <div className="telemetry-card-title">{title}</div>
        {action}
      </div>
      <div className="telemetry-card-body">{children}</div>
    </div>
  );
}

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div className="telemetry-stat">
      <span className="telemetry-stat-label">{label}</span>
      <span className="telemetry-stat-value">{value}</span>
    </div>
  );
}

function MotorGrid({ motors }: { motors: MotorState[] }) {
  const maxTemp = Math.max(30, ...motors.map((m) => m.temperature ?? 0));
  return (
    <div className="motor-grid">
      {motors.map((m, i) => {
        const temp = m.temperature ?? 0;
        const t = Math.min(1, Math.max(0, (temp - 30) / Math.max(1, maxTemp - 30)));
        const color = `rgb(${Math.floor(80 + t * 175)}, ${Math.floor(120 - t * 80)}, ${Math.floor(180 - t * 140)})`;
        return (
          <div key={i} className="motor-cell" style={{ background: color }} title={`M${i}: ${temp}°C, q=${m.angle.toFixed(2)}`}>
            <span className="motor-idx">{i}</span>
            <span className="motor-temp">{temp || "—"}</span>
          </div>
        );
      })}
    </div>
  );
}

function KeyHint({ label, active }: { label: string; active?: boolean }) {
  return (
    <kbd className={`keyboard-key${active ? " active" : ""}`}>{label}</kbd>
  );
}

function MotionCard({
  sport,
  pose,
  velocity,
  keyboardEnabled,
  onKeyboardEnabledChange,
  pressedKeys,
}: {
  sport: SportState | null;
  pose: FramePose | null;
  velocity: number[] | null;
  keyboardEnabled?: boolean;
  onKeyboardEnabledChange?: (enabled: boolean) => void;
  pressedKeys?: ReadonlySet<string>;
}) {
  const speed = velocity ? Math.hypot(velocity[0], velocity[1], velocity[2] ?? 0) : null;
  const showKeyboard = onKeyboardEnabledChange != null;

  return (
    <Card
      title="Motion"
      action={
        showKeyboard ? (
          <label className="keyboard-toggle">
            <input
              type="checkbox"
              checked={keyboardEnabled ?? false}
              onChange={(e) => onKeyboardEnabledChange(e.target.checked)}
            />
            <span>Enable keyboard</span>
          </label>
        ) : undefined
      }
    >
      <div className="telemetry-stats-grid">
        <Stat label="Mode" value={sport ? String(sport.mode) : "—"} />
        <Stat label="Gait" value={sport ? String(sport.gait_type) : "—"} />
        <Stat label="Body H" value={sport ? `${sport.body_height.toFixed(3)} m` : "—"} />
        <Stat label="Yaw rate" value={sport ? `${sport.yaw_rate.toFixed(3)} rad/s` : "—"} />
        <Stat label="Height" value={pose ? `${pose.z.toFixed(3)} m` : "—"} />
        <Stat label="Speed" value={speed != null ? `${speed.toFixed(3)} m/s` : "—"} />
        <Stat label="Error" value={sport ? String(sport.error_code) : "—"} />
      </div>
      {velocity && (
        <div className="telemetry-subsection">
          <div className="telemetry-subtitle">Velocity (m/s)</div>
          <AxisBar value={velocity[0]} max={2} label="X" unit="" />
          <AxisBar value={velocity[1]} max={2} label="Y" unit="" />
          <AxisBar value={velocity[2] ?? 0} max={1} label="Z" unit="" />
        </div>
      )}
      {sport?.range_obstacle && sport.range_obstacle.some((v) => v > 0) && (
        <div className="telemetry-subsection">
          <div className="telemetry-subtitle">Obstacle range</div>
          <FootForceChart forces={sport.range_obstacle} labels={["F", "R", "L", "B"]} />
        </div>
      )}
      {showKeyboard && keyboardEnabled && (
        <div className="telemetry-subsection">
          <div className="telemetry-subtitle">Keyboard drive</div>
          <p className="keyboard-hint">
            Click the page first, then use keys. Focus must not be in a text field.
          </p>
          <div className="keyboard-keys">
            <div className="keyboard-keys-row">
              <span />
              <KeyHint label="W" active={pressedKeys?.has("w")} />
              <span />
            </div>
            <div className="keyboard-keys-row">
              <KeyHint label="A" active={pressedKeys?.has("a")} />
              <KeyHint label="S" active={pressedKeys?.has("s")} />
              <KeyHint label="D" active={pressedKeys?.has("d")} />
            </div>
          </div>
          <p className="keyboard-hint subtle">
            <KeyHint label="←" active={pressedKeys?.has("arrowleft")} />{" "}
            <KeyHint label="→" active={pressedKeys?.has("arrowright")} /> turn · combine with WASD to strafe while turning
          </p>
        </div>
      )}
    </Card>
  );
}

function ImuCard({ sport }: { sport: SportState | null }) {
  const imu = sport?.imu;
  if (!imu) {
    return (
      <Card title="IMU">
        <span className="telemetry-empty">No IMU data</span>
      </Card>
    );
  }

  return (
    <Card title="IMU">
      <div className="telemetry-stats-grid">
        <Stat label="Roll" value={fmtRadDeg(imu.rpy?.[0] ?? null)} />
        <Stat label="Pitch" value={fmtRadDeg(imu.rpy?.[1] ?? null)} />
        <Stat label="Yaw" value={fmtRadDeg(imu.rpy?.[2] ?? null)} />
        <Stat label="Temp" value={imu.temperature != null ? `${imu.temperature}°C` : "—"} />
      </div>
      {imu.gyro && (
        <div className="telemetry-subsection">
          <div className="telemetry-subtitle">Gyro (rad/s)</div>
          <AxisBar value={imu.gyro[0]} max={1} label="X" unit="" />
          <AxisBar value={imu.gyro[1]} max={1} label="Y" unit="" />
          <AxisBar value={imu.gyro[2]} max={1} label="Z" unit="" />
        </div>
      )}
      {imu.accel && (
        <div className="telemetry-subsection">
          <div className="telemetry-subtitle">Accel (m/s²)</div>
          <AxisBar value={imu.accel[0]} max={15} label="X" unit="" />
          <AxisBar value={imu.accel[1]} max={15} label="Y" unit="" />
          <AxisBar value={imu.accel[2]} max={15} label="Z" unit="" />
        </div>
      )}
    </Card>
  );
}

function PowerCard({
  battery,
  batteryV,
  motors,
}: {
  battery: BatteryState | null;
  batteryV: number | null;
  motors: MotorState[] | null;
}) {
  const voltage = battery?.voltage ?? batteryV;
  const maxMotorTemp = motors?.length
    ? Math.max(...motors.map((m) => m.temperature ?? 0))
    : null;

  return (
    <Card title="Power & motors">
      <div className="telemetry-stats-grid">
        <Stat label="Voltage" value={voltage != null ? `${voltage.toFixed(2)} V` : "—"} />
        <Stat label="Current" value={battery?.current_ma != null ? `${battery.current_ma} mA` : "—"} />
        <Stat label="Pack temp" value={battery?.temperature_c != null ? `${battery.temperature_c.toFixed(0)}°C` : "—"} />
        <Stat label="Max motor" value={maxMotorTemp != null && maxMotorTemp > 0 ? `${maxMotorTemp}°C` : "—"} />
      </div>
      {battery?.soc != null && (
        <div className="telemetry-subsection">
          <div className="telemetry-subtitle">State of charge</div>
          <SocBar soc={battery.soc} />
        </div>
      )}
      {battery?.foot_force && battery.foot_force.some((f) => f > 0) && (
        <div className="telemetry-subsection">
          <div className="telemetry-subtitle">Foot force</div>
          <FootForceChart forces={battery.foot_force} labels={["FL", "FR", "RL", "RR"]} />
        </div>
      )}
      {motors && motors.length > 0 && (
        <div className="telemetry-subsection">
          <div className="telemetry-subtitle">Joint temperature</div>
          <MotorGrid motors={motors} />
        </div>
      )}
    </Card>
  );
}

function LidarSensorCard({ lidarState, pointCount }: { lidarState: LidarSensorState | null; pointCount: number | null }) {
  return (
    <Card title="Lidar sensor">
      <div className="telemetry-stats-grid">
        <Stat label="Points" value={pointCount != null ? pointCount.toLocaleString() : "—"} />
        <Stat label="Cloud size" value={lidarState?.cloud_size != null ? lidarState.cloud_size.toLocaleString() : "—"} />
        <Stat label="Frequency" value={lidarState?.cloud_frequency != null ? `${lidarState.cloud_frequency.toFixed(1)} Hz` : "—"} />
        <Stat label="Error" value={lidarState?.error_state != null ? String(lidarState.error_state) : "—"} />
        <Stat label="Dirty" value={lidarState?.dirty_percentage != null ? `${lidarState.dirty_percentage.toFixed(0)}%` : "—"} />
      </div>
      {lidarState?.dirty_percentage != null && (
        <div className="telemetry-subsection">
          <Bar value={lidarState.dirty_percentage} max={100} label="Lens dirty" />
        </div>
      )}
    </Card>
  );
}

function RemoteSystemCard({
  uwb,
  system,
  audio,
}: {
  uwb: UwbState | null;
  system: SystemState | null;
  audio: AudioState | null;
}) {
  const hasData = uwb || system || audio;
  if (!hasData) {
    return (
      <Card title="Remote & system">
        <span className="telemetry-empty">No remote or system data</span>
      </Card>
    );
  }

  return (
    <Card title="Remote & system">
      <div className="telemetry-stats-grid">
        <Stat label="UWB dist" value={uwb?.distance != null ? `${uwb.distance.toFixed(2)} m` : "—"} />
        <Stat label="UWB yaw" value={fmtRadDeg(uwb?.yaw ?? null)} />
        <Stat label="Joy mode" value={uwb?.joy_mode != null ? String(uwb.joy_mode) : "—"} />
        <Stat label="App ctrl" value={boolLabel(uwb?.enabled_from_app)} />
        <Stat label="Volume" value={system?.volume != null ? String(system.volume) : "—"} />
        <Stat label="Brightness" value={system?.brightness != null ? String(system.brightness) : "—"} />
        <Stat label="Obs. avoid" value={boolLabel(system?.obstacles_avoid)} />
        <Stat label="UWB switch" value={boolLabel(system?.uwb_switch)} />
        <Stat label="Audio" value={audio?.play_state || "—"} />
        <Stat label="Playing" value={boolLabel(audio?.is_playing)} />
      </div>
      {uwb?.joystick && (
        <div className="telemetry-subsection">
          <div className="telemetry-subtitle">Joystick</div>
          <AxisBar value={uwb.joystick[0]} max={1} label="X" unit="" />
          <AxisBar value={uwb.joystick[1]} max={1} label="Y" unit="" />
        </div>
      )}
      {audio?.track && (
        <div className="telemetry-subsection">
          <div className="telemetry-subtitle">Track</div>
          <div className="audio-track">{audio.track}</div>
        </div>
      )}
    </Card>
  );
}

export function TelemetryPanel({
  frame,
  pose,
  section = "primary",
  bare = false,
  keyboardEnabled,
  onKeyboardEnabledChange,
  pressedKeys,
}: TelemetryPanelProps) {
  const primary = (
    <>
      <MotionCard
        sport={frame?.sport ?? null}
        pose={pose}
        velocity={frame?.velocity ?? null}
        keyboardEnabled={keyboardEnabled}
        onKeyboardEnabledChange={onKeyboardEnabledChange}
        pressedKeys={pressedKeys}
      />
      <ImuCard sport={frame?.sport ?? null} />
      <PowerCard
        battery={frame?.battery ?? null}
        batteryV={frame?.battery_v ?? null}
        motors={frame?.battery?.motors ?? null}
      />
      <LidarSensorCard lidarState={frame?.lidar_state ?? null} pointCount={frame?.lidar?.point_count ?? null} />
    </>
  );

  const secondary = (
    <RemoteSystemCard
      uwb={frame?.uwb ?? null}
      system={frame?.system ?? null}
      audio={frame?.audio ?? null}
    />
  );

  const content = section === "secondary" ? secondary : primary;
  if (bare) return content;

  return <div className="telemetry-grid">{content}</div>;
}
