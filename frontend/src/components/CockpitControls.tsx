import { useCallback, useState } from "react";
import { sendSportCommand } from "../api";

interface CockpitControlsProps {
  connected: boolean;
  onError: (message: string) => void;
}

interface CommandButton {
  command: string;
  label: string;
  parameter?: Record<string, unknown>;
}

const WALK_MODES: CommandButton[] = [
  { command: "FreeWalk", label: "Free walk", parameter: { data: true } },
  { command: "ClassicWalk", label: "Classic", parameter: { data: true } },
  { command: "TrotRun", label: "Trot run", parameter: { data: true } },
  { command: "StaticWalk", label: "Static", parameter: { data: true } },
  { command: "EconomicGait", label: "Economic", parameter: { data: true } },
  { command: "CrossStep", label: "Cross step", parameter: { data: true } },
];

const STANCES: CommandButton[] = [
  { command: "StandUp", label: "Stand up" },
  { command: "StandDown", label: "Stand down" },
  { command: "Sit", label: "Sit" },
  { command: "RiseSit", label: "Rise sit" },
  { command: "BalanceStand", label: "Balance" },
  { command: "RecoveryStand", label: "Recovery" },
  { command: "Damp", label: "Damp" },
];

const TRICKS: CommandButton[] = [
  { command: "Hello", label: "Hello" },
  { command: "Stretch", label: "Stretch" },
  { command: "Heart", label: "Heart" },
  { command: "Dance1", label: "Dance 1" },
  { command: "Dance2", label: "Dance 2" },
  { command: "Pose", label: "Pose mode", parameter: { data: true } },
  { command: "StopMove", label: "Stop move" },
];

function CommandGrid({
  items,
  disabled,
  busy,
  onRun,
}: {
  items: CommandButton[];
  disabled: boolean;
  busy: string | null;
  onRun: (item: CommandButton) => void;
}) {
  return (
    <div className="cockpit-cmd-grid">
      {items.map((item) => (
        <button
          key={item.command}
          type="button"
          className="cockpit-cmd-btn"
          disabled={disabled || busy === item.command}
          onClick={() => onRun(item)}
        >
          {busy === item.command ? "…" : item.label}
        </button>
      ))}
    </div>
  );
}

export function CockpitControls({ connected, onError }: CockpitControlsProps) {
  const [busy, setBusy] = useState<string | null>(null);

  const runCommand = useCallback(
    async (item: CommandButton) => {
      if (!connected) return;
      setBusy(item.command);
      try {
        await sendSportCommand(item.command, item.parameter);
      } catch (err) {
        onError(err instanceof Error ? err.message : "Command failed");
      } finally {
        setBusy(null);
      }
    },
    [connected, onError],
  );

  return (
    <div className="cockpit-controls">
      <div className="cockpit-section">
        <h3>Walk modes</h3>
        <CommandGrid
          items={WALK_MODES}
          disabled={!connected}
          busy={busy}
          onRun={(item) => void runCommand(item)}
        />
      </div>

      <div className="cockpit-section">
        <h3>Stances</h3>
        <CommandGrid
          items={STANCES}
          disabled={!connected}
          busy={busy}
          onRun={(item) => void runCommand(item)}
        />
      </div>

      <div className="cockpit-section">
        <h3>Tricks &amp; poses</h3>
        <CommandGrid
          items={TRICKS}
          disabled={!connected}
          busy={busy}
          onRun={(item) => void runCommand(item)}
        />
      </div>
    </div>
  );
}
