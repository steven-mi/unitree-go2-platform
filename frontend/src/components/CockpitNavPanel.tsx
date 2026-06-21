import { LocateFixed, MapPin, Navigation, RotateCcw, Square, Undo2, X } from "lucide-react";
import type { PathPoint } from "../api";

interface CockpitNavPanelProps {
  connected: boolean;
  destinations: PathPoint[];
  selectedIndex: number;
  routePoints: PathPoint[];
  planning: boolean;
  following: boolean;
  localizing: boolean;
  localizeConfidence: number | null;
  hasPose: boolean;
  onSelect: (index: number) => void;
  onRemove: (index: number) => void;
  onUndo: () => void;
  onClear: () => void;
  onLocate: () => void;
  onGo: () => void;
  onStop: () => void;
}

export function CockpitNavPanel({
  connected,
  destinations,
  selectedIndex,
  routePoints,
  planning,
  following,
  localizing,
  localizeConfidence,
  hasPose,
  onSelect,
  onRemove,
  onUndo,
  onClear,
  onLocate,
  onGo,
  onStop,
}: CockpitNavPanelProps) {
  const hasDestinations = destinations.length > 0;
  const selectedDestination = destinations[selectedIndex] ?? null;
  const canGo =
    connected &&
    hasPose &&
    selectedDestination != null &&
    routePoints.length >= 2 &&
    !following &&
    !planning;

  const lowMatch = localizeConfidence != null && localizeConfidence < 0.35;
  const pct = localizeConfidence != null ? ` · ${Math.round(localizeConfidence * 100)}%` : "";
  const statusText = lowMatch
    ? `Low map match${pct} — press Locate`
    : localizeConfidence != null
      ? `Located on map${pct}`
      : "Tracking live map";

  const goTitle = !connected
    ? "Connect to the robot first"
    : !selectedDestination
      ? "Click the floor plan to add a destination"
      : routePoints.length < 2
        ? planning
          ? "Planning route…"
          : "Waiting for a route to the destination"
        : `Guide the dog to destination ${selectedIndex + 1}`;

  return (
    <div className="cockpit-nav">
      <div className="cockpit-nav-header">
        <h3>
          <MapPin size={15} strokeWidth={1.75} />
          Point &amp; Go
        </h3>
        <div className="cockpit-nav-actions">
          <button
            type="button"
            className="path-action-btn"
            onClick={onLocate}
            disabled={!connected || localizing || following}
            title={connected ? "Match the live lidar to this map" : "Connect first"}
          >
            <LocateFixed size={14} strokeWidth={1.75} />
            {localizing ? "Locating…" : "Locate"}
          </button>
          {hasDestinations && (
            <>
              <button
                type="button"
                className="path-action-btn"
                onClick={onUndo}
                disabled={following || planning}
              >
                <Undo2 size={14} strokeWidth={1.75} />
                Undo
              </button>
              <button
                type="button"
                className="path-action-btn"
                onClick={onClear}
                disabled={following || planning}
              >
                <RotateCcw size={14} strokeWidth={1.75} />
                Clear
              </button>
            </>
          )}
        </div>
      </div>

      {connected && (
        <div className={`cockpit-nav-status${lowMatch ? "" : " ok"}`}>{statusText}</div>
      )}

      {hasDestinations ? (
        <ul className="path-point-list cockpit-nav-list">
          {destinations.map((pt, i) => (
            <li key={`${pt.x}-${pt.y}-${i}`} className="path-point-row">
              <button
                type="button"
                className={`path-point-item${i === selectedIndex ? " selected" : ""}`}
                onClick={() => onSelect(i)}
                disabled={following || planning}
                aria-pressed={i === selectedIndex}
              >
                <span className="path-point-badge end">{i + 1}</span>
                <span className="path-point-coords">
                  {pt.x.toFixed(2)}, {pt.y.toFixed(2)}
                </span>
              </button>
              <button
                type="button"
                className="path-point-delete"
                onClick={() => onRemove(i)}
                disabled={following || planning}
                aria-label={`Remove destination ${i + 1}`}
                title="Remove destination"
              >
                <X size={14} strokeWidth={1.75} />
              </button>
            </li>
          ))}
        </ul>
      ) : (
        <p className="cockpit-nav-empty">
          {connected
            ? "Click the floor plan to drop a destination. The route follows the latest scan and updates as the dog moves."
            : "Connect to the robot to drop destinations on the live map."}
        </p>
      )}

      <div className="path-follow-row">
        {following ? (
          <button type="button" className="path-stop-btn" onClick={onStop}>
            <Square size={16} strokeWidth={1.75} />
            Stop
          </button>
        ) : (
          <button
            type="button"
            className="path-follow-btn"
            onClick={onGo}
            disabled={!canGo}
            title={goTitle}
          >
            <Navigation size={16} strokeWidth={1.75} />
            {hasDestinations ? `Go to ${selectedIndex + 1}` : "Go"}
          </button>
        )}
      </div>
    </div>
  );
}
