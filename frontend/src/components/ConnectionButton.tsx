import type { LiveStatus } from "../api";

interface ConnectionButtonProps {
  liveStatus: LiveStatus | null;
  connecting: boolean;
  disconnecting: boolean;
  onClick: () => void;
}

export function connectionButtonLabel(
  liveStatus: LiveStatus | null,
  connecting: boolean,
  disconnecting: boolean,
): string {
  const isConnected = Boolean(liveStatus?.connected);
  const isConnecting =
    connecting || disconnecting || liveStatus?.state === "connecting";

  if (disconnecting) return "Disconnecting…";
  if (isConnected) return "Disconnect";
  if (isConnecting) return "Connecting…";
  return "Connect";
}

export function ConnectionButton({
  liveStatus,
  connecting,
  disconnecting,
  onClick,
}: ConnectionButtonProps) {
  const isConnected = Boolean(liveStatus?.connected);
  const isConnecting =
    connecting || disconnecting || liveStatus?.state === "connecting";

  return (
    <button
      type="button"
      className={`connection-btn${isConnected ? " connected" : ""}${isConnecting && !isConnected ? " connecting" : ""}`}
      onClick={onClick}
      disabled={disconnecting}
    >
      {connectionButtonLabel(liveStatus, connecting, disconnecting)}
    </button>
  );
}
